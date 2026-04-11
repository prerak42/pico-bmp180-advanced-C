#include "bmp180.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "pico/error.h"
#include "hardware/i2c.h"

// ── Register map ──────────────────────────────────────────────────

#define REG_ID          0xD0
#define REG_SOFT_RESET  0xE0
#define REG_CTRL        0xF4
#define REG_DATA        0xF6

#define CMD_RESET       0xB6
#define CMD_TEMP        0x2E
#define CMD_PRESSURE(o) (0x34 | ((o) << 6))

// SCO bit in REG_CTRL: set while conversion is in progress, cleared when done
#define SCO_BIT         0x20

// Calibration EEPROM base address — all 11 coefficients (22 bytes) are
// stored contiguously from 0xAA to 0xBF and read in a single burst.
#define CALIB_BASE      0xAA
#define CALIB_LEN       22u

// OSS conversion wait times in ms (from datasheet table 7)
static const uint8_t oss_wait_ms[] = {5, 8, 14, 26};

// ── Locking helpers ───────────────────────────────────────────────
//
// Every public function that touches device state or the I2C bus
// must call LOCK() on entry and UNLOCK() before every return.
// Internal static helpers (reg_read, reg_write, compensate_*, etc.)
// must NOT call LOCK — they are always called from within a lock.
//
// The averaging helpers hold the lock for their entire window so that
// no other core can interleave a measurement mid-average. They call
// the internal static helpers (read_raw_temp, read_raw_pressure,
// compensate_*) directly rather than the public functions to avoid
// re-entering the non-recursive mutex and deadlocking.

#define LOCK(dev)   mutex_enter_blocking(&(dev)->_mutex)
#define UNLOCK(dev) mutex_exit(&(dev)->_mutex)

// ── Internal I2C helpers ──────────────────────────────────────────
//
// Both helpers use the timeout variants of the Pico SDK I2C functions.
// A hung bus (disconnected sensor, stuck SDA) will now return
// BMP180_ERR_TIMEOUT rather than blocking forever.
// Must be called with the mutex already held.

static bmp180_status_t reg_write(bmp180_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    int result = i2c_write_timeout_us(dev->i2c, BMP180_I2C_ADDR,
                                      buf, 2, false,
                                      BMP180_I2C_TIMEOUT_US);
    if (result == PICO_ERROR_TIMEOUT) return BMP180_ERR_TIMEOUT;
    return (result == 2) ? BMP180_OK : BMP180_ERR_I2C;
}

static bmp180_status_t reg_read(bmp180_t *dev, uint8_t reg,
                                 uint8_t *dst, size_t len) {
    int result = i2c_write_timeout_us(dev->i2c, BMP180_I2C_ADDR,
                                      &reg, 1, true,
                                      BMP180_I2C_TIMEOUT_US);
    if (result == PICO_ERROR_TIMEOUT) return BMP180_ERR_TIMEOUT;
    if (result != 1)                  return BMP180_ERR_I2C;

    result = i2c_read_timeout_us(dev->i2c, BMP180_I2C_ADDR,
                                 dst, len, false,
                                 BMP180_I2C_TIMEOUT_US);
    if (result == PICO_ERROR_TIMEOUT) return BMP180_ERR_TIMEOUT;
    if (result != (int)len)           return BMP180_ERR_I2C;
    return BMP180_OK;
}

// ── Calibration ───────────────────────────────────────────────────

// The 11 BMP180 calibration coefficients are stored contiguously in EEPROM
// starting at 0xAA. Rather than 11 separate I2C transactions (the original
// approach), we do a single 22-byte burst read and decode in-place.
// All values are big-endian (MSB first).
// Must be called with the mutex already held.
static bmp180_status_t read_calibration(bmp180_t *dev) {
    uint8_t raw[CALIB_LEN];
    bmp180_status_t s = reg_read(dev, CALIB_BASE, raw, CALIB_LEN);
    if (s != BMP180_OK) return s;

#define SIGNED16(i)   ((int16_t) ((raw[(i)] << 8) | raw[(i)+1]))
#define UNSIGNED16(i) ((uint16_t)((raw[(i)] << 8) | raw[(i)+1]))

    dev->calib.AC1 = SIGNED16(0);
    dev->calib.AC2 = SIGNED16(2);
    dev->calib.AC3 = SIGNED16(4);
    dev->calib.AC4 = UNSIGNED16(6);
    dev->calib.AC5 = UNSIGNED16(8);
    dev->calib.AC6 = UNSIGNED16(10);
    dev->calib.B1  = SIGNED16(12);
    dev->calib.B2  = SIGNED16(14);
    dev->calib.MB  = SIGNED16(16);
    dev->calib.MC  = SIGNED16(18);
    dev->calib.MD  = SIGNED16(20);

#undef SIGNED16
#undef UNSIGNED16
    return BMP180_OK;
}

// The datasheet specifies that calibration values of 0x0000 or 0xFFFF
// indicate a broken chip or a bad I2C connection.
static bool calib_valid(const bmp180_calib_t *c) {
    if ((uint16_t)c->AC1 == 0x0000 || (uint16_t)c->AC1 == 0xFFFF) return false;
    if ((uint16_t)c->AC2 == 0x0000 || (uint16_t)c->AC2 == 0xFFFF) return false;
    if ((uint16_t)c->AC3 == 0x0000 || (uint16_t)c->AC3 == 0xFFFF) return false;
    if (c->AC4 == 0x0000 || c->AC4 == 0xFFFF) return false;
    if (c->AC5 == 0x0000 || c->AC5 == 0xFFFF) return false;
    if (c->AC6 == 0x0000 || c->AC6 == 0xFFFF) return false;
    return true;
}

// ── Raw ADC reads ─────────────────────────────────────────────────
// Must be called with the mutex already held.

static bmp180_status_t read_raw_temp(bmp180_t *dev, int32_t *out) {
    bmp180_status_t s = reg_write(dev, REG_CTRL, CMD_TEMP);
    if (s != BMP180_OK) return s;
    sleep_ms(5);
    uint8_t buf[2];
    s = reg_read(dev, REG_DATA, buf, 2);
    if (s != BMP180_OK) return s;
    *out = (int32_t)((buf[0] << 8) | buf[1]);
    return BMP180_OK;
}

static bmp180_status_t read_raw_pressure(bmp180_t *dev, int32_t *out) {
    bmp180_status_t s = reg_write(dev, REG_CTRL, CMD_PRESSURE(dev->oss));
    if (s != BMP180_OK) return s;
    sleep_ms(oss_wait_ms[dev->oss]);

    uint8_t buf[3];
    s = reg_read(dev, REG_DATA, buf, 3);
    if (s != BMP180_OK) return s;

    *out = ((int32_t)buf[0] << 16 | (int32_t)buf[1] << 8 | buf[2])
           >> (8 - dev->oss);
    return BMP180_OK;
}

// ── Compensation (datasheet section 4.1.2) ───────────────────────
//
// B5 is an intermediate value shared between temperature and pressure
// compensation. It is computed from UT (raw temperature) and reused
// in both calculations, which is why temperature must be read before
// pressure in every code path.

static int32_t compute_B5(const bmp180_t *dev, int32_t UT) {
    int32_t X1 = ((UT - (int32_t)dev->calib.AC6)
                  * (int32_t)dev->calib.AC5) >> 15;
    int32_t denom = X1 + dev->calib.MD;
    if (denom == 0) return X1;   // MC == 0 edge case; X2 contribution is zero
    int32_t X2 = ((int32_t)dev->calib.MC << 11) / denom;
    return X1 + X2;
}

static float compensate_temperature(const bmp180_t *dev, int32_t UT) {
    int32_t B5 = compute_B5(dev, UT);
    int32_t T  = (B5 + 8) >> 4;   // units of 0.1 C
    return T / 10.0f;
}

// Returns false if compensation yields a degenerate result (B4 == 0),
// which would cause a division-by-zero and produce garbage pressure.
// This can happen with a broken sensor or corrupted calibration data.
static bool compensate_pressure(const bmp180_t *dev, int32_t UT,
                                 int32_t UP, float *out) {
    int32_t B5 = compute_B5(dev, UT);
    int32_t B6 = B5 - 4000;

    int32_t X1 = (dev->calib.B2 * ((B6 * B6) >> 12)) >> 11;
    int32_t X2 = (dev->calib.AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((((int32_t)dev->calib.AC1) * 4 + X3)
                   << dev->oss) + 2) >> 2;

    X1 = (dev->calib.AC3 * B6) >> 13;
    X2 = (dev->calib.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;

    uint32_t B4 = ((uint32_t)dev->calib.AC4
                   * (uint32_t)(X3 + 32768)) >> 15;

    // Guard against division-by-zero. Per the enjoyneering library and
    // general defensive practice: if B4 is zero the calibration data
    // is degenerate and we must not proceed.
    if (B4 == 0) return false;

    uint32_t B7 = ((uint32_t)UP - B3)
                  * (uint32_t)(50000 >> dev->oss);

    int32_t p;
    if (B7 < 0x80000000u)
        p = (int32_t)((B7 * 2) / B4);
    else
        p = (int32_t)((B7 / B4) * 2);

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p  = p + ((X1 + X2 + 3791) >> 4);

    *out = (float)p / 100.0f;  // Pa -> hPa
    return true;
}

// ── Public API — core ─────────────────────────────────────────────

bmp180_status_t bmp180_init(bmp180_t *dev, i2c_inst_t *i2c, bmp180_oss_t oss) {
    // Initialise the mutex before anything else so that LOCK/UNLOCK
    // are safe to use from this point forward.
    mutex_init(&dev->_mutex);

    dev->i2c           = i2c;
    dev->oss           = oss;
    dev->sea_level_hpa = BMP180_SEA_LEVEL_HPA;
    dev->initialised   = false;
    dev->_temp_valid   = false;
    dev->_raw_temp     = 0;
    dev->_meas         = BMP180_MEAS_NONE;

    // No other core can have a reference to this struct yet (it was
    // just passed in uninitialised), so no lock is needed above.
    // Lock from here to protect the I2C transactions.
    LOCK(dev);

    // Verify chip identity — propagate I2C/timeout errors, only return
    // NOT_FOUND when a device is present but returns the wrong chip ID.
    uint8_t id;
    bmp180_status_t s = reg_read(dev, REG_ID, &id, 1);
    if (s != BMP180_OK)        { UNLOCK(dev); return s; }
    if (id != BMP180_CHIP_ID)  { UNLOCK(dev); return BMP180_ERR_NOT_FOUND; }

    // Read all 11 calibration coefficients in a single 22-byte burst
    s = read_calibration(dev);
    if (s != BMP180_OK)        { UNLOCK(dev); return s; }

    if (!calib_valid(&dev->calib)) { UNLOCK(dev); return BMP180_ERR_BAD_DATA; }

    dev->initialised = true;
    UNLOCK(dev);
    return BMP180_OK;
}

bmp180_status_t bmp180_reset(bmp180_t *dev) {
    LOCK(dev);
    bmp180_status_t s = reg_write(dev, REG_SOFT_RESET, CMD_RESET);
    if (s != BMP180_OK) { UNLOCK(dev); return s; }
    dev->initialised = false;
    dev->_temp_valid = false;
    dev->_meas       = BMP180_MEAS_NONE;
    UNLOCK(dev);
    sleep_ms(10);  // datasheet: 2 ms startup, 10 ms to be safe
                   // sleep outside the lock — no reason to block other cores
    return BMP180_OK;
}

void bmp180_set_oss(bmp180_t *dev, bmp180_oss_t oss) {
    LOCK(dev);
    dev->oss = oss;
    UNLOCK(dev);
}

void bmp180_set_sea_level(bmp180_t *dev, float sea_level_hpa) {
    LOCK(dev);
    dev->sea_level_hpa = sea_level_hpa;
    UNLOCK(dev);
}

bmp180_status_t bmp180_read_temperature(bmp180_t *dev, float *out) {
    LOCK(dev);
    if (!dev->initialised) { UNLOCK(dev); return BMP180_ERR_NOT_INIT; }
    int32_t UT;
    bmp180_status_t s = read_raw_temp(dev, &UT);
    if (s == BMP180_OK) {
        dev->_raw_temp   = UT;
        dev->_temp_valid = true;
        *out = compensate_temperature(dev, UT);
    }
    UNLOCK(dev);
    return s;
}

bmp180_status_t bmp180_read_pressure(bmp180_t *dev, float *out) {
    LOCK(dev);
    if (!dev->initialised) { UNLOCK(dev); return BMP180_ERR_NOT_INIT; }
    int32_t UT, UP;
    bmp180_status_t s = read_raw_temp(dev, &UT);
    if (s == BMP180_OK) {
        dev->_raw_temp   = UT;
        dev->_temp_valid = true;
        s = read_raw_pressure(dev, &UP);
    }
    if (s == BMP180_OK) {
        if (!compensate_pressure(dev, UT, UP, out)) s = BMP180_ERR_BAD_DATA;
    }
    UNLOCK(dev);
    return s;
}

bmp180_status_t bmp180_read_all(bmp180_t *dev,
                                 float *temp_out,
                                 float *pres_out) {
    LOCK(dev);
    if (!dev->initialised) { UNLOCK(dev); return BMP180_ERR_NOT_INIT; }
    int32_t UT, UP;
    // Temperature first — UT is needed for pressure compensation via B5
    bmp180_status_t s = read_raw_temp(dev, &UT);
    if (s == BMP180_OK) {
        dev->_raw_temp   = UT;
        dev->_temp_valid = true;
        s = read_raw_pressure(dev, &UP);
    }
    if (s == BMP180_OK) {
        *temp_out = compensate_temperature(dev, UT);
        if (!compensate_pressure(dev, UT, UP, pres_out)) s = BMP180_ERR_BAD_DATA;
    }
    UNLOCK(dev);
    return s;
}

// ── Public API — async ────────────────────────────────────────────

bmp180_status_t bmp180_trigger_temperature(bmp180_t *dev) {
    LOCK(dev);
    if (!dev->initialised) { UNLOCK(dev); return BMP180_ERR_NOT_INIT; }
    bmp180_status_t s = reg_write(dev, REG_CTRL, CMD_TEMP);
    if (s == BMP180_OK) dev->_meas = BMP180_MEAS_TEMPERATURE;
    UNLOCK(dev);
    return s;
}

bmp180_status_t bmp180_trigger_pressure(bmp180_t *dev) {
    LOCK(dev);
    if (!dev->initialised) { UNLOCK(dev); return BMP180_ERR_NOT_INIT; }
    bmp180_status_t s;
    if (!dev->_temp_valid) {
        // Cannot compensate pressure without a cached UT —
        // this is a sequence violation, not an init failure
        s = BMP180_ERR_SEQUENCE;
    } else {
        s = reg_write(dev, REG_CTRL, CMD_PRESSURE(dev->oss));
        if (s == BMP180_OK) dev->_meas = BMP180_MEAS_PRESSURE;
    }
    UNLOCK(dev);
    return s;
}

bmp180_status_t bmp180_data_ready(bmp180_t *dev, bool *ready) {
    LOCK(dev);
    if (!dev->initialised) { UNLOCK(dev); return BMP180_ERR_NOT_INIT; }
    bmp180_status_t s = BMP180_OK;
    if (dev->_meas == BMP180_MEAS_NONE) {
        *ready = false;
    } else {
        uint8_t ctrl;
        s = reg_read(dev, REG_CTRL, &ctrl, 1);
        if (s == BMP180_OK) {
            // SCO bit (bit 5): 1 = conversion in progress, 0 = done
            *ready = !(ctrl & SCO_BIT);
        }
    }
    UNLOCK(dev);
    return s;
}

bmp180_status_t bmp180_fetch_temperature(bmp180_t *dev, float *out) {
    LOCK(dev);
    if (!dev->initialised) { UNLOCK(dev); return BMP180_ERR_NOT_INIT; }
    if (dev->_meas != BMP180_MEAS_TEMPERATURE) {
        UNLOCK(dev);
        return BMP180_ERR_SEQUENCE;
    }
    uint8_t buf[2];
    bmp180_status_t s = reg_read(dev, REG_DATA, buf, 2);
    if (s == BMP180_OK) {
        int32_t UT = (int32_t)((buf[0] << 8) | buf[1]);
        dev->_raw_temp   = UT;
        dev->_temp_valid = true;
        dev->_meas       = BMP180_MEAS_NONE;
        *out = compensate_temperature(dev, UT);
    }
    UNLOCK(dev);
    return s;
}

bmp180_status_t bmp180_fetch_pressure(bmp180_t *dev, float *out) {
    LOCK(dev);
    if (!dev->initialised) { UNLOCK(dev); return BMP180_ERR_NOT_INIT; }
    if (dev->_meas != BMP180_MEAS_PRESSURE || !dev->_temp_valid) {
        UNLOCK(dev);
        return BMP180_ERR_SEQUENCE;
    }
    uint8_t buf[3];
    bmp180_status_t s = reg_read(dev, REG_DATA, buf, 3);
    if (s == BMP180_OK) {
        int32_t UP = ((int32_t)buf[0] << 16 | (int32_t)buf[1] << 8 | buf[2])
                     >> (8 - dev->oss);
        dev->_meas = BMP180_MEAS_NONE;
        if (!compensate_pressure(dev, dev->_raw_temp, UP, out))
            s = BMP180_ERR_BAD_DATA;
    }
    UNLOCK(dev);
    return s;
}

// ── Altitude and conversion ───────────────────────────────────────

float bmp180_pressure_to_altitude(bmp180_t *dev, float pressure_hpa) {
    // Standard barometric formula:
    //   h = 44330 * (1 - (P / P0) ^ (1/5.255))
    // BMP180_BARO_EXP = 1/5.255 so this is consistent with
    // bmp180_sea_level_from_altitude which uses BMP180_BARO_POW = 5.255.
    LOCK(dev);
    float sea = dev->sea_level_hpa;
    UNLOCK(dev);
    return 44330.0f * (1.0f - powf(pressure_hpa / sea, BMP180_BARO_EXP));
}

float bmp180_sea_level_from_altitude(float pressure_hpa, float altitude_m) {
    // Inverse of the barometric formula:
    //   P0 = P / (1 - h/44330) ^ 5.255
    return pressure_hpa / powf(1.0f - (altitude_m / 44330.0f),
                                BMP180_BARO_POW);
}

// ── Averaging helpers ─────────────────────────────────────────────
//
// These call the internal static helpers (read_raw_temp,
// read_raw_pressure, compensate_*) directly rather than the public
// functions. This avoids re-entering the non-recursive mutex and
// deadlocking. The lock is held for the full averaging window so
// that no other core can interleave a measurement mid-average.

bmp180_status_t bmp180_read_temperature_avg(bmp180_t *dev,
                                             uint8_t samples,
                                             uint32_t delay_ms,
                                             float *out) {
    if (samples == 0 || samples > 64) return BMP180_ERR_BAD_PARAM;
    LOCK(dev);
    if (!dev->initialised) { UNLOCK(dev); return BMP180_ERR_NOT_INIT; }
    float sum = 0.0f;
    bmp180_status_t s = BMP180_OK;
    for (uint8_t i = 0; i < samples; i++) {
        int32_t UT;
        s = read_raw_temp(dev, &UT);
        if (s != BMP180_OK) break;
        dev->_raw_temp   = UT;
        dev->_temp_valid = true;
        sum += compensate_temperature(dev, UT);
        if (i < samples - 1) sleep_ms(delay_ms);
    }
    if (s == BMP180_OK) *out = sum / samples;
    UNLOCK(dev);
    return s;
}

bmp180_status_t bmp180_read_pressure_avg(bmp180_t *dev,
                                          uint8_t samples,
                                          uint32_t delay_ms,
                                          float *out) {
    if (samples == 0 || samples > 64) return BMP180_ERR_BAD_PARAM;
    LOCK(dev);
    if (!dev->initialised) { UNLOCK(dev); return BMP180_ERR_NOT_INIT; }

    // Read temperature ONCE. Pressure compensation only needs UT (raw
    // temperature) to compute B5. Over a short averaging window temperature
    // does not change meaningfully, so one measurement is sufficient and
    // avoids N redundant temperature reads (the original code's flaw).
    int32_t UT;
    bmp180_status_t s = read_raw_temp(dev, &UT);
    if (s != BMP180_OK) { UNLOCK(dev); return s; }
    dev->_raw_temp   = UT;
    dev->_temp_valid = true;

    float sum = 0.0f;
    for (uint8_t i = 0; i < samples; i++) {
        int32_t UP;
        s = read_raw_pressure(dev, &UP);
        if (s != BMP180_OK) break;

        float p;
        if (!compensate_pressure(dev, UT, UP, &p)) {
            s = BMP180_ERR_BAD_DATA;
            break;
        }
        sum += p;
        if (i < samples - 1) sleep_ms(delay_ms);
    }
    if (s == BMP180_OK) *out = sum / samples;
    UNLOCK(dev);
    return s;
}

// ── Debug helpers ─────────────────────────────────────────────────

void bmp180_dump_calibration(bmp180_t *dev) {
    // Copy calibration data under the lock, then print outside it.
    // This keeps the mutex held for the minimum time and avoids
    // holding it across the slow printf calls.
    LOCK(dev);
    bmp180_calib_t c = dev->calib;
    UNLOCK(dev);

    printf("=== BMP180 Calibration Coefficients ===\n");
    printf("AC1 = %d\n",  c.AC1);
    printf("AC2 = %d\n",  c.AC2);
    printf("AC3 = %d\n",  c.AC3);
    printf("AC4 = %u\n",  c.AC4);
    printf("AC5 = %u\n",  c.AC5);
    printf("AC6 = %u\n",  c.AC6);
    printf("B1  = %d\n",  c.B1);
    printf("B2  = %d\n",  c.B2);
    printf("MB  = %d\n",  c.MB);
    printf("MC  = %d\n",  c.MC);
    printf("MD  = %d\n",  c.MD);
    printf("=======================================\n");
}

const char *bmp180_status_str(bmp180_status_t status) {
    switch (status) {
        case BMP180_OK:            return "OK";
        case BMP180_ERR_NOT_FOUND: return "Chip not found";
        case BMP180_ERR_NOT_INIT:  return "Not initialised";
        case BMP180_ERR_I2C:       return "I2C error";
        case BMP180_ERR_BAD_DATA:  return "Bad calibration data / degenerate coefficient";
        case BMP180_ERR_TIMEOUT:   return "I2C timeout";
        case BMP180_ERR_SEQUENCE:  return "Async API called out of sequence";
        case BMP180_ERR_BAD_PARAM: return "Invalid parameter";
        default:                   return "Unknown error";
    }
}
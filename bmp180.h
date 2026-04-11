/**
 * @file bmp180.h
 * @brief Full-featured BMP180 barometric pressure sensor library
 *        for the Raspberry Pi Pico C/C++ SDK.
 *
 * Part of: pico-bmp180-advanced-C
 *
 * Features:
 *  - Device-struct based — no global state, supports multiple sensors
 *  - Consistent error codes on every function
 *  - Compile-time warning if a status return value is discarded
 *  - All four oversampling modes
 *  - Soft reset
 *  - Temperature, pressure, and altitude readings
 *  - Software-level averaging for noise reduction
 *  - Async (non-blocking) trigger / poll / fetch API
 *  - Timeout-safe I2C — a hung bus cannot freeze the MCU
 *  - hPa <-> mmHg conversion utility
 *  - Thread-safe — embedded mutex_t protects all device state
 *
 * Wiring (I2C):
 *  VCC -> 3.3V (do NOT connect to 5V — will damage the chip)
 *  GND -> GND
 *  SDA -> any Pico SDA pin (e.g. GP4)
 *  SCL -> any Pico SCL pin (e.g. GP5)
 *
 * Known hardware quirks:
 *  - The BMP180 self-heats slightly — temperature reads 1-2 C high
 *    in enclosed spaces. Leave it exposed to air for best accuracy.
 *  - Altitude accuracy depends on sea level pressure which changes
 *    daily with weather. Use bmp180_set_sea_level() for best results.
 *  - I2C address is fixed at 0x77 — you cannot change it.
 *    Only one BMP180 per I2C bus is possible.
 *
 * Thread / multicore safety:
 *  This library is thread-safe for the blocking API. Each device struct
 *  contains its own mutex_t. Concurrent calls from two cores or two
 *  FreeRTOS tasks on the same device struct will block until the first
 *  call completes. Calls on different device structs never contend.
 *
 *  The async API (trigger / data_ready / fetch) holds the mutex only
 *  for the duration of each individual call, not across the full
 *  trigger -> poll -> fetch sequence. If you use the async API from
 *  multiple cores on the same device, you must hold an external mutex
 *  across the full sequence yourself to prevent one core from issuing
 *  a new trigger between another core's trigger and fetch.
 *  For multicore use, the blocking API (bmp180_read_all etc.) is
 *  recommended — it holds the lock for the entire measurement.
 *
 * Async API sequencing:
 *  Temperature and pressure compensation share an intermediate value
 *  (B5 / UT). For the async API, temperature MUST be fetched before
 *  pressure can be triggered:
 *
 *    bmp180_trigger_temperature(&dev);
 *    while (!ready) bmp180_data_ready(&dev, &ready);
 *    bmp180_fetch_temperature(&dev, &t);      // caches raw UT internally
 *
 *    bmp180_trigger_pressure(&dev);           // fails with BMP180_ERR_SEQUENCE
 *    while (!ready) bmp180_data_ready(&dev, &ready);  // if _temp_valid is false
 *    bmp180_fetch_pressure(&dev, &p);
 *
 *  The blocking helpers (bmp180_read_temperature, bmp180_read_pressure,
 *  bmp180_read_all) also keep the internal UT cache up to date, so mixing
 *  blocking and async calls is safe as long as the order above is respected.
 *
 * @author  Prerak Timbadiya
 * @version 2.1.0
 */

#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"
#include "pico/mutex.h"

// ── Compiler hints ────────────────────────────────────────────────

/**
 * @brief Marks a function such that the compiler warns if its return
 *        value is silently discarded.
 *
 * Every function returning bmp180_status_t is annotated with this.
 * If you intentionally want to ignore a result, cast to (void).
 */
#if defined(__GNUC__) || defined(__clang__)
#  define BMP180_MUST_CHECK __attribute__((warn_unused_result))
#else
#  define BMP180_MUST_CHECK
#endif

// ── Constants ─────────────────────────────────────────────────────

#define BMP180_I2C_ADDR       0x77
#define BMP180_CHIP_ID        0x55    ///< Value REG_ID always returns
#define BMP180_SEA_LEVEL_HPA  1013.25f

/**
 * @brief Per-transaction I2C timeout in microseconds.
 *
 * Applies to every read and write. At 400 kHz, a 22-byte calibration
 * burst takes ~0.5 ms, so 10 ms is very generous. Raise this only if
 * you are running at a lower I2C clock speed.
 */
#define BMP180_I2C_TIMEOUT_US 10000u

/**
 * @brief Barometric formula exponent (1 / 5.255).
 *
 * Used in bmp180_pressure_to_altitude. Its reciprocal (5.255) is used
 * in bmp180_sea_level_from_altitude. Both constants are derived from
 * the same value so the two functions are mathematical inverses.
 */
#define BMP180_BARO_POW   5.255f
#define BMP180_BARO_EXP   (1.0f / BMP180_BARO_POW)

// ── Error codes ───────────────────────────────────────────────────

/**
 * @brief Return codes for all BMP180 functions.
 *
 * Every function that can fail returns one of these.
 * Always check the return value — never assume success.
 * The BMP180_MUST_CHECK annotation will produce a compiler warning
 * if you forget.
 */
typedef enum {
    BMP180_OK             =  0,  ///< Success
    BMP180_ERR_NOT_FOUND  = -1,  ///< Chip not found / wrong ID
    BMP180_ERR_NOT_INIT   = -2,  ///< bmp180_init() not called yet
    BMP180_ERR_I2C        = -3,  ///< I2C read/write failed
    BMP180_ERR_BAD_DATA   = -4,  ///< Calibration data looks invalid (or B4 == 0)
    BMP180_ERR_TIMEOUT    = -5,  ///< I2C transaction timed out
    BMP180_ERR_SEQUENCE   = -6,  ///< Async API called out of order
    BMP180_ERR_BAD_PARAM  = -7,  ///< Invalid argument passed to function
} bmp180_status_t;

// ── Oversampling modes ────────────────────────────────────────────

/**
 * @brief Oversampling setting for pressure measurement.
 *
 * Higher OSS = more internal averaging = lower noise, but slower.
 * Temperature is always measured at the same speed regardless of OSS.
 *
 * | Mode            | Samples | Conv. time | RMS noise |
 * |-----------------|---------|------------|-----------|
 * | BMP180_OSS_LOW  |    1    |   4.5 ms   |  0.06 hPa |
 * | BMP180_OSS_STD  |    2    |   7.5 ms   |  0.05 hPa |
 * | BMP180_OSS_HIGH |    4    |  13.5 ms   |  0.04 hPa |
 * | BMP180_OSS_MAX  |    8    |  25.5 ms   |  0.03 hPa |
 */
typedef enum {
    BMP180_OSS_LOW  = 0,   ///< Ultra low power
    BMP180_OSS_STD  = 1,   ///< Standard (good default)
    BMP180_OSS_HIGH = 2,   ///< High resolution
    BMP180_OSS_MAX  = 3,   ///< Ultra high resolution
} bmp180_oss_t;

// ── Async measurement state ───────────────────────────────────────

/**
 * @brief Tracks which async measurement is currently pending.
 *
 * Internal use. Set by bmp180_trigger_*, cleared by bmp180_fetch_*.
 */
typedef enum {
    BMP180_MEAS_NONE        = 0,
    BMP180_MEAS_TEMPERATURE = 1,
    BMP180_MEAS_PRESSURE    = 2,
} bmp180_meas_t;

// ── Calibration data ──────────────────────────────────────────────

/**
 * @brief Factory calibration coefficients stored in chip EEPROM.
 *
 * These are unique per chip, read once during init in a single burst
 * transaction, and used to compensate every reading. Do not modify.
 */
typedef struct {
    int16_t  AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t  B1,  B2;
    int16_t  MB,  MC,  MD;
} bmp180_calib_t;

// ── Device struct ─────────────────────────────────────────────────

/**
 * @brief BMP180 device handle.
 *
 * Allocate one of these per sensor. Pass a pointer to every library
 * function. Do not modify any field directly.
 *
 * Fields prefixed with _ are internal state; treat as opaque.
 *
 * Example:
 * @code
 *   bmp180_t sensor;
 *   bmp180_init(&sensor, i2c0, BMP180_OSS_STD);
 * @endcode
 */
typedef struct {
    i2c_inst_t    *i2c;           ///< I2C peripheral (i2c0 or i2c1)
    bmp180_calib_t calib;         ///< Factory calibration data
    bmp180_oss_t   oss;           ///< Active oversampling mode
    float          sea_level_hpa; ///< Reference pressure for altitude
    bool           initialised;   ///< Guard against uninitialised use

    /* Internal async state — do not access directly */
    int32_t       _raw_temp;      ///< Cached UT for pressure compensation
    bool          _temp_valid;    ///< Whether _raw_temp holds a fresh reading
    bmp180_meas_t _meas;          ///< Type of the pending async measurement

    /* Thread safety — do not access directly */
    mutex_t       _mutex;         ///< Protects all device state and I2C access
} bmp180_t;

// ── Core blocking API ─────────────────────────────────────────────

/**
 * @brief Initialise a BMP180 device.
 *
 * Verifies the chip ID, reads all 11 calibration coefficients in a
 * single burst I2C transaction, and validates them.
 * The I2C peripheral must already be initialised with i2c_init()
 * and the SDA/SCL pins configured before calling this.
 *
 * @param dev   Pointer to an uninitialised bmp180_t struct
 * @param i2c   I2C peripheral to use (i2c0 or i2c1)
 * @param oss   Oversampling mode for pressure readings
 * @return      BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_init(bmp180_t *dev,
                                               i2c_inst_t *i2c,
                                               bmp180_oss_t oss);

/**
 * @brief Soft-reset the BMP180.
 *
 * Equivalent to a power cycle. Clears all measurement state on the
 * chip. You must call bmp180_init() again after this.
 *
 * @param dev   Device handle (need not be fully initialised)
 * @return      BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_reset(bmp180_t *dev);

/**
 * @brief Change the oversampling mode after initialisation.
 *
 * @param dev   Initialised device handle
 * @param oss   New oversampling mode
 */
void bmp180_set_oss(bmp180_t *dev, bmp180_oss_t oss);

/**
 * @brief Update the sea level pressure reference.
 *
 * Set this from a local weather service for accurate absolute
 * altitude. Default is 1013.25 hPa (standard atmosphere).
 *
 * @param dev           Initialised device handle
 * @param sea_level_hpa Current sea level pressure in hPa
 */
void bmp180_set_sea_level(bmp180_t *dev, float sea_level_hpa);

/**
 * @brief Read compensated temperature (blocking).
 *
 * Blocks for ~5 ms while the chip performs the conversion.
 * Also updates the internal raw temperature cache used by the
 * async pressure fetch path.
 *
 * @param dev   Initialised device handle
 * @param out   Output: temperature in degrees Celsius
 * @return      BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_read_temperature(bmp180_t *dev,
                                                           float *out);

/**
 * @brief Read compensated pressure (blocking).
 *
 * Internally reads temperature first (required for compensation),
 * then reads pressure. Blocks for 5 ms + OSS conversion time.
 * Also updates the internal raw temperature cache.
 *
 * @param dev   Initialised device handle
 * @param out   Output: pressure in hPa
 * @return      BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_read_pressure(bmp180_t *dev,
                                                        float *out);

/**
 * @brief Read both temperature and pressure in one operation (blocking).
 *
 * More efficient than calling both separately — only one temperature
 * measurement is taken (which is needed anyway for pressure compensation).
 * Updates the internal raw temperature cache.
 *
 * @param dev       Initialised device handle
 * @param temp_out  Output: temperature in degrees Celsius
 * @param pres_out  Output: pressure in hPa
 * @return          BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_read_all(bmp180_t *dev,
                                                   float *temp_out,
                                                   float *pres_out);

// ── Async (non-blocking) API ──────────────────────────────────────
//
// Use these when you cannot afford to block the MCU during sensor
// conversions (e.g. you are running other tasks or driving a display).
//
// The BMP180 does not have a DRDY interrupt pin; you must poll
// bmp180_data_ready() or simply wait the conversion time yourself.
//
// Required sequence:
//
//   // 1. Trigger and fetch temperature (must come first)
//   bmp180_trigger_temperature(&dev);
//   bool ready = false;
//   while (!ready) {
//       // do other work here
//       bmp180_data_ready(&dev, &ready);
//   }
//   float t;
//   bmp180_fetch_temperature(&dev, &t);  // caches raw UT internally
//
//   // 2. Trigger and fetch pressure
//   bmp180_trigger_pressure(&dev);
//   ready = false;
//   while (!ready) {
//       // do other work here
//       bmp180_data_ready(&dev, &ready);
//   }
//   float p;
//   bmp180_fetch_pressure(&dev, &p);

/**
 * @brief Start a temperature measurement (non-blocking).
 *
 * Returns immediately. Call bmp180_data_ready() to check completion,
 * then bmp180_fetch_temperature() to retrieve the result.
 *
 * @param dev   Initialised device handle
 * @return      BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_trigger_temperature(bmp180_t *dev);

/**
 * @brief Start a pressure measurement (non-blocking).
 *
 * bmp180_fetch_temperature() MUST have been called at least once
 * before this, because pressure compensation requires the cached
 * raw temperature (UT). Returns BMP180_ERR_SEQUENCE if it has not.
 *
 * @param dev   Initialised device handle
 * @return      BMP180_OK on success, BMP180_ERR_SEQUENCE if temperature
 *              has not been fetched yet
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_trigger_pressure(bmp180_t *dev);

/**
 * @brief Check whether the current async measurement is complete.
 *
 * Reads the SCO bit (bit 5) of the control register. When 0, the chip
 * has finished and the result can be fetched.
 *
 * Returns *ready = false with BMP180_OK if no measurement is pending.
 *
 * @param dev   Initialised device handle
 * @param ready Output: true if data is ready to fetch
 * @return      BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_data_ready(bmp180_t *dev,
                                                     bool *ready);

/**
 * @brief Retrieve a completed temperature measurement.
 *
 * Must be called after bmp180_trigger_temperature() and only once
 * bmp180_data_ready() returns true. Caches the raw ADC value for
 * use by bmp180_fetch_pressure().
 *
 * @param dev   Initialised device handle
 * @param out   Output: temperature in degrees Celsius
 * @return      BMP180_OK on success, BMP180_ERR_SEQUENCE if
 *              bmp180_trigger_temperature() was not called first
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_fetch_temperature(bmp180_t *dev,
                                                            float *out);

/**
 * @brief Retrieve a completed pressure measurement.
 *
 * Must be called after bmp180_trigger_pressure() and only once
 * bmp180_data_ready() returns true.
 *
 * @param dev   Initialised device handle
 * @param out   Output: pressure in hPa
 * @return      BMP180_OK on success, BMP180_ERR_SEQUENCE if called
 *              out of order, BMP180_ERR_BAD_DATA if compensation
 *              detects a degenerate calibration coefficient (B4 == 0)
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_fetch_pressure(bmp180_t *dev,
                                                         float *out);

// ── Altitude and pressure conversions ────────────────────────────

/**
 * @brief Calculate altitude from pressure using the barometric formula.
 *
 * Uses the sea level pressure set via bmp180_set_sea_level().
 * For higher accuracy above ~3000 m, use bmp180_altitude_icao()
 * from bmp180_advanced.h instead.
 *
 * @param dev           Initialised device handle
 * @param pressure_hpa  Pressure reading in hPa
 * @return              Altitude in metres
 */
float bmp180_pressure_to_altitude(bmp180_t *dev, float pressure_hpa);

/**
 * @brief Calculate what sea level pressure must be given a known altitude.
 *
 * If you know you are at exactly N metres, pass your current pressure
 * reading and N here, then feed the result to bmp180_set_sea_level().
 *
 * @param pressure_hpa  Current pressure reading in hPa
 * @param altitude_m    Your known true altitude in metres
 * @return              Estimated sea level pressure in hPa
 */
float bmp180_sea_level_from_altitude(float pressure_hpa, float altitude_m);

/**
 * @brief Convert pressure in hPa to millimetres of mercury (mmHg).
 *
 * @param hpa   Pressure in hPa
 * @return      Pressure in mmHg
 */
static inline float bmp180_hpa_to_mmhg(float hpa) {
    return hpa * 0.750064f;
}

// ── Averaging helpers ─────────────────────────────────────────────

/**
 * @brief Read averaged temperature over N samples (blocking).
 *
 * Takes N readings with a delay between each and returns the mean.
 * Holds the device mutex for the entire averaging window, preventing
 * any other core from interleaving measurements. With large delay_ms
 * values this can block other cores for an extended period — keep
 * delay_ms small (< 500 ms) when running on multiple cores.
 *
 * @param dev           Initialised device handle
 * @param samples       Number of samples to average (1-64)
 * @param delay_ms      Delay between samples in milliseconds
 * @param out           Output: averaged temperature in Celsius
 * @return              BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_read_temperature_avg(
    bmp180_t *dev, uint8_t samples, uint32_t delay_ms, float *out);

/**
 * @brief Read averaged pressure over N samples (blocking).
 *
 * Reads temperature once at the start (required for compensation),
 * then takes N pressure samples with a delay between each.
 * Holds the device mutex for the entire averaging window. With large
 * delay_ms values this can block other cores for an extended period —
 * keep delay_ms small (< 500 ms) when running on multiple cores.
 *
 * @param dev           Initialised device handle
 * @param samples       Number of samples to average (1-64)
 * @param delay_ms      Delay between samples in milliseconds
 * @param out           Output: averaged pressure in hPa
 * @return              BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_read_pressure_avg(
    bmp180_t *dev, uint8_t samples, uint32_t delay_ms, float *out);

// ── Debug helpers ─────────────────────────────────────────────────

/**
 * @brief Print all calibration coefficients over serial.
 *
 * Requires stdio to be initialised (stdio_init_all()).
 *
 * @param dev   Initialised device handle
 */
void bmp180_dump_calibration(bmp180_t *dev);

/**
 * @brief Convert a bmp180_status_t to a human-readable string.
 *
 * @param status    Error code
 * @return          Null-terminated string description
 */
const char *bmp180_status_str(bmp180_status_t status);

#endif // BMP180_H
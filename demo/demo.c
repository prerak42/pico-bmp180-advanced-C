/**
 * @file demo.c
 * @brief Full feature demonstration for the pico-bmp180 library.
 *
 * Demonstrates every major API feature in five stages:
 *
 *   Stage 1 — Basic blocking read (simplest usage)
 *   Stage 2 — Unit conversion and sea level calibration
 *   Stage 3 — Software averaging
 *   Stage 4 — Non-blocking async API
 *   Stage 5 — Advanced: weather trend, session statistics, ICAO altitude
 *
 * Wiring:
 *   BMP180 VCC  -> Pico Pin 36 (3V3)
 *   BMP180 GND  -> Pico GND
 *   BMP180 SDA  -> Pico Pin 6 (GP4)
 *   BMP180 SCL  -> Pico Pin 7 (GP5)
 *
 * Build: ensure CMakeLists links bmp180, hardware_i2c, pico_sync, m
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bmp180.h"
#include "bmp180_advanced.h"

// ── Pin and I2C configuration ─────────────────────────────────────

#define I2C_PORT  i2c0
#define I2C_SDA   4
#define I2C_SCL   5

// ── Demo configuration ────────────────────────────────────────────

// If you know your true altitude in metres, set it here and uncomment
// the calibration block in main(). This makes absolute altitude accurate.
// #define KNOWN_ALTITUDE_M  721.0f

// Interval between readings in the live monitoring loop (ms)
#define SAMPLE_INTERVAL_MS  2000

// Update the weather trend tracker every N samples
#define TREND_EVERY_N  3

// ── Helper: abort on fatal sensor error ──────────────────────────

static void require(bmp180_status_t s, const char *context) {
    if (s != BMP180_OK) {
        printf("\n[FATAL] %s: %s\n", context, bmp180_status_str(s));
        printf("Check wiring and restart.\n");
        while (true) sleep_ms(1000);
    }
}

// ── Helper: print a section header ───────────────────────────────

static void section(const char *title) {
    printf("\n");
    printf("══════════════════════════════════════════\n");
    printf("  %s\n", title);
    printf("══════════════════════════════════════════\n");
}

// ── Stage 4: async demo ───────────────────────────────────────────

static void demo_async(bmp180_t *sensor) {
    section("Stage 4 — Non-blocking Async API");

    printf("Triggering temperature measurement...\n");
    require(bmp180_trigger_temperature(sensor), "trigger_temperature");

    // While waiting for the sensor, you would normally do other work.
    // Here we just poll — but in a real application this loop body
    // would drive a display, read buttons, etc.
    bool ready = false;
    while (!ready) {
        require(bmp180_data_ready(sensor, &ready), "data_ready");
        tight_loop_contents();  // yield to other SDK internals
    }

    float t;
    require(bmp180_fetch_temperature(sensor, &t), "fetch_temperature");
    printf("Temperature fetched: %.2f C\n", t);

    // Temperature must always be fetched before pressure can be triggered.
    // Attempting bmp180_trigger_pressure() without fetching temperature first
    // returns BMP180_ERR_SEQUENCE.
    printf("Triggering pressure measurement...\n");
    require(bmp180_trigger_pressure(sensor), "trigger_pressure");

    ready = false;
    while (!ready) {
        require(bmp180_data_ready(sensor, &ready), "data_ready");
        tight_loop_contents();
    }

    float p;
    require(bmp180_fetch_pressure(sensor, &p), "fetch_pressure");
    printf("Pressure fetched: %.2f hPa (%.2f mmHg)\n",
           p, bmp180_hpa_to_mmhg(p));
    printf("Altitude (simple):  %.1f m\n",
           bmp180_pressure_to_altitude(sensor, p));
    printf("Altitude (ICAO):    %.1f m\n",
           bmp180_altitude_icao(p, t, sensor->sea_level_hpa));
}

// ── Main ──────────────────────────────────────────────────────────

int main(void) {
    stdio_init_all();

    // Wait for Serial Monitor to connect before printing anything.
    // Remove this line if you want output immediately on power-up
    // (e.g. when running from battery without a PC connected).
    while (!stdio_usb_connected()) sleep_ms(100);

    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║         pico-bmp180 Feature Demo         ║\n");
    printf("╚══════════════════════════════════════════╝\n");

    // ── I2C initialisation ────────────────────────────────────────
    // Must happen before bmp180_init(). 400 kHz = I2C fast mode.
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // I2C is an open-drain bus — pull-ups are required on SDA and SCL.
    // The BMP180 module usually has them on-board, but enabling the
    // Pico's internal ones ensures the bus works regardless.
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // ── Sensor initialisation ─────────────────────────────────────
    // BMP180_OSS_HIGH gives 13.5 ms conversion time and 0.04 hPa RMS noise.
    // Good default for a demo that isn't battery-constrained.
    bmp180_t sensor;
    require(bmp180_init(&sensor, I2C_PORT, BMP180_OSS_HIGH), "bmp180_init");
    printf("BMP180 initialised OK.\n");

    // ── Calibration dump (debug) ──────────────────────────────────
    // Prints all 11 factory calibration coefficients read from EEPROM.
    // Useful for verifying the chip is genuine and communication is clean.
    bmp180_dump_calibration(&sensor);

    // ── Optional: calibrate altitude from known position ──────────
    // If you know your true altitude, uncomment this block.
    // It sets the sea level reference so absolute altitude is accurate
    // for today's atmospheric conditions.
    //
    // float pressure_now;
    // require(bmp180_read_pressure(&sensor, &pressure_now), "read_pressure");
    // float sea_lvl = bmp180_sea_level_from_altitude(pressure_now,
    //                                                 KNOWN_ALTITUDE_M);
    // bmp180_set_sea_level(&sensor, sea_lvl);
    // printf("Sea level calibrated to: %.2f hPa\n\n", sea_lvl);

    // ─────────────────────────────────────────────────────────────
    section("Stage 1 — Basic Blocking Read");
    // ─────────────────────────────────────────────────────────────

    // bmp180_read_all() is the most efficient way to read both values.
    // It measures temperature once and reuses it for pressure compensation,
    // avoiding the redundant temperature read that bmp180_read_pressure()
    // alone would need.
    float temp, pressure;
    require(bmp180_read_all(&sensor, &temp, &pressure), "read_all");

    float altitude = bmp180_pressure_to_altitude(&sensor, pressure);

    printf("Temperature : %.2f C\n",   temp);
    printf("Pressure    : %.2f hPa\n", pressure);
    printf("Altitude    : %.1f m\n",   altitude);

    // ─────────────────────────────────────────────────────────────
    section("Stage 2 — Unit Conversion and mmHg Output");
    // ─────────────────────────────────────────────────────────────

    float pressure_mmhg;
    require(bmp180_read_pressure_mmhg(&sensor, &pressure_mmhg),
            "read_pressure_mmhg");

    // bmp180_hpa_to_mmhg() is also available as an inline utility for
    // converting any hPa value you already have.
    printf("Pressure    : %.2f hPa  =  %.2f mmHg\n",
           pressure, bmp180_hpa_to_mmhg(pressure));
    printf("From mmHg fn: %.2f mmHg\n", pressure_mmhg);

    // ICAO altitude incorporates measured temperature into the formula,
    // making it more accurate than the simple barometric formula,
    // especially above ~3000 m.
    float alt_icao = bmp180_altitude_icao(pressure, temp,
                                           sensor.sea_level_hpa);
    printf("Altitude (simple): %.1f m\n", altitude);
    printf("Altitude (ICAO):   %.1f m\n", alt_icao);

    // ─────────────────────────────────────────────────────────────
    section("Stage 3 — Software Averaging");
    // ─────────────────────────────────────────────────────────────

    printf("Taking 8-sample temperature average (50 ms apart)...\n");
    float avg_temp;
    require(bmp180_read_temperature_avg(&sensor, 8, 50, &avg_temp),
            "read_temperature_avg");
    printf("Average temperature: %.2f C\n", avg_temp);

    printf("Taking 8-sample pressure average (50 ms apart)...\n");
    float avg_pressure;
    require(bmp180_read_pressure_avg(&sensor, 8, 50, &avg_pressure),
            "read_pressure_avg");
    printf("Average pressure:    %.2f hPa\n", avg_pressure);
    printf("Average altitude:    %.1f m\n",
           bmp180_pressure_to_altitude(&sensor, avg_pressure));

    // ─────────────────────────────────────────────────────────────
    // Stage 4 is in its own function above
    demo_async(&sensor);
    // ─────────────────────────────────────────────────────────────

    // ─────────────────────────────────────────────────────────────
    section("Stage 5 — Advanced: Trend, Stats, Live Monitoring");
    // ─────────────────────────────────────────────────────────────

    // Initialise the trend tracker.
    // In a real weather station you'd call bmp180_trend_update() every
    // 5-10 minutes. Here we call it every TREND_EVERY_N samples just
    // to populate it for the demo.
    bmp180_trend_tracker_t trend;
    bmp180_trend_init(&trend);

    // Initialise the session statistics tracker.
    bmp180_stats_t stats;
    bmp180_stats_reset(&stats);

    uint32_t count = 0;

    printf("Live readings every %d ms. Ctrl+C or reset to stop.\n\n",
           SAMPLE_INTERVAL_MS);

    while (true) {
        float t, p;
        bmp180_status_t s = bmp180_read_all(&sensor, &t, &p);

        if (s != BMP180_OK) {
            printf("[%4lu] Read error: %s\n",
                   (unsigned long)count, bmp180_status_str(s));
            sleep_ms(SAMPLE_INTERVAL_MS);
            continue;
        }

        float alt_simple = bmp180_pressure_to_altitude(&sensor, p);
        float alt_icao   = bmp180_altitude_icao(p, t, sensor.sea_level_hpa);
        float p_mmhg     = bmp180_hpa_to_mmhg(p);

        // Update statistics every reading
        bmp180_stats_update(&stats, t, p);
        count++;

        // Update trend every N readings
        if (count % TREND_EVERY_N == 0) {
            bmp180_trend_update(&trend, p, time_us_32() / 1000);
        }

        bmp180_trend_t trend_now = bmp180_trend_get(&trend);
        float rate               = bmp180_trend_rate(&trend);

        // Main reading line
        printf("[%4lu] T: %5.2f C | P: %7.2f hPa (%6.2f mmHg)"
               " | Alt: %6.1f m (ICAO: %6.1f m)\n",
               (unsigned long)count, t, p, p_mmhg, alt_simple, alt_icao);

        // Trend line
        printf("       Trend: %-26s | Rate: %+.3f hPa/hr\n",
               bmp180_trend_str(trend_now), rate);

        // Print session statistics every 10 readings
        if (count % 10 == 0) {
            printf("\n");
            bmp180_stats_print(&stats);
            printf("\n");
        }

        sleep_ms(SAMPLE_INTERVAL_MS);
    }
}
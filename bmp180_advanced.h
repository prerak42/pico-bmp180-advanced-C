/**
 * @file bmp180_advanced.h
 * @brief Advanced features for the BMP180 library.
 *
 * Includes:
 *  - Weather trend detection (rising/falling pressure)
 *    Uses least-squares linear regression over the full history window
 *    rather than a naive oldest-vs-newest comparison, giving a more
 *    stable and noise-resistant rate estimate.
 *  - ICAO standard atmosphere altitude model (more accurate than the
 *    simple barometric formula, especially above ~3000 m)
 *  - Session statistics (min/max/average for temperature and pressure)
 *  - mmHg pressure output variants
 *
 * Thread / multicore safety:
 *  bmp180_trend_tracker_t and bmp180_stats_t each embed a
 *  critical_section_t. All update and read functions acquire this
 *  section for the duration of their access, making them safe to call
 *  from both RP2040/RP2350 cores concurrently without external locking.
 *
 *  critical_section vs mutex: these structs are updated with simple
 *  array writes and arithmetic, not I2C transactions, so the shorter
 *  critical section (which disables interrupts on the calling core and
 *  spin-waits the other) is more appropriate than a full mutex.
 */

#ifndef BMP180_ADVANCED_H
#define BMP180_ADVANCED_H

#include "bmp180.h"
#include "pico/critical_section.h"

// ── Weather trend ─────────────────────────────────────────────────

/**
 * @brief Weather trend classification.
 *
 * A pressure drop of more than 0.5 hPa/hour generally indicates
 * approaching bad weather. A rise indicates improving conditions.
 * These thresholds match the enjoyneering library and common
 * meteorological practice for consumer sensors.
 */
typedef enum {
    BMP180_TREND_UNKNOWN  =  0,
    BMP180_TREND_RISING   =  1,   ///< Pressure rising  -> improving weather
    BMP180_TREND_FALLING  = -1,   ///< Pressure falling -> worsening weather
    BMP180_TREND_STABLE   =  2,   ///< Change less than 0.5 hPa/hour
} bmp180_trend_t;

/**
 * @brief Circular-buffer weather trend tracker.
 *
 * Holds a rolling window of pressure readings and their timestamps.
 * bmp180_trend_rate() fits a least-squares line to the full window
 * rather than just comparing the oldest and newest points, which
 * makes the rate estimate significantly more robust to single noisy
 * readings.
 *
 * Call bmp180_trend_update() at regular intervals (e.g. every 5
 * minutes). The tracker needs at least 2 readings before it can
 * produce a meaningful trend.
 *
 * Thread safety: all functions that take a pointer to this struct
 * are thread-safe via the embedded critical_section_t.
 */
#define BMP180_TREND_HISTORY  12   ///< Number of samples in the rolling window

typedef struct {
    float    history[BMP180_TREND_HISTORY];     ///< Circular pressure buffer
    uint32_t timestamps[BMP180_TREND_HISTORY];  ///< Corresponding times (ms)
    uint8_t  head;                              ///< Next write index
    uint8_t  count;                             ///< Number of valid entries

    /* Thread safety — do not access directly */
    critical_section_t _cs;                     ///< Protects all fields above
} bmp180_trend_tracker_t;

/**
 * @brief Initialise a trend tracker to a clean state.
 *
 * Must be called before any other trend function.
 * Initialises the embedded critical section as well as clearing data.
 *
 * @param tracker   Pointer to an uninitialised tracker struct
 */
void bmp180_trend_init(bmp180_trend_tracker_t *tracker);

/**
 * @brief Push a new pressure reading into the trend tracker.
 *
 * Call this at regular intervals. Irregular intervals are handled
 * correctly because the regression uses actual timestamps.
 * Thread-safe.
 *
 * @param tracker       Trend tracker
 * @param pressure_hpa  Current pressure reading in hPa
 * @param time_ms       Current time in ms (use time_us_32() / 1000)
 */
void bmp180_trend_update(bmp180_trend_tracker_t *tracker,
                          float pressure_hpa,
                          uint32_t time_ms);

/**
 * @brief Get the current weather trend classification.
 *
 * Requires at least 2 readings. Thread-safe.
 *
 * @param tracker   Trend tracker
 * @return          Current trend, or BMP180_TREND_UNKNOWN if not enough data
 */
bmp180_trend_t bmp180_trend_get(bmp180_trend_tracker_t *tracker);

/**
 * @brief Get pressure change rate using least-squares linear regression.
 *
 * Fits a line to all samples in the history window and returns the
 * slope in hPa per hour. Negative means falling pressure.
 * Returns 0.0 if fewer than 2 readings are available.
 * Thread-safe.
 *
 * @param tracker   Trend tracker
 * @return          Rate of change in hPa/hour
 */
float bmp180_trend_rate(bmp180_trend_tracker_t *tracker);

/**
 * @brief Convert a trend value to a human-readable string.
 *
 * @param trend     Trend value
 * @return          Null-terminated string
 */
const char *bmp180_trend_str(bmp180_trend_t trend);

// ── Session statistics ────────────────────────────────────────────

/**
 * @brief Tracks min, max, and running average for temperature and pressure.
 *
 * Reset with bmp180_stats_reset() at the start of each session or day.
 *
 * Thread safety: all functions that take a pointer to this struct
 * are thread-safe via the embedded critical_section_t.
 */
typedef struct {
    float    temp_min, temp_max, temp_sum;
    float    pres_min, pres_max, pres_sum;
    uint32_t count;

    /* Thread safety — do not access directly */
    critical_section_t _cs;                     ///< Protects all fields above
} bmp180_stats_t;

/**
 * @brief Reset a stats struct to its initial state.
 *
 * Must be called before any other stats function.
 * Initialises the embedded critical section as well as clearing data.
 *
 * @param stats     Stats struct to reset
 */
void bmp180_stats_reset(bmp180_stats_t *stats);

/**
 * @brief Push a new reading pair into the stats tracker.
 *
 * Thread-safe.
 *
 * @param stats     Stats struct
 * @param temp      Temperature reading in Celsius
 * @param pressure  Pressure reading in hPa
 */
void bmp180_stats_update(bmp180_stats_t *stats, float temp, float pressure);

/**
 * @brief Print a formatted stats summary over serial.
 *
 * Takes a consistent snapshot under the critical section before
 * printing, so the output is internally consistent even if another
 * core is updating the struct concurrently.
 * Requires stdio to be initialised.
 *
 * @param stats     Stats struct
 */
void bmp180_stats_print(bmp180_stats_t *stats);

// ── ICAO altitude model ───────────────────────────────────────────

/**
 * @brief Calculate altitude using the ICAO standard atmosphere model.
 *
 * More accurate than the simple barometric formula in bmp180.h,
 * particularly above ~3000 m. Incorporates measured temperature to
 * account for departures from the standard lapse rate.
 *
 * Formula:
 *   H = (T / L) * (1 - (P / P0) ^ (R*L/g))
 * where:
 *   T  = temperature at measurement point in Kelvin
 *   L  = standard lapse rate = 0.0065 K/m
 *   R*L/g = 0.190263 (dimensionless exponent)
 *
 * This function is stateless and requires no locking.
 *
 * @param pressure_hpa      Current pressure in hPa
 * @param temperature_c     Current temperature in Celsius
 * @param sea_level_hpa     Sea level reference pressure in hPa
 * @return                  Altitude in metres
 */
float bmp180_altitude_icao(float pressure_hpa,
                            float temperature_c,
                            float sea_level_hpa);

// ── mmHg output variants ──────────────────────────────────────────
//
// These are thin wrappers around the hPa functions. They call the
// same underlying measurement and convert the result.
// Thread safety is inherited from the underlying bmp180_t mutex.

/**
 * @brief Read compensated pressure in millimetres of mercury (blocking).
 *
 * Equivalent to bmp180_read_pressure() followed by bmp180_hpa_to_mmhg().
 *
 * @param dev   Initialised device handle
 * @param out   Output: pressure in mmHg
 * @return      BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_read_pressure_mmhg(bmp180_t *dev,
                                                              float *out);

/**
 * @brief Read both temperature and pressure, with pressure in mmHg (blocking).
 *
 * @param dev       Initialised device handle
 * @param temp_out  Output: temperature in Celsius
 * @param pres_out  Output: pressure in mmHg
 * @return          BMP180_OK on success, error code otherwise
 */
BMP180_MUST_CHECK bmp180_status_t bmp180_read_all_mmhg(bmp180_t *dev,
                                                         float *temp_out,
                                                         float *pres_out);

/**
 * @brief Calculate sea level pressure in mmHg given a known altitude.
 *
 * @param pressure_mmhg Current pressure in mmHg
 * @param altitude_m    Known true altitude in metres
 * @return              Estimated sea level pressure in mmHg
 */
float bmp180_sea_level_from_altitude_mmhg(float pressure_mmhg,
                                           float altitude_m);

#endif // BMP180_ADVANCED_H

#include "bmp180_advanced.h"
#include <stdint.h>
#include <stddef.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/critical_section.h"

// ── Trend tracker ─────────────────────────────────────────────────

void bmp180_trend_init(bmp180_trend_tracker_t *tracker) {
    // Initialise the critical section before zeroing the rest of the
    // struct, so that LOCK/UNLOCK are safe from this point forward.
    critical_section_init(&tracker->_cs);
    tracker->head  = 0;
    tracker->count = 0;
    memset(tracker->history,    0, sizeof(tracker->history));
    memset(tracker->timestamps, 0, sizeof(tracker->timestamps));
}

void bmp180_trend_update(bmp180_trend_tracker_t *tracker,
                          float pressure_hpa,
                          uint32_t time_ms) {
    critical_section_enter_blocking(&tracker->_cs);
    tracker->history[tracker->head]    = pressure_hpa;
    tracker->timestamps[tracker->head] = time_ms;
    tracker->head = (tracker->head + 1) % BMP180_TREND_HISTORY;
    if (tracker->count < BMP180_TREND_HISTORY) tracker->count++;
    critical_section_exit(&tracker->_cs);
}

float bmp180_trend_rate(bmp180_trend_tracker_t *tracker) {
    // Copy the volatile fields under the lock, then compute outside it.
    // The regression is purely arithmetic — no reason to hold the lock
    // across the entire calculation.
    critical_section_enter_blocking(&tracker->_cs);
    uint8_t  n    = tracker->count;
    uint8_t  head = tracker->head;
    float    history_copy[BMP180_TREND_HISTORY];
    uint32_t ts_copy[BMP180_TREND_HISTORY];
    memcpy(history_copy, tracker->history,    sizeof(history_copy));
    memcpy(ts_copy,      tracker->timestamps, sizeof(ts_copy));
    critical_section_exit(&tracker->_cs);

    if (n < 2) return 0.0f;

    // Index of the oldest sample in the circular buffer
    uint8_t base = (n < BMP180_TREND_HISTORY) ? 0 : head;

    // Normalise timestamps relative to the oldest sample to keep
    // floating-point values small and preserve precision.
    // (uint32_t subtraction wraps correctly even across a rollover.)
    uint32_t t0 = ts_copy[base];

    // Least-squares linear regression: fit y = a + b*x where
    //   x = elapsed time in ms (relative)
    //   y = pressure in hPa
    // and we want slope b in hPa/ms, then convert to hPa/hour.
    //
    // b = (n*Sxy - Sx*Sy) / (n*Sxx - Sx*Sx)
    float Sx  = 0.0f;
    float Sy  = 0.0f;
    float Sxx = 0.0f;
    float Sxy = 0.0f;

    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (base + i) % BMP180_TREND_HISTORY;
        float x = (float)(ts_copy[idx] - t0);  // ms, relative
        float y = history_copy[idx];
        Sx  += x;
        Sy  += y;
        Sxx += x * x;
        Sxy += x * y;
    }

    float fn    = (float)n;
    float denom = fn * Sxx - Sx * Sx;

    // If all timestamps are identical (e.g. a repeated call with the same
    // time_ms), denom is zero. Return 0 rather than divide.
    if (fabsf(denom) < 1.0f) return 0.0f;

    float slope_per_ms = (fn * Sxy - Sx * Sy) / denom;

    // Convert slope from hPa/ms to hPa/hour
    return slope_per_ms * 3600000.0f;
}

bmp180_trend_t bmp180_trend_get(bmp180_trend_tracker_t *tracker) {
    // bmp180_trend_rate() already acquires the lock internally,
    // so we just call it directly. The count check below is a quick
    // early-out that avoids the lock acquisition in the common case
    // of not yet having enough data. A race here is harmless — if count
    // races from 1 to 2, bmp180_trend_rate() handles n < 2 itself.
    if (tracker->count < 2) return BMP180_TREND_UNKNOWN;
    float rate = bmp180_trend_rate(tracker);
    if      (rate >  0.5f) return BMP180_TREND_RISING;
    else if (rate < -0.5f) return BMP180_TREND_FALLING;
    else                   return BMP180_TREND_STABLE;
}

const char *bmp180_trend_str(bmp180_trend_t trend) {
    switch (trend) {
        case BMP180_TREND_RISING:  return "Rising   (improving)";
        case BMP180_TREND_FALLING: return "Falling  (worsening)";
        case BMP180_TREND_STABLE:  return "Stable";
        default:                   return "Unknown (need more data)";
    }
}

// ── Session statistics ────────────────────────────────────────────

void bmp180_stats_reset(bmp180_stats_t *stats) {
    // Initialise the critical section first.
    critical_section_init(&stats->_cs);
    stats->temp_min  =  FLT_MAX;
    stats->temp_max  = -FLT_MAX;
    stats->temp_sum  = 0.0f;
    stats->pres_min  =  FLT_MAX;
    stats->pres_max  = -FLT_MAX;
    stats->pres_sum  = 0.0f;
    stats->count     = 0;
}

void bmp180_stats_update(bmp180_stats_t *stats, float temp, float pressure) {
    critical_section_enter_blocking(&stats->_cs);
    if (temp < stats->temp_min) stats->temp_min = temp;
    if (temp > stats->temp_max) stats->temp_max = temp;
    stats->temp_sum += temp;

    if (pressure < stats->pres_min) stats->pres_min = pressure;
    if (pressure > stats->pres_max) stats->pres_max = pressure;
    stats->pres_sum += pressure;

    stats->count++;
    critical_section_exit(&stats->_cs);
}

void bmp180_stats_print(bmp180_stats_t *stats) {
    // Take a consistent snapshot under the lock, then print outside it.
    // This ensures the printed values are internally consistent even if
    // another core is calling bmp180_stats_update() concurrently, and
    // avoids holding the critical section across slow printf calls.
    critical_section_enter_blocking(&stats->_cs);
    uint32_t count    = stats->count;
    float temp_min    = stats->temp_min;
    float temp_max    = stats->temp_max;
    float temp_sum    = stats->temp_sum;
    float pres_min    = stats->pres_min;
    float pres_max    = stats->pres_max;
    float pres_sum    = stats->pres_sum;
    critical_section_exit(&stats->_cs);

    if (count == 0) {
        printf("No readings yet.\n");
        return;
    }

    float temp_avg = temp_sum / (float)count;
    float pres_avg = pres_sum / (float)count;

    printf("=== BMP180 Statistics (%lu readings) ===\n",
           (unsigned long)count);
    printf("Temperature -- Min: %.2f C   Max: %.2f C   Avg: %.2f C\n",
           temp_min, temp_max, temp_avg);
    printf("Pressure    -- Min: %.2f hPa  Max: %.2f hPa  Avg: %.2f hPa\n",
           pres_min, pres_max, pres_avg);
    printf("              Min: %.2f mmHg  Max: %.2f mmHg  Avg: %.2f mmHg\n",
           bmp180_hpa_to_mmhg(pres_min),
           bmp180_hpa_to_mmhg(pres_max),
           bmp180_hpa_to_mmhg(pres_avg));
    printf("========================================\n");
}

// ── ICAO altitude ─────────────────────────────────────────────────

float bmp180_altitude_icao(float pressure_hpa,
                            float temperature_c,
                            float sea_level_hpa) {
    // ICAO standard atmosphere formula (troposphere layer, 0-11 km):
    //   H = (T / L) * (1 - (P / P0) ^ (R*L/g))
    // where:
    //   T         = local temperature in Kelvin
    //   L         = standard temperature lapse rate = 0.0065 K/m
    //   icao_exp  = R*L/g = 0.190263 (derived from gas constant, lapse
    //               rate, and gravity; dimensionless)
    //
    // This differs from the simple barometric formula (BMP180_BARO_EXP)
    // in that it incorporates measured temperature rather than assuming
    // a fixed sea-level temperature of 288.15 K.

    float T        = temperature_c + 273.15f;  // Celsius -> Kelvin
    float L        = 0.0065f;                  // K/m lapse rate
    float icao_exp = 0.190263f;                // R*L/g exponent

    return (T / L) * (1.0f - powf(pressure_hpa / sea_level_hpa, icao_exp));
}

// ── mmHg output variants ──────────────────────────────────────────

bmp180_status_t bmp180_read_pressure_mmhg(bmp180_t *dev, float *out) {
    float hpa;
    bmp180_status_t s = bmp180_read_pressure(dev, &hpa);
    if (s != BMP180_OK) return s;
    *out = bmp180_hpa_to_mmhg(hpa);
    return BMP180_OK;
}

bmp180_status_t bmp180_read_all_mmhg(bmp180_t *dev,
                                       float *temp_out,
                                       float *pres_out) {
    float hpa;
    bmp180_status_t s = bmp180_read_all(dev, temp_out, &hpa);
    if (s != BMP180_OK) return s;
    *pres_out = bmp180_hpa_to_mmhg(hpa);
    return BMP180_OK;
}

float bmp180_sea_level_from_altitude_mmhg(float pressure_mmhg,
                                            float altitude_m) {
    // Convert input to hPa, compute, convert result back
    float hpa = pressure_mmhg / 0.750064f;
    return bmp180_hpa_to_mmhg(bmp180_sea_level_from_altitude(hpa, altitude_m));
}

# pico-bmp180

A full-featured, production-quality C library for the BMP180 barometric pressure and temperature sensor, written for the **Raspberry Pi Pico C/C++ SDK**.

> Developed by Prerak Timbadiya

---

## Features

- **Device-struct based** — no global state, supports multiple sensors simultaneously
- **Full error propagation** — every function returns a typed error code; silent failures are impossible
- **Thread-safe** — embedded `mutex_t` protects all device state; safe to call from both RP2040/RP2350 cores
- **Timeout-safe I2C** — a disconnected or hung sensor cannot freeze the MCU
- **All four oversampling modes** — from ultra-low-power to ultra-high-resolution
- **Non-blocking async API** — trigger/poll/fetch pattern for use alongside display drivers, game loops, etc.
- **Software averaging** — configurable multi-sample averages for noise reduction
- **Soft reset** — equivalent to a power cycle, programmatically
- **Calibration validation** — detects broken chips and bad I2C connections at init time
- **Calibration dump** — print all 11 factory coefficients for debugging
- **hPa ↔ mmHg conversion** — built-in unit utility
- **`BMP180_MUST_CHECK`** — GCC/Clang attribute that warns at compile time if you accidentally discard an error code

### Advanced module (`bmp180_advanced.h`)

- **Weather trend detection** — rising/falling/stable classification using least-squares linear regression over a configurable rolling window; more robust than naive oldest-vs-newest comparison
- **ICAO standard atmosphere altitude** — more accurate than the simple barometric formula, especially above ~3000 m; incorporates measured temperature
- **Session statistics** — min/max/average for both temperature and pressure, with mmHg output
- **mmHg pressure variants** — `bmp180_read_pressure_mmhg()`, `bmp180_read_all_mmhg()`, `bmp180_sea_level_from_altitude_mmhg()`
- **Thread-safe advanced structs** — `bmp180_trend_tracker_t` and `bmp180_stats_t` each embed a `critical_section_t`; all functions are safe to call from both cores without external locking

---

## Hardware Compatibility

| Board | Status |
|---|---|
| Raspberry Pi Pico (RP2040) | ✅ Fully supported |
| Raspberry Pi Pico 2 (RP2350) | ✅ Fully supported |

The library uses only `hardware_i2c`, `pico_stdlib`, and `pico_sync` — all of which are identical across RP2040 and RP2350. No board-specific code exists in this library. Set `PICO_BOARD` in your own project's CMakeLists to target the correct board.

---

## Wiring

> ⚠️ The BMP180 runs on **3.3V only**. Connecting VCC to 5V will permanently damage the chip.

| BMP180 Pin | Pico Pin | Notes |
|---|---|---|
| VCC | 3V3 (Pin 36) | 3.3V only |
| GND | GND (any) | |
| SDA | Any SDA-capable GPIO | e.g. GP4 (Pin 6) |
| SCL | Any SCL-capable GPIO | e.g. GP5 (Pin 7) |

The I2C address is fixed at **0x77**. Only one BMP180 can share a given I2C bus.

---

## Installation

### Option A — Git submodule (recommended)

```bash
cd your_project
git submodule add https://github.com/YOUR_USERNAME/pico-bmp180 libs/bmp180
```

In your `CMakeLists.txt`:
```cmake
add_subdirectory(libs/bmp180)
target_link_libraries(your_target PRIVATE bmp180)
```

### Option B — Copy files

Copy `bmp180.h`, `bmp180.c`, `bmp180_advanced.h`, `bmp180_advanced.c`, and `CMakeLists.txt` into your project directory and add them to your build.

---

## Quick Start

```c
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bmp180.h"

int main() {
    stdio_init_all();

    // 1. Initialise I2C — do this before bmp180_init()
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);  // SDA
    gpio_set_function(5, GPIO_FUNC_I2C);  // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);

    // 2. Initialise the sensor
    bmp180_t sensor;
    bmp180_status_t s = bmp180_init(&sensor, i2c0, BMP180_OSS_STD);
    if (s != BMP180_OK) {
        printf("Init failed: %s\n", bmp180_status_str(s));
        return 1;
    }

    // 3. Read temperature and pressure together
    float temp, pressure;
    s = bmp180_read_all(&sensor, &temp, &pressure);
    if (s != BMP180_OK) {
        printf("Read failed: %s\n", bmp180_status_str(s));
        return 1;
    }

    float altitude = bmp180_pressure_to_altitude(&sensor, pressure);
    printf("Temp: %.2f C  Pressure: %.2f hPa  Altitude: %.1f m\n",
           temp, pressure, altitude);

    return 0;
}
```

---

## API Reference

### Initialisation

| Function | Description |
|---|---|
| `bmp180_init(dev, i2c, oss)` | Initialise device, verify chip ID, read calibration |
| `bmp180_reset(dev)` | Soft-reset the chip (requires re-init after) |
| `bmp180_set_oss(dev, oss)` | Change oversampling mode after init |
| `bmp180_set_sea_level(dev, hpa)` | Set sea level reference for altitude |

### Blocking reads

| Function | Description |
|---|---|
| `bmp180_read_temperature(dev, &out)` | Read temperature in °C |
| `bmp180_read_pressure(dev, &out)` | Read pressure in hPa |
| `bmp180_read_all(dev, &temp, &pres)` | Read both in one operation (most efficient) |
| `bmp180_read_temperature_avg(dev, n, delay_ms, &out)` | Average N temperature samples |
| `bmp180_read_pressure_avg(dev, n, delay_ms, &out)` | Average N pressure samples |

### Non-blocking async API

```
bmp180_trigger_temperature() → bmp180_data_ready() → bmp180_fetch_temperature()
                                                    ↓
bmp180_trigger_pressure()   → bmp180_data_ready() → bmp180_fetch_pressure()
```

Temperature must always be triggered and fetched first.

### Conversions

| Function | Description |
|---|---|
| `bmp180_pressure_to_altitude(dev, hpa)` | hPa → metres (barometric formula) |
| `bmp180_sea_level_from_altitude(hpa, alt_m)` | Calibrate sea level from known altitude |
| `bmp180_hpa_to_mmhg(hpa)` | hPa → mmHg (inline utility) |

### Error codes

| Code | Meaning |
|---|---|
| `BMP180_OK` | Success |
| `BMP180_ERR_NOT_FOUND` | Chip absent or wrong chip ID |
| `BMP180_ERR_NOT_INIT` | `bmp180_init()` not called |
| `BMP180_ERR_I2C` | I2C transaction failed |
| `BMP180_ERR_BAD_DATA` | Invalid calibration or degenerate compensation |
| `BMP180_ERR_TIMEOUT` | I2C bus timed out |
| `BMP180_ERR_SEQUENCE` | Async API called out of order |
| `BMP180_ERR_BAD_PARAM` | Invalid argument (e.g. samples > 64) |

### Oversampling modes

| Mode | Conversion time | RMS noise |
|---|---|---|
| `BMP180_OSS_LOW` | 4.5 ms | 0.06 hPa |
| `BMP180_OSS_STD` | 7.5 ms | 0.05 hPa |
| `BMP180_OSS_HIGH` | 13.5 ms | 0.04 hPa |
| `BMP180_OSS_MAX` | 25.5 ms | 0.03 hPa |

---

## CMakeLists Integration

```cmake
add_subdirectory(libs/bmp180)

add_executable(my_project main.c)

target_link_libraries(my_project
    pico_stdlib
    bmp180         # ← pulls in hardware_i2c, pico_sync, and pico_critical_section automatically
)

pico_enable_stdio_usb(my_project 1)
pico_enable_stdio_uart(my_project 0)
pico_add_extra_outputs(my_project)
```

---

## Thread Safety

The blocking API (`bmp180_read_all`, `bmp180_read_temperature`, etc.) is fully thread-safe. Each device struct contains its own `mutex_t`; concurrent calls from two cores block until the first completes.

The async API holds the mutex per individual call only. If you use trigger/fetch from two cores on the same device, wrap the full sequence in an external mutex.

The `bmp180_trend_tracker_t` and `bmp180_stats_t` structs in the advanced module each embed a `critical_section_t`. All update and read functions are thread-safe — no external locking is required.

---

## Known Hardware Quirks

- **Self-heating** — the BMP180 reads 1–2°C above true air temperature in enclosed spaces due to heat from nearby components. Leave it exposed to airflow for best accuracy.
- **Altitude drift** — absolute altitude depends on sea level pressure, which changes with weather. For accurate readings, update `sea_level_hpa` from a local weather service. For *relative* altitude changes, the default is accurate regardless.
- **Fixed I2C address** — the BMP180's address (0x77) cannot be changed. Only one BMP180 per I2C bus is possible.
- **3.3V only** — connecting VCC to 5V destroys the chip instantly.

---

## License

MIT License — see `LICENSE` for details.
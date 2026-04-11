# Guide — pico-bmp180-advanced-C

This guide is included to serve the purpose of getting a working BMP180 setup

---

## The BMP180:

The BMP180 is a small sensor made by Bosch that measures two things:

- **Barometric pressure** — the weight of the atmosphere above you, in hectopascals (hPa). Drops before bad weather, rises before good weather.
- **Temperature** — the ambient temperature in degrees Celsius.

From pressure and a reference sea level value, you can also derive **altitude** — this is how aircraft altimeters work.

---

## Required:

- Raspberry Pi Pico or Pico 2
- BMP180 breakout module
- 4 jumper wires
- Breadboard (optional but convenient)

---

## 1. Wiring

The BMP180 communicates over **I2C**.

| BMP180 Label | Where to connect | Why |
|---|---|---|
| VCC | Pico Pin 36 (3V3 OUT) | Power — **must be 3.3V, never 5V** |
| GND | Any Pico GND pin | Ground reference |
| SDA | Pico Pin 6 (GP4) | Data line |
| SCL | Pico Pin 7 (GP5) | Clock line |

>  **The BMP180 will be permanently damaged if 5V is supplied to it.** Always use the 3.3V pin or a 3.3V voltage regulator

---

## 2. Adding the Library to A Project

Copy these four files into the project folder alongside `main.c`:

```
my_project/
├── main.c
├── bmp180.h
├── bmp180.c
├── bmp180_advanced.h    (optional — needed for trend/stats/ICAO)
├── bmp180_advanced.c    (optional)
└── CMakeLists.txt
```

In `CMakeLists.txt`, add `bmp180.c` to executable and link the required libraries:

```cmake
add_executable(my_project
    main.c
    bmp180.c
    bmp180_advanced.c    # only if you use bmp180_advanced.h
)

target_link_libraries(my_project
    pico_stdlib
    hardware_i2c
    pico_sync
    m                    # math library for powf()
)
```

---

## 3. Demo Program

Demo program:

```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bmp180.h"

#define I2C_SDA 4
#define I2C_SCL 5

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(100);

    // 1. Initialise I2C at 400 kHz (fast mode)
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // 2. Initialise the BMP180
    bmp180_t sensor;
    bmp180_status_t status = bmp180_init(&sensor, i2c0, BMP180_OSS_STD);
    if (status != BMP180_OK) {
        printf("Sensor not found: %s\n", bmp180_status_str(status));
        while (true) sleep_ms(1000);
    }
    printf("BMP180 ready!\n");

    // 3. Read in a loop
    while (true) {
        float temp, pressure;
        status = bmp180_read_all(&sensor, &temp, &pressure);

        if (status != BMP180_OK) {
            printf("Read error: %s\n", bmp180_status_str(status));
        } else {
            float altitude = bmp180_pressure_to_altitude(&sensor, pressure);
            printf("Temp: %.2f C  |  Pressure: %.2f hPa  |  Altitude: %.1f m\n",
                   temp, pressure, altitude);
        }

        sleep_ms(2000);
    }
}
```

---

## Explanation

**`bmp180_t sensor;`**
This declares a *device handle* — a struct that holds everything about your sensor: its calibration data, current settings, and internal state. You pass a pointer to this in every library call. Having it as a separate struct means you could have two BMP180s on two I2C buses simply by declaring two of these.

**`bmp180_init(&sensor, i2c0, BMP180_OSS_STD)`**
This does three things: verifies the chip is present by reading its ID register (should return `0x55`), reads 11 factory calibration coefficients stored in the chip's EEPROM, and checks they look valid. If anything goes wrong, it returns an error code instead of `BMP180_OK`.

**`BMP180_OSS_STD`**
The oversampling setting. The BMP180 internally averages multiple pressure samples before reporting. `STD` gives a good balance of speed and accuracy. See the README for the full table.

**`bmp180_status_t status`**
Every function that can fail returns one of these. Never ignore it. The `BMP180_MUST_CHECK` annotation will give you a compiler warning if you accidentally discard a return value.

**`bmp180_status_str(status)`**
Converts an error code to a human-readable string like `"Chip not found"` or `"I2C timeout"`. Very useful for debugging.

**`bmp180_read_all(&sensor, &temp, &pressure)`**
Reads temperature and pressure in one go. More efficient than calling them separately because temperature must be measured first anyway for the pressure compensation math to work — this function does it once and reuses the result for both outputs.

**`bmp180_pressure_to_altitude(&sensor, pressure)`**
Uses the barometric formula to convert pressure to approximate altitude above sea level. Accuracy depends on the sea level reference pressure, which defaults to the standard 1013.25 hPa. See the calibration section below for how to improve this.

---

## Understanding Error Handling

Every function that can fail returns a `bmp180_status_t`. Always check it:

```c
bmp180_status_t s = bmp180_read_temperature(&sensor, &temp);
if (s != BMP180_OK) {
    printf("Error: %s\n", bmp180_status_str(s));
    // handle the error — retry, halt, log, etc.
}
```

If a result hsa to be intentionally ignored, cast it explicitly:
```c
(void)bmp180_set_sea_level(&sensor, 1013.25f);  // set* functions rarely fail
```

---

## Calibrating Altitude

The default sea level pressure (1013.25 hPa) changes with weather, so altitude readings drift by tens of metres from day to day. If exact altitude is known,calibrate using:

```c
float known_altitude_m = 721.0f;  // true altitude in metres

float pressure;
bmp180_read_pressure(&sensor, &pressure);

// Calculate sea level pressure at this instant
float sea_level = bmp180_sea_level_from_altitude(pressure, known_altitude_m);

// Set it
bmp180_set_sea_level(&sensor, sea_level);

printf("Calibrated sea level: %.2f hPa\n", sea_level);
```

After this, `bmp180_pressure_to_altitude()` will give accurate absolute readings for that day's weather.

---

## The Async (Non-Blocking) API

The blocking functions (`bmp180_read_all` etc.) pause the program for ~13-30ms while the sensor converts. For most uses this is fine. But if also driving a display, handling buttons, or running game logic, stalling is not favourable.

The async API lets you start a measurement, do other work, then come back for the result:

```c
// Trigger temperature (returns immediately)
bmp180_trigger_temperature(&sensor);

// Do other work
do_other_work();

// Poll until ready
bool ready = false;
while (!ready) {
    bmp180_data_ready(&sensor, &ready);
}

// Fetch result
float temp;
bmp180_fetch_temperature(&sensor, &temp);

// Trigger pressure
bmp180_trigger_pressure(&sensor);

do_other_work();

ready = false;
while (!ready) {
    bmp180_data_ready(&sensor, &ready);
}

float pressure;
bmp180_fetch_pressure(&sensor, &pressure);
```
Note that temperature **MUST** be called first before pressure.
The `BMP180_ERR_SEQUENCE` error exists specifically for catching the mistake of triggering pressure before fetching temperature.

---

## Oversampling

The BMP180 chip has a built-in hardware averaging feature for pressure. More samples means lower noise but slower readings:

| Setting | Time | Best for |
|---|---|---|
| `BMP180_OSS_LOW` | 4.5 ms | Battery-powered devices, fast polling |
| `BMP180_OSS_STD` | 7.5 ms | General use — good starting point |
| `BMP180_OSS_HIGH` | 13.5 ms | Weather stations, precise altitude |
| `BMP180_OSS_MAX` | 25.5 ms | Highest accuracy, slowest |

Temperature is always measured at the same speed regardless of the OSS setting.

---

## Software Averaging

For smoother output, the mean of multiple readings can be taken in software

```c
float avg_pressure;
// Take 10 samples, 100ms apart, and return the mean
bmp180_read_pressure_avg(&sensor, 10, 100, &avg_pressure);
```

Note: samples must be between 1 and 64. Passing 0 or more than 64 returns `BMP180_ERR_BAD_PARAM`.

---

## Weather Trend Detection (Advanced)

After including `bmp180_advanced.h`, weather trends can be tracked by:

```c
#include "bmp180_advanced.h"

bmp180_trend_tracker_t trend;
bmp180_trend_init(&trend);   // initialises internal critical section
float pressure;
bmp180_read_pressure(&sensor, &pressure);
bmp180_trend_update(&trend, pressure, time_us_32() / 1000);

bmp180_trend_t t = bmp180_trend_get(&trend);
printf("Weather: %s\n", bmp180_trend_str(t));

// get the rate in hPa/hour
float rate = bmp180_trend_rate(&trend);
printf("Rate: %.3f hPa/hr\n", rate);
```

The tracker uses **least-squares linear regression** over a 12-sample rolling window. All functions are thread-safe; you can call `bmp180_trend_update()` from one core and `bmp180_trend_get()` from the other without any external locking.

---

## Common Problems

**"Chip not found" on init**
- Check wiring. VCC to 3V3, GND to GND, SDA and SCL to the correct pins.
- Make sure SDA and SCL aren't swapped.
- Run the I2C scanner from the main README to see if the chip responds at all.
- Check these `i2c_init()`, `gpio_set_function()`, and `gpio_pull_up()` were called before before `bmp180_init()`.

**Temperature reads 2-3°C too high**
- The chip self-heats slightly. Leave it in open air away from other components for more accuracy.

**Altitude varies by tens of metres between days**
- This happens due to sea level pressure change with weather. Calibrate with `bmp180_sea_level_from_altitude()` using known altitude for that day.

**Compilation error: `powf` undefined**
- Add `m` to `target_link_libraries` in CMakeLists.

**`BMP180_ERR_SEQUENCE` from `bmp180_trigger_pressure`**
- You must call and fetch temperature before triggering pressure. See the async API section above.

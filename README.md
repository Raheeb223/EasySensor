# EasySensor

[![Arduino Library Manager](https://img.shields.io/badge/Arduino-Library%20Manager-blue)](https://www.arduino.cc/reference/en/libraries/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Version](https://img.shields.io/badge/version-1.0.0-orange)](library.properties)

A **universal sensor abstraction library** for Arduino with built-in signal filtering, auto-calibration, threshold events, and multi-sensor fusion.

---

## Features

| Feature | Detail |
|---|---|
| **Unified API** | One `read()` call works for any analog or digital sensor |
| **Moving Average** | Configurable window (2–32 samples) |
| **Kalman Filter** | 1-D discrete Kalman with tunable Q and R |
| **EMA Filter** | Exponential moving average with configurable α |
| **Auto-Calibration** | Average N samples and store as zero baseline |
| **Threshold Events** | Hysteresis-based callbacks, no chattering |
| **Sensor Fusion** | Weighted average of up to 8 sensors |
| **Non-Blocking** | Configurable sample interval; never blocks `loop()` |
| **Percentage Mapping** | Raw ADC → 0–100 % with configurable range |
| **Zero Dependencies** | No external libraries required |

---

## Installation

### Via Arduino Library Manager (recommended)
1. Open Arduino IDE → **Sketch → Include Library → Manage Libraries…**
2. Search for **EasySensor**
3. Click **Install**

### Manual install
1. Download this repository as a ZIP file
2. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library…**
3. Select the downloaded ZIP

---

## Quick Start

```cpp
#include <EasySensor.h>

// Analog sensor on A0, moving-average filter, 8-sample window
EasySensor ldr(A0, SENSOR_LIGHT, FILTER_AVERAGE);

void setup() {
    Serial.begin(9600);
    ldr.begin(8);             // 8-sample window
    ldr.setSampleInterval(100); // read every 100 ms
    ldr.setRange(0, 1023);    // map to 0–100 %
}

void loop() {
    if (ldr.update()) {
        Serial.print("Light: ");
        Serial.print(ldr.readPercent());
        Serial.println(" %");
    }
}
```

---

## API Reference

### EasySensor

```cpp
EasySensor(pin, type, filterMode);  // constructor
void  begin(windowSize = 8);        // call in setup()
bool  update();                     // call every loop(); returns true on new sample
float read();                       // filtered + offset-corrected value
int   rawRead();                    // raw ADC value
float readPercent();                // 0–100 % mapped reading

void  setSampleInterval(uint32_t ms);
void  setFilter(filterMode, windowSize = 8);
void  setKalmanParams(float Q, float R);
void  setEmaAlpha(float alpha);
void  setRange(int rawMin, int rawMax);

void  calibrate(uint8_t samples = 20);
void  setOffset(float offset);
void  resetCalibration();

void  onThreshold(float threshold, float hysteresis, SensorEventCallback cb);
void  clearThreshold();
bool  isAboveThreshold();
```

### SensorFusion

```cpp
SensorFusion fusion;
fusion.addSensor(&sensor, weight);  // add up to 8 sensors
fusion.update();                    // updates all member sensors
float fused = fusion.read();        // weighted average reading
```

### Filter modes

| Constant | Description |
|---|---|
| `FILTER_NONE` | Raw unfiltered ADC value |
| `FILTER_AVERAGE` | Simple moving average |
| `FILTER_KALMAN` | 1-D Kalman filter (best for noisy sensors) |
| `FILTER_EMA` | Exponential moving average (low memory, fast) |

### Sensor types

`SENSOR_GENERIC`, `SENSOR_LIGHT`, `SENSOR_TEMP`, `SENSOR_MOISTURE`,
`SENSOR_GAS`, `SENSOR_DISTANCE`, `SENSOR_SOUND`, `SENSOR_DIGITAL`

---

## Examples

| Example | Description |
|---|---|
| `BasicReading` | Potentiometer → Serial Monitor with percent display |
| `AdvancedFiltering` | All four filters on one pin → Serial Plotter comparison |
| `MultiSensor` | Plant monitor: moisture fusion + threshold alert LED |

---

## Compatibility

Tested on: **Uno, Nano, Mega 2560, Leonardo, Due, MKR Zero, ESP8266, ESP32**

Compiles cleanly on all `architectures=*` boards with `avr-gcc`, `arm-none-eabi-gcc`, and `xtensa-gcc`.

---

## License

MIT License — see [LICENSE](LICENSE) for details.

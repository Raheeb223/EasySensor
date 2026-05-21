/**
 * @file EasySensor.h
 * @brief Universal sensor abstraction with filtering, calibration, and fusion.
 *
 * EasySensor — v1.0.0
 * A production-quality Arduino library that provides a unified interface for
 * any analog or digital sensor, with built-in signal processing and event detection.
 *
 * Features:
 *  - Unified read() API for analog and digital sensors
 *  - Moving Average filter (configurable window 2–32 samples)
 *  - 1-D Kalman filter for noisy signals
 *  - Exponential smoothing (EMA)
 *  - Auto-calibration (baseline offset)
 *  - Threshold-based event callbacks
 *  - Multi-sensor weighted fusion
 *  - Non-blocking sampling with configurable interval
 *  - Percentage mapping (raw → 0–100 %)
 *  - Zero external dependencies; works on all Arduino-compatible boards
 *
 * License: MIT
 * Repository: https://github.com/YOUR_USERNAME/EasySensor
 */

#ifndef EASY_SENSOR_H
#define EASY_SENSOR_H

#include <Arduino.h>

// ──────────────────────────────────────────────────────────────
// Constants
// ──────────────────────────────────────────────────────────────
#define EASY_SENSOR_VERSION       "1.0.0"
#define EASY_SENSOR_MAX_WINDOW    32    ///< Maximum moving-average window size
#define EASY_SENSOR_MAX_SENSORS   8     ///< Maximum sensors in a Fusion group

// ──────────────────────────────────────────────────────────────
// Filter mode selector
// ──────────────────────────────────────────────────────────────
enum EasySensorFilter : uint8_t {
    FILTER_NONE    = 0,  ///< Raw ADC / digital value
    FILTER_AVERAGE = 1,  ///< Simple moving average
    FILTER_KALMAN  = 2,  ///< 1-D discrete Kalman filter
    FILTER_EMA     = 3   ///< Exponential moving average (IIR)
};

// ──────────────────────────────────────────────────────────────
// Sensor type hint (used for meaningful percentage mapping)
// ──────────────────────────────────────────────────────────────
enum EasySensorType : uint8_t {
    SENSOR_GENERIC  = 0,
    SENSOR_LIGHT    = 1,
    SENSOR_TEMP     = 2,
    SENSOR_MOISTURE = 3,
    SENSOR_GAS      = 4,
    SENSOR_DISTANCE = 5,
    SENSOR_SOUND    = 6,
    SENSOR_DIGITAL  = 7
};

// ──────────────────────────────────────────────────────────────
// Threshold event data passed to the callback
// ──────────────────────────────────────────────────────────────
struct SensorEvent {
    uint8_t  pin;       ///< Pin that triggered
    float    value;     ///< Filtered reading that triggered the event
    bool     rising;    ///< true = crossed threshold upward
};

typedef void (*SensorEventCallback)(const SensorEvent &event);

// ──────────────────────────────────────────────────────────────
// EasySensor — single-sensor class
// ──────────────────────────────────────────────────────────────
class EasySensor {
public:
    /**
     * @brief Construct a new EasySensor.
     * @param pin          Arduino pin number (analog or digital).
     * @param type         Semantic type hint (default SENSOR_GENERIC).
     * @param filterMode   Signal processing mode (default FILTER_AVERAGE).
     */
    EasySensor(uint8_t pin,
               EasySensorType   type       = SENSOR_GENERIC,
               EasySensorFilter filterMode = FILTER_AVERAGE);

    // ── Lifecycle ────────────────────────────────────────────
    /**
     * @brief Initialise the sensor; call once in setup().
     * @param windowSize   Moving-average window (2–32, used when FILTER_AVERAGE).
     */
    void begin(uint8_t windowSize = 8);

    /**
     * @brief Non-blocking update; call every loop() iteration.
     *        Reads the sensor only when the sample interval has elapsed.
     * @return true if a new sample was taken this call.
     */
    bool update();

    // ── Configuration ────────────────────────────────────────
    /** Set the minimum time between hardware reads (milliseconds). */
    void setSampleInterval(uint32_t ms);

    /** Select or change the filter at runtime. */
    void setFilter(EasySensorFilter filterMode, uint8_t windowSize = 8);

    /**
     * @brief Set Kalman filter tuning parameters.
     * @param processNoise  Q — how much the true value can change per step (default 0.01).
     * @param measureNoise  R — sensor noise variance (default 0.1).
     */
    void setKalmanParams(float processNoise, float measureNoise);

    /**
     * @brief Set EMA smoothing factor α ∈ (0, 1].
     *        α = 1 → no smoothing; α → 0 → very heavy smoothing.
     */
    void setEmaAlpha(float alpha);

    /**
     * @brief Map the raw ADC range to a 0–100 % scale.
     * @param rawMin  Raw value that maps to 0 %.
     * @param rawMax  Raw value that maps to 100 %.
     */
    void setRange(int rawMin, int rawMax);

    // ── Calibration ──────────────────────────────────────────
    /**
     * @brief Auto-calibrate: average N samples and store as the zero baseline.
     * @param samples  Number of samples to average (default 20).
     */
    void calibrate(uint8_t samples = 20);

    /** Manually set the calibration offset. */
    void setOffset(float offset);

    /** Reset calibration offset to zero. */
    void resetCalibration();

    // ── Thresholds & Events ──────────────────────────────────
    /**
     * @brief Register a threshold event.
     *        The callback fires once each time the filtered reading crosses
     *        the threshold (hysteresis prevents chattering).
     * @param threshold   Value to watch.
     * @param hysteresis  Dead-band around threshold (default 2.0).
     * @param cb          Function to call on crossing.
     */
    void onThreshold(float threshold, float hysteresis,
                     SensorEventCallback cb);

    /** Remove any registered threshold callback. */
    void clearThreshold();

    // ── Reading ──────────────────────────────────────────────
    /** Return the latest filtered (and offset-corrected) value. */
    float read() const;

    /** Return the latest raw (unfiltered) ADC value. */
    int   rawRead() const;

    /** Return the reading mapped to 0–100 %. */
    float readPercent() const;

    /** Return true if the sensor is currently above the threshold. */
    bool  isAboveThreshold() const;

    /** Return the pin this sensor is attached to. */
    uint8_t pin() const;

    /** Return the library version string. */
    static const char* version();

private:
    // Hardware
    uint8_t          _pin;
    EasySensorType   _type;
    EasySensorFilter _filter;

    // Sampling
    uint32_t _sampleInterval;
    uint32_t _lastSampleTime;

    // Raw & filtered values
    int   _rawValue;
    float _filtered;
    float _offset;

    // Range mapping
    int   _rawMin;
    int   _rawMax;

    // Moving average buffer
    float   _window[EASY_SENSOR_MAX_WINDOW];
    uint8_t _windowSize;
    uint8_t _windowIndex;
    bool    _windowFull;

    // Kalman state
    float _kEstimate;
    float _kError;
    float _kQ;
    float _kR;

    // EMA state
    float _emaAlpha;
    float _emaValue;
    bool  _emaInitialised;

    // Threshold / event
    float               _threshold;
    float               _hysteresis;
    bool                _aboveThreshold;
    bool                _thresholdActive;
    SensorEventCallback _callback;

    // Internal helpers
    int   _hardwareRead() const;
    float _applyFilter(float raw);
    float _applyMovingAverage(float raw);
    float _applyKalman(float raw);
    float _applyEma(float raw);
    void  _checkThreshold(float value);
};

// ──────────────────────────────────────────────────────────────
// SensorFusion — weighted average of up to 8 EasySensor objects
// ──────────────────────────────────────────────────────────────
class SensorFusion {
public:
    SensorFusion();

    /**
     * @brief Add a sensor to the fusion group.
     * @param sensor  Pointer to an EasySensor instance.
     * @param weight  Relative weight (default 1.0); values are normalised.
     * @return true if added; false if the group is full.
     */
    bool addSensor(EasySensor *sensor, float weight = 1.0f);

    /**
     * @brief Non-blocking update — propagates to all member sensors.
     */
    void update();

    /**
     * @brief Return the weighted average of all member sensor readings.
     */
    float read() const;

    /** Number of sensors currently in the group. */
    uint8_t count() const;

private:
    EasySensor *_sensors[EASY_SENSOR_MAX_SENSORS];
    float       _weights[EASY_SENSOR_MAX_SENSORS];
    uint8_t     _count;
};

#endif // EASY_SENSOR_H

/**
 * @file EasySensor.cpp
 * @brief Implementation of EasySensor and SensorFusion.
 *
 * EasySensor — v1.0.0
 * License: MIT
 */

#include "EasySensor.h"

// ══════════════════════════════════════════════════════════════
// EasySensor implementation
// ══════════════════════════════════════════════════════════════

EasySensor::EasySensor(uint8_t pin, EasySensorType type, EasySensorFilter filterMode)
    : _pin(pin)
    , _type(type)
    , _filter(filterMode)
    , _sampleInterval(50)
    , _lastSampleTime(0)
    , _rawValue(0)
    , _filtered(0.0f)
    , _offset(0.0f)
    , _rawMin(0)
    , _rawMax(1023)
    , _windowSize(8)
    , _windowIndex(0)
    , _windowFull(false)
    , _kEstimate(0.0f)
    , _kError(1.0f)
    , _kQ(0.01f)
    , _kR(0.1f)
    , _emaAlpha(0.1f)
    , _emaValue(0.0f)
    , _emaInitialised(false)
    , _threshold(0.0f)
    , _hysteresis(2.0f)
    , _aboveThreshold(false)
    , _thresholdActive(false)
    , _callback(nullptr)
{
    memset(_window, 0, sizeof(_window));
}

// ── Lifecycle ────────────────────────────────────────────────

void EasySensor::begin(uint8_t windowSize)
{
    // Clamp window size
    if (windowSize < 2)  windowSize = 2;
    if (windowSize > EASY_SENSOR_MAX_WINDOW) windowSize = EASY_SENSOR_MAX_WINDOW;
    _windowSize = windowSize;

    // Configure pin mode
    if (_type == SENSOR_DIGITAL) {
        pinMode(_pin, INPUT_PULLUP);
    }
    // Analog pins need no explicit pinMode on most Arduino boards,
    // but calling it doesn't hurt.

    // Seed filter state with one real reading
    int seed = _hardwareRead();
    _rawValue  = seed;
    _filtered  = (float)seed;
    _kEstimate = (float)seed;
    _emaValue  = (float)seed;
    _emaInitialised = true;

    // Seed the moving average window
    for (uint8_t i = 0; i < _windowSize; i++) {
        _window[i] = (float)seed;
    }
    _windowFull = true;
}

bool EasySensor::update()
{
    uint32_t now = millis();
    if ((now - _lastSampleTime) < _sampleInterval) {
        return false;
    }
    _lastSampleTime = now;

    _rawValue = _hardwareRead();
    _filtered = _applyFilter((float)_rawValue) - _offset;
    _checkThreshold(_filtered);
    return true;
}

// ── Configuration ────────────────────────────────────────────

void EasySensor::setSampleInterval(uint32_t ms)
{
    _sampleInterval = ms;
}

void EasySensor::setFilter(EasySensorFilter filterMode, uint8_t windowSize)
{
    _filter = filterMode;
    if (windowSize < 2)  windowSize = 2;
    if (windowSize > EASY_SENSOR_MAX_WINDOW) windowSize = EASY_SENSOR_MAX_WINDOW;
    _windowSize = windowSize;
    // Reset window so the new size takes effect cleanly
    _windowIndex = 0;
    _windowFull  = false;
    _emaInitialised = false;
}

void EasySensor::setKalmanParams(float processNoise, float measureNoise)
{
    _kQ = processNoise;
    _kR = measureNoise;
}

void EasySensor::setEmaAlpha(float alpha)
{
    if (alpha <= 0.0f) alpha = 0.01f;
    if (alpha >  1.0f) alpha = 1.0f;
    _emaAlpha = alpha;
}

void EasySensor::setRange(int rawMin, int rawMax)
{
    _rawMin = rawMin;
    _rawMax = rawMax;
}

// ── Calibration ──────────────────────────────────────────────

void EasySensor::calibrate(uint8_t samples)
{
    if (samples == 0) samples = 1;
    long sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += _hardwareRead();
        delay(10);
    }
    _offset = (float)(sum / samples);
}

void EasySensor::setOffset(float offset)
{
    _offset = offset;
}

void EasySensor::resetCalibration()
{
    _offset = 0.0f;
}

// ── Thresholds & Events ──────────────────────────────────────

void EasySensor::onThreshold(float threshold, float hysteresis, SensorEventCallback cb)
{
    _threshold      = threshold;
    _hysteresis     = (hysteresis < 0.0f) ? 0.0f : hysteresis;
    _callback       = cb;
    _thresholdActive = true;
    _aboveThreshold  = (_filtered >= threshold);
}

void EasySensor::clearThreshold()
{
    _thresholdActive = false;
    _callback        = nullptr;
}

// ── Reading ──────────────────────────────────────────────────

float EasySensor::read() const
{
    return _filtered;
}

int EasySensor::rawRead() const
{
    return _rawValue;
}

float EasySensor::readPercent() const
{
    int span = _rawMax - _rawMin;
    if (span == 0) return 0.0f;
    float pct = ((_filtered + _offset) - (float)_rawMin) / (float)span * 100.0f;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

bool EasySensor::isAboveThreshold() const
{
    return _aboveThreshold;
}

uint8_t EasySensor::pin() const
{
    return _pin;
}

const char* EasySensor::version()
{
    return EASY_SENSOR_VERSION;
}

// ── Private helpers ──────────────────────────────────────────

int EasySensor::_hardwareRead() const
{
    if (_type == SENSOR_DIGITAL) {
        return digitalRead(_pin);
    }
    return analogRead(_pin);
}

float EasySensor::_applyFilter(float raw)
{
    switch (_filter) {
        case FILTER_AVERAGE: return _applyMovingAverage(raw);
        case FILTER_KALMAN:  return _applyKalman(raw);
        case FILTER_EMA:     return _applyEma(raw);
        default:             return raw;
    }
}

float EasySensor::_applyMovingAverage(float raw)
{
    _window[_windowIndex] = raw;
    _windowIndex = (_windowIndex + 1) % _windowSize;
    if (_windowIndex == 0) _windowFull = true;

    uint8_t count = _windowFull ? _windowSize : _windowIndex;
    if (count == 0) return raw;

    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum += _window[i];
    }
    return sum / (float)count;
}

float EasySensor::_applyKalman(float raw)
{
    // Prediction
    _kError += _kQ;

    // Update (Kalman gain)
    float gain = _kError / (_kError + _kR);
    _kEstimate += gain * (raw - _kEstimate);
    _kError    *= (1.0f - gain);

    return _kEstimate;
}

float EasySensor::_applyEma(float raw)
{
    if (!_emaInitialised) {
        _emaValue       = raw;
        _emaInitialised = true;
        return _emaValue;
    }
    _emaValue = _emaAlpha * raw + (1.0f - _emaAlpha) * _emaValue;
    return _emaValue;
}

void EasySensor::_checkThreshold(float value)
{
    if (!_thresholdActive || _callback == nullptr) return;

    bool nowAbove = (value >= _threshold);

    // Rising edge — was below, now above (with hysteresis)
    if (!_aboveThreshold && value >= _threshold) {
        _aboveThreshold = true;
        SensorEvent ev = { _pin, value, true };
        _callback(ev);
    }
    // Falling edge — was above, now below threshold minus hysteresis
    else if (_aboveThreshold && value < (_threshold - _hysteresis)) {
        _aboveThreshold = false;
        SensorEvent ev = { _pin, value, false };
        _callback(ev);
    }

    (void)nowAbove; // suppress unused warning
}

// ══════════════════════════════════════════════════════════════
// SensorFusion implementation
// ══════════════════════════════════════════════════════════════

SensorFusion::SensorFusion()
    : _count(0)
{
    memset(_sensors, 0, sizeof(_sensors));
    memset(_weights, 0, sizeof(_weights));
}

bool SensorFusion::addSensor(EasySensor *sensor, float weight)
{
    if (_count >= EASY_SENSOR_MAX_SENSORS) return false;
    if (sensor == nullptr)                 return false;
    if (weight <= 0.0f)                    weight = 1.0f;

    _sensors[_count] = sensor;
    _weights[_count] = weight;
    _count++;
    return true;
}

void SensorFusion::update()
{
    for (uint8_t i = 0; i < _count; i++) {
        _sensors[i]->update();
    }
}

float SensorFusion::read() const
{
    if (_count == 0) return 0.0f;

    float weightSum = 0.0f;
    float valueSum  = 0.0f;

    for (uint8_t i = 0; i < _count; i++) {
        weightSum += _weights[i];
        valueSum  += _sensors[i]->read() * _weights[i];
    }

    if (weightSum == 0.0f) return 0.0f;
    return valueSum / weightSum;
}

uint8_t SensorFusion::count() const
{
    return _count;
}

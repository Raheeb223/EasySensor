/**
 * AdvancedFiltering.ino
 * ─────────────────────────────────────────────────────────────
 * EasySensor — Example 2: Advanced Filtering
 *
 * Runs three EasySensor instances on the same pin with three
 * different filters so you can compare them side-by-side in
 * the Serial Plotter (Tools → Serial Plotter in the IDE).
 *
 * Hardware:
 *   • Any noisy analog sensor (LDR, microphone, etc.) → A0
 *
 * Wiring (LDR voltage divider):
 *   5 V → 10 kΩ → A0 → LDR → GND
 */

#include <EasySensor.h>

// Three instances on the same pin, different filters
EasySensor rawSensor  (A0, SENSOR_LIGHT, FILTER_NONE);
EasySensor avgSensor  (A0, SENSOR_LIGHT, FILTER_AVERAGE);
EasySensor kalmanSensor(A0, SENSOR_LIGHT, FILTER_KALMAN);
EasySensor emaSensor  (A0, SENSOR_LIGHT, FILTER_EMA);

void setup() {
    Serial.begin(115200);

    rawSensor.begin(1);           // window size irrelevant for FILTER_NONE
    avgSensor.begin(16);          // 16-sample average (smoother, more lag)
    kalmanSensor.begin(1);
    emaSensor.begin(1);

    // Tune Kalman for a moderately noisy light sensor
    kalmanSensor.setKalmanParams(0.005f, 0.5f);

    // EMA: α = 0.08  →  heavy smoothing, low lag compared to 16-point average
    emaSensor.setEmaAlpha(0.08f);

    // All sensors sample every 20 ms (≈ 50 Hz)
    rawSensor.setSampleInterval(20);
    avgSensor.setSampleInterval(20);
    kalmanSensor.setSampleInterval(20);
    emaSensor.setSampleInterval(20);

    // CSV header for Serial Plotter
    Serial.println(F("Raw,Moving_Avg_16,Kalman,EMA_0.08"));
}

void loop() {
    // Update all — only one hardware read per sensor per interval
    bool newData = rawSensor.update();
    avgSensor.update();
    kalmanSensor.update();
    emaSensor.update();

    if (newData) {
        // CSV format → compatible with Arduino Serial Plotter
        Serial.print(rawSensor.read(),    1);  Serial.print(',');
        Serial.print(avgSensor.read(),    1);  Serial.print(',');
        Serial.print(kalmanSensor.read(), 1);  Serial.print(',');
        Serial.println(emaSensor.read(),  1);
    }
}

/**
 * MultiSensor.ino
 * ─────────────────────────────────────────────────────────────
 * EasySensor — Example 3: Multi-Sensor Fusion + Threshold Events
 *
 * Fuses two soil-moisture sensors with a light sensor using
 * weighted averaging, and triggers an LED + Serial alert when
 * the fused reading crosses a "needs watering" threshold.
 *
 * Hardware:
 *   • Soil moisture sensor 1  → A0
 *   • Soil moisture sensor 2  → A1
 *   • LDR light sensor        → A2
 *   • LED (with 220 Ω)        → pin 13 (built-in LED)
 *
 * Note: works on 3.3 V boards too; sensors just scale differently.
 */

#include <EasySensor.h>

// ── Sensors ──────────────────────────────────────────────────
// Moisture sensors: Kalman filter to reject electrical noise
EasySensor moisture1(A0, SENSOR_MOISTURE, FILTER_KALMAN);
EasySensor moisture2(A1, SENSOR_MOISTURE, FILTER_KALMAN);

// Light sensor: EMA is ideal for slow ambient light changes
EasySensor light(A2, SENSOR_LIGHT, FILTER_EMA);

// ── Fusion ───────────────────────────────────────────────────
SensorFusion plant;

// ── Alert LED ────────────────────────────────────────────────
const uint8_t ALERT_LED = 13;

// ── Threshold callback ────────────────────────────────────────
void onMoistureEvent(const SensorEvent &ev) {
    if (ev.rising) {
        // Value went above threshold — soil is wet enough now
        digitalWrite(ALERT_LED, LOW);
        Serial.println(F("✓  Moisture OK — no watering needed."));
    } else {
        // Value fell below threshold — soil is dry
        digitalWrite(ALERT_LED, HIGH);
        Serial.println(F("⚠  ALERT: Soil is dry — please water the plant!"));
    }
}

void setup() {
    Serial.begin(9600);
    pinMode(ALERT_LED, OUTPUT);

    // Initialise individual sensors
    moisture1.begin(1);
    moisture2.begin(1);
    light.begin(1);

    moisture1.setKalmanParams(0.01f, 0.5f);
    moisture2.setKalmanParams(0.01f, 0.5f);
    light.setEmaAlpha(0.05f);

    // Calibrate moisture sensors (hold in air for a dry baseline)
    Serial.println(F("Calibrating... keep sensors in air."));
    moisture1.calibrate(30);
    moisture2.calibrate(30);
    Serial.println(F("Calibration done."));

    // Map raw range to 0–100 % (adjust these for your specific sensors)
    moisture1.setRange(300, 700);
    moisture2.setRange(300, 700);

    // Sample every 500 ms — moisture doesn't change fast
    moisture1.setSampleInterval(500);
    moisture2.setSampleInterval(500);
    light.setSampleInterval(500);

    // ── Build the fusion group ────────────────────────────────
    // Moisture sensors are weighted 2× more than light
    plant.addSensor(&moisture1, 2.0f);
    plant.addSensor(&moisture2, 2.0f);
    plant.addSensor(&light,     1.0f);

    // ── Threshold event ───────────────────────────────────────
    // Fire callback when fused reading crosses 40 % (±5 % hysteresis)
    // We attach it to moisture1 as the "trigger" sensor; adjust as needed.
    moisture1.onThreshold(40.0f, 5.0f, onMoistureEvent);

    Serial.println(F("EasySensor — MultiSensor fusion example running."));
    Serial.println(F("Fused %, M1 %, M2 %, Light %"));
}

void loop() {
    // update() calls update() on all member sensors
    plant.update();

    // Print fused reading and individual percentages
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        Serial.print(plant.read(), 1);           Serial.print(F(",  "));
        Serial.print(moisture1.readPercent(), 1); Serial.print(F(",  "));
        Serial.print(moisture2.readPercent(), 1); Serial.print(F(",  "));
        Serial.println(light.readPercent(), 1);
    }
}

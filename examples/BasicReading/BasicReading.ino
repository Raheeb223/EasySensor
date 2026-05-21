/**
 * BasicReading.ino
 * ─────────────────────────────────────────────────────────────
 * EasySensor — Example 1: Basic Reading
 *
 * Reads a potentiometer (or any analog sensor) on pin A0 with
 * a moving-average filter and prints the result to Serial.
 *
 * Hardware:
 *   • Potentiometer (or analog sensor) → A0
 *   • Arduino Uno / Nano / Mega / ESP32 / ESP8266 / any board
 *
 * Wiring (potentiometer):
 *   Left leg  → GND
 *   Wiper     → A0
 *   Right leg → 5 V (or 3.3 V on 3.3 V boards)
 */

#include <EasySensor.h>

// Create a sensor on pin A0, generic type, moving-average filter
EasySensor pot(A0, SENSOR_GENERIC, FILTER_AVERAGE);

void setup() {
    Serial.begin(9600);
    while (!Serial) { ; }   // wait for Serial on Leonardo / Pro Micro

    // Initialise with an 8-sample moving-average window
    pot.begin(8);

    // Only sample every 100 ms — no need to hammer the ADC
    pot.setSampleInterval(100);

    // Map 0–1023 raw ADC counts to 0–100 %
    pot.setRange(0, 1023);

    Serial.println(F("EasySensor — BasicReading example"));
    Serial.println(F("Rotate the potentiometer and watch the values change."));
    Serial.println(F("─────────────────────────────────────────────────────"));
}

void loop() {
    // update() is non-blocking; it reads only when the interval elapses
    if (pot.update()) {
        Serial.print(F("Filtered: "));
        Serial.print(pot.read(), 1);       // one decimal place
        Serial.print(F("  Raw: "));
        Serial.print(pot.rawRead());
        Serial.print(F("  Percent: "));
        Serial.print(pot.readPercent(), 1);
        Serial.println(F(" %"));
    }
}

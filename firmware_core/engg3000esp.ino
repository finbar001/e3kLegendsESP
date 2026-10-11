#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "calibration.h"
#include "tracking.h"
#include "comms.h"

// Subsystem Instances
UltrasonicModule    sensors;
CalibrationManager  calibMgr;
PositionTracker     tracker;
CommsManager        comms;

bool isPrimaryModule = true;
uint32_t lastTrackingMs = 0;
uint32_t lastDiagMs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    // Setup Status LED & Buzzer pins
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    // Read Hardware Role Jumper (Pin 4 with internal pull-up)
    // HIGH (unconnected) = PRIMARY (Left Hub)
    // LOW (jumpered to GND) = SECONDARY (Right Node)
    pinMode(PIN_ROLE_SELECT, INPUT_PULLUP);
    delay(50);
    isPrimaryModule = (digitalRead(PIN_ROLE_SELECT) == HIGH);

    Serial.println("\n========================================================");
    Serial.println("  ENGG3000 Ultrasonic Player Tracking System");
    Serial.println("  Macquarie University - S2 2026");
    Serial.printf("  BOOT MODE: %s\n", isPrimaryModule ? "PRIMARY (LEFT HUB)" : "SECONDARY (RIGHT SENSOR NODE)");
    Serial.println("========================================================");

    // Initialize Ultrasonic Sensors
    sensors.begin(
        PIN_SENSOR_OUTER_TRIG, PIN_SENSOR_OUTER_ECHO,
        PIN_SENSOR_INNER_TRIG, PIN_SENSOR_INNER_ECHO
    );

    if (isPrimaryModule) {
        // 1. Primary Hub Initialization
        calibMgr.loadFromFlash();
        const SystemCalibration& cal = calibMgr.getCalibration();
        sensors.setEmaAlpha(cal.emaAlpha);
        sensors.setSpikeGateCm(cal.spikeGateCm);
        tracker.begin(cal);
        comms.beginPrimary(&calibMgr, &tracker, &sensors);
        digitalWrite(PIN_STATUS_LED, HIGH);
    } else {
        // 2. Secondary Node Initialization
        comms.beginSecondary();
    }
}

void loop() {
    if (isPrimaryModule) {
        // -------------------------------------------------------------
        // PRIMARY (LEFT HUB) WORKFLOW
        // -------------------------------------------------------------
        
        // 1. Maintain HTTP server and WebSocket connections
        comms.updatePrimary();

        // 2. Periodic Sensor Measurement & Position Tracking (~20Hz)
        uint32_t now = millis();
        if (now - lastTrackingMs >= TRACKING_UPDATE_INTERVAL_MS) {
            float dt = (float)(now - lastTrackingMs) * 0.001f;
            lastTrackingMs = now;

            // Measure local Left sensors (LO, LI)
            sensors.update();
            const SensorReading& rLO = sensors.getOuterReading();
            const SensorReading& rLI = sensors.getInnerReading();

            // Retrieve latest received Right sensors (RI, RO) from Secondary via ESP-NOW
            float distRI = comms.getSecondaryInnerDist();
            float distRO = comms.getSecondaryOuterDist();
            bool  valRI  = comms.isSecondaryInnerValid();
            bool  valRO  = comms.isSecondaryOuterValid();
            bool  secOnline = comms.isSecondaryOnline();

            // Passive environment baseline monitoring during boot
            if (now < BOOT_CALIB_DURATION_MS) {
                calibMgr.recordBootSample(rLO.filteredDistanceCm, rLI.filteredDistanceCm, distRI, distRO);
            } else if (!calibMgr.isBootCalibrated()) {
                calibMgr.finalizeBootCalibration();
            }

            // Feed samples if active Quick-Align collection is in progress
            if (calibMgr.isQuickAligning()) {
                calibMgr.feedQuickAlignSample(rLO.filteredDistanceCm, rLI.filteredDistanceCm, distRI, distRO);
                if (!calibMgr.isQuickAligning()) {
                    // Just finalized
                    tracker.updateCalibration(calibMgr.getCalibration());
                    tracker.resetAlignmentHealth();
                }
            }

            // Fuse all 4 sensors with Multilateration + 2D Adaptive Kalman Filter
            TrackedPosition pos = tracker.process(
                rLO.filteredDistanceCm, rLO.isValid,
                rLI.filteredDistanceCm, rLI.isValid,
                distRI, valRI,
                distRO, valRO,
                dt
            );

            // Handle Dead Zone Alarm (Audible Buzzer + Status LED toggle)
            if (pos.inDeadZone && pos.isDetected) {
                digitalWrite(PIN_BUZZER, HIGH);
                digitalWrite(PIN_STATUS_LED, (millis() % 200 < 100) ? HIGH : LOW);
            } else {
                digitalWrite(PIN_BUZZER, LOW);
                digitalWrite(PIN_STATUS_LED, HIGH);
            }

            // Stream live telemetry over WebSocket to Game & Diagnostic Web UI
            comms.broadcastPosition(
                pos,
                rLO.filteredDistanceCm, rLI.filteredDistanceCm,
                distRI, distRO,
                rLO.jitterCm, rLI.jitterCm,
                secOnline
            );
        }

        // 3. Serial Diagnostic Printout (Every 1 Second)
        if (now - lastDiagMs >= 1000) {
            lastDiagMs = now;
            const SensorReading& rLO = sensors.getOuterReading();
            const SensorReading& rLI = sensors.getInnerReading();
            Serial.printf(
                "[HUB] LO: %5.1f (±%.1f) | LI: %5.1f (±%.1f) | RI: %5.1f (±%.1f) | RO: %5.1f (±%.1f) | SecNode: %s\n",
                rLO.filteredDistanceCm, rLO.jitterCm,
                rLI.filteredDistanceCm, rLI.jitterCm,
                comms.getSecondaryInnerDist(), comms.getSecondaryInnerJitter(),
                comms.getSecondaryOuterDist(), comms.getSecondaryOuterJitter(),
                comms.isSecondaryOnline() ? "ONLINE" : "OFFLINE"
            );
        }

    } else {
        // -------------------------------------------------------------
        // SECONDARY (RIGHT SENSOR NODE) WORKFLOW
        // -------------------------------------------------------------
        
        // Measure local Right sensors (RI = Inner, RO = Outer)
        sensors.update();
        const SensorReading& rRO = sensors.getOuterReading();
        const SensorReading& rRI = sensors.getInnerReading();

        // Ship readings and jitter over ESP-NOW to Primary Hub
        comms.sendSecondaryPacket(
            rRI.filteredDistanceCm, rRI.isValid,
            rRO.filteredDistanceCm, rRO.isValid,
            rRI.jitterCm, rRO.jitterCm
        );

        // Flash LED briefly per measurement
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(10);
        digitalWrite(PIN_STATUS_LED, LOW);

        // Interval before next scan
        delay(TRACKING_UPDATE_INTERVAL_MS - 10);
    }
}

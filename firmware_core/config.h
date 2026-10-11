#pragma once
#include <Arduino.h>

// =============================================================================
// ROLE CONFIGURATION (JUMPER SELECT)
// =============================================================================
// Pin with internal pullup used to detect module role at boot:
// - Unconnected (HIGH / Pulled-up) -> PRIMARY (Left Module, Hub, WiFi AP + WebSockets)
// - Jumpered to GND (LOW)           -> SECONDARY (Right Module, ESP-NOW Sender)
#define PIN_ROLE_SELECT       18

// Status / Activity LED pin
#define PIN_STATUS_LED        2

// Optional Buzzer pin for dead-zone audible alarm (software active)
#define PIN_BUZZER            25

// =============================================================================
// ULTRASONIC SENSOR PINS (RCWL-1601 / HC-SR04 compatible)
// =============================================================================
// Sensor 1: Outer Sensor (Module Left = LO / Module Right = RO)
#define PIN_SENSOR_OUTER_TRIG 33
#define PIN_SENSOR_OUTER_ECHO 32

// Sensor 2: Inner Sensor (Module Left = LI / Module Right = RI)
#define PIN_SENSOR_INNER_TRIG 27
#define PIN_SENSOR_INNER_ECHO 26

// =============================================================================
// PHYSICAL GEOMETRY DEFAULTS (Centimetres & Degrees)
// =============================================================================
// Playing Area: 1.5m wide x 1.4m deep in front of 0.6m dead zone
#define DEFAULT_AREA_WIDTH_CM     150.0f
#define DEFAULT_AREA_DEPTH_CM     140.0f
#define DEFAULT_DEAD_ZONE_CM       60.0f

// Inter-sensor center distance on the same physical module box (cm)
#define DEFAULT_MODULE_SENSOR_SEP_CM  7.0f

// Default Sensor Boresight Angles (measured from +X baseline axis in degrees)
// Module Left (at x=0, y=0):
#define DEFAULT_ANGLE_LO_DEG      75.0f   // Left Outer: Points mostly forward (75°)
#define DEFAULT_ANGLE_LI_DEG      45.0f   // Left Inner: Angled diagonally inward (45°)

// Module Right (at x=150, y=0):
#define DEFAULT_ANGLE_RI_DEG     135.0f   // Right Inner: Angled diagonally inward (135° = 180° - 45°)
#define DEFAULT_ANGLE_RO_DEG     105.0f   // Right Outer: Points mostly forward (105° = 180° - 75°)

// Sensor beam half-angle (RCWL-1601 nominal is ±15° cone = 30° total)
#define SENSOR_CONE_HALF_ANGLE_DEG 15.0f

// Distance limits (cm)
#define SENSOR_MIN_DIST_CM        10.0f
#define SENSOR_MAX_DIST_CM       350.0f
#define SENSOR_TIMEOUT_US        22000    // ~375cm round trip timeout

// =============================================================================
// FILTER & SENSOR HARDENING CONSTANTS
// =============================================================================
#define SENSOR_MEDIAN_BUFFER_SIZE  5       // 5-sample median window for robust outlier rejection
#define SENSOR_DEFAULT_EMA_ALPHA   0.35f   // Running EMA blend post-median
#define SENSOR_SPIKE_GATE_CM       25.0f   // Max allowable single-step jump in cm
#define SENSOR_SPIKE_CONFIRM_COUNT 3       // Consecutive agreeing readings needed to accept jump
#define SENSOR_GHOST_ECHO_NEAR_CM  28.0f   // Near-field enclosure/lip ghost reflection threshold (cm)
#define SENSOR_GHOST_ECHO_FAR_CM   75.0f   // If other sensors exceed this, near readings are rejected as ghosts

// =============================================================================
// ADAPTIVE KALMAN FILTER TUNING
// =============================================================================
#define KALMAN_ADAPTIVE_Q_GAIN     1.0f    // Scaling factor for process noise during physical movement
#define KALMAN_ADAPTIVE_R_GAIN     2.5f    // Scaling factor to INCREASE measurement noise on outlier innovations
#define KALMAN_MIN_Q               0.015f  // Minimum Q (high smoothing at rest)
#define KALMAN_MAX_Q               0.30f   // Maximum Q
#define KALMAN_MAX_INNOV_STEP_CM   20.0f   // Maximum innovation jump accepted per frame (cm)

// =============================================================================
// TRACKING PROFILES & TUNING PRESETS
// =============================================================================
enum TrackingProfile : uint8_t {
    PROFILE_BALANCED   = 0,
    PROFILE_RESPONSIVE = 1,
    PROFILE_STABLE     = 2,
    PROFILE_CUSTOM     = 3
};

// Profile 1: Responsive (Fast lunges, low lag, competitive Whack-a-Mole)
#define PROFILE_RESPONSIVE_EMA_ALPHA   0.50f
#define PROFILE_RESPONSIVE_KALMAN_Q    0.15f
#define PROFILE_RESPONSIVE_KALMAN_R    0.20f
#define PROFILE_RESPONSIVE_SPIKE_CM    35.0f
#define PROFILE_RESPONSIVE_DAMP_REST   0.90f
#define PROFILE_RESPONSIVE_DAMP_MOVE   0.98f

// Profile 0: Balanced (Default optimal blend of smoothing and responsiveness)
#define PROFILE_BALANCED_EMA_ALPHA     0.35f
#define PROFILE_BALANCED_KALMAN_Q      0.08f
#define PROFILE_BALANCED_KALMAN_R      0.35f
#define PROFILE_BALANCED_SPIKE_CM      25.0f
#define PROFILE_BALANCED_DAMP_REST     0.84f
#define PROFILE_BALANCED_DAMP_MOVE     0.97f

// Profile 2: Stable (Rock-solid cursor, zero jitter at rest, smooth menus)
#define PROFILE_STABLE_EMA_ALPHA       0.20f
#define PROFILE_STABLE_KALMAN_Q        0.03f
#define PROFILE_STABLE_KALMAN_R        0.60f
#define PROFILE_STABLE_SPIKE_CM        18.0f
#define PROFILE_STABLE_DAMP_REST       0.75f
#define PROFILE_STABLE_DAMP_MOVE       0.95f

// =============================================================================
// DETECTION HYSTERESIS & TARGET GUIDANCE (AIM ASSIST)
// =============================================================================
#define DETECTION_HYSTERESIS_ON    2       // Consecutive valid frames before declaring player detected
#define DETECTION_HYSTERESIS_OFF   20      // Consecutive missing frames (~1.0s) before declaring player lost

#define TARGET_MAX_COUNT           16      // Maximum target points (mole holes / UI buttons)
#define TARGET_DEFAULT_SNAP        0.20f   // Legacy default snap strength

// Target Types
enum TargetType : uint8_t {
    TARGET_TYPE_HOLE   = 0,  // Gameplay mole hole (subtle guidance, ~0.20)
    TARGET_TYPE_BUTTON = 1   // UI Button / Menu item (high stickiness, ~0.50)
};

// Differentiated snap defaults (gentle gravitational pull, not hard snap)
#define DEFAULT_SNAP_HOLE          0.08f   // Subtle gentle gravity for gameplay mole holes
#define DEFAULT_SNAP_BUTTON        0.20f   // Gentle sticky gravity for UI buttons
#define DEFAULT_SNAP_GLOBAL        1.0f    // Master snap multiplier

#define DEFAULT_DRIFT_CORRECTION_RATE 0.015f // Angle adjustment rate per stable second (degrees)
#define DEFAULT_DRIFT_MAX_DEG         10.0f  // Max allowable angle drift from factory/calibrated baseline
#define DRIFT_CORRECTION_RATE         DEFAULT_DRIFT_CORRECTION_RATE
#define BOOT_CALIB_DURATION_MS        2500   // Passive baseline calibration on boot (ms)

// Quick-Align Single-Point Calibration
#define QUICK_ALIGN_SAMPLE_COUNT      20     // ~1 second collection at 20Hz
#define QUICK_ALIGN_MIN_DIST_CM       15.0f  // Minimum valid sensor reading for quick-align
#define RESIDUAL_MISALIGN_THRESHOLD_CM 15.0f // Average residual threshold to flag suspected misalignment

// =============================================================================
// TIMING INTERVALS
// =============================================================================
#define SENSOR_STAGGER_INTERVAL_MS 15     // Delay between firing sensors on same module
#define TRACKING_UPDATE_INTERVAL_MS 50    // 20Hz position update rate
#define ESPNOW_PACKET_TIMEOUT_MS   300    // Mark secondary offline if no packet received

// =============================================================================
// WIFI & WEBSOCKET CONFIGURATION
// =============================================================================
#define WIFI_AP_SSID          "ESP32-Ultrasonic-Lab"
#define WIFI_AP_PASSWORD      "password123"
#define WIFI_AP_CHANNEL       1           // Locked channel for ESP-NOW coexistence
#define WEBSOCKET_PORT        81
#define HTTP_SERVER_PORT      80

// ESP-NOW Broadcast MAC
static const uint8_t ESPNOW_BROADCAST_MAC[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

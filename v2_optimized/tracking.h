#pragma once
#include <Arduino.h>
#include "config.h"
#include "calibration.h"

struct TargetPoint {
    float xCm;
    float yCm;
    float radiusCm;
    TargetType type;     // TARGET_TYPE_HOLE = 0, TARGET_TYPE_BUTTON = 1
    float snapOverride; // -1.0f to use type default, or 0.0 to 1.0 custom override
    bool  active;
};

struct TrackedPosition {
    float xCm;            // Estimated X in cm (0 to Area Width)
    float yCm;            // Estimated Y in cm (0 to Area Depth + Dead Zone)
    float normX;          // Normalised 0.0 to 1.0 (left to right in playing area)
    float normY;          // Normalised 0.0 to 1.0 (front to back in playing area)
    float rawXCm;         // Pre-Kalman raw multilateration X (cm)
    float rawYCm;         // Pre-Kalman raw multilateration Y (cm)
    float vxCmS;          // Estimated velocity in X (cm/s)
    float vyCmS;          // Estimated velocity in Y (cm/s)
    float speedCmS;       // Speed magnitude in cm/s
    float confidence;     // 0.0 to 1.0 confidence score
    bool  isDetected;     // True if player position is actively tracked (debounced)
    bool  inDeadZone;     // True if player is within dead zone (Y < deadZoneCm)
    uint8_t activeSensors;// Bitmask of active sensors (b0=LO, b1=LI, b2=RI, b3=RO)
    uint32_t timestampMs;
};

struct SensorAlignmentHealth {
    float avgResidualCm;       // Running average residual error (cm)
    float maxResidualCm;       // Peak residual in recent window (cm)
    bool  suspectedMisalign;   // True if residual persistently high (> 15cm)
};

class PositionTracker {
public:
    PositionTracker();
    void begin(const SystemCalibration& calib);
    void updateCalibration(const SystemCalibration& calib);

    // Feed the 4 sensor readings and calculate the filtered target position
    TrackedPosition process(
        float distLO, bool validLO,
        float distLI, bool validLI,
        float distRI, bool validRI,
        float distRO, bool validRO,
        float dtSeconds
    );

    void resetFilter();

    // Alignment & Residual Health
    const SensorAlignmentHealth* getSensorHealth() const { return _sensorHealth; }
    void resetAlignmentHealth();

    // Profile Management (Responsive, Balanced, Stable, Custom)
    void setProfile(TrackingProfile profile);
    TrackingProfile getProfile() const { return _profile; }
    const char* getProfileName() const;

    // Target-aware guidance (aim assist for mole holes & UI buttons)
    void setTargets(const TargetPoint* targets, uint8_t count);
    void setTargets(const TargetPoint* targets, uint8_t count, float snapStrength) {
        setTargets(targets, count);
        if (snapStrength >= 0.0f) {
            _snapHole = snapStrength;
            _snapGlobal = 1.0f;
        }
    }
    void clearTargets();
    uint8_t getTargets(TargetPoint* outTargets, uint8_t maxCount) const;

    void setSnapStrength(float snap) {
        _snapHole = snap;
        _snapGlobal = 1.0f;
    }
    void setSnapConfig(float holeSnap, float buttonSnap, float globalSnap = 1.0f);
    void getSnapConfig(float& holeSnap, float& buttonSnap, float& globalSnap) const {
        holeSnap = _snapHole;
        buttonSnap = _snapButton;
        globalSnap = _snapGlobal;
    }

    float getAdaptiveQ() const { return _adaptiveQ; }
    float getAdaptiveR() const { return _adaptiveR; }
    float getInnovation() const { return _innovationMag; }

private:
    SystemCalibration _calib;

    // Active Profile & Damping
    TrackingProfile _profile;
    float _dampRest;
    float _dampMove;

    // Alignment health per sensor [LO, LI, RI, RO]
    SensorAlignmentHealth _sensorHealth[4];

    // 2D Kalman Filter state [x, y, vx, vy]
    float _kfX, _kfY;
    float _kfVx, _kfVy;
    float _cov[4][4];

    // Adaptive noise tuning
    float _adaptiveQ;
    float _adaptiveR;
    float _innovationMag;

    // Debounce / Detection hysteresis
    uint8_t _consecutiveDetected;
    uint8_t _consecutiveUndetected;
    bool _stableDetected;

    // Target gravity wells
    TargetPoint _targets[TARGET_MAX_COUNT];
    uint8_t _numTargets;
    float _snapHole;
    float _snapButton;
    float _snapGlobal;

    uint32_t _lastValidMs;
    bool _isInitialized;

    void kalmanPredict(float dt);
    void kalmanUpdate(float measX, float measY, float confidence);
    void applyTargetGuidance(float& x, float& y);
    void updateDriftCorrection(float x, float y, float conf, const float dists[4], const bool valids[4]);
    void updateAlignmentHealth(float x, float y, const float dists[4], const bool valids[4]);

    bool solveMultilateration(
        float dLO, bool vLO,
        float dLI, bool vLI,
        float dRI, bool vRI,
        float dRO, bool vRO,
        float& outX, float& outY,
        float& outConfidence,
        uint8_t& outMask
    );
};


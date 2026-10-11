#include "tracking.h"
#include <math.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105f
#endif

static inline float clampf(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

static inline float wrapAngleDeg(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

PositionTracker::PositionTracker() 
    : _profile(PROFILE_BALANCED),
      _dampRest(PROFILE_BALANCED_DAMP_REST),
      _dampMove(PROFILE_BALANCED_DAMP_MOVE),
      _kfX(75.0f), _kfY(70.0f), _kfVx(0.0f), _kfVy(0.0f),
      _adaptiveQ(0.08f), _adaptiveR(0.35f), _innovationMag(0.0f),
      _consecutiveDetected(0), _consecutiveUndetected(0), _stableDetected(false),
      _numTargets(0),
      _snapHole(DEFAULT_SNAP_HOLE),
      _snapButton(DEFAULT_SNAP_BUTTON),
      _snapGlobal(DEFAULT_SNAP_GLOBAL),
      _lastValidMs(0), _isInitialized(false) {
    resetFilter();
    resetAlignmentHealth();
}

void PositionTracker::resetAlignmentHealth() {
    for (int i = 0; i < 4; i++) {
        _sensorHealth[i].avgResidualCm = 0.0f;
        _sensorHealth[i].maxResidualCm = 0.0f;
        _sensorHealth[i].suspectedMisalign = false;
    }
}

void PositionTracker::begin(const SystemCalibration& calib) {
    _calib = calib;
    _profile = (TrackingProfile)calib.profile;
    _snapHole = calib.snapHole;
    _snapButton = calib.snapButton;
    _snapGlobal = calib.snapGlobal;
    setProfile(_profile);
    resetFilter();
    resetAlignmentHealth();
}

void PositionTracker::updateCalibration(const SystemCalibration& calib) {
    _calib = calib;
    _snapHole = calib.snapHole;
    _snapButton = calib.snapButton;
    _snapGlobal = calib.snapGlobal;
    if (_profile == PROFILE_CUSTOM) {
        _adaptiveQ = _calib.kalmanQ;
        _adaptiveR = _calib.kalmanR;
    }
}

void PositionTracker::setProfile(TrackingProfile profile) {
    _profile = profile;
    switch (profile) {
        case PROFILE_RESPONSIVE:
            _adaptiveQ = PROFILE_RESPONSIVE_KALMAN_Q;
            _adaptiveR = PROFILE_RESPONSIVE_KALMAN_R;
            _dampRest  = PROFILE_RESPONSIVE_DAMP_REST;
            _dampMove  = PROFILE_RESPONSIVE_DAMP_MOVE;
            break;
        case PROFILE_STABLE:
            _adaptiveQ = PROFILE_STABLE_KALMAN_Q;
            _adaptiveR = PROFILE_STABLE_KALMAN_R;
            _dampRest  = PROFILE_STABLE_DAMP_REST;
            _dampMove  = PROFILE_STABLE_DAMP_MOVE;
            break;
        case PROFILE_CUSTOM:
            _adaptiveQ = _calib.kalmanQ > 0.001f ? _calib.kalmanQ : PROFILE_BALANCED_KALMAN_Q;
            _adaptiveR = _calib.kalmanR > 0.001f ? _calib.kalmanR : PROFILE_BALANCED_KALMAN_R;
            _dampRest  = PROFILE_BALANCED_DAMP_REST;
            _dampMove  = PROFILE_BALANCED_DAMP_MOVE;
            break;
        case PROFILE_BALANCED:
        default:
            _profile   = PROFILE_BALANCED;
            _adaptiveQ = PROFILE_BALANCED_KALMAN_Q;
            _adaptiveR = PROFILE_BALANCED_KALMAN_R;
            _dampRest  = PROFILE_BALANCED_DAMP_REST;
            _dampMove  = PROFILE_BALANCED_DAMP_MOVE;
            break;
    }
    _calib.profile = (uint8_t)_profile;
    _calib.kalmanQ = _adaptiveQ;
    _calib.kalmanR = _adaptiveR;
}

const char* PositionTracker::getProfileName() const {
    switch (_profile) {
        case PROFILE_RESPONSIVE: return "responsive";
        case PROFILE_STABLE:     return "stable";
        case PROFILE_CUSTOM:     return "custom";
        case PROFILE_BALANCED:
        default:                 return "balanced";
    }
}

void PositionTracker::setSnapConfig(float holeSnap, float buttonSnap, float globalSnap) {
    _snapHole   = clampf(holeSnap, 0.0f, 0.95f);
    _snapButton = clampf(buttonSnap, 0.0f, 0.95f);
    _snapGlobal = clampf(globalSnap, 0.0f, 2.0f);
    _calib.snapHole   = _snapHole;
    _calib.snapButton = _snapButton;
    _calib.snapGlobal = _snapGlobal;
}

void PositionTracker::resetFilter() {
    _kfX = _calib.areaWidthCm * 0.5f;
    _kfY = _calib.deadZoneCm + (_calib.areaDepthCm * 0.5f);
    _kfVx = 0.0f;
    _kfVy = 0.0f;
    _adaptiveQ = _calib.kalmanQ > 0.001f ? _calib.kalmanQ : 0.08f;
    _adaptiveR = _calib.kalmanR > 0.001f ? _calib.kalmanR : 0.35f;
    _innovationMag = 0.0f;
    _consecutiveDetected = 0;
    _consecutiveUndetected = 0;
    _stableDetected = false;
    _lastValidMs = 0;
    _isInitialized = false;

    // Reset covariance matrix with initial uncertainty
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            _cov[i][j] = (i == j) ? 15.0f : 0.0f;
        }
    }
}

void PositionTracker::setTargets(const TargetPoint* targets, uint8_t count) {
    _numTargets = (count > TARGET_MAX_COUNT) ? TARGET_MAX_COUNT : count;
    for (uint8_t i = 0; i < _numTargets; i++) {
        _targets[i] = targets[i];
    }
}

void PositionTracker::clearTargets() {
    _numTargets = 0;
}

uint8_t PositionTracker::getTargets(TargetPoint* outTargets, uint8_t maxCount) const {
    if (!outTargets) return 0;
    uint8_t count = (_numTargets < maxCount) ? _numTargets : maxCount;
    for (uint8_t i = 0; i < count; i++) {
        outTargets[i] = _targets[i];
    }
    return count;
}

void PositionTracker::applyTargetGuidance(float& x, float& y) {
    if (_numTargets == 0 || _snapGlobal <= 0.001f) return;

    float bestDist = 9999.0f;
    int bestIdx = -1;

    for (uint8_t i = 0; i < _numTargets; i++) {
        if (!_targets[i].active) continue;
        float dx = x - _targets[i].xCm;
        float dy = y - _targets[i].yCm;
        float d = sqrtf(dx * dx + dy * dy);
        float captureRadius = _targets[i].radiusCm * 2.5f;
        if (d < captureRadius && d < bestDist) {
            bestDist = d;
            bestIdx = i;
        }
    }

    if (bestIdx >= 0) {
        const TargetPoint& tgt = _targets[bestIdx];
        float captureRadius = tgt.radiusCm * 2.5f;
        // Smooth quadratic falloff: pull is strongest at center, smoothly approaches 0 at edge
        float normDist = bestDist / captureRadius; // 0.0 at center, 1.0 at edge

        // Base snap determined by target type (mole hole vs button) or individual override
        float baseSnap;
        if (tgt.snapOverride >= 0.0f) {
            baseSnap = tgt.snapOverride;
        } else if (tgt.type == TARGET_TYPE_BUTTON) {
            baseSnap = _snapButton;
        } else {
            baseSnap = _snapHole;
        }

        float effectiveSnap = clampf(baseSnap * _snapGlobal, 0.0f, 0.95f);
        float pullFactor = effectiveSnap * (1.0f - normDist) * (1.0f - normDist);
        x += pullFactor * (tgt.xCm - x);
        y += pullFactor * (tgt.yCm - y);
    }
}

void PositionTracker::kalmanPredict(float dt) {
    if (dt <= 0.0f || dt > 0.3f) dt = 0.05f;

    // 1. State transition: pos = pos + vel * dt
    _kfX += _kfVx * dt;
    _kfY += _kfVy * dt;

    // 2. Profile-dependent velocity damping:
    // Heavy damping at rest prevents wandering/drift;
    // Light damping at higher speeds preserves responsive hand/body lunges.
    float speed = sqrtf(_kfVx * _kfVx + _kfVy * _kfVy);
    float damping;
    if (speed < 10.0f) {
        damping = _dampRest;
    } else if (speed < 60.0f) {
        float t = (speed - 10.0f) / 50.0f;
        damping = _dampRest + t * (_dampMove - _dampRest);
    } else {
        damping = _dampMove;
    }
    _kfVx *= damping;
    _kfVy *= damping;

    // 3. Covariance Propagation with full cross-covariance coupling P = F * P * F^T + Q
    // For X:
    float p00 = _cov[0][0] + 2.0f * dt * _cov[0][2] + dt * dt * _cov[2][2];
    float p02 = _cov[0][2] + dt * _cov[2][2];
    float p22 = _cov[2][2];

    // For Y:
    float p11 = _cov[1][1] + 2.0f * dt * _cov[1][3] + dt * dt * _cov[3][3];
    float p13 = _cov[1][3] + dt * _cov[3][3];
    float p33 = _cov[3][3];

    // Process noise addition
    float q = _adaptiveQ > 0.001f ? _adaptiveQ : _calib.kalmanQ;
    _cov[0][0] = p00 + q * dt;
    _cov[0][2] = p02;
    _cov[2][0] = p02;
    _cov[2][2] = p22 + q * dt * 2.5f;

    _cov[1][1] = p11 + q * dt;
    _cov[1][3] = p13;
    _cov[3][1] = p13;
    _cov[3][3] = p33 + q * dt * 2.5f;
}

void PositionTracker::kalmanUpdate(float measX, float measY, float confidence) {
    // 1. Innovation (measurement residual)
    float y_x = measX - _kfX;
    float y_y = measY - _kfY;
    _innovationMag = sqrtf(y_x * y_x + y_y * y_y);

    // 2. Adaptive noise parameters:
    // High innovation -> player in rapid motion -> increase Q, decrease R to track immediately without lag
    // Low innovation  -> stationary player -> decrease Q, increase R to suppress jitter completely
    float baseQ = _calib.kalmanQ > 0.001f ? _calib.kalmanQ : 0.08f;
    float baseR = _calib.kalmanR > 0.001f ? _calib.kalmanR : 0.35f;
    float conf = clampf(confidence, 0.15f, 1.0f);

    float innovNorm = clampf(_innovationMag / 35.0f, 0.0f, 3.0f);
    _adaptiveQ = clampf(baseQ * (1.0f + KALMAN_ADAPTIVE_Q_GAIN * innovNorm), KALMAN_MIN_Q, KALMAN_MAX_Q);
    _adaptiveR = clampf((baseR / conf) / (1.0f + KALMAN_ADAPTIVE_R_GAIN * innovNorm), 0.05f, 2.5f);

    // 3. Innovation covariance S
    float s_x = _cov[0][0] + _adaptiveR;
    float s_y = _cov[1][1] + _adaptiveR;

    // 4. Optimal Kalman gain vector (properly updates both position and velocity)
    float k_x_pos = _cov[0][0] / s_x;
    float k_x_vel = _cov[2][0] / s_x;
    float k_y_pos = _cov[1][1] / s_y;
    float k_y_vel = _cov[3][1] / s_y;

    // 5. State update
    _kfX  += k_x_pos * y_x;
    _kfVx += k_x_vel * y_x;
    _kfY  += k_y_pos * y_y;
    _kfVy += k_y_vel * y_y;

    // Clamp velocities to human physical limits (< 350 cm/s)
    _kfVx = clampf(_kfVx, -350.0f, 350.0f);
    _kfVy = clampf(_kfVy, -350.0f, 350.0f);

    // 6. Covariance update: P = (I - K*H) * P
    _cov[0][0] -= k_x_pos * _cov[0][0];
    _cov[0][2] -= k_x_pos * _cov[0][2];
    _cov[2][0] = _cov[0][2];
    _cov[2][2] -= k_x_vel * _cov[0][2];

    _cov[1][1] -= k_y_pos * _cov[1][1];
    _cov[1][3] -= k_y_pos * _cov[1][3];
    _cov[3][1] = _cov[1][3];
    _cov[3][3] -= k_y_vel * _cov[1][3];

    // Guarantee positive semi-definite diagonal
    if (_cov[0][0] < 0.01f) _cov[0][0] = 0.01f;
    if (_cov[1][1] < 0.01f) _cov[1][1] = 0.01f;
    if (_cov[2][2] < 0.01f) _cov[2][2] = 0.01f;
    if (_cov[3][3] < 0.01f) _cov[3][3] = 0.01f;
}

void PositionTracker::updateDriftCorrection(float x, float y, float conf, const float dists[4], const bool valids[4]) {
    // Only apply tiny angle nudge if player is stationary, confidence is high, and all sensors valid
    float speed = sqrtf(_kfVx * _kfVx + _kfVy * _kfVy);
    if (conf < 0.85f || speed > 5.0f) return;
    if (!valids[0] || !valids[1] || !valids[2] || !valids[3]) return;

    float rate = (_calib.driftRate > 0.0001f) ? _calib.driftRate : DEFAULT_DRIFT_CORRECTION_RATE;
    float maxCap = (_calib.driftMaxDeg > 0.5f) ? _calib.driftMaxDeg : DEFAULT_DRIFT_MAX_DEG;

    struct SensorGeom {
        float sx, sy;
        float* anglePtr;
        float defaultAngle;
    } geoms[4] = {
        { _calib.sensorLO_x, _calib.sensorLO_y, &_calib.angleLO_deg, DEFAULT_ANGLE_LO_DEG },
        { _calib.sensorLI_x, _calib.sensorLI_y, &_calib.angleLI_deg, DEFAULT_ANGLE_LI_DEG },
        { _calib.sensorRI_x, _calib.sensorRI_y, &_calib.angleRI_deg, DEFAULT_ANGLE_RI_DEG },
        { _calib.sensorRO_x, _calib.sensorRO_y, &_calib.angleRO_deg, DEFAULT_ANGLE_RO_DEG }
    };

    for (int i = 0; i < 4; i++) {
        float dx = x - geoms[i].sx;
        float dy = y - geoms[i].sy;
        float r = sqrtf(dx * dx + dy * dy);
        if (r < 15.0f) continue;

        // Angle from sensor to target
        float targetAngleDeg = atan2f(dy, dx) * RAD_TO_DEG;
        float diffAngle = wrapAngleDeg(targetAngleDeg - *(geoms[i].anglePtr));

        // Gradual drift adjustment toward target ray
        float adjustment = diffAngle * rate;
        float newAngle = *(geoms[i].anglePtr) + adjustment;

        // Constrain within allowable deviation cap of factory default
        if (fabsf(newAngle - geoms[i].defaultAngle) <= maxCap) {
            *(geoms[i].anglePtr) = newAngle;
        }
    }
}

void PositionTracker::updateAlignmentHealth(float x, float y, const float dists[4], const bool valids[4]) {
    struct SensorPos { float sx, sy; } sensors[4] = {
        { _calib.sensorLO_x, _calib.sensorLO_y },
        { _calib.sensorLI_x, _calib.sensorLI_y },
        { _calib.sensorRI_x, _calib.sensorRI_y },
        { _calib.sensorRO_x, _calib.sensorRO_y }
    };

    for (int i = 0; i < 4; i++) {
        if (!valids[i] || dists[i] < 10.0f) {
            // Decay max residual when no echo
            _sensorHealth[i].maxResidualCm *= 0.995f;
            continue;
        }
        float dx = x - sensors[i].sx;
        float dy = y - sensors[i].sy;
        float expectedR = sqrtf(dx * dx + dy * dy);
        float residual = fabsf(expectedR - dists[i]);

        // EWMA update (alpha = 0.05)
        _sensorHealth[i].avgResidualCm = 0.05f * residual + 0.95f * _sensorHealth[i].avgResidualCm;
        if (residual > _sensorHealth[i].maxResidualCm) {
            _sensorHealth[i].maxResidualCm = residual;
        } else {
            _sensorHealth[i].maxResidualCm *= 0.995f;
        }

        _sensorHealth[i].suspectedMisalign = (_sensorHealth[i].avgResidualCm > RESIDUAL_MISALIGN_THRESHOLD_CM);
    }
}

bool PositionTracker::solveMultilateration(
    float dLO, bool vLO,
    float dLI, bool vLI,
    float dRI, bool vRI,
    float dRO, bool vRO,
    float& outX, float& outY,
    float& outConfidence,
    uint8_t& outMask
) {
    outMask = 0;
    if (vLO && dLO > 10.0f) outMask |= (1 << 0);
    if (vLI && dLI > 10.0f) outMask |= (1 << 1);
    if (vRI && dRI > 10.0f) outMask |= (1 << 2);
    if (vRO && dRO > 10.0f) outMask |= (1 << 3);

    int count = __builtin_popcount(outMask);
    if (count == 0) {
        outConfidence = 0.0f;
        return false;
    }

    struct SensorInfo {
        float x, y, angleDeg, dist;
        bool valid;
    } sensors[4] = {
        { _calib.sensorLO_x, _calib.sensorLO_y, _calib.angleLO_deg, dLO, (outMask & (1 << 0)) != 0 },
        { _calib.sensorLI_x, _calib.sensorLI_y, _calib.angleLI_deg, dLI, (outMask & (1 << 1)) != 0 },
        { _calib.sensorRI_x, _calib.sensorRI_y, _calib.angleRI_deg, dRI, (outMask & (1 << 2)) != 0 },
        { _calib.sensorRO_x, _calib.sensorRO_y, _calib.angleRO_deg, dRO, (outMask & (1 << 3)) != 0 }
    };

    // Separate active sensors by module
    bool hasLeft = (outMask & 0x03) != 0;   // LO or LI
    bool hasRight = (outMask & 0x0C) != 0;  // RI or RO

    // Compute Left module ray / estimate
    float leftX = 0, leftY = 0, leftW = 0;
    if (sensors[0].valid) { // LO
        float rad = sensors[0].angleDeg * DEG_TO_RAD;
        leftX += (sensors[0].x + sensors[0].dist * cosf(rad));
        leftY += (sensors[0].y + sensors[0].dist * sinf(rad));
        leftW += 1.0f;
    }
    if (sensors[1].valid) { // LI
        float rad = sensors[1].angleDeg * DEG_TO_RAD;
        leftX += (sensors[1].x + sensors[1].dist * cosf(rad));
        leftY += (sensors[1].y + sensors[1].dist * sinf(rad));
        leftW += 1.0f;
    }
    if (leftW > 0) { leftX /= leftW; leftY /= leftW; }

    // Compute Right module ray / estimate
    float rightX = 0, rightY = 0, rightW = 0;
    if (sensors[2].valid) { // RI
        float rad = sensors[2].angleDeg * DEG_TO_RAD;
        rightX += (sensors[2].x + sensors[2].dist * cosf(rad));
        rightY += (sensors[2].y + sensors[2].dist * sinf(rad));
        rightW += 1.0f;
    }
    if (sensors[3].valid) { // RO
        float rad = sensors[3].angleDeg * DEG_TO_RAD;
        rightX += (sensors[3].x + sensors[3].dist * cosf(rad));
        rightY += (sensors[3].y + sensors[3].dist * sinf(rad));
        rightW += 1.0f;
    }
    if (rightW > 0) { rightX /= rightW; rightY /= rightW; }

    // Single-module fallback with smooth transition from previous state
    if (hasLeft && !hasRight) {
        outX = leftX;
        outY = fmaxf(5.0f, leftY);
        if (_isInitialized) {
            outX = 0.75f * leftX + 0.25f * _kfX;
            outY = 0.75f * leftY + 0.25f * _kfY;
        }
        outConfidence = (count >= 2) ? 0.70f : 0.45f;
        return true;
    }
    if (!hasLeft && hasRight) {
        outX = rightX;
        outY = fmaxf(5.0f, rightY);
        if (_isInitialized) {
            outX = 0.75f * rightX + 0.25f * _kfX;
            outY = 0.75f * rightY + 0.25f * _kfY;
        }
        outConfidence = (count >= 2) ? 0.70f : 0.45f;
        return true;
    }

    // Both modules have readings: seed Gauss-Newton initial estimate
    float estX = (leftX + rightX) * 0.5f;
    float estY = (leftY + rightY) * 0.5f;
    if (estY < 10.0f) estY = 30.0f;

    // Fast 2D Gauss-Newton iterative solver (4 iterations)
    for (int iter = 0; iter < 4; iter++) {
        float JtJ_00 = 0.0f, JtJ_01 = 0.0f, JtJ_11 = 0.0f;
        float Jtr_0 = 0.0f, Jtr_1 = 0.0f;

        for (int i = 0; i < 4; i++) {
            if (!sensors[i].valid) continue;

            float dx = estX - sensors[i].x;
            float dy = estY - sensors[i].y;
            float r = sqrtf(dx * dx + dy * dy);
            if (r < 0.001f) r = 0.001f;

            // Cone weighting: angle deviation from sensor boresight
            float targetAngleDeg = atan2f(dy, dx) * RAD_TO_DEG;
            float devDeg = fabsf(wrapAngleDeg(targetAngleDeg - sensors[i].angleDeg));
            
            // Weight decreases smoothly towards or beyond cone edge (±25°)
            float w = 1.0f;
            if (devDeg > 25.0f) {
                w = 0.15f;
            } else {
                float normDev = devDeg / 25.0f;
                w = cosf(normDev * 1.5707963f);
                w = w * w;
                if (w < 0.15f) w = 0.15f;
            }

            // Residual: estimated radius minus measured distance
            float res = r - sensors[i].dist;
            float jx = dx / r;
            float jy = dy / r;

            JtJ_00 += w * jx * jx;
            JtJ_01 += w * jx * jy;
            JtJ_11 += w * jy * jy;

            Jtr_0 += w * jx * res;
            Jtr_1 += w * jy * res;
        }

        // Add Levenberg-Marquardt regularisation damping
        JtJ_00 += 0.1f;
        JtJ_11 += 0.1f;

        // 2x2 Matrix Inversion: [a b; b c]^-1 = 1/det * [c -b; -b a]
        float det = JtJ_00 * JtJ_11 - JtJ_01 * JtJ_01;
        if (fabsf(det) < 1e-6f) break;

        float invDet = 1.0f / det;
        float stepX = invDet * (JtJ_11 * Jtr_0 - JtJ_01 * Jtr_1);
        float stepY = invDet * (-JtJ_01 * Jtr_0 + JtJ_00 * Jtr_1);

        estX -= clampf(stepX, -30.0f, 30.0f);
        estY -= clampf(stepY, -30.0f, 30.0f);

        // Constrain Y to be in front of the modules (prevent collapsing behind wall)
        if (estY < 2.0f) estY = 2.0f;
    }

    outX = clampf(estX, -10.0f, _calib.areaWidthCm + 10.0f);
    outY = fmaxf(2.0f, estY);

    // Confidence calculation based on number of sensors and residual agreement
    float totalResidual = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (!sensors[i].valid) continue;
        float dx = outX - sensors[i].x;
        float dy = outY - sensors[i].y;
        float r = sqrtf(dx * dx + dy * dy);
        totalResidual += fabsf(r - sensors[i].dist);
    }
    float avgResidual = totalResidual / (float)count;
    
    // Confidence formula
    float baseConf = (count >= 3) ? 0.95f : (count == 2 ? 0.80f : 0.45f);
    outConfidence = clampf(baseConf - (avgResidual * 0.015f), 0.25f, 1.0f);

    return true;
}

TrackedPosition PositionTracker::process(
    float distLO, bool validLO,
    float distLI, bool validLI,
    float distRI, bool validRI,
    float distRO, bool validRO,
    float dtSeconds
) {
    TrackedPosition result;
    result.timestampMs = millis();
    result.activeSensors = 0;

    float rawX = 0.0f, rawY = 0.0f, rawConf = 0.0f;
    uint8_t mask = 0;

    bool solved = solveMultilateration(distLO, validLO, distLI, validLI, distRI, validRI, distRO, validRO, rawX, rawY, rawConf, mask);
    result.activeSensors = mask;

    kalmanPredict(dtSeconds);

    if (solved) {
        if (!_isInitialized) {
            _kfX = rawX;
            _kfY = rawY;
            _isInitialized = true;
        } else {
            kalmanUpdate(rawX, rawY, rawConf);
        }
        _lastValidMs = millis();

        // Debounce detection onset
        _consecutiveDetected++;
        _consecutiveUndetected = 0;
        if (_consecutiveDetected >= DETECTION_HYSTERESIS_ON) {
            _stableDetected = true;
        }

        // Runtime gentle angle drift calibration
        const float dists[4] = { distLO, distLI, distRI, distRO };
        const bool valids[4] = { validLO, validLI, validRI, validRO };
        updateDriftCorrection(_kfX, _kfY, rawConf, dists, valids);

        // Continuous alignment residual monitoring
        updateAlignmentHealth(_kfX, _kfY, dists, valids);

    } else {
        // Lost target debounce
        _consecutiveUndetected++;
        _consecutiveDetected = 0;
        if (_consecutiveUndetected >= DETECTION_HYSTERESIS_OFF || (millis() - _lastValidMs > 600)) {
            _stableDetected = false;
        }
    }

    result.isDetected = _stableDetected;
    if (_stableDetected) {
        result.confidence = solved ? rawConf : (0.35f * (1.0f - clampf((float)(millis() - _lastValidMs) / 600.0f, 0.0f, 1.0f)));
    } else {
        result.confidence = 0.0f;
    }

    // Apply target-aware guidance (gravity wells) to filtered position
    float guidedX = _kfX;
    float guidedY = _kfY;
    if (_stableDetected) {
        applyTargetGuidance(guidedX, guidedY);
    }

    result.xCm = guidedX;
    result.yCm = guidedY;
    result.rawXCm = rawX;
    result.rawYCm = rawY;
    result.vxCmS = _kfVx;
    result.vyCmS = _kfVy;
    result.speedCmS = sqrtf(_kfVx * _kfVx + _kfVy * _kfVy);

    // Normalised coordinates (0.0 to 1.0) clamped to playing area:
    // normX: 0.0 = Left wall (0cm), 1.0 = Right wall (areaWidthCm)
    // normY: 0.0 = Front edge of playing area (deadZoneCm), 1.0 = Back edge of playing area (deadZoneCm + areaDepthCm)
    result.normX = clampf(guidedX / _calib.areaWidthCm, 0.0f, 1.0f);
    result.normY = clampf((guidedY - _calib.deadZoneCm) / _calib.areaDepthCm, 0.0f, 1.0f);

    // Dead Zone Check: The dead zone is the 60cm region directly in front of the screen/wall (Y < deadZoneCm)
    result.inDeadZone = (guidedY < _calib.deadZoneCm);

    return result;
}


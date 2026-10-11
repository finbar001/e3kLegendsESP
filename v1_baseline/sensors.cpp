#include "sensors.h"
#include <math.h>

UltrasonicModule::UltrasonicModule() 
    : _trigOuter(0), _echoOuter(0), _trigInner(0), _echoInner(0),
      _histIdxOuter(0), _histIdxInner(0),
      _histCountOuter(0), _histCountInner(0),
      _emaOuter(0.0f), _emaInner(0.0f),
      _jitterOuter(0.0f), _jitterInner(0.0f),
      _spikeCountOuter(0), _spikeCountInner(0),
      _emaAlpha(SENSOR_DEFAULT_EMA_ALPHA),
      _spikeGateCm(SENSOR_SPIKE_GATE_CM) {
    memset(&_readingOuter, 0, sizeof(_readingOuter));
    memset(&_readingInner, 0, sizeof(_readingInner));
    for (int i = 0; i < SENSOR_MEDIAN_BUFFER_SIZE; i++) {
        _historyOuter[i] = 0.0f;
        _historyInner[i] = 0.0f;
    }
}

void UltrasonicModule::begin(uint8_t trigOuter, uint8_t echoOuter, uint8_t trigInner, uint8_t echoInner) {
    _trigOuter = trigOuter;
    _echoOuter = echoOuter;
    _trigInner = trigInner;
    _echoInner = echoInner;

    pinMode(_trigOuter, OUTPUT);
    pinMode(_echoOuter, INPUT);
    pinMode(_trigInner, OUTPUT);
    pinMode(_echoInner, INPUT);

    digitalWrite(_trigOuter, LOW);
    digitalWrite(_trigInner, LOW);
}

float UltrasonicModule::computeMedian(const float* buf, uint8_t count) {
    if (count == 0) return 0.0f;
    if (count == 1) return buf[0];

    float temp[SENSOR_MEDIAN_BUFFER_SIZE];
    for (uint8_t i = 0; i < count; i++) {
        temp[i] = buf[i];
    }

    // In-place insertion sort (tiny array, very fast)
    for (uint8_t i = 1; i < count; i++) {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }

    return temp[count / 2];
}

float UltrasonicModule::measureSingle(uint8_t trigPin, uint8_t echoPin, float speedOfSoundCmUs) {
    // Ensure clean LOW state before trigger
    digitalWrite(trigPin, LOW);
    delayMicroseconds(4);

    // Send 10 microsecond HIGH pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read echo pulse with timeout
    unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);
    if (duration == 0) {
        return 0.0f; // Timeout or no echo
    }

    // Distance = (Time * Speed of sound) / 2
    float dist = (duration * speedOfSoundCmUs) * 0.5f;
    if (dist < SENSOR_MIN_DIST_CM || dist > SENSOR_MAX_DIST_CM) {
        return 0.0f;
    }
    return dist;
}

void UltrasonicModule::filterChannel(
    float rawDist,
    SensorReading& reading,
    float* histBuf,
    uint8_t& histIdx,
    uint8_t& histCount,
    float& emaVal,
    float& jitterVal,
    uint8_t& spikeCount
) {
    reading.rawDistanceCm = rawDist;
    reading.timestampMs = millis();

    if (rawDist > 0.0f) {
        // Push into median circular buffer
        histBuf[histIdx] = rawDist;
        histIdx = (histIdx + 1) % SENSOR_MEDIAN_BUFFER_SIZE;
        if (histCount < SENSOR_MEDIAN_BUFFER_SIZE) histCount++;

        // 1. Compute Median
        float med = computeMedian(histBuf, histCount);

        // 2. Spike Gate against running EMA
        float candidate = med;
        if (emaVal > 0.0f) {
            float delta = fabsf(med - emaVal);
            if (delta > _spikeGateCm) {
                spikeCount++;
                if (spikeCount < SENSOR_SPIKE_CONFIRM_COUNT) {
                    // Reject sudden spike, cap deviation to gate limit
                    candidate = (med > emaVal) ? (emaVal + _spikeGateCm) : (emaVal - _spikeGateCm);
                } else {
                    // Sustained movement or target step change
                    spikeCount = 0;
                }
            } else {
                spikeCount = 0;
            }
        }

        // 3. Update Exponential Moving Average (EMA)
        if (emaVal <= 0.0f) {
            emaVal = candidate;
        } else {
            emaVal = (_emaAlpha * candidate) + ((1.0f - _emaAlpha) * emaVal);
        }

        // 4. Update running jitter metric (EWMA of deviation between raw and smoothed)
        float dev = fabsf(rawDist - emaVal);
        if (jitterVal <= 0.0f) {
            jitterVal = dev;
        } else {
            jitterVal = (0.2f * dev) + (0.8f * jitterVal);
        }

        reading.filteredDistanceCm = emaVal;
        reading.emaDistanceCm = emaVal;
        reading.jitterCm = jitterVal;
        reading.isValid = true;
        reading.consecutiveValid++;
        reading.consecutiveInvalid = 0;

    } else {
        // Missing echo / timeout
        reading.consecutiveInvalid++;
        reading.consecutiveValid = 0;

        // Coast using EMA for up to 2 frames (~100ms) to prevent solver whiplash
        if (reading.consecutiveInvalid <= 2 && emaVal > 0.0f) {
            reading.isValid = true;
            reading.filteredDistanceCm = emaVal;
        } else {
            reading.isValid = false;
            // Clear filter state if missing for several consecutive frames
            if (reading.consecutiveInvalid > 4) {
                emaVal = 0.0f;
                jitterVal = 0.0f;
                histCount = 0;
                reading.filteredDistanceCm = 0.0f;
                reading.emaDistanceCm = 0.0f;
                reading.jitterCm = 0.0f;
            }
        }
    }
}

void UltrasonicModule::resetFilter() {
    _histIdxOuter = 0;
    _histIdxInner = 0;
    _histCountOuter = 0;
    _histCountInner = 0;
    _emaOuter = 0.0f;
    _emaInner = 0.0f;
    _jitterOuter = 0.0f;
    _jitterInner = 0.0f;
    _spikeCountOuter = 0;
    _spikeCountInner = 0;
    for (int i = 0; i < SENSOR_MEDIAN_BUFFER_SIZE; i++) {
        _historyOuter[i] = 0.0f;
        _historyInner[i] = 0.0f;
    }
    memset(&_readingOuter, 0, sizeof(_readingOuter));
    memset(&_readingInner, 0, sizeof(_readingInner));
}

void UltrasonicModule::update(float ambientTempC) {
    // Speed of sound in dry air: v = 331.3 + 0.606 * T (m/s) -> cm/us: v / 10000.0
    float speedCmUs = (331.3f + 0.606f * ambientTempC) * 0.0001f;

    // 1. Measure Outer Sensor
    float distOuter = measureSingle(_trigOuter, _echoOuter, speedCmUs);
    filterChannel(
        distOuter, _readingOuter,
        _historyOuter, _histIdxOuter, _histCountOuter,
        _emaOuter, _jitterOuter, _spikeCountOuter
    );

    // Stagger to prevent echo interference between closely mounted sensors
    delay(SENSOR_STAGGER_INTERVAL_MS);

    // 2. Measure Inner Sensor
    float distInner = measureSingle(_trigInner, _echoInner, speedCmUs);
    filterChannel(
        distInner, _readingInner,
        _historyInner, _histIdxInner, _histCountInner,
        _emaInner, _jitterInner, _spikeCountInner
    );
}


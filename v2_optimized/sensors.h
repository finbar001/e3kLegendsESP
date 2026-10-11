#pragma once
#include <Arduino.h>
#include "config.h"

struct SensorReading {
    float rawDistanceCm;      // Latest single raw reading
    float filteredDistanceCm; // Filtered distance (median + spike-gate + EMA)
    float emaDistanceCm;      // Running EMA distance
    float jitterCm;           // Running jitter metric (stddev proxy in cm)
    bool  isValid;            // True if echo received within valid bounds
    uint8_t consecutiveValid;  // Number of consecutive valid frames
    uint8_t consecutiveInvalid;// Number of consecutive missed frames
    uint32_t lastEchoMicros;  // Duration of echo pulse
    uint32_t timestampMs;     // Millis when measured
};

class UltrasonicModule {
public:
    UltrasonicModule();
    void begin(uint8_t trigOuter, uint8_t echoOuter, uint8_t trigInner, uint8_t echoInner);
    
    // Perform sequential measurements of both sensors (staggered to avoid mutual echo cross-talk)
    void update(float ambientTempC = 20.0f);
    
    const SensorReading& getOuterReading() const { return _readingOuter; }
    const SensorReading& getInnerReading() const { return _readingInner; }

    void setEmaAlpha(float alpha) { _emaAlpha = constrain(alpha, 0.05f, 0.95f); }
    float getEmaAlpha() const { return _emaAlpha; }

    void setSpikeGateCm(float cm) { _spikeGateCm = constrain(cm, 5.0f, 100.0f); }
    float getSpikeGateCm() const { return _spikeGateCm; }

    void resetFilter();

private:
    uint8_t _trigOuter, _echoOuter;
    uint8_t _trigInner, _echoInner;
    
    SensorReading _readingOuter;
    SensorReading _readingInner;
    
    // Circular buffer for 5-sample median filter
    float _historyOuter[SENSOR_MEDIAN_BUFFER_SIZE];
    float _historyInner[SENSOR_MEDIAN_BUFFER_SIZE];
    uint8_t _histIdxOuter;
    uint8_t _histIdxInner;
    uint8_t _histCountOuter;
    uint8_t _histCountInner;

    float _emaOuter;
    float _emaInner;
    float _jitterOuter;
    float _jitterInner;
    uint8_t _spikeCountOuter;
    uint8_t _spikeCountInner;
    float _emaAlpha;
    float _spikeGateCm;
    
    float measureSingle(uint8_t trigPin, uint8_t echoPin, float speedOfSoundCmUs);
    float computeMedian(const float* buf, uint8_t count);
    void filterChannel(
        float rawDist,
        SensorReading& reading,
        float* histBuf,
        uint8_t& histIdx,
        uint8_t& histCount,
        float& emaVal,
        float& jitterVal,
        uint8_t& spikeCount
    );
};


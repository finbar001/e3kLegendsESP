#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "config.h"
#include "sensors.h"
#include "tracking.h"
#include "calibration.h"

// Packed binary packet for high-speed ESP-NOW transmission
typedef struct __attribute__((packed)) {
    uint8_t  nodeId;          // 2 = Secondary (Module Right)
    uint16_t distInner_mm;    // Right Inner distance (RI) in mm
    uint16_t distOuter_mm;    // Right Outer distance (RO) in mm
    uint8_t  validFlags;      // bit 0 = RI valid, bit 1 = RO valid
    uint16_t jitterInner_mm;  // Right Inner jitter in mm
    uint16_t jitterOuter_mm;  // Right Outer jitter in mm
    uint32_t timestampMs;     // Timestamp
} EspNowSensorPacket;

class CommsManager {
public:
    CommsManager();

    // Primary (Left Hub) initialization
    void beginPrimary(CalibrationManager* calibMgr, PositionTracker* tracker, UltrasonicModule* sensors);

    // Secondary (Right Node) initialization
    void beginSecondary();

    // Loop processing
    void updatePrimary();
    void updateSecondary();

    // Send readings from Secondary to Primary via ESP-NOW
    void sendSecondaryPacket(
        float distInnerCm, bool validInner,
        float distOuterCm, bool validOuter,
        float jitterInnerCm = 0.0f, float jitterOuterCm = 0.0f
    );

    // Broadcast tracked position and sensor telemetry over WebSocket
    void broadcastPosition(
        const TrackedPosition& pos,
        float distLO, float distLI,
        float distRI, float distRO,
        float jitterLO, float jitterLI,
        bool secOnline
    );

    // Secondary data access on Primary
    float getSecondaryInnerDist() const { return _secDistInnerCm; }
    float getSecondaryOuterDist() const { return _secDistOuterCm; }
    float getSecondaryInnerJitter() const { return _secJitterInnerCm; }
    float getSecondaryOuterJitter() const { return _secJitterOuterCm; }
    bool  isSecondaryInnerValid() const { return _secValidInner; }
    bool  isSecondaryOuterValid() const { return _secValidOuter; }
    bool  isSecondaryOnline() const;

private:
    bool _isPrimary;
    CalibrationManager* _calibMgr;
    PositionTracker* _tracker;
    UltrasonicModule* _sensors;

    WebServer _httpServer;
    WebSocketsServer _wsServer;

    volatile float _secDistInnerCm;
    volatile float _secDistOuterCm;
    volatile float _secJitterInnerCm;
    volatile float _secJitterOuterCm;
    volatile bool  _secValidInner;
    volatile bool  _secValidOuter;
    volatile uint32_t _lastSecPacketMs;

    static CommsManager* _instance;
    static void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len);
    void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
    void handleHttpRoot();
};


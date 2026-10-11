#include "comms.h"
#include "webui.h"

CommsManager* CommsManager::_instance = nullptr;

CommsManager::CommsManager()
    : _isPrimary(true), _calibMgr(nullptr), _tracker(nullptr), _sensors(nullptr),
      _httpServer(HTTP_SERVER_PORT), _wsServer(WEBSOCKET_PORT),
      _secDistInnerCm(0.0f), _secDistOuterCm(0.0f),
      _secJitterInnerCm(0.0f), _secJitterOuterCm(0.0f),
      _secValidInner(false), _secValidOuter(false),
      _lastSecPacketMs(0) {
    _instance = this;
}

void CommsManager::onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (_instance == nullptr || data == nullptr || len < (int)sizeof(EspNowSensorPacket)) {
        return;
    }

    const EspNowSensorPacket* pkt = reinterpret_cast<const EspNowSensorPacket*>(data);
    if (pkt->nodeId == 2) {
        _instance->_secDistInnerCm   = (float)pkt->distInner_mm * 0.1f;
        _instance->_secDistOuterCm   = (float)pkt->distOuter_mm * 0.1f;
        _instance->_secJitterInnerCm = (float)pkt->jitterInner_mm * 0.1f;
        _instance->_secJitterOuterCm = (float)pkt->jitterOuter_mm * 0.1f;
        _instance->_secValidInner    = (pkt->validFlags & (1 << 0)) != 0;
        _instance->_secValidOuter    = (pkt->validFlags & (1 << 1)) != 0;
        _instance->_lastSecPacketMs  = millis();
    }
}

void CommsManager::handleHttpRoot() {
    _httpServer.send_P(200, "text/html", INDEX_HTML);
}

void CommsManager::handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    if (type == WStype_CONNECTED) {
        // Send initial system state sync to newly connected WebUI or game client
        if (_calibMgr) {
            const SystemCalibration& cal = _calibMgr->getCalibration();
            char syncBuf[512];
            snprintf(
                syncBuf, sizeof(syncBuf),
                "{\"type\":\"sync\",\"w\":%.1f,\"h\":%.1f,\"dz\":%.1f,\"alo\":%.1f,\"ali\":%.1f,\"ari\":%.1f,\"aro\":%.1f,\"prof\":\"%s\",\"q\":%.2f,\"r\":%.2f,\"alpha\":%.2f,\"sg\":%.1f,\"sh\":%.2f,\"sb\":%.2f,\"sgb\":%.2f,\"drate\":%.3f,\"dmax\":%.1f}",
                cal.areaWidthCm, cal.areaDepthCm, cal.deadZoneCm,
                cal.angleLO_deg, cal.angleLI_deg, cal.angleRI_deg, cal.angleRO_deg,
                _tracker ? _tracker->getProfileName() : "balanced",
                cal.kalmanQ, cal.kalmanR, cal.emaAlpha, cal.spikeGateCm,
                cal.snapHole, cal.snapButton, cal.snapGlobal,
                cal.driftRate, cal.driftMaxDeg
            );
            _wsServer.sendTXT(num, syncBuf);
        }
        return;
    }

    if (type == WStype_TEXT && payload != nullptr && length > 0) {
        String msg = String((char*)payload);
        if (!_calibMgr) return;
        SystemCalibration& cal = _calibMgr->getCalibration();

        if (msg.indexOf("\"profile\"") >= 0) {
            // Profile switch command: {"cmd":"profile","name":"responsive"|"balanced"|"stable"|"custom"}
            TrackingProfile prof = PROFILE_BALANCED;
            if (msg.indexOf("\"responsive\"") >= 0) {
                prof = PROFILE_RESPONSIVE;
            } else if (msg.indexOf("\"stable\"") >= 0) {
                prof = PROFILE_STABLE;
            } else if (msg.indexOf("\"custom\"") >= 0) {
                prof = PROFILE_CUSTOM;
            } else {
                prof = PROFILE_BALANCED;
            }

            if (_tracker) {
                _tracker->setProfile(prof);
            }
            cal.profile = (uint8_t)prof;

            // Synchronize filter parameters across sensors and calibration
            if (prof == PROFILE_RESPONSIVE) {
                cal.emaAlpha    = PROFILE_RESPONSIVE_EMA_ALPHA;
                cal.kalmanQ     = PROFILE_RESPONSIVE_KALMAN_Q;
                cal.kalmanR     = PROFILE_RESPONSIVE_KALMAN_R;
                cal.spikeGateCm = PROFILE_RESPONSIVE_SPIKE_CM;
            } else if (prof == PROFILE_STABLE) {
                cal.emaAlpha    = PROFILE_STABLE_EMA_ALPHA;
                cal.kalmanQ     = PROFILE_STABLE_KALMAN_Q;
                cal.kalmanR     = PROFILE_STABLE_KALMAN_R;
                cal.spikeGateCm = PROFILE_STABLE_SPIKE_CM;
            } else if (prof == PROFILE_BALANCED) {
                cal.emaAlpha    = PROFILE_BALANCED_EMA_ALPHA;
                cal.kalmanQ     = PROFILE_BALANCED_KALMAN_Q;
                cal.kalmanR     = PROFILE_BALANCED_KALMAN_R;
                cal.spikeGateCm = PROFILE_BALANCED_SPIKE_CM;
            }

            if (_sensors) {
                _sensors->setEmaAlpha(cal.emaAlpha);
                _sensors->setSpikeGateCm(cal.spikeGateCm);
            }

            Serial.printf("[COMMS] Profile switched to: %s (Alpha=%.2f, Q=%.2f, R=%.2f, Spike=%.1fcm)\n",
                _tracker ? _tracker->getProfileName() : "balanced", cal.emaAlpha, cal.kalmanQ, cal.kalmanR, cal.spikeGateCm);

        } else if (msg.indexOf("\"config\"") >= 0) {
            // Parse geometry & tuning adjustments
            int wIdx = msg.indexOf("\"w\":");
            if (wIdx > 0) {
                float w = msg.substring(wIdx + 4).toFloat();
                if (w >= 30.0f && w <= 300.0f) {
                    cal.areaWidthCm = w;
                    cal.sensorRI_x = w - DEFAULT_MODULE_SENSOR_SEP_CM;
                    cal.sensorRO_x = w;
                }
            }
            int hIdx = msg.indexOf("\"h\":");
            if (hIdx > 0) {
                float h = msg.substring(hIdx + 4).toFloat();
                if (h >= 40.0f && h <= 300.0f) cal.areaDepthCm = h;
            }
            int dzIdx = msg.indexOf("\"dz\":");
            if (dzIdx > 0) {
                float dz = msg.substring(dzIdx + 5).toFloat();
                if (dz >= 10.0f && dz <= 120.0f) cal.deadZoneCm = dz;
            }
            int aloIdx = msg.indexOf("\"alo\":");
            if (aloIdx > 0) cal.angleLO_deg = msg.substring(aloIdx + 6).toFloat();
            int aliIdx = msg.indexOf("\"ali\":");
            if (aliIdx > 0) cal.angleLI_deg = msg.substring(aliIdx + 6).toFloat();
            int ariIdx = msg.indexOf("\"ari\":");
            if (ariIdx > 0) cal.angleRI_deg = msg.substring(ariIdx + 6).toFloat();
            int aroIdx = msg.indexOf("\"aro\":");
            if (aroIdx > 0) cal.angleRO_deg = msg.substring(aroIdx + 6).toFloat();

            int qIdx = msg.indexOf("\"q\":");
            if (qIdx > 0) {
                cal.kalmanQ = msg.substring(qIdx + 4).toFloat();
                cal.profile = (uint8_t)PROFILE_CUSTOM;
                if (_tracker) _tracker->setProfile(PROFILE_CUSTOM);
            }
            int rIdx = msg.indexOf("\"r\":");
            if (rIdx > 0) {
                cal.kalmanR = msg.substring(rIdx + 4).toFloat();
                cal.profile = (uint8_t)PROFILE_CUSTOM;
                if (_tracker) _tracker->setProfile(PROFILE_CUSTOM);
            }
            int alphaIdx = msg.indexOf("\"alpha\":");
            if (alphaIdx > 0) {
                cal.emaAlpha = msg.substring(alphaIdx + 8).toFloat();
                if (_sensors) _sensors->setEmaAlpha(cal.emaAlpha);
                cal.profile = (uint8_t)PROFILE_CUSTOM;
                if (_tracker) _tracker->setProfile(PROFILE_CUSTOM);
            }
            int sgIdx = msg.indexOf("\"sg\":");
            if (sgIdx > 0) {
                cal.spikeGateCm = msg.substring(sgIdx + 5).toFloat();
                if (_sensors) _sensors->setSpikeGateCm(cal.spikeGateCm);
                cal.profile = (uint8_t)PROFILE_CUSTOM;
                if (_tracker) _tracker->setProfile(PROFILE_CUSTOM);
            }

            int shIdx = msg.indexOf("\"sh\":");
            if (shIdx > 0) cal.snapHole = msg.substring(shIdx + 5).toFloat();
            int sbIdx = msg.indexOf("\"sb\":");
            if (sbIdx > 0) cal.snapButton = msg.substring(sbIdx + 5).toFloat();
            int sgbIdx = msg.indexOf("\"sgb\":");
            if (sgbIdx > 0) cal.snapGlobal = msg.substring(sgbIdx + 6).toFloat();

            int drateIdx = msg.indexOf("\"drate\":");
            if (drateIdx > 0) cal.driftRate = msg.substring(drateIdx + 8).toFloat();
            int dmaxIdx = msg.indexOf("\"dmax\":");
            if (dmaxIdx > 0) cal.driftMaxDeg = msg.substring(dmaxIdx + 7).toFloat();

            if (_tracker) {
                _tracker->updateCalibration(cal);
                _tracker->setSnapConfig(cal.snapHole, cal.snapButton, cal.snapGlobal);
            }
            Serial.printf("[COMMS] Config updated: Width=%.1f Depth=%.1f DZ=%.1f Angles=(%.1f, %.1f, %.1f, %.1f) Q=%.2f R=%.2f Alpha=%.2f DriftRate=%.3f DriftMax=%.1f\n",
                cal.areaWidthCm, cal.areaDepthCm, cal.deadZoneCm, cal.angleLO_deg, cal.angleLI_deg, cal.angleRI_deg, cal.angleRO_deg, cal.kalmanQ, cal.kalmanR, cal.emaAlpha, cal.driftRate, cal.driftMaxDeg);

        } else if (msg.indexOf("\"quickAlign\"") >= 0) {
            // Start multi-sample quick align
            float tx = cal.areaWidthCm * 0.5f;
            float ty = cal.deadZoneCm + (cal.areaDepthCm * 0.5f);

            int xIdx = msg.indexOf("\"x\":");
            int yIdx = msg.indexOf("\"y\":");
            if (xIdx > 0) tx = msg.substring(xIdx + 4).toFloat();
            if (yIdx > 0) ty = msg.substring(yIdx + 4).toFloat();

            if (msg.indexOf("\"pos\":\"front_center\"") >= 0) {
                tx = cal.areaWidthCm * 0.5f;
                ty = cal.deadZoneCm + 30.0f;
            } else if (msg.indexOf("\"pos\":\"back_center\"") >= 0) {
                tx = cal.areaWidthCm * 0.5f;
                ty = cal.deadZoneCm + cal.areaDepthCm * 0.85f;
            }

            _calibMgr->startQuickAlign(tx, ty);
            Serial.printf("[COMMS] Quick-align requested at target (%.1f, %.1f) cm\n", tx, ty);

        } else if (msg.indexOf("\"quickAlignNow\"") >= 0) {
            // Single-shot quick align immediately using current live distances
            float tx = cal.areaWidthCm * 0.5f;
            float ty = cal.deadZoneCm + (cal.areaDepthCm * 0.5f);

            int xIdx = msg.indexOf("\"x\":");
            int yIdx = msg.indexOf("\"y\":");
            if (xIdx > 0) tx = msg.substring(xIdx + 4).toFloat();
            if (yIdx > 0) ty = msg.substring(yIdx + 4).toFloat();

            float dLO = _sensors ? _sensors->getOuterReading().filteredDistanceCm : 0.0f;
            float dLI = _sensors ? _sensors->getInnerReading().filteredDistanceCm : 0.0f;
            float dRI = _secDistInnerCm;
            float dRO = _secDistOuterCm;

            bool ok = _calibMgr->quickAlign(dLO, dLI, dRI, dRO, tx, ty);
            if (ok && _tracker) {
                _tracker->updateCalibration(_calibMgr->getCalibration());
                _tracker->resetAlignmentHealth();
            }

            char reply[128];
            snprintf(reply, sizeof(reply), "{\"type\":\"qa_done\",\"ok\":%s,\"alo\":%.1f,\"ali\":%.1f,\"ari\":%.1f,\"aro\":%.1f}",
                     ok ? "true" : "false", cal.angleLO_deg, cal.angleLI_deg, cal.angleRI_deg, cal.angleRO_deg);
            _wsServer.broadcastTXT(reply);

        } else if (msg.indexOf("\"estimateWidth\"") >= 0) {
            float dLI = _sensors ? _sensors->getInnerReading().filteredDistanceCm : 0.0f;
            float dRI = _secDistInnerCm;
            float estW = _calibMgr->estimateWidth(dLI, dRI);

            char reply[64];
            snprintf(reply, sizeof(reply), "{\"type\":\"est_w\",\"w\":%.1f}", estW);
            _wsServer.broadcastTXT(reply);

        } else if (msg.indexOf("\"setWidth\"") >= 0) {
            int wIdx = msg.indexOf("\"w\":");
            if (wIdx > 0) {
                float w = msg.substring(wIdx + 4).toFloat();
                _calibMgr->setModuleWidth(w);
                if (_tracker) _tracker->updateCalibration(_calibMgr->getCalibration());
            }

        } else if (msg.indexOf("\"snap\"") >= 0) {
            // Live snap configuration update
            int shIdx = msg.indexOf("\"sh\":");
            int sbIdx = msg.indexOf("\"sb\":");
            int sgbIdx = msg.indexOf("\"sgb\":");
            if (shIdx > 0) cal.snapHole = msg.substring(shIdx + 5).toFloat();
            if (sbIdx > 0) cal.snapButton = msg.substring(sbIdx + 5).toFloat();
            if (sgbIdx > 0) cal.snapGlobal = msg.substring(sgbIdx + 6).toFloat();

            if (_tracker) _tracker->setSnapConfig(cal.snapHole, cal.snapButton, cal.snapGlobal);
            Serial.printf("[COMMS] Snap updated: Hole=%.2f Button=%.2f Global=%.2f\n",
                cal.snapHole, cal.snapButton, cal.snapGlobal);

        } else if (msg.indexOf("\"targets\"") >= 0) {
            // Target injection: {"cmd":"targets","pts":[{"x":0.25,"y":0.3,"r":0.08,"t":0,"s":0.2},...],"snap":1.0}
            if (_tracker) {
                TargetPoint targets[TARGET_MAX_COUNT];
                uint8_t count = 0;

                int searchPos = msg.indexOf("\"pts\":[");
                if (searchPos > 0) {
                    int endPts = msg.indexOf(']', searchPos);
                    int curPos = searchPos + 7;
                    while (curPos < endPts && count < TARGET_MAX_COUNT) {
                        int objStart = msg.indexOf('{', curPos);
                        if (objStart < 0 || objStart > endPts) break;
                        int objEnd = msg.indexOf('}', objStart);
                        if (objEnd < 0 || objEnd > endPts) break;

                        String objStr = msg.substring(objStart, objEnd + 1);
                        int xIdx = objStr.indexOf("\"x\":");
                        int yIdx = objStr.indexOf("\"y\":");
                        int rIdx = objStr.indexOf("\"r\":");
                        int tIdx = objStr.indexOf("\"t\":");
                        int sIdx = objStr.indexOf("\"s\":");

                        if (xIdx >= 0 && yIdx >= 0) {
                            float nx = objStr.substring(xIdx + 4).toFloat();
                            float ny = objStr.substring(yIdx + 4).toFloat();
                            float nr = (rIdx >= 0) ? objStr.substring(rIdx + 4).toFloat() : 0.08f;
                            uint8_t t = (tIdx >= 0) ? (uint8_t)objStr.substring(tIdx + 4).toInt() : (uint8_t)TARGET_TYPE_HOLE;
                            float s = (sIdx >= 0) ? objStr.substring(sIdx + 4).toFloat() : -1.0f;

                            // Map normalized (0-1) to physical coordinates (cm)
                            targets[count].xCm = constrain(nx * cal.areaWidthCm, 0.0f, cal.areaWidthCm);
                            targets[count].yCm = constrain(cal.deadZoneCm + (ny * cal.areaDepthCm), cal.deadZoneCm, cal.deadZoneCm + cal.areaDepthCm);
                            targets[count].radiusCm = constrain(nr * ((cal.areaWidthCm + cal.areaDepthCm) * 0.5f), 4.0f, 35.0f);
                            targets[count].type = (TargetType)t;
                            targets[count].snapOverride = s;
                            targets[count].active = true;
                            count++;
                        }
                        curPos = objEnd + 1;
                    }
                }
                _tracker->setTargets(targets, count);

                int snapIdx = msg.indexOf("\"snap\":");
                if (snapIdx > 0) {
                    float sgb = msg.substring(snapIdx + 7).toFloat();
                    cal.snapGlobal = sgb;
                    _tracker->setSnapConfig(cal.snapHole, cal.snapButton, cal.snapGlobal);
                }

                Serial.printf("[COMMS] Injected %d targets (HoleSnap=%.2f, ButtonSnap=%.2f, Global=%.2f)\n",
                    count, cal.snapHole, cal.snapButton, cal.snapGlobal);
            }

        } else if (msg.indexOf("\"clearTargets\"") >= 0) {
            if (_tracker) _tracker->clearTargets();
            Serial.println("[COMMS] Cleared all game targets.");

        } else if (msg.indexOf("\"smoothness\"") >= 0) {
            int aIdx = msg.indexOf("\"alpha\":");
            if (aIdx > 0 && _sensors) {
                float a = msg.substring(aIdx + 8).toFloat();
                _sensors->setEmaAlpha(a);
                cal.emaAlpha = a;
                cal.profile = (uint8_t)PROFILE_CUSTOM;
                if (_tracker) _tracker->setProfile(PROFILE_CUSTOM);
                Serial.printf("[COMMS] Sensor EMA alpha updated: %.2f\n", _sensors->getEmaAlpha());
            }

        } else if (msg.indexOf("\"save\"") >= 0) {
            _calibMgr->saveToFlash();
            Serial.println("[COMMS] Saved current calibration and profile to Flash.");

        } else if (msg.indexOf("\"reset\"") >= 0) {
            _calibMgr->resetToDefaults();
            if (_tracker) {
                _tracker->updateCalibration(_calibMgr->getCalibration());
                _tracker->setProfile(PROFILE_BALANCED);
                _tracker->setSnapConfig(DEFAULT_SNAP_HOLE, DEFAULT_SNAP_BUTTON, DEFAULT_SNAP_GLOBAL);
                _tracker->resetFilter();
            }
            if (_sensors) {
                _sensors->setEmaAlpha(PROFILE_BALANCED_EMA_ALPHA);
                _sensors->setSpikeGateCm(PROFILE_BALANCED_SPIKE_CM);
                _sensors->resetFilter();
            }
            Serial.println("[COMMS] Reset calibration, filters, profile, and geometry to defaults.");
        }
    }
}

void CommsManager::beginPrimary(CalibrationManager* calibMgr, PositionTracker* tracker, UltrasonicModule* sensors) {
    _isPrimary = true;
    _calibMgr = calibMgr;
    _tracker = tracker;
    _sensors = sensors;

    // Set WiFi to AP + STA mode on fixed channel
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false); // Low latency
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);

    Serial.println("=================================================");
    Serial.printf("[COMMS] Primary Hub AP Started: %s\n", WIFI_AP_SSID);
    Serial.printf("[COMMS] IP Address: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.println("=================================================");

    // Initialize ESP-NOW
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(onEspNowRecv);
        Serial.println("[COMMS] ESP-NOW initialized on Primary Hub.");
    } else {
        Serial.println("[COMMS] ESP-NOW init failed on Primary!");
    }

    // Initialize HTTP WebServer
    _httpServer.on("/", [this]() { handleHttpRoot(); });
    _httpServer.begin();

    // Initialize WebSocketsServer
    _wsServer.begin();
    _wsServer.onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        this->handleWebSocketEvent(num, type, payload, length);
    });

    Serial.println("[COMMS] HTTP and WebSocket servers ready.");
}

void CommsManager::beginSecondary() {
    _isPrimary = false;

    // Set WiFi to Station mode on matching channel
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    
    // Disconnect from any AP and lock channel
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(WIFI_AP_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    Serial.println("=================================================");
    Serial.printf("[COMMS] Secondary Node Initialized on Channel %d\n", WIFI_AP_CHANNEL);
    Serial.println("=================================================");

    if (esp_now_init() == ESP_OK) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, ESPNOW_BROADCAST_MAC, 6);
        peerInfo.channel = WIFI_AP_CHANNEL;
        peerInfo.encrypt = false;
        
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            Serial.println("[COMMS] ESP-NOW Broadcast Peer registered on Secondary.");
        }
    } else {
        Serial.println("[COMMS] ESP-NOW init failed on Secondary!");
    }
}

void CommsManager::updatePrimary() {
    _httpServer.handleClient();
    _wsServer.loop();
}

void CommsManager::updateSecondary() {
    // Secondary only sends packets on demand
}

bool CommsManager::isSecondaryOnline() const {
    return (millis() - _lastSecPacketMs < ESPNOW_PACKET_TIMEOUT_MS);
}

void CommsManager::sendSecondaryPacket(
    float distInnerCm, bool validInner,
    float distOuterCm, bool validOuter,
    float jitterInnerCm, float jitterOuterCm
) {
    EspNowSensorPacket pkt;
    pkt.nodeId = 2;
    pkt.distInner_mm   = validInner ? (uint16_t)(distInnerCm * 10.0f) : 0;
    pkt.distOuter_mm   = validOuter ? (uint16_t)(distOuterCm * 10.0f) : 0;
    pkt.jitterInner_mm = (uint16_t)(jitterInnerCm * 10.0f);
    pkt.jitterOuter_mm = (uint16_t)(jitterOuterCm * 10.0f);
    pkt.validFlags = 0;
    if (validInner) pkt.validFlags |= (1 << 0);
    if (validOuter) pkt.validFlags |= (1 << 1);
    pkt.timestampMs = millis();

    esp_now_send(ESPNOW_BROADCAST_MAC, (uint8_t*)&pkt, sizeof(pkt));
}

void CommsManager::broadcastPosition(
    const TrackedPosition& pos,
    float distLO, float distLI,
    float distRI, float distRO,
    float jitterLO, float jitterLI,
    bool secOnline
) {
    if (_wsServer.connectedClients() == 0) {
        return;
    }

    const SensorAlignmentHealth* health = _tracker ? _tracker->getSensorHealth() : nullptr;
    float hLO = health ? health[0].avgResidualCm : 0.0f;
    float hLI = health ? health[1].avgResidualCm : 0.0f;
    float hRI = health ? health[2].avgResidualCm : 0.0f;
    float hRO = health ? health[3].avgResidualCm : 0.0f;

    bool qaActive = _calibMgr ? _calibMgr->isQuickAligning() : false;
    uint8_t qaProg = _calibMgr ? _calibMgr->getQuickAlignProgress() : 0;

    char buffer[640];
    snprintf(
        buffer, sizeof(buffer),
        "{\"type\":\"pos\",\"x\":%.1f,\"y\":%.1f,\"nx\":%.3f,\"ny\":%.3f,\"c\":%.2f,\"det\":%s,\"dz\":%s,\"lo\":%.1f,\"li\":%.1f,\"ri\":%.1f,\"ro\":%.1f,\"so\":%s,\"vx\":%.1f,\"vy\":%.1f,\"spd\":%.1f,\"rx\":%.1f,\"ry\":%.1f,\"jlo\":%.1f,\"jli\":%.1f,\"jri\":%.1f,\"jro\":%.1f,\"prof\":\"%s\",\"aq\":%.2f,\"ar\":%.2f,\"dz_th\":%.1f,\"hlth\":[%.1f,%.1f,%.1f,%.1f],\"qa\":{\"a\":%s,\"p\":%u}}",
        pos.xCm, pos.yCm,
        pos.normX, pos.normY,
        pos.confidence,
        pos.isDetected ? "true" : "false",
        pos.inDeadZone ? "true" : "false",
        distLO, distLI, distRI, distRO,
        secOnline ? "true" : "false",
        pos.vxCmS, pos.vyCmS,
        pos.speedCmS,
        pos.rawXCm, pos.rawYCm,
        jitterLO, jitterLI,
        _secJitterInnerCm, _secJitterOuterCm,
        _tracker ? _tracker->getProfileName() : "balanced",
        _tracker ? _tracker->getAdaptiveQ() : 0.08f,
        _tracker ? _tracker->getAdaptiveR() : 0.35f,
        _calibMgr ? _calibMgr->getCalibration().deadZoneCm : DEFAULT_DEAD_ZONE_CM,
        hLO, hLI, hRI, hRO,
        qaActive ? "true" : "false",
        qaProg
    );

    _wsServer.broadcastTXT(buffer);
}


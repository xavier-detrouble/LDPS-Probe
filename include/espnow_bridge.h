#pragma once
// ============================================================
//  ESP-NOW Bridge — forward EN: commands to/from Node
//  Same protocol as Dongle, compatible with LDPS Factory app
// ============================================================

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "config.h"

class ESPNowBridge {
public:
    bool begin();
    bool forward(const char* mac, const char* payload);

private:
    static void _recvCB(const esp_now_recv_info_t* info, const uint8_t* data, int len);
    bool _ensurePeer(const uint8_t* mac);
    static bool _parseMac(const char* str, uint8_t* out);
    static String _macToStr(const uint8_t* mac);

    static ESPNowBridge* _instance;
    bool _initialized = false;
};

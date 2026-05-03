// ============================================================
//  ESP-NOW Bridge — forward commands between UART and Node
// ============================================================

#include "espnow_bridge.h"

ESPNowBridge* ESPNowBridge::_instance = nullptr;

bool ESPNowBridge::begin() {
    _instance = this;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(WIFI_CH_DEFAULT, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNow] init failed");
        return false;
    }

    esp_now_register_recv_cb(_recvCB);
    _initialized = true;

    Serial.printf("[ESPNow] Ready, MAC=%s\n", WiFi.macAddress().c_str());
    return true;
}

bool ESPNowBridge::forward(const char* macStr, const char* payload) {
    if (!_initialized) return false;

    uint8_t mac[6];
    // Handle broadcast
    if (strcmp(macStr, "FF") == 0 || strcmp(macStr, "ff") == 0) {
        memset(mac, 0xFF, 6);
    } else if (!_parseMac(macStr, mac)) {
        return false;
    }

    _ensurePeer(mac);

    size_t len = strlen(payload);
    if (len > ESPNOW_MAX_DATA_LEN) len = ESPNOW_MAX_DATA_LEN;

    for (int attempt = 0; attempt < 3; attempt++) {
        if (esp_now_send(mac, (const uint8_t*)payload, len) == ESP_OK) {
            return true;
        }
        if (attempt < 2) delay(50);
    }
    return false;
}

void ESPNowBridge::_recvCB(const esp_now_recv_info_t* info,
                            const uint8_t* data, int len) {
    if (!_instance || !info || len <= 0) return;

    String mac = _macToStr(info->src_addr);
    char buf[256];
    size_t copyLen = (len < (int)sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, data, copyLen);
    buf[copyLen] = '\0';

    // Send upstream: en:{mac},{payload}
    Serial.printf("en:%s,%s\n", mac.c_str(), buf);
}

bool ESPNowBridge::_ensurePeer(const uint8_t* mac) {
    if (esp_now_is_peer_exist(mac)) return true;
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    return esp_now_add_peer(&peerInfo) == ESP_OK;
}

bool ESPNowBridge::_parseMac(const char* str, uint8_t* out) {
    // Parse "AA:BB:CC:DD:EE:FF" → 6 bytes
    if (strlen(str) != 17) return false;
    return sscanf(str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                  &out[0], &out[1], &out[2], &out[3], &out[4], &out[5]) == 6;
}

String ESPNowBridge::_macToStr(const uint8_t* mac) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
}

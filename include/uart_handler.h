#pragma once
// ============================================================
//  UART Handler — USB CDC command parser
//  Routes: EN: → ESP-NOW, SX: → SX1262 TX, TB: → Test Board
// ============================================================

#include <Arduino.h>

class ESPNowBridge;
class SX1262TX;
class WS2812Analyzer;
class I2CSniffer;

class UartHandler {
public:
    void begin(ESPNowBridge* espnow, SX1262TX* sx, WS2812Analyzer* analyzer, I2CSniffer* i2c = nullptr);
    void poll();

private:
    void _dispatch(const char* line);
    void _handleEN(const char* payload);
    void _handleSX(const char* payload);
    void _handleTB(const char* payload);
    void _handleDG(const char* payload);

    ESPNowBridge* _espnow = nullptr;
    SX1262TX* _sx = nullptr;
    WS2812Analyzer* _analyzer = nullptr;
    I2CSniffer* _i2c = nullptr;

    char _buf[512];
    size_t _bufLen = 0;
};

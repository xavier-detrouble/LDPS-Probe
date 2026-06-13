#pragma once
// ============================================================
//  SX1262 TX — Send 14-byte v2 playback frames to Node (ADR-016)
//  Same binary protocol as Hub (protocol.py)
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "config.h"

class SX1262TX {
public:
    bool begin();
    void reset();  // Hardware reset + reinitialize SX1262
    void loop();   // Send periodic frame if playing

    // Commands (from UART SX: namespace)
    void play(uint8_t seqIndex, uint16_t packId, uint8_t brightness = 100);
    void stop();
    void seek(uint32_t progressMs);
    void setBrightness(uint8_t b);

    bool isPlaying() const { return _playing; }

private:
    void _buildFrame(uint8_t* buf);
    void _sendFrame();
    static uint8_t _crc8maxim(const uint8_t* data, size_t len);

    SPIClass _spi{FSPI};
    SX1262* _radio = nullptr;
    bool _initialized = false;

    bool     _playing = false;
    uint8_t  _seqIndex = 0;
    uint16_t _packId = 0;
    uint8_t  _brightness = 100;
    uint32_t _progressMs = 0;
    uint32_t _playStartMs = 0;
    uint8_t  _seqCounter = 0;
    uint32_t _lastSendMs = 0;
};

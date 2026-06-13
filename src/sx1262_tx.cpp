// ============================================================
//  SX1262 TX — 14-byte v2 binary playback protocol (ADR-016)
//  Same as Hub's protocol.py, implemented in C++
// ============================================================

#include "sx1262_tx.h"

bool SX1262TX::begin() {
    // Hard reset SX1262 via RST pin — ensures clean state after firmware reflash
    pinMode(SX_RST, OUTPUT);
    digitalWrite(SX_RST, LOW);
    delay(10);
    digitalWrite(SX_RST, HIGH);
    delay(10);

    _spi.begin(SX_SCK, SX_MISO, SX_MOSI, SX_NSS);
    Module* mod = new Module(SX_NSS, SX_DIO1, SX_RST, SX_BUSY, _spi);
    _radio = new SX1262(mod);

    // Wait for BUSY to go LOW after reset (with timeout tracking)
    pinMode(SX_BUSY, INPUT);
    unsigned long t0 = millis();
    bool busyTimeout = false;
    while (digitalRead(SX_BUSY) == HIGH) {
        if (millis() - t0 > 100) { busyTimeout = true; break; }
        delay(1);
    }
    Serial.printf("[SX1262] After reset: BUSY=%d (%s, %lums)\n",
                  digitalRead(SX_BUSY), busyTimeout ? "TIMEOUT" : "OK", millis() - t0);

    // Raw SPI probe: read register 0x0320 (chip identification)
    {
        pinMode(SX_NSS, OUTPUT);
        digitalWrite(SX_NSS, HIGH);
        _spi.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
        digitalWrite(SX_NSS, LOW);
        _spi.transfer(0x1D);      // ReadRegister command
        _spi.transfer(0x03);      // addr MSB = 0x03
        _spi.transfer(0x20);      // addr LSB = 0x20
        _spi.transfer(0x00);      // NOP (status)
        uint8_t b0 = _spi.transfer(0x00);
        uint8_t b1 = _spi.transfer(0x00);
        uint8_t b2 = _spi.transfer(0x00);
        uint8_t b3 = _spi.transfer(0x00);
        digitalWrite(SX_NSS, HIGH);
        _spi.endTransaction();
        Serial.printf("[SX1262] SPI probe reg 0x0320: %02X %02X %02X %02X '%c%c%c%c'\n",
                      b0, b1, b2, b3,
                      (b0 >= 0x20 && b0 < 0x7F) ? b0 : '.',
                      (b1 >= 0x20 && b1 < 0x7F) ? b1 : '.',
                      (b2 >= 0x20 && b2 < 0x7F) ? b2 : '.',
                      (b3 >= 0x20 && b3 < 0x7F) ? b3 : '.');
        if (b0 == 0x00 && b1 == 0x00 && b2 == 0x00 && b3 == 0x00) {
            Serial.println("[SX1262] SPI probe: all zeros — chip not responding");
        } else if (b0 == 0xFF && b1 == 0xFF && b2 == 0xFF && b3 == 0xFF) {
            Serial.println("[SX1262] SPI probe: all 0xFF — MISO floating");
        }
    }

    // tcxoVoltage = 0.0 → module uses XTAL, not TCXO
    int state = _radio->begin(RF_FREQ, RF_BW, RF_SF, RF_CR, RF_SYNC, RF_POWER, RF_PREAMBLE, 0.0);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[SX1262] Init failed: %d (CHIP_NOT_FOUND=-2, SPI_CMD_TIMEOUT=-4)\n", state);
        return false;
    }

    // DIO2 as RF switch control (required for DX-LR30 module)
    _radio->setDio2AsRfSwitch(true);

    _initialized = true;
    Serial.printf("[SX1262] TX ready (BUSY=%d)\n", digitalRead(SX_BUSY));
    return true;
}

void SX1262TX::reset() {
    _initialized = false;
    _playing = false;
    pinMode(SX_RST, OUTPUT);
    digitalWrite(SX_RST, LOW);
    delay(10);
    digitalWrite(SX_RST, HIGH);
    delay(10);
    pinMode(SX_BUSY, INPUT);
    unsigned long t0 = millis();
    while (digitalRead(SX_BUSY) == HIGH && millis() - t0 < 100) delay(1);

    int state = _radio->begin(RF_FREQ, RF_BW, RF_SF, RF_CR, RF_SYNC, RF_POWER, RF_PREAMBLE, 0.0);
    if (state == RADIOLIB_ERR_NONE) {
        _radio->setDio2AsRfSwitch(true);
    }
    _initialized = (state == RADIOLIB_ERR_NONE);
    Serial.printf("[SX1262] Reset: %s\n", _initialized ? "OK" : "FAILED");
}

void SX1262TX::loop() {
    if (!_playing || !_initialized) return;

    uint32_t now = millis();
    if (now - _lastSendMs < PLAYBACK_INTERVAL_MS) return;
    _lastSendMs = now;

    // Advance progress
    _progressMs = now - _playStartMs;

    _sendFrame();
}

void SX1262TX::play(uint8_t seqIndex, uint16_t packId, uint8_t brightness) {
    _seqIndex = seqIndex;
    _packId = packId;
    _brightness = brightness;
    _playing = true;
    _playStartMs = millis();
    _progressMs = 0;
    _seqCounter++;

    // Send burst (3 frames, 10ms apart) with HARD sync
    for (int i = 0; i < 3; i++) {
        if (i > 0) delay(10);
        _sendFrame();
    }
    _lastSendMs = millis();
}

void SX1262TX::stop() {
    _playing = false;
    // Send stop burst
    for (int i = 0; i < 3; i++) {
        if (i > 0) delay(10);
        _sendFrame();
    }
}

void SX1262TX::seek(uint32_t progressMs) {
    _progressMs = progressMs;
    _playStartMs = millis() - progressMs;
    _seqCounter++;
    for (int i = 0; i < 3; i++) {
        if (i > 0) delay(10);
        _sendFrame();
    }
}

void SX1262TX::setBrightness(uint8_t b) {
    _brightness = b > 100 ? 100 : b;
    // Send burst so Node applies immediately (not wait for periodic frame)
    if (_initialized) {
        for (int i = 0; i < 3; i++) {
            if (i > 0) delay(10);
            _sendFrame();
        }
    }
}

void SX1262TX::_buildFrame(uint8_t* buf) {
    buf[0] = PROTO_MAGIC;

    // flags: [7:6]=version(01=v2), [2]=operation, [1]=sync_mode, [0]=playing
    uint8_t flags = (uint8_t)(PROTO_VERSION << 6);
    if (_playing) flags |= 0x01;
    // HARD sync on play/seek burst, SOFT on periodic
    // For simplicity, always use HARD during burst, caller handles
    flags |= 0x02;  // sync_mode = HARD
    buf[1] = flags;

    buf[2] = _seqIndex;
    buf[3] = _seqCounter;

    // progress_ms (uint32 BE)
    buf[4] = (_progressMs >> 24) & 0xFF;
    buf[5] = (_progressMs >> 16) & 0xFF;
    buf[6] = (_progressMs >> 8) & 0xFF;
    buf[7] = _progressMs & 0xFF;

    // hub_ms (uint16 BE, modulo 65536)
    uint16_t hubMs = (uint16_t)(millis() & 0xFFFF);
    buf[8] = (hubMs >> 8) & 0xFF;
    buf[9] = hubMs & 0xFF;

    buf[10] = _brightness;
    // pack_id (uint16 BE) — v2: CRC-16/CCITT of pack_uuid, supplied by host.
    // The node filters frames on this; the test pack's pack_id must be passed.
    buf[11] = (uint8_t)((_packId >> 8) & 0xFF);
    buf[12] = (uint8_t)(_packId & 0xFF);
    buf[13] = _crc8maxim(buf, 13);
}

void SX1262TX::_sendFrame() {
    if (!_initialized) return;
    uint8_t frame[PROTO_FRAME_LEN];
    _buildFrame(frame);
    int state = _radio->transmit(frame, PROTO_FRAME_LEN);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[SX1262] TX error: %d\n", state);
    }
    // Small guard: let SX1262 settle before ESP-NOW can use WiFi
    delayMicroseconds(500);
}

uint8_t SX1262TX::_crc8maxim(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ PROTO_CRC8_POLY) : (crc << 1);
        }
    }
    return crc;
}

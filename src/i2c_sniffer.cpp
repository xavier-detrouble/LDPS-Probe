// ============================================================
//  I2C Bus Sniffer — Passive GPIO monitoring
//
//  Detects I2C START (SDA falls while SCL high) and reads
//  the 7-bit address + R/W bit from the first byte.
//  Purely passive — never drives SDA or SCL.
// ============================================================

#include "i2c_sniffer.h"

bool I2CSniffer::begin() {
    // Configure as input only — no pull-ups from our side
    // (Node's I2C bus has its own pull-ups)
    pinMode(_sdaPin, INPUT);
    pinMode(_sclPin, INPUT);
    memset(&_result, 0, sizeof(_result));
    Serial.printf("[I2C] Sniffer ready on SDA=%d SCL=%d\n", _sdaPin, _sclPin);
    return true;
}

void I2CSniffer::sniff(uint32_t durationMs) {
    memset(&_result, 0, sizeof(_result));
    _result.durationMs = durationMs;

    uint32_t endMs = millis() + durationMs;
    bool prevSDA = digitalRead(_sdaPin);
    bool prevSCL = digitalRead(_sclPin);

    // State machine
    enum State { IDLE, READING_ADDR };
    State state = IDLE;
    uint8_t bitCount = 0;
    uint8_t addrByte = 0;

    while (millis() < endMs) {
        bool sda = digitalRead(_sdaPin);
        bool scl = digitalRead(_sclPin);

        // START condition: SDA falls while SCL is high
        if (prevSDA && !sda && scl) {
            _result.transactions++;
            state = READING_ADDR;
            bitCount = 0;
            addrByte = 0;
        }

        // STOP condition: SDA rises while SCL is high
        if (!prevSDA && sda && scl) {
            state = IDLE;
        }

        // Clock rising edge: sample data bit
        if (!prevSCL && scl && state == READING_ADDR) {
            if (bitCount < 8) {
                addrByte = (addrByte << 1) | (sda ? 1 : 0);
                bitCount++;

                if (bitCount == 8) {
                    // Got address byte: [7:1]=address, [0]=R/W
                    uint8_t addr = addrByte >> 1;
                    state = IDLE;  // Don't need more bits

                    // Record unique address
                    bool found = false;
                    for (uint8_t i = 0; i < _result.addressCount; i++) {
                        if (_result.addresses[i] == addr) { found = true; break; }
                    }
                    if (!found && _result.addressCount < 16) {
                        _result.addresses[_result.addressCount++] = addr;
                    }

                    if (addr == 0x3C) _result.oledSeen = true;
                    if (addr == 0x40) _result.ina226Seen = true;
                }
            }
        }

        prevSDA = sda;
        prevSCL = scl;

        // Small yield to avoid watchdog, but keep responsive
        // I2C standard mode is 100kHz, fast mode 400kHz
        // digitalRead polling is fast enough for 100kHz
        delayMicroseconds(1);
    }
}

String I2CSniffer::getResultJson() const {
    String json = "{\"found\":[";
    for (uint8_t i = 0; i < _result.addressCount; i++) {
        if (i > 0) json += ",";
        char buf[8];
        snprintf(buf, sizeof(buf), "\"0x%02X\"", _result.addresses[i]);
        json += buf;
    }
    json += "],\"transactions\":";
    json += String(_result.transactions);
    json += ",\"oled\":";
    json += _result.oledSeen ? "true" : "false";
    json += ",\"ina226\":";
    json += _result.ina226Seen ? "true" : "false";
    json += ",\"duration_ms\":";
    json += String(_result.durationMs);
    json += "}";
    return json;
}

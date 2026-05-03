#pragma once
// ============================================================
//  I2C Bus Sniffer — Passive monitoring
//
//  Listens on SDA/SCL without driving (input-only, no pull-ups).
//  Detects START/STOP conditions, captures addresses being
//  communicated on the bus. Validates OLED (0x3C) and INA226
//  (0x40) are active.
// ============================================================

#include <Arduino.h>
#include "config.h"

struct I2CSniffResult {
    uint8_t  addresses[16];   // Unique addresses seen
    uint8_t  addressCount;
    uint16_t transactions;    // Total START conditions counted
    bool     oledSeen;        // 0x3C observed
    bool     ina226Seen;      // 0x40 observed
    uint32_t durationMs;
};

class I2CSniffer {
public:
    bool begin();
    void sniff(uint32_t durationMs);
    String getResultJson() const;

private:
    I2CSniffResult _result = {};
    uint8_t _sdaPin = I2C_SNIFF_SDA;
    uint8_t _sclPin = I2C_SNIFF_SCL;
};

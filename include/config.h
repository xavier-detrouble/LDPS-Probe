#pragma once
// ============================================================
//  LDPS-Probe — Configuration
//  ESP32-S3 N16R8: ESP-NOW + SX1262 TX + WS2812 Analyzer
// ============================================================

// ── Version ─────────────────────────────────────────────────
#ifndef TB_VERSION
#define TB_VERSION "1.0.0"
#endif

// ── SX1262 SPI (same pins as Dongle/Edge-Node) ──────────────
#define SX_SCK     4
#define SX_MISO    5
#define SX_MOSI    6
#define SX_NSS     7
#define SX_BUSY    15
#define SX_DIO1    16
#define SX_RST     17

// ── SX1262 RF Parameters (must match Edge-Node) ─────────────
#define RF_FREQ       433.0f   // MHz
#define RF_BW         125.0f   // kHz
#define RF_SF         7
#define RF_CR         5        // 4/5
#define RF_SYNC       0x12
#define RF_POWER      14       // dBm
#define RF_PREAMBLE   8

// ── SX1262 Binary Protocol ──────────────────────────────────
#define PROTO_MAGIC       0xD5
#define PROTO_FRAME_LEN   13
#define PROTO_CRC8_POLY   0x31
#define PROTO_PACK_ID_NONE 0

// ── WS2812 Capture (8 GPIO inputs) ──────────────────────────
// GPIO 33-37 unavailable on N16R8 (OPI PSRAM).
// Future: use same pins as Node (8,18,21,38,39,40,47,48) for simpler wiring.
#define CAP_CH0   38    // → Node GPIO 8  (CH1)
#define CAP_CH1   39    // → Node GPIO 18 (CH2)
#define CAP_CH2   40    // → Node GPIO 21 (CH3)
#define CAP_CH3   41    // → Node GPIO 38 (CH4)
#define CAP_CH4   42    // → Node GPIO 39 (CH5)
#define CAP_CH5   45    // → Node GPIO 40 (CH6)
#define CAP_CH6   47    // → Node GPIO 47 (CH7)
#define CAP_CH7   48    // → Node GPIO 48 (CH8)
#define CAP_NUM_CHANNELS 8

// ── I2C Sniff (passive, input-only) ─────────────────────────
#define I2C_SNIFF_SDA  1
#define I2C_SNIFF_SCL  2

// ── ADC (12V via divider) ───────────────────────────────────
#define ADC_VIN_PIN    8   // ADC1_CH7
#define ADC_DIVIDER_RATIO 11.0f  // 100K/10K divider → 11:1

// ── ESP-NOW ─────────────────────────────────────────────────
#define ESPNOW_MAX_DATA_LEN 250
#define WIFI_CH_DEFAULT     1

// ── Playback loop interval ──────────────────────────────────
#define PLAYBACK_INTERVAL_MS 100  // Send RF frame every 100ms

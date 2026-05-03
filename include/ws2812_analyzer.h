#pragma once
// ============================================================
//  WS2812 Analyzer — RMT RX hardware capture
//
//  Uses ESP-IDF 5.x RMT receive for precise pulse timing.
//  4 RX channels → sequential capture: CH0-3 then CH4-7.
//  100ns resolution, hardware-driven, zero CPU during capture.
// ============================================================

#include <Arduino.h>
#include "config.h"
#include "driver/rmt_types.h"

#define MAX_CAPTURE_FRAMES  128
#define MAX_PIXELS_PER_CH   512

struct CaptureResult {
    uint8_t  channel;
    uint16_t frames_captured;
    float    avg_fps;
    float    min_fps;
    float    max_fps;
    bool     timing_ok;
    float    avg_t0h_ns;
    float    avg_t1h_ns;
    uint16_t pixel_mismatches;
    uint16_t total_checked;
    uint16_t dropped_frames;
};

class WS2812Analyzer {
public:
    bool begin();

    void startCapture(uint32_t durationMs);
    bool isCapturing() const { return _capturing; }

    void setExpected(uint8_t channel, const uint8_t* pixels, uint16_t pixelCount);

    String getResultJson() const;

private:
    static void _captureTask(void* param);
    void _doCapture(uint32_t durationMs);
    bool _captureGroup(int startCh, int count, uint32_t durationMs);
    void _decodeRmtSymbols(int ch, const rmt_symbol_word_t* symbols, size_t count);

    volatile bool _capturing = false;
    uint32_t _captureDurationMs = 0;
    CaptureResult _results[CAP_NUM_CHANNELS];

    uint8_t _pins[CAP_NUM_CHANNELS] = {
        CAP_CH0, CAP_CH1, CAP_CH2, CAP_CH3,
        CAP_CH4, CAP_CH5, CAP_CH6, CAP_CH7,
    };
};

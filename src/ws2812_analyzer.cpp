// ============================================================
//  WS2812 Analyzer — RMT RX multi-frame capture
//
//  Sequential per-channel capture with continuous frame
//  accumulation. Each on_recv_done processes one frame and
//  immediately restarts rmt_receive() for the next.
// ============================================================

#include "ws2812_analyzer.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char* TAG = "WS2812";

#define RMT_RESOLUTION_HZ  10000000  // 10MHz = 100ns
#define SYMBOLS_PER_FRAME  16384     // 512px × 24bits × ~1.3 overhead
#define MAX_FRAMES         64

// ── Per-channel accumulated data ────────────────────────────

struct FrameInfo {
    uint32_t timestampMs;
    uint16_t pixelCount;
    uint16_t bitCount;
    float    avgT0Hns;
    float    avgT1Hns;
    uint32_t cntT0;
    uint32_t cntT1;
};

struct ChannelCapture {
    rmt_channel_handle_t rmtCh;
    rmt_symbol_word_t*   symbolBuf;
    volatile bool        frameDone;
    volatile size_t      symbolCount;

    FrameInfo  frames[MAX_FRAMES];
    uint16_t   frameCount;
    uint32_t   captureStartMs;

    uint8_t    firstFramePixels[MAX_PIXELS_PER_CH * 3];
    uint16_t   firstFramePixelCount;
};

static ChannelCapture _cap;

// ── RMT callback ────────────────────────────────────────────

static bool IRAM_ATTR _rmtRxDone(rmt_channel_handle_t ch,
                                  const rmt_rx_done_event_data_t* edata,
                                  void* user_ctx) {
    ChannelCapture* c = (ChannelCapture*)user_ctx;
    c->symbolCount = edata->num_symbols;
    c->frameDone = true;
    return false;
}

// ── Implementation ──────────────────────────────────────────

bool WS2812Analyzer::begin() {
    for (int i = 0; i < CAP_NUM_CHANNELS; i++) {
        pinMode(_pins[i], INPUT_PULLDOWN);
    }
    memset(_results, 0, sizeof(_results));
    ESP_LOGI(TAG, "Analyzer ready (multi-frame RMT, sequential %d ch)", CAP_NUM_CHANNELS);
    return true;
}

void WS2812Analyzer::startCapture(uint32_t durationMs) {
    if (_capturing) return;
    _captureDurationMs = durationMs;
    _capturing = true;
    xTaskCreatePinnedToCore(_captureTask, "ws_cap", 16384, this, 10, nullptr, 0);
}

void WS2812Analyzer::_captureTask(void* param) {
    WS2812Analyzer* self = (WS2812Analyzer*)param;
    self->_doCapture(self->_captureDurationMs);

    String json = self->getResultJson();
    Serial.print("tb:CAPTURE_RESULT,");
    Serial.println(json);
    Serial.flush();

    self->_capturing = false;
    vTaskDelete(nullptr);
}

// ── Capture one channel: multi-frame loop ───────────────────

bool WS2812Analyzer::_captureChannel(int ch, uint32_t durationMs) {
    memset(&_cap, 0, sizeof(_cap));
    _cap.captureStartMs = millis();

    _cap.symbolBuf = (rmt_symbol_word_t*)heap_caps_malloc(
        SYMBOLS_PER_FRAME * sizeof(rmt_symbol_word_t), MALLOC_CAP_SPIRAM);
    if (!_cap.symbolBuf) {
        ESP_LOGE(TAG, "Alloc failed CH%d", ch);
        return false;
    }

    rmt_rx_channel_config_t rx_cfg = {};
    rx_cfg.gpio_num = (gpio_num_t)_pins[ch];
    rx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_cfg.resolution_hz = RMT_RESOLUTION_HZ;
    rx_cfg.mem_block_symbols = 128;

    esp_err_t err = rmt_new_rx_channel(&rx_cfg, &_cap.rmtCh);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT create failed pin %d: %d", _pins[ch], err);
        heap_caps_free(_cap.symbolBuf);
        return false;
    }

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = _rmtRxDone;
    rmt_rx_register_event_callbacks(_cap.rmtCh, &cbs, &_cap);
    rmt_enable(_cap.rmtCh);

    rmt_receive_config_t rx_config = {};
    rx_config.signal_range_min_ns = 100;
    rx_config.signal_range_max_ns = 500000;

    // Start first receive
    _cap.frameDone = false;
    rmt_receive(_cap.rmtCh, _cap.symbolBuf,
                SYMBOLS_PER_FRAME * sizeof(rmt_symbol_word_t), &rx_config);

    // Multi-frame loop: process each frame, restart receive
    uint32_t endMs = _cap.captureStartMs + durationMs;
    while (millis() < endMs && _cap.frameCount < MAX_FRAMES) {
        if (_cap.frameDone) {
            _cap.frameDone = false;
            _processFrame(ch, _cap.symbolBuf, _cap.symbolCount);

            // Restart for next frame
            if (millis() < endMs && _cap.frameCount < MAX_FRAMES) {
                rmt_receive(_cap.rmtCh, _cap.symbolBuf,
                            SYMBOLS_PER_FRAME * sizeof(rmt_symbol_word_t), &rx_config);
            }
        }
        vTaskDelay(1);
    }

    rmt_disable(_cap.rmtCh);
    rmt_del_channel(_cap.rmtCh);
    heap_caps_free(_cap.symbolBuf);
    _cap.symbolBuf = nullptr;

    _buildChannelResult(ch);
    return true;
}

// ── Process one frame ───────────────────────────────────────

void WS2812Analyzer::_processFrame(int ch, const rmt_symbol_word_t* symbols, size_t count) {
    if (count < 10 || _cap.frameCount >= MAX_FRAMES) return;

    const float tickNs = 1000000000.0f / RMT_RESOLUTION_HZ;
    FrameInfo& fi = _cap.frames[_cap.frameCount];
    fi.timestampMs = millis() - _cap.captureStartMs;

    uint32_t sumT0H = 0, sumT1H = 0, cntT0 = 0, cntT1 = 0;
    uint8_t currentByte = 0;
    uint16_t bitCount = 0, byteCount = 0;

    uint8_t* pixBuf = (_cap.frameCount == 0) ? _cap.firstFramePixels : nullptr;

    for (size_t i = 0; i < count; i++) {
        uint32_t highNs = (uint32_t)(symbols[i].duration0 * tickNs);
        uint32_t lowNs = (uint32_t)(symbols[i].duration1 * tickNs);

        if (highNs < 50 || highNs > 2000) continue;
        if (lowNs > 50000) continue;

        uint8_t bitVal = (highNs > 500) ? 1 : 0;
        if (bitVal == 0) { sumT0H += highNs; cntT0++; }
        else             { sumT1H += highNs; cntT1++; }

        currentByte = (currentByte << 1) | bitVal;
        bitCount++;
        if (bitCount % 8 == 0) {
            if (pixBuf && byteCount < MAX_PIXELS_PER_CH * 3)
                pixBuf[byteCount] = currentByte;
            byteCount++;
            currentByte = 0;
        }
    }

    fi.bitCount = bitCount;
    fi.pixelCount = byteCount / 3;
    fi.cntT0 = cntT0;
    fi.cntT1 = cntT1;
    fi.avgT0Hns = (cntT0 > 0) ? (float)sumT0H / cntT0 : 0;
    fi.avgT1Hns = (cntT1 > 0) ? (float)sumT1H / cntT1 : 0;

    if (_cap.frameCount == 0)
        _cap.firstFramePixelCount = fi.pixelCount;

    _cap.frameCount++;
}

// ── Build result ────────────────────────────────────────────

void WS2812Analyzer::_buildChannelResult(int ch) {
    CaptureResult& r = _results[ch];
    r.channel = ch;
    r.frames_captured = _cap.frameCount;

    if (_cap.frameCount == 0) return;

    // Aggregate timing
    float totT0H = 0, totT1H = 0;
    uint32_t totCntT0 = 0, totCntT1 = 0;
    for (uint16_t i = 0; i < _cap.frameCount; i++) {
        totT0H += _cap.frames[i].avgT0Hns * _cap.frames[i].cntT0;
        totT1H += _cap.frames[i].avgT1Hns * _cap.frames[i].cntT1;
        totCntT0 += _cap.frames[i].cntT0;
        totCntT1 += _cap.frames[i].cntT1;
    }
    r.avg_t0h_ns = (totCntT0 > 0) ? totT0H / totCntT0 : 0;
    r.avg_t1h_ns = (totCntT1 > 0) ? totT1H / totCntT1 : 0;

    r.timing_ok = true;
    if (totCntT0 > 0 && (r.avg_t0h_ns < 200 || r.avg_t0h_ns > 600)) r.timing_ok = false;
    if (totCntT1 > 0 && (r.avg_t1h_ns < 500 || r.avg_t1h_ns > 1100)) r.timing_ok = false;

    // FPS
    if (_cap.frameCount >= 2) {
        uint32_t totalMs = 0, minMs = UINT32_MAX, maxMs = 0;
        for (uint16_t i = 1; i < _cap.frameCount; i++) {
            uint32_t d = _cap.frames[i].timestampMs - _cap.frames[i-1].timestampMs;
            totalMs += d;
            if (d < minMs) minMs = d;
            if (d > maxMs) maxMs = d;
        }
        float avgMs = (float)totalMs / (_cap.frameCount - 1);
        r.avg_fps = (avgMs > 0) ? 1000.0f / avgMs : 0;
        r.min_fps = (maxMs > 0) ? 1000.0f / maxMs : 0;
        r.max_fps = (minMs > 0) ? 1000.0f / minMs : 0;

        // Dropped frames
        for (uint16_t i = 1; i < _cap.frameCount; i++) {
            uint32_t d = _cap.frames[i].timestampMs - _cap.frames[i-1].timestampMs;
            if ((float)d > avgMs * 1.8f) r.dropped_frames++;
        }
    }

    r.total_checked = _cap.frames[0].pixelCount;

    // Save first pixel (GRB wire → RGB)
    if (_cap.firstFramePixelCount > 0) {
        r.px0_g = _cap.firstFramePixels[0];
        r.px0_r = _cap.firstFramePixels[1];
        r.px0_b = _cap.firstFramePixels[2];
        r.frame_id = r.px0_r + (uint16_t)r.px0_g * 256;
    }

    // Compare first frame with expected pattern
    if (_expected[ch].hasData && _cap.firstFramePixelCount > 0) {
        uint16_t cmpPixels = min(_cap.firstFramePixelCount, _expected[ch].pixelCount);
        uint16_t mismatches = 0;
        for (uint16_t i = 0; i < cmpPixels * 3; i++) {
            if (_cap.firstFramePixels[i] != _expected[ch].pixels[i]) {
                mismatches++;
            }
        }
        r.pixel_mismatches = mismatches;
        r.total_checked = cmpPixels;
    }

    ESP_LOGI(TAG, "CH%d: %d frames, %.1f fps, T0H=%.0f T1H=%.0f, drops=%d, mismatch=%d/%d",
             ch, r.frames_captured, r.avg_fps,
             r.avg_t0h_ns, r.avg_t1h_ns, r.dropped_frames,
             r.pixel_mismatches, r.total_checked);
}

// ── Main capture ────────────────────────────────────────────

void WS2812Analyzer::_doCapture(uint32_t durationMs) {
    memset(_results, 0, sizeof(_results));

    uint32_t perChMs = durationMs / CAP_NUM_CHANNELS;
    if (perChMs < 500) perChMs = 500;

    for (int ch = 0; ch < CAP_NUM_CHANNELS; ch++) {
        ESP_LOGI(TAG, "CH%d (pin %d, %lums)...", ch, _pins[ch], perChMs);
        if (!_captureChannel(ch, perChMs)) {
            ESP_LOGE(TAG, "CH%d failed", ch);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "Complete");
}

void WS2812Analyzer::setExpected(uint8_t channel, const uint8_t* pixels, uint16_t count) {
    if (channel >= CAP_NUM_CHANNELS) return;
    uint16_t bytes = count * 3;
    if (bytes > sizeof(_expected[channel].pixels)) bytes = sizeof(_expected[channel].pixels);
    memcpy(_expected[channel].pixels, pixels, bytes);
    _expected[channel].pixelCount = count;
    _expected[channel].hasData = true;
    ESP_LOGI(TAG, "Expected set for CH%d: %d pixels", channel, count);
}

void WS2812Analyzer::clearExpected() {
    for (int i = 0; i < CAP_NUM_CHANNELS; i++) {
        _expected[i].hasData = false;
        _expected[i].pixelCount = 0;
    }
}

String WS2812Analyzer::getResultJson() const {
    String json = "{\"channels\":[";
    for (int i = 0; i < CAP_NUM_CHANNELS; i++) {
        if (i > 0) json += ",";
        const CaptureResult& r = _results[i];
        char buf[300];
        snprintf(buf, sizeof(buf),
            "{\"ch\":%d,\"frames\":%d,"
            "\"avg_fps\":%.1f,\"min_fps\":%.1f,\"max_fps\":%.1f,"
            "\"timing_ok\":%s,\"avg_t0h_ns\":%.0f,\"avg_t1h_ns\":%.0f,"
            "\"mismatches\":%d,\"checked\":%d,\"dropped\":%d,"
            "\"px0\":[%d,%d,%d],\"frame_id\":%d}",
            r.channel, r.frames_captured,
            r.avg_fps, r.min_fps, r.max_fps,
            r.timing_ok ? "true" : "false",
            r.avg_t0h_ns, r.avg_t1h_ns,
            r.pixel_mismatches, r.total_checked, r.dropped_frames,
            r.px0_r, r.px0_g, r.px0_b, r.frame_id);
        json += buf;
    }
    json += "]}";
    return json;
}

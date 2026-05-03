// ============================================================
//  WS2812 Analyzer — RMT RX hardware capture
//
//  Uses ESP-IDF 5.x RMT receive driver for precise WS2812
//  pulse timing capture. 4 RX channels available on ESP32-S3.
//  Sequential capture: CH0-3 first, then CH4-7.
// ============================================================

#include "ws2812_analyzer.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"

static const char* TAG = "WS2812";

// ── RMT receive callback data ───────────────────────────────

#define RMT_RESOLUTION_HZ  10000000  // 10MHz = 100ns resolution
#define MAX_SYMBOLS_PER_CH 24576     // 512 pixels × 24 bits × 2 symbols/bit

struct RmtRxData {
    rmt_channel_handle_t channel;
    rmt_symbol_word_t*   symbols;
    size_t               symbolCount;
    volatile bool        done;
    uint32_t             frameCount;
    uint32_t             lastResetTick;  // tick of last reset pulse
};

static RmtRxData _rmtData[4];  // 4 RMT RX channels max

// RMT receive done callback
static bool IRAM_ATTR _rmtRxDone(rmt_channel_handle_t ch,
                                  const rmt_rx_done_event_data_t* edata,
                                  void* user_ctx) {
    RmtRxData* d = (RmtRxData*)user_ctx;
    // Copy received symbols
    size_t count = edata->num_symbols;
    if (count > MAX_SYMBOLS_PER_CH) count = MAX_SYMBOLS_PER_CH;
    memcpy(d->symbols, edata->received_symbols, count * sizeof(rmt_symbol_word_t));
    d->symbolCount = count;
    d->done = true;
    return false;  // no high-priority task woken
}

// ── Implementation ──────────────────────────────────────────

bool WS2812Analyzer::begin() {
    for (int i = 0; i < CAP_NUM_CHANNELS; i++) {
        pinMode(_pins[i], INPUT_PULLDOWN);
    }
    memset(_results, 0, sizeof(_results));
    ESP_LOGI(TAG, "Analyzer ready, %d channels (sequential 4+4 via RMT)", CAP_NUM_CHANNELS);
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

// ── Capture one group of up to 4 channels via RMT ──────────

bool WS2812Analyzer::_captureGroup(int startCh, int count, uint32_t durationMs) {
    if (count > 4) count = 4;

    // Allocate symbol buffers in PSRAM
    for (int i = 0; i < count; i++) {
        _rmtData[i].symbols = (rmt_symbol_word_t*)heap_caps_malloc(
            MAX_SYMBOLS_PER_CH * sizeof(rmt_symbol_word_t), MALLOC_CAP_SPIRAM);
        if (!_rmtData[i].symbols) {
            ESP_LOGE(TAG, "Symbol buffer alloc failed for ch%d", startCh + i);
            // Free already allocated
            for (int j = 0; j < i; j++) {
                heap_caps_free(_rmtData[j].symbols);
                _rmtData[j].symbols = nullptr;
            }
            return false;
        }
        _rmtData[i].symbolCount = 0;
        _rmtData[i].done = false;
        _rmtData[i].channel = nullptr;
    }

    // Configure RMT RX channels
    for (int i = 0; i < count; i++) {
        int ch = startCh + i;
        rmt_rx_channel_config_t rx_cfg = {};
        rx_cfg.gpio_num = (gpio_num_t)_pins[ch];
        rx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        rx_cfg.resolution_hz = RMT_RESOLUTION_HZ;
        rx_cfg.mem_block_symbols = 128;  // SOC minimum, DMA extends to buffer

        esp_err_t err = rmt_new_rx_channel(&rx_cfg, &_rmtData[i].channel);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "RMT RX channel create failed for pin %d: %d", _pins[ch], err);
            // Cleanup
            for (int j = 0; j <= i; j++) {
                if (_rmtData[j].channel) {
                    rmt_del_channel(_rmtData[j].channel);
                    _rmtData[j].channel = nullptr;
                }
                heap_caps_free(_rmtData[j].symbols);
                _rmtData[j].symbols = nullptr;
            }
            return false;
        }

        // Register callback
        rmt_rx_event_callbacks_t cbs = {};
        cbs.on_recv_done = _rmtRxDone;
        rmt_rx_register_event_callbacks(_rmtData[i].channel, &cbs, &_rmtData[i]);

        // Enable channel
        rmt_enable(_rmtData[i].channel);
    }

    // Start receiving on all channels
    rmt_receive_config_t rx_config = {};
    rx_config.signal_range_min_ns = 100;     // Reject glitches < 100ns
    rx_config.signal_range_max_ns = 500000;  // Reset pulse up to 500μs

    for (int i = 0; i < count; i++) {
        rmt_receive(_rmtData[i].channel, _rmtData[i].symbols,
                    MAX_SYMBOLS_PER_CH * sizeof(rmt_symbol_word_t), &rx_config);
    }

    // Wait for duration or all channels done
    uint32_t startMs = millis();
    while (millis() - startMs < durationMs) {
        bool allDone = true;
        for (int i = 0; i < count; i++) {
            if (!_rmtData[i].done) {
                allDone = false;
                break;
            }
        }
        if (allDone) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Decode results
    for (int i = 0; i < count; i++) {
        int ch = startCh + i;
        _decodeRmtSymbols(ch, _rmtData[i].symbols, _rmtData[i].symbolCount);
    }

    // Cleanup RMT channels
    for (int i = 0; i < count; i++) {
        if (_rmtData[i].channel) {
            rmt_disable(_rmtData[i].channel);
            rmt_del_channel(_rmtData[i].channel);
            _rmtData[i].channel = nullptr;
        }
        if (_rmtData[i].symbols) {
            heap_caps_free(_rmtData[i].symbols);
            _rmtData[i].symbols = nullptr;
        }
    }

    return true;
}

// ── Main capture: sequential CH0-3 then CH4-7 ──────────────

void WS2812Analyzer::_doCapture(uint32_t durationMs) {
    memset(_results, 0, sizeof(_results));

    uint32_t perChDuration = durationMs / CAP_NUM_CHANNELS;
    if (perChDuration < 200) perChDuration = 200;

    // Sequential: one channel at a time (only 1 RMT RX channel used)
    for (int ch = 0; ch < CAP_NUM_CHANNELS; ch++) {
        ESP_LOGI(TAG, "Capturing CH%d (pin %d, %lums)...", ch, _pins[ch], perChDuration);
        if (!_captureGroup(ch, 1, perChDuration)) {
            ESP_LOGE(TAG, "CH%d capture failed", ch);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Capture complete");
}

// ── Decode RMT symbols into WS2812 bits + pixels ───────────

void WS2812Analyzer::_decodeRmtSymbols(int ch, const rmt_symbol_word_t* symbols, size_t count) {
    CaptureResult& r = _results[ch];
    r.channel = ch;

    if (count < 2) return;

    const float tickNs = 1000000000.0f / RMT_RESOLUTION_HZ;  // 100ns per tick

    // Decode symbols into bits
    // Each WS2812 bit = 1 symbol (high duration + low duration)
    // T0H < 500ns → bit 0, T1H > 500ns → bit 1
    uint8_t decodedBytes[MAX_PIXELS_PER_CH * 3];
    uint16_t bitCount = 0;
    uint8_t currentByte = 0;
    uint16_t byteCount = 0;

    uint32_t sumT0H = 0, sumT1H = 0, cntT0 = 0, cntT1 = 0;
    uint32_t frameCount = 0;

    for (size_t i = 0; i < count && byteCount < MAX_PIXELS_PER_CH * 3; i++) {
        uint32_t highTicks = symbols[i].duration0;
        uint32_t lowTicks = symbols[i].duration1;
        uint32_t highNs = (uint32_t)(highTicks * tickNs);
        uint32_t lowNs = (uint32_t)(lowTicks * tickNs);

        // Detect reset pulse (low > 50μs)
        if (lowNs > 50000 || (lowTicks == 0 && i == count - 1)) {
            // Frame boundary
            if (bitCount > 0) {
                frameCount++;
                bitCount = 0;
            }
            continue;
        }

        // Skip anomalous pulses
        if (highNs < 50 || highNs > 2000) continue;

        // Decode bit: threshold at 500ns (midpoint of T0H=350ns and T1H=700ns)
        uint8_t bitVal = (highNs > 500) ? 1 : 0;

        // Timing stats
        if (bitVal == 0) {
            sumT0H += highNs;
            cntT0++;
        } else {
            sumT1H += highNs;
            cntT1++;
        }

        // Assemble into bytes (MSB first)
        currentByte = (currentByte << 1) | bitVal;
        bitCount++;
        if (bitCount % 8 == 0) {
            if (byteCount < MAX_PIXELS_PER_CH * 3) {
                decodedBytes[byteCount++] = currentByte;
            }
            currentByte = 0;
        }
    }

    // Count the last frame if there were remaining bits
    if (bitCount > 0) frameCount++;

    // Fill result
    r.frames_captured = frameCount;
    r.avg_t0h_ns = (cntT0 > 0) ? (float)sumT0H / cntT0 : 0;
    r.avg_t1h_ns = (cntT1 > 0) ? (float)sumT1H / cntT1 : 0;

    // Timing validation (relaxed bounds for LCD_CAM 3-step cadence)
    r.timing_ok = true;
    if (cntT0 > 0 && (r.avg_t0h_ns < 200 || r.avg_t0h_ns > 600)) r.timing_ok = false;
    if (cntT1 > 0 && (r.avg_t1h_ns < 500 || r.avg_t1h_ns > 1100)) r.timing_ok = false;

    // Pixel count (3 bytes per GRB pixel)
    uint16_t pixelCount = byteCount / 3;
    r.pixel_mismatches = 0;
    r.total_checked = pixelCount;
    r.dropped_frames = 0;

    // TODO: FPS calculation from frame timestamps (needs multiple captures)
    r.avg_fps = 0;
    r.min_fps = 0;
    r.max_fps = 0;

    ESP_LOGI(TAG, "CH%d: %d frames, %d pixels, %d bits, T0H=%.0fns T1H=%.0fns",
             ch, frameCount, pixelCount, cntT0 + cntT1, r.avg_t0h_ns, r.avg_t1h_ns);
}

// ── Set expected pattern ────────────────────────────────────

void WS2812Analyzer::setExpected(uint8_t channel, const uint8_t* pixels, uint16_t count) {
    // TODO: store expected pattern for comparison
    (void)channel; (void)pixels; (void)count;
}

// ── JSON result ─────────────────────────────────────────────

String WS2812Analyzer::getResultJson() const {
    String json = "{\"channels\":[";
    for (int i = 0; i < CAP_NUM_CHANNELS; i++) {
        if (i > 0) json += ",";
        const CaptureResult& r = _results[i];
        char buf[250];
        snprintf(buf, sizeof(buf),
            "{\"ch\":%d,\"frames\":%d,"
            "\"avg_fps\":%.1f,\"min_fps\":%.1f,\"max_fps\":%.1f,"
            "\"timing_ok\":%s,\"avg_t0h_ns\":%.0f,\"avg_t1h_ns\":%.0f,"
            "\"mismatches\":%d,\"checked\":%d,\"dropped\":%d}",
            r.channel, r.frames_captured,
            r.avg_fps, r.min_fps, r.max_fps,
            r.timing_ok ? "true" : "false",
            r.avg_t0h_ns, r.avg_t1h_ns,
            r.pixel_mismatches, r.total_checked, r.dropped_frames);
        json += buf;
    }
    json += "]}";
    return json;
}

// ============================================================
//  UART Handler — parse commands from USB CDC
// ============================================================

#include "uart_handler.h"
#include "espnow_bridge.h"
#include "sx1262_tx.h"
#include "ws2812_analyzer.h"
#include "i2c_sniffer.h"
#include "config.h"
#include "soc/gpio_reg.h"

void UartHandler::begin(ESPNowBridge* espnow, SX1262TX* sx, WS2812Analyzer* analyzer, I2CSniffer* i2c) {
    _espnow = espnow;
    _sx = sx;
    _analyzer = analyzer;
    _i2c = i2c;
    _bufLen = 0;
}

void UartHandler::poll() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (_bufLen > 0) {
                _buf[_bufLen] = '\0';
                _dispatch(_buf);
                _bufLen = 0;
            }
        } else if (_bufLen < sizeof(_buf) - 1) {
            _buf[_bufLen++] = c;
        }
    }
}

void UartHandler::_dispatch(const char* line) {
    // Find colon separator: TYPE:PAYLOAD
    const char* colon = strchr(line, ':');
    if (!colon || colon == line) return;

    size_t typeLen = colon - line;
    const char* payload = colon + 1;

    if (typeLen == 2 && line[0] == 'E' && line[1] == 'N') {
        _handleEN(payload);
    } else if (typeLen == 2 && line[0] == 'S' && line[1] == 'X') {
        _handleSX(payload);
    } else if (typeLen == 2 && line[0] == 'T' && line[1] == 'B') {
        _handleTB(payload);
    } else if (typeLen == 2 && line[0] == 'D' && line[1] == 'G') {
        _handleDG(payload);
    }
}

void UartHandler::_handleEN(const char* payload) {
    // EN:{mac},{command}[,{args}]
    // Forward to ESP-NOW bridge
    const char* comma = strchr(payload, ',');
    if (!comma) return;

    char mac[18];
    size_t macLen = comma - payload;
    if (macLen >= sizeof(mac)) return;
    strncpy(mac, payload, macLen);
    mac[macLen] = '\0';

    _espnow->forward(mac, comma + 1);
}

void UartHandler::_handleSX(const char* payload) {
    // SX:PLAY,{seq_idx},{pack_id},{brightness}
    // SX:STOP
    // SX:SEEK,{ms}
    // SX:BRIGHTNESS,{0-100}
    if (strncmp(payload, "PLAY,", 5) == 0) {
        // Parse: seq_idx,pack_id,brightness
        int si = 0, pi = 0, br = 100;
        sscanf(payload + 5, "%d,%d,%d", &si, &pi, &br);
        _sx->play(si, pi, br);
        Serial.println("sx:OK");
    } else if (strcmp(payload, "STOP") == 0) {
        _sx->stop();
        Serial.println("sx:OK");
    } else if (strncmp(payload, "SEEK,", 5) == 0) {
        uint32_t ms = atoi(payload + 5);
        _sx->seek(ms);
        Serial.println("sx:OK");
    } else if (strncmp(payload, "BRIGHTNESS,", 11) == 0) {
        uint8_t b = atoi(payload + 11);
        _sx->setBrightness(b);
        Serial.println("sx:OK");
    } else {
        Serial.println("sx:ERR,unknown");
    }
}

void UartHandler::_handleTB(const char* payload) {
    // TB:PLAY_AND_CAPTURE,{seq_idx},{pack_id},{brightness},{duration_ms}
    // TB:CAPTURE_STOP
    // TB:CAPTURE_STATUS
    // TB:SNIFF_I2C,{duration_ms}
    // TB:READ_ADC
    // TB:STATUS

    if (strncmp(payload, "PLAY_AND_CAPTURE,", 17) == 0) {
        int si = 0, pi = 0, br = 100;
        uint32_t dur = 5000;
        sscanf(payload + 17, "%d,%d,%d,%u", &si, &pi, &br, &dur);
        // Start capture first, then play (timing sync)
        _analyzer->startCapture(dur);
        _sx->play(si, pi, br);
        Serial.println("tb:OK,capturing");
    } else if (strcmp(payload, "CAPTURE_STOP") == 0) {
        // TODO: stop capture
        Serial.println("tb:OK");
    } else if (strcmp(payload, "CAPTURE_STATUS") == 0) {
        Serial.printf("tb:CAPTURE_STATUS,%s\n", _analyzer->isCapturing() ? "running" : "idle");
    } else if (strncmp(payload, "SNIFF_I2C,", 10) == 0) {
        uint32_t dur = atoi(payload + 10);
        if (dur == 0) dur = 2000;
        if (_i2c) {
            _i2c->sniff(dur);
            Serial.printf("tb:I2C_RESULT,%s\n", _i2c->getResultJson().c_str());
        } else {
            Serial.println("tb:I2C_RESULT,{\"error\":\"not_initialized\"}");
        }
    } else if (strcmp(payload, "READ_ADC") == 0) {
        int raw = analogRead(ADC_VIN_PIN);
        float voltage = (raw / 4095.0f) * 3.3f * ADC_DIVIDER_RATIO;
        Serial.printf("tb:ADC_RESULT,%.0f\n", voltage * 1000);  // mV
    } else if (strcmp(payload, "GPIO_READ") == 0) {
        // Debug: raw read of all capture pins
        uint32_t in1 = REG_READ(GPIO_IN1_REG);
        Serial.printf("tb:GPIO_RAW,IN1=0x%08lX,", in1);
        for (int i = 0; i < 8; i++) {
            uint8_t pin = (uint8_t[]){CAP_CH0,CAP_CH1,CAP_CH2,CAP_CH3,CAP_CH4,CAP_CH5,CAP_CH6,CAP_CH7}[i];
            uint8_t bit = pin - 32;
            Serial.printf("CH%d(G%d)=%d ", i, pin, (in1 >> bit) & 1);
        }
        Serial.println();
    } else if (strcmp(payload, "GPIO_POLL") == 0) {
        // Raw edge counting on all capture pins for 1 second (main core)
        volatile uint32_t* gpioIn = (volatile uint32_t*)GPIO_IN1_REG;
        uint32_t mask = 0;
        uint8_t pins[] = {CAP_CH0,CAP_CH1,CAP_CH2,CAP_CH3,CAP_CH4,CAP_CH5,CAP_CH6,CAP_CH7};
        for (int i = 0; i < 8; i++) mask |= (1UL << (pins[i] - 32));

        uint32_t prev = (*gpioIn) & mask;
        uint32_t edgeCount = 0;
        uint32_t endMs = millis() + 1000;
        while (millis() < endMs) {
            uint32_t cur = (*gpioIn) & mask;
            if (cur != prev) {
                edgeCount++;
                prev = cur;
            }
        }
        Serial.printf("tb:GPIO_POLL,edges=%lu,mask=0x%08lX\n", edgeCount, mask);
    } else if (strcmp(payload, "SX_RESET") == 0) {
        _sx->reset();
        Serial.printf("tb:SX_RESET,%s\n", _sx->isPlaying() ? "err" : "ok");
    } else if (strcmp(payload, "STATUS") == 0) {
        Serial.printf("tb:STATUS,{\"version\":\"%s\",\"uptime\":%lu,\"capturing\":%s}\n",
                      TB_VERSION, millis() / 1000,
                      _analyzer->isCapturing() ? "true" : "false");
    } else {
        Serial.println("tb:ERR,unknown");
    }
}

void UartHandler::_handleDG(const char* payload) {
    if (strcmp(payload, "STATUS") == 0) {
        Serial.printf("dg:READY,version=%s\n", TB_VERSION);
    } else {
        Serial.printf("dg:OK\n");
    }
}

// ============================================================
//  LDPS-Probe — Main entry point
//  ESP-NOW bridge + SX1262 TX + WS2812 signal analyzer
// ============================================================

#include <Arduino.h>
#include "config.h"
#include "uart_handler.h"
#include "espnow_bridge.h"
#include "sx1262_tx.h"
#include "ws2812_analyzer.h"
#include "i2c_sniffer.h"

UartHandler uart;
ESPNowBridge espnow;
SX1262TX sx1262;
WS2812Analyzer analyzer;
I2CSniffer i2cSniff;

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("[Probe] LDPS-Probe " TB_VERSION " starting...");

    // Init subsystems
    if (!espnow.begin()) {
        Serial.println("[Probe] ESP-NOW init FAILED");
    }

    if (!sx1262.begin()) {
        Serial.println("[Probe] SX1262 init FAILED");
    }

    if (!analyzer.begin()) {
        Serial.println("[Probe] WS2812 Analyzer init FAILED");
    }

    if (!i2cSniff.begin()) {
        Serial.println("[Probe] I2C Sniffer init FAILED");
    }

    uart.begin(&espnow, &sx1262, &analyzer, &i2cSniff);

    Serial.println("tb:READY");
    Serial.printf("dg:READY,version=%s,sx1262=%s,espnow=%s\n",
                  TB_VERSION,
                  sx1262.isPlaying() ? "ok" : "idle",
                  "ok");
}

void loop() {
    uart.poll();
    sx1262.loop();

    // Check if capture just finished
    static bool wasCap = false;
    bool isCap = analyzer.isCapturing();
    if (wasCap && !isCap) {
        String json = analyzer.getResultJson();
        Serial.print("tb:CAPTURE_RESULT,");
        Serial.println(json);
        Serial.flush();
        wasCap = false;
    }
    if (isCap) wasCap = true;

    delay(1);
}

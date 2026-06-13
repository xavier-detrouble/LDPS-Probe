# LDPS-Probe

ESP32-S3 N16R8 生產測試板韌體。集成 ESP-NOW bridge + SX1262 TX + WS2812 RMT signal analyzer + I2C sniffer。

## 功能
- **ESP-NOW Bridge**: 轉發 EN: 指令到 Node（和 Dongle 相同協議）
- **SX1262 TX**: 發送 **14-byte v2** binary playback frame（ADR-016；sync 0x4C；需要 `setDio2AsRfSwitch(true)` for DX-LR30，ADR-002）
- **WS2812 Analyzer**: Sequential RMT RX capture（每次 1 channel，10MHz resolution = 100ns/tick，ADR-001）
- **I2C Sniff**: 被動監聽 Node I2C bus（偵測 OLED 0x3C + INA226 0x40）
- **ADC**: 量測 12V 電壓（GPIO 8, voltage divider）

## UART Protocol
```
下行: EN:{mac},{cmd}\n  SX:{cmd}\n  TB:{cmd}\n  DG:{cmd}\n
上行: en:{mac},{rsp}\n  sx:{rsp}\n  tb:{rsp}\n  dg:{rsp}\n
```

## Pin Mapping
```
SX1262:          SCK=4, MISO=5, MOSI=6, CS=7, RST=17, BUSY=15, DIO1=16
WS2812 Capture:  CH0=38, CH1=39, CH2=40, CH3=41, CH4=42, CH5=45, CH6=47, CH7=48
                 (GPIO 33-37 不可用 — N16R8 OPI PSRAM 佔用)
I2C Sniff:       SDA=1, SCL=2
ADC:             GPIO 8 (12V divider)
```

## 開發
```bash
/tmp/pio312/bin/pio run -e probe                                          # 編譯
/tmp/pio312/bin/pio run -e probe -t upload --upload-port /dev/cu.usbmodem11101  # 上傳（必須指定 port！）
```

## 關鍵設計決策
- ADR-001: WS2812 capture 用 RMT RX 不用 GPIO polling（GPIO APB bus 太慢）
- ADR-002: SX1262 必須 setDio2AsRfSwitch(true)（DX-LR30 RF 前端開關）
- ADR-003: Node 的 Show() 必須 loop 所有 buses（LCD_CAM mux 要求）

## 實機測試結果 (2026-05-03)
- 8 channels WS2812 全部捕捉成功
- T0H=389-408ns, T1H=808-816ns（LCD_CAM 3-step cadence）
- I2C sniff: OLED + INA226 偵測到, 1316 transactions/2s
- SX1262 TX: Node RF RX count 持續增加（當時 v1 0x12/13B，與當時的 Node 匹配）

## ⚠️ RF v2 對齊 (2026-06-13)
本 session 把 Edge-Node 升到 RF v2（ADR-016：sync 0x4C，14-byte frame，
pack_id 改 uint16 CRC-16/CCITT）。Probe 是第三個說 RF 協議的韌體，當時漏改，
導致它送的 frame 被 node PHY 拒收 → RF RX 測試失效。已補齊到 v2（config.h
RF_SYNC/PROTO_FRAME_LEN/PROTO_VERSION + _buildFrame + play() packId uint16）。
**已 build 未刷**：需治具硬件 + 一個 node 上機驗證 RF RX count 重新遞增；
host 端 `SX:PLAY,{seq},{pack_id},{br}` 的 pack_id 須傳測試 pack 的 CRC-16/CCITT
值（Provisioning-Station 目前用 CRC-8，需同步改成 CRC-16，否則 node 過濾掉）。

## TODO
- FPS 計算（multi-frame capture）
- Expected pattern 比對（SET_EXPECTED）
- Dropped frame detection

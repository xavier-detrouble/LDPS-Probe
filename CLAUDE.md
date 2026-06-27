# LDPS-Probe

ESP32-S3 N16R8 production-test-board firmware. Integrates an ESP-NOW bridge + SX1262 TX +
WS2812 RMT signal analyzer + I2C sniffer.

## Features
- **ESP-NOW Bridge**: forwards `EN:` commands to a Node (same protocol as the Dongle)
- **SX1262 TX**: sends the **14-byte v2** binary playback frame (ADR-016; sync 0x4C; needs
  `setDio2AsRfSwitch(true)` for the DX-LR30, ADR-002)
- **WS2812 Analyzer**: sequential RMT RX capture (1 channel at a time, 10 MHz resolution =
  100 ns/tick, ADR-001)
- **I2C Sniff**: passively monitors the Node's I2C bus (detects OLED 0x3C + INA226 0x40)
- **ADC**: measures 12 V (GPIO 8, voltage divider)

## UART protocol
```
downlink: EN:{mac},{cmd}\n  SX:{cmd}\n  TB:{cmd}\n  DG:{cmd}\n
uplink:   en:{mac},{rsp}\n  sx:{rsp}\n  tb:{rsp}\n  dg:{rsp}\n
```

## Pin mapping
```
SX1262:          SCK=4, MISO=5, MOSI=6, CS=7, RST=17, BUSY=15, DIO1=16
WS2812 Capture:  CH0=38, CH1=39, CH2=40, CH3=41, CH4=42, CH5=45, CH6=47, CH7=48
                 (GPIO 33-37 unavailable — taken by the N16R8 OPI PSRAM)
I2C Sniff:       SDA=1, SCL=2
ADC:             GPIO 8 (12V divider)
```

## Build
```bash
/tmp/pio312/bin/pio run -e probe                                                # build
/tmp/pio312/bin/pio run -e probe -t upload --upload-port /dev/cu.usbmodem11101  # upload (always specify the port!)
```

## Key design decisions
- ADR-001: WS2812 capture uses RMT RX, not GPIO polling (the GPIO APB bus is too slow)
- ADR-002: SX1262 must `setDio2AsRfSwitch(true)` (DX-LR30 RF front-end switch)
- ADR-003: the Node's `Show()` must loop all buses (LCD_CAM mux requirement)

## Bench-test results (2026-05-03)
- All 8 WS2812 channels captured successfully
- T0H = 389–408 ns, T1H = 808–816 ns (LCD_CAM 3-step cadence)
- I2C sniff: OLED + INA226 detected, 1316 transactions / 2 s
- SX1262 TX: Node RF-RX count kept increasing (then v1 0x12 / 13 B, matching the Node at the time)

## ⚠️ RF v2 alignment (2026-06-13)
This session upgraded the Edge-Node to RF v2 (ADR-016: sync 0x4C, 14-byte frame, `pack_id`
changed to uint16 CRC-16/CCITT). The Probe is the third firmware that speaks the RF protocol
and was missed at the time, so its frames were rejected by the node PHY → the RF-RX test broke.
Now brought up to v2 (`config.h` RF_SYNC / PROTO_FRAME_LEN / PROTO_VERSION + `_buildFrame` +
`play()` packId uint16). **Built, not flashed**: needs the jig hardware + a node on the bench to
verify the RF-RX count rises again; the host-side `SX:PLAY,{seq},{pack_id},{br}` must pass the
test pack's CRC-16/CCITT `pack_id` (the Provisioning-Station currently uses CRC-8 and must be
changed to CRC-16 to match, or the node filters it out).

## TODO
- FPS calculation (multi-frame capture)
- Expected-pattern comparison (SET_EXPECTED)
- Dropped-frame detection

See the platform docs map: [../docs/index.md](../docs/index.md).

# Release Notes

## DJI-Remote v1.1.0

Firmware for M5Stack Basic V2.7 (ESP32).

This release migrates the UI rendering engine to LVGL and the Bluetooth stack to
Apache NimBLE, improving reliability and enabling a cleaner architecture for future
development.

### What's New in v1.1.0

- **LVGL 9.5.0 UI rendering** — All screens replaced with LVGL widget-based rendering via esp_lvgl_port 2.7.2. Replaces manual TFT/SPI drawing.
- **NimBLE BLE stack** — Apache NimBLE replaces Bluedroid as the GATT client stack.
- **Boot splash screen** — LVGL-rendered splash screen replaces the raw bitmap boot logo.
- **Release packaging automation** — GitHub Actions workflow and `build_release.sh` produce merged binary and ZIP for every tagged release.
- **NimBLE scan fixes** — Device names now shown during scan; duplicate filter disabled; reconnect-on-unpair prevented.
- **Production log level** — Default log level set to ERROR for production builds.

### Flash Instructions

**Option 1 — Web Flash (easiest)**

Use [espflash.app](https://espflash.app) or [ESP Web Tools](https://web.esptool.io/)
with the merged binary `dji-remote-merged.bin`. Requires Chrome or Edge.

**Option 2 — Manual Flash**

Download `DJI-Remote-v1.1.0-esp32.zip`, extract it, and follow the instructions
in `README.txt`. Requires [esptool](https://github.com/espressif/esptool/releases).

### Flash Settings

| Setting         | Value  |
|-----------------|--------|
| Chip            | ESP32  |
| Flash mode      | DIO    |
| Flash size      | 16 MB  |
| Flash frequency | 80 MHz |

---

## DJI-Remote v1.0.0

Initial public release. Firmware for M5Stack Basic V2.7 (ESP32).

### Features

- Control up to three DJI Osmo Action cameras simultaneously over BLE
- Live GPS forwarding to all connected cameras (10 Hz)
- Start/stop recording, highlight tags, sleep/wake, snapshot-while-sleeping
- Mode switching via QS button emulation
- Automatic boot-time scanning and reconnection
- Multi-camera action coordination with sequential wake queue
- Optional external hardware buttons (GPIO26, GPIO21, GPIO22)
- Supported cameras: Action 4, Action 5 Pro, Action 6, Osmo 360

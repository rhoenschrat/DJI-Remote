# Release Notes

## DJI-Remote v1.2.0

This release adds the Waveshare ESP32-S3-LCD-1.9 as a second supported hardware
target and switches to a web-flash-only release model.

### What's New in v1.2.0

- **Waveshare ESP32-S3-LCD-1.9 support** — New HAL, ESP32-S3 target,
  320x170 landscape IPS display (ST7789V2).
- **Dual-target release builds** — Two merged binaries produced per release,
  one per hardware target.
- **Web-flash only** — Manual flash ZIP and scripts removed from release
  artifacts and documentation.
- **UI layout for 320x170** — Adaptive layout system supporting both
  320x240 (M5Stack) and 320x170 (Waveshare) screen resolutions.
- **GPS Kconfig** — GPS UART pins and baud rate configurable per board
  via Kconfig (no hardcoded pin assignments).

### Flash Instructions

Flash via [espflash.app](https://espflash.app) (Chrome or Edge required).
Download the binary for your hardware target from the
[latest release](https://github.com/rhoenschrat/DJI-Remote/releases/latest):

- `dji-remote-v1.2.0-m5stack-basic-v27.bin` — M5Stack Basic V2.7
- `dji-remote-v1.2.0-waveshare-s3-lcd19.bin` — Waveshare ESP32-S3-LCD-1.9

### Flash Settings — M5Stack Basic V2.7

| Setting         | Value  |
|-----------------|--------|
| Chip            | ESP32  |
| Flash mode      | DIO    |
| Flash size      | 16 MB  |
| Flash frequency | 80 MHz |

### Flash Settings — Waveshare ESP32-S3-LCD-1.9

| Setting         | Value    |
|-----------------|----------|
| Chip            | ESP32-S3 |
| Flash mode      | DIO      |
| Flash size      | 16 MB    |
| Flash frequency | 80 MHz   |

# Changelog

All notable changes to this project are documented in this file.

## [v1.3.0]

### Added

- **Per-slot liveness watchdog** — A connected camera that stops sending its
  status is marked unresponsive after about 6 s (yellow warning triangle) and
  disconnected after 15 s.
- **Per-slot command workers** — Button-triggered camera commands run in the
  background per camera; the remote never waits for a camera ("Busy" on a
  second press).
- **Fault diagnostics** — Connection faults and restarts are recorded in a
  dedicated `diag` flash partition and read over USB with `tools/diag/dji_diag`.
- **Long press A/B/C** — Start/stop recording on camera 1/2/3 (or wake it for
  a snapshot) without changing the selection; a paired, disconnected camera
  is connected instead.
- **Busy indicator (Waveshare)** — Per-slot cyan icon while an action runs.
- **Configurable slot cycle** — Button B cycles through four user-defined
  entries (single slot, slot combination or Empty), set up on the new Slots
  screens and stored in NVS. Replaces the fixed "All Cameras" entry.
- **Row background warning** — Low SD card (<= 16 GB / <= 6 GB) and battery
  (<= 25 % / <= 10 %) values color the whole status row yellow or red. The
  Kconfig choice `UI_STATUS_WARN_STYLE` keeps the previous text tint as an
  option.

### Changed

- **Per-camera link handling** — Link events are handled off the BLE host
  task; a disconnect only resets its own slot.
- **Connection parameters** — Explicit parameters with a 5 s supervision
  timeout; the boot scan ends about 10 s after the first camera was found.
- **GPS push time** — The camera GPS push carries the UTC date and time from
  the GPS module instead of the unset system clock.
- **Button input** — Non-blocking per-button state machine with debouncing;
  external M5Stack buttons behave like the built-in ones.
- **Waveshare 320x170 layout** — Adapted to the housing buttons (from the
  unreleased v1.2.1); Button C = Back in all submenus on all targets.
- **GPS icon** — Replaced with a map pin (Material Symbols `location_on`).
- **Build** — Project-local ESP-IDF setup with the `idf` wrapper.

### Fixed

- **Main screen flicker** — The main screen is populated before it is loaded.

## [v1.2.0]

### Added

- **Waveshare ESP32-S3-LCD-1.9 support** — New HAL, ESP32-S3 target,
  320x170 landscape IPS display (ST7789V2).
- **Dual-target release builds** — Two merged binaries produced per release,
  one per hardware target.
- **UI layout for 320x170** — Adaptive layout system supporting both
  320x240 (M5Stack) and 320x170 (Waveshare) screen resolutions.
- **GPS Kconfig** — GPS UART pins and baud rate configurable per board
  via Kconfig (no hardcoded pin assignments).

### Removed

- **Manual flash ZIP and scripts** — Web-flash only from this release onward.

## [v1.1.0]

### Added

- **LVGL 9.5.0 UI rendering** — All screens replaced with LVGL widget-based
  rendering via esp_lvgl_port 2.7.2. Replaces manual TFT/SPI drawing.
- **NimBLE BLE stack** — Apache NimBLE replaces Bluedroid as the GATT client
  stack.
- **Boot splash screen** — LVGL-rendered splash screen replaces the raw bitmap
  boot logo.
- **Release packaging automation** — GitHub Actions workflow and
  `build_release.sh` produce merged binary and ZIP for every tagged release.

### Fixed

- **NimBLE scan fixes** — Device names now shown during scan; duplicate filter
  disabled; reconnect-on-unpair prevented.

### Changed

- **Production log level** — Default log level set to ERROR for production
  builds.

## [v1.0.0]

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

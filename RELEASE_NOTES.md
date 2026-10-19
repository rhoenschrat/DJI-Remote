# Release Notes

## DJI-Remote v1.3.0 (in preparation)

Stability release: each camera connection is handled independently, so one
camera with a problem no longer affects the others or the remote.

### What's New in v1.3.0

- **Per-camera link loss** — A camera that drops its connection only resets
  its own slot; the other cameras keep working. Reconnect it with Button A.
- **Unresponsive cameras are detected** — A camera that stays connected but
  stops sending status shows a yellow warning triangle after about 6 s and
  is disconnected after 15 s.
- **The remote never waits for a camera** — Commands run in the background
  per camera. Pressing again before the camera answered shows "Busy".
- **More tolerant connections** — Short radio dropouts (up to about 5 s) no
  longer close the connection.
- **Faster boot scan** — If a paired camera is switched off, the boot scan
  ends about 10 s after the first camera is found instead of after 30 s.
- **Fault diagnostics** — The remote records connection faults and restarts
  in its own flash area, also without a computer attached. Read them over
  USB after a tour with `tools/diag/dji_diag` (see `tools/diag/README.md`).
- **Real GPS date and time for the cameras** — The GPS data sent to the
  cameras now carries the UTC date and time from the GPS module. Before, the
  remote sent the time of its own clock, which was never set (1970). Note:
  the cameras store this GPS time one hour earlier than UTC in the clip
  metadata; the time in DJI Mimo comes from the camera clock and is correct.

### Update Notes

- This update adds a new flash partition for the diagnostics. Flashing the
  merged binary updates the partition table automatically. **Pairing data is
  kept.**

## DJI-Remote v1.2.1

Maintenance release for the Waveshare ESP32-S3-LCD-1.9 target. The 320x170 UI
preset was adapted to the physical button positions of the 3D-printed housing.
Submenu navigation was unified on all targets.

### What's New in v1.2.1

- **Reworked bottom row (320x170)** — Element order is now GPS icon (compact,
  far left) followed by three equally sized button slots filling the remaining
  width.
- **M5Stack-style buttons on Waveshare** — Buttons A/B/C now render with icon
  and text label, white on black, matching the 320x240 layout. This applies to
  the main screen and to the Settings, Pairing and Mode Switch screens; the
  colored button column on the right edge is gone.
- **Notification area removed (320x170)** — The center notification field of
  the bottom row was dropped to make room for the button slots. The Pairing
  screen shows "Scanning..." in the list area instead.
- **Two-column lists (320x170)** — The Pairing list holds up to 8 cameras in
  two columns; Settings shows all options in one full-width column.
- **Button C = Back (all targets)** — "Back" is no longer a list entry. Press
  Button C to leave Settings, Pairing (also stops the scan) and Mode Switch.
  On Mode Switch, Button B has no function.
- **Long names are shortened with "…"** in all submenu lists.

The 320x240 M5Stack Basic V2.7 main screen is unchanged.

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

- `dji-remote-v1.2.1-m5stack-basic-v27.bin` — M5Stack Basic V2.7
- `dji-remote-v1.2.1-waveshare-s3-lcd19.bin` — Waveshare ESP32-S3-LCD-1.9

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

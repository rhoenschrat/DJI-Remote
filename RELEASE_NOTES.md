# Release Notes

## DJI-Remote v1.3.0

Stability and usability release: each camera connection is handled
independently, so one camera with a problem no longer affects the others or
the remote. The camera selection can be configured, and low SD card and
battery values are easier to see. This release also contains the Waveshare
housing adaptation that was prepared as v1.2.1 (see below).

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
- **Buttons no longer block the remote** — While a button is held, the
  display, the recording timer and the camera connections keep running.
  Short electrical glitches are filtered out. External buttons on the M5Stack
  (G26/G21/G22) now behave exactly like the built-in buttons, without the
  previous 1 s cooldown.
- **Long press: record on one camera** — On the main screen, holding Button
  A, B or C for 1 s starts or stops recording on camera 1, 2 or 3 (or
  wakes it for a snapshot) without changing the selection. Ideal for
  controlling a single camera while several cameras are selected. A paired
  camera that is not connected is connected instead. Because of this, short
  presses on the main screen now react when the button is released.
- **Busy indicator (Waveshare)** — A cyan icon on the camera's block (and in
  its Options menu) shows that an action is running: connect, start/stop
  recording, photo, sleep, wake, snapshot, disconnect, unpair. It disappears
  when the camera shows the result, on an error, or after 10 s at the latest.
- **Configurable camera selection (Slots)** — Button B cycles through four
  entries that you can set up yourself under **Slots** (Options Menu, or
  Button B during the boot scan). Each entry is one camera, a combination
  such as **Slot 1 + 3**, or Empty. The factory cycle Slot 1 → Slot 2 →
  Slot 3 → Slot 1 + 2 + 3 matches the previous behavior; "All Cameras" is
  now the entry **Slot 1 + 2 + 3**. Multi-camera actions only affect the
  cameras of the selected entry. The cycle is kept after a restart.
- **Low SD card and battery: colored row** — When the free SD card space or
  the battery level gets low, the whole row turns yellow (16 GB / 25 % or
  less) or red (6 GB / 10 % or less) instead of only the text, so it is
  visible at a glance.
- **New GPS icon** — The GPS status is shown as a map pin.

### Update Notes

- This update adds a new flash partition for the diagnostics. Flashing the
  merged binary updates the partition table automatically. **Pairing data is
  kept.**

## DJI-Remote v1.2.1 (not released separately, included in v1.3.0)

Maintenance update for the Waveshare ESP32-S3-LCD-1.9 target. The 320x170 UI
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

- `dji-remote-v1.3.0-m5stack-basic-v27.bin` — M5Stack Basic V2.7
- `dji-remote-v1.3.0-waveshare-s3-lcd19.bin` — Waveshare ESP32-S3-LCD-1.9

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

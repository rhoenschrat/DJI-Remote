# DJI-Remote – Implementation Overview

This document provides a developer-focused overview of the DJI-Remote firmware:
architecture, hardware targets, UI system, BLE/GPS logic, and protocol details
as the project exists today.

---

# 1. High-Level Summary

DJI-Remote is an ESP32/ESP32-S3 remote controller that controls up to **three
DJI Osmo Action cameras** simultaneously over BLE. It provides **GPS injection**,
**recording control**, **mode switching**, **sleep/wake**, and a multi-resolution
LVGL-rendered UI.

Main components:

- **Multi-target HAL** for two supported boards (M5Stack Basic v2.7, Waveshare ESP32-S3-LCD-1.9)
- **LVGL UI subsystem** with four screens and a multi-resolution layout system
- **BLE GATT client** using the NimBLE stack
- **GPS reader**, NMEA parser, and per-slot GPS push
- **Camera state & notification parsing** (DJI protocols 1D02 and 1D06)
- **Startup scanning & reconnect logic**
- **Optional external shutter/buttons**

---

# 2. Hardware Architecture

## 2.1 Supported Boards

| Property | M5Stack Basic v2.7 | Waveshare ESP32-S3-LCD-1.9 |
|----------|--------------------|----------------------------|
| MCU | ESP32 (Xtensa LX6, dual-core) | ESP32-S3 (Xtensa LX7, dual-core) |
| Display | 320×240 TFT (ST7789) | 320×170 TFT (ST7789V2, landscape) |
| Buttons | A (GPIO39), B (GPIO38), C (GPIO37) | A (GPIO5), B (GPIO6), C (GPIO16) |
| GPS | M5Stack GPS v2.0 (UART2, 115200) | ATGM336H (UART1, 9600) |
| Backlight | GPIO32 (active-high) | GPIO14 (active-low) |
| PSRAM | None | 8 MB OPI |
| Flash | 16 MB | 16 MB |

The target board is selected at compile time via `CONFIG_BOARD_*` in Kconfig.
Each board has its own `sdkconfig.defaults.<board>` file.

## 2.2 External Buttons (Compile-Time Optional)

Controlled by compile-time flag `UI_ENABLE_EXTERNAL_BUTTONS` in `main/ui.c`.

| Function | GPIO | Notes |
|----------|------|-------|
| External Button A (Shutter) | GPIO26 | Internal pull-up |
| External Button B | GPIO21 | External 10k pull-up required |
| External Button C | GPIO22 | External 10k pull-up required |

When enabled, these behave identically to internal Buttons A/B/C on all screens.

## 2.3 Hardware Abstraction Layer (HAL)

Each supported board has its own HAL implementation file. Only the active
board's HAL is compiled.

| File | Board |
|------|-------|
| `main/m5stack_basic_v27_hal.c/.h` | M5Stack Basic v2.7 (ESP32) |
| `main/waveshare_s3_lcd19_hal.c/.h` | Waveshare ESP32-S3-LCD-1.9 (ESP32-S3) |

Both HALs expose the same API: display init, LVGL registration, backlight
control, and button polling. `app_main.c` uses `HAL_*` preprocessor macros
to call the active HAL without `#ifdef` chains throughout the code.

All UI rendering goes through LVGL. The HAL is responsible only for SPI bus
setup, ST7789 panel initialization, and LVGL buffer/flush registration.

---

# 3. Firmware Structure

```
main/
 ├── app_main.c                    → Init, main loop, subsystem startup
 ├── ui.c/.h                       → UI state machine, button dispatch, screen switching
 ├── ui_layout.c/.h                → Multi-resolution layout system (320×240, 320×170, 240×135)
 ├── ui_screen_main.c/.h           → Main screen LVGL widget tree
 ├── ui_screen_pairing.c/.h        → Pairing screen LVGL widget tree
 ├── ui_screen_settings.c/.h       → Settings screen LVGL widget tree
 ├── ui_screen_mode_switch.c/.h    → Mode Switch screen LVGL widget tree
 ├── ui_screen_splash.c/.h         → Boot splash screen (LVGL)
 ├── lvgl_icons.c/.h               → LVGL image descriptors (1-bit icons)
 ├── splash_logo.c/.h              → Splash logo image data
 ├── m5stack_basic_v27_hal.c/.h    → M5Stack HAL (ESP32)
 ├── waveshare_s3_lcd19_hal.c/.h   → Waveshare HAL (ESP32-S3)
 ├── icons.c/.h                    → 1-bit packed icon bitmaps (source data)
 └── Kconfig.projbuild             → Board selector + GPS configuration

logic/
 ├── connect_logic.c/.h            → BLE scanning & connect/reconnect logic
 ├── status_logic.c/.h             → Camera state updates, sleep/wake, snapshot logic
 ├── command_logic.c/.h            → Camera command execution
 ├── enums_logic.c/.h              → Camera state enumerations
 └── light_logic.c/.h              → RGB LED status indication

ble/
 └── ble.c/.h                      → BLE GATT client (NimBLE stack)

gps/
 └── gps_reader.c/.h               → GPS UART reader, NMEA parser, GPS push

data/
 └── data.c/.h                     → BLE notification dispatcher task

protocol/
 ├── dji_protocol_parser.c/.h          → DJI protocol frame parsing
 ├── dji_protocol_data_structures.c/.h → Protocol data type definitions
 ├── dji_protocol_data_processor.c/.h  → Protocol data processing
 └── dji_protocol_data_descriptors.c/.h → Protocol field descriptors

utils/crc/
 ├── custom_crc16.c/.h             → CRC16 for DJI protocol
 └── custom_crc32.c/.h             → CRC32 for DJI protocol

log_config.c/.h                    → Central logging configuration
```

---

# 4. User Interface System

## 4.1 LVGL Rendering

All screen content is rendered via **LVGL 9**. LVGL automatically tracks dirty
regions and only repaints changed pixels — no manual redraw logic is needed.
The `esp_lvgl_port` component creates a dedicated FreeRTOS task that calls
`lv_timer_handler()` and flushes dirty regions to the ST7789 via SPI DMA.

All LVGL API calls from the main task are wrapped in
`lvgl_port_lock(0)` / `lvgl_port_unlock()` for thread safety.

## 4.2 Screen Modules

Each screen follows a `create()` / `update()` / `destroy()` lifecycle:

| Screen | Module | Trigger |
|--------|--------|---------|
| Splash | `ui_screen_splash.c` | Boot (before Main Screen) |
| Main Shutter | `ui_screen_main.c` | Boot |
| Pairing | `ui_screen_pairing.c` | Button on unpaired slot |
| Settings | `ui_screen_settings.c` | Button C on paired slot |
| Mode Switch | `ui_screen_mode_switch.c` | Settings → Mode Switch |

`create()` builds the LVGL widget tree. `update()` syncs widget content with
`g_camera_states[]`. `destroy()` deletes the screen and nulls widget pointers.
`ui_switch_screen()` destroys the old screen and creates the new one atomically.

## 4.3 Multi-Resolution Layout System

All screen modules read positions and sizes from `ui_layout_get()` rather than
hardcoded constants. Three presets are available, selected by display resolution
at startup:

| Preset | Board | Camera columns | Button style |
|--------|-------|----------------|--------------|
| 320×240 | M5Stack Basic v2.7 | 3 | Wide buttons with text labels |
| 320×170 | Waveshare ESP32-S3-LCD-1.9 | 3 | Compact colored squares, icon only |
| 240×135 | M5StickC Plus2 | 1 (cycles with Button B) | Compact |

The 320×170 preset omits Camera Mode Icon and Video Mode Text rows (no vertical
space). These widgets are set to `NULL` and guarded in all update paths.

### 320×170 Button Style

On the Waveshare board, buttons are 27×27 colored squares:
- Button A: Red `rgb(255, 0, 0)`
- Button B: Yellow `rgb(255, 255, 0)`
- Button C: Blue `rgb(0, 0, 255)`
- Icons: 24×24, centered at offset (2, 2), color **Black**

On submenu screens, buttons are grouped as a vertical column at the right edge.
Button B's `next_icon` (right-pointing arrow) is rotated 90° CW on all targets
to indicate "move selection down".

## 4.4 Main Screen Camera Block

Each camera block contains (top to bottom):

- **Selection indicator** (97×4 px; red = selected, black = not selected)
- **Title** — camera model name
- **Camera Mode Icon** — photo or video (320×240 only)
- **Video Mode Text** — resolution/fps/EIS or mode_name/mode_param (320×240 only)
- **Status icon** — record/pause/sleep/connecting/bluetooth/found
- **Recording timer** — `hh:mm:ss`
- **SD card** — icon + capacity text with threshold colors
- **Battery** — icon + percentage with threshold colors

## 4.5 Button Area Modes

| Mode | Condition | Visible |
|------|-----------|---------|
| `SCAN_ONLY_C` | Boot scan active | C only ("Stop") |
| `BOOT_CONNECT_HIDDEN` | Connecting after scan | None |
| `NORMAL` | Standard operation | A, B, C with dynamic labels |

Button A/B/C dynamic mapping in NORMAL mode:

- **A:** Pair / Connect / Record / Pause / Snapsht (state-dependent)
- **B:** Next (cycle selection)
- **C:** Highlight / Options / Sleep / Wake

---

# 5. Camera State System

Each slot (0–2) maintains independently:

- Pairing info (MAC address, device_id, model name)
- Connection state: not paired / paired / found / connecting / connected
- Camera state: mode, recording, paused, sleeping
- Status data: SD capacity, battery, record_time, remain_time
- Protocol capability flag: `camera_supports_new_status_push`

## 5.1 Sleep Mode

Derived from 1D02 `power_mode == 3`.

- Commands blocked (except wake)
- GPS push paused for that slot
- UI shows `sleep_icon`
- Snapshot requests queued via `snapshot_pending` flag

## 5.2 Snapshot via Wake

Button A on a sleeping camera:

1. Set `snapshot_pending`
2. Send wake broadcast (3 seconds)
3. Camera wakes (1D02 power_mode `3 → 0`)
4. `status_logic.c` detects the transition and sends the snapshot key
5. In "All Cameras" mode: sleeping cameras enter a serialized wake queue

---

# 6. BLE Architecture

DJI-Remote uses the ESP32/ESP32-S3 as a **GATT client** with the **NimBLE** BLE
stack. NimBLE provides lower RAM/flash usage than Bluedroid and a unified GAP
event callback model.

## 6.1 Key NimBLE Details

- Single `gap_event_cb()` handles scan, connect, disconnect, notify, and MTU
- GATT discovery uses chained per-operation callbacks
- Scanning and connecting are mutually exclusive (scan cancelled before connect)
- Local GATT services (GAP 0x1800, GATT 0x1801) are registered via
  `ble_svc_gap_init()` / `ble_svc_gatt_init()` — required for cameras to
  perform reverse service discovery

## 6.2 Scan Modes

| Mode | Trigger | Behavior |
|------|---------|----------|
| Pairing scan | Pairing Screen | Lists all DJI cameras nearby |
| Autoconnect boot scan | Boot (if any slots paired) | Finds all paired cameras at once |
| Slot reconnect | Button A / Settings → Connect | Scans for a specific MAC |

No automatic background reconnection — all reconnects are user-triggered.

## 6.3 Wake Broadcast

Wake uses connectable undirected advertising with manufacturer-specific data:
`WKP` + reversed camera MAC. Connectable advertising (`ADV_IND`) is required
because DJI cameras filter on PDU type when scanning for wake packets.

---

# 7. GPS Subsystem

- UART NMEA parser (board-specific port/baud via Kconfig)
- Valid fix: GPS icon shown in green (320×170) or yellow (320×240), coordinates displayed
- No fix: red icon, no text
- GPS push (10 Hz) sent to all connected and awake cameras
- Push pauses when a camera sleeps and resumes on wake

---

# 8. Camera Status Protocols

## 8.1 1D02 – Primary Status Push

Fields: `camera_mode`, `camera_status`, `video_resolution`, `fps_idx`,
`eis_mode`, SD capacity, battery %, `record_time`, `remain_time`, `power_mode`.

Triggers updates for: mode icon, video mode text (if 1D06 not supported),
status icon, recording timer, SD/battery.

## 8.2 1D06 – Enhanced Mode Push

Fields: `mode_name` (21 bytes), `mode_param` (21 bytes).

Used for newer cameras (Action 5 Pro, Action 6, Osmo 360). The capability flag
`camera_supports_new_status_push` is set when the first valid 1D06 frame arrives
and controls the video mode text rendering path.

---

# 9. FreeRTOS Task Architecture

| Task | Purpose | Stack |
|------|---------|-------|
| Main loop (`app_main`) | Button polling, UI updates, state dispatch | 8192 bytes |
| LVGL timer (`esp_lvgl_port`) | `lv_timer_handler()`, rendering, DMA flush | 8192 bytes |
| Notification processing (`data.c`) | BLE notification parsing and routing | 4096 bytes |
| GPS reader (`gps_reader.c`) | UART NMEA parsing, GPS push | 4096 bytes |
| NimBLE host (`ble.c`) | GATT client event handling | NimBLE default |
| FreeRTOS Timer Service | LED timer callbacks, NVS cleanup | 4096 bytes |

The FreeRTOS Timer Service task stack is set to 4096 bytes in
`sdkconfig.defaults` (`CONFIG_FREERTOS_TIMER_TASK_STACK_SIZE`). The default
of 2048 bytes is insufficient when timer callbacks use `ESP_LOGI` with format
strings.

---

# 10. Boot Sequence

1. HAL initializes SPI bus, ST7789 panel (backlight OFF)
2. LVGL registers the panel via `esp_lvgl_port`
3. Splash screen is created and rendered by `ui_screen_splash`
4. Backlight turns on after LVGL completes the first splash frame (no white-flash)
5. Autoconnect boot scan runs (if any slots paired)
6. `ui_init()` creates and loads the Main Shutter Screen

---

# 11. Icons

Icons are 24×24, 1-bit packed bitmaps stored in `icons.c`. `lvgl_icons_init()`
converts them to `LV_COLOR_FORMAT_A1` (alpha-only) at startup by inverting
each byte. Icon color is set per-widget via LVGL's image recolor API.

Color rules:

| State | Color |
|-------|-------|
| Recording | Red |
| GPS valid fix | Yellow (320×240) / Green (320×170) |
| Sleeping | White |
| Paired, disconnected | Blue (bluetooth icon) |
| Not paired | White (bluetooth icon) |
| Found in boot scan | Green (found icon) |
| Connecting | Blue |

Battery and SD card icons follow threshold-based color logic.

---

# 12. Logging

- All logs use `ESP_LOG*` macros
- Default log level set at build time via `APP_DEFAULT_LOG_LEVEL` in `log_config.h`
- Runtime log level adjustable per-tag via `app_log_set_level()`

---

# 13. Developer Notes

- All LVGL calls outside the LVGL task must be wrapped in `lvgl_port_lock(0)` / `lvgl_port_unlock()`
- Capture `g_camera_states[]` to local variables before updating widgets (TOCTOU prevention)
- BLE/scan logic is slot-based — every feature must work for all three slots independently
- No background reconnection; reconnect is always user-triggered
- Board-specific code stays in the HAL; UI code must never call raw TFT/SPI functions
- When adding a new board: create `<board>_hal.c/.h`, add Kconfig choice, add `sdkconfig.defaults.<board>`, add `HAL_*` macros in `app_main.c`

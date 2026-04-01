# DJI-Remote – Implementation Overview

This document provides a structured, clean, and developer‑focused overview of the DJI‑Remote firmware.  
It is intended for contributors who want to understand the architecture, hardware interaction, UI state
machine, and BLE/GPS logic *as the project exists today*.  


# 1. High‑Level Summary

DJI‑Remote is an ESP32‑based remote controller running on the **M5Stack Basic v2.7**.  
It controls up to **three DJI Osmo Action cameras** simultaneously over BLE and provides  
**GPS injection**, **recording control**, **mode switching**, **sleep/wake**, and a compact but highly‑informative UI.

Main components:

- **BLE connection & command logic** for DJI cameras  
- **GPS reader**, NMEA parser, and GPS‑push system  
- **UI subsystem** using LVGL 9.5.0 with widget-based rendering and automatic dirty-region tracking
- **HAL layer** for display, buttons, and board‑specific hardware  
- **Camera state & notification parsing** (DJI protocols 1D02 and 1D06)  
- **Startup scanning & reconnect logic**  
- **Optional external shutter/buttons**


# 2. Hardware Architecture

## 2.1 Target Board

- **Board:** M5Stack Basic v2.7  
- **CPU:** ESP32 dual‑core  
- **Display:** 320×240 TFT (ST7789)  
- **Internal Buttons:** A, B, C  
- **External GPS:** M5Stack Module GPS v2.0 (UART, NMEA)  

## 2.2 External Buttons (Compile‑Time Optional)

Controlled by compile‑time flag `UI_ENABLE_EXTERNAL_BUTTONS`.

| Function | GPIO | Notes |
|---------|------|-------|
| External Button A (Shutter) | GPIO26 | Internal pull‑up, falling edge interrupt |
| External Button B | GPIO21 | External 10k pull‑up, falling edge interrupt |
| External Button C | GPIO22 | External 10k pull‑up, falling edge interrupt |

When enabled, these behave **identically** to internal Buttons A/B/C across all screens.

## 2.3 Hardware Abstraction Layer (HAL)

The HAL initializes the display (ST7789 via SPI), configures the backlight,
registers hardware buttons, and registers the display and input drivers with
the LVGL `esp_lvgl_port`. All rendering goes through LVGL widgets; the HAL
does not expose drawing functions to UI code.


# 3. Firmware Structure

```
main/
 ├── app_main.c                    → Init, main loop, subsystem startup
 ├── ui.c/.h                       → UI router — delegates to LVGL screen modules
 ├── ui_layout.c/.h                → Centralized layout presets (320x240, 240x135)
 ├── ui_screen_main.c/.h           → Main shutter screen (LVGL)
 ├── ui_screen_pairing.c/.h        → Pairing screen (LVGL)
 ├── ui_screen_settings.c/.h       → Settings screen (LVGL)
 ├── ui_screen_mode_switch.c/.h    → Mode switch screen (LVGL)
 ├── ui_screen_splash.c/.h         → Boot splash screen (LVGL)
 ├── lvgl_icons.c/.h               → LVGL A8 image descriptors for all icons
 ├── splash_logo.c/.h              → Splash logo image data
 └── m5stack_basic_v27_hal.c/.h    → M5Stack display, buttons, LVGL port registration

logic/
 ├── connect_logic.c/.h            → BLE scanning & connect/reconnect logic
 ├── status_logic.c/.h             → Camera state updates, sleep/wake, snapshot logic
 ├── command_logic.c/.h            → Camera command execution
 ├── enums_logic.c/.h              → Camera state enumerations
 └── light_logic.c/.h              → RGB LED status indication

ble/
 └── ble.c/.h                      → BLE GATT client implementation

gps/
 └── gps_reader.c/.h               → GPS UART reader, NMEA parser, GPS push

data/
 └── data.c/.h                     → Notification dispatcher task

protocol/
 ├── dji_protocol_parser.c/.h      → DJI protocol parsing
 ├── dji_protocol_data_structures.c/.h → Protocol data structures
 ├── dji_protocol_data_processor.c/.h  → Protocol data processing
 └── dji_protocol_data_descriptors.c/.h → Protocol descriptors

log_config.c/.h                   → Central logging system (root level)
```


# 4. User Interface

All UI elements are LVGL widgets (labels, images, containers). LVGL tracks
dirty regions automatically; the firmware does not manage redraws. Screen
modules follow a `create()` / `update()` / `destroy()` pattern. All `lv_*`
calls from non-LVGL tasks must be wrapped in `lvgl_port_lock(0)` /
`lvgl_port_unlock()`.

## 4.2 Camera Block Structure

Each camera block contains:

- Selection indicator  
- Title (model name)  
- Mode icon (photo/video)  
- Video mode text (based on 1D02 or 1D06)  
- Status icon (record, pause, sleep, connecting, found, bluetooth)  
- Recording timer  
- SD card info  
- Battery info  

LVGL automatically tracks which fields need redrawing.

## 4.3 Buttons Area

Buttons A/B/C show **icons + labels** reflecting current action:

- A: Pair / Connect / Record / Pause  
- B: Next (cycle selection)  
- C: Options / Highlight / Sleep / Wake (priority logic applies)

During boot scan:

- Only Button C is visible (“Stop”)
- During connection phase all buttons are hidden
- When boot completes, the normal mapping activates


# 5. Camera State System

Each slot (0–2) maintains:

- paired / connected / found / connecting  
- recording / paused  
- sleep state  
- SD capacity  
- battery percentage  
- mode (photo/video)  
- protocol capabilities (1D06 support)

## 5.1 Sleep Mode

Derived from 1D02 `power_mode == 3`.

Rules:

- Commands (except wake) are blocked while sleeping  
- GPS push stops  
- UI shows sleep icon  
- Snapshot requests queue until wake  

## 5.2 Snapshot via Wake

Button A on a sleeping camera:

1. Mark `snapshot_pending`  
2. Send wake broadcast (3 seconds)  
3. When camera wakes via 1D02 transition `3 → 0`  
   → A single snapshot key command is sent  
4. Supports multi‑camera wake queue in “All Cameras” mode

Race‑free by centralizing snapshot sending inside `status_logic.c`.


# 6. BLE Architecture

The BLE stack is **Apache NimBLE** (included via ESP-IDF). The firmware acts
as a **GATT client**.

## 6.1 Scan Modes

### **Pairing Scan**
- Triggered in Pairing Screen  
- Lists all DJI cameras nearby  
- User chooses one to pair

### **Autoconnect Boot Scan**
- Runs once at boot if any slots are paired  
- Searches for all paired cameras at once  
- Stops early if all are found  
- After scan, uses **direct connect** (no new scan)

### **Slot Reconnect Scan**
- Triggered manually by the user (Button A or Settings → Connect)  
- Scans only for the specific MAC of the selected slot  

## 6.2 Connection States

- Not paired  
- Paired, not connected  
- Discovered (boot scan found)  
- Connecting  
- Connected  
- Sleeping  

The UI reflects these precisely with dedicated icons.


# 7. GPS Subsystem

- UART NMEA parser  
- Once valid fix is available:
  - Yellow GPS icon  
  - Coordinates displayed  
- No fix:
  - Red icon  
  - No text  

GPS push (10 Hz) is sent to all **connected and awake** cameras.

State transitions:

- If camera sleeps → GPS push stops  
- When waking → GPS push resumes automatically

UI updates only the GPS region and only when values change.


# 8. Screens

## 8.1 Main Screen

The primary operating interface.

Selection model:

- Camera 0 → 1 → 2 → All Cameras → …

All rendering is handled by LVGL widgets.

## 8.2 Pairing Screen

Unified submenu layout:

- Title  
- List of discovered cameras  
- Button A = Select  
- Button B = Next  
- Button C = none  
- Notification area used for pairing progress  

## 8.3 Settings Screen

Entries:

1. Back  
2. Mode Switch  
3. Sleep/Wakeup  
4. Connect/Disconnect  
5. Unpair  

Follows same submenu layout.

## 8.4 Mode Switch Screen

Displays:

- Camera Mode icon  
- Video Mode text  

Button A simulates QS button press to cycle modes.

Live updates handled via 1D02 / 1D06 notifications.


# 9. Icons & Graphics

Icons are stored as LVGL A8 (alpha-8) image descriptors in `lvgl_icons.h`
and rendered via `lv_image_set_src()`. Color is applied at render time using
LVGL styles. Detailed color rules are documented in `docs/ui/UI-SPEC.md`.


# 10. State Update & Redraw Rules

## 10.1 Flicker Avoidance

LVGL handles dirty-region detection and partial screen updates automatically.
The firmware does not need manual redraw coordination; setting a widget
property to its current value is a no-op.

## 10.2 Concurrency Safety

All LVGL widget updates must be wrapped in `lvgl_port_lock(0)` /
`lvgl_port_unlock()`. Capturing state into local variables before entering
the lock is recommended to keep lock sections short and avoid TOCTOU races
with the notification task.


# 11. Camera Status Protocols

## 11.1 1D02 – Primary Status Push

Includes:

- camera_mode  
- camera_status  
- video_resolution  
- fps_idx  
- eis_mode  
- SD capacity  
- battery %  
- record_time / remain_time  
- power_mode  

Triggers updates for:

- Mode icon  
- Video mode (when 1D06 not supported)  
- Status icon  
- Timer  
- SD / Battery  

## 11.2 1D06 – Enhanced Mode Push

Contains:

- mode_name (21 bytes)  
- mode_param (21 bytes)  

Used for newer cameras:

- Action 5 Pro  
- Action 6  
- Osmo 360  

UI center‑prints mode_name and mode_param.

## 11.3 Capability Flag

`camera_supports_new_status_push` is set when first valid 1D06 frame is received.

Controls video mode rendering path.


# 12. Logging System

Central logging:

- `app_log_set_level()`  
- Build‑time default via `APP_DEFAULT_LOG_LEVEL`  
- All logs use ESP‑IDF `ESP_LOG*` macros  

GPS module also uses this system; logging can be fully disabled.


# 13. Boot Behavior

- Display initializes  
- The splash screen is an LVGL screen rendered by `ui_screen_splash`. The HAL enables the backlight after LVGL completes the first splash frame.
- Autoconnect boot scan may run  
- Buttons area shows special temporary state  
- After connections complete, normal UI resumes  


# 14. Developer Notes

- HAL abstraction is mandatory  
- All UI changes must use LVGL widgets and respect thread safety (`lvgl_port_lock`/`unlock`)
- BLE/scanning logic is slot‑based and deterministic  
- Multi‑camera operations must treat slots independently  
- No background reconnection; user‑triggered reconnect only  

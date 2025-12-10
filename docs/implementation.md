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
- **UI subsystem** with partial redraw logic to avoid flicker  
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

Platform implements drawing primitives:

- `hal_fill_rect()`
- `hal_draw_bitmap()`
- `hal_draw_text()`
- backlight and display‑init routines  
- no direct TFT calls in UI code

A clean separation ensures portability and predictable rendering timing.


# 3. Firmware Structure

```
main/
 ├── app_main.c                    → Init, main loop, subsystem startup
 ├── ui.c/.h                       → All UI rendering & state machine
 ├── m5stack_basic_v27_hal.c/.h    → M5Stack display, buttons, drawing
 └── icons.h/.c                    → Packed 1‑bit icon bitmaps

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

The UI avoids full‑screen redraws. Everything is drawn in **sections**, and updates only happen when needed.

## 4.1 Screen Layout

Resolution: **320×240**

Sections:

1. **Three Camera Blocks** (0–2)
2. **Remote Status Area**  
   - Notification Area  
   - GPS Status Area  
3. **Buttons Area** (A/B/C, dynamic icons & labels)

Only the affected sub‑region is redrawn when state changes.

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

Every section has independent redraw triggers and tracking fields.

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

DJI‑Remote uses the ESP32 as a **GATT client**.

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

All rendering is partial and flicker‑free.

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

Icons are 1‑bit packed arrays in `icons.h`.  
Rendering rules:

- Recording → red  
- GPS fix → yellow  
- Sleep → white  
- Bluetooth (found/paired) → blue/green  

Battery & SD icons follow threshold‑based color logic.


# 10. State Update & Redraw Rules

## 10.1 Flicker Avoidance

- Never redraw full screen  
- Never redraw full camera block unless required  
- Always clear/redraw only the minimal region  
- Use per‑field tracking variables to detect changes  
- Insert small delays (`vTaskDelay(1)`) when drawing multiple regions quickly (ST7789 timing requirement)

## 10.2 Concurrency Safety

UI must capture state values into **local copies** before drawing to avoid race conditions with the status task.


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
- Boot logo drawn completely before enabling backlight  
- Autoconnect boot scan may run  
- Buttons area shows special temporary state  
- After connections complete, normal UI resumes  


# 14. Developer Notes

- HAL abstraction is mandatory  
- All UI changes must follow partial redraw rules  
- BLE/scanning logic is slot‑based and deterministic  
- Multi‑camera operations must treat slots independently  
- No background reconnection; user‑triggered reconnect only  

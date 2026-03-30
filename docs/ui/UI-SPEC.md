# DJI-Remote – UI Layout Specification

> Authoritative pixel-level layout reference for all screens.
> The source of truth for layout values is `main/ui_layout.c` (presets) and the
> individual screen modules (`main/ui_screen_*.c`). All positions are LVGL
> widget coordinates set via `lv_obj_set_pos()` and read from the centralized
> `ui_layout_t` struct (see `main/ui_layout.h`).

**Display:** 320x240 TFT (ST7789), landscape, origin top-left `(0,0)`.

**Rendering:** All UI elements are LVGL widgets. Positions listed below
correspond to LVGL absolute or parent-relative coordinates. LVGL handles
dirty-region tracking and partial redraws automatically.

**Multi-resolution:** Layout values are stored in `ui_layout.h/.c` with
presets for 320x240 (M5Stack Basic v2.7, 3-column) and 240x135 (M5StickC
Plus2, 1-column). The tables below document the 320x240 preset.

**Visual reference:** [main-screen-mockup.png](main-screen-mockup.png)
— mockup of the Main Shutter Screen layout.

---

## 1. Main Shutter Screen

### 1.1 Top-Level Regions

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Camera 0 Block | 10 | 10 | 100 | 178 |
| Camera 1 Block | 110 | 10 | 100 | 178 |
| Camera 2 Block | 210 | 10 | 100 | 178 |
| Remote Status | 10 | 192 | 300 | 24 |
| Buttons Area | 10 | 216 | 300 | 24 |

### 1.2 Remote Status Sub-Regions

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Notification Area | 10 | 192 | 150 | 24 |
| GPS Status Area | 160 | 192 | 150 | 24 |

**GPS Status Area contents:**
- Icon: `gps_icon` 24x24 at `(160, 192)`
- Text: `"lat, lon"` at `(194, 196)` — 2 decimal places
- Colors: Red icon (no fix), Yellow icon+text (valid fix)

**Notification Area contents:**
- Temporary text messages at `(10, 192)` — status feedback

### 1.3 Buttons Area

| Button | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Button A | 10 | 216 | 93 | 24 |
| Button B | 113 | 216 | 93 | 24 |
| Button C | 217 | 216 | 93 | 24 |

Each button contains:
- **Background:** Gray `rgb(128, 128, 128)`
- **Icon:** 24x24 at button origin `(x, y)`
- **Label:** at `(x+29, y)`, width=65, height=24, left-aligned, vertically centered
- **Default colors:** Icon and text black

**Button Area Display Modes:**

| Mode | Condition | Visible |
|------|-----------|---------|
| `BUTTONS_MODE_SCAN_ONLY_C` | Boot scan active | C only: `scanning_icon` + "Stop" |
| `BUTTONS_MODE_BOOT_CONNECT_HIDDEN` | Connecting after scan | None (black) |
| `BUTTONS_MODE_NORMAL` | Standard operation | All three with dynamic mapping |

---

## 2. Camera Block Layout

Each camera block uses a base coordinate (`base_x`, `base_y`) with fixed
offsets. The same layout is used for all three slots.

### 2.1 Sub-Section Offsets

| Sub-Section | X Offset | Y Offset | Width | Height |
|-------------|----------|----------|-------|--------|
| Selection Indicator | +1 | +1 | 98 | 4 |
| Title (Model Name) | +5 | +5 | 90 | 24 |
| Camera Mode Icon | +5 | +29 | 90 | 24 |
| Video Mode Text | +5 | +53 | 90 | 24 |
| Status Icon | +5 | +77 | 90 | 24 |
| Recording Timer | +5 | +101 | 90 | 24 |
| SD Card Status | +5 | +125 | 90 | 24 |
| Battery Status | +5 | +149 | 90 | 24 |

### 2.2 Selection Indicator

- Red fill = selected, Black fill = not selected
- In "All Cameras" mode: all three indicators are red

### 2.3 Title Area

- Model name from `device_id`: Action 4, Action 5, Action 6, Osmo 360
- Shown only when slot is paired (even if disconnected)
- Larger font, left-aligned

### 2.4 Camera Mode Icon

- 24x24 icon centered at `(base_x + 38, base_y + 29)`
- `camera_mode == 0x05` → `photo_icon`
- All other modes → `video_icon`
- **Always derived from 1D02** regardless of 1D06 support

### 2.5 Video Mode Text

Two-line text block, horizontally centered within 90px width:
- **Line 1** at y_offset +4 within area
- **Line 2** at y_offset +14 within area
- Font: regular (size 1), color: white

Source depends on `camera_supports_new_status_push`:
- **false (1D02):** resolution string + "fps EIS" string
- **true (1D06):** `mode_name` + `mode_param`

### 2.6 Status Icon

24x24 icon centered at `(base_x + 38, base_y + 77)`.

Priority order (first match wins):
1. Connected + paused → `pause_icon` (white)
2. Connected + recording → `record_icon` (red)
3. Connected + sleeping → `sleep_icon` (white)
4. Not paired → `bluetooth_icon` (white)
5. Connecting → `connecting_icon` (blue)
6. Found in boot scan → `found_icon` (green)
7. Paired but disconnected → `bluetooth_icon` (blue)

### 2.7 Recording Timer

- `hh:mm:ss` format, centered in area
- Recording → shows `record_time`
- Paused → shows `remain_time`
- Larger font

### 2.8 SD Card Status

- Icon: `sd_card_icon` 24x24 at `(base_x + 5, base_y + 125)`
- Text: capacity string at `(base_x + 34, base_y + 125)`, width=61
- Color thresholds: >16GB white, ≤16GB yellow, ≤6GB red

### 2.9 Battery Status

- Icon: threshold-based battery icon at `(base_x + 5, base_y + 149)`
  - >75%→`battery_100_icon`, ≤75%→`battery_75_icon`, ≤50%→`battery_50_icon`,
    ≤25%→`battery_25_icon`, ≤10%→`battery_0_icon`
- Text: `"NN %"` at `(base_x + 34, base_y + 149)`, width=61
- Color thresholds: >25% white, ≤25% yellow, ≤10% red

---

## 3. Submenu Screen Layout (Settings, Pairing)

Both Settings and Pairing screens share a unified layout.

### 3.1 Regions

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Base | 10 | 10 | 300 | 220 |
| Title | 10 | 10 | 300 | 24 |
| Entry 0 | 10 | 39 | 300 | 24 |
| Entry 1 | 10 | 68 | 300 | 24 |
| Entry 2 | 10 | 97 | 300 | 24 |
| Entry 3 | 10 | 126 | 300 | 24 |
| Entry 4 | 10 | 155 | 300 | 24 |
| Notification | 10 | 184 | 300 | 27 |
| Buttons | 10 | 216 | 300 | 24 |

Entry Y positions: `base_y + 29 + (index * 29)` → 39, 68, 97, 126, 155.

### 3.2 Entry Structure

| Element | X Offset | Width | Height |
|---------|----------|-------|--------|
| Selection Indicator | +0 | 10 | 24 |
| Icon | +15 | 24 | 24 |
| Label | +44 | 256 | 24 |

Indicator: Red if selected, Black if not.

**Selection change clearing area** (prevents artifacts):
- Position: `(entry_x - 5, entry_y - 5)`
- Size: 20 x 34
- Applied only to the previously selected entry

### 3.3 Submenu Buttons

- Button A: `select_icon` + "Select"
- Button B: `next_icon` + "Next"
- Button C: Empty (no function)

### 3.4 Title Text

- "Camera X" where X is 1-indexed (1, 2, 3)
- Larger font, horizontally and vertically centered

---

## 4. Settings Screen Entries

1. `back_icon` + "Back"
2. `switch_icon` + "Mode Switch"
3. Dynamic: `sleep_icon`/"Sleep" or `wakeup_icon`/"Wakeup"
4. Dynamic: `connect_icon`/"Connect" or `disconnect_icon`/"Disconnect"
5. Dynamic: `unpair_icon`/"Unpair Camera" or `pair_icon`/"Pair Camera"

---

## 5. Pairing Screen Entries

1. `back_icon` + "Back"
2-5. Discovered cameras: `photo_icon` + camera name

---

## 6. Mode Switch Screen

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Base | 10 | 10 | 300 | 220 |
| Title | 10 | 10 | 300 | 24 |
| Mode Icon | 10 | 39 | 300 | 24 |
| Mode Label | 10 | 68 | 300 | 143 |

- **Title:** "Camera X" (1-indexed), larger font, centered
- **Mode Icon:** Same logic as Camera Mode Area (photo/video from 1D02)
- **Mode Label:** Same logic as Video Mode Area (1D02 or 1D06 source), larger font

### 6.1 Mode Switch Buttons

- Button A: `switch_icon` + "Switch" → sends QS button key reporting
- Button B: `back_icon` + "Back" → returns to Main Screen
- Button C: Empty (no function)

---

## 7. Color Reference

### Default Colors

| Element | Color |
|---------|-------|
| Background | Black |
| Default icon | White |
| Default text | White |
| Button background | Gray `rgb(128, 128, 128)` |
| Button icon/text | Black |
| Selected indicator | Red |

### Dynamic Icon Colors

| State | Color |
|-------|-------|
| Recording | Red |
| Paired not connected | Blue (bluetooth_icon) |
| Not paired | White (bluetooth_icon) |
| Found in scan | Green (found_icon) |
| Connecting | Blue (connecting_icon) |
| GPS no fix | Red |
| GPS valid fix | Yellow |

### Threshold Colors (SD Card)

| Condition | Color |
|-----------|-------|
| >16 GB free | White |
| ≤16 GB free | Yellow |
| ≤6 GB free | Red |

### Threshold Colors (Battery)

| Condition | Color |
|-----------|-------|
| >25% | White |
| ≤25% | Yellow |
| ≤10% | Red |

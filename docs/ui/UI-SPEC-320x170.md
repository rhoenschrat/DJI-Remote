# DJI-Remote – UI Layout Specification (320×170)

> Pixel-level layout reference for the Waveshare ESP32-S3-LCD-1.9 display variant.
> The source of truth for layout values is `main/ui_layout.c` (preset `LAYOUT_320x170`) and
> the individual screen modules (`main/ui_screen_*.c`). All positions are LVGL
> widget coordinates set via `lv_obj_set_pos()` and read from the centralized
> `ui_layout_t` struct (see `main/ui_layout.h`).

**Display:** 320×170 TFT (ST7789V2), landscape, origin top-left `(0,0)`.

**Hardware:** Waveshare ESP32-S3-LCD-1.9 (ESP32-S3R8).

**Rendering:** All UI elements are LVGL widgets. Positions listed below
correspond to LVGL absolute or parent-relative coordinates. LVGL handles
dirty-region tracking and partial redraws automatically.

**Design differences from 320×240 preset:**
- Camera Mode Icon row removed from camera blocks (saves 24px height)
- Video Mode Text row removed from camera blocks (saves 24px height)
- Buttons displayed as colored squares with icon only — no text label
- Button colors reflect hardware button colors: Red (A), Yellow (B), Blue (C)
- GPS shows icon only — no coordinate text; Red = no fix, Green = fix
- Notification text fills the space between buttons and GPS icon in one unified bottom row
- Submenu screens: no title bar (camera slot name), no notification area
- Submenu buttons: 3 vertical colored squares on the right edge (same style as main screen)

**Visual reference:** [main-screen-320x170-mockup.png](main-screen-320x170-mockup.png)

---

## 1. Main Shutter Screen

### 1.1 Top-Level Regions

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Camera 0 Block | 5 | 5 | 100 | 134 |
| Camera 1 Block | 110 | 5 | 100 | 134 |
| Camera 2 Block | 215 | 5 | 100 | 134 |
| Bottom Row | 0 | 143 | 320 | 27 |

### 1.2 Bottom Row Sub-Regions

All elements in the bottom row share `y=143`, `height=27`.

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Button A (Red) | 5 | 143 | 27 | 27 |
| Button B (Yellow) | 36 | 143 | 27 | 27 |
| Button C (Blue) | 67 | 143 | 27 | 27 |
| Notification Area | 99 | 143 | 189 | 27 |
| GPS Status Icon | 291 | 145 | 24 | 24 |

**Button appearance:**
- Solid colored background square: Red `rgb(255,0,0)` / Yellow `rgb(255,255,0)` / Blue `rgb(0,0,255)`
- Icon: 24×24, centered at offset `(2,2)` in the 27×27 square, color Black
- No text label

**Notification Area contents:**
- Temporary text messages at `(99, 143)` — status feedback, left-aligned, vertically centered

**GPS Status Icon:**
- `gps_icon` 24×24 at `(291, 145)`
- Color: Red = no fix, Green = valid fix
- No coordinate text

### 1.3 Button Display Modes

| Mode | Condition | Visible |
|------|-----------|---------|
| `BUTTONS_MODE_SCAN_ONLY_C` | Boot scan active | C only: `scanning_icon` |
| `BUTTONS_MODE_BOOT_CONNECT_HIDDEN` | Connecting after scan | None (black) |
| `BUTTONS_MODE_NORMAL` | Standard operation | All three with dynamic mapping |

---

## 2. Camera Block Layout

Each camera block uses a base coordinate (`base_x`, `base_y`) with fixed
offsets. The same layout is used for all three slots.

### 2.1 Sub-Section Offsets

| Sub-Section | X Offset | Y Offset | Width | Height |
|-------------|----------|----------|-------|--------|
| Selection Indicator | +0 | +0 | 97 | 4 |
| Title (Model Name) | +5 | +6 | 90 | 22 |
| Status Icon | +5 | +30 | 90 | 28 |
| Recording Timer | +5 | +61 | 90 | 22 |
| SD Card Status | +5 | +86 | 90 | 22 |
| Battery Status | +5 | +111 | 90 | 22 |

> **Note:** Camera Mode Icon and Video Mode Text are omitted in this preset
> compared to the 320×240 layout. These rows are not rendered.

### 2.2 Selection Indicator

- Red fill = selected, Black fill = not selected
- In "All Cameras" mode: all three indicators are red

### 2.3 Title Area

- Model name from `device_id`: Action 4, Action 5, Action 6, Osmo 360
- Shown only when slot is paired (even if disconnected)
- Larger font, left-aligned

### 2.4 Status Icon

28×28 icon, centered horizontally within the 90px wide area (x offset +31 from area left = `base_x + 36`).
Center point: `(base_x + 36, base_y + 44)`.

Priority order (first match wins):

1. Connected + paused → `pause_icon` (white)
2. Connected + recording → `record_icon` (red)
3. Connected + sleeping → `sleep_icon` (white)
4. Not paired → `bluetooth_icon` (white)
5. Connecting → `connecting_icon` (blue)
6. Found in boot scan → `found_icon` (green)
7. Paired but disconnected → `bluetooth_icon` (blue)

### 2.5 Recording Timer

- `hh:mm:ss` format, centered in 90px area
- Recording → shows `record_time`
- Paused → shows `remain_time`
- Larger font

### 2.6 SD Card Status

- Icon: `sd_card_icon` 20×20 at `(base_x + 5, base_y + 86)`
- Text: capacity string at `(base_x + 30, base_y + 86)`, width=65, vertically centered
- Color thresholds: >16 GB white, ≤16 GB yellow, ≤6 GB red

### 2.7 Battery Status

- Icon: threshold-based battery icon 20×20 at `(base_x + 5, base_y + 111)`
  - >75% → `battery_100_icon`, ≤75% → `battery_75_icon`, ≤50% → `battery_50_icon`,
    ≤25% → `battery_25_icon`, ≤10% → `battery_0_icon`
- Text: `"NN %"` at `(base_x + 30, base_y + 111)`, width=65, vertically centered
- Color thresholds: >25% white, ≤25% yellow, ≤10% red

---

## 3. Submenu Screens (Settings, Pairing)

Both Settings and Pairing screens share a unified layout. The 320×240 title bar
(camera slot name) and notification area are omitted. Buttons are placed as
three vertical colored squares on the right edge, matching the main screen style.

**Visual reference:** [320x170-Settings.png](320x170-Settings.png)

### 3.1 Top-Level Regions

The full 320×170 display area is used. No outer margins.

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Entry Area | 0 | 0 | 288 | 170 |
| Button Column | 293 | 0 | 27 | 170 |

The 5px gap between entry area and button column (`x=288` to `x=293`) acts as
a visual separator.

### 3.2 Entry Layout

5 entries fill the full 170px height: **entry height = 34px each** (5 × 34 = 170).

| Entry | Y |
|-------|---|
| Entry 0 | 0 |
| Entry 1 | 34 |
| Entry 2 | 68 |
| Entry 3 | 102 |
| Entry 4 | 136 |

**Entry structure** (all offsets relative to entry top-left `(0, entry_y)`):

| Element | X | Y offset | Width | Height |
|---------|---|----------|-------|--------|
| Selection Indicator | 0 | 0 | 4 | 34 |
| Icon | 10 | 5 | 24 | 24 |
| Label | 38 | 9 | 246 | 24 |

- Selection Indicator: Red fill = selected, Black = not selected
- Icon: 24×24 white icon, vertically centered in 34px row (`y + 5`)
- Label: left-aligned, white text

### 3.3 Button Column

3 buttons grouped tightly at top-right, 27×27 squares, 4px gap between each.

| Button | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Button A (Red) | 293 | 3 | 27 | 27 |
| Button B (Yellow) | 293 | 34 | 27 | 27 |
| Button C (Blue) | 293 | 65 | 27 | 27 |

Button appearance: solid colored square (`rgb(255,0,0)` / `rgb(255,255,0)` / `rgb(0,0,255)`),
24×24 icon at offset `(2,2)` in black. No text label.

### 3.4 Settings Entries

| # | Icon | Label | Notes |
|---|------|-------|-------|
| 0 | `back_icon` | "Back" | Always present |
| 1 | `highlight_icon` | "Highlight" | Toggle onboard WS2812 LED |
| 2 | `sleep_icon` / `wakeup_icon` | "Sleep" / "Wakeup" | Dynamic |
| 3 | `connect_icon` / `disconnect_icon` | "Connect" / "Disconnect" | Dynamic |
| 4 | `pair_icon` / `unpair_icon` | "Pair Camera" / "Unpair Camera" | Dynamic |

### 3.5 Settings Buttons

| Button | Icon | Action |
|--------|------|--------|
| Button A (Red) | `select_icon` | Select highlighted entry |
| Button B (Yellow) | `next_icon` | Move to next entry |
| Button C (Blue) | *(no function)* | — |

### 3.6 Pairing Screen Entries

| # | Icon | Label |
|---|------|-------|
| 0 | `back_icon` | "Back" |
| 1–4 | `photo_icon` | Discovered camera name |

---

## 4. Mode Switch Screen

The Mode Switch screen uses the same entry-area / button-column layout as the
submenu screens. Only one content row is needed (current mode display), so the
layout is simplified.

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Mode Icon | 10 | 40 | 270 | 40 |
| Mode Label | 10 | 85 | 270 | 80 |
| Button Column | 293 | 0 | 27 | 170 |

- **Mode Icon:** Same logic as Camera Mode Area (photo/video from `1D02`)
- **Mode Label:** Same logic as Video Mode Area (`1D02` or `1D06` source), larger font, centered

### 4.1 Mode Switch Buttons

| Button | Icon | Action |
|--------|------|--------|
| Button A (Red) | `switch_icon` | Switch mode → sends QS button key reporting |
| Button B (Yellow) | `back_icon` | Return to Main Screen |
| Button C (Blue) | *(no function)* | — |

---

## 5. Color Reference

### Default Colors

| Element | Color |
|---------|-------|
| Background | Black |
| Default icon | White |
| Default text | White |
| Button A background | Red `rgb(255, 0, 0)` |
| Button B background | Yellow `rgb(255, 255, 0)` |
| Button C background | Blue `rgb(0, 0, 255)` |
| Button icon | Black |
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
| GPS valid fix | Green |

> **Note:** GPS icon uses Green (not Yellow) for valid fix in this preset,
> to match the mockup visual. The 320×240 preset uses Yellow for valid fix.

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

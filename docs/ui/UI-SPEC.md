# DJI-Remote – UI Layout Specification

> Authoritative pixel-level layout reference for all screens.
> The source of truth for layout values is `main/ui_layout.c` (presets) and the
> individual screen modules (`main/ui_screen_*.c`). All positions are LVGL
> widget coordinates set via `lv_obj_set_pos()` and read from the centralized
> `ui_layout_t` struct (see `main/ui_layout.h`).

> For the 320x170 Waveshare preset, see [UI-SPEC-320x170.md](UI-SPEC-320x170.md).

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
- Icon: `gps_icon` 24x24 at `(160, 192)` — map pin (Material Symbols
  `location_on`, filled, full 24 px height; source `docs/ui/icons/map-pin.svg`)
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
| `BUTTONS_MODE_BOOT_SCAN` | Boot scan active | B: `slots_icon` + "Slots", C: `scanning_icon` + "Stop"; A hidden |
| `BUTTONS_MODE_BOOT_CONNECT_HIDDEN` | Connecting after scan | None (black) |
| `BUTTONS_MODE_NORMAL` | Standard operation | All three with dynamic mapping |

---

## 2. Camera Block Layout

Each camera block uses a base coordinate (`base_x`, `base_y`) with fixed
offsets. The same layout is used for all three slots.

### 2.1 Sub-Section Offsets

| Sub-Section | X Offset | Y Offset | Width | Height |
|-------------|----------|----------|-------|--------|
| Selection Indicator | +1 | +1 | 97 | 4 |
| Title (Model Name) | +5 | +5 | 90 | 24 |
| Camera Mode Icon | +5 | +29 | 90 | 24 |
| Video Mode Text | +5 | +53 | 90 | 24 |
| Status Icon | +5 | +77 | 90 | 24 |
| Recording Timer | +5 | +101 | 90 | 24 |
| SD Card Status | +5 | +125 | 90 | 24 |
| Battery Status | +5 | +149 | 90 | 24 |

### 2.2 Selection Indicator

- Red fill = slot belongs to the selected cycle entry, Black fill = not
- Multi-slot entry (e.g. "Slot 1 + 3"): every slot of the entry is red
  (`slot_cycle_selected_has()`)

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

Priority order (first match wins, `get_status_icon_and_color()`):
1. Unresponsive (link up, no status push ≥ 6 s) → `alert_icon`
   (warning triangle, yellow)
2. Connected + sleeping → `sleep_icon` (white)
3. Connected + recording → `record_icon` (red)
4. Connected (paused / idle) → `pause_icon` (white)
5. Not paired → `bluetooth_icon` (white)
6. Connecting → `connecting_icon` (blue)
7. Found in boot scan → `found_icon` (green)
8. Paired but disconnected → `bluetooth_icon` (blue)

An unresponsive slot hides the mode, timer, SD and battery rows like a
disconnected slot (they are only shown for `CAM_STATE_CONNECTED`).

### 2.7 Recording Timer

- `hh:mm:ss` format, centered in area
- Recording → shows `record_time`
- Paused → shows `remain_time`
- Larger font

### 2.8 SD Card Status

- Icon: `sd_card_icon` 24x24 at `(base_x + 5, base_y + 125)`
- Text: capacity string at `(base_x + 34, base_y + 125)`, width=61
- Thresholds: >16GB normal, ≤16GB warning (yellow), ≤6GB critical (red);
  look per level: §2.10

### 2.9 Battery Status

- Icon: threshold-based battery icon at `(base_x + 5, base_y + 149)`
  - >75%→`battery_100_icon`, ≤75%→`battery_75_icon`, ≤50%→`battery_50_icon`,
    ≤25%→`battery_25_icon`, ≤10%→`battery_0_icon`
- Text: `"NN %"` at `(base_x + 34, base_y + 149)`, width=61
- Thresholds: >25% normal, ≤25% warning (yellow), ≤10% critical (red);
  look per level: §2.10

### 2.10 Row Warning Style

Selected at build time by Kconfig `UI_STATUS_WARN_STYLE` (menu "UI
Options"); each row (SD, battery) is evaluated independently.

| Level | Background variant (default, `UI_STATUS_WARN_BACKGROUND`) | Tint variant (legacy, `UI_STATUS_WARN_TINT`) |
|-------|-----------------------------------------------------------|----------------------------------------------|
| Normal | No background; icon + text white | Icon + text white |
| Warning | Row background yellow; icon + text **black** | Icon + text yellow |
| Critical | Row background red; icon + text **white** | Icon + text red |

Row background rectangle (block-relative, LVGL content area = inside the
1 px block border, so x = 0 is the first inner pixel; layout `L->row_bg`):

| Preset | SD row (x, y, w, h) | Battery row (x, y, w, h) |
|--------|---------------------|--------------------------|
| 320x240 | 0, 125, 98, 24 | 0, 149, 98, 24 (rows touch) |
| 320x170 | 0, 80, 98, 24 | 0, 108, 98, 24 (4 px gap) |
| 240x135 | 0, 52, 120, 24 | 120, 52, 98, 24 (one line, split at the battery icon x) |

The backgrounds are created before the row's icon and text (drawn below
them) and are hidden whenever the rows are hidden (slot not
`CAM_STATE_CONNECTED`; sleeping slots keep their rows). The busy indicator
(320x170) stays on top.

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
| Label | +44 | 252 | 24 |

Indicator: Red if selected, Black if not.

Label width = `submenu.w − label_ofs_x − 4` (right margin) = `300 − 44 − 4`.
Labels use `LV_LABEL_LONG_DOT`: text that does not fit ends with an ellipsis
(Pairing names are additionally cut to 23 characters, which always fit).

**Selection change clearing area** (prevents artifacts):
- Position: `(entry_x - 5, entry_y - 5)`
- Size: 20 x 34
- Applied only to the previously selected entry

### 3.3 Submenu Buttons

- Button A: `select_icon` + "Select" → executes the selected entry
- Button B: `next_icon` + "Next" (icon rotated 90° CW to point downward) →
  next entry, wraps after the last visible entry
- Button C: `back_icon` + "Back" → returns to the Main Screen (Pairing: also
  stops the scan)

"Back" is **not** a list entry; entry 0 is the first real option.

### 3.4 Title Text

- "Camera X" where X is 1-indexed (1, 2, 3)
- Larger font, horizontally and vertically centered

---

## 4. Settings Screen Entries

1. `switch_icon` + "Mode Switch"
2. Dynamic: `sleep_icon`/"Sleep" or `wakeup_icon`/"Wakeup"
3. Dynamic: `connect_icon`/"Connect" or `disconnect_icon`/"Disconnect"
4. Dynamic: `unpair_icon`/"Unpair" or `pair_icon`/"Pair Camera"
5. `slots_icon` + "Slots" (row y = 155; global slot cycle)

"Slots" is shown only where the preset has ≥ 5 entries (`max_entries`);
the 240x135 preset (3 entries) does not show it.

---

## 5. Pairing Screen Entries

1–5. Discovered cameras only: `photo_icon` + camera name (up to 5 on this
preset). Entries stay hidden until a camera is found; the notification area
shows "Scanning..." (cyan) meanwhile.

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
- Button B: empty (no widget, no function)
- Button C: `back_icon` + "Back" → returns to Main Screen

---

## 6A. Slots Screens

Reached from the Options Menu ("Slots") or from Main Screen Button B
"Slots" during the boot scan.
Two screens with the submenu row geometry of §3.1/§3.2 (rows at y = 39, 68,
97, 126), single column, **no title** (rows keep their positions; the title
area stays empty), no notification text. Buttons as in §3.3 (A Select,
B Next rotated, C Back).

**Cycle list** (`SCREEN_SLOTS_CYCLE`): 4 rows = cycle entries in order.

| Element | X | Width | Notes |
|---------|---|-------|-------|
| Selection Indicator | 10 | 10 | Red = highlighted entry |
| Icon | 25 | 24 | `slots_icon`, white |
| Label | 54 | 252 | Derived name: "Slot 1", "Slot 2 + 3", "Slot 1 + 2 + 3", "Empty" |

**Entry screen** (`SCREEN_SLOTS_ENTRY`): 3 rows "Slot 1", "Slot 2", "Slot 3".

| Element | X | Width | Notes |
|---------|---|-------|-------|
| Selection Indicator | 10 | 10 | Red = highlighted slot |
| Label | 54 | 213 | Ends 4 px before the check mark |
| Check Mark | 271 | 24 | `check_icon`, green; visible only for active slots |

Check mark x = `screen_w − (base_x + icon_ofs_x) − 24` = `320 − 25 − 24`
(mirrors the left icon inset). No left icon on the entry screen.

Rejected toggle (would leave every entry Empty): the highlighted row's label
turns red for 1 s, then white again.

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
| Unresponsive | Yellow (alert_icon) |
| GPS no fix | Red |
| GPS valid fix | Yellow |

### Threshold Colors (SD Card)

| Condition | Background variant (default) | Tint variant |
|-----------|------------------------------|--------------|
| >16 GB free | White icon + text | White |
| ≤16 GB free | Yellow row, black icon + text | Yellow |
| ≤6 GB free | Red row, white icon + text | Red |

### Threshold Colors (Battery)

| Condition | Background variant (default) | Tint variant |
|-----------|------------------------------|--------------|
| >25% | White icon + text | White |
| ≤25% | Yellow row, black icon + text | Yellow |
| ≤10% | Red row, white icon + text | Red |

# UI Specification — 320x170 (Waveshare ESP32-S3-LCD-1.9)

> Authoritative pixel-level specification for the 320x170 layout preset.
> For the 320x240 M5Stack layout, see [UI-SPEC.md](UI-SPEC.md).

---

## 1. Display

- Resolution: 320x170 px, landscape (HAL sets `swap_xy = true` in LVGL port)
- Origin: top-left `(0, 0)`

---

## 2. Main Screen

### 2.1 Camera Blocks — Top-Level Regions

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Camera 0 Block | 5 | 5 | 100 | 134 |
| Camera 1 Block | 110 | 5 | 100 | 134 |
| Camera 2 Block | 215 | 5 | 100 | 134 |
| Bottom Row | 0 | 143 | 320 | 27 |

### 2.2 Camera Block Internal Offsets

| Sub-Section | X Offset | Y Offset | Width | Height | Notes |
|-------------|----------|----------|-------|--------|-------|
| Selection Indicator | +1 | +1 | 97 | 4 | 97px avoids overlapping the 1px block border |
| Title (Model Name) | +5 | +9 | 90 | 20 | |
| Status Icon | +38 | +29 | 24 | 24 | Centered |
| Recording Timer | +5 | +58 | 90 | 16 | |
| SD Card Icon | +5 | +80 | 24 | 24 | |
| SD Card Text | +34 | +85 | 61 | 16 | |
| Battery Icon | +5 | +108 | 24 | 24 | |
| Battery Text | +34 | +113 | 61 | 16 | |
| SD Row Background | +0 | +80 | 98 | 24 | Warning background, inside the 1 px border; hidden at normal level |
| Battery Row Background | +0 | +108 | 98 | 24 | As above; 4 px gap to the SD row |
| Busy Indicator | +34 | +84 | 32 | 32 | `busy_icon_32`, cyan, transparent; hidden unless the slot is busy |

Mode Icon row and Video Mode Text row are **absent** in this preset.

The status icon uses the same priority list and colours as 320×240
(`UI-SPEC.md` §2.6), including the yellow `alert_icon` warning triangle for an
unresponsive slot as the first match. Only the position differs.

**Busy indicator.** A static cyan `busy_icon_32` (no
blink or rotation) is shown per slot while an action on that slot is running
(connect, start/stop recording, photo, sleep, wake, snapshot, disconnect,
unpair). It is centred horizontally (`(100 − 32) / 2 = 34`) and sits in the
lower half of the block (y 84–115), so title, status icon and recording timer
stay visible; it covers the middle of the SD and battery rows. It is the last
child of the block, so it is drawn on top. Layout: `L->busy.main_*`.

### 2.3 Bottom Row Contents

| Element | X | Y | Width | Height | Notes |
|---------|---|---|-------|--------|-------|
| GPS Icon | 5 | 145 | 24 | 24 | Leftmost; icon-only (no coordinate text); map pin `gps_icon` (UI-SPEC §1) |
| Button A | 32 | 143 | 96 | 27 | Black background, 24x24 icon at (0,1), label at (29,7), white icon + white text |
| Button B | 128 | 143 | 96 | 27 | Same style as Button A |
| Button C | 224 | 143 | 96 | 27 | Same style as Button A |

The Notification Area is intentionally absent on 320×170; status feedback that
previously rendered there is omitted on this preset to make room for the
M5Stack-style buttons that the housing's button positions require.

Math: `32 + 3 × 96 = 320` (no gaps between buttons; the 5 + 24 = 29 GPS slot
leaves a 3 px gap to Button A's left edge).

Button labels and visibility per mode are shared with 320×240 (UI-SPEC §1,
Button Area Display Modes): during the boot scan A is hidden, B shows
`slots_icon` + "Slots" and C `scanning_icon` + "Stop".

### 2.4 Button Colors

Bottom-row buttons on 320×170 match the M5Stack 320×240 visual style
(icon + text label, white-on-black). The same buttons are used on the
submenu and mode-switch screens (§3.5, §4). No screen on 320×170 uses the
former colored-square button style.

| Button | Background Color | Icon Color | Label Color |
|--------|------------------|------------|-------------|
| A | Black | White | White |
| B | Black | White | White |
| C | Black | White | White |

State-specific icon colors (e.g. red `record_icon` when recording) are applied
at runtime by `set_button()` and override the default white.

### 2.5 GPS Color Difference

| State | Color |
|-------|-------|
| No fix | Red |
| Valid fix | **Green** (not Yellow as in 320x240) |

---

## 3. Submenu Screens (Settings, Pairing)

### 3.1 Regions

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Entry Area | 0 | 0 | 320 | 143 |
| Bottom Row | 0 | 143 | 320 | 27 |

No title bar. The entry area is a **2 columns × 4 rows** grid (up to 8
entries) above the bottom row. The last row ends at y = 140, leaving a 3 px
gap to the bottom row.

### 3.2 Entry Grid

Entries fill **column-major**: the left column top to bottom first, then the
right column. Button B navigates in the same order (index order).

| idx | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|-----|---|---|---|---|---|---|---|---|
| X | 0 | 0 | 0 | 0 | 160 | 160 | 160 | 160 |
| Y | 1 | 36 | 71 | 106 | 1 | 36 | 71 | 106 |

`x = base_x + (idx / rows) × col_w`, `y = entry_first_y + (idx % rows) × entry_spacing`
with `rows = 4`, `col_w = 160`, `entry_first_y = 1`, `entry_spacing = 35`.

With **≤ 4 entries** the list is a single column spanning the full width
(Slots screens, §4A). With **> 4 entries** the right column opens; entry
positions never move, only the label widths shrink. Settings has 5 entries
(Mode Switch, Sleep/Wakeup, Connect/Disconnect, Pair/Unpair, **Slots**), so
"Slots" is the first entry of the right column (idx 4: x = 160, y = 1) and
all Settings labels are 118 px wide.

### 3.3 Entry Elements

| Element | X Offset within entry | Y Offset within entry | Width | Height |
|---------|-----------------------|-----------------------|-------|--------|
| Selection Indicator | 0 | 0 | 4 | 34 |
| Icon | 10 | 5 | 24 | 24 |
| Label | 38 | 9 | 278 (single column) / 118 (two columns) | 24 |

Icon is vertically centered in the 34px row (`y + 5` = (34-24)/2 rounded down).
Label offset `38` = indicator(4) + gap(6) + icon(24) + gap(4).
Label width = column span − 38 − 4 (right margin): `320 − 42 = 278` or
`160 − 42 = 118`. Labels use `LV_LABEL_LONG_DOT` — text that does not fit
ends with an ellipsis.

### 3.4 Pairing Scanning Placeholder

There is no notification area on 320×170. While Pairing has found no camera
yet, a "Scanning..." label is shown at entry 0's label position:

| Element | X | Y | Width | Font | Color |
|---------|---|---|-------|------|-------|
| Scanning placeholder | 38 | 10 | 278 (`LONG_DOT`) | `font_heading` (Montserrat 16) | Cyan |

It is hidden as soon as the first camera appears and shown again when the
camera list is reset.

### 3.5 Bottom-Row Buttons

Identical geometry and style to the main screen (§2.3): black background,
24×24 white icon at (0,1), white label at (29,7). The GPS slot (x 0–31) stays
**empty** on submenu screens, except for the busy indicator on the Settings
screen:

| Element | X | Y | Width | Height | Notes |
|---------|---|---|-------|--------|-------|
| Busy Indicator (Settings only) | 5 | 145 | 24 | 24 | `busy_icon_24`, cyan, for the slot the menu belongs to; hidden otherwise. `L->busy.menu_*` |

| Button | X | Y | Width | Height | Icon | Label |
|--------|---|---|-------|--------|------|-------|
| A | 32 | 143 | 96 | 27 | `select_icon` | Select |
| B | 128 | 143 | 96 | 27 | `next_icon` (**unrotated**) | Next |
| C | 224 | 143 | 96 | 27 | `back_icon` | Back |

Button B's `next_icon` is not rotated on 320×170 because navigation can move
into a second column (`submenu.next_icon_rotated = false`). "Back" is not a
list entry — Button C leaves the screen (Pairing: also stops the scan).

---

## 4A. Slots Screens

Two screens with the submenu row geometry of §3.2/§3.3, always a **single
full-width column** (≤ 4 rows: column 0, label span 320). No title, no
notification area, no busy indicator; the GPS slot of the bottom row stays
empty. Bottom row identical to §3.5 (A Select, B Next unrotated, C Back).

**Cycle list** (`SCREEN_SLOTS_CYCLE`): 4 rows = cycle entries in order.

| Element | X | Y (row 0..3) | Width | Notes |
|---------|---|--------------|-------|-------|
| Selection Indicator | 0 | 1 / 36 / 71 / 106 | 4 | Red = highlighted entry |
| Icon | 10 | row + 5 | 24 | `slots_icon`, white |
| Label | 38 | row + 9 | 278 | Derived name: "Slot 1", "Slot 2 + 3", "Slot 1 + 2 + 3", "Empty" |

**Entry screen** (`SCREEN_SLOTS_ENTRY`): 3 rows "Slot 1", "Slot 2", "Slot 3".

| Element | X | Y (row 0..2) | Width | Notes |
|---------|---|--------------|-------|-------|
| Selection Indicator | 0 | 1 / 36 / 71 | 4 | Red = highlighted slot |
| Label | 38 | row + 9 | 244 | Ends 4 px before the check mark |
| Check Mark | 286 | row + 5 | 24 | `check_icon`, green; visible only for active slots |

Check mark x = `screen_w − (base_x + icon_ofs_x) − 24` = `320 − 10 − 24`
(mirrors the left icon inset). No left icon on the entry screen.

Rejected toggle (would leave every entry Empty): the highlighted row's label
turns red for 1 s (`lv_timer`, one-shot), then white again.

---

## 4. Mode Switch Screen

| Region | X | Y | Width | Height |
|--------|---|---|-------|--------|
| Content Area | 0 | 0 | 320 | 143 |
| Mode Icon | 148 | 28 | 24 | 24 |
| Mode Label (2 lines, centered) | 0 | 73 | 320 | ≈42 |
| Bottom Row | 0 | 143 | 320 | 27 |

No title bar. Icon and text are centered horizontally on the full 320 px
width and vertically in the 143 px content area.

| Button | X | Icon | Label | Function |
|--------|---|------|-------|----------|
| A | 32 | `switch_icon` | Switch | Quick Switch |
| B | 128 | — | — | **empty** (no widget, no function) |
| C | 224 | `back_icon` | Back | Return to main screen |

---

## 5. Color Summary

| Element | 320x240 | 320x170 |
|---------|---------|---------|
| GPS valid fix | Yellow | Green |
| Bottom-row Button A/B/C background (all screens) | Black | Black |
| Bottom-row Button A/B/C icon + label color (all screens) | White | White |
| Submenu selection indicator | Red | Red |
| Pairing "Scanning..." text | Cyan (notification area) | Cyan (entry-area placeholder) |
| Busy indicator (main screen, Settings) | — (not shown) | Cyan |
| All threshold colors (battery, SD) | unchanged | unchanged (same row warning style, UI-SPEC §2.10) |

No colored-square button style is used anywhere on 320×170.

# dji_diag – Fault Diagnostics Reader

DJI-Remote records connection faults and restarts in its own flash
partition (`diag`), also when no computer is attached and in release builds.
`dji_diag` reads these records over USB and prints them: restarts,
unexpected disconnects, unresponsive cameras, recoveries, watchdog
teardowns and aborted connects, each with boot number, uptime, GPS UTC time
(if a fix was available), slot and camera model.

Tested on macOS with the M5Stack Basic V2.7 (USB-UART) and the Waveshare
ESP32-S3-LCD-1.9 (USB-Serial-JTAG). Linux should work the same way.

## Setup

The wrapper `tools/diag/dji_diag` runs the script with the project-local
ESP-IDF Python (`.idf/`), which already contains `esptool` and ESP-IDF's
NVS parser. No extra packages are needed. If the toolchain is not installed
yet, run once from the repository root:

```bash
tools/idf/setup.sh
```

Without the project-local toolchain, run `python tools/diag/dji_diag.py …`
inside any ESP-IDF v5.x environment (`IDF_PATH` set, `esptool` installed).

## Commands

Run from the repository root. Close any serial monitor first; the port must
be free.

```bash
# Find the port
ls /dev/cu.usb*

# Read the device and print the report (optionally keep the raw dump)
tools/diag/dji_diag read -p /dev/cu.usbmodem21401 --save tour-2026-10-18.bin

# Decode a saved dump without a device
tools/diag/dji_diag decode tour-2026-10-18.bin

# Erase the diag partition only (asks for confirmation unless --yes)
tools/diag/dji_diag clear -p /dev/cu.usbmodem21401

# Check the record layout and the decoder without a device
tools/diag/dji_diag selftest
```

- `read` and `clear` first read the partition table at 0x8000 and use the
  `diag` entry. Firmware without the partition is reported cleanly
  ("No 'diag' partition on this device").
- Reading or clearing **restarts the remote** (esptool resets it). That
  restart appears as the next boot's `RESTART` record.
- `clear` erases only the `diag` partition. Pairing data and settings live in
  the separate `nvs` partition and are not touched.
- The `diag` partition is not part of the release binary, so flashing a new
  firmware keeps the records (unless the whole flash is erased).

## Example Output

```
DJI-Remote fault diagnostics
============================
Firmware (last boot): v1.3.0
Current boot_id:      7
Records stored:       3 (next seq 131)
Suppressed (rate):    2
Queue drops:          0
Write errors:         0

Restarts: 7

Event           slot 0  slot 1  slot 2
--------------------------------------
DISCONNECT           0       1       0
UNRESPONSIVE         0       0       1
RECOVERED            0       0       0
TEARDOWN             0       0       1
CONNECT_ABORT        0       0       0

   seq  boot       uptime  UTC                  event          slot  model     detail
   128     7 00:00:00.812  -                    RESTART           -  -         reset=POWERON
   129     7 01:02:03.456  2026-10-18T13:20:00Z DISCONNECT        1  Action 6  reason=0x08 supervision timeout
   130     7 00:01:30.000  -                    TEARDOWN          2  Osmo 360  silence=15.0 s
```

## Reading the Report

| Column / event | Meaning |
|----------------|---------|
| `seq` | Record number; the remote keeps the newest 128 records |
| `boot` | Boot counter at the time of the event |
| `uptime` | Time since that boot |
| `UTC` | GPS time, `-` without a recent fix |
| `RESTART` | The remote started; detail = reset reason (`POWERON`, `SW`, `PANIC`, `BROWNOUT`, `USB`, …) |
| `DISCONNECT` | Unexpected connection loss (not user-initiated, not during a connect, not after a watchdog teardown) |
| `UNRESPONSIVE` | The camera stayed connected but sent no status for about 6 s (yellow triangle) |
| `RECOVERED` | The camera started sending again before the teardown |
| `TEARDOWN` | The remote closed the connection after 15 s without status |
| `CONNECT_ABORT` | The connection dropped while the remote was still connecting |
| `Suppressed` | Events not stored as records because of the write limit (10 in a burst, then 1 per 30 s); they are still in the counters |
| Counters | Every event since the last `clear`, including suppressed ones |

Disconnect reasons: `0x08` supervision timeout (radio loss or camera
switched off), `0x13` the camera closed the connection, `0x16` the remote
closed it, `0x3e` the connection could not be established.

## For Developers

The record and meta layouts in `dji_diag.py` must match `diag/diag.c`.
Event types are append-only. When the firmware layout changes, bump the
version in both files, add new types to `EVENT_NAMES`, and run
`dji_diag selftest`. The model table mirrors `ui_get_camera_model_name()` in
`main/ui.c`.

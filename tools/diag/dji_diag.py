#!/usr/bin/env python3
"""DJI-Remote persistent fault diagnostics reader.

Reads the "diag" NVS partition of a DJI-Remote over USB with esptool,
decodes the fault records and counters, and prints them. Can also decode a
saved dump offline and erase the partition.

Run through the wrapper next to this file, which uses the project-local
ESP-IDF Python (esptool and IDF's NVS parser, no extra packages):

    tools/diag/dji_diag read  -p /dev/cu.usbmodemXXXX [--save dump.bin]
    tools/diag/dji_diag decode dump.bin
    tools/diag/dji_diag clear -p /dev/cu.usbmodemXXXX [--yes]
    tools/diag/dji_diag selftest

The record and meta layouts below must match diag/diag.c exactly.
Reading or clearing resets the device; that restart appears as the next
boot's RESTART record.
"""

import argparse
import datetime
import os
import struct
import subprocess
import sys
import tempfile

# ---------------------------------------------------------------------------
# Layouts (keep in sync with diag/diag.c)
# ---------------------------------------------------------------------------

NAMESPACE = "diag"
META_KEY = "meta"
RING_SIZE = 128

RECORD_VERSION = 1
RECORD_FMT = "<BBBBHHIIII"          # diag_record_t, 24 bytes
RECORD_SIZE = struct.calcsize(RECORD_FMT)

META_VERSION = 1
NUM_SLOTS = 3
NUM_TYPES = 6
META_FMT = "<BBHI" + "I" * (NUM_SLOTS * NUM_TYPES) + "III" + "32s"   # diag_meta_t, 124 bytes
META_SIZE = struct.calcsize(META_FMT)

FLAG_UTC_VALID = 0x01
SLOT_NONE = 0xFF

assert RECORD_SIZE == 24, RECORD_SIZE
assert META_SIZE == 124, META_SIZE

# Append-only, same values as diag_event_type_t
EVENT_NAMES = {
    1: "RESTART",
    2: "DISCONNECT",
    3: "UNRESPONSIVE",
    4: "RECOVERED",
    5: "TEARDOWN",
    6: "CONNECT_ABORT",
}

# Keep in sync with ui_get_camera_model_name() in main/ui.c
MODEL_NAMES = {
    0xFF33: "Action 4",
    0xFF44: "Action 5",
    0xFF55: "Action 6",
    0xFF66: "Osmo 360",
}

# esp_reset_reason_t (ESP-IDF v5.5)
RESET_REASONS = {
    0: "UNKNOWN",
    1: "POWERON",
    2: "EXT",
    3: "SW",
    4: "PANIC",
    5: "INT_WDT",
    6: "TASK_WDT",
    7: "WDT",
    8: "DEEPSLEEP",
    9: "BROWNOUT",
    10: "SDIO",
    11: "USB",
    12: "JTAG",
    13: "EFUSE",
    14: "PWR_GLITCH",
    15: "CPU_LOCKUP",
}

# HCI disconnect reasons. NimBLE reports them as 0x200 + HCI code.
HCI_REASONS = {
    0x08: "supervision timeout",
    0x13: "remote user terminated",
    0x16: "local host terminated",
    0x22: "LL response timeout",
    0x3B: "unacceptable conn params",
    0x3D: "MIC failure",
    0x3E: "failed to establish",
}
BLE_HS_ERR_HCI_BASE = 0x200

DEFAULT_DIAG_OFFSET = 0x310000
DEFAULT_DIAG_SIZE = 0x6000
PARTITION_TABLE_OFFSET = 0x8000
PARTITION_TABLE_SIZE = 0xC00


# ---------------------------------------------------------------------------
# IDF helpers
# ---------------------------------------------------------------------------

def idf_path():
    path = os.environ.get("IDF_PATH")
    if path and os.path.isdir(path):
        return path
    here = os.path.dirname(os.path.abspath(__file__))
    guess = os.path.join(here, "..", "..", ".idf", "esp-idf")
    if os.path.isdir(guess):
        return os.path.abspath(guess)
    sys.exit("IDF_PATH not set and .idf/esp-idf not found. "
             "Run through tools/diag/dji_diag or inside an ESP-IDF environment.")


def import_nvs_parser():
    sys.path.insert(0, os.path.join(idf_path(), "components", "nvs_flash", "nvs_partition_tool"))
    import nvs_parser  # noqa: E402  (IDF module, found via sys.path)
    return nvs_parser


def esptool(port, *args):
    cmd = [sys.executable, "-m", "esptool", "--chip", "auto", "-p", port, *args]
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if result.returncode != 0:
        sys.stdout.write(result.stdout)
        sys.exit(f"esptool failed: {' '.join(args)}")
    return result.stdout


def read_flash(port, offset, size):
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "dump.bin")
        esptool(port, "read_flash", hex(offset), hex(size), path)
        with open(path, "rb") as f:
            return f.read()


# ---------------------------------------------------------------------------
# Partition table
# ---------------------------------------------------------------------------

def find_diag_partition(table):
    """Returns (offset, size) of the 'diag' partition, or None."""
    for i in range(0, len(table) - 31, 32):
        entry = table[i:i + 32]
        magic = struct.unpack_from("<H", entry, 0)[0]
        if magic == 0xFFFF:
            break
        if magic != 0x50AA:  # bytes AA 50
            continue
        _type, _subtype, offset, size = struct.unpack_from("<BBII", entry, 2)
        name = entry[12:28].split(b"\0", 1)[0].decode("ascii", "replace")
        if name == "diag":
            return offset, size
    return None


def locate_partition(port):
    table = read_flash(port, PARTITION_TABLE_OFFSET, PARTITION_TABLE_SIZE)
    found = find_diag_partition(table)
    if found is None:
        if any(b != 0xFF for b in table[:32]):
            sys.exit("No 'diag' partition on this device (firmware without "
                     "persistent diagnostics).")
        print(f"WARNING: no partition table found, assuming diag at "
              f"0x{DEFAULT_DIAG_OFFSET:x}/0x{DEFAULT_DIAG_SIZE:x}")
        return DEFAULT_DIAG_OFFSET, DEFAULT_DIAG_SIZE
    return found


# ---------------------------------------------------------------------------
# NVS decoding
# ---------------------------------------------------------------------------

def nvs_blobs(raw, namespace):
    """Returns {key: bytes} for all written blobs of one namespace."""
    nvs_parser = import_nvs_parser()
    part = nvs_parser.NVS_Partition("diag", bytearray(raw))

    entries = [e for page in part.pages for e in page.entries if e.state == "Written"]

    ns_index = None
    for e in entries:
        if e.metadata["namespace"] == 0 and e.key == namespace:
            ns_index = e.data["value"]
            break
    if ns_index is None:
        return {}

    def payload(entry):
        data = b"".join(bytes(c.raw) for c in entry.children)
        return data[:entry.data["size"]]

    blobs = {}
    for idx in (e for e in entries
                if e.metadata["namespace"] == ns_index and e.metadata["type"] == "blob_index"):
        start = idx.data["chunk_start"]
        count = idx.data["chunk_count"]
        chunks = {}
        for e in entries:
            if (e.metadata["namespace"] == ns_index and e.metadata["type"] == "blob_data"
                    and e.key == idx.key
                    and start <= e.metadata["chunk_index"] < start + count):
                chunks[e.metadata["chunk_index"]] = payload(e)
        if len(chunks) == count:
            data = b"".join(chunks[i] for i in sorted(chunks))
            blobs[idx.key] = data[:idx.data["size"]]
    return blobs


def decode_record(data):
    if len(data) != RECORD_SIZE:
        return {"raw": data.hex()}
    (version, rtype, slot, flags, boot_id, detail,
     uptime_ms, utc_epoch, device_id, seq) = struct.unpack(RECORD_FMT, data)
    return {
        "version": version, "type": rtype, "slot": slot, "flags": flags,
        "boot_id": boot_id, "detail": detail, "uptime_ms": uptime_ms,
        "utc_epoch": utc_epoch, "device_id": device_id, "seq": seq,
    }


def encode_record(rec):
    return struct.pack(RECORD_FMT, rec["version"], rec["type"], rec["slot"], rec["flags"],
                       rec["boot_id"], rec["detail"], rec["uptime_ms"], rec["utc_epoch"],
                       rec["device_id"], rec["seq"])


def decode_meta(data):
    if len(data) != META_SIZE:
        return None
    values = struct.unpack(META_FMT, data)
    version, _reserved, boot_id, next_seq = values[0:4]
    flat = values[4:4 + NUM_SLOTS * NUM_TYPES]
    counters = [list(flat[s * NUM_TYPES:(s + 1) * NUM_TYPES]) for s in range(NUM_SLOTS)]
    suppressed, queue_drops, write_errors = values[4 + NUM_SLOTS * NUM_TYPES:-1]
    fw = values[-1].split(b"\0", 1)[0].decode("ascii", "replace")
    return {
        "version": version, "boot_id": boot_id, "next_seq": next_seq,
        "counters": counters, "suppressed": suppressed, "queue_drops": queue_drops,
        "write_errors": write_errors, "fw_version": fw,
    }


def encode_meta(meta):
    flat = [c for row in meta["counters"] for c in row]
    return struct.pack(META_FMT, meta["version"], 0, meta["boot_id"], meta["next_seq"], *flat,
                       meta["suppressed"], meta["queue_drops"], meta["write_errors"],
                       meta["fw_version"].encode("ascii"))


# ---------------------------------------------------------------------------
# Formatting
# ---------------------------------------------------------------------------

def fmt_uptime(ms):
    s, ms = divmod(ms, 1000)
    h, s = divmod(s, 3600)
    m, s = divmod(s, 60)
    return f"{h:02d}:{m:02d}:{s:02d}.{ms:03d}"


def fmt_utc(rec):
    if not rec["flags"] & FLAG_UTC_VALID:
        return "-"
    return datetime.datetime.fromtimestamp(rec["utc_epoch"], datetime.timezone.utc) \
        .strftime("%Y-%m-%dT%H:%M:%SZ")


def fmt_ble_reason(reason):
    code = reason - BLE_HS_ERR_HCI_BASE if BLE_HS_ERR_HCI_BASE <= reason < 0x300 else None
    if code is not None:
        return f"0x{code:02x} {HCI_REASONS.get(code, 'HCI error')}"
    return f"0x{reason:x} (host error)"


def fmt_detail(rec):
    rtype, detail = rec["type"], rec["detail"]
    if rtype == 1:
        return f"reset={RESET_REASONS.get(detail, detail)}"
    if rtype in (2, 6):
        return f"reason={fmt_ble_reason(detail)}"
    if rtype in (3, 4, 5):
        return "silence=" + (">6553.5 s" if detail == 0xFFFF else f"{detail / 10:.1f} s")
    return f"detail={detail}"


def fmt_slot(slot):
    return "-" if slot == SLOT_NONE else str(slot)


def fmt_model(device_id, rtype):
    if rtype == 1:
        return "-"
    return MODEL_NAMES.get(device_id, f"0x{device_id:x}")


def print_report(blobs):
    meta = decode_meta(blobs[META_KEY]) if META_KEY in blobs else None
    records = []
    for key, data in blobs.items():
        if len(key) == 4 and key[0] == "r" and key[1:].isdigit():
            records.append(decode_record(data))

    print("DJI-Remote fault diagnostics")
    print("=" * 28)
    if meta is None:
        print("No meta blob found (partition empty or foreign).")
    elif meta["version"] != META_VERSION:
        print(f"Unknown meta version {meta['version']}; raw: {blobs[META_KEY].hex()}")
        meta = None
    else:
        print(f"Firmware (last boot): {meta['fw_version'] or '-'}")
        print(f"Current boot_id:      {meta['boot_id']}")
        print(f"Records stored:       {len(records)} (next seq {meta['next_seq']})")
        print(f"Suppressed (rate):    {meta['suppressed']}")
        print(f"Queue drops:          {meta['queue_drops']}")
        print(f"Write errors:         {meta['write_errors']}")
        print()
        print(f"Restarts: {meta['counters'][0][0]}")
        print()
        header = f"{'Event':<14}" + "".join(f"{'slot ' + str(s):>8}" for s in range(NUM_SLOTS))
        print(header)
        print("-" * len(header))
        for t in range(2, NUM_TYPES + 1):
            row = f"{EVENT_NAMES[t]:<14}" + "".join(
                f"{meta['counters'][s][t - 1]:>8}" for s in range(NUM_SLOTS))
            print(row)
    print()

    if not records:
        print("No event records.")
        return

    records.sort(key=lambda r: r.get("seq", -1))
    print(f"{'seq':>6} {'boot':>5} {'uptime':>12}  {'UTC':<20} {'event':<14} "
          f"{'slot':>4}  {'model':<9} detail")
    for r in records:
        if "raw" in r or r["version"] != RECORD_VERSION:
            print(f"{'?':>6} unknown record: {r.get('raw') or encode_record(r).hex()}")
            continue
        name = EVENT_NAMES.get(r["type"], f"type {r['type']}")
        print(f"{r['seq']:>6} {r['boot_id']:>5} {fmt_uptime(r['uptime_ms']):>12}  "
              f"{fmt_utc(r):<20} {name:<14} {fmt_slot(r['slot']):>4}  "
              f"{fmt_model(r['device_id'], r['type']):<9} {fmt_detail(r)}")


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_read(args):
    offset, size = locate_partition(args.port)
    print(f"Reading diag partition 0x{offset:x} ({size // 1024} KB)...")
    raw = read_flash(args.port, offset, size)
    if args.save:
        with open(args.save, "wb") as f:
            f.write(raw)
        print(f"Saved dump to {args.save}")
    print()
    print_report(nvs_blobs(raw, NAMESPACE))


def cmd_decode(args):
    with open(args.file, "rb") as f:
        raw = f.read()
    print_report(nvs_blobs(raw, NAMESPACE))


def cmd_clear(args):
    offset, size = locate_partition(args.port)
    if not args.yes:
        answer = input(f"Erase diag partition at 0x{offset:x} ({size // 1024} KB)? [y/N] ")
        if answer.strip().lower() not in ("y", "yes"):
            print("Aborted.")
            return
    esptool(args.port, "erase_region", hex(offset), hex(size))
    print("Diagnostics cleared.")


def cmd_selftest(_args):
    """Encodes records and meta, builds a real NVS image with IDF's
    generator, and decodes it through the same path as 'read'."""
    records = [
        dict(version=1, type=1, slot=SLOT_NONE, flags=0, boot_id=7, detail=1,
             uptime_ms=812, utc_epoch=0, device_id=0, seq=128),
        dict(version=1, type=2, slot=1, flags=FLAG_UTC_VALID, boot_id=7, detail=0x208,
             uptime_ms=3_723_456, utc_epoch=1_792_329_600, device_id=0xFF55, seq=129),
        dict(version=1, type=5, slot=2, flags=0, boot_id=7, detail=150,
             uptime_ms=90_000, utc_epoch=0, device_id=0xFF66, seq=130),
    ]
    meta = dict(version=1, boot_id=7, next_seq=131,
                counters=[[7, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 1, 0]],
                suppressed=2, queue_drops=0, write_errors=0, fw_version="v1.3.0-test")

    for r in records:
        assert decode_record(encode_record(r)) == r, r
    assert decode_meta(encode_meta(meta)) == meta

    gen = os.path.join(idf_path(), "components", "nvs_flash", "nvs_partition_generator",
                       "nvs_partition_gen.py")
    with tempfile.TemporaryDirectory() as tmp:
        csv_path = os.path.join(tmp, "in.csv")
        bin_path = os.path.join(tmp, "out.bin")
        with open(csv_path, "w") as f:
            f.write("key,type,encoding,value\n")
            f.write(f"{NAMESPACE},namespace,,\n")
            f.write(f"{META_KEY},data,hex2bin,{encode_meta(meta).hex()}\n")
            for r in records:
                f.write(f"r{r['seq'] % RING_SIZE:03d},data,hex2bin,{encode_record(r).hex()}\n")
        subprocess.run([sys.executable, gen, "generate", csv_path, bin_path,
                        hex(DEFAULT_DIAG_SIZE)], check=True, stdout=subprocess.DEVNULL)
        with open(bin_path, "rb") as f:
            raw = f.read()

    blobs = nvs_blobs(raw, NAMESPACE)
    assert decode_meta(blobs[META_KEY]) == meta, "meta mismatch"
    for r in records:
        assert decode_record(blobs[f"r{r['seq'] % RING_SIZE:03d}"]) == r, r

    table = bytearray(b"\xff" * 64)
    struct.pack_into("<HBBII16sI", table, 0, 0x50AA, 1, 2, DEFAULT_DIAG_OFFSET,
                     DEFAULT_DIAG_SIZE, b"diag", 0)
    assert find_diag_partition(bytes(table)) == (DEFAULT_DIAG_OFFSET, DEFAULT_DIAG_SIZE)

    print_report(blobs)
    print()
    print("selftest OK")


def main():
    parser = argparse.ArgumentParser(description="DJI-Remote fault diagnostics reader")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("read", help="read and decode the diag partition from a device")
    p.add_argument("-p", "--port", required=True)
    p.add_argument("--save", help="also save the raw partition dump to this file")
    p.set_defaults(func=cmd_read)

    p = sub.add_parser("decode", help="decode a saved partition dump")
    p.add_argument("file")
    p.set_defaults(func=cmd_decode)

    p = sub.add_parser("clear", help="erase the diag partition (nothing else)")
    p.add_argument("-p", "--port", required=True)
    p.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    p.set_defaults(func=cmd_clear)

    p = sub.add_parser("selftest", help="check the record/meta layout and NVS decoding")
    p.set_defaults(func=cmd_selftest)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

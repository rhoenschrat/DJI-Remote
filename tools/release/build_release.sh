#!/usr/bin/env bash
# Build a release package from compiled ESP-IDF firmware.
#
# Usage: build_release.sh [VERSION]
#
# Run after a successful "idf.py build". Produces:
#   release/dji-remote-merged.bin          — single binary for espflash.app
#   release/DJI-Remote-v{VERSION}-esp32.zip — full package (bins + manifest + flash scripts)
#   release/RELEASE_NOTES.md               — release description template
#
# VERSION defaults to "dev" if not provided.

set -euo pipefail

VERSION="${1:-dev}"
PROJECT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
RELEASE_DIR="$PROJECT_ROOT/release"
PACKAGE_DIR="$RELEASE_DIR/package"

# ---------------------------------------------------------------------------
# Verify build artifacts
# ---------------------------------------------------------------------------
BOOTLOADER="$BUILD_DIR/bootloader/bootloader.bin"
PARTITION_TABLE="$BUILD_DIR/partition_table/partition-table.bin"
APP="$BUILD_DIR/dji_camera_bluetooth_control.bin"

for f in "$BOOTLOADER" "$PARTITION_TABLE" "$APP"; do
    if [ ! -f "$f" ]; then
        echo "ERROR: Missing $f — run 'idf.py build' first." >&2
        exit 1
    fi
done

# ---------------------------------------------------------------------------
# Prepare output directory
# ---------------------------------------------------------------------------
rm -rf "$RELEASE_DIR"
mkdir -p "$PACKAGE_DIR"

# ---------------------------------------------------------------------------
# 1. Create merged binary (for espflash.app and similar tools)
# ---------------------------------------------------------------------------
echo "Creating merged binary..."
python -m esptool --chip esp32 merge_bin \
    -o "$RELEASE_DIR/dji-remote-merged.bin" \
    --flash_mode dio \
    --flash_size 16MB \
    --flash_freq 80m \
    0x1000  "$BOOTLOADER" \
    0x8000  "$PARTITION_TABLE" \
    0x10000 "$APP"

# ---------------------------------------------------------------------------
# 2. Copy binaries into package
# ---------------------------------------------------------------------------
cp "$BOOTLOADER"      "$PACKAGE_DIR/bootloader.bin"
cp "$PARTITION_TABLE"  "$PACKAGE_DIR/partition-table.bin"
cp "$APP"              "$PACKAGE_DIR/dji_camera_bluetooth_control.bin"
cp "$RELEASE_DIR/dji-remote-merged.bin" "$PACKAGE_DIR/"

# ---------------------------------------------------------------------------
# 3. Generate ESP Web Tools manifest
# ---------------------------------------------------------------------------
cat > "$PACKAGE_DIR/manifest.json" <<EOF
{
  "name": "DJI-Remote",
  "version": "$VERSION",
  "builds": [
    {
      "chipFamily": "ESP32",
      "parts": [
        { "path": "bootloader.bin", "offset": 4096 },
        { "path": "partition-table.bin", "offset": 32768 },
        { "path": "dji_camera_bluetooth_control.bin", "offset": 65536 }
      ]
    }
  ]
}
EOF

# ---------------------------------------------------------------------------
# 4. Copy flash scripts
# ---------------------------------------------------------------------------
FLASH_DIR="$PROJECT_ROOT/tools/flash"
if [ -d "$FLASH_DIR" ]; then
    cp "$FLASH_DIR/flash-mac.sh"  "$PACKAGE_DIR/" 2>/dev/null || true
    cp "$FLASH_DIR/flash-win.bat" "$PACKAGE_DIR/" 2>/dev/null || true
    cp "$FLASH_DIR/README.txt"    "$PACKAGE_DIR/" 2>/dev/null || true
fi

# ---------------------------------------------------------------------------
# 5. Create ZIP archive
# ---------------------------------------------------------------------------
echo "Creating ZIP archive..."
cd "$RELEASE_DIR"
zip -r "DJI-Remote-v${VERSION}-esp32.zip" package/

# ---------------------------------------------------------------------------
# 6. Generate release notes
# ---------------------------------------------------------------------------
cat > "$RELEASE_DIR/RELEASE_NOTES.md" <<EOF
## DJI-Remote v${VERSION}

Firmware for M5Stack Basic V2.7 (ESP32).

### Flash Options

**Option 1 — Web Flash (easiest)**
Use [espflash.app](https://espflash.app) or [ESP Web Tools](https://web.esptool.io/)
with the merged binary \`dji-remote-merged.bin\`.

**Option 2 — Manual Flash**
Download the ZIP, extract, and follow the instructions in \`README.txt\`.

### Flash Settings
| Setting | Value |
|---------|-------|
| Chip | ESP32 |
| Flash mode | DIO |
| Flash size | 16 MB |
| Flash frequency | 80 MHz |
EOF

echo ""
echo "Release artifacts created in $RELEASE_DIR/"
ls -lh "$RELEASE_DIR/dji-remote-merged.bin" "$RELEASE_DIR/DJI-Remote-v${VERSION}-esp32.zip"

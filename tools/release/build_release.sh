#!/usr/bin/env bash
# Build a release package from compiled ESP-IDF firmware.
#
# Usage: build_release.sh <VERSION> <BOARD>
#   BOARD: m5stack_basic_v27 | waveshare_s3_lcd19
#
# Run after a successful "idf.py build". Produces:
#   release/dji-remote-v{VERSION}-{board-slug}.bin  — merged binary per board
#   release/RELEASE_NOTES.md                        — copied from repo root
#
# The script does NOT wipe the release/ directory, so it can be called once per
# board target and both binaries accumulate in the same output folder.

set -euo pipefail

VERSION="${1:?Usage: build_release.sh <VERSION> <BOARD>}"
BOARD="${2:?Usage: build_release.sh <VERSION> <BOARD>}"
PROJECT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
RELEASE_DIR="$PROJECT_ROOT/release"

# ---------------------------------------------------------------------------
# Derive chip type and bootloader offset from BOARD
# ---------------------------------------------------------------------------
case "$BOARD" in
  m5stack_basic_v27)
    CHIP="esp32"
    BOOTLOADER_OFFSET="0x1000"
    BOARD_SLUG="m5stack-basic-v27"
    ;;
  waveshare_s3_lcd19)
    CHIP="esp32s3"
    BOOTLOADER_OFFSET="0x0"
    BOARD_SLUG="waveshare-s3-lcd19"
    ;;
  *)
    echo "ERROR: Unknown BOARD '$BOARD'. Use m5stack_basic_v27 or waveshare_s3_lcd19." >&2
    exit 1
    ;;
esac

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
# Prepare output directory (no rm — both targets accumulate)
# ---------------------------------------------------------------------------
mkdir -p "$RELEASE_DIR"

# ---------------------------------------------------------------------------
# 1. Create merged binary (for espflash.app and similar tools)
# ---------------------------------------------------------------------------
MERGED_BIN="$RELEASE_DIR/dji-remote-v${VERSION}-${BOARD_SLUG}.bin"
echo "Creating merged binary for $BOARD ($CHIP)..."
python -m esptool --chip "$CHIP" merge_bin \
    -o "$MERGED_BIN" \
    --flash_mode dio \
    --flash_size 16MB \
    --flash_freq 80m \
    "$BOOTLOADER_OFFSET" "$BOOTLOADER" \
    0x8000  "$PARTITION_TABLE" \
    0x10000 "$APP"

# ---------------------------------------------------------------------------
# 2. Copy release notes from repo root
# ---------------------------------------------------------------------------
if [ ! -f "$PROJECT_ROOT/RELEASE_NOTES.md" ]; then
    echo "WARNING: $PROJECT_ROOT/RELEASE_NOTES.md not found — skipping." >&2
else
    cp "$PROJECT_ROOT/RELEASE_NOTES.md" "$RELEASE_DIR/RELEASE_NOTES.md"
fi

echo ""
echo "Release artifacts created in $RELEASE_DIR/"
ls -lh "$MERGED_BIN"

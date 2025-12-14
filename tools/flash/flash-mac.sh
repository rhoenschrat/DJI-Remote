#!/bin/sh

# ----------------------------------------
# DJI Remote Firmware Flash Script (macOS)
# ----------------------------------------

if [ -z "$1" ]; then
  echo ""
  echo "ERROR: No serial port specified."
  echo ""
  echo "Usage:"
  echo "  ./flash_mac.sh <serial-port>"
  echo ""
  echo "Example:"
  echo "  ./flash_mac.sh /dev/tty.usbserial-XXXXXXXX"
  echo ""
  exit 1
fi

PORT="$1"

echo ""
echo "Flashing DJI Remote firmware..."
echo "Using serial port: $PORT"
echo ""

./esptool --no-stub --chip esp32 \
  --port "$PORT" \
  --baud 921600 \
  --before default-reset \
  --after hard-reset \
  write-flash -z \
  --flash-mode dio \
  --flash-size 16MB \
  --flash-freq 80m \
  0x1000 bootloader.bin \
  0x8000 partition-table.bin \
  0x10000 dji_camera_bluetooth_control.bin

RESULT=$?

if [ $RESULT -ne 0 ]; then
  echo ""
  echo "ERROR: Flashing failed."
  exit $RESULT
fi

echo ""
echo "Flashing completed successfully."
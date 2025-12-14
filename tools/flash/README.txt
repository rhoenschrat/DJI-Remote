DJI Remote – Firmware Flash Instructions
=======================================

This ZIP contains precompiled firmware binaries for the DJI Remote project.
No ESP-IDF or development environment is required.

Flashing is done using Espressif's official "esptool" utility.


----------------------------------------
REQUIRED TOOL
----------------------------------------

You need "esptool" from Espressif.

Download it here:
https://github.com/espressif/esptool/releases

Download the appropriate version for your operating system and extract it.

Tested with:
- esptool version 5.1.0

Other recent versions should also work, but only version 5.1.0
has been verified.

IMPORTANT:
After downloading, copy the esptool binary into this "flash" directory,
so that it is located next to the flash script and the .bin files.


----------------------------------------
FILES IN THIS DIRECTORY
----------------------------------------

bootloader.bin
partition-table.bin
dji_camera_bluetooth_control.bin
flasher_args.json
flash_mac.sh      (macOS / Linux)
flash_win.bat     (Windows)


----------------------------------------
FLASHING ON macOS
----------------------------------------

1. Connect the device via USB.
2. Find the serial port.
   It usually looks like:
   /dev/cu.usbserial-XXXXXXXX

3. Make the flash script executable (only once):
   chmod +x flash_mac.sh

4. Run the flash script:
   ./flash_mac.sh /dev/cu.usbserial-XXXXXXXX

Replace the port name with the one on your system.


----------------------------------------
FLASHING ON WINDOWS
----------------------------------------

1. Connect the device via USB.
2. Open Device Manager and note the COM port
   (for example: COM5).

3. Run the flash script:
   flash_win.bat COM5

Replace COM5 with the correct port number.


----------------------------------------
TECHNICAL DETAILS
----------------------------------------

Chip:            ESP32
Flash mode:      DIO
Flash frequency: 80 MHz
Flash size:      16 MB

The firmware is written to the following flash offsets:

0x1000   bootloader.bin
0x8000   partition-table.bin
0x10000  application firmware


----------------------------------------
TROUBLESHOOTING
----------------------------------------

- Use a high-quality USB cable (not charge-only).
- Avoid USB hubs if possible.
- If flashing fails, try a lower baud rate
  by editing the flash script.
- On macOS, prefer "/dev/cu.*" over "/dev/tty.*" ports.

macOS Gatekeeper notice:
On recent macOS versions, the first execution of "esptool"
may be blocked by Gatekeeper because it is an unsigned binary.

If the flash script fails immediately on the first run:
1. Open "System Settings"
2. Go to "Privacy & Security"
3. Scroll down to the security section
4. You should see a message that "esptool" was blocked
5. Click "Allow Anyway"

After allowing it, run the flash script again.


----------------------------------------
DISCLAIMER
----------------------------------------

Use this firmware at your own risk.
The authors are not responsible for any damage
caused by flashing or using this firmware.
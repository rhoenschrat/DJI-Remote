@echo off
REM ----------------------------------------
REM DJI Remote Firmware Flash Script (Windows)
REM ----------------------------------------

IF "%~1"=="" (
  echo.
  echo ERROR: No serial port specified.
  echo.
  echo Usage:
  echo   flash_win.bat COMx
  echo.
  echo Example:
  echo   flash_win.bat COM5
  echo.
  pause
  exit /b 1
)

SET PORT=%1

echo.
echo Flashing DJI Remote firmware...
echo Using serial port: %PORT%
echo.

esptool.exe --no-stub --chip esp32 ^
  --port %PORT% ^
  --baud 921600 ^
  --before default-reset ^
  --after hard-reset ^
  write-flash -z ^
  --flash-mode dio ^
  --flash-size 16MB ^
  --flash-freq 80m ^
  0x1000 bootloader.bin ^
  0x8000 partition-table.bin ^
  0x10000 dji_camera_bluetooth_control.bin

IF ERRORLEVEL 1 (
  echo.
  echo ERROR: Flashing failed.
  pause
  exit /b 1
)

echo.
echo Flashing completed successfully.
pause
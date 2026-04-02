# Waveshare ESP32-S3-LCD-1.9 — Hardware Reference

## Board Overview

The [Waveshare ESP32-S3-LCD-1.9](https://www.waveshare.com/esp32-s3-lcd-1.9.htm)
is a compact development board built around the ESP32-S3R8 (Xtensa LX7,
dual-core, 8 MB OPI PSRAM, 16 MB flash). It features a 320x170 IPS display
driven by an ST7789V2 controller over SPI, oriented in landscape mode.

The board has no onboard user buttons — three external momentary push-buttons
must be wired to GPIO5, GPIO6, and GPIO16. An external ATGM336H (or compatible)
GPS module connects via UART.

## Pin Assignment Table

| Signal | GPIO | Notes |
|--------|------|-------|
| Button A (Shutter) | GPIO5 | Internal pull-up, idle HIGH, pressed LOW |
| Button B (Next / Nav) | GPIO6 | Internal pull-up, idle HIGH, pressed LOW |
| Button C (Options / Highlight / Sleep) | GPIO16 | Internal pull-up, idle HIGH, pressed LOW |
| Backlight | GPIO14 | LEDC PWM, active-low |
| Display RST | GPIO9 | SPI2 |
| Display CLK | GPIO10 | SPI2 |
| Display DC | GPIO11 | SPI2 |
| Display CS | GPIO12 | SPI2 |
| Display MOSI | GPIO13 | SPI2 |
| GPS TX (to module RX) | GPIO18 | UART1, 9600 bps |
| GPS RX (from module TX) | GPIO17 | UART1, 9600 bps |

> **Note:** GPIO assignments are from the official Waveshare schematic
> (Altium, 2025-11-29); the Waveshare wiki pin table is incorrect.

## Wiring Diagrams

### Three External Buttons

Each button connects between its GPIO pin and GND. Internal pull-ups are
enabled in firmware — no external resistors required.

```
Button A (GPIO5)
----------------
GPIO5 ----[ internal pull-up ]----> 3.3V
   |
  [Button]
   |
  GND


Button B (GPIO6)
-----------------
GPIO6 ----[ internal pull-up ]----> 3.3V
   |
  [Button]
   |
  GND


Button C (GPIO16)
------------------
GPIO16 ----[ internal pull-up ]----> 3.3V
   |
  [Button]
   |
  GND
```

### GPS Module (ATGM336H)

UART connection between the Waveshare board and GPS module:

```
Waveshare               ATGM336H
---------               --------
GPIO18 (TX) ──────────► RX
GPIO17 (RX) ◄────────── TX
3.3V ──────────────────► VCC
GND ───────────────────► GND
```

## Required External Components

| Component | Purpose |
|-----------|---------|
| 3x momentary push-button | Button A / B / C |
| ATGM336H (or compatible) GPS module | Live GPS injection |
| USB-C cable | Power and flashing |

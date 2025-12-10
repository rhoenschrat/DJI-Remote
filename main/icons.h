/*
 * icons.h
 * 24x24 bitmap icon data for DJI Camera Remote UI
 */

#ifndef ICONS_H
#define ICONS_H

#include <stdint.h>

// Icon declarations - defined in icons.c
extern const uint8_t bluetooth_icon[72];
extern const uint8_t record_icon[72];
extern const uint8_t mode_icon[72];
extern const uint8_t gps_icon[72];
extern const uint8_t sleep_icon[72];
extern const uint8_t key_icon[72];
extern const uint8_t location_icon[72];
extern const uint8_t sd_card_icon[72];
extern const uint8_t battery_0_icon[72];
extern const uint8_t battery_25_icon[72];
extern const uint8_t battery_50_icon[72];
extern const uint8_t battery_75_icon[72];
extern const uint8_t battery_100_icon[72];
extern const uint8_t pause_icon[72];
extern const uint8_t video_icon[72];
extern const uint8_t photo_icon[72];
extern const uint8_t scanning_icon[72];
extern const uint8_t found_icon[72];
extern const uint8_t connecting_icon[72];
extern const uint8_t back_icon[72];
extern const uint8_t highlight_icon[72];
extern const uint8_t wakeup_icon[72];
extern const uint8_t connect_icon[72];
extern const uint8_t disconnect_icon[72];
extern const uint8_t pair_icon[72];
extern const uint8_t unpair_icon[72];
extern const uint8_t select_icon[72];
extern const uint8_t next_icon[72];
extern const uint8_t options_icon[72];
extern const uint8_t switch_icon[72];

// Icon color definitions (RBG565 - swapped green/blue for this display)
#define ICON_BLUE    0x07E0   // Blue for Connect (was green in RGB)
#define ICON_RED     0xF800   // Red for Record (stays same)
#define ICON_ORANGE  0xFC00   // Orange for Mode Switch
#define ICON_YELLOW  0xF81F   // Yellow for Quick Switch (red + green)
#define ICON_WHITE   0xFFFF   // White default
#define ICON_CYAN    0x07FF   // Cyan alternate

// Color definitions are in m5stack_basic_v27_hal.h

#endif // ICONS_H
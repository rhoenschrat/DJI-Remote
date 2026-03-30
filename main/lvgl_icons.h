#ifndef LVGL_ICONS_H
#define LVGL_ICONS_H

#include "lvgl.h"

/**
 * Initialize all LVGL icon descriptors.
 *
 * Converts the 1-bit packed icon bitmaps (icons.c) into LV_COLOR_FORMAT_A8
 * alpha-channel images suitable for LVGL rendering.  Must be called after
 * LVGL is initialised but before any screen is created.
 *
 * Original bitmaps use inverted polarity (1 = background, 0 = icon shape).
 * The conversion flips this so that opaque (0xFF) pixels represent the icon
 * and transparent (0x00) pixels represent the background.  The actual
 * foreground colour is set per-widget via lv_obj_set_style_image_recolor().
 */
void lvgl_icons_init(void);

extern lv_image_dsc_t lvgl_bluetooth_icon;
extern lv_image_dsc_t lvgl_record_icon;
extern lv_image_dsc_t lvgl_mode_icon;
extern lv_image_dsc_t lvgl_gps_icon;
extern lv_image_dsc_t lvgl_sleep_icon;
extern lv_image_dsc_t lvgl_key_icon;
extern lv_image_dsc_t lvgl_location_icon;
extern lv_image_dsc_t lvgl_sd_card_icon;
extern lv_image_dsc_t lvgl_battery_0_icon;
extern lv_image_dsc_t lvgl_battery_25_icon;
extern lv_image_dsc_t lvgl_battery_50_icon;
extern lv_image_dsc_t lvgl_battery_75_icon;
extern lv_image_dsc_t lvgl_battery_100_icon;
extern lv_image_dsc_t lvgl_pause_icon;
extern lv_image_dsc_t lvgl_video_icon;
extern lv_image_dsc_t lvgl_photo_icon;
extern lv_image_dsc_t lvgl_scanning_icon;
extern lv_image_dsc_t lvgl_found_icon;
extern lv_image_dsc_t lvgl_connecting_icon;
extern lv_image_dsc_t lvgl_back_icon;
extern lv_image_dsc_t lvgl_highlight_icon;
extern lv_image_dsc_t lvgl_wakeup_icon;
extern lv_image_dsc_t lvgl_connect_icon;
extern lv_image_dsc_t lvgl_disconnect_icon;
extern lv_image_dsc_t lvgl_pair_icon;
extern lv_image_dsc_t lvgl_unpair_icon;
extern lv_image_dsc_t lvgl_select_icon;
extern lv_image_dsc_t lvgl_next_icon;
extern lv_image_dsc_t lvgl_options_icon;
extern lv_image_dsc_t lvgl_switch_icon;

#endif

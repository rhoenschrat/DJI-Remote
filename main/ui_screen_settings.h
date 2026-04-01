#ifndef UI_SCREEN_SETTINGS_H
#define UI_SCREEN_SETTINGS_H

#include "lvgl.h"
#include <stdbool.h>

/* Settings menu item indices */
#define SETTINGS_ITEM_BACK              0
#define SETTINGS_ITEM_MODE_SWITCH       1
#define SETTINGS_ITEM_SLEEP_WAKEUP      2
#define SETTINGS_ITEM_CONNECT_DISCONNECT 3
#define SETTINGS_ITEM_PAIR_UNPAIR       4
#define SETTINGS_ITEM_COUNT             5

/**
 * Create the Settings screen LVGL widget tree.
 * @param camera_index Slot index (0-2) whose settings to show
 * Returns the screen object suitable for lv_screen_load().
 * Caller must hold lvgl_port_lock.
 */
lv_obj_t *ui_screen_settings_create(int camera_index);

/**
 * Sync Settings screen widgets with current camera state.
 * Caller must hold lvgl_port_lock.
 */
void ui_screen_settings_update(void);

/**
 * Tear down Settings screen and null out widget pointers.
 */
void ui_screen_settings_destroy(void);

/**
 * Returns true if the screen has been created and not yet destroyed.
 */
bool ui_screen_settings_is_created(void);

/**
 * Button handlers for settings screen (called with lock held).
 */
void ui_screen_settings_button_a(void);
void ui_screen_settings_button_b(void);

/**
 * Show a notification in the settings screen notification area.
 * Caller must hold lvgl_port_lock.
 */
void ui_screen_settings_set_notification(const char *text, lv_color_t color);

/**
 * Get current selection index.
 */
int ui_screen_settings_get_selection(void);

/**
 * Get the camera slot index this settings screen was created for.
 */
int ui_screen_settings_get_camera_index(void);

#endif

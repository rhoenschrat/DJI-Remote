#ifndef UI_SCREEN_MAIN_H
#define UI_SCREEN_MAIN_H

#include "lvgl.h"

/**
 * Create the Main screen LVGL widget tree.
 * Returns the screen object suitable for lv_screen_load().
 * Caller must hold lvgl_port_lock.
 */
lv_obj_t *ui_screen_main_create(void);

/**
 * Sync Main screen widgets with current camera state.
 * Safe to call repeatedly -- LVGL only redraws changed properties.
 * Caller must hold lvgl_port_lock.
 */
void ui_screen_main_update(void);

/**
 * Tear down Main screen and null out widget pointers.
 * Called before switching away if auto_del is not used.
 */
void ui_screen_main_destroy(void);

/**
 * Returns true if the screen has been created and not yet destroyed.
 */
bool ui_screen_main_is_created(void);

/**
 * Show a temporary notification message in the status bar area.
 * Caller must hold lvgl_port_lock.
 */
void ui_screen_main_set_notification(const char *text, lv_color_t color);

/**
 * Clear the notification label.
 * Caller must hold lvgl_port_lock.
 */
void ui_screen_main_clear_notification(void);

#endif

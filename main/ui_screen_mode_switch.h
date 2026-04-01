#ifndef UI_SCREEN_MODE_SWITCH_H
#define UI_SCREEN_MODE_SWITCH_H

#include "lvgl.h"
#include <stdbool.h>

/**
 * Create the Mode Switch screen LVGL widget tree.
 * @param camera_index Slot index (0-2) whose modes to show
 * Returns the screen object suitable for lv_screen_load().
 * Caller must hold lvgl_port_lock.
 */
lv_obj_t *ui_screen_mode_switch_create(int camera_index);

/**
 * Sync Mode Switch screen widgets with current camera state.
 * LVGL only redraws if widget values actually changed.
 * Caller must hold lvgl_port_lock.
 */
void ui_screen_mode_switch_update(void);

/**
 * Tear down Mode Switch screen and null out widget pointers.
 */
void ui_screen_mode_switch_destroy(void);

/**
 * Returns true if the screen has been created and not yet destroyed.
 */
bool ui_screen_mode_switch_is_created(void);

/**
 * Button handlers for mode switch screen (called with lock held).
 */
void ui_screen_mode_switch_button_a(void);
void ui_screen_mode_switch_button_b(void);

/**
 * Get the camera slot index this mode switch screen was created for.
 */
int ui_screen_mode_switch_get_camera_index(void);

#endif

#ifndef UI_SCREEN_PAIRING_H
#define UI_SCREEN_PAIRING_H

#include "lvgl.h"
#include <stdbool.h>

/**
 * Create the Pairing screen LVGL widget tree.
 * @param camera_index Slot index (0-2) being paired
 * Returns the screen object suitable for lv_screen_load().
 * Caller must hold lvgl_port_lock.
 */
lv_obj_t *ui_screen_pairing_create(int camera_index);

/**
 * Sync Pairing screen widgets with current scan results.
 * Caller must hold lvgl_port_lock.
 */
void ui_screen_pairing_update(void);

/**
 * Tear down Pairing screen and null out widget pointers.
 */
void ui_screen_pairing_destroy(void);

/**
 * Returns true if the screen has been created and not yet destroyed.
 */
bool ui_screen_pairing_is_created(void);

/**
 * Button handlers for pairing screen (called with lock held).
 * Button C (cancel / Back) is handled directly in ui.c.
 */
void ui_screen_pairing_button_a(void);
void ui_screen_pairing_button_b(void);

/**
 * Show a notification in the pairing screen notification area.
 * Caller must hold lvgl_port_lock.
 */
void ui_screen_pairing_set_notification(const char *text, lv_color_t color);

/**
 * Add a discovered camera entry to the pairing screen.
 * @param name Camera name string
 * @param index Index in the discovered list (0-based, equals the entry index).
 *              May be called again with an existing index to rename it.
 */
void ui_screen_pairing_add_camera(const char *name, int index);

/**
 * Reset all discovered camera entries (e.g. when scan restarts).
 */
void ui_screen_pairing_reset_cameras(void);

/**
 * Get the current selection index (= index into the discovered list).
 */
int ui_screen_pairing_get_selection(void);

/**
 * Get the camera slot index this pairing screen was created for.
 */
int ui_screen_pairing_get_camera_index(void);

#endif

#ifndef UI_SCREEN_SLOTS_H
#define UI_SCREEN_SLOTS_H

#include "lvgl.h"
#include <stdbool.h>

/*
 * Slots configuration screens (plan 0018, ADR-026)
 *
 * One module, two modes sharing one widget set:
 * - SLOTS_MODE_CYCLE: the four cycle entries by name (A: Select -> entry
 *   screen, B: Next, C: Back -> Main Screen, saves the cycle)
 * - SLOTS_MODE_ENTRY: Slot 1/2/3 of one entry with a green check mark for
 *   active slots (A: toggle, B: Next, C: Back -> cycle list)
 *
 * Single full-width column on every preset, no title, no busy icon.
 * Button logic lives in ui.c; all functions need lvgl_port_lock held.
 */

typedef enum {
    SLOTS_MODE_CYCLE = 0,
    SLOTS_MODE_ENTRY,
} ui_slots_mode_t;

/**
 * Create the screen.
 * @param mode      Cycle list or entry screen
 * @param entry_pos Cycle list: initially highlighted entry; entry screen:
 *                  the entry being edited (row 0 highlighted)
 * Returns the screen object suitable for lv_screen_load().
 */
lv_obj_t *ui_screen_slots_create(ui_slots_mode_t mode, int entry_pos);

/** Sync names, check marks and the selection bar with the slot cycle */
void ui_screen_slots_update(void);

/** Tear down the screen and null out widget pointers */
void ui_screen_slots_destroy(void);

/** True if the screen has been created and not yet destroyed */
bool ui_screen_slots_is_created(void);

/** Button B: highlight the next row (wraps) */
void ui_screen_slots_button_b(void);

/** Highlighted row (cycle list: entry position; entry screen: slot) */
int ui_screen_slots_get_selection(void);

/** Entry edited by the entry screen */
int ui_screen_slots_get_entry_pos(void);

/** Flash the highlighted row's label red for about 1 s (rejected toggle) */
void ui_screen_slots_flash_rejected(void);

#endif /* UI_SCREEN_SLOTS_H */

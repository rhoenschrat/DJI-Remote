/*
 * Configurable Main Screen slot cycle (plan 0018, ADR-026)
 *
 * The Next button cycles through SLOT_CYCLE_ENTRIES entries. Each entry is a
 * set of camera slots (bit n = slot n). An entry with exactly one slot
 * behaves like a single-camera selection, an entry with two or more slots
 * like "All Cameras" restricted to its slots, and Empty entries are skipped.
 *
 * Pure logic plus NVS persistence: no LVGL, no BLE, no camera state. All
 * functions are called from the main task only (button handlers, screen
 * updates), so no locking is needed.
 */

#ifndef SLOT_CYCLE_H
#define SLOT_CYCLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Number of cycle entries (fixed positions 1-4 in the UI) */
#define SLOT_CYCLE_ENTRIES   4
/* Number of camera slots an entry can contain; equals NUM_CAMERAS (checked
 * in ui.c, which owns NUM_CAMERAS) */
#define SLOT_CYCLE_NUM_SLOTS 3

/* Slot set of one entry, bit n = slot n */
typedef uint8_t slot_mask_t;

#define SLOT_CYCLE_SLOT_BIT(slot) ((slot_mask_t)(1u << (slot)))
#define SLOT_CYCLE_MASK_ALL       ((slot_mask_t)((1u << SLOT_CYCLE_NUM_SLOTS) - 1u))
#define SLOT_CYCLE_MASK_EMPTY     ((slot_mask_t)0)

/* Longest name: "Slot 1 + 2 + 3" */
#define SLOT_CYCLE_NAME_LEN  16

/**
 * @brief Load the cycle from NVS, or the factory default
 *
 * Default `[1] -> [2] -> [3] -> [1+2+3]`. Missing, malformed or rule-breaking
 * (all entries Empty) stored data falls back to the default. The selection starts at the first
 * non-empty entry. Call once after nvs_flash_init(), before the first main
 * screen update.
 */
void slot_cycle_init(void);

/** @brief Slot set of cycle entry `pos` (0..SLOT_CYCLE_ENTRIES-1) */
slot_mask_t slot_cycle_get_entry(int pos);

/** @brief Current cycle position (0..SLOT_CYCLE_ENTRIES-1) */
int slot_cycle_get_position(void);

/**
 * @brief Toggle one slot in one entry, enforcing the consistency rule
 *
 * Rule: at least one entry is non-empty. A cycle without any single-slot
 * entry is allowed; the Options Menu is then unreachable, but the Slots
 * configuration stays reachable via Button B during the boot scan (ADR-026).
 * A toggle that would empty every entry is rejected and changes nothing.
 *
 * @return true if applied, false if rejected or arguments are invalid
 */
bool slot_cycle_toggle(int pos, int slot);

/**
 * @brief Advance the selection to the next non-empty entry (wraps)
 * @return The new position
 */
int slot_cycle_next(void);

/**
 * @brief Keep the selection on a non-empty entry after editing
 *
 * Keeps the current entry if it is non-empty, otherwise moves to the next
 * non-empty entry (wrapping).
 */
void slot_cycle_normalize_selection(void);

/** @brief Slot set of the selected entry */
slot_mask_t slot_cycle_selected_mask(void);

/** @brief Whether `slot` belongs to the selected entry */
bool slot_cycle_selected_has(int slot);

/**
 * @brief The slot of a single-slot selection
 * @return Slot 0..SLOT_CYCLE_NUM_SLOTS-1 if the selected entry has exactly
 *         one slot, -1 for a multi-slot selection
 */
int slot_cycle_selected_single(void);

/**
 * @brief Lowest slot of the selected entry (0 if none; never happens after
 *        slot_cycle_init())
 */
int slot_cycle_selected_first(void);

/** @brief Number of slots in `mask` */
int slot_cycle_mask_count(slot_mask_t mask);

/**
 * @brief Display name of a slot set: "Slot 1", "Slot 2 + 3", "Empty", ...
 * @param len Buffer size; SLOT_CYCLE_NAME_LEN fits every name
 */
void slot_cycle_format_name(slot_mask_t mask, char *buf, size_t len);

/**
 * @brief Persist the cycle if it differs from the last loaded/saved one
 *
 * Main task only, on leaving the configuration screens; never from BLE
 * callbacks, command workers or LVGL timers (ADR-022). Call without the
 * LVGL lock held.
 *
 * @return true if NVS was written
 */
bool slot_cycle_save_if_changed(void);

#endif /* SLOT_CYCLE_H */

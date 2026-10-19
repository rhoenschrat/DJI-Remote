/*
 * Per-slot busy state (plan 0017, ADR-025)
 *
 * Remembers, per camera slot, which user-visible action is still running
 * (connect, record start/stop, photo, sleep, wake, snapshot, disconnect,
 * unpair) so the screens can show a busy indicator until the camera's
 * status reflects the result.
 *
 * Threading: slot_busy_begin() is called from the main task only.
 * slot_busy_clear*() and slot_busy_is_active() may be called from the main
 * task and from the LVGL task (screen timers). The state is held in
 * atomics; no lock is needed and no LVGL object is touched here.
 */

#ifndef SLOT_BUSY_H
#define SLOT_BUSY_H

#include <stdbool.h>
#include <stdint.h>

/* Safety limit: an entry never stays active longer than this */
#define SLOT_BUSY_TIMEOUT_MS  10000

typedef enum {
    SLOT_BUSY_NONE = 0,
    SLOT_BUSY_CONNECT,      /* done: slot connected (or unresponsive) */
    SLOT_BUSY_REC_START,    /* done: camera reports recording */
    SLOT_BUSY_REC_STOP,     /* done: camera reports not recording */
    SLOT_BUSY_PHOTO,        /* done: the slot's command job has finished */
    SLOT_BUSY_SLEEP,        /* done: camera reports sleeping */
    SLOT_BUSY_WAKE,         /* done: camera reports awake */
    SLOT_BUSY_SNAPSHOT,     /* done: camera reports awake */
    SLOT_BUSY_DISCONNECT,   /* done: link released, slot not connected */
    SLOT_BUSY_UNPAIR,       /* done: slot unpaired */
    SLOT_BUSY_KIND_COUNT
} slot_busy_kind_t;

/** Bit for slot_busy_clear_kinds() */
#define SLOT_BUSY_BIT(kind)  (1u << (kind))

/**
 * @brief Mark an action as started on a slot (main task)
 *
 * Replaces any running entry of the slot (latest action wins) and restarts
 * the safety window. Logs one INFO line.
 */
void slot_busy_begin(int slot, slot_busy_kind_t kind);

/** @brief End the slot's entry, whatever its kind (e.g. action failed) */
void slot_busy_clear(int slot);

/**
 * @brief End the slot's entry only if its kind is in @p kind_mask
 *
 * Used by failure paths that must not end an unrelated, newer action
 * (e.g. a failed highlight job must not clear a running wake).
 */
void slot_busy_clear_kinds(int slot, uint32_t kind_mask);

/**
 * @brief Whether an action is still running on the slot
 *
 * Evaluates the entry first: when the camera state shows the result, the
 * slot is no longer connected (except for connect/disconnect/unpair, which
 * have their own targets) or the safety limit has passed, the entry is
 * cleared and false is returned.
 */
bool slot_busy_is_active(int slot);

/** @brief Evaluate all slots (main loop), so finished entries never linger */
void slot_busy_evaluate_all(void);

#endif /* SLOT_BUSY_H */

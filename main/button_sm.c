#include "button_sm.h"

#include <stddef.h>

/* Debounce: the debounced level follows the raw level only after
 * BUTTON_DEBOUNCE_SAMPLES consecutive samples that differ from it, so a
 * single-sample glitch (e.g. ESP32 erratum GPIO-3.11 on GPIO39) never
 * becomes a press. Returns true when the debounced level changed. */
static bool debounce(button_sm_t *sm, bool raw_pressed)
{
    if (raw_pressed == sm->pressed) {
        sm->diff_count = 0;
        return false;
    }
    if (++sm->diff_count < BUTTON_DEBOUNCE_SAMPLES) {
        return false;
    }
    sm->diff_count = 0;
    sm->pressed = raw_pressed;
    return true;
}

static void enter_lockout(button_sm_t *sm, uint32_t now_ms)
{
    sm->release_ms = now_ms;
#if BUTTON_RELEASE_LOCKOUT_MS > 0
    sm->state = BTN_ST_LOCKOUT;
#else
    /* No lockout: accept the next press right away */
    sm->state = BTN_ST_IDLE;
#endif
}

button_event_t button_sm_step(button_sm_t *sm, bool raw_pressed, uint32_t now_ms,
                              bool has_long, button_note_t *note)
{
    button_note_t dummy;
    if (note == NULL) {
        note = &dummy;
    }
    *note = BTN_NOTE_NONE;

    if (sm->state == BTN_ST_INIT) {
        /* Boot guard: take the first sample as the debounced level. A button
         * that is already pressed is ignored until it was released once. */
        sm->pressed = raw_pressed;
        sm->diff_count = 0;
        if (raw_pressed) {
            sm->state = BTN_ST_IGNORED_HELD;
            sm->stuck_noted = true;
            *note = BTN_NOTE_BOOT_HELD;
        } else {
            sm->state = BTN_ST_IDLE;
        }
        return BTN_EVENT_NONE;
    }

    bool changed = debounce(sm, raw_pressed);
    bool released = changed && !sm->pressed;

    switch (sm->state) {
    case BTN_ST_IDLE:
        /* Level-triggered: a press that started during the lockout and is
         * still held is accepted as soon as the lockout ends */
        if (!sm->pressed) {
            return BTN_EVENT_NONE;
        }
        sm->press_ms = now_ms;
        sm->has_long = has_long;
        sm->stuck_noted = false;
        if (has_long) {
            sm->state = BTN_ST_PRESSED;
            return BTN_EVENT_NONE;
        }
        /* No long-press function in this context: click on press (low
         * latency, like before), never LONG */
        sm->state = BTN_ST_WAIT_RELEASE;
        return BTN_EVENT_CLICK;

    case BTN_ST_PRESSED:
        if (released) {
            enter_lockout(sm, now_ms);
            return BTN_EVENT_CLICK;     /* held shorter than the threshold */
        }
        if ((uint32_t)(now_ms - sm->press_ms) >= BUTTON_LONG_PRESS_MS) {
            sm->state = BTN_ST_WAIT_RELEASE;
            return BTN_EVENT_LONG;      /* once, at the threshold, no repeat */
        }
        return BTN_EVENT_NONE;

    case BTN_ST_WAIT_RELEASE:
        if (released) {
            enter_lockout(sm, now_ms);
            return BTN_EVENT_NONE;
        }
        if (!sm->stuck_noted &&
            (uint32_t)(now_ms - sm->press_ms) >= BUTTON_STUCK_LIMIT_MS) {
            sm->stuck_noted = true;
            sm->state = BTN_ST_IGNORED_HELD;
            *note = BTN_NOTE_STUCK;
        }
        return BTN_EVENT_NONE;

    case BTN_ST_IGNORED_HELD:
        if (released) {
            enter_lockout(sm, now_ms);
        }
        return BTN_EVENT_NONE;

    case BTN_ST_LOCKOUT:
#if BUTTON_RELEASE_LOCKOUT_MS > 0
        if ((uint32_t)(now_ms - sm->release_ms) >= BUTTON_RELEASE_LOCKOUT_MS) {
            sm->state = BTN_ST_IDLE;
        }
#else
        sm->state = BTN_ST_IDLE;    /* not entered without a lockout */
#endif
        return BTN_EVENT_NONE;

    case BTN_ST_INIT:
    default:
        sm->state = BTN_ST_INIT;
        return BTN_EVENT_NONE;
    }
}

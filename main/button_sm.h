/*
 * Per-button input state machine (ADR-024)
 *
 * Pure logic: no GPIO, no UI, no ESP-IDF dependencies, so it is
 * board-independent and can be compiled and tested on the host.
 * button_input.c owns the pin sources and calls button_sm_step() once per
 * main-loop cycle for every physical button.
 */

#ifndef BUTTON_SM_H
#define BUTTON_SM_H

#include <stdint.h>
#include <stdbool.h>

/* Hold time that turns a press into a long press. Initial value; the final
 * value is to be confirmed with a test ride wearing motorcycle gloves. */
#define BUTTON_LONG_PRESS_MS       1000
/* Inputs are ignored for this long after every release. 0 = no lockout:
 * the debounced release (BUTTON_DEBOUNCE_SAMPLES) already rejects contact
 * bounce, and a 200 ms lockout made quick repeated presses (e.g. Next)
 * feel sluggish and swallowed taps (ADR-024). */
#define BUTTON_RELEASE_LOCKOUT_MS  0
/* A hold this long is reported once as stuck; nothing more is emitted until
 * the button is released (e.g. remote wedged in a tank bag) */
#define BUTTON_STUCK_LIMIT_MS      10000
/* Consecutive equal samples needed to accept a press or a release */
#define BUTTON_DEBOUNCE_SAMPLES    2

/** Logical button; internal and external buttons map to the same ids */
typedef enum {
    BTN_A = 0,
    BTN_B,
    BTN_C,
    BTN_COUNT
} button_id_t;

/** Event emitted by the state machine */
typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_CLICK,
    BTN_EVENT_LONG
} button_event_t;

/** Side information for logging; never changes the emitted event */
typedef enum {
    BTN_NOTE_NONE = 0,
    BTN_NOTE_BOOT_HELD,     /* pressed at the first evaluation, ignored until released */
    BTN_NOTE_STUCK,         /* held >= BUTTON_STUCK_LIMIT_MS, ignored until released */
} button_note_t;

typedef enum {
    BTN_ST_INIT = 0,        /* no sample evaluated yet */
    BTN_ST_IDLE,            /* released, accepting presses */
    BTN_ST_PRESSED,         /* held, long-press context: CLICK on release or LONG at threshold */
    BTN_ST_WAIT_RELEASE,    /* event already emitted for this press; wait for release */
    BTN_ST_IGNORED_HELD,    /* boot-held or stuck; emit nothing, wait for release */
    BTN_ST_LOCKOUT,         /* released; inputs ignored for BUTTON_RELEASE_LOCKOUT_MS */
} button_state_t;

/** State of one physical button; zero-initialise before the first step */
typedef struct {
    button_state_t state;
    bool     pressed;        /* debounced level */
    uint8_t  diff_count;     /* consecutive samples that differ from `pressed` */
    bool     has_long;       /* long-press context, latched at press */
    bool     stuck_noted;    /* BTN_NOTE_STUCK already reported for this press */
    uint32_t press_ms;       /* time the press was accepted */
    uint32_t release_ms;     /* time the release was accepted */
} button_sm_t;

/**
 * @brief Evaluate one sample of a button
 *
 * @param sm        Button state
 * @param raw_pressed Raw level of this sample (true = pressed)
 * @param now_ms    Monotonic milliseconds (wrap-safe, unsigned arithmetic)
 * @param has_long  Whether the current context has a long-press function for
 *                  this button; only read when a press is accepted
 * @param note      Optional; receives side information for logging
 * @return BTN_EVENT_CLICK, BTN_EVENT_LONG or BTN_EVENT_NONE
 */
button_event_t button_sm_step(button_sm_t *sm, bool raw_pressed, uint32_t now_ms,
                              bool has_long, button_note_t *note);

#endif /* BUTTON_SM_H */

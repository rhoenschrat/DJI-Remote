/*
 * Unified, non-blocking button input (ADR-024)
 *
 * Every physical button (internal A/B/C on both boards, external A/B/C on
 * the M5Stack when UI_ENABLE_EXTERNAL_BUTTONS is 1) runs its own instance
 * of the state machine in button_sm.c. button_input_poll() samples all of
 * them once per main-loop cycle and hands CLICK/LONG events to
 * ui_handle_button_event(). Nothing here blocks.
 */

#ifndef BUTTON_INPUT_H
#define BUTTON_INPUT_H

#include "esp_err.h"
#include "button_sm.h"

/**
 * @brief Configure the external button pins (if enabled)
 *
 * Internal buttons are configured by the board HAL. External pins are plain
 * inputs without interrupts. Call once from the main task before the first
 * button_input_poll().
 */
esp_err_t button_input_init(void);

/**
 * @brief Sample every button once and dispatch emitted events
 *
 * Main task only (handlers take the LVGL lock themselves). Call once per
 * main-loop cycle.
 */
void button_input_poll(void);

#endif /* BUTTON_INPUT_H */

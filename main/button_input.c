/*
 * Unified, non-blocking button input (ADR-024)
 *
 * Source table: one entry per physical button, each with its own state
 * machine instance (button_sm.c). Internal and external buttons with the
 * same logical id dispatch to the same handler but never share state, so a
 * glitch on one source cannot affect the other.
 */

#include "button_input.h"

#include <stddef.h>
#include "board_hal.h"
#include "ui.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "BTN"

/* External Buttons Feature Flag (ADR-008)
 *
 * 1: external buttons A/B/C on GPIO26/21/22 (M5Stack bottom header), active
 *    LOW, polled through the same state machine as the internal buttons.
 *    The user must provide the wiring (see docs/ARCHITECTURE.md §1.2).
 * 0: the firmware never touches GPIO26, GPIO21 or GPIO22.
 *
 * The Waveshare ESP32-S3 has no external button header; GPIOs 22/25 do not
 * exist on that chip (used internally for SPI flash/PSRAM). External
 * sleep/wake trigger pins are not wired. */
#if defined(CONFIG_BOARD_WAVESHARE_S3_LCD19)
#define UI_ENABLE_EXTERNAL_BUTTONS 0
#else
#define UI_ENABLE_EXTERNAL_BUTTONS 1
#endif

#if UI_ENABLE_EXTERNAL_BUTTONS
#define EXT_BTN_A_PIN  26   /* internal pull-up */
#define EXT_BTN_B_PIN  21   /* internal pull-up + external 10k */
#define EXT_BTN_C_PIN  22   /* external 10k pull-up only */
#endif

typedef struct {
    button_id_t  id;
    const char  *name;
    bool       (*is_pressed)(void);   /* raw level, true = pressed */
    button_sm_t  sm;
} button_source_t;

static bool int_a_pressed(void) { return HAL_BTN_A_PRESSED(); }
static bool int_b_pressed(void) { return HAL_BTN_B_PRESSED(); }
static bool int_c_pressed(void) { return HAL_BTN_C_PRESSED(); }

#if UI_ENABLE_EXTERNAL_BUTTONS
static bool ext_a_pressed(void) { return gpio_get_level(EXT_BTN_A_PIN) == 0; }
static bool ext_b_pressed(void) { return gpio_get_level(EXT_BTN_B_PIN) == 0; }
static bool ext_c_pressed(void) { return gpio_get_level(EXT_BTN_C_PIN) == 0; }
#endif

static button_source_t s_sources[] = {
    { BTN_A, "A",          int_a_pressed, {0} },
    { BTN_B, "B",          int_b_pressed, {0} },
    { BTN_C, "C",          int_c_pressed, {0} },
#if UI_ENABLE_EXTERNAL_BUTTONS
    { BTN_A, "A (ext G26)", ext_a_pressed, {0} },
    { BTN_B, "B (ext G21)", ext_b_pressed, {0} },
    { BTN_C, "C (ext G22)", ext_c_pressed, {0} },
#endif
};

#define NUM_SOURCES (sizeof(s_sources) / sizeof(s_sources[0]))

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

esp_err_t button_input_init(void)
{
#if UI_ENABLE_EXTERNAL_BUTTONS
    /* Plain inputs, no interrupts. G26 and G21 use the internal pull-up;
     * G22 relies on the external 10k pull-up. */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << EXT_BTN_A_PIN) | (1ULL << EXT_BTN_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure external buttons A/B: %s", esp_err_to_name(ret));
        return ret;
    }
    io_conf.pin_bit_mask = (1ULL << EXT_BTN_C_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure external button C: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "External buttons enabled (A=G26, B=G21, C=G22, polled, active LOW)");
#else
    ESP_LOGI(TAG, "External buttons disabled");
#endif
    return ESP_OK;
}

void button_input_poll(void)
{
    for (size_t i = 0; i < NUM_SOURCES; i++) {
        button_source_t *src = &s_sources[i];
        button_note_t note;
        /* Time and context are taken per source: a handler dispatched for an
         * earlier source may block (e.g. deferred reconnect) */
        button_event_t ev = button_sm_step(&src->sm, src->is_pressed(), now_ms(),
                                           ui_button_has_long_press(src->id), &note);

        if (note == BTN_NOTE_BOOT_HELD) {
            ESP_LOGW(TAG, "Button %s pressed at boot - ignored until released", src->name);
        } else if (note == BTN_NOTE_STUCK) {
            ESP_LOGW(TAG, "Button %s held for %d s - ignored until released",
                     src->name, BUTTON_STUCK_LIMIT_MS / 1000);
        }

        if (ev != BTN_EVENT_NONE) {
            ESP_LOGI(TAG, "Button %s: %s", src->name,
                     ev == BTN_EVENT_LONG ? "LONG" : "CLICK");
            ui_handle_button_event(src->id, ev);
        }
    }
}

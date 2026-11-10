/*
 * Per-slot busy state (plan 0017, ADR-025)
 *
 * Each slot holds one packed atomic word (kind in the low 8 bits, a
 * generation counter above) and the start time. Clearing uses
 * compare-and-swap on the packed word, so an evaluation in the LVGL task
 * that raced with a new slot_busy_begin() on the main task can never clear
 * the newer entry, even if it has the same kind.
 */

#include "slot_busy.h"

#include <stdatomic.h>
#include "ui.h"
#include "ble.h"
#include "command_worker.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "SLOT_BUSY"

#define WORD_KIND(w)        ((slot_busy_kind_t)((w) & 0xFFu))
#define WORD_GEN(w)         ((w) >> 8)
#define WORD_MAKE(kind, g)  (((uint32_t)(g) << 8) | ((uint32_t)(kind) & 0xFFu))

typedef struct {
    _Atomic uint32_t word;       /* kind + generation */
    _Atomic uint32_t start_ms;   /* written before the word */
} slot_busy_entry_t;

static slot_busy_entry_t s_busy[NUM_CAMERAS];

static const char *const s_kind_names[SLOT_BUSY_KIND_COUNT] = {
    [SLOT_BUSY_NONE]       = "none",
    [SLOT_BUSY_CONNECT]    = "connect",
    [SLOT_BUSY_REC_START]  = "start recording",
    [SLOT_BUSY_REC_STOP]   = "stop recording",
    [SLOT_BUSY_PHOTO]      = "photo",
    [SLOT_BUSY_SLEEP]      = "sleep",
    [SLOT_BUSY_WAKE]       = "wake",
    [SLOT_BUSY_SNAPSHOT]   = "snapshot",
    [SLOT_BUSY_DISCONNECT] = "disconnect",
    [SLOT_BUSY_UNPAIR]     = "unpair",
};

static inline bool slot_valid(int slot)
{
    return slot >= 0 && slot < NUM_CAMERAS;
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static const char *kind_name(slot_busy_kind_t kind)
{
    return ((unsigned)kind < SLOT_BUSY_KIND_COUNT) ? s_kind_names[kind] : "?";
}

void slot_busy_begin(int slot, slot_busy_kind_t kind)
{
    if (!slot_valid(slot) || kind == SLOT_BUSY_NONE || kind >= SLOT_BUSY_KIND_COUNT) {
        return;
    }
    slot_busy_entry_t *e = &s_busy[slot];
    atomic_store(&e->start_ms, now_ms());
    uint32_t w = atomic_load(&e->word);
    while (!atomic_compare_exchange_weak(&e->word, &w, WORD_MAKE(kind, WORD_GEN(w) + 1))) {
        /* w was reloaded; retry */
    }
    ESP_LOGI(TAG, "Slot %d busy: %s", slot, kind_name(kind));
}

/* Clear the entry if it still holds exactly the word that was evaluated */
static bool clear_word(int slot, uint32_t expected, const char *why)
{
    slot_busy_entry_t *e = &s_busy[slot];
    uint32_t w = expected;
    if (atomic_compare_exchange_strong(&e->word, &w,
                                       WORD_MAKE(SLOT_BUSY_NONE, WORD_GEN(expected)))) {
        ESP_LOGD(TAG, "Slot %d idle: %s (%s)", slot, kind_name(WORD_KIND(expected)), why);
        return true;
    }
    return false;
}

void slot_busy_clear_kinds(int slot, uint32_t kind_mask)
{
    if (!slot_valid(slot)) return;
    uint32_t w = atomic_load(&s_busy[slot].word);
    slot_busy_kind_t kind = WORD_KIND(w);
    if (kind != SLOT_BUSY_NONE && (kind_mask & SLOT_BUSY_BIT(kind))) {
        clear_word(slot, w, "ended");
    }
}

void slot_busy_clear(int slot)
{
    slot_busy_clear_kinds(slot, UINT32_MAX);
}

/* Whether the camera state shows that the action has finished. Fields are
 * copied first: the notification task updates them concurrently. */
static bool kind_done(int slot, slot_busy_kind_t kind)
{
    const camera_state_t *st = &g_camera_states[slot];
    camera_connection_state_t conn = st->connection_state;
    bool sleeping  = st->is_sleeping || st->power_mode == 3;
    bool recording = st->is_recording;

    switch (kind) {
    case SLOT_BUSY_CONNECT:
        /* An unresponsive link ends the connect too (ADR-019) */
        return conn == CAM_STATE_CONNECTED || conn == CAM_STATE_UNRESPONSIVE;
    case SLOT_BUSY_DISCONNECT:
        return conn != CAM_STATE_CONNECTED && conn != CAM_STATE_UNRESPONSIVE &&
               ble_is_link_released(slot);
    case SLOT_BUSY_UNPAIR:
        return conn == CAM_STATE_UNPAIRED;
    default:
        break;
    }

    /* Camera actions: a slot that is no longer connected (disconnect,
     * unresponsive) cannot finish them */
    if (conn != CAM_STATE_CONNECTED) {
        return true;
    }

    switch (kind) {
    case SLOT_BUSY_REC_START: return recording;
    case SLOT_BUSY_REC_STOP:  return !recording;
    /* A photo produces no lasting state change; the job's end is the result */
    case SLOT_BUSY_PHOTO:     return !command_worker_slot_busy(slot);
    case SLOT_BUSY_SLEEP:     return sleeping;
    case SLOT_BUSY_WAKE:
    case SLOT_BUSY_SNAPSHOT:  return !sleeping;
    default:                  return true;
    }
}

bool slot_busy_is_active(int slot)
{
    if (!slot_valid(slot)) return false;
    slot_busy_entry_t *e = &s_busy[slot];

    uint32_t w = atomic_load(&e->word);
    slot_busy_kind_t kind = WORD_KIND(w);
    if (kind == SLOT_BUSY_NONE) {
        return false;
    }
    /* Read after the word: begin() stores the time first */
    uint32_t start = atomic_load(&e->start_ms);

    if (kind_done(slot, kind)) {
        clear_word(slot, w, "done");
        return false;
    }
    if ((uint32_t)(now_ms() - start) >= SLOT_BUSY_TIMEOUT_MS) {
        if (clear_word(slot, w, "safety limit")) {
            ESP_LOGW(TAG, "Slot %d: %s still running after %d s - indicator cleared",
                     slot, kind_name(kind), SLOT_BUSY_TIMEOUT_MS / 1000);
        }
        return false;
    }
    return true;
}

void slot_busy_evaluate_all(void)
{
    for (int i = 0; i < NUM_CAMERAS; i++) {
        (void)slot_busy_is_active(i);
    }
}

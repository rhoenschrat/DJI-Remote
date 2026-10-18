/*
 * Persistent fault diagnostics (ADR-022)
 *
 * Records connection faults and restarts in a ring of 128 records in the
 * dedicated NVS partition "diag". Producers never block: diag_record() only
 * bumps a RAM counter and queues a fixed-size record; a low-priority task
 * writes it, limited by a token bucket. The partition is read and erased
 * from the host with esptool (no read-out code in the firmware).
 */
#ifndef __DIAG_H__
#define __DIAG_H__

#include <stdint.h>

/* Event types. Append-only: never renumber or reuse a value; a layout
 * change bumps DIAG_RECORD_VERSION in diag.c and the host tool. */
typedef enum {
    DIAG_EVT_RESTART = 1,        /* slot 0xFF, detail = esp_reset_reason() */
    DIAG_EVT_DISCONNECT = 2,     /* unexpected link loss, detail = BLE reason */
    DIAG_EVT_UNRESPONSIVE = 3,   /* detail = silence in 100 ms units */
    DIAG_EVT_RECOVERED = 4,      /* detail = silence in 100 ms units */
    DIAG_EVT_TEARDOWN = 5,       /* detail = silence in 100 ms units */
    DIAG_EVT_CONNECT_ABORT = 6,  /* link lost during a connect flow, detail = BLE reason */
} diag_event_type_t;

/**
 * @brief Initialize diagnostics (call once, early in app_main)
 *
 * Opens the "diag" partition, increments the boot counter and writes the
 * RESTART record synchronously, then starts the writer task. On any error
 * the module stays disabled and diag_record() only counts in RAM.
 */
void diag_init(void);

/**
 * @brief Record one event (any task except ISRs; never blocks)
 *
 * @param type      Event type
 * @param slot      Camera slot 0..2 (ignored for DIAG_EVT_RESTART)
 * @param device_id Camera model ID (g_camera_states[slot].device_id), 0 if unknown
 * @param detail    Type-specific detail (see diag_event_type_t)
 */
void diag_record(diag_event_type_t type, int slot, uint32_t device_id, uint16_t detail);

/** Converts a silence time in ms to 100 ms units, saturating at UINT16_MAX */
static inline uint16_t diag_silence_units(uint32_t silence_ms) {
    uint32_t units = silence_ms / 100;
    return (units > UINT16_MAX) ? UINT16_MAX : (uint16_t)units;
}

#endif

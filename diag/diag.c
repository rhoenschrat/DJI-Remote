/*
 * Persistent fault diagnostics (ADR-022)
 *
 * Storage: NVS partition "diag", namespace "diag".
 *   r000..r127  blob diag_record_t, ring slot = seq % DIAG_RING_SIZE
 *   meta        blob diag_meta_t (boot counter, next seq, counters, ...)
 *
 * The record and meta layouts below are read by the host tool
 * tools/diag/dji_diag.py. Any change to them must be made there too, must
 * bump DIAG_RECORD_VERSION / DIAG_META_VERSION, and is checked with
 * "tools/diag/dji_diag selftest".
 *
 * Threading:
 *   - diag_record() runs on any task (mostly link_evt_task): atomic counter
 *     increment, timestamp, xQueueSend(..., 0). Never logs, never touches
 *     NVS, never takes the GPS mutex.
 *   - The diag task (prio 1) is the only user of the NVS handle after
 *     diag_init(). It adds boot_id, seq and the GPS UTC time and writes,
 *     limited by a token bucket (burst DIAG_BUCKET_MAX, one token per
 *     DIAG_BUCKET_REFILL_MS, DIAG_BUCKET_INITIAL after boot).
 *   - NVS errors are counted and ignored (at most one WARN per boot).
 */

#include <string.h>
#include <stdio.h>
#include <stdatomic.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "diag.h"
#include "gps_reader.h"

#define TAG "DIAG"

#define DIAG_PARTITION          "diag"
#define DIAG_NAMESPACE          "diag"
#define DIAG_META_KEY           "meta"

#define DIAG_RECORD_VERSION     1
#define DIAG_META_VERSION       1

#define DIAG_RING_SIZE          128
#define DIAG_BUCKET_MAX         10
#define DIAG_BUCKET_REFILL_MS   30000
#define DIAG_BUCKET_INITIAL     3
#define DIAG_QUEUE_DEPTH        16
#define DIAG_TASK_PRIO          1
#define DIAG_TASK_STACK         3072
#define DIAG_GPS_MAX_AGE_MS     5000

#define DIAG_NUM_SLOTS          3
#define DIAG_NUM_TYPES          6      /* DIAG_EVT_RESTART .. DIAG_EVT_CONNECT_ABORT */
#define DIAG_SLOT_NONE          0xFF

#define DIAG_FLAG_UTC_VALID     0x01

/* ---- Persistent layouts (little-endian, packed; must match the tool) ---- */

typedef struct __attribute__((packed)) {
    uint8_t  version;      /* DIAG_RECORD_VERSION */
    uint8_t  type;         /* diag_event_type_t */
    uint8_t  slot;         /* 0..2, DIAG_SLOT_NONE for RESTART */
    uint8_t  flags;        /* bit0: utc_valid */
    uint16_t boot_id;      /* boot counter at the time of the event */
    uint16_t detail;       /* type-specific */
    uint32_t uptime_ms;    /* since boot, captured by the producer */
    uint32_t utc_epoch;    /* seconds since 1970 UTC, 0 if !utc_valid */
    uint32_t device_id;    /* camera model ID, 0 for RESTART */
    uint32_t seq;          /* global monotonic record number */
} diag_record_t;
_Static_assert(sizeof(diag_record_t) == 24, "diag_record_t layout changed; update the host tool");

typedef struct __attribute__((packed)) {
    uint8_t  version;                                  /* DIAG_META_VERSION */
    uint8_t  reserved;
    uint16_t boot_id;
    uint32_t next_seq;
    uint32_t counters[DIAG_NUM_SLOTS][DIAG_NUM_TYPES]; /* RESTART counted at slot 0 */
    uint32_t suppressed;                               /* records not written (rate limit) */
    uint32_t queue_drops;                              /* records lost on a full queue */
    uint32_t write_errors;                             /* failed NVS writes */
    char     fw_version[32];                           /* firmware of the last boot */
} diag_meta_t;
_Static_assert(sizeof(diag_meta_t) == 124, "diag_meta_t layout changed; update the host tool");

/* Queue item: what the producer knows */
typedef struct {
    uint8_t  type;
    uint8_t  slot;
    uint16_t detail;
    uint32_t device_id;
    uint32_t uptime_ms;
} diag_item_t;

/* ---- State ---- */

static bool          s_enabled = false;
static nvs_handle_t  s_nvs = 0;
static QueueHandle_t s_queue = NULL;

/* Written by producers, read by the diag task */
static atomic_uint   s_counters[DIAG_NUM_SLOTS][DIAG_NUM_TYPES];
static atomic_uint   s_queue_drops = 0;
static atomic_bool   s_meta_dirty = false;

/* Diag task only (and diag_init() before the task exists) */
static uint16_t      s_boot_id = 0;
static uint32_t      s_next_seq = 0;
static uint32_t      s_suppressed = 0;
static uint32_t      s_write_errors = 0;
static bool          s_warned = false;
static char          s_fw_version[32];

static inline uint32_t uptime_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void note_error(const char *what, esp_err_t err) {
    s_write_errors++;
    if (!s_warned) {
        s_warned = true;
        ESP_LOGW(TAG, "%s failed: %s (further errors not logged)", what, esp_err_to_name(err));
    }
}

static void build_meta(diag_meta_t *meta) {
    memset(meta, 0, sizeof(*meta));
    meta->version = DIAG_META_VERSION;
    meta->boot_id = s_boot_id;
    meta->next_seq = s_next_seq;
    for (int s = 0; s < DIAG_NUM_SLOTS; s++) {
        for (int t = 0; t < DIAG_NUM_TYPES; t++) {
            meta->counters[s][t] = atomic_load(&s_counters[s][t]);
        }
    }
    meta->suppressed = s_suppressed;
    meta->queue_drops = atomic_load(&s_queue_drops);
    meta->write_errors = s_write_errors;
    memcpy(meta->fw_version, s_fw_version, sizeof(meta->fw_version));
}

/* Writes meta (and optionally a record) and commits. Returns true on success. */
static bool persist(const diag_record_t *rec) {
    esp_err_t err;
    bool ok = true;

    if (rec != NULL) {
        char key[8];
        snprintf(key, sizeof(key), "r%03u", (unsigned)(rec->seq % DIAG_RING_SIZE));
        err = nvs_set_blob(s_nvs, key, rec, sizeof(*rec));
        if (err != ESP_OK) {
            note_error("record write", err);
            ok = false;
        }
    }

    /* Clear before the snapshot so an event during the write marks it again */
    atomic_store(&s_meta_dirty, false);
    diag_meta_t meta;
    build_meta(&meta);
    err = nvs_set_blob(s_nvs, DIAG_META_KEY, &meta, sizeof(meta));
    if (err != ESP_OK) {
        note_error("meta write", err);
        atomic_store(&s_meta_dirty, true);
        ok = false;
    }

    err = nvs_commit(s_nvs);
    if (err != ESP_OK) {
        note_error("commit", err);
        atomic_store(&s_meta_dirty, true);
        ok = false;
    }
    return ok;
}

static void fill_record(diag_record_t *rec, const diag_item_t *item, uint32_t seq) {
    memset(rec, 0, sizeof(*rec));
    rec->version = DIAG_RECORD_VERSION;
    rec->type = item->type;
    rec->slot = item->slot;
    rec->boot_id = s_boot_id;
    rec->detail = item->detail;
    rec->uptime_ms = item->uptime_ms;
    rec->device_id = item->device_id;
    rec->seq = seq;

    /* GPS time now, minus the time the record spent in the queue */
    uint32_t utc_now;
    if (gps_get_utc_epoch(&utc_now, DIAG_GPS_MAX_AGE_MS)) {
        uint32_t queued_s = (uptime_ms() - item->uptime_ms) / 1000;
        rec->utc_epoch = utc_now - queued_s;
        rec->flags |= DIAG_FLAG_UTC_VALID;
    }
}

static void diag_task(void *arg) {
    int tokens = DIAG_BUCKET_INITIAL;
    TickType_t last_refill = xTaskGetTickCount();
    const TickType_t refill_ticks = pdMS_TO_TICKS(DIAG_BUCKET_REFILL_MS);

    for (;;) {
        diag_item_t item;
        bool have_item = (xQueueReceive(s_queue, &item, refill_ticks) == pdTRUE);

        /* Refill whole tokens and keep the remainder */
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - last_refill;
        if (elapsed >= refill_ticks) {
            uint32_t n = elapsed / refill_ticks;
            last_refill += n * refill_ticks;
            tokens = (tokens + (int)n > DIAG_BUCKET_MAX) ? DIAG_BUCKET_MAX : tokens + (int)n;
        }

        if (have_item) {
            if (tokens > 0) {
                diag_record_t rec;
                fill_record(&rec, &item, s_next_seq);
                tokens--;
                if (persist(&rec)) {
                    s_next_seq++;  /* only written records advance: no gaps in the ring */
                }
            } else {
                s_suppressed++;
                atomic_store(&s_meta_dirty, true);
            }
        } else if (atomic_load(&s_meta_dirty) && tokens > 0) {
            /* Persist counters after a storm */
            tokens--;
            persist(NULL);
        }
    }
}

static esp_err_t open_partition(void) {
    esp_err_t err = nvs_flash_init_partition(DIAG_PARTITION);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* Only this partition may ever be erased here */
        ESP_LOGW(TAG, "Partition unusable (%s), erasing it", esp_err_to_name(err));
        err = nvs_flash_erase_partition(DIAG_PARTITION);
        if (err == ESP_OK) {
            err = nvs_flash_init_partition(DIAG_PARTITION);
        }
    }
    if (err != ESP_OK) {
        return err;
    }
    return nvs_open_from_partition(DIAG_PARTITION, DIAG_NAMESPACE, NVS_READWRITE, &s_nvs);
}

static void load_meta(void) {
    diag_meta_t meta;
    size_t len = sizeof(meta);
    esp_err_t err = nvs_get_blob(s_nvs, DIAG_META_KEY, &meta, &len);
    if (err != ESP_OK || len != sizeof(meta) || meta.version != DIAG_META_VERSION) {
        return;  /* missing or foreign: start from zero */
    }
    s_boot_id = meta.boot_id;
    s_next_seq = meta.next_seq;
    s_suppressed = meta.suppressed;
    s_write_errors = meta.write_errors;
    atomic_store(&s_queue_drops, meta.queue_drops);
    for (int s = 0; s < DIAG_NUM_SLOTS; s++) {
        for (int t = 0; t < DIAG_NUM_TYPES; t++) {
            atomic_store(&s_counters[s][t], meta.counters[s][t]);
        }
    }
}

void diag_init(void) {
    if (s_enabled) {
        return;
    }

    esp_err_t err = open_partition();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Diagnostics disabled: %s", esp_err_to_name(err));
        return;
    }

    load_meta();

    s_boot_id++;
    memset(s_fw_version, 0, sizeof(s_fw_version));
    strncpy(s_fw_version, esp_app_get_description()->version, sizeof(s_fw_version) - 1);

    /* RESTART: the only synchronous write, before any other task exists */
    atomic_fetch_add(&s_counters[0][DIAG_EVT_RESTART - 1], 1);
    diag_item_t item = {
        .type = DIAG_EVT_RESTART,
        .slot = DIAG_SLOT_NONE,
        .detail = (uint16_t)esp_reset_reason(),
        .device_id = 0,
        .uptime_ms = uptime_ms(),
    };
    diag_record_t rec;
    fill_record(&rec, &item, s_next_seq);
    if (persist(&rec)) {
        s_next_seq++;
    }

    s_queue = xQueueCreate(DIAG_QUEUE_DEPTH, sizeof(diag_item_t));
    if (s_queue == NULL ||
        xTaskCreate(diag_task, "diag", DIAG_TASK_STACK, NULL, DIAG_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGW(TAG, "Diagnostics writer not started; counting in RAM only");
        return;
    }

    s_enabled = true;
    ESP_LOGI(TAG, "Diagnostics ready: boot %u, next seq %lu, reset reason %d",
             s_boot_id, (unsigned long)s_next_seq, (int)item.detail);
}

void diag_record(diag_event_type_t type, int slot, uint32_t device_id, uint16_t detail) {
    if (type < DIAG_EVT_RESTART || type > DIAG_EVT_CONNECT_ABORT) {
        return;
    }
    int counter_slot = (type == DIAG_EVT_RESTART) ? 0 : slot;
    if (counter_slot < 0 || counter_slot >= DIAG_NUM_SLOTS) {
        return;
    }

    atomic_fetch_add(&s_counters[counter_slot][type - 1], 1);
    atomic_store(&s_meta_dirty, true);

    if (!s_enabled) {
        return;  /* counter-only no-op */
    }

    diag_item_t item = {
        .type = (uint8_t)type,
        .slot = (type == DIAG_EVT_RESTART) ? DIAG_SLOT_NONE : (uint8_t)slot,
        .detail = detail,
        .device_id = device_id,
        .uptime_ms = uptime_ms(),
    };
    if (xQueueSend(s_queue, &item, 0) != pdTRUE) {
        atomic_fetch_add(&s_queue_drops, 1);
    }
}

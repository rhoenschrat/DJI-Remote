/*
 * DJI Camera Remote Control - Data Management Layer
 *
 * Sits between the BLE layer and the application logic: queues incoming
 * notifications, parses DJI protocol frames, matches responses to commands
 * and paces outgoing frames.
 *
 * Response matching (per camera, ADR-020):
 * - Every entry is keyed by its slot. A frame from camera A can never
 *   complete a waiter of camera B.
 * - Seq-based entries are created by data_write_with_response() and freed
 *   only by their owner (the writer on failure, the waiter on result or
 *   timeout). They are never evicted.
 * - Unsolicited frames without a seq match are stored cmd-based by
 *   (slot, cmd_set, cmd_id), e.g. the camera's 0x00/0x19 during the
 *   handshake, which can arrive before the wait starts. Only cmd-based
 *   entries nobody waits on (waiters == 0) may be evicted or expire.
 * - Status pushes (1D02, 1D06) bypass the table and go only to their
 *   callbacks; nobody waits for them.
 *
 * Notify path:
 * - receive_camera_notify_handler() runs on the NimBLE host task. It copies
 *   the frame into a queue of NOTIFY_QUEUE_DEPTH entries and only counts
 *   drops (no logging per frame).
 * - notify_processing_task parses frames and reports drops at most every
 *   NOTIFY_DROP_REPORT_MS.
 *
 * TX pacing:
 * - Both write functions go through paced_write(): per-slot mutex, at least
 *   TX_MIN_GAP_MS between two frames to the same camera, refuses a write
 *   handle of 0.
 * - The TX mutex covers only the write itself, never a response wait. The
 *   notify task delivers responses and also sends frames (snapshot after
 *   wake), so holding it across a wait could deadlock.
 *
 * Framework: FreeRTOS (mutexes, binary semaphores, timer, queue)
 */

#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "data.h"
#include "ble.h"
#include "dji_protocol_parser.h"

/* Logging tag for ESP_LOG functions */
#define TAG "DATA"

#define DATA_NUM_SLOTS 3

/* Tracked command/response entries. Status pushes no longer take entries,
 * so this only has to cover outstanding commands plus a few unsolicited
 * frames. */
#define MAX_SEQ_ENTRIES 16

/* Cleanup timer interval and max. age of an unused cmd-based entry */
#define CLEANUP_INTERVAL_MS 60000
#define MAX_ENTRY_AGE 120

/* Notification queue (host task -> notify task). Worst case
 * NOTIFY_QUEUE_DEPTH x frame size (<= MTU, ~512 B) of heap. */
#define NOTIFY_QUEUE_DEPTH      64
#define NOTIFY_DROP_REPORT_MS   5000

/* TX pacing per camera */
#define TX_MIN_GAP_MS           30
#define TX_MUTEX_TIMEOUT_MS     200

/* Map mutex timeout */
#define MAP_MUTEX_TIMEOUT_MS    100

/* Debug-only "silent camera" simulation (Plan 0011 tests). Set
 * DATA_DEBUG_SILENT_SLOT to 0..2: the first record-control frame (0x1D/0x03)
 * sent to that slot starts a window of DATA_DEBUG_SILENT_MS in which every
 * received frame of the slot is dropped (once per boot). Must stay -1 in
 * committed code. */
#define DATA_DEBUG_SILENT_SLOT  (-1)
#define DATA_DEBUG_SILENT_MS    8000

#if DATA_DEBUG_SILENT_SLOT >= 0
static volatile bool s_dbg_silent_started = false;
static volatile TickType_t s_dbg_silent_start = 0;

/* Frame layout: 12-byte header, then CmdSet, CmdID */
static void debug_silent_on_tx(int slot, const uint8_t *data, size_t len) {
    if (slot == DATA_DEBUG_SILENT_SLOT && !s_dbg_silent_started &&
        len > 13 && data[12] == 0x1D && data[13] == 0x03) {
        s_dbg_silent_start = xTaskGetTickCount();
        s_dbg_silent_started = true;
        ESP_LOGW(TAG, "DEBUG: camera %d silent for %d ms", slot, DATA_DEBUG_SILENT_MS);
    }
}

static bool debug_silent_drop_rx(int slot) {
    return slot == DATA_DEBUG_SILENT_SLOT && s_dbg_silent_started &&
           (TickType_t)(xTaskGetTickCount() - s_dbg_silent_start) < pdMS_TO_TICKS(DATA_DEBUG_SILENT_MS);
}
#endif

static bool data_layer_initialized = false;

/* Command tracking entry */
typedef struct {
    bool in_use;
    bool is_seq_based;          /* true: keyed by (slot, seq); false: (slot, cmd_set, cmd_id) */
    int slot;
    uint16_t seq;
    uint8_t cmd_set;
    uint8_t cmd_id;
    uint8_t waiters;            /* tasks blocked on sem; such entries are never freed by others */
    void *parse_result;
    size_t parse_result_length;
    SemaphoreHandle_t sem;
    TickType_t last_access_time;
} entry_t;

static entry_t s_entries[MAX_SEQ_ENTRIES];

/* Protects s_entries */
static SemaphoreHandle_t s_map_mutex = NULL;

static TimerHandle_t cleanup_timer = NULL;
static TaskHandle_t notify_task_handle = NULL;
static QueueHandle_t notify_queue = NULL;

/* Dropped notifications (queue full or malloc failure), counted on the
 * host task, reported by the notify task */
static atomic_uint s_notify_dropped = 0;

/* Same drops per slot, cumulative since boot (BLE_DIAG line) */
static atomic_uint s_notify_dropped_slot[DATA_NUM_SLOTS];

/* TX pacing state per slot */
static SemaphoreHandle_t s_tx_mutex[DATA_NUM_SLOTS];
static TickType_t s_last_tx_tick[DATA_NUM_SLOTS];
static bool s_tx_ever[DATA_NUM_SLOTS];

typedef struct {
    int camera_index;
    uint8_t *data;
    size_t data_length;
} notify_data_t;

static void notify_processing_task(void *pvParameters);
static void process_notification_data(int camera_index, const uint8_t *raw_data, size_t raw_data_length);

static inline bool slot_valid(int slot) {
    return slot >= 0 && slot < DATA_NUM_SLOTS;
}

/* ----------------------------------------------------------------
 *  Entry table (all helpers expect s_map_mutex to be held)
 * ---------------------------------------------------------------- */

static void clear_entry_fields(entry_t *entry) {
    entry->in_use = false;
    entry->is_seq_based = false;
    entry->slot = -1;
    entry->seq = 0;
    entry->cmd_set = 0;
    entry->cmd_id = 0;
    entry->waiters = 0;
    entry->last_access_time = 0;
    entry->parse_result_length = 0;
}

static void free_entry(entry_t *entry) {
    if (!entry) return;
    if (entry->parse_result) {
        free(entry->parse_result);
        entry->parse_result = NULL;
    }
    if (entry->sem) {
        vSemaphoreDelete(entry->sem);
        entry->sem = NULL;
    }
    clear_entry_fields(entry);
}

static void reset_entries(void) {
    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        free_entry(&s_entries[i]);
    }
}

static entry_t* find_entry_by_seq(int slot, uint16_t seq) {
    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        entry_t *e = &s_entries[i];
        if (e->in_use && e->is_seq_based && e->slot == slot && e->seq == seq) {
            e->last_access_time = xTaskGetTickCount();
            return e;
        }
    }
    return NULL;
}

static entry_t* find_entry_by_cmd(int slot, uint8_t cmd_set, uint8_t cmd_id) {
    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        entry_t *e = &s_entries[i];
        if (e->in_use && !e->is_seq_based && e->slot == slot &&
            e->cmd_set == cmd_set && e->cmd_id == cmd_id) {
            e->last_access_time = xTaskGetTickCount();
            return e;
        }
    }
    return NULL;
}

/* Free slot, or else the oldest cmd-based entry nobody waits on.
 * Seq-based entries are never evicted (their owner frees them). */
static entry_t* take_free_or_evictable(void) {
    entry_t *oldest = NULL;
    TickType_t oldest_age = 0;
    TickType_t now = xTaskGetTickCount();

    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        entry_t *e = &s_entries[i];
        if (!e->in_use) {
            return e;
        }
        if (!e->is_seq_based && e->waiters == 0) {
            TickType_t age = now - e->last_access_time;  /* wrap-safe */
            if (oldest == NULL || age > oldest_age) {
                oldest = e;
                oldest_age = age;
            }
        }
    }

    if (oldest) {
        ESP_LOGW(TAG, "Evicting unused entry slot=%d cmd_set=0x%02X cmd_id=0x%02X",
                 oldest->slot, oldest->cmd_set, oldest->cmd_id);
        free_entry(oldest);
    }
    return oldest;
}

static entry_t* init_entry(entry_t *e, int slot, bool seq_based, uint16_t seq,
                           uint8_t cmd_set, uint8_t cmd_id) {
    e->sem = xSemaphoreCreateBinary();
    if (e->sem == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        clear_entry_fields(e);
        return NULL;
    }
    e->in_use = true;
    e->is_seq_based = seq_based;
    e->slot = slot;
    e->seq = seq;
    e->cmd_set = cmd_set;
    e->cmd_id = cmd_id;
    e->waiters = 0;
    e->parse_result = NULL;
    e->parse_result_length = 0;
    e->last_access_time = xTaskGetTickCount();
    return e;
}

static entry_t* allocate_entry_by_seq(int slot, uint16_t seq) {
    entry_t *existing = find_entry_by_seq(slot, seq);
    if (existing) {
        if (existing->waiters > 0) {
            /* 16-bit seq wrapped onto a command that is still being waited on */
            ESP_LOGE(TAG, "Slot %d: seq=0x%04X still awaited, refusing reuse", slot, seq);
            return NULL;
        }
        ESP_LOGW(TAG, "Slot %d: overwriting stale entry for seq=0x%04X", slot, seq);
        free_entry(existing);
    }

    entry_t *e = take_free_or_evictable();
    if (!e) {
        ESP_LOGE(TAG, "Slot %d: no entry available for seq=0x%04X", slot, seq);
        return NULL;
    }
    return init_entry(e, slot, true, seq, 0, 0);
}

/* Returns an entry for an unsolicited frame; an existing one is reused and
 * its old result dropped. */
static entry_t* allocate_entry_by_cmd(int slot, uint8_t cmd_set, uint8_t cmd_id) {
    entry_t *existing = find_entry_by_cmd(slot, cmd_set, cmd_id);
    if (existing) {
        if (existing->parse_result) {
            free(existing->parse_result);
            existing->parse_result = NULL;
            existing->parse_result_length = 0;
        }
        return existing;
    }

    entry_t *e = take_free_or_evictable();
    if (!e) {
        ESP_LOGE(TAG, "Slot %d: no entry available for cmd_set=0x%02X cmd_id=0x%02X",
                 slot, cmd_set, cmd_id);
        return NULL;
    }
    return init_entry(e, slot, false, 0, cmd_set, cmd_id);
}

/* Copies the stored result out; caller holds the mutex */
static esp_err_t copy_result(const entry_t *entry, void **out_result, size_t *out_result_length) {
    if (!entry->parse_result) {
        return ESP_ERR_NOT_FOUND;
    }
    *out_result = malloc(entry->parse_result_length);
    if (*out_result == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for out_result");
        return ESP_ERR_NO_MEM;
    }
    memcpy(*out_result, entry->parse_result, entry->parse_result_length);
    *out_result_length = entry->parse_result_length;
    return ESP_OK;
}

/**
 * @brief Periodic cleanup of unused cmd-based entries (timer service task)
 *
 * Entries somebody waits on are never touched.
 */
static void cleanup_old_entries(TimerHandle_t xTimer) {
    TickType_t now = xTaskGetTickCount();
    if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(MAP_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex in cleanup");
        return;
    }
    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        entry_t *e = &s_entries[i];
        if (e->in_use && e->waiters == 0 &&
            (TickType_t)(now - e->last_access_time) > pdMS_TO_TICKS(MAX_ENTRY_AGE * 1000)) {
            if (e->is_seq_based) {
                ESP_LOGD(TAG, "Cleaning up unused entry slot=%d seq=0x%04X", e->slot, e->seq);
            } else {
                ESP_LOGD(TAG, "Cleaning up unused entry slot=%d cmd_set=0x%02X cmd_id=0x%02X",
                         e->slot, e->cmd_set, e->cmd_id);
            }
            free_entry(e);
        }
    }
    xSemaphoreGive(s_map_mutex);
}

/* ----------------------------------------------------------------
 *  Init
 * ---------------------------------------------------------------- */

void data_init(void) {
    s_map_mutex = xSemaphoreCreateMutex();
    if (s_map_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    for (int i = 0; i < DATA_NUM_SLOTS; i++) {
        s_tx_mutex[i] = xSemaphoreCreateMutex();
        if (s_tx_mutex[i] == NULL) {
            ESP_LOGE(TAG, "Failed to create TX mutex for slot %d", i);
            return;
        }
        s_last_tx_tick[i] = 0;
        s_tx_ever[i] = false;
    }

    for (int i = 0; i < MAX_SEQ_ENTRIES; i++) {
        s_entries[i].parse_result = NULL;
        s_entries[i].sem = NULL;
    }
    reset_entries();

    cleanup_timer = xTimerCreate("cleanup_timer", pdMS_TO_TICKS(CLEANUP_INTERVAL_MS), pdTRUE, NULL, cleanup_old_entries);
    if (cleanup_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create cleanup timer");
    } else {
        xTimerStart(cleanup_timer, 0);
    }

    notify_queue = xQueueCreate(NOTIFY_QUEUE_DEPTH, sizeof(notify_data_t));
    if (notify_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create notification queue");
    }

    if (xTaskCreate(notify_processing_task, "notify_processing_task", 4096, NULL, 1, &notify_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create notification processing task");
    }

    data_layer_initialized = true;
    ESP_LOGI(TAG, "Data layer initialized successfully");
}

bool is_data_layer_initialized(void) {
    return data_layer_initialized;
}

/* ----------------------------------------------------------------
 *  TX
 * ---------------------------------------------------------------- */

/**
 * @brief Write one frame to a camera, paced per slot
 *
 * Holds the slot's TX mutex only for the gap wait and the write itself,
 * never for a response wait.
 */
static esp_err_t paced_write(int slot, bool with_rsp, const uint8_t *data, size_t len) {
    if (ble_get_write_handle(slot) == 0) {
        ESP_LOGW(TAG, "Camera %d: write handle not discovered, frame refused", slot);
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_tx_mutex[slot], pdMS_TO_TICKS(TX_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Camera %d: TX mutex busy for %d ms, frame dropped", slot, TX_MUTEX_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }

    if (s_tx_ever[slot]) {
        TickType_t since = xTaskGetTickCount() - s_last_tx_tick[slot];
        TickType_t gap = pdMS_TO_TICKS(TX_MIN_GAP_MS);
        if (since < gap) {
            vTaskDelay(gap - since);
        }
    }

#if DATA_DEBUG_SILENT_SLOT >= 0
    debug_silent_on_tx(slot, data, len);
#endif

    esp_err_t ret;
    if (with_rsp) {
        ret = ble_write_with_response(ble_get_conn_id(slot), ble_get_write_handle(slot), data, len);
    } else {
        ret = ble_write_without_response(ble_get_conn_id(slot), ble_get_write_handle(slot), data, len);
    }

    s_last_tx_tick[slot] = xTaskGetTickCount();
    s_tx_ever[slot] = true;
    xSemaphoreGive(s_tx_mutex[slot]);
    return ret;
}

/**
 * @brief Send a frame whose response will be awaited by seq
 *
 * Creates the (slot, seq) entry before writing so an early response is
 * never lost. On write failure the entry is freed again.
 */
esp_err_t data_write_with_response(int camera_index, uint16_t seq, const uint8_t *raw_data, size_t raw_data_length) {
    if (!raw_data || raw_data_length == 0) {
        ESP_LOGE(TAG, "Invalid data or length");
        return ESP_ERR_INVALID_ARG;
    }
    if (!slot_valid(camera_index)) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(MAP_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_INVALID_STATE;
    }
    entry_t *entry = allocate_entry_by_seq(camera_index, seq);
    xSemaphoreGive(s_map_mutex);
    if (!entry) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = paced_write(camera_index, true, raw_data, raw_data_length);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera %d: write with response failed: %s", camera_index, esp_err_to_name(ret));
        if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(MAP_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            if (entry->in_use && entry->slot == camera_index && entry->seq == seq) {
                free_entry(entry);
            }
            xSemaphoreGive(s_map_mutex);
        }
        return ret;
    }
    return ESP_OK;
}

/**
 * @brief Send a frame nobody waits for (no table entry)
 */
esp_err_t data_write_without_response(int camera_index, uint16_t seq, const uint8_t *raw_data, size_t raw_data_length) {
    (void)seq;
    if (!raw_data || raw_data_length == 0) {
        ESP_LOGE(TAG, "Invalid raw_data or raw_data_length");
        return ESP_ERR_INVALID_ARG;
    }
    if (!slot_valid(camera_index)) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = paced_write(camera_index, false, raw_data, raw_data_length);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera %d: write without response failed: %s", camera_index, esp_err_to_name(ret));
    }
    return ret;
}

/* ----------------------------------------------------------------
 *  Waiting for results
 * ---------------------------------------------------------------- */

/**
 * @brief Wait for the response to (slot, seq)
 *
 * The entry is owned by the waiter: it is freed here on result or timeout.
 */
esp_err_t data_wait_for_result_by_seq(int slot, uint16_t seq, int timeout_ms, void **out_result, size_t *out_result_length) {
    if (!out_result || !out_result_length) {
        ESP_LOGE(TAG, "out_result or out_result_length is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!slot_valid(slot)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(MAP_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_INVALID_STATE;
    }
    entry_t *entry = find_entry_by_seq(slot, seq);
    if (!entry) {
        /* data_write_with_response() creates it; missing means write failed */
        xSemaphoreGive(s_map_mutex);
        ESP_LOGW(TAG, "Slot %d: no entry for seq=0x%04X", slot, seq);
        return ESP_ERR_NOT_FOUND;
    }
    entry->waiters++;
    SemaphoreHandle_t sem = entry->sem;  /* use only the local copy while blocked */
    xSemaphoreGive(s_map_mutex);

    bool got = (xSemaphoreTake(sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);

    /* The mutex must be taken to release the entry; wait as long as needed
     * so the entry can never leak with waiters > 0 */
    xSemaphoreTake(s_map_mutex, portMAX_DELAY);
    entry->waiters--;
    esp_err_t ret;
    if (!got) {
        ESP_LOGW(TAG, "Slot %d: wait for seq=0x%04X timed out", slot, seq);
        ret = ESP_ERR_TIMEOUT;
    } else {
        ret = copy_result(entry, out_result, out_result_length);
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Slot %d: parse result is NULL for seq=0x%04X", slot, seq);
        }
    }
    free_entry(entry);
    xSemaphoreGive(s_map_mutex);
    return ret;
}

/**
 * @brief Wait for an unsolicited frame (slot, cmd_set, cmd_id)
 *
 * A frame that arrived before the call is returned immediately.
 */
esp_err_t data_wait_for_result_by_cmd(int slot, uint8_t cmd_set, uint8_t cmd_id, int timeout_ms,
                                      uint16_t *out_seq, void **out_result, size_t *out_result_length) {
    if (!out_result || !out_seq || !out_result_length) {
        ESP_LOGE(TAG, "out_result, out_seq or out_result_length is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!slot_valid(slot)) {
        return ESP_ERR_INVALID_ARG;
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while (true) {
        if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(MAP_MUTEX_TIMEOUT_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex");
            return ESP_ERR_INVALID_STATE;
        }

        entry_t *entry = find_entry_by_cmd(slot, cmd_set, cmd_id);
        if (entry && entry->parse_result) {
            esp_err_t ret = copy_result(entry, out_result, out_result_length);
            *out_seq = entry->seq;
            if (entry->waiters == 0) {
                free_entry(entry);
            }
            xSemaphoreGive(s_map_mutex);
            return ret;
        }

        if (entry) {
            /* Entry without result yet: block on it */
            entry->waiters++;
            SemaphoreHandle_t sem = entry->sem;
            xSemaphoreGive(s_map_mutex);

            TickType_t elapsed = xTaskGetTickCount() - start;
            TickType_t remaining = (elapsed < timeout_ticks) ? (timeout_ticks - elapsed) : 0;
            bool got = (xSemaphoreTake(sem, remaining) == pdTRUE);

            xSemaphoreTake(s_map_mutex, portMAX_DELAY);
            entry->waiters--;
            esp_err_t ret = ESP_ERR_TIMEOUT;
            if (got) {
                ret = copy_result(entry, out_result, out_result_length);
                *out_seq = entry->seq;
            } else {
                ESP_LOGW(TAG, "Slot %d: wait for cmd_set=0x%02X cmd_id=0x%02X timed out",
                         slot, cmd_set, cmd_id);
            }
            if (entry->waiters == 0) {
                free_entry(entry);
            }
            xSemaphoreGive(s_map_mutex);
            return ret;
        }

        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout_ticks) {
            ESP_LOGW(TAG, "Slot %d: timeout waiting for cmd_set=0x%02X cmd_id=0x%02X, no entry found",
                     slot, cmd_set, cmd_id);
            xSemaphoreGive(s_map_mutex);
            return ESP_ERR_TIMEOUT;
        }

        xSemaphoreGive(s_map_mutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Register camera status update callback
 * 
 * This function registers a callback function for camera status updates. After registration,
 * the callback function will be called to synchronize the latest camera status when specific notifications are received.
 * 
 * @param callback Callback function pointer, pointing to user-defined callback function
 */
static camera_status_update_cb_t status_update_callback = NULL;
void data_register_status_update_callback(camera_status_update_cb_t callback) {
    status_update_callback = callback;
}

static new_camera_status_update_cb_t new_status_update_callback = NULL;
void data_register_new_status_update_callback(new_camera_status_update_cb_t callback) {
    new_status_update_callback = callback;
}

/**
 * @brief Notify task: parses queued frames and reports drops
 *
 * Waits with a timeout so a drop report is also emitted when the queue
 * runs empty after a burst.
 */
static void notify_processing_task(void *pvParameters) {
    notify_data_t notify_data;
    unsigned int reported_drops = 0;
    TickType_t last_report = xTaskGetTickCount();

    while (1) {
        if (xQueueReceive(notify_queue, &notify_data, pdMS_TO_TICKS(NOTIFY_DROP_REPORT_MS)) == pdTRUE) {
            process_notification_data(notify_data.camera_index, notify_data.data, notify_data.data_length);
            free(notify_data.data);
        }

        unsigned int total = atomic_load(&s_notify_dropped);
        TickType_t now = xTaskGetTickCount();
        if (total != reported_drops &&
            (TickType_t)(now - last_report) >= pdMS_TO_TICKS(NOTIFY_DROP_REPORT_MS)) {
            ESP_LOGW(TAG, "Dropped %u notification(s) since last report (total %u)",
                     total - reported_drops, total);
            reported_drops = total;
            last_report = now;
        }
    }
}

/* Hands a private copy of a status push to its callback (callback frees it) */
static void dispatch_status_copy(void (*cb)(int, void *), int camera_index,
                                 const void *parse_result, size_t len) {
    if (!cb || !parse_result || len == 0) return;
    void *copy = malloc(len);
    if (copy == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for status callback");
        return;
    }
    memcpy(copy, parse_result, len);
    cb(camera_index, copy);
}

/**
 * @brief Parse one notification frame and route it (notify task)
 *
 * 1. Response to an outstanding command of this camera (slot, seq) -> entry
 * 2. Status push (1D02 / 1D06) -> callbacks only, not stored
 * 3. Any other unsolicited frame -> cmd-based entry (slot, cmd_set, cmd_id)
 *
 * Status callback copies are made before ownership of parse_result moves
 * into an entry, because a waiter may free it right after.
 */
static void process_notification_data(int camera_index, const uint8_t *raw_data, size_t raw_data_length) {
    if (!raw_data || raw_data_length < 2) {
        ESP_LOGW(TAG, "Notify data is too short or null, skip parse");
        return;
    }
    if (raw_data[0] != 0xAA) {
        return;
    }
#if DATA_DEBUG_SILENT_SLOT >= 0
    if (debug_silent_drop_rx(camera_index)) {
        return;
    }
#endif

    ESP_LOGD(TAG, "RX notification frame (%d bytes):", raw_data_length);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, raw_data, raw_data_length, ESP_LOG_DEBUG);

    protocol_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    int ret = protocol_parse_notification(raw_data, raw_data_length, &frame);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to parse notification frame, error: %d", ret);
        return;
    }

    if (!frame.data || frame.data_length == 0) {
        ESP_LOGW(TAG, "Data segment is empty, skipping data parsing");
        return;
    }

    size_t parse_result_length = 0;
    void *parse_result = protocol_parse_data(frame.data, frame.data_length, frame.cmd_type, &parse_result_length);
    if (parse_result == NULL) {
        ESP_LOGD(TAG, "No parsed result for frame (send-only or unknown command)");
        return;
    }

    uint16_t actual_seq = frame.seq;
    uint8_t actual_cmd_set = frame.data[0];
    uint8_t actual_cmd_id = frame.data[1];
    bool is_status_push = (actual_cmd_set == 0x1D &&
                           (actual_cmd_id == 0x02 || actual_cmd_id == 0x06));
    ESP_LOGD(TAG, "Camera %d: parsed seq=0x%04X, cmd_set=0x%02X, cmd_id=0x%02X",
             camera_index, actual_seq, actual_cmd_set, actual_cmd_id);

    /* Callback copies first: after the table step parse_result may belong
     * to an entry and be freed by a waiter at any time */
    if (actual_cmd_set == 0x1D && actual_cmd_id == 0x02) {
        dispatch_status_copy(status_update_callback, camera_index, parse_result, parse_result_length);
    } else if (actual_cmd_set == 0x1D && actual_cmd_id == 0x06) {
        dispatch_status_copy(new_status_update_callback, camera_index, parse_result, parse_result_length);
    }

    bool stored = false;
    if (xSemaphoreTake(s_map_mutex, pdMS_TO_TICKS(MAP_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        /* A status push never completes a command, even if the camera's own
         * seq happens to equal one of ours */
        entry_t *entry = is_status_push ? NULL : find_entry_by_seq(camera_index, actual_seq);
        if (entry && entry->parse_result == NULL) {
            entry->parse_result = parse_result;
            entry->parse_result_length = parse_result_length;
            xSemaphoreGive(entry->sem);
            stored = true;
        } else if (!entry && !is_status_push) {
            entry = allocate_entry_by_cmd(camera_index, actual_cmd_set, actual_cmd_id);
            if (entry) {
                entry->parse_result = parse_result;
                entry->parse_result_length = parse_result_length;
                entry->seq = actual_seq;
                entry->last_access_time = xTaskGetTickCount();
                xSemaphoreGive(entry->sem);
                stored = true;
            }
        }
        xSemaphoreGive(s_map_mutex);
    } else {
        ESP_LOGE(TAG, "Camera %d: map mutex timeout, frame seq=0x%04X discarded", camera_index, actual_seq);
    }

    if (!stored) {
        free(parse_result);
    }
}

/**
 * @brief Queue a camera notification (NimBLE host task)
 *
 * Copy and enqueue only (ADR-018). Drops are counted, never logged here.
 */
static inline void count_notify_drop(int camera_index) {
    atomic_fetch_add(&s_notify_dropped, 1);
    if (camera_index >= 0 && camera_index < DATA_NUM_SLOTS) {
        atomic_fetch_add(&s_notify_dropped_slot[camera_index], 1);
    }
}

unsigned int data_get_notify_drops(int slot) {
    if (slot < 0 || slot >= DATA_NUM_SLOTS) {
        return 0;
    }
    return atomic_load(&s_notify_dropped_slot[slot]);
}

void receive_camera_notify_handler(int camera_index, const uint8_t *raw_data, size_t raw_data_length) {
    if (!raw_data || raw_data_length < 2 || notify_queue == NULL) {
        return;
    }

    uint8_t *data_copy = malloc(raw_data_length);
    if (data_copy == NULL) {
        count_notify_drop(camera_index);
        return;
    }
    memcpy(data_copy, raw_data, raw_data_length);

    notify_data_t notify_data = {
        .camera_index = camera_index,
        .data = data_copy,
        .data_length = raw_data_length
    };

    if (xQueueSend(notify_queue, &notify_data, 0) != pdTRUE) {
        count_notify_drop(camera_index);
        free(data_copy);
    }
}

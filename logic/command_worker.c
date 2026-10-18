/*
 * Per-slot command workers (ADR-020)
 *
 * Three tasks (cmd_w0..cmd_w2), one per camera slot, each with a job queue
 * of depth 1. A per-slot busy flag is set by command_worker_submit() before
 * the job is queued and cleared by the worker after it has posted the
 * result, so a slot never has more than one job and nothing is applied late
 * to a changed state.
 *
 * Workers never touch LVGL or UI functions. Frames are still built only in
 * command_logic.c. Each camera therefore has at most one outstanding Write
 * Request from a worker (at most 3 of NimBLE's 4 GATT procedures).
 *
 * Direct senders that stay outside the workers: GPS push (GPS task),
 * snapshot after wake (notify task), connect/handshake flow (main task).
 */

#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "command_worker.h"
#include "command_logic.h"
#include "dji_protocol_data_structures.h"
#include "../main/ui.h"  // NUM_CAMERAS

#define TAG "CMD_WORKER"

#define CMD_WORKER_STACK        4096
#define CMD_WORKER_PRIO         2     /* above main/notify (1), below link_evt (3) */
#define CMD_RESULT_QUEUE_DEPTH  8

static QueueHandle_t s_job_queue[NUM_CAMERAS];
static QueueHandle_t s_result_queue = NULL;
static atomic_bool   s_busy[NUM_CAMERAS];
static atomic_uint   s_results_dropped = 0;
static bool          s_initialized = false;

const char *command_worker_job_name(cmd_job_t job) {
    switch (job) {
        case CMD_JOB_RECORD_START:       return "record_start";
        case CMD_JOB_RECORD_STOP:        return "record_stop";
        case CMD_JOB_RECORD_START_ASYNC: return "record_start_async";
        case CMD_JOB_RECORD_STOP_ASYNC:  return "record_stop_async";
        case CMD_JOB_SLEEP:              return "sleep";
        case CMD_JOB_HIGHLIGHT:          return "highlight";
        case CMD_JOB_KEY_QS:             return "key_qs";
        default:                         return "unknown";
    }
}

static inline bool slot_valid(int slot) {
    return slot >= 0 && slot < NUM_CAMERAS;
}

/* Gate shared by submit (press time) and the worker (send time) */
static bool slot_gate_ok(int slot) {
    return command_logic_slot_is_connected(slot);  /* excludes UNRESPONSIVE */
}

/* Runs the job on the worker task; returns status and camera ret_code */
static esp_err_t run_job(int slot, cmd_job_t job, int *ret_code) {
    *ret_code = -1;

    switch (job) {
        case CMD_JOB_RECORD_START:
        case CMD_JOB_RECORD_STOP: {
            record_control_response_frame_t *rsp = (job == CMD_JOB_RECORD_START)
                ? command_logic_start_record(slot)
                : command_logic_stop_record(slot);
            if (rsp == NULL) {
                return ESP_ERR_TIMEOUT;
            }
            *ret_code = rsp->ret_code;
            free(rsp);
            return ESP_OK;
        }
        case CMD_JOB_RECORD_START_ASYNC:
            return command_logic_start_record_async(slot);
        case CMD_JOB_RECORD_STOP_ASYNC:
            return command_logic_stop_record_async(slot);
        case CMD_JOB_SLEEP: {
            camera_power_mode_switch_response_frame_t *rsp = command_logic_power_mode_switch_sleep(slot);
            if (rsp == NULL) {
                return ESP_ERR_TIMEOUT;
            }
            *ret_code = rsp->ret_code;
            free(rsp);
            return ESP_OK;
        }
        case CMD_JOB_HIGHLIGHT:
            return command_logic_send_highlight_for_slot(slot);
        case CMD_JOB_KEY_QS:
            return command_logic_send_key_report_for_slot(slot, 0x02, 0x01, 0x00);
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static void post_result(const cmd_result_t *res) {
    if (xQueueSend(s_result_queue, res, 0) != pdTRUE) {
        /* Only a lost message: the 1D02 push stays the source of truth */
        if (atomic_fetch_add(&s_results_dropped, 1) == 0) {
            ESP_LOGW(TAG, "Result queue full, dropping result for slot %d", res->slot);
        }
    }
}

static void worker_task(void *arg) {
    int slot = (int)(intptr_t)arg;
    cmd_job_t job;

    for (;;) {
        if (xQueueReceive(s_job_queue[slot], &job, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int64_t t0 = esp_timer_get_time();
        cmd_result_t res = { .slot = slot, .job = job, .status = ESP_OK, .ret_code = -1 };

        ESP_LOGI(TAG, "Camera %d: job %s started", slot, command_worker_job_name(job));

        /* State may have changed since the press */
        if (!slot_gate_ok(slot)) {
            res.status = ESP_ERR_INVALID_STATE;
        } else {
            res.status = run_job(slot, job, &res.ret_code);
        }

        ESP_LOGI(TAG, "Camera %d: job %s finished: %s (ret_code=%d, %lld ms)",
                 slot, command_worker_job_name(job), esp_err_to_name(res.status),
                 res.ret_code, (long long)((esp_timer_get_time() - t0) / 1000));

        post_result(&res);
        atomic_store(&s_busy[slot], false);
    }
}

esp_err_t command_worker_init(void) {
    if (s_initialized) {
        return ESP_OK;
    }

    s_result_queue = xQueueCreate(CMD_RESULT_QUEUE_DEPTH, sizeof(cmd_result_t));
    if (s_result_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create result queue");
        return ESP_ERR_NO_MEM;
    }

    static const char *names[NUM_CAMERAS] = { "cmd_w0", "cmd_w1", "cmd_w2" };
    for (int i = 0; i < NUM_CAMERAS; i++) {
        atomic_store(&s_busy[i], false);
        s_job_queue[i] = xQueueCreate(1, sizeof(cmd_job_t));
        if (s_job_queue[i] == NULL) {
            ESP_LOGE(TAG, "Failed to create job queue for slot %d", i);
            return ESP_ERR_NO_MEM;
        }
        if (xTaskCreate(worker_task, names[i], CMD_WORKER_STACK, (void *)(intptr_t)i,
                        CMD_WORKER_PRIO, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create worker task for slot %d", i);
            return ESP_ERR_NO_MEM;
        }
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Command workers started (%d slots)", NUM_CAMERAS);
    return ESP_OK;
}

esp_err_t command_worker_submit(int slot, cmd_job_t job) {
    if (!s_initialized || !slot_valid(slot) || !slot_gate_ok(slot)) {
        return ESP_ERR_INVALID_STATE;
    }

    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_busy[slot], &expected, true)) {
        ESP_LOGI(TAG, "Camera %d: busy, job %s rejected", slot, command_worker_job_name(job));
        return ESP_ERR_NOT_FINISHED;
    }

    if (xQueueSend(s_job_queue[slot], &job, 0) != pdTRUE) {
        atomic_store(&s_busy[slot], false);
        ESP_LOGW(TAG, "Camera %d: job queue full although not busy", slot);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool command_worker_slot_busy(int slot) {
    return slot_valid(slot) && atomic_load(&s_busy[slot]);
}

bool command_worker_take_result(cmd_result_t *out) {
    if (s_result_queue == NULL || out == NULL) {
        return false;
    }
    return xQueueReceive(s_result_queue, out, 0) == pdTRUE;
}

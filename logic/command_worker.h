/*
 * Per-slot command workers (ADR-020)
 *
 * Every button-triggered camera command runs on a worker task that belongs
 * to one slot, so a camera that does not answer blocks only its own worker,
 * never the main loop. Each slot accepts one job at a time; results come
 * back to the main loop through a shared result queue.
 */
#ifndef __COMMAND_WORKER_H__
#define __COMMAND_WORKER_H__

#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    CMD_JOB_RECORD_START,        // single slot, waits for response (also photo)
    CMD_JOB_RECORD_STOP,         // single slot, waits for response
    CMD_JOB_RECORD_START_ASYNC,  // All Cameras, write without response
    CMD_JOB_RECORD_STOP_ASYNC,   // All Cameras, write without response
    CMD_JOB_SLEEP,               // waits for response
    CMD_JOB_HIGHLIGHT,           // key report QS (highlight-capable models)
    CMD_JOB_KEY_QS,              // Mode Switch screen QS key
} cmd_job_t;

typedef struct {
    int        slot;
    cmd_job_t  job;
    esp_err_t  status;    // ESP_OK, ESP_ERR_TIMEOUT, ESP_ERR_INVALID_STATE, ...
    int        ret_code;  // camera ret_code if a response frame was parsed, else -1
} cmd_result_t;

/** Creates the three worker tasks, their job queues and the result queue. */
esp_err_t command_worker_init(void);

/**
 * @brief Queue one job for a slot (main task)
 *
 * @return ESP_OK                queued
 *         ESP_ERR_INVALID_STATE gate failed (invalid slot, not connected,
 *                               unresponsive, workers not initialized)
 *         ESP_ERR_NOT_FINISHED  slot busy with a previous job ("Busy")
 *         ESP_ERR_NO_MEM        job queue unexpectedly full
 */
esp_err_t command_worker_submit(int slot, cmd_job_t job);

/** True while a job for the slot is queued or running. */
bool command_worker_slot_busy(int slot);

/** Non-blocking: fetch the next result, false if none. */
bool command_worker_take_result(cmd_result_t *out);

/** Short English name of a job for logs. */
const char *command_worker_job_name(cmd_job_t job);

#endif

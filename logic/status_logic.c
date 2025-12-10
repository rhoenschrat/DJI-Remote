/*
 * DJI Camera Remote Control - Status Logic Layer
 * 
 * This file handles camera status updates received from DJI cameras via BLE.
 * It processes status push messages (1D02 and 1D06) and updates camera state
 * structures for the UI system.
 * 
 * Key features:
 * - Status subscription management
 * - Camera state update callbacks
 * - Recording state detection
 * - Sleep mode tracking
 * - Multi-camera status synchronization
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "dji_protocol_data_structures.h"
#include "../main/ui.h"
#include "../main/m5stack_basic_v27_hal.h"  // For M5_COLOR constants

static const char *TAG = "LOGIC_STATUS";

// Global variables to store various camera status information
uint8_t current_camera_mode = 0;
uint8_t current_camera_status = 0;
uint8_t current_video_resolution = 0;
uint8_t current_fps_idx = 0;
uint8_t current_eis_mode = 0;
uint8_t current_user_mode = 0;
uint8_t current_camera_mode_next_flag = 0;
uint16_t current_record_time = 0;
uint16_t current_timelapse_interval = 0;
uint32_t current_remain_capacity = 0;
uint32_t current_remain_time = 0;
uint8_t current_camera_bat_percentage = 0;
uint8_t current_power_mode = 0;  // Sleep mode tracking: 0=normal, 3=sleep
bool camera_status_initialized = false;
uint32_t g_last_status_push_timestamp = 0;  // Timestamp of last camera status push (milliseconds)

// Global variables for new camera status push command frame
uint8_t current_type_mode_name = 0;
uint8_t current_mode_name_length = 0;
uint8_t current_mode_name[20] = {0};
uint8_t current_type_mode_param = 0;
uint8_t current_mode_param_length = 0;
uint8_t current_mode_param[20] = {0};


/**
 * @brief Check if camera is recording
 * 
 * Check if camera is in recording or pre-recording state, and status is initialized.
 * 
 * @return bool Returns true if camera is recording, false otherwise
 */
bool is_camera_recording() {
    if ((current_camera_status == CAMERA_STATUS_PHOTO_OR_RECORDING || current_camera_status == CAMERA_STATUS_PRE_RECORDING) && camera_status_initialized) {
        return true;
    }
    return false;
}

/**
 * @brief Print current camera status (partial status, other status can be printed as needed)
 * 
 * Print camera mode, status, resolution, frame rate and electronic image stabilization mode.
 */
void print_camera_status() {
    if (!camera_status_initialized) {
        ESP_LOGW(TAG, "Camera status has not been initialized.");
        return;
    }

    const char *mode_str = camera_mode_to_string((camera_mode_t)current_camera_mode);
    const char *status_str = camera_status_to_string((camera_status_t)current_camera_status);
    const char *resolution_str = video_resolution_to_string((video_resolution_t)current_video_resolution);
    const char *fps_str = fps_idx_to_string((fps_idx_t)current_fps_idx);
    const char *eis_str = eis_mode_to_string((eis_mode_t)current_eis_mode);

    ESP_LOGI(TAG, "[1D02] =========== Camera Status Push ===========");
    ESP_LOGI(TAG, "  Mode: %s", mode_str);
    ESP_LOGI(TAG, "  Status: %s", status_str);
    ESP_LOGI(TAG, "  Resolution: %s (value: %d)", resolution_str, current_video_resolution);
    ESP_LOGI(TAG, "  FPS: %s", fps_str);
    ESP_LOGI(TAG, "  EIS: %s", eis_str);
    ESP_LOGI(TAG, "  User mode: %d", current_user_mode);
    ESP_LOGI(TAG, "  Camera mode next flag: %d", current_camera_mode_next_flag);
    ESP_LOGI(TAG, "  Record time: %d", current_record_time);
    ESP_LOGI(TAG, "  Timelapse interval: %d", current_timelapse_interval);
    ESP_LOGI(TAG, "=================================================");
}

/**
 * @brief Subscribe to camera status
 * 
 * @param camera_index Camera index (0, 1, or 2)
 * @param push_mode Subscription mode
 * @param push_freq Subscription frequency
 * @return int Returns 0 on success, -1 on failure
 */
int subscript_camera_status(int camera_index, uint8_t push_mode, uint8_t push_freq) {
    ESP_LOGI(TAG, "Subscribing to Camera %d Status with push_mode: %d, push_freq: %d", camera_index, push_mode, push_freq);

    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        ESP_LOGE(TAG, "Protocol connection to the camera failed. Current connection state: %d", connect_logic_get_state());
        return -1;
    }

    uint16_t seq = generate_seq();

    camera_status_subscription_command_frame command_frame = {
        .push_mode = push_mode,
        .push_freq = push_freq,
        .reserved = {0, 0, 0, 0}
    };

    send_command(camera_index, 0x1D, 0x05, CMD_NO_RESPONSE, &command_frame, seq, 5000);

    return 0;
}

/**
 * @brief Update camera state machine (callback function)
 * 
 * Process and update various camera states, check for state changes and print updated information.
 * 
 * @param data Input camera status data
 */
void update_camera_state_handler(int camera_index, void *data) {
    if (!data) {
        ESP_LOGE(TAG, "Camera %d: logic_update_camera_state: Received NULL data.", camera_index);
        return;
    }

    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera_index %d in status update", camera_index);
        return;
    }

    const camera_status_push_command_frame *parsed_data = (const camera_status_push_command_frame *)data;

    // Get reference to this camera's state
    camera_state_t *cam_state = &g_camera_states[camera_index];

    // Update timestamp of last status push for wake-and-record confirmation
    g_last_status_push_timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    cam_state->last_status_timestamp = g_last_status_push_timestamp;

    bool state_changed = false;

    // Check and update camera mode
    if (cam_state->camera_mode != parsed_data->camera_mode) {
        cam_state->camera_mode = parsed_data->camera_mode;
        ESP_LOGI(TAG, "Camera %d: Camera mode updated to: %d", camera_index, cam_state->camera_mode);
        state_changed = true;
        
        // Update old global for camera 0
        if (camera_index == 0) {
            current_camera_mode = parsed_data->camera_mode;
        }
    }

    // Check and update camera status
    // IMPORTANT: Recording state is determined by camera_status field from status push (0x1D/0x02)
    // Status 0x03 = Recording/Photo in progress, Status 0x05 = Pre-recording
    // This is the authoritative source - never assume state from command responses
    if (cam_state->camera_status != parsed_data->camera_status) {
        uint8_t old_status = cam_state->camera_status;
        cam_state->camera_status = parsed_data->camera_status;
        
        const char *old_status_str = camera_status_to_string((camera_status_t)old_status);
        const char *new_status_str = camera_status_to_string((camera_status_t)cam_state->camera_status);
        
        ESP_LOGI(TAG, "Camera %d status changed: %d (%s) -> %d (%s)", 
                 camera_index, old_status, old_status_str, cam_state->camera_status, new_status_str);
        
        // Log recording state change specifically
        bool was_recording = (old_status == CAMERA_STATUS_PHOTO_OR_RECORDING || old_status == CAMERA_STATUS_PRE_RECORDING);
        bool is_now_recording = (cam_state->camera_status == CAMERA_STATUS_PHOTO_OR_RECORDING || cam_state->camera_status == CAMERA_STATUS_PRE_RECORDING);
        if (was_recording != is_now_recording) {
            ESP_LOGI(TAG, "Camera %d recording state changed: %s -> %s", 
                     camera_index,
                     was_recording ? "RECORDING" : "NOT RECORDING",
                     is_now_recording ? "RECORDING" : "NOT RECORDING");
        }
        
        // Update is_recording field
        cam_state->is_recording = is_now_recording;
        
        state_changed = true;
        
        // Update old global for camera 0
        if (camera_index == 0) {
            current_camera_status = parsed_data->camera_status;
        }
    }

    // Check and update video resolution
    if (cam_state->video_resolution != parsed_data->video_resolution) {
        cam_state->video_resolution = parsed_data->video_resolution;
        ESP_LOGI(TAG, "Camera %d: Video resolution updated to: %d", camera_index, cam_state->video_resolution);
        state_changed = true;
        if (camera_index == 0) current_video_resolution = parsed_data->video_resolution;
    }

    // Check and update frame rate
    if (cam_state->fps_idx != parsed_data->fps_idx) {
        cam_state->fps_idx = parsed_data->fps_idx;
        ESP_LOGI(TAG, "Camera %d: FPS index updated to: %d", camera_index, cam_state->fps_idx);
        state_changed = true;
        if (camera_index == 0) current_fps_idx = parsed_data->fps_idx;
    }

    // Check and update electronic image stabilization mode
    if (cam_state->eis_mode != parsed_data->eis_mode) {
        cam_state->eis_mode = parsed_data->eis_mode;
        ESP_LOGI(TAG, "Camera %d: EIS mode updated to: %d", camera_index, cam_state->eis_mode);
        state_changed = true;
        if (camera_index == 0) current_eis_mode = parsed_data->eis_mode;
    }

    // Check and update user mode  
    if (cam_state->user_mode != parsed_data->user_mode) {
        cam_state->user_mode = parsed_data->user_mode;
        ESP_LOGI(TAG, "Camera %d: User mode updated to: %d", camera_index, cam_state->user_mode);
        state_changed = true;
        if (camera_index == 0) current_user_mode = parsed_data->user_mode;
    }

    // Check and update camera mode next flag
    if (cam_state->camera_mode_next_flag != parsed_data->camera_mode_next_flag) {
        cam_state->camera_mode_next_flag = parsed_data->camera_mode_next_flag;
        ESP_LOGI(TAG, "Camera %d: Camera mode next flag updated to: %d", camera_index, cam_state->camera_mode_next_flag);
        state_changed = true;
        if (camera_index == 0) current_camera_mode_next_flag = parsed_data->camera_mode_next_flag;
    }

    // Check and update record time
    if (cam_state->record_time != parsed_data->record_time) {
        cam_state->record_time = parsed_data->record_time;
        ESP_LOGI(TAG, "Camera %d: Record time updated to: %d", camera_index, cam_state->record_time);
        state_changed = true;
        if (camera_index == 0) current_record_time = parsed_data->record_time;
    }

    // Check and update timelapse interval
    if (cam_state->timelapse_interval != parsed_data->timelapse_interval) {
        cam_state->timelapse_interval = parsed_data->timelapse_interval;
        ESP_LOGI(TAG, "Camera %d: Timelapse interval updated to: %d", camera_index, cam_state->timelapse_interval);
        state_changed = true;
        if (camera_index == 0) current_timelapse_interval = parsed_data->timelapse_interval;
    }

    // Check and update remain capacity
    if (cam_state->remain_capacity != parsed_data->remain_capacity) {
        cam_state->remain_capacity = parsed_data->remain_capacity;
        ESP_LOGI(TAG, "Camera %d: Remain capacity updated to: %lu MB", camera_index, (unsigned long)cam_state->remain_capacity);
        state_changed = true;
        if (camera_index == 0) current_remain_capacity = parsed_data->remain_capacity;
    }

    // Check and update remain time
    if (cam_state->remain_time != parsed_data->remain_time) {
        cam_state->remain_time = parsed_data->remain_time;
        ESP_LOGI(TAG, "Camera %d: Remain time updated to: %lu seconds", camera_index, (unsigned long)cam_state->remain_time);
        state_changed = true;
        if (camera_index == 0) current_remain_time = parsed_data->remain_time;
    }

    // Check and update camera battery percentage
    if (cam_state->camera_bat_percentage != parsed_data->camera_bat_percentage) {
        cam_state->camera_bat_percentage = parsed_data->camera_bat_percentage;
        cam_state->battery_percentage = parsed_data->camera_bat_percentage; // Also update alias
        ESP_LOGI(TAG, "Camera %d: Camera battery updated to: %d%%", camera_index, cam_state->camera_bat_percentage);
        state_changed = true;
        if (camera_index == 0) current_camera_bat_percentage = parsed_data->camera_bat_percentage;
    }

    // Check and update power mode
    // According to DJI DATA Segment protocol (Camera Power Mode Settings, feature ID 001A):
    // - power_mode = 0: Normal working mode
    // - power_mode = 3: Sleep mode
    // When camera is in sleep mode, remote must stop sending commands (except wake commands)
    // Reference: Osmo-GPS-Controller-Demo, protocol_data_segment.md
    bool was_sleeping = (cam_state->power_mode == 3);
    if (cam_state->power_mode != parsed_data->power_mode) {
        cam_state->power_mode = parsed_data->power_mode;
        bool prev_is_sleeping = cam_state->is_sleeping;
        cam_state->is_sleeping = (cam_state->power_mode == 3);
        
        // Track sleep state transitions
        if (prev_is_sleeping != cam_state->is_sleeping) {
            cam_state->last_sleep_state_change_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (cam_state->is_sleeping) {
                ESP_LOGI(TAG, "Camera %d: Entered SLEEP mode (power_mode=%d)", camera_index, cam_state->power_mode);
            } else {
                ESP_LOGI(TAG, "Camera %d: Woke up from SLEEP mode (power_mode=%d)", camera_index, cam_state->power_mode);
            }
        }
        
        ESP_LOGI(TAG, "Camera %d: Power mode updated to: %d (%s)", camera_index, cam_state->power_mode, 
                 cam_state->is_sleeping ? "SLEEP" : "NORMAL");
        state_changed = true;
        if (camera_index == 0) current_power_mode = parsed_data->power_mode;
    }
    
    // Check for wake-up transition (power_mode changed from 3 to 0)
    bool is_now_sleeping = cam_state->is_sleeping;
    if (was_sleeping && !is_now_sleeping) {
        ESP_LOGI(TAG, "LOGIC_STATUS: Camera %d: Woke up from SLEEP (power_mode changed 3 → 0).", camera_index);
        
        // Check if snapshot is pending for this camera
        if (cam_state->snapshot_pending) {
            // ALWAYS send snapshot key from status handler - this is the single point of responsibility
            // The queue processor only manages broadcast timing and early termination
            ESP_LOGI(TAG, "LOGIC_STATUS: Camera %d: Snapshot pending, sending SNAPSHOT key after wake-up", camera_index);
            
            // Send snapshot key command
            extern esp_err_t command_logic_send_snapshot_key_for_slot(int camera_index);
            esp_err_t snapshot_result = command_logic_send_snapshot_key_for_slot(camera_index);
            
            if (snapshot_result == ESP_OK) {
                ESP_LOGI(TAG, "LOGIC_STATUS: Camera %d: Snapshot key (0x03) sent successfully after wake-up.", camera_index);
            } else {
                ESP_LOGE(TAG, "LOGIC_STATUS: Camera %d: Failed to send Snapshot key after wake-up.", camera_index);
            }
            
            // Always clear snapshot pending flag after attempting to send
            cam_state->snapshot_pending = false;
            ESP_LOGD(TAG, "LOGIC_STATUS: Camera %d: snapshot_pending cleared (was handled by status handler)", camera_index);
            
            // Notify queue processor if this camera was being processed in "All Cameras" mode
            // This allows early termination of the broadcast and moving to next camera
            extern int g_current_wake_camera_index;
            if (g_current_wake_camera_index == camera_index) {
                extern void wake_queue_notify_camera_woke_up(int camera_index);
                wake_queue_notify_camera_woke_up(camera_index);
                ESP_LOGI(TAG, "LOGIC_STATUS: Camera %d: Notified wake queue of early wake-up", camera_index);
            }
        }
    }

    // Mark camera state as initialized
    bool was_just_initialized = false;
    if (!cam_state->is_initialized) {
        cam_state->is_initialized = true;
        ESP_LOGI(TAG, "Camera %d state fully updated and marked as initialized.", camera_index);
        state_changed = true;  // Force status print as this is initialization
        was_just_initialized = true;
        
        // Also mark global flag for camera 0
        if (camera_index == 0 && !camera_status_initialized) {
            camera_status_initialized = true;
        }
    }

    // If state changed or first initialization, request UI update
    if (state_changed) {
        // Only print for camera 0 to avoid log spam
        if (camera_index == 0) {
            print_camera_status();
        } else {
            ESP_LOGI(TAG, "Camera %d status updated", camera_index);
        }
        
        // Request UI update (always on Main screen now)
        // The UI's selective update logic will handle what actually needs redrawing
        g_ui_state.display_needs_update = true;
        
        // Only force full redraw on first initialization
        // For subsequent updates, the UI's selective update logic handles it
        if (was_just_initialized) {
            cam_state->needs_full_redraw = true;
        }
    }

    free(data);
}

void update_new_camera_state_handler(int camera_index, void *data) {
    if (!data) {
        ESP_LOGE(TAG, "Camera %d: update_new_camera_state_handler: Received NULL data.", camera_index);
        return;
    }

    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera_index %d in new status update", camera_index);
        free(data);
        return;
    }

    const new_camera_status_push_command_frame *parsed_data = (const new_camera_status_push_command_frame *)data;

    // Get reference to this camera's state
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    camera_state_t *cam_state = &g_camera_states[camera_index];

    ESP_LOGI(TAG, "Camera %d [1D06] =============New Status Push============", camera_index);

    // Store mode_name (ensure null termination)
    memset(cam_state->mode_name, 0, sizeof(cam_state->mode_name));
    size_t mode_name_len = (parsed_data->mode_name_length < 20) ? parsed_data->mode_name_length : 20;
    memcpy(cam_state->mode_name, parsed_data->mode_name, mode_name_len);
    cam_state->mode_name[20] = '\0';  // Ensure null termination
    ESP_LOGI(TAG, "[1D06] Mode name: %s (length: %d)", cam_state->mode_name, parsed_data->mode_name_length);
    
    // Store mode_param (ensure null termination)
    memset(cam_state->mode_param, 0, sizeof(cam_state->mode_param));
    size_t mode_param_len = (parsed_data->mode_param_length < 20) ? parsed_data->mode_param_length : 20;
    memcpy(cam_state->mode_param, parsed_data->mode_param, mode_param_len);
    cam_state->mode_param[20] = '\0';  // Ensure null termination
    ESP_LOGI(TAG, "[1D06] Mode parameters: '%s' (length: %d)", cam_state->mode_param, parsed_data->mode_param_length);

    // Mark this camera as supporting new status push (1D06)
    // Once set to true, this flag stays true for the entire pairing session.
    // Video Mode Area will now be updated ONLY from 1D06, not from 1D02.
    cam_state->camera_supports_new_status_push = true;
    ESP_LOGI(TAG, "[1D06] Camera %d now uses new status push (1D06)", camera_index);

    // Trigger UI update
    extern ui_state_t g_ui_state;
    g_ui_state.display_needs_update = true;

    ESP_LOGI(TAG, "[1D06] ================================");

    free(data);
}
/*
 * DJI Camera Remote Control - Command Logic Layer
 * 
 * This file implements all camera command operations for the DJI remote control system.
 * It provides a unified interface for sending commands to DJI cameras and handling
 * their responses through the established BLE connection.
 * 
 * Supported Command Categories:
 * - Recording Control: Start/stop photo and video recording
 * - Mode Management: Switch between camera modes (photo, video, timelapse, etc.)
 * - Power Management: Sleep and wake camera operations
 * - Status Subscription: Request real-time camera status updates
 * 
 * Command Flow:
 * 1. Command Construction: Build DJI protocol frames with proper headers
 * 2. Sequence Management: Generate unique sequence numbers for tracking
 * 3. Transmission: Send commands via BLE to camera
 * 4. Response Handling: Wait for and parse camera responses
 * 5. Error Management: Handle timeouts and error responses
 * 
 * Protocol Details:
 * - Command Set 0x00: General camera operations
 * - Command Set 0x01: Recording and mode control
 * - Command Set 0x05: Status and telemetry
 * 
 * All commands use the DJI proprietary protocol format with CRC validation
 * and sequence numbering for reliable communication.
 * 
 * Hardware: M5Stack Basic V2.7 with ESP32 BLE capabilities
 * Protocol: DJI proprietary communication protocol
 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ble.h"
#include "data.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "dji_protocol_parser.h"
#include "dji_protocol_data_structures.h"
#include "../gps/gps_reader.h"
#include "../main/ui.h"

/* Logging tag for ESP_LOG functions */
#define TAG "LOGIC_COMMAND"

/* Global sequence number generator for command tracking
 * Each command gets a unique sequence number to match with responses
 */
uint16_t s_current_seq = 0;

/**
 * @brief Generate unique sequence number for command tracking
 * 
 * Creates incremental sequence numbers used to match commands with their
 * responses in the DJI protocol. Each command must have a unique sequence
 * number to prevent confusion with concurrent or delayed responses.
 * 
 * @return Unique 16-bit sequence number
 */
uint16_t generate_seq(void) {
    return s_current_seq += 1;
}

/**
 * @brief Check if a camera can accept commands (not sleeping, or wake command exception)
 * 
 * When a camera is in sleep mode (power_mode == 3), all commands must be blocked
 * except wake commands (CmdSet=0x00, CmdID=0x1A with power_mode=0x00).
 * This prevents sending commands to sleeping cameras which would fail.
 * 
 * @param camera_index Camera slot index (0-2)
 * @param cmd_set Command set to check if this is a wake command
 * @param cmd_id Command ID to check if this is a wake command
 * @param input_raw_data Command data to check if this is a wake command (may be NULL)
 * @return true if camera can accept commands, false if sleeping (and not a wake command)
 */
static bool camera_can_accept_commands(int camera_index, uint8_t cmd_set, uint8_t cmd_id, const void *input_raw_data) {
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    
    // Check if camera is sleeping
    if (g_camera_states[camera_index].is_sleeping) {
        // Exception: Wake commands (Camera Power Mode Settings, feature ID 001A)
        // CmdSet=0x00, CmdID=0x1A with power_mode=0x00 is allowed even when sleeping
        if (cmd_set == 0x00 && cmd_id == 0x1A && input_raw_data != NULL) {
            const camera_power_mode_switch_command_frame_t *power_frame = 
                (const camera_power_mode_switch_command_frame_t *)input_raw_data;
            if (power_frame->power_mode == 0x00) {
                // This is a wake command - allow it
                ESP_LOGD(TAG, "Camera %d: Allowing wake command (0x00/0x1A) even though sleeping", camera_index);
                return true;
            }
        }
        // Camera is sleeping and this is not a wake command - block it
        ESP_LOGD(TAG, "Camera %d: Blocking command (0x%02X/0x%02X) - camera is sleeping", 
                 camera_index, cmd_set, cmd_id);
        return false;
    }
    
    // Camera is awake - allow all commands
    return true;
}

/**
 * @brief General function for constructing data frames and sending commands
 *
 * @param camera_index Camera slot index (0-2)
 * @param cmd_set Command set, used to specify command category
 * @param cmd_id Command ID, used to identify specific command
 * @param cmd_type Command type, indicates features like response requirement
 * @param input_raw_data Data structure pointer, contains input data for command frame
 * @param seq Sequence number, used to match request and response
 * @param timeout_ms Timeout for waiting result (in milliseconds)
 * 
 * Note: The caller needs to free the dynamically allocated memory after using the returned structure.
 * 
 * @return CommandResult Returns parsed structure pointer and data length on success, NULL pointer and length 0 on failure
 */
CommandResult send_command(int camera_index, uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const void *input_raw_data, uint16_t seq, int timeout_ms) { 
    CommandResult result = { NULL, 0 };
    
    // Validate camera index
    if (camera_index < 0 || camera_index >= 3) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return result;
    }

    // Check per-camera BLE connection state (not global state)
    // This ensures commands work for connected cameras even if other cameras are disconnected
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: BLE not connected", camera_index);
        return result;
    }

    // Check if camera can accept commands (sleep mode blocking)
    // All commands to sleeping cameras are blocked except wake commands (0x00/0x1A with power_mode=0x00)
    if (!camera_can_accept_commands(camera_index, cmd_set, cmd_id, input_raw_data)) {
        ESP_LOGD(TAG, "Camera %d: Command blocked - camera is in sleep mode", camera_index);
        return result;
    }

    esp_err_t ret;

    // Create protocol frame
    // 创建协议帧
    size_t frame_length = 0;
    uint8_t *protocol_frame = protocol_create_frame(cmd_set, cmd_id, cmd_type, input_raw_data, seq, &frame_length);
    if (protocol_frame == NULL) {
        ESP_LOGE(TAG, "Failed to create protocol frame");
        return result;
    }

    ESP_LOGI(TAG, "Protocol frame created successfully, length: %zu", frame_length);

    // Print ByteArray format for debugging using ESP_LOGD
    // Build hex dump string (max 512 bytes for hex representation: 256 bytes * 2 + commas)
    if (frame_length > 0 && frame_length <= 256) {
        char hex_buffer[512];
        size_t offset = 0;
        offset += snprintf(hex_buffer + offset, sizeof(hex_buffer) - offset, "TX: [");
        for (size_t i = 0; i < frame_length && offset < sizeof(hex_buffer) - 10; i++) {
            offset += snprintf(hex_buffer + offset, sizeof(hex_buffer) - offset, "%02X", protocol_frame[i]);
            if (i < frame_length - 1) {
                offset += snprintf(hex_buffer + offset, sizeof(hex_buffer) - offset, ", ");
            }
        }
        snprintf(hex_buffer + offset, sizeof(hex_buffer) - offset, "]");
        ESP_LOGD(TAG, "%s", hex_buffer);
    } else if (frame_length > 256) {
        ESP_LOGD(TAG, "TX: [Frame too large to display: %zu bytes]", frame_length);
    }

    void *structure_data = NULL;
    size_t structure_data_length = 0;

    switch (cmd_type) {
        case CMD_NO_RESPONSE:
        case ACK_NO_RESPONSE:
            ret = data_write_without_response(camera_index, seq, protocol_frame, frame_length);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Camera %d: Failed to send data frame (no response), error: %s", camera_index, esp_err_to_name(ret));
                free(protocol_frame);
                return result;
            }
            ESP_LOGI(TAG, "Camera %d: Data frame sent without response.", camera_index);
            break;

        case CMD_RESPONSE_OR_NOT:
        case ACK_RESPONSE_OR_NOT:
            ret = data_write_with_response(camera_index, seq, protocol_frame, frame_length);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Camera %d: Failed to send data frame (with response), error: %s", camera_index, esp_err_to_name(ret));
                free(protocol_frame);
                return result;
            }
            ESP_LOGI(TAG, "Camera %d: Data frame sent, waiting for response...", camera_index);
            
            ret = data_wait_for_result_by_seq(seq, timeout_ms, &structure_data, &structure_data_length);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Camera %d: No result received, but continuing (seq=0x%04X)", camera_index, seq);
            }

            break;

        case CMD_WAIT_RESULT:
        case ACK_WAIT_RESULT:
            ret = data_write_with_response(camera_index, seq, protocol_frame, frame_length);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Camera %d: Failed to send data frame (wait result), error: %s", camera_index, esp_err_to_name(ret));
                free(protocol_frame);
                return result;
            }
            ESP_LOGI(TAG, "Camera %d: Data frame sent, waiting for result...", camera_index);

            ret = data_wait_for_result_by_seq(seq, timeout_ms, &structure_data, &structure_data_length);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Camera %d: Failed to get parse result for seq=0x%04X, error: 0x%x", camera_index, seq, ret);
                free(protocol_frame);
                return result;
            }

            if (structure_data == NULL) {
                ESP_LOGE(TAG, "Camera %d: Parse result is NULL for seq=0x%04X", camera_index, seq);
                free(protocol_frame);
                return result;
            }

            break;

        default:
            ESP_LOGE(TAG, "Camera %d: Invalid cmd_type: %d", camera_index, cmd_type);
            free(protocol_frame);
            return result;
    }

    free(protocol_frame);
    ESP_LOGI(TAG, "Command executed successfully");

    result.structure = structure_data;
    result.length = structure_data_length;

    return result;
}

/**
 * @brief Switch camera mode
 *        切换相机模式
 *
 * @param mode Camera mode
 *             相机模式
 * 
 * @return camera_mode_switch_response_frame_t* Returns parsed structure pointer, NULL on error
 *                                              返回解析后的结构体指针，如果发生错误返回 NULL
 */
camera_mode_switch_response_frame_t* command_logic_switch_camera_mode(int camera_index, camera_mode_t mode) {
    ESP_LOGI(TAG, "%s: Camera %d: Switching camera mode to: %d", __FUNCTION__, camera_index, mode);
    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: Not connected", camera_index);
        return NULL;
    }

    uint16_t seq = generate_seq();

    camera_mode_switch_command_frame_t command_frame = {
        .device_id = g_device_id,
        .mode = mode,
        .reserved = {0x01, 0x47, 0x39, 0x36}  // Reserved field
                                              // 预留字段
    };

    ESP_LOGI(TAG, "Constructed command frame: device_id=0x%08X, mode=%d", (unsigned int)command_frame.device_id, command_frame.mode);

    CommandResult result = send_command(
        camera_index,
        0x1D,
        0x04,
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    camera_mode_switch_response_frame_t *response = (camera_mode_switch_response_frame_t *)result.structure;

    ESP_LOGI(TAG, "Received response: ret_code=%d", response->ret_code);
    return response;
}

/**
 * @brief Query device version
 *        查询设备版本号
 *
 * This function sends a query command to get device version information.
 * 该函数通过发送查询命令，获取设备的版本号信息。
 * 
 * The returned version information includes acknowledgment result (`ack_result`),
 * product ID (`product_id`) and SDK version (`sdk_version`).
 * 返回的版本号信息包括应答结果 (`ack_result`)、产品 ID (`product_id`) 和 SDK 版本号 (`sdk_version`)。
 *
 * @return version_query_response_frame_t* Returns parsed version info structure, NULL on error
 *                                         返回解析后的版本信息结构体，如果发生错误返回 NULL
 */
version_query_response_frame_t* command_logic_get_version(int camera_index) {
    ESP_LOGI(TAG, "%s: Camera %d: Querying device version", __FUNCTION__, camera_index);
    
    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: Not connected", camera_index);
        return NULL;
    }

    uint16_t seq = generate_seq();

    CommandResult result = send_command(
        camera_index,
        0x00,
        0x00,
        CMD_WAIT_RESULT,
        NULL,
        seq,
        5000
    );

    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    version_query_response_frame_t *response = (version_query_response_frame_t *)result.structure;

    ESP_LOGI(TAG, "Version Query Response: ack_result=%u, product_id=%s, sdk_version=%.*s",
             response->ack_result, response->product_id, 
             (int)(result.length - (sizeof(uint16_t) + sizeof(response->product_id))),
             response->sdk_version);

    return response;
}

/**
 * @brief Start recording (non-blocking)
 * 
 * Sends start recording command to camera without waiting for response.
 * This is used for synchronized multi-camera recording in "All cameras" mode.
 * 
 * @param camera_index Camera index (0, 1, or 2)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t command_logic_start_record_async(int camera_index) {
    ESP_LOGI(TAG, "%s: Camera %d: Sending start recording command (0x1D/0x03) - non-blocking", __FUNCTION__, camera_index);

    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: Not connected", camera_index);
        return ESP_FAIL;
    }

    uint16_t seq = generate_seq();

    record_control_command_frame_t command_frame = {
        .device_id = g_device_id,
        .record_ctrl = 0x00,  // 0x00 = Start recording
        .reserved = {0x00, 0x00, 0x00, 0x00}
    };

    ESP_LOGI(TAG, "Sending start record command (async): device_id=0x%08X, record_ctrl=0x%02X", 
             command_frame.device_id, command_frame.record_ctrl);

    send_command(
        camera_index,
        0x1D,                    // Command set 0x1D for recording control
        0x03,                    // Command ID 0x03 for recording control
        CMD_NO_RESPONSE,         // Non-blocking: don't wait for response
        &command_frame,
        seq,
        0                        // No timeout for non-blocking
    );

    // For CMD_NO_RESPONSE, result is always {NULL, 0}
    // Success is indicated by send_command not logging errors
    return ESP_OK;
}

/**
 * @brief Start recording (blocking)
 * 
 * Sends start recording command to camera (CmdSet=0x1D, CmdID=0x03).
 * Waits for camera response before returning.
 * 
 * IMPORTANT: This function does NOT update internal recording state.
 * The actual recording state is determined by the camera_status field
 * in the status push messages (CmdSet=0x1D, CmdID=0x02) received from the camera.
 * Recording state should be checked via is_camera_recording() which reads
 * current_camera_status from the latest status push.
 * 
 * @return record_control_response_frame_t* Returns parsed response structure pointer, NULL on error
 */
record_control_response_frame_t* command_logic_start_record(int camera_index) {
    ESP_LOGI(TAG, "%s: Camera %d: Sending start recording command (0x1D/0x03)", __FUNCTION__, camera_index);

    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: Not connected", camera_index);
        return NULL;
    }

    uint16_t seq = generate_seq();

    record_control_command_frame_t command_frame = {
        .device_id = g_device_id,
        .record_ctrl = 0x00,  // 0x00 = Start recording
        .reserved = {0x00, 0x00, 0x00, 0x00}
    };

    ESP_LOGI(TAG, "Sending start record command: device_id=0x%08X, record_ctrl=0x%02X", 
             command_frame.device_id, command_frame.record_ctrl);

    CommandResult result = send_command(
        camera_index,
        0x1D,                    // Command set 0x1D for recording control
        0x03,                    // Command ID 0x03 for recording control
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send start record command or receive response");
        return NULL;
    }

    record_control_response_frame_t *response = (record_control_response_frame_t *)result.structure;

    ESP_LOGI(TAG, "Start Record Response: ret_code=%d (waiting for status push to confirm actual state)", response->ret_code);

    return response;
}

/**
 * @brief Stop recording (non-blocking)
 * 
 * Sends stop recording command to camera without waiting for response.
 * This is used for synchronized multi-camera recording in "All cameras" mode.
 * 
 * @param camera_index Camera index (0, 1, or 2)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t command_logic_stop_record_async(int camera_index) {
    ESP_LOGI(TAG, "%s: Camera %d: Sending stop recording command (0x1D/0x03) - non-blocking", __FUNCTION__, camera_index);

    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: Not connected", camera_index);
        return ESP_FAIL;
    }

    uint16_t seq = generate_seq();

    record_control_command_frame_t command_frame = {
        .device_id = g_device_id,
        .record_ctrl = 0x01,  // 0x01 = Stop recording
        .reserved = {0x00, 0x00, 0x00, 0x00}
    };

    ESP_LOGI(TAG, "Sending stop record command (async): device_id=0x%08X, record_ctrl=0x%02X", 
             command_frame.device_id, command_frame.record_ctrl);

    send_command(
        camera_index,
        0x1D,                    // Command set 0x1D for recording control
        0x03,                    // Command ID 0x03 for recording control
        CMD_NO_RESPONSE,         // Non-blocking: don't wait for response
        &command_frame,
        seq,
        0                        // No timeout for non-blocking
    );

    // For CMD_NO_RESPONSE, result is always {NULL, 0}
    // Success is indicated by send_command not logging errors
    return ESP_OK;
}

/**
 * @brief Stop recording (blocking)
 * 
 * Sends stop recording command to camera (CmdSet=0x1D, CmdID=0x03).
 * Waits for camera response before returning.
 * 
 * IMPORTANT: This function does NOT update internal recording state.
 * The actual recording state is determined by the camera_status field
 * in the status push messages (CmdSet=0x1D, CmdID=0x02) received from the camera.
 * Recording state should be checked via is_camera_recording() which reads
 * current_camera_status from the latest status push.
 * 
 * @return record_control_response_frame_t* Returns parsed response structure pointer, NULL on error
 */
record_control_response_frame_t* command_logic_stop_record(int camera_index) {
    ESP_LOGI(TAG, "%s: Camera %d: Sending stop recording command (0x1D/0x03)", __FUNCTION__, camera_index);

    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: Not connected", camera_index);
        return NULL;
    }

    uint16_t seq = generate_seq();

    record_control_command_frame_t command_frame = {
        .device_id = g_device_id,
        .record_ctrl = 0x01,  // 0x01 = Stop recording
        .reserved = {0x00, 0x00, 0x00, 0x00}
    };

    ESP_LOGI(TAG, "Sending stop record command: device_id=0x%08X, record_ctrl=0x%02X", 
             command_frame.device_id, command_frame.record_ctrl);

    CommandResult result = send_command(
        camera_index,
        0x1D,                    // Command set 0x1D for recording control
        0x03,                    // Command ID 0x03 for recording control
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send stop record command or receive response");
        return NULL;
    }

    record_control_response_frame_t *response = (record_control_response_frame_t *)result.structure;

    ESP_LOGI(TAG, "Stop Record Response: ret_code=%d (waiting for status push to confirm actual state)", response->ret_code);

    return response;
}


/**
 * @brief Quick switch mode key report
 *        快速切换模式按键上报
 *
 * @return key_report_response_frame_t* Returns parsed response structure pointer, NULL on error
 *                                      返回解析后的应答结构体指针，如果发生错误返回 NULL
 */
key_report_response_frame_t* command_logic_key_report_qs(int camera_index) {
    ESP_LOGI(TAG, "%s: Camera %d: Reporting key press for mode switch", __FUNCTION__, camera_index);

    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: Not connected", camera_index);
        return NULL;
    }

    uint16_t seq = generate_seq();

    key_report_command_frame_t command_frame = {
        .key_code = 0x02,          // QS key code for mode switch
                                   // QS按键码，模式切换
        .mode = 0x01,              // Fixed as 0x01
                                   // 固定为 0x01
        .key_value = 0x00,         // Fixed as 0x00, short press event
                                   // 固定为 0x00，短按事件
    };

    CommandResult result = send_command(
        camera_index,
        0x00,
        0x11,
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    key_report_response_frame_t *response = (key_report_response_frame_t *)result.structure;

    ESP_LOGI(TAG, "Key Report Response: ret_code=%d", response->ret_code);

    return response;
}

/**
 * @brief Put camera to sleep mode
 *        将相机设置为睡眠模式
 *
 * @return camera_power_mode_switch_response_frame_t* Returns parsed response structure pointer, NULL on error
 *                                                    返回解析后的应答结构体指针，如果发生错误返回 NULL
 */
camera_power_mode_switch_response_frame_t* command_logic_power_mode_switch_sleep(int camera_index) {
    // Log message indicating power mode switch to sleep
    // 睡眠模式切换日志
    ESP_LOGI(TAG, "%s: Reporting power mode switch to sleep", __FUNCTION__);

    // Check if the protocol is connected
    // 检查协议是否已连接
    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: Not connected", camera_index);
        return NULL;
    }

    uint16_t seq = generate_seq();  // Generate sequence number
                                    // 生成序列号

    // Create command frame with power mode set to sleep mode (0x03)
    // 创建命令帧，将电源模式设置为睡眠模式 (0x03)
    camera_power_mode_switch_command_frame_t command_frame = {
        .power_mode = 0x03,        // Set to 0x03 for sleep mode
                                   // 设置为 0x03 表示睡眠模式
    };

    // Send the command and receive the response
    // 发送命令并接收响应
    CommandResult result = send_command(
        camera_index,
        0x00,
        0x1A,                 // Use CmdSet = 0x00 and CmdID = 0x1A for power mode switch
                              // 使用 CmdSet = 0x00 和 CmdID = 0x1A 进行电源模式切换
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    // If no response structure is returned, the send or receive failed
    // 如果未返回响应结构体，则表示发送或接收失败
    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    // Convert the returned data structure to the corresponding response frame
    // 将返回的数据结构转换为相应的响应帧
    camera_power_mode_switch_response_frame_t *response = (camera_power_mode_switch_response_frame_t *)result.structure;

    // Log the response information for power mode switch
    // 记录电源模式切换的响应信息
    ESP_LOGI(TAG, "Power Mode Switch Response: ret_code=%d", response->ret_code);

    return response;  // Return the parsed response frame
                      // 返回解析后的响应帧
}

/**
 * @brief Send a power mode switch command to wake the camera from sleep mode
 * @brief 发送电源模式切换命令以唤醒睡眠状态的相机
 * 
 * This function sends a command to switch the camera's power mode to normal (awake) mode.
 * The power mode is set to 0x00 (normal/awake mode).
 * 该函数发送命令将相机的电源模式切换为正常（唤醒）模式。
 * 电源模式设置为 0x00（正常/唤醒模式）。
 * 
 * @return Pointer to the power mode switch response frame, or NULL if the operation failed
 * @return 指向电源模式切换响应帧的指针，如果操作失败则返回 NULL
 */
camera_power_mode_switch_response_frame_t* command_logic_power_mode_switch_wake(int camera_index) {
    // Log message indicating power mode switch to wake
    // 唤醒模式切换日志
    ESP_LOGI(TAG, "%s: Reporting power mode switch to wake", __FUNCTION__);

    // Check if the protocol is connected
    // 检查协议是否已连接
    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: Not connected", camera_index);
        return NULL;
    }

    uint16_t seq = generate_seq();  // Generate sequence number
                                    // 生成序列号

    // Create command frame with power mode set to normal mode (0x00)
    // 创建命令帧，将电源模式设置为正常模式 (0x00)
    camera_power_mode_switch_command_frame_t command_frame = {
        .power_mode = 0x00,        // Set to 0x00 for normal/awake mode
                                   // 设置为 0x00 表示正常/唤醒模式
    };

    // Send the command and receive the response
    // 发送命令并接收响应
    CommandResult result = send_command(
        camera_index,
        0x00,
        0x1A,                 // Use CmdSet = 0x00 and CmdID = 0x1A for power mode switch
                              // 使用 CmdSet = 0x00 和 CmdID = 0x1A 进行电源模式切换
        CMD_RESPONSE_OR_NOT,
        &command_frame,
        seq,
        5000
    );

    // If no response structure is returned, the send or receive failed
    // 如果未返回响应结构体，则表示发送或接收失败
    if (result.structure == NULL) {
        ESP_LOGE(TAG, "Failed to send command or receive response");
        return NULL;
    }

    // Convert the returned data structure to the corresponding response frame
    // 将返回的数据结构转换为相应的响应帧
    camera_power_mode_switch_response_frame_t *response = (camera_power_mode_switch_response_frame_t *)result.structure;

    // Log the response information for power mode switch
    // 记录电源模式切换的响应信息
    ESP_LOGI(TAG, "Power Mode Switch Response: ret_code=%d", response->ret_code);

    return response;  // Return the parsed response frame
                      // 返回已解析的响应帧
}

/**
 * @brief Push GPS data to camera
 * 
 * Wraps the supplied GPS data frame into a protocol frame and sends it to the camera.
 * This function does not gather GPS data internally - the caller must provide
 * a fully-filled gps_data_push_command_frame structure.
 * 
 * DJI Protocol Details:
 * - Command Set: 0x00
 * - Command ID: 0x17
 * - Command Type: CMD_NO_RESPONSE (push data, no response expected)
 * - Total frame size: ~66 bytes (including protocol headers)
 * - Payload size: ~45 bytes (gps_data_push_command_frame_t structure)
 * 
 * The camera stores GPS metadata in recorded video files only when the GPS push
 * format exactly matches DJI's expectations. This function uses the correct
 * protocol command (0x00/0x17) as observed in working iOS implementations.
 * 
 * @param frame Pointer to fully-filled GPS data push command frame
 * @return Pointer to response frame (NULL for CMD_NO_RESPONSE commands)
 */
gps_data_push_response_frame_t* command_logic_push_gps_data(int camera_index, const gps_data_push_command_frame_t* frame) {
    if (frame == NULL) {
        ESP_LOGE(TAG, "Camera %d: GPS frame is NULL", camera_index);
        return NULL;
    }

    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGD("GPS_TX", "Camera %d: Not sending GPS - camera not connected", camera_index);
        return NULL;
    }

    uint16_t seq = generate_seq();

    // Send GPS data push command (no response expected)
    // DJI protocol: GPS push uses CmdSet=0x00, CmdID=0x17
    // This is the correct command as observed in working iOS app implementations
    // Frame size should be ~66 bytes total (including protocol headers)
    CommandResult result = send_command(
        camera_index,
        0x00,                    // Command set 0x00 for GPS data push (DJI protocol)
        0x17,                    // Command ID 0x17 for GPS data push (DJI protocol)
        CMD_NO_RESPONSE,         // No response expected for push data
        frame,
        seq,
        0                        // No timeout for no-response commands
    );

    // For CMD_NO_RESPONSE commands, send_command returns {NULL, 0} on both success and failure
    // We check if send_command logged an error by checking if it returned early
    // If BLE was not connected or frame creation failed, result will be {NULL, 0}
    // But we already check connection state above, so if we get here and result is {NULL, 0},
    // it means send_command failed internally (logged already)
    // For success, result is also {NULL, 0}, so we can't distinguish by result alone
    // The send_command function logs errors internally, so we don't need to log again here
    
    // Clean up if structure was allocated (shouldn't happen for CMD_NO_RESPONSE)
    if (result.structure != NULL) {
        free(result.structure);
    }

    // CMD_NO_RESPONSE commands don't return a response frame
    return NULL;
}

/**
 * @brief Check if a camera slot supports highlight tags
 * 
 * Determines if the camera model in the specified slot supports highlight tags
 * based on its device_id. Action 4, 5 Pro, and 6 support highlights; Osmo 360 does not.
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if camera supports highlight tags, false otherwise
 */
bool command_logic_slot_supports_highlight(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return false;
    }
    
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    uint32_t device_id = g_camera_states[slot_index].device_id;
    
    // Return false if device_id is not yet known (0)
    if (device_id == 0) {
        return false;
    }
    
    // Check for supported models: Action 4 (0xFF33), Action 5 (0xFF44), Action 6 (0xFF55)
    if (device_id == 0xFF33 || device_id == 0xFF44 || device_id == 0xFF55) {
        return true;
    }
    
    // Osmo 360 (0xFF66) and any unknown models do not support highlights
    return false;
}

/**
 * @brief Check if a camera slot is paired
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if slot is paired, false otherwise
 */
bool command_logic_slot_is_paired(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return false;
    }
    
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    return g_camera_states[slot_index].is_paired;
}

/**
 * @brief Check if a camera slot is connected
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if slot is connected, false otherwise
 */
bool command_logic_slot_is_connected(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return false;
    }
    
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    return g_camera_states[slot_index].is_connected && 
           g_camera_states[slot_index].connection_state == CAM_STATE_CONNECTED;
}

/**
 * @brief Check if a camera slot is awake (not sleeping)
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if camera is awake, false if sleeping or invalid slot
 */
bool command_logic_slot_is_awake(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return false;
    }
    
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    // power_mode 3 indicates sleep mode
    return g_camera_states[slot_index].power_mode != 3;
}

/**
 * @brief Check if a camera slot is currently recording
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if camera is recording, false otherwise
 */
bool command_logic_slot_is_recording(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return false;
    }
    
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    return g_camera_states[slot_index].is_recording;
}

/**
 * @brief Send Key Reporting command to a specific camera slot
 * 
 * Generic function for sending Key Reporting (0011) commands with any key code.
 * Used for highlight tags, snapshot, and other key-based operations.
 * 
 * @param camera_index Camera slot index (0-2)
 * @param key_code Key code (0x02=QS button, 0x03=SNAPSHOT button, etc.)
 * @param mode Mode (0x01=Report key event)
 * @param key_value Key value (0x00=Short press event)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t command_logic_send_key_report_for_slot(int camera_index, uint8_t key_code, uint8_t mode, uint8_t key_value) {
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_ERR_INVALID_ARG;
    }
    
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    camera_state_t *cam_state = &g_camera_states[camera_index];
    
    // Check if slot is paired
    if (!cam_state->is_paired) {
        ESP_LOGD(TAG, "Camera %d: Not paired, cannot send key report", camera_index);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if slot is connected
    if (!cam_state->is_connected || cam_state->connection_state != CAM_STATE_CONNECTED) {
        ESP_LOGD(TAG, "Camera %d: Not connected, cannot send key report", camera_index);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check per-camera connection state (not global state)
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: Not connected, cannot send key report", camera_index);
        return ESP_ERR_INVALID_STATE;
    }
    
    uint16_t seq = generate_seq();
    
    // Build Key Reporting command frame
    key_report_command_frame_t command_frame = {
        .key_code = key_code,
        .mode = mode,
        .key_value = key_value
    };
    
    ESP_LOGI(TAG, "Camera %d: Sending Key Reporting command (key_code=0x%02X, mode=0x%02X, key_value=0x%02X)", 
             camera_index, key_code, mode, key_value);
    
    // Send command without waiting for response (non-blocking)
    CommandResult result = send_command(
        camera_index,
        0x00,                    // Command set 0x00 for Key Reporting
        0x11,                    // Command ID 0x11 for Key Reporting
        CMD_NO_RESPONSE,         // Non-blocking: don't wait for response
        &command_frame,
        seq,
        0                        // No timeout for non-blocking
    );
    
    // For CMD_NO_RESPONSE, result is always {NULL, 0}
    // Success is indicated by send_command not logging errors
    if (result.structure != NULL) {
        free(result.structure);
    }
    
    ESP_LOGI(TAG, "Camera %d: Key Reporting command sent", camera_index);
    return ESP_OK;
}

/**
 * @brief Send SNAPSHOT key command to a specific camera slot
 * 
 * Sends a Key Reporting (0011) command with SNAPSHOT key code (0x03) to trigger
 * a snapshot capture. This is used for snapshot mode when a camera wakes up from sleep.
 * 
 * @param camera_index Camera slot index (0-2)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t command_logic_send_snapshot_key_for_slot(int camera_index) {
    ESP_LOGI(TAG, "COMMAND_LOGIC: Sending SNAPSHOT key (key_code=0x03) to camera %d", camera_index);
    return command_logic_send_key_report_for_slot(camera_index, 0x03, 0x01, 0x00);
}

/**
 * @brief Send highlight tag command to a specific camera slot
 * 
 * Sends a Key Reporting (0011) command that simulates a short press of the QS button
 * to set a highlight marker in the currently-recording clip. Only works for cameras
 * that support highlights (Action 4, 5 Pro, 6) and only when the camera is recording.
 * 
 * @param slot_index Camera slot index (0-2)
 * @return ESP_OK on success, ESP_FAIL or error code on failure
 */
esp_err_t command_logic_send_highlight_for_slot(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return ESP_ERR_INVALID_ARG;
    }
    
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    camera_state_t *cam_state = &g_camera_states[slot_index];
    
    // Check if camera supports highlight tags
    if (!command_logic_slot_supports_highlight(slot_index)) {
        ESP_LOGD(TAG, "Camera %d: Does not support highlight tags (device_id=0x%04X)", 
                 slot_index, (unsigned int)cam_state->device_id);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    // Use generic key reporting function with QS button key code (0x02)
    return command_logic_send_key_report_for_slot(slot_index, 0x02, 0x01, 0x00);
}

/**
 * @brief Send highlight tag command to all eligible cameras
 * 
 * Sends highlight tag commands to all camera slots that meet the conditions:
 * - Paired and connected
 * - Awake (not sleeping)
 * - Currently recording
 * - Support highlight tags (Action 4, 5 Pro, 6, not Osmo 360)
 * 
 * @return ESP_OK if at least one highlight was sent, ESP_FAIL if none eligible
 */
esp_err_t command_logic_send_highlight_for_all_active(void) {
    int success_count = 0;
    
    for (int i = 0; i < NUM_CAMERAS; i++) {
        // Check all conditions for this slot
        if (command_logic_slot_is_paired(i) &&
            command_logic_slot_is_connected(i) &&
            command_logic_slot_is_awake(i) &&
            command_logic_slot_is_recording(i) &&
            command_logic_slot_supports_highlight(i)) {
            
            esp_err_t ret = command_logic_send_highlight_for_slot(i);
            if (ret == ESP_OK) {
                success_count++;
            }
        }
    }
    
    if (success_count > 0) {
        ESP_LOGI(TAG, "Highlight tag sent to %d camera(s)", success_count);
        return ESP_OK;
    } else {
        ESP_LOGD(TAG, "No eligible cameras for highlight tag");
        return ESP_FAIL;
    }
}
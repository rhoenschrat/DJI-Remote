/*
 * DJI Camera Remote Control - Connection Logic Layer
 * 
 * This file implements the complete connection management system for DJI camera
 * communication, handling both BLE and protocol-level connections.
 * 
 * Connection Flow:
 * 1. BLE Initialization: Set up ESP32 BLE stack
 * 2. Device Discovery: Scan for and connect to target camera
 * 3. Service Discovery: Find DJI communication characteristics
 * 4. Protocol Handshake: Establish DJI protocol connection
 * 5. Maintenance: Handle disconnections and reconnection attempts
 * 
 * State Management:
 * - BLE_NOT_INIT: Initial state before BLE initialization
 * - BLE_INIT_COMPLETE: BLE ready, no connection
 * - BLE_SEARCHING: Actively scanning for cameras
 * - BLE_CONNECTED: BLE link established, protocol pending
 * - PROTOCOL_CONNECTED: Full connection, ready for commands
 * - BLE_DISCONNECTING: Graceful disconnection in progress
 * 
 * Connection Types:
 * - Pairing (verify_mode=1): Initial camera registration
 * - Reconnection (verify_mode=0): Automatic connection to known camera
 * - Wake-up: BLE advertising to rouse sleeping cameras
 * 
 * Error Handling:
 * - Automatic reconnection attempts on unexpected disconnection
 * - Timeout management for all connection phases
 * - State restoration on connection failures
 * 
 * Hardware: M5Stack Basic V2.7 with ESP32 BLE capabilities
 * Protocol: DJI proprietary communication protocol
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ble.h"
#include "data.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "dji_protocol_data_structures.h"
#include "../main/ui.h"  // For camera_state_t and g_camera_states

/* Logging tag for ESP_LOG functions */
#define TAG "LOGIC_CONNECT"

/* Boot scan timeout in milliseconds */
#define BOOT_SCAN_TIMEOUT_MS 30000

/* Global connection state tracking
 * Manages the current state of BLE and protocol connections
 * Used throughout the system to determine available operations
 */
static connect_state_t connect_state = BLE_NOT_INIT;

/* Boot scan tracking structure
 * Manages the single boot scan for all paired camera slots
 */
static bool s_boot_scan_active = false;
static uint32_t s_boot_scan_start_tick = 0;
static bool s_boot_scan_slot_pending[3] = {false, false, false};
static bool s_boot_scan_slot_found[3] = {false, false, false};
static bool s_boot_scan_slot_not_found[3] = {false, false, false};  // Track cameras not found during boot scan

/* Boot connect phase tracking
 * True when the system is connecting to cameras found during boot scan.
 * Set to true after boot scan ends and connections begin.
 * Set to false after all boot-initiated connection attempts complete.
 */
static bool s_boot_connect_in_progress = false;

/* Per-slot connection attempt tracking
 * Tracks whether an active connection attempt is in progress for each slot
 */
static bool s_slot_is_connecting[3] = {false, false, false};

/**
 * @brief Get current connection state
 * 
 * Returns the current state of the connection system, allowing other
 * components to determine what operations are available and respond
 * appropriately to connection status changes.
 * 
 * @return connect_state_t Current connection state
 */
connect_state_t connect_logic_get_state(void) {
    return connect_state;
}

/**
 * @brief Handle camera disconnection events
 * 
 * Callback function triggered when the BLE connection to the camera is lost.
 * Implements sophisticated disconnection handling based on current state:
 * 
 * - Expected disconnections: Clean state reset
 * - Unexpected disconnections: Automatic reconnection attempt
 * - Failed reconnections: Graceful fallback to disconnected state
 * 
 * The function attempts one automatic reconnection for unexpected disconnections
 * to maintain seamless operation during temporary connection issues.
 */
void receive_camera_disconnect_handler() {
    switch (connect_state) {
        case BLE_SEARCHING:
            /* Already searching - no action needed */
            break;
        case BLE_INIT_COMPLETE:
            ESP_LOGI(TAG, "Already in DISCONNECTED state.");
            break;
        case BLE_DISCONNECTING: {
            ESP_LOGI(TAG, "Normal disconnection process.");
            /* Expected disconnection - clean state reset */
            connect_state = BLE_INIT_COMPLETE;
            camera_status_initialized = false;
            ESP_LOGI(TAG, "Current state: DISCONNECTED.");
            break;
        }
        case BLE_CONNECTED:
        case PROTOCOL_CONNECTED:
        default: {
            ESP_LOGW(TAG, "Unexpected disconnection from state: %d, attempting reconnection...", connect_state);
            
            /* Unexpected disconnection - attempt automatic reconnection */
            bool reconnected = false;
            ESP_LOGI(TAG, "Reconnection attempt...");
            // Attempt reconnection for camera 0
            if (connect_logic_ble_connect(0, SCAN_MODE_SLOT_RECONNECT) == ESP_OK) {
                /* Wait up to 30 seconds for reconnection to complete */
                for (int j = 0; j < 300; j++) {
                    if (ble_is_camera_connected(0)) {  // Check camera 0 connection
                        ESP_LOGI(TAG, "Reconnection successful");
                        reconnected = true;
                        return;  /* Successful reconnection - maintain current operation */
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            if (!reconnected) {
                ESP_LOGE(TAG, "Reconnection failed after 1 attempts");
                /* Reconnection failed - perform clean disconnection */
                connect_state = BLE_INIT_COMPLETE;
                camera_status_initialized = false;
                ble_disconnect(0);  // Disconnect camera 0
                ESP_LOGI(TAG, "Current state: DISCONNECTED.");
            }
            break;
        }
    }
}

/**
 * @brief Initialize BLE subsystem for camera communication
 * 
 * Performs one-time initialization of the ESP32 BLE stack and prepares
 * the system for camera connections. This must be called before any
 * connection attempts.
 * 
 * Initialization includes:
 * - ESP32 BLE controller and host setup
 * - GATT client profile registration
 * - Service and characteristic UUID configuration
 * - Connection parameter setup
 * 
 * @return 0 on success, -1 on failure
 */
int connect_logic_ble_init() {
    esp_err_t ret;

    /* Initialize ESP32 BLE stack with DJI camera service configuration */
    ret = ble_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE, error: %s", esp_err_to_name(ret));
        return -1;
    }

    connect_state = BLE_INIT_COMPLETE;
    ESP_LOGI(TAG, "BLE init successfully");
    return 0;
}

/**
 * @brief Connect to BLE device
 * 
 * Execute the following steps: set callbacks, start scanning and attempt connection, wait for connection completion and characteristic handle discovery.
 * 
 * If connection fails, returns error and resets connection state.
 * 
 * @param camera_index Camera slot index (0-2)
 * @param scan_mode Scan mode to use (PAIRING, AUTOCONNECT_BOOT, or SLOT_RECONNECT)
 * @return int Returns 0 on success, -1 on failure
 */
int connect_logic_ble_connect(int camera_index, scan_mode_t scan_mode) {
    // Mark this slot as connecting
    if (camera_index >= 0 && camera_index < 3) {
        s_slot_is_connecting[camera_index] = true;
    }
    
    // Don't set BLE_SEARCHING for AUTOCONNECT_BOOT mode - it's handled by boot scan logic
    if (scan_mode != SCAN_MODE_AUTOCONNECT_BOOT) {
        connect_state = BLE_SEARCHING;
    }

    esp_err_t ret;

    /* 1. Set a global Notify callback for receiving remote data and protocol parsing */
    ble_set_notify_callback(receive_camera_notify_handler);
    ble_set_state_callback(receive_camera_disconnect_handler);

    /* 2. Start scanning with specified mode */
    ESP_LOGI(TAG, "Starting BLE scan for camera %d with mode %d", camera_index, scan_mode);
    ret = ble_start_scan(scan_mode, camera_index, 30000);  // 30 second timeout
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scan for camera %d, error: 0x%x", camera_index, ret);
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 3. Wait up to 30 seconds to ensure BLE connection success */
    ESP_LOGI(TAG, "Waiting up to 10s for BLE to connect for camera %d...", camera_index);
    bool connected = false;
    for (int i = 0; i < 100; i++) { // 100 * 100ms = 10s
        if (ble_is_camera_connected(camera_index)) {  // Check specified camera connection
            ESP_LOGI(TAG, "BLE connected successfully to camera %d", camera_index);
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!connected) {
        ESP_LOGW(TAG, "BLE connection timed out for camera %d", camera_index);
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 4. Wait for characteristic handle discovery completion (up to 10 seconds) */
    ESP_LOGI(TAG, "Waiting up to 10s for characteristic handles discovery...");
    bool handles_found = false;
    for (int i = 0; i < 100; i++) { // 100 * 100ms = 10s
        if (ble_get_notify_handle(camera_index) != 0 && ble_get_write_handle(camera_index) != 0) {
            ESP_LOGI(TAG, "Required characteristic handles found for camera %d", camera_index);
            handles_found = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!handles_found) {
        ESP_LOGW(TAG, "Characteristic handles not found within timeout for camera %d", camera_index);
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 5. Register notification */
    ret = ble_register_notify(ble_get_conn_id(camera_index), ble_get_notify_handle(camera_index));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register notify for camera %d, error: %s", camera_index, esp_err_to_name(ret));
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    // Update state to BLE connected
    connect_state = BLE_CONNECTED;

    // Delay RGB light display
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "BLE connect successfully");
    return 0;
}

/**
 * @brief Connect directly to a camera without scanning (for boot scan results)
 * 
 * Used after boot scan completes to connect to cameras that were already
 * discovered during the scan. This avoids starting a new per-camera scan.
 * The target device info must already be set via ble_set_target_device().
 * 
 * @param camera_index Camera slot index (0-2)
 * @return int 0 on success, -1 on failure
 */
int connect_logic_ble_connect_direct(int camera_index) {
    if (camera_index < 0 || camera_index >= 3) {
        ESP_LOGE(TAG, "connect_logic_ble_connect_direct: Invalid camera index: %d", camera_index);
        return -1;
    }
    
    ESP_LOGI(TAG, "AUTOCONNECT_BOOT direct connection for slot %d (no new scan)", camera_index);
    
    // Mark this slot as connecting
    s_slot_is_connecting[camera_index] = true;
    
    // Note: We don't set connect_state to BLE_SEARCHING because we're not scanning
    // The boot scan has already completed

    esp_err_t ret;

    /* 1. Set global Notify callback for receiving remote data and protocol parsing */
    ble_set_notify_callback(receive_camera_notify_handler);
    ble_set_state_callback(receive_camera_disconnect_handler);

    /* 2. Initiate direct connection (no scanning) */
    ESP_LOGI(TAG, "Starting direct BLE connection for camera %d", camera_index);
    ret = ble_connect_direct(camera_index);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate direct connection for camera %d, error: 0x%x", camera_index, ret);
        s_slot_is_connecting[camera_index] = false;
        return -1;
    }

    /* 3. Wait up to 10 seconds to ensure BLE connection success */
    ESP_LOGI(TAG, "Waiting up to 10s for BLE direct connect for camera %d...", camera_index);
    bool connected = false;
    for (int i = 0; i < 100; i++) { // 100 * 100ms = 10s
        if (ble_is_camera_connected(camera_index)) {
            ESP_LOGI(TAG, "BLE connected successfully to camera %d (direct)", camera_index);
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!connected) {
        ESP_LOGW(TAG, "BLE direct connection timed out for camera %d", camera_index);
        s_slot_is_connecting[camera_index] = false;
        return -1;
    }

    /* 4. Wait for characteristic handle discovery completion (up to 10 seconds) */
    ESP_LOGI(TAG, "Waiting up to 10s for characteristic handles discovery...");
    bool handles_found = false;
    for (int i = 0; i < 100; i++) { // 100 * 100ms = 10s
        if (ble_get_notify_handle(camera_index) != 0 && ble_get_write_handle(camera_index) != 0) {
            ESP_LOGI(TAG, "Required characteristic handles found for camera %d", camera_index);
            handles_found = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!handles_found) {
        ESP_LOGW(TAG, "Characteristic handles not found within timeout for camera %d", camera_index);
        s_slot_is_connecting[camera_index] = false;
        return -1;
    }

    /* 5. Register notification */
    ret = ble_register_notify(ble_get_conn_id(camera_index), ble_get_notify_handle(camera_index));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register notify for camera %d, error: %s", camera_index, esp_err_to_name(ret));
        s_slot_is_connecting[camera_index] = false;
        return -1;
    }

    // Update state to BLE connected
    connect_state = BLE_CONNECTED;

    // Small delay for stability
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "BLE direct connect successfully for camera %d", camera_index);
    return 0;
}

/**
 * @brief Disconnect BLE connection
 * 
 * Attempt to disconnect from BLE device.
 * 
 * @return int Returns 0 on success, -1 on failure
 */
int connect_logic_ble_disconnect(void) {
    connect_state_t old_state = connect_state;
    connect_state = BLE_DISCONNECTING;
    
    ESP_LOGI(TAG, "Disconnecting camera...");

    // Call BLE layer's ble_disconnect function for camera 0
    esp_err_t ret = ble_disconnect(0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect camera 0, BLE error: %s", esp_err_to_name(ret));
        connect_state = old_state;
        return -1;
    }

    ESP_LOGI(TAG, "Camera disconnected successfully");
    return 0;
}

/**
 * @brief Establish DJI protocol connection with camera
 * 
 * Performs the complete DJI protocol handshake sequence over the established
 * BLE connection. This involves a complex bidirectional authentication process
 * that varies depending on whether this is a new pairing or reconnection.
 * 
 * Protocol Handshake Sequence:
 * 1. Send connection request with device credentials
 * 2. Handle camera response (may be response or command frame)
 * 3. Wait for camera's connection command with verification
 * 4. Send final connection response to complete handshake
 * 
 * Verification Modes:
 * - verify_mode=0: Reconnection to previously paired camera
 * - verify_mode=1: New pairing requiring camera-side confirmation
 * - verify_mode=2: Camera verification response
 * 
 * @param device_id Unique device identifier for this remote
 * @param mac_addr_len Length of MAC address (typically 6)
 * @param mac_addr Device MAC address for protocol identification
 * @param fw_version Firmware version for compatibility checking
 * @param verify_mode Authentication mode (0=reconnect, 1=pair)
 * @param verify_data Random verification code for security
 * @param camera_reserved Camera-specific identifier
 * @return 0 on successful protocol connection, -1 on failure
 */
int connect_logic_protocol_connect(int camera_index, uint32_t device_id, uint8_t mac_addr_len, const int8_t *mac_addr,
                                   uint32_t fw_version, uint8_t verify_mode, uint16_t verify_data,
                                   uint8_t camera_reserved) {
    ESP_LOGI(TAG, "%s: Camera %d: Starting protocol connection", __FUNCTION__, camera_index);
    uint16_t seq = generate_seq();

    /* Construct DJI protocol connection request frame */
    connection_request_command_frame connection_request = {
        .device_id = device_id,
        .mac_addr_len = mac_addr_len,
        .fw_version = fw_version,
        .verify_mode = verify_mode,
        .verify_data = verify_data,
    };
    memcpy(connection_request.mac_addr, mac_addr, mac_addr_len);


    // STEP1: Send connection request command to camera
    ESP_LOGI(TAG, "Sending connection request to camera %d...", camera_index);
    CommandResult result = send_command(camera_index, 0x00, 0x19, CMD_WAIT_RESULT, &connection_request, seq, 1000);

    /**** Connection issue: camera may return either response frame or command frame ****/

    if (result.structure == NULL) {
        // If a command frame is sent, execute this block of code

        // Directly call data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);
        
        // If != OK, it means no message was received, timeout occurred
        
        // Otherwise, GOTO wait_for_camera_command label
        void *parse_result = NULL;
        size_t parse_result_length = 0;
        uint16_t received_seq = 0;
        esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 1000, &received_seq, &parse_result, &parse_result_length);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Timeout or error waiting for camera connection command, GOTO Failed.");
            if (camera_index >= 0 && camera_index < 3) {
                s_slot_is_connecting[camera_index] = false;
            }
            connect_logic_ble_disconnect();
            return -1;
        } else {
            // If data is received, skip parsing camera response and directly enter STEP3
            goto wait_for_camera_command;
        }
    }

    // STEP2: Parse the response returned from camera
    connection_request_response_frame *response = (connection_request_response_frame *)result.structure;
    if (response->ret_code != 0) {
        ESP_LOGE(TAG, "Connection handshake failed: unexpected response from camera, ret_code: %d", response->ret_code);
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        free(response);
        connect_logic_ble_disconnect();
        return -1;
    }

    ESP_LOGI(TAG, "Handshake successful, waiting for the camera to actively send the connection command frame...");
    free(response);

    // STEP3: Wait for camera to send connection request
wait_for_camera_command:
    void *parse_result = NULL;
    size_t parse_result_length = 0;
    uint16_t received_seq = 0;
    esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);

    if (ret != ESP_OK || parse_result == NULL) {
        ESP_LOGE(TAG, "Timeout or error waiting for camera connection command");
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        connect_logic_ble_disconnect();
        return -1;
    }

    // Parse the connection request command sent by camera
    connection_request_command_frame *camera_request = (connection_request_command_frame *)parse_result;
    
    // Extract and save camera's device_id for model identification
    uint32_t camera_device_id = camera_request->device_id;
    ESP_LOGI(TAG, "Camera %d device_id: 0x%04X", camera_index, (unsigned int)camera_device_id);
    
    // Save device_id to camera state for persistent model name display
    g_camera_states[camera_index].device_id = camera_device_id;
    
    // Set model name based on device_id
    const char* model_name = ui_get_camera_model_name(camera_device_id);
    strncpy(g_camera_states[camera_index].model_name, model_name, 
            sizeof(g_camera_states[camera_index].model_name) - 1);
    g_camera_states[camera_index].model_name[sizeof(g_camera_states[camera_index].model_name) - 1] = '\0';
    ESP_LOGI(TAG, "Camera %d identified as: %s", camera_index, model_name);

    if (camera_request->verify_mode != 2) {
        ESP_LOGE(TAG, "Unexpected verify_mode from camera: %d", camera_request->verify_mode);
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        free(parse_result);
        connect_logic_ble_disconnect();
        return -1;
    }

    if (camera_request->verify_data == 0) {
        ESP_LOGI(TAG, "Camera approved the connection, sending response...");

        // Construct connection response frame
        connection_request_response_frame connection_response = {
            .device_id = device_id,
            .ret_code = 0,
        };
        memset(connection_response.reserved, 0, sizeof(connection_response.reserved));
        connection_response.reserved[0] = camera_reserved;

        ESP_LOGI(TAG, "Constructed connection response for camera %d, sending...", camera_index);

        // STEP4: Send connection response frame
        send_command(camera_index, 0x00, 0x19, ACK_NO_RESPONSE, &connection_response, received_seq, 5000);

        // Set connection state to protocol connected
        connect_state = PROTOCOL_CONNECTED;
        
        // Clear connecting flag - connection is now complete
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }

        ESP_LOGI(TAG, "Connection successfully established with camera.");
        
        // Subscribe to camera status push (CmdSet=0x1D, CmdID=0x05)
        // This must be done after handshake completes to receive status updates
        // The camera will then periodically send status pushes via CmdSet=0x1D, CmdID=0x02
        ESP_LOGI(TAG, "Subscribing to camera status push (0x1D/0x05)...");
        extern int subscript_camera_status(int camera_index, uint8_t push_mode, uint8_t push_freq);
        subscript_camera_status(camera_index, 3, 20);  // PUSH_MODE_PERIODIC_WITH_STATE_CHANGE=3, PUSH_FREQ_2HZ=20
        
        // Save camera pairing (including device_id) to NVS for persistence
        esp_err_t save_err = save_all_cameras_to_nvs();
        if (save_err == ESP_OK) {
            ESP_LOGI(TAG, "Camera %d pairing saved to NVS (device_id: 0x%04X)", 
                     camera_index, (unsigned int)camera_device_id);
        } else {
            ESP_LOGW(TAG, "Failed to save camera %d pairing to NVS", camera_index);
        }
        
        free(parse_result);
        return 0;
    } else {
        ESP_LOGW(TAG, "Camera rejected the connection, closing Bluetooth link...");
        if (camera_index >= 0 && camera_index < 3) {
            s_slot_is_connecting[camera_index] = false;
        }
        free(parse_result);
        connect_logic_ble_disconnect();
        return -1;
    }
}

int connect_logic_ble_wakeup(void) {
    ESP_LOGI(TAG, "Attempting to wake up camera via BLE advertising");

    esp_err_t ret = ble_start_advertising();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %s", esp_err_to_name(ret));
        return -1;
    }

    ESP_LOGI(TAG, "BLE advertising started, attempting to wake up camera");
    return 0;
}

/**
 * @brief Start wake-up broadcast for a specific camera slot
 * 
 * Initiates a BLE advertising broadcast to wake up a sleeping camera.
 * The broadcast uses the "WKP" format with the camera's MAC address.
 * 
 * @param camera_index Camera slot index (0-2)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t connect_logic_start_wake_broadcast_for_slot(int camera_index) {
    if (camera_index < 0 || camera_index >= 3) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_ERR_INVALID_ARG;
    }
    
    // g_camera_states is already declared as extern in ui.h
    camera_state_t *cam_state = &g_camera_states[camera_index];
    
    // Check if slot is paired
    if (!cam_state->is_paired) {
        ESP_LOGE(TAG, "Camera %d: Not paired, cannot start wake broadcast", camera_index);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get camera MAC address (camera_mac is an array, not a pointer)
    const uint8_t *camera_mac = cam_state->camera_mac;
    // Check if MAC address is valid (not all zeros)
    if (camera_mac[0] == 0 && camera_mac[1] == 0 && camera_mac[2] == 0 && 
        camera_mac[3] == 0 && camera_mac[4] == 0 && camera_mac[5] == 0) {
        ESP_LOGE(TAG, "Camera %d: MAC address not available for wake broadcast", camera_index);
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting wake broadcast for camera %d (MAC: %02X:%02X:%02X:%02X:%02X:%02X)",
             camera_index,
             camera_mac[0], camera_mac[1], camera_mac[2],
             camera_mac[3], camera_mac[4], camera_mac[5]);
    
    // Call BLE wake function with camera MAC
    esp_err_t ret = ble_wake_camera(camera_mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start wake broadcast for camera %d: %s", camera_index, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Wake broadcast started successfully for camera %d", camera_index);
    return ESP_OK;
}

/**
 * @brief Check if any camera slot is paired
 * 
 * @return true if at least one slot is paired, false otherwise
 */
static bool connect_logic_has_any_paired_slot(void) {
    for (int i = 0; i < 3; i++) {
        if (g_camera_states[i].is_paired) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Reset all boot-scan tracking state
 */
static void connect_logic_reset_boot_scan_state(void) {
    s_boot_scan_active = false;
    s_boot_scan_start_tick = 0;
    s_boot_connect_in_progress = false;
    for (int i = 0; i < 3; i++) {
        s_boot_scan_slot_pending[i] = false;
        s_boot_scan_slot_found[i] = false;
        s_boot_scan_slot_not_found[i] = false;
        s_slot_is_connecting[i] = false;
    }
}

/**
 * @brief Mark a camera slot as found during boot scan
 * 
 * Called by BLE layer when a paired camera is discovered during boot scan
 * 
 * @param slot_index Camera slot index (0-2)
 */
void connect_logic_mark_slot_found(int slot_index) {
    if (slot_index < 0 || slot_index >= 3) {
        ESP_LOGW(TAG, "Invalid slot index %d in mark_slot_found", slot_index);
        return;
    }
    
    if (s_boot_scan_active && s_boot_scan_slot_pending[slot_index]) {
        s_boot_scan_slot_found[slot_index] = true;
        s_boot_scan_slot_pending[slot_index] = false;
        ESP_LOGI(TAG, "Boot scan: Slot %d marked as found", slot_index);
        
        // Trigger UI update to show found_icon for this slot
        extern ui_state_t g_ui_state;
        extern camera_state_t g_camera_states[NUM_CAMERAS];
        if (slot_index < NUM_CAMERAS) {
            g_camera_states[slot_index].needs_full_redraw = true;
            g_ui_state.display_needs_update = true;
            ESP_LOGI(TAG, "Boot scan: Triggered UI update for slot %d (found state changed)", slot_index);
        }
    }
}

/**
 * @brief Check if all pending slots have been found
 * 
 * @return true if all pending slots are found, false otherwise
 */
static bool connect_logic_all_slots_found(void) {
    for (int i = 0; i < 3; i++) {
        if (s_boot_scan_slot_pending[i]) {
            return false;  // Still waiting for this slot
        }
    }
    return true;  // All pending slots found
}

/**
 * @brief Start single boot scan for all paired camera slots
 * 
 * Initiates a single BLE scan that searches for all paired cameras simultaneously.
 * Sets connect_state to BLE_SEARCHING during the scan so UI can show scanning_icon.
 * 
 * @return 0 on success, -1 on failure or no paired slots
 */
int connect_logic_start_boot_scan(void) {
    // Check if any slot is paired
    if (!connect_logic_has_any_paired_slot()) {
        ESP_LOGI(TAG, "No paired slots found, skipping boot scan");
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }
    
    // Reset all camera connection states to "paired but not connected" before boot scan
    // This ensures we start with a clean state and don't show stale connections
    // Note: Sleep state (is_sleeping, power_mode) is preserved during disconnect/reconnect.
    // The sleep state will be updated when new Camera Status Push (1D02) packets arrive.
    for (int i = 0; i < 3; i++) {
        if (g_camera_states[i].is_paired) {
            g_camera_states[i].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
            g_camera_states[i].is_connected = false;
            if (g_camera_states[i].snapshot_pending) {
                ESP_LOGD(TAG, "Boot scan: Camera %d snapshot_pending cleared (reset before scan)", i);
            }
            g_camera_states[i].snapshot_pending = false;
            ESP_LOGI(TAG, "Boot scan: Reset slot %d to PAIRED_DISCONNECTED before scan", i);
        }
    }
    
    // Reset boot scan state
    connect_logic_reset_boot_scan_state();
    
    // Disconnect any stale BLE connections from previous sessions
    // This ensures we start with a clean state
    for (int i = 0; i < 3; i++) {
        if (ble_is_camera_connected(i)) {
            ESP_LOGI(TAG, "Boot scan: Disconnecting stale connection for slot %d", i);
            ble_disconnect(i);
        }
    }
    
    // Mark all paired slots as pending
    int pending_count = 0;
    for (int i = 0; i < 3; i++) {
        if (g_camera_states[i].is_paired) {
            s_boot_scan_slot_pending[i] = true;
            pending_count++;
            ESP_LOGI(TAG, "Boot scan: Slot %d marked as pending", i);
        }
    }
    
    if (pending_count == 0) {
        ESP_LOGI(TAG, "No pending slots, skipping boot scan");
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }
    
    // Set up BLE profiles with target MAC addresses for matching
    // This must be done before starting the scan so matching works
    for (int i = 0; i < 3; i++) {
        if (s_boot_scan_slot_pending[i]) {
            // Set target device info in BLE profile for this slot
            ble_set_target_device(i, g_camera_states[i].camera_name, g_camera_states[i].camera_mac);
            ESP_LOGI(TAG, "Boot scan: Set target for slot %d: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     i, g_camera_states[i].camera_name,
                     g_camera_states[i].camera_mac[0], g_camera_states[i].camera_mac[1],
                     g_camera_states[i].camera_mac[2], g_camera_states[i].camera_mac[3],
                     g_camera_states[i].camera_mac[4], g_camera_states[i].camera_mac[5]);
        }
    }
    
    // Set up autoconnect pending flags for BLE layer
    bool autoconnect_pending[BLE_MAX_CAMERAS] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        autoconnect_pending[i] = s_boot_scan_slot_pending[i];
    }
    ble_set_autoconnect_pending(autoconnect_pending);
    
    // Set connect_state to BLE_SEARCHING so UI shows scanning_icon
    connect_state = BLE_SEARCHING;
    
    // Start the boot scan
    esp_err_t ret = ble_start_scan(SCAN_MODE_AUTOCONNECT_BOOT, -1, BOOT_SCAN_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start boot scan: %s", esp_err_to_name(ret));
        connect_logic_reset_boot_scan_state();
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }
    
    // Record scan start time
    s_boot_scan_start_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_boot_scan_active = true;
    
    ESP_LOGI(TAG, "Boot scan started for %d paired slot(s), timeout=%d ms", 
             pending_count, BOOT_SCAN_TIMEOUT_MS);
    ESP_LOGI(TAG, "connect_state set to BLE_SEARCHING - scanning_icon should be visible");
    
    // Force UI update to show scanning indicator immediately
    g_ui_state.display_needs_update = true;
    
    return 0;
}

/**
 * @brief Update boot scan state and handle timeout/connection initiation
 * 
 * This function should be called periodically (every 100-500ms) during boot scan.
 * It checks for timeout, stops the scan when all slots are found or timeout expires,
 * and initiates connections for all found slots.
 * 
 * @return 0 if scan is still active, 1 if scan completed, -1 if not active
 */
int connect_logic_update_boot_scan(void) {
    if (!s_boot_scan_active) {
        return -1;  // Boot scan not active
    }
    
    uint32_t current_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed_ms = current_tick - s_boot_scan_start_tick;
    
    // Check if all slots are found
    bool all_found = connect_logic_all_slots_found();
    
    // Check if timeout expired
    bool timeout_expired = (elapsed_ms >= BOOT_SCAN_TIMEOUT_MS);
    
    // Check if scan is still running
    bool scan_still_running = ble_is_scanning();
    
    // If all found or timeout expired, stop the scan
    if ((all_found || timeout_expired) && scan_still_running) {
        ESP_LOGI(TAG, "Boot scan stopping: all_found=%d, timeout=%d, elapsed=%lu ms", 
                 all_found, timeout_expired, elapsed_ms);
        ble_stop_scan();
    }
    
    // If scan has stopped (either by us or by BLE layer), process results
    if (!scan_still_running || (!all_found && timeout_expired)) {
        // Scan is complete - process results
        s_boot_scan_active = false;
        
        // Leave BLE_SEARCHING state
        connect_state = BLE_INIT_COMPLETE;
        
        // Log results and mark cameras as not found
        int found_count = 0;
        int not_found_count = 0;
        for (int i = 0; i < 3; i++) {
            if (s_boot_scan_slot_found[i]) {
                found_count++;
                ESP_LOGI(TAG, "Boot scan: Slot %d found", i);
            } else if (s_boot_scan_slot_pending[i]) {
                not_found_count++;
                s_boot_scan_slot_not_found[i] = true;  // Mark as not found during boot scan
                ESP_LOGW(TAG, "Boot scan: Slot %d not found", i);
            }
        }
        
        ESP_LOGI(TAG, "Boot scan completed: %d found, %d not found", found_count, not_found_count);
        
        // Reset connection state for cameras that were not found
        // They should be in "paired but not connected" state
        // Note: Sleep state (is_sleeping, power_mode) is preserved - only connection state is reset.
        // When camera reconnects, status push will update sleep state based on actual power_mode.
        for (int i = 0; i < 3; i++) {
            if (g_camera_states[i].is_paired && !s_boot_scan_slot_found[i]) {
                g_camera_states[i].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
                g_camera_states[i].is_connected = false;
                if (g_camera_states[i].snapshot_pending) {
                    ESP_LOGD(TAG, "Boot scan: Camera %d snapshot_pending cleared (slot not found)", i);
                }
                g_camera_states[i].snapshot_pending = false;
                g_camera_states[i].needs_full_redraw = true;
                ESP_LOGI(TAG, "Boot scan: Slot %d not found, set to PAIRED_DISCONNECTED", i);
            }
        }
        
        // Set boot connect flag if any cameras were found
        if (found_count > 0) {
            s_boot_connect_in_progress = true;
            ESP_LOGI(TAG, "Boot connect phase starting for %d cameras", found_count);
        }
        
        // Trigger connection initiation in UI layer
        extern void ui_initiate_boot_scan_connections(void);
        ui_initiate_boot_scan_connections();
        
        return 1;  // Scan completed
    }
    
    return 0;  // Scan still active
}

/**
 * @brief Check if boot scan is currently active
 * 
 * @return true if boot scan is active, false otherwise
 */
bool connect_logic_is_boot_scan_active(void) {
    return s_boot_scan_active;
}

/**
 * @brief Check if boot-time connection phase is in progress
 * 
 * Returns true when the system is connecting to cameras found during boot scan.
 * This phase starts after the boot scan ends and lasts until all boot-initiated
 * connection attempts complete (regardless of success/failure).
 * 
 * @return true if boot connect phase is in progress, false otherwise
 */
bool connect_logic_is_boot_connect_in_progress(void) {
    return s_boot_connect_in_progress;
}

/**
 * @brief Clear the boot connect in progress flag
 * 
 * Called after all boot-initiated connection attempts have completed.
 * This allows the UI to show normal button mappings.
 */
void connect_logic_clear_boot_connect_flag(void) {
    s_boot_connect_in_progress = false;
    ESP_LOGI(TAG, "Boot connect phase completed");
}

/**
 * @brief Stop boot scan immediately and initiate connections to found cameras
 * 
 * This function stops the active boot scan and immediately begins connecting
 * to all cameras that were found during the scan. Called when user presses
 * Button C during an active boot scan.
 * 
 * Note: We do NOT mark all found cameras as "connecting" here. Instead, we let
 * ui_initiate_boot_scan_connections() set the connecting state for each camera
 * sequentially as it connects them one by one. This ensures:
 * - Only the camera currently being connected shows the connecting_icon
 * - Other found cameras keep showing the found_icon until their turn
 */
void connect_logic_stop_boot_scan_and_connect_found(void) {
    if (!s_boot_scan_active) {
        ESP_LOGI(TAG, "Boot scan not active, nothing to stop");
        return;
    }
    
    ESP_LOGI(TAG, "Stopping boot scan early (user requested)");
    
    // Stop the BLE scan immediately
    ble_stop_scan();
    
    // Mark boot scan as inactive
    s_boot_scan_active = false;
    
    // Leave BLE_SEARCHING state
    connect_state = BLE_INIT_COMPLETE;
    
    // Count found slots (for logging) but do NOT set connecting state
    // The connecting state will be set by ui_initiate_boot_scan_connections()
    // for each camera sequentially as it connects them
    int found_count = 0;
    for (int i = 0; i < 3; i++) {
        if (g_camera_states[i].is_paired && 
            s_boot_scan_slot_found[i] && 
            !g_camera_states[i].is_connected) {
            found_count++;
            ESP_LOGI(TAG, "Boot scan stop: Slot %d found, will connect", i);
        }
    }
    
    // Set boot connect flag if any cameras were found
    if (found_count > 0) {
        s_boot_connect_in_progress = true;
        ESP_LOGI(TAG, "Boot connect phase starting for %d cameras (user abort)", found_count);
    }
    
    // Initiate connections for all found slots
    // This will set connecting state for each camera one at a time
    extern void ui_initiate_boot_scan_connections(void);
    ui_initiate_boot_scan_connections();
    
    // Trigger UI refresh
    extern ui_state_t g_ui_state;
    g_ui_state.display_needs_update = true;
    
    ESP_LOGI(TAG, "Boot scan stopped, connections initiated for found cameras");
}

/**
 * @brief Get boot scan status for a specific slot
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if slot was found during boot scan, false otherwise
 */
bool connect_logic_is_slot_found(int slot_index) {
    if (slot_index < 0 || slot_index >= 3) {
        return false;
    }
    return s_boot_scan_slot_found[slot_index];
}

/**
 * @brief Check if a camera slot was not found during boot scan
 * 
 * This is used to prevent background reconnection from attempting to reconnect
 * cameras that were not available during boot scan (e.g., they were turned off).
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if slot was not found during boot scan, false otherwise
 */
bool connect_logic_was_slot_not_found_during_boot(int slot_index) {
    if (slot_index < 0 || slot_index >= 3) {
        return false;
    }
    return s_boot_scan_slot_not_found[slot_index];
}

/**
 * @brief Clear the "not found during boot" flag for a camera slot
 * 
 * This should be called when a camera is manually connected (e.g., by user action)
 * so that it can be reconnected by background reconnection if it disconnects later.
 * 
 * @param slot_index Camera slot index (0-2)
 */
void connect_logic_clear_slot_not_found_flag(int slot_index) {
    if (slot_index >= 0 && slot_index < 3) {
        s_boot_scan_slot_not_found[slot_index] = false;
        ESP_LOGI(TAG, "Cleared 'not found during boot' flag for slot %d", slot_index);
    }
}

/**
 * @brief Mark a camera slot as not found during boot scan
 * 
 * This sets the "not found during boot" flag for a camera slot, which prevents
 * background reconnection from attempting to reconnect this camera automatically.
 * The camera was not available at boot time, so it's likely turned off.
 * 
 * @param slot_index Camera slot index (0-2)
 */
void connect_logic_mark_slot_not_found_during_boot(int slot_index) {
    if (slot_index >= 0 && slot_index < 3) {
        s_boot_scan_slot_not_found[slot_index] = true;
        ESP_LOGI(TAG, "Marked slot %d as 'not found during boot'", slot_index);
    }
}

/**
 * @brief Check if a camera slot has an active connection attempt in progress
 * 
 * @param slot_index Camera slot index (0-2)
 * @return true if connection attempt is in progress, false otherwise
 */
bool connect_logic_slot_is_connecting(int slot_index) {
    if (slot_index < 0 || slot_index >= 3) {
        return false;
    }
    return s_slot_is_connecting[slot_index];
}

/**
 * @brief Set the connecting state for a camera slot
 * 
 * This function allows the UI layer to set the connecting flag before
 * calling connect_logic_ble_connect(), ensuring the UI can show the
 * connecting icon immediately.
 * 
 * @param slot_index Camera slot index (0-2)
 * @param is_connecting true to mark slot as connecting, false to clear
 */
void connect_logic_set_slot_connecting(int slot_index, bool is_connecting) {
    if (slot_index >= 0 && slot_index < 3) {
        s_slot_is_connecting[slot_index] = is_connecting;
    }
}
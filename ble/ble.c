/*
 * DJI Camera Remote Control - BLE Communication Layer
 * 
 * This file implements the complete Bluetooth Low Energy communication system
 * for connecting to and controlling DJI cameras. It provides the low-level
 * BLE interface that the connection logic layer uses for camera communication.
 * 
 * BLE Architecture:
 * - GATT Client role for connecting to DJI cameras (which act as servers)
 * - Service discovery for DJI-specific UUIDs
 * - Characteristic handling for data transmission and notifications
 * - Connection management with automatic reconnection capability
 * 
 * Key Features:
 * - Device scanning with RSSI-based best device selection
 * - Targeted reconnection to previously paired cameras
 * - Robust connection state management
 * - Wake-up advertising for sleeping cameras
 * - Data transmission with proper MTU handling
 * 
 * DJI Service Discovery:
 * - Scans for devices advertising DJI service UUIDs
 * - Discovers write and notify characteristics
 * - Handles GATT service enumeration
 * - Manages connection parameters optimization
 * 
 * Connection Modes:
 * - Initial pairing: Connects to any compatible DJI camera
 * - Reconnection: Targets specific camera by name and MAC address
 * - Wake broadcast: Sends advertising to wake sleeping cameras
 * 
 * The BLE layer provides callback-based notification to upper layers
 * for connection events and received data processing.
 * 
 * Hardware: M5Stack Basic V2.7 with ESP32 BLE capabilities
 * Protocol: BLE GATT over DJI proprietary service UUIDs
 */

#include <string.h>
#include "ble.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "../main/ui.h"  // For camera_state_t and NUM_CAMERAS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

/* Logging tag for ESP_LOG functions */
#define TAG "BLE"

/* Connection state tracking
 * Prevents multiple simultaneous connection attempts
 */
static bool s_connecting = false;

/* Global callback for received data notifications
 * Called when data is received from the camera via BLE notifications
 */
static ble_notify_callback_t s_notify_cb = NULL;

/* Global callback for connection state changes
 * Notifies upper layers of connection and disconnection events
 */
static connect_logic_state_callback_t s_state_cb = NULL;

/* Scan controller for slot-based scanning
 * Manages three distinct scan modes: pairing, autoconnect on boot, and manual reconnect
 */
static scan_controller_t s_scan_controller = {
    .mode = SCAN_MODE_IDLE,
    .target_slot = -1,
    .autoconnect_pending = {false, false, false},
    .scan_timeout_ms = 0,
    .scan_start_time = 0
};

/* BLE scan active flag - true when any BLE scan is running */
static bool s_ble_scan_active = false;

/* BLE GATT client profiles - one per camera slot
 * Stores all connection state, handles, and discovery information for each camera
 */
ble_profile_t s_ble_profiles[BLE_MAX_CAMERAS] = {0};

/* Active scanning camera index - which camera slot we're currently scanning for
 * Set to -1 when not actively scanning for a specific camera
 */
static int s_active_scan_camera_index = -1;

/**
 * @brief Initialize all BLE profiles to default state
 * Called during ble_init() to set up the profile array
 */
static void init_ble_profiles(void) {
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        s_ble_profiles[i].conn_id = 0;
        s_ble_profiles[i].gattc_if = ESP_GATT_IF_NONE;
        s_ble_profiles[i].camera_index = -1;  // Mark as unused
        s_ble_profiles[i].notify_char_handle = 0;
        s_ble_profiles[i].write_char_handle = 0;
        s_ble_profiles[i].read_char_handle = 0;
        s_ble_profiles[i].service_start_handle = 0;
        s_ble_profiles[i].service_end_handle = 0;
        memset(s_ble_profiles[i].remote_bda, 0, sizeof(esp_bd_addr_t));
        memset(s_ble_profiles[i].target_name, 0, sizeof(s_ble_profiles[i].target_name));
        memset(s_ble_profiles[i].target_mac, 0, sizeof(s_ble_profiles[i].target_mac));
        s_ble_profiles[i].connection_status.is_connected = false;
        s_ble_profiles[i].handle_discovery.notify_char_handle_found = false;
        s_ble_profiles[i].handle_discovery.write_char_handle_found = false;
    }
    ESP_LOGI(TAG, "Initialized %d BLE profiles for multi-camera support", BLE_MAX_CAMERAS);
}

/**
 * @brief Find profile by camera index
 * @return Pointer to profile or NULL if not found
 */
static ble_profile_t* get_profile_by_camera_index(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        return NULL;
    }
    // Return the profile for this camera slot (may or may not be active)
    return &s_ble_profiles[camera_index];
}

/**
 * @brief Find profile by connection ID
 * @return Pointer to profile or NULL if not found
 */
static ble_profile_t* get_profile_by_conn_id(uint16_t conn_id) {
    // First, try to find by conn_id and active connection status
    // This ensures we get the most recent connection
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        if (s_ble_profiles[i].camera_index >= 0 && 
            s_ble_profiles[i].conn_id == conn_id &&
            s_ble_profiles[i].connection_status.is_connected) {
            return &s_ble_profiles[i];
        }
    }
    // Fallback: find by conn_id only (for cases where connection status hasn't been set yet)
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        if (s_ble_profiles[i].camera_index >= 0 && s_ble_profiles[i].conn_id == conn_id) {
            return &s_ble_profiles[i];
        }
    }
    return NULL;
}

/**
 * @brief Find profile by GATT interface
 * @return Pointer to profile or NULL if not found
 */
__attribute__((unused))
static ble_profile_t* get_profile_by_gattc_if(esp_gatt_if_t gattc_if) {
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        if (s_ble_profiles[i].camera_index >= 0 && s_ble_profiles[i].gattc_if == gattc_if) {
            return &s_ble_profiles[i];
        }
    }
    return NULL;
}

/**
 * @brief Find profile by remote device address
 * @return Pointer to profile or NULL if not found
 */
__attribute__((unused))
static ble_profile_t* get_profile_by_bda(esp_bd_addr_t bda) {
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        if (s_ble_profiles[i].camera_index >= 0 && 
            memcmp(s_ble_profiles[i].remote_bda, bda, sizeof(esp_bd_addr_t)) == 0) {
            return &s_ble_profiles[i];
        }
    }
    return NULL;
}

/* Define the Service/Characteristic UUIDs to filter, for search use */
#define REMOTE_TARGET_SERVICE_UUID   0xFFF0
#define REMOTE_NOTIFY_CHAR_UUID      0xFFF4
#define REMOTE_WRITE_CHAR_UUID       0xFFF5

static esp_bt_uuid_t s_filter_notify_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = REMOTE_NOTIFY_CHAR_UUID,
};

static esp_bt_uuid_t s_filter_write_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = REMOTE_WRITE_CHAR_UUID,
};

static esp_bt_uuid_t s_notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
};

/* Scan parameters, adjustable as needed */
static esp_ble_scan_params_t s_ble_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = 0x50,
    .scan_window        = 0x30,
    .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE
};

/* Callback function declarations */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gattc_event_handler(esp_gattc_cb_event_t event,
                                esp_gatt_if_t gattc_if,
                                esp_ble_gattc_cb_param_t *param);

static TimerHandle_t scan_timer = NULL;

/**
 * @brief Stop and delete the scan timer if it exists
 * 
 * Call this before creating a new timer or when scan stops
 */
static void cleanup_scan_timer(void) {
    if (scan_timer != NULL) {
        xTimerStop(scan_timer, 0);
        xTimerDelete(scan_timer, 0);
        scan_timer = NULL;
    }
}

void scan_stop_timer_callback(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Scan timer expired, stopping scan (mode=%d)", s_scan_controller.mode);
    
    // Only stop scanning if we're actually in a scan mode
    if (s_scan_controller.mode != SCAN_MODE_IDLE) {
        esp_ble_gap_stop_scanning();
    }
    
    // Timer will be deleted in ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT handler
    // or when a new scan starts
}

static void trigger_scan_task(void) {
    ESP_LOGI(TAG, "esp_ble_gap_start_scanning (mode=%d, slot=%d)...", 
             s_scan_controller.mode, s_scan_controller.target_slot);
    esp_err_t ret = esp_ble_gap_start_scanning(4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scanning: %s", esp_err_to_name(ret));
        s_ble_scan_active = false;  // Scan failed to start
    } else {
        s_ble_scan_active = true;   // Scan started successfully
        ESP_LOGI(TAG, "Scan started successfully, s_ble_scan_active = true (mode=%d, slot=%d)", 
                 s_scan_controller.mode, s_scan_controller.target_slot);
    }
    
    // For AUTOCONNECT_BOOT mode, use the configured timeout instead of hardcoded 4 seconds
    // For other modes, use 4 seconds as before
    uint32_t timer_ms = 4000;  // Default 4 seconds
    if (s_scan_controller.mode == SCAN_MODE_AUTOCONNECT_BOOT && s_scan_controller.scan_timeout_ms > 0) {
        timer_ms = s_scan_controller.scan_timeout_ms;
        ESP_LOGI(TAG, "Using configured timeout %lu ms for AUTOCONNECT_BOOT scan", timer_ms);
    }
    
    // Clean up any existing timer before creating a new one
    cleanup_scan_timer();
    
    // Start a timer to stop scanning after the configured timeout
    scan_timer = xTimerCreate("scan_timer", pdMS_TO_TICKS(timer_ms), pdFALSE, (void *)0, scan_stop_timer_callback);
    if (scan_timer != NULL) {
        xTimerStart(scan_timer, 0);
    }
}

/* -------------------------
 *  Initialization/Scan/Connection related interfaces
 * ------------------------- */

/**
 * @brief BLE client initialization
 *
 * @return esp_err_t
 *         - ESP_OK on success
 *         - Others on failure
 */
esp_err_t ble_init() {
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    /* Initialize all camera profiles */
    init_ble_profiles();

    /* Release classic Bluetooth memory */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    /* Configure and initialize the Bluetooth controller */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "initialize controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start the BLE controller */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "enable controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize the Bluedroid stack */
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "init bluedroid failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Enable Bluedroid */
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "enable bluedroid failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register GAP callback */
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gap register error, err code = %x", ret);
        return ret;
    }

    /* Register GATTC callback */
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gattc register error, err code = %x", ret);
        return ret;
    }

    /* Register GATTC application (single app for all cameras) */
    ret = esp_ble_gattc_app_register(0);
    if (ret) {
        ESP_LOGE(TAG, "gattc app register error, err code = %x", ret);
        return ret;
    }

    /* Set local MTU (optional) */
    esp_ble_gatt_set_local_mtu(500);

    ESP_LOGI(TAG, "ble_init success!");
    return ESP_OK;
}

/**
 * @brief Start BLE scan with specified mode
 * 
 * @param mode Scan mode (PAIRING, AUTOCONNECT_BOOT, or SLOT_RECONNECT)
 * @param target_slot Camera slot index for slot-specific modes (-1 for AUTOCONNECT_BOOT)
 * @param timeout_ms Scan timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t ble_start_scan(scan_mode_t mode, int target_slot, uint32_t timeout_ms) {
    // Validate parameters based on mode
    if (mode == SCAN_MODE_PAIRING || mode == SCAN_MODE_SLOT_RECONNECT) {
        if (target_slot < 0 || target_slot >= BLE_MAX_CAMERAS) {
            ESP_LOGE(TAG, "Invalid target_slot %d for mode %d", target_slot, mode);
            return ESP_FAIL;
        }
    }
    
    // Initialize scan controller state
    s_scan_controller.mode = mode;
    s_scan_controller.target_slot = target_slot;
    s_scan_controller.scan_timeout_ms = timeout_ms;
    s_scan_controller.scan_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // For AUTOCONNECT_BOOT mode, autoconnect_pending flags should be set by caller
    // For other modes, clear all pending flags
    if (mode != SCAN_MODE_AUTOCONNECT_BOOT) {
        for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
            s_scan_controller.autoconnect_pending[i] = false;
        }
    }
    
    ESP_LOGI(TAG, "Starting scan: mode=%d, target_slot=%d, timeout=%lu ms", 
             mode, target_slot, timeout_ms);
    
    // Clear GATT cache for all known remote devices before starting scan
    // This removes stale cached data that could cause false positive detections
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        // Check if this profile has a valid remote BDA (not all zeros)
        bool has_valid_bda = false;
        for (int j = 0; j < 6; j++) {
            if (s_ble_profiles[i].remote_bda[j] != 0) {
                has_valid_bda = true;
                break;
            }
        }
        if (has_valid_bda) {
            esp_ble_gattc_cache_clean(s_ble_profiles[i].remote_bda);
        }
    }
    
    // Set scan parameters
    esp_err_t ret = esp_ble_gap_set_scan_params(&s_ble_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set scan params: %s", esp_err_to_name(ret));
        s_scan_controller.mode = SCAN_MODE_IDLE;
        return ret;
    }
    
    return ESP_OK;
}

/**
 * @brief Stop active BLE scan
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ble_stop_scan(void) {
    if (s_scan_controller.mode == SCAN_MODE_IDLE) {
        ESP_LOGI(TAG, "No active scan to stop");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping scan (mode was %d)", s_scan_controller.mode);
    
    // Clean up the scan timer
    cleanup_scan_timer();
    
    // Stop BLE scanning
    esp_err_t ret = esp_ble_gap_stop_scanning();
    
    // Reset scan controller to IDLE
    s_scan_controller.mode = SCAN_MODE_IDLE;
    s_scan_controller.target_slot = -1;
    s_scan_controller.scan_timeout_ms = 0;
    s_scan_controller.scan_start_time = 0;
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        s_scan_controller.autoconnect_pending[i] = false;
    }
    
    return ret;
}

/**
 * @brief Get current scan mode
 * 
 * @return scan_mode_t Current scan mode
 */
scan_mode_t ble_get_scan_mode(void) {
    return s_scan_controller.mode;
}

/**
 * @brief Set autoconnect pending flags for multiple slots
 * 
 * Used by AUTOCONNECT_BOOT mode to mark which slots need connection
 * 
 * @param pending Array of boolean flags for each camera slot
 */
void ble_set_autoconnect_pending(bool pending[BLE_MAX_CAMERAS]) {
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        s_scan_controller.autoconnect_pending[i] = pending[i];
        if (pending[i]) {
            ESP_LOGI(TAG, "Slot %d marked for autoconnect", i);
        }
    }
}

/**
 * @brief Check if BLE scan is currently active
 * 
 * @return true if scanning, false otherwise
 */
bool ble_is_scanning(void) {
    return s_ble_scan_active;
}

/**
 * @brief Connect to a device with a specified name
 *
 * @note  This function is kept for compatibility. New code should use ble_start_scan().
 * @return esp_err_t
 */
esp_err_t ble_start_scanning_and_connect(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_FAIL;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    if (!profile) {
        ESP_LOGE(TAG, "Failed to get profile for camera %d", camera_index);
        return ESP_FAIL;
    }
    
    // Check if this is a reconnection (target_mac is set) or new pairing
    bool has_target = false;
    for (int i = 0; i < 6; i++) {
        if (profile->target_mac[i] != 0) {
            has_target = true;
            break;
        }
    }
    
    // Set active scanning camera
    s_active_scan_camera_index = camera_index;
    
    // Use appropriate scan mode based on whether we have a target
    scan_mode_t mode = has_target ? SCAN_MODE_SLOT_RECONNECT : SCAN_MODE_PAIRING;
    
    ESP_LOGI(TAG, "Scan started for camera %d (mode=%d)", camera_index, mode);
    return ble_start_scan(mode, camera_index, 30000);  // 30 second timeout
}

static void try_to_connect(int camera_index, esp_bd_addr_t addr) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    if (!profile) {
        ESP_LOGE(TAG, "Failed to get profile for camera %d", camera_index);
        return;
    }
    
    // Check if already connecting
    if (s_connecting) {
        ESP_LOGW(TAG, "Already in connecting state, please wait...");
        return;
    }

    // Check if the address is valid (not all zeros)
    bool is_valid = false;
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        if (addr[i] != 0) {
            is_valid = true;
            break;
        }
    }

    if (!is_valid) {
        ESP_LOGE(TAG, "Invalid device address (all zeros) for camera %d", camera_index);
        return;
    }

    // Store the remote device address in the profile
    memcpy(profile->remote_bda, addr, sizeof(esp_bd_addr_t));
    
    s_connecting = true;
    ESP_LOGI(TAG, "Camera %d: Connecting to %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             camera_index, profile->target_name,
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Initiate GATT connection
    // The gattc_if should be set during ESP_GATTC_REG_EVT
    if (profile->gattc_if == ESP_GATT_IF_NONE) {
        ESP_LOGE(TAG, "Camera %d: GATT interface not registered yet", camera_index);
        s_connecting = false;
        return;
    }
    
    esp_ble_gattc_open(profile->gattc_if, addr, BLE_ADDR_TYPE_PUBLIC, true);
}

void ble_set_reconnecting(bool flag) {
    // Reconnection mode is now managed by scan_mode_t
    // This function is kept for compatibility but does nothing
    (void)flag;
    ESP_LOGW(TAG, "ble_set_reconnecting() is deprecated, use ble_start_scan() with appropriate mode");
}

bool ble_get_reconnecting(void) {
    // Return true if in reconnect scan mode
    return (s_scan_controller.mode == SCAN_MODE_SLOT_RECONNECT || 
            s_scan_controller.mode == SCAN_MODE_AUTOCONNECT_BOOT);
}

/**
 * @brief Reconnect to a previously paired camera
 * 
 * @param camera_index Camera slot index (0-2)
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t ble_reconnect(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_FAIL;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    if (!profile) {
        ESP_LOGE(TAG, "Failed to get profile for camera %d", camera_index);
        return ESP_FAIL;
    }
    
    // Check if target device is set
    bool is_valid = false;
    for (int i = 0; i < 6; i++) {
        if (profile->target_mac[i] != 0) {
            is_valid = true;
            break;
        }
    }

    if (!is_valid) {
        ESP_LOGE(TAG, "Camera %d: No valid target device set", camera_index);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Camera %d: Reconnecting to %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             camera_index, profile->target_name,
             profile->target_mac[0], profile->target_mac[1], profile->target_mac[2],
             profile->target_mac[3], profile->target_mac[4], profile->target_mac[5]);

    // Set active scanning camera
    s_active_scan_camera_index = camera_index;
    
    // Use SLOT_RECONNECT mode for targeted reconnection
    return ble_start_scan(SCAN_MODE_SLOT_RECONNECT, camera_index, 30000);  // 30 second timeout
}

/**
 * @brief Connect directly to a camera using stored MAC address (no scanning)
 * 
 * Used after boot scan to connect to cameras that were already discovered.
 * This avoids starting a new scan when we already know the device's MAC address.
 * The target device info must be set via ble_set_target_device() before calling.
 * 
 * @param camera_index Camera slot index (0-2)
 * @return esp_err_t ESP_OK on connection initiated, ESP_FAIL on failure
 */
esp_err_t ble_connect_direct(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "ble_connect_direct: Invalid camera index: %d", camera_index);
        return ESP_FAIL;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    if (!profile) {
        ESP_LOGE(TAG, "ble_connect_direct: Failed to get profile for camera %d", camera_index);
        return ESP_FAIL;
    }
    
    // Check if target MAC is set (should be set by ble_set_target_device during boot scan)
    bool has_valid_mac = false;
    for (int i = 0; i < 6; i++) {
        if (profile->target_mac[i] != 0) {
            has_valid_mac = true;
            break;
        }
    }
    
    if (!has_valid_mac) {
        ESP_LOGE(TAG, "ble_connect_direct: Camera %d has no valid target MAC address", camera_index);
        return ESP_FAIL;
    }
    
    // Check if already connecting
    if (s_connecting) {
        ESP_LOGW(TAG, "ble_connect_direct: Already connecting, please wait...");
        return ESP_FAIL;
    }
    
    // Check if GATT interface is registered
    if (profile->gattc_if == ESP_GATT_IF_NONE) {
        ESP_LOGE(TAG, "ble_connect_direct: Camera %d GATT interface not registered", camera_index);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "AUTOCONNECT_BOOT direct connect for slot %d (using boot scan result)", camera_index);
    ESP_LOGI(TAG, "  Target: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             profile->target_name,
             profile->target_mac[0], profile->target_mac[1], profile->target_mac[2],
             profile->target_mac[3], profile->target_mac[4], profile->target_mac[5]);
    
    // Copy target_mac to remote_bda for the connection
    memcpy(profile->remote_bda, profile->target_mac, sizeof(esp_bd_addr_t));
    
    // Set active camera index for event handlers
    s_active_scan_camera_index = camera_index;
    
    // Mark as connecting
    s_connecting = true;
    
    // Initiate direct GATT connection without scanning
    esp_err_t ret = esp_ble_gattc_open(profile->gattc_if, profile->target_mac, BLE_ADDR_TYPE_PUBLIC, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ble_connect_direct: Failed to open GATT connection for camera %d: %s", 
                 camera_index, esp_err_to_name(ret));
        s_connecting = false;
        s_active_scan_camera_index = -1;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "ble_connect_direct: GATT connection initiated for camera %d", camera_index);
    return ESP_OK;
}

/**
 * @brief Disconnect from a camera
 *
 * @param camera_index Camera slot index (0-2)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ble_disconnect(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_FAIL;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    if (!profile) {
        ESP_LOGE(TAG, "Failed to get profile for camera %d", camera_index);
        return ESP_FAIL;
    }
    
    if (profile->connection_status.is_connected && profile->gattc_if != ESP_GATT_IF_NONE) {
        ESP_LOGI(TAG, "Disconnecting camera %d", camera_index);
        esp_ble_gattc_close(profile->gattc_if, profile->conn_id);
        profile->connection_status.is_connected = false;
    } else {
        ESP_LOGW(TAG, "Camera %d is not connected", camera_index);
    }
    
    return ESP_OK;
}

/* -------------------------
 *  Read/Write and Notify related interfaces
 * ------------------------- */
/**
 * @brief Read a specified characteristic
 *
 * @param conn_id  Connection ID (obtained from callback events or internal management)
 * @param handle   Handle of the characteristic
 * @return esp_err_t
 */
esp_err_t ble_read(uint16_t conn_id, uint16_t handle) {
    ble_profile_t* profile = get_profile_by_conn_id(conn_id);
    if (!profile || !profile->connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected or invalid conn_id, skip read");
        return ESP_FAIL;
    }
    /* Initiate GATTC read request */
    esp_err_t ret = esp_ble_gattc_read_char(profile->gattc_if,
                                            conn_id,
                                            handle,
                                            ESP_GATT_AUTH_REQ_NONE);
    if (ret) {
        ESP_LOGE(TAG, "Camera %d: read_char failed: %s", 
                 profile->camera_index, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Write characteristic (Write Without Response)
 *
 * @param conn_id   Connection ID
 * @param handle    Handle of the characteristic
 * @param data      Data to be written
 * @param length    Length of the data
 * @return esp_err_t
 */
esp_err_t ble_write_without_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length) {
    ble_profile_t* profile = get_profile_by_conn_id(conn_id);
    if (!profile || !profile->connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected or invalid conn_id, skip write_without_response");
        return ESP_FAIL;
    }
    esp_err_t ret = esp_ble_gattc_write_char(profile->gattc_if,
                                             conn_id,
                                             handle,
                                             length,
                                             (uint8_t *)data,
                                             ESP_GATT_WRITE_TYPE_NO_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
    if (ret) {
        ESP_LOGE(TAG, "Camera %d: write_char NO_RSP failed: %s", 
                 profile->camera_index, esp_err_to_name(ret));
        
        /* Mark camera as disconnected on ESP_FAIL to stop further sends
         * This prevents repeated failed write attempts when camera is unreachable
         */
        if (ret == ESP_FAIL) {
            profile->connection_status.is_connected = false;
            ESP_LOGW(TAG, "Camera %d: Marking as disconnected due to BLE write failure", 
                     profile->camera_index);
        }
    }
    return ret;
}

/**
 * @brief Write characteristic (Write With Response)
 *
 * @param conn_id   Connection ID
 * @param handle    Handle of the characteristic
 * @param data      Data to be written
 * @param length    Length of the data
 * @return esp_err_t
 */
esp_err_t ble_write_with_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length) {
    ble_profile_t* profile = get_profile_by_conn_id(conn_id);
    if (!profile || !profile->connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected or invalid conn_id, skip write_with_response");
        return ESP_FAIL;
    }
    esp_err_t ret = esp_ble_gattc_write_char(profile->gattc_if,
                                             conn_id,
                                             handle,
                                             length,
                                             (uint8_t *)data,
                                             ESP_GATT_WRITE_TYPE_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
    if (ret) {
        ESP_LOGE(TAG, "Camera %d: write_char RSP failed: %s", 
                 profile->camera_index, esp_err_to_name(ret));
        
        /* Mark camera as disconnected on ESP_FAIL to stop further sends
         * This prevents repeated failed write attempts when camera is unreachable
         */
        if (ret == ESP_FAIL) {
            profile->connection_status.is_connected = false;
            ESP_LOGW(TAG, "Camera %d: Marking as disconnected due to BLE write failure", 
                     profile->camera_index);
        }
    }
    return ret;
}

/**
 * @brief Register (enable) Notify
 *
 * @param conn_id   Connection ID
 * @param char_handle Handle of the characteristic to enable notification
 * @return esp_err_t
 */
esp_err_t ble_register_notify(uint16_t conn_id, uint16_t char_handle) {
    ble_profile_t* profile = get_profile_by_conn_id(conn_id);
    if (!profile || !profile->connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected or invalid conn_id, skip register_notify");
        return ESP_FAIL;
    }
    /* Request to subscribe to notifications from the protocol stack */
    esp_err_t ret = esp_ble_gattc_register_for_notify(profile->gattc_if,
                                                      profile->remote_bda,
                                                      char_handle);
    if (ret) {
        ESP_LOGE(TAG, "Camera %d: register_notify failed: %s", 
                 profile->camera_index, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Unregister (disable) Notify
 *
 * @note  This is just a demonstration logic. You need the Client Config descriptor handle of the characteristic to operate.
 *        If needed in actual development, you can directly save the descr handle previously, and then close it by writing 0x0000 here.
 *
 * @param conn_id   Connection ID
 * @param char_handle Handle of the characteristic to disable notification
 * @return esp_err_t
 */
esp_err_t ble_unregister_notify(uint16_t conn_id, uint16_t char_handle) {
    /* In fact, you need to get the corresponding descriptor handle and then write 0x0000 to disable it */
    /* This is just a demonstration of the process. If needed, you can save the descr handle during register_notify */
    ESP_LOGI(TAG, "ble_unregister_notify called (demo), not fully implemented");
    return ESP_OK;
}

/**
 * @brief Set global Notify callback (for receiving data)
 *
 * @param cb Callback function pointer
 */
void ble_set_notify_callback(ble_notify_callback_t cb) {
    s_notify_cb = cb;
}

/**
 * @brief Set global logic layer disconnection state callback
 *
 * @param cb Callback function pointer
 */
void ble_set_state_callback(connect_logic_state_callback_t cb) {
    s_state_cb = cb;
}

/* ----------------------------------------------------------------
 *   GAP & GATTC callback function implementation (simplified version)
 * ---------------------------------------------------------------- */

/* Determine whether it is a DJI camera advertisement */
static uint8_t bsp_link_is_dji_camera_adv(esp_ble_gap_cb_param_t *scan_result) {
    const uint8_t *ble_adv = scan_result->scan_rst.ble_adv;
    const uint8_t adv_len = scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len;

    for (int i = 0; i < adv_len; ) {
        const uint8_t len = ble_adv[i];
        
        if (len == 0 || (i + len + 1) > adv_len) break;

        const uint8_t type = ble_adv[i+1];
        const uint8_t *data = &ble_adv[i+2];
        const uint8_t data_len = len - 1;

        if (type == ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE) {
            if (data_len >= 5) {
                if (data[0] == 0xAA && data[1] == 0x08 && data[4] == 0xFA) {
                    return 1;
                }
            }
        }
        i += (len + 1);
    }
    return 0;
}

/* Extract device_id from DJI camera advertisement manufacturer data */
static uint32_t bsp_link_get_dji_device_id(esp_ble_gap_cb_param_t *scan_result) {
    const uint8_t *ble_adv = scan_result->scan_rst.ble_adv;
    const uint8_t adv_len = scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len;

    for (int i = 0; i < adv_len; ) {
        const uint8_t len = ble_adv[i];
        
        if (len == 0 || (i + len + 1) > adv_len) break;

        const uint8_t type = ble_adv[i+1];
        const uint8_t *data = &ble_adv[i+2];
        const uint8_t data_len = len - 1;

        if (type == ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE) {
            // DJI manufacturer data format: [0xAA, 0x08, device_id_low, device_id_high, 0xFA, ...]
            // device_id is at bytes 2-3 (little endian uint16_t)
            if (data_len >= 5 && data[0] == 0xAA && data[1] == 0x08 && data[4] == 0xFA) {
                // Extract device_id (bytes 2-3, little endian)
                uint16_t device_id = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
                return (uint32_t)device_id;
            }
        }
        i += (len + 1);
    }
    return 0; // Unknown device
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
        trigger_scan_task();
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT received (mode was %d)", s_scan_controller.mode);
        
        // Clean up the scan timer to prevent stale callbacks
        cleanup_scan_timer();
        
        // Clear scan active flag
        s_ble_scan_active = false;
        ESP_LOGI(TAG, "Scan stopped, s_ble_scan_active = false (previous mode=%d)", s_scan_controller.mode);
        
        // Handle scan completion based on mode
        switch (s_scan_controller.mode) {
            case SCAN_MODE_PAIRING:
                // PAIRING mode: Scan stopped, user will select from discovered list
                // No automatic connection, UI handles selection
                ESP_LOGI(TAG, "Pairing scan completed, waiting for user selection");
                break;
                
            case SCAN_MODE_AUTOCONNECT_BOOT:
                // AUTOCONNECT_BOOT mode: Check which slots were found vs not found
                // Use connect_logic to check if slots were actually found
                extern bool connect_logic_is_slot_found(int slot_index);
                extern void connect_logic_mark_slot_not_found_during_boot(int slot_index);
                for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
                    if (s_scan_controller.autoconnect_pending[i]) {
                        // This slot was pending but check if it was actually found
                        if (!connect_logic_is_slot_found(i)) {
                            ESP_LOGW(TAG, "Autoconnect timeout: Camera slot %d not found", i);
                            // Mark as not found during boot - prevents background reconnection attempts
                            connect_logic_mark_slot_not_found_during_boot(i);
                            // Leave slot in "paired but not connected" state
                            // UI will show blue bluetooth icon
                        } else {
                            ESP_LOGI(TAG, "Autoconnect: Camera slot %d was found", i);
                        }
                    }
                }
                ESP_LOGI(TAG, "Autoconnect scan completed");
                break;
                
            case SCAN_MODE_SLOT_RECONNECT:
                // SLOT_RECONNECT mode: Check if target was found
                if (s_scan_controller.target_slot >= 0 && s_scan_controller.target_slot < BLE_MAX_CAMERAS) {
                    // If we're still here and target_slot is valid, it means we didn't find the camera
                    // (if we had found it, we would have stopped the scan and initiated connection)
                    // Check if a connection attempt is already in progress
                    if (!s_connecting) {
                        ESP_LOGW(TAG, "Slot reconnect timeout: Camera %d not found", s_scan_controller.target_slot);
                        // UI should handle displaying "Camera X not found" message
                    }
                }
                break;
                
            case SCAN_MODE_IDLE:
            default:
                ESP_LOGI(TAG, "Scan stopped in IDLE or unknown mode");
                break;
        }
        
        // Reset scan controller to IDLE
        s_scan_controller.mode = SCAN_MODE_IDLE;
        s_scan_controller.target_slot = -1;
        s_scan_controller.scan_timeout_ms = 0;
        s_scan_controller.scan_start_time = 0;
        for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
            s_scan_controller.autoconnect_pending[i] = false;
        }
        
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *r = param;
        if (r->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            // Check if it is a DJI camera advertisement
            if (!bsp_link_is_dji_camera_adv(r)) {
                break;
            }
            
            // Check scan timeout
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (s_scan_controller.mode != SCAN_MODE_IDLE && 
                s_scan_controller.scan_timeout_ms > 0 &&
                (current_time - s_scan_controller.scan_start_time) >= s_scan_controller.scan_timeout_ms) {
                ESP_LOGI(TAG, "Scan timeout reached (mode=%d), calling esp_ble_gap_stop_scanning", 
                         s_scan_controller.mode);
                esp_ble_gap_stop_scanning();
                break;
            }
            
            // Get the complete name from the advertisement data
            uint8_t *adv_name = NULL;
            uint8_t adv_name_len = 0;
            adv_name = esp_ble_resolve_adv_data_by_type(r->scan_rst.ble_adv,
                                r->scan_rst.adv_data_len + r->scan_rst.scan_rsp_len,
                                ESP_BLE_AD_TYPE_NAME_CMPL,
                                &adv_name_len);

            // Prepare a safe string pointer for logging
            const char *adv_name_str = NULL;
            if (adv_name && adv_name_len > 0) {
                static char name_buf[64];
                size_t copy_len = adv_name_len < sizeof(name_buf) - 1 ? adv_name_len : sizeof(name_buf) - 1;
                memcpy(name_buf, adv_name, copy_len);
                name_buf[copy_len] = '\0';
                adv_name_str = name_buf;
            } else {
                adv_name_str = "NULL";
            }

            // Extract device_id from manufacturer data
            uint32_t device_id = bsp_link_get_dji_device_id(r);
            
            ESP_LOGI(TAG, "Found device: %s with RSSI: %d, MAC: %02X:%02X:%02X:%02X:%02X:%02X, device_id: 0x%04X (scan_mode=%d)",
                adv_name_str,
                r->scan_rst.rssi,
                r->scan_rst.bda[0], r->scan_rst.bda[1], r->scan_rst.bda[2],
                r->scan_rst.bda[3], r->scan_rst.bda[4], r->scan_rst.bda[5],
                (unsigned int)device_id, s_scan_controller.mode);

            // Process based on scan mode
            switch (s_scan_controller.mode) {
                case SCAN_MODE_PAIRING: {
                    // PAIRING mode: Add all discovered cameras to list (no RSSI filtering)
                    // Notify pairing screen to add this camera to the list
                    extern void ui_pairing_add_discovered_camera(const char *name, const uint8_t *mac, int8_t rssi, uint32_t device_id);
                    ui_pairing_add_discovered_camera(adv_name_str, r->scan_rst.bda, r->scan_rst.rssi, device_id);
                    break;
                }
                
                case SCAN_MODE_AUTOCONNECT_BOOT: {
                    // AUTOCONNECT_BOOT mode: Match against all pending slots
                    // Mark slots as found instead of immediately connecting
                    // Connections will be initiated after scan completes
                    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
                        if (!s_scan_controller.autoconnect_pending[i]) {
                            continue;  // Skip non-pending slots
                        }
                        
                        ble_profile_t* profile = get_profile_by_camera_index(i);
                        if (!profile) continue;
                        
                        // Check if this device matches the paired MAC for this slot
                        if (memcmp(profile->target_mac, r->scan_rst.bda, 6) == 0) {
                            ESP_LOGI(TAG, "AUTOCONNECT_BOOT: Found paired camera for slot %d: %s", i, adv_name_str);
                            
                            // Mark slot as found in connect_logic (don't connect yet)
                            extern void connect_logic_mark_slot_found(int slot_index);
                            connect_logic_mark_slot_found(i);
                            
                            // Clear pending flag for this slot
                            s_scan_controller.autoconnect_pending[i] = false;
                            
                            // Check if all pending slots are resolved
                            bool all_resolved = true;
                            for (int j = 0; j < BLE_MAX_CAMERAS; j++) {
                                if (s_scan_controller.autoconnect_pending[j]) {
                                    all_resolved = false;
                                    break;
                                }
                            }
                            
                            // Stop scanning early if all pending slots are found
                            if (all_resolved) {
                                ESP_LOGI(TAG, "All autoconnect slots found, stopping scan early");
                                esp_ble_gap_stop_scanning();
                            }
                            
                            break;  // Found match for this slot, continue scanning for others
                        }
                    }
                    break;
                }
                
                case SCAN_MODE_SLOT_RECONNECT: {
                    // SLOT_RECONNECT mode: Match only against specific target slot
                    if (s_scan_controller.target_slot < 0 || s_scan_controller.target_slot >= BLE_MAX_CAMERAS) {
                        ESP_LOGW(TAG, "Invalid target_slot in SLOT_RECONNECT mode");
                        break;
                    }
                    
                    ble_profile_t* profile = get_profile_by_camera_index(s_scan_controller.target_slot);
                    if (!profile) break;
                    
                    // Check if this device matches the target slot's paired MAC
                    if (memcmp(profile->target_mac, r->scan_rst.bda, 6) == 0) {
                        ESP_LOGI(TAG, "SLOT_RECONNECT: Found target camera for slot %d: %s", 
                                 s_scan_controller.target_slot, adv_name_str);
                        
                        // Set active scan index for connection
                        s_active_scan_camera_index = s_scan_controller.target_slot;
                        
                        // Initiate connection immediately
                        try_to_connect(s_scan_controller.target_slot, r->scan_rst.bda);
                        
                        // Stop scanning immediately (found our target)
                        ESP_LOGI(TAG, "Target camera found (slot=%d), calling esp_ble_gap_stop_scanning", 
                                 s_scan_controller.target_slot);
                        esp_ble_gap_stop_scanning();
                    }
                    break;
                }
                
                case SCAN_MODE_IDLE:
                default:
                    // Should not receive scan results when idle
                    ESP_LOGW(TAG, "Received scan result in IDLE mode, ignoring");
                    break;
            }
        }
        break;
    }

    default:
        break;
    }
}

static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    ble_profile_t* profile = NULL;
    
    switch (event) {
    case ESP_GATTC_REG_EVT: {
        // Handle GATT client registration event
        // All camera profiles share the same gattc_if
        if (param->reg.status == ESP_GATT_OK) {
            for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
                s_ble_profiles[i].gattc_if = gattc_if;
            }
            ESP_LOGI(TAG, "GATTC register OK, app_id=%d, gattc_if=%d (shared by %d cameras)",
                     param->reg.app_id, gattc_if, BLE_MAX_CAMERAS);
        } else {
            ESP_LOGE(TAG, "GATTC register failed, status=%d", param->reg.status);
        }
        break;
    }
    case ESP_GATTC_CONNECT_EVT: {
        // Handle connection event
        // Match the connection to the correct camera slot by MAC address
        // This is critical for autoconnect where multiple connections happen simultaneously
        profile = NULL;
        for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
            if (s_ble_profiles[i].camera_index >= 0 &&
                memcmp(s_ble_profiles[i].target_mac, param->connect.remote_bda, 6) == 0) {
                profile = &s_ble_profiles[i];
                ESP_LOGI(TAG, "CONNECT_EVT: Matched to camera slot %d by MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                         i, param->connect.remote_bda[0], param->connect.remote_bda[1],
                         param->connect.remote_bda[2], param->connect.remote_bda[3],
                         param->connect.remote_bda[4], param->connect.remote_bda[5]);
                break;
            }
        }
        
        if (!profile) {
            ESP_LOGW(TAG, "Connection event: No matching profile found for MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     param->connect.remote_bda[0], param->connect.remote_bda[1],
                     param->connect.remote_bda[2], param->connect.remote_bda[3],
                     param->connect.remote_bda[4], param->connect.remote_bda[5]);
            break;
        }
        
        profile->conn_id = param->connect.conn_id;
        profile->connection_status.is_connected = true;
        memcpy(profile->remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        
        // Ensure camera_index is set correctly (should already be set by ble_set_target_device)
        // But verify it matches the slot we matched to
        int matched_slot = profile->camera_index;
        if (matched_slot < 0 || matched_slot >= BLE_MAX_CAMERAS) {
            ESP_LOGW(TAG, "CONNECT_EVT: Profile camera_index invalid (%d), setting from matched slot", matched_slot);
            // Find which slot this profile belongs to
            for (int j = 0; j < BLE_MAX_CAMERAS; j++) {
                if (&s_ble_profiles[j] == profile) {
                    profile->camera_index = j;
                    matched_slot = j;
                    break;
                }
            }
        }
        
        ESP_LOGI(TAG, "Camera %d connected successfully! conn_id=%d, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 matched_slot, profile->conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1],
                 param->connect.remote_bda[2], param->connect.remote_bda[3],
                 param->connect.remote_bda[4], param->connect.remote_bda[5]);

        // Initiate MTU request
        esp_ble_gattc_send_mtu_req(gattc_if, param->connect.conn_id);
        break;
    }
    case ESP_GATTC_OPEN_EVT: {
        // Handle connection open event
        s_connecting = false;
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Open failed for camera %d, status=%d", 
                     s_active_scan_camera_index, param->open.status);
            s_active_scan_camera_index = -1;  // Reset on failure
            break;
        }
        ESP_LOGI(TAG, "Open success for camera %d, MTU=%u", 
                 s_active_scan_camera_index, param->open.mtu);
        // Keep s_active_scan_camera_index valid for subsequent events
        break;
    }
    case ESP_GATTC_CFG_MTU_EVT: {
        // Handle MTU configuration event
        profile = get_profile_by_conn_id(param->cfg_mtu.conn_id);
        if (!profile) {
            ESP_LOGW(TAG, "MTU event for unknown conn_id=%d", param->cfg_mtu.conn_id);
            break;
        }
        
        if (param->cfg_mtu.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Camera %d: Config MTU Error, status=%d", 
                     profile->camera_index, param->cfg_mtu.status);
        }
        ESP_LOGI(TAG, "Camera %d: MTU=%d", profile->camera_index, param->cfg_mtu.mtu);

        // Start service discovery after MTU configuration
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        break;
    }
    case ESP_GATTC_SEARCH_RES_EVT: {
        // Handle service search result event
        profile = get_profile_by_conn_id(param->search_res.conn_id);
        if (!profile) {
            ESP_LOGW(TAG, "Search result for unknown conn_id=%d", param->search_res.conn_id);
            break;
        }
        
        if ((param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) &&
            (param->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_TARGET_SERVICE_UUID)) {
            profile->service_start_handle = param->search_res.start_handle;
            profile->service_end_handle   = param->search_res.end_handle;
            ESP_LOGI(TAG, "Camera %d: Service found: start=%d, end=%d",
                     profile->camera_index,
                     profile->service_start_handle,
                     profile->service_end_handle);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
        // Handle service search complete event
        profile = get_profile_by_conn_id(param->search_cmpl.conn_id);
        if (!profile) {
            ESP_LOGW(TAG, "Search complete for unknown conn_id=%d", param->search_cmpl.conn_id);
            break;
        }
        
        if (param->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Camera %d: Service search failed, status=%d", 
                     profile->camera_index, param->search_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "Camera %d: Service search complete, getting characteristics", 
                 profile->camera_index);

        // Get notify characteristic handle
        uint16_t count = 1;
        esp_gattc_char_elem_t char_elem_result;
        esp_ble_gattc_get_char_by_uuid(gattc_if,
                                       profile->conn_id,
                                       profile->service_start_handle,
                                       profile->service_end_handle,
                                       s_filter_notify_char_uuid,
                                       &char_elem_result,
                                       &count);
        if (count > 0) {
            profile->notify_char_handle = char_elem_result.char_handle;
            profile->handle_discovery.notify_char_handle_found = true;
            ESP_LOGI(TAG, "Camera %d: Notify Char found, handle=0x%x",
                     profile->camera_index, profile->notify_char_handle);
        }

        // Get write characteristic handle
        count = 1;
        esp_gattc_char_elem_t write_char_elem_result;
        esp_ble_gattc_get_char_by_uuid(gattc_if,
                                       profile->conn_id,
                                       profile->service_start_handle,
                                       profile->service_end_handle,
                                       s_filter_write_char_uuid,
                                       &write_char_elem_result,
                                       &count);
        if (count > 0) {
            profile->write_char_handle = write_char_elem_result.char_handle;
            profile->handle_discovery.write_char_handle_found = true;
            ESP_LOGI(TAG, "Camera %d: Write Char found, handle=0x%x",
                     profile->camera_index, profile->write_char_handle);
        }

        // Connection setup complete, reset active scan camera index
        s_active_scan_camera_index = -1;
        break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        // Handle notification registration event
        profile = get_profile_by_conn_id(param->reg_for_notify.handle);  // Note: Using handle as identifier here
        if (!profile) {
            // Try to find by any connected profile (fallback)
            for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
                if (s_ble_profiles[i].camera_index >= 0 && 
                    s_ble_profiles[i].connection_status.is_connected) {
                    profile = &s_ble_profiles[i];
                    break;
                }
            }
        }
        
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Notify register failed, status=%d", param->reg_for_notify.status);
            break;
        }
        ESP_LOGI(TAG, "Camera %d: Notify register success, handle=0x%x",
                 profile ? profile->camera_index : -1, param->reg_for_notify.handle);

        if (profile) {
            // Find descriptor and write 0x01 to enable notification
            uint16_t count = 1;
            esp_gattc_descr_elem_t descr_elem;
            esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                   profile->conn_id,
                                                   param->reg_for_notify.handle,
                                                   s_notify_descr_uuid,
                                                   &descr_elem,
                                                   &count);
            if (count > 0 && descr_elem.handle) {
                uint16_t notify_en = 1;
                esp_ble_gattc_write_char_descr(gattc_if,
                                               profile->conn_id,
                                               descr_elem.handle,
                                               sizeof(notify_en),
                                               (uint8_t *)&notify_en,
                                               ESP_GATT_WRITE_TYPE_RSP,
                                               ESP_GATT_AUTH_REQ_NONE);
            }
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
        // Handle notification data event
        // Find which camera this notification is from
        profile = get_profile_by_conn_id(param->notify.conn_id);
        
        // Call notification callback with data including camera_index
        if (s_notify_cb && profile) {
            s_notify_cb(profile->camera_index, param->notify.value, param->notify.value_len);
        } else if (s_notify_cb && !profile) {
            ESP_LOGW(TAG, "Notification received but no profile found for conn_id=%d", param->notify.conn_id);
        }
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
        // Handle disconnection event
        profile = get_profile_by_conn_id(param->disconnect.conn_id);
        if (profile) {
            int camera_idx = profile->camera_index;
            profile->connection_status.is_connected = false;
            profile->conn_id = 0;  // Clear conn_id to prevent stale matches
            profile->handle_discovery.write_char_handle_found = false;
            profile->handle_discovery.notify_char_handle_found = false;
            profile->notify_char_handle = 0;
            profile->write_char_handle = 0;
            ESP_LOGI(TAG, "Camera %d disconnected, reason=0x%x", 
                     camera_idx, param->disconnect.reason);
            
            // Reset active scan camera index if this was the camera we were connecting to
            if (s_active_scan_camera_index == camera_idx) {
                s_active_scan_camera_index = -1;
            }
        } else {
            ESP_LOGW(TAG, "Disconnect event for unknown conn_id=%d, reason=0x%x",
                     param->disconnect.conn_id, param->disconnect.reason);
        }
        
        s_connecting = false;

        if (s_state_cb) {
            s_state_cb();
        }
        break;
    }
    default:
        break;
    }
}

// BLE Advertising Data Format
static uint8_t adv_data[] = {
    10, 0xff, 'W','K','P','1','2','3','4','5','6'
};

static esp_timer_handle_t adv_timer;

/**
 * @brief Stop advertising early (before 2-second timeout)
 * 
 * This allows stopping the wake broadcast if camera wakes up early.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t ble_stop_advertising_early(void) {
    if (adv_timer == NULL) {
        ESP_LOGW(TAG, "Advertising timer not active, cannot stop early");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Stop the timer
    esp_err_t ret = esp_timer_stop(adv_timer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop advertising timer: %s", esp_err_to_name(ret));
    }
    
    // Stop advertising
    ret = esp_ble_gap_stop_advertising();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Failed to stop advertising: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Advertising stopped early");
    }
    
    return ESP_OK;
}

static void stop_adv_after_3s(void* arg) {
    esp_ble_gap_stop_advertising();
    ESP_LOGI(TAG, "Wake broadcast advertising stopped after 3 seconds");
    
    // NOTE: This callback ONLY stops the BLE advertising.
    // Timeout handling and snapshot_pending flag clearing is done by:
    // - ui_process_wake_queue() for "All Cameras" mode (queue processing)
    // - ui_process_single_camera_wake_timeout() for single camera mode
    // This separation of concerns prevents race conditions and ensures
    // consistent flag management across all wake scenarios.
}

esp_err_t ble_start_advertising() {
    // Use camera 0's remote BDA for advertising
    ble_profile_t* profile = get_profile_by_camera_index(0);
    if (!profile) {
        ESP_LOGE(TAG, "Error: Camera 0 profile not available!");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if remote_bda is initialized
    if (memcmp(profile->remote_bda, "\x00\x00\x00\x00\x00\x00", 6) == 0) {
        ESP_LOGE(TAG, "Error: Camera 0 remote_bda not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < 6; i++) {
        adv_data[5 + i] = profile->remote_bda[5 - i];
    }

    ESP_LOGI(TAG, "Modified Advertising Data (with MAC):");
    ESP_LOG_BUFFER_HEX(TAG, adv_data, sizeof(adv_data));

    esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .channel_map = ADV_CHNL_ALL,
    };

    esp_err_t ret = esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set adv data.");
        return ret;
    }

    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
        return ret;
    }

    if (adv_timer == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = &stop_adv_after_3s,
            .name = "adv_timer"
        };
        esp_timer_create(&timer_args, &adv_timer);
    }
    
    esp_timer_start_once(adv_timer, 3000000);  // 3000ms = 3,000,000us

    ESP_LOGI(TAG, "Advertising started (will auto-stop after 3s)");
    return ESP_OK;
}

// Functions to get connected camera information
const char* ble_get_connected_device_name(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        return NULL;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    if (profile && profile->connection_status.is_connected) {
        return profile->target_name;
    }
    return NULL;
}

const uint8_t* ble_get_connected_device_mac(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        return NULL;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    if (profile && profile->connection_status.is_connected) {
        return (const uint8_t*)profile->remote_bda;
    }
    return NULL;
}

bool ble_get_connected_device_info(int camera_index, char* name, size_t name_size, uint8_t* mac) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS || !name || !mac) {
        return false;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    if (!profile || !profile->connection_status.is_connected) {
        return false;
    }
    
    // Copy device name
    strncpy(name, profile->target_name, name_size - 1);
    name[name_size - 1] = '\0';
    
    // Copy MAC address
    memcpy(mac, profile->remote_bda, 6);
    
    return true;
}

uint16_t ble_get_conn_id(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        return 0;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    return profile ? profile->conn_id : 0;
}

uint16_t ble_get_write_handle(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        return 0;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    return profile ? profile->write_char_handle : 0;
}

uint16_t ble_get_notify_handle(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        return 0;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    return profile ? profile->notify_char_handle : 0;
}

bool ble_is_camera_connected(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        return false;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    return profile ? profile->connection_status.is_connected : false;
}

// Function to wake up camera using BLE advertising broadcast
esp_err_t ble_wake_camera(const uint8_t* camera_mac) {
    if (!camera_mac) {
        ESP_LOGE(TAG, "Camera MAC address is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Starting wake broadcast for camera: %02X:%02X:%02X:%02X:%02X:%02X",
             camera_mac[0], camera_mac[1], camera_mac[2], 
             camera_mac[3], camera_mac[4], camera_mac[5]);
    
    // Prepare wake-up advertising data
    // Format: [10, 0xff, 'W','K','P', MAC[5], MAC[4], MAC[3], MAC[2], MAC[1], MAC[0]]
    static uint8_t wake_adv_data[] = {
        10, 0xff, 'W','K','P','1','2','3','4','5','6'
    };
    
    // Fill in camera MAC address in reverse order (as per documentation)
    for (int i = 0; i < 6; i++) {
        wake_adv_data[5 + i] = camera_mac[5 - i];
    }
    
    ESP_LOGI(TAG, "Wake broadcast data: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             wake_adv_data[0], wake_adv_data[1], wake_adv_data[2], wake_adv_data[3],
             wake_adv_data[4], wake_adv_data[5], wake_adv_data[6], wake_adv_data[7],
             wake_adv_data[8], wake_adv_data[9], wake_adv_data[10]);
    
    // Set advertising data
    esp_err_t ret = esp_ble_gap_config_adv_data_raw(wake_adv_data, sizeof(wake_adv_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure wake advertising data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure advertising parameters
    esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .channel_map = ADV_CHNL_ALL,
    };
    
    // Start advertising
    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start wake advertising: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create timer to stop advertising after 3 seconds (increased from 2s for better reliability)
    if (adv_timer == NULL) {
        esp_timer_create_args_t timer_args = {
            .callback = &stop_adv_after_3s,
            .name = "wake_adv_timer"
        };
        esp_timer_create(&timer_args, &adv_timer);
    }
    
    esp_timer_start_once(adv_timer, 3000000);  // 3000ms = 3,000,000us
    
    ESP_LOGI(TAG, "Wake advertising started (will auto-stop after 3s)");
    return ESP_OK;
}

// Function to set target device for reconnection
void ble_set_target_device(int camera_index, const char* name, const uint8_t* mac) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return;
    }
    
    if (!name || !mac) {
        ESP_LOGE(TAG, "Invalid name or MAC address");
        return;
    }
    
    ble_profile_t* profile = get_profile_by_camera_index(camera_index);
    if (!profile) {
        ESP_LOGE(TAG, "Failed to get profile for camera %d", camera_index);
        return;
    }
    
    // Set the target device name and MAC in the profile
    strncpy(profile->target_name, name, sizeof(profile->target_name) - 1);
    profile->target_name[sizeof(profile->target_name) - 1] = '\0';
    memcpy(profile->target_mac, mac, 6);
    profile->camera_index = camera_index;  // Mark this profile as active for this camera
    
    // Clear any stale connection state for this profile
    profile->connection_status.is_connected = false;
    profile->conn_id = 0;
    profile->notify_char_handle = 0;
    profile->write_char_handle = 0;
    profile->handle_discovery.notify_char_handle_found = false;
    profile->handle_discovery.write_char_handle_found = false;
    
    ESP_LOGI(TAG, "Camera %d target device set: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             camera_index, name,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

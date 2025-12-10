#ifndef __BLE_H__
#define __BLE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "esp_err.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"

/* Connection status structure */
typedef struct {
    bool is_connected; // Connection status
} connection_status_t;

/* Handle discovery status structure */
typedef struct {
    bool notify_char_handle_found; // Notify characteristic handle found
    bool write_char_handle_found;  // Write characteristic handle found
} handle_discovery_t;

/* Maximum number of simultaneous camera connections */
#define BLE_MAX_CAMERAS 3

/* Scan modes for different connection scenarios */
typedef enum {
    SCAN_MODE_IDLE = 0,                    /* No active scan */
    SCAN_MODE_PAIRING,                     /* User-initiated pairing for specific slot */
    SCAN_MODE_AUTOCONNECT_BOOT,            /* Boot-time autoconnect for paired slots */
    SCAN_MODE_SLOT_RECONNECT               /* Manual reconnect for specific slot */
} scan_mode_t;

/* Scan controller state */
typedef struct {
    scan_mode_t mode;                      /* Current scan mode */
    int target_slot;                       /* Target camera slot (-1 if not applicable) */
    bool autoconnect_pending[BLE_MAX_CAMERAS]; /* Pending autoconnect flags per slot */
    uint32_t scan_timeout_ms;              /* Scan timeout in milliseconds */
    uint32_t scan_start_time;              /* Scan start timestamp */
} scan_controller_t;

/* Global profile structure to manage connection and characteristic information */
typedef struct {
    uint16_t conn_id;              // Connection ID
    esp_gatt_if_t gattc_if;        // GATT client interface
    int camera_index;              // Camera slot index (0, 1, 2) or -1 if unused

    /* Handles for characteristics we need to operate on */
    uint16_t notify_char_handle;   // Notify characteristic handle
    uint16_t write_char_handle;    // Write characteristic handle
    uint16_t read_char_handle;     // Read characteristic handle

    /* Start and end handles of the service */
    uint16_t service_start_handle; // Service start handle
    uint16_t service_end_handle;   // Service end handle

    /* Remote device address */
    esp_bd_addr_t remote_bda;      // Remote Bluetooth device address
    
    /* Target device info for connection */
    char target_name[ESP_BLE_ADV_NAME_LEN_MAX];  // Target camera name
    uint8_t target_mac[6];                        // Target camera MAC

    connection_status_t connection_status;     // Connection status
    handle_discovery_t handle_discovery;       // Handle discovery status
} ble_profile_t;

/* Array of BLE profiles for multi-camera support */
extern ble_profile_t s_ble_profiles[BLE_MAX_CAMERAS];

/**
 * @brief Notify callback function type for receiving data from remote
 *
 * @param data   Pointer to the notification data
 * @param length Length of the notification data
 */
typedef void (*ble_notify_callback_t)(int camera_index, const uint8_t *data, size_t length);

typedef void (*connect_logic_state_callback_t)(void);

esp_err_t ble_init();

// Multi-camera connection functions
esp_err_t ble_start_scanning_and_connect(int camera_index);
esp_err_t ble_reconnect(int camera_index);
esp_err_t ble_disconnect(int camera_index);

/**
 * @brief Connect directly to a camera using stored MAC address (no scanning)
 * 
 * Used after boot scan to connect to cameras that were already discovered.
 * This avoids starting a new scan when we already know the device's MAC address.
 * 
 * @param camera_index Camera slot index (0-2)
 * @return esp_err_t ESP_OK on connection initiated, ESP_FAIL on failure
 */
esp_err_t ble_connect_direct(int camera_index);

// Legacy single-camera functions (use camera 0)
void ble_set_reconnecting(bool flag);
bool ble_get_reconnecting(void);

// Read/write operations (use conn_id from profile)
esp_err_t ble_read(uint16_t conn_id, uint16_t handle);
esp_err_t ble_write_without_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length);
esp_err_t ble_write_with_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length);

// Notification management
esp_err_t ble_register_notify(uint16_t conn_id, uint16_t char_handle);
esp_err_t ble_unregister_notify(uint16_t conn_id, uint16_t char_handle);
void ble_set_notify_callback(ble_notify_callback_t cb);
void ble_set_state_callback(connect_logic_state_callback_t cb);

// Advertising for wake-up
esp_err_t ble_start_advertising(void);
esp_err_t ble_wake_camera(const uint8_t* camera_mac);
esp_err_t ble_stop_advertising_early(void);

// Multi-camera info functions
const char* ble_get_connected_device_name(int camera_index);
const uint8_t* ble_get_connected_device_mac(int camera_index);
bool ble_get_connected_device_info(int camera_index, char* name, size_t name_size, uint8_t* mac);
uint16_t ble_get_conn_id(int camera_index);
uint16_t ble_get_write_handle(int camera_index);
uint16_t ble_get_notify_handle(int camera_index);
bool ble_is_camera_connected(int camera_index);

// Target device configuration
void ble_set_target_device(int camera_index, const char* name, const uint8_t* mac);

// Scan mode control
/* Start scan with specific mode */
esp_err_t ble_start_scan(scan_mode_t mode, int target_slot, uint32_t timeout_ms);

/* Stop active scan */
esp_err_t ble_stop_scan(void);

/* Get current scan mode */
scan_mode_t ble_get_scan_mode(void);

/* Set autoconnect pending flags for AUTOCONNECT_BOOT mode (for future parallel scan support) */
void ble_set_autoconnect_pending(bool pending[BLE_MAX_CAMERAS]);

/* Check if BLE scan is currently active */
bool ble_is_scanning(void);

#endif
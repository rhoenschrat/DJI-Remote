#ifndef __BLE_H__
#define __BLE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "esp_err.h"

/* Connection status structure */
typedef struct {
    bool is_connected;
} connection_status_t;

/* Handle discovery status structure */
typedef struct {
    bool notify_char_handle_found;
    bool write_char_handle_found;
} handle_discovery_t;

/* Maximum number of simultaneous camera connections */
#define BLE_MAX_CAMERAS 3

/* Maximum device name length (replaces Bluedroid's ESP_BLE_ADV_NAME_LEN_MAX) */
#define BLE_DEVICE_NAME_MAX_LEN 32

/* Scan modes for different connection scenarios */
typedef enum {
    SCAN_MODE_IDLE = 0,
    SCAN_MODE_PAIRING,
    SCAN_MODE_AUTOCONNECT_BOOT,
    SCAN_MODE_SLOT_RECONNECT
} scan_mode_t;

/* Scan controller state */
typedef struct {
    scan_mode_t mode;
    int target_slot;
    bool autoconnect_pending[BLE_MAX_CAMERAS];
    uint32_t scan_timeout_ms;
    uint32_t scan_start_time;
} scan_controller_t;

/*
 * Per-camera BLE profile.
 *
 * conn_id is the NimBLE conn_handle (uint16_t). The field name is kept as
 * conn_id so that callers using ble_get_conn_id() need zero changes.
 */
typedef struct {
    uint16_t conn_id;
    int camera_index;

    uint16_t notify_char_handle;
    uint16_t write_char_handle;
    uint16_t read_char_handle;
    uint16_t cccd_handle;

    uint16_t service_start_handle;
    uint16_t service_end_handle;

    uint8_t remote_bda[6];

    char target_name[BLE_DEVICE_NAME_MAX_LEN];
    uint8_t target_mac[6];

    connection_status_t connection_status;
    handle_discovery_t handle_discovery;

    /* esp_timer time (us) of the last notification; 0 = none since connect */
    int64_t last_rx_us;

    /* Applied supervision timeout of the current link in 10 ms units;
     * 0 = no link (ADR-021). Written on the host task only. */
    uint16_t supervision_timeout;

    /* One supervision-timeout correction per link (ADR-021); reset on connect */
    bool sto_corrected;
} ble_profile_t;

/* Array of BLE profiles for multi-camera support */
extern ble_profile_t s_ble_profiles[BLE_MAX_CAMERAS];

typedef void (*ble_notify_callback_t)(int camera_index, const uint8_t *data, size_t length);

/* Link event: an established camera link was closed (by the peer, by
 * supervision timeout, or by a local ble_disconnect()). */
typedef struct {
    int      slot;        /* 0..BLE_MAX_CAMERAS-1 */
    int      reason;      /* NimBLE disconnect reason (BLE_HS_ERR_HCI_BASE + HCI code) */
    uint32_t silent_ms;   /* ms since last notification; UINT32_MAX if none since connect */
} ble_link_event_t;

/* Called on the NimBLE host task. MUST NOT block, wait, poll, or call
 * connect logic — enqueue and return. */
typedef void (*ble_link_event_callback_t)(const ble_link_event_t *evt);

esp_err_t ble_init(void);

esp_err_t ble_start_scanning_and_connect(int camera_index);
esp_err_t ble_reconnect(int camera_index);
esp_err_t ble_disconnect(int camera_index);
esp_err_t ble_connect_direct(int camera_index);

void ble_set_reconnecting(bool flag);
bool ble_get_reconnecting(void);

esp_err_t ble_read(uint16_t conn_id, uint16_t handle);
esp_err_t ble_write_without_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length);
esp_err_t ble_write_with_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length);

esp_err_t ble_register_notify(uint16_t conn_id, uint16_t char_handle);
esp_err_t ble_unregister_notify(uint16_t conn_id, uint16_t char_handle);
void ble_set_notify_callback(ble_notify_callback_t cb);
void ble_set_link_event_callback(ble_link_event_callback_t cb);

esp_err_t ble_start_advertising(void);
esp_err_t ble_wake_camera(const uint8_t* camera_mac);
esp_err_t ble_stop_advertising_early(void);

const char* ble_get_connected_device_name(int camera_index);
const uint8_t* ble_get_connected_device_mac(int camera_index);
bool ble_get_connected_device_info(int camera_index, char* name, size_t name_size, uint8_t* mac);
uint16_t ble_get_conn_id(int camera_index);
uint16_t ble_get_write_handle(int camera_index);
uint16_t ble_get_notify_handle(int camera_index);
uint16_t ble_get_cccd_handle(int camera_index);
bool ble_is_camera_connected(int camera_index);
/* True once the slot has no connection handle any more, i.e. the
 * BLE_GAP_EVENT_DISCONNECT of the last link was processed. ble_disconnect()
 * clears is_connected immediately; the handle is only released by the event. */
bool ble_is_link_released(int camera_index);

void ble_set_target_device(int camera_index, const char* name, const uint8_t* mac);

/* Minimum supervision timeout in 10 ms units (5 s), used for our own
 * connect and as floor for camera-requested updates (ADR-021) */
#define BLE_SUPERVISION_TIMEOUT_FLOOR 500

/* Per-slot link diagnostics (liveness estimate, BLE_DIAG line) */
typedef struct {
    bool     link_up;           /* ble_is_camera_connected() */
    uint32_t rx_age_ms;         /* UINT32_MAX if nothing received on this link */
    uint32_t supervision_ms;    /* applied supervision timeout in ms, 0 if no link */
    uint32_t gatt_failures;     /* failed GATT writes, cumulative since boot */
    int      last_disc_reason;  /* last disconnect reason, -1 if never disconnected */
} ble_slot_diag_t;

/**
 * @brief Snapshot of a slot's link diagnostics (any task)
 *
 * Read without locking. A torn read of the 64-bit RX timestamp on a 32-bit
 * CPU is possible and acceptable for diagnostics and the release-wait
 * estimate.
 */
esp_err_t ble_get_slot_diag(int camera_index, ble_slot_diag_t *out);

esp_err_t ble_start_scan(scan_mode_t mode, int target_slot, uint32_t timeout_ms);
esp_err_t ble_stop_scan(void);
scan_mode_t ble_get_scan_mode(void);
void ble_set_autoconnect_pending(bool pending[BLE_MAX_CAMERAS]);
bool ble_is_scanning(void);

#endif

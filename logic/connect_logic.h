#ifndef __CONNECT_LOGIC_H__
#define __CONNECT_LOGIC_H__

#include "ble.h"  // For scan_mode_t

typedef enum {
    BLE_NOT_INIT = -1,
    BLE_INIT_COMPLETE = 0,
    BLE_SEARCHING = 1,
    BLE_CONNECTED = 2,
    PROTOCOL_CONNECTED = 3,
    BLE_DISCONNECTING = 4,   // Actively disconnecting state
} connect_state_t;

connect_state_t connect_logic_get_state(void);

int connect_logic_ble_init();

int connect_logic_ble_connect(int camera_index, scan_mode_t scan_mode);

/**
 * @brief Connect directly to a camera without scanning (for boot scan results)
 * 
 * Used after boot scan completes to connect to cameras that were already
 * discovered during the scan. This avoids starting a new per-camera scan.
 * 
 * @param camera_index Camera slot index (0-2)
 * @return int 0 on success, -1 on failure
 */
int connect_logic_ble_connect_direct(int camera_index);

int connect_logic_ble_disconnect(void);

int connect_logic_protocol_connect(int camera_index, uint32_t device_id, uint8_t mac_addr_len, const int8_t *mac_addr,
                                    uint32_t fw_version, uint8_t verify_mode, uint16_t verify_data,
                                    uint8_t camera_reserved);

int connect_logic_ble_wakeup(void);
esp_err_t connect_logic_start_wake_broadcast_for_slot(int camera_index);

/* Boot scan functions for single unified boot scan */
int connect_logic_start_boot_scan(void);
int connect_logic_update_boot_scan(void);
bool connect_logic_is_boot_scan_active(void);
bool connect_logic_is_boot_connect_in_progress(void);
void connect_logic_clear_boot_connect_flag(void);
bool connect_logic_is_slot_found(int slot_index);
void connect_logic_mark_slot_found(int slot_index);
bool connect_logic_was_slot_not_found_during_boot(int slot_index);
void connect_logic_clear_slot_not_found_flag(int slot_index);
void connect_logic_mark_slot_not_found_during_boot(int slot_index);
void connect_logic_stop_boot_scan_and_connect_found(void);
bool connect_logic_slot_is_connecting(int slot_index);
void connect_logic_set_slot_connecting(int slot_index, bool is_connecting);

#endif
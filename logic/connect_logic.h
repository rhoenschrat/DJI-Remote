#ifndef __CONNECT_LOGIC_H__
#define __CONNECT_LOGIC_H__

#include "ble.h"  // For scan_mode_t

/* True once the BLE stack and the link-event task are initialized. */
bool connect_logic_is_ble_initialized(void);

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

/**
 * @brief App-initiated disconnect of one slot
 *
 * Every app-initiated disconnect must use this function so the resulting
 * link event is treated as expected (no link-loss reset for that slot).
 */
int connect_logic_ble_disconnect(int camera_index);

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

/* ---- Per-slot link state (ADR-018) ---- */

/* True while the DJI handshake has completed on the slot's current link. */
bool connect_logic_slot_is_protocol_connected(int slot);

/**
 * @brief Reset a slot to "paired but not connected"
 *
 * Single source of truth for link loss and every connect-failure path.
 * Clears connection, protocol and pending-action flags of that slot only;
 * last-known status values (battery, SD, sleep state) are kept.
 */
void connect_logic_reset_slot_link_state(int slot);

/**
 * @brief Handle a closed link for one slot (runs on link_evt_task)
 *
 * Expected disconnects and drops during a connect flow are only logged.
 * An unexpected drop resets the slot and flags it for main-loop cleanup.
 * Never reconnects (ADR-001).
 */
void connect_logic_handle_link_lost(int slot, int reason, uint32_t silent_ms);

/**
 * @brief Test-and-clear the "link lost" flag of a slot (main loop only)
 *
 * @return true if the slot lost its link since the last call
 */
bool connect_logic_take_link_lost(int slot);

#endif
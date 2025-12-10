#ifndef __COMMAND_LOGIC_H__
#define __COMMAND_LOGIC_H__

#include "enums_logic.h"

#include "dji_protocol_data_structures.h"

// External global device ID from ui.c
extern uint32_t g_device_id;

uint16_t generate_seq(void);

typedef struct {
    void *structure;
    size_t length;  // This is not the length of structure, but the length of DATA segment excluding CmdSet and CmdID
} CommandResult;

CommandResult send_command(int camera_index, uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const void *structure, uint16_t seq, int timeout_ms);

camera_mode_switch_response_frame_t* command_logic_switch_camera_mode(int camera_index, camera_mode_t mode);

version_query_response_frame_t* command_logic_get_version(int camera_index);

record_control_response_frame_t* command_logic_start_record(int camera_index);
esp_err_t command_logic_start_record_async(int camera_index);

record_control_response_frame_t* command_logic_stop_record(int camera_index);
esp_err_t command_logic_stop_record_async(int camera_index);


key_report_response_frame_t* command_logic_key_report_qs(int camera_index);

camera_power_mode_switch_response_frame_t* command_logic_power_mode_switch_sleep(int camera_index);
camera_power_mode_switch_response_frame_t* command_logic_power_mode_switch_wake(int camera_index);

gps_data_push_response_frame_t* command_logic_push_gps_data(int camera_index, const gps_data_push_command_frame_t* frame);

// Key Reporting functions
esp_err_t command_logic_send_key_report_for_slot(int camera_index, uint8_t key_code, uint8_t mode, uint8_t key_value);
esp_err_t command_logic_send_snapshot_key_for_slot(int camera_index);

// Highlight tag functions
bool command_logic_slot_supports_highlight(int slot_index);
bool command_logic_slot_is_paired(int slot_index);
bool command_logic_slot_is_connected(int slot_index);
bool command_logic_slot_is_awake(int slot_index);
bool command_logic_slot_is_recording(int slot_index);
esp_err_t command_logic_send_highlight_for_slot(int slot_index);
esp_err_t command_logic_send_highlight_for_all_active(void);

#endif

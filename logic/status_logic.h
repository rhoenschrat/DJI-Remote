#ifndef STATUS_LOGIC_H
#define STATUS_LOGIC_H

#include <stdint.h>

// Declaration of global variables for camera status (more can be added later)
extern uint8_t current_camera_mode;
extern uint8_t current_camera_status;
extern uint8_t current_video_resolution;
extern uint8_t current_fps_idx;
extern uint8_t current_eis_mode;
extern uint16_t current_record_time;
extern uint32_t current_remain_capacity;
extern uint32_t current_remain_time;
extern uint8_t current_camera_bat_percentage;
extern uint8_t current_power_mode;  // Sleep mode tracking: 0=normal, 3=sleep
extern bool camera_status_initialized;
extern uint32_t g_last_status_push_timestamp;  // Timestamp of last camera status push (in milliseconds)

bool is_camera_recording();

void print_camera_status();

int subscript_camera_status(int camera_index, uint8_t push_mode, uint8_t push_freq);

void update_camera_state_handler(int camera_index, void *data);

void update_new_camera_state_handler(int camera_index, void *data);

#endif
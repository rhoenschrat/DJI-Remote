#ifndef STATUS_LOGIC_H
#define STATUS_LOGIC_H

#include <stdint.h>

#include <stdbool.h>

/* True if any connected slot is recording. */
bool status_logic_any_slot_recording(void);

/* Log the current 1D02 status of one slot. */
void print_camera_status(int slot);

int subscript_camera_status(int camera_index, uint8_t push_mode, uint8_t push_freq);

void update_camera_state_handler(int camera_index, void *data);

void update_new_camera_state_handler(int camera_index, void *data);

#endif
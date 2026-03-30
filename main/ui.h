/*
 * DJI Camera Remote Control - User Interface Header
 *
 * Defines the UI system for the DJI camera remote control.
 * LVGL handles rendering and dirty-region tracking.
 *
 * Main features:
 * - Multi-camera state management (up to 3 cameras)
 * - Screen-based navigation (Main, Pairing, Settings, Mode Switch)
 * - Wake-and-record state machine
 * - Wake queue for serialized camera wake-up in "All Cameras" mode
 */

#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "connect_logic.h"
#include "ui_screen_main.h"
#include "ui_screen_pairing.h"
#include "ui_screen_settings.h"
#include "ui_screen_mode_switch.h"

// UI Screen definitions
typedef enum {
    SCREEN_MAIN = 0,            // Main camera screen (shutter, status, GPS)
    SCREEN_PAIRING,             // Pairing screen for adding new cameras
    SCREEN_CAMERA_SETTINGS,     // Per-camera settings screen
    SCREEN_MODE_SWITCH,         // Mode Switch screen for changing camera modes
    SCREEN_COUNT                // Total number of screens
} ui_screen_t;

// Wake state machine for wake-and-record functionality
typedef enum {
    WAKE_STATE_IDLE = 0,                // Normal operation, not waking up
    WAKE_STATE_BROADCASTING,            // Broadcasting wake packet (2 seconds)
    WAKE_STATE_WAITING_CONFIRMATION,    // Waiting for camera status updates
    WAKE_STATE_READY                    // Camera confirmed awake, ready to send command
} wake_state_t;

// Multi-camera support constants
#define NUM_CAMERAS 3

// Camera selection for multi-camera control
typedef enum {
    CAMERA_SELECT_0 = 0,  // Camera 0 selected
    CAMERA_SELECT_1,      // Camera 1 selected
    CAMERA_SELECT_2,      // Camera 2 selected
    CAMERA_SELECT_ALL     // All cameras selected
} camera_selection_t;

extern camera_selection_t g_camera_selection;

// Wake-up queue state machine for serialized wake broadcasts in "All Cameras" mode
typedef enum {
    WAKE_QUEUE_IDLE = 0,              // No wake-up in progress
    WAKE_QUEUE_BROADCASTING,           // Currently broadcasting wake packet
    WAKE_QUEUE_WAITING_WAKEUP         // Waiting for camera to wake up
} wake_queue_state_t;

// Wake-up queue structure for serialized processing
typedef struct {
    int camera_indices[NUM_CAMERAS];  // Queue of camera indices to wake
    int queue_head;                    // Index of next camera to process
    int queue_tail;                    // Index where next camera will be added
    int queue_count;                   // Number of cameras in queue
    bool queue_active;                 // True when queue processing is active
} wake_queue_t;

// Camera connection state enum
typedef enum {
    CAM_STATE_UNPAIRED = 0,           /* No camera paired in this slot */
    CAM_STATE_PAIRED_DISCONNECTED,    /* Camera paired but not currently connected */
    CAM_STATE_CONNECTING,             /* Currently attempting to connect */
    CAM_STATE_CONNECTED               /* Camera connected and ready */
} camera_connection_state_t;

typedef struct camera_state_s {
    // Pairing information
    bool is_paired;
    char camera_name[64];
    uint8_t camera_mac[6];
    uint32_t device_id;
    uint8_t mac_addr_len;
    int8_t mac_addr[6];
    uint32_t fw_version;
    uint16_t verify_data;
    uint8_t camera_reserved;

    // Connection and status
    camera_connection_state_t connection_state;
    bool is_connected;
    bool is_initialized;
    uint32_t last_status_timestamp;
    char model_name[64];

    // Camera status fields
    uint8_t camera_mode;
    uint8_t camera_status;
    uint8_t video_resolution;
    uint8_t fps_idx;
    uint8_t eis_mode;
    uint8_t user_mode;
    uint8_t camera_mode_next_flag;
    bool is_recording;
    uint16_t record_time;
    uint16_t timelapse_interval;
    uint32_t remain_capacity;
    uint32_t remain_time;
    uint8_t camera_bat_percentage;
    uint8_t battery_percentage;
    int power_mode;
    bool is_sleeping;
    uint32_t last_sleep_state_change_ms;
    bool pending_start_recording;
    bool snapshot_pending;
    uint32_t last_snapshot_request_ms;
    uint8_t wake_retry_count;

    // New Camera Status Push (1D06) fields
    // camera_supports_new_status_push: Per-slot flag that tracks whether this camera sends 1D06.
    // When true: Video Mode Area uses mode_name/mode_param from 1D06.
    // When false: Video Mode Area uses video_resolution/fps_idx/eis_mode from 1D02.
    // Camera Mode icon is ALWAYS derived from 1D02 camera_mode, regardless of this flag.
    bool camera_supports_new_status_push;
    char mode_name[21];
    char mode_param[21];
    char last_mode_name[21];
    char last_mode_param[21];
} camera_state_t;

// Global camera states array
extern camera_state_t g_camera_states[NUM_CAMERAS];

// UI state structure
typedef struct {
    ui_screen_t current_screen;
    bool display_needs_update;
} ui_state_t;

// Global UI state
extern ui_state_t g_ui_state;

// Wake-up queue state (for serialized wake broadcasts in "All Cameras" mode)
extern wake_queue_t g_wake_queue;
extern wake_queue_state_t g_wake_queue_state;
extern int g_current_wake_camera_index;  // Camera currently being woken (-1 if none)
extern uint32_t g_wake_broadcast_start_time_ms;  // Timestamp when current broadcast started

// LVGL screen management
/**
 * Switch active LVGL screen.  Acquires lvgl_port_lock internally.
 * Currently a no-op wrapper that sets g_ui_state.current_screen.
 * Phase 4/5 will load the corresponding LVGL screen via lv_screen_load().
 */
void ui_switch_screen(ui_screen_t screen);

// UI Function declarations
void ui_early_init(void);
void ui_init(void);
void ui_auto_connect_on_startup(void);
void ui_initiate_boot_scan_connections(void);
int ui_attempt_background_reconnection(void);
void ui_update_display(void);
void ui_execute_main_screen_action(void);
void ui_cycle_camera_selection(void);
void ui_show_message(const char* message, uint16_t color, int duration_ms);
void ui_show_shutter_bottom_message(const char* message, uint16_t color, int duration_ms);
void ui_show_not_connected_message(void);
const char* ui_get_camera_model_name(uint32_t device_id);
esp_err_t save_all_cameras_to_nvs(void);
void ui_process_pending_gpio_actions(void);
void ui_process_wake_and_record(void);
void ui_process_wake_queue(void);
void ui_process_single_camera_wake_timeout(void);

/**
 * @brief Check for GPS state changes and trigger display update if needed
 * 
 * This function should be called periodically (every 500ms) from the main loop.
 * It compares current GPS state with the last displayed state and sets the
 * display_needs_update flag only when GPS status actually changed.
 */
void ui_check_gps_update(void);

// Wake-up queue management functions
void wake_queue_init(void);
bool wake_queue_add(int camera_index);
int wake_queue_get_next(void);
void wake_queue_clear(void);
bool wake_queue_is_empty(void);
int wake_queue_get_count(void);
void wake_queue_notify_camera_woke_up(int camera_index);

// Main screen function (formerly shutter screen)
void ui_screen_main(void);

// Multi-camera pairing and settings
void ui_start_pairing(int camera_index);
void ui_pairing_add_discovered_camera(const char *name, const uint8_t *mac, int8_t rssi, uint32_t device_id);
void ui_pairing_update_discovered_camera_name(const char *name, const uint8_t *mac);
void ui_start_settings(int camera_index);

// Screen-aware button handlers
void ui_handle_button_a(void);
void ui_handle_button_b(void);
void ui_handle_button_c(void);

#endif // UI_H
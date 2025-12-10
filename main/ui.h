/*
 * DJI Camera Remote Control - User Interface Header
 * 
 * This header defines the user interface system for the DJI camera remote control.
 * The UI provides multi-camera control with partial screen redraws to minimize flicker.
 * 
 * Main features:
 * - Multi-camera state management (up to 3 cameras)
 * - Screen-based navigation (Main, Pairing, Settings, Mode Switch)
 * - Partial redraw system for efficient display updates
 * - Wake-and-record state machine
 * - Wake queue for serialized camera wake-up in "All Cameras" mode
 */

#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "icons.h"
#include "connect_logic.h"

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

// Status icon type enum for tracking last drawn icon
typedef enum {
    STATUS_ICON_NONE = 0,             /* No icon drawn (initial state) */
    STATUS_ICON_BLUETOOTH_WHITE,      /* White bluetooth (unpaired) */
    STATUS_ICON_BLUETOOTH_BLUE,       /* Blue bluetooth (paired but disconnected) */
    STATUS_ICON_FOUND,                /* Green found icon (boot scan found) */
    STATUS_ICON_CONNECTING,           /* Blue connecting icon */
    STATUS_ICON_PAUSE,                /* White pause icon (connected, not recording) */
    STATUS_ICON_RECORD,               /* Red record icon (connected, recording) */
    STATUS_ICON_SLEEP                 /* White sleep icon (connected, sleeping) */
} status_icon_type_t;

// Forward declaration of camera state structure (full definition in ui.c)
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
    uint8_t camera_bat_percentage;  /* Battery level 0-100% */
    uint8_t battery_percentage;     /* Alias for compatibility */
    int power_mode;
    bool is_sleeping;
    uint32_t last_sleep_state_change_ms;  // Timestamp of last sleep state transition (milliseconds)
    bool pending_start_recording;
    bool snapshot_pending;  // True when snapshot has been requested for this camera slot
    uint32_t last_snapshot_request_ms;  // Timestamp when snapshot was requested (milliseconds)
    uint8_t wake_retry_count;  // Number of wake broadcast retries attempted (0 = first attempt, max 1 retry)

    // New Camera Status Push (1D06) fields
    // camera_supports_new_status_push: Per-slot flag that tracks whether this camera sends 1D06.
    // - Starts as false for all slots at initialization.
    // - Set to true when the first valid 1D06 is received for this slot.
    // - Once true, it stays true for the entire pairing session (never reverts to false).
    // - When true: Video Mode Area is updated ONLY from 1D06 (mode_name, mode_param).
    // - When false: Video Mode Area is updated from 1D02 (video_resolution, fps_idx, eis_mode).
    // - Camera Mode icon is ALWAYS updated from 1D02 camera_mode field, regardless of this flag.
    bool camera_supports_new_status_push;
    char mode_name[21];  // Mode name from 1D06 (null-terminated)
    char mode_param[21];  // Mode parameter from 1D06 (null-terminated)
    char last_mode_name[21];  // Track last drawn mode_name for change detection
    char last_mode_param[21];  // Track last drawn mode_param for change detection

    // UI state tracking for partial redraws
    // last_drawn_* fields track what was actually rendered on screen to enable change detection.
    // These are separate from the current state values to detect when a redraw is needed.
    // Initialized to 0xFF (sentinel) or empty string to guarantee first-use redraw.
    char last_drawn_model_name[32];       // Last drawn model_name (for camera title)
    uint8_t last_drawn_camera_mode;       // Last drawn camera_mode (for Camera Mode icon)
    uint8_t last_drawn_video_resolution;  // Last drawn video_resolution (for Video Mode Area from 1D02)
    uint8_t last_drawn_fps_idx;           // Last drawn fps_idx (for Video Mode Area from 1D02)
    uint8_t last_drawn_eis_mode;          // Last drawn eis_mode (for Video Mode Area from 1D02)
    status_icon_type_t last_drawn_status_icon;  // Last drawn status icon type (for Camera Status Area)
    
    char last_resolution_fps[64];
    char last_sd_display[20];
    char last_battery_display[20];
    uint16_t last_time_value;
    bool last_recording_state;
    bool last_sleep_state;  // Track what sleep state was actually drawn (for change detection)
    bool needs_full_redraw;
    uint8_t last_sd_color_category;
    uint8_t last_battery_color_category;
    uint8_t last_battery_icon_index;
} camera_state_t;

// Global camera states array
extern camera_state_t g_camera_states[NUM_CAMERAS];

// UI state structure
typedef struct {
    ui_screen_t current_screen;
    bool display_needs_update;
    bool is_plus2_device;
    float scale_factor;
    int scaled_text_size;
} ui_state_t;

// Screen layout structure for different device sizes
typedef struct {
    int icon_x, icon_y;           // Icon position
    int text_x, text_y;           // Text position  
    int status_x, status_y;       // Connection status position
    int connection_radius;        // Status circle radius
} screen_layout_t;

// Global UI state
extern ui_state_t g_ui_state;
extern screen_layout_t g_layout;

// Wake-up queue state (for serialized wake broadcasts in "All Cameras" mode)
extern wake_queue_t g_wake_queue;
extern wake_queue_state_t g_wake_queue_state;
extern int g_current_wake_camera_index;  // Camera currently being woken (-1 if none)
extern uint32_t g_wake_broadcast_start_time_ms;  // Timestamp when current broadcast started

// UI Function declarations
void ui_init(void);
void ui_detect_device_and_set_scale(void);
void ui_auto_connect_on_startup(void);
void ui_initiate_boot_scan_connections(void);
int ui_attempt_background_reconnection(void);
void ui_update_display(void);
void ui_execute_main_screen_action(void);
void ui_cycle_camera_selection(void);
void ui_show_message(const char* message, uint16_t color, int duration_ms);
void ui_show_shutter_bottom_message(const char* message, uint16_t color, int duration_ms);
void ui_show_not_connected_message(void);
void ui_draw_bitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
void ui_draw_connection_status(void);
void ui_draw_gps_status(void);
int ui_get_text_width(const char* text, int text_size);
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
void ui_start_settings(int camera_index);

// Screen-aware button handlers
void ui_handle_button_a(void);
void ui_handle_button_b(void);
void ui_handle_button_c(void);

#endif // UI_H
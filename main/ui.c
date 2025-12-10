/*
 * DJI Camera Remote Control - User Interface System
 * 
 * This file implements the complete user interface system for the DJI camera remote
 * control device, including:
 * 
 * - Display management and screen rendering
 * - Button-based navigation and function execution
 * - GPIO trigger system for external hardware integration
 * - Camera pairing and connection state management
 * - Persistent storage of camera information using NVS
 * - Device ID generation and management
 * - Status updates and real-time camera state tracking
 * 
 * The UI system supports multiple screens accessed via button navigation:
 * 1. Connect - Camera pairing and connection management
 * 2. Shutter - Photo capture and video recording control
 * 3. Mode - Camera mode switching (photo, video, timelapse, etc.)
 * 4. Sleep - Put camera into sleep mode
 * 5. Wake - Wake camera from sleep using BLE broadcast
 * 
 * GPIO Integration:
 * - G0 (pull LOW): Trigger shutter function
 * - G26 (pull HIGH): Trigger sleep function
 * - G25 (pull HIGH): Trigger wake function
 * 
 * All GPIO triggers include device ID-based randomized delays (1-100ms)
 * to prevent interference when multiple devices are used simultaneously.
 * 
 * Hardware: M5Stack Basic V2.7 (ESP32, 320x240 TFT LCD)
 * Framework: ESP-IDF v5.5 with FreeRTOS
 */

#include "ui.h"
#include "m5stack_basic_v27_hal.h"
#include "command_logic.h"
#include "status_logic.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "data.h"
#include "ble.h"
#include "icons.h"
#include "../gps/gps_reader.h"
#include "../logic/status_logic.h"
#include "../logic/enums_logic.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "esp_random.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "esp_gap_ble_api.h"

/* Logging tag for ESP_LOG functions */
#define TAG "UI"

/* Multi-camera support constant */
#define NUM_CAMERAS 3

/* NVS (Non-Volatile Storage) configuration for persistent data */
#define NVS_CAMERA_NAMESPACE "camera"      /* Namespace for camera pairing data */
#define NVS_CAMERA_KEY "paired_info"       /* Key for stored camera information */
#define NVS_DEVICE_NAMESPACE "device"      /* Namespace for device configuration */
#define NVS_DEVICE_ID_KEY "device_id"      /* Key for unique device identifier */

/* Camera selection frame coordinates */
#define CAMERA_0_FRAME_X 10
#define CAMERA_0_FRAME_Y 10
#define CAMERA_1_FRAME_X 110
#define CAMERA_1_FRAME_Y 10
#define CAMERA_2_FRAME_X 210
#define CAMERA_2_FRAME_Y 10
#define CAMERA_FRAME_WIDTH 100
#define CAMERA_FRAME_HEIGHT 178

#define ALL_CAMERAS_FRAME_X 10
#define ALL_CAMERAS_FRAME_Y 10
#define ALL_CAMERAS_FRAME_WIDTH 300
#define ALL_CAMERAS_FRAME_HEIGHT 177

/* Camera selection indicator bar geometry (computed dynamically in code)
 * Indicator dimensions: 98x4 pixels, positioned 1px inside top-left of each camera block
 * Camera 0 indicator: (11, 11)
 * Camera 1 indicator: (111, 11)
 * Camera 2 indicator: (211, 11)
 */

/* Remote Status Area coordinates (contains Notification + GPS areas) */
#define REMOTE_STATUS_X         10
#define REMOTE_STATUS_Y         192
#define REMOTE_STATUS_WIDTH     300
#define REMOTE_STATUS_HEIGHT    24

/* Notification Area (left half of Remote Status Area) */
#define NOTIFICATION_X          REMOTE_STATUS_X
#define NOTIFICATION_Y          REMOTE_STATUS_Y
#define NOTIFICATION_WIDTH      150
#define NOTIFICATION_HEIGHT     24

/* GPS Status Area (right half of Remote Status Area) */
#define GPS_STATUS_X            (REMOTE_STATUS_X + 150)
#define GPS_STATUS_Y            REMOTE_STATUS_Y
#define GPS_STATUS_WIDTH        150
#define GPS_STATUS_HEIGHT       24

/* Buttons Area coordinates (bottom of screen, three button rectangles) */
#define BUTTONS_BASE_X          10
#define BUTTONS_BASE_Y          216
#define BUTTONS_WIDTH           300
#define BUTTONS_HEIGHT          24

/* Individual button dimensions and positions */
#define BUTTON_WIDTH            93
#define BUTTON_HEIGHT           24
#define BUTTON_ICON_SIZE        24
#define BUTTON_LABEL_OFFSET_X   29
#define BUTTON_LABEL_WIDTH      65

/* Button A position */
#define BUTTON_A_X              BUTTONS_BASE_X
#define BUTTON_A_Y              BUTTONS_BASE_Y

/* Button B position (Button A + gap of 10px) */
#define BUTTON_B_X              (BUTTONS_BASE_X + 103)
#define BUTTON_B_Y              BUTTONS_BASE_Y

/* Button C position */
#define BUTTON_C_X              (BUTTONS_BASE_X + 207)
#define BUTTON_C_Y              BUTTONS_BASE_Y

/* Submenu screen layout constants */
#define SUBMENU_BASE_X          10
#define SUBMENU_BASE_Y          10
#define SUBMENU_WIDTH           300
#define SUBMENU_HEIGHT          220

/* Submenu Title Area */
#define SUBMENU_TITLE_X         SUBMENU_BASE_X
#define SUBMENU_TITLE_Y         SUBMENU_BASE_Y
#define SUBMENU_TITLE_WIDTH     300
#define SUBMENU_TITLE_HEIGHT    24

/* Submenu Entry Area (up to 5 entries) */
#define SUBMENU_ENTRY_X         SUBMENU_BASE_X
#define SUBMENU_ENTRY_HEIGHT    24
#define SUBMENU_ENTRY_INDICATOR_WIDTH   10
#define SUBMENU_ENTRY_ICON_OFFSET       15
#define SUBMENU_ENTRY_LABEL_OFFSET      44
#define SUBMENU_ENTRY_LABEL_WIDTH       256
#define SUBMENU_MAX_ENTRIES     5

/* Submenu Selection Indicator Clearing Area
 * When selection changes, clear a larger region around the old indicator
 * to prevent visual artifacts. Applied only to the previously selected entry.
 * Clearing area is 5 pixels larger than indicator in all directions:
 *   Indicator: x=10, width=10, height=24
 *   Clearing:  x=5 (10-5), width=20 (10+10), height=34 (24+10)
 */
#define SUBMENU_ENTRY_CLEAR_X_OFFSET    (-5)
#define SUBMENU_ENTRY_CLEAR_Y_OFFSET    (-5)
#define SUBMENU_ENTRY_CLEAR_WIDTH       20
#define SUBMENU_ENTRY_CLEAR_HEIGHT      34

/* Submenu Notification Area */
#define SUBMENU_NOTIFICATION_X      SUBMENU_BASE_X
#define SUBMENU_NOTIFICATION_Y      (SUBMENU_BASE_Y + 174)
#define SUBMENU_NOTIFICATION_WIDTH  300
#define SUBMENU_NOTIFICATION_HEIGHT 27

/* GPIO pin definitions for external hardware triggers
 * These pins allow external devices to trigger camera functions
 * with randomized delays based on the device's unique ID
 */
#define GPIO_SHUTTER_PIN    26  /* G26 - Pull LOW to trigger SHUTTER (photo/video) */
#define GPIO_BUTTON_B_PIN   21  /* G21 - External Button B, Pull LOW to trigger (navigation/selection) */
#define GPIO_BUTTON_C_PIN   22  /* G22 - External Button C, Pull LOW to trigger (context/enter sub-screens) */
#define GPIO_SLEEP_PIN     26   /* G26 - Pull HIGH to trigger SLEEP mode */
#define GPIO_WAKE_PIN      25   /* G25 - Pull HIGH to trigger WAKE from sleep */

/* External Buttons Feature Flag
 * 
 * UI_ENABLE_EXTERNAL_BUTTONS: Compile-time flag to enable/disable external button support
 * 
 * When set to 1:
 *   - External GPIO buttons A/B/C are fully supported (GPIO26, GPIO21, GPIO22)
 *   - User must provide correct hardware wiring, including appropriate pull-up resistors
 *   - GPIO pins are configured as inputs with interrupts
 *   - External button events are processed and routed to UI handlers
 * 
 * When set to 0:
 *   - External buttons are completely disabled
 *   - Firmware never touches GPIO26, GPIO21, or GPIO22
 *   - No GPIO configuration or ISR handlers for external pins
 *   - No external button events are generated or processed
 *   - Internal buttons (GPIO39/38/37) remain fully functional
 * 
 * Default: 1 (external buttons enabled)
 */
#define UI_ENABLE_EXTERNAL_BUTTONS 1

/* Enumeration of GPIO trigger types for external hardware integration
 * These correspond to the three main camera functions that can be
 * triggered via GPIO pins with randomized delays
 */
typedef enum {
    GPIO_TRIGGER_SHUTTER = 0,    /* Photo capture or video recording toggle */
    GPIO_TRIGGER_BUTTON_B,       /* Cycle camera selection (mirrors internal Button B) */
    GPIO_TRIGGER_BUTTON_C,       /* Context/enter sub-screens (mirrors internal Button C) */
    GPIO_TRIGGER_SLEEP,          /* Put camera into sleep mode */
    GPIO_TRIGGER_WAKE            /* Wake camera from sleep mode */
} gpio_trigger_type_t;

/* Legacy structure - kept for backward compatibility */
typedef struct {
    bool is_paired;                 /* True if camera has been successfully paired */
    char camera_name[64];           /* Camera BLE advertising name (e.g., "OsmoAction5Pro1C59") */
    uint8_t camera_mac[6];          /* Camera's BLE MAC address for targeted connection */
    uint32_t device_id;             /* Protocol-level device identifier from camera */
    uint8_t mac_addr_len;           /* Length of protocol MAC address (typically 6) */
    int8_t mac_addr[6];             /* Protocol-level MAC address from camera handshake */
    uint32_t fw_version;            /* Camera firmware version for compatibility checks */
    uint16_t verify_data;           /* Last successful verification code for reconnection */
    uint8_t camera_reserved;        /* Camera identifier number in multi-camera setups */
} stored_camera_t;

/* Multi-camera state array - holds state for all camera slots
 * Index 0, 1, 2 correspond to camera slots shown on Main Screen
 * This data is synchronized with NVS for persistence across power cycles
 */
/* Global camera states array - extern declared in ui.h for access from other modules */
camera_state_t g_camera_states[NUM_CAMERAS] = {0};

/* Legacy single camera storage - kept for backward compatibility */
static stored_camera_t g_stored_camera = {0};

/* Global DJI protocol connection parameters
 * These variables are used for camera communication and are either
 * generated randomly (device_id) or received from the camera during handshake
 */
uint32_t g_device_id = 0x12345678;                           /* Unique device identifier (randomized on first boot) */
uint8_t g_mac_addr_len = 6;                                  /* MAC address length (always 6 for BLE) */
int8_t g_mac_addr[6] = {0x38, 0x34, 0x56, 0x78, 0x9A, 0xBC}; /* Protocol MAC address from remote device */
uint32_t g_fw_version = 0x00;                                /* Firmware version for compatibility */
uint8_t g_verify_mode = 0;                                   /* Authentication mode: 0=reconnect, 1=pair */
uint16_t g_verify_data = 0;                                  /* Random verification code for security */
uint8_t g_camera_reserved = 0;                               /* Camera number in multi-camera environments */

/* Global user interface state management
 * Tracks current screen, display update requirements, and display scaling parameters
 */
ui_state_t g_ui_state = {
    .current_screen = SCREEN_MAIN,    /* Start on main screen */
    .display_needs_update = true,        /* Force initial display update */
    .is_plus2_device = false,
    .scale_factor = 1.0f,               /* Display scaling factor for text/graphics */
    .scaled_text_size = 1               /* Text size multiplier for readability */
};

/* Screen layout configuration - positions and sizes for UI elements
 * Automatically configured based on detected device type and screen resolution
 */
screen_layout_t g_layout = {0};

/* Camera selection for multi-camera control */
typedef enum {
    CAMERA_SELECT_0,      // Camera 0 selected
    CAMERA_SELECT_1,      // Camera 1 selected
    CAMERA_SELECT_2,      // Camera 2 selected
    CAMERA_SELECT_ALL     // All cameras selected
} camera_selection_t;

/* Main screen state tracking */
static bool g_main_screen_initialized = false;
static float g_last_gps_lat = 0.0f;  // Track GPS latitude for change detection
static float g_last_gps_lon = 0.0f;  // Track GPS longitude for change detection
static bool g_last_gps_has_fix = false;  // Track GPS fix state for transition detection

/* GPS display state tracking (for independent updates) */
static bool g_last_displayed_gps_fix = false;  // Last displayed GPS fix state
static char g_last_displayed_gps_text[24] = "";  // Last displayed GPS coordinates text
static camera_selection_t g_camera_selection = CAMERA_SELECT_0;
static camera_selection_t g_prev_camera_selection = CAMERA_SELECT_0;

/* Pairing screen state */
#define MAX_DISCOVERED_CAMERAS 10
typedef struct {
    char name[64];
    uint8_t mac[6];
    int8_t rssi;
    uint32_t device_id;
    bool is_valid;
} discovered_camera_t;

static int g_pairing_active_camera_index = 0;  // Which slot we're pairing (0, 1, 2)
static int g_pairing_selection_index = 0;      // Selected item (0=Back, 1+=discovered cameras)
static discovered_camera_t g_discovered_cameras[MAX_DISCOVERED_CAMERAS] = {0};
static int g_discovered_camera_count = 0;
static bool g_pairing_scan_active = false;

/* Settings screen state */
static int g_settings_active_camera_index = 0;  // Which camera's settings (0, 1, 2)
static int g_settings_selection_index = 0;      // Selected item index
static int g_settings_prev_selection_index = 0; // Previous selection for partial redraw
#define SETTINGS_ITEM_BACK 0
#define SETTINGS_ITEM_MODE_SWITCH 1
#define SETTINGS_ITEM_SLEEP_WAKEUP 2
#define SETTINGS_ITEM_CONNECT_DISCONNECT 3
#define SETTINGS_ITEM_PAIR_UNPAIR 4
#define SETTINGS_ITEM_COUNT 5

// Mode Switch Screen state
static int g_mode_switch_camera_index = 0;  // Camera slot for Mode Switch Screen
static bool g_mode_switch_screen_initialized = false;  // True after first full draw

// Mode Switch Screen tracking fields (independent of main screen tracking)
// These track what was LAST DRAWN on the Mode Switch Screen to detect changes
static uint8_t g_mode_switch_last_camera_mode = 0xFF;  // Sentinel for first draw
static char g_mode_switch_last_mode_name[21] = "";
static char g_mode_switch_last_mode_param[21] = "";
static uint8_t g_mode_switch_last_resolution = 0xFF;
static uint8_t g_mode_switch_last_fps_idx = 0xFF;
static uint8_t g_mode_switch_last_eis_mode = 0xFF;

/* Wake state machine for reliable wake-and-record functionality */
static wake_state_t g_wake_state = WAKE_STATE_IDLE;
static uint32_t g_wake_broadcast_start_time = 0;  // Timestamp when wake broadcast started (milliseconds)
static uint32_t g_wake_timeout_start_time = 0;     // Timestamp when waiting for confirmation started

/* Wake-up queue for serialized wake broadcasts in "All Cameras" mode */
wake_queue_t g_wake_queue = {0};
wake_queue_state_t g_wake_queue_state = WAKE_QUEUE_IDLE;
int g_current_wake_camera_index = -1;  // Camera currently being woken (-1 if none)
uint32_t g_wake_broadcast_start_time_ms = 0;  // Timestamp when current broadcast started

/* Button visual state tracking for flicker-free updates */
typedef struct {
    const uint8_t *last_icon;       // Last drawn icon pointer (NULL if empty)
    uint16_t last_icon_color;       // Last drawn icon color
    char last_label[16];            // Last drawn label text
    bool needs_redraw;              // True when button visual state changed
} button_visual_state_t;

static button_visual_state_t g_button_states[3] = {0};  // A=0, B=1, C=2
static bool g_buttons_area_initialized = false;         // True after first draw

/* Buttons Area display mode during startup sequence
 * Controls which buttons are visible on the Main Shutter Screen:
 * - SCAN_ONLY_C: Boot scan active, only Button C "Stop" is visible
 * - BOOT_CONNECT_HIDDEN: Boot connections in progress, all buttons hidden
 * - NORMAL: Normal operation, all A/B/C buttons visible with dynamic mappings
 */
typedef enum {
    BUTTONS_MODE_SCAN_ONLY_C,         // Boot scan active - only C "Stop" visible
    BUTTONS_MODE_BOOT_CONNECT_HIDDEN, // Boot connect in progress - all hidden
    BUTTONS_MODE_NORMAL               // Normal operation - all A/B/C visible
} buttons_mode_t;

static buttons_mode_t s_last_buttons_mode = BUTTONS_MODE_NORMAL;

/**
 * @brief Save camera pairing information to non-volatile storage
 * 
 * Stores complete camera information including BLE details and protocol
 * parameters to NVS for automatic reconnection after device restart.
 * 
 * @param camera Pointer to camera information structure to save
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
/**
 * @brief Generate a unique random device identifier
 * 
 * Creates a 32-bit random device ID using ESP32's hardware random number
 * generator. This ID is used for DJI protocol communication and GPIO
 * trigger delay calculation to prevent interference between multiple devices.
 * 
 * @return 32-bit random device identifier (never zero)
 */
static uint32_t generate_random_device_id(void) {
    /* Use ESP32's hardware-based random number generator for cryptographic quality */
    uint32_t random_id = esp_random();
    
    /* Ensure ID is never zero to avoid protocol issues */
    if (random_id == 0) {
        random_id = 0x12345678;  /* Fallback pattern if hardware RNG fails */
    }
    
    ESP_LOGI(TAG, "Generated random device ID: 0x%08X", (unsigned int)random_id);
    return random_id;
}

/**
 * @brief Save device ID to non-volatile storage for persistence
 * 
 * Stores the device's unique identifier to NVS so it remains consistent
 * across power cycles and reboots. This ensures GPIO trigger delays and
 * protocol communication remain stable.
 * 
 * @param device_id The 32-bit device identifier to save
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t save_device_id_to_nvs(uint32_t device_id) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_DEVICE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening device NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_u32(nvs_handle, NVS_DEVICE_ID_KEY, device_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving device ID to NVS: %s", esp_err_to_name(err));
    } else {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error committing device ID to NVS: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Device ID 0x%08X saved to NVS successfully", (unsigned int)device_id);
        }
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Load device ID from non-volatile storage
 * 
 * Retrieves the previously stored device identifier from NVS.
 * Returns ESP_ERR_NVS_NOT_FOUND if this is the first boot.
 * 
 * @param device_id Pointer to store the loaded device ID
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not found, ESP_ERR_* on other failures
 */
static esp_err_t load_device_id_from_nvs(uint32_t* device_id) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_DEVICE_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening device NVS handle for reading: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_get_u32(nvs_handle, NVS_DEVICE_ID_KEY, device_id);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Device ID not found in NVS, this is a first boot");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error loading device ID from NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Device ID 0x%08X loaded from NVS successfully", (unsigned int)*device_id);
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Initialize device ID on system startup
 * 
 * Handles device ID initialization by either loading an existing ID from NVS
 * or generating a new random ID on first boot. The device ID is used for:
 * - DJI protocol communication
 * - GPIO trigger delay calculation
 * - Multi-device interference prevention
 */
static void initialize_device_id(void) {
    uint32_t stored_device_id;
    esp_err_t err = load_device_id_from_nvs(&stored_device_id);
    
    ESP_LOGI(TAG, "Device ID initialization starting, current g_device_id: 0x%08X", (unsigned int)g_device_id);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // First boot - generate new random device ID
        ESP_LOGI(TAG, "First boot detected, generating random device ID...");
        g_device_id = generate_random_device_id();
        
        // Save to NVS for future boots
        err = save_device_id_to_nvs(g_device_id);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save device ID to NVS, using session-only ID");
        }
    } else if (err == ESP_OK) {
        // Device ID found in NVS - use it
        g_device_id = stored_device_id;
        ESP_LOGI(TAG, "Using stored device ID: 0x%08X", (unsigned int)g_device_id);
    } else {
        // Error loading from NVS - use default but try to save random one
        ESP_LOGW(TAG, "Error loading device ID from NVS, generating new one");
        g_device_id = generate_random_device_id();
        save_device_id_to_nvs(g_device_id);  // Try to save for next time
    }
    
    ESP_LOGI(TAG, "Device ID initialization complete, final g_device_id: 0x%08X", (unsigned int)g_device_id);
}

/* GPIO control system components for external hardware integration
 * Implements thread-safe GPIO trigger processing with debouncing and delays
 */
static QueueHandle_t gpio_trigger_queue = NULL;                              /* FreeRTOS queue for GPIO events */
static TickType_t last_gpio_trigger[3] = {0, 0, 0};                         /* Per-pin debouncing timestamps */
static TickType_t last_global_gpio_trigger = 0;                             /* Global cooldown timestamp */
static volatile gpio_trigger_type_t pending_gpio_action = (gpio_trigger_type_t)-1; /* Pending action for main thread */

/**
 * @brief Calculate device-specific GPIO trigger delay
 * 
 * Generates a consistent but pseudo-random delay (1-100ms) based on the device's
 * unique ID. This prevents multiple devices from triggering simultaneously when
 * connected to the same external trigger source, reducing RF interference and
 * improving reliability in multi-device deployments.
 * 
 * @return Delay in milliseconds (1-100ms range)
 */
static uint32_t calculate_gpio_delay_ms(void) {
    /* Safety check: ensure device ID is initialized */
    if (g_device_id == 0) {
        ESP_LOGW(TAG, "Device ID not initialized, using default 50ms delay");
        return 50;
    }
    
    /* Use lower 16 bits of device ID as entropy source for consistent randomization */
    uint16_t id_hash = (uint16_t)(g_device_id & 0xFFFF);
    
    /* Map 16-bit hash (0-65535) to delay range (1-100ms) with minimum 1ms guarantee */
    uint32_t delay_ms = 1 + ((id_hash * 99) / 65535);
    
    ESP_LOGI(TAG, "GPIO trigger delay calculated: %lu ms (based on device ID 0x%08X)", 
             delay_ms, (unsigned int)g_device_id);
    
    return delay_ms;
}

/* GPIO debouncing and cooldown configuration
 * Aggressive timing to prevent multiple triggers from mechanical switch bounce
 * and to ensure stable operation with external hardware
 */
#define GPIO_DEBOUNCE_TIME_MS 1000      /* Per-pin debouncing period (1 second) */
#define GPIO_GLOBAL_COOLDOWN_MS 1000    /* Global cooldown between any GPIO triggers (1 second) */

/**
 * @brief GPIO interrupt service routine
 * 
 * Handles GPIO pin state changes by queuing trigger events for processing
 * in the main task context. The ISR must be kept minimal and fast, so all
 * actual processing is deferred to the GPIO monitor task.
 * 
 * @param arg Pointer to gpio_trigger_type_t indicating which trigger type
 */
#if UI_ENABLE_EXTERNAL_BUTTONS
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    gpio_trigger_type_t trigger_type = (gpio_trigger_type_t)(uintptr_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Send trigger type to queue for processing in main task
    xQueueSendFromISR(gpio_trigger_queue, &trigger_type, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
#endif /* UI_ENABLE_EXTERNAL_BUTTONS */

/**
 * @brief Process GPIO trigger with debouncing and delay calculation
 * 
 * Implements comprehensive GPIO trigger processing including:
 * - Per-pin and global debouncing to prevent multiple triggers
 * - Device ID-based delay calculation for multi-device coordination
 * - Thread-safe pending action system for main thread execution
 * 
 * @param trigger_type Type of GPIO trigger to process
 */
static void process_gpio_trigger(gpio_trigger_type_t trigger_type) {
    TickType_t current_time = xTaskGetTickCount();
    TickType_t debounce_ticks = pdMS_TO_TICKS(GPIO_DEBOUNCE_TIME_MS);
    TickType_t global_cooldown_ticks = pdMS_TO_TICKS(GPIO_GLOBAL_COOLDOWN_MS);
    
    // Check for global cooldown (any GPIO trigger within 1 second)
    if ((current_time - last_global_gpio_trigger) < global_cooldown_ticks) {
        ESP_LOGW(TAG, "GPIO trigger %d ignored due to global cooldown", trigger_type);
        return;
    }
    
    // Check for per-pin debouncing
    if (trigger_type < 3 && (current_time - last_gpio_trigger[trigger_type]) < debounce_ticks) {
        ESP_LOGW(TAG, "GPIO trigger %d ignored due to pin debouncing", trigger_type);
        return;
    }
    
    // Check if there's already a pending action
    if (pending_gpio_action != (gpio_trigger_type_t)-1) {
        ESP_LOGW(TAG, "GPIO trigger %d ignored, action already pending", trigger_type);
        return;
    }
    
    // Update timestamps
    last_global_gpio_trigger = current_time;
    if (trigger_type < 3) {
        last_gpio_trigger[trigger_type] = current_time;
    }
    
    uint32_t delay_ms = calculate_gpio_delay_ms();
    
    ESP_LOGI(TAG, "GPIO trigger %d processing, delaying %lu ms", trigger_type, delay_ms);
    
    // Wait for the calculated delay
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    // Set the pending action for main thread to execute
    pending_gpio_action = trigger_type;
    ESP_LOGI(TAG, "GPIO trigger %d ready for execution by main thread", trigger_type);
}

/**
 * @brief GPIO monitoring task - processes queued GPIO events
 * 
 * FreeRTOS task that receives GPIO trigger events from the ISR queue
 * and processes them with appropriate delays and debouncing. Runs
 * continuously waiting for events.
 * 
 * @param pvParameters Unused task parameter (required by FreeRTOS)
 */
static void gpio_monitor_task(void* pvParameters) {
    gpio_trigger_type_t trigger_type;
    
    while (1) {
        if (xQueueReceive(gpio_trigger_queue, &trigger_type, portMAX_DELAY)) {
            process_gpio_trigger(trigger_type);
        }
    }
}

/**
 * @brief Initialize the complete GPIO trigger system
 * 
 * Sets up GPIO pins, interrupt handlers, debouncing system, and monitoring task.
 * Configures:
 * - G0 (SHUTTER): Input with pull-up, trigger on falling edge (pull to LOW)
 * - G26 (SLEEP): Input with pull-down, trigger on rising edge (pull to HIGH)
 * - G25 (WAKE): Input with pull-down, trigger on rising edge (pull to HIGH)
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t init_gpio_system(void) {
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "Initializing GPIO trigger system");
    ESP_LOGI(TAG, "Current device ID: 0x%08X", (unsigned int)g_device_id);
    
    /* Create FreeRTOS queue for GPIO trigger events (ISR to task communication) */
    gpio_trigger_queue = xQueueCreate(10, sizeof(gpio_trigger_type_t));
    if (gpio_trigger_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create GPIO trigger queue");
        return ESP_FAIL;
    }
    
    /* Initialize debouncing timestamps and pending action state */
    for (int i = 0; i < 3; i++) {
        last_gpio_trigger[i] = 0;
    }
    last_global_gpio_trigger = 0;
    pending_gpio_action = (gpio_trigger_type_t)-1;
    
    /* Configure GPIO pins with appropriate pull resistors and interrupt types */
#if UI_ENABLE_EXTERNAL_BUTTONS
    gpio_config_t io_conf = {};
    /* G26 (SHUTTER) - Input with internal pull-up resistor
     * External circuit should pull pin LOW to trigger shutter function
     */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;      /* Interrupt on falling edge (HIGH to LOW) */
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_SHUTTER_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SHUTTER GPIO pin");
        return ret;
    }
    
    /* G21 (BUTTON_B) - External Button B, input with internal pull-up.
     * External circuit: 10k pull-up to 3.3V, button from G21 to GND.
     * Press = LOW, so we use falling-edge interrupt.
     */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;       // interrupt on falling edge (HIGH -> LOW)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_BUTTON_B_PIN);
    io_conf.pull_down_en = 0;                    // no internal pulldown
    io_conf.pull_up_en = 1;                      // internal pull-up enabled in addition to external 10k
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BUTTON_B GPIO pin");
        return ret;
    }
    
    /* G22 (BUTTON_C) - External Button C, input with NO internal pull-up/pull-down.
     * External circuit: 10k pull-up to 3.3V, button from G22 to GND.
     * Press = LOW, so we use falling-edge interrupt.
     * Idle level is HIGH (external pull-up), press drives pin LOW.
     */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;       // interrupt on falling edge (HIGH -> LOW)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_BUTTON_C_PIN);
    io_conf.pull_down_en = 0;                    // no internal pulldown
    io_conf.pull_up_en = 0;                      // no internal pull-up (external 10k provides pull-up)
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BUTTON_C GPIO pin");
        return ret;
    }
#endif /* UI_ENABLE_EXTERNAL_BUTTONS */
    
    /* G26 (SLEEP) - Input with internal pull-down resistor
     * External circuit should pull pin HIGH to trigger sleep function
     * DISABLED: External sleep/wake GPIO triggers are not used in this hardware configuration
     */
    /*
    io_conf.intr_type = GPIO_INTR_POSEDGE;      // Interrupt on rising edge (LOW to HIGH)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_SLEEP_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SLEEP GPIO pin");
        return ret;
    }
    */
    
    /* G25 (WAKE) - Input with internal pull-down resistor
     * External circuit should pull pin HIGH to trigger wake function
     * DISABLED: External sleep/wake GPIO triggers are not used in this hardware configuration
     */
    /*
    io_conf.intr_type = GPIO_INTR_POSEDGE;      // Interrupt on rising edge (LOW to HIGH)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_WAKE_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure WAKE GPIO pin");
        return ret;
    }
    */
    
    /* Install GPIO interrupt service if not already installed */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  /* ESP_ERR_INVALID_STATE = already installed */
        ESP_LOGE(TAG, "Failed to install GPIO ISR service");
        return ret;
    }
    
    /* Register interrupt handlers for each GPIO pin with trigger type identification */
#if UI_ENABLE_EXTERNAL_BUTTONS
    ret = gpio_isr_handler_add(GPIO_SHUTTER_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_SHUTTER);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SHUTTER GPIO ISR handler");
        return ret;
    }
    
    ret = gpio_isr_handler_add(GPIO_BUTTON_B_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_BUTTON_B);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BUTTON_B GPIO ISR handler");
        return ret;
    }
    
    ret = gpio_isr_handler_add(GPIO_BUTTON_C_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_BUTTON_C);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BUTTON_C GPIO ISR handler");
        return ret;
    }
#endif /* UI_ENABLE_EXTERNAL_BUTTONS */
    
    /* DISABLED: External sleep/wake GPIO triggers are not used in this hardware configuration */
    /*
    ret = gpio_isr_handler_add(GPIO_SLEEP_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_SLEEP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SLEEP GPIO ISR handler");
        return ret;
    }
    
    ret = gpio_isr_handler_add(GPIO_WAKE_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_WAKE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add WAKE GPIO ISR handler");
        return ret;
    }
    */
    
    /* Create FreeRTOS task for GPIO event processing with sufficient stack size */
    BaseType_t task_ret = xTaskCreate(gpio_monitor_task, "gpio_monitor", 4096, NULL, 10, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPIO monitor task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "GPIO trigger system initialized successfully");
    
#if UI_ENABLE_EXTERNAL_BUTTONS
    ESP_LOGI(TAG, "External buttons: ENABLED (GPIO26/21/22 mapped to A/B/C)");
    ESP_LOGI(TAG, "  G26 (Shutter): External Button A, falling-edge trigger, internal pull-up enabled, button to GND");
    ESP_LOGI(TAG, "  G21 (Button B): External Button B, falling-edge trigger, internal pull-up enabled, external 10k pull-up to 3.3V, button to GND");
    ESP_LOGI(TAG, "  G22 (Button C): External Button C, falling-edge trigger, external pull-up required");
    ESP_LOGI(TAG, "External Button A: Routed through UI button dispatch (same as internal Button A)");
#else
    ESP_LOGI(TAG, "External buttons: DISABLED (no external GPIOs configured)");
#endif /* UI_ENABLE_EXTERNAL_BUTTONS */
    
    return ESP_OK;
}

/**
 * @brief Process pending GPIO actions from main thread
 * 
 * This function must be called regularly from the main application loop
 * to execute GPIO-triggered actions in the main thread context. This
 * ensures thread safety by avoiding UI function calls from GPIO tasks.
 */
void ui_process_pending_gpio_actions(void) {
    if (pending_gpio_action != (gpio_trigger_type_t)-1) {
        gpio_trigger_type_t action = pending_gpio_action;
        pending_gpio_action = (gpio_trigger_type_t)-1; // Clear the pending action first
        
        ESP_LOGI(TAG, "Executing pending GPIO action: %d", action);
        
        switch (action) {
#if UI_ENABLE_EXTERNAL_BUTTONS
            case GPIO_TRIGGER_SHUTTER:
                ui_handle_button_a();
                break;
                
            case GPIO_TRIGGER_BUTTON_B:
                ui_handle_button_b();
                break;
                
            case GPIO_TRIGGER_BUTTON_C:
                ui_handle_button_c();
                break;
#else
            /* External button triggers are disabled - these cases should never be reached */
            case GPIO_TRIGGER_SHUTTER:
            case GPIO_TRIGGER_BUTTON_B:
            case GPIO_TRIGGER_BUTTON_C:
                ESP_LOGW(TAG, "External button trigger received but external buttons are disabled");
                break;
#endif /* UI_ENABLE_EXTERNAL_BUTTONS */
                
            case GPIO_TRIGGER_SLEEP:
                ESP_LOGI(TAG, "SLEEP GPIO trigger - not implemented in single-screen mode");
                // Sleep command logic can be added here if needed
                break;
                
            case GPIO_TRIGGER_WAKE:
                ESP_LOGI(TAG, "WAKE GPIO trigger - not implemented in single-screen mode");
                // Wake command logic can be added here if needed
                break;
        }
    }
}

/**
 * @brief Load camera pairing information from non-volatile storage
 * 
 * Retrieves previously stored camera information from NVS for automatic
 * reconnection capabilities. Returns ESP_ERR_NVS_NOT_FOUND if no camera
 * has been paired yet.
 * 
 * @param camera Pointer to structure to fill with loaded camera data
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not found, ESP_ERR_* on other failures
 */
static esp_err_t load_camera_from_nvs(stored_camera_t* camera) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    size_t required_size = sizeof(stored_camera_t);
    
    err = nvs_open(NVS_CAMERA_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error opening NVS handle for reading: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_get_blob(nvs_handle, NVS_CAMERA_KEY, camera, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Camera info loaded from NVS successfully");
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No camera info found in NVS");
        // Initialize with default values
        memset(camera, 0, sizeof(stored_camera_t));
    } else {
        ESP_LOGE(TAG, "Error loading camera from NVS: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Save camera pairing information for all slots to NVS
 * 
 * Stores pairing data for up to 3 cameras to non-volatile storage for
 * persistence across reboots. Only saves the pairing fields, not runtime status.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t save_all_cameras_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_CAMERA_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle for writing: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save each camera slot's pairing info
    for (int i = 0; i < NUM_CAMERAS; i++) {
        char key[16];
        snprintf(key, sizeof(key), "camera_%d", i);
        
        // Create a stored_camera_t from camera_state_t (only pairing fields)
        stored_camera_t stored = {
            .is_paired = g_camera_states[i].is_paired,
            .device_id = g_camera_states[i].device_id,
            .mac_addr_len = g_camera_states[i].mac_addr_len,
            .fw_version = g_camera_states[i].fw_version,
            .verify_data = g_camera_states[i].verify_data,
            .camera_reserved = g_camera_states[i].camera_reserved
        };
        strncpy(stored.camera_name, g_camera_states[i].camera_name, sizeof(stored.camera_name));
        memcpy(stored.camera_mac, g_camera_states[i].camera_mac, 6);
        memcpy(stored.mac_addr, g_camera_states[i].mac_addr, 6);
        
        err = nvs_set_blob(nvs_handle, key, &stored, sizeof(stored_camera_t));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error saving camera %d to NVS: %s", i, esp_err_to_name(err));
            nvs_close(nvs_handle);
            return err;
        }
        
        if (stored.is_paired) {
            ESP_LOGI(TAG, "Saved camera %d pairing: %s", i, stored.camera_name);
        }
    }
    
    // Delete old single-camera NVS key if camera 0 is unpaired
    if (!g_camera_states[0].is_paired) {
        err = nvs_erase_key(nvs_handle, NVS_CAMERA_KEY);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Deleted old camera key (camera 0 is unpaired)");
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGD(TAG, "Old camera key not found");
            err = ESP_OK;
        } else {
            ESP_LOGW(TAG, "Error erasing old camera key: %s", esp_err_to_name(err));
        }
    }
    
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Load camera pairing information for all slots from NVS
 * 
 * Retrieves previously stored camera information from NVS for all camera slots.
 * Also loads old single-camera format data if found for compatibility.
 * 
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if no cameras found, ESP_ERR_* on other failures
 */
static esp_err_t load_all_cameras_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_CAMERA_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error opening NVS handle for reading: %s", esp_err_to_name(err));
        return err;
    }
    
    bool any_loaded = false;
    
    // Load each camera slot's pairing info
    for (int i = 0; i < NUM_CAMERAS; i++) {
        char key[16];
        snprintf(key, sizeof(key), "camera_%d", i);
        
        stored_camera_t stored;
        size_t required_size = sizeof(stored_camera_t);
        
        err = nvs_get_blob(nvs_handle, key, &stored, &required_size);
        if (err == ESP_OK && stored.is_paired) {
            // Copy pairing info to camera_state_t
            g_camera_states[i].is_paired = stored.is_paired;
            strncpy(g_camera_states[i].camera_name, stored.camera_name, sizeof(g_camera_states[i].camera_name));
            memcpy(g_camera_states[i].camera_mac, stored.camera_mac, 6);
            g_camera_states[i].device_id = stored.device_id;
            g_camera_states[i].mac_addr_len = stored.mac_addr_len;
            memcpy(g_camera_states[i].mac_addr, stored.mac_addr, 6);
            g_camera_states[i].fw_version = stored.fw_version;
            g_camera_states[i].verify_data = stored.verify_data;
            g_camera_states[i].camera_reserved = stored.camera_reserved;
            g_camera_states[i].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
            // Initialize 1D06 support flag and tracking fields for loaded camera
            g_camera_states[i].camera_supports_new_status_push = false;  // Will be set to true if 1D06 is received
            g_camera_states[i].mode_name[0] = '\0';
            g_camera_states[i].mode_param[0] = '\0';
            g_camera_states[i].last_mode_name[0] = '\0';
            g_camera_states[i].last_mode_param[0] = '\0';
            // Initialize last_drawn_* tracking fields with sentinel values (0xFF or empty) to force first-draw
            g_camera_states[i].last_drawn_model_name[0] = '\0';
            g_camera_states[i].last_drawn_camera_mode = 0xFF;
            g_camera_states[i].last_drawn_video_resolution = 0xFF;
            g_camera_states[i].last_drawn_fps_idx = 0xFF;
            g_camera_states[i].last_drawn_eis_mode = 0xFF;
            g_camera_states[i].last_drawn_status_icon = STATUS_ICON_NONE;
            
            ESP_LOGI(TAG, "Loaded camera %d pairing: %s (MAC: %02X:%02X:%02X:%02X:%02X:%02X)",
                     i, stored.camera_name,
                     stored.camera_mac[0], stored.camera_mac[1], stored.camera_mac[2],
                     stored.camera_mac[3], stored.camera_mac[4], stored.camera_mac[5]);
            any_loaded = true;
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            // No pairing for this slot - leave as unpaired
            g_camera_states[i].is_paired = false;
            g_camera_states[i].connection_state = CAM_STATE_UNPAIRED;
            g_camera_states[i].snapshot_pending = false;  // Clear snapshot pending for unpaired slot
            g_camera_states[i].camera_supports_new_status_push = false;  // Reset 1D06 flag
            g_camera_states[i].mode_name[0] = '\0';  // Clear mode_name
            g_camera_states[i].mode_param[0] = '\0';  // Clear mode_param
            g_camera_states[i].last_mode_name[0] = '\0';  // Clear last_mode_name
            g_camera_states[i].last_mode_param[0] = '\0';  // Clear last_mode_param
            // Initialize last_drawn_* tracking fields with sentinel values (0xFF or empty) to force first-draw
            g_camera_states[i].last_drawn_model_name[0] = '\0';
            g_camera_states[i].last_drawn_camera_mode = 0xFF;
            g_camera_states[i].last_drawn_video_resolution = 0xFF;
            g_camera_states[i].last_drawn_fps_idx = 0xFF;
            g_camera_states[i].last_drawn_eis_mode = 0xFF;
            g_camera_states[i].last_drawn_status_icon = STATUS_ICON_NONE;
        }
    }
    
    nvs_close(nvs_handle);
    return any_loaded ? ESP_OK : ESP_ERR_NVS_NOT_FOUND;
}

/**
 * @brief Store camera information after successful pairing
 * 
 * Captures and stores complete camera information including BLE details
 * and protocol parameters for future automatic reconnection. Called
 * automatically after successful camera pairing.
 */
/**
 * @brief Load stored camera information for reconnection
 * 
 * Restores global protocol variables from stored camera information
 * to enable reconnection using previously established parameters.
 */
static void load_stored_camera_info(void) {
    if (g_stored_camera.is_paired) {
        g_device_id = g_stored_camera.device_id;
        g_mac_addr_len = g_stored_camera.mac_addr_len;
        memcpy(g_mac_addr, g_stored_camera.mac_addr, sizeof(g_stored_camera.mac_addr));
        g_fw_version = g_stored_camera.fw_version;
        g_verify_data = g_stored_camera.verify_data;
        g_camera_reserved = g_stored_camera.camera_reserved;
        ESP_LOGI(TAG, "Loaded stored camera info for reconnection");
    }
}

/**
 * @brief Attempt automatic connection to paired camera on startup
 * 
 * Called during system initialization to automatically connect to a
 * previously paired camera if one exists. Uses stored BLE and protocol
 * information to establish connection without user intervention.
 */
/**
 * @brief Perform complete camera reconnection (BLE + Protocol)
 * 
 * Internal function that handles both BLE and protocol reconnection.
 * Used by both startup auto-connect and background reconnection.
 * 
 * @param show_messages Whether to show UI status messages
 * @return int 0 on success, -1 on failure
 */
static int ui_perform_complete_reconnection(int camera_index, bool show_messages) {
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return -1;
    }
    
    if (!g_camera_states[camera_index].is_paired) {
        ESP_LOGI(TAG, "Camera %d is not paired, cannot reconnect", camera_index);
        return -1;
    }
    
    ESP_LOGI(TAG, "Performing complete reconnection for camera %d", camera_index);
    
    // Set connecting flag in connect_logic BEFORE UI update so connecting icon shows immediately
    connect_logic_set_slot_connecting(camera_index, true);
    
    // Update camera slot UI immediately to show "connecting" state
    g_camera_states[camera_index].connection_state = CAM_STATE_CONNECTING;
    g_camera_states[camera_index].needs_full_redraw = true;
    g_ui_state.display_needs_update = true;
    
    // Skip intermediate UI updates when on Settings Screen or Pairing Screen to avoid full redraws
    // Both screens have their own notification areas for progress
    bool skip_intermediate_ui = (g_ui_state.current_screen == SCREEN_CAMERA_SETTINGS || 
                                  g_ui_state.current_screen == SCREEN_PAIRING);
    
    // Force immediate UI update so user sees "connecting" state right away
    // (only on main screen - settings screen handles its own UI)
    if (!skip_intermediate_ui) {
        ui_update_display();
    }
    
    if (show_messages && !skip_intermediate_ui) {
        ui_show_shutter_bottom_message("Reconnecting...", M5_COLOR_CYAN, 1000);
    }
    
    /* Configure BLE layer to target the specific paired camera */
    ble_set_target_device(camera_index, g_camera_states[camera_index].camera_name, g_camera_states[camera_index].camera_mac);
    
    /* For camera 0, also update old structures for compatibility */
    if (camera_index == 0) {
        load_stored_camera_info();
    }
    g_verify_mode = 0;
    
    /* Initiate BLE connection with slot reconnect scan mode */
    int res = connect_logic_ble_connect(camera_index, SCAN_MODE_SLOT_RECONNECT);
    if (res == 0) {
        // BLE connected - update UI before protocol handshake
        g_camera_states[camera_index].needs_full_redraw = true;
        g_ui_state.display_needs_update = true;
        
        // Force immediate UI update (skip on settings screen)
        if (!skip_intermediate_ui) {
            ui_update_display();
        }
        
        if (show_messages && !skip_intermediate_ui) {
            ui_show_shutter_bottom_message("BLE Connected\nProtocol...", M5_COLOR_BLUE, 1000);
        }
        
        /* Establish DJI protocol connection using stored parameters */
        res = connect_logic_protocol_connect(
            camera_index,
            g_device_id,
            g_camera_states[camera_index].mac_addr_len,
            g_camera_states[camera_index].mac_addr,
            g_camera_states[camera_index].fw_version,
            g_verify_mode,
            g_camera_states[camera_index].verify_data,
            g_camera_states[camera_index].camera_reserved
        );
        
        if (res == 0) {
            if (show_messages && !skip_intermediate_ui) {
                ui_show_shutter_bottom_message("Connected!", M5_COLOR_GREEN, 1000);
            }
            /* Enable real-time camera status monitoring */
            subscript_camera_status(camera_index, PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
            
            // Clear the "not found during boot" flag since camera is now connected
            extern void connect_logic_clear_slot_not_found_flag(int slot_index);
            connect_logic_clear_slot_not_found_flag(camera_index);
            
            // Update camera state to connected
            g_camera_states[camera_index].connection_state = CAM_STATE_CONNECTED;
            g_camera_states[camera_index].is_connected = true;
            g_camera_states[camera_index].needs_full_redraw = true;
            g_ui_state.display_needs_update = true;
            
            // Force immediate UI update to show connected state (skip on settings screen)
            if (!skip_intermediate_ui) {
                ui_update_display();
            }
            
            ESP_LOGI(TAG, "Complete reconnection successful for camera %d", camera_index);
            return 0;
        } else {
            if (show_messages && !skip_intermediate_ui) {
                ui_show_shutter_bottom_message("Protocol Failed", M5_COLOR_RED, 1500);
            }
            g_camera_states[camera_index].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
            g_camera_states[camera_index].is_connected = false;
            if (g_camera_states[camera_index].snapshot_pending) {
                ESP_LOGD(TAG, "Camera %d: snapshot_pending cleared (protocol connection failed)", camera_index);
            }
            g_camera_states[camera_index].snapshot_pending = false;
            g_camera_states[camera_index].camera_supports_new_status_push = false;  // Reset 1D06 flag on disconnect
            g_camera_states[camera_index].mode_name[0] = '\0';  // Clear mode_name
            g_camera_states[camera_index].mode_param[0] = '\0';  // Clear mode_param
            g_camera_states[camera_index].last_mode_name[0] = '\0';  // Clear last_mode_name
            g_camera_states[camera_index].last_mode_param[0] = '\0';  // Clear last_mode_param
            // Reset last_drawn_* tracking fields with sentinel values (0xFF or empty) to force redraw on reconnect
            g_camera_states[camera_index].last_drawn_model_name[0] = '\0';
            g_camera_states[camera_index].last_drawn_camera_mode = 0xFF;
            g_camera_states[camera_index].last_drawn_video_resolution = 0xFF;
            g_camera_states[camera_index].last_drawn_fps_idx = 0xFF;
            g_camera_states[camera_index].last_drawn_eis_mode = 0xFF;
            g_camera_states[camera_index].last_drawn_status_icon = STATUS_ICON_NONE;
            
            // Remove from wake queue if present
            if (g_current_wake_camera_index == camera_index) {
                ESP_LOGW(TAG, "Camera %d disconnected during wake-up, clearing from queue", camera_index);
                g_current_wake_camera_index = -1;
                g_wake_queue_state = WAKE_QUEUE_IDLE;
            }
            
            g_camera_states[camera_index].needs_full_redraw = true;
            g_ui_state.display_needs_update = true;
            
            // Force immediate UI update to show disconnected state (skip on settings screen)
            if (!skip_intermediate_ui) {
                ui_update_display();
            }
            
            ESP_LOGW(TAG, "Protocol connection failed during reconnection for camera %d", camera_index);
        }
    } else {
        if (show_messages && !skip_intermediate_ui) {
            ui_show_shutter_bottom_message("BLE Failed", M5_COLOR_RED, 1500);
        }
        g_camera_states[camera_index].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
        g_camera_states[camera_index].is_connected = false;
        if (g_camera_states[camera_index].snapshot_pending) {
            ESP_LOGD(TAG, "Camera %d: snapshot_pending cleared (BLE connection failed)", camera_index);
        }
        g_camera_states[camera_index].snapshot_pending = false;
        
        // Remove from wake queue if present
        if (g_current_wake_camera_index == camera_index) {
            ESP_LOGW(TAG, "Camera %d disconnected during wake-up, clearing from queue", camera_index);
            g_current_wake_camera_index = -1;
            g_wake_queue_state = WAKE_QUEUE_IDLE;
        }
        
        g_camera_states[camera_index].needs_full_redraw = true;
        g_ui_state.display_needs_update = true;
        
        // Force immediate UI update to show disconnected state (skip on settings screen)
        if (!skip_intermediate_ui) {
            ui_update_display();
        }
        
        ESP_LOGW(TAG, "BLE connection failed during reconnection for camera %d", camera_index);
    }
    
    return -1;
}

/**
 * @brief Perform boot scan connection (BLE direct + Protocol) for a single camera
 * 
 * Uses direct BLE connection (no scanning) since camera was already discovered
 * during boot scan. This is the boot-scan-specific connection path.
 * 
 * @param camera_index Camera slot index (0-2)
 * @return int 0 on success, -1 on failure
 */
static int ui_perform_boot_scan_connection(int camera_index) {
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "ui_perform_boot_scan_connection: Invalid camera index: %d", camera_index);
        return -1;
    }
    
    if (!g_camera_states[camera_index].is_paired) {
        ESP_LOGI(TAG, "Camera %d is not paired, cannot connect", camera_index);
        return -1;
    }
    
    ESP_LOGI(TAG, "AUTOCONNECT_BOOT: Starting direct connection for camera %d (no new scan)", camera_index);
    
    // Set connecting flag in connect_logic BEFORE UI update so connecting icon shows immediately
    connect_logic_set_slot_connecting(camera_index, true);
    
    // Update camera slot UI immediately to show "connecting" state
    g_camera_states[camera_index].connection_state = CAM_STATE_CONNECTING;
    g_camera_states[camera_index].needs_full_redraw = true;
    g_ui_state.display_needs_update = true;
    
    // Force immediate UI update so user sees "connecting" state right away
    ui_update_display();
    
    /* Configure BLE layer to target the specific paired camera
     * (should already be set during boot scan, but ensure it's correct) */
    ble_set_target_device(camera_index, g_camera_states[camera_index].camera_name, g_camera_states[camera_index].camera_mac);
    
    /* For camera 0, also update old structures for compatibility */
    if (camera_index == 0) {
        load_stored_camera_info();
    }
    g_verify_mode = 0;
    
    /* Initiate direct BLE connection (no scanning - uses boot scan result) */
    int res = connect_logic_ble_connect_direct(camera_index);
    if (res == 0) {
        // BLE connected - update UI before protocol handshake
        g_camera_states[camera_index].needs_full_redraw = true;
        g_ui_state.display_needs_update = true;
        
        // Force immediate UI update
        ui_update_display();
        
        /* Establish DJI protocol connection using stored parameters */
        res = connect_logic_protocol_connect(
            camera_index,
            g_device_id,
            g_camera_states[camera_index].mac_addr_len,
            g_camera_states[camera_index].mac_addr,
            g_camera_states[camera_index].fw_version,
            g_verify_mode,
            g_camera_states[camera_index].verify_data,
            g_camera_states[camera_index].camera_reserved
        );
        
        if (res == 0) {
            /* Enable real-time camera status monitoring */
            subscript_camera_status(camera_index, PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
            
            // Clear the "not found during boot" flag since camera is now connected
            extern void connect_logic_clear_slot_not_found_flag(int slot_index);
            connect_logic_clear_slot_not_found_flag(camera_index);
            
            // Update camera state to connected
            g_camera_states[camera_index].connection_state = CAM_STATE_CONNECTED;
            g_camera_states[camera_index].is_connected = true;
            g_camera_states[camera_index].needs_full_redraw = true;
            g_ui_state.display_needs_update = true;
            
            // Force immediate UI update to show connected state
            ui_update_display();
            
            ESP_LOGI(TAG, "AUTOCONNECT_BOOT: Camera %d connected successfully", camera_index);
            return 0;
        } else {
            // Protocol connection failed - reset state
            g_camera_states[camera_index].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
            g_camera_states[camera_index].is_connected = false;
            g_camera_states[camera_index].snapshot_pending = false;
            g_camera_states[camera_index].camera_supports_new_status_push = false;
            g_camera_states[camera_index].mode_name[0] = '\0';
            g_camera_states[camera_index].mode_param[0] = '\0';
            g_camera_states[camera_index].last_mode_name[0] = '\0';
            g_camera_states[camera_index].last_mode_param[0] = '\0';
            // Reset last_drawn_* tracking fields
            g_camera_states[camera_index].last_drawn_model_name[0] = '\0';
            g_camera_states[camera_index].last_drawn_camera_mode = 0xFF;
            g_camera_states[camera_index].last_drawn_video_resolution = 0xFF;
            g_camera_states[camera_index].last_drawn_fps_idx = 0xFF;
            g_camera_states[camera_index].last_drawn_eis_mode = 0xFF;
            g_camera_states[camera_index].last_drawn_status_icon = STATUS_ICON_NONE;
            
            g_camera_states[camera_index].needs_full_redraw = true;
            g_ui_state.display_needs_update = true;
            ui_update_display();
            
            ESP_LOGW(TAG, "AUTOCONNECT_BOOT: Protocol connection failed for camera %d", camera_index);
        }
    } else {
        // BLE connection failed - reset state
        g_camera_states[camera_index].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
        g_camera_states[camera_index].is_connected = false;
        g_camera_states[camera_index].snapshot_pending = false;
        
        g_camera_states[camera_index].needs_full_redraw = true;
        g_ui_state.display_needs_update = true;
        ui_update_display();
        
        ESP_LOGW(TAG, "AUTOCONNECT_BOOT: BLE direct connection failed for camera %d", camera_index);
    }
    
    return -1;
}

/**
 * @brief Initiate connections for all cameras found during boot scan
 * 
 * Called after boot scan completes to start connection procedures
 * for all cameras that were discovered during the scan.
 * Uses direct BLE connections (no new scans) since cameras were already
 * discovered during the boot scan.
 */
void ui_initiate_boot_scan_connections(void) {
    ESP_LOGI(TAG, "AUTOCONNECT_BOOT: Initiating direct connections for cameras found during boot scan");
    
    // Count how many cameras were found
    int found_count = 0;
    for (int i = 0; i < NUM_CAMERAS; i++) {
        if (connect_logic_is_slot_found(i)) {
            found_count++;
        }
    }
    ESP_LOGI(TAG, "AUTOCONNECT_BOOT: %d camera(s) found, starting direct connections (no new scans)", found_count);
    
    // Connect cameras sequentially to avoid conflicts
    // This ensures each connection completes before starting the next one
    for (int i = 0; i < NUM_CAMERAS; i++) {
        if (connect_logic_is_slot_found(i)) {
            ESP_LOGI(TAG, "AUTOCONNECT_BOOT: Processing slot %d", i);
            
            // For camera 0, also sync to old structure for compatibility
            if (i == 0) {
                g_stored_camera.is_paired = g_camera_states[0].is_paired;
                strncpy(g_stored_camera.camera_name, g_camera_states[0].camera_name, sizeof(g_stored_camera.camera_name));
                memcpy(g_stored_camera.camera_mac, g_camera_states[0].camera_mac, 6);
                g_stored_camera.device_id = g_camera_states[0].device_id;
                g_stored_camera.mac_addr_len = g_camera_states[0].mac_addr_len;
                memcpy(g_stored_camera.mac_addr, g_camera_states[0].mac_addr, 6);
                g_stored_camera.fw_version = g_camera_states[0].fw_version;
                g_stored_camera.verify_data = g_camera_states[0].verify_data;
                g_stored_camera.camera_reserved = g_camera_states[0].camera_reserved;
            }
            
            // Perform boot scan connection (BLE direct + protocol)
            // This uses the direct connection path without scanning
            int result = ui_perform_boot_scan_connection(i);
            
            if (result == 0) {
                ESP_LOGI(TAG, "AUTOCONNECT_BOOT: Camera %d connected successfully", i);
            } else {
                ESP_LOGW(TAG, "AUTOCONNECT_BOOT: Camera %d connection failed", i);
            }
            
            // Small delay between connections to avoid overwhelming the BLE stack
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    
    // Clear boot connect flag - all boot-initiated connections are now done
    connect_logic_clear_boot_connect_flag();
    
    // Trigger UI refresh to show normal buttons now that boot connect phase is complete
    g_ui_state.display_needs_update = true;
    
    ESP_LOGI(TAG, "AUTOCONNECT_BOOT: Connection phase completed");
}

void ui_auto_connect_on_startup(void) {
    ESP_LOGI(TAG, "Starting auto-connect for all paired cameras");
    
    // Start single boot scan for all paired slots
    // This will set connect_state to BLE_SEARCHING and show scanning_icon
    int ret = connect_logic_start_boot_scan();
    if (ret != 0) {
        ESP_LOGI(TAG, "Boot scan not started (no paired slots or error)");
        return;
    }
    
    ESP_LOGI(TAG, "Boot scan started, waiting for cameras to be discovered...");
    // Note: Connection initiation will happen when connect_logic_update_boot_scan()
    // detects scan completion, which is called periodically from the main loop
}

/**
 * @brief Attempt background reconnection without UI messages
 * 
 * Public function for background reconnection attempts. This function
 * performs a complete reconnection (BLE + protocol) without showing
 * UI messages, making it suitable for periodic background attempts.
 * 
 * @return int 0 on success, -1 on failure or not needed
 */
int ui_attempt_background_reconnection(void) {
    /* Only attempt reconnection if we're disconnected but initialized */
    connect_state_t current_state = connect_logic_get_state();
    if (current_state >= BLE_SEARCHING) {
        /* Already connected or connecting */
        return 0;
    }
    
    if (current_state < BLE_INIT_COMPLETE) {
        /* BLE not initialized, cannot reconnect */
        return -1;
    }
    
    // Try to reconnect any paired but disconnected cameras
    // Skip cameras that were not found during boot scan (they were likely turned off)
    for (int i = 0; i < NUM_CAMERAS; i++) {
        if (g_camera_states[i].is_paired && 
            g_camera_states[i].connection_state == CAM_STATE_PAIRED_DISCONNECTED) {
            
            // Skip cameras that were not found during boot scan
            // These cameras were likely turned off and shouldn't be reconnected automatically
            extern bool connect_logic_was_slot_not_found_during_boot(int slot_index);
            if (connect_logic_was_slot_not_found_during_boot(i)) {
                ESP_LOGI(TAG, "Skipping background reconnection for camera %d (not found during boot scan)", i);
                continue;
            }
            
            ESP_LOGI(TAG, "Background reconnection attempt for camera %d", i);
            
            // For camera 0, also sync to old structure for compatibility
            if (i == 0) {
                g_stored_camera.is_paired = g_camera_states[0].is_paired;
                strncpy(g_stored_camera.camera_name, g_camera_states[0].camera_name, sizeof(g_stored_camera.camera_name));
                memcpy(g_stored_camera.camera_mac, g_camera_states[0].camera_mac, 6);
                g_stored_camera.device_id = g_camera_states[0].device_id;
                g_stored_camera.mac_addr_len = g_camera_states[0].mac_addr_len;
                memcpy(g_stored_camera.mac_addr, g_camera_states[0].mac_addr, 6);
                g_stored_camera.fw_version = g_camera_states[0].fw_version;
                g_stored_camera.verify_data = g_camera_states[0].verify_data;
                g_stored_camera.camera_reserved = g_camera_states[0].camera_reserved;
            }
            
            return ui_perform_complete_reconnection(i, false);  /* No UI messages */
        }
    }
    
    return -1;  /* No cameras need reconnection */
}

/**
 * @brief Initialize the complete user interface system
 * 
 * Performs comprehensive UI system initialization including:
 * - NVS (Non-Volatile Storage) initialization
 * - Device ID generation/loading
 * - Camera pairing data loading
 * - Data layer initialization
 * - Display configuration
 * - Automatic connection attempts
 * - GPIO trigger system setup
 */
void ui_init(void) {
    ESP_LOGI(TAG, "Initializing UI system");
    
    /* Initialize NVS flash storage for persistent data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized successfully");
    
    /* Initialize unique device identifier (random generation on first boot) */
    initialize_device_id();
    
    /* Load all camera pairing information from persistent storage */
    esp_err_t load_err = load_all_cameras_from_nvs();
    if (load_err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded camera pairings from NVS");
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (g_camera_states[i].is_paired) {
                ESP_LOGI(TAG, "  Camera %d: %s", i, g_camera_states[i].camera_name);
            }
        }
    } else {
        ESP_LOGI(TAG, "No camera pairings found in NVS, checking old format");
        esp_err_t legacy_err = load_camera_from_nvs(&g_stored_camera);
        if (legacy_err == ESP_OK && g_stored_camera.is_paired) {
            ESP_LOGI(TAG, "Found old paired camera, migrating to slot 0: %s", g_stored_camera.camera_name);
            // Migrate to camera slot 0
            g_camera_states[0].is_paired = g_stored_camera.is_paired;
            strncpy(g_camera_states[0].camera_name, g_stored_camera.camera_name, sizeof(g_camera_states[0].camera_name));
            memcpy(g_camera_states[0].camera_mac, g_stored_camera.camera_mac, 6);
            g_camera_states[0].device_id = g_stored_camera.device_id;
            g_camera_states[0].mac_addr_len = g_stored_camera.mac_addr_len;
            memcpy(g_camera_states[0].mac_addr, g_stored_camera.mac_addr, 6);
            g_camera_states[0].fw_version = g_stored_camera.fw_version;
            g_camera_states[0].verify_data = g_stored_camera.verify_data;
            g_camera_states[0].camera_reserved = g_stored_camera.camera_reserved;
            g_camera_states[0].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
            // Save in new format
            save_all_cameras_to_nvs();
        }
    }
    
    /* Initialize DJI protocol data layer with status update callbacks */
    if (!is_data_layer_initialized()) {
        ESP_LOGI(TAG, "Initializing data layer...");
        data_init();
        data_register_status_update_callback(update_camera_state_handler);
        data_register_new_status_update_callback(update_new_camera_state_handler);
        if (!is_data_layer_initialized()) {
            ESP_LOGE(TAG, "Failed to initialize data layer");
            return;
        }
        ESP_LOGI(TAG, "Data layer initialized successfully");
    }
    
    /* Configure display scaling and layout for detected hardware */
    ui_detect_device_and_set_scale();
    g_ui_state.current_screen = SCREEN_MAIN;
    g_ui_state.display_needs_update = true;
    
    /* Perform initial display update to show Main Screen before connection */
    ui_update_display();
    
    /* Attempt automatic connection to previously paired camera */
    ui_auto_connect_on_startup();
    
    /* Initialize GPIO trigger system after all other subsystems are ready */
    esp_err_t gpio_err = init_gpio_system();
    if (gpio_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GPIO system: %s", esp_err_to_name(gpio_err));
        ESP_LOGW(TAG, "Continuing without GPIO triggers");
    }
    
    /* Initialize wake-up queue for serialized wake broadcasts */
    wake_queue_init();
    ESP_LOGI(TAG, "Wake-up queue initialized");
}

/**
 * @brief Configure display scaling for M5Stack Basic V2.7
 * 
 * Sets display scaling factors for optimal rendering on 320x240 display.
 */
void ui_detect_device_and_set_scale(void) {
    g_ui_state.is_plus2_device = false;
    g_ui_state.scale_factor = 2.0f;      /* Larger scale for bigger screen */
    g_ui_state.scaled_text_size = 2;
    
    ESP_LOGI(TAG, "Detected: M5Stack Basic V2.7 (320x240)");
    ESP_LOGI(TAG, "Scale factor: %.1f, Text size: %d", 
             g_ui_state.scale_factor, g_ui_state.scaled_text_size);
    
    /* Configure screen layout coordinates for M5Stack Basic V2.7 (320x240 resolution)
     * All positions calculated for optimal visual balance and readability on larger screen
     */
    g_layout.icon_x = (320 - 24) / 2;           /* Center 24px icons horizontally (148px from left) */
    g_layout.icon_y = 40;                      /* Icon vertical position with more space */
    g_layout.text_x = 320 / 2;                  /* Center text horizontally (160px from left) */
    g_layout.text_y = g_layout.icon_y + 24 + 16; /* Text below icon with spacing for double-size text */
    g_layout.status_x = 300;                   /* Connection status indicator position (top-right) */
    g_layout.status_y = 12;
    g_layout.connection_radius = 8;             /* Slightly larger connection status indicator */
}

/**
 * @brief Draw bitmap icon at specified position
 * 
 * Renders a bitmap icon with specified dimensions and color using the
 * hardware-specific display driver functions.
 * 
 * @param x Horizontal position
 * @param y Vertical position  
 * @param bitmap Pointer to bitmap data array
 * @param w Width in pixels
 * @param h Height in pixels
 * @param color Foreground color (RGB565 format)
 */
void ui_draw_bitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
    /* Use hardware-optimized bitmap rendering with black background */
    m5stack_basic_v27_display_draw_bitmap(x, y, w, h, bitmap, color, M5_COLOR_BLACK);
}

/**
 * @brief Draw inverted bitmap icon (for icons that need color inversion)
 * 
 * Draws a bitmap where unset bits (0) are drawn with the foreground color,
 * and set bits (1) are drawn with the background color (black).
 * 
 * @param x Horizontal position
 * @param y Vertical position
 * @param bitmap Pointer to bitmap data
 * @param w Width in pixels
 * @param h Height in pixels
 * @param color Foreground color (drawn for unset bits)
 */
static void ui_draw_bitmap_inverted(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
    if (bitmap == NULL) return;
    
    /* Draw bitmap with inverted logic: unset bits get color, set bits get black */
    int byte_width = (w + 7) / 8;  /* Bytes per row */
    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            int bitmap_byte_idx = row * byte_width + col / 8;
            int bit_idx = 7 - (col % 8);  /* MSB first bit ordering */
            bool pixel_set = (bitmap[bitmap_byte_idx] >> bit_idx) & 1;
            
            /* Draw with color where bitmap bit is 0, black where bit is 1 (inverted logic) */
            if ((x + col < 320) && (y + row < 240)) {
                uint16_t pixel_color = pixel_set ? M5_COLOR_BLACK : color;
                m5stack_basic_v27_display_fill_rect(x + col, y + row, 1, 1, pixel_color);
            }
        }
    }
}

/**
 * @brief Draw connection status indicator
 * 
 * Displays a colored rectangle indicating the current camera connection state:
 * - Green: Fully connected to camera (BLE + protocol)
 * - Red: Not connected or connection in progress
 */
void ui_draw_connection_status(void) {
    uint16_t color;
    connect_state_t state = connect_logic_get_state();
    
    /* Color coding: Green for fully connected, Red for any other state */
    if (state == PROTOCOL_CONNECTED) {
        color = M5_COLOR_GREEN;
    } else {
        color = M5_COLOR_RED;
    }
    
    /* Draw connection status as small filled rectangle in top-right corner */
    int size = g_layout.connection_radius * 2;
    m5stack_basic_v27_display_fill_rect(g_layout.status_x - size/2, g_layout.status_y - size/2, 
                                     size, size, color);
}

/**
 * @brief Draw GPS status indicator
 * 
 * Displays GPS fix status and satellite count:
 * - Green: GPS has valid fix
 * - Yellow: GPS is searching for fix
 * - Red: GPS not available or no fix
 */
void ui_draw_gps_status(void) {
    uint16_t color;
    bool has_fix = gps_has_fix();
    uint8_t satellites = gps_get_satellite_count();
    
    /* Color coding based on GPS fix status */
    if (has_fix && satellites > 0) {
        color = M5_COLOR_GREEN;  /* Valid fix */
    } else if (satellites > 0) {
        color = M5_COLOR_YELLOW; /* Searching for fix */
    } else {
        color = M5_COLOR_RED;    /* No GPS signal */
    }
    
    /* Draw GPS status indicator below connection status */
    int size = 6;  /* Smaller indicator for GPS */
    int gps_x = g_layout.status_x;
    int gps_y = g_layout.status_y + g_layout.connection_radius * 2 + 4;
    m5stack_basic_v27_display_fill_rect(gps_x - size/2, gps_y - size/2, size, size, color);
    
    /* Optionally display satellite count as small text */
    if (satellites > 0) {
        char sat_str[4];
        snprintf(sat_str, sizeof(sat_str), "%d", satellites);
        m5stack_basic_v27_display_print(gps_x + 6, gps_y - 4, sat_str, M5_COLOR_GREY);
    }
}

/**
 * @brief Calculate text width for centering calculations
 * 
 * Estimates the rendered width of text based on character count and scaling.
 * Used for centering text on the display.
 * 
 * @param text Text string to measure
 * @param text_size Text scaling factor
 * @return Estimated text width in pixels
 */
int ui_get_text_width(const char* text, int text_size) {
    /* Calculate width based on fixed-width font assumption (8 pixels base * scale) */
    int char_width = 8 * text_size;
    return strlen(text) * char_width;
}

/**
 * @brief Get camera model name from device_id
 * 
 * Maps device_id to camera model name following DJI reference implementation.
 * Reference: https://github.com/dji-sdk/Osmo-GPS-Controller-Demo/blob/main/docs/protocol_data_segment.md
 * 
 * @param device_id The camera's device identifier
 * @return Camera model name string (shortened for display)
 */
const char* ui_get_camera_model_name(uint32_t device_id) {
    // Map device_id to shortened model names (as per DJI reference)
    switch (device_id) {
        case 0xFF33:
            return "Action 4";
        case 0xFF44:
            return "Action 5";
        case 0xFF55:
            return "Action 6";
        case 0xFF66:
            return "Osmo 360";
        default:
            // For unknown device IDs, return generic name
            return "Unknown";
    }
}

/**
 * @brief Format time in HH:MM:SS format
 * 
 * Converts time in seconds to HH:MM:SS display format.
 * 
 * @param seconds Time in seconds
 * @param buffer Output buffer for formatted string
 * @param size Size of output buffer
 */
static void ui_format_time_hhmmss(uint32_t seconds, char* buffer, size_t size) {
    uint32_t hours = seconds / 3600;
    uint32_t minutes = (seconds % 3600) / 60;
    uint32_t secs = seconds % 60;
    
    if (hours > 99) {
        // Handle overflow - cap at 99:59:59
        snprintf(buffer, size, "99:59:59");
    } else {
        snprintf(buffer, size, "%02lu:%02lu:%02lu", 
                 (unsigned long)hours, (unsigned long)minutes, (unsigned long)secs);
    }
}

/**
 * @brief Get shortened video resolution string
 * 
 * Maps video resolution enum values to shortened display strings.
 * 
 * @param video_resolution Video resolution enum value
 * @return Shortened resolution string
 */
static const char* ui_get_short_resolution_string(uint8_t video_resolution) {
    switch (video_resolution) {
        case 10: return "1080P";
        case 16: return "4K 16:9";
        case 45: return "2.7K 16:9";
        case 66: return "1080P 9:16";
        case 67: return "2.7K 9:16";
        case 95: return "2.7K 4:3";
        case 103: return "4K 4:3";
        case 109: return "4K 9:16";
        default: return "-";  // Unknown resolution
    }
}

/**
 * @brief Clear a specific line region on the display
 * 
 * Clears only a rectangular region to prepare for redrawing updated content.
 * Used for selective updates to avoid full screen redraws.
 * 
 * @param x X coordinate of region
 * @param y Y coordinate of region
 * @param width Width of region to clear
 * @param height Height of region to clear
 */
static void ui_clear_line_region(int x, int y, int width, int height) {
    m5stack_basic_v27_display_fill_rect(x, y, width, height, M5_COLOR_BLACK);
}

/* ======================================================================
 * BUTTONS AREA DRAWING FUNCTIONS
 * ====================================================================== */

/**
 * @brief Draw all three button background rectangles
 * 
 * Draws the background for Buttons A, B, and C as gray rectangles.
 * Called once during initial screen draw or when switching screens.
 */
static void ui_draw_button_area_backgrounds(void) {
    // Draw Button A background (black, same as screen background)
    m5stack_basic_v27_display_fill_rect(BUTTON_A_X, BUTTON_A_Y, 
                                         BUTTON_WIDTH, BUTTON_HEIGHT, M5_COLOR_BLACK);
    
    // Draw Button B background
    m5stack_basic_v27_display_fill_rect(BUTTON_B_X, BUTTON_B_Y, 
                                         BUTTON_WIDTH, BUTTON_HEIGHT, M5_COLOR_BLACK);
    
    // Draw Button C background
    m5stack_basic_v27_display_fill_rect(BUTTON_C_X, BUTTON_C_Y, 
                                         BUTTON_WIDTH, BUTTON_HEIGHT, M5_COLOR_BLACK);
    
    g_buttons_area_initialized = true;
    
    // Reset button states to force redraw of icons/labels
    for (int i = 0; i < 3; i++) {
        g_button_states[i].last_icon = NULL;
        g_button_states[i].last_label[0] = '\0';
        g_button_states[i].needs_redraw = true;
    }
}

/**
 * @brief Draw icon and label for a single button
 * 
 * Clears only the button's content area (icon + label region, not entire button)
 * and draws the new icon and label. Uses gray background.
 * 
 * @param button_id Button identifier (0=A, 1=B, 2=C)
 * @param icon Pointer to 24x24 1-bit icon bitmap (NULL for no icon)
 * @param icon_color Color for the icon (typically M5_COLOR_BLACK or other)
 * @param label Text label to display (empty string for no label)
 */
static void ui_draw_button_icon_and_label(int button_id, const uint8_t *icon, 
                                           uint16_t icon_color, const char *label) {
    if (button_id < 0 || button_id > 2) return;
    
    // Get button base coordinates
    int base_x, base_y;
    switch (button_id) {
        case 0: base_x = BUTTON_A_X; base_y = BUTTON_A_Y; break;
        case 1: base_x = BUTTON_B_X; base_y = BUTTON_B_Y; break;
        case 2: base_x = BUTTON_C_X; base_y = BUTTON_C_Y; break;
        default: return;
    }
    
    // Check if visual state actually changed
    button_visual_state_t *state = &g_button_states[button_id];
    bool icon_changed = (state->last_icon != icon) || (state->last_icon_color != icon_color);
    bool label_changed = (label == NULL) ? (state->last_label[0] != '\0') 
                                          : (strcmp(state->last_label, label) != 0);
    
    if (!icon_changed && !label_changed && !state->needs_redraw) {
        return;  // No change, skip redraw
    }
    
    // Clear entire button area with black background
    m5stack_basic_v27_display_fill_rect(base_x, base_y, BUTTON_WIDTH, BUTTON_HEIGHT, M5_COLOR_BLACK);
    
    // Draw icon if provided (white on black background)
    if (icon != NULL) {
        ui_draw_bitmap_inverted(base_x, base_y, icon, BUTTON_ICON_SIZE, BUTTON_ICON_SIZE, M5_COLOR_WHITE);
    }
    
    // Draw label if provided (white text)
    if (label != NULL && label[0] != '\0') {
        // Text position: x offset by 29, vertically centered
        int text_x = base_x + BUTTON_LABEL_OFFSET_X;
        int text_y = base_y + 8;  // Adjusted for better vertical centering in 24px height
        m5stack_basic_v27_display_print(text_x, text_y, label, M5_COLOR_WHITE);
    }
    
    // Update tracking state
    state->last_icon = icon;
    state->last_icon_color = icon_color;
    if (label != NULL) {
        strncpy(state->last_label, label, sizeof(state->last_label) - 1);
        state->last_label[sizeof(state->last_label) - 1] = '\0';
    } else {
        state->last_label[0] = '\0';
    }
    state->needs_redraw = false;
}

/**
 * @brief Clear a button to show empty gray area (no icon, no label)
 * 
 * Used when a button has no function on the current screen.
 * 
 * @param button_id Button identifier (0=A, 1=B, 2=C)
 */
static void ui_clear_button(int button_id) {
    ui_draw_button_icon_and_label(button_id, NULL, M5_COLOR_BLACK, "");
}

/**
 * @brief Update Button A icon and label based on selected camera state
 * 
 * Determines the appropriate icon/label for Button A on the Main Screen:
 * - Not paired: pair_icon (blue) + "Pair"
 * - Paired but disconnected: connect_icon (blue) + "Connect"
 * - Connected, awake, not recording: record_icon (red) + "Record"
 * - Connected, awake, recording: pause_icon + "Pause"
 * - "All Cameras" mode: Uses aggregate state logic
 */
static void ui_update_button_a_main_screen(void) {
    const uint8_t *icon = NULL;
    uint16_t icon_color = M5_COLOR_BLACK;
    const char *label = "";
    
    if (g_camera_selection == CAMERA_SELECT_ALL) {
        // "All Cameras" mode - check if any camera is recording
        bool any_recording = false;
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (command_logic_slot_is_paired(i) && 
                command_logic_slot_is_connected(i) &&
                command_logic_slot_is_awake(i) &&
                command_logic_slot_is_recording(i)) {
                any_recording = true;
                break;
            }
        }
        
        if (any_recording) {
            icon = pause_icon;
            icon_color = M5_COLOR_BLACK;
            label = "Pause";
        } else {
            icon = record_icon;
            icon_color = M5_COLOR_BLACK;
            label = "Record";
        }
    } else {
        // Single camera selection
        int slot = (int)g_camera_selection;  // 0, 1, or 2
        
        if (!command_logic_slot_is_paired(slot)) {
            // Not paired
            icon = pair_icon;
            icon_color = M5_COLOR_BLACK;
            label = "Pair";
        } else if (!command_logic_slot_is_connected(slot)) {
            // Paired but not connected
            icon = connect_icon;
            icon_color = M5_COLOR_BLACK;
            label = "Connect";
        } else if (command_logic_slot_is_recording(slot)) {
            // Connected and recording
            icon = pause_icon;
            icon_color = M5_COLOR_BLACK;
            label = "Pause";
        } else {
            // Connected, awake or sleeping, not recording
            icon = record_icon;
            icon_color = M5_COLOR_BLACK;
            label = "Record";
        }
    }
    
    ui_draw_button_icon_and_label(0, icon, icon_color, label);
}

/**
 * @brief Check if a camera slot is eligible for sleep command in "All Cameras" mode
 * 
 * Returns true if the camera is:
 * - Paired to the slot
 * - Currently connected (BLE and protocol connected)
 * - Awake (power_mode != 3 and is_sleeping flag is false)
 * - NOT currently recording
 * 
 * @param slot Camera slot index (0, 1, or 2)
 * @return true if camera can be put to sleep, false otherwise
 */
static bool is_camera_sleep_candidate(int slot) {
    return command_logic_slot_is_paired(slot) &&
           command_logic_slot_is_connected(slot) &&
           command_logic_slot_is_awake(slot) &&
           !command_logic_slot_is_recording(slot);
}

/**
 * @brief Check if a camera slot is eligible for wake command in "All Cameras" mode
 * 
 * Returns true if the camera is:
 * - Paired to the slot
 * - Currently connected (BLE and protocol connected)
 * - Currently in sleep mode (power_mode == 3 or is_sleeping flag is true)
 * 
 * @param slot Camera slot index (0, 1, or 2)
 * @return true if camera can be woken up, false otherwise
 */
static bool is_camera_wake_candidate(int slot) {
    if (!command_logic_slot_is_paired(slot) ||
        !command_logic_slot_is_connected(slot)) {
        return false;
    }
    // Camera is sleeping if power_mode == 3 or is_sleeping flag is true
    // Use the same sleep detection logic as elsewhere in the codebase
    camera_state_t *cam = &g_camera_states[slot];
    return (cam->power_mode == 3) || cam->is_sleeping;
}

/**
 * @brief Update Button B icon and label (always "Next" on Main Screen)
 */
static void ui_update_button_b_main_screen(void) {
    ui_draw_button_icon_and_label(1, next_icon, M5_COLOR_BLACK, "Next");
}

/**
 * @brief Update Button C icon and label based on current state
 * 
 * Determines the appropriate icon/label for Button C on the Main Screen:
 * - Highlight eligible: highlight_icon + "Highlight"
 * - Sleep candidates: sleep_icon + "Sleep"
 * - Wake candidates: wakeup_icon + "Wakeup"
 * - Otherwise: options_icon + "Options"
 * - "All Cameras" + no highlight/sleep/wake capability: Empty gray
 * 
 * Note: Boot scan handling is done by ui_update_main_screen_buttons_for_state()
 * which shows only Button C with "Stop" during boot scan.
 */
static void ui_update_button_c_main_screen(void) {
    const uint8_t *icon = NULL;
    uint16_t icon_color = M5_COLOR_BLACK;
    const char *label = "";
    
    // Note: Boot scan active case is now handled by ui_update_main_screen_buttons_for_state()
    // which uses BUTTONS_MODE_SCAN_ONLY_C mode. This function is only called in BUTTONS_MODE_NORMAL.
    
    if (g_camera_selection == CAMERA_SELECT_ALL) {
        // "All Cameras" mode - check if any camera is eligible for highlight
        bool any_highlight_eligible = false;
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (command_logic_slot_is_paired(i) && 
                command_logic_slot_is_connected(i) &&
                command_logic_slot_is_awake(i) &&
                command_logic_slot_is_recording(i) &&
                command_logic_slot_supports_highlight(i)) {
                any_highlight_eligible = true;
                break;
            }
        }
        
        if (any_highlight_eligible) {
            // Highlight takes priority - at least one camera is recording and supports highlights
            icon = highlight_icon;
            icon_color = M5_COLOR_BLACK;
            label = "Tag";
        } else {
            // No highlight-eligible cameras - check for sleep candidates
            // Sleep candidates are: paired, connected, awake, NOT recording
            bool any_sleep_candidate = false;
            for (int i = 0; i < NUM_CAMERAS; i++) {
                if (is_camera_sleep_candidate(i)) {
                    any_sleep_candidate = true;
                    break;
                }
            }
            
            if (any_sleep_candidate) {
                // Show Sleep option - at least one camera can be put to sleep
                icon = sleep_icon;
                icon_color = M5_COLOR_BLACK;
                label = "Sleep";
            } else {
                // No sleep candidates - check for wake candidates (sleeping cameras)
                bool any_wake_candidate = false;
                for (int i = 0; i < NUM_CAMERAS; i++) {
                    if (is_camera_wake_candidate(i)) {
                        any_wake_candidate = true;
                        break;
                    }
                }
                
                if (any_wake_candidate) {
                    // Show Wakeup option - at least one sleeping camera can be woken up
                    icon = wakeup_icon;
                    icon_color = M5_COLOR_BLACK;
                    label = "Wakeup";
                } else {
                    // No highlight, no sleep, no wake candidates - show empty button
                    ui_clear_button(2);
                    return;
                }
            }
        }
    } else {
        // Single camera selection
        int slot = (int)g_camera_selection;
        
        if (command_logic_slot_is_paired(slot) && 
            command_logic_slot_is_connected(slot) &&
            command_logic_slot_is_awake(slot) &&
            command_logic_slot_is_recording(slot) &&
            command_logic_slot_supports_highlight(slot)) {
            // Highlight eligible
            icon = highlight_icon;
            icon_color = M5_COLOR_BLACK;
            label = "Tag";
        } else {
            // Default: Options (opens Settings or Pairing screen)
            icon = options_icon;
            icon_color = M5_COLOR_BLACK;
            label = "Options";
        }
    }
    
    ui_draw_button_icon_and_label(2, icon, icon_color, label);
}

/**
 * @brief Update all button icons and labels for the Main Shutter Screen
 * 
 * Computes the current state and updates button visuals accordingly.
 * Uses state tracking to avoid unnecessary redraws.
 */
static void ui_update_main_screen_buttons(void) {
    ui_update_button_a_main_screen();
    ui_update_button_b_main_screen();
    ui_update_button_c_main_screen();
}

/**
 * @brief Master button update function for Main Shutter Screen with mode handling
 * 
 * Determines the current Buttons Area display mode based on boot scan/connect state
 * and draws the appropriate button configuration:
 * 
 * - BUTTONS_MODE_SCAN_ONLY_C: During boot scan, only Button C "Stop" is visible
 * - BUTTONS_MODE_BOOT_CONNECT_HIDDEN: During boot connect, all buttons hidden
 * - BUTTONS_MODE_NORMAL: Normal operation with full A/B/C dynamic mappings
 * 
 * Uses mode tracking to minimize redraws - only repaints when mode changes.
 */
static void ui_update_main_screen_buttons_for_state(void) {
    buttons_mode_t current_mode;
    
    // Determine current mode based on boot scan/connect state
    if (connect_logic_is_boot_scan_active()) {
        current_mode = BUTTONS_MODE_SCAN_ONLY_C;
    } else if (connect_logic_is_boot_connect_in_progress()) {
        current_mode = BUTTONS_MODE_BOOT_CONNECT_HIDDEN;
    } else {
        current_mode = BUTTONS_MODE_NORMAL;
    }
    
    // Check if mode changed - requires full redraw of buttons area
    bool mode_changed = (current_mode != s_last_buttons_mode);
    
    if (mode_changed) {
        // Clear entire buttons row to black on mode change
        m5stack_basic_v27_display_fill_rect(0, BUTTONS_BASE_Y, 320, BUTTONS_HEIGHT, M5_COLOR_BLACK);
        
        // Reset button states since we're changing modes
        for (int i = 0; i < 3; i++) {
            g_button_states[i].last_icon = NULL;
            g_button_states[i].last_label[0] = '\0';
            g_button_states[i].needs_redraw = true;
        }
        g_buttons_area_initialized = false;
    }
    
    switch (current_mode) {
        case BUTTONS_MODE_SCAN_ONLY_C:
            // Boot scan active - only show Button C with "Stop" label
            if (mode_changed) {
                // Draw only Button C gray background
                m5stack_basic_v27_display_fill_rect(BUTTON_C_X, BUTTON_C_Y, 
                    BUTTON_WIDTH, BUTTON_HEIGHT, M5_COLOR_GRAY);
            }
            // Draw Button C icon and label
            ui_draw_button_icon_and_label(2, scanning_icon, M5_COLOR_BLACK, "Stop");
            break;
            
        case BUTTONS_MODE_BOOT_CONNECT_HIDDEN:
            // Boot connect in progress - all buttons hidden (already cleared to black)
            // Nothing to draw
            break;
            
        case BUTTONS_MODE_NORMAL:
            // Normal operation - show all buttons with dynamic mappings
            if (mode_changed || !g_buttons_area_initialized) {
                ui_draw_button_area_backgrounds();
            }
            ui_update_main_screen_buttons();
            break;
    }
    
    s_last_buttons_mode = current_mode;
}

/* ======================================================================
 * SUBMENU SCREEN HELPER FUNCTIONS
 * 
 * These functions implement the unified Submenu layout used by both
 * Settings Screen and Pairing Screen.
 * ====================================================================== */

/**
 * @brief Calculate Y position for a submenu entry
 * 
 * Entry Y positions (5 entries total):
 *   Entry 0: y = 39  (Base y + 29)
 *   Entry 1: y = 68  (Base y + 58)
 *   Entry 2: y = 97  (Base y + 87)
 *   Entry 3: y = 126 (Base y + 116)
 *   Entry 4: y = 155 (Base y + 145)
 * 
 * @param entry_index Entry index (0-4)
 * @return Y coordinate for the entry
 */
static int submenu_entry_y(int entry_index) {
    // Entry Y positions: 39, 68, 97, 126, 155 for indices 0-4
    // Formula: Base y + 29 + (entry_index * 29)
    return SUBMENU_BASE_Y + 29 + (entry_index * 29);
}

/**
 * @brief Draw the submenu title "Camera X" (1-indexed)
 * 
 * @param camera_slot Camera slot index (0-2), will be displayed as 1-3
 */
static void ui_draw_submenu_title(int camera_slot) {
    char title[32];
    snprintf(title, sizeof(title), "Camera %d", camera_slot + 1);  // 1-indexed for humans
    
    // Clear title area
    ui_clear_line_region(SUBMENU_TITLE_X, SUBMENU_TITLE_Y, SUBMENU_TITLE_WIDTH, SUBMENU_TITLE_HEIGHT);
    
    // Calculate centered position for title
    int text_width = ui_get_text_width(title, 2);  // Scale 2 for larger font
    int text_x = SUBMENU_TITLE_X + (SUBMENU_TITLE_WIDTH - text_width) / 2;
    int text_y = SUBMENU_TITLE_Y + 2;  // Vertically centered
    
    m5stack_basic_v27_display_print_scaled(text_x, text_y, title, M5_COLOR_WHITE, 2);
}

/**
 * @brief Draw a single submenu entry with selection indicator, icon, and label
 * 
 * @param entry_index Entry index (0-4)
 * @param selected True if this entry is currently selected
 * @param icon Pointer to 24x24 icon bitmap (NULL for no icon)
 * @param label Text label for the entry
 */
static void ui_draw_submenu_entry(int entry_index, bool selected, const uint8_t *icon, const char *label) {
    if (entry_index < 0 || entry_index >= SUBMENU_MAX_ENTRIES) return;
    
    int entry_y = submenu_entry_y(entry_index);
    
    // Clear the entire entry area first (from indicator to end of label)
    // This prevents clutter from previous content
    ui_clear_line_region(SUBMENU_ENTRY_X, entry_y, 
                         SUBMENU_ENTRY_INDICATOR_WIDTH + SUBMENU_ENTRY_ICON_OFFSET + 24 + SUBMENU_ENTRY_LABEL_WIDTH - SUBMENU_ENTRY_ICON_OFFSET,
                         SUBMENU_ENTRY_HEIGHT);
    
    // Draw selection indicator (10px wide red bar if selected, black otherwise)
    uint16_t indicator_color = selected ? M5_COLOR_RED : M5_COLOR_BLACK;
    m5stack_basic_v27_display_fill_rect(SUBMENU_ENTRY_X, entry_y, 
                                         SUBMENU_ENTRY_INDICATOR_WIDTH, SUBMENU_ENTRY_HEIGHT, 
                                         indicator_color);
    
    // Draw icon if provided
    if (icon != NULL) {
        int icon_x = SUBMENU_ENTRY_X + SUBMENU_ENTRY_ICON_OFFSET;
        int icon_y = entry_y;
        // Icon is drawn on black background using inverted logic
        ui_draw_bitmap_inverted(icon_x, icon_y, icon, 24, 24, M5_COLOR_WHITE);
    }
    
    // Draw label
    if (label != NULL && label[0] != '\0') {
        int label_x = SUBMENU_ENTRY_X + SUBMENU_ENTRY_LABEL_OFFSET;
        int label_y = entry_y + 2;  // Vertically centered
        
        m5stack_basic_v27_display_print_scaled(label_x, label_y, label, M5_COLOR_WHITE, 2);
    }
}

/**
 * @brief Update only the selection indicators for two entries (minimal redraw)
 * 
 * Used when selection changes to avoid redrawing entire entries.
 * Process:
 *   1. Clear larger area around old indicator (20x34 at offset -5,-5)
 *   2. Draw red indicator at new position (10x24)
 * 
 * Note: The clearing rectangle is 5 pixels larger than the indicator in all directions.
 * Indicator: x=10, y=entry_y, width=10, height=24
 * Clearing:  x=5,  y=entry_y-5, width=20, height=34
 * 
 * @param old_index Previous selected entry index (-1 if none)
 * @param new_index New selected entry index
 */
static void ui_update_submenu_selection(int old_index, int new_index) {
    // Step 1: Clear old selection area with larger clearing rectangle
    if (old_index >= 0 && old_index < SUBMENU_MAX_ENTRIES) {
        int old_y = submenu_entry_y(old_index);
        
        // Calculate clearing rectangle coordinates explicitly
        int clear_x = SUBMENU_ENTRY_X - 5;   // 10 - 5 = 5
        int clear_y = old_y - 5;              // entry_y - 5
        int clear_w = 20;                     // indicator width (10) + 5 on each side
        int clear_h = 34;                     // indicator height (24) + 5 on each side
        
        // Fill clearing rectangle with black
        m5stack_basic_v27_display_fill_rect(clear_x, clear_y, clear_w, clear_h, M5_COLOR_BLACK);
    }
    
    // Step 2: Draw new selection indicator (red, 10x24)
    if (new_index >= 0 && new_index < SUBMENU_MAX_ENTRIES) {
        int new_y = submenu_entry_y(new_index);
        m5stack_basic_v27_display_fill_rect(
            SUBMENU_ENTRY_X, new_y,
            SUBMENU_ENTRY_INDICATOR_WIDTH, SUBMENU_ENTRY_HEIGHT,
            M5_COLOR_RED);
    }
}

/**
 * @brief Draw a notification message in the submenu notification area
 * 
 * Notification area: y = 184, height = 27
 * 
 * @param message Text to display
 * @param color Text color
 */
static void ui_draw_submenu_notification(const char *message, uint16_t color) {
    // Clear notification area
    ui_clear_line_region(SUBMENU_NOTIFICATION_X, SUBMENU_NOTIFICATION_Y, 
                         SUBMENU_NOTIFICATION_WIDTH, SUBMENU_NOTIFICATION_HEIGHT);
    
    if (message != NULL && message[0] != '\0') {
        // Center the text vertically (y + 5 for 27px height)
        int text_y = SUBMENU_NOTIFICATION_Y + 5;
        m5stack_basic_v27_display_print(SUBMENU_NOTIFICATION_X + 5, text_y, message, color);
    }
}

/**
 * @brief Draw the button area for submenu screens
 * 
 * Draws buttons A (Select), B (Next), and C (empty) for submenu screens.
 */
static void ui_draw_submenu_buttons(void) {
    ui_draw_button_area_backgrounds();
    
    // Button A: Select
    ui_draw_button_icon_and_label(0, select_icon, M5_COLOR_BLACK, "Select");
    
    // Button B: Next
    ui_draw_button_icon_and_label(1, next_icon, M5_COLOR_BLACK, "Next");
    
    // Button C: No function (empty)
    ui_clear_button(2);
}

/* ======================================================================
 * NEW SHUTTER SCREEN HELPER FUNCTIONS
 * ====================================================================== */

/**
 * @brief Get camera state for a given camera index
 * 
 * Populates camera state structure for drawing from g_camera_states array.
 * For camera 0, also syncs with old global variables from status_logic if not yet initialized.
 * 
 * @param index Camera index (0, 1, or 2)
 * @param state Pointer to camera_state_t to populate (temporary for drawing)
 */
static void get_camera_state_for_index(int index, camera_state_t *state) {
    if (state == NULL || index < 0 || index >= NUM_CAMERAS) return;
    
    // Copy current persistent state from g_camera_states
    memcpy(state, &g_camera_states[index], sizeof(camera_state_t));
    
    // For camera 0, sync with old global variables only if not yet initialized
    if (index == 0 && !g_camera_states[0].is_initialized) {
        extern uint8_t current_camera_mode, current_camera_status, current_video_resolution;
        extern uint8_t current_fps_idx, current_eis_mode, current_power_mode;
        extern uint16_t current_record_time;
        extern uint32_t current_remain_time, current_remain_capacity;
        extern uint8_t current_camera_bat_percentage;
        extern bool camera_status_initialized;
        extern bool is_camera_recording(void);
        
        // Sync from global status logic vars to camera state (first-time initialization only)
        state->camera_mode = current_camera_mode;
        state->camera_status = current_camera_status;
        state->video_resolution = current_video_resolution;
        state->fps_idx = current_fps_idx;
        state->eis_mode = current_eis_mode;
        state->power_mode = current_power_mode;
        state->is_sleeping = (current_power_mode == 3);
        state->record_time = current_record_time;
        state->remain_time = current_remain_time;
        state->remain_capacity = current_remain_capacity;
        state->camera_bat_percentage = current_camera_bat_percentage;
        state->battery_percentage = current_camera_bat_percentage;  // Alias
        state->is_recording = is_camera_recording();
        state->is_initialized = camera_status_initialized;
        
        // Sync connection state - check Camera 0 specifically, not global state
        // The global connect_logic_get_state() reflects ANY camera's connection,
        // so we must check Camera 0's actual connection status
        extern bool ble_is_camera_connected(int camera_index);
        if (ble_is_camera_connected(0) && g_camera_states[0].is_connected) {
            // Camera 0 is actually connected
            state->connection_state = CAM_STATE_CONNECTED;
            state->is_connected = true;
        } else if (state->is_paired) {
            // Camera 0 is paired but not connected
            state->connection_state = CAM_STATE_PAIRED_DISCONNECTED;
            state->is_connected = false;
        } else {
            // Camera 0 is not paired
            state->connection_state = CAM_STATE_UNPAIRED;
            state->is_connected = false;
        }
        
        // Sync from legacy stored_camera if needed
        if (!state->is_paired && g_stored_camera.is_paired) {
            state->is_paired = g_stored_camera.is_paired;
            strncpy(state->camera_name, g_stored_camera.camera_name, sizeof(state->camera_name));
            memcpy(state->camera_mac, g_stored_camera.camera_mac, 6);
            state->device_id = g_stored_camera.device_id;
        }
        
        // Set model name
        const char* model = ui_get_camera_model_name(state->device_id);
        strncpy(state->model_name, model, sizeof(state->model_name) - 1);
        state->model_name[sizeof(state->model_name) - 1] = '\0';
    }
    
    // For all cameras (including camera 0 after initialization), ensure model name is set
    if (state->is_paired && state->model_name[0] == '\0') {
        const char* model = ui_get_camera_model_name(state->device_id);
        strncpy(state->model_name, model, sizeof(state->model_name) - 1);
        state->model_name[sizeof(state->model_name) - 1] = '\0';
    }
}

/**
 * @brief Draw camera title (model name)
 * 
 * @param base_x Camera block base X coordinate
 * @param base_y Camera block base Y coordinate
 * @param model_name Camera model name string
 * @param state Camera state with tracking fields (last_drawn_model_name updated after drawing)
 */
static void draw_camera_title(int base_x, int base_y, const char *model_name, camera_state_t *state) {
    int title_x = base_x + 5;
    int title_y = base_y + 5;
    int title_width = 90;
    int title_height = 24;
    
    // Check if title changed (compare against what was last drawn, not current model_name)
    bool title_changed = (state == NULL) || (strcmp(state->last_drawn_model_name, model_name) != 0);
    
    if (!title_changed) {
        return;  // No change, skip redraw to avoid flicker
    }
    
    // Clear title area
    ui_clear_line_region(title_x, title_y, title_width, title_height);
    
    // Truncate model name to fit within width (approximate 8 pixels per character)
    char truncated[16];
    strncpy(truncated, model_name, sizeof(truncated) - 1);
    truncated[sizeof(truncated) - 1] = '\0';
    
    // Calculate centered position for regular text (scale=1)
    int text_scale = 1;
    int text_width = ui_get_text_width(truncated, text_scale);
    // Clip text width to available space
    if (text_width > title_width) {
        // Truncate further if needed
        int chars_to_keep = title_width / 8;
        if (chars_to_keep > 0 && chars_to_keep < sizeof(truncated)) {
            truncated[chars_to_keep] = '\0';
            text_width = ui_get_text_width(truncated, text_scale);
        }
    }
    int centered_x = title_x + (title_width - text_width) / 2;
    int baseline_y = title_y + 12;  // Approximate vertical centering for regular font
    
    // Draw model name with regular font
    m5stack_basic_v27_display_print(centered_x, baseline_y, truncated, M5_COLOR_WHITE);
    
    // Update tracking field for next comparison
    if (state != NULL) {
        strncpy(state->last_drawn_model_name, model_name, sizeof(state->last_drawn_model_name) - 1);
        state->last_drawn_model_name[sizeof(state->last_drawn_model_name) - 1] = '\0';
    }
}

/**
 * @brief Draw camera mode icon (photo or video)
 * 
 * @param base_x Camera block base X coordinate
 * @param base_y Camera block base Y coordinate
 * @param camera_mode Camera mode value (0x05 = photo, others = video)
 */
static void draw_camera_mode_icon(int base_x, int base_y, uint8_t camera_mode) {
    int cmode_x = base_x + 5;
    int cmode_y = base_y + 29;
    int icon_x = cmode_x + 33;  // Centered: (90 - 24) / 2 = 33
    int icon_y = cmode_y;
    
    // Clear icon area
    ui_clear_line_region(cmode_x, cmode_y, 90, 24);
    
    // Draw icon based on mode
    if (camera_mode == 0x05) {
        // Photo mode
        ui_draw_bitmap_inverted(icon_x, icon_y, photo_icon, 24, 24, M5_COLOR_WHITE);
    } else {
        // Video mode (or other modes)
        ui_draw_bitmap_inverted(icon_x, icon_y, video_icon, 24, 24, M5_COLOR_WHITE);
    }
}

/**
 * @brief Draw video mode text (resolution, FPS, EIS)
 * 
 * @param base_x Camera block base X coordinate
 * @param base_y Camera block base Y coordinate
 * @param resolution Video resolution value
 * @param fps_idx FPS index value
 * @param eis_mode EIS mode value
 */
static void draw_video_mode_text(int base_x, int base_y, uint8_t resolution, uint8_t fps_idx, uint8_t eis_mode) {
    int vmode_x = base_x + 5;
    int vmode_y = base_y + 53;
    int vmode_width = 90;
    int vmode_height = 24;
    
    // Build resolution and FPS/EIS strings
    const char* res_str = ui_get_short_resolution_string(resolution);
    const char* fps_str = fps_idx_to_string((fps_idx_t)fps_idx);
    const char* eis_str = eis_mode_to_string((eis_mode_t)eis_mode);
    
    // Format two-line display with truncation
    char line1[16] = "";
    char line2[16] = "";
    
    if (strlen(res_str) > 0) {
        strncpy(line1, res_str, sizeof(line1) - 1);
        line1[sizeof(line1) - 1] = '\0';
    }
    snprintf(line2, sizeof(line2), "%s %s", fps_str, eis_str);
    
    // Clear area
    ui_clear_line_region(vmode_x, vmode_y, vmode_width, vmode_height);
    
    // Center text horizontally (clip to available width)
    int line1_width = ui_get_text_width(line1, 1);
    int line2_width = ui_get_text_width(line2, 1);
    int line1_x = vmode_x + ((line1_width < vmode_width) ? (vmode_width - line1_width) / 2 : 0);
    int line2_x = vmode_x + ((line2_width < vmode_width) ? (vmode_width - line2_width) / 2 : 0);
    
    // Draw two lines with regular font
    if (strlen(line1) > 0) {
        m5stack_basic_v27_display_print(line1_x, vmode_y + 4, line1, M5_COLOR_WHITE);
    }
    m5stack_basic_v27_display_print(line2_x, vmode_y + 14, line2, M5_COLOR_WHITE);
}

/**
 * @brief Draw video mode text from New Camera Status Push (1D06)
 * 
 * Displays mode_name on line 1 and mode_param on line 2.
 * 
 * @param base_x Camera block base X coordinate
 * @param base_y Camera block base Y coordinate
 * @param mode_name Mode name string from 1D06 (max 20 chars)
 * @param mode_param Mode parameter string from 1D06 (max 20 chars)
 */
static void draw_video_mode_text_from_1d06(int base_x, int base_y, const char *mode_name, const char *mode_param) {
    int vmode_x = base_x + 5;
    int vmode_y = base_y + 53;
    int vmode_width = 90;
    int vmode_height = 24;
    
    // Truncate strings if too long (max 20 chars each)
    char line1[21] = "";
    char line2[21] = "";
    
    if (mode_name != NULL && strlen(mode_name) > 0) {
        strncpy(line1, mode_name, sizeof(line1) - 1);
        line1[sizeof(line1) - 1] = '\0';
    }
    
    if (mode_param != NULL && strlen(mode_param) > 0) {
        strncpy(line2, mode_param, sizeof(line2) - 1);
        line2[sizeof(line2) - 1] = '\0';
    }
    
    // Clear area
    ui_clear_line_region(vmode_x, vmode_y, vmode_width, vmode_height);
    
    // Center text horizontally (clip to available width)
    int line1_width = ui_get_text_width(line1, 1);
    int line2_width = ui_get_text_width(line2, 1);
    int line1_x = vmode_x + ((line1_width < vmode_width) ? (vmode_width - line1_width) / 2 : 0);
    int line2_x = vmode_x + ((line2_width < vmode_width) ? (vmode_width - line2_width) / 2 : 0);
    
    // Draw two lines with regular font
    if (strlen(line1) > 0) {
        m5stack_basic_v27_display_print(line1_x, vmode_y + 4, line1, M5_COLOR_WHITE);
    }
    if (strlen(line2) > 0) {
        m5stack_basic_v27_display_print(line2_x, vmode_y + 14, line2, M5_COLOR_WHITE);
    }
}

/**
 * @brief Determine which status icon type should be displayed for a camera
 * 
 * This is used for change detection to avoid unnecessary redraws.
 * 
 * Visual state precedence (evaluated in order):
 * 1. If connected: pause/record/sleep based on camera state
 * 2. Else if not paired: white bluetooth
 * 3. Else if connecting: blue connecting icon
 * 4. Else if found in current scan: green found icon
 * 5. Else: blue bluetooth (paired but not connected)
 * 
 * @param camera_index Camera index (0, 1, or 2)
 * @param connection_state Connection state enum
 * @param is_sleeping Whether camera is sleeping
 * @param is_recording Whether camera is recording
 * @param is_found_in_scan Whether camera was found during boot scan
 * @return status_icon_type_t The icon type that should be displayed
 */
static status_icon_type_t determine_status_icon_type(int camera_index, uint8_t connection_state,
                                                     bool is_sleeping, bool is_recording, bool is_found_in_scan) {
    // Precedence 1: Connected state
    if (connection_state == CAM_STATE_CONNECTED) {
        if (is_sleeping) {
            return STATUS_ICON_SLEEP;
        } else if (is_recording) {
            return STATUS_ICON_RECORD;
        } else {
            return STATUS_ICON_PAUSE;
        }
    }
    
    // Precedence 2: Not paired
    if (connection_state == CAM_STATE_UNPAIRED) {
        return STATUS_ICON_BLUETOOTH_WHITE;
    }
    
    // Precedence 3: Connecting (check per-slot connecting flag)
    if (connect_logic_slot_is_connecting(camera_index)) {
        return STATUS_ICON_CONNECTING;
    }
    
    // Precedence 4: Found in current scan
    if (is_found_in_scan) {
        return STATUS_ICON_FOUND;
    }
    
    // Precedence 5: Paired but not connected
    return STATUS_ICON_BLUETOOTH_BLUE;
}

/**
 * @brief Draw camera status icon (pause/record/bluetooth/sleep/found/connecting)
 * 
 * Visual state precedence (evaluated in order):
 * 1. If connected: show pause/record/sleep icons based on camera state
 * 2. Else if not paired: show white bluetooth_icon
 * 3. Else if connecting: show blue connecting_icon (overrides found state)
 * 4. Else if found in current scan: show green found_icon
 * 5. Else: show blue bluetooth_icon (paired but not connected)
 * 
 * @param camera_index Camera index (0, 1, or 2)
 * @param base_x Camera block base X coordinate
 * @param base_y Camera block base Y coordinate
 * @param connection_state Connection state (0=never paired, 1=paired but disconnected, 2=connecting, 3=connected)
 * @param is_sleeping Whether camera is in sleep mode (only relevant for connected state)
 * @param is_recording Whether camera is recording (only relevant for connected state)
 * @param is_found_in_scan Whether camera was found during current boot scan (only relevant when not connecting)
 */
static void draw_camera_status_icon(int camera_index, int base_x, int base_y, uint8_t connection_state, 
                                    bool is_sleeping, bool is_recording, bool is_found_in_scan) {
    int status_x = base_x + 5;
    int status_y = base_y + 77;
    int icon_x = status_x + 33;  // Centered: (90 - 24) / 2 = 33
    int icon_y = status_y;
    
    // Clear icon area
    ui_clear_line_region(status_x, status_y, 90, 24);
    
    // Precedence 1: Connected state - show camera status icons
    if (connection_state == CAM_STATE_CONNECTED) {
        // Connected - check for sleep mode first (sleep icon takes precedence over recording/pause)
        // Sleep mode is detected per camera via power_mode == 3 from Camera Status Push (1D02)
        if (is_sleeping) {
            // Camera is sleeping - show sleep icon in white
            ui_draw_bitmap_inverted(icon_x, icon_y, sleep_icon, 24, 24, M5_COLOR_WHITE);
        } else if (is_recording) {
            // Recording - record icon in red
            ui_draw_bitmap_inverted(icon_x, icon_y, record_icon, 24, 24, M5_COLOR_RED);
        } else {
            // Connected but paused - pause icon in white
            ui_draw_bitmap_inverted(icon_x, icon_y, pause_icon, 24, 24, M5_COLOR_WHITE);
        }
        return;  // Early return - connected state handled
    }
    
    // Precedence 2: Not paired - show white bluetooth icon
    if (connection_state == CAM_STATE_UNPAIRED) {
        ui_draw_bitmap_inverted(icon_x, icon_y, bluetooth_icon, 24, 24, M5_COLOR_WHITE);
        return;  // Early return - unpaired state handled
    }
    
    // Precedence 3: Connecting state - show blue connecting icon (overrides found state)
    if (connect_logic_slot_is_connecting(camera_index)) {
        ui_draw_bitmap_inverted(icon_x, icon_y, connecting_icon, 24, 24, M5_COLOR_BLUE);
        return;  // Early return - connecting state handled
    }
    
    // Precedence 4: Found in current scan - show green found icon
    if (is_found_in_scan) {
        ui_draw_bitmap_inverted(icon_x, icon_y, found_icon, 24, 24, M5_COLOR_GREEN);
        return;  // Early return - found state handled
    }
    
    // Precedence 5: Paired but disconnected - show blue bluetooth icon
    // This handles CAM_STATE_PAIRED_DISCONNECTED and CAM_STATE_CONNECTING (when not actually connecting)
    ui_draw_bitmap_inverted(icon_x, icon_y, bluetooth_icon, 24, 24, M5_COLOR_BLUE);
}

/**
 * @brief Draw recording time text
 * 
 * @param base_x Camera block base X coordinate
 * @param base_y Camera block base Y coordinate
 * @param is_connected Whether camera is connected
 * @param is_recording Whether camera is recording
 * @param record_time Recording time in seconds
 * @param remain_time Remaining time in seconds
 * @param prev_state Previous camera state for selective updates
 */
static void draw_recording_time(int base_x, int base_y, bool is_connected, bool is_recording, 
                                uint16_t record_time, uint16_t remain_time, 
                                camera_state_t *prev_state) {
    int recording_x = base_x + 5;
    int recording_y = base_y + 101;
    int recording_width = 90;
    
    // Determine time value to display
    uint16_t time_value = 0;
    if (is_connected) {
        time_value = is_recording ? record_time : remain_time;
    } else {
        time_value = 0;  // Show 00:00:00 for disconnected cameras
    }
    
    // Format time string
    char time_str[16];
    ui_format_time_hhmmss(time_value, time_str, sizeof(time_str));
    
    // Check if time changed or needs initial draw
    bool time_changed = (prev_state == NULL) || (prev_state->last_time_value != time_value);
    
    if (time_changed) {
        // Clear only the time text area (not entire region to avoid flicker)
        int text_scale = 1;  // Regular font
        int text_width = ui_get_text_width(time_str, text_scale);
        // Clip to available width
        if (text_width > recording_width) {
            text_width = recording_width;
        }
        int text_x = recording_x + (recording_width - text_width) / 2;
        int text_y = recording_y + 8;  // Approximate vertical centering
        
        // Clear text area
        ui_clear_line_region(text_x, text_y, text_width + 4, 10);
        
        // Draw time with regular font
        m5stack_basic_v27_display_print(text_x, text_y, time_str, M5_COLOR_WHITE);
        
        // Update previous state
        if (prev_state != NULL) {
            prev_state->last_time_value = time_value;
        }
    }
}

/**
 * @brief Get SD card color based on remaining capacity
 * 
 * Returns color code based on threshold:
 * - > 16 GB: WHITE
 * - 6-16 GB: YELLOW
 * - <= 6 GB: RED
 * 
 * @param remain_capacity_mb Remaining capacity in megabytes
 * @param category Output parameter for color category (0=white, 1=yellow, 2=red)
 * @return RGB565 color value
 */
static uint16_t get_sd_card_color(uint32_t remain_capacity_mb, uint8_t *category) {
    // Convert MB to GB for comparison
    float capacity_gb = remain_capacity_mb / 1024.0f;
    
    if (capacity_gb > 16.0f) {
        *category = 0;
        return M5_COLOR_WHITE;
    } else if (capacity_gb > 6.0f) {
        *category = 1;
        return M5_COLOR_YELLOW;
    } else {
        *category = 2;
        return M5_COLOR_RED;
    }
}

/**
 * @brief Get battery color based on percentage
 * 
 * Returns color code based on threshold:
 * - > 25%: WHITE
 * - 10-25%: YELLOW
 * - <= 10%: RED
 * 
 * @param battery_percentage Battery percentage (0-100)
 * @param category Output parameter for color category (0=white, 1=yellow, 2=red)
 * @return RGB565 color value
 */
static uint16_t get_battery_color(uint8_t battery_percentage, uint8_t *category) {
    if (battery_percentage > 25) {
        *category = 0;
        return M5_COLOR_WHITE;
    } else if (battery_percentage > 10) {
        *category = 1;
        return M5_COLOR_YELLOW;
    } else {
        *category = 2;
        return M5_COLOR_RED;
    }
}

/**
 * @brief Get battery icon based on percentage
 * 
 * Returns appropriate battery icon based on threshold:
 * - > 75%: battery_100_icon
 * - 50-75%: battery_75_icon
 * - 25-50%: battery_50_icon
 * - 10-25%: battery_25_icon
 * - <= 10%: battery_0_icon
 * 
 * @param battery_percentage Battery percentage (0-100)
 * @param icon_index Output parameter for icon index (0-4)
 * @return Pointer to battery icon bitmap data
 */
static const uint8_t* get_battery_icon(uint8_t battery_percentage, uint8_t *icon_index) {
    if (battery_percentage > 75) {
        *icon_index = 4;
        return battery_100_icon;
    } else if (battery_percentage > 50) {
        *icon_index = 3;
        return battery_75_icon;
    } else if (battery_percentage > 25) {
        *icon_index = 2;
        return battery_50_icon;
    } else if (battery_percentage > 10) {
        *icon_index = 1;
        return battery_25_icon;
    } else {
        *icon_index = 0;
        return battery_0_icon;
    }
}

/**
 * @brief Draw SD card status (icon and capacity) with dynamic color coding
 * 
 * Color changes based on remaining capacity:
 * - > 16 GB: WHITE
 * - 6-16 GB: ORANGE
 * - <= 6 GB: RED
 * 
 * @param base_x Camera block base X coordinate
 * @param base_y Camera block base Y coordinate
 * @param remain_capacity Remaining capacity in MB
 * @param prev_state Previous camera state for selective updates
 */
static void draw_sd_card_status(int base_x, int base_y, uint32_t remain_capacity, 
                                camera_state_t *prev_state) {
    int sd_x = base_x + 5;
    int sd_y = base_y + 125;
    int icon_x = sd_x;
    int icon_y = sd_y;
    int text_x = sd_x + 29;
    int text_y = sd_y;
    
    // Format capacity string
    char current_display[16];
    if (remain_capacity >= 1024) {
        snprintf(current_display, sizeof(current_display), "%.1fGB", (float)remain_capacity / 1024.0f);
    } else {
        snprintf(current_display, sizeof(current_display), "%luMB", (unsigned long)remain_capacity);
    }
    
    // Get dynamic color based on capacity
    uint8_t color_category;
    uint16_t sd_color = get_sd_card_color(remain_capacity, &color_category);
    
    // Determine what needs to be redrawn
    bool first_draw = (prev_state == NULL || prev_state->last_sd_display[0] == '\0');
    bool color_changed = (prev_state != NULL && prev_state->last_sd_color_category != color_category);
    bool value_changed = (prev_state == NULL || strcmp(prev_state->last_sd_display, current_display) != 0);
    
    if (first_draw || color_changed || value_changed) {
        if (first_draw || color_changed) {
            // Redraw both icon and text when color category changes
            ui_clear_line_region(sd_x, sd_y, 90, 24);
            ui_draw_bitmap_inverted(icon_x, icon_y, sd_card_icon, 24, 24, sd_color);
        } else {
            // Only value changed - clear text area only (x=29, y=125, 66x24 relative to camera base)
            ui_clear_line_region(base_x + 29, base_y + 125, 66, 24);
        }
        
        // Draw capacity text with dynamic color
        m5stack_basic_v27_display_print(text_x, text_y + 8, current_display, sd_color);
        
        // Update previous state
        if (prev_state != NULL) {
            strncpy(prev_state->last_sd_display, current_display, sizeof(prev_state->last_sd_display) - 1);
            prev_state->last_sd_display[sizeof(prev_state->last_sd_display) - 1] = '\0';
            prev_state->last_sd_color_category = color_category;
        }
    }
}

/**
 * @brief Draw battery status (icon and percentage) with dynamic color coding and icon selection
 * 
 * Color changes based on battery percentage:
 * - > 25%: WHITE
 * - 10-25%: ORANGE
 * - <= 10%: RED
 * 
 * Icon selection based on battery percentage:
 * - > 75%: battery_100_icon
 * - 50-75%: battery_75_icon
 * - 25-50%: battery_50_icon
 * - 10-25%: battery_25_icon
 * - <= 10%: battery_0_icon
 * 
 * @param base_x Camera block base X coordinate
 * @param base_y Camera block base Y coordinate
 * @param battery_percentage Battery percentage (0-100)
 * @param prev_state Previous camera state for selective updates
 */
static void draw_battery_status(int base_x, int base_y, uint8_t battery_percentage, 
                                camera_state_t *prev_state) {
    int battery_x = base_x + 5;
    int battery_y = base_y + 149;
    int icon_x = battery_x;
    int icon_y = battery_y;
    int text_x = battery_x + 29;
    int text_y = battery_y;
    
    // Format battery string with space before %
    char current_display[16];
    snprintf(current_display, sizeof(current_display), "%d%%", battery_percentage);
    
    // Get dynamic color based on battery level
    uint8_t color_category;
    uint16_t battery_color = get_battery_color(battery_percentage, &color_category);
    
    // Get appropriate battery icon based on level
    uint8_t icon_index;
    const uint8_t* battery_icon = get_battery_icon(battery_percentage, &icon_index);
    
    // Determine what needs to be redrawn
    bool first_draw = (prev_state == NULL || prev_state->last_battery_display[0] == '\0');
    bool color_changed = (prev_state != NULL && prev_state->last_battery_color_category != color_category);
    bool icon_changed = (prev_state != NULL && prev_state->last_battery_icon_index != icon_index);
    bool value_changed = (prev_state == NULL || strcmp(prev_state->last_battery_display, current_display) != 0);
    
    if (first_draw || color_changed || icon_changed || value_changed) {
        if (first_draw || color_changed || icon_changed) {
            // Redraw both icon and text when color category or icon changes
            ui_clear_line_region(battery_x, battery_y, 90, 24);
            ui_draw_bitmap_inverted(icon_x, icon_y, battery_icon, 24, 24, battery_color);
        } else {
            // Only value changed - clear text area only (x=29, y=149, 66x24 relative to camera base)
            ui_clear_line_region(base_x + 29, base_y + 149, 66, 24);
        }
        
        // Draw battery text with dynamic color
        m5stack_basic_v27_display_print(text_x, text_y + 8, current_display, battery_color);
        
        // Update previous state
        if (prev_state != NULL) {
            strncpy(prev_state->last_battery_display, current_display, sizeof(prev_state->last_battery_display) - 1);
            prev_state->last_battery_display[sizeof(prev_state->last_battery_display) - 1] = '\0';
            prev_state->last_battery_color_category = color_category;
            prev_state->last_battery_icon_index = icon_index;
        }
    }
}

/**
 * @brief Draw a frame border around a camera block
 * 
 * @param x Frame X coordinate
 * @param y Frame Y coordinate
 * @param width Frame width
 * @param height Frame height
 * @param color Frame color (RGB565)
 */
static void draw_camera_frame(int x, int y, int width, int height, uint16_t color) {
    // Draw 1-pixel border using 4 rectangles
    // Top border
    m5stack_basic_v27_display_fill_rect(x, y, width, 1, color);
    // Bottom border
    m5stack_basic_v27_display_fill_rect(x, y + height - 1, width, 1, color);
    // Left border
    m5stack_basic_v27_display_fill_rect(x, y, 1, height, color);
    // Right border
    m5stack_basic_v27_display_fill_rect(x + width - 1, y, 1, height, color);
}

/**
 * @brief Draw selection indicator bars for all three cameras
 * 
 * Draws a thin horizontal bar at the top of each camera block.
 * Selected cameras show red bar, unselected show black bar.
 * When CAMERA_SELECT_ALL is active, all three bars are red.
 * 
 * @param selection Current camera selection state
 */
static void draw_camera_selection_indicators(camera_selection_t selection) {
    // Draw indicator bar for each camera based on selection state
    for (int i = 0; i < 3; i++) {
        // Compute base coordinates (same as camera block drawing)
        int base_x = 10 + (i * 100);  // Camera 0: 10, Camera 1: 110, Camera 2: 210
        int base_y = 10;
        
        // Compute indicator position (1px inside top-left of camera block)
        int ind_x = base_x + 1;
        int ind_y = base_y + 1;
        int ind_w = 98;
        int ind_h = 4;
        
        // Determine if this camera index is selected
        bool selected = false;
        switch (selection) {
            case CAMERA_SELECT_0:
                selected = (i == 0);
                break;
            case CAMERA_SELECT_1:
                selected = (i == 1);
                break;
            case CAMERA_SELECT_2:
                selected = (i == 2);
                break;
            case CAMERA_SELECT_ALL:
                selected = true;  // All cameras selected
                break;
        }
        
        // Draw indicator bar (red if selected, black if not)
        uint16_t color = selected ? M5_COLOR_RED : M5_COLOR_BLACK;
        m5stack_basic_v27_display_fill_rect(ind_x, ind_y, ind_w, ind_h, color);

        // Small delay after each indicator to ensure display driver processes it
        // This replaces the implicit delay that ESP_LOGI was providing
        vTaskDelay(1);
    }
}

/**
 * @brief Draw complete camera block with conditional rendering based on connection state
 * 
 * Uses tracking fields in the state itself (last_drawn_*, last_mode_*) for change detection.
 * These fields are updated in place after drawing to track what was actually rendered.
 * 
 * @param camera_index Camera index (0, 1, or 2)
 * @param base_x Camera block base X coordinate
 * @param base_y Camera block base Y coordinate
 * @param state Pointer to camera state (g_camera_states[i]) - tracking fields updated in place
 * @param prev_state Unused, kept for API compatibility (pass NULL)
 */
static void draw_camera_block(int camera_index, int base_x, int base_y, 
                              camera_state_t *state, 
                              camera_state_t *prev_state) {
    if (state == NULL) return;
    (void)prev_state;  // Suppress unused parameter warning
    
    bool needs_full_redraw = state->needs_full_redraw;
    
    // Connection state change detection using static tracker per slot
    static camera_connection_state_t s_last_connection_state[NUM_CAMERAS] = {
        CAM_STATE_UNPAIRED, CAM_STATE_UNPAIRED, CAM_STATE_UNPAIRED
    };
    bool state_changed = (s_last_connection_state[camera_index] != state->connection_state);
    if (state_changed) {
        s_last_connection_state[camera_index] = state->connection_state;
    }
    
    // Handle connection state transitions
    switch (state->connection_state) {
        case CAM_STATE_UNPAIRED: // Never paired
            if (state_changed) {
                // Clear all areas except status
                // Clear title through video mode (base_y+5 to base_y+77 = 72 height)
                ui_clear_line_region(base_x + 5, base_y + 5, 90, 72);
                // Clear recording through battery (base_y+101 to base_y+173 = 72 height)
                ui_clear_line_region(base_x + 5, base_y + 101, 90, 72);
            }
            // Draw only status icon - use smart redraw based on actual icon change
            {
                bool boot_phase_active = connect_logic_is_boot_scan_active() || connect_logic_is_boot_connect_in_progress();
                bool is_found = connect_logic_is_slot_found(camera_index) && boot_phase_active;
                status_icon_type_t current_icon = determine_status_icon_type(camera_index, state->connection_state, 
                                                                             state->is_sleeping, false, is_found);
                // Only redraw if icon type actually changed or needs full redraw
                if (needs_full_redraw || state_changed || current_icon != state->last_drawn_status_icon) {
                    draw_camera_status_icon(camera_index, base_x, base_y, state->connection_state, state->is_sleeping, false, is_found);
                    state->last_drawn_status_icon = current_icon;
                }
            }
            break;
            
        case CAM_STATE_PAIRED_DISCONNECTED: // Paired but disconnected
        case CAM_STATE_CONNECTING: // Treat connecting same as disconnected for now
            if (state_changed) {
                // Clear mode, video mode areas (base_y+29 to base_y+77 = 48 height)
                ui_clear_line_region(base_x + 5, base_y + 29, 90, 48);
                // Clear recording through battery (base_y+101 to base_y+173 = 72 height)
                ui_clear_line_region(base_x + 5, base_y + 101, 90, 72);
            }
            // Draw title and status icon only
            draw_camera_title(base_x, base_y, state->model_name, state);
            
            // Use smart icon redraw - only redraw if the actual icon type changed
            // This prevents flickering during boot connect phase by checking per-slot state
            {
                bool boot_phase_active = connect_logic_is_boot_scan_active() || connect_logic_is_boot_connect_in_progress();
                bool is_found = connect_logic_is_slot_found(camera_index) && boot_phase_active;
                status_icon_type_t current_icon = determine_status_icon_type(camera_index, state->connection_state,
                                                                             state->is_sleeping, false, is_found);
                // Only redraw if icon type actually changed or needs full redraw
                if (needs_full_redraw || state_changed || current_icon != state->last_drawn_status_icon) {
                    draw_camera_status_icon(camera_index, base_x, base_y, state->connection_state, state->is_sleeping, false, is_found);
                    state->last_drawn_status_icon = current_icon;
                }
            }
            break;
            
        case CAM_STATE_CONNECTED: // Connected
            // Draw all areas as before
            
            // Draw title (only if changed - uses last model_name in state for comparison)
            draw_camera_title(base_x, base_y, state->model_name, state);
            
            // Draw Camera Mode icon
            // ALWAYS derived from 1D02 camera_mode field, regardless of 1D06 support.
            // Use last_drawn_camera_mode (sentinel 0xFF forces first-draw) for change detection.
            {
                bool camera_mode_changed = (state->last_drawn_camera_mode != state->camera_mode);
                
                if (needs_full_redraw || state_changed || camera_mode_changed) {
                    draw_camera_mode_icon(base_x, base_y, state->camera_mode);
                    state->last_drawn_camera_mode = state->camera_mode;
                }
            }
            
            // Draw Video Mode Area
            // - If camera_supports_new_status_push is true: use mode_name (line 1) and mode_param (line 2) from 1D06
            // - If camera_supports_new_status_push is false: use video_resolution, fps_idx, eis_mode from 1D02
            if (state->camera_supports_new_status_push) {
                // 1D06 mode: Use mode_name and mode_param for Video Mode Area
                // Compare against last_mode_name and last_mode_param for change detection
                bool mode_name_changed = (strcmp(state->last_mode_name, state->mode_name) != 0);
                bool mode_param_changed = (strcmp(state->last_mode_param, state->mode_param) != 0);
                
                if (needs_full_redraw || state_changed || mode_name_changed || mode_param_changed) {
                    draw_video_mode_text_from_1d06(base_x, base_y, state->mode_name, state->mode_param);
                    // Update tracking fields after drawing
                    strncpy(state->last_mode_name, state->mode_name, sizeof(state->last_mode_name) - 1);
                    state->last_mode_name[sizeof(state->last_mode_name) - 1] = '\0';
                    strncpy(state->last_mode_param, state->mode_param, sizeof(state->last_mode_param) - 1);
                    state->last_mode_param[sizeof(state->last_mode_param) - 1] = '\0';
                }
            } else {
                // 1D02 mode: Use video_resolution, fps_idx, eis_mode for Video Mode Area
                // Use last_drawn_* fields (sentinel 0xFF forces first-draw) for change detection
                bool video_mode_changed = (state->last_drawn_video_resolution != state->video_resolution) ||
                                          (state->last_drawn_fps_idx != state->fps_idx) ||
                                          (state->last_drawn_eis_mode != state->eis_mode);
                
                if (needs_full_redraw || state_changed || video_mode_changed) {
                    draw_video_mode_text(base_x, base_y, state->video_resolution, state->fps_idx, state->eis_mode);
                    // Update tracking fields after drawing
                    state->last_drawn_video_resolution = state->video_resolution;
                    state->last_drawn_fps_idx = state->fps_idx;
                    state->last_drawn_eis_mode = state->eis_mode;
                }
            }
            
            // Draw status icon (recording/sleep state changed, full redraw, or state transition)
            // Use last_recording_state and last_sleep_state to track what was ACTUALLY DRAWN
            //
            // IMPORTANT: Capture state values BEFORE checking and drawing to prevent TOCTOU race.
            // The notification processing task may update is_recording/is_sleeping concurrently.
            // Without capturing, we could: check old value, draw old icon, but update tracking
            // with NEW value (changed by concurrent task), causing the icon to never update.
            bool current_is_sleeping = state->is_sleeping;
            bool current_is_recording = state->is_recording;
            
            // Use smart icon tracking - determine what icon should be shown now
            status_icon_type_t current_icon = determine_status_icon_type(camera_index, state->connection_state,
                                                                         current_is_sleeping, current_is_recording, false);
            bool icon_changed = (current_icon != state->last_drawn_status_icon);
            
            if (needs_full_redraw || state_changed || icon_changed) {
                // For connected cameras, found-in-scan is not relevant (always false)
                // draw_camera_status_icon() performs a partial redraw: clears only the icon region
                // (status_x, status_y, 90, 24) before redrawing, avoiding full-screen flicker
                draw_camera_status_icon(camera_index, base_x, base_y, state->connection_state, 
                                       current_is_sleeping, current_is_recording, false);
                // Update tracking fields with the CAPTURED values that were actually drawn
                // This ensures tracking matches what was rendered, even if state changed during draw
                state->last_recording_state = current_is_recording;
                state->last_sleep_state = current_is_sleeping;
                state->last_drawn_status_icon = current_icon;
                
                // Small delay to ensure display driver processes the icon draw.
                // This prevents timing issues when multiple cameras update simultaneously
                // in "All cameras" mode. Same pattern as draw_camera_selection_indicators().
                vTaskDelay(1);
            }
            
            // Draw recording time (updates every second for connected cameras)
            draw_recording_time(base_x, base_y, state->is_connected, state->is_recording,
                               state->record_time, state->remain_time, state);
            
            // Draw SD card status (only if capacity changed)
            draw_sd_card_status(base_x, base_y, state->remain_capacity, state);
            
            // Draw battery status (only if percentage changed)
            draw_battery_status(base_x, base_y, state->battery_percentage, state);
            break;
    }
    
    // Clear needs_full_redraw flag after drawing
    state->needs_full_redraw = false;
}

/**
 * @brief Draw GPS status area in the Remote Status Area
 * 
 * GPS Status Area is the right half of the Remote Status Area:
 * - GPS Base x = 160 (Remote Base x + 150)
 * - GPS Base y = 192 (Remote Base y)
 * - Width = 150, Height = 24
 * 
 * Contents:
 * - Icon: gps_icon (24x24) at (GPS Base x, GPS Base y)
 * - Text: Single line "lat, lon" with 2 decimal places at (GPS Base x + 34, GPS Base y + 4)
 * - Yellow when fix valid, red icon only when no fix
 * 
 * @param prev_lat Previous latitude value for change detection
 * @param prev_lon Previous longitude value for change detection
 * @param force_redraw Force full redraw (for initialization)
 */
static void draw_gps_status_area(float *prev_lat, float *prev_lon, bool force_redraw) {
    // Use layout constants for GPS Status Area
    int gps_x = GPS_STATUS_X;
    int gps_y = GPS_STATUS_Y;
    int icon_x = gps_x;
    int icon_y = gps_y;
    int text_x = gps_x + 34;  // Icon width (24) + 10px padding
    int text_y = gps_y + 4;   // Vertically centered for 24px height
    
    // Get GPS data and fix status
    gps_data_t gps_data;
    float latitude = 0.0f;
    float longitude = 0.0f;
    bool has_fix = gps_has_fix();
    
    if (has_fix && gps_get_data(&gps_data) == ESP_OK) {
        latitude = gps_data.latitude;
        longitude = gps_data.longitude;
    }
    
    // Check if GPS fix state changed
    bool fix_state_changed = (g_last_gps_has_fix != has_fix);
    
    // Check if GPS values changed (only relevant when we have fix)
    // Use 0.001 threshold to match 2 decimal place display
    bool coords_changed = has_fix && (
        (prev_lat == NULL) || (prev_lon == NULL) ||
        (fabsf(*prev_lat - latitude) > 0.001f) ||
        (fabsf(*prev_lon - longitude) > 0.001f)
    );
    
    if (force_redraw || fix_state_changed) {
        // Clear entire GPS area (24px height) and redraw icon with appropriate color
        ui_clear_line_region(gps_x, gps_y, GPS_STATUS_WIDTH, GPS_STATUS_HEIGHT);
        
        if (has_fix) {
            // GPS fix available: Draw yellow icon
            ui_draw_bitmap_inverted(icon_x, icon_y, gps_icon, 24, 24, M5_COLOR_YELLOW);
            // Force text redraw
            coords_changed = true;
        } else {
            // No GPS fix: Draw red icon only, no coordinates
            ui_draw_bitmap_inverted(icon_x, icon_y, gps_icon, 24, 24, M5_COLOR_RED);
        }
        
        g_last_gps_has_fix = has_fix;
    }
    
    // Draw coordinates only if we have GPS fix
    if (has_fix && coords_changed) {
        // Format single-line coordinates with 2 decimal places: "12.34, 56.78"
        char coords_str[24];
        snprintf(coords_str, sizeof(coords_str), "%.2f, %.2f", latitude, longitude);
        
        // Clear text area only (after icon), width = 150-34 = 116
        ui_clear_line_region(text_x, gps_y, 116, GPS_STATUS_HEIGHT);
        
        // Draw single-line coordinates in yellow
        m5stack_basic_v27_display_print(text_x, text_y, coords_str, M5_COLOR_YELLOW);
        
        // Update tracking values
        if (prev_lat != NULL) *prev_lat = latitude;
        if (prev_lon != NULL) *prev_lon = longitude;
    }
}


/**
 * @brief Cycle camera selection to next state
 * 
 * Called when Button B is pressed. Cycles through Camera 0 → 1 → 2 → All → 0.
 * Updates only the selection frame borders, not camera content.
 */
void ui_cycle_camera_selection(void) {
    // Save previous selection
    g_prev_camera_selection = g_camera_selection;
    
    // Cycle to next selection
    switch (g_camera_selection) {
        case CAMERA_SELECT_0:
            g_camera_selection = CAMERA_SELECT_1;
            break;
        case CAMERA_SELECT_1:
            g_camera_selection = CAMERA_SELECT_2;
            break;
        case CAMERA_SELECT_2:
            g_camera_selection = CAMERA_SELECT_ALL;
            break;
        case CAMERA_SELECT_ALL:
            g_camera_selection = CAMERA_SELECT_0;
            break;
    }
    
    ESP_LOGI(TAG, "Camera selection changed: %d -> %d", g_prev_camera_selection, g_camera_selection);
    
    // Trigger display update which will draw indicators with correct order
    g_ui_state.display_needs_update = true;
}

/**
 * @brief Start pairing mode for a camera slot
 */
void ui_start_pairing(int camera_index) {
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index for pairing: %d", camera_index);
        return;
    }
    
    ESP_LOGI(TAG, "Starting pairing for camera slot %d", camera_index);
    
    g_pairing_active_camera_index = camera_index;
    g_pairing_selection_index = 0;
    g_discovered_camera_count = 0;
    memset(g_discovered_cameras, 0, sizeof(g_discovered_cameras));
    
    g_ui_state.current_screen = SCREEN_PAIRING;
    g_ui_state.display_needs_update = true;
    
    // Start BLE scanning in PAIRING mode
    g_pairing_scan_active = true;
    esp_err_t ret = ble_start_scan(SCAN_MODE_PAIRING, camera_index, 30000);  // 30 second timeout
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE scanning for camera %d pairing", camera_index);
        g_pairing_scan_active = false;
    }
}

/**
 * @brief Callback for BLE scan results during pairing
 * Called by BLE layer when a device is discovered
 */
void ui_pairing_add_discovered_camera(const char *name, const uint8_t *mac, int8_t rssi, uint32_t device_id) {
    if (!g_pairing_scan_active) return;
    if (g_discovered_camera_count >= MAX_DISCOVERED_CAMERAS) return;
    
    // Check if already in list
    for (int i = 0; i < g_discovered_camera_count; i++) {
        if (memcmp(g_discovered_cameras[i].mac, mac, 6) == 0) {
            // Update RSSI if this is a stronger signal
            if (rssi > g_discovered_cameras[i].rssi) {
                g_discovered_cameras[i].rssi = rssi;
                g_ui_state.display_needs_update = true;
            }
            return;
        }
    }
    
    // Check if this camera is already paired in another slot
    for (int i = 0; i < NUM_CAMERAS; i++) {
        if (g_camera_states[i].is_paired && 
            memcmp(g_camera_states[i].camera_mac, mac, 6) == 0) {
            ESP_LOGD(TAG, "Camera %s already paired in slot %d, skipping", name, i);
            return;
        }
    }
    
    // Add new camera to list
    discovered_camera_t *cam = &g_discovered_cameras[g_discovered_camera_count];
    strncpy(cam->name, name, sizeof(cam->name) - 1);
    memcpy(cam->mac, mac, 6);
    cam->rssi = rssi;
    cam->device_id = device_id;
    cam->is_valid = true;
    
    g_discovered_camera_count++;
    g_ui_state.display_needs_update = true;
    
    ESP_LOGI(TAG, "Discovered camera for pairing: %s (RSSI: %d)", name, rssi);
}

/**
 * @brief Draw pairing screen for adding new camera to a slot
 */
/* Pairing screen state tracking for partial redraws */
static int g_pairing_prev_selection_index = 0;

/* Forward declaration for ui_return_to_main_screen (defined later in file) */
static void ui_return_to_main_screen(void);

/**
 * @brief Draw the pairing screen using unified Submenu layout
 * 
 * Pairing screen entries (5 total max):
 * 0: Back - back_icon + "Back"
 * 1-4: Discovered cameras - photo_icon + camera name (up to 4 cameras)
 */
static void ui_draw_pairing_screen(void) {
    // Clear screen
    m5stack_basic_v27_display_fill_rect(0, 0, 320, 240, M5_COLOR_BLACK);
    
    // Draw title "Camera X" (1-indexed)
    ui_draw_submenu_title(g_pairing_active_camera_index);
    
    // Entry 0: Back
    ui_draw_submenu_entry(0, (g_pairing_selection_index == 0), back_icon, "Back");
    
    // Draw discovered cameras (entries 1-4)
    if (g_discovered_camera_count == 0) {
        // Show scanning message in notification area
        ui_draw_submenu_notification("Scanning...", M5_COLOR_CYAN);
    } else {
        // Draw up to 4 discovered cameras (5 entries total including Back)
        int max_cameras = (g_discovered_camera_count < 4) ? g_discovered_camera_count : 4;
        for (int i = 0; i < max_cameras; i++) {
            if (!g_discovered_cameras[i].is_valid) continue;
            
            bool is_selected = (g_pairing_selection_index == (i + 1));
            
            // Truncate camera name to fit in entry
            char camera_name[24];
            strncpy(camera_name, g_discovered_cameras[i].name, sizeof(camera_name) - 1);
            camera_name[sizeof(camera_name) - 1] = '\0';
            
            ui_draw_submenu_entry(i + 1, is_selected, photo_icon, camera_name);
        }
    }
    
    // Draw submenu buttons (A: Select, B: Next, C: empty)
    ui_draw_submenu_buttons();
    
    // Save current selection for partial redraw tracking
    g_pairing_prev_selection_index = g_pairing_selection_index;
}

/**
 * @brief Handle button press on pairing screen
 */
static void ui_handle_pairing_screen_button_a(void) {
    if (g_pairing_selection_index == 0) {
        // Back selected - return to main screen
        g_pairing_scan_active = false;
        ble_stop_scan();  // Stop scanning using new API
        
        // Clear discovered cameras
        g_discovered_camera_count = 0;
        memset(g_discovered_cameras, 0, sizeof(g_discovered_cameras));
        
        ESP_LOGI(TAG, "Pairing cancelled, returning to main screen");
        ui_return_to_main_screen();
    } else {
        // Camera selected - pair it
        int camera_idx = g_pairing_selection_index - 1;
        if (camera_idx >= 0 && camera_idx < g_discovered_camera_count && 
            g_discovered_cameras[camera_idx].is_valid) {
            
            ESP_LOGI(TAG, "Pairing camera: %s to slot %d", 
                     g_discovered_cameras[camera_idx].name, 
                     g_pairing_active_camera_index);
            
            // Copy pairing info to camera state
            camera_state_t *cam = &g_camera_states[g_pairing_active_camera_index];
            cam->is_paired = true;
            strncpy(cam->camera_name, g_discovered_cameras[camera_idx].name, sizeof(cam->camera_name) - 1);
            cam->camera_name[sizeof(cam->camera_name) - 1] = '\0';
            memcpy(cam->camera_mac, g_discovered_cameras[camera_idx].mac, 6);
            cam->device_id = g_discovered_cameras[camera_idx].device_id;
            cam->connection_state = CAM_STATE_PAIRED_DISCONNECTED;
            cam->camera_reserved = g_pairing_active_camera_index;
            // Initialize 1D06 support and tracking fields for newly paired camera
            cam->camera_supports_new_status_push = false;  // Will be set to true if 1D06 is received
            cam->mode_name[0] = '\0';
            cam->mode_param[0] = '\0';
            cam->last_mode_name[0] = '\0';
            cam->last_mode_param[0] = '\0';
            // Set sentinel values for last_drawn_* to force first-draw
            cam->last_drawn_model_name[0] = '\0';
            cam->last_drawn_camera_mode = 0xFF;
            cam->last_drawn_video_resolution = 0xFF;
            cam->last_drawn_fps_idx = 0xFF;
            cam->last_drawn_eis_mode = 0xFF;
            cam->last_drawn_status_icon = STATUS_ICON_NONE;
            
            ESP_LOGI(TAG, "Camera %d paired: %s (MAC: %02X:%02X:%02X:%02X:%02X:%02X)",
                     g_pairing_active_camera_index, cam->camera_name,
                     cam->camera_mac[0], cam->camera_mac[1], cam->camera_mac[2],
                     cam->camera_mac[3], cam->camera_mac[4], cam->camera_mac[5]);
            
            // Save pairing to NVS
            esp_err_t save_err = save_all_cameras_to_nvs();
            if (save_err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save camera pairings to NVS: %s", esp_err_to_name(save_err));
            }
            
            // Sync to old structure for camera 0
            if (g_pairing_active_camera_index == 0) {
                g_stored_camera.is_paired = cam->is_paired;
                strncpy(g_stored_camera.camera_name, cam->camera_name, sizeof(g_stored_camera.camera_name));
                memcpy(g_stored_camera.camera_mac, cam->camera_mac, 6);
                g_stored_camera.device_id = cam->device_id;
                g_stored_camera.camera_reserved = cam->camera_reserved;
                // Other fields will be filled during handshake
            }
            
            cam->connection_state = CAM_STATE_CONNECTING;
            
            g_pairing_scan_active = false;
            ble_stop_scan();  // Stop scanning using new API
            
            // Clear discovered cameras
            g_discovered_camera_count = 0;
            memset(g_discovered_cameras, 0, sizeof(g_discovered_cameras));
            
            // Initiate connection for newly paired camera
            // This triggers the confirmation popup on the camera if needed
            ESP_LOGI(TAG, "Initiating connection to newly paired camera %d (will trigger camera confirmation)", g_pairing_active_camera_index);
            ui_draw_submenu_notification("Pairing... Confirm on Camera", M5_COLOR_CYAN);
            vTaskDelay(pdMS_TO_TICKS(2000));  // Let message display
            
            // Set pairing mode (requires manual confirmation on camera)
            g_verify_mode = 1;
            
            // Perform complete connection (BLE + protocol handshake)
            // This will block until connection succeeds or fails
            int connect_result = ui_perform_complete_reconnection(g_pairing_active_camera_index, true);
            
            if (connect_result == 0) {
                ESP_LOGI(TAG, "Pairing and connection successful for camera %d", g_pairing_active_camera_index);
                ui_draw_submenu_notification("Pairing Complete!", M5_COLOR_GREEN);
                vTaskDelay(pdMS_TO_TICKS(1500));
            } else {
                ESP_LOGW(TAG, "Pairing connection failed for camera %d", g_pairing_active_camera_index);
                cam->connection_state = CAM_STATE_PAIRED_DISCONNECTED;
                ui_draw_submenu_notification("Pairing Failed - Retry from Settings", M5_COLOR_RED);
                vTaskDelay(pdMS_TO_TICKS(2500));
            }
            
            // Return to main screen
            ui_return_to_main_screen();
        }
    }
}

/**
 * @brief Handle button B press on pairing screen
 * 
 * Cycles through selections using partial redraw (selection indicators only).
 * Selection wraps from entry 4 back to entry 0.
 */
static void ui_handle_pairing_screen_button_b(void) {
    // Save previous selection for partial redraw
    int prev_selection = g_pairing_selection_index;
    
    // Cycle through selections - up to discovered camera count + 1 (for Back)
    // Max 4 discovered cameras (entries 1-4) + Back (entry 0) = 5 entries total
    int max_selection = (g_discovered_camera_count < 4) ? g_discovered_camera_count : 4;
    g_pairing_selection_index++;
    if (g_pairing_selection_index > max_selection) {
        g_pairing_selection_index = 0;
    }
    
    // Use partial redraw for selection change (indicators only)
    ui_update_submenu_selection(prev_selection, g_pairing_selection_index);
}

/* ================== Camera Settings Screen ================== */

/**
 * @brief Start camera settings screen for a specific camera slot
 * 
 * @param camera_index Camera slot to configure (0, 1, or 2)
 */
void ui_start_settings(int camera_index) {
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index for settings: %d", camera_index);
        return;
    }
    
    if (!g_camera_states[camera_index].is_paired) {
        ESP_LOGW(TAG, "Cannot open settings for unpaired camera %d", camera_index);
        return;
    }
    
    ESP_LOGI(TAG, "Opening settings for camera %d", camera_index);
    
    g_settings_active_camera_index = camera_index;
    g_settings_selection_index = 0;  // Start with "Back" selected
    
    g_ui_state.current_screen = SCREEN_CAMERA_SETTINGS;
    g_ui_state.display_needs_update = true;
}

/**
 * @brief Draw the camera settings screen using unified Submenu layout
 * 
 * Settings screen entries:
 * 0: Back - back_icon + "Back"
 * 1: Highlight - highlight_icon + "Highlight"
 * 2: Sleep/Wakeup - sleep_icon/wakeup_icon + "Sleep"/"Wakeup"
 * 3: Connect/Disconnect - connect_icon/disconnect_icon + "Connect"/"Disconnect"
 * 4: Pair/Unpair - pair_icon/unpair_icon + "Pair Camera"/"Unpair Camera"
 */
static void ui_draw_settings_screen(void) {
    camera_state_t *cam = &g_camera_states[g_settings_active_camera_index];
    
    // Clear screen
    m5stack_basic_v27_display_fill_rect(0, 0, 320, 240, M5_COLOR_BLACK);
    
    // Draw title "Camera X" (1-indexed)
    ui_draw_submenu_title(g_settings_active_camera_index);
    
    // Entry 0: Back
    ui_draw_submenu_entry(0, (g_settings_selection_index == SETTINGS_ITEM_BACK), 
                          back_icon, "Back");
    
    // Entry 1: Mode Switch
    ui_draw_submenu_entry(1, (g_settings_selection_index == SETTINGS_ITEM_MODE_SWITCH),
                          switch_icon, "Mode Switch");
    
    // Entry 2: Sleep/Wakeup (dynamic based on camera state)
    // Use same sleep detection as snapshot mode for consistency
    bool is_sleeping = (cam->power_mode == 3) || cam->is_sleeping;
    if (is_sleeping) {
        ui_draw_submenu_entry(2, (g_settings_selection_index == SETTINGS_ITEM_SLEEP_WAKEUP),
                              wakeup_icon, "Wakeup");
    } else {
        ui_draw_submenu_entry(2, (g_settings_selection_index == SETTINGS_ITEM_SLEEP_WAKEUP),
                              sleep_icon, "Sleep");
    }
    
    // Entry 3: Connect/Disconnect (dynamic based on connection state)
    if (cam->connection_state == CAM_STATE_CONNECTED) {
        ui_draw_submenu_entry(3, (g_settings_selection_index == SETTINGS_ITEM_CONNECT_DISCONNECT),
                              disconnect_icon, "Disconnect");
    } else {
        ui_draw_submenu_entry(3, (g_settings_selection_index == SETTINGS_ITEM_CONNECT_DISCONNECT),
                              connect_icon, "Connect");
    }
    
    // Entry 4: Pair/Unpair (dynamic based on pairing state)
    if (cam->is_paired) {
        ui_draw_submenu_entry(4, (g_settings_selection_index == SETTINGS_ITEM_PAIR_UNPAIR),
                              unpair_icon, "Unpair");
    } else {
        ui_draw_submenu_entry(4, (g_settings_selection_index == SETTINGS_ITEM_PAIR_UNPAIR),
                              pair_icon, "Pair Camera");
    }
    
    // Draw submenu buttons (A: Select, B: Next, C: empty)
    ui_draw_submenu_buttons();
    
    // Save current selection for partial redraw tracking
    g_settings_prev_selection_index = g_settings_selection_index;
}

/**
 * @brief Return from settings/pairing screen to main screen
 * 
 * Helper function to properly reset state and trigger main screen redraw.
 */
static void ui_return_to_main_screen(void) {
    // Force main screen to redraw completely
    g_main_screen_initialized = false;
    g_buttons_area_initialized = false;
    for (int i = 0; i < NUM_CAMERAS; i++) {
        g_camera_states[i].needs_full_redraw = true;
        // Clear tracking fields to force redraw of all elements
        g_camera_states[i].last_drawn_model_name[0] = '\0';
        g_camera_states[i].last_drawn_camera_mode = 0xFF;
        g_camera_states[i].last_drawn_video_resolution = 0xFF;
        g_camera_states[i].last_drawn_fps_idx = 0xFF;
        g_camera_states[i].last_drawn_eis_mode = 0xFF;
        g_camera_states[i].last_drawn_status_icon = STATUS_ICON_NONE;
        g_camera_states[i].last_mode_name[0] = '\0';
        g_camera_states[i].last_mode_param[0] = '\0';
        g_camera_states[i].last_sd_display[0] = '\0';
        g_camera_states[i].last_battery_display[0] = '\0';
    }
    
    g_ui_state.current_screen = SCREEN_MAIN;
    g_ui_state.display_needs_update = true;
}

// ============================================================================
// MODE SWITCH SCREEN IMPLEMENTATION
// ============================================================================

/**
 * @brief Draw mode switch screen buttons
 * 
 * Button A: Switch (switch_icon)
 * Button B: Back (back_icon)
 * Button C: Blank (no function)
 */
static void ui_draw_mode_switch_buttons(void) {
    // Draw button backgrounds
    ui_draw_button_area_backgrounds();
    
    // Button A: Switch
    ui_draw_button_icon_and_label(0, switch_icon, M5_COLOR_BLACK, "Switch");
    
    // Button B: Back
    ui_draw_button_icon_and_label(1, back_icon, M5_COLOR_BLACK, "Back");
    
    // Button C: Empty (no function)
    ui_clear_button(2);
}

/**
 * @brief Draw camera mode icon for Mode Switch Screen
 * 
 * Draws the camera mode icon (photo/video) centered in the Mode Switch Screen
 * icon area (300px width).
 * 
 * @param camera_mode Camera mode value (0x05 = photo, others = video)
 */
static void draw_mode_switch_camera_mode_icon(uint8_t camera_mode) {
    // Mode Switch Icon area per UI Guide SECTION 10.2
    int icon_area_x = 10;       // Mode Base x
    int icon_area_y = 10 + 29;  // Mode Base y + 29
    int icon_area_width = 300;
    int icon_area_height = 24;
    
    // Center icon horizontally: (300 - 24) / 2 = 138
    int icon_x = icon_area_x + 138;
    int icon_y = icon_area_y;
    
    // Clear icon area
    ui_clear_line_region(icon_area_x, icon_area_y, icon_area_width, icon_area_height);
    
    // Draw icon based on mode
    if (camera_mode == 0x05) {
        // Photo mode
        ui_draw_bitmap_inverted(icon_x, icon_y, photo_icon, 24, 24, M5_COLOR_WHITE);
    } else {
        // Video mode (or other modes)
        ui_draw_bitmap_inverted(icon_x, icon_y, video_icon, 24, 24, M5_COLOR_WHITE);
    }
}

/**
 * @brief Draw video mode text for Mode Switch Screen (from 1D02)
 * 
 * Displays resolution on line 1 and FPS/EIS on line 2, centered in the
 * Mode Switch Label area.
 * 
 * @param resolution Video resolution value
 * @param fps_idx FPS index value
 * @param eis_mode EIS mode value
 */
static void draw_mode_switch_video_mode_text(uint8_t resolution, uint8_t fps_idx, uint8_t eis_mode) {
    // Mode Switch Label area per UI Guide SECTION 10.3
    int label_area_x = 10;       // Mode Base x
    int label_area_y = 10 + 58;  // Mode Base y + 58
    int label_area_width = 300;
    
    // Build resolution and FPS/EIS strings
    const char* res_str = ui_get_short_resolution_string(resolution);
    const char* fps_str = fps_idx_to_string((fps_idx_t)fps_idx);
    const char* eis_str = eis_mode_to_string((eis_mode_t)eis_mode);
    
    // Format two-line display
    char line1[32] = "";
    char line2[32] = "";
    
    if (strlen(res_str) > 0) {
        strncpy(line1, res_str, sizeof(line1) - 1);
        line1[sizeof(line1) - 1] = '\0';
    }
    snprintf(line2, sizeof(line2), "%s %s", fps_str, eis_str);
    
    // Clear label area (only top portion for video mode text)
    ui_clear_line_region(label_area_x, label_area_y, label_area_width, 40);
    
    // Center text horizontally using larger font (size 2)
    int line1_width = ui_get_text_width(line1, 2);
    int line2_width = ui_get_text_width(line2, 2);
    int line1_x = label_area_x + (label_area_width - line1_width) / 2;
    int line2_x = label_area_x + (label_area_width - line2_width) / 2;
    
    // Draw two lines with larger font (scale=2), vertically centered in area
    int text_y_offset = 10;  // Padding from top of label area
    if (strlen(line1) > 0) {
        m5stack_basic_v27_display_print_scaled(line1_x, label_area_y + text_y_offset, line1, M5_COLOR_WHITE, 2);
    }
    m5stack_basic_v27_display_print_scaled(line2_x, label_area_y + text_y_offset + 20, line2, M5_COLOR_WHITE, 2);
}

/**
 * @brief Draw video mode text for Mode Switch Screen (from 1D06)
 * 
 * Displays mode_name on line 1 and mode_param on line 2, centered in the
 * Mode Switch Label area.
 * 
 * @param mode_name Mode name string from 1D06 (max 20 chars)
 * @param mode_param Mode parameter string from 1D06 (max 20 chars)
 */
static void draw_mode_switch_video_mode_text_from_1d06(const char *mode_name, const char *mode_param) {
    // Mode Switch Label area per UI Guide SECTION 10.3
    int label_area_x = 10;       // Mode Base x
    int label_area_y = 10 + 58;  // Mode Base y + 58
    int label_area_width = 300;
    
    // Truncate strings if too long
    char line1[21] = "";
    char line2[21] = "";
    
    if (mode_name != NULL && strlen(mode_name) > 0) {
        strncpy(line1, mode_name, sizeof(line1) - 1);
        line1[sizeof(line1) - 1] = '\0';
    }
    
    if (mode_param != NULL && strlen(mode_param) > 0) {
        strncpy(line2, mode_param, sizeof(line2) - 1);
        line2[sizeof(line2) - 1] = '\0';
    }
    
    // Clear label area (only top portion for video mode text)
    ui_clear_line_region(label_area_x, label_area_y, label_area_width, 40);
    
    // Center text horizontally using larger font (size 2)
    int line1_width = ui_get_text_width(line1, 2);
    int line2_width = ui_get_text_width(line2, 2);
    int line1_x = label_area_x + (label_area_width - line1_width) / 2;
    int line2_x = label_area_x + (label_area_width - line2_width) / 2;
    
    // Draw two lines with larger font (scale=2)
    int text_y_offset = 10;  // Padding from top of label area
    if (strlen(line1) > 0) {
        m5stack_basic_v27_display_print_scaled(line1_x, label_area_y + text_y_offset, line1, M5_COLOR_WHITE, 2);
    }
    if (strlen(line2) > 0) {
        m5stack_basic_v27_display_print_scaled(line2_x, label_area_y + text_y_offset + 20, line2, M5_COLOR_WHITE, 2);
    }
}

/**
 * @brief Perform full draw of the Mode Switch Screen
 * 
 * Called on first entry to the screen. Clears screen and draws all elements.
 */
static void ui_draw_mode_switch_screen_full(void) {
    camera_state_t *cam = &g_camera_states[g_mode_switch_camera_index];
    
    // Clear screen
    m5stack_basic_v27_display_fill_rect(0, 0, 320, 240, M5_COLOR_BLACK);
    
    // Draw title "Camera X" (1-indexed) per UI Guide SECTION 10.1
    // Title area: x=10, y=10, width=300, height=24
    char title[32];
    snprintf(title, sizeof(title), "Camera %d", g_mode_switch_camera_index + 1);
    int title_width = ui_get_text_width(title, 2);
    int title_x = 10 + (300 - title_width) / 2;  // Center in 300px width
    int title_y = 10 + 4;  // Vertically centered in 24px height
    m5stack_basic_v27_display_print_scaled(title_x, title_y, title, M5_COLOR_WHITE, 2);
    
    // Draw Camera Mode icon per UI Guide SECTION 10.2
    draw_mode_switch_camera_mode_icon(cam->camera_mode);
    
    // Draw Video Mode text per UI Guide SECTION 10.3
    // Use 1D06 data if camera supports it, otherwise use 1D02 data
    if (cam->camera_supports_new_status_push) {
        draw_mode_switch_video_mode_text_from_1d06(cam->mode_name, cam->mode_param);
    } else {
        draw_mode_switch_video_mode_text(cam->video_resolution, cam->fps_idx, cam->eis_mode);
    }
    
    // Draw buttons (A: Switch, B: Back, C: Blank)
    ui_draw_mode_switch_buttons();
    
    // Update Mode Switch Screen tracking fields (separate from main screen tracking)
    g_mode_switch_last_camera_mode = cam->camera_mode;
    if (cam->camera_supports_new_status_push) {
        strncpy(g_mode_switch_last_mode_name, cam->mode_name, sizeof(g_mode_switch_last_mode_name) - 1);
        g_mode_switch_last_mode_name[sizeof(g_mode_switch_last_mode_name) - 1] = '\0';
        strncpy(g_mode_switch_last_mode_param, cam->mode_param, sizeof(g_mode_switch_last_mode_param) - 1);
        g_mode_switch_last_mode_param[sizeof(g_mode_switch_last_mode_param) - 1] = '\0';
    } else {
        g_mode_switch_last_resolution = cam->video_resolution;
        g_mode_switch_last_fps_idx = cam->fps_idx;
        g_mode_switch_last_eis_mode = cam->eis_mode;
    }
    
    g_mode_switch_screen_initialized = true;
    
    ESP_LOGI(TAG, "Mode Switch Screen FULL draw for camera %d (mode_name='%s', mode_param='%s')", 
             g_mode_switch_camera_index + 1,
             cam->camera_supports_new_status_push ? cam->mode_name : "N/A",
             cam->camera_supports_new_status_push ? cam->mode_param : "N/A");
}

/**
 * @brief Update the Mode Switch Screen with partial redraws
 * 
 * Only redraws elements that have actually changed. Compares current camera
 * state against Mode Switch Screen-specific tracking fields.
 */
static void ui_update_mode_switch_screen(void) {
    camera_state_t *cam = &g_camera_states[g_mode_switch_camera_index];
    
    // Check if camera mode icon needs update
    bool camera_mode_changed = (cam->camera_mode != g_mode_switch_last_camera_mode);
    
    // Check if video mode text needs update (depends on camera type)
    bool video_mode_changed = false;
    if (cam->camera_supports_new_status_push) {
        video_mode_changed = (strcmp(cam->mode_name, g_mode_switch_last_mode_name) != 0) ||
                             (strcmp(cam->mode_param, g_mode_switch_last_mode_param) != 0);
    } else {
        video_mode_changed = (cam->video_resolution != g_mode_switch_last_resolution) ||
                             (cam->fps_idx != g_mode_switch_last_fps_idx) ||
                             (cam->eis_mode != g_mode_switch_last_eis_mode);
    }
    
    // Only log and redraw if something actually changed
    if (!camera_mode_changed && !video_mode_changed) {
        return;  // Nothing to update
    }
    
    ESP_LOGI(TAG, "Mode Switch Screen UPDATE for camera %d: mode_changed=%d, video_changed=%d",
             g_mode_switch_camera_index + 1, camera_mode_changed, video_mode_changed);
    
    // Redraw camera mode icon if changed
    if (camera_mode_changed) {
        ESP_LOGD(TAG, "Mode Switch: Camera mode icon changed from 0x%02X to 0x%02X",
                 g_mode_switch_last_camera_mode, cam->camera_mode);
        draw_mode_switch_camera_mode_icon(cam->camera_mode);
        g_mode_switch_last_camera_mode = cam->camera_mode;
    }
    
    // Redraw video mode text if changed
    if (video_mode_changed) {
        if (cam->camera_supports_new_status_push) {
            ESP_LOGI(TAG, "Mode Switch: Video mode updated: '%s' / '%s'",
                     cam->mode_name, cam->mode_param);
            draw_mode_switch_video_mode_text_from_1d06(cam->mode_name, cam->mode_param);
            strncpy(g_mode_switch_last_mode_name, cam->mode_name, sizeof(g_mode_switch_last_mode_name) - 1);
            g_mode_switch_last_mode_name[sizeof(g_mode_switch_last_mode_name) - 1] = '\0';
            strncpy(g_mode_switch_last_mode_param, cam->mode_param, sizeof(g_mode_switch_last_mode_param) - 1);
            g_mode_switch_last_mode_param[sizeof(g_mode_switch_last_mode_param) - 1] = '\0';
        } else {
            ESP_LOGD(TAG, "Mode Switch: Video mode (1D02) updated: res=0x%02X fps=0x%02X eis=0x%02X",
                     cam->video_resolution, cam->fps_idx, cam->eis_mode);
            draw_mode_switch_video_mode_text(cam->video_resolution, cam->fps_idx, cam->eis_mode);
            g_mode_switch_last_resolution = cam->video_resolution;
            g_mode_switch_last_fps_idx = cam->fps_idx;
            g_mode_switch_last_eis_mode = cam->eis_mode;
        }
    }
}

/**
 * @brief Draw or update the Mode Switch Screen
 * 
 * Decides whether to do a full draw or partial update based on
 * whether the screen was already initialized.
 */
static void ui_draw_mode_switch_screen(void) {
    if (!g_mode_switch_screen_initialized) {
        // First time drawing - do full draw
        ui_draw_mode_switch_screen_full();
    } else {
        // Screen already drawn - only update changed elements
        ui_update_mode_switch_screen();
    }
}

/**
 * @brief Handle button A press on Mode Switch Screen
 * 
 * Sends QS button short press command to switch camera mode.
 * Does NOT return to main screen - stays on Mode Switch Screen
 * to allow repeated mode switching.
 */
static void ui_handle_mode_switch_button_a(void) {
    ESP_LOGI(TAG, "Mode Switch: Switch button pressed for camera %d", g_mode_switch_camera_index + 1);
    
    // Check if camera is connected and awake
    camera_state_t *cam = &g_camera_states[g_mode_switch_camera_index];
    if (!cam->is_connected) {
        ESP_LOGW(TAG, "Mode Switch: Camera %d not connected", g_mode_switch_camera_index + 1);
        return;
    }
    
    bool is_sleeping = (cam->power_mode == 3) || cam->is_sleeping;
    if (is_sleeping) {
        ESP_LOGW(TAG, "Mode Switch: Camera %d is sleeping", g_mode_switch_camera_index + 1);
        return;
    }
    
    // Send QS button short press command (key_code=0x02, mode=0x01, key_value=0x00)
    esp_err_t ret = command_logic_send_key_report_for_slot(g_mode_switch_camera_index, 0x02, 0x01, 0x00);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Mode Switch: QS command sent to camera %d", g_mode_switch_camera_index + 1);
    } else {
        ESP_LOGW(TAG, "Mode Switch: QS command failed for camera %d: %s", 
                 g_mode_switch_camera_index + 1, esp_err_to_name(ret));
    }
    
    // Stay on Mode Switch Screen - do NOT return to main screen
    // The camera mode/video mode will update via status push notifications
}

/**
 * @brief Handle button B press on Mode Switch Screen
 * 
 * Returns directly to Main Screen (not Settings Screen).
 */
static void ui_handle_mode_switch_button_b(void) {
    ESP_LOGI(TAG, "Mode Switch: Back button pressed, returning to Main Screen");
    // Reset Mode Switch Screen tracking for next entry
    g_mode_switch_screen_initialized = false;
    ui_return_to_main_screen();
}

// ============================================================================
// END MODE SWITCH SCREEN IMPLEMENTATION
// ============================================================================

/**
 * @brief Handle button A press on settings screen
 */
static void ui_handle_settings_screen_button_a(void) {
    camera_state_t *cam = &g_camera_states[g_settings_active_camera_index];
    
    switch (g_settings_selection_index) {
        case SETTINGS_ITEM_BACK:
            // Return to main screen
            ESP_LOGI(TAG, "Settings: Back selected");
            ui_return_to_main_screen();
            break;
            
        case SETTINGS_ITEM_MODE_SWITCH:
            // Open Mode Switch Screen for the current camera slot
            ESP_LOGI(TAG, "Settings: Mode Switch selected for camera %d", g_settings_active_camera_index + 1);
            g_mode_switch_camera_index = g_settings_active_camera_index;
            // Reset tracking fields to force full redraw on entry
            g_mode_switch_screen_initialized = false;
            g_mode_switch_last_camera_mode = 0xFF;
            g_mode_switch_last_mode_name[0] = '\0';
            g_mode_switch_last_mode_param[0] = '\0';
            g_mode_switch_last_resolution = 0xFF;
            g_mode_switch_last_fps_idx = 0xFF;
            g_mode_switch_last_eis_mode = 0xFF;
            g_ui_state.current_screen = SCREEN_MODE_SWITCH;
            g_ui_state.display_needs_update = true;
            break;
            
        case SETTINGS_ITEM_SLEEP_WAKEUP: {
            ESP_LOGI(TAG, "Settings: Sleep/Wakeup selected");
            // Use same sleep detection as snapshot mode for consistency
            bool is_sleeping = (cam->power_mode == 3) || cam->is_sleeping;
            
            if (is_sleeping) {
                // Wake up the camera using the same broadcast as snapshot mode
                ui_draw_submenu_notification("Waking up...", M5_COLOR_CYAN);
                
                esp_err_t wake_result = connect_logic_start_wake_broadcast_for_slot(g_settings_active_camera_index);
                if (wake_result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to start wake broadcast: %s", esp_err_to_name(wake_result));
                    ui_draw_submenu_notification("Wake failed", M5_COLOR_RED);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                } else {
                    // Wait for wake broadcast to complete (3 seconds for better reliability)
                    vTaskDelay(pdMS_TO_TICKS(3500));
                    ui_draw_submenu_notification("Wake broadcast sent", M5_COLOR_GREEN);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            } else {
                // Put camera to sleep
                ui_draw_submenu_notification("Sleeping...", M5_COLOR_CYAN);
                command_logic_power_mode_switch_sleep(g_settings_active_camera_index);
                vTaskDelay(pdMS_TO_TICKS(1000));
                ui_draw_submenu_notification("Sleep command sent", M5_COLOR_GREEN);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            ui_return_to_main_screen();
            break;
        }
            
        case SETTINGS_ITEM_CONNECT_DISCONNECT:
            if (cam->connection_state == CAM_STATE_CONNECTED) {
                // Disconnect
                ESP_LOGI(TAG, "Settings: Disconnecting camera %d", g_settings_active_camera_index);
                ui_draw_submenu_notification("Disconnecting...", M5_COLOR_YELLOW);
                
                // Disconnect BLE
                ble_disconnect(g_settings_active_camera_index);
                
                // Update state
                cam->connection_state = CAM_STATE_PAIRED_DISCONNECTED;
                cam->is_connected = false;
                cam->is_initialized = false;
                cam->needs_full_redraw = true;
                
                vTaskDelay(pdMS_TO_TICKS(1000));
                ui_draw_submenu_notification("Disconnected", M5_COLOR_YELLOW);
                vTaskDelay(pdMS_TO_TICKS(1000));
                ui_return_to_main_screen();
            } else {
                // Connect
                ESP_LOGI(TAG, "Settings: Connecting camera %d", g_settings_active_camera_index);
                ui_draw_submenu_notification("Connecting...", M5_COLOR_BLUE);
                
                vTaskDelay(pdMS_TO_TICKS(1000));
                
                // Attempt reconnection
                int result = ui_perform_complete_reconnection(g_settings_active_camera_index, true);
                
                if (result == 0) {
                    ESP_LOGI(TAG, "Connection successful");
                    ui_draw_submenu_notification("Connected!", M5_COLOR_GREEN);
                } else {
                    ESP_LOGW(TAG, "Connection failed");
                    ui_draw_submenu_notification("Connection failed", M5_COLOR_RED);
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                ui_return_to_main_screen();
            }
            break;
            
        case SETTINGS_ITEM_PAIR_UNPAIR:
            if (cam->is_paired) {
                // Unpair the camera
                ESP_LOGI(TAG, "Settings: Unpairing camera %d", g_settings_active_camera_index);
                
                // First disconnect if connected
                if (cam->connection_state == CAM_STATE_CONNECTED) {
                    ESP_LOGI(TAG, "Camera %d is connected, disconnecting before unpairing", g_settings_active_camera_index);
                    ui_draw_submenu_notification("Disconnecting...", M5_COLOR_YELLOW);
                    
                    ble_disconnect(g_settings_active_camera_index);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                
                // Now delete the pairing
                ui_draw_submenu_notification("Unpairing...", M5_COLOR_RED);
                vTaskDelay(pdMS_TO_TICKS(1000));
                
                // Clear pairing data
                memset(cam, 0, sizeof(camera_state_t));
                cam->connection_state = CAM_STATE_UNPAIRED;
                cam->is_paired = false;
                cam->needs_full_redraw = true;
                // Set sentinel values for last_drawn_* tracking fields
                cam->last_drawn_model_name[0] = '\0';
                cam->last_drawn_camera_mode = 0xFF;
                cam->last_drawn_video_resolution = 0xFF;
                cam->last_drawn_fps_idx = 0xFF;
                cam->last_drawn_eis_mode = 0xFF;
                cam->last_drawn_status_icon = STATUS_ICON_NONE;
                
                // Save to NVS
                esp_err_t save_result = save_all_cameras_to_nvs();
                if (save_result == ESP_OK) {
                    ESP_LOGI(TAG, "Pairing deleted and saved to NVS");
                } else {
                    ESP_LOGW(TAG, "Failed to save after unpairing");
                }
                
                ui_draw_submenu_notification("Camera unpaired", M5_COLOR_RED);
                vTaskDelay(pdMS_TO_TICKS(1000));
                ui_return_to_main_screen();
            } else {
                // Switch to pairing screen for this slot
                ESP_LOGI(TAG, "Settings: Switching to pairing screen for slot %d", g_settings_active_camera_index);
                ui_start_pairing(g_settings_active_camera_index);
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief Handle button B press on settings screen
 * 
 * Cycles through selections using partial redraw (selection indicators only).
 */
static void ui_handle_settings_screen_button_b(void) {
    // Save previous selection for partial redraw
    int prev_selection = g_settings_selection_index;
    
    // Cycle through selections
    g_settings_selection_index++;
    if (g_settings_selection_index >= SETTINGS_ITEM_COUNT) {
        g_settings_selection_index = 0;
    }
    
    // Use partial redraw for selection change (indicators only)
    ui_update_submenu_selection(prev_selection, g_settings_selection_index);
}

/**
 * @brief Update display with current UI state
 * 
 * Renders the complete user interface including:
 * - Connection status indicator
 * - Current screen icon and text
 * - Screen navigation dots
 * - Instruction text
 * 
 * Only updates when display_needs_update flag is set for efficiency.
 * For Shutter screen, uses selective updates to only redraw changed elements.
 */
void ui_update_display(void) {
    if (!g_ui_state.display_needs_update) {
        return;
    }
    
    // Route to appropriate screen renderer
    if (g_ui_state.current_screen == SCREEN_PAIRING) {
        ui_draw_pairing_screen();
        g_ui_state.display_needs_update = false;
        return;
    }
    
    if (g_ui_state.current_screen == SCREEN_CAMERA_SETTINGS) {
        ui_draw_settings_screen();
        g_ui_state.display_needs_update = false;
        return;
    }
    
    if (g_ui_state.current_screen == SCREEN_MODE_SWITCH) {
        ui_draw_mode_switch_screen();
        g_ui_state.display_needs_update = false;
        return;
    }
    
    /* Display Main Screen with 3-camera layout */
    // Handle first-time initialization
    bool is_first_draw = !g_main_screen_initialized;
    if (is_first_draw) {
        // Clear entire screen for full redraw
        m5stack_basic_v27_display_fill_rect(0, 0, 320, 240, M5_COLOR_BLACK);
        
        // Mark all camera states as needing full redraw
        for (int i = 0; i < 3; i++) {
            g_camera_states[i].needs_full_redraw = true;
            // Clear previous state strings
            g_camera_states[i].last_sd_display[0] = '\0';
            g_camera_states[i].last_battery_display[0] = '\0';
            g_camera_states[i].last_time_value = 0;
        }
        
        g_main_screen_initialized = true;
    }
    
    // Draw Buttons Area backgrounds on first draw
    if (is_first_draw || !g_buttons_area_initialized) {
        // Clear the entire buttons row to black first (including edges and gaps between buttons)
        // This ensures no grey artifacts appear from uninitialized display memory
        m5stack_basic_v27_display_fill_rect(0, BUTTONS_BASE_Y, 320, BUTTONS_HEIGHT, M5_COLOR_BLACK);
        ui_draw_button_area_backgrounds();
    }
    
    // Draw GPS status area (with selective update for value changes)
    draw_gps_status_area(&g_last_gps_lat, &g_last_gps_lon, is_first_draw);
    
    // Draw camera blocks (all three cameras)
    // Note: We pass g_camera_states[i] directly, not a copy, so that tracking fields
    // (last_drawn_*, last_mode_name, last_mode_param) are updated in the persistent state.
    for (int i = 0; i < NUM_CAMERAS; i++) {  // Render all three cameras
        // Sync camera 0 with old globals if needed
        if (i == 0 && !g_camera_states[0].is_initialized) {
            camera_state_t temp_state;
            get_camera_state_for_index(0, &temp_state);
            // Copy sync data back to g_camera_states[0]
            g_camera_states[0].camera_mode = temp_state.camera_mode;
            g_camera_states[0].camera_status = temp_state.camera_status;
            g_camera_states[0].video_resolution = temp_state.video_resolution;
            g_camera_states[0].fps_idx = temp_state.fps_idx;
            g_camera_states[0].eis_mode = temp_state.eis_mode;
            g_camera_states[0].is_recording = temp_state.is_recording;
            g_camera_states[0].record_time = temp_state.record_time;
            g_camera_states[0].remain_time = temp_state.remain_time;
            g_camera_states[0].remain_capacity = temp_state.remain_capacity;
            g_camera_states[0].camera_bat_percentage = temp_state.camera_bat_percentage;
            g_camera_states[0].battery_percentage = temp_state.battery_percentage;
            g_camera_states[0].power_mode = temp_state.power_mode;
            g_camera_states[0].is_sleeping = temp_state.is_sleeping;
        }
        
        // Determine base coordinates for this camera block
        int base_x = 10 + (i * 100);  // Camera 0: 10, Camera 1: 110, Camera 2: 210
        int base_y = 10;
        
        // Draw camera block with selective updates
        // Pass g_camera_states[i] directly so tracking fields are updated in place
        draw_camera_block(i, base_x, base_y, &g_camera_states[i], NULL);
    }
    
    // Draw static white frames around all camera blocks on first draw
    if (is_first_draw) {
        draw_camera_frame(CAMERA_0_FRAME_X, CAMERA_0_FRAME_Y, 
                         CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT, M5_COLOR_WHITE);
        draw_camera_frame(CAMERA_1_FRAME_X, CAMERA_1_FRAME_Y, 
                         CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT, M5_COLOR_WHITE);
        draw_camera_frame(CAMERA_2_FRAME_X, CAMERA_2_FRAME_Y, 
                         CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT, M5_COLOR_WHITE);
    }
    
    // Draw selection indicators only when selection changes or on first draw
    // This avoids redrawing indicators during recording time updates which can cause glitches
    static camera_selection_t s_last_drawn_selection = (camera_selection_t)-1;  // Invalid initial value
    if (is_first_draw || g_camera_selection != s_last_drawn_selection) {
        draw_camera_selection_indicators(g_camera_selection);
        s_last_drawn_selection = g_camera_selection;
    }
    
    // Update button icons and labels based on current state and boot scan/connect phase
    // This function handles three modes:
    // - Boot scan active: only Button C "Stop" visible
    // - Boot connect in progress: all buttons hidden
    // - Normal operation: full A/B/C with dynamic mappings
    ui_update_main_screen_buttons_for_state();
    
    g_ui_state.display_needs_update = false;
    
    ESP_LOGD(TAG, "Display updated - Main Screen");
}

/**
 * @brief Navigate to next screen in sequence
 * 
 * Cycles through available screens: Connect → Shutter → Mode → Sleep → Wake → Connect...
 * Called when Button B is pressed.
 */
/**
 * @brief Handle Button A press (screen-aware)
 */
void ui_handle_button_a(void) {
    switch (g_ui_state.current_screen) {
        case SCREEN_MAIN:
            ui_screen_main();
            g_ui_state.display_needs_update = true;
            break;
        case SCREEN_PAIRING:
            ui_handle_pairing_screen_button_a();
            break;
        case SCREEN_CAMERA_SETTINGS:
            ui_handle_settings_screen_button_a();
            break;
        case SCREEN_MODE_SWITCH:
            ui_handle_mode_switch_button_a();
            break;
        default:
            break;
    }
}

/**
 * @brief Handle Button B press (screen-aware)
 */
void ui_handle_button_b(void) {
    switch (g_ui_state.current_screen) {
        case SCREEN_MAIN:
            ui_cycle_camera_selection();
            break;
        case SCREEN_PAIRING:
            ui_handle_pairing_screen_button_b();
            break;
        case SCREEN_CAMERA_SETTINGS:
            ui_handle_settings_screen_button_b();
            break;
        case SCREEN_MODE_SWITCH:
            ui_handle_mode_switch_button_b();
            break;
        default:
            break;
    }
}

/**
 * @brief Handle Button C press (screen-aware)
 * Button C opens pairing or settings screen from main screen
 * Special behavior: During active boot scan, stops scan and connects to found cameras
 * Highlight Tag: If camera is recording and highlight-capable, sends highlight tag command
 */
void ui_handle_button_c(void) {
    if (g_ui_state.current_screen != SCREEN_MAIN) {
        return;  // Button C only works on main screen
    }
    
    // PRECEDENCE 1: Special behavior - If boot scan is active, stop it and connect to found cameras
    // This must happen FIRST before any highlight tag logic
    if (connect_logic_is_boot_scan_active()) {
        ESP_LOGI(TAG, "Button C pressed during boot scan - stopping scan and connecting to found cameras");
        connect_logic_stop_boot_scan_and_connect_found();
        return;  // Early return - don't proceed to highlight tag or pairing/settings logic
    }
    
    extern camera_selection_t g_camera_selection;
    
    // PRECEDENCE 2: Single camera slot selected (0, 1, or 2)
    if (g_camera_selection != CAMERA_SELECT_ALL) {
        // Get the selected camera index
        int cam_idx = (int)g_camera_selection;  // CAMERA_SELECT_0=0, etc.
        
        // Check if all conditions are met for highlight tag:
        // - paired, connected, awake, recording, highlight-supported
        if (command_logic_slot_is_paired(cam_idx) &&
            command_logic_slot_is_connected(cam_idx) &&
            command_logic_slot_is_awake(cam_idx) &&
            command_logic_slot_is_recording(cam_idx) &&
            command_logic_slot_supports_highlight(cam_idx)) {
            
            // Send highlight tag command
            esp_err_t ret = command_logic_send_highlight_for_slot(cam_idx);
            if (ret == ESP_OK) {
                // Show notification
                ui_show_shutter_bottom_message("Highlight tag sent", M5_COLOR_GREEN, 1500);
            } else {
                // Show error notification
                ui_show_shutter_bottom_message("Highlight failed", M5_COLOR_RED, 1500);
            }
            // Do NOT change screen - remain on Main Shutter Screen
            return;  // Early return - don't open Settings/Pairing
        }
        
        // Conditions not met - fall back to existing behavior
        if (g_camera_states[cam_idx].connection_state == CAM_STATE_UNPAIRED) {
            // Open pairing screen
            ui_start_pairing(cam_idx);
        } else {
            // Open settings screen
            ui_start_settings(cam_idx);
        }
        return;
    }
    
    // PRECEDENCE 3: "All Cameras" selection
    if (g_camera_selection == CAMERA_SELECT_ALL) {
        // Count how many cameras meet all conditions for highlight tag
        int highlight_eligible_count = 0;
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (command_logic_slot_is_paired(i) &&
                command_logic_slot_is_connected(i) &&
                command_logic_slot_is_awake(i) &&
                command_logic_slot_is_recording(i) &&
                command_logic_slot_supports_highlight(i)) {
                highlight_eligible_count++;
            }
        }
        
        if (highlight_eligible_count > 0) {
            // Highlight takes priority - send highlight to all eligible cameras
            esp_err_t ret = command_logic_send_highlight_for_all_active();
            if (ret == ESP_OK) {
                // Show notification with count
                char msg[32];
                snprintf(msg, sizeof(msg), "Highlight sent to %d", highlight_eligible_count);
                ui_show_shutter_bottom_message(msg, M5_COLOR_GREEN, 1500);
            } else {
                ui_show_shutter_bottom_message("Highlight failed", M5_COLOR_RED, 1500);
            }
            // Do NOT change screen - remain on Main Shutter Screen
            return;  // Early return - highlight was sent
        }
        
        // No highlight-eligible cameras - check for sleep candidates
        // Sleep candidates are: paired, connected, awake, NOT recording
        int sleep_candidates_count = 0;
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (is_camera_sleep_candidate(i)) {
                sleep_candidates_count++;
            }
        }
        
        if (sleep_candidates_count > 0) {
            // Send sleep command to all sleep-candidate cameras
            ESP_LOGI(TAG, "Button C: Sending sleep to %d cameras in All Cameras mode", sleep_candidates_count);
            int sent_count = 0;
            for (int i = 0; i < NUM_CAMERAS; i++) {
                if (is_camera_sleep_candidate(i)) {
                    // Use the same sleep command as Settings screen
                    camera_power_mode_switch_response_frame_t* response = 
                        command_logic_power_mode_switch_sleep(i);
                    if (response != NULL) {
                        sent_count++;
                        ESP_LOGI(TAG, "Sleep command sent to camera %d", i);
                        // Free the response structure (allocated by send_command)
                        free(response);
                    } else {
                        ESP_LOGW(TAG, "Sleep command failed for camera %d", i);
                    }
                }
            }
            
            // Show notification with count
            if (sent_count > 0) {
                char msg[32];
                snprintf(msg, sizeof(msg), "Sleep sent to %d", sent_count);
                ui_show_shutter_bottom_message(msg, M5_COLOR_GREEN, 1500);
            } else {
                ui_show_shutter_bottom_message("Sleep failed", M5_COLOR_RED, 1500);
            }
            // Do NOT change screen - remain on Main Shutter Screen
            return;  // Early return - sleep commands were sent
        }
        
        // No sleep candidates - check for wake candidates (sleeping cameras)
        int wake_candidates_count = 0;
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (is_camera_wake_candidate(i)) {
                wake_candidates_count++;
            }
        }
        
        if (wake_candidates_count > 0) {
            // Add all sleeping cameras to wake queue for serialized processing
            // NOTE: BLE can only run one advertising at a time, so we MUST use the
            // serialized wake queue. Starting multiple broadcasts in a loop would
            // cause each subsequent broadcast to overwrite the previous one.
            ESP_LOGI(TAG, "Button C: Adding %d cameras to wake queue (wake-only mode)", wake_candidates_count);
            
            // Clear the wake queue first in case there are stale entries
            wake_queue_clear();
            
            int queued_count = 0;
            for (int i = 0; i < NUM_CAMERAS; i++) {
                if (is_camera_wake_candidate(i)) {
                    // Add to wake queue but do NOT set snapshot_pending
                    // This is wake-only mode, not snapshot mode
                    if (wake_queue_add(i)) {
                        queued_count++;
                        ESP_LOGI(TAG, "Camera %d added to wake queue (wake-only, no snapshot)", i);
                    } else {
                        ESP_LOGW(TAG, "Failed to add camera %d to wake queue", i);
                    }
                }
            }
            
            // Start queue processing if we added any cameras
            if (queued_count > 0) {
                g_wake_queue.queue_active = true;
                ESP_LOGI(TAG, "Wake queue has %d camera(s), starting serialized wake-only processing", queued_count);
                
                char msg[32];
                snprintf(msg, sizeof(msg), "Waking %d cameras...", queued_count);
                ui_show_shutter_bottom_message(msg, M5_COLOR_YELLOW, 2000);
            } else {
                ui_show_shutter_bottom_message("Wakeup failed", M5_COLOR_RED, 1500);
            }
            // Do NOT change screen - remain on Main Shutter Screen
            return;  // Early return - wake queue was started
        }
        
        // No highlight, no sleep, no wake candidates - do nothing
        return;
    }
}

/**
 * @brief Execute main screen button action
 * 
 * Wrapper function used for GPIO triggers.
 */
void ui_execute_main_screen_action(void) {
    ui_handle_button_a();
}

/**
 * @brief Display temporary message with automatic timeout
 * 
 * Shows a status or feedback message for a specified duration, then
 * returns to normal UI display. Used for operation feedback.
 * 
 * @param message Text to display
 * @param color Text color (RGB565 format)
 * @param duration_ms Display duration in milliseconds
 */
void ui_show_message(const char* message, uint16_t color, int duration_ms) {
    m5stack_basic_v27_display_clear(M5_COLOR_BLACK);
    
    /* Calculate message position */
    int msg_x = 30;
    int msg_y = 40;
    
    m5stack_basic_v27_display_print(msg_x, msg_y, message, color);
    
    /* Block for specified duration (simple implementation) */
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    /* Schedule normal UI restore */
    g_ui_state.display_needs_update = true;
}

/**
 * @brief Show message in notification area on the Main Shutter Screen
 * 
 * Displays a temporary message in the Notification Area (left half of Remote Status Area).
 * Notification Area: x=10, y=192, width=150, height=24
 * Used for command feedback on Shutter screen to maintain camera status visibility.
 * Only clears and redraws the notification region, never the full screen.
 * 
 * @param message Text to display
 * @param color Text color (RGB565 format)
 * @param duration_ms Display duration in milliseconds
 */
void ui_show_shutter_bottom_message(const char* message, uint16_t color, int duration_ms) {
    // Use layout constants for Notification Area
    int msg_x = NOTIFICATION_X;
    int msg_y = NOTIFICATION_Y;
    int msg_width = NOTIFICATION_WIDTH;
    int msg_height = NOTIFICATION_HEIGHT;
    
    // Clear notification area only
    ui_clear_line_region(msg_x, msg_y, msg_width, msg_height);
    
    // Truncate message to fit within width (approximate 8 pixels per character)
    char truncated[20];
    strncpy(truncated, message, sizeof(truncated) - 1);
    truncated[sizeof(truncated) - 1] = '\0';
    
    // Clip text to available width
    int text_width = ui_get_text_width(truncated, 1);
    if (text_width > msg_width) {
        // Truncate further if needed
        int chars_to_keep = msg_width / 8;
        if (chars_to_keep > 0 && chars_to_keep < sizeof(truncated)) {
            truncated[chars_to_keep] = '\0';
        }
    }
    
    // Draw message vertically centered in notification area (y + 4 for 24px height)
    m5stack_basic_v27_display_print(msg_x + 5, msg_y + 4, truncated, color);
    
    // Wait for specified duration
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    // Clear notification area again
    ui_clear_line_region(msg_x, msg_y, msg_width, msg_height);
}

/**
 * @brief Display "not connected" error message
 * 
 * Convenience function to show connection error when user attempts
 * camera operations without an active connection.
 */
void ui_show_not_connected_message(void) {
    ui_show_shutter_bottom_message("Not Connected!", M5_COLOR_RED, 1500);
}

/**
 * @brief Attempt manual camera pairing as fallback
 * 
 * Called when automatic reconnection fails, initiating a fresh pairing
 * process that requires user interaction on the camera side.
 */
/**
 * @brief Handle main screen activation (shutter control)
 * 
 * Controls photo capture and video recording based on current camera mode:
 * - Photo mode: Always takes a photo
 * - Video modes: Toggles recording start/stop
 * 
 * Requires active camera connection to function.
 */
void ui_screen_main(void) {
    ESP_LOGI(TAG, "Executing main screen action (shutter control)");
    
    // Check which camera(s) are selected
    if (g_camera_selection == CAMERA_SELECT_ALL) {
        // "All cameras" mode - control all connected cameras
        ESP_LOGI(TAG, "All cameras mode - controlling all connected cameras");
        
        // Determine if any connected camera is recording
        bool any_recording = false;
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (g_camera_states[i].connection_state == CAM_STATE_CONNECTED && g_camera_states[i].is_recording) {
                any_recording = true;
                break;
            }
        }
        
        // Toggle: if any are recording, stop all; otherwise start all
        // Use async (non-blocking) commands for better synchronization
        if (any_recording) {
            // Stop all connected cameras that are recording
            ESP_LOGI(TAG, "Stopping all recording cameras (synchronized)");
            int cameras_stopped = 0;
            for (int i = 0; i < NUM_CAMERAS; i++) {
                if (g_camera_states[i].connection_state == CAM_STATE_CONNECTED && g_camera_states[i].is_recording) {
                    ESP_LOGI(TAG, "Sending stop command to camera %d (non-blocking)", i);
                    command_logic_stop_record_async(i);
                    cameras_stopped++;
                }
            }
            ESP_LOGI(TAG, "Sent stop commands to %d cameras (commands sent back-to-back for better sync)", cameras_stopped);
            ui_show_shutter_bottom_message("Stopping All", M5_COLOR_YELLOW, 1000);
        } else {
            // Start all connected cameras (awake cameras get recording command, sleeping cameras queued for snapshot)
            ESP_LOGI(TAG, "Starting all connected cameras (synchronized)");
            int cameras_started = 0;
            int cameras_queued = 0;
            
            // First pass: Process all non-sleeping cameras immediately
            for (int i = 0; i < NUM_CAMERAS; i++) {
                camera_state_t *cam_state = &g_camera_states[i];
                
                // Only process connected cameras
                if (cam_state->connection_state != CAM_STATE_CONNECTED) {
                    continue;
                }
                
                // Check if camera is sleeping
                bool is_sleeping = (cam_state->power_mode == 3) || cam_state->is_sleeping;
                
                if (!is_sleeping) {
                    // Camera is awake - send normal start recording command
                    ESP_LOGI(TAG, "Sending start command to camera %d (non-blocking)", i);
                    command_logic_start_record_async(i);
                    cameras_started++;
                }
            }
            
            // Second pass: Queue all sleeping cameras for serialized wake-up
            for (int i = 0; i < NUM_CAMERAS; i++) {
                camera_state_t *cam_state = &g_camera_states[i];
                
                // Only process connected cameras
                if (cam_state->connection_state != CAM_STATE_CONNECTED) {
                    continue;
                }
                
                // Check if camera is sleeping
                bool is_sleeping = (cam_state->power_mode == 3) || cam_state->is_sleeping;
                
                if (is_sleeping) {
                    // Camera is sleeping - add to wake queue for snapshot mode
                    if (cam_state->snapshot_pending) {
                        ESP_LOGW(TAG, "Camera %d: Snapshot already pending, skipping duplicate request", i);
                        continue;
                    }
                    
                    // Add to wake queue
                    if (wake_queue_add(i)) {
                        // Set snapshot pending flag and record timestamp
                        cam_state->snapshot_pending = true;
                        cam_state->last_snapshot_request_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        cam_state->wake_retry_count = 0;  // Reset retry counter for new request
                        cameras_queued++;
                        ESP_LOGI(TAG, "Camera %d: snapshot_pending SET (All Cameras mode, added to wake queue)", i);
                        ESP_LOGD(TAG, "Camera %d: last_snapshot_request_ms = %lu", i, cam_state->last_snapshot_request_ms);
                    } else {
                        ESP_LOGW(TAG, "Failed to add camera %d to wake queue", i);
                    }
                }
            }
            
            // Start queue processing if queue is not empty
            if (!wake_queue_is_empty()) {
                g_wake_queue.queue_active = true;
                ESP_LOGI(TAG, "Wake queue has %d camera(s), starting serialized wake-up processing", wake_queue_get_count());
            }
            
            // Show appropriate notification
            if (cameras_queued > 0 && cameras_started > 0) {
                char msg[48];
                snprintf(msg, sizeof(msg), "Starting %d, Queued %d", cameras_started, cameras_queued);
                ui_show_shutter_bottom_message(msg, M5_COLOR_GREEN, 1500);
            } else if (cameras_queued > 0) {
                char msg[32];
                snprintf(msg, sizeof(msg), "Queued %d camera(s)", cameras_queued);
                ui_show_shutter_bottom_message(msg, M5_COLOR_YELLOW, 1500);
            } else if (cameras_started > 0) {
                ESP_LOGI(TAG, "Sent start commands to %d cameras (commands sent back-to-back for better sync)", cameras_started);
                ui_show_shutter_bottom_message("Starting All", M5_COLOR_GREEN, 1000);
            } else {
                ui_show_shutter_bottom_message("No cameras ready", M5_COLOR_ORANGE, 1000);
            }
        }
        return;
    }
    
    // Single camera selected - get the camera index
    int cam_idx = (int)g_camera_selection;  // CAMERA_SELECT_0=0, etc.
    ESP_LOGI(TAG, "Single camera mode - camera %d selected", cam_idx);
    
    // Check if this camera is paired - if not, open Pairing Screen
    if (g_camera_states[cam_idx].connection_state == CAM_STATE_UNPAIRED) {
        ESP_LOGI(TAG, "Camera %d is not paired - opening Pairing Screen", cam_idx);
        ui_start_pairing(cam_idx);
        return;
    }
    
    // Check if this camera is connected
    if (g_camera_states[cam_idx].connection_state != CAM_STATE_CONNECTED) {
        // Camera is paired but not connected - attempt to reconnect
        if (g_camera_states[cam_idx].connection_state == CAM_STATE_PAIRED_DISCONNECTED || 
            g_camera_states[cam_idx].connection_state == CAM_STATE_CONNECTING) {
            ESP_LOGI(TAG, "Camera %d is paired but disconnected - attempting reconnect", cam_idx);
            ui_show_shutter_bottom_message("Reconnecting...", M5_COLOR_CYAN, 1000);
            
            // Attempt reconnection (same as Settings Screen)
            int result = ui_perform_complete_reconnection(cam_idx, true);
            
            if (result == 0) {
                ESP_LOGI(TAG, "Reconnection successful for camera %d", cam_idx);
                ui_show_shutter_bottom_message("Connected!", M5_COLOR_GREEN, 1500);
            } else {
                ESP_LOGW(TAG, "Reconnection failed for camera %d", cam_idx);
                ui_show_shutter_bottom_message("Reconnect Failed", M5_COLOR_RED, 1500);
            }
            return;
        } else {
            // Unknown connection state
            ESP_LOGW(TAG, "Camera %d is in unknown connection state: %d", cam_idx, g_camera_states[cam_idx].connection_state);
            ui_show_shutter_bottom_message("Not Connected", M5_COLOR_RED, 1500);
            return;
        }
    }
    
    // Get camera state for the selected camera
    camera_state_t *cam_state = &g_camera_states[cam_idx];
    
    // Check if camera is in sleep mode (use per-camera state)
    bool is_sleeping = (cam_state->power_mode == 3) || cam_state->is_sleeping;
    
    if (is_sleeping) {
        // Camera is asleep - initiate snapshot mode
        // Single camera mode: send wake broadcast immediately, snapshot key is sent
        // automatically when camera wakes (power_mode 3 → 0) via status update handler
        ESP_LOGI(TAG, "LOGIC_STATUS: Camera %d: Snapshot requested while sleeping. Starting wake-up broadcast.", cam_idx);
        
        // Check if snapshot is already pending (avoid stacking multiple requests)
        if (cam_state->snapshot_pending) {
            ESP_LOGW(TAG, "Camera %d: snapshot_pending already set, ignoring duplicate request", cam_idx);
            return;
        }
        
        // Set snapshot pending flag and record timestamp
        cam_state->snapshot_pending = true;
        cam_state->last_snapshot_request_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        cam_state->wake_retry_count = 0;  // Reset retry counter for new request
        ESP_LOGI(TAG, "Camera %d: snapshot_pending SET (single camera mode)", cam_idx);
        ESP_LOGD(TAG, "Camera %d: last_snapshot_request_ms = %lu", cam_idx, cam_state->last_snapshot_request_ms);
        
        // Start wake-up broadcast for this camera slot
        extern esp_err_t connect_logic_start_wake_broadcast_for_slot(int camera_index);
        esp_err_t wake_result = connect_logic_start_wake_broadcast_for_slot(cam_idx);
        if (wake_result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start wake broadcast for camera %d, aborting snapshot request: %s", 
                     cam_idx, esp_err_to_name(wake_result));
            // Clear snapshot pending since we couldn't initiate the wake sequence
            cam_state->snapshot_pending = false;
            ESP_LOGD(TAG, "Camera %d: snapshot_pending cleared (wake broadcast failed to start)", cam_idx);
            ui_show_shutter_bottom_message("Snapshot failed: wake error", M5_COLOR_RED, 2000);
            return;
        }
        
        // Show notification that we're waking the camera for snapshot
        char msg[32];
        snprintf(msg, sizeof(msg), "Waking camera %d for snapshot...", cam_idx);
        ui_show_shutter_bottom_message(msg, M5_COLOR_YELLOW, 2000);
        
        // Mark UI for update to reflect any state changes
        g_ui_state.display_needs_update = true;
        
        // Don't send recording command - snapshot key will be sent automatically when camera wakes
        // (handled in status update code when power_mode changes from 3 to 0)
        return;
    }
    
    // Camera is awake - proceed with normal recording logic
    
    /* Check if camera status has been received for mode-aware operation */
    if (!cam_state->is_initialized) {
        ESP_LOGW(TAG, "Camera %d status not yet initialized, attempting command anyway", cam_idx);
        ui_show_shutter_bottom_message("Status Unknown\nTrying...", M5_COLOR_ORANGE, 1000);
    }
    
    /* Determine shutter behavior based on current camera mode */
    camera_mode_t current_mode = (camera_mode_t)cam_state->camera_mode;
    
    ESP_LOGI(TAG, "Current camera mode: %d, status: %d, recording: %s", 
             current_mode, cam_state->camera_status, cam_state->is_recording ? "yes" : "no");
    
    if (current_mode == CAMERA_MODE_PHOTO) {
        /* Photo mode - single shot capture */
        ESP_LOGI(TAG, "Taking photo in photo mode");
        record_control_response_frame_t* response = command_logic_start_record(cam_idx);
        if (response) {
            ui_show_shutter_bottom_message("Photo Taken", M5_COLOR_GREEN, 1000);
            free(response);
        } else {
            ui_show_shutter_bottom_message("Photo Failed", M5_COLOR_RED, 1500);
        }
    } else {
        /* Video modes - toggle recording state */
        bool is_recording = cam_state->is_recording;
        
        if (is_recording) {
            /* Stop current recording */
            ESP_LOGI(TAG, "Stopping recording in video mode");
            record_control_response_frame_t* response = command_logic_stop_record(cam_idx);
            if (response) {
                ui_show_shutter_bottom_message("Recording Stopped", M5_COLOR_YELLOW, 1000);
                free(response);
            } else {
                ui_show_shutter_bottom_message("Stop Failed", M5_COLOR_RED, 1500);
            }
        } else {
            /* Start new recording */
            ESP_LOGI(TAG, "Starting recording in video mode");
            record_control_response_frame_t* response = command_logic_start_record(cam_idx);
            if (response) {
                ui_show_shutter_bottom_message("Recording Started", M5_COLOR_GREEN, 1000);
                free(response);
            } else {
                ui_show_shutter_bottom_message("Start Failed", M5_COLOR_RED, 1500);
            }
        }
    }
}

/**
 * @brief Process wake-and-record state machine
 * 
 * This function manages the wake-and-record state machine and should be called
 * from the main loop every cycle. It handles the timing and confirmation logic
 * for waking up a sleeping camera and starting recording after confirmation.
 * 
 * State Machine:
 * - WAKE_STATE_IDLE: No wake-up in progress
 * - WAKE_STATE_BROADCASTING: Broadcasting wake packet for 2 seconds
 * - WAKE_STATE_WAITING_CONFIRMATION: Waiting for camera status updates to confirm wake
 * - WAKE_STATE_READY: Camera confirmed awake, send recording command
 * 
 * Called from main loop in app_main.c
 */
void ui_process_wake_and_record(void) {
    // Only process if we're not in IDLE state
    if (g_wake_state == WAKE_STATE_IDLE) {
        return;
    }
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    switch (g_wake_state) {
        case WAKE_STATE_BROADCASTING:
            // Wait for 3-second broadcast to complete (increased from 2s for reliability)
            if (current_time - g_wake_broadcast_start_time >= 3000) {
                ESP_LOGI(TAG, "Wake broadcast completed (3s elapsed), waiting for camera confirmation");
                g_wake_state = WAKE_STATE_WAITING_CONFIRMATION;
                g_wake_timeout_start_time = current_time;
            }
            break;
            
        case WAKE_STATE_WAITING_CONFIRMATION:
            // Check if camera has woken up (power_mode changed from 3 to 0)
            if (current_power_mode != 3) {
                // Camera is no longer in sleep mode
                // Verify camera is actively communicating (status push is recent)
                uint32_t status_age = current_time - g_last_status_push_timestamp;
                
                if (status_age < 1000) {
                    // Status push is fresh (< 1 second old), camera is confirmed awake
                    ESP_LOGI(TAG, "Camera wake confirmed (power_mode=%d, status_age=%lu ms), ready to send recording command",
                             current_power_mode, status_age);
                    g_wake_state = WAKE_STATE_READY;
                } else {
                    // Status push is stale, keep waiting
                    ESP_LOGD(TAG, "Camera reports awake but status push is stale (%lu ms), waiting for fresh status",
                             status_age);
                }
            }
            
            // Check for timeout (6 seconds total: 3s broadcast + 3s confirmation)
            if (current_time - g_wake_broadcast_start_time >= 6000) {
                ESP_LOGW(TAG, "Wake-and-record timeout (6s elapsed), camera did not respond");
                ui_show_shutter_bottom_message("Wake Timeout", M5_COLOR_RED, 2000);
                g_wake_state = WAKE_STATE_IDLE;
                g_camera_states[0].pending_start_recording = false;
            }
            break;
            
        case WAKE_STATE_READY:
            // Camera is confirmed awake, send recording command
            if (g_camera_states[0].pending_start_recording) {
                ESP_LOGI(TAG, "Sending start recording command after wake-up for camera 0");
                
                record_control_response_frame_t* response = command_logic_start_record(0);
                if (response) {
                    ESP_LOGI(TAG, "Recording command sent successfully after wake-up");
                    ui_show_shutter_bottom_message("Recording Started", M5_COLOR_GREEN, 1000);
                    free(response);
                } else {
                    ESP_LOGE(TAG, "Failed to send recording command after wake-up");
                    ui_show_shutter_bottom_message("Start Failed", M5_COLOR_RED, 1500);
                }
                
                // Clear pending flag and reset state
                g_camera_states[0].pending_start_recording = false;
                g_wake_state = WAKE_STATE_IDLE;
                g_ui_state.display_needs_update = true;
            } else {
                // Shouldn't happen, but handle gracefully
                ESP_LOGW(TAG, "Wake state READY but no pending recording, resetting to IDLE");
                g_wake_state = WAKE_STATE_IDLE;
            }
            break;
            
        default:
            // Invalid state, reset to IDLE
            ESP_LOGW(TAG, "Invalid wake state %d, resetting to IDLE", g_wake_state);
            g_wake_state = WAKE_STATE_IDLE;
            break;
    }
}

/* ================== Wake-Up Queue Management ================== */

/**
 * @brief Initialize wake-up queue to empty state
 */
void wake_queue_init(void) {
    memset(&g_wake_queue, 0, sizeof(wake_queue_t));
    g_wake_queue_state = WAKE_QUEUE_IDLE;
    g_current_wake_camera_index = -1;
    g_wake_broadcast_start_time_ms = 0;
}

/**
 * @brief Add camera to wake-up queue
 * 
 * @param camera_index Camera slot index (0-2) to add to queue
 * @return true if camera was added, false if queue is full or camera already in queue
 */
bool wake_queue_add(int camera_index) {
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index for wake queue: %d", camera_index);
        return false;
    }
    
    // Check if queue is full
    if (g_wake_queue.queue_count >= NUM_CAMERAS) {
        ESP_LOGW(TAG, "Wake queue is full, cannot add camera %d", camera_index);
        return false;
    }
    
    // Check if camera is already in queue
    for (int i = 0; i < g_wake_queue.queue_count; i++) {
        int idx = (g_wake_queue.queue_head + i) % NUM_CAMERAS;
        if (g_wake_queue.camera_indices[idx] == camera_index) {
            ESP_LOGD(TAG, "Camera %d already in wake queue, skipping", camera_index);
            return false;
        }
    }
    
    // Add camera to queue
    g_wake_queue.camera_indices[g_wake_queue.queue_tail] = camera_index;
    g_wake_queue.queue_tail = (g_wake_queue.queue_tail + 1) % NUM_CAMERAS;
    g_wake_queue.queue_count++;
    
    ESP_LOGI(TAG, "Camera %d added to wake queue (queue count: %d)", camera_index, g_wake_queue.queue_count);
    return true;
}

/**
 * @brief Get next camera index from wake-up queue
 * 
 * @return Camera index (-1 if queue is empty)
 */
int wake_queue_get_next(void) {
    if (g_wake_queue.queue_count == 0) {
        return -1;
    }
    
    int camera_index = g_wake_queue.camera_indices[g_wake_queue.queue_head];
    g_wake_queue.queue_head = (g_wake_queue.queue_head + 1) % NUM_CAMERAS;
    g_wake_queue.queue_count--;
    
    ESP_LOGI(TAG, "Camera %d removed from wake queue (remaining: %d)", camera_index, g_wake_queue.queue_count);
    return camera_index;
}

/**
 * @brief Clear entire wake-up queue
 */
void wake_queue_clear(void) {
    g_wake_queue.queue_head = 0;
    g_wake_queue.queue_tail = 0;
    g_wake_queue.queue_count = 0;
    g_wake_queue.queue_active = false;
    g_current_wake_camera_index = -1;
    ESP_LOGI(TAG, "Wake queue cleared");
}

/**
 * @brief Check if wake-up queue is empty
 * 
 * @return true if queue is empty, false otherwise
 */
bool wake_queue_is_empty(void) {
    return g_wake_queue.queue_count == 0;
}

/**
 * @brief Get number of cameras in wake-up queue
 * 
 * @return Number of cameras in queue
 */
int wake_queue_get_count(void) {
    return g_wake_queue.queue_count;
}

/**
 * @brief Notify wake queue that a camera has woken up
 * 
 * This function is called by status_logic.c when it detects a power_mode
 * transition from 3 (sleep) to 0 (normal) for a camera that is currently
 * being processed by the wake queue. This allows early termination of the
 * broadcast and immediate progression to the next camera.
 * 
 * NOTE: The snapshot key is sent by status_logic.c, not by this queue processor.
 * This function only handles queue state management and broadcast termination.
 * 
 * @param camera_index Camera slot index (0-2) that woke up
 */
void wake_queue_notify_camera_woke_up(int camera_index) {
    if (g_current_wake_camera_index != camera_index) {
        ESP_LOGD(TAG, "wake_queue_notify_camera_woke_up: Camera %d is not the current queue camera (%d), ignoring",
                 camera_index, g_current_wake_camera_index);
        return;
    }
    
    if (g_wake_queue_state != WAKE_QUEUE_BROADCASTING) {
        ESP_LOGD(TAG, "wake_queue_notify_camera_woke_up: Not in BROADCASTING state (%d), ignoring",
                 g_wake_queue_state);
        return;
    }
    
    ESP_LOGI(TAG, "Wake queue: Camera %d woke up early, stopping broadcast and moving to next camera", camera_index);
    
    // Stop broadcast early
    extern esp_err_t ble_stop_advertising_early(void);
    ble_stop_advertising_early();
    
    // Move to next camera (snapshot key was already sent by status_logic.c)
    g_current_wake_camera_index = -1;
    g_wake_queue_state = WAKE_QUEUE_IDLE;
}

/**
 * @brief Process wake-up queue serially
 * 
 * This function manages the serialized wake-up queue for "All Cameras" mode.
 * It processes one camera at a time, handling wake broadcast timing and timeouts.
 * 
 * IMPORTANT: The snapshot key is ALWAYS sent by status_logic.c when it detects
 * power_mode transition from 3 to 0. This queue processor only:
 * - Starts wake broadcasts for sleeping cameras
 * - Monitors for early wake-up (to stop broadcast early)
 * - Handles timeouts (clears snapshot_pending if camera doesn't wake)
 * - Manages queue progression
 * 
 * State Machine:
 * - WAKE_QUEUE_IDLE: No wake-up in progress, check for next camera in queue
 * - WAKE_QUEUE_BROADCASTING: Broadcasting wake packet, waiting for wake-up or timeout
 * 
 * Called from main loop in app_main.c
 */
void ui_process_wake_queue(void) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    const uint32_t BROADCAST_TIMEOUT_MS = 3000;  // 3 seconds for wake broadcast (increased from 2s for reliability)
    
    switch (g_wake_queue_state) {
        case WAKE_QUEUE_IDLE:
            // Check if queue has cameras to process
            if (!wake_queue_is_empty() && g_wake_queue.queue_active) {
                // Get next camera from queue
                int camera_index = wake_queue_get_next();
                if (camera_index >= 0) {
                    camera_state_t *cam_state = &g_camera_states[camera_index];
                    
                    // Verify camera is still connected
                    if (cam_state->connection_state != CAM_STATE_CONNECTED) {
                        ESP_LOGW(TAG, "Wake queue: Camera %d no longer connected, clearing snapshot_pending and skipping", camera_index);
                        cam_state->snapshot_pending = false;
                        ESP_LOGD(TAG, "Wake queue: Camera %d snapshot_pending cleared (disconnected)", camera_index);
                        // Continue to next camera (stay in IDLE, will process next iteration)
                        break;
                    }
                    
                    // Check if camera is sleeping
                    bool is_sleeping = (cam_state->power_mode == 3) || cam_state->is_sleeping;
                    if (!is_sleeping) {
                        // Camera already awake - status_logic.c should have already sent snapshot key
                        // if snapshot_pending was true. Just clear and move on.
                        ESP_LOGI(TAG, "Wake queue: Camera %d already awake, skipping (snapshot handled by status logic)", camera_index);
                        if (cam_state->snapshot_pending) {
                            // This shouldn't happen if status_logic.c is working correctly,
                            // but clear it as a safety net
                            ESP_LOGW(TAG, "Wake queue: Camera %d has stale snapshot_pending, clearing", camera_index);
                            cam_state->snapshot_pending = false;
                        }
                        // Continue to next camera (stay in IDLE, will process next iteration)
                        break;
                    }
                    
                    // Start wake broadcast for this camera
                    ESP_LOGI(TAG, "Wake queue: Starting broadcast for camera %d (snapshot_pending=%d)", 
                             camera_index, cam_state->snapshot_pending);
                    extern esp_err_t connect_logic_start_wake_broadcast_for_slot(int camera_index);
                    esp_err_t wake_result = connect_logic_start_wake_broadcast_for_slot(camera_index);
                    if (wake_result != ESP_OK) {
                        ESP_LOGE(TAG, "Wake queue: Failed to start broadcast for camera %d: %s", 
                                 camera_index, esp_err_to_name(wake_result));
                        cam_state->snapshot_pending = false;
                        ESP_LOGD(TAG, "Wake queue: Camera %d snapshot_pending cleared (broadcast failed)", camera_index);
                        // Continue to next camera (stay in IDLE, will process next iteration)
                        break;
                    }
                    
                    // Set state to broadcasting
                    g_wake_queue_state = WAKE_QUEUE_BROADCASTING;
                    g_current_wake_camera_index = camera_index;
                    g_wake_broadcast_start_time_ms = current_time;
                    ESP_LOGI(TAG, "Wake queue: Broadcast started for camera %d at time %lu ms", 
                             camera_index, g_wake_broadcast_start_time_ms);
                }
            } else if (wake_queue_is_empty() && g_wake_queue.queue_active) {
                // Queue is empty, deactivate
                g_wake_queue.queue_active = false;
                g_current_wake_camera_index = -1;
                ESP_LOGI(TAG, "Wake queue: Processing completed, queue deactivated");
            }
            break;
            
        case WAKE_QUEUE_BROADCASTING:
            // Check if camera has woken up (for early broadcast stop)
            // This works for both snapshot mode AND wake-only mode:
            // - Snapshot mode: status_logic.c clears snapshot_pending when camera wakes
            // - Wake-only mode: snapshot_pending is always false, detect via power_mode only
            if (g_current_wake_camera_index >= 0) {
                camera_state_t *cam_state = &g_camera_states[g_current_wake_camera_index];
                bool is_still_sleeping = (cam_state->power_mode == 3) || cam_state->is_sleeping;
                
                // Camera woke up if it's no longer sleeping
                // For snapshot mode: snapshot_pending will have been cleared by status_logic.c
                // For wake-only mode: snapshot_pending was never set (always false)
                if (!is_still_sleeping) {
                    // Camera woke up - stop broadcast early and move to next
                    if (cam_state->snapshot_pending) {
                        // Snapshot mode: status_logic.c should have cleared this, but it hasn't yet
                        // This means we detected wake before status_logic.c processed the 1D02
                        // Just log it - status_logic.c will handle the snapshot when it runs
                        ESP_LOGI(TAG, "Wake queue: Camera %d woke up (snapshot_pending still set, status_logic will handle)", 
                                 g_current_wake_camera_index);
                    } else {
                        // Either wake-only mode OR status_logic.c already handled the snapshot
                        ESP_LOGI(TAG, "Wake queue: Camera %d woke up (wake-only or snapshot already handled), moving to next", 
                                 g_current_wake_camera_index);
                    }
                    
                    // Stop broadcast (may have already stopped)
                    extern esp_err_t ble_stop_advertising_early(void);
                    ble_stop_advertising_early();
                    
                    // Reset retry counter on successful wake
                    cam_state->wake_retry_count = 0;
                    
                    // Move to next camera
                    g_current_wake_camera_index = -1;
                    g_wake_queue_state = WAKE_QUEUE_IDLE;
                    break;
                }
            }
            
            // Check if 3 seconds have elapsed (broadcast timeout)
            if (current_time - g_wake_broadcast_start_time_ms >= BROADCAST_TIMEOUT_MS) {
                ESP_LOGI(TAG, "Wake queue: Broadcast timeout (3s) for camera %d", g_current_wake_camera_index);
                
                // Check if camera is still sleeping after timeout
                if (g_current_wake_camera_index >= 0) {
                    camera_state_t *cam_state = &g_camera_states[g_current_wake_camera_index];
                    bool is_still_sleeping = (cam_state->power_mode == 3) || cam_state->is_sleeping;
                    const uint8_t MAX_QUEUE_WAKE_RETRIES = 1;  // Allow 1 retry per camera in queue
                    
                    if (is_still_sleeping) {
                        // Camera didn't wake up within timeout - check if we should retry
                        if (cam_state->wake_retry_count < MAX_QUEUE_WAKE_RETRIES) {
                            // Retry: restart wake broadcast for the same camera
                            cam_state->wake_retry_count++;
                            ESP_LOGW(TAG, "Wake queue: Camera %d still sleeping, retrying wake broadcast (attempt %d)", 
                                     g_current_wake_camera_index, cam_state->wake_retry_count + 1);
                            
                            // Stop current advertising and restart
                            extern esp_err_t ble_stop_advertising_early(void);
                            ble_stop_advertising_early();
                            
                            esp_err_t wake_result = connect_logic_start_wake_broadcast_for_slot(g_current_wake_camera_index);
                            if (wake_result == ESP_OK) {
                                // Reset broadcast start time for retry
                                g_wake_broadcast_start_time_ms = current_time;
                                ESP_LOGI(TAG, "Wake queue: Retry broadcast started for camera %d at time %lu ms", 
                                         g_current_wake_camera_index, g_wake_broadcast_start_time_ms);
                                // Stay in WAKE_QUEUE_BROADCASTING state
                            } else {
                                // Failed to start retry broadcast - give up on this camera
                                ESP_LOGE(TAG, "Wake queue: Failed to start retry broadcast for camera %d: %s", 
                                         g_current_wake_camera_index, esp_err_to_name(wake_result));
                                if (cam_state->snapshot_pending) {
                                    cam_state->snapshot_pending = false;
                                }
                                cam_state->wake_retry_count = 0;
                                g_current_wake_camera_index = -1;
                                g_wake_queue_state = WAKE_QUEUE_IDLE;
                            }
                        } else {
                            // Max retries exceeded - give up on this camera
                            if (cam_state->snapshot_pending) {
                                ESP_LOGW(TAG, "Wake queue: Camera %d did not wake up after %d attempts (snapshot mode), clearing snapshot_pending", 
                                         g_current_wake_camera_index, cam_state->wake_retry_count + 1);
                                cam_state->snapshot_pending = false;
                            } else {
                                ESP_LOGW(TAG, "Wake queue: Camera %d did not wake up after %d attempts (wake-only mode)", 
                                         g_current_wake_camera_index, cam_state->wake_retry_count + 1);
                            }
                            cam_state->wake_retry_count = 0;  // Reset for next time
                            g_current_wake_camera_index = -1;
                            g_wake_queue_state = WAKE_QUEUE_IDLE;
                        }
                    } else {
                        // Camera woke up during the 3 seconds but we didn't detect it earlier
                        if (cam_state->snapshot_pending) {
                            // Stale flag - status_logic.c should have cleared this
                            ESP_LOGW(TAG, "Wake queue: Camera %d woke up but has stale snapshot_pending, clearing", 
                                     g_current_wake_camera_index);
                            cam_state->snapshot_pending = false;
                        } else {
                            ESP_LOGI(TAG, "Wake queue: Camera %d woke up during broadcast", 
                                     g_current_wake_camera_index);
                        }
                        cam_state->wake_retry_count = 0;  // Reset for next time
                        g_current_wake_camera_index = -1;
                        g_wake_queue_state = WAKE_QUEUE_IDLE;
                    }
                } else {
                    // Invalid state - no current camera but timeout occurred
                    g_current_wake_camera_index = -1;
                    g_wake_queue_state = WAKE_QUEUE_IDLE;
                }
            }
            break;
            
        case WAKE_QUEUE_WAITING_WAKEUP:
            // This state is not currently used - transition to IDLE
            g_wake_queue_state = WAKE_QUEUE_IDLE;
            break;
            
        default:
            // Invalid state, reset to IDLE
            ESP_LOGW(TAG, "Wake queue: Invalid state %d, resetting to IDLE", g_wake_queue_state);
            g_wake_queue_state = WAKE_QUEUE_IDLE;
            g_current_wake_camera_index = -1;
            break;
    }
}

/**
 * @brief Process timeout for single-camera wake operations
 * 
 * This function handles timeout detection for cameras that have snapshot_pending
 * set but are not part of the wake queue (single camera mode).
 * 
 * If a camera has snapshot_pending=true for longer than the timeout period
 * and is still sleeping, the flag is cleared to allow the user to retry.
 * 
 * IMPORTANT: This function should NOT run when the wake queue is active,
 * because all cameras in the queue have their own timeout handling via
 * ui_process_wake_queue(). Running both would cause race conditions where
 * cameras waiting in the queue get timed out prematurely.
 * 
 * Called from main loop in app_main.c
 */
void ui_process_single_camera_wake_timeout(void) {
    // Skip entirely if wake queue is active - queue handles its own timeouts
    // This prevents premature timeout of cameras waiting in the queue
    if (g_wake_queue.queue_active) {
        return;
    }
    
    // Timeout: 3.5 seconds (3s broadcast + 0.5s grace period for status updates)
    const uint32_t SINGLE_CAMERA_WAKE_TIMEOUT_MS = 3500;
    const uint8_t MAX_WAKE_RETRIES = 1;  // Allow 1 automatic retry before giving up
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    for (int i = 0; i < NUM_CAMERAS; i++) {
        camera_state_t *cam_state = &g_camera_states[i];
        
        // Skip if no snapshot pending
        if (!cam_state->snapshot_pending) {
            continue;
        }
        
        // Check if timeout has elapsed
        uint32_t time_since_request = current_time - cam_state->last_snapshot_request_ms;
        if (time_since_request < SINGLE_CAMERA_WAKE_TIMEOUT_MS) {
            continue;  // Not timed out yet
        }
        
        // Check camera state
        bool is_sleeping = (cam_state->power_mode == 3) || cam_state->is_sleeping;
        bool is_connected = (cam_state->connection_state == CAM_STATE_CONNECTED) && cam_state->is_connected;
        
        if (!is_connected) {
            // Camera disconnected - clear the flag and reset retry counter
            ESP_LOGW(TAG, "Single camera wake timeout: Camera %d disconnected, clearing snapshot_pending", i);
            cam_state->snapshot_pending = false;
            cam_state->wake_retry_count = 0;
            ESP_LOGD(TAG, "Camera %d: snapshot_pending cleared (disconnected during wake)", i);
        } else if (is_sleeping) {
            // Camera is still sleeping after timeout
            if (cam_state->wake_retry_count < MAX_WAKE_RETRIES) {
                // Retry: restart wake broadcast
                cam_state->wake_retry_count++;
                ESP_LOGW(TAG, "Single camera wake timeout: Camera %d still sleeping, retrying wake broadcast (attempt %d)", 
                         i, cam_state->wake_retry_count + 1);
                
                // Restart wake broadcast
                esp_err_t wake_result = connect_logic_start_wake_broadcast_for_slot(i);
                if (wake_result == ESP_OK) {
                    // Reset timeout by updating last_snapshot_request_ms
                    cam_state->last_snapshot_request_ms = current_time;
                    ESP_LOGI(TAG, "Single camera wake: Retry broadcast started for camera %d", i);
                } else {
                    // Failed to start retry broadcast - give up
                    ESP_LOGE(TAG, "Single camera wake: Failed to start retry broadcast for camera %d: %s", 
                             i, esp_err_to_name(wake_result));
                    cam_state->snapshot_pending = false;
                    cam_state->wake_retry_count = 0;
                    ESP_LOGD(TAG, "Camera %d: snapshot_pending cleared (retry broadcast failed)", i);
                }
            } else {
                // Max retries exceeded - give up
                ESP_LOGW(TAG, "Single camera wake timeout: Camera %d still sleeping after %d attempts, clearing snapshot_pending", 
                         i, cam_state->wake_retry_count + 1);
                cam_state->snapshot_pending = false;
                cam_state->wake_retry_count = 0;
                ESP_LOGD(TAG, "Camera %d: snapshot_pending cleared (wake timeout after retries)", i);
            }
        } else {
            // Camera woke up but snapshot_pending wasn't cleared - this is a stale flag
            // status_logic.c should have handled this, but clear it as a safety net
            ESP_LOGW(TAG, "Single camera wake timeout: Camera %d woke up but has stale snapshot_pending, clearing", i);
            cam_state->snapshot_pending = false;
            cam_state->wake_retry_count = 0;
            ESP_LOGD(TAG, "Camera %d: snapshot_pending cleared (stale flag after wake)", i);
        }
    }
}

/**
 * @brief Check for GPS state changes and trigger display update if needed
 * 
 * This function should be called periodically (every 500ms) from the main loop.
 * It compares current GPS state with the last displayed state and sets the
 * display_needs_update flag only when GPS status actually changed.
 * 
 * This ensures GPS updates work independently of camera status pushes,
 * so the GPS display updates even when all cameras are asleep.
 */
void ui_check_gps_update(void) {
    // Skip if not on main screen
    if (g_ui_state.current_screen != SCREEN_MAIN) {
        return;
    }
    
    bool current_fix = gps_has_fix();
    char current_text[24] = "";
    
    if (current_fix) {
        gps_data_t gps_data;
        if (gps_get_data(&gps_data) == ESP_OK) {
            snprintf(current_text, sizeof(current_text), "%.2f, %.2f", 
                     gps_data.latitude, gps_data.longitude);
        }
    }
    
    // Check if fix state changed
    bool fix_changed = (current_fix != g_last_displayed_gps_fix);
    
    // Check if text changed
    bool text_changed = (strcmp(current_text, g_last_displayed_gps_text) != 0);
    
    if (fix_changed || text_changed) {
        if (fix_changed) {
            ESP_LOGI(TAG, "GPS state changed: %s -> %s", 
                     g_last_displayed_gps_fix ? "FIX" : "NO_FIX",
                     current_fix ? "FIX" : "NO_FIX");
        }
        if (text_changed && current_fix) {
            ESP_LOGD(TAG, "GPS text changed: \"%s\" -> \"%s\"", 
                     g_last_displayed_gps_text, current_text);
        }
        
        // Update tracking state
        g_last_displayed_gps_fix = current_fix;
        strncpy(g_last_displayed_gps_text, current_text, sizeof(g_last_displayed_gps_text) - 1);
        g_last_displayed_gps_text[sizeof(g_last_displayed_gps_text) - 1] = '\0';  // Ensure null-termination
        
        // Trigger display update
        g_ui_state.display_needs_update = true;
    }
}
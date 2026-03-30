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
#include "../gps/gps_reader.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_random.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "esp_lvgl_port.h"

/* Logging tag for ESP_LOG functions */
#define TAG "UI"

/* Multi-camera support constant */
#define NUM_CAMERAS 3

/* NVS (Non-Volatile Storage) configuration for persistent data */
#define NVS_CAMERA_NAMESPACE "camera"      /* Namespace for camera pairing data */
#define NVS_CAMERA_KEY "paired_info"       /* Key for stored camera information */
#define NVS_DEVICE_NAMESPACE "device"      /* Namespace for device configuration */
#define NVS_DEVICE_ID_KEY "device_id"      /* Key for unique device identifier */

/* Old layout constants removed in Phase 6 -- layout is now in LVGL screen modules. */

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
};

/* camera_selection_t is defined in ui.h */

/* Main screen state tracking */
static bool g_main_screen_initialized = false;

/* GPS display state tracking (for independent updates) */
static bool g_last_displayed_gps_fix = false;  // Last displayed GPS fix state
static char g_last_displayed_gps_text[24] = "";  // Last displayed GPS coordinates text
camera_selection_t g_camera_selection = CAMERA_SELECT_0;
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
static discovered_camera_t g_discovered_cameras[MAX_DISCOVERED_CAMERAS] = {0};
static int g_discovered_camera_count = 0;
static bool g_pairing_scan_active = false;

/* Settings screen state (slot index used by ui_switch_screen) */
static int g_settings_active_camera_index = 0;

/* Mode Switch Screen state (slot index used by ui_switch_screen) */
static int g_mode_switch_camera_index = 0;

/* Wake state machine for reliable wake-and-record functionality */
static wake_state_t g_wake_state = WAKE_STATE_IDLE;
static uint32_t g_wake_broadcast_start_time = 0;  // Timestamp when wake broadcast started (milliseconds)
static uint32_t g_wake_timeout_start_time = 0;     // Timestamp when waiting for confirmation started

/* Wake-up queue for serialized wake broadcasts in "All Cameras" mode */
wake_queue_t g_wake_queue = {0};
wake_queue_state_t g_wake_queue_state = WAKE_QUEUE_IDLE;
int g_current_wake_camera_index = -1;  // Camera currently being woken (-1 if none)
uint32_t g_wake_broadcast_start_time_ms = 0;  // Timestamp when current broadcast started


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
            
            // Remove from wake queue if present
            if (g_current_wake_camera_index == camera_index) {
                ESP_LOGW(TAG, "Camera %d disconnected during wake-up, clearing from queue", camera_index);
                g_current_wake_camera_index = -1;
                g_wake_queue_state = WAKE_QUEUE_IDLE;
            }
            
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
            
            g_ui_state.display_needs_update = true;
            ui_update_display();
            
            ESP_LOGW(TAG, "AUTOCONNECT_BOOT: Protocol connection failed for camera %d", camera_index);
        }
    } else {
        // BLE connection failed - reset state
        g_camera_states[camera_index].connection_state = CAM_STATE_PAIRED_DISCONNECTED;
        g_camera_states[camera_index].is_connected = false;
        g_camera_states[camera_index].snapshot_pending = false;
        
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

static void ui_detect_device_and_set_scale(void);

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
void ui_early_init(void) {
    ESP_LOGI(TAG, "UI early init: NVS, pairings, data layer, boot scan");

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

    /* Start scanning for paired cameras while splash screen is still visible */
    ui_auto_connect_on_startup();
}

void ui_init(void) {
    ESP_LOGI(TAG, "Initializing UI system");

    /* Configure display scaling and layout for detected hardware */
    ui_detect_device_and_set_scale();
    g_ui_state.current_screen = SCREEN_MAIN;
    g_ui_state.display_needs_update = true;

    /* Perform initial display update to show Main Screen */
    ui_update_display();
    
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
static void ui_detect_device_and_set_scale(void) {
    ESP_LOGI(TAG, "Detected: M5Stack Basic V2.7 (320x240)");
}

/* Old draw functions (ui_draw_bitmap, ui_draw_bitmap_inverted, ui_draw_connection_status,
 * ui_draw_gps_status, ui_get_text_width) removed in Phase 6.
 * Rendering is now handled entirely by LVGL screen modules. */

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


/* Old draw functions removed in Phase 6 -- see ui_screen_main.c, ui_screen_pairing.c,
 * ui_screen_settings.c, ui_screen_mode_switch.c for LVGL replacements. */

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
    g_discovered_camera_count = 0;
    memset(g_discovered_cameras, 0, sizeof(g_discovered_cameras));
    
    ui_switch_screen(SCREEN_PAIRING);
    
    // Start BLE scanning in PAIRING mode
    g_pairing_scan_active = true;
    esp_err_t ret = ble_start_scan(SCAN_MODE_PAIRING, camera_index, 30000);
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
            if (rssi > g_discovered_cameras[i].rssi) {
                g_discovered_cameras[i].rssi = rssi;
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

    // Update LVGL pairing screen with the new camera
    lvgl_port_lock(0);
    ui_screen_pairing_add_camera(name, g_discovered_camera_count);
    lvgl_port_unlock();
    
    g_discovered_camera_count++;
    g_ui_state.display_needs_update = true;
    
    ESP_LOGI(TAG, "Discovered camera for pairing: %s (RSSI: %d)", name, rssi);
}

/**
 * @brief Update the name of an already-discovered camera from scan response data
 * Called by BLE layer when a SCAN_RSP with a device name is received
 */
void ui_pairing_update_discovered_camera_name(const char *name, const uint8_t *mac) {
    if (!g_pairing_scan_active) return;

    for (int i = 0; i < g_discovered_camera_count; i++) {
        if (memcmp(g_discovered_cameras[i].mac, mac, 6) == 0) {
            if (strcmp(g_discovered_cameras[i].name, name) == 0) return;

            strncpy(g_discovered_cameras[i].name, name,
                    sizeof(g_discovered_cameras[i].name) - 1);
            g_discovered_cameras[i].name[sizeof(g_discovered_cameras[i].name) - 1] = '\0';

            lvgl_port_lock(0);
            ui_screen_pairing_add_camera(name, i);
            lvgl_port_unlock();

            ESP_LOGI(TAG, "Updated camera name from scan response: %s", name);
            return;
        }
    }
}

static void ui_return_to_main_screen(void);

/**
 * @brief Handle button press on pairing screen
 */
static void ui_handle_pairing_screen_button_a(void) {
    int sel = ui_screen_pairing_get_selection();
    int active_slot = ui_screen_pairing_get_camera_index();

    if (sel == 0) {
        g_pairing_scan_active = false;
        ble_stop_scan();
        g_discovered_camera_count = 0;
        memset(g_discovered_cameras, 0, sizeof(g_discovered_cameras));
        ESP_LOGI(TAG, "Pairing cancelled, returning to main screen");
        ui_return_to_main_screen();
    } else {
        int camera_idx = sel - 1;
        if (camera_idx >= 0 && camera_idx < g_discovered_camera_count &&
            g_discovered_cameras[camera_idx].is_valid) {

            ESP_LOGI(TAG, "Pairing camera: %s to slot %d",
                     g_discovered_cameras[camera_idx].name, active_slot);

            camera_state_t *cam = &g_camera_states[active_slot];
            cam->is_paired = true;
            strncpy(cam->camera_name, g_discovered_cameras[camera_idx].name, sizeof(cam->camera_name) - 1);
            cam->camera_name[sizeof(cam->camera_name) - 1] = '\0';
            memcpy(cam->camera_mac, g_discovered_cameras[camera_idx].mac, 6);
            cam->device_id = g_discovered_cameras[camera_idx].device_id;
            cam->connection_state = CAM_STATE_PAIRED_DISCONNECTED;
            cam->camera_reserved = active_slot;
            cam->camera_supports_new_status_push = false;
            cam->mode_name[0] = '\0';
            cam->mode_param[0] = '\0';
            cam->last_mode_name[0] = '\0';
            cam->last_mode_param[0] = '\0';

            ESP_LOGI(TAG, "Camera %d paired: %s (MAC: %02X:%02X:%02X:%02X:%02X:%02X)",
                     active_slot, cam->camera_name,
                     cam->camera_mac[0], cam->camera_mac[1], cam->camera_mac[2],
                     cam->camera_mac[3], cam->camera_mac[4], cam->camera_mac[5]);

            esp_err_t save_err = save_all_cameras_to_nvs();
            if (save_err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save camera pairings to NVS: %s", esp_err_to_name(save_err));
            }

            if (active_slot == 0) {
                g_stored_camera.is_paired = cam->is_paired;
                strncpy(g_stored_camera.camera_name, cam->camera_name, sizeof(g_stored_camera.camera_name));
                memcpy(g_stored_camera.camera_mac, cam->camera_mac, 6);
                g_stored_camera.device_id = cam->device_id;
                g_stored_camera.camera_reserved = cam->camera_reserved;
            }

            cam->connection_state = CAM_STATE_CONNECTING;
            g_pairing_scan_active = false;
            ble_stop_scan();
            g_discovered_camera_count = 0;
            memset(g_discovered_cameras, 0, sizeof(g_discovered_cameras));

            ESP_LOGI(TAG, "Initiating connection to newly paired camera %d", active_slot);
            ui_screen_pairing_set_notification("Pairing... Confirm on Camera",
                                                lv_color_make(0, 255, 255));
            lvgl_port_unlock();
            vTaskDelay(pdMS_TO_TICKS(2000));
            lvgl_port_lock(0);

            g_verify_mode = 1;

            int connect_result = ui_perform_complete_reconnection(active_slot, true);

            if (connect_result == 0) {
                ESP_LOGI(TAG, "Pairing and connection successful for camera %d", active_slot);
                ui_screen_pairing_set_notification("Pairing Complete!",
                                                    lv_color_make(0, 255, 0));
                lvgl_port_unlock();
                vTaskDelay(pdMS_TO_TICKS(1500));
                lvgl_port_lock(0);
            } else {
                ESP_LOGW(TAG, "Pairing connection failed for camera %d", active_slot);
                cam->connection_state = CAM_STATE_PAIRED_DISCONNECTED;
                ui_screen_pairing_set_notification("Pairing Failed - Retry from Settings",
                                                    lv_color_make(255, 0, 0));
                lvgl_port_unlock();
                vTaskDelay(pdMS_TO_TICKS(2500));
                lvgl_port_lock(0);
            }

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
    ui_screen_pairing_button_b();
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
    ui_switch_screen(SCREEN_CAMERA_SETTINGS);
}

/**
 * @brief Return from settings/pairing screen to main screen
 * 
 * Helper function to properly reset state and trigger main screen redraw.
 */
static void ui_return_to_main_screen(void) {
    ui_switch_screen(SCREEN_MAIN);
}

void ui_switch_screen(ui_screen_t screen) {
    lvgl_port_lock(0);

    ui_screen_t old_screen = g_ui_state.current_screen;

    /* Destroy the screen we're leaving */
    switch (old_screen) {
        case SCREEN_MAIN:
            ui_screen_main_destroy();
            g_main_screen_initialized = false;
            break;
        case SCREEN_PAIRING:
            ui_screen_pairing_destroy();
            break;
        case SCREEN_CAMERA_SETTINGS:
            ui_screen_settings_destroy();
            break;
        case SCREEN_MODE_SWITCH:
            ui_screen_mode_switch_destroy();
            break;
        default:
            break;
    }

    /* Create and load the new LVGL screen */
    lv_obj_t *scr = NULL;
    switch (screen) {
        case SCREEN_MAIN:
            scr = ui_screen_main_create();
            g_main_screen_initialized = true;
            break;
        case SCREEN_PAIRING:
            scr = ui_screen_pairing_create(g_pairing_active_camera_index);
            break;
        case SCREEN_CAMERA_SETTINGS:
            scr = ui_screen_settings_create(g_settings_active_camera_index);
            break;
        case SCREEN_MODE_SWITCH:
            scr = ui_screen_mode_switch_create(g_mode_switch_camera_index);
            break;
        default:
            break;
    }

    if (scr) lv_screen_load(scr);

    g_ui_state.current_screen = screen;
    g_ui_state.display_needs_update = true;
    lvgl_port_unlock();
    ESP_LOGI(TAG, "Screen switched to %d", (int)screen);
}

// Mode switch and settings button handlers delegate to LVGL screen modules

static void ui_handle_mode_switch_button_a(void) {
    ui_screen_mode_switch_button_a();
}

static void ui_handle_mode_switch_button_b(void) {
    ESP_LOGI(TAG, "Mode Switch: Back button pressed, returning to Main Screen");
    ui_return_to_main_screen();
}

/**
 * @brief Handle button A press on settings screen
 */
static void ui_handle_settings_screen_button_a(void) {
    int sel = ui_screen_settings_get_selection();
    int active_slot = ui_screen_settings_get_camera_index();
    camera_state_t *cam = &g_camera_states[active_slot];

    switch (sel) {
        case SETTINGS_ITEM_BACK:
            ESP_LOGI(TAG, "Settings: Back selected");
            ui_return_to_main_screen();
            break;

        case SETTINGS_ITEM_MODE_SWITCH:
            ESP_LOGI(TAG, "Settings: Mode Switch selected for camera %d", active_slot + 1);
            g_mode_switch_camera_index = active_slot;
            ui_switch_screen(SCREEN_MODE_SWITCH);
            break;

        case SETTINGS_ITEM_SLEEP_WAKEUP: {
            ESP_LOGI(TAG, "Settings: Sleep/Wakeup selected");
            bool is_sleeping = (cam->power_mode == 3) || cam->is_sleeping;

            if (is_sleeping) {
                ui_screen_settings_set_notification("Waking up...", lv_color_make(0, 255, 255));

                esp_err_t wake_result = connect_logic_start_wake_broadcast_for_slot(active_slot);
                if (wake_result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to start wake broadcast: %s", esp_err_to_name(wake_result));
                    ui_screen_settings_set_notification("Wake failed", lv_color_make(255, 0, 0));
                    lvgl_port_unlock();
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    lvgl_port_lock(0);
                } else {
                    lvgl_port_unlock();
                    vTaskDelay(pdMS_TO_TICKS(3500));
                    lvgl_port_lock(0);
                    ui_screen_settings_set_notification("Wake broadcast sent", lv_color_make(0, 255, 0));
                    lvgl_port_unlock();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    lvgl_port_lock(0);
                }
            } else {
                ui_screen_settings_set_notification("Sleeping...", lv_color_make(0, 255, 255));
                command_logic_power_mode_switch_sleep(active_slot);
                lvgl_port_unlock();
                vTaskDelay(pdMS_TO_TICKS(1000));
                lvgl_port_lock(0);
                ui_screen_settings_set_notification("Sleep command sent", lv_color_make(0, 255, 0));
                lvgl_port_unlock();
                vTaskDelay(pdMS_TO_TICKS(1000));
                lvgl_port_lock(0);
            }
            ui_return_to_main_screen();
            break;
        }

        case SETTINGS_ITEM_CONNECT_DISCONNECT:
            if (cam->connection_state == CAM_STATE_CONNECTED) {
                ESP_LOGI(TAG, "Settings: Disconnecting camera %d", active_slot);
                ui_screen_settings_set_notification("Disconnecting...", lv_color_make(255, 255, 0));
                ble_disconnect(active_slot);

                cam->connection_state = CAM_STATE_PAIRED_DISCONNECTED;
                cam->is_connected = false;
                cam->is_initialized = false;

                lvgl_port_unlock();
                vTaskDelay(pdMS_TO_TICKS(1000));
                lvgl_port_lock(0);
                ui_screen_settings_set_notification("Disconnected", lv_color_make(255, 255, 0));
                lvgl_port_unlock();
                vTaskDelay(pdMS_TO_TICKS(1000));
                lvgl_port_lock(0);
                ui_return_to_main_screen();
            } else {
                ESP_LOGI(TAG, "Settings: Connecting camera %d", active_slot);
                ui_screen_settings_set_notification("Connecting...", lv_color_make(0, 0, 255));
                lvgl_port_unlock();
                vTaskDelay(pdMS_TO_TICKS(1000));
                lvgl_port_lock(0);

                int result = ui_perform_complete_reconnection(active_slot, true);

                if (result == 0) {
                    ESP_LOGI(TAG, "Connection successful");
                    ui_screen_settings_set_notification("Connected!", lv_color_make(0, 255, 0));
                } else {
                    ESP_LOGW(TAG, "Connection failed");
                    ui_screen_settings_set_notification("Connection failed", lv_color_make(255, 0, 0));
                }
                lvgl_port_unlock();
                vTaskDelay(pdMS_TO_TICKS(1000));
                lvgl_port_lock(0);
                ui_return_to_main_screen();
            }
            break;

        case SETTINGS_ITEM_PAIR_UNPAIR:
            if (cam->is_paired) {
                ESP_LOGI(TAG, "Settings: Unpairing camera %d", active_slot);

                if (cam->connection_state == CAM_STATE_CONNECTED) {
                    ESP_LOGI(TAG, "Camera %d connected, disconnecting before unpairing", active_slot);
                    ui_screen_settings_set_notification("Disconnecting...", lv_color_make(255, 255, 0));
                    connect_logic_ble_disconnect(active_slot);
                    lvgl_port_unlock();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    lvgl_port_lock(0);
                }

                ui_screen_settings_set_notification("Unpairing...", lv_color_make(255, 0, 0));
                lvgl_port_unlock();
                vTaskDelay(pdMS_TO_TICKS(1000));
                lvgl_port_lock(0);

                memset(cam, 0, sizeof(camera_state_t));
                cam->connection_state = CAM_STATE_UNPAIRED;
                cam->is_paired = false;

                esp_err_t save_result = save_all_cameras_to_nvs();
                if (save_result == ESP_OK) {
                    ESP_LOGI(TAG, "Pairing deleted and saved to NVS");
                } else {
                    ESP_LOGW(TAG, "Failed to save after unpairing");
                }

                ui_screen_settings_set_notification("Camera unpaired", lv_color_make(255, 0, 0));
                lvgl_port_unlock();
                vTaskDelay(pdMS_TO_TICKS(1000));
                lvgl_port_lock(0);
                ui_return_to_main_screen();
            } else {
                ESP_LOGI(TAG, "Settings: Switching to pairing screen for slot %d", active_slot);
                g_pairing_active_camera_index = active_slot;
                ui_start_pairing(active_slot);
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
    ui_screen_settings_button_b();
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

    lvgl_port_lock(0);

    switch (g_ui_state.current_screen) {
        case SCREEN_PAIRING:
            ui_screen_pairing_update();
            break;

        case SCREEN_CAMERA_SETTINGS:
            ui_screen_settings_update();
            break;

        case SCREEN_MODE_SWITCH:
            ui_screen_mode_switch_update();
            break;

        case SCREEN_MAIN:
        default: {
            if (!ui_screen_main_is_created()) {
                lv_obj_t *scr = ui_screen_main_create();
                if (scr) lv_screen_load(scr);
                g_main_screen_initialized = true;
            }

            ui_screen_main_update();
            break;
        }
    }

    g_ui_state.display_needs_update = false;
    lvgl_port_unlock();
}

static bool is_camera_sleep_candidate(int slot) {
    camera_state_t *cam = &g_camera_states[slot];
    return cam->is_paired
        && cam->connection_state == CAM_STATE_CONNECTED
        && !cam->is_sleeping
        && !cam->is_recording;
}

static bool is_camera_wake_candidate(int slot) {
    camera_state_t *cam = &g_camera_states[slot];
    return cam->is_paired
        && cam->connection_state == CAM_STATE_CONNECTED
        && cam->is_sleeping;
}

/**
 * @brief Handle Button A press (screen-aware)
 */
void ui_handle_button_a(void) {
    lvgl_port_lock(0);
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
    lvgl_port_unlock();

    // Deferred reconnection: runs outside the LVGL lock so the display
    // can refresh to show "connecting" state during the blocking BLE call.
    if (g_ui_state.current_screen == SCREEN_MAIN &&
        g_camera_selection != CAMERA_SELECT_ALL) {
        int cam_idx = (int)g_camera_selection;
        if (cam_idx >= 0 && cam_idx < NUM_CAMERAS &&
            g_camera_states[cam_idx].connection_state == CAM_STATE_CONNECTING &&
            !g_camera_states[cam_idx].is_connected) {
            int result = ui_perform_complete_reconnection(cam_idx, true);
            if (result == 0) {
                ESP_LOGI(TAG, "Reconnection successful for camera %d", cam_idx);
            } else {
                ESP_LOGW(TAG, "Reconnection failed for camera %d", cam_idx);
            }
            lvgl_port_lock(0);
            g_ui_state.display_needs_update = true;
            lvgl_port_unlock();
        }
    }
}

/**
 * @brief Handle Button B press (screen-aware)
 */
void ui_handle_button_b(void) {
    lvgl_port_lock(0);
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
    lvgl_port_unlock();
}

/**
 * @brief Handle Button C press (screen-aware)
 * Button C opens pairing or settings screen from main screen
 * Special behavior: During active boot scan, stops scan and connects to found cameras
 * Highlight Tag: If camera is recording and highlight-capable, sends highlight tag command
 */
void ui_handle_button_c(void) {
    lvgl_port_lock(0);
    if (g_ui_state.current_screen != SCREEN_MAIN) {
        lvgl_port_unlock();
        return;
    }
    
    // PRECEDENCE 1: Special behavior - If boot scan is active, stop it and connect to found cameras
    // This must happen FIRST before any highlight tag logic
    if (connect_logic_is_boot_scan_active()) {
        ESP_LOGI(TAG, "Button C pressed during boot scan - stopping scan and connecting to found cameras");
        lvgl_port_unlock();
        connect_logic_stop_boot_scan_and_connect_found();
        return;
    }
    

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
            lvgl_port_unlock();
            return;
        }
        
        // Conditions not met - fall back to existing behavior
        if (g_camera_states[cam_idx].connection_state == CAM_STATE_UNPAIRED) {
            ui_start_pairing(cam_idx);
        } else {
            ui_start_settings(cam_idx);
        }
        lvgl_port_unlock();
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
            lvgl_port_unlock();
            return;
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
            lvgl_port_unlock();
            return;
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
            lvgl_port_unlock();
            return;
        }
        
        // No highlight, no sleep, no wake candidates - do nothing
        lvgl_port_unlock();
        return;
    }
    lvgl_port_unlock();
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
    (void)color;
    lvgl_port_lock(0);
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *lbl = lv_label_create(scr);
    lv_label_set_text(lbl, message);
    lv_obj_set_style_text_color(lbl, lv_color_make(255, 0, 0), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl);

    lv_screen_load(scr);
    lvgl_port_unlock();

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

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
    lv_color_t lv_clr = lv_color_white();
    if (color == M5_COLOR_RED)         lv_clr = lv_color_make(255,   0,   0);
    else if (color == M5_COLOR_GREEN)  lv_clr = lv_color_make(  0, 255,   0);
    else if (color == M5_COLOR_YELLOW) lv_clr = lv_color_make(255, 255,   0);
    else if (color == M5_COLOR_BLUE)   lv_clr = lv_color_make(  0,   0, 255);
    else if (color == M5_COLOR_CYAN)   lv_clr = lv_color_make(  0, 255, 255);
    else if (color == M5_COLOR_ORANGE) lv_clr = lv_color_make(255, 165,   0);

    if (g_ui_state.current_screen == SCREEN_MAIN && ui_screen_main_is_created()) {
        lvgl_port_lock(0);
        ui_screen_main_set_notification(message, lv_clr);
        lvgl_port_unlock();

        vTaskDelay(pdMS_TO_TICKS(duration_ms));

        lvgl_port_lock(0);
        ui_screen_main_clear_notification();
        lvgl_port_unlock();
        return;
    }

    /* Non-main screens: just delay (the LVGL screen will refresh naturally) */
    ESP_LOGI(TAG, "Notification (screen %d): %s", g_ui_state.current_screen, message);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
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
    
    if (connect_logic_is_boot_scan_active()) {
        ESP_LOGI(TAG, "Boot scan active - ignoring shutter action");
        return;
    }

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
        // Camera is paired but not connected - mark as connecting and defer to after lock release
        if (g_camera_states[cam_idx].connection_state == CAM_STATE_PAIRED_DISCONNECTED || 
            g_camera_states[cam_idx].connection_state == CAM_STATE_CONNECTING) {
            ESP_LOGI(TAG, "Camera %d is paired but disconnected - scheduling reconnect", cam_idx);
            g_camera_states[cam_idx].connection_state = CAM_STATE_CONNECTING;
            g_ui_state.display_needs_update = true;
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
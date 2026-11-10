/*
 * DJI Camera Remote Control - Main Application Entry Point
 * 
 * This file contains the main application entry point and core system initialization
 * for the DJI camera remote control system running on M5Stack Basic V2.7 hardware.
 * 
 * The system provides a Bluetooth Low Energy (BLE) interface to control DJI cameras
 * with features including:
 * - Camera connection management
 * - Shutter control (photo/video recording)
 * - Camera mode switching
 * - Sleep/wake functionality
 * - Internal and optional external buttons (non-blocking, ADR-024)
 * 
 * Hardware: M5Stack Basic V2.7 (ESP32-based)
 * Display: 320x240 TFT LCD
 * Connectivity: Bluetooth Low Energy
 */

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_system.h"
#include <math.h>

#include "connect_logic.h"
#include "light_logic.h"
#include "diag.h"
#include "sdkconfig.h"

#include "board_hal.h"

#include "ui.h"
#include "button_input.h"
#include "slot_busy.h"
#include "ui_layout.h"
#include "ui_screen_splash.h"
#include "lvgl_icons.h"
#include "esp_lvgl_port.h"
#include "../gps/gps_reader.h"
#include "command_logic.h"
#include "command_worker.h"
#include "status_logic.h"
#include "freertos/task.h"
#include "dji_protocol_data_structures.h"
#include "dji_gps_timestamp.h"
#include "../log_config.h"

/* Maximum age of GPS position and UTC time for a GPS push; position and
 * timestamp share the same limit (ADR-023) */
#define GPS_PUSH_MAX_DATA_AGE_MS 5000

/* Forward declaration for GPS transmission task */
static void gps_transmission_task(void *pvParameters);


/**
 * @brief Main application entry point
 * 
 * This function serves as the ESP-IDF application entry point and implements
 * the complete system initialization sequence followed by the main event loop.
 * 
 * Initialization sequence:
 * 1. M5Stack Basic V2.7 hardware (display, buttons, power management)
 * 2. RGB LED light system
 * 3. Bluetooth Low Energy subsystem
 * 4. User interface system (including external button pins)
 * 
 * Main loop handles (50 ms cycle, never blocks on a held button):
 * - Button input (internal and external, CLICK/LONG events)
 * - Link events, command results, wake queue
 * - Display updates
 * 
 * @note This function never returns under normal operation
 */
void app_main(void) {
    static const char *TAG = "MAIN";
    int res = 0;

    /* 
     * LOGGING CONFIGURATION
     * Configure global log level before any subsystems start logging
     */
    app_log_set_level(APP_DEFAULT_LOG_LEVEL);

    /* Persistent fault diagnostics (ADR-022). Before HAL_INIT so a restart is
     * recorded even if a later init step fails; uses its own NVS partition. */
    diag_init();

    /* 
     * HARDWARE INITIALIZATION PHASE
     * Initialize all hardware components in dependency order
     */
    
    /* Initialize M5Stack Basic V2.7 hardware platform
     * This includes: display controller, button GPIO, power management,
     * I2C bus, SPI bus, and other core hardware peripherals
     */
    res = HAL_INIT();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize hardware platform");
        return;
    }
    ESP_LOGI(TAG, "Hardware platform initialized");

    /* Initialize RGB LED light system
     * Sets up the WS2812 LED strip driver for status indication
     * Colors indicate: connection state, operation status, errors
     */
    res = init_light_logic();
    if (res != 0) {
        ESP_LOGE(TAG, "Failed to initialize light logic");
        return;
    }
    ESP_LOGI(TAG, "Light logic initialized");

    /* Initialize Bluetooth Low Energy subsystem
     * Configures ESP32 BLE stack for DJI camera communication
     * Sets up GATT client, advertising scanner, and connection management
     */
    ESP_LOGI(TAG, "Initializing Bluetooth...");
    res = connect_logic_ble_init();
    if (res != 0) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth");
        return;
    }
    ESP_LOGI(TAG, "Bluetooth initialized successfully");

    /* Initialize GPS module
     * Sets up UART2 communication with M5Stack Module GPS v2.0
     * and starts NMEA parsing task
     */
    ESP_LOGI(TAG, "Initializing GPS module...");
    res = gps_init();
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize GPS module (continuing without GPS)");
    } else {
        ESP_LOGI(TAG, "GPS module initialized successfully");
        
        /* Create 10Hz GPS data transmission task
         * Sends GPS position to camera every 100ms when connected
         */
        xTaskCreate(gps_transmission_task, "gps_tx_task", 4096, NULL, 5, NULL);
        ESP_LOGI(TAG, "GPS transmission task created (10Hz)");
    }

    /* Initialize LVGL display framework
     * Uses the existing SPI panel handle from display_init; creates
     * its own FreeRTOS task for rendering.
     */
    lv_display_t *lvgl_disp = HAL_LVGL_INIT();
    if (lvgl_disp == NULL) {
        ESP_LOGE(TAG, "Failed to initialize LVGL display");
        return;
    }
    ESP_LOGI(TAG, "LVGL display initialized");

    /* Show LVGL splash screen before enabling backlight (ADR-010).
     * Hold the LVGL lock so the render task cannot draw the default
     * white screen before our splash is loaded.
     */
    lvgl_port_lock(0);
    lv_obj_t *splash = ui_screen_splash_create();
    lv_screen_load(splash);
    lvgl_port_unlock();

    /* Let LVGL render one frame, then turn on the backlight so the
     * first visible frame is the splash logo on a black background. */
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_BACKLIGHT_ON();

    ui_layout_init(lvgl_disp);
    lvgl_icons_init();

    /* Load NVS, pairing data, and start the BLE boot scan while the
     * splash screen is still visible.  The scan runs asynchronously on
     * the BLE stack tasks. */
    ui_early_init();

    /* Per-slot command workers (ADR-020); need the data layer from
     * ui_early_init() and must exist before buttons are handled */
    if (command_worker_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start command workers");
    }

    /* Keep splash screen visible for ~2 seconds total (includes the
     * 100 ms pre-backlight wait above). */
    vTaskDelay(pdMS_TO_TICKS(1900));

    /* Initialize the main UI (creates and loads the main screen) */
    ui_init();
    ESP_LOGI(TAG, "UI system initialized");

    /* Destroy the splash screen now that the main UI is active */
    lvgl_port_lock(0);
    ui_screen_splash_destroy();
    lvgl_port_unlock();

    /* System ready - log operational information for user */
    ESP_LOGI(TAG, "System ready - Icon-based UI active!");
    ESP_LOGI(TAG, "Free heap after init: %lu bytes (min %lu)",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "Button A: Select/execute current option");
    ESP_LOGI(TAG, "Button B: Cycle through options");

    /*
     * MAIN APPLICATION EVENT LOOP
     * Handles all user input, system events, and display updates
     */
    
    while (1) {
        /*
         * USER INPUT (ADR-024)
         * Samples internal and external buttons once and dispatches CLICK /
         * LONG events to the screen-aware handlers. Non-blocking: a held
         * button does not stall the rest of this loop.
         */
        button_input_poll();

        /*
         * LINK-LOSS CLEANUP (ADR-018)
         * Remove slots that lost their link from the wake queue and stop a
         * wake broadcast aimed at them. Must run before the wake queue.
         */
        ui_process_link_events();

        /*
         * COMMAND RESULTS (ADR-020)
         * Show results of per-slot worker jobs without blocking
         */
        ui_process_command_results();
        
        /*
         * WAKE QUEUE PROCESSING
         * Process serialized wake-up queue for "All Cameras" mode
         * Handles one camera at a time with early wake-up detection
         */
        ui_process_wake_queue();
        
        /*
         * SINGLE CAMERA WAKE TIMEOUT
         * Check for stale snapshot_pending flags in single-camera mode
         * Clears flags if camera didn't wake up within timeout period
         */
        ui_process_single_camera_wake_timeout();

        /*
         * BUSY INDICATOR STATE (plan 0017)
         * End finished per-slot busy entries even where no screen shows
         * them (M5Stack, other screens), so they never linger
         */
        slot_busy_evaluate_all();

        /*
         * DISPLAY UPDATE
         * Refresh display if any UI elements have changed
         * Uses dirty flag system for efficiency
         */
        ui_update_display();
        
        /*
         * PERIODIC MAIN SCREEN REFRESH
         * Update recording time display every second when camera is recording
         */
        static uint32_t shutter_refresh_timer = 0;
        shutter_refresh_timer += 50;
        if (shutter_refresh_timer >= 1000) {
            shutter_refresh_timer = 0;
            extern ui_state_t g_ui_state;

            /* Refresh Main Screen when any slot records to update elapsed time */
            if (status_logic_any_slot_recording()) {
                g_ui_state.display_needs_update = true;
            }
        }

        /*
         * PERIODIC GPS STATUS CHECK
         * Update GPS display independently of camera status updates
         * This ensures GPS updates work even when all cameras are asleep
         */
        static uint32_t gps_check_timer = 0;
        gps_check_timer += 50;
        if (gps_check_timer >= 500) {
            gps_check_timer = 0;
            ui_check_gps_update();
        }

        /*
         * BOOT SCAN UPDATE
         * Update boot scan state and handle timeout/connection initiation
         * Called periodically to check scan status and initiate connections
         */
        connect_logic_update_boot_scan();

        /*
         * BACKGROUND TASKS
         * Light logic runs on its own FreeRTOS timer task
         * BLE operations run on ESP-IDF BLE stack tasks
         * No manual updates needed for these systems
         */

        /* Main loop timing - 50ms cycle time
         * Provides responsive UI while preventing excessive CPU usage
         */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief GPS data transmission task (10Hz)
 * 
 * Sends GPS position data to the camera at 10Hz (every 100ms) when connected.
 * This task runs independently and handles GPS data transmission according
 * to the DJI reference implementation. Constructs the full GPS frame with
 * all required fields including date, time, position, velocity, and accuracy.
 * 
 * @param pvParameters Task parameters (unused)
 */
static void gps_transmission_task(void *pvParameters) {
    const char *TAG_TASK = "GPS_TX";
    ESP_LOGI(TAG_TASK, "GPS transmission task started (10Hz)");
    
    static uint32_t last_log_time = 0;
    static uint32_t last_gps_send_time = 0;
    static bool last_was_connected = false;
    static bool last_had_fix = false;
    /* Separate from last_log_time so the UTC skip warning cannot shift the
     * timing of the fix/no-fix logs */
    static uint32_t last_utc_warn_time = 0;
    
    /* GPS push throttling: minimum interval between sends (100ms = 10Hz max) */
    const uint32_t GPS_SEND_INTERVAL_MS = 100;
    /* Maximum acceptable horizontal accuracy (meters) */
    const float MAX_HORIZONTAL_ACCURACY = 50.0f;
    /* Maximum acceptable vertical accuracy (meters) */
    const float MAX_VERTICAL_ACCURACY = 100.0f;
    
    while (1) {
        /* Wait 100ms for 10Hz transmission rate */
        vTaskDelay(pdMS_TO_TICKS(100));
        
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        /* Check if ANY camera is connected and awake */
        bool any_camera_ready = false;
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (g_camera_states[i].connection_state == CAM_STATE_CONNECTED && 
                !g_camera_states[i].is_sleeping) {
                any_camera_ready = true;
                break;
            }
        }
        
        if (!any_camera_ready) {
            if (last_was_connected || (current_time - last_log_time > 10000)) {
                ESP_LOGD(TAG_TASK, "Not sending GPS: No cameras connected and awake");
                last_log_time = current_time;
            }
            last_was_connected = false;
            continue;
        }
        last_was_connected = true;

        /* Get GPS data */
        gps_data_t gps_data;
        if (gps_get_data(&gps_data) != ESP_OK) {
            if (current_time - last_log_time > 10000) {
                ESP_LOGW(TAG_TASK, "Not sending GPS: No GPS data available");
                last_log_time = current_time;
            }
            continue;
        }

        /* Check GPS fix status - must have valid fix and satellites */
        if (!gps_data.has_fix || gps_data.satellite_count == 0) {
            if (last_had_fix || (current_time - last_log_time > 10000)) {
                ESP_LOGI(TAG_TASK, "Not sending GPS: No GPS fix (satellites: %d)", gps_data.satellite_count);
                last_log_time = current_time;
            }
            last_had_fix = false;
            continue;
        }
        
        /* Check GPS data age - don't send stale data */
        uint32_t gps_age_ms = gps_get_last_update_age_ms();
        if (gps_age_ms > GPS_PUSH_MAX_DATA_AGE_MS) {
            if (current_time - last_log_time > 10000) {
                ESP_LOGW(TAG_TASK, "Not sending GPS: Data too old (%lu ms)", gps_age_ms);
                last_log_time = current_time;
            }
            continue;
        }
        
        /* Check accuracy - only send if accuracy is acceptable */
        float horizontal_acc = (gps_data.hdop > 0.0f) ? (gps_data.hdop * 5.0f) : 5.0f;
        float vertical_acc = (gps_data.vdop > 0.0f) ? (gps_data.vdop * 5.0f) : 10.0f;
        
        if (horizontal_acc > MAX_HORIZONTAL_ACCURACY || vertical_acc > MAX_VERTICAL_ACCURACY) {
            if (current_time - last_log_time > 10000) {
                ESP_LOGW(TAG_TASK, "Not sending GPS: Accuracy too poor (H: %.1fm, V: %.1fm)", 
                         horizontal_acc, vertical_acc);
                last_log_time = current_time;
            }
            continue;
        }
        
        /* Throttle GPS sends - ensure minimum interval between transmissions */
        if (current_time - last_gps_send_time < GPS_SEND_INTERVAL_MS) {
            continue;  /* Skip this cycle, wait for next */
        }
        
        /* Log when we get first fix */
        if (!last_had_fix) {
            ESP_LOGI(TAG_TASK, "✓ GPS fix acquired! Starting GPS data transmission to camera");
            ESP_LOGI(TAG_TASK, "  Position: %.6f, %.6f, Alt: %.1f m, Satellites: %d",
                     gps_data.latitude, gps_data.longitude, gps_data.altitude, gps_data.satellite_count);
            last_had_fix = true;
            last_log_time = current_time;
        }

        /* Timestamp from GPS UTC (never the system clock, which is not set).
         * Without a fresh RMC time the frame is skipped rather than sent with
         * a made-up time; last_gps_send_time stays untouched so the next
         * cycle retries (ADR-023). */
        uint32_t utc_epoch;
        if (!gps_get_utc_epoch(&utc_epoch, GPS_PUSH_MAX_DATA_AGE_MS)) {
            if (current_time - last_utc_warn_time > 10000) {
                ESP_LOGW(TAG_TASK, "Not sending GPS: fix reported but no valid UTC time (RMC missing or stale)");
                last_utc_warn_time = current_time;
            }
            continue;
        }
        int32_t year_month_day;
        int32_t hour_minute_second;
        dji_gps_encode_timestamp(utc_epoch, &year_month_day, &hour_minute_second);

        /* Convert coordinates to degrees * 1e7 */
        int32_t gps_latitude = (int32_t)(gps_data.latitude * 1e7f);
        int32_t gps_longitude = (int32_t)(gps_data.longitude * 1e7f);
        
        /* Convert altitude to millimeters */
        int32_t height = (int32_t)(gps_data.altitude * 1000.0f);

        /* Calculate velocity components from speed and course */
        /* Convert course (degrees) to radians for calculations */
        float course_rad = gps_data.course * M_PI / 180.0f;
        float speed_cm_per_s = gps_data.speed * 100.0f;  /* Convert m/s to cm/s */
        
        float speed_to_north = speed_cm_per_s * cosf(course_rad);
        float speed_to_east = speed_cm_per_s * sinf(course_rad);
        float speed_to_wnward = 0.0f;

        /* Set accuracy fields - use GPS DOP values if available, otherwise use defaults */
        float horizontal_accuracy = (gps_data.hdop > 0.0f) ? (gps_data.hdop * 5.0f) : 5.0f;
        float vertical_accuracy = (gps_data.vdop > 0.0f) ? (gps_data.vdop * 5.0f) : 10.0f;
        float speed_accuracy = 0.5f;

        /* Create fully-filled GPS data push command frame */
        gps_data_push_command_frame_t gps_frame = {
            .year_month_day = year_month_day,
            .hour_minute_second = hour_minute_second,
            .gps_longitude = gps_longitude,
            .gps_latitude = gps_latitude,
            .height = height,
            .speed_to_north = speed_to_north,
            .speed_to_east = speed_to_east,
            .speed_to_wnward = speed_to_wnward,
            .vertical_accuracy = vertical_accuracy,
            .horizontal_accuracy = horizontal_accuracy,
            .speed_accuracy = speed_accuracy,
            .satellite_number = gps_data.satellite_count,
        };

        /* Push GPS data to all connected cameras using DJI protocol (CmdSet=0x00, CmdID=0x17) */
        int cameras_receiving_gps = 0;
        for (int cam_idx = 0; cam_idx < NUM_CAMERAS; cam_idx++) {
            /* Sleep mode detection: When power_mode == 3, GPS updates must be stopped
             * for that camera to avoid BLE write failures. GPS sending resumes automatically
             * when camera wakes (power_mode != 3).
             */
            if (g_camera_states[cam_idx].connection_state == CAM_STATE_CONNECTED && 
                !g_camera_states[cam_idx].is_sleeping) {
                
                command_logic_push_gps_data(cam_idx, &gps_frame);
                cameras_receiving_gps++;
            }
        }
        
        /* Log GPS data being sent periodically */
        static uint32_t last_tx_log = 0;
        if (current_time - last_tx_log > 5000) {
            ESP_LOGI(TAG_TASK, "Sending GPS to %d camera(s) (0x00/0x17): Lat=%.6f, Lon=%.6f, Alt=%.1fm, Sats=%d, HAcc=%.1fm, VAcc=%.1fm, ts=%ld/%06ld",
                     cameras_receiving_gps,
                     gps_data.latitude, gps_data.longitude, gps_data.altitude,
                     gps_data.satellite_count, horizontal_acc, vertical_acc,
                     (long)year_month_day, (long)hour_minute_second);
            last_tx_log = current_time;
        }
        
        /* Update last send time for throttling */
        last_gps_send_time = current_time;
    }
}
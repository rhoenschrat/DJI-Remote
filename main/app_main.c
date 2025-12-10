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
 * - GPIO trigger support for external hardware
 * 
 * Hardware: M5Stack Basic V2.7 (ESP32-based)
 * Display: 320x240 TFT LCD
 * Connectivity: Bluetooth Low Energy
 */

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <time.h>
#include <math.h>

#include "connect_logic.h"
#include "light_logic.h"
#include "m5stack_basic_v27_hal.h"
#include "ui.h"
#include "../gps/gps_reader.h"
#include "command_logic.h"
#include "freertos/task.h"
#include "dji_protocol_data_structures.h"
#include "../log_config.h"

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
 * 4. User interface system (including GPIO triggers)
 * 
 * Main loop handles:
 * - Power button monitoring (3-second hold for shutdown)
 * - User input from physical buttons
 * - GPIO trigger processing from external hardware
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

    /* 
     * HARDWARE INITIALIZATION PHASE
     * Initialize all hardware components in dependency order
     */
    
    /* Initialize M5Stack Basic V2.7 hardware platform
     * This includes: display controller, button GPIO, power management,
     * I2C bus, SPI bus, and other core hardware peripherals
     */
    res = m5stack_basic_v27_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize M5Stack Basic V2.7 hardware");
        return;
    }
    ESP_LOGI(TAG, "M5Stack Basic V2.7 hardware initialized");

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

    /* Initialize user interface system
     * Sets up: display rendering, screen management, button handlers,
     * GPIO triggers, camera state management, NVS storage
     */
    ui_init();
    ESP_LOGI(TAG, "UI system initialized");

    /* System ready - log operational information for user */
    ESP_LOGI(TAG, "System ready - Icon-based UI active!");
    ESP_LOGI(TAG, "Button A: Select/execute current option");
    ESP_LOGI(TAG, "Button B: Cycle through options");

    /*
     * MAIN APPLICATION EVENT LOOP
     * Handles all user input, system events, and display updates
     */
    
    /* Power button state tracking for shutdown detection */
    static uint32_t power_button_hold_time = 0;
    static bool power_button_was_pressed = false;
    
    while (1) {
        /*
         * POWER MANAGEMENT
         * Monitor power button for 3-second hold to initiate shutdown
         */
        bool power_button_pressed = m5stack_basic_v27_button_c_pressed();
        
        if (power_button_pressed && !power_button_was_pressed) {
            /* Power button press detected - start hold timer */
            power_button_hold_time = 0;
            power_button_was_pressed = true;
            ESP_LOGI(TAG, "Power button pressed - hold for 3s to shutdown");
            
        } else if (power_button_pressed && power_button_was_pressed) {
            /* Power button held - increment timer and check for shutdown threshold */
            power_button_hold_time += 50;
            
            if (power_button_hold_time >= 3000) {
                ESP_LOGI(TAG, "Power button held for 3s - shutting down");
                
                /* Display shutdown message to user */
                ui_show_message("Shutting down...", M5_COLOR_RED, 1000);
                
                /* M5Stack Basic V2.7 does not have a power hold circuit
                 * Power management is handled differently - may need to enter deep sleep
                 * For now, just log the shutdown request
                 */
                ESP_LOGI(TAG, "Power button held - shutdown requested (power management not implemented)");
                
                /* Infinite loop in case shutdown fails */
                while(1) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        } else if (!power_button_pressed && power_button_was_pressed) {
            /* Power button released before shutdown threshold - reset timer */
            power_button_was_pressed = false;
            power_button_hold_time = 0;
        }

        /*
         * USER INPUT HANDLING
         * Process physical button presses with debouncing
         */
        
        /* Button A: Screen-aware action
         * Main Screen: Shutter control (photo/video)
         * Pairing Screen: Select item
         * Settings Screen: Execute action
         */
        if (m5stack_basic_v27_button_a_pressed()) {
            ESP_LOGI(TAG, "Button A pressed");
            ui_handle_button_a();
            
            /* Wait for button release to prevent multiple triggers */
            while (m5stack_basic_v27_button_a_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        /* Button B: Screen-aware navigation
         * Main Screen: Cycle camera selection (0 → 1 → 2 → All)
         * Pairing Screen: Cycle item selection
         * Settings Screen: Cycle item selection
         */
        if (m5stack_basic_v27_button_b_pressed()) {
            ESP_LOGI(TAG, "Button B pressed");
            ui_handle_button_b();
            
            /* Wait for button release to prevent multiple triggers */
            while (m5stack_basic_v27_button_b_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        /* Button C: Open pairing or settings
         * Only works on Main Screen for single camera selection
         * Opens Pairing Screen if camera slot is unpaired
         * Opens Settings Screen if camera slot is paired
         */
        if (m5stack_basic_v27_button_c_pressed()) {
            ESP_LOGI(TAG, "Button C pressed");
            ui_handle_button_c();
            
            /* Wait for button release to prevent multiple triggers */
            while (m5stack_basic_v27_button_c_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        /*
         * GPIO TRIGGER PROCESSING
         * Handle external GPIO triggers with randomized delays
         * This allows external hardware to trigger camera functions
         */
        ui_process_pending_gpio_actions();

        /*
         * WAKE-AND-RECORD PROCESSING
         * Process wake state machine for reliable camera wake-up and recording
         * Must be called before display update to ensure state is current
         */
        ui_process_wake_and_record();
        
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
            extern bool camera_status_initialized;
            bool is_camera_recording(void);
            
            /* Refresh Main Screen when recording to update elapsed time */
            if (camera_status_initialized && is_camera_recording()) {
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
        if (gps_age_ms > 5000) {
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

        /* Get current system time */
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        
        /* Apply timezone +8 offset (DJI reference uses timezone +8) */
        int original_hour = timeinfo.tm_hour;
        timeinfo.tm_hour = (timeinfo.tm_hour + 8) % 24;
        
        /* Handle day rollover if hour wrapped around */
        if (original_hour + 8 >= 24) {
            timeinfo.tm_mday++;
            /* Normalize date (handle month/year rollover) */
            time_t adjusted_time = mktime(&timeinfo);
            localtime_r(&adjusted_time, &timeinfo);
        }
        
        /* Format date as YYYYMMDD */
        int32_t year_month_day = (timeinfo.tm_year + 1900) * 10000 + 
                                 (timeinfo.tm_mon + 1) * 100 + 
                                 timeinfo.tm_mday;
        
        /* Format time as HHMMSS */
        int32_t hour_minute_second = timeinfo.tm_hour * 10000 + 
                                     timeinfo.tm_min * 100 + 
                                     timeinfo.tm_sec;

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
            ESP_LOGI(TAG_TASK, "Sending GPS to %d camera(s) (0x00/0x17): Lat=%.6f, Lon=%.6f, Alt=%.1fm, Sats=%d, HAcc=%.1fm, VAcc=%.1fm",
                     cameras_receiving_gps,
                     gps_data.latitude, gps_data.longitude, gps_data.altitude,
                     gps_data.satellite_count, horizontal_acc, vertical_acc);
            last_tx_log = current_time;
        }
        
        /* Update last send time for throttling */
        last_gps_send_time = current_time;
    }
}
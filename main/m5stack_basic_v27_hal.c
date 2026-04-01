/*
 * DJI Camera Remote Control - M5Stack Basic V2.7 Hardware Abstraction Layer
 * 
 * This file implements the complete hardware abstraction layer for the M5Stack Basic V2.7
 * development board, providing unified interfaces for all hardware components:
 * 
 * Hardware Components:
 * - ST7789 TFT LCD Display (320x240, 16-bit color)
 * - Power management (backlight PWM)
 * - Physical buttons (A, B, C)
 * - I2C bus (internal sensors communication)
 * - SPI bus (display communication)
 * - GPIO control and monitoring
 * 
 * Display Features:
 * - Hardware-accelerated bitmap rendering
 * - Text rendering with scalable 8x8 font
 * - Color fill operations with RGB565 format
 * - Transparent bitmap drawing
 * - Configurable brightness control
 * 
 * The HAL provides a clean interface for the UI layer while handling all
 * low-level hardware details, timing, and ESP-IDF driver integration.
 * 
 * Hardware: M5Stack Basic V2.7 (ESP32)
 * Display: ST7789 controller, 320x240 resolution
 * Framework: ESP-IDF v5.5 with ESP-LCD drivers
 * 
 * Based on M5Stack hardware specifications
 */

#include "m5stack_basic_v27_hal.h"
#include "driver/gpio.h"
/* I2C driver not currently used - sensors not implemented
 * #include "driver/i2c.h"
 */
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include "esp_heap_caps.h"

/* LCD support for M5Stack Basic V2.7 */
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

/* LVGL port for ESP-IDF */
#include "esp_lvgl_port.h"

/* 8x8 font removed in Phase 6 -- LVGL Montserrat fonts used instead */

/* Logging tag for ESP_LOG functions */
static const char *TAG = "M5STACK_BASIC_V27_HAL";

/* ESP-LCD driver handles for display communication */
static esp_lcd_panel_handle_t panel_handle = NULL;  /* ST7789 panel handle */
static esp_lcd_panel_io_handle_t io_handle = NULL;  /* SPI I/O handle */

/* Old draw helpers removed in Phase 6 -- rendering now handled by LVGL */

/* I2C Bus Configuration - not currently used, sensors not implemented
 * If sensors are needed in the future, use the new i2c_master.h API
 */

/**
 * @brief Initialize all M5Stack Basic V2.7 hardware components
 * 
 * Performs complete hardware initialization in the correct dependency order:
 * 1. Power management (backlight control)
 * 2. I2C bus (for internal sensors communication)
 * 3. Display subsystem (ILI9341 TFT LCD)
 * 4. Button inputs (A, B, C buttons)
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stack_basic_v27_init(void) {
    ESP_LOGI(TAG, "Initializing M5Stack Basic V2.7 hardware");
    
    /* Initialize power management - backlight control */
    int ret = m5stack_basic_v27_power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power management");
        return ret;
    }
    
    /* I2C bus initialization skipped - not currently used for sensors
     * If I2C sensors are needed in the future, migrate to new i2c_master.h API
     * ret = m5stack_basic_v27_i2c_init();
     */
    
    /* Initialize display subsystem (SPI, ST7789 controller) */
    ret = m5stack_basic_v27_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display");
        return ret;
    }
    
    /* Initialize button input handling */
    ret = m5stack_basic_v27_buttons_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buttons");
        return ret;
    }
    
    ESP_LOGI(TAG, "M5Stack Basic V2.7 hardware initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize M5Stack Basic V2.7 power management
 * 
 * Sets up display backlight control.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stack_basic_v27_power_init(void) {
    ESP_LOGI(TAG, "Initializing power management for M5Stack Basic V2.7");
    
    /* Configure display backlight control pin
     * Used for both on/off control and PWM brightness adjustment
     */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_LCD_BL_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure backlight pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Set backlight to OFF initially - will be enabled after boot logo is rendered */
    gpio_set_level(M5_LCD_BL_PIN, 0);
    
    ESP_LOGI(TAG, "Power management initialized - Backlight GPIO configured and set to OFF (BL=%d)", M5_LCD_BL_PIN);
    return ESP_OK;
}

/* I2C initialization function removed - not currently used
 * If sensors are needed, implement using new i2c_master.h API
 */

int m5stack_basic_v27_display_init(void) {
    ESP_LOGI(TAG, "Display init - Step 1: GPIO initialization");
    ESP_LOGI(TAG, "Pins: MOSI=%d, SCLK=%d, CS=%d, DC=%d, RST=%d, BL=%d", 
             M5_LCD_MOSI_PIN, M5_LCD_SCLK_PIN, M5_LCD_CS_PIN, 
             M5_LCD_DC_PIN, M5_LCD_RST_PIN, M5_LCD_BL_PIN);
    
    // Step 1: Configure the GPIO pins
    ESP_LOGI(TAG, "Configuring display control GPIOs...");
    
    // Configure reset pin
    gpio_config_t rst_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_LCD_RST_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&rst_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure RST pin: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "RST pin configured");
    
    // Configure DC pin
    gpio_config_t dc_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_LCD_DC_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ret = gpio_config(&dc_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DC pin: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "DC pin configured");
    
    // Configure CS pin
    gpio_config_t cs_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_LCD_CS_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ret = gpio_config(&cs_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS pin: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(M5_LCD_CS_PIN, 1);  // CS high (inactive)
    ESP_LOGI(TAG, "CS pin configured and set high");
    
    // Perform a simple reset sequence
    ESP_LOGI(TAG, "Performing display reset sequence...");
    gpio_set_level(M5_LCD_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(M5_LCD_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Display reset complete");
    
    // Step 2: Initialize SPI bus
    ESP_LOGI(TAG, "Step 2: Initializing SPI bus...");
    ESP_LOGI(TAG, "Initializing VSPI bus (MOSI=%d, MISO=%d, SCLK=%d)...", 
             M5_LCD_MOSI_PIN, 19, M5_LCD_SCLK_PIN);
    
    spi_bus_config_t buscfg = {
        .mosi_io_num = M5_LCD_MOSI_PIN,  // GPIO23
        .miso_io_num = 19,  // GPIO19 - not used by display but configured for SPI bus compatibility
        .sclk_io_num = M5_LCD_SCLK_PIN,  // GPIO18
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    // Use VSPI_HOST (SPI3) for display
    // Use SPI_DMA_CH_AUTO to let ESP-IDF choose the DMA channel
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SPI bus initialized on VSPI_HOST (SPI3)");
    
    // Step 3: Create LCD panel IO handle
    ESP_LOGI(TAG, "Step 3: Creating LCD panel IO...");
    ESP_LOGI(TAG, "LCD panel IO using VSPI_HOST (SPI3), CS=GPIO%d, DC=GPIO%d", 
             M5_LCD_CS_PIN, M5_LCD_DC_PIN);
    
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = M5_LCD_DC_PIN,
        .cs_gpio_num = M5_LCD_CS_PIN,
        .pclk_hz = 26 * 1000 * 1000,  // 26MHz for better performance
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)VSPI_HOST, &io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        spi_bus_free(VSPI_HOST);
        return ret;
    }
    ESP_LOGI(TAG, "LCD panel IO created successfully on VSPI_HOST");
    
    // Step 4: Create ST7789 panel
    // Note: M5Stack Basic V2.7 uses ST7789 display controller (compatible with ILI9341)
    ESP_LOGI(TAG, "Step 4: Creating ST7789 panel...");
    
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = M5_LCD_RST_PIN,
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = 16,
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = NULL,
    };
    
    ret = esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ST7789 panel: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "ST7789 panel created");
    
    // Step 5: Initialize the panel
    ESP_LOGI(TAG, "Step 5: Initializing panel...");
    
    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset panel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init panel: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Panel initialized");
    
    // Step 6: Configure display settings
    ESP_LOGI(TAG, "Step 6: Configuring display settings...");
    
    // ILI9342C (compatible with ST7789) typically needs color inversion
    esp_lcd_panel_invert_color(panel_handle, true);
    
    // Configure orientation for landscape mode (320x240)
    // Standard landscape orientation - no swap or mirror needed
    esp_lcd_panel_swap_xy(panel_handle, false);
    esp_lcd_panel_mirror(panel_handle, false, false);
    
    // Turn on display
    esp_lcd_panel_disp_on_off(panel_handle, true);
    
    ESP_LOGI(TAG, "Display initialization complete!");
    
    // Small delay to ensure display is ready
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Clear display to black (single initialization clear)
    ESP_LOGI(TAG, "Clearing display to black...");
    m5stack_basic_v27_display_clear(M5_COLOR_BLACK);
    
    /* Backlight stays OFF — enabled by app_main after LVGL splash screen
     * is rendered (see ADR-010) */

    return ESP_OK;
}

/**
 * @brief Initialize button input handling
 * 
 * Configures GPIO pins for the three physical buttons on M5Stack Basic V2.7:
 * - Button A (GPIO39): Primary action button
 * - Button B (GPIO38): Secondary/navigation button  
 * - Button C (GPIO37): Third button
 * 
 * Note: GPIOs 37, 39 are input-only pins on ESP32 and cannot use
 * internal pull-up resistors. M5Stack Basic V2.7 has external pull-ups.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stack_basic_v27_buttons_init(void) {
    /* Configure button pins as inputs
     * GPIOs 34-39 are input-only on ESP32 and have NO internal pull-up/pull-down resistors
     * Buttons A (39), B (38), C (37) all fall in this range
     * M5Stack Basic V2.7 hardware provides external pull-up resistors for all buttons
     */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << M5_BTN_A_PIN) | (1ULL << M5_BTN_B_PIN) | (1ULL << M5_BTN_C_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,  /* No internal pull-ups available; external on M5Stack */
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Buttons initialized - A=%d, B=%d, C=%d", M5_BTN_A_PIN, M5_BTN_B_PIN, M5_BTN_C_PIN);
    return ESP_OK;
}

/**
 * @brief Check if Button A is currently pressed
 * 
 * Button A is the primary action button used for executing current screen functions.
 * Returns true when button is physically pressed (GPIO reads LOW).
 * 
 * @return true if button is pressed, false otherwise
 */
bool m5stack_basic_v27_button_a_pressed(void) {
    return gpio_get_level(M5_BTN_A_PIN) == 0;
}

/**
 * @brief Check if Button B is currently pressed
 * 
 * Button B is used for navigation between different screens and functions.
 * Returns true when button is physically pressed (GPIO reads LOW).
 * 
 * @return true if button is pressed, false otherwise
 */
bool m5stack_basic_v27_button_b_pressed(void) {
    return gpio_get_level(M5_BTN_B_PIN) == 0;
}

/**
 * @brief Check if Button C is currently pressed
 * 
 * Button C is the third button available on M5Stack Basic V2.7.
 * Returns true when button is physically pressed (GPIO reads LOW).
 * 
 * @return true if button is pressed, false otherwise
 */
bool m5stack_basic_v27_button_c_pressed(void) {
    return gpio_get_level(M5_BTN_C_PIN) == 0;
}

/**
 * @brief Set display backlight brightness
 * 
 * Controls display brightness using PWM on the backlight pin.
 * Initializes LEDC PWM controller on first call for smooth brightness control.
 * 
 * @param brightness Brightness level (0-255, 0=off, 255=maximum)
 */
void m5stack_basic_v27_display_set_brightness(uint8_t brightness) {
    /* Initialize PWM controller once for backlight control */
    static bool ledc_initialized = false;
    
    if (!ledc_initialized) {
        /* Configure LEDC timer for 8-bit PWM at 5kHz (flicker-free) */
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .duty_resolution = LEDC_TIMER_8_BIT,  /* 8-bit resolution (0-255) */
            .freq_hz = 5000,                      /* 5kHz frequency to avoid flicker */
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&ledc_timer);
        
        /* Configure LEDC channel for backlight control */
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = M5_LCD_BL_PIN,            /* Backlight control pin */
            .duty = 0,
            .hpoint = 0,
        };
        ledc_channel_config(&ledc_channel);
        ledc_initialized = true;
    }
    
    /* Set PWM duty cycle for brightness control (0-255) */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    
    ESP_LOGI(TAG, "Display brightness set to %d/255", brightness);
}

/**
 * @brief Get the ESP-LCD panel handle for direct access
 * 
 * Returns the internal LCD panel handle for advanced operations
 * that require direct ESP-LCD API access.
 * 
 * @return ESP-LCD panel handle, or NULL if not initialized
 */
esp_lcd_panel_handle_t m5stack_basic_v27_get_display_handle(void) {
    return panel_handle;
}

/**
 * @brief Clear entire display with solid color
 * 
 * Fills the complete display area (320x240) with the specified color.
 * Uses small chunked rendering to fit in internal RAM (no PSRAM available).
 * 
 * @param color RGB565 color value to fill the display with
 */
void m5stack_basic_v27_display_clear(uint16_t color) {
    if (panel_handle == NULL) {
        ESP_LOGE(TAG, "Display clear: panel_handle is NULL");
        return;
    }
    
    ESP_LOGI(TAG, "Clearing display with color 0x%04X", color);
    
    /* Clear display using small chunked approach to fit in internal RAM
     * Use only 8 lines at a time to minimize memory usage
     */
    const int chunk_height = 8;  /* Process only 8 rows at a time */
    const int total_pixels = M5_LCD_H_RES * chunk_height;
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(total_pixels * sizeof(uint16_t), MALLOC_CAP_DMA);
    
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate display clear buffer (%d bytes)", total_pixels * sizeof(uint16_t));
        return;
    }
    
    ESP_LOGI(TAG, "Allocated buffer: %d bytes for %d pixels", total_pixels * sizeof(uint16_t), total_pixels);
    
    /* Fill buffer with solid color (RGB565 format) */
    for (int i = 0; i < total_pixels; i++) {
        buffer[i] = color;
    }
    
    /* Render display in small horizontal chunks for memory efficiency */
    int chunks_rendered = 0;
    for (int y = 0; y < M5_LCD_V_RES; y += chunk_height) {
        int height = (y + chunk_height > M5_LCD_V_RES) ? (M5_LCD_V_RES - y) : chunk_height;
        esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, y, M5_LCD_H_RES, y + height, buffer);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw bitmap at y=%d: %s", y, esp_err_to_name(ret));
        } else {
            chunks_rendered++;
        }
    }
    
    heap_caps_free(buffer);
    ESP_LOGI(TAG, "Display clear complete: %d chunks rendered", chunks_rendered);
}

void m5stack_basic_v27_backlight_on(void) {
    gpio_set_level(M5_LCD_BL_PIN, 1);
    ESP_LOGI(TAG, "Backlight enabled on GPIO %d", M5_LCD_BL_PIN);
}

lv_display_t *m5stack_basic_v27_lvgl_init(void) {
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t ret = lvgl_port_init(&lvgl_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LVGL port: %s", esp_err_to_name(ret));
        return NULL;
    }

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = M5_LCD_H_RES * 40,
        .double_buffer = true,
        .hres = M5_LCD_H_RES,
        .vres = M5_LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = true,
        },
    };

    lv_display_t *disp = lvgl_port_add_disp(&disp_cfg);
    if (disp == NULL) {
        ESP_LOGE(TAG, "Failed to add LVGL display");
        return NULL;
    }

    ESP_LOGI(TAG, "LVGL display initialized (%dx%d, double-buffered DMA)",
             M5_LCD_H_RES, M5_LCD_V_RES);
    return disp;
}

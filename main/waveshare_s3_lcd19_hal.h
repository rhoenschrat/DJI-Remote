/*
 * Waveshare ESP32-S3-LCD-1.9 Hardware Abstraction Layer Header
 */

#ifndef WAVESHARE_S3_LCD19_HAL_H
#define WAVESHARE_S3_LCD19_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_types.h"
#include "lvgl.h"

/* Display pins — from official Waveshare schematic (2025-11-29) */
#define WS_LCD_RST_PIN      9
#define WS_LCD_SCLK_PIN    10
#define WS_LCD_DC_PIN      11
#define WS_LCD_CS_PIN      12
#define WS_LCD_MOSI_PIN    13
#define WS_LCD_BL_PIN      14

/* Display resolution — 170x320 native, operated in landscape (swap_xy) */
#define WS_LCD_H_RES       320
#define WS_LCD_V_RES       170

/* Button pins — external buttons on header, active LOW, internal pull-up */
#define WS_BTN_A_PIN        5
#define WS_BTN_B_PIN        6
#define WS_BTN_C_PIN        16

/* Function prototypes */

/**
 * @brief Initialize all Waveshare ESP32-S3-LCD-1.9 hardware
 * @return ESP_OK on success, error code otherwise
 */
int waveshare_s3_lcd19_init(void);

/**
 * @brief Initialize power management (backlight GPIO)
 * @return ESP_OK on success, error code otherwise
 */
int waveshare_s3_lcd19_power_init(void);

/**
 * @brief Initialize display
 * @return ESP_OK on success, error code otherwise
 */
int waveshare_s3_lcd19_display_init(void);

/**
 * @brief Initialize buttons
 * @return ESP_OK on success, error code otherwise
 */
int waveshare_s3_lcd19_buttons_init(void);

/**
 * @brief Check if button A is pressed
 * @return true if pressed, false otherwise
 */
bool waveshare_s3_lcd19_button_a_pressed(void);

/**
 * @brief Check if button B is pressed
 * @return true if pressed, false otherwise
 */
bool waveshare_s3_lcd19_button_b_pressed(void);

/**
 * @brief Check if button C is pressed
 * @return true if pressed, false otherwise
 */
bool waveshare_s3_lcd19_button_c_pressed(void);

/**
 * @brief Set display brightness
 * @param brightness Brightness level (0-255)
 */
void waveshare_s3_lcd19_display_set_brightness(uint8_t brightness);

/**
 * @brief Get display panel handle
 * @return Display panel handle
 */
esp_lcd_panel_handle_t waveshare_s3_lcd19_get_display_handle(void);

/**
 * @brief Clear display with specified color (used during boot before LVGL starts)
 * @param color 16-bit RGB565 color
 */
void waveshare_s3_lcd19_display_clear(uint16_t color);

/**
 * @brief Turn the display backlight on
 *
 * Called by app_main after the LVGL splash screen has been rendered,
 * ensuring the first visible frame is the splash (see ADR-010).
 */
void waveshare_s3_lcd19_backlight_on(void);

/**
 * @brief Initialize LVGL port with the existing display panel
 *
 * Must be called after waveshare_s3_lcd19_display_init(). Sets up LVGL core,
 * creates a FreeRTOS task for lv_timer_handler(), and registers the
 * ST7789V2 panel as an LVGL display with double-buffered PSRAM rendering.
 *
 * @return LVGL display handle, or NULL on failure
 */
lv_display_t *waveshare_s3_lcd19_lvgl_init(void);

#endif /* WAVESHARE_S3_LCD19_HAL_H */

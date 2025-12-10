/*
 * M5Stack Basic V2.7 Hardware Abstraction Layer Header
 */

#ifndef M5STACK_BASIC_V27_HAL_H
#define M5STACK_BASIC_V27_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_types.h"

/* M5Stack Basic V2.7 GPIO Pin Definitions (ESP32) */

/* Display pins (ILI9342C) - M5Stack Basic V2.7 verified pins
 * According to documentation:
 * ESP32: G23, G19, G18, G14, G27, G33, G32, G4
 * ILI9342C: MOSI, (unused), CLK, CS, DC, RST, BL, (unused)
 */
#define M5_LCD_MOSI_PIN     23  /* SPI MOSI (GPIO23) */
#define M5_LCD_SCLK_PIN     18  /* SPI CLK (GPIO18) */
#define M5_LCD_CS_PIN       14  /* Chip Select (GPIO14) - CORRECTED */
#define M5_LCD_DC_PIN       27  /* Data/Command (GPIO27) - CORRECTED */
#define M5_LCD_RST_PIN      33  /* Reset (GPIO33) - CORRECTED */
#define M5_LCD_BL_PIN       32  /* Backlight (GPIO32) - CORRECTED */

/* Display resolution - M5Stack Basic V2.7 in landscape mode */
#define M5_LCD_H_RES        320
#define M5_LCD_V_RES        240

/* Button pins - M5Stack Basic V2.7 actual pins */
#define M5_BTN_A_PIN        39  /* Button A - Note: GPIO39 is input-only on ESP32 */
#define M5_BTN_B_PIN        38  /* Button B */
#define M5_BTN_C_PIN        37  /* Button C - Note: GPIO37 is input-only on ESP32 */

/* I2C pins (for internal sensors) - DISABLED
 * I2C is not used in this project. GPIO22 (M5_I2C_SCL_PIN) is reserved for external Button C.
 * GPIO21 (M5_I2C_SDA_PIN) is used for external Button B.
 */
#define M5_I2C_SDA_PIN      21
#define M5_I2C_SCL_PIN      22

/* Power control - M5Stack Basic V2.7 may not have power hold circuit */
/* Note: Review power management requirements for M5Stack Basic */

/* Function prototypes */

/**
 * @brief Initialize all M5Stack Basic V2.7 hardware
 * @return ESP_OK on success, error code otherwise
 */
int m5stack_basic_v27_init(void);

/**
 * @brief Initialize power management
 * @return ESP_OK on success, error code otherwise
 */
int m5stack_basic_v27_power_init(void);

/* I2C init removed - not currently used, sensors not implemented */

/**
 * @brief Initialize display
 * @return ESP_OK on success, error code otherwise
 */
int m5stack_basic_v27_display_init(void);

/**
 * @brief Initialize buttons
 * @return ESP_OK on success, error code otherwise
 */
int m5stack_basic_v27_buttons_init(void);

/**
 * @brief Check if button A is pressed
 * @return true if pressed, false otherwise
 */
bool m5stack_basic_v27_button_a_pressed(void);

/**
 * @brief Check if button B is pressed
 * @return true if pressed, false otherwise
 */
bool m5stack_basic_v27_button_b_pressed(void);

/**
 * @brief Check if button C is pressed
 * @return true if pressed, false otherwise
 */
bool m5stack_basic_v27_button_c_pressed(void);

/**
 * @brief Set display brightness
 * @param brightness Brightness level (0-255)
 */
void m5stack_basic_v27_display_set_brightness(uint8_t brightness);

/**
 * @brief Get display panel handle
 * @return Display panel handle
 */
esp_lcd_panel_handle_t m5stack_basic_v27_get_display_handle(void);

/**
 * @brief Clear display with specified color
 * @param color 16-bit RGB565 color
 */
void m5stack_basic_v27_display_clear(uint16_t color);

/**
 * @brief Show boot logo
 * Displays the 320x240 RGB565 boot logo stored in boot_logo.c
 */
void m5stack_basic_v27_display_show_boot_logo(void);

/**
 * @brief Print text on display
 * @param x X coordinate
 * @param y Y coordinate
 * @param text Text to display
 * @param color Text color (RGB565)
 */
void m5stack_basic_v27_display_print(int x, int y, const char *text, uint16_t color);

/**
 * @brief Print scaled text on display
 * @param x X coordinate
 * @param y Y coordinate
 * @param text Text to display
 * @param color Text color (RGB565)
 * @param scale Scale factor (1=normal, 2=double size, etc.)
 */
void m5stack_basic_v27_display_print_scaled(int x, int y, const char *text, uint16_t color, int scale);

/**
 * @brief Draw a bitmap using proper buffer-based approach
 * @param x X coordinate
 * @param y Y coordinate
 * @param width Bitmap width
 * @param height Bitmap height
 * @param bitmap Bitmap data
 * @param color Color for set bits
 * @param bg_color Background color for unset bits
 */
void m5stack_basic_v27_display_draw_bitmap(int x, int y, int width, int height, const uint8_t *bitmap, uint16_t color, uint16_t bg_color);

/**
 * @brief Draw a filled circle using buffer-based approach
 * @param x Center X coordinate
 * @param y Center Y coordinate
 * @param radius Circle radius
 * @param color Fill color (RGB565)
 */
void m5stack_basic_v27_display_fill_circle(int x, int y, int radius, uint16_t color);

/**
 * @brief Draw a filled rectangle
 * @param x X coordinate
 * @param y Y coordinate
 * @param width Rectangle width
 * @param height Rectangle height
 * @param color Fill color (RGB565)
 */
void m5stack_basic_v27_display_fill_rect(int x, int y, int width, int height, uint16_t color);

/* Color definitions (RGB565 but display uses GBR ordering) */
#define M5_COLOR_BLACK      0x0000
#define M5_COLOR_WHITE      0xFFFF
#define M5_COLOR_RED        0x07E0  // GBR: G=63 appears as RED on screen
#define M5_COLOR_GREEN      0x001F  // GBR: B=31 appears as GREEN on screen
#define M5_COLOR_BLUE       0xF800  // GBR: R=31 appears as BLUE on screen
#define M5_COLOR_YELLOW     0x07FF  // GBR: G+B appears as YELLOW on screen
#define M5_COLOR_CYAN       0xF81F  // GBR: R+B appears as CYAN on screen
#define M5_COLOR_MAGENTA    0xFFE0  // GBR: R+G appears as MAGENTA on screen
#define M5_COLOR_ORANGE     0xFDA0  
#define M5_COLOR_PURPLE     0x8010  // Purple (approximate for GBR)
#define M5_COLOR_DARKGREY   0x39C6  // Dark grey for indicators
#define M5_COLOR_GREY       0x7BEF  // Medium grey for text
#define M5_COLOR_GRAY       0x8410  // Button background gray (r=128, g=128, b=128)

#endif /* M5STACK_BASIC_V27_HAL_H */


/*
 * DJI Camera Remote Control - Waveshare ESP32-S3-LCD-1.9 Hardware Abstraction Layer
 *
 * This file implements the HAL for the Waveshare ESP32-S3-LCD-1.9 board:
 *   - ST7789V2 TFT LCD Display (170x320 native, 320x170 landscape)
 *   - Backlight PWM on GPIO14
 *   - Three external buttons on GPIO5/6/7 (active LOW, internal pull-up)
 *   - No AXP192 / I2C power controller — direct GPIO backlight
 *
 * Hardware: Waveshare ESP32-S3-LCD-1.9 (ESP32-S3R8, 8 MB PSRAM, 16 MB Flash)
 * Display: ST7789V2, 170x320 native resolution, SPI interface
 * Framework: ESP-IDF v5.5.1 with ESP-LCD drivers
 */

#include "waveshare_s3_lcd19_hal.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include "esp_heap_caps.h"

/* LCD support */
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

/* LVGL port for ESP-IDF */
#include "esp_lvgl_port.h"

static const char *TAG = "WS_S3_LCD19_HAL";

/* ESP-LCD driver handles */
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;

int waveshare_s3_lcd19_init(void) {
    ESP_LOGI(TAG, "Initializing Waveshare ESP32-S3-LCD-1.9 hardware");

    int ret = waveshare_s3_lcd19_power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power management");
        return ret;
    }

    ret = waveshare_s3_lcd19_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display");
        return ret;
    }

    ret = waveshare_s3_lcd19_buttons_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buttons");
        return ret;
    }

    ESP_LOGI(TAG, "Waveshare ESP32-S3-LCD-1.9 hardware initialized successfully");
    return ESP_OK;
}

int waveshare_s3_lcd19_power_init(void) {
    ESP_LOGI(TAG, "Initializing power management");

    /* Configure backlight pin as output, start OFF */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << WS_LCD_BL_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure backlight pin: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Backlight is active LOW on this board: LOW = on, HIGH = off.
     * Start with backlight OFF (GPIO HIGH). */
    gpio_set_level(WS_LCD_BL_PIN, 1);
    ESP_LOGI(TAG, "Power management initialized - Backlight GPIO%d OFF (active-low)", WS_LCD_BL_PIN);
    return ESP_OK;
}

int waveshare_s3_lcd19_display_init(void) {
    ESP_LOGI(TAG, "Display init - Pins: MOSI=%d, SCLK=%d, CS=%d, DC=%d, RST=%d, BL=%d",
             WS_LCD_MOSI_PIN, WS_LCD_SCLK_PIN, WS_LCD_CS_PIN,
             WS_LCD_DC_PIN, WS_LCD_RST_PIN, WS_LCD_BL_PIN);

    /* Configure reset pin */
    gpio_config_t rst_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << WS_LCD_RST_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&rst_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure RST pin: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure DC pin */
    gpio_config_t dc_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << WS_LCD_DC_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ret = gpio_config(&dc_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DC pin: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure CS pin */
    gpio_config_t cs_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << WS_LCD_CS_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ret = gpio_config(&cs_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS pin: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(WS_LCD_CS_PIN, 1);

    /* Hardware reset sequence */
    gpio_set_level(WS_LCD_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(WS_LCD_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Display reset complete");

    /* Initialize SPI bus — use SPI2_HOST (VSPI_HOST does not exist on ESP32-S3) */
    spi_bus_config_t buscfg = {
        .mosi_io_num = WS_LCD_MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = WS_LCD_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = WS_LCD_H_RES * 40 * sizeof(uint16_t),
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SPI bus initialized on SPI2_HOST");

    /* Create LCD panel IO handle */
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = WS_LCD_DC_PIN,
        .cs_gpio_num = WS_LCD_CS_PIN,
        .pclk_hz = 26 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        spi_bus_free(SPI2_HOST);
        return ret;
    }
    ESP_LOGI(TAG, "LCD panel IO created on SPI2_HOST");

    /* Create ST7789 panel */
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = WS_LCD_RST_PIN,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
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

    /* Initialize the panel */
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

    /* Color inversion — required for correct colors on ST7789V2 */
    esp_lcd_panel_invert_color(panel_handle, true);

    /* RAM offset for portrait clear (MV=0, MADCTL swap_xy not yet applied).
     * ST7789 has 240 physical columns, but this panel uses only 170, centred
     * at column 35 (columns 35..204 are visible).
     * x_gap=35 adds 35 to CASET so the clear writes to columns 35..204.
     * In portrait mode (MV=0): CASET = columns (0..239), RASET = rows (0..319).
     * This gap is overridden in lvgl_init() to (0, 35) for landscape (MV=1). */
    esp_lcd_panel_set_gap(panel_handle, 35, 0);

    /* Turn on display */
    esp_lcd_panel_disp_on_off(panel_handle, true);
    ESP_LOGI(TAG, "Display initialization complete");

    vTaskDelay(pdMS_TO_TICKS(50));

    /* Clear display to black before LVGL takes over */
    waveshare_s3_lcd19_display_clear(0x0000);

    /* Backlight stays OFF — enabled by app_main after LVGL splash screen */
    return ESP_OK;
}

int waveshare_s3_lcd19_buttons_init(void) {
    /* ESP32-S3 GPIOs 5/6/7 support internal pull-ups */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << WS_BTN_A_PIN) | (1ULL << WS_BTN_B_PIN) | (1ULL << WS_BTN_C_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Buttons initialized - A=%d, B=%d, C=%d", WS_BTN_A_PIN, WS_BTN_B_PIN, WS_BTN_C_PIN);
    return ESP_OK;
}

bool waveshare_s3_lcd19_button_a_pressed(void) {
    return gpio_get_level(WS_BTN_A_PIN) == 0;
}

bool waveshare_s3_lcd19_button_b_pressed(void) {
    return gpio_get_level(WS_BTN_B_PIN) == 0;
}

bool waveshare_s3_lcd19_button_c_pressed(void) {
    return gpio_get_level(WS_BTN_C_PIN) == 0;
}

void waveshare_s3_lcd19_display_set_brightness(uint8_t brightness) {
    static bool ledc_initialized = false;

    if (!ledc_initialized) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .freq_hz = 5000,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = WS_LCD_BL_PIN,
            .duty = 0,
            .hpoint = 0,
        };
        ledc_channel_config(&ledc_channel);
        ledc_initialized = true;
    }

    /* Active-low backlight: invert duty (255 = full brightness = duty 0) */
    uint8_t inverted_duty = 255 - brightness;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, inverted_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ESP_LOGI(TAG, "Display brightness set to %d/255 (duty=%d, active-low)", brightness, inverted_duty);
}

esp_lcd_panel_handle_t waveshare_s3_lcd19_get_display_handle(void) {
    return panel_handle;
}

void waveshare_s3_lcd19_display_clear(uint16_t color) {
    if (panel_handle == NULL) {
        ESP_LOGE(TAG, "Display clear: panel_handle is NULL");
        return;
    }

    /* Clear using chunked approach — 8 rows at a time with DMA memory.
     * Use native (portrait) resolution since this runs before LVGL
     * applies swap_xy rotation. Native: 170 columns x 320 rows. */
    const int native_w = WS_LCD_V_RES;  /* 170 (native columns) */
    const int native_h = WS_LCD_H_RES;  /* 320 (native rows) */
    const int chunk_height = 8;
    const int total_pixels = native_w * chunk_height;
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(total_pixels * sizeof(uint16_t), MALLOC_CAP_DMA);

    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate display clear buffer");
        return;
    }

    for (int i = 0; i < total_pixels; i++) {
        buffer[i] = color;
    }

    for (int y = 0; y < native_h; y += chunk_height) {
        int height = (y + chunk_height > native_h) ? (native_h - y) : chunk_height;
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, native_w, y + height, buffer);
    }

    heap_caps_free(buffer);
    ESP_LOGI(TAG, "Display cleared with color 0x%04X", color);
}

void waveshare_s3_lcd19_backlight_on(void) {
    /* Active-low backlight: LOW = on */
    gpio_set_level(WS_LCD_BL_PIN, 0);
    ESP_LOGI(TAG, "Backlight enabled on GPIO%d (active-low, set LOW)", WS_LCD_BL_PIN);
}

lv_display_t *waveshare_s3_lcd19_lvgl_init(void) {
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t ret = lvgl_port_init(&lvgl_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LVGL port: %s", esp_err_to_name(ret));
        return NULL;
    }

    /* Switch from portrait gap (35, 0) to landscape gap (0, 35).
     *
     * With MADCTL MV=1 (swap_xy), the ST7789 swaps its addressing:
     *   CASET addresses physical rows   (0..319) — landscape horizontal
     *   RASET addresses physical columns (0..239) — landscape vertical
     * The 170 visible columns sit at physical columns 35..204, so y_gap=35
     * offsets RASET into the visible range.  x_gap=0 because rows 0..319
     * need no offset.
     *
     * Portrait clear in display_init() used gap (35, 0) where MV=0 and
     * CASET addressed columns (35..204 visible). */
    esp_lcd_panel_set_gap(panel_handle, 0, 35);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = WS_LCD_H_RES * 40,
        .double_buffer = true,
        .hres = WS_LCD_H_RES,
        .vres = WS_LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = true,
            .mirror_x = true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
            .swap_bytes = true,
        },
    };

    lv_display_t *disp = lvgl_port_add_disp(&disp_cfg);
    if (disp == NULL) {
        ESP_LOGE(TAG, "Failed to add LVGL display");
        return NULL;
    }

    ESP_LOGI(TAG, "LVGL display initialized (%dx%d, double-buffered DMA, landscape)",
             WS_LCD_H_RES, WS_LCD_V_RES);
    return disp;
}

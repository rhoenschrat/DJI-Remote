#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "../main/ui.h"  // NUM_CAMERAS

#define TAG "LOGIC_LIGHT"

/* LED GPIO — board-specific via Kconfig board selector */
#if defined(CONFIG_BOARD_M5STACK_BASIC_V27)
#define LED_GPIO 2                /* M5Stack Basic WS2812 GPIO */
#elif defined(CONFIG_BOARD_WAVESHARE_S3_LCD19)
#define LED_GPIO 15               /* Waveshare onboard WS2812 (2 LEDs) */
#else
#define LED_GPIO 2                /* Fallback default */
#endif

#define LED_STRIP_LENGTH 1        // Set the number of LEDs to 1

// Create a led_strip handle
static led_strip_handle_t led_strip = NULL;

// Initialize RGB LED related configurations and settings
static void init_rgb_led(void) {
    // Configure LED
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_STRIP_LENGTH // Set the number of LEDs to 1
    };

    // Configure RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // Set RMT resolution to 10 MHz
        .flags.with_dma = false,           // Disable DMA
    };

    // Use &led_strip as the third parameter because it needs a pointer to led_strip_handle_t
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    led_strip_clear(led_strip); // Clear all LEDs and turn them off
    ESP_LOGI(TAG, "RGB LED initialized");
}

// Set RGB LED color
static void set_rgb_color(uint8_t red, uint8_t green, uint8_t blue) {
    led_strip_set_pixel(led_strip, 0, red, green, blue);  // Set LED color
    led_strip_refresh(led_strip); // Refresh LED Strip to update color
}

// Initialize variables needed for RGB LED status
uint8_t led_red = 0, led_green = 0, led_blue = 0;   // RGB values
bool led_blinking = false;                          // Whether to blink
bool current_led_on = false;                        // Current LED status (on or off)

#define LED_LEVEL 13   /* 255 * 0.05 */

static void set_led(uint8_t r, uint8_t g, uint8_t b, bool blink) {
    led_red = r;
    led_green = g;
    led_blue = b;
    led_blinking = blink;
}

static bool any_slot_connecting(void) {
    for (int i = 0; i < NUM_CAMERAS; i++) {
        if (connect_logic_slot_is_connecting(i)) return true;
    }
    return false;
}

static bool any_slot_connected(void) {
    for (int i = 0; i < NUM_CAMERAS; i++) {
        if (command_logic_slot_is_connected(i)) return true;
    }
    return false;
}

/* LED state computed from all slots; first match wins. Runs on the FreeRTOS
 * timer task and only reads state. */
static void update_led_state() {
    if (!connect_logic_is_ble_initialized()) {
        set_led(LED_LEVEL, 0, 0, false);                 /* red: BLE not initialized */
    } else if (connect_logic_is_boot_scan_active()) {
        set_led(0, 0, LED_LEVEL, true);                  /* blue blinking: boot scan */
    } else if (any_slot_connecting()) {
        set_led(0, 0, LED_LEVEL, false);                 /* blue: connect in progress */
    } else if (status_logic_any_slot_recording()) {
        set_led(0, LED_LEVEL, 0, true);                  /* green blinking: recording */
    } else if (any_slot_connected()) {
        set_led(0, LED_LEVEL, 0, false);                 /* green: at least one camera connected */
    } else {
        set_led(LED_LEVEL, LED_LEVEL, 0, false);         /* yellow: ready, nothing connected */
    }
}

// Timer callback function for periodic LED state updates
static void led_state_timer_callback(TimerHandle_t xTimer) {
    update_led_state();
}

// Timer callback function for LED blinking effect
static void led_blink_timer_callback(TimerHandle_t xTimer) {
    if (led_blinking) {
        // If in blinking state and LED is currently on, turn it off
        if (current_led_on) {
            set_rgb_color(0, 0, 0);  // Turn off LED
        } else {
            // If LED is currently off, set it to RGB color
            set_rgb_color(led_red, led_green, led_blue);
        }
        current_led_on = !current_led_on;
    } else {
        // If not in blinking state, directly set to RGB color
        set_rgb_color(led_red, led_green, led_blue);
    }
}

// Initialize light logic, including LED state updates and blink timer
int init_light_logic() {
    init_rgb_led();
    
    // Create a timer that executes update_led_state every 500ms
    TimerHandle_t led_state_timer = xTimerCreate("led_state_timer", pdMS_TO_TICKS(500), pdTRUE, (void *)0, led_state_timer_callback);

    if (led_state_timer != NULL) {
        xTimerStart(led_state_timer, 0);
        ESP_LOGI(TAG, "LED state timer started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create LED state timer");
        return -1;
    }

    // Create another timer to control LED blinking state (on/off)
    TimerHandle_t led_blink_timer = xTimerCreate("led_blink_timer", pdMS_TO_TICKS(500), pdTRUE, (void *)0, led_blink_timer_callback);

    if (led_blink_timer != NULL) {
        xTimerStart(led_blink_timer, 0);
        ESP_LOGI(TAG, "LED blink timer started successfully");
        return 0;
    } else {
        ESP_LOGE(TAG, "Failed to create LED blink timer");
        return -1;
    }
}
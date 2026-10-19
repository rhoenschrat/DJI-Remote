/*
 * Board HAL selection (ADR-004) — the one place that maps the generic HAL_*
 * macros to the active board, driven by Kconfig (main/Kconfig.projbuild).
 *
 * The button macros return the raw pin level (true = pressed). Debounce,
 * timing and click/long-press detection live in button_sm.c (ADR-024).
 */

#ifndef BOARD_HAL_H
#define BOARD_HAL_H

#include "sdkconfig.h"

#if defined(CONFIG_BOARD_M5STACK_BASIC_V27)
  #include "m5stack_basic_v27_hal.h"
  #define HAL_INIT()                m5stack_basic_v27_init()
  #define HAL_LVGL_INIT()           m5stack_basic_v27_lvgl_init()
  #define HAL_BACKLIGHT_ON()        m5stack_basic_v27_backlight_on()
  #define HAL_BTN_A_PRESSED()       m5stack_basic_v27_button_a_pressed()
  #define HAL_BTN_B_PRESSED()       m5stack_basic_v27_button_b_pressed()
  #define HAL_BTN_C_PRESSED()       m5stack_basic_v27_button_c_pressed()
#elif defined(CONFIG_BOARD_WAVESHARE_S3_LCD19)
  #include "waveshare_s3_lcd19_hal.h"
  #define HAL_INIT()                waveshare_s3_lcd19_init()
  #define HAL_LVGL_INIT()           waveshare_s3_lcd19_lvgl_init()
  #define HAL_BACKLIGHT_ON()        waveshare_s3_lcd19_backlight_on()
  #define HAL_BTN_A_PRESSED()       waveshare_s3_lcd19_button_a_pressed()
  #define HAL_BTN_B_PRESSED()       waveshare_s3_lcd19_button_b_pressed()
  #define HAL_BTN_C_PRESSED()       waveshare_s3_lcd19_button_c_pressed()
#else
  #error "No board target defined. Set CONFIG_BOARD_M5STACK_BASIC_V27 or CONFIG_BOARD_WAVESHARE_S3_LCD19."
#endif

#endif /* BOARD_HAL_H */

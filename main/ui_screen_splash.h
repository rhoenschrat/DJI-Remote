#pragma once
#include "lvgl.h"

/**
 * @brief Create the boot splash screen
 *
 * Black background with the project logo centered and recolored white.
 * Must be called while holding the LVGL port lock.
 *
 * @return The splash screen object
 */
lv_obj_t *ui_screen_splash_create(void);

/**
 * @brief Destroy the splash screen and free resources
 *
 * Must be called while holding the LVGL port lock.
 */
void ui_screen_splash_destroy(void);

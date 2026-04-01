/*
 * DJI Camera Remote Control - Boot Splash Screen
 *
 * Displays the project logo centered on a black background using LVGL.
 * Shown immediately after LVGL init, before the backlight is enabled,
 * ensuring a clean first frame with no flicker (see ADR-010).
 */

#include "ui_screen_splash.h"
#include "splash_logo.h"

static lv_obj_t *s_screen = NULL;

lv_obj_t *ui_screen_splash_create(void)
{
    s_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);

    lv_obj_t *img = lv_image_create(s_screen);
    lv_image_set_src(img, &splash_logo);

    /* Recolor the A8 alpha map to white */
    lv_obj_set_style_image_recolor(img, lv_color_white(), 0);
    lv_obj_set_style_image_recolor_opa(img, LV_OPA_COVER, 0);

    lv_obj_center(img);

    return s_screen;
}

void ui_screen_splash_destroy(void)
{
    if (s_screen != NULL) {
        lv_obj_del(s_screen);
        s_screen = NULL;
    }
}

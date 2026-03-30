/**
 * Centralized layout system for multi-resolution support.
 *
 * All screen modules read positions and sizes from a ui_layout_t struct
 * initialized once at startup from the LVGL display resolution.
 * Presets: 320x240 (M5Stack Basic v2.7), 240x135 (M5StickC Plus2).
 *
 * Also provides shared LVGL colour and style helpers so that screen
 * modules do not duplicate boilerplate inline functions.
 */

#ifndef UI_LAYOUT_H
#define UI_LAYOUT_H

#include "lvgl.h"
#include <stdbool.h>

/* ======================================================================
 * Layout Struct
 * ====================================================================== */

typedef struct {
    int32_t screen_w;
    int32_t screen_h;

    const lv_font_t *font_small;
    const lv_font_t *font_body;
    const lv_font_t *font_heading;
    int32_t icon_size;

    /* Main screen: camera blocks */
    struct {
        int32_t count;      /* visible blocks (3 on 320x240, 1 on 240x135) */
        int32_t x_start;   /* x of first block */
        int32_t y;
        int32_t w;
        int32_t h;
        int32_t spacing;   /* x distance between block origins */
    } cam;

    /* Camera block internal offsets (relative to block container) */
    struct {
        int32_t ind_x, ind_y, ind_w, ind_h;
        int32_t title_x, title_y, title_w;
        int32_t mode_icon_x, mode_icon_y;
        int32_t vmode_x, vmode_y, vmode_w;
        int32_t vmode_line_space;
        int32_t status_icon_x, status_icon_y;
        int32_t time_x, time_y, time_w;
        int32_t sd_icon_x, sd_icon_y, sd_text_x, sd_text_y;
        int32_t bat_icon_x, bat_icon_y, bat_text_x, bat_text_y;
    } cam_detail;

    /* Status bar (notification + GPS) */
    struct {
        int32_t notif_x, notif_y;
        int32_t notif_w;
        int32_t gps_icon_x, gps_icon_y;
        int32_t gps_text_x, gps_text_y;
    } status;

    /* Button bar (shared across screens) */
    struct {
        int32_t y;
        int32_t w, h;
        int32_t x[3];
        int32_t label_ofs_x;
        int32_t label_ofs_y;
    } btn;

    /* Submenu screens (pairing, settings) */
    struct {
        int32_t base_x, base_y;
        int32_t w;
        int32_t title_ofs_y;
        int32_t entry_h;
        int32_t entry_first_y;
        int32_t entry_spacing;
        int32_t indicator_w;
        int32_t icon_ofs_x;
        int32_t label_ofs_x;
        int32_t label_ofs_y;
        int32_t max_entries;
        int32_t notif_x, notif_y;
        int32_t notif_w;
    } submenu;

    /* Mode switch screen */
    struct {
        int32_t icon_ofs_y;
        int32_t label_ofs_y;
    } mode;

} ui_layout_t;

/* Initialize layout from display resolution. Call once after lvgl_init(). */
void ui_layout_init(lv_display_t *disp);

/* Get current layout (valid after ui_layout_init). */
const ui_layout_t *ui_layout_get(void);

/* ======================================================================
 * Shared LVGL Colour Helpers
 * ====================================================================== */

static inline lv_color_t ui_clr_white(void)  { return lv_color_white(); }
static inline lv_color_t ui_clr_black(void)  { return lv_color_black(); }
static inline lv_color_t ui_clr_red(void)    { return lv_color_make(255,   0,   0); }
static inline lv_color_t ui_clr_green(void)  { return lv_color_make(  0, 255,   0); }
static inline lv_color_t ui_clr_blue(void)   { return lv_color_make(  0,   0, 255); }
static inline lv_color_t ui_clr_yellow(void) { return lv_color_make(255, 255,   0); }
static inline lv_color_t ui_clr_orange(void) { return lv_color_make(255, 165,   0); }
static inline lv_color_t ui_clr_cyan(void)   { return lv_color_make(  0, 255, 255); }

/* ======================================================================
 * Shared LVGL Widget Helpers
 * ====================================================================== */

static inline void ui_style_img_recolor(lv_obj_t *img, lv_color_t color) {
    lv_obj_set_style_image_recolor(img, color, 0);
    lv_obj_set_style_image_recolor_opa(img, LV_OPA_COVER, 0);
}

static inline void ui_set_visible(lv_obj_t *obj, bool visible) {
    if (visible)
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

/* Compute the x position for a camera block on the main screen. */
static inline int32_t ui_cam_block_x(const ui_layout_t *L, int idx) {
    if (L->cam.count >= 3) {
        return L->cam.x_start + idx * L->cam.spacing;
    }
    return L->cam.x_start;
}

/* Compute the y position for a submenu entry (absolute). */
static inline int32_t ui_submenu_entry_y(const ui_layout_t *L, int idx) {
    return L->submenu.entry_first_y + idx * L->submenu.entry_spacing;
}

#endif /* UI_LAYOUT_H */

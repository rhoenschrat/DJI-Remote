/**
 * Layout preset initialization for multi-resolution support.
 *
 * Three presets are provided:
 *   - 320x240 (M5Stack Basic v2.7)        — 3-column main screen, 5-entry submenus
 *   - 320x170 (Waveshare ESP32-S3-LCD-1.9) — 3-column, compact buttons, no vmode
 *   - 240x135 (M5StickC Plus2)             — 1-column main screen, 3-entry submenus
 *
 * The 240x135 preset is designed but not yet hardware-tested.
 */

#include "ui_layout.h"
#include "esp_log.h"

#define TAG "UI_LAYOUT"

static ui_layout_t s_layout;
static bool s_initialized = false;

/* ---------------------------------------------------------------------- */
/* 320 x 240   (M5Stack Basic v2.7)                                       */
/* ---------------------------------------------------------------------- */
static void init_layout_320x240(ui_layout_t *L) {
    L->screen_w = 320;
    L->screen_h = 240;

    L->font_small   = &lv_font_montserrat_12;
    L->font_body    = &lv_font_montserrat_14;
    L->font_heading = &lv_font_montserrat_16;
    L->icon_size    = 24;

    /* Camera blocks (3 columns, 100px each, contiguous) */
    L->cam.count   = 3;
    L->cam.x_start = 10;
    L->cam.y       = 10;
    L->cam.w       = 100;
    L->cam.h       = 178;
    L->cam.spacing = 100;

    /* Camera block internal offsets */
    L->cam_detail.ind_x  = 1;   L->cam_detail.ind_y  = 1;
    L->cam_detail.ind_w  = 97;  L->cam_detail.ind_h  = 4;

    L->cam_detail.title_x = 5;  L->cam_detail.title_y = 9;
    L->cam_detail.title_w = 90;

    L->cam_detail.mode_icon_x = 38;  L->cam_detail.mode_icon_y = 29;

    L->cam_detail.vmode_x = 5;  L->cam_detail.vmode_y = 53;
    L->cam_detail.vmode_w = 90;
    L->cam_detail.vmode_line_space = -4;

    L->cam_detail.status_icon_x = 38;  L->cam_detail.status_icon_y = 77;

    L->cam_detail.time_x = 5;  L->cam_detail.time_y = 106;
    L->cam_detail.time_w = 90;

    L->cam_detail.sd_icon_x = 5;   L->cam_detail.sd_icon_y = 125;
    L->cam_detail.sd_text_x = 34;  L->cam_detail.sd_text_y = 130;

    L->cam_detail.bat_icon_x = 5;   L->cam_detail.bat_icon_y = 149;
    L->cam_detail.bat_text_x = 34;  L->cam_detail.bat_text_y = 154;

    /* Status bar */
    L->status.notif_x    = 15;   L->status.notif_y    = 197;
    L->status.notif_w    = 140;
    L->status.gps_icon_x = 160;  L->status.gps_icon_y = 192;
    L->status.gps_text_x = 194;  L->status.gps_text_y = 196;
    L->status.gps_fix_color   = lv_color_make(255, 255, 0);  /* Yellow */
    L->status.gps_text_visible = true;

    /* Button bar */
    L->btn.y          = 216;
    L->btn.w          = 93;
    L->btn.h          = 24;
    L->btn.x[0]       = 10;
    L->btn.x[1]       = 113;
    L->btn.x[2]       = 217;
    L->btn.label_ofs_x = 29;
    L->btn.label_ofs_y = 5;

    /* Button style — standard wide buttons with labels */
    L->btn_style.compact    = false;
    L->btn_style.bg_color[0] = lv_color_black();
    L->btn_style.bg_color[1] = lv_color_black();
    L->btn_style.bg_color[2] = lv_color_black();
    L->btn_style.icon_ofs_x = 0;
    L->btn_style.icon_ofs_y = 0;
    L->btn_style.icon_size  = 24;

    /* Submenu layout */
    L->submenu.base_x        = 10;
    L->submenu.base_y        = 10;
    L->submenu.w             = 300;
    L->submenu.title_ofs_y   = 4;
    L->submenu.entry_h       = 24;
    L->submenu.entry_first_y = 39;
    L->submenu.entry_spacing = 29;
    L->submenu.indicator_w   = 10;
    L->submenu.icon_ofs_x    = 15;
    L->submenu.icon_ofs_y    = 0;
    L->submenu.label_ofs_x   = 44;
    L->submenu.label_ofs_y   = 4;
    L->submenu.max_entries   = 5;
    L->submenu.notif_x       = 15;
    L->submenu.notif_y       = 189;
    L->submenu.notif_w       = 290;

    /* Submenu buttons — standard bottom bar */
    L->submenu_btns.column      = false;
    L->submenu_btns.col_x       = 0;
    L->submenu_btns.col_btn_y[0] = 0;  L->submenu_btns.col_btn_h[0] = 0;
    L->submenu_btns.col_btn_y[1] = 0;  L->submenu_btns.col_btn_h[1] = 0;
    L->submenu_btns.col_btn_y[2] = 0;  L->submenu_btns.col_btn_h[2] = 0;

    /* Mode switch */
    L->mode.icon_ofs_y  = 29;
    L->mode.label_ofs_y = 68;
}

/* ---------------------------------------------------------------------- */
/* 240 x 135   (M5StickC Plus2)  —  untested, needs hardware validation   */
/* ---------------------------------------------------------------------- */
static void init_layout_240x135(ui_layout_t *L) {
    L->screen_w = 240;
    L->screen_h = 135;

    L->font_small   = &lv_font_montserrat_12;
    L->font_body    = &lv_font_montserrat_14;
    L->font_heading = &lv_font_montserrat_14;
    L->icon_size    = 24;

    /* Single camera block visible at a time; user cycles with Button B */
    L->cam.count   = 1;
    L->cam.x_start = 10;
    L->cam.y       = 5;
    L->cam.w       = 220;
    L->cam.h       = 80;
    L->cam.spacing = 220;

    /* Camera block internals — two-column compact layout */
    L->cam_detail.ind_x  = 1;    L->cam_detail.ind_y  = 1;
    L->cam_detail.ind_w  = 218;  L->cam_detail.ind_h  = 3;

    L->cam_detail.title_x = 5;  L->cam_detail.title_y = 5;
    L->cam_detail.title_w = 80;

    L->cam_detail.mode_icon_x = 5;   L->cam_detail.mode_icon_y = 24;

    L->cam_detail.vmode_x = 34;  L->cam_detail.vmode_y = 24;
    L->cam_detail.vmode_w = 76;
    L->cam_detail.vmode_line_space = -4;

    L->cam_detail.status_icon_x = 120;  L->cam_detail.status_icon_y = 5;

    L->cam_detail.time_x = 120;  L->cam_detail.time_y = 30;
    L->cam_detail.time_w = 90;

    L->cam_detail.sd_icon_x = 5;   L->cam_detail.sd_icon_y = 52;
    L->cam_detail.sd_text_x = 34;  L->cam_detail.sd_text_y = 55;

    L->cam_detail.bat_icon_x = 120;  L->cam_detail.bat_icon_y = 52;
    L->cam_detail.bat_text_x = 149;  L->cam_detail.bat_text_y = 55;

    /* Status bar */
    L->status.notif_x    = 10;   L->status.notif_y    = 90;
    L->status.notif_w    = 110;
    L->status.gps_icon_x = 120;  L->status.gps_icon_y = 88;
    L->status.gps_text_x = 148;  L->status.gps_text_y = 90;
    L->status.gps_fix_color   = lv_color_make(255, 255, 0);  /* Yellow */
    L->status.gps_text_visible = true;

    /* Button bar */
    L->btn.y          = 111;
    L->btn.w          = 73;
    L->btn.h          = 24;
    L->btn.x[0]       = 5;
    L->btn.x[1]       = 83;
    L->btn.x[2]       = 162;
    L->btn.label_ofs_x = 26;
    L->btn.label_ofs_y = 5;

    /* Button style — standard */
    L->btn_style.compact    = false;
    L->btn_style.bg_color[0] = lv_color_black();
    L->btn_style.bg_color[1] = lv_color_black();
    L->btn_style.bg_color[2] = lv_color_black();
    L->btn_style.icon_ofs_x = 0;
    L->btn_style.icon_ofs_y = 0;
    L->btn_style.icon_size  = 24;

    /* Submenu layout */
    L->submenu.base_x        = 5;
    L->submenu.base_y        = 5;
    L->submenu.w             = 230;
    L->submenu.title_ofs_y   = 2;
    L->submenu.entry_h       = 22;
    L->submenu.entry_first_y = 28;
    L->submenu.entry_spacing = 24;
    L->submenu.indicator_w   = 8;
    L->submenu.icon_ofs_x    = 12;
    L->submenu.icon_ofs_y    = 0;
    L->submenu.label_ofs_x   = 38;
    L->submenu.label_ofs_y   = 3;
    L->submenu.max_entries   = 3;
    L->submenu.notif_x       = 10;
    L->submenu.notif_y       = 82;
    L->submenu.notif_w       = 220;

    /* Submenu buttons — standard bottom bar */
    L->submenu_btns.column      = false;
    L->submenu_btns.col_x       = 0;
    L->submenu_btns.col_btn_y[0] = 0;  L->submenu_btns.col_btn_h[0] = 0;
    L->submenu_btns.col_btn_y[1] = 0;  L->submenu_btns.col_btn_h[1] = 0;
    L->submenu_btns.col_btn_y[2] = 0;  L->submenu_btns.col_btn_h[2] = 0;

    /* Mode switch */
    L->mode.icon_ofs_y  = 20;
    L->mode.label_ofs_y = 48;
}

/* ---------------------------------------------------------------------- */
/* 320 x 170   (Waveshare ESP32-S3-LCD-1.9)                               */
/* ---------------------------------------------------------------------- */
static void init_layout_320x170(ui_layout_t *L) {
    L->screen_w = 320;
    L->screen_h = 170;

    L->font_small   = &lv_font_montserrat_12;
    L->font_body    = &lv_font_montserrat_14;
    L->font_heading = &lv_font_montserrat_16;
    L->icon_size    = 24;

    /* Camera blocks (3 columns) */
    L->cam.count   = 3;
    L->cam.x_start = 5;
    L->cam.y       = 5;
    L->cam.w       = 100;
    L->cam.h       = 134;
    L->cam.spacing = 105;

    /* Camera block internal offsets — no mode icon or vmode rows */
    L->cam_detail.ind_x  = 1;   L->cam_detail.ind_y  = 1;
    L->cam_detail.ind_w  = 97;  L->cam_detail.ind_h  = 4;

    L->cam_detail.title_x = 5;  L->cam_detail.title_y = 9;
    L->cam_detail.title_w = 90;

    L->cam_detail.mode_icon_x = 0;  L->cam_detail.mode_icon_y = 0;
    L->cam_detail.vmode_x = 0;  L->cam_detail.vmode_y = 0;
    L->cam_detail.vmode_w = 0;  /* sentinel: 0 = omit mode/vmode widgets */
    L->cam_detail.vmode_line_space = 0;

    L->cam_detail.status_icon_x = 38;  L->cam_detail.status_icon_y = 29;

    L->cam_detail.time_x = 5;  L->cam_detail.time_y = 58;
    L->cam_detail.time_w = 90;

    L->cam_detail.sd_icon_x = 5;   L->cam_detail.sd_icon_y = 80;
    L->cam_detail.sd_text_x = 34;  L->cam_detail.sd_text_y = 85;

    L->cam_detail.bat_icon_x = 5;   L->cam_detail.bat_icon_y = 108;
    L->cam_detail.bat_text_x = 34;  L->cam_detail.bat_text_y = 113;

    /* Status bar — merged bottom row */
    L->status.notif_x    = 99;   L->status.notif_y    = 143;
    L->status.notif_w    = 189;
    L->status.gps_icon_x = 291;  L->status.gps_icon_y = 145;
    L->status.gps_text_x = 0;    L->status.gps_text_y = 0;
    L->status.gps_fix_color   = lv_color_make(0, 200, 0);  /* Green */
    L->status.gps_text_visible = false;

    /* Button bar — compact 27x27 colored squares */
    L->btn.y          = 143;
    L->btn.w          = 27;
    L->btn.h          = 27;
    L->btn.x[0]       = 5;
    L->btn.x[1]       = 36;
    L->btn.x[2]       = 67;
    L->btn.label_ofs_x = 0;
    L->btn.label_ofs_y = 0;

    /* Button style — compact colored squares */
    L->btn_style.compact    = true;
    L->btn_style.bg_color[0] = lv_color_make(255, 0, 0);
    L->btn_style.bg_color[1] = lv_color_make(255, 255, 0);
    L->btn_style.bg_color[2] = lv_color_make(0, 0, 255);
    L->btn_style.icon_ofs_x = 2;
    L->btn_style.icon_ofs_y = 2;
    L->btn_style.icon_size  = 20;

    /* Submenu layout — no title, full-height entries */
    L->submenu.base_x        = 0;
    L->submenu.base_y        = 0;
    L->submenu.w             = 288;
    L->submenu.title_ofs_y   = 0;
    L->submenu.entry_h       = 34;
    L->submenu.entry_first_y = 0;
    L->submenu.entry_spacing = 34;
    L->submenu.indicator_w   = 4;
    L->submenu.icon_ofs_x    = 10;
    L->submenu.icon_ofs_y    = 5;
    L->submenu.label_ofs_x   = 38;
    L->submenu.label_ofs_y   = 9;
    L->submenu.max_entries   = 5;
    L->submenu.notif_x       = 0;
    L->submenu.notif_y       = 0;
    L->submenu.notif_w       = 0;  /* no separate notification area */

    /* Submenu buttons — vertical right-edge column */
    L->submenu_btns.column      = true;
    L->submenu_btns.col_x       = 293;
    /* Three 27x27 squares grouped tightly at top, 4px gap between each */
    L->submenu_btns.col_btn_y[0] = 3;    L->submenu_btns.col_btn_h[0] = 27;
    L->submenu_btns.col_btn_y[1] = 34;   L->submenu_btns.col_btn_h[1] = 27;
    L->submenu_btns.col_btn_y[2] = 65;   L->submenu_btns.col_btn_h[2] = 27;

    /* Mode switch */
    L->mode.icon_ofs_y  = 40;
    L->mode.label_ofs_y = 85;
}

/* ---------------------------------------------------------------------- */
/* Public API                                                              */
/* ---------------------------------------------------------------------- */

void ui_layout_init(lv_display_t *disp) {
    int32_t w = lv_display_get_horizontal_resolution(disp);
    int32_t h = lv_display_get_vertical_resolution(disp);

    if (w >= 320 && h >= 240) {
        init_layout_320x240(&s_layout);
    } else if (w >= 320 && h >= 170) {
        init_layout_320x170(&s_layout);
    } else {
        init_layout_240x135(&s_layout);
    }

    s_layout.screen_w = w;
    s_layout.screen_h = h;
    s_initialized = true;

    ESP_LOGI(TAG, "Layout initialized for %ldx%ld (%d camera columns)",
             (long)w, (long)h, (int)s_layout.cam.count);
}

const ui_layout_t *ui_layout_get(void) {
    if (!s_initialized) {
        ESP_LOGW(TAG, "Layout not initialized, falling back to 320x240");
        init_layout_320x240(&s_layout);
        s_initialized = true;
    }
    return &s_layout;
}

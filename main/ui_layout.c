/**
 * Layout preset initialization for multi-resolution support.
 *
 * Two presets are provided:
 *   - 320x240 (M5Stack Basic v2.7)   — 3-column main screen, 5-entry submenus
 *   - 240x135 (M5StickC Plus2)       — 1-column main screen, 3-entry submenus
 *
 * The 240x135 preset is designed but not yet hardware-tested.  It should be
 * treated as a starting point and adjusted after flashing on real hardware.
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
    L->cam_detail.ind_w  = 98;  L->cam_detail.ind_h  = 4;

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

    /* Button bar */
    L->btn.y          = 216;
    L->btn.w          = 93;
    L->btn.h          = 24;
    L->btn.x[0]       = 10;
    L->btn.x[1]       = 113;
    L->btn.x[2]       = 217;
    L->btn.label_ofs_x = 29;
    L->btn.label_ofs_y = 5;

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
    L->submenu.label_ofs_x   = 44;
    L->submenu.label_ofs_y   = 4;
    L->submenu.max_entries   = 5;
    L->submenu.notif_x       = 15;
    L->submenu.notif_y       = 189;
    L->submenu.notif_w       = 290;

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

    /* Button bar */
    L->btn.y          = 111;
    L->btn.w          = 73;
    L->btn.h          = 24;
    L->btn.x[0]       = 5;
    L->btn.x[1]       = 83;
    L->btn.x[2]       = 162;
    L->btn.label_ofs_x = 26;
    L->btn.label_ofs_y = 5;

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
    L->submenu.label_ofs_x   = 38;
    L->submenu.label_ofs_y   = 3;
    L->submenu.max_entries   = 3;
    L->submenu.notif_x       = 10;
    L->submenu.notif_y       = 82;
    L->submenu.notif_w       = 220;

    /* Mode switch */
    L->mode.icon_ofs_y  = 20;
    L->mode.label_ofs_y = 48;
}

/* ---------------------------------------------------------------------- */
/* Public API                                                              */
/* ---------------------------------------------------------------------- */

void ui_layout_init(lv_display_t *disp) {
    int32_t w = lv_display_get_horizontal_resolution(disp);
    int32_t h = lv_display_get_vertical_resolution(disp);

    if (w >= 320 && h >= 240) {
        init_layout_320x240(&s_layout);
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

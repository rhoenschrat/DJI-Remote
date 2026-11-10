/**
 * Main Screen module (LVGL).
 *
 * Builds and updates the LVGL widget tree for the multi-camera main screen.
 * Layout positions come from ui_layout_get() for multi-resolution support.
 */

#include "ui_screen_main.h"
#include "ui.h"
#include "ui_layout.h"
#include "lvgl_icons.h"
#include "slot_busy.h"
#include "slot_cycle.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "enums_logic.h"
#include "../gps/gps_reader.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdio.h>

#define TAG "UI_SCR_MAIN"

/* SD card / battery warning style (plan 0019, Kconfig UI_STATUS_WARN_STYLE).
 * A C constant instead of #if around functions: both variants compile in
 * every build, so the unused one cannot rot. */
#ifdef CONFIG_UI_STATUS_WARN_BACKGROUND
#define UI_WARN_BACKGROUND 1
#else
#define UI_WARN_BACKGROUND 0
#endif

/* ======================================================================
 * Widget Structures
 * ====================================================================== */

typedef struct {
    lv_obj_t *block;
    lv_obj_t *indicator;
    lv_obj_t *title;
    lv_obj_t *mode_icon;
    lv_obj_t *vmode_label;
    lv_obj_t *status_icon;
    lv_obj_t *time_label;
    lv_obj_t *sd_bg;        /* row warning background, below icon and text */
    lv_obj_t *sd_icon;
    lv_obj_t *sd_text;
    lv_obj_t *bat_bg;
    lv_obj_t *bat_icon;
    lv_obj_t *bat_text;
    lv_obj_t *busy_icon;    /* NULL where the layout has no busy indicator */
} cam_block_t;

typedef struct {
    lv_obj_t *container;
    lv_obj_t *icon;
    lv_obj_t *label;
} btn_widget_t;

/* ======================================================================
 * Static State
 * ====================================================================== */

static lv_obj_t    *s_screen       = NULL;
static cam_block_t  s_cam[NUM_CAMERAS];
static lv_obj_t    *s_notification = NULL;
/* One-shot timer that clears a posted (non-blocking) notification */
static lv_timer_t  *s_notif_timer = NULL;
static lv_obj_t    *s_gps_icon     = NULL;
static lv_obj_t    *s_gps_text     = NULL;
static btn_widget_t s_btn[3];
/* Syncs the busy indicators with slot_busy while the main task is blocked
 * (research §4.1); NULL where the layout has no busy indicator */
static lv_timer_t  *s_busy_timer   = NULL;

#define BUSY_TIMER_PERIOD_MS 100

/* ======================================================================
 * Icon / Colour Selection Helpers
 * ====================================================================== */

static const lv_image_dsc_t *get_battery_lvgl_icon(uint8_t pct) {
    if (pct > 75) return &lvgl_battery_100_icon;
    if (pct > 50) return &lvgl_battery_75_icon;
    if (pct > 25) return &lvgl_battery_50_icon;
    if (pct > 10) return &lvgl_battery_25_icon;
    return &lvgl_battery_0_icon;
}

/* Warning level of a status row; shared by both warning styles so the
 * thresholds are identical */
typedef enum {
    ROW_LEVEL_NORMAL = 0,
    ROW_LEVEL_WARN,         /* yellow */
    ROW_LEVEL_CRIT,         /* red */
} row_level_t;

/* Look of one status row for a level in the compiled warning style */
typedef struct {
    bool       bg_visible;
    lv_color_t bg;
    lv_color_t fg;          /* icon and text */
} row_style_t;

static row_level_t bat_level(uint8_t pct) {
    if (pct > 25) return ROW_LEVEL_NORMAL;
    if (pct > 10) return ROW_LEVEL_WARN;
    return ROW_LEVEL_CRIT;
}

static row_level_t sd_level(uint32_t capacity_mb) {
    uint32_t gb_x10 = capacity_mb * 10 / 1024;
    if (gb_x10 > 160) return ROW_LEVEL_NORMAL;
    if (gb_x10 >  60) return ROW_LEVEL_WARN;
    return ROW_LEVEL_CRIT;
}

/* Background style: yellow row + black fg, red row + white fg.
 * Tint style (legacy): no background, fg yellow / red. */
static row_style_t row_style(row_level_t level) {
    row_style_t r = { .bg_visible = false, .bg = ui_clr_black(), .fg = ui_clr_white() };
    if (level == ROW_LEVEL_NORMAL) {
        return r;
    }
    if (UI_WARN_BACKGROUND) {
        r.bg_visible = true;
        r.bg = (level == ROW_LEVEL_WARN) ? ui_clr_yellow() : ui_clr_red();
        r.fg = (level == ROW_LEVEL_WARN) ? ui_clr_black()  : ui_clr_white();
    } else {
        r.fg = (level == ROW_LEVEL_WARN) ? ui_clr_yellow() : ui_clr_red();
    }
    return r;
}

static void apply_row_style(lv_obj_t *bg, lv_obj_t *icon, lv_obj_t *text,
                            row_level_t level) {
    row_style_t r = row_style(level);
    if (r.bg_visible) {
        lv_obj_set_style_bg_color(bg, r.bg, 0);
    }
    ui_set_visible(bg, r.bg_visible);
    ui_style_img_recolor(icon, r.fg);
    lv_obj_set_style_text_color(text, r.fg, 0);
}

/* Plain rectangle below a status row; hidden until a warning level needs it */
static lv_obj_t *create_row_bg(lv_obj_t *parent, int32_t x, int32_t y,
                               int32_t w, int32_t h) {
    lv_obj_t *bg = lv_obj_create(parent);
    lv_obj_remove_style_all(bg);
    lv_obj_set_pos(bg, x, y);
    lv_obj_set_size(bg, w, h);
    lv_obj_set_style_bg_color(bg, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(bg, LV_OBJ_FLAG_HIDDEN);
    return bg;
}

static const lv_image_dsc_t *get_mode_icon(uint8_t camera_mode) {
    return (camera_mode == 0x05) ? &lvgl_photo_icon : &lvgl_video_icon;
}

static void get_status_icon_and_color(int idx, const camera_state_t *st,
                                      const lv_image_dsc_t **icon_out,
                                      lv_color_t *color_out) {
    bool boot_active = connect_logic_is_boot_scan_active()
                     || connect_logic_is_boot_connect_in_progress();
    bool found = connect_logic_is_slot_found(idx) && boot_active;

    /* Checked first: link up but no status push (ADR-019) */
    if (st->connection_state == CAM_STATE_UNRESPONSIVE) {
        *icon_out  = &lvgl_alert_icon;
        *color_out = ui_clr_yellow();
        return;
    }

    if (st->connection_state == CAM_STATE_CONNECTED) {
        if (st->is_sleeping) {
            *icon_out  = &lvgl_sleep_icon;
            *color_out = ui_clr_white();
        } else if (st->is_recording) {
            *icon_out  = &lvgl_record_icon;
            *color_out = ui_clr_red();
        } else {
            *icon_out  = &lvgl_pause_icon;
            *color_out = ui_clr_white();
        }
        return;
    }

    if (st->connection_state == CAM_STATE_UNPAIRED) {
        *icon_out  = &lvgl_bluetooth_icon;
        *color_out = ui_clr_white();
        return;
    }

    if (connect_logic_slot_is_connecting(idx)) {
        *icon_out  = &lvgl_connecting_icon;
        *color_out = ui_clr_blue();
        return;
    }

    if (found) {
        *icon_out  = &lvgl_found_icon;
        *color_out = ui_clr_green();
        return;
    }

    *icon_out  = &lvgl_bluetooth_icon;
    *color_out = ui_clr_blue();
}

/* ======================================================================
 * Resolution / FPS Helpers (mirrors logic in ui.c)
 * ====================================================================== */

static const char *short_resolution(uint8_t res) {
    switch (res) {
        case 10:  return "1080P";
        case 16:  return "4K 16:9";
        case 45:  return "2.7K 16:9";
        case 66:  return "1080P 9:16";
        case 67:  return "2.7K 9:16";
        case 95:  return "2.7K 4:3";
        case 103: return "4K 4:3";
        case 109: return "4K 9:16";
        default:  return "-";
    }
}

/* ======================================================================
 * Camera Block Creation
 * ====================================================================== */

static void create_camera_block(int idx) {
    const ui_layout_t *L = ui_layout_get();
    int base_x = ui_cam_block_x(L, idx);
    cam_block_t *cb = &s_cam[idx];

    cb->block = lv_obj_create(s_screen);
    lv_obj_remove_style_all(cb->block);
    lv_obj_set_pos(cb->block, base_x, L->cam.y);
    lv_obj_set_size(cb->block, L->cam.w, L->cam.h);
    lv_obj_set_style_bg_color(cb->block, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(cb->block, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(cb->block, ui_clr_white(), 0);
    lv_obj_set_style_border_width(cb->block, 1, 0);
    lv_obj_set_style_pad_all(cb->block, 0, 0);
    lv_obj_set_scrollbar_mode(cb->block, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(cb->block, LV_OBJ_FLAG_SCROLLABLE);

    /* Selection indicator bar */
    cb->indicator = lv_obj_create(cb->block);
    lv_obj_remove_style_all(cb->indicator);
    lv_obj_set_pos(cb->indicator, L->cam_detail.ind_x, L->cam_detail.ind_y);
    lv_obj_set_size(cb->indicator, L->cam_detail.ind_w, L->cam_detail.ind_h);
    lv_obj_set_style_bg_color(cb->indicator, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(cb->indicator, LV_OPA_COVER, 0);
    lv_obj_clear_flag(cb->indicator, LV_OBJ_FLAG_SCROLLABLE);

    /* Title label */
    cb->title = lv_label_create(cb->block);
    lv_obj_set_pos(cb->title, L->cam_detail.title_x, L->cam_detail.title_y);
    lv_obj_set_width(cb->title, L->cam_detail.title_w);
    lv_obj_set_style_text_align(cb->title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(cb->title, ui_clr_white(), 0);
    lv_obj_set_style_text_font(cb->title, L->font_heading, 0);
    lv_label_set_text(cb->title, "");

    /* Camera mode icon — omitted on 320x170 (vmode_w == 0) */
    if (L->cam_detail.vmode_w > 0) {
        cb->mode_icon = lv_image_create(cb->block);
        lv_obj_set_pos(cb->mode_icon, L->cam_detail.mode_icon_x, L->cam_detail.mode_icon_y);
        lv_image_set_src(cb->mode_icon, &lvgl_video_icon);
        ui_style_img_recolor(cb->mode_icon, ui_clr_white());

        /* Video mode text */
        cb->vmode_label = lv_label_create(cb->block);
        lv_obj_set_pos(cb->vmode_label, L->cam_detail.vmode_x, L->cam_detail.vmode_y);
        lv_obj_set_width(cb->vmode_label, L->cam_detail.vmode_w);
        lv_obj_set_style_text_align(cb->vmode_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(cb->vmode_label, ui_clr_white(), 0);
        lv_obj_set_style_text_font(cb->vmode_label, L->font_small, 0);
        lv_obj_set_style_text_line_space(cb->vmode_label, L->cam_detail.vmode_line_space, 0);
        lv_label_set_text(cb->vmode_label, "");
    } else {
        cb->mode_icon  = NULL;
        cb->vmode_label = NULL;
    }

    /* Status icon */
    cb->status_icon = lv_image_create(cb->block);
    lv_obj_set_pos(cb->status_icon, L->cam_detail.status_icon_x, L->cam_detail.status_icon_y);
    lv_image_set_src(cb->status_icon, &lvgl_bluetooth_icon);
    ui_style_img_recolor(cb->status_icon, ui_clr_white());

    /* Recording time label */
    cb->time_label = lv_label_create(cb->block);
    lv_obj_set_pos(cb->time_label, L->cam_detail.time_x, L->cam_detail.time_y);
    lv_obj_set_width(cb->time_label, L->cam_detail.time_w);
    lv_obj_set_style_text_align(cb->time_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(cb->time_label, ui_clr_white(), 0);
    lv_obj_set_style_text_font(cb->time_label, L->font_body, 0);
    lv_label_set_text(cb->time_label, "");

    /* Row warning backgrounds (plan 0019): created before the SD and
     * battery icons and texts so they are drawn below them */
    cb->sd_bg  = create_row_bg(cb->block, L->row_bg.sd_x, L->row_bg.sd_y,
                               L->row_bg.sd_w, L->row_bg.sd_h);
    cb->bat_bg = create_row_bg(cb->block, L->row_bg.bat_x, L->row_bg.bat_y,
                               L->row_bg.bat_w, L->row_bg.bat_h);

    /* SD card icon */
    cb->sd_icon = lv_image_create(cb->block);
    lv_obj_set_pos(cb->sd_icon, L->cam_detail.sd_icon_x, L->cam_detail.sd_icon_y);
    lv_image_set_src(cb->sd_icon, &lvgl_sd_card_icon);
    ui_style_img_recolor(cb->sd_icon, ui_clr_white());

    /* SD card capacity text */
    cb->sd_text = lv_label_create(cb->block);
    lv_obj_set_pos(cb->sd_text, L->cam_detail.sd_text_x, L->cam_detail.sd_text_y);
    lv_obj_set_style_text_color(cb->sd_text, ui_clr_white(), 0);
    lv_obj_set_style_text_font(cb->sd_text, L->font_body, 0);
    lv_label_set_text(cb->sd_text, "");

    /* Battery icon */
    cb->bat_icon = lv_image_create(cb->block);
    lv_obj_set_pos(cb->bat_icon, L->cam_detail.bat_icon_x, L->cam_detail.bat_icon_y);
    lv_image_set_src(cb->bat_icon, &lvgl_battery_100_icon);
    ui_style_img_recolor(cb->bat_icon, ui_clr_white());

    /* Battery percentage text */
    cb->bat_text = lv_label_create(cb->block);
    lv_obj_set_pos(cb->bat_text, L->cam_detail.bat_text_x, L->cam_detail.bat_text_y);
    lv_obj_set_style_text_color(cb->bat_text, ui_clr_white(), 0);
    lv_obj_set_style_text_font(cb->bat_text, L->font_body, 0);
    lv_label_set_text(cb->bat_text, "");

    /* Busy indicator (plan 0017): created last so it is drawn on top of the
     * timer, SD and battery rows; A1 image, so the background stays
     * transparent */
    if (L->busy.enabled) {
        cb->busy_icon = lv_image_create(cb->block);
        lv_obj_set_pos(cb->busy_icon, L->busy.main_x, L->busy.main_y);
        lv_image_set_src(cb->busy_icon,
                         L->busy.main_size >= 32 ? &lvgl_busy_icon_32 : &lvgl_busy_icon_24);
        ui_style_img_recolor(cb->busy_icon, ui_clr_cyan());
        lv_obj_clear_flag(cb->busy_icon, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(cb->busy_icon, LV_OBJ_FLAG_HIDDEN);
    } else {
        cb->busy_icon = NULL;
    }
}

/* Show or hide the busy indicators from the per-slot busy state. Setting
 * the hidden flag to its current value is a cheap no-op. */
static void sync_busy_icons(void) {
    for (int i = 0; i < NUM_CAMERAS; i++) {
        if (s_cam[i].busy_icon != NULL) {
            ui_set_visible(s_cam[i].busy_icon, slot_busy_is_active(i));
        }
    }
}

/* LVGL task context: no extra lock needed */
static void busy_timer_cb(lv_timer_t *t) {
    (void)t;
    sync_busy_icons();
}

/* ======================================================================
 * Status Bar Creation (Notification + GPS)
 * ====================================================================== */

static void create_status_bar(void) {
    const ui_layout_t *L = ui_layout_get();

    /* Notification widget — omitted on presets where notif_w == 0
     * (e.g. 320x170, where the bottom row has no space for it). */
    if (L->status.notif_w > 0) {
        s_notification = lv_label_create(s_screen);
        lv_obj_set_pos(s_notification, L->status.notif_x, L->status.notif_y);
        lv_obj_set_width(s_notification, L->status.notif_w);
        lv_obj_set_style_text_color(s_notification, ui_clr_white(), 0);
        lv_obj_set_style_text_font(s_notification, L->font_body, 0);
        lv_label_set_long_mode(s_notification, LV_LABEL_LONG_CLIP);
        lv_label_set_text(s_notification, "");
    } else {
        s_notification = NULL;
    }

    s_gps_icon = lv_image_create(s_screen);
    lv_obj_set_pos(s_gps_icon, L->status.gps_icon_x, L->status.gps_icon_y);
    lv_image_set_src(s_gps_icon, &lvgl_gps_icon);
    ui_style_img_recolor(s_gps_icon, ui_clr_red());

    if (L->status.gps_text_visible) {
        s_gps_text = lv_label_create(s_screen);
        lv_obj_set_pos(s_gps_text, L->status.gps_text_x, L->status.gps_text_y);
        lv_obj_set_style_text_color(s_gps_text, ui_clr_yellow(), 0);
        lv_obj_set_style_text_font(s_gps_text, L->font_body, 0);
        lv_label_set_text(s_gps_text, "");
    } else {
        s_gps_text = NULL;
    }
}

/* ======================================================================
 * Button Bar Creation
 * ====================================================================== */

static void create_button(int idx) {
    const ui_layout_t *L = ui_layout_get();
    btn_widget_t *b = &s_btn[idx];

    b->container = lv_obj_create(s_screen);
    lv_obj_remove_style_all(b->container);
    lv_obj_set_pos(b->container, L->btn.x[idx], L->btn.y);
    lv_obj_set_size(b->container, L->btn.w, L->btn.h);
    lv_obj_set_style_pad_all(b->container, 0, 0);
    lv_obj_set_scrollbar_mode(b->container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(b->container, LV_OBJ_FLAG_SCROLLABLE);

    if (L->btn_style.compact) {
        lv_obj_set_style_bg_color(b->container, L->btn_style.bg_color[idx], 0);
    } else {
        lv_obj_set_style_bg_color(b->container, ui_clr_black(), 0);
    }
    lv_obj_set_style_bg_opa(b->container, LV_OPA_COVER, 0);

    b->icon = lv_image_create(b->container);
    lv_obj_set_pos(b->icon, L->btn_style.icon_ofs_x, L->btn_style.icon_ofs_y);
    ui_style_img_recolor(b->icon, ui_clr_white());
    lv_obj_add_flag(b->icon, LV_OBJ_FLAG_HIDDEN);

    if (!L->btn_style.compact) {
        b->label = lv_label_create(b->container);
        lv_obj_set_pos(b->label, L->btn.label_ofs_x, L->btn.label_ofs_y);
        lv_obj_set_style_text_color(b->label, ui_clr_white(), 0);
        lv_obj_set_style_text_font(b->label, L->font_body, 0);
        lv_label_set_text(b->label, "");
    } else {
        b->label = NULL;
    }
}

static void create_button_bar(void) {
    for (int i = 0; i < 3; i++)
        create_button(i);
}

/* ======================================================================
 * Button Update Helpers
 * ====================================================================== */

static void set_button(int idx, const lv_image_dsc_t *icon,
                       lv_color_t icon_clr, const char *label) {
    const ui_layout_t *L = ui_layout_get();
    btn_widget_t *b = &s_btn[idx];
    ui_set_visible(b->container, true);

    if (icon) {
        lv_image_set_src(b->icon, icon);
        /* On compact (colored-square) buttons the icon sits on a bright
         * background, so render it in black for contrast. */
        lv_color_t effective_clr = L->btn_style.compact ? ui_clr_black() : icon_clr;
        ui_style_img_recolor(b->icon, effective_clr);
        ui_set_visible(b->icon, true);
    } else {
        ui_set_visible(b->icon, false);
    }

    if (b->label != NULL) {
        lv_label_set_text(b->label, label ? label : "");
    }
}

static void hide_button(int idx) {
    ui_set_visible(s_btn[idx].container, false);
}

/* ======================================================================
 * Camera Block Update
 * ====================================================================== */

static void update_camera_block(int idx) {
    cam_block_t *cb = &s_cam[idx];
    const camera_state_t *st = &g_camera_states[idx];

    camera_connection_state_t conn = st->connection_state;
    bool sleeping   = st->is_sleeping;
    bool recording  = st->is_recording;
    uint8_t cam_mode = st->camera_mode;

    /* Selection indicator */
    bool selected = slot_cycle_selected_has(idx);
    lv_obj_set_style_bg_color(cb->indicator, selected ? ui_clr_red() : ui_clr_black(), 0);

    /* Status icon (always visible) */
    const lv_image_dsc_t *status_ico;
    lv_color_t status_clr;
    get_status_icon_and_color(idx, st, &status_ico, &status_clr);
    lv_image_set_src(cb->status_icon, status_ico);
    ui_style_img_recolor(cb->status_icon, status_clr);
    ui_set_visible(cb->status_icon, true);

    /* Visibility based on connection state */
    bool show_title = (conn != CAM_STATE_UNPAIRED);
    bool show_full  = (conn == CAM_STATE_CONNECTED);

    ui_set_visible(cb->title,      show_title);
    if (cb->mode_icon)  ui_set_visible(cb->mode_icon,  show_full);
    if (cb->vmode_label) ui_set_visible(cb->vmode_label, show_full);
    ui_set_visible(cb->time_label, show_full);
    ui_set_visible(cb->sd_icon,    show_full);
    ui_set_visible(cb->sd_text,    show_full);
    ui_set_visible(cb->bat_icon,   show_full);
    ui_set_visible(cb->bat_text,   show_full);
    if (!show_full) {
        /* Backgrounds follow their rows; shown again by apply_row_style() */
        ui_set_visible(cb->sd_bg,  false);
        ui_set_visible(cb->bat_bg, false);
    }

    if (!show_title) return;

    lv_label_set_text(cb->title, st->model_name);

    if (!show_full) return;

    /* Camera mode icon + video mode text (absent on 320x170) */
    if (cb->mode_icon != NULL) {
        lv_image_set_src(cb->mode_icon, get_mode_icon(cam_mode));
        ui_style_img_recolor(cb->mode_icon, ui_clr_white());

        if (st->camera_supports_new_status_push && st->mode_name[0] != '\0') {
            lv_label_set_text_fmt(cb->vmode_label, "%s\n%s",
                                  st->mode_name, st->mode_param);
        } else {
            const char *res = short_resolution(st->video_resolution);
            const char *fps = fps_idx_to_string((fps_idx_t)st->fps_idx);
            const char *eis = eis_mode_to_string((eis_mode_t)st->eis_mode);
            lv_label_set_text_fmt(cb->vmode_label, "%s\n%s %s", res, fps, eis);
        }
    }

    /* Recording time */
    uint32_t time_val = 0;
    if (recording)        time_val = st->record_time;
    else if (!sleeping)   time_val = st->remain_time;

    if (time_val > 0) {
        uint32_t h = time_val / 3600;
        uint32_t m = (time_val % 3600) / 60;
        uint32_t s = time_val % 60;
        if (h > 99) { h = 99; m = 59; s = 59; }
        lv_label_set_text_fmt(cb->time_label, "%02lu:%02lu:%02lu",
                              (unsigned long)h, (unsigned long)m, (unsigned long)s);
    } else {
        lv_label_set_text(cb->time_label, "");
    }

    /* SD card */
    uint32_t cap = st->remain_capacity;
    apply_row_style(cb->sd_bg, cb->sd_icon, cb->sd_text, sd_level(cap));
    if (cap >= 1024) {
        uint32_t gb_int = cap / 1024;
        uint32_t gb_dec = (cap % 1024) * 10 / 1024;
        lv_label_set_text_fmt(cb->sd_text, "%lu.%luGB",
                              (unsigned long)gb_int, (unsigned long)gb_dec);
    } else {
        lv_label_set_text_fmt(cb->sd_text, "%luMB", (unsigned long)cap);
    }

    /* Battery */
    uint8_t bat_pct = st->camera_bat_percentage;
    const lv_image_dsc_t *bat_ico = get_battery_lvgl_icon(bat_pct);
    lv_image_set_src(cb->bat_icon, bat_ico);
    apply_row_style(cb->bat_bg, cb->bat_icon, cb->bat_text, bat_level(bat_pct));
    lv_label_set_text_fmt(cb->bat_text, "%d%%", bat_pct);
}

/* ======================================================================
 * GPS Update
 * ====================================================================== */

static void update_gps(void) {
    const ui_layout_t *L = ui_layout_get();
    bool has_fix = gps_has_fix();
    gps_data_t gps_data;

    if (has_fix && gps_get_data(&gps_data) == ESP_OK) {
        ui_style_img_recolor(s_gps_icon, L->status.gps_fix_color);
        if (s_gps_text != NULL) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%.2f, %.2f",
                     gps_data.latitude, gps_data.longitude);
            lv_label_set_text(s_gps_text, buf);
        }
    } else {
        ui_style_img_recolor(s_gps_icon, ui_clr_red());
        if (s_gps_text != NULL) {
            lv_label_set_text(s_gps_text, "");
        }
    }
}

/* ======================================================================
 * Button Bar Update
 * ====================================================================== */

static void update_button_a(void) {
    const lv_image_dsc_t *icon = NULL;
    const char *label = "";

    int single = slot_cycle_selected_single();
    if (single < 0) {
        /* Multi-slot entry: aggregate over the entry's slots only */
        bool any_recording = false;
        bool any_connected_sleeping = false;
        bool any_awake = false;
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (!slot_cycle_selected_has(i)) continue;
            if (command_logic_slot_is_paired(i) &&
                command_logic_slot_is_connected(i)) {
                if (command_logic_slot_is_awake(i)) {
                    any_awake = true;
                    if (command_logic_slot_is_recording(i)) {
                        any_recording = true;
                    }
                } else {
                    any_connected_sleeping = true;
                }
            }
        }
        if (any_recording) {
            icon = &lvgl_pause_icon; label = "Stop";
        } else if (any_connected_sleeping && !any_awake) {
            icon = &lvgl_record_icon; label = "Snapsht";
        } else {
            icon = &lvgl_record_icon; label = "Record";
        }
    } else {
        int slot = single;
        if (!command_logic_slot_is_paired(slot)) {
            icon = &lvgl_pair_icon; label = "Pair";
        } else if (!command_logic_slot_is_connected(slot)) {
            icon = &lvgl_connect_icon; label = "Connect";
        } else if (command_logic_slot_is_recording(slot)) {
            icon = &lvgl_pause_icon; label = "Stop";
        } else if (g_camera_states[slot].is_sleeping) {
            icon = &lvgl_record_icon; label = "Snapsht";
        } else if (g_camera_states[slot].camera_mode == 0x05) {
            icon = &lvgl_photo_icon; label = "Photo";
        } else {
            icon = &lvgl_record_icon; label = "Record";
        }
    }
    set_button(0, icon, ui_clr_white(), label);
}

static void update_button_b(void) {
    if (ui_main_slots_shortcut_active()) {
        set_button(1, &lvgl_slots_icon, ui_clr_white(), "Slots");
    } else {
        set_button(1, &lvgl_next_icon, ui_clr_white(), "Next");
    }
}

static void update_button_c(void) {
    const lv_image_dsc_t *icon = NULL;
    const char *label = "";

    int single = slot_cycle_selected_single();
    if (single < 0) {
        /* Multi-slot entry: aggregate over the entry's slots only */
        bool any_highlight = false;
        bool any_sleep = false;
        bool any_wake  = false;
        for (int i = 0; i < NUM_CAMERAS; i++) {
            if (!slot_cycle_selected_has(i)) continue;
            if (command_logic_slot_is_paired(i) &&
                command_logic_slot_is_connected(i) &&
                command_logic_slot_is_awake(i)) {
                if (command_logic_slot_is_recording(i) &&
                    command_logic_slot_supports_highlight(i))
                    any_highlight = true;
                if (!command_logic_slot_is_recording(i))
                    any_sleep = true;
            }
            if (command_logic_slot_is_paired(i) &&
                command_logic_slot_is_connected(i) &&
                g_camera_states[i].is_sleeping)
                any_wake = true;
        }
        if (any_highlight)      { icon = &lvgl_highlight_icon; label = "Tag"; }
        else if (any_sleep)     { icon = &lvgl_sleep_icon;     label = "Sleep"; }
        else if (any_wake)      { icon = &lvgl_wakeup_icon;    label = "Wakeup"; }
        else                    { hide_button(2); return; }
    } else {
        int slot = single;
        if (command_logic_slot_is_paired(slot) &&
            command_logic_slot_is_connected(slot) &&
            command_logic_slot_is_awake(slot) &&
            command_logic_slot_is_recording(slot) &&
            command_logic_slot_supports_highlight(slot)) {
            icon = &lvgl_highlight_icon; label = "Tag";
        } else {
            icon = &lvgl_options_icon; label = "Options";
        }
    }
    set_button(2, icon, ui_clr_white(), label);
}

static void update_buttons(void) {
    if (connect_logic_is_boot_scan_active()) {
        hide_button(0);
        update_button_b();      /* "Slots" during the boot scan (ADR-026) */
        set_button(2, &lvgl_scanning_icon, ui_clr_white(), "Stop");
        return;
    }
    if (connect_logic_is_boot_connect_in_progress()) {
        hide_button(0);
        hide_button(1);
        hide_button(2);
        return;
    }
    update_button_a();
    update_button_b();
    update_button_c();
}

/* ======================================================================
 * Public API
 * ====================================================================== */

lv_obj_t *ui_screen_main_create(void) {
    if (s_screen != NULL) {
        return s_screen;
    }

    s_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_screen, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
    lv_obj_set_scrollbar_mode(s_screen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < NUM_CAMERAS; i++) {
        create_camera_block(i);
    }

    /* On small screens only the selected camera is visible */
    const ui_layout_t *L = ui_layout_get();
    if (L->cam.count < NUM_CAMERAS) {
        for (int i = 1; i < NUM_CAMERAS; i++)
            ui_set_visible(s_cam[i].block, false);
    }

    create_status_bar();
    create_button_bar();

    if (L->busy.enabled) {
        s_busy_timer = lv_timer_create(busy_timer_cb, BUSY_TIMER_PERIOD_MS, NULL);
    }

    /* Initial dynamic update before the caller loads the screen. Without it
     * the LVGL task renders the placeholder widget state first and the next
     * main-loop ui_update_display() repaints it with real data, which shows
     * as a visible flicker when returning from another screen. */
    ui_screen_main_update();

    ESP_LOGI(TAG, "Main screen created (LVGL, %ld-col)",
             (long)ui_layout_get()->cam.count);
    return s_screen;
}

void ui_screen_main_update(void) {
    if (s_screen == NULL) return;

    const ui_layout_t *L = ui_layout_get();

    /* On small screens, show only the selected camera block */
    if (L->cam.count < NUM_CAMERAS) {
        int visible_idx = slot_cycle_selected_first();
        for (int i = 0; i < NUM_CAMERAS; i++)
            ui_set_visible(s_cam[i].block, (i == visible_idx));
    }

    for (int i = 0; i < NUM_CAMERAS; i++) {
        update_camera_block(i);
    }
    sync_busy_icons();
    update_gps();
    update_buttons();
}

void ui_screen_main_destroy(void) {
    ui_screen_main_cancel_notification_timer();
    if (s_busy_timer != NULL) {
        lv_timer_delete(s_busy_timer);
        s_busy_timer = NULL;
    }
    if (s_screen != NULL) {
        lv_obj_delete(s_screen);
        s_screen = NULL;
        memset(s_cam, 0, sizeof(s_cam));
        memset(s_btn, 0, sizeof(s_btn));
        s_notification = NULL;
        s_gps_icon     = NULL;
        s_gps_text     = NULL;
        ESP_LOGI(TAG, "Main screen destroyed");
    }
}

bool ui_screen_main_is_created(void) {
    return s_screen != NULL;
}

void ui_screen_main_set_notification(const char *text, lv_color_t color) {
    if (s_notification == NULL) return;
    lv_obj_set_style_text_color(s_notification, color, 0);
    lv_label_set_text(s_notification, text ? text : "");
}

void ui_screen_main_clear_notification(void) {
    if (s_notification == NULL) return;
    lv_label_set_text(s_notification, "");
}

/* LVGL task context: clears the posted text and ends the one-shot timer */
static void notif_timer_cb(lv_timer_t *t) {
    if (s_notification != NULL) {
        lv_label_set_text(s_notification, "");
    }
    lv_timer_delete(t);
    s_notif_timer = NULL;
}

void ui_screen_main_cancel_notification_timer(void) {
    if (s_notif_timer != NULL) {
        lv_timer_delete(s_notif_timer);
        s_notif_timer = NULL;
    }
}

void ui_screen_main_post_notification(const char *text, lv_color_t color, uint32_t duration_ms) {
    if (s_notification == NULL) return;
    ui_screen_main_set_notification(text, color);
    if (s_notif_timer != NULL) {
        lv_timer_set_period(s_notif_timer, duration_ms);
        lv_timer_reset(s_notif_timer);
    } else {
        s_notif_timer = lv_timer_create(notif_timer_cb, duration_ms, NULL);
    }
}

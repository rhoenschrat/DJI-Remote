/**
 * Mode Switch Screen module (LVGL).
 *
 * Displays the current camera mode with a large centered icon and text.
 * Button A sends the QS (Quick Switch) command; Button B returns to main.
 * Layout positions come from ui_layout_get() for multi-resolution support.
 */

#include "ui_screen_mode_switch.h"
#include "ui.h"
#include "ui_layout.h"
#include "lvgl_icons.h"
#include "command_logic.h"
#include "enums_logic.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

#define TAG "UI_SCR_MODE"

/* ======================================================================
 * Widget Structures
 * ====================================================================== */

typedef struct {
    lv_obj_t *container;
    lv_obj_t *icon;
    lv_obj_t *label;
} btn_widget_t;

/* ======================================================================
 * Static State
 * ====================================================================== */

static lv_obj_t    *s_screen     = NULL;
static int          s_camera_index = 0;
static lv_obj_t    *s_title      = NULL;
static lv_obj_t    *s_mode_icon  = NULL;
static lv_obj_t    *s_mode_text  = NULL;
static btn_widget_t s_btn_a;
static btn_widget_t s_btn_b;

/* ======================================================================
 * Resolution Helper (same as ui_screen_main.c)
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
 * Widget Creation
 * ====================================================================== */

static void create_button(btn_widget_t *b, int btn_idx,
                           const lv_image_dsc_t *ico, const char *lbl) {
    const ui_layout_t *L = ui_layout_get();

    b->container = lv_obj_create(s_screen);
    lv_obj_remove_style_all(b->container);
    lv_obj_set_pos(b->container, L->btn.x[btn_idx], L->btn.y);
    lv_obj_set_size(b->container, L->btn.w, L->btn.h);
    lv_obj_set_style_bg_color(b->container, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(b->container, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(b->container, 0, 0);
    lv_obj_set_scrollbar_mode(b->container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(b->container, LV_OBJ_FLAG_SCROLLABLE);

    b->icon = lv_image_create(b->container);
    lv_obj_set_pos(b->icon, 0, 0);
    lv_image_set_src(b->icon, ico);
    ui_style_img_recolor(b->icon, ui_clr_white());

    b->label = lv_label_create(b->container);
    lv_obj_set_pos(b->label, L->btn.label_ofs_x, L->btn.label_ofs_y);
    lv_obj_set_style_text_color(b->label, ui_clr_white(), 0);
    lv_obj_set_style_text_font(b->label, L->font_body, 0);
    lv_label_set_text(b->label, lbl);
}

/* ======================================================================
 * Public API
 * ====================================================================== */

lv_obj_t *ui_screen_mode_switch_create(int camera_index) {
    const ui_layout_t *L = ui_layout_get();

    if (s_screen != NULL) {
        lv_obj_delete(s_screen);
        s_screen = NULL;
    }

    s_camera_index = camera_index;

    s_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_screen, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
    lv_obj_set_scrollbar_mode(s_screen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE);

    /* Title */
    s_title = lv_label_create(s_screen);
    lv_obj_set_pos(s_title, L->submenu.base_x,
                   L->submenu.base_y + L->submenu.title_ofs_y);
    lv_obj_set_width(s_title, L->submenu.w);
    lv_obj_set_style_text_align(s_title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_title, ui_clr_white(), 0);
    lv_obj_set_style_text_font(s_title, L->font_heading, 0);
    lv_label_set_text_fmt(s_title, "Camera %d", camera_index + 1);

    /* Camera mode icon (centered) */
    s_mode_icon = lv_image_create(s_screen);
    lv_obj_set_pos(s_mode_icon,
                   L->submenu.base_x + (L->submenu.w - L->icon_size) / 2,
                   L->submenu.base_y + L->mode.icon_ofs_y);
    lv_image_set_src(s_mode_icon, &lvgl_video_icon);
    ui_style_img_recolor(s_mode_icon, ui_clr_white());

    /* Video mode text (centered, larger font) */
    s_mode_text = lv_label_create(s_screen);
    lv_obj_set_pos(s_mode_text, L->submenu.base_x,
                   L->submenu.base_y + L->mode.label_ofs_y);
    lv_obj_set_width(s_mode_text, L->submenu.w);
    lv_obj_set_style_text_align(s_mode_text, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_mode_text, ui_clr_white(), 0);
    lv_obj_set_style_text_font(s_mode_text, L->font_heading, 0);
    lv_obj_set_style_text_line_space(s_mode_text, 4, 0);
    lv_label_set_text(s_mode_text, "");

    /* Button bar */
    create_button(&s_btn_a, 0, &lvgl_switch_icon, "Switch");
    create_button(&s_btn_b, 1, &lvgl_back_icon, "Back");

    ui_screen_mode_switch_update();

    ESP_LOGI(TAG, "Mode switch screen created for slot %d", camera_index);
    return s_screen;
}

void ui_screen_mode_switch_update(void) {
    if (s_screen == NULL) return;

    const camera_state_t *cam = &g_camera_states[s_camera_index];

    const lv_image_dsc_t *mode_ico = (cam->camera_mode == 0x05)
                                         ? &lvgl_photo_icon
                                         : &lvgl_video_icon;
    lv_image_set_src(s_mode_icon, mode_ico);
    ui_style_img_recolor(s_mode_icon, ui_clr_white());

    if (cam->camera_supports_new_status_push && cam->mode_name[0] != '\0') {
        lv_label_set_text_fmt(s_mode_text, "%s\n%s",
                              cam->mode_name, cam->mode_param);
    } else {
        const char *res = short_resolution(cam->video_resolution);
        const char *fps = fps_idx_to_string((fps_idx_t)cam->fps_idx);
        const char *eis = eis_mode_to_string((eis_mode_t)cam->eis_mode);
        lv_label_set_text_fmt(s_mode_text, "%s\n%s %s", res, fps, eis);
    }
}

void ui_screen_mode_switch_destroy(void) {
    if (s_screen != NULL) {
        lv_obj_delete(s_screen);
        s_screen = NULL;
        s_title = NULL;
        s_mode_icon = NULL;
        s_mode_text = NULL;
        memset(&s_btn_a, 0, sizeof(s_btn_a));
        memset(&s_btn_b, 0, sizeof(s_btn_b));
        ESP_LOGI(TAG, "Mode switch screen destroyed");
    }
}

bool ui_screen_mode_switch_is_created(void) {
    return s_screen != NULL;
}

int ui_screen_mode_switch_get_camera_index(void) {
    return s_camera_index;
}

/* ======================================================================
 * Button Handlers
 * ====================================================================== */

void ui_screen_mode_switch_button_a(void) {
    const camera_state_t *cam = &g_camera_states[s_camera_index];

    if (!cam->is_connected) {
        ESP_LOGW(TAG, "Camera %d not connected", s_camera_index + 1);
        return;
    }

    bool is_sleeping = (cam->power_mode == 3) || cam->is_sleeping;
    if (is_sleeping) {
        ESP_LOGW(TAG, "Camera %d is sleeping", s_camera_index + 1);
        return;
    }

    esp_err_t ret = command_logic_send_key_report_for_slot(
        s_camera_index, 0x02, 0x01, 0x00);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "QS command sent to camera %d", s_camera_index + 1);
    } else {
        ESP_LOGW(TAG, "QS command failed for camera %d: %s",
                 s_camera_index + 1, esp_err_to_name(ret));
    }
}

void ui_screen_mode_switch_button_b(void) {
    /* Return to main screen -- handled by ui.c via ui_switch_screen */
}

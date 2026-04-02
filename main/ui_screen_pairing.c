/**
 * Pairing Screen module (LVGL).
 *
 * Displays a submenu with discovered cameras for BLE pairing.
 * Layout positions come from ui_layout_get() for multi-resolution support.
 */

#include "ui_screen_pairing.h"
#include "ui.h"
#include "ui_layout.h"
#include "lvgl_icons.h"
#include "connect_logic.h"
#include "ble.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

#define TAG "UI_SCR_PAIR"

/* ======================================================================
 * Widget Structures
 * ====================================================================== */

typedef struct {
    lv_obj_t *indicator;
    lv_obj_t *icon;
    lv_obj_t *label;
} entry_widget_t;

typedef struct {
    lv_obj_t *container;
    lv_obj_t *icon;
    lv_obj_t *label;
} btn_widget_t;

/* ======================================================================
 * Static State
 * ====================================================================== */

#define MAX_PAIRING_ENTRIES 5

static lv_obj_t      *s_screen       = NULL;
static int             s_camera_index = 0;
static lv_obj_t       *s_title        = NULL;
static entry_widget_t  s_entries[MAX_PAIRING_ENTRIES];
static int             s_entry_count  = 0;
static lv_obj_t       *s_notification = NULL;
static btn_widget_t    s_btn_a;
static btn_widget_t    s_btn_b;

static int  s_selection_index = 0;
static int  s_discovered_count = 0;

/* ======================================================================
 * Widget Creation
 * ====================================================================== */

static void create_entry(int idx) {
    const ui_layout_t *L = ui_layout_get();
    entry_widget_t *e = &s_entries[idx];
    int y = ui_submenu_entry_y(L, idx);

    e->indicator = lv_obj_create(s_screen);
    lv_obj_remove_style_all(e->indicator);
    lv_obj_set_pos(e->indicator, L->submenu.base_x, y);
    lv_obj_set_size(e->indicator, L->submenu.indicator_w, L->submenu.entry_h);
    lv_obj_set_style_bg_color(e->indicator, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(e->indicator, LV_OPA_COVER, 0);
    lv_obj_clear_flag(e->indicator, LV_OBJ_FLAG_SCROLLABLE);

    e->icon = lv_image_create(s_screen);
    lv_obj_set_pos(e->icon, L->submenu.base_x + L->submenu.icon_ofs_x,
                   y + L->submenu.icon_ofs_y);
    lv_image_set_src(e->icon, &lvgl_back_icon);
    ui_style_img_recolor(e->icon, ui_clr_white());

    e->label = lv_label_create(s_screen);
    lv_obj_set_pos(e->label, L->submenu.base_x + L->submenu.label_ofs_x,
                   y + L->submenu.label_ofs_y);
    lv_obj_set_style_text_color(e->label, ui_clr_white(), 0);
    lv_obj_set_style_text_font(e->label, L->font_heading, 0);
    lv_label_set_text(e->label, "");
}

static void create_button(btn_widget_t *b, int btn_idx,
                           const lv_image_dsc_t *ico, const char *lbl) {
    const ui_layout_t *L = ui_layout_get();

    b->container = lv_obj_create(s_screen);
    lv_obj_remove_style_all(b->container);

    if (L->submenu_btns.column) {
        lv_obj_set_pos(b->container,
                       L->submenu_btns.col_x,
                       L->submenu_btns.col_btn_y[btn_idx]);
        lv_obj_set_size(b->container, 27, L->submenu_btns.col_btn_h[btn_idx]);
    } else {
        lv_obj_set_pos(b->container, L->btn.x[btn_idx], L->btn.y);
        lv_obj_set_size(b->container, L->btn.w, L->btn.h);
    }

    lv_color_t bg = L->btn_style.compact
                    ? L->btn_style.bg_color[btn_idx]
                    : ui_clr_black();
    lv_obj_set_style_bg_color(b->container, bg, 0);
    lv_obj_set_style_bg_opa(b->container, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(b->container, 0, 0);
    lv_obj_set_scrollbar_mode(b->container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(b->container, LV_OBJ_FLAG_SCROLLABLE);

    b->icon = lv_image_create(b->container);
    lv_obj_set_pos(b->icon, L->btn_style.icon_ofs_x, L->btn_style.icon_ofs_y);
    lv_image_set_src(b->icon, ico);
    ui_style_img_recolor(b->icon, L->btn_style.compact ? ui_clr_black() : ui_clr_white());

    /* Rotate Button B (next_icon) 90° CW to point downward on all targets. */
    if (btn_idx == 1) {
        lv_image_set_pivot(b->icon, 12, 12);
        lv_image_set_rotation(b->icon, 900);
    }

    if (!L->btn_style.compact) {
        b->label = lv_label_create(b->container);
        lv_obj_set_pos(b->label, L->btn.label_ofs_x, L->btn.label_ofs_y);
        lv_obj_set_style_text_color(b->label, ui_clr_white(), 0);
        lv_obj_set_style_text_font(b->label, L->font_body, 0);
        lv_label_set_text(b->label, lbl);
    } else {
        b->label = NULL;
    }
}

/* ======================================================================
 * Public API
 * ====================================================================== */

lv_obj_t *ui_screen_pairing_create(int camera_index) {
    const ui_layout_t *L = ui_layout_get();

    if (s_screen != NULL) {
        lv_obj_delete(s_screen);
        s_screen = NULL;
    }

    s_camera_index = camera_index;
    s_selection_index = 0;
    s_discovered_count = 0;
    s_entry_count = L->submenu.max_entries;
    if (s_entry_count > MAX_PAIRING_ENTRIES)
        s_entry_count = MAX_PAIRING_ENTRIES;

    s_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_screen, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
    lv_obj_set_scrollbar_mode(s_screen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE);

    /* Title — omitted on 320x170 (column layout uses full height for entries) */
    if (!L->submenu_btns.column) {
        s_title = lv_label_create(s_screen);
        lv_obj_set_pos(s_title, L->submenu.base_x,
                       L->submenu.base_y + L->submenu.title_ofs_y);
        lv_obj_set_width(s_title, L->submenu.w);
        lv_obj_set_style_text_align(s_title, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(s_title, ui_clr_white(), 0);
        lv_obj_set_style_text_font(s_title, L->font_heading, 0);
        lv_label_set_text_fmt(s_title, "Camera %d", camera_index + 1);
    } else {
        s_title = NULL;
    }

    /* Menu entries (0=Back, rest=discovered cameras) */
    for (int i = 0; i < s_entry_count; i++) {
        create_entry(i);
    }

    lv_image_set_src(s_entries[0].icon, &lvgl_back_icon);
    lv_label_set_text(s_entries[0].label, "Back");

    for (int i = 1; i < s_entry_count; i++) {
        ui_set_visible(s_entries[i].indicator, false);
        ui_set_visible(s_entries[i].icon, false);
        ui_set_visible(s_entries[i].label, false);
    }

    /* Notification area — omitted on 320x170 (no space) */
    if (L->submenu.notif_w > 0) {
        s_notification = lv_label_create(s_screen);
        lv_obj_set_pos(s_notification, L->submenu.notif_x, L->submenu.notif_y);
        lv_obj_set_width(s_notification, L->submenu.notif_w);
        lv_obj_set_style_text_color(s_notification, ui_clr_cyan(), 0);
        lv_obj_set_style_text_font(s_notification, L->font_body, 0);
        lv_label_set_long_mode(s_notification, LV_LABEL_LONG_CLIP);
        lv_label_set_text(s_notification, "Scanning...");
    } else {
        s_notification = NULL;
    }

    /* Button bar */
    create_button(&s_btn_a, 0, &lvgl_select_icon, "Select");
    create_button(&s_btn_b, 1, &lvgl_next_icon, "Next");

    ESP_LOGI(TAG, "Pairing screen created for slot %d", camera_index);
    return s_screen;
}

void ui_screen_pairing_update(void) {
    if (s_screen == NULL) return;

    for (int i = 0; i < s_entry_count; i++) {
        bool visible = (i == 0) || (i <= s_discovered_count && i < s_entry_count);
        ui_set_visible(s_entries[i].indicator, visible);
        ui_set_visible(s_entries[i].icon, visible);
        ui_set_visible(s_entries[i].label, visible);

        if (visible) {
            bool selected = (i == s_selection_index);
            lv_obj_set_style_bg_color(s_entries[i].indicator,
                                       selected ? ui_clr_red() : ui_clr_black(), 0);
        }
    }

    if (s_discovered_count == 0 && s_notification != NULL) {
        lv_obj_set_style_text_color(s_notification, ui_clr_cyan(), 0);
        lv_label_set_text(s_notification, "Scanning...");
    }
}

void ui_screen_pairing_destroy(void) {
    if (s_screen != NULL) {
        lv_obj_delete(s_screen);
        s_screen = NULL;
        s_title = NULL;
        s_notification = NULL;
        memset(s_entries, 0, sizeof(s_entries));
        memset(&s_btn_a, 0, sizeof(s_btn_a));
        memset(&s_btn_b, 0, sizeof(s_btn_b));
        ESP_LOGI(TAG, "Pairing screen destroyed");
    }
}

bool ui_screen_pairing_is_created(void) {
    return s_screen != NULL;
}

void ui_screen_pairing_set_notification(const char *text, lv_color_t color) {
    if (s_notification == NULL) return;
    lv_obj_set_style_text_color(s_notification, color, 0);
    lv_label_set_text(s_notification, text ? text : "");
}

void ui_screen_pairing_add_camera(const char *name, int index) {
    if (s_screen == NULL || index < 0 || index >= (s_entry_count - 1)) return;

    int entry_idx = index + 1;
    s_discovered_count = index + 1;

    char truncated[24];
    strncpy(truncated, name, sizeof(truncated) - 1);
    truncated[sizeof(truncated) - 1] = '\0';

    lv_image_set_src(s_entries[entry_idx].icon, &lvgl_photo_icon);
    ui_style_img_recolor(s_entries[entry_idx].icon, ui_clr_white());
    lv_label_set_text(s_entries[entry_idx].label, truncated);

    ui_set_visible(s_entries[entry_idx].indicator, true);
    ui_set_visible(s_entries[entry_idx].icon, true);
    ui_set_visible(s_entries[entry_idx].label, true);

    if (s_notification != NULL) {
        lv_label_set_text(s_notification, "");
    }
}

void ui_screen_pairing_reset_cameras(void) {
    s_discovered_count = 0;
    s_selection_index = 0;
    for (int i = 1; i < s_entry_count; i++) {
        ui_set_visible(s_entries[i].indicator, false);
        ui_set_visible(s_entries[i].icon, false);
        ui_set_visible(s_entries[i].label, false);
    }
}

int ui_screen_pairing_get_selection(void) {
    return s_selection_index;
}

int ui_screen_pairing_get_camera_index(void) {
    return s_camera_index;
}

/* ======================================================================
 * Button Handlers
 * ====================================================================== */

void ui_screen_pairing_button_a(void) {
    /* Actual pairing logic remains in ui.c */
}

void ui_screen_pairing_button_b(void) {
    int max_selection = (s_discovered_count < (s_entry_count - 1))
                            ? s_discovered_count : (s_entry_count - 1);
    s_selection_index++;
    if (s_selection_index > max_selection) {
        s_selection_index = 0;
    }

    for (int i = 0; i < s_entry_count; i++) {
        if (!lv_obj_has_flag(s_entries[i].indicator, LV_OBJ_FLAG_HIDDEN)) {
            bool selected = (i == s_selection_index);
            lv_obj_set_style_bg_color(s_entries[i].indicator,
                                       selected ? ui_clr_red() : ui_clr_black(), 0);
        }
    }
}

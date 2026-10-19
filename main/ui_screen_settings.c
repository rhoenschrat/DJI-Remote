/**
 * Camera Settings Screen module (LVGL).
 *
 * Displays a submenu with per-camera actions (Mode Switch, Sleep/Wake,
 * Connect/Disconnect, Pair/Unpair). Button A selects, Button B moves to the
 * next entry, Button C returns to the main screen (handled in ui.c).
 * Layout positions come from ui_layout_get() for multi-resolution support.
 */

#include "ui_screen_settings.h"
#include "ui.h"
#include "ui_layout.h"
#include "lvgl_icons.h"
#include "slot_busy.h"
#include "esp_log.h"
#include <string.h>

#define TAG "UI_SCR_SETT"

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

static lv_obj_t       *s_screen       = NULL;
static int             s_camera_index = 0;
static lv_obj_t       *s_title        = NULL;
static entry_widget_t  s_entries[SETTINGS_ITEM_COUNT];
static int             s_entry_count  = 0;
static lv_obj_t       *s_notification = NULL;
static btn_widget_t    s_btn_a;
static btn_widget_t    s_btn_b;
static btn_widget_t    s_btn_c;
static int             s_selection_index = 0;
/* Busy indicator of s_camera_index (plan 0017) and the timer that keeps it
 * in sync while the main task blocks; NULL where the layout has none */
static lv_obj_t       *s_busy_icon    = NULL;
static lv_timer_t     *s_busy_timer   = NULL;

#define BUSY_TIMER_PERIOD_MS 100

/* ======================================================================
 * Widget Creation
 * ====================================================================== */

static void create_entry(int idx) {
    const ui_layout_t *L = ui_layout_get();
    entry_widget_t *e = &s_entries[idx];
    int x = ui_submenu_entry_x(L, idx);
    int y = ui_submenu_entry_y(L, idx);

    e->indicator = lv_obj_create(s_screen);
    lv_obj_remove_style_all(e->indicator);
    lv_obj_set_pos(e->indicator, x, y);
    lv_obj_set_size(e->indicator, L->submenu.indicator_w, L->submenu.entry_h);
    lv_obj_set_style_bg_color(e->indicator, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(e->indicator, LV_OPA_COVER, 0);
    lv_obj_clear_flag(e->indicator, LV_OBJ_FLAG_SCROLLABLE);

    e->icon = lv_image_create(s_screen);
    lv_obj_set_pos(e->icon, x + L->submenu.icon_ofs_x,
                   y + L->submenu.icon_ofs_y);
    ui_style_img_recolor(e->icon, ui_clr_white());

    /* Fixed width + ellipsis so long text never overflows its column */
    e->label = lv_label_create(s_screen);
    lv_obj_set_pos(e->label, x + L->submenu.label_ofs_x,
                   y + L->submenu.label_ofs_y);
    lv_obj_set_width(e->label, ui_submenu_label_w(L, s_entry_count));
    lv_label_set_long_mode(e->label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(e->label, ui_clr_white(), 0);
    lv_obj_set_style_text_font(e->label, L->font_heading, 0);
    lv_label_set_text(e->label, "");
}

static void create_button(btn_widget_t *b, int btn_idx,
                           const lv_image_dsc_t *ico, const char *lbl) {
    const ui_layout_t *L = ui_layout_get();

    /* Bottom-bar button, same geometry and style as the main screen */
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
    lv_obj_set_pos(b->icon, L->btn_style.icon_ofs_x, L->btn_style.icon_ofs_y);
    lv_image_set_src(b->icon, ico);
    ui_style_img_recolor(b->icon, ui_clr_white());

    /* Rotate Button B (next_icon) 90 deg CW to point downward where the
     * preset asks for it (vertical single-column lists). */
    if (btn_idx == 1 && L->submenu.next_icon_rotated) {
        lv_image_set_pivot(b->icon, 12, 12);
        lv_image_set_rotation(b->icon, 900);
    }

    b->label = lv_label_create(b->container);
    lv_obj_set_pos(b->label, L->btn.label_ofs_x, L->btn.label_ofs_y);
    lv_obj_set_style_text_color(b->label, ui_clr_white(), 0);
    lv_obj_set_style_text_font(b->label, L->font_body, 0);
    lv_label_set_text(b->label, lbl);
}

static void sync_busy_icon(void) {
    if (s_busy_icon != NULL) {
        ui_set_visible(s_busy_icon, slot_busy_is_active(s_camera_index));
    }
}

/* LVGL task context: no extra lock needed */
static void busy_timer_cb(lv_timer_t *t) {
    (void)t;
    sync_busy_icon();
}

static void delete_busy_widgets(void) {
    if (s_busy_timer != NULL) {
        lv_timer_delete(s_busy_timer);
        s_busy_timer = NULL;
    }
    s_busy_icon = NULL;     /* child of s_screen, deleted with it */
}

/* ======================================================================
 * Public API
 * ====================================================================== */

lv_obj_t *ui_screen_settings_create(int camera_index) {
    const ui_layout_t *L = ui_layout_get();

    if (s_screen != NULL) {
        delete_busy_widgets();
        lv_obj_delete(s_screen);
        s_screen = NULL;
    }

    s_camera_index = camera_index;
    s_selection_index = 0;
    s_entry_count = L->submenu.max_entries;
    if (s_entry_count > SETTINGS_ITEM_COUNT)
        s_entry_count = SETTINGS_ITEM_COUNT;

    s_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_screen, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
    lv_obj_set_scrollbar_mode(s_screen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE);

    /* Title — omitted on 320x170 (entries use the full height) */
    if (L->submenu.show_title) {
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

    /* Menu entries */
    for (int i = 0; i < s_entry_count; i++) {
        create_entry(i);
    }

    /* Static entry: Mode Switch */
    lv_image_set_src(s_entries[SETTINGS_ITEM_MODE_SWITCH].icon, &lvgl_switch_icon);
    lv_label_set_text(s_entries[SETTINGS_ITEM_MODE_SWITCH].label, "Mode Switch");

    /* Notification area — omitted on 320x170 (no space) */
    if (L->submenu.notif_w > 0) {
        s_notification = lv_label_create(s_screen);
        lv_obj_set_pos(s_notification, L->submenu.notif_x, L->submenu.notif_y);
        lv_obj_set_width(s_notification, L->submenu.notif_w);
        lv_obj_set_style_text_color(s_notification, ui_clr_white(), 0);
        lv_obj_set_style_text_font(s_notification, L->font_body, 0);
        lv_label_set_long_mode(s_notification, LV_LABEL_LONG_CLIP);
        lv_label_set_text(s_notification, "");
    } else {
        s_notification = NULL;
    }

    /* Button bar */
    create_button(&s_btn_a, 0, &lvgl_select_icon, "Select");
    create_button(&s_btn_b, 1, &lvgl_next_icon, "Next");
    create_button(&s_btn_c, 2, &lvgl_back_icon, "Back");

    /* Busy indicator in the empty bottom-left slot (the GPS icon position of
     * the main screen); hidden until an action on this slot is running */
    if (L->busy.enabled) {
        s_busy_icon = lv_image_create(s_screen);
        lv_obj_set_pos(s_busy_icon, L->busy.menu_x, L->busy.menu_y);
        lv_image_set_src(s_busy_icon,
                         L->busy.menu_size >= 32 ? &lvgl_busy_icon_32 : &lvgl_busy_icon_24);
        ui_style_img_recolor(s_busy_icon, ui_clr_cyan());
        lv_obj_clear_flag(s_busy_icon, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(s_busy_icon, LV_OBJ_FLAG_HIDDEN);
        s_busy_timer = lv_timer_create(busy_timer_cb, BUSY_TIMER_PERIOD_MS, NULL);
    }

    /* Initial dynamic update */
    ui_screen_settings_update();

    ESP_LOGI(TAG, "Settings screen created for slot %d", camera_index);
    return s_screen;
}

void ui_screen_settings_update(void) {
    if (s_screen == NULL) return;

    const camera_state_t *cam = &g_camera_states[s_camera_index];

    /* Sleep/Wakeup */
    if (s_entry_count > SETTINGS_ITEM_SLEEP_WAKEUP) {
        entry_widget_t *e = &s_entries[SETTINGS_ITEM_SLEEP_WAKEUP];
        bool is_sleeping = (cam->power_mode == 3) || cam->is_sleeping;
        if (is_sleeping) {
            lv_image_set_src(e->icon, &lvgl_wakeup_icon);
            lv_label_set_text(e->label, "Wakeup");
        } else {
            lv_image_set_src(e->icon, &lvgl_sleep_icon);
            lv_label_set_text(e->label, "Sleep");
        }
    }

    /* Connect/Disconnect */
    if (s_entry_count > SETTINGS_ITEM_CONNECT_DISCONNECT) {
        entry_widget_t *e = &s_entries[SETTINGS_ITEM_CONNECT_DISCONNECT];
        if (cam->connection_state == CAM_STATE_CONNECTED) {
            lv_image_set_src(e->icon, &lvgl_disconnect_icon);
            lv_label_set_text(e->label, "Disconnect");
        } else {
            lv_image_set_src(e->icon, &lvgl_connect_icon);
            lv_label_set_text(e->label, "Connect");
        }
    }

    /* Pair/Unpair */
    if (s_entry_count > SETTINGS_ITEM_PAIR_UNPAIR) {
        entry_widget_t *e = &s_entries[SETTINGS_ITEM_PAIR_UNPAIR];
        if (cam->is_paired) {
            lv_image_set_src(e->icon, &lvgl_unpair_icon);
            lv_label_set_text(e->label, "Unpair");
        } else {
            lv_image_set_src(e->icon, &lvgl_pair_icon);
            lv_label_set_text(e->label, "Pair Camera");
        }
    }

    for (int i = 0; i < s_entry_count; i++) {
        ui_style_img_recolor(s_entries[i].icon, ui_clr_white());
    }

    /* Selection indicators */
    for (int i = 0; i < s_entry_count; i++) {
        bool selected = (i == s_selection_index);
        lv_obj_set_style_bg_color(s_entries[i].indicator,
                                   selected ? ui_clr_red() : ui_clr_black(), 0);
    }

    sync_busy_icon();
}

void ui_screen_settings_destroy(void) {
    if (s_screen != NULL) {
        delete_busy_widgets();
        lv_obj_delete(s_screen);
        s_screen = NULL;
        s_title = NULL;
        s_notification = NULL;
        memset(s_entries, 0, sizeof(s_entries));
        memset(&s_btn_a, 0, sizeof(s_btn_a));
        memset(&s_btn_b, 0, sizeof(s_btn_b));
        memset(&s_btn_c, 0, sizeof(s_btn_c));
        ESP_LOGI(TAG, "Settings screen destroyed");
    }
}

bool ui_screen_settings_is_created(void) {
    return s_screen != NULL;
}

void ui_screen_settings_set_notification(const char *text, lv_color_t color) {
    if (s_notification == NULL) return;
    lv_obj_set_style_text_color(s_notification, color, 0);
    lv_label_set_text(s_notification, text ? text : "");
}

int ui_screen_settings_get_selection(void) {
    return s_selection_index;
}

int ui_screen_settings_get_camera_index(void) {
    return s_camera_index;
}

/* ======================================================================
 * Button Handlers
 * ====================================================================== */

void ui_screen_settings_button_a(void) {
    /* Actual action logic remains in ui.c */
}

void ui_screen_settings_button_b(void) {
    int max_idx = (s_entry_count < SETTINGS_ITEM_COUNT)
                      ? s_entry_count : SETTINGS_ITEM_COUNT;
    s_selection_index++;
    if (s_selection_index >= max_idx) {
        s_selection_index = 0;
    }

    for (int i = 0; i < s_entry_count; i++) {
        bool selected = (i == s_selection_index);
        lv_obj_set_style_bg_color(s_entries[i].indicator,
                                   selected ? ui_clr_red() : ui_clr_black(), 0);
    }
}

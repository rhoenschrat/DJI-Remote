/**
 * Pairing Screen module (LVGL).
 *
 * Displays a submenu with discovered cameras for BLE pairing. The list
 * contains only discovered cameras (entry index == discovered index).
 * Button A pairs the selected camera, Button B moves to the next entry,
 * Button C cancels the scan and returns to main (handled in ui.c).
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

#define MAX_PAIRING_ENTRIES 8   /* 2 x 4 grid on 320x170 */

static lv_obj_t      *s_screen       = NULL;
static int             s_camera_index = 0;
static lv_obj_t       *s_title        = NULL;
static entry_widget_t  s_entries[MAX_PAIRING_ENTRIES];
static int             s_entry_count  = 0;
static lv_obj_t       *s_notification = NULL;
static lv_obj_t       *s_scan_placeholder = NULL;  /* only when notif_w == 0 */
static btn_widget_t    s_btn_a;
static btn_widget_t    s_btn_b;
static btn_widget_t    s_btn_c;

static int  s_selection_index = 0;
static int  s_discovered_count = 0;

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
    lv_image_set_src(e->icon, &lvgl_photo_icon);
    ui_style_img_recolor(e->icon, ui_clr_white());

    /* Fixed width + ellipsis; width is adjusted when a second column opens */
    e->label = lv_label_create(s_screen);
    lv_obj_set_pos(e->label, x + L->submenu.label_ofs_x,
                   y + L->submenu.label_ofs_y);
    lv_obj_set_width(e->label, ui_submenu_label_w(L, 0));
    lv_label_set_long_mode(e->label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(e->label, ui_clr_white(), 0);
    lv_obj_set_style_text_font(e->label, L->font_heading, 0);
    lv_label_set_text(e->label, "");
}

static void set_entry_visible(int idx, bool visible) {
    ui_set_visible(s_entries[idx].indicator, visible);
    ui_set_visible(s_entries[idx].icon, visible);
    ui_set_visible(s_entries[idx].label, visible);
}

static int shown_entry_count(void) {
    return (s_discovered_count < s_entry_count) ? s_discovered_count : s_entry_count;
}

/* Entry positions never move; only label widths shrink once the list spills
 * into a second column (and grow back after a reset). */
static void apply_label_widths(void) {
    const ui_layout_t *L = ui_layout_get();
    int32_t w = ui_submenu_label_w(L, shown_entry_count());
    for (int i = 0; i < s_entry_count; i++) {
        lv_obj_set_width(s_entries[i].label, w);
    }
}

static void update_scan_placeholder(void) {
    if (s_scan_placeholder == NULL) return;
    ui_set_visible(s_scan_placeholder, s_discovered_count == 0);
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

    /* Menu entries (discovered cameras only) — hidden until found */
    for (int i = 0; i < s_entry_count; i++) {
        create_entry(i);
        set_entry_visible(i, false);
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
        s_scan_placeholder = NULL;
    } else {
        s_notification = NULL;

        /* No notification area: show the scan state in the entry area at the
         * position of entry 0's label until the first camera appears. */
        s_scan_placeholder = lv_label_create(s_screen);
        lv_obj_set_pos(s_scan_placeholder,
                       ui_submenu_entry_x(L, 0) + L->submenu.label_ofs_x,
                       ui_submenu_entry_y(L, 0) + L->submenu.label_ofs_y);
        lv_obj_set_width(s_scan_placeholder, ui_submenu_label_w(L, 0));
        lv_label_set_long_mode(s_scan_placeholder, LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_color(s_scan_placeholder, ui_clr_cyan(), 0);
        lv_obj_set_style_text_font(s_scan_placeholder, L->font_heading, 0);
        lv_label_set_text(s_scan_placeholder, "Scanning...");
    }

    /* Button bar */
    create_button(&s_btn_a, 0, &lvgl_select_icon, "Select");
    create_button(&s_btn_b, 1, &lvgl_next_icon, "Next");
    create_button(&s_btn_c, 2, &lvgl_back_icon, "Back");

    ESP_LOGI(TAG, "Pairing screen created for slot %d", camera_index);
    return s_screen;
}

void ui_screen_pairing_update(void) {
    if (s_screen == NULL) return;

    for (int i = 0; i < s_entry_count; i++) {
        bool visible = (i < s_discovered_count);
        set_entry_visible(i, visible);

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
    update_scan_placeholder();
}

void ui_screen_pairing_destroy(void) {
    if (s_screen != NULL) {
        lv_obj_delete(s_screen);
        s_screen = NULL;
        s_title = NULL;
        s_notification = NULL;
        s_scan_placeholder = NULL;
        memset(s_entries, 0, sizeof(s_entries));
        memset(&s_btn_a, 0, sizeof(s_btn_a));
        memset(&s_btn_b, 0, sizeof(s_btn_b));
        memset(&s_btn_c, 0, sizeof(s_btn_c));
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
    if (s_screen == NULL || index < 0 || index >= s_entry_count) return;

    /* MAX: this is also called to rename an existing entry, which must not
     * shrink the count. */
    int entry_idx = index;
    if (index + 1 > s_discovered_count) {
        s_discovered_count = index + 1;
    }

    /* 23-char cut keeps the 320x240 look; ellipsis handles pixel overflow */
    char truncated[24];
    strncpy(truncated, name, sizeof(truncated) - 1);
    truncated[sizeof(truncated) - 1] = '\0';

    lv_image_set_src(s_entries[entry_idx].icon, &lvgl_photo_icon);
    ui_style_img_recolor(s_entries[entry_idx].icon, ui_clr_white());
    lv_label_set_text(s_entries[entry_idx].label, truncated);
    set_entry_visible(entry_idx, true);

    apply_label_widths();
    update_scan_placeholder();

    if (s_notification != NULL) {
        lv_label_set_text(s_notification, "");
    }
}

void ui_screen_pairing_reset_cameras(void) {
    s_discovered_count = 0;
    s_selection_index = 0;
    if (s_screen == NULL) return;
    for (int i = 0; i < s_entry_count; i++) {
        set_entry_visible(i, false);
    }
    apply_label_widths();
    update_scan_placeholder();
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
    int shown = shown_entry_count();
    if (shown == 0) return;

    /* Navigation order = index order = column-major fill order */
    s_selection_index++;
    if (s_selection_index >= shown) {
        s_selection_index = 0;
    }

    for (int i = 0; i < shown; i++) {
        bool selected = (i == s_selection_index);
        lv_obj_set_style_bg_color(s_entries[i].indicator,
                                   selected ? ui_clr_red() : ui_clr_black(), 0);
    }
}

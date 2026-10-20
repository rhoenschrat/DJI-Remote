/**
 * Slots configuration screens (LVGL), plan 0018 / ADR-026.
 *
 * Cycle list: the four cycle entries by derived name ("Slot 1",
 * "Slot 2 + 3", "Empty"). Entry screen: Slot 1/2/3 of one entry with a
 * green check mark at the right screen edge for every active slot.
 *
 * Rows reuse the Options Menu geometry (ui_submenu_entry_x/y); with at most
 * four rows they always form a single full-width column. No title, no
 * notification line: a rejected toggle flashes the row label red instead,
 * which works on every preset.
 */

#include "ui_screen_slots.h"
#include "ui_layout.h"
#include "lvgl_icons.h"
#include "slot_cycle.h"
#include "esp_log.h"
#include <string.h>

#define TAG "UI_SCR_SLOTS"

/* Rows: cycle list = entries, entry screen = slots */
#define SLOTS_MAX_ROWS        SLOT_CYCLE_ENTRIES
#define SLOTS_ICON_SIZE       24
#define SLOTS_FLASH_MS        1000

_Static_assert(SLOT_CYCLE_NUM_SLOTS <= SLOTS_MAX_ROWS, "row pool too small");

/* ======================================================================
 * Widget Structures
 * ====================================================================== */

typedef struct {
    lv_obj_t *indicator;
    lv_obj_t *icon;       /* cycle list: slots icon; entry screen: check mark */
    lv_obj_t *label;
} row_widget_t;

typedef struct {
    lv_obj_t *container;
    lv_obj_t *icon;
    lv_obj_t *label;
} btn_widget_t;

/* ======================================================================
 * Static State
 * ====================================================================== */

static lv_obj_t        *s_screen      = NULL;
static ui_slots_mode_t  s_mode        = SLOTS_MODE_CYCLE;
static int              s_entry_pos   = 0;
static int              s_row_count   = 0;
static int              s_selection   = 0;
static row_widget_t     s_rows[SLOTS_MAX_ROWS];
static btn_widget_t     s_btn_a;
static btn_widget_t     s_btn_b;
static btn_widget_t     s_btn_c;
/* One-shot timer that ends the red rejection flash; NULL when idle */
static lv_timer_t      *s_flash_timer = NULL;

/* ======================================================================
 * Widget Creation
 * ====================================================================== */

/* Check mark x: mirrors the left icon inset at the right screen edge */
static int32_t check_icon_x(const ui_layout_t *L) {
    return L->screen_w - (L->submenu.base_x + L->submenu.icon_ofs_x) - SLOTS_ICON_SIZE;
}

static void create_row(int idx) {
    const ui_layout_t *L = ui_layout_get();
    row_widget_t *r = &s_rows[idx];
    int32_t x = ui_submenu_entry_x(L, idx);
    int32_t y = ui_submenu_entry_y(L, idx);
    int32_t label_x = x + L->submenu.label_ofs_x;

    r->indicator = lv_obj_create(s_screen);
    lv_obj_remove_style_all(r->indicator);
    lv_obj_set_pos(r->indicator, x, y);
    lv_obj_set_size(r->indicator, L->submenu.indicator_w, L->submenu.entry_h);
    lv_obj_set_style_bg_color(r->indicator, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(r->indicator, LV_OPA_COVER, 0);
    lv_obj_clear_flag(r->indicator, LV_OBJ_FLAG_SCROLLABLE);

    r->icon = lv_image_create(s_screen);
    if (s_mode == SLOTS_MODE_CYCLE) {
        lv_obj_set_pos(r->icon, x + L->submenu.icon_ofs_x, y + L->submenu.icon_ofs_y);
        lv_image_set_src(r->icon, &lvgl_slots_icon);
        ui_style_img_recolor(r->icon, ui_clr_white());
    } else {
        lv_obj_set_pos(r->icon, check_icon_x(L), y + L->submenu.icon_ofs_y);
        lv_image_set_src(r->icon, &lvgl_check_icon);
        ui_style_img_recolor(r->icon, ui_clr_green());
        lv_obj_add_flag(r->icon, LV_OBJ_FLAG_HIDDEN);
    }

    /* Full-width single column; on the entry screen the label stops before
     * the check mark */
    int32_t label_w = ui_submenu_label_w(L, s_row_count);
    if (s_mode == SLOTS_MODE_ENTRY) {
        label_w = check_icon_x(L) - label_x - UI_SUBMENU_LABEL_MARGIN_R;
    }
    r->label = lv_label_create(s_screen);
    lv_obj_set_pos(r->label, label_x, y + L->submenu.label_ofs_y);
    lv_obj_set_width(r->label, label_w);
    lv_label_set_long_mode(r->label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(r->label, ui_clr_white(), 0);
    lv_obj_set_style_text_font(r->label, L->font_heading, 0);
    if (s_mode == SLOTS_MODE_ENTRY) {
        lv_label_set_text_fmt(r->label, "Slot %d", idx + 1);
    } else {
        lv_label_set_text(r->label, "");
    }
}

static void create_button(btn_widget_t *b, int btn_idx,
                          const lv_image_dsc_t *ico, const char *lbl) {
    const ui_layout_t *L = ui_layout_get();

    /* Bottom-bar button, same geometry and style as the Options Menu */
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

    /* Vertical list: Button B's next icon points down where the preset
     * asks for it (same as the Options Menu) */
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
 * Rejection Flash
 * ====================================================================== */

static void restore_label_colors(void) {
    for (int i = 0; i < s_row_count; i++) {
        if (s_rows[i].label != NULL) {
            lv_obj_set_style_text_color(s_rows[i].label, ui_clr_white(), 0);
        }
    }
}

/* LVGL task context: no extra lock needed. One-shot: LVGL deletes the
 * timer after this call. */
static void flash_timer_cb(lv_timer_t *t) {
    (void)t;
    s_flash_timer = NULL;
    restore_label_colors();
}

static void delete_flash_timer(void) {
    if (s_flash_timer != NULL) {
        lv_timer_delete(s_flash_timer);
        s_flash_timer = NULL;
    }
}

static void update_selection_bar(void) {
    for (int i = 0; i < s_row_count; i++) {
        lv_obj_set_style_bg_color(s_rows[i].indicator,
                                  (i == s_selection) ? ui_clr_red() : ui_clr_black(), 0);
    }
}

/* ======================================================================
 * Public API
 * ====================================================================== */

lv_obj_t *ui_screen_slots_create(ui_slots_mode_t mode, int entry_pos) {
    if (s_screen != NULL) {
        ui_screen_slots_destroy();
    }

    if (entry_pos < 0 || entry_pos >= SLOT_CYCLE_ENTRIES) {
        entry_pos = 0;
    }
    s_mode      = mode;
    s_entry_pos = entry_pos;
    s_row_count = (mode == SLOTS_MODE_CYCLE) ? SLOT_CYCLE_ENTRIES : SLOT_CYCLE_NUM_SLOTS;
    s_selection = (mode == SLOTS_MODE_CYCLE) ? entry_pos : 0;

    s_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_screen, ui_clr_black(), 0);
    lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
    lv_obj_set_scrollbar_mode(s_screen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < s_row_count; i++) {
        create_row(i);
    }

    create_button(&s_btn_a, 0, &lvgl_select_icon, "Select");
    create_button(&s_btn_b, 1, &lvgl_next_icon, "Next");
    create_button(&s_btn_c, 2, &lvgl_back_icon, "Back");

    /* Populate before the caller loads the screen (no placeholder frame) */
    ui_screen_slots_update();

    if (mode == SLOTS_MODE_CYCLE) {
        ESP_LOGI(TAG, "Slots cycle list created");
    } else {
        ESP_LOGI(TAG, "Slots entry screen created for entry %d", entry_pos + 1);
    }
    return s_screen;
}

void ui_screen_slots_update(void) {
    if (s_screen == NULL) return;

    if (s_mode == SLOTS_MODE_CYCLE) {
        for (int i = 0; i < s_row_count; i++) {
            char name[SLOT_CYCLE_NAME_LEN];
            slot_cycle_format_name(slot_cycle_get_entry(i), name, sizeof(name));
            lv_label_set_text(s_rows[i].label, name);
        }
    } else {
        slot_mask_t mask = slot_cycle_get_entry(s_entry_pos);
        for (int i = 0; i < s_row_count; i++) {
            ui_set_visible(s_rows[i].icon, (mask & SLOT_CYCLE_SLOT_BIT(i)) != 0);
        }
    }

    update_selection_bar();
}

void ui_screen_slots_destroy(void) {
    if (s_screen == NULL) return;

    /* Timer first: its callback touches the labels */
    delete_flash_timer();
    lv_obj_delete(s_screen);
    s_screen = NULL;
    s_row_count = 0;
    memset(s_rows, 0, sizeof(s_rows));
    memset(&s_btn_a, 0, sizeof(s_btn_a));
    memset(&s_btn_b, 0, sizeof(s_btn_b));
    memset(&s_btn_c, 0, sizeof(s_btn_c));
    ESP_LOGI(TAG, "Slots screen destroyed");
}

bool ui_screen_slots_is_created(void) {
    return s_screen != NULL;
}

void ui_screen_slots_button_b(void) {
    if (s_screen == NULL || s_row_count == 0) return;
    s_selection = (s_selection + 1) % s_row_count;
    update_selection_bar();
}

int ui_screen_slots_get_selection(void) {
    return s_selection;
}

int ui_screen_slots_get_entry_pos(void) {
    return s_entry_pos;
}

void ui_screen_slots_flash_rejected(void) {
    if (s_screen == NULL || s_selection < 0 || s_selection >= s_row_count) return;

    restore_label_colors();
    lv_obj_set_style_text_color(s_rows[s_selection].label, ui_clr_red(), 0);

    if (s_flash_timer != NULL) {
        lv_timer_reset(s_flash_timer);
    } else {
        s_flash_timer = lv_timer_create(flash_timer_cb, SLOTS_FLASH_MS, NULL);
        lv_timer_set_repeat_count(s_flash_timer, 1);
    }
}

/**
 * LVGL icon descriptors — A1 alpha format.
 *
 * Converts the legacy 1-bit packed bitmaps (icons.c; 24x24 = 72 bytes, the
 * busy indicator also 32x32 = 128 bytes) into LV_COLOR_FORMAT_A1 images by
 * inverting the bit polarity at startup.
 *
 * Original bitmap polarity: 1 = background fill, 0 = icon shape.
 * A1 polarity:              1 = opaque (icon),    0 = transparent (bg).
 *
 * The foreground colour is applied per-widget with
 * lv_obj_set_style_image_recolor() + lv_obj_set_style_image_recolor_opa().
 *
 * Stride is set explicitly (width / 8 bytes per row, e.g. 3 for 24 px) to
 * avoid LVGL's automatic stride alignment padding.
 */

#include "icons.h"
#include "lvgl_icons.h"
#include <assert.h>
#include <string.h>

#define ICON_COUNT_24  32                       /* 24x24 icons in the table */
#define ICON_COUNT_32  1                        /* 32x32 icons in the table */
#define ICON_COUNT     (ICON_COUNT_24 + ICON_COUNT_32)
#define ICON_BYTES(sz) (((sz) / 8) * (sz))      /* 24 -> 72, 32 -> 128 */
#define ICON_POOL_SIZE (ICON_COUNT_24 * ICON_BYTES(24) + ICON_COUNT_32 * ICON_BYTES(32))

/* ----- A1 pixel buffers (inverted copies, filled by lvgl_icons_init) --- */

static uint8_t s_a1_pool[ICON_POOL_SIZE];

/* ----- Image descriptors (filled once by lvgl_icons_init) -------------- */

lv_image_dsc_t lvgl_bluetooth_icon;
lv_image_dsc_t lvgl_record_icon;
lv_image_dsc_t lvgl_mode_icon;
lv_image_dsc_t lvgl_gps_icon;
lv_image_dsc_t lvgl_sleep_icon;
lv_image_dsc_t lvgl_key_icon;
lv_image_dsc_t lvgl_location_icon;
lv_image_dsc_t lvgl_sd_card_icon;
lv_image_dsc_t lvgl_battery_0_icon;
lv_image_dsc_t lvgl_battery_25_icon;
lv_image_dsc_t lvgl_battery_50_icon;
lv_image_dsc_t lvgl_battery_75_icon;
lv_image_dsc_t lvgl_battery_100_icon;
lv_image_dsc_t lvgl_pause_icon;
lv_image_dsc_t lvgl_video_icon;
lv_image_dsc_t lvgl_photo_icon;
lv_image_dsc_t lvgl_scanning_icon;
lv_image_dsc_t lvgl_found_icon;
lv_image_dsc_t lvgl_connecting_icon;
lv_image_dsc_t lvgl_back_icon;
lv_image_dsc_t lvgl_highlight_icon;
lv_image_dsc_t lvgl_wakeup_icon;
lv_image_dsc_t lvgl_connect_icon;
lv_image_dsc_t lvgl_disconnect_icon;
lv_image_dsc_t lvgl_pair_icon;
lv_image_dsc_t lvgl_unpair_icon;
lv_image_dsc_t lvgl_select_icon;
lv_image_dsc_t lvgl_next_icon;
lv_image_dsc_t lvgl_options_icon;
lv_image_dsc_t lvgl_switch_icon;
lv_image_dsc_t lvgl_alert_icon;
lv_image_dsc_t lvgl_busy_icon_32;
lv_image_dsc_t lvgl_busy_icon_24;

/* ----- Helpers --------------------------------------------------------- */

static void invert_bitmap(const uint8_t *src, uint8_t *dst, int bytes)
{
    for (int i = 0; i < bytes; i++)
        dst[i] = (uint8_t)~src[i];
}

static void init_dsc(lv_image_dsc_t *dsc, const uint8_t *a1_data, int size)
{
    memset(dsc, 0, sizeof(*dsc));
    dsc->header.cf     = LV_COLOR_FORMAT_A1;
    dsc->header.w      = size;
    dsc->header.h      = size;
    dsc->header.stride = size / 8;
    dsc->data_size     = ICON_BYTES(size);
    dsc->data          = a1_data;
}

/* ----- Public API ------------------------------------------------------ */

typedef struct {
    lv_image_dsc_t *dsc;
    const uint8_t  *src_1bit;
    uint8_t         size;       /* width = height in pixels (24 or 32) */
} icon_entry_t;

void lvgl_icons_init(void)
{
    static const icon_entry_t table[ICON_COUNT] = {
        { &lvgl_bluetooth_icon,    bluetooth_icon,    24 },
        { &lvgl_record_icon,       record_icon,       24 },
        { &lvgl_mode_icon,         mode_icon,         24 },
        { &lvgl_gps_icon,          gps_icon,          24 },
        { &lvgl_sleep_icon,        sleep_icon,        24 },
        { &lvgl_key_icon,          key_icon,          24 },
        { &lvgl_location_icon,     location_icon,     24 },
        { &lvgl_sd_card_icon,      sd_card_icon,      24 },
        { &lvgl_battery_0_icon,    battery_0_icon,    24 },
        { &lvgl_battery_25_icon,   battery_25_icon,   24 },
        { &lvgl_battery_50_icon,   battery_50_icon,   24 },
        { &lvgl_battery_75_icon,   battery_75_icon,   24 },
        { &lvgl_battery_100_icon,  battery_100_icon,  24 },
        { &lvgl_pause_icon,        pause_icon,        24 },
        { &lvgl_video_icon,        video_icon,        24 },
        { &lvgl_photo_icon,        photo_icon,        24 },
        { &lvgl_scanning_icon,     scanning_icon,     24 },
        { &lvgl_found_icon,        found_icon,        24 },
        { &lvgl_connecting_icon,   connecting_icon,   24 },
        { &lvgl_back_icon,         back_icon,         24 },
        { &lvgl_highlight_icon,    highlight_icon,    24 },
        { &lvgl_wakeup_icon,       wakeup_icon,       24 },
        { &lvgl_connect_icon,      connect_icon,      24 },
        { &lvgl_disconnect_icon,   disconnect_icon,   24 },
        { &lvgl_pair_icon,         pair_icon,         24 },
        { &lvgl_unpair_icon,       unpair_icon,       24 },
        { &lvgl_select_icon,       select_icon,       24 },
        { &lvgl_next_icon,         next_icon,         24 },
        { &lvgl_options_icon,      options_icon,      24 },
        { &lvgl_switch_icon,       switch_icon,       24 },
        { &lvgl_alert_icon,        alert_icon,        24 },
        { &lvgl_busy_icon_24,      busy_24_icon,      24 },
        { &lvgl_busy_icon_32,      busy_32_icon,      32 },
    };

    size_t ofs = 0;
    for (int i = 0; i < ICON_COUNT; i++) {
        int size = table[i].size;
        int bytes = ICON_BYTES(size);
        /* ICON_COUNT_24/32 must match the table */
        assert(ofs + bytes <= sizeof(s_a1_pool));
        invert_bitmap(table[i].src_1bit, &s_a1_pool[ofs], bytes);
        init_dsc(table[i].dsc, &s_a1_pool[ofs], size);
        ofs += bytes;
    }
}

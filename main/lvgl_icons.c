/**
 * LVGL icon descriptors — A1 alpha format.
 *
 * Converts the legacy 1-bit packed bitmaps (icons.c, 72 bytes each) into
 * LV_COLOR_FORMAT_A1 images by inverting the bit polarity at startup.
 *
 * Original bitmap polarity: 1 = background fill, 0 = icon shape.
 * A1 polarity:              1 = opaque (icon),    0 = transparent (bg).
 *
 * The foreground colour is applied per-widget with
 * lv_obj_set_style_image_recolor() + lv_obj_set_style_image_recolor_opa().
 *
 * Stride is set to 3 explicitly (24 px / 8 = 3 bytes per row) to avoid
 * LVGL's automatic stride alignment padding.
 */

#include "icons.h"
#include "lvgl_icons.h"
#include <string.h>

#define ICON_W         24
#define ICON_H         24
#define ICON_STRIDE    3                        /* 24 / 8 */
#define ICON_DATA_SIZE (ICON_STRIDE * ICON_H)   /* 72     */
#define ICON_COUNT     30

/* ----- A1 pixel buffers (inverted copies, filled by lvgl_icons_init) --- */

static uint8_t s_a1[ICON_COUNT][ICON_DATA_SIZE];

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

/* ----- Helpers --------------------------------------------------------- */

static void invert_bitmap(const uint8_t *src, uint8_t *dst)
{
    for (int i = 0; i < ICON_DATA_SIZE; i++)
        dst[i] = (uint8_t)~src[i];
}

static void init_dsc(lv_image_dsc_t *dsc, const uint8_t *a1_data)
{
    memset(dsc, 0, sizeof(*dsc));
    dsc->header.cf     = LV_COLOR_FORMAT_A1;
    dsc->header.w      = ICON_W;
    dsc->header.h      = ICON_H;
    dsc->header.stride = ICON_STRIDE;
    dsc->data_size     = ICON_DATA_SIZE;
    dsc->data          = a1_data;
}

/* ----- Public API ------------------------------------------------------ */

typedef struct {
    lv_image_dsc_t *dsc;
    const uint8_t  *src_1bit;
} icon_entry_t;

void lvgl_icons_init(void)
{
    static const icon_entry_t table[ICON_COUNT] = {
        { &lvgl_bluetooth_icon,   bluetooth_icon   },
        { &lvgl_record_icon,      record_icon      },
        { &lvgl_mode_icon,        mode_icon        },
        { &lvgl_gps_icon,         gps_icon         },
        { &lvgl_sleep_icon,       sleep_icon       },
        { &lvgl_key_icon,         key_icon         },
        { &lvgl_location_icon,    location_icon    },
        { &lvgl_sd_card_icon,     sd_card_icon     },
        { &lvgl_battery_0_icon,   battery_0_icon   },
        { &lvgl_battery_25_icon,  battery_25_icon  },
        { &lvgl_battery_50_icon,  battery_50_icon  },
        { &lvgl_battery_75_icon,  battery_75_icon  },
        { &lvgl_battery_100_icon, battery_100_icon },
        { &lvgl_pause_icon,       pause_icon       },
        { &lvgl_video_icon,       video_icon       },
        { &lvgl_photo_icon,       photo_icon       },
        { &lvgl_scanning_icon,    scanning_icon    },
        { &lvgl_found_icon,       found_icon       },
        { &lvgl_connecting_icon,  connecting_icon  },
        { &lvgl_back_icon,        back_icon        },
        { &lvgl_highlight_icon,   highlight_icon   },
        { &lvgl_wakeup_icon,      wakeup_icon      },
        { &lvgl_connect_icon,     connect_icon     },
        { &lvgl_disconnect_icon,  disconnect_icon  },
        { &lvgl_pair_icon,        pair_icon        },
        { &lvgl_unpair_icon,      unpair_icon      },
        { &lvgl_select_icon,      select_icon      },
        { &lvgl_next_icon,        next_icon        },
        { &lvgl_options_icon,     options_icon     },
        { &lvgl_switch_icon,      switch_icon      },
    };

    for (int i = 0; i < ICON_COUNT; i++) {
        invert_bitmap(table[i].src_1bit, s_a1[i]);
        init_dsc(table[i].dsc, s_a1[i]);
    }
}

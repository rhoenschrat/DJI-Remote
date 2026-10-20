/*
 * Configurable Main Screen slot cycle (plan 0018, ADR-026)
 */

#include "slot_cycle.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs.h"

#define TAG "SLOT_CYCLE"

/* Default NVS partition (user settings, written from the main task); the
 * diag partition is reserved for fault events (ADR-022) */
#define NVS_SETTINGS_NAMESPACE "settings"
#define NVS_SLOT_CYCLE_KEY     "slot_cycle"

/* Stored u32: bits 0..11 = four 3-bit masks (entry n at bit 3n),
 * bits 16..23 = format version, bits 24..31 = magic */
#define SLOT_CYCLE_MAGIC        0x5Cu
#define SLOT_CYCLE_VERSION      1u
#define SLOT_CYCLE_MASK_BITS    3
#define SLOT_CYCLE_PAYLOAD_MASK 0x0FFFu
#define SLOT_CYCLE_RESERVED     0x0000F000u

_Static_assert(SLOT_CYCLE_NUM_SLOTS <= SLOT_CYCLE_MASK_BITS,
               "slot mask does not fit the stored format");
_Static_assert(SLOT_CYCLE_ENTRIES * SLOT_CYCLE_MASK_BITS <= 12,
               "cycle does not fit the stored payload");

/* Factory default: 1 -> 2 -> 3 -> 1+2+3 (the former fixed cycle) */
static const slot_mask_t s_default[SLOT_CYCLE_ENTRIES] = {
    SLOT_CYCLE_SLOT_BIT(0),
    SLOT_CYCLE_SLOT_BIT(1),
    SLOT_CYCLE_SLOT_BIT(2),
    SLOT_CYCLE_MASK_ALL,
};

/* Working configuration (edited by the Slots screens) and the copy last
 * loaded from or written to NVS (change detection). Initialised to the
 * default so queries are valid even before slot_cycle_init(). */
static slot_mask_t s_entries[SLOT_CYCLE_ENTRIES] = {
    SLOT_CYCLE_SLOT_BIT(0),
    SLOT_CYCLE_SLOT_BIT(1),
    SLOT_CYCLE_SLOT_BIT(2),
    SLOT_CYCLE_MASK_ALL,
};
static slot_mask_t s_persisted[SLOT_CYCLE_ENTRIES] = {
    SLOT_CYCLE_SLOT_BIT(0),
    SLOT_CYCLE_SLOT_BIT(1),
    SLOT_CYCLE_SLOT_BIT(2),
    SLOT_CYCLE_MASK_ALL,
};
static int s_position = 0;

/* ----- Helpers ----------------------------------------------------------- */

int slot_cycle_mask_count(slot_mask_t mask)
{
    int n = 0;
    for (int s = 0; s < SLOT_CYCLE_NUM_SLOTS; s++) {
        if (mask & SLOT_CYCLE_SLOT_BIT(s)) {
            n++;
        }
    }
    return n;
}

static bool pos_valid(int pos)
{
    return pos >= 0 && pos < SLOT_CYCLE_ENTRIES;
}

/* Consistency rule: >= 1 non-empty entry. Entries containing only
 * multi-slot sets are allowed: the Slots configuration stays reachable from
 * the boot phase (Button B), see ADR-026. */
static bool config_valid(const slot_mask_t cfg[SLOT_CYCLE_ENTRIES])
{
    bool any_non_empty = false;
    for (int i = 0; i < SLOT_CYCLE_ENTRIES; i++) {
        if ((cfg[i] & ~SLOT_CYCLE_MASK_ALL) != 0) {
            return false;
        }
        if (cfg[i] != SLOT_CYCLE_MASK_EMPTY) {
            any_non_empty = true;
        }
    }
    return any_non_empty;
}

/* First non-empty entry after `from`, wrapping; `from` itself is checked
 * last. Returns `from` if every entry is empty (prevented by the rule). */
static int next_non_empty(int from)
{
    for (int step = 1; step <= SLOT_CYCLE_ENTRIES; step++) {
        int pos = (from + step) % SLOT_CYCLE_ENTRIES;
        if (s_entries[pos] != SLOT_CYCLE_MASK_EMPTY) {
            return pos;
        }
    }
    return from;
}

static uint32_t pack(const slot_mask_t cfg[SLOT_CYCLE_ENTRIES])
{
    uint32_t v = (SLOT_CYCLE_MAGIC << 24) | (SLOT_CYCLE_VERSION << 16);
    for (int i = 0; i < SLOT_CYCLE_ENTRIES; i++) {
        v |= (uint32_t)(cfg[i] & SLOT_CYCLE_MASK_ALL) << (i * SLOT_CYCLE_MASK_BITS);
    }
    return v;
}

static bool unpack(uint32_t v, slot_mask_t cfg[SLOT_CYCLE_ENTRIES])
{
    if ((v >> 24) != SLOT_CYCLE_MAGIC ||
        ((v >> 16) & 0xFFu) != SLOT_CYCLE_VERSION ||
        (v & SLOT_CYCLE_RESERVED) != 0) {
        return false;
    }
    for (int i = 0; i < SLOT_CYCLE_ENTRIES; i++) {
        uint32_t m = (v >> (i * SLOT_CYCLE_MASK_BITS)) & ((1u << SLOT_CYCLE_MASK_BITS) - 1u);
        if (m & ~(uint32_t)SLOT_CYCLE_MASK_ALL) {
            return false;
        }
        cfg[i] = (slot_mask_t)m;
    }
    return config_valid(cfg);
}

static void log_config(const char *what)
{
    char names[SLOT_CYCLE_ENTRIES][SLOT_CYCLE_NAME_LEN];
    for (int i = 0; i < SLOT_CYCLE_ENTRIES; i++) {
        slot_cycle_format_name(s_entries[i], names[i], sizeof(names[i]));
    }
    ESP_LOGI(TAG, "%s: [%s] -> [%s] -> [%s] -> [%s]", what,
             names[0], names[1], names[2], names[3]);
}

/* ----- Public API -------------------------------------------------------- */

void slot_cycle_init(void)
{
    slot_mask_t loaded[SLOT_CYCLE_ENTRIES];
    bool ok = false;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_SETTINGS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        uint32_t v = 0;
        err = nvs_get_u32(h, NVS_SLOT_CYCLE_KEY, &v);
        nvs_close(h);
        if (err == ESP_OK) {
            ok = unpack(v, loaded);
            if (!ok) {
                ESP_LOGW(TAG, "Stored slot cycle 0x%08lX invalid, using default",
                         (unsigned long)v);
            }
        } else if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "Reading slot cycle failed (%s), using default",
                     esp_err_to_name(err));
        }
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        /* NOT_FOUND: namespace never written (factory state) */
        ESP_LOGW(TAG, "Opening settings NVS failed (%s), using default",
                 esp_err_to_name(err));
    }

    memcpy(s_entries, ok ? loaded : s_default, sizeof(s_entries));
    /* Invalid data is not rewritten here: the default is saved only when
     * the user edits the cycle (no NVS write at boot) */
    memcpy(s_persisted, s_entries, sizeof(s_persisted));

    s_position = (s_entries[0] != SLOT_CYCLE_MASK_EMPTY) ? 0 : next_non_empty(0);
    log_config(ok ? "Slot cycle loaded" : "Slot cycle default");
}

slot_mask_t slot_cycle_get_entry(int pos)
{
    return pos_valid(pos) ? s_entries[pos] : SLOT_CYCLE_MASK_EMPTY;
}

int slot_cycle_get_position(void)
{
    return s_position;
}

bool slot_cycle_toggle(int pos, int slot)
{
    if (!pos_valid(pos) || slot < 0 || slot >= SLOT_CYCLE_NUM_SLOTS) {
        return false;
    }

    slot_mask_t candidate[SLOT_CYCLE_ENTRIES];
    memcpy(candidate, s_entries, sizeof(candidate));
    candidate[pos] ^= SLOT_CYCLE_SLOT_BIT(slot);

    if (!config_valid(candidate)) {
        ESP_LOGI(TAG, "Entry %d: toggling slot %d rejected (needs a non-empty "
                 "entry)", pos + 1, slot + 1);
        return false;
    }

    s_entries[pos] = candidate[pos];
    char name[SLOT_CYCLE_NAME_LEN];
    slot_cycle_format_name(s_entries[pos], name, sizeof(name));
    ESP_LOGI(TAG, "Entry %d: slot %d %s -> %s", pos + 1, slot + 1,
             (s_entries[pos] & SLOT_CYCLE_SLOT_BIT(slot)) ? "added" : "removed", name);
    return true;
}

int slot_cycle_next(void)
{
    s_position = next_non_empty(s_position);
    return s_position;
}

void slot_cycle_normalize_selection(void)
{
    if (s_entries[s_position] == SLOT_CYCLE_MASK_EMPTY) {
        s_position = next_non_empty(s_position);
    }
}

slot_mask_t slot_cycle_selected_mask(void)
{
    return s_entries[s_position];
}

bool slot_cycle_selected_has(int slot)
{
    if (slot < 0 || slot >= SLOT_CYCLE_NUM_SLOTS) {
        return false;
    }
    return (s_entries[s_position] & SLOT_CYCLE_SLOT_BIT(slot)) != 0;
}

int slot_cycle_selected_single(void)
{
    slot_mask_t m = s_entries[s_position];
    if (slot_cycle_mask_count(m) != 1) {
        return -1;
    }
    return slot_cycle_selected_first();
}

int slot_cycle_selected_first(void)
{
    slot_mask_t m = s_entries[s_position];
    for (int s = 0; s < SLOT_CYCLE_NUM_SLOTS; s++) {
        if (m & SLOT_CYCLE_SLOT_BIT(s)) {
            return s;
        }
    }
    return 0;
}

void slot_cycle_format_name(slot_mask_t mask, char *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return;
    }
    if ((mask & SLOT_CYCLE_MASK_ALL) == SLOT_CYCLE_MASK_EMPTY) {
        snprintf(buf, len, "Empty");
        return;
    }

    size_t used = (size_t)snprintf(buf, len, "Slot");
    bool first = true;
    for (int s = 0; s < SLOT_CYCLE_NUM_SLOTS && used < len; s++) {
        if (mask & SLOT_CYCLE_SLOT_BIT(s)) {
            used += (size_t)snprintf(buf + used, len - used,
                                     first ? " %d" : " + %d", s + 1);
            first = false;
        }
    }
}

bool slot_cycle_save_if_changed(void)
{
    if (memcmp(s_entries, s_persisted, sizeof(s_entries)) == 0) {
        ESP_LOGD(TAG, "Slot cycle unchanged, not saved");
        return false;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_SETTINGS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Opening settings NVS failed (%s), slot cycle not saved",
                 esp_err_to_name(err));
        return false;
    }
    err = nvs_set_u32(h, NVS_SLOT_CYCLE_KEY, pack(s_entries));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err != ESP_OK) {
        /* s_persisted stays unchanged, so the next Back retries */
        ESP_LOGW(TAG, "Saving slot cycle failed (%s)", esp_err_to_name(err));
        return false;
    }
    memcpy(s_persisted, s_entries, sizeof(s_persisted));
    log_config("Slot cycle saved");
    return true;
}

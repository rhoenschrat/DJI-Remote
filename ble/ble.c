/*
 * DJI Camera Remote Control - BLE Communication Layer (NimBLE)
 *
 * GATT Client for connecting to DJI cameras. Uses NimBLE stack with a unified
 * GAP event callback and per-operation GATTC completion callbacks.
 *
 * Key differences from the previous Bluedroid implementation:
 * - Single gap_event_cb() replaces separate GAP + GATTC handlers
 * - No gattc_if concept; operations use conn_handle directly
 * - Must cancel scan before initiating a connection
 * - Notification data arrives as os_mbuf (copied to flat buffer)
 * - GATT discovery uses chained per-operation callbacks
 * - ble_gap_disc() has built-in timeout (no manual FreeRTOS timer needed)
 */

#include <string.h>
#include <stdint.h>
#include "ble.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "../main/ui.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#define TAG "BLE"

/* ----------------------------------------------------------------
 *  Static state
 * ---------------------------------------------------------------- */

static bool s_connecting = false;
static bool s_ble_ready = false;
static bool s_advertising_active = false;

static struct ble_gap_event_listener s_global_listener;

static ble_notify_callback_t s_notify_cb = NULL;
static connect_logic_state_callback_t s_state_cb = NULL;

static scan_controller_t s_scan_controller = {
    .mode = SCAN_MODE_IDLE,
    .target_slot = -1,
    .autoconnect_pending = {false, false, false},
    .scan_timeout_ms = 0,
    .scan_start_time = 0
};

static bool s_ble_scan_active = false;

ble_profile_t s_ble_profiles[BLE_MAX_CAMERAS] = {0};

static int s_active_scan_camera_index = -1;

/* ----------------------------------------------------------------
 *  UUIDs
 * ---------------------------------------------------------------- */

#define REMOTE_TARGET_SERVICE_UUID  0xFFF0
#define REMOTE_NOTIFY_CHAR_UUID     0xFFF4
#define REMOTE_WRITE_CHAR_UUID      0xFFF5

static const ble_uuid16_t s_cccd_uuid = BLE_UUID16_INIT(0x2902);

/* ----------------------------------------------------------------
 *  Forward declarations
 * ---------------------------------------------------------------- */

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void try_to_connect(int camera_index, const uint8_t *addr);

/* GATT discovery chain */
static int on_mtu_exchanged(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg);
static int on_svc_discovered(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc, void *arg);
static int on_chr_discovered(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg);
static int on_dsc_discovered(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg);

/* ----------------------------------------------------------------
 *  Profile helpers
 * ---------------------------------------------------------------- */

static void init_ble_profiles(void) {
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        s_ble_profiles[i].conn_id = UINT16_MAX;
        s_ble_profiles[i].camera_index = -1;
        s_ble_profiles[i].notify_char_handle = 0;
        s_ble_profiles[i].write_char_handle = 0;
        s_ble_profiles[i].read_char_handle = 0;
        s_ble_profiles[i].cccd_handle = 0;
        s_ble_profiles[i].service_start_handle = 0;
        s_ble_profiles[i].service_end_handle = 0;
        memset(s_ble_profiles[i].remote_bda, 0, 6);
        memset(s_ble_profiles[i].target_name, 0, sizeof(s_ble_profiles[i].target_name));
        memset(s_ble_profiles[i].target_mac, 0, 6);
        s_ble_profiles[i].connection_status.is_connected = false;
        s_ble_profiles[i].handle_discovery.notify_char_handle_found = false;
        s_ble_profiles[i].handle_discovery.write_char_handle_found = false;
    }
    ESP_LOGI(TAG, "Initialized %d BLE profiles for multi-camera support", BLE_MAX_CAMERAS);
}

static ble_profile_t* get_profile_by_camera_index(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        return NULL;
    }
    return &s_ble_profiles[camera_index];
}

static ble_profile_t* get_profile_by_conn_id(uint16_t conn_id) {
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        if (s_ble_profiles[i].camera_index >= 0 &&
            s_ble_profiles[i].conn_id == conn_id &&
            s_ble_profiles[i].connection_status.is_connected) {
            return &s_ble_profiles[i];
        }
    }
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        if (s_ble_profiles[i].camera_index >= 0 && s_ble_profiles[i].conn_id == conn_id) {
            return &s_ble_profiles[i];
        }
    }
    return NULL;
}

__attribute__((unused))
static ble_profile_t* get_profile_by_bda(const uint8_t *bda) {
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        if (s_ble_profiles[i].camera_index >= 0 &&
            memcmp(s_ble_profiles[i].remote_bda, bda, 6) == 0) {
            return &s_ble_profiles[i];
        }
    }
    return NULL;
}

/* ----------------------------------------------------------------
 *  Address byte-order conversion
 *
 *  Bluedroid (and our NVS / target_mac / remote_bda fields) stores
 *  addresses big-endian (MSB first): E4:7A:2C:D1:C8:B8
 *  NimBLE ble_addr_t.val is little-endian (LSB first): B8:C8:D1:2C:7A:E4
 *  This helper converts between the two representations.
 * ---------------------------------------------------------------- */

static void bda_reverse(const uint8_t *src, uint8_t *dst) {
    dst[0] = src[5];
    dst[1] = src[4];
    dst[2] = src[3];
    dst[3] = src[2];
    dst[4] = src[1];
    dst[5] = src[0];
}

/* ----------------------------------------------------------------
 *  Advertisement parsing helpers
 * ---------------------------------------------------------------- */

static const uint8_t* find_adv_field(const uint8_t *data, uint8_t data_len,
                                     uint8_t type, uint8_t *out_len)
{
    int i = 0;
    while (i < data_len) {
        uint8_t len = data[i];
        if (len == 0 || (i + len + 1) > data_len) break;
        if (data[i + 1] == type) {
            *out_len = len - 1;
            return &data[i + 2];
        }
        i += len + 1;
    }
    *out_len = 0;
    return NULL;
}

static uint8_t is_dji_camera_adv(const uint8_t *data, uint8_t data_len) {
    uint8_t mfg_len = 0;
    const uint8_t *mfg = find_adv_field(data, data_len,
                                         BLE_HS_ADV_TYPE_MFG_DATA, &mfg_len);
    if (mfg && mfg_len >= 5 &&
        mfg[0] == 0xAA && mfg[1] == 0x08 && mfg[4] == 0xFA) {
        return 1;
    }
    return 0;
}

static uint32_t get_dji_device_id(const uint8_t *data, uint8_t data_len) {
    uint8_t mfg_len = 0;
    const uint8_t *mfg = find_adv_field(data, data_len,
                                         BLE_HS_ADV_TYPE_MFG_DATA, &mfg_len);
    if (mfg && mfg_len >= 5 &&
        mfg[0] == 0xAA && mfg[1] == 0x08 && mfg[4] == 0xFA) {
        return (uint32_t)((uint16_t)mfg[2] | ((uint16_t)mfg[3] << 8));
    }
    return 0;
}

static const char* extract_device_name(const uint8_t *data, uint8_t data_len) {
    static char name_buf[64];
    uint8_t name_len = 0;

    /* Try complete name first, then shortened/incomplete name */
    const uint8_t *name = find_adv_field(data, data_len,
                                          BLE_HS_ADV_TYPE_COMP_NAME, &name_len);
    if (!name || name_len == 0) {
        name = find_adv_field(data, data_len,
                               BLE_HS_ADV_TYPE_INCOMP_NAME, &name_len);
    }
    if (name && name_len > 0) {
        size_t copy_len = name_len < sizeof(name_buf) - 1 ? name_len : sizeof(name_buf) - 1;
        memcpy(name_buf, name, copy_len);
        name_buf[copy_len] = '\0';
        return name_buf;
    }
    return "DJI Camera";
}

/* ----------------------------------------------------------------
 *  NimBLE host callbacks
 * ---------------------------------------------------------------- */

static SemaphoreHandle_t s_ble_sync_sem = NULL;

static void on_sync(void) {
    ble_hs_util_ensure_addr(0);
    s_ble_ready = true;
    if (s_ble_sync_sem) {
        xSemaphoreGive(s_ble_sync_sem);
    }
}

static void on_reset(int reason) {
    ESP_LOGW(TAG, "NimBLE host reset, reason=%d", reason);
}

static int global_event_listener(struct ble_gap_event *event, void *arg)
{
    (void)event;
    (void)arg;
    return 0;
}

static void nimble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ----------------------------------------------------------------
 *  GATT discovery chain callbacks
 * ---------------------------------------------------------------- */

static int on_mtu_exchanged(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t mtu, void *arg)
{
    int camera_index = (int)(intptr_t)arg;
    ble_profile_t *profile = get_profile_by_camera_index(camera_index);
    if (!profile) return 0;

    if (error->status != 0) {
        ESP_LOGW(TAG, "Camera %d: MTU exchange status=%d, using default", camera_index, error->status);
    }
    ESP_LOGI(TAG, "Camera %d: MTU=%d", camera_index, mtu);

    /* Discover ALL services (not just 0xFFF0) to match Bluedroid's
       esp_ble_gattc_search_service(NULL) behavior. Some peripherals
       require full GATT database discovery before enabling notifications. */
    ble_gattc_disc_all_svcs(conn_handle, on_svc_discovered, arg);
    return 0;
}

static int on_svc_discovered(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_svc *svc, void *arg)
{
    int camera_index = (int)(intptr_t)arg;
    ble_profile_t *profile = get_profile_by_camera_index(camera_index);
    if (!profile) return 0;

    if (error->status == 0 && svc != NULL) {
        uint16_t uuid16 = 0;
        if (svc->uuid.u.type == BLE_UUID_TYPE_16) {
            uuid16 = BLE_UUID16(&svc->uuid.u)->value;
        }
        if (uuid16 == REMOTE_TARGET_SERVICE_UUID) {
            profile->service_start_handle = svc->start_handle;
            profile->service_end_handle   = svc->end_handle;
            ESP_LOGI(TAG, "Camera %d: Service 0x%04X found: start=%d, end=%d",
                     camera_index, REMOTE_TARGET_SERVICE_UUID,
                     svc->start_handle, svc->end_handle);
        } else {
            ESP_LOGD(TAG, "Camera %d: Skipping service 0x%04X (start=%d, end=%d)",
                     camera_index, uuid16, svc->start_handle, svc->end_handle);
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (profile->service_start_handle != 0) {
            ble_gattc_disc_all_chrs(conn_handle,
                                     profile->service_start_handle,
                                     profile->service_end_handle,
                                     on_chr_discovered, arg);
        } else {
            ESP_LOGE(TAG, "Camera %d: DJI service not found", camera_index);
            s_active_scan_camera_index = -1;
        }
    }
    return 0;
}

static int on_chr_discovered(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_chr *chr, void *arg)
{
    int camera_index = (int)(intptr_t)arg;
    ble_profile_t *profile = get_profile_by_camera_index(camera_index);
    if (!profile) return 0;

    if (error->status == 0 && chr != NULL) {
        uint16_t uuid16 = 0;
        if (chr->uuid.u.type == BLE_UUID_TYPE_16) {
            uuid16 = BLE_UUID16(&chr->uuid.u)->value;
        }

        if (uuid16 == REMOTE_NOTIFY_CHAR_UUID) {
            profile->notify_char_handle = chr->val_handle;
            profile->handle_discovery.notify_char_handle_found = true;
            ESP_LOGI(TAG, "Camera %d: Notify Char (0x%04X) found, handle=0x%x",
                     camera_index, uuid16, chr->val_handle);
        } else if (uuid16 == REMOTE_WRITE_CHAR_UUID) {
            profile->write_char_handle = chr->val_handle;
            profile->handle_discovery.write_char_handle_found = true;
            ESP_LOGI(TAG, "Camera %d: Write Char (0x%04X) found, handle=0x%x",
                     camera_index, uuid16, chr->val_handle);
        } else {
            ESP_LOGI(TAG, "Camera %d: Char 0x%04X found, handle=0x%x (def=0x%x)",
                     camera_index, uuid16, chr->val_handle, chr->def_handle);
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (profile->notify_char_handle != 0) {
            uint16_t dsc_end = profile->service_end_handle;
            if (profile->write_char_handle > profile->notify_char_handle) {
                dsc_end = profile->write_char_handle - 1;
            }
            ble_gattc_disc_all_dscs(conn_handle,
                                     profile->notify_char_handle,
                                     dsc_end,
                                     on_dsc_discovered, arg);
        } else {
            s_active_scan_camera_index = -1;
            ESP_LOGI(TAG, "Camera %d: GATT discovery complete (no notify char)", camera_index);
        }
    }
    return 0;
}

static int on_dsc_discovered(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             uint16_t chr_val_handle,
                             const struct ble_gatt_dsc *dsc, void *arg)
{
    int camera_index = (int)(intptr_t)arg;
    ble_profile_t *profile = get_profile_by_camera_index(camera_index);
    if (!profile) return 0;

    if (error->status == 0 && dsc != NULL) {
        if (ble_uuid_cmp(&dsc->uuid.u, &s_cccd_uuid.u) == 0) {
            if (profile->cccd_handle == 0) {
                profile->cccd_handle = dsc->handle;
                ESP_LOGI(TAG, "Camera %d: CCCD found, handle=0x%x",
                         camera_index, dsc->handle);
            }
        }
    } else if (error->status == BLE_HS_EDONE) {
        s_active_scan_camera_index = -1;
        ESP_LOGI(TAG, "Camera %d: GATT discovery complete (cccd=0x%04x notify=0x%04x write=0x%04x)",
                 camera_index, profile->cccd_handle,
                 profile->notify_char_handle, profile->write_char_handle);
    }
    return 0;
}

/* ----------------------------------------------------------------
 *  GATTC read/write completion callbacks
 * ---------------------------------------------------------------- */

static int on_read_complete(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            struct ble_gatt_attr *attr, void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "Read failed, conn_handle=%d status=%d", conn_handle, error->status);
    }
    return 0;
}

static int on_write_complete(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attr, void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "Write failed, conn_handle=%d attr_handle=0x%x status=%d",
                 conn_handle, attr ? attr->handle : 0, error->status);
        ble_profile_t *profile = get_profile_by_conn_id(conn_handle);
        if (profile) {
            profile->connection_status.is_connected = false;
            ESP_LOGW(TAG, "Camera %d: Marking as disconnected due to write failure",
                     profile->camera_index);
        }
    } else {
        ESP_LOGI(TAG, "Write complete, conn_handle=%d attr_handle=0x%x",
                 conn_handle, attr ? attr->handle : 0);
    }
    return 0;
}

/* ----------------------------------------------------------------
 *  Unified GAP event callback
 * ---------------------------------------------------------------- */

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    ble_profile_t *profile = NULL;

    switch (event->type) {

    /* ---------- Scan result ---------- */
    case BLE_GAP_EVENT_DISC: {
        const uint8_t *adv = event->disc.data;
        uint8_t adv_len    = event->disc.length_data;

        /* Pure scan responses (name only, no manufacturer data) can update
         * the displayed name of an already-discovered camera during pairing.
         * Note: some controllers combine ADV_IND + SCAN_RSP into a single
         * report, so we must NOT break here — always fall through to the
         * is_dji_camera_adv() check which handles combined reports. */
        if (!is_dji_camera_adv(adv, adv_len)) {
            if (s_scan_controller.mode == SCAN_MODE_PAIRING) {
                const char *rsp_name = extract_device_name(adv, adv_len);
                if (strcmp(rsp_name, "DJI Camera") != 0) {
                    uint8_t bda_be[6];
                    bda_reverse(event->disc.addr.val, bda_be);
                    extern void ui_pairing_update_discovered_camera_name(
                            const char *name, const uint8_t *mac);
                    ui_pairing_update_discovered_camera_name(rsp_name, bda_be);
                }
            }
            break;
        }

        const char *adv_name_str = extract_device_name(adv, adv_len);
        uint32_t device_id       = get_dji_device_id(adv, adv_len);
        int8_t rssi              = event->disc.rssi;

        uint8_t bda_be[6];
        bda_reverse(event->disc.addr.val, bda_be);

        ESP_LOGI(TAG, "Found device: %s RSSI=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X "
                 "device_id=0x%04X mode=%d",
                 adv_name_str, rssi,
                 bda_be[0], bda_be[1], bda_be[2], bda_be[3], bda_be[4], bda_be[5],
                 (unsigned int)device_id, s_scan_controller.mode);

        switch (s_scan_controller.mode) {
        case SCAN_MODE_PAIRING: {
            extern void ui_pairing_add_discovered_camera(const char *name,
                    const uint8_t *mac, int8_t rssi, uint32_t device_id);
            ui_pairing_add_discovered_camera(adv_name_str, bda_be, rssi, device_id);
            break;
        }
        case SCAN_MODE_AUTOCONNECT_BOOT: {
            for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
                if (!s_scan_controller.autoconnect_pending[i]) continue;
                profile = get_profile_by_camera_index(i);
                if (!profile) continue;

                if (memcmp(profile->target_mac, bda_be, 6) == 0) {
                    ESP_LOGI(TAG, "AUTOCONNECT_BOOT: Found camera for slot %d: %s", i, adv_name_str);
                    extern void connect_logic_mark_slot_found(int slot_index);
                    connect_logic_mark_slot_found(i);
                    s_scan_controller.autoconnect_pending[i] = false;

                    bool all_resolved = true;
                    for (int j = 0; j < BLE_MAX_CAMERAS; j++) {
                        if (s_scan_controller.autoconnect_pending[j]) {
                            all_resolved = false;
                            break;
                        }
                    }
                    if (all_resolved) {
                        ESP_LOGI(TAG, "All autoconnect slots found, stopping scan early");
                        ble_gap_disc_cancel();
                    }
                    break;
                }
            }
            break;
        }
        case SCAN_MODE_SLOT_RECONNECT: {
            if (s_scan_controller.target_slot < 0 ||
                s_scan_controller.target_slot >= BLE_MAX_CAMERAS) break;

            profile = get_profile_by_camera_index(s_scan_controller.target_slot);
            if (!profile) break;

            if (memcmp(profile->target_mac, bda_be, 6) == 0) {
                ESP_LOGI(TAG, "SLOT_RECONNECT: Found target for slot %d: %s",
                         s_scan_controller.target_slot, adv_name_str);
                s_active_scan_camera_index = s_scan_controller.target_slot;
                try_to_connect(s_scan_controller.target_slot, bda_be);
            }
            break;
        }
        default:
            break;
        }
        break;
    }

    /* ---------- Scan complete ---------- */
    case BLE_GAP_EVENT_DISC_COMPLETE: {
        s_ble_scan_active = false;
        ESP_LOGI(TAG, "Scan complete (reason=%d, mode=%d)",
                 event->disc_complete.reason, s_scan_controller.mode);

        if (s_connecting) {
            break;
        }

        switch (s_scan_controller.mode) {
        case SCAN_MODE_PAIRING:
            ESP_LOGI(TAG, "Pairing scan completed, waiting for user selection");
            break;
        case SCAN_MODE_AUTOCONNECT_BOOT: {
            extern bool connect_logic_is_slot_found(int slot_index);
            extern void connect_logic_mark_slot_not_found_during_boot(int slot_index);
            for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
                if (s_scan_controller.autoconnect_pending[i]) {
                    if (!connect_logic_is_slot_found(i)) {
                        ESP_LOGW(TAG, "Autoconnect timeout: Camera slot %d not found", i);
                        connect_logic_mark_slot_not_found_during_boot(i);
                    } else {
                        ESP_LOGI(TAG, "Autoconnect: Camera slot %d was found", i);
                    }
                }
            }
            ESP_LOGI(TAG, "Autoconnect scan completed");
            break;
        }
        case SCAN_MODE_SLOT_RECONNECT:
            if (s_scan_controller.target_slot >= 0 &&
                s_scan_controller.target_slot < BLE_MAX_CAMERAS &&
                !s_connecting) {
                ESP_LOGW(TAG, "Slot reconnect timeout: Camera %d not found",
                         s_scan_controller.target_slot);
            }
            break;
        default:
            break;
        }

        s_scan_controller.mode = SCAN_MODE_IDLE;
        s_scan_controller.target_slot = -1;
        s_scan_controller.scan_timeout_ms = 0;
        s_scan_controller.scan_start_time = 0;
        for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
            s_scan_controller.autoconnect_pending[i] = false;
        }
        break;
    }

    /* ---------- Connection established / failed ---------- */
    case BLE_GAP_EVENT_CONNECT: {
        s_connecting = false;

        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "Connection failed, status=%d (slot %d)",
                     event->connect.status, s_active_scan_camera_index);
            s_active_scan_camera_index = -1;
            break;
        }

        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_conn_find failed: %d", rc);
            ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            break;
        }

        /* Convert NimBLE LE address to BE for comparison with stored MACs */
        uint8_t peer_bda_be[6];
        bda_reverse(desc.peer_id_addr.val, peer_bda_be);

        profile = NULL;
        for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
            if (s_ble_profiles[i].camera_index >= 0 &&
                memcmp(s_ble_profiles[i].target_mac, peer_bda_be, 6) == 0) {
                profile = &s_ble_profiles[i];
                break;
            }
        }
        if (!profile && s_active_scan_camera_index >= 0) {
            profile = get_profile_by_camera_index(s_active_scan_camera_index);
        }
        if (!profile) {
            ESP_LOGW(TAG, "Connection to unknown device, terminating");
            ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            break;
        }

        profile->conn_id = event->connect.conn_handle;
        profile->connection_status.is_connected = true;
        memcpy(profile->remote_bda, peer_bda_be, 6);

        int slot = profile->camera_index;
        if (slot < 0 || slot >= BLE_MAX_CAMERAS) {
            for (int j = 0; j < BLE_MAX_CAMERAS; j++) {
                if (&s_ble_profiles[j] == profile) {
                    profile->camera_index = j;
                    slot = j;
                    break;
                }
            }
        }

        ESP_LOGI(TAG, "Camera %d connected! conn_handle=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X",
                 slot, profile->conn_id,
                 peer_bda_be[0], peer_bda_be[1], peer_bda_be[2],
                 peer_bda_be[3], peer_bda_be[4], peer_bda_be[5]);

        ble_gattc_exchange_mtu(event->connect.conn_handle,
                                on_mtu_exchanged,
                                (void *)(intptr_t)slot);
        break;
    }

    /* ---------- Disconnection ---------- */
    case BLE_GAP_EVENT_DISCONNECT: {
        uint16_t conn_handle = event->disconnect.conn.conn_handle;
        profile = get_profile_by_conn_id(conn_handle);

        if (profile) {
            int camera_idx = profile->camera_index;
            profile->connection_status.is_connected = false;
            profile->conn_id = UINT16_MAX;
            profile->handle_discovery.write_char_handle_found = false;
            profile->handle_discovery.notify_char_handle_found = false;
            profile->notify_char_handle = 0;
            profile->write_char_handle = 0;
            profile->cccd_handle = 0;
            ESP_LOGI(TAG, "Camera %d disconnected, reason=0x%x",
                     camera_idx, event->disconnect.reason);

            if (s_active_scan_camera_index == camera_idx) {
                s_active_scan_camera_index = -1;
            }
        } else {
            ESP_LOGW(TAG, "Disconnect for unknown conn_handle=%d, reason=0x%x",
                     conn_handle, event->disconnect.reason);
        }

        s_connecting = false;
        if (s_state_cb) {
            s_state_cb();
        }
        break;
    }

    /* ---------- Notification / indication received ---------- */
    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t conn_handle = event->notify_rx.conn_handle;
        uint16_t attr_handle = event->notify_rx.attr_handle;
        uint16_t data_len = OS_MBUF_PKTLEN(event->notify_rx.om);
        ESP_LOGI(TAG, "NOTIFY_RX: conn_handle=%d attr_handle=0x%x len=%d ind=%d",
                 conn_handle, attr_handle, data_len, event->notify_rx.indication);

        profile = get_profile_by_conn_id(conn_handle);

        if (s_notify_cb && profile) {
            uint8_t buf[512];
            if (data_len > sizeof(buf)) data_len = sizeof(buf);
            os_mbuf_copydata(event->notify_rx.om, 0, data_len, buf);
            s_notify_cb(profile->camera_index, buf, data_len);
        } else if (!s_notify_cb) {
            ESP_LOGW(TAG, "NOTIFY_RX: s_notify_cb is NULL!");
        } else if (!profile) {
            ESP_LOGW(TAG, "NOTIFY_RX: no profile for conn_handle=%d", conn_handle);
        }
        break;
    }

    /* ---------- MTU updated ---------- */
    case BLE_GAP_EVENT_MTU: {
        ESP_LOGI(TAG, "MTU update: conn_handle=%d mtu=%d",
                 event->mtu.conn_handle, event->mtu.value);
        break;
    }

    /* ---------- Advertising complete ---------- */
    case BLE_GAP_EVENT_ADV_COMPLETE: {
        s_advertising_active = false;
        ESP_LOGI(TAG, "Advertising completed (reason=%d)", event->adv_complete.reason);
        break;
    }

    /* ---------- Link established (post-connect sync) ---------- */
    case BLE_GAP_EVENT_LINK_ESTAB:
        ESP_LOGD(TAG, "Link established: conn_handle=%d status=%d",
                 event->link_estab.conn_handle, event->link_estab.status);
        break;

    /* ---------- Data length changed ---------- */
    case BLE_GAP_EVENT_DATA_LEN_CHG:
        ESP_LOGD(TAG, "Data length changed: conn_handle=%d",
                 event->data_len_chg.conn_handle);
        break;

    /* ---------- Connection parameters updated ---------- */
    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGD(TAG, "Connection parameters updated: conn_handle=%d status=%d",
                 event->conn_update.conn_handle, event->conn_update.status);
        break;

    /* ---------- Encryption change ---------- */
    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "Encryption change: conn_handle=%d status=%d",
                 event->enc_change.conn_handle, event->enc_change.status);
        break;

    /* ---------- L2CAP connection parameter update request from peripheral ---------- */
    case BLE_GAP_EVENT_L2CAP_UPDATE_REQ:
        ESP_LOGI(TAG, "L2CAP param update req: conn_handle=%d itvl_min=%d itvl_max=%d "
                 "latency=%d timeout=%d",
                 event->conn_update_req.conn_handle,
                 event->conn_update_req.peer_params->itvl_min,
                 event->conn_update_req.peer_params->itvl_max,
                 event->conn_update_req.peer_params->latency,
                 event->conn_update_req.peer_params->supervision_timeout);
        return 0;  /* Return 0 to accept the update request */

    default:
        ESP_LOGW(TAG, "Unhandled GAP event: type=%d", event->type);
        break;
    }
    return 0;
}

/* ----------------------------------------------------------------
 *  Initialization
 * ---------------------------------------------------------------- */

esp_err_t ble_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_ble_profiles();

    /* nimble_port_init() handles controller mem-release, init, and enable internally */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ble_hs_cfg.reset_cb          = on_reset;
    ble_hs_cfg.sync_cb           = on_sync;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;

    /* Disable security to match the original Bluedroid config which had no
       SMP/encryption.  DJI cameras operate without BLE-level encryption. */
    ble_hs_cfg.sm_io_cap         = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding        = 0;
    ble_hs_cfg.sm_mitm           = 0;
    ble_hs_cfg.sm_sc             = 0;
    ble_hs_cfg.sm_our_key_dist   = 0;
    ble_hs_cfg.sm_their_key_dist = 0;

    ble_att_set_preferred_mtu(500);

    /* Initialize mandatory BLE services before starting the host.
       ble_svc_gap_init()  registers GAP service (0x1800) with Device Name + Appearance.
       ble_svc_gatt_init() registers GATT service (0x1801) with Service Changed.
       Without these, the camera may reject the connection or refuse to send
       notifications if it performs reverse service discovery on our device. */
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("DJI-Remote");

    ble_gap_event_listener_register(&s_global_listener, global_event_listener, NULL);

    s_ble_sync_sem = xSemaphoreCreateBinary();
    nimble_port_freertos_init(nimble_host_task);

    /* Block until the NimBLE host is synced with the controller */
    xSemaphoreTake(s_ble_sync_sem, portMAX_DELAY);
    vSemaphoreDelete(s_ble_sync_sem);
    s_ble_sync_sem = NULL;

    ESP_LOGI(TAG, "ble_init success (NimBLE)!");
    return ESP_OK;
}

/* ----------------------------------------------------------------
 *  Scanning
 * ---------------------------------------------------------------- */

esp_err_t ble_start_scan(scan_mode_t mode, int target_slot, uint32_t timeout_ms) {
    if (mode == SCAN_MODE_PAIRING || mode == SCAN_MODE_SLOT_RECONNECT) {
        if (target_slot < 0 || target_slot >= BLE_MAX_CAMERAS) {
            ESP_LOGE(TAG, "Invalid target_slot %d for mode %d", target_slot, mode);
            return ESP_FAIL;
        }
    }

    s_scan_controller.mode = mode;
    s_scan_controller.target_slot = target_slot;
    s_scan_controller.scan_timeout_ms = timeout_ms;
    s_scan_controller.scan_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (mode != SCAN_MODE_AUTOCONNECT_BOOT) {
        for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
            s_scan_controller.autoconnect_pending[i] = false;
        }
    }

    ESP_LOGI(TAG, "Starting scan: mode=%d, target_slot=%d, timeout=%lu ms",
             mode, target_slot, timeout_ms);

    /* Disable duplicate filtering so we receive both ADV_IND (manufacturer
     * data for DJI detection) and SCAN_RSP (device name) as separate events.
     * Deduplication is handled at the application level (MAC-based). */
    struct ble_gap_disc_params disc_params = {
        .filter_duplicates = 0,
        .passive           = 0,
        .itvl              = 0x50,
        .window            = 0x30,
        .filter_policy     = 0,
        .limited           = 0,
    };

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC,
                           (int32_t)timeout_ms,
                           &disc_params,
                           gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start scan: %d", rc);
        s_scan_controller.mode = SCAN_MODE_IDLE;
        return ESP_FAIL;
    }

    s_ble_scan_active = true;
    ESP_LOGI(TAG, "Scan started (mode=%d, slot=%d)", mode, target_slot);
    return ESP_OK;
}

esp_err_t ble_stop_scan(void) {
    if (s_scan_controller.mode == SCAN_MODE_IDLE) {
        s_ble_scan_active = false;
        ESP_LOGI(TAG, "No active scan to stop");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping scan (mode was %d)", s_scan_controller.mode);
    ble_gap_disc_cancel();
    s_ble_scan_active = false;

    s_scan_controller.mode = SCAN_MODE_IDLE;
    s_scan_controller.target_slot = -1;
    s_scan_controller.scan_timeout_ms = 0;
    s_scan_controller.scan_start_time = 0;
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        s_scan_controller.autoconnect_pending[i] = false;
    }

    return ESP_OK;
}

scan_mode_t ble_get_scan_mode(void) {
    return s_scan_controller.mode;
}

void ble_set_autoconnect_pending(bool pending[BLE_MAX_CAMERAS]) {
    for (int i = 0; i < BLE_MAX_CAMERAS; i++) {
        s_scan_controller.autoconnect_pending[i] = pending[i];
        if (pending[i]) {
            ESP_LOGI(TAG, "Slot %d marked for autoconnect", i);
        }
    }
}

bool ble_is_scanning(void) {
    return s_ble_scan_active;
}

/* ----------------------------------------------------------------
 *  Connection management
 * ---------------------------------------------------------------- */

esp_err_t ble_start_scanning_and_connect(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_FAIL;
    }

    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    if (!p) {
        ESP_LOGE(TAG, "Failed to get profile for camera %d", camera_index);
        return ESP_FAIL;
    }

    bool has_target = false;
    for (int i = 0; i < 6; i++) {
        if (p->target_mac[i] != 0) { has_target = true; break; }
    }

    s_active_scan_camera_index = camera_index;
    scan_mode_t mode = has_target ? SCAN_MODE_SLOT_RECONNECT : SCAN_MODE_PAIRING;

    ESP_LOGI(TAG, "Scan started for camera %d (mode=%d)", camera_index, mode);
    return ble_start_scan(mode, camera_index, 30000);
}

static void try_to_connect(int camera_index, const uint8_t *addr) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return;
    }

    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    if (!p) {
        ESP_LOGE(TAG, "Failed to get profile for camera %d", camera_index);
        return;
    }

    if (s_connecting) {
        ESP_LOGW(TAG, "Already in connecting state, please wait...");
        return;
    }

    bool is_valid = false;
    for (int i = 0; i < 6; i++) {
        if (addr[i] != 0) { is_valid = true; break; }
    }
    if (!is_valid) {
        ESP_LOGE(TAG, "Invalid device address (all zeros) for camera %d", camera_index);
        return;
    }

    /* NimBLE cannot scan and connect simultaneously */
    if (s_ble_scan_active) {
        ble_gap_disc_cancel();
        s_ble_scan_active = false;
    }

    memcpy(p->remote_bda, addr, 6);
    s_connecting = true;

    ESP_LOGI(TAG, "Camera %d: Connecting to %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             camera_index, p->target_name,
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    ble_addr_t peer_addr;
    peer_addr.type = BLE_ADDR_PUBLIC;
    bda_reverse(addr, peer_addr.val);

    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &peer_addr, 30000,
                              NULL, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Camera %d: ble_gap_connect failed: %d", camera_index, rc);
        s_connecting = false;
    }
}

void ble_set_reconnecting(bool flag) {
    (void)flag;
    ESP_LOGW(TAG, "ble_set_reconnecting() is deprecated, use ble_start_scan() with appropriate mode");
}

bool ble_get_reconnecting(void) {
    return (s_scan_controller.mode == SCAN_MODE_SLOT_RECONNECT ||
            s_scan_controller.mode == SCAN_MODE_AUTOCONNECT_BOOT);
}

esp_err_t ble_reconnect(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_FAIL;
    }

    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    if (!p) {
        ESP_LOGE(TAG, "Failed to get profile for camera %d", camera_index);
        return ESP_FAIL;
    }

    bool is_valid = false;
    for (int i = 0; i < 6; i++) {
        if (p->target_mac[i] != 0) { is_valid = true; break; }
    }
    if (!is_valid) {
        ESP_LOGE(TAG, "Camera %d: No valid target device set", camera_index);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Camera %d: Reconnecting to %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             camera_index, p->target_name,
             p->target_mac[0], p->target_mac[1], p->target_mac[2],
             p->target_mac[3], p->target_mac[4], p->target_mac[5]);

    s_active_scan_camera_index = camera_index;
    return ble_start_scan(SCAN_MODE_SLOT_RECONNECT, camera_index, 30000);
}

esp_err_t ble_connect_direct(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "ble_connect_direct: Invalid camera index: %d", camera_index);
        return ESP_FAIL;
    }

    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    if (!p) {
        ESP_LOGE(TAG, "ble_connect_direct: Failed to get profile for camera %d", camera_index);
        return ESP_FAIL;
    }

    bool has_valid_mac = false;
    for (int i = 0; i < 6; i++) {
        if (p->target_mac[i] != 0) { has_valid_mac = true; break; }
    }
    if (!has_valid_mac) {
        ESP_LOGE(TAG, "ble_connect_direct: Camera %d has no valid target MAC", camera_index);
        return ESP_FAIL;
    }

    if (s_connecting) {
        ESP_LOGW(TAG, "ble_connect_direct: Already connecting, please wait...");
        return ESP_FAIL;
    }

    if (!s_ble_ready) {
        ESP_LOGE(TAG, "ble_connect_direct: NimBLE host not ready");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "AUTOCONNECT_BOOT direct connect for slot %d", camera_index);
    ESP_LOGI(TAG, "  Target: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             p->target_name,
             p->target_mac[0], p->target_mac[1], p->target_mac[2],
             p->target_mac[3], p->target_mac[4], p->target_mac[5]);

    memcpy(p->remote_bda, p->target_mac, 6);
    s_active_scan_camera_index = camera_index;
    s_connecting = true;

    /* NimBLE cannot scan and connect simultaneously */
    if (s_ble_scan_active) {
        ble_gap_disc_cancel();
        s_ble_scan_active = false;
    }

    ble_addr_t peer_addr;
    peer_addr.type = BLE_ADDR_PUBLIC;
    bda_reverse(p->target_mac, peer_addr.val);

    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &peer_addr, 30000,
                              NULL, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_connect_direct: ble_gap_connect failed for camera %d: %d",
                 camera_index, rc);
        s_connecting = false;
        s_active_scan_camera_index = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ble_connect_direct: connection initiated for camera %d", camera_index);
    return ESP_OK;
}

esp_err_t ble_disconnect(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_FAIL;
    }

    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    if (!p) {
        ESP_LOGE(TAG, "Failed to get profile for camera %d", camera_index);
        return ESP_FAIL;
    }

    if (p->connection_status.is_connected && p->conn_id != UINT16_MAX) {
        ESP_LOGI(TAG, "Disconnecting camera %d", camera_index);
        ble_gap_terminate(p->conn_id, BLE_ERR_REM_USER_CONN_TERM);
        p->connection_status.is_connected = false;
    } else {
        ESP_LOGW(TAG, "Camera %d is not connected", camera_index);
    }

    return ESP_OK;
}

/* ----------------------------------------------------------------
 *  Read / Write / Notify
 * ---------------------------------------------------------------- */

esp_err_t ble_read(uint16_t conn_id, uint16_t handle) {
    ble_profile_t* p = get_profile_by_conn_id(conn_id);
    if (!p || !p->connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected or invalid conn_id, skip read");
        return ESP_FAIL;
    }

    int rc = ble_gattc_read(conn_id, handle, on_read_complete, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Camera %d: read failed: %d", p->camera_index, rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_write_without_response(uint16_t conn_id, uint16_t handle,
                                     const uint8_t *data, size_t length)
{
    ble_profile_t* p = get_profile_by_conn_id(conn_id);
    if (!p || !p->connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected or invalid conn_id, skip write_without_response");
        return ESP_FAIL;
    }

    int rc = ble_gattc_write_no_rsp_flat(conn_id, handle, data, (uint16_t)length);
    if (rc != 0) {
        ESP_LOGE(TAG, "Camera %d: write_no_rsp failed: %d", p->camera_index, rc);
        if (rc == BLE_HS_ENOTCONN || rc == BLE_HS_EDISABLED) {
            p->connection_status.is_connected = false;
            ESP_LOGW(TAG, "Camera %d: Marking as disconnected due to BLE write failure",
                     p->camera_index);
        }
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_write_with_response(uint16_t conn_id, uint16_t handle,
                                  const uint8_t *data, size_t length)
{
    ble_profile_t* p = get_profile_by_conn_id(conn_id);
    if (!p || !p->connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected or invalid conn_id, skip write_with_response");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Camera %d: write_rsp conn=%d handle=0x%x len=%d",
             p->camera_index, conn_id, handle, (int)length);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, length > 64 ? 64 : length, ESP_LOG_INFO);

    int rc = ble_gattc_write_flat(conn_id, handle, data, (uint16_t)length,
                                   on_write_complete, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Camera %d: write failed: %d", p->camera_index, rc);
        if (rc == BLE_HS_ENOTCONN || rc == BLE_HS_EDISABLED) {
            p->connection_status.is_connected = false;
            ESP_LOGW(TAG, "Camera %d: Marking as disconnected due to BLE write failure",
                     p->camera_index);
        }
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_register_notify(uint16_t conn_id, uint16_t char_handle) {
    ble_profile_t* p = get_profile_by_conn_id(conn_id);
    if (!p || !p->connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected or invalid conn_id, skip register_notify");
        return ESP_FAIL;
    }

    if (p->cccd_handle == 0) {
        ESP_LOGE(TAG, "Camera %d: CCCD handle not discovered", p->camera_index);
        return ESP_FAIL;
    }

    uint8_t enable[] = {0x01, 0x00};
    ESP_LOGI(TAG, "Camera %d: Writing CCCD 0x%04x → [%02x %02x] (enable notify)",
             p->camera_index, p->cccd_handle, enable[0], enable[1]);
    int rc = ble_gattc_write_flat(conn_id, p->cccd_handle,
                                   enable, sizeof(enable),
                                   on_write_complete, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Camera %d: register_notify (CCCD write) failed: %d",
                 p->camera_index, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Camera %d: Notifications enabled (CCCD handle=0x%x)",
             p->camera_index, p->cccd_handle);
    return ESP_OK;
}

esp_err_t ble_unregister_notify(uint16_t conn_id, uint16_t char_handle) {
    ble_profile_t* p = get_profile_by_conn_id(conn_id);
    if (!p || !p->connection_status.is_connected || p->cccd_handle == 0) {
        ESP_LOGW(TAG, "Cannot unregister notify");
        return ESP_FAIL;
    }

    uint8_t disable[] = {0x00, 0x00};
    int rc = ble_gattc_write_flat(conn_id, p->cccd_handle,
                                   disable, sizeof(disable), NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Camera %d: unregister_notify failed: %d", p->camera_index, rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

void ble_set_notify_callback(ble_notify_callback_t cb) {
    s_notify_cb = cb;
}

void ble_set_state_callback(connect_logic_state_callback_t cb) {
    s_state_cb = cb;
}

/* ----------------------------------------------------------------
 *  Advertising (wake broadcast)
 * ---------------------------------------------------------------- */

static uint8_t adv_data[] = {
    10, 0xff, 'W','K','P','1','2','3','4','5','6'
};

static esp_timer_handle_t adv_timer = NULL;

static void stop_adv_after_3s(void* arg) {
    ble_gap_adv_stop();
    s_advertising_active = false;
    ESP_LOGI(TAG, "Wake broadcast advertising stopped after 3 seconds");
}

esp_err_t ble_stop_advertising_early(void) {
    if (!s_advertising_active) {
        ESP_LOGW(TAG, "Advertising not active, cannot stop early");
        return ESP_ERR_INVALID_STATE;
    }

    if (adv_timer != NULL) {
        esp_timer_stop(adv_timer);
    }

    int rc = ble_gap_adv_stop();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "Failed to stop advertising: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising stopped early");
    }

    s_advertising_active = false;
    return ESP_OK;
}

static esp_err_t start_adv_with_data(const uint8_t *data, size_t data_len) {
    /* NimBLE cannot scan and advertise simultaneously */
    if (s_ble_scan_active) {
        ble_gap_disc_cancel();
        s_ble_scan_active = false;
    }

    int rc = ble_gap_adv_set_data(data, (int)data_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set adv data: %d", rc);
        return ESP_FAIL;
    }

    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min  = 0x20,
        .itvl_max  = 0x40,
    };

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, 3000,
                            &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
        return ESP_FAIL;
    }

    s_advertising_active = true;

    if (adv_timer == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = &stop_adv_after_3s,
            .name = "adv_timer"
        };
        esp_timer_create(&timer_args, &adv_timer);
    }
    esp_timer_start_once(adv_timer, 3000000);

    return ESP_OK;
}

esp_err_t ble_start_advertising(void) {
    ble_profile_t* p = get_profile_by_camera_index(0);
    if (!p) {
        ESP_LOGE(TAG, "Error: Camera 0 profile not available!");
        return ESP_ERR_INVALID_STATE;
    }

    if (memcmp(p->remote_bda, "\x00\x00\x00\x00\x00\x00", 6) == 0) {
        ESP_LOGE(TAG, "Error: Camera 0 remote_bda not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < 6; i++) {
        adv_data[5 + i] = p->remote_bda[5 - i];
    }

    ESP_LOGI(TAG, "Modified Advertising Data (with MAC):");
    ESP_LOG_BUFFER_HEX(TAG, adv_data, sizeof(adv_data));

    esp_err_t ret = start_adv_with_data(adv_data, sizeof(adv_data));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Advertising started (will auto-stop after 3s)");
    }
    return ret;
}

esp_err_t ble_wake_camera(const uint8_t* camera_mac) {
    if (!camera_mac) {
        ESP_LOGE(TAG, "Camera MAC address is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Starting wake broadcast for camera: %02X:%02X:%02X:%02X:%02X:%02X",
             camera_mac[0], camera_mac[1], camera_mac[2],
             camera_mac[3], camera_mac[4], camera_mac[5]);

    static uint8_t wake_adv_data[] = {
        10, 0xff, 'W','K','P','1','2','3','4','5','6'
    };

    for (int i = 0; i < 6; i++) {
        wake_adv_data[5 + i] = camera_mac[5 - i];
    }

    ESP_LOGI(TAG, "Wake broadcast data: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             wake_adv_data[0], wake_adv_data[1], wake_adv_data[2], wake_adv_data[3],
             wake_adv_data[4], wake_adv_data[5], wake_adv_data[6], wake_adv_data[7],
             wake_adv_data[8], wake_adv_data[9], wake_adv_data[10]);

    esp_err_t ret = start_adv_with_data(wake_adv_data, sizeof(wake_adv_data));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Wake advertising started (will auto-stop after 3s)");
    }
    return ret;
}

/* ----------------------------------------------------------------
 *  Info getters
 * ---------------------------------------------------------------- */

const char* ble_get_connected_device_name(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) return NULL;
    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    if (p && p->connection_status.is_connected) return p->target_name;
    return NULL;
}

const uint8_t* ble_get_connected_device_mac(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) return NULL;
    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    if (p && p->connection_status.is_connected) return p->remote_bda;
    return NULL;
}

bool ble_get_connected_device_info(int camera_index, char* name, size_t name_size, uint8_t* mac) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS || !name || !mac) return false;
    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    if (!p || !p->connection_status.is_connected) return false;

    strncpy(name, p->target_name, name_size - 1);
    name[name_size - 1] = '\0';
    memcpy(mac, p->remote_bda, 6);
    return true;
}

uint16_t ble_get_conn_id(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) return 0;
    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    return (p && p->conn_id != UINT16_MAX) ? p->conn_id : 0;
}

uint16_t ble_get_write_handle(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) return 0;
    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    return p ? p->write_char_handle : 0;
}

uint16_t ble_get_notify_handle(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) return 0;
    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    return p ? p->notify_char_handle : 0;
}

uint16_t ble_get_cccd_handle(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) return 0;
    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    return p ? p->cccd_handle : 0;
}

bool ble_is_camera_connected(int camera_index) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) return false;
    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    return p ? p->connection_status.is_connected : false;
}

/* ----------------------------------------------------------------
 *  Target device configuration
 * ---------------------------------------------------------------- */

void ble_set_target_device(int camera_index, const char* name, const uint8_t* mac) {
    if (camera_index < 0 || camera_index >= BLE_MAX_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return;
    }
    if (!name || !mac) {
        ESP_LOGE(TAG, "Invalid name or MAC address");
        return;
    }

    ble_profile_t* p = get_profile_by_camera_index(camera_index);
    if (!p) {
        ESP_LOGE(TAG, "Failed to get profile for camera %d", camera_index);
        return;
    }

    strncpy(p->target_name, name, sizeof(p->target_name) - 1);
    p->target_name[sizeof(p->target_name) - 1] = '\0';
    memcpy(p->target_mac, mac, 6);
    p->camera_index = camera_index;

    p->connection_status.is_connected = false;
    p->conn_id = UINT16_MAX;
    p->notify_char_handle = 0;
    p->write_char_handle = 0;
    p->cccd_handle = 0;
    p->handle_discovery.notify_char_handle_found = false;
    p->handle_discovery.write_char_handle_found = false;

    ESP_LOGI(TAG, "Camera %d target device set: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             camera_index, name,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/*
 * ble_server.c - Phase 2: NimBLE GATT server
 *
 * Custom service layout (128-bit UUIDs, base 4D434100-0000-1000-8000-00805F9B34FB):
 *   MyCaddie Service      (0x...00)
 *     Control chr  (write)  — 2 bytes: int8 throttle, int8 steering → motor_ctrl_post()
 *     E-stop  chr  (write)  — any write → motor_stop_all() immediately
 *     Telemetry chr (notify) — 3 bytes: uint16 battery_mv, uint8 flags
 *
 * Safety: BLE disconnect → motor_stop_all() before restarting advertising.
 */

#include "ble_server.h"
#include "motor_driver.h"
#include "sdkconfig.h"

#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_server";

#define DEVICE_NAME "MyCaddie"

/* ---------------------------------------------------------------------------
 * UUIDs  (128-bit, little-endian byte order as required by NimBLE)
 * Human-readable form:
 *   Service:   4D434100-0000-1000-8000-00805F9B34FB
 *   Control:   4D434101-0000-1000-8000-00805F9B34FB
 *   E-stop:    4D434102-0000-1000-8000-00805F9B34FB
 *   Telemetry: 4D434103-0000-1000-8000-00805F9B34FB
 * --------------------------------------------------------------------------- */
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0xFB,0x34,0x9B,0x5F,0x80,0x00,0x00,0x80,
                     0x00,0x10,0x00,0x00,0x00,0x41,0x43,0x4D);

static const ble_uuid128_t ctrl_chr_uuid =
    BLE_UUID128_INIT(0xFB,0x34,0x9B,0x5F,0x80,0x00,0x00,0x80,
                     0x00,0x10,0x00,0x00,0x01,0x41,0x43,0x4D);

static const ble_uuid128_t estop_chr_uuid =
    BLE_UUID128_INIT(0xFB,0x34,0x9B,0x5F,0x80,0x00,0x00,0x80,
                     0x00,0x10,0x00,0x00,0x02,0x41,0x43,0x4D);

static const ble_uuid128_t telem_chr_uuid =
    BLE_UUID128_INIT(0xFB,0x34,0x9B,0x5F,0x80,0x00,0x00,0x80,
                     0x00,0x10,0x00,0x00,0x03,0x41,0x43,0x4D);

/* ---------------------------------------------------------------------------
 * Runtime state
 * --------------------------------------------------------------------------- */
static bool     s_connected        = false;
static uint16_t s_conn_handle      = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_telem_val_handle = 0;
static uint8_t  s_own_addr_type;

/* ---------------------------------------------------------------------------
 * GATT access callbacks
 * --------------------------------------------------------------------------- */
static int chr_control_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 2) {
        ESP_LOGW(TAG, "Control write too short (%d bytes)", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t buf[2];
    ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), NULL);
    int8_t throttle = (int8_t)buf[0];
    int8_t steering = (int8_t)buf[1];
    motor_ctrl_post(throttle, steering);
    return 0;
}

static int chr_estop_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    ESP_LOGW(TAG, "Emergency stop received");
    motor_stop_all();
    return 0;
}

static int chr_telem_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* Telemetry is notify-only; reads are not expected but must not crash */
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

/* ---------------------------------------------------------------------------
 * GATT service table
 * --------------------------------------------------------------------------- */
static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {   /* Control: throttle + steering */
                .uuid       = &ctrl_chr_uuid.u,
                .access_cb  = chr_control_cb,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {   /* Emergency stop */
                .uuid       = &estop_chr_uuid.u,
                .access_cb  = chr_estop_cb,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {   /* Telemetry: battery + status (notify) */
                .uuid        = &telem_chr_uuid.u,
                .access_cb   = chr_telem_cb,
                .val_handle  = &s_telem_val_handle,
                .flags       = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }   /* end of characteristics */
        },
    },
    { 0 }   /* end of services */
};

/* Forward declaration — defined below */
static int ble_server_gap_event(struct ble_gap_event *event, void *arg);

/* ---------------------------------------------------------------------------
 * Advertising
 * --------------------------------------------------------------------------- */
static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags                = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name                 = (const uint8_t *)DEVICE_NAME;
    fields.name_len             = sizeof(DEVICE_NAME) - 1;
    fields.name_is_complete     = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields error: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_server_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start error: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as \"%s\"", DEVICE_NAME);
    }
}

/* ---------------------------------------------------------------------------
 * GAP event handler
 * --------------------------------------------------------------------------- */
static int ble_server_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected   = true;
            ESP_LOGI(TAG, "Connected (handle %d)", s_conn_handle);
        } else {
            ESP_LOGW(TAG, "Connection failed, restarting advertising");
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected (reason 0x%02x) — stopping motors",
                 event->disconnect.reason);
        s_connected   = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        motor_stop_all();
        start_advertising();
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update: conn %d mtu %d",
                 event->mtu.conn_handle, event->mtu.value);
        break;

    default:
        break;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * NimBLE host sync / reset callbacks
 * --------------------------------------------------------------------------- */
static void on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset (reason %d)", reason);
    s_connected = false;
    motor_stop_all();
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    assert(rc == 0);
    start_advertising();
}

/* ---------------------------------------------------------------------------
 * NimBLE host task (runs on Core 0 per sdkconfig)
 * --------------------------------------------------------------------------- */
static void ble_host_task(void *param)
{
    nimble_port_run();          /* blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
void ble_server_init(void)
{
    nimble_port_init();

    ble_hs_cfg.reset_cb          = on_reset;
    ble_hs_cfg.sync_cb           = on_sync;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    assert(rc == 0);

    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    assert(rc == 0);

    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "BLE server initialised");
}

bool ble_server_is_connected(void)
{
    return s_connected;
}

bool ble_server_get_rssi(int8_t *rssi_dbm)
{
    if (!s_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return false;
    if (!rssi_dbm) return false;
    return ble_gap_conn_rssi(s_conn_handle, rssi_dbm) == 0;
}

void ble_server_notify_telemetry(uint16_t battery_mv, bool motors_active)
{
    if (!s_connected || s_telem_val_handle == 0) return;

    uint8_t buf[3];
    buf[0] = (uint8_t)(battery_mv & 0xFF);
    buf[1] = (uint8_t)(battery_mv >> 8);
    buf[2] = motors_active ? 0x01 : 0x00;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (!om) return;

    int rc = ble_gatts_notify_custom(s_conn_handle, s_telem_val_handle, om);
    if (rc != 0) {
        ESP_LOGD(TAG, "Telemetry notify failed: %d", rc);
    }
}

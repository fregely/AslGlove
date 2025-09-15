#include "nimble_gatt.h"
#include "nimble_gap.h"
#include "esp_nimble_hci.h"
#include "nimble_common.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"



/* === External functions in your gap module (implemented in gap.c) ===
   We call gap_init() to set device name/address and adv_init() to start advertising.
   Your gap.c already exposes gap_init() and adv_init() in your last post.
*/
extern int gap_init(void);
extern void adv_init(void);

/* Forward declarations for callbacks */
static int cmd_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg);
static int time_rw_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static int led_rw_cb(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg);

/* NimBLE reset/sync callbacks */
static void bleprph_on_reset(int reason);
static void bleprph_on_sync(void);

/* IMU notify state (updated from subscription callback) */
static uint16_t imu_data_handle = 0;         // filled by GATT stack
static uint16_t time_sync_data_handle = 0;
static uint16_t LED_state_data_handle = 0;
static uint16_t imu_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool imu_notify_enabled = false;

/* UUIDs */
static const ble_uuid128_t IMU_SERVICE_UUID =
    BLE_UUID128_INIT(0x99,0x88,0x77,0x66,0x55,0x44,0x33,0x22,0x11,0x00,0xaa,0xbb,0xcc,0xdd,0xee,0xff);

static const ble_uuid128_t IMU_DATA_UUID =
    BLE_UUID128_INIT(0xc4,0xe7,0xa1,0x80,0x7b,0x2f,0x4c,0x95,0xbf,0xc5,0x1d,0x5c,0x62,0x12,0x34,0x56);

static const ble_uuid128_t TIME_SYNC_UUID =
    BLE_UUID128_INIT(0xab,0xcd,0xef,0x12,0x34,0x56,0x78,0x90,0xab,0xcd,0xef,0x12,0x34,0x56,0x78,0x90);

static const ble_uuid128_t LED_STATE_UUID =
    BLE_UUID128_INIT(0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef);

/* GATT table */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &IMU_SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &IMU_DATA_UUID.u,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &imu_data_handle,
            },
            {
                .uuid = &TIME_SYNC_UUID.u,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = time_rw_cb,
                .val_handle = &time_sync_data_handle,
            },
            {
                .uuid = &LED_STATE_UUID.u,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = led_rw_cb,
                .val_handle = &LED_state_data_handle,
            },
            { 0 } // terminator
        },
    },
    { 0 } // terminator
};

/* --------- Access Callbacks (same as you had) ---------- */
static int cmd_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int len = OS_MBUF_PKTLEN(ctxt->om);
    uint8_t buf[64];
    ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), NULL);
    ESP_LOGI(TAG, "START_BLE written: %d bytes, first byte=0x%02X", len, buf[0]);

    // If first byte is 1 => start streaming; 0 => stop (example approach)
    if (len > 0) {
        if (buf[0] == 1) {
            ESP_LOGI(TAG, "Start IMU streaming requested (via START_BLE)");
            // the IMU task should check imu_notify_enabled and imu_conn_handle
            // you can set a flag or notify a task here if desired
        } else {
            ESP_LOGI(TAG, "Stop IMU streaming requested (via START_BLE)");
        }
    }

    return 0;
}

static int time_rw_cb(uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
int rc;

if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
uint8_t buf[8]; // Example: 64-bit timestamp
rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), NULL);
if (rc == 0) {
ESP_LOGI(TAG, "Time sync write: %d bytes, first=%02x", sizeof(buf), buf[0]);
// TODO: parse/store timestamp here
} else {
return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
}
} else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
uint8_t response[4] = {0, 1, 2, 3}; // Example payload
rc = os_mbuf_append(ctxt->om, response, sizeof(response));
if (rc != 0) {
return BLE_ATT_ERR_INSUFFICIENT_RES;
}
}

return 0;
}


static int led_rw_cb(uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
static uint8_t led_state = 0;
int rc;

if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
rc = ble_hs_mbuf_to_flat(ctxt->om, &led_state, sizeof(led_state), NULL);
if (rc != 0) {
return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
}
ESP_LOGI(TAG, "LED write: %d", led_state);

// TODO: gpio_set_level(GPIO_NUM_X, led_state);

} else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
rc = os_mbuf_append(ctxt->om, &led_state, sizeof(led_state));
if (rc != 0) {
return BLE_ATT_ERR_INSUFFICIENT_RES;
}
}

return 0;
}


/* Called by gap.c when a subscribe event occurs.
   gap.c already calls gatt_svr_subscribe_cb(event) in its gap_event_handler().
*/
void gatt_svr_subscribe_cb(struct ble_gap_event *event)
{
    ESP_LOGI(TAG, "gatt_svr_subscribe_cb: attr_handle=%d conn=%d cur_notify=%d cur_ind=%d",
             event->subscribe.attr_handle,
             event->subscribe.conn_handle,
             event->subscribe.cur_notify,
             event->subscribe.cur_indicate);

    if (event->subscribe.attr_handle == imu_data_handle) {
        imu_conn_handle = event->subscribe.conn_handle;
        imu_notify_enabled = event->subscribe.cur_notify;
        ESP_LOGI(TAG, "IMU notify %s on conn %d",
                 imu_notify_enabled ? "enabled" : "disabled",
                 imu_conn_handle);
    }
}

/* Register GATT services */
int gatt_svr_init(void)
{
    int rc;        
    ESP_LOGI(TAG, "in gatt_svr_init");
    
    /* Initialize base GATT service first */
    ble_svc_gatt_init();
    
    /* Count and add custom services */
    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0){
        ESP_LOGE(TAG, "failed ble_gatts_count_cfg, error: %d", rc);
        return rc;
    }
    
    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_LOGI(TAG, "ble_gatts_add_svcs result: %d", rc);
    
    return rc;
}

/* ble_hs reset callback */
static void bleprph_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset; reason=%d", reason);
}

/* ble_hs sync callback
   - call gap_init() (sets device name/address)
   - call adv_init() (gap module will call start_advertising())
*/
static void bleprph_on_sync(void)
{
    ESP_LOGI(TAG, "BLE on_sync: stack ready");

    int rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "gap_init failed: %d", rc);
        return;
    }

    /* init GATT services (must be done after ble_hs is up) */
    rc = gatt_svr_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "gatt_svr_init failed: %d", rc);
        return;
    }

    /* now ask the gap module to start advertising */
    adv_init();
}

/* Host task that runs nimble loop */
void ble_hs_task(void *param)
{
    ESP_LOGI(TAG, "BLE host thread starting");
    nimble_port_run();
    vTaskDelete(NULL);

}

// /* Public startup helper: call from app_main (or create as a task) */
void nimble_start(void)
{
    esp_err_t err;
    
    ESP_LOGI(TAG, "Starting BLE initialization...");
    
    /* initialize NVS */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized");
    
    /* Release classic BT memory (optional but recommended) */
    err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    ESP_LOGI(TAG, "Classic BT memory release: %s", esp_err_to_name(err));
    
    /* Initialize NimBLE port - this handles controller init internally */
    err = nimble_port_init();
    ESP_LOGI(TAG, "NimBLE port init result: %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE port init failed - stopping here");
        return;
    }
    
    /* Set callbacks */
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb  = bleprph_on_sync;
    ESP_LOGI(TAG, "Callbacks set");
    
    /* Start NimBLE host task */
    nimble_port_freertos_init(ble_hs_task);
    ESP_LOGI(TAG, "NimBLE host task started");
    
    ESP_LOGI(TAG, "NimBLE initialization completed successfully");
}

/* Utility: send IMU notification if subscribed. Use from your IMU task. */
int gatt_send_imu_notification(const uint8_t *data, size_t len)
{
    if (!imu_notify_enabled || imu_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return BLE_HS_ENOTCONN;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat((const void *)data, len);
    if (!om) return BLE_HS_ENOMEM;

    int rc = ble_gattc_notify_custom(imu_conn_handle, imu_data_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gattc_notify_custom rc=%d", rc);
    }
    return rc;
}

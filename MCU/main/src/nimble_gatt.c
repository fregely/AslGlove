#include "nimble_gatt.h"
#include "nimble_gap.h"
#include "esp_nimble_hci.h"
#include "nimble_common.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

extern int gap_init(void);
extern void adv_init(void);

/* Forward declarations for callbacks */
static int cmd_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg);
static int time_rw_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static int led_rw_cb(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg);

/* IMU notify state (updated from subscription callback) */
static uint16_t imu_data_handle = 0;         // filled by GATT stack
static uint16_t time_sync_data_handle = 0;
static uint16_t LED_state_data_handle = 0;
static uint16_t imu_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool imu_notify_enabled = false;

/* UUIDs */
static const ble_uuid128_t IMU_SERVICE_UUID =
    BLE_UUID128_INIT(0xff,0xee,0xdd,0xcc,0xbb,0xaa,0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99);
static const ble_uuid128_t IMU_DATA_UUID =
    BLE_UUID128_INIT(0x56,0x34,0x12,0x62,0x5c,0x1d,0xc5,0xbf,0x95,0x4c,0x2f,0x7b,0x80,0xa1,0xe7,0xc4);
static const ble_uuid128_t TIME_SYNC_UUID =
    BLE_UUID128_INIT(0x90,0x78,0x56,0x34,0x12,0xef,0xcd,0xab,0x90,0x78,0x56,0x34,0x12,0xef,0xcd,0xab);
static const ble_uuid128_t LED_STATE_UUID =
    BLE_UUID128_INIT(0xef,0xcd,0xab,0x89,0x67,0x45,0x23,0x01,0xef,0xcd,0xab,0x89,0x67,0x45,0x23,0x01);
/* GATT table */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &IMU_SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &IMU_DATA_UUID.u,
                .access_cb = cmd_write_cb,
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
/* Add standard services alongside your custom service */


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

/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
 void gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op) {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}


void gatt_subscribe_cb(struct ble_gap_event *event)
{
    /* Check connection handle */
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle);
    } else {
        ESP_LOGI(TAG, "subscribe by nimble stack; attr_handle=%d",
                 event->subscribe.attr_handle);
    }


    if (event->subscribe.attr_handle == imu_data_handle) {
        imu_conn_handle = event->subscribe.conn_handle;
        imu_notify_enabled = event->subscribe.cur_notify;
        ESP_LOGI(TAG, "IMU notify %s on conn %d",
                 imu_notify_enabled ? "enabled" : "disabled",
                 imu_conn_handle);
    }
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
 int gatt_init(void) {
    /* Local variables */
    int rc;
    ESP_LOGI(TAG, "Kill youself 1");
    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    ESP_LOGI(TAG, "Kill youself 2");

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        
        ESP_LOGI(TAG, "Kill youself 3");
        return rc;
    }
    ESP_LOGI(TAG, "Kill youself 4");


    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_LOGI(TAG, "Kill youself 5");

    if (rc != 0) {
        ESP_LOGI(TAG, "Kill youself 6");

        return rc;
    }
    ESP_LOGI(TAG, "Kill youself ");


    return 0;
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

/* Add this task function to your code */
void imu_data_task(void *param) {
    uint8_t dummy_data[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06}; // Example IMU data
    int counter = 0;
    
    while (1) {
        // Only send if someone is subscribed
        if (imu_notify_enabled && imu_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            // Increment counter to show changing data
            dummy_data[0] = counter++;
            
            int rc = gatt_send_imu_notification(dummy_data, sizeof(dummy_data));
            if (rc == 0) {
                ESP_LOGI(TAG, "Sent IMU notification: %d", counter-1);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Send every 1 second
    }
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// UUIDs for your services/characteristics
static const ble_uuid128_t IMU_DATA_UUID =
    BLE_UUID128_INIT(0xc4,0xe7,0xa1,0x80,0x7b,0x2f,0x4c,0x95,0xbf,0xc5,0x1d,0x5c,0x62,0x12,0x34,0x56);

static const ble_uuid128_t START_BLE_UUID =
    BLE_UUID128_INIT(0x1a,0x2b,0x3c,0x4d,0x11,0x11,0x22,0x22,0x33,0x33,0x44,0x44,0x55,0x55,0x66,0x66);

static const ble_uuid128_t TIME_SYNC_UUID =
    BLE_UUID128_INIT(0xab,0xcd,0xef,0x12,0x34,0x56,0x78,0x90,0xab,0xcd,0xef,0x12,0x34,0x56,0x78,0x90);

static const ble_uuid128_t LED_STATE_UUID =
    BLE_UUID128_INIT(0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef);



static const char *TAG = "NIMBLE_BLE";

// Forward declare
static int cmd_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg);
static int time_rw_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static int led_rw_cb(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg);

// Handle for notifications
static uint16_t imu_data_handle;

// ------------------ GATT Table ------------------
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(0x99,0x88,0x77,0x66,0x55,0x44,0x33,0x22,0x11,0x00,0xaa,0xbb,0xcc,0xdd,0xee,0xff),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &IMU_DATA_UUID.u,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &imu_data_handle, //just will continusily send this data, doesn't need to be called like the other callbacks
            },
            {
                .uuid = &START_BLE_UUID.u,
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = cmd_write_cb,
            },
            {
                .uuid = &TIME_SYNC_UUID.u,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = time_rw_cb,
            },
            {
                .uuid = &LED_STATE_UUID.u,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = led_rw_cb,
            },
            {0} // end
        },
    },
    {0}
};

// ------------------ Callbacks ------------------
static int cmd_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int len = OS_MBUF_PKTLEN(ctxt->om); // Gets the length of the message 
    uint8_t buf[64];  // adjust size to what you expect
    ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), NULL); // puts the message together
    ESP_LOGI(TAG, "Received %d bytes, first byte: 0x%02X", len, buf[0]); //send the message to log
    return 0;
}

static int time_rw_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        ESP_LOGI(TAG, "Time sync written");
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t response[4] = {0,1,2,3}; //place holder, will be current time or sync token of somesort 
        os_mbuf_append(ctxt->om, response, sizeof(response));
    }
    return 0;
}

static int led_rw_cb(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        ESP_LOGI(TAG, "LED state written");
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t led_state = 1;
        os_mbuf_append(ctxt->om, &led_state, 1);
    }
    return 0;
}

// ------------------ Generic Access Profile Event ------------------
static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connected: status=%d", event->connect.status);
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected");
        ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, NULL, gap_event_cb, NULL);
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete, restarting...");
        ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, NULL, gap_event_cb, NULL);
        break;
    default:
        break;
    }
    return 0;
}

// ------------------ GATT Init ------------------
static void gatt_svr_init(void) {
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
}

// ------------------ BLE Task ------------------


void ble_hs_task(void *param) {
    ESP_LOGI(TAG, "BLE host thread starting");

    // Init GATT
    gatt_svr_init();

    // Start advertising
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL);

    nimble_port_run(); // This blocks
}

void nimble_ble_task(void *pvParameters) {
    nimble_port_freertos_init(ble_hs_task); // Start host thread
}

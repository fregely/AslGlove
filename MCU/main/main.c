#include "nimble_common.h"
#include "nimble_gatt.h"
#include "nimble_gap.h"

/* Library function declarations */
void ble_store_config_init(void);


/* Private function declarations */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_config_init(void);
static void nimble_host_task(void *param);

/* Private functions */
/*
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 */
 static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    adv_init();
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

void app_main(void) {
    // /* Local variables */
    // int rc;
    // esp_err_t ret;
    // ESP_LOGI("MAIN", "Hello, world! The app is running.");

    // /*
    //  * NVS flash initialization
    //  * Dependency of BLE stack to store configurations
    //  */
    // ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
    //     ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
    //     return;
    // }

    // /* NimBLE stack initialization */
    // ret = nimble_port_init();
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
    //              ret);
    //     return;
    // }

    // /* GAP service initialization */
    // rc = gap_init();
    // if (rc != 0) {
    //     ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", rc);
    //     return;
    // }

    // // /* GATT server initialization */
    // // rc = gatt_svr_init();
    // // if (rc != 0) {
    // //     ESP_LOGE(TAG, "failed to initialize GATT server, error code: %d", rc);
    // //     return;
    // // }

    // /* NimBLE host configuration initialization */
    // nimble_host_config_init();

    // /* Start NimBLE host task thread and return */
    // xTaskCreate(nimble_host_task, "NimBLE Host", 4*1024, NULL, 5, NULL);
    // return;
    
    ESP_LOGI("MAIN", "Hello, world! The app is running.");
    nimble_start();  // This handles everything
    
}

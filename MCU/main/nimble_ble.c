#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "nimble/ble.h"
#include "nimble_ble.h"

// Tag for logging
static const char *TAG = "NIMBLE BLE";

// Callback when NimBLE stack syncs
static void ble_app_on_sync(void) {
    ESP_LOGI(TAG, "NimBLE stack synced, ready to advertise!");
    // TODO: start advertising or init GATT server here
}

// BLE host task (needed for NimBLE)
static void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();   // This blocks until NimBLE shutdown
    nimble_port_freertos_deinit();
}


/ Public FreeRTOS task you launch from main.c
void nimble_ble_task(void *pvParameters) {
    ESP_LOGI(TAG, "Starting NimBLE init...");

    // Init NimBLE
    esp_nimble_hci_and_controller_init();
    nimble_port_init();

    // Set sync callback
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    // Start BLE host task
    nimble_port_freertos_init(ble_host_task);

    // FreeRTOS task loop (optional for periodic work)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "BLE running...");
    }
}  

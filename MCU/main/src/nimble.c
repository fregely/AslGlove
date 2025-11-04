#include "nimble.h"
#include "nimble_common.h"
#include "nimble_gatt.h"
#include "nimble_gap.h"
#include "imu_packet.h"


/* Library function declarations */
void ble_store_config_init(void);

/* Private function declarations */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_config_init(void);
static void nimble_host_task(void *param);
static void ble_data_task(void *param);


/* Statistics tracking */
static uint32_t packets_sent = 0;
static uint32_t packets_failed = 0;
static uint32_t last_log_time = 0;

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
    ble_hs_cfg.gatts_register_cb = gatt_register_cb;
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

static void ble_data_task(void *param) {
    imu_packet_t packet;
    int rc;
    
    // Wait for queue to be available
    while (imu_data_queue == NULL) {
        ESP_LOGW(TAG, "Waiting for IMU queue to be created...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "BLE data task started, queue available");
    ESP_LOGI(TAG, "Packet size: %d bytes", sizeof(imu_packet_t));
    
    while (1) {
        // Wait for data from the IMU task (block indefinitely)
        if (xQueueReceive(imu_data_queue, &packet, portMAX_DELAY) == pdTRUE) {
            
            // Send the packet via BLE
            rc = gatt_send_notification((const uint8_t *)&packet, sizeof(packet));
            
            if (rc != 0) {
                packets_failed++;
                ESP_LOGD(TAG, "Failed to send packet (rc=%d)", rc);
            } else {
                packets_sent++;
                ESP_LOGV(TAG, "Sent channel %d @ %llu us", packet.channel, packet.timestamp_us);
            }
            
            // Periodic statistics logging (every 5 seconds)
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_log_time >= 5000) {
                float success_rate = 100.0f * packets_sent / (packets_sent + packets_failed);
                ESP_LOGI(TAG, "BLE Stats: Sent=%lu, Failed=%lu, Success=%.1f%%", 
                         packets_sent, packets_failed, success_rate);
                
                // Calculate data rate
                uint32_t elapsed_sec = (now - last_log_time) / 1000;
                if (elapsed_sec > 0) {
                    uint32_t packets_per_sec = packets_sent / elapsed_sec;
                    uint32_t bytes_per_sec = packets_per_sec * sizeof(imu_packet_t);
                    ESP_LOGI(TAG, "Data rate: %lu packets/sec, %lu bytes/sec (%.2f KB/s)",
                             packets_per_sec, bytes_per_sec, bytes_per_sec / 1024.0f);
                }
                
                // Reset counters
                packets_sent = 0;
                packets_failed = 0;
                last_log_time = now;
            }
        }
    }
}

void start_ble_task(void) {
    /* Local variables */
    int rc;
    esp_err_t ret;
    /*
     * NVS flash initialization
     * Dependency of BLE stack to store configurations
     */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return;
    }

    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return;
    }

    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", rc);
        return;
    }

    /* GATT server initialization */
    rc = gatt_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GATT server, error code: %d", rc);
        return;
    }

    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    /* Start NimBLE host task thread and return */
    xTaskCreate(nimble_host_task, "NimBLE Host", 4*1024, NULL, 5, NULL);
    xTaskCreate(ble_data_task, "BLE Sender", 3072, NULL, 4, NULL);
    return;
}


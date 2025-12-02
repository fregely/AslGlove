#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "I2C_SCAN"

// 🔧 CHANGE THESE TO MATCH YOUR WIRING
#define I2C_MASTER_SCL_IO          5     // e.g. GPIO4
#define I2C_MASTER_SDA_IO          4     // e.g. GPIO5
#define I2C_MASTER_PORT            I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         400000
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define I2C_MASTER_TIMEOUT_MS      1000

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0, // optional, for newer ESP-IDF
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(
        I2C_MASTER_PORT,
        conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE,
        0
    ));

    ESP_LOGI(TAG, "I2C master initialized (SDA=%d, SCL=%d)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    return ESP_OK;
}

static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Starting I2C scan...");

    for (uint8_t addr = 1; addr < 127; addr++) {
        // i2c_master_probe is available in newer ESP-IDF.
        // If your version doesn’t have it, we can build manual transactions instead.
        esp_err_t ret = i2c_master_probe(I2C_MASTER_PORT, addr, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

        if (ret == ESP_OK) {
            printf("✅ I2C device found at address 0x%02X\n", addr);
        } else if (ret != ESP_ERR_TIMEOUT) {
            // For debugging: show other errors too
            // printf("Addr 0x%02X: error 0x%x\n", addr, ret);
        }
    }

    ESP_LOGI(TAG, "I2C scan finished.");
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    i2c_scan();

    // Keep app running so you can still see logs in monitor
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
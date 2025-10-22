#include "imu_i2c.h"
#include "TCA9548A.h"
#include "ICM20948.h"
#include "common.h"
#include "driver/i2c_master.h"
#include "imu_packet.h"


static i2c_master_bus_handle_t i2c_bus;
QueueHandle_t imu_data_queue = NULL;


static void i2c_master_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus)); // Declares the MCU as I2C master
    ESP_ERROR_CHECK(tca9548a_init(i2c_bus, I2C_MASTER_FREQ_HZ)); // Initialize TCA9548A multiplexer as a buss
    test_multiplexer_detection();
    ESP_ERROR_CHECK(imu_device_init(i2c_bus, I2C_MASTER_FREQ_HZ)); // Initialize ICM20948 IMU device

    test_multiplexer_detection();
}

static void imu_task(void *param) {
    // Wake up all IMUs
    for(uint8_t ch = 0; ch < MAX_IMUS; ch++) {
        if (imu_write_reg(ch, ICM20948_PWR_MGMT_1, 0x01) == ESP_OK) {
            ESP_LOGI(TAG, "IMU on channel %d woken up", ch);
        }
        if (imu_write_reg(ch, ICM20948_PWR_MGMT_2, 0x00) == ESP_OK) {
            ESP_LOGI(TAG, "IMU on channel %d sensors enabled", ch);
        }
    }
    
    while (1) {
        // Read data from each IMU channel
        for(uint8_t ch = 0; ch < MAX_IMUS; ch++) {
            imu_packet_t packet;
            packet.timestamp_us = esp_timer_get_time();
            packet.channel = ch;
            if (imu_data_get(ch, packet.raw_data) == ESP_OK) {
                ESP_LOGD(TAG, "Getting IMU Raw data from IMU: %d", ch);
                // Send the packet to the queue
                if (xQueueSend(imu_data_queue, &packet, pdMS_TO_TICKS(100)) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to send IMU data to queue");
                }
                        vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                ESP_LOGD(TAG, "Failed to read data from IMU on channel %d", ch);
            }
             // Small delay between IMU reads to avoid bus congestion

        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust based on desired sampling rate
    }
}


void start_imu_task(void) {
    i2c_master_init();
    
    // Create the queue
    imu_data_queue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(imu_packet_t));
    if (imu_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU data queue");
        return;
    }

    xTaskCreate(imu_task, "imu_task", 2048, NULL, 5, NULL);
}

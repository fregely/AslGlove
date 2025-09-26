#include "imu_i2c.h"
#include "TCA9548A.h"
#include "ICM20948.h"
#include "common.h"
#include "driver/i2c_master.h"


static i2c_master_bus_handle_t i2c_bus;



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
    uint8_t whoami = 0;
    
        
    // Wake up all IMUs
    for(uint8_t ch = 0; ch < MAX_IMUS; ch++) {
        if (imu_write_reg(ch, ICM20948_PWR_MGMT_1, 0x01) == ESP_OK) {
            ESP_LOGI(TAG, "IMU on channel %d woken up", ch);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        
        if (imu_write_reg(ch, ICM20948_PWR_MGMT_2, 0x00) == ESP_OK) {
            ESP_LOGI(TAG, "IMU on channel %d sensors enabled", ch);
        }
    }
    
    while (1) {
        for (uint8_t ch = 0; ch < MAX_IMUS; ch++) {
            if (imu_read_reg(ch, ICM20948_WHO_AM_I, &whoami, 1) == ESP_OK) {
                ESP_LOGI(TAG, "IMU on channel %d WHO_AM_I = 0x%X", ch, whoami);
            } else {
                ESP_LOGW(TAG, "Failed to read IMU on channel %d", ch);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void start_imu_task(void) {
    
    i2c_master_init();
    
    xTaskCreate(imu_task, "imu_task", 2048, NULL, 5, NULL);
}

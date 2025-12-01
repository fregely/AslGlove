#include "ICM20948.h"
#include "TCA9548A.h"
#include "common.h"
#include "imu_i2c.h"


static i2c_master_dev_handle_t imu_dev_handle = NULL;

// Initialise device head
esp_err_t imu_device_init(i2c_master_bus_handle_t bus_handle, uint32_t scl_speed_hz) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_I2C_ADDR,  // Same address for all IMUs
        .scl_speed_hz = scl_speed_hz,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &imu_dev_handle);
}

// Helper function for bank switching
static esp_err_t imu_select_bank(uint8_t bank) {
    return imu_write_reg(ICM20948_REG_BANK_SEL, bank);
}

// Sets up IMUs so that data can be read, and enables passthrough mode so we can read mag data
esp_err_t imu_setup() {
    esp_err_t ret;

    uint8_t who_am_i = 0;
    ret = imu_read_reg(ICM20948_WHO_AM_I, &who_am_i, 2);
    if (ret != ESP_OK) return ret;

    
    if (who_am_i != 0xEA) {
        ESP_LOGE(TAG, "ICM20948 WHO_AM_I mismatch: expected 0x%02X, got 0x%02X", 0xEA, who_am_i);
        return ESP_FAIL;
    }

    // Switch to Bank 0
    ret = imu_select_bank(0);
    if (ret != ESP_OK) return ret;

    // Wake up device and set clock source
    ret = imu_write_reg(ICM20948_PWR_MGMT_1, ICM20948_CLK_AUTO);
    if (ret != ESP_OK) return ret;

    // Enable all sensors
    ret = imu_write_reg(ICM20948_PWR_MGMT_2, 0x00);
    if (ret != ESP_OK) return ret;

    // Switch to Bank 2 for sensor configuration
    ret = imu_select_bank(2);
    if (ret != ESP_OK) return ret;
    
    // Configure Gyroscope: ±250°/s range (GYRO_FS_SEL = 0)
    // Register: GYRO_CONFIG_1 (0x01)
    // Bits [2:1] = 00 for ±250°/s
    ret = imu_write_reg(ICM20948_GYRO_CONFIG_1, 0x01);  // 0x01 = ±250°/s with DLPF enabled
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return ret;
    }
    ESP_LOGI(TAG, "Gyro configured: ±250°/s range");
    
    // Configure Accelerometer: ±2g range (ACCEL_FS_SEL = 0)
    // Register: ACCEL_CONFIG (0x14)
    // Bits [2:1] = 00 for ±2g
    ret = imu_write_reg(ICM20948_ACCEL_CONFIG, 0x01);  // 0x01 = ±2g with DLPF enabled
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ret;
    }
    ESP_LOGI(TAG, "Accel configured: ±2g range");
    
    // Switch back to Bank 0
    ret = imu_select_bank(0);
    if (ret != ESP_OK) return ret;
    // Disable I2C Master mode to use I2C bypass for magnetometer
    ret = imu_write_reg(ICM20948_USER_CTRL, 0x00);
    if (ret != ESP_OK) return ret;

    // Enable I2C bypass to access magnetometer directly
    ret = imu_write_reg(ICM20948_INT_PIN_CFG, 0x02);
    if (ret != ESP_OK) return ret;
    return ESP_OK;
}

esp_err_t imu_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    // Now communicate with the IMU on that channel
    return i2c_master_transmit_receive(imu_dev_handle, 
                                       &reg_addr, 1, 
                                       data, len, 
                                       I2C_MASTER_TIMEOUT_MS);
}

esp_err_t imu_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t buf[2] = {reg_addr, data};
    return i2c_master_transmit(imu_dev_handle, 
                               buf, sizeof(buf), 
                               I2C_MASTER_TIMEOUT_MS);
}

esp_err_t imu_data_get(uint8_t raw_data[12]) {
    // 6 bytes accel, 6 bytes gyro
    esp_err_t ret = imu_read_reg(ICM20948_ACCEL_XOUT_H, raw_data, 12); // Read all sensor data in one go // cant use sizeof here because its a pointer
    // I2C registers are sequential so it will start with the first 8 bits at the accel X high byte and then count to read the next 14 8 byte registers as well. 
        if (ret != ESP_OK) {
            return ret;
        }
        return ESP_OK;
    }




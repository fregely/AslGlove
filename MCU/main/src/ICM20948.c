#include "ICM20948.h"
#include "TCA9548A.h"
#include "imu_i2c.h"


static i2c_master_dev_handle_t imu_dev_handle = NULL;

esp_err_t imu_device_init(i2c_master_bus_handle_t bus_handle, uint32_t scl_speed_hz) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_I2C_ADDR,  // Same address for all IMUs
        .scl_speed_hz = scl_speed_hz,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &imu_dev_handle);
}

esp_err_t imu_read_reg(uint8_t channel, uint8_t reg_addr, uint8_t *data, size_t len) {
    // Select the multiplexer channel first
    ESP_ERROR_CHECK(tca9548a_select(channel));
    
    // Now communicate with the IMU on that channel
    return i2c_master_transmit_receive(imu_dev_handle, 
                                       &reg_addr, 1, 
                                       data, len, 
                                       I2C_MASTER_TIMEOUT_MS);
}

esp_err_t imu_write_reg(uint8_t channel, uint8_t reg_addr, uint8_t data) {
    // Select the multiplexer channel first
    ESP_ERROR_CHECK(tca9548a_select(channel));
    
    uint8_t buf[2] = {reg_addr, data};
    return i2c_master_transmit(imu_dev_handle, 
                               buf, sizeof(buf), 
                               I2C_MASTER_TIMEOUT_MS);
}


#include "TCA9548A.h"
#include "imu_i2c.h"
#include "common.h"
static i2c_master_dev_handle_t tca9548a_dev_handle = NULL;

esp_err_t tca9548a_init(i2c_master_bus_handle_t bus_handle, uint32_t scl_speed_hz) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9548A_I2C_ADDR,
        .scl_speed_hz = scl_speed_hz,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &tca9548a_dev_handle);
}

esp_err_t tca9548a_select(uint8_t channel) {
    if (channel > 7) return ESP_ERR_INVALID_ARG;
    uint8_t mask = 1 << channel;
    return i2c_master_transmit(tca9548a_dev_handle, &mask, 1, 1000);
}
// Add this test function to check basic I2C communication
esp_err_t test_multiplexer_detection(void) {
    uint8_t test_data;
    
    // Try to read current channel selection from multiplexer
    esp_err_t ret = i2c_master_receive(tca9548a_dev_handle, &test_data, 1, 1000);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Multiplexer detected, current channels: 0x%02X", test_data);
    } else {
        ESP_LOGE(TAG, "Multiplexer not responding: %s", esp_err_to_name(ret));
    }
    return ret;
}

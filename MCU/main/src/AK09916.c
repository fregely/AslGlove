#include "AK09916.h"
#include "imu_i2c.h"
#include "common.h"

static i2c_master_dev_handle_t mag_dev_handle = NULL;

esp_err_t ak09916_device_init(i2c_master_bus_handle_t bus_handle, uint32_t scl_speed_hz) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AK09916_I2C_ADDR,
        .scl_speed_hz = scl_speed_hz,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &mag_dev_handle);
}

static esp_err_t ak09916_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    // Now communicate with the Magnetometer on that channel
    return i2c_master_transmit_receive(mag_dev_handle, 
                                       &reg_addr, 1, 
                                       data, len, 
                                       I2C_MASTER_TIMEOUT_MS);
}

esp_err_t ak09916_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t buf[2] = {reg_addr, data};
    return i2c_master_transmit(mag_dev_handle, 
                               buf, sizeof(buf), 
                               I2C_MASTER_TIMEOUT_MS);
}
esp_err_t ak09916_setup() {
    esp_err_t ret;
    uint8_t who_am_i;
    ret = ak09916_read_reg(AK09916_WIA2, &who_am_i, 2);
    if (ret != ESP_OK) return ret;
    if (who_am_i != AK09916_DEVICE_ID) {
        ESP_LOGE(TAG, "AK09916 WHO_AM_I mismatch: expected 0x%02X, got 0x%02X", AK09916_DEVICE_ID, who_am_i);
        return ESP_FAIL;
    }
    // Soft reset
    ret = ak09916_write_reg(AK09916_CNTL3, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set to 50 Hz continuous mode (Mode 3)
    ESP_LOGI(TAG, "Setting magnetometer to 50 Hz continuous mode");
    ret = ak09916_write_reg(AK09916_CNTL2, 0x06);  // ← Mode 3
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify
    uint8_t mode_check;
    ret = ak09916_read_reg(AK09916_CNTL2, &mode_check, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Magnetometer CNTL2 = 0x%02X (should be 0x06)", mode_check);
    }
    
    return ESP_OK;
}
esp_err_t ak09916_read_mag_data(uint8_t mag_data[6]) {
    esp_err_t ret;
    uint8_t st1, st2;
    
    // Step 1: Check if data is ready (optional but good practice)
    ret = ak09916_read_reg(AK09916_ST1, &st1, 1);
    if (ret != ESP_OK) return ret;
    
    if (!(st1 & 0x01)) {
        ESP_LOGD(TAG, "Mag data not ready");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Step 2: Read 6 bytes of mag data
    ret = ak09916_read_reg(AK09916_HXL, mag_data, 6);
    if (ret != ESP_OK) return ret;
    
    // Step 3: Read ST2 to complete the sequence
    ret = ak09916_read_reg(AK09916_ST2, &st2, 1);
    if (ret != ESP_OK) return ret;
    
    // Optional: Check for overflow
    if (st2 & 0x08) {
        ESP_LOGW(TAG, "Mag overflow");
        return ESP_ERR_INVALID_STATE;
    }
    
    return ESP_OK;
}



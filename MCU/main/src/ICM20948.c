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

// Dont need to pass by reference here, because raw data is already a pointer
esp_err_t imu_data_get(uint8_t channel, uint8_t raw_data[12]) {
    // 6 bytes accel, 2 bytes temp, 6 bytes gyro
    esp_err_t ret = imu_read_reg(channel, ICM20948_ACCEL_XOUT_H, raw_data, 12); // Read all sensor data in one go // cant use sizeof here because its a pointer
    // I2C registers are sequential so it will start with the first 8 bits at the accel X high byte and then count to read the next 14 8 byte registers as well. 
        if (ret != ESP_OK) {
            return ret;
        }
        // Add code to process raw_data if needed
        return ESP_OK;
    }


    /*
    Keep the raw data and we'll send that to a computer via BLE
    // Convert raw data to physical values (assuming default sensitivity settings)
    int16_t ax = (raw_data[0] << 8) | raw_data[1];
    int16_t ay = (raw_data[2] << 8) | raw_data[3];
    int16_t az = (raw_data[4] << 8) | raw_data[5];
    int16_t gx = (raw_data[6] << 8) | raw_data[7];
    int16_t gy = (raw_data[8] << 8) | raw_data[9];
    int16_t gz = (raw_data[10] << 8) | raw_data[11];


    // Assuming default full scale ranges: Accel ±2g, Gyro ±250dps
    data->accel_x = ax / 16384.0f; // g
    data->accel_y = ay / 16384.0f; // g
    data->accel_z = az / 16384.0f; // g
    data->gyro_x = gx / 131.0f; // dps
    data->gyro_y = gy / 131.0f; // dps
    data->gyro_z = gz / 131.0f; // dps

    // Magnetometer reading can be added similarly if needed
   
    return ESP_OK;
}
 */ 
// For setting up sync notes
/*
Start with software timestamps for initial development
Add FSYNC hardware sync for production system
Use TIME_SYNC characteristic for computer synchronization
Let computer handle final sensor fusion with camera data
typedef struct {
    uint8_t channel;
    uint64_t timestamp_us;  // ESP32 timestamp
    uint8_t raw_data[14];
} __attribute__((packed)) synced_imu_packet_t;

AI's suggestion for synchronized reading function:
void synchronized_read_all_imus(void) {
    uint64_t sync_time = esp_timer_get_time(); // Get precise timestamp
    
    for (uint8_t ch = 0; ch < 6; ch++) {
        synced_imu_packet_t packet;
        packet.channel = ch;
        packet.timestamp_us = sync_time;
        
        if (imu_read_reg(ch, ICM20948_ACCEL_XOUT_H, packet.raw_data, 14) == ESP_OK) {
            // Send immediately or buffer for transmission
            gatt_send_imu_notification((uint8_t*)&packet, sizeof(packet));
        }
    }
}

AI's sugestion for FSYNC configuration and generation:
// Configure FSYNC for external sync
esp_err_t imu_configure_fsync(uint8_t channel) {
    // Bank 2, register 0x52 (FSYNC_CONFIG)
    // Enable FSYNC and set to trigger on rising edge
    return imu_write_reg_with_bank(channel, 2, ICM20948_FSYNC_CONFIG, 0x06);
}

// Generate sync pulse from ESP32 GPIO
void generate_sync_pulse(void) {
    gpio_set_level(SYNC_GPIO_PIN, 1);
    esp_rom_delay_us(10); // 10us pulse
    gpio_set_level(SYNC_GPIO_PIN, 0);
}
*/

#ifndef AK09916_H
#define AK09916_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

// AK09916 I2C Address
#define AK09916_I2C_ADDR        0x0C

// AK09916 Register Map
#define AK09916_WIA1            0x00  // Company ID (should be 0x48)
#define AK09916_WIA2            0x01  // Device ID (should be 0x09)
#define AK09916_RSV1            0x02  // Reserved
#define AK09916_RSV2            0x03  // Reserved
#define AK09916_ST1             0x10  // Status 1
#define AK09916_HXL             0x11  // X-axis data low
#define AK09916_HXH             0x12  // X-axis data high
#define AK09916_HYL             0x13  // Y-axis data low
#define AK09916_HYH             0x14  // Y-axis data high
#define AK09916_HZL             0x15  // Z-axis data low
#define AK09916_HZH             0x16  // Z-axis data high
#define AK09916_ST2             0x18  // Status 2
#define AK09916_CNTL2           0x31  // Control 2
#define AK09916_CNTL3           0x32  // Control 3

// AK09916 Status bits
#define AK09916_ST1_DRDY        0x01  // Data ready
#define AK09916_ST1_DOR         0x02  // Data overrun
#define AK09916_ST2_HOFL        0x08  // Magnetic sensor overflow

// AK09916 Measurement modes (CNTL2 register)
#define AK09916_MODE_POWER_DOWN     0x00
#define AK09916_MODE_SINGLE         0x01
#define AK09916_MODE_CONT1          0x02  // 10Hz
#define AK09916_MODE_CONT2          0x04  // 20Hz
#define AK09916_MODE_CONT3          0x06  // 50Hz
#define AK09916_MODE_CONT4          0x08  // 100Hz
#define AK09916_MODE_SELF_TEST      0x10

// AK09916 Control commands (CNTL3 register)
#define AK09916_SOFT_RESET          0x01

// Expected WHO_AM_I values
#define AK09916_COMPANY_ID          0x48
#define AK09916_DEVICE_ID           0x09


esp_err_t ak09916_device_init(i2c_master_bus_handle_t bus_handle, uint32_t scl_speed_hz);
esp_err_t ak09916_read_mag_data(uint8_t mag_data[6]);
esp_err_t ak09916_write_reg(uint8_t reg_addr, uint8_t data);

esp_err_t ak09916_setup();

#endif // AK09916_H

#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>
#include "driver/i2c_master.h"

// ICM-20948 I2C address (AD0 = 0 -> 0x68, AD0 = 1 -> 0x69)
#define ICM20948_I2C_ADDR       0x68

// Register banks
#define ICM20948_REG_BANK_SEL   0x7F
#define ICM20948_BANK0          0x00
#define ICM20948_BANK1          0x10
#define ICM20948_BANK2          0x20
#define ICM20948_BANK3          0x30

// ---- BANK 0 ----
// Who Am I
#define ICM20948_WHO_AM_I       0x00  // Should return 0xEA

// Power management
#define ICM20948_PWR_MGMT_1     0x06
#define ICM20948_PWR_MGMT_2     0x07

// Accelerometer & Gyroscope data registers
#define ICM20948_ACCEL_XOUT_H   0x2D
#define ICM20948_ACCEL_XOUT_L   0x2E
#define ICM20948_ACCEL_YOUT_H   0x2F
#define ICM20948_ACCEL_YOUT_L   0x30
#define ICM20948_ACCEL_ZOUT_H   0x31
#define ICM20948_ACCEL_ZOUT_L   0x32

#define ICM20948_GYRO_XOUT_H    0x33
#define ICM20948_GYRO_XOUT_L    0x34
#define ICM20948_GYRO_YOUT_H    0x35
#define ICM20948_GYRO_YOUT_L    0x36
#define ICM20948_GYRO_ZOUT_H    0x37
#define ICM20948_GYRO_ZOUT_L    0x38

// Temperature
#define ICM20948_TEMP_OUT_H     0x39
#define ICM20948_TEMP_OUT_L     0x3A

// ---- BANK 2 ----
// Gyro configuration
#define ICM20948_GYRO_CONFIG_1  0x01
#define ICM20948_GYRO_CONFIG_2  0x02

// Accel configuration
#define ICM20948_ACCEL_CONFIG   0x14
#define ICM20948_ACCEL_CONFIG_2 0x15

// ---- Magnetometer (AK09916 inside ICM-20948) ----
// Accessible via I2C master passthrough
#define AK09916_I2C_ADDR        0x0C
#define AK09916_WHO_AM_I        0x00  // Should return 0x09
#define AK09916_ST1             0x10
#define AK09916_HXL             0x11
#define AK09916_HXH             0x12
#define AK09916_HYL             0x13
#define AK09916_HYH             0x14
#define AK09916_HZL             0x15
#define AK09916_HZH             0x16
#define AK09916_ST2             0x18

// ---- Useful constants ----
#define ICM20948_RESET          0x80
#define ICM20948_CLK_AUTO       0x01

// Data structure to hold sensor readings
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
    float mag_x;
    float mag_y;
    float mag_z;
} icm20948_data_t;

esp_err_t imu_read_reg(uint8_t channel, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t imu_write_reg(uint8_t channel, uint8_t reg_addr, uint8_t data);
esp_err_t imu_device_init(i2c_master_bus_handle_t bus_handle, uint32_t scl_speed_hz);
//add something to change and track the banks, for now itll just be in bank 1


#endif // ICM20948_H

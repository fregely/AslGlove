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

// User control
#define ICM20948_USER_CTRL      0x03

// Interrupt pin configuration
#define ICM20948_INT_PIN_CFG    0x0F

// External sensor data registers (for I2C Master mode)
#define ICM20948_EXT_SENS_DATA_00   0x3B
#define ICM20948_EXT_SENS_DATA_01   0x3C
// ... up to EXT_SENS_DATA_23

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

// Temperature (not used, but kept for reference)
#define ICM20948_TEMP_OUT_H     0x39
#define ICM20948_TEMP_OUT_L     0x3A

// ---- BANK 2 ----
// Gyro configuration
#define ICM20948_GYRO_CONFIG_1  0x01
#define ICM20948_GYRO_CONFIG_2  0x02

// Accel configuration
#define ICM20948_ACCEL_CONFIG   0x14
#define ICM20948_ACCEL_CONFIG_2 0x15

// ---- BANK 3 ----
// I2C Master control
#define ICM20948_I2C_MST_CTRL   0x01
#define ICM20948_I2C_MST_STATUS 0x17

// I2C Slave configuration (for I2C master mode, if needed later)
#define ICM20948_I2C_SLV0_ADDR  0x03
#define ICM20948_I2C_SLV0_REG   0x04
#define ICM20948_I2C_SLV0_CTRL  0x05

// ---- Useful constants ----
#define ICM20948_RESET          0x80
#define ICM20948_CLK_AUTO       0x01

// Function declarations
esp_err_t imu_device_init(i2c_master_bus_handle_t bus_handle, uint32_t scl_speed_hz);
esp_err_t imu_read_reg(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t imu_write_reg(uint8_t reg_addr, uint8_t data);
esp_err_t imu_setup();

// Accel + Gyro (12 bytes, no temperature)
esp_err_t imu_data_get(uint8_t raw_data[12]);

#endif // ICM20948_H

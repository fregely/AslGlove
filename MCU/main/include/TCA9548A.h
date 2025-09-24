#ifndef TCA9548A_H
#define TCA9548A_H

#pragma once // so it only runs this file once 
#include "driver/i2c_master.h"

#define TCA9548A_I2C_ADDR 0x70  // Default address

esp_err_t tca9548a_init(i2c_master_bus_handle_t bus_handle, uint32_t scl_speed_hz);
esp_err_t tca9548a_select(uint8_t channel);
esp_err_t test_multiplexer_detection(void);

#endif

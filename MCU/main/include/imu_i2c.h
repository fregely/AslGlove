#ifndef IMU_I2C_H
#define IMU_I2C_H


#define TAG "IMU_I2C"

// I2C configuration
#define I2C_MASTER_SCL_IO           4 // GPIO number for SCL
#define I2C_MASTER_SDA_IO           5 // GPIO number for SDA
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          200000 // 200kHz I2C frequency
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define MAX_IMUS 6  // Maximum number of IMUs connected via TCA9548A multiplexer

void start_imu_task(void);

#endif // IMU_I2C_H

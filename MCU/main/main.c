#include "nimble.h"
#include "imu_i2c.h"
#include "common.h"

void app_main(void) {
    imu_init();


    start_imu_task();
    
    start_ble_task();
}

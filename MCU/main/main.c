#include "nimble.h"
#include "imu_i2c.h"
#include "common.h"

void app_main(void) {

    start_imu_task();

    start_ble_task();
}

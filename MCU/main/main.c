#include "nimble.h"
#include "imu_i2c.h"
#include "common.h"
#include "IRFlasher.h"


void app_main(void) {

    start_imu_task();

    start_ble_task();

    // start IR flasher task
    xTaskCreate(
        irflasher_task,      // Task function
        "IR_Flasher",        // Name (for debugging)
        4096,                // Stack size (bytes or words depending on config)
        NULL,                // Task argument
        5,                   // Priority
        NULL                 // Task handle (optional)
    );

}

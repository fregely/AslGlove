#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble_ble.h"

void app_main(void) {
    printf("Starting app_main...\n");

    // Create a dedicated task for BLE
    xTaskCreate(
        nimble_ble_task,     // Function to run
        "nimble_ble_task",   // Task name
        4096,                // Stack size
        NULL,                // Params
        5,                   // Priority
        NULL                 // Task handle
    );
}

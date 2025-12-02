#include "nimble.h"
#include "imu_i2c.h"
#include "common.h"
#include "IRFlasher.h"


void app_main(void) {

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << 1) | (1ULL << 3) | (1ULL << 20) | (1ULL << 6) | (1ULL << 7);
    gpio_config(&io_conf);
    // Dumps current GPIO configuration to stdout for debugging
    // gpio_dump_io_configuration(stdout, (1ULL << 1) | (1ULL << 3) | (1ULL << 20) | (1ULL << 6) | (1ULL << 7) | (1ULL << 4) | (1ULL << 5));

    start_imu_task();

    start_ble_task();

    // start IR flasher task
    xTaskCreate(irflasher_task,"IR_Flasher",4096, NULL,5,NULL);

}

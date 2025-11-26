#include "IRFlasher.h"
#include "nimble_gatt.h"   // for gatt_send_notification()
#include "common.h"       // for gpio_dump_io_configuration()


#define NUM_LEDS 5

static const int leds[NUM_LEDS] = {1,3,20,6,7};

static volatile bool started = false;
static volatile bool next_flag = false;

void irflasher_start(void) {
    started = true;
    //uint16_t current = 0;
}

void irflasher_next(void) {
    next_flag = true;
}

void irflasher_notify_ready(uint8_t led) {
    gatt_send_notification(&led, 1);
}

void irflasher_task(void *arg) {
    

    uint16_t current = 0;
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
        gpio_set_level(leds[i], 0);
    }

    while (1) {

        // Wait for Python to send START
        if (!started) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Light LED
        gpio_set_level(leds[current], 1);

        // Notify Python
        irflasher_notify_ready(current);

        // On for 3 ms
        vTaskDelay(pdMS_TO_TICKS(3));

        // Wait for Python frame-capture + NEXT command
        while (!next_flag) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        next_flag = false;

        // TURN LED OFF
        gpio_set_level(leds[current], 0);

        // Move to next LED
        current = (current + 1) % NUM_LEDS;
    }
}

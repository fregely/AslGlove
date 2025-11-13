#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// --- IR LED GPIOs ---
#define LED1_GPIO 1
#define LED2_GPIO 2
#define LED3_GPIO 3
#define LED4_GPIO 6
#define LED5_GPIO 7

// --- Target frame rate (60 FPS → ~16.6ms period) ---
#define FRAME_PERIOD_MS 16   // Full frame duration
#define PULSE_WIDTH_MS 2     // LED ON time

static void configure_leds(void) {
    int leds[] = {LED1_GPIO, LED2_GPIO, LED3_GPIO, LED4_GPIO, LED5_GPIO};
    for (int i = 0; i < 5; i++) {
        gpio_reset_pin(leds[i]);
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
    }
}

void app_main(void) {
    configure_leds();
    printf("IR LED flasher running at ~60Hz on GPIOs 1,2,3,6,7\n");

    while (1) {
        // --- Turn all LEDs ON ---
        gpio_set_level(LED1_GPIO, 1);
        gpio_set_level(LED2_GPIO, 1);
        gpio_set_level(LED3_GPIO, 1);
        gpio_set_level(LED4_GPIO, 1);
        gpio_set_level(LED5_GPIO, 1);

        // Keep ON briefly (2ms pulse)
        vTaskDelay(pdMS_TO_TICKS(PULSE_WIDTH_MS));

        // --- Turn all LEDs OFF ---
        gpio_set_level(LED1_GPIO, 0);
        gpio_set_level(LED2_GPIO, 0);
        gpio_set_level(LED3_GPIO, 0);
        gpio_set_level(LED4_GPIO, 0);
        gpio_set_level(LED5_GPIO, 0);

        // Wait remainder of frame period
        vTaskDelay(pdMS_TO_TICKS(FRAME_PERIOD_MS - PULSE_WIDTH_MS));
    }
}
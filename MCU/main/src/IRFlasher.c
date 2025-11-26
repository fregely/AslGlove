#include "IRFlasher.h"
#include "nimble_gatt.h"
#include "common.h"

#define NUM_LEDS 5
static const int leds[NUM_LEDS] = {1,3,20,6,7};

static volatile bool started = false;
static volatile bool next_flag = false;

// NEW: LED override for calibration (GPIO number), -1 = normal cycling
static volatile int override_gpio = -1;

void irflasher_start(void) {
    started = true;
}

void irflasher_next(void) {
    next_flag = true;
}

// NEW: Called when Python sends CMD_LED_SELECT
void irflasher_select_led(int gpio)
{
    override_gpio = gpio;    // store requested GPIO
    next_flag = true;        // force update
}

void irflasher_notify_ready(uint8_t led) {
    gatt_send_notification(&led, 1);
}

void irflasher_task(void *arg)
{
    gpio_dump_io_configuration(stdout,
        (1ULL<<1)|(1ULL<<3)|(1ULL<<20)|(1ULL<<6)|(1ULL<<7));

    uint16_t current = 0;

    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
        gpio_set_level(leds[i], 0);
    }

    while (1) {

        if (!started) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // ===========================================
        // 🔵 CALIBRATION MODE (override_gpio >= 0)
        // ===========================================
        if (override_gpio >= 0) {

            // Turn off all LEDs first
            for (int i = 0; i < NUM_LEDS; i++)
                gpio_set_level(leds[i], 0);

            // 255 = special code to exit calibration mode
            if (override_gpio == 255) {
                uint8_t off_val = 255;
                irflasher_notify_ready(off_val);
                override_gpio = -1;     // EXIT override mode
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }

            // Otherwise turn on the requested LED
            gpio_set_level(override_gpio, 1);

            // notify Python of LED index
            uint8_t idx = 0;
            for (int i = 0; i < NUM_LEDS; i++)
                if (leds[i] == override_gpio)
                    idx = i;

            irflasher_notify_ready(idx);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // ===========================================
        // 🔵 NORMAL CYCLING MODE
        // ===========================================
        gpio_set_level(leds[current], 1);

        irflasher_notify_ready(current);

        vTaskDelay(pdMS_TO_TICKS(3));

        while (!next_flag)
            vTaskDelay(pdMS_TO_TICKS(1));

        next_flag = false;

        gpio_set_level(leds[current], 0);

        current = (current + 1) % NUM_LEDS;
    }
}
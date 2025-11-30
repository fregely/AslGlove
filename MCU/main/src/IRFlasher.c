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
    
void irflasher_reset(void) {
    ESP_LOGI("IR_Flasher", "Resetting state due to BLE disconnect");
    started = false;
    next_flag = false;
    // Turn off all LEDs
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_set_level(leds[i], 0);
    }
}
void irflasher_task(void *arg) {
    uint16_t current = 0;
    
    // Initialize GPIOs
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
        gpio_set_level(leds[i], 0);
    }
    
    ESP_LOGI("IR_Flasher", "Task started, waiting for START command");
    
    while (1) {
        // Wait for Python to send START
        if (!started) {
            current = 0;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Light LED
        gpio_set_level(leds[current], 1);

        // Notify Python
        irflasher_notify_ready(current);

        // On for 3 ms
        vTaskDelay(pdMS_TO_TICKS(3));

        // Wait for NEXT command with realistic timeout
        uint32_t wait_count = 0;
        const uint32_t TIMEOUT_MS = 2000;      // 2 second timeout (was too short!)
        const uint32_t WAIT_INTERVAL_MS = 10;  // Check every 10ms
        const uint32_t MAX_WAITS = TIMEOUT_MS / WAIT_INTERVAL_MS;
        while (!next_flag && started) {
            vTaskDelay(pdMS_TO_TICKS(WAIT_INTERVAL_MS));
            wait_count++;
            
            // Log every 500ms so we can see it's waiting
            if (wait_count % (500 / WAIT_INTERVAL_MS) == 0) {
                ESP_LOGD("IR_Flasher", "Still waiting... (%d ms elapsed)", 
                         wait_count * WAIT_INTERVAL_MS);
            }
            
            if (wait_count > MAX_WAITS) {
                ESP_LOGW("IR_Flasher", "⏱️ TIMEOUT after %dms - no NEXT received", TIMEOUT_MS);
                irflasher_reset();
                break;
            }
        }
        next_flag = false;

        // TURN LED OFF
        gpio_set_level(leds[current], 0);
        // Move to next LED
        current = (current + 1) % NUM_LEDS;
        


                
        
        
    
    }
}

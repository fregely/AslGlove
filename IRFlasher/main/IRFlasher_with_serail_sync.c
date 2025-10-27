#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// --- UART Configuration ---
#define UART_PORT       UART_NUM_0   // default USB serial port
#define UART_BAUD_RATE  115200
#define BUF_SIZE        1024

// --- IR LED GPIOs ---
#define LED1_GPIO 1
#define LED2_GPIO 2
#define LED3_GPIO 3
#define LED4_GPIO 6
#define LED5_GPIO 7
#define NUM_LEDS  5

// --- Timing ---
#define PULSE_WIDTH_MS 3   // LED on time in milliseconds

static const int leds[NUM_LEDS] = {LED1_GPIO, LED2_GPIO, LED3_GPIO, LED4_GPIO, LED5_GPIO};

// --- Configure LED pins ---
static void configure_leds(void) {
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_reset_pin(leds[i]);
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
        gpio_set_level(leds[i], 0);
    }
}

// --- Configure UART for serial comms with Python ---
static void configure_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
}

// --- Main Application ---
void app_main(void) {
    configure_leds();
    configure_uart();

    printf("ESP32 IRFlasher synchronized mode ready\n");
    printf("Waiting for START from Python...\n");

    uint8_t rx_data[BUF_SIZE];
    int current_led = 0;
    bool started = false;

    while (1) {
        // --- Read any incoming UART data ---
        int len = uart_read_bytes(UART_PORT, rx_data, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            rx_data[len] = '\0'; // Null-terminate received string
            if (strstr((char *)rx_data, "START")) {
                started = true;
                current_led = 0;
                printf("START received from Python\n");
            } else if (strstr((char *)rx_data, "NEXT")) {
                // Turn off the current LED
                gpio_set_level(leds[current_led], 0);
                // Move to next LED (wrap around)
                current_led = (current_led + 1) % NUM_LEDS;
            }
        }

        if (!started) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // --- Turn ON current LED ---
        gpio_set_level(leds[current_led], 1);

        // --- Send READY message ---
        char msg[32];
        snprintf(msg, sizeof(msg), "READY:%d\n", current_led);
        uart_write_bytes(UART_PORT, msg, strlen(msg));

        // --- Keep LED on briefly ---
        vTaskDelay(pdMS_TO_TICKS(PULSE_WIDTH_MS));

        // NOTE: Turning off LED happens after Python sends "NEXT"
        // so we don't turn it off here yet
    }
}
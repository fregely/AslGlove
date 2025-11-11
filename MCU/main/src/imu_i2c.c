#include "imu_i2c.h"
#include "TCA9548A.h"
#include "ICM20948.h"
#include "AK09916.h"
#include "common.h"
#include "driver/i2c_master.h"
#include "imu_packet.h"


static i2c_master_bus_handle_t i2c_bus;
QueueHandle_t imu_data_queue = NULL;

// Track which channels have working sensors
static bool imu_present[MAX_IMUS] = {false};
static bool mag_present[MAX_IMUS] = {false};

// Creates and Declares the ESP as the Master for the I2C line, and then initialise all other addresses. 
static void i2c_master_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus)); // Declares the MCU as I2C master
    ESP_ERROR_CHECK(tca9548a_init(i2c_bus, I2C_MASTER_FREQ_HZ)); // Initialize TCA9548A multiplexer as a buss
    ESP_ERROR_CHECK(imu_device_init(i2c_bus, I2C_MASTER_FREQ_HZ)); // Initialize ICM20948 IMU device
    ESP_ERROR_CHECK(ak09916_device_init(i2c_bus, I2C_MASTER_FREQ_HZ)); // Initialize AK09916 Magnetometer device
}


// Main Task for Gather IMU data, What RTOS runs. 
// Tests multiplexer, and then activates all the IMUs and Mags, and notes which ones are present and which are not
static void imu_task(void *param) {
    ESP_LOGI(TAG, "Starting IMU initialization sequence...");
    
    // Test multiplexer
    if(test_multiplexer_detection() != ESP_OK) {
        ESP_LOGE(TAG, "Multiplexer detection failed!");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Multiplexer detected successfully");
    
    // Initialize all channels
    uint8_t imu_count = 0;
    uint8_t mag_count = 0;
    
    for(uint8_t ch = 0; ch < MAX_IMUS; ch++) {
        ESP_LOGI(TAG, "Scanning channel %d...", ch);
        
        // Select channel ONCE for all operations on this channel
        ESP_ERROR_CHECK(tca9548a_select(ch));
        
        // Try to initialize IMU
        if(imu_setup() == ESP_OK) {
            ESP_LOGI(TAG, "IMU initialized on channel %d", ch);
            imu_present[ch] = true;
            imu_count++;
            
            // Only try magnetometer if IMU works
            if(ak09916_setup() == ESP_OK) {
                ESP_LOGI(TAG, "Magnetometer initialized on channel %d", ch);
                mag_present[ch] = true;
                mag_count++;
            } else {
                ESP_LOGW(TAG, "Magnetometer not detected on channel %d", ch);
            }
        } else {
            ESP_LOGW(TAG, "No IMU detected on channel %d", ch);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Print summary
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initialization Summary:");
    ESP_LOGI(TAG, "  IMUs detected: %d/%d", imu_count, MAX_IMUS);
    ESP_LOGI(TAG, "  Magnetometers detected: %d/%d", mag_count, MAX_IMUS);
    ESP_LOGI(TAG, "========================================");
    
    for(uint8_t ch = 0; ch < MAX_IMUS; ch++) {
        if(imu_present[ch]) {
            ESP_LOGI(TAG, "  Channel %d: IMU ✓  Mag %s", ch, mag_present[ch] ? "✓" : "✗");
        }
    }
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    
    // Kills the task if their are no IMUs, to save power. 
    if(imu_count == 0) {
        ESP_LOGE(TAG, "No IMUs detected! Task will exit.");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Starting sensor read loop...");
    
    // Main reading loop
    while (1) {
        for(uint8_t ch = 0; ch < MAX_IMUS; ch++) {
            // Skip channels without working IMUs
            if(!imu_present[ch]) {
                continue; // Will have it go to next of the for loop
            }
            
            imu_packet_t packet;
            packet.timestamp_us = esp_timer_get_time();
            packet.channel = ch;
            
            ESP_ERROR_CHECK(tca9548a_select(ch));
            
            // Read IMU data
            esp_err_t imu_ret = imu_data_get(packet.raw_data);
            if (imu_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to read IMU data on channel %d", ch);
                continue;  // Skips to next channel
            }
            
            ESP_LOGD(TAG, "Read IMU data from channel %d", ch);
            
            // Read magnetometer data
            if(mag_present[ch]) {
                esp_err_t mag_ret = ak09916_read_mag_data(packet.mag_data);
                if(mag_ret != ESP_OK) {
                    ESP_LOGD(TAG, "Mag data not ready on channel %d, zeroing", ch);
                    memset(packet.mag_data, 0, 6);
                }
            } else {
                // No magnetometer on this channel sets mag data to 0
                memset(packet.mag_data, 0, 6);
            }
            
            // Queues packet
            if (xQueueSend(imu_data_queue, &packet, pdMS_TO_TICKS(10)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send IMU data to queue (queue full?)");
            } else {
                ESP_LOGD(TAG, "Sent packet from channel %d", ch);
            }
            
            // Small delay between channels
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        // Delay between full scans
        vTaskDelay(pdMS_TO_TICKS(50));

        // Currently running 1.4kB/s can go up to 10kB/s if needed
        // After testing should increase speed, by decreasing delay to 5, and 20, and increasing the I2C speed queue size to 100. 
        // 3.2Kb/s 20Hz/IMU vs current of 9 Hz/IMU
    }
}

// The entry point for the task
void start_imu_task(void) {
    i2c_master_init();
    
    // Create the queue
    imu_data_queue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(imu_packet_t));
    if (imu_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU data queue");
        return;
    }

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}

#ifndef IMU_PACKET_H
#define IMU_PACKET_H


 /* For Making Packets */
 #include "esp_timer.h" 
 #include "freertos/FreeRTOS.h"
 #include "freertos/queue.h" 
 
 #define IMU_QUEUE_SIZE 50  // Adjust based on your needs

 extern QueueHandle_t imu_data_queue;

 typedef struct {
     uint8_t channel;
     uint64_t timestamp_us;  // ESP32 timestamp
     uint8_t raw_data[12];
     uint8_t mag_data[6]; 
 } __attribute__((packed)) imu_packet_t;  //attribute packed to avoid padding, not needed but good practice for data packets <- from ai, i was gonna say it might break it, but ai is very confident.

#endif // IMU_PACKET_H

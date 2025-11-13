# ASL Glove - ESP32 BLE Firmware

> Real-time IMU data acquisition system using ESP32 with multiple ICM-20948 9-axis sensors, transmitting orientation and motion data via Bluetooth Low Energy.

## Overview

This is an ESP-IDF project that runs a NimBLE BLE server to broadcast data collected from up to six IMU sensors. The firmware reads from ICM-20948 9-axis sensors connected through a TCA9548A I2C multiplexer and streams timestamped sensor data packets over BLE.

**Key Features:**
- Multi-IMU support (6 sensors via I2C multiplexer)
- Real-time BLE data streaming
- Microsecond-precision timestamps
- 9-axis sensor data (accelerometer, gyroscope, magnetometer)
- FreeRTOS-based task management    

### Gatt Characteristics

| Characteristic |                  UUID                  | Properties |        Description        |
|----------------|----------------------------------------|------------|---------------------------|
| **IMU_DATA**   | `c4e7a180-7b2f-4c95-bfc5-1d5c62123456` | Notify     | Streams 27-byte packets   |
| **TIME_SYNC**  | `abcdef12-3456-7890-abcd-ef1234567890` | Read/Write | Time synchronization      |
| **LED_STATE**  | `01234567-89ab-cdef-0123-456789abcdef` | Read/Write | LED control (placeholder) |

## Packet Format

**Total Size:** 27 bytes

| Offset |  Size   |      Field     |   Data Type   |   Endianness  |
|--------|---------|----------------|---------------|---------------|
|    0   | 1 byte  | Channel        | `uint8_t`     |        -      |
|    1   | 8 bytes | Timestamp (µs) | `uint64_t`    | Little-endian |
|    9   | 6 bytes | Accel X,Y,Z    | `int16_t x 3` | Big-endian    |
|   15   | 6 bytes | Gyro X,Y,Z     | `int16_t x 3` | Big-endian    |
|   21   | 6 bytes | Mag X,Y,Z      | `int16_t x 3` | Little-endian | <--- Double check if this is little endian

## RTOS documentation
https://www.freertos.org/Documentation/01-FreeRTOS-quick-start/01-Beginners-guide/00-Overview

## Guide for Setting up ESP/RTOS 
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html
https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c3/esp32-c3-devkitm-1/user_guide.html
https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf

## NimBLE information
https://www.bluetooth.com/specifications/specs/battery-service/ - Battery information GATT service
https://learn.adafruit.com/introduction-to-bluetooth-low-energy/introduction - Information on BLE and how they work

## Linux Build Guide

### First Time Setup
```bash
# Set up ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# Or if you have get_idf alias set up:
get_idf

# Navigate to project directory
cd MCU

# Clean previous builds
idf.py fullclean

# Set target chip
idf.py set-target esp32c3

# Build firmware
idf.py build
```

### Flashing and Monitoring
```bash
# Set up environment (if not already done)
get_idf  # or: . $HOME/esp/esp-idf/export.sh

# Navigate to project
cd MCU

# Flash and monitor (replace /dev/ttyUSB0 with your port)
idf.py -p /dev/ttyUSB0 flash monitor
```

### Finding USB Port
```bash
# List USB serial devices
ls -l /dev/ttyUSB*

# Or for all USB devices
lsusb

# On macOS, use:
ls -l /dev/cu.usbserial-*
```

## Exit Monitor

Press `Ctrl + ]` to exit the serial monitor.

## 🔧 Useful Commands
```bash
# Clean build artifacts
idf.py fullclean

# Monitor serial output only (without flashing)
idf.py -p /dev/ttyUSB0 monitor

# Set target chip (ESP32-C3, ESP32, ESP32-S3, etc.)
idf.py set-target esp32c3

# Erase entire flash memory
idf.py -p /dev/ttyUSB0 erase-flash

# View partition table
idf.py partition-table

# Analyze binary size by component
idf.py size-components

# Configure project settings
idf.py menuconfig
```

## Connecting via Bluetooth (Linux)

### Using bluetoothctl
```bash
# Start bluetoothctl
bluetoothctl

# Scan for devices
scan on

# Connect to device (replace with your device MAC address)
connect 18:8B:0E:AE:BF:CE

# Access GATT menu
menu gatt

# List all attributes
list-attributes

# Select IMU_DATA characteristic (replace with actual path)
select-attribute /org/bluez/hci0/dev_18_8B_0E_B0_7B_B6/service000e/char000f

# Enable notifications
notify on

# To disconnect
disconnect
quit
```

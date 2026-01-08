# ESP32 RMT-Based DHT11 Driver

A high-precision ESP-IDF component for reading DHT11 or DHT22 temperature and humidity sensors using the RMT (Remote Control) hardware peripheral.

## Features

- Uses the ESP32 RMT peripheral for microsecond-accurate pulse decoding.
- Built on FreeRTOS tasks and semaphores, non-blocking allows for extensions.


## Hardware

- Tested on ESP32-WROVER-B
- DHT11 temperature and humidity sensor
- TP223 Capacitive touch sensor


## Usage

Modify the pin assignments in temp_humidity_sensor.h to match your hardware:
```C

const int DHT_IO = GPIO_NUM_32;      // Data Pin
#define TOUCH_SENSOR_GPIO GPIO_NUM_23 // Trigger Pin
```



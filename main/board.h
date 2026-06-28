//
// board.h — GPIO pin map and board-level constants.
//
#pragma once

#include <driver/gpio.h>

/* The "Boot" button doubles as Wi-Fi reset (short) / factory reset (long). */
#define RESET_GPIO        GPIO_NUM_0

#define RUNNING_LED_GPIO  GPIO_NUM_2
#define DOOR_LED_GPIO     GPIO_NUM_16
#define DOOR_RELAY_GPIO   GPIO_NUM_5
#define DOOR_SENSOR_GPIO  GPIO_NUM_4
#define DHT_SENSOR_GPIO   GPIO_NUM_22

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<RUNNING_LED_GPIO) | (1ULL<<DOOR_LED_GPIO) | (1ULL<<DOOR_RELAY_GPIO))
#define GPIO_INPUT_PIN_SEL   (1ULL<<DOOR_SENSOR_GPIO)

#define ESP_INTR_FLAG_DEFAULT 0

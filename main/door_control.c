//
// door_control.c — door state machine, relay, reed sensor, failsafe timer.
//
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "door_control.h"
#include "hap_garage.h"
#include "board.h"

static const char *TAG = "HAP Garage";

static QueueHandle_t gpio_evt_queue = NULL;
static uint8_t door_state = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN;
static esp_timer_handle_t oneshot_timer;

uint8_t door_get_state(void) { return door_state; }
static void door_set_state(uint8_t s) { door_state = s; }

/* Reed switch: hardware reads inverted, so XOR to logical DOOR_OPEN/DOOR_CLOSED. */
static uint8_t read_door_sensor(void) { return gpio_get_level(DOOR_SENSOR_GPIO) ^ 1; }

static void kick_door_relay(void) {
  gpio_set_level(DOOR_RELAY_GPIO, 1);
  vTaskDelay(300 / portTICK_PERIOD_MS);
  gpio_set_level(DOOR_RELAY_GPIO, 0);
}

static void timer_callback(void *args) {
  ESP_LOGI(TAG, "Timer expired. Setting state according to sensor.");
  if (read_door_sensor() == DOOR_OPEN) {
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
  } else {
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
  }
}

static const esp_timer_create_args_t oneshot_timer_args = {
  .callback = &timer_callback, .name = "one-shot"
};

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void door_handle_homekit_request(void) {
  if (read_door_sensor() == DOOR_OPEN ||
      door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING) {
    kick_door_relay();
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
    ESP_LOGI(TAG, "Received request to close door. Closing.");
  } else {
    kick_door_relay();
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING);
    ESP_LOGI(TAG, "Received request to open door. Opening.");
  }
  esp_timer_start_once(oneshot_timer, DOOR_MOVING_MAXTIME);
}

void door_control_init(void) {
  gpio_config_t io_conf = {0};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  gpio_config(&io_conf);

  gpio_set_level(RUNNING_LED_GPIO, 1);
  gpio_set_level(DOOR_LED_GPIO, 0);
  gpio_set_level(DOOR_RELAY_GPIO, 0);

  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(DOOR_SENSOR_GPIO, gpio_isr_handler, (void *) DOOR_SENSOR_GPIO);
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

  esp_timer_create(&oneshot_timer_args, &oneshot_timer);

  uint8_t sensor = read_door_sensor();
  ESP_LOGI(TAG, "Initial door state is: (%s)", sensor == DOOR_OPEN ? "open" : "closed");
  if (sensor == DOOR_OPEN) {
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
    gpio_set_level(DOOR_LED_GPIO, 1);
  } else {
    door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
  }
}

/* Ignore reed edges that arrive within this window of the last accepted edge. */
#define DOOR_DEBOUNCE_US 50000

void door_control_run_loop(void) {
  uint32_t io_num;
  uint8_t last_sensor = read_door_sensor();
  int64_t last_change_us = 0;

  while (1) {
    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY) == pdFALSE) {
      ESP_LOGI(TAG, "Door trigger FAIL");
      continue;
    }

    int64_t now = esp_timer_get_time();
    if (now - last_change_us < DOOR_DEBOUNCE_US) {
      continue;  // debounce: too soon after the last accepted edge
    }

    uint8_t sensor = read_door_sensor();
    if (sensor == last_sensor) {
      continue;  // spurious edge, no real change
    }
    last_sensor = sensor;
    last_change_us = now;

    ESP_LOGI(TAG, "Door sensor %s (state was %s)",
             sensor == DOOR_OPEN ? "open" : "closed",
             door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN ? "open" : "not-open");

    if (sensor == DOOR_OPEN) {
      gpio_set_level(DOOR_LED_GPIO, 1);
      hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
      hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
      door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    } else {
      gpio_set_level(DOOR_LED_GPIO, 0);
      hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
      hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
      door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    }
  }
}

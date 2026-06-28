//
// Homekit garage opener — door state machine, DHT sensor, task entry points.
// HomeKit (HAP) I/O lives in hap_garage.{c,h}.
//
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "board.h"
#include "hap_garage.h"
#include "door_control.h"
#include "dht_sensor.h"

static const char *TAG = "HAP Garage";

#define GARAGE_TASK_PRIORITY  1
#define GARAGE_TASK_STACKSIZE 4 * 1024
#define GARAGE_TASK_NAME      "hap_garage"

#define DHT_TASK_PRIORITY  1
#define DHT_TASK_STACKSIZE 4 * 1024
#define DHT_TASK_NAME      "hap_dht"

//
// Global variables
//
static QueueHandle_t gpio_evt_queue = NULL;

uint8_t door_state = 0;

esp_timer_handle_t oneshot_timer;

// Forward declarations
static void timer_callback(void *args);
uint8_t read_door_sensor(void);
void set_door_state(uint8_t state);

// One shot timer definition
const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &timer_callback,
        /* argument specified here will be passed to timer callback function */
        .name = "one-shot"
};

//
// Timer callback when timer expires. Setting state according to door sensor.
//
static void timer_callback(void *args) {
  ESP_LOGI(TAG, "Timer expired. Setting state according to sensor.");

  if(read_door_sensor() == DOOR_OPEN) {
    // Open
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
  } else {
    // Closed
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
  }
}

//
// Kick door relay. Will open or close door.
//
static void kick_door_relay(void) {
  // Close relay
  gpio_set_level(DOOR_RELAY_GPIO, 1);

  // Wait for a while
  vTaskDelay(300 / portTICK_PERIOD_MS);

  // Open relay
  gpio_set_level(DOOR_RELAY_GPIO, 0);
}

//
// Door sensor interrupt
//
static void gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;

    // Send to queue
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

//
// GPIO setup
//
static void setup_gpio(void)
{
  //
  // Set config for outputs
  //
  gpio_config_t io_conf = {0};
  //disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO15/16
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  // Set initial value
  gpio_set_level(RUNNING_LED_GPIO, 1);
  gpio_set_level(DOOR_LED_GPIO, 0);
  gpio_set_level(DOOR_RELAY_GPIO, 0);

  //
  // Set config for inputs
  //
  //interrupt of rising edge
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  //bit mask of the pins, use GPIO4/5 here
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  /* Disable internal pull-down */
  io_conf.pull_down_en = 0;
  gpio_config(&io_conf);

  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(DOOR_SENSOR_GPIO, gpio_isr_handler, (void *) DOOR_SENSOR_GPIO);

  //create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
}

//
// Read door sensor status
//
uint8_t read_door_sensor(void) {
  return (gpio_get_level(DOOR_SENSOR_GPIO)^1);
}

//
// Read door state
//
uint8_t get_door_state(void) {
  return (door_state);
}

//
// Set door state
//
void set_door_state(uint8_t state) {
  door_state = state;
}

/* Temporary shims — removed in Tasks 4–5 when these move to their modules. */
uint8_t door_get_state(void) { return get_door_state(); }

//
// Handle a HomeKit open/close request (kicks relay, arms timer).
// Temporary shim — moves to door_control.c in Task 5.
//
void door_handle_homekit_request(void) {
  if((read_door_sensor() == DOOR_OPEN) || (door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING)) {
    // Door open
    kick_door_relay();
    set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
    ESP_LOGI(TAG, "Received request to close door. Closing.");
    esp_timer_start_once(oneshot_timer, DOOR_MOVING_MAXTIME);

  } else {
    // Door closed
    kick_door_relay();
    set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING);
    ESP_LOGI(TAG, "Received request to open door. Opening.");
    esp_timer_start_once(oneshot_timer, DOOR_MOVING_MAXTIME);
  }
}

//
// Initialize door GPIO + ISR + queue, create timer, and publish the initial state.
// Temporary shim — moves to door_control.c in Task 5.
//
void door_control_init(void) {
  uint8_t current_sensor_value;

  // Setup GPIO
  setup_gpio();

  // Create one-shot timer for door-move failsafe
  esp_timer_create(&oneshot_timer_args, &oneshot_timer);

  //
  // Set initial door state
  //
  current_sensor_value = read_door_sensor();
  ESP_LOGI(TAG, "Initial door state is: (%s)", (current_sensor_value == DOOR_OPEN)?"open":"closed");
  if(current_sensor_value == DOOR_OPEN) {
    // Door Open
    set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
  } else {
    // Door Closed
    set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
  }
}

//
// Run the reed-sensor event loop forever (call from the garage task after hap_garage_start_wifi).
// Temporary shim — moves to door_control.c in Task 5.
//
void door_control_run_loop(void) {
    uint32_t io_num = DOOR_SENSOR_GPIO;
    uint8_t current_sensor_value;
    uint8_t old_sensor_value;

    // Snapshot initial sensor value to detect changes
    old_sensor_value = read_door_sensor();

    //
    // Listen for magnetic sensor state change events. Other read/write functionality will be handled
    // by the HAP Core.
    //
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY) == pdFALSE) {
            ESP_LOGI(TAG, "Door trigger FAIL");
        } else {
            // Read sensor
            current_sensor_value = read_door_sensor();

            // This will remove contact bounces in a bit unreliable way
            if(current_sensor_value != old_sensor_value) {
              ESP_LOGI(TAG, "Door sensor %s, Door state: %s", (current_sensor_value == DOOR_OPEN)?"open":"closed", door_state_to_string(door_get_state()));
              old_sensor_value = current_sensor_value;
              if(current_sensor_value == DOOR_OPEN) {
                //
                // Door open, opening or closing
                //
                gpio_set_level(DOOR_LED_GPIO, 1);
                if(door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING) {
                  // Door is opening on request from homekit. Set it to be open.
                  hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
                  hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
                  ESP_LOGI(TAG, "Door is opening on request from homekit. Set it to be open. (State: %s, Sensor: %s)", door_state_to_string(door_get_state()), current_sensor_value?"Open":"Closed");

                } else if(door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING) {
                  // Door is closing on request from homekit.
                  hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
                  hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
                  ESP_LOGI(TAG, "Door is closing on request from homekit. Setting it to be closing. (State: %s, Sensor: %s)", door_state_to_string(door_get_state()), current_sensor_value?"Open":"Closed");

                } else if(door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN) {
                  // Door is already open in homekit.
                  hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
                  hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
                  ESP_LOGI(TAG, "Door is already open in homekit. Doing nothing. Setting top open again. (State: %s, Sensor: %s)", door_state_to_string(door_get_state()), current_sensor_value?"Open":"Closed");

                } else {
                  // Door has been opened manually.
                  hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
                  hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
                  ESP_LOGI(TAG, "Door has been opened manually. Setting it to be open. (State: %s, Sensor: %s)", door_state_to_string(door_get_state()), current_sensor_value?"Open":"Closed");
                }
              } else {

                //
                // Door closed
                //
                gpio_set_level(DOOR_LED_GPIO, 0);
                if(door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING) {
                  // Door has probably been closed manually (override).
                  hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
                  hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  ESP_LOGI(TAG, "Door has probably been closed manually (overridden) when trying to open by homekit. (State: %s, Sensor: %s)", door_state_to_string(door_get_state()), current_sensor_value?"Open":"Closed");

                } else if(door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING) {
                  // Door is closing on request from homekit.
                  hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
                  hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  ESP_LOGI(TAG, "Door is closing on request from homekit. Setting it to be closed. (State: %s, Sensor: %s)", door_state_to_string(door_get_state()), current_sensor_value?"Open":"Closed");

                } else if(door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN) {
                  // Door has probably been closed manually.
                  hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
                  hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  ESP_LOGI(TAG, "Door has probably been closed manually while open. Setting it to be closed. (State: %s, Sensor: %s)", door_state_to_string(door_get_state()), current_sensor_value?"Open":"Closed");

                } else {
                  // Door has probably been closed manually.
                  hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
                  hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  ESP_LOGI(TAG, "Door has probably been closed manually. Setting it to be closed. (State: %s, Sensor: %s)", door_state_to_string(door_get_state()), current_sensor_value?"Open":"Closed");
                }
              }
            }
        }
    }
}

//
// Main thread for garage accessory
//
static void garage_thread_entry(void *p) {
  hap_garage_init();       // HAP accessory + services + hap_start (non-blocking)
  door_control_init();     // GPIO + timer + initial state
  hap_garage_start_wifi(); // blocks until Wi-Fi connected
  door_control_run_loop(); // never returns
}

//
// Start here
//
void app_main(void)
{
  xTaskCreate(garage_thread_entry, GARAGE_TASK_NAME, GARAGE_TASK_STACKSIZE, NULL, GARAGE_TASK_PRIORITY, NULL);
  dht_sensor_start();
}

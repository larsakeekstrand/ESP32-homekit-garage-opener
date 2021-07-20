//
// Homekit garage opener.
//
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#include <hap_fw_upgrade.h>
#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#include <dht.h>

static const dht_sensor_type_t sensor_type = DHT_TYPE_AM2301;

/*  Required for server verification during OTA, PEM format as string  */
char server_cert[] = {};

static const char *TAG = "HAP Garage";

#define GARAGE_TASK_PRIORITY  1
#define GARAGE_TASK_STACKSIZE 4 * 1024
#define GARAGE_TASK_NAME      "hap_garage"

#define DHT_TASK_PRIORITY  1
#define DHT_TASK_STACKSIZE 4 * 1024
#define DHT_TASK_NAME      "hap_dht"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO  GPIO_NUM_0

#define RUNNING_LED_GPIO  GPIO_NUM_2
#define DOOR_LED_GPIO  GPIO_NUM_16
#define DOOR_RELAY_GPIO  GPIO_NUM_5
#define DOOR_SENSOR_GPIO  GPIO_NUM_4

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<RUNNING_LED_GPIO) | (1ULL<<DOOR_LED_GPIO) | (1ULL<<DOOR_RELAY_GPIO))
#define GPIO_INPUT_PIN_SEL  (1ULL<<DOOR_SENSOR_GPIO)

#define DHT_SENSOR_GPIO  GPIO_NUM_22

#define ESP_INTR_FLAG_DEFAULT 0

#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN 0
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED 1
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING 2
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING 3
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED 4
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN 255

#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN 0
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED 1
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN 255

#define DOOR_OPEN 1
#define DOOR_CLOSED 0

#define DOOR_MOVING_MAXTIME 8000000

static xQueueHandle gpio_evt_queue = NULL;

uint8_t door_state = 0;

// Function Definitions
static void timer_callback(void *args);
uint8_t read_door_sensor();
void send_target_door_state(uint8_t state);
void send_current_door_state(uint8_t state);
void set_door_state(uint8_t state);
//
// Global variables
//
esp_timer_handle_t oneshot_timer;
hap_acc_t *accessory;

float current_temperature = 0.0;
float current_humidity = 0.0;


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
    send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
    send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
  } else {
    // Closed
    send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
    send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
    set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
  }
}

//
// Kick door relay. Will open or close door.
//
static void kick_door_relay() {
  // Close relay
  gpio_set_level(DOOR_RELAY_GPIO, 1);

  // Wait for a while
  vTaskDelay(300 / portTICK_RATE_MS);

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
static void setup_gpio()
{
  //
  // Set config for outputs
  //
  gpio_config_t io_conf;
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

/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    ESP_LOGI(TAG, "Reset network");
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    ESP_LOGI(TAG, "Reset to factory");
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

//
// Mandatory identify routine for the accessory.
//
// TODO: Maybe add a LED blink later?
//
static int garage_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

//
// An optional HomeKit Event handler which can be used to track HomeKit
// specific events.
//
static void garage_hap_event_handler(void* arg, esp_event_base_t event_base, int event, void *data)
{
    switch(event) {
        case HAP_EVENT_PAIRING_STARTED :
            ESP_LOGI(TAG, "Pairing Started");
            break;
        case HAP_EVENT_PAIRING_ABORTED :
            ESP_LOGI(TAG, "Pairing Aborted");
            break;
        case HAP_EVENT_CTRL_PAIRED :
            ESP_LOGI(TAG, "Controller %s Paired. Controller count: %d",
                        (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_UNPAIRED :
            ESP_LOGI(TAG, "Controller %s Removed. Controller count: %d",
                        (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_CONNECTED :
            ESP_LOGI(TAG, "Controller %s Connected", (char *)data);
            break;
        case HAP_EVENT_CTRL_DISCONNECTED :
            ESP_LOGI(TAG, "Controller %s Disconnected", (char *)data);
            break;
        case HAP_EVENT_ACC_REBOOTING : {
            char *reason = (char *)data;
            ESP_LOGI(TAG, "Accessory Rebooting (Reason: %s)",  reason ? reason : "null");
            break;
        }
        default:
            /* Silently ignore unknown events */
            break;
    }
}

//
// Read door sensor status
//
uint8_t read_door_sensor() {
  return (gpio_get_level(DOOR_SENSOR_GPIO)^1);
}

//
// Read door state
//
uint8_t get_door_state() {
  return (door_state);
}

//
// Convert door state to string
//
char* door_state_to_string(uint8_t state) {
  switch(state) {
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN: return "open"; break;
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED: return "closed"; break;
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING: return "opening"; break;
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING: return "closing"; break;
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED: return "stopped"; break;
    default: return "unknown"; break;
  }
}

//
// Send update of current door state
//
void send_current_door_state(uint8_t state) {
  hap_val_t door_value = {
      .u = state,
  };
  hap_serv_t *service;
  hap_char_t *handle;

  service = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_GARAGE_DOOR_OPENER);
  handle = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CURRENT_DOOR_STATE);

  ESP_LOGI(TAG, "Sending current doorstate [%s]", door_state_to_string(state));
  hap_char_update_val(handle, &door_value);
}


//
// Send update of target door state
//
void send_target_door_state(uint8_t state) {
  hap_val_t door_value = {
      .u = state,
  };
  hap_serv_t *service;
  hap_char_t *handle;

  service = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_GARAGE_DOOR_OPENER);
  handle = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_TARGET_DOOR_STATE);
  ESP_LOGI(TAG, "Sending target doorstate [%s]", door_state_to_string(state));
  hap_char_update_val(handle, &door_value);
}

//
// Send update of current temperature
//
void send_current_temperature() {
  hap_val_t temperature = {
      .f = current_temperature,
  };
  hap_serv_t *service;
  hap_char_t *handle;

  service = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_TEMPERATURE_SENSOR);
  handle = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CURRENT_TEMPERATURE);

  ESP_LOGI(TAG, "Sending current temperature [%f]", temperature.f);
  hap_char_update_val(handle, &temperature);
}

//
// Send update of current humidity
//
void send_current_humidity() {
  hap_val_t humidity = {
      .f = current_humidity,
  };
  hap_serv_t *service;
  hap_char_t *handle;

  service = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_HUMIDITY_SENSOR);
  handle = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY);

  ESP_LOGI(TAG, "Sending current humidity [%f]", humidity.f);
  hap_char_update_val(handle, &humidity);
}

//
// Set door state
//
void set_door_state(uint8_t state) {
  door_state = state;
}

//
// Incoming read
//
static int garage_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
  const hap_val_t *cur_val = hap_char_get_val(hc);
  hap_val_t new_val;

  if (hap_req_get_ctrl_id(read_priv)) {
    ESP_LOGI(TAG, "Received read from %s", hap_req_get_ctrl_id(read_priv));
    ESP_LOGI(TAG, "Received hap_char_get_type_uuid: %s", hap_char_get_type_uuid(hc));
  }

  if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_TARGET_DOOR_STATE)) {

    // Get the door state

    if(get_door_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN || get_door_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING ) {

      new_val.u = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN;
    } else {
      new_val.u = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED;
    }

    ESP_LOGI(TAG, "Target door state : hap_char_get_val: %d", new_val.u);

    hap_char_update_val(hc, &new_val);

    *status_code = HAP_STATUS_SUCCESS;
  }

  if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_DOOR_STATE)) {

    new_val.u = get_door_state();

    ESP_LOGI(TAG, "Current door state : hap_char_get_val: %d", new_val.u);
    hap_char_update_val(hc, &new_val);

    *status_code = HAP_STATUS_SUCCESS;
  }

  if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_OBSTRUCTION_DETECTED)) {
   /* Read the current value, toggle it and set the new value.
    * A separate variable should be used for the new value, as the hap_char_get_val()
    * API returns a const pointer
    */

    ESP_LOGI(TAG, "obstruction state : hap_char_get_val: %d", cur_val->u);

    hap_char_update_val(hc, cur_val);
    *status_code = HAP_STATUS_SUCCESS;
  }

  return HAP_SUCCESS;
}


static int dht_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv) {
  const hap_val_t *cur_val = hap_char_get_val(hc);
  //hap_val_t new_val;

  //new_val.f = temperature;
  //hap_char_update_val(hc, &new_val);

  //*status_code = HAP_STATUS_SUCCESS;

  return HAP_SUCCESS;
}

//
// Incoming write
//
static int garage_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
  int ret = HAP_SUCCESS;

  if((read_door_sensor() == DOOR_OPEN) || (get_door_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING)) {
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


  return ret;
}

//
// Main thread for garage accessory
//
static void garage_thread_entry(void *p)
{
    //hap_acc_t *accessory;
    hap_serv_t *service;
    hap_serv_t *temp_service;
    hap_serv_t *hum_service;

    uint8_t current_sensor_value, old_sensor_value;

    // Setup GPIO
    setup_gpio();

    //
    // Configure HomeKit core to make the Accessory name (and thus the WAC SSID) unique,
    // instead of the default configuration wherein only the WAC SSID is made unique.
    //
    hap_cfg_t hap_cfg;
    hap_get_config(&hap_cfg);
    hap_cfg.unique_param = UNIQUE_NAME;
    hap_set_config(&hap_cfg);

    // Initialize the HAP core
    hap_init(HAP_TRANSPORT_WIFI);

    //
    // Initialise the mandatory parameters for Accessory which will be added as
    // the mandatory services internally
    //
    hap_acc_cfg_t cfg = {
        .name = "GarageOpener",
        .manufacturer = "Ekstrand",
        .model = "Garage01",
        .serial_num = "111111",
        .fw_rev = "1.0.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = garage_identify,
        .cid = HAP_CID_GARAGE_DOOR_OPENER,
    };
    // Create accessory object
    accessory = hap_acc_create(&cfg);

    // Add a dummy Product Data
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    service = hap_serv_garage_door_opener_create(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN, HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN, false);

    hap_serv_add_char(service, hap_char_name_create("GarageOpener"));

    //Set the write callback for the service
    hap_serv_set_write_cb(service, garage_write);

    //Set the read callback for the service
    hap_serv_set_read_cb(service, garage_read);

    // Add the service to the Accessory Object
    hap_acc_add_serv(accessory, service);

    temp_service = hap_serv_temperature_sensor_create((float)10);
    hap_serv_add_char(temp_service, hap_char_name_create("Temperature"));
    hap_serv_set_read_cb(temp_service, dht_read);
    hap_acc_add_serv(accessory, temp_service);
    hum_service = hap_serv_humidity_sensor_create((float)20);
    hap_serv_add_char(hum_service, hap_char_name_create("Humidity"));
    hap_serv_set_read_cb(hum_service, dht_read);
    hap_acc_add_serv(accessory, hum_service);

    /* Create the Firmware Upgrade HomeKit Custom Service.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    hap_fw_upgrade_config_t ota_config = {
        .server_cert_pem = server_cert,
    };
    service = hap_serv_fw_upgrade_create(&ota_config);

    // Add the service to the Accessory Object
    hap_acc_add_serv(accessory, service);

    // Add the Accessory to the HomeKit Database
    hap_add_accessory(accessory);

    // Register a common button for reset Wi-Fi network and reset to factory.
    reset_key_init(RESET_GPIO);

    // Query the controller count (just for information)
    ESP_LOGI(TAG, "Accessory is paired with %d controllers",
                hap_get_paired_controller_count());

    /* TODO: Do the actual hardware initialization here */

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* Initialize Wi-Fi */
    app_wifi_init();

    /* Register an event handler for HomeKit specific events.
     * All event handlers should be registered only after app_wifi_init()
     */
    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, &garage_hap_event_handler, NULL);

    /* After all the initializations are done, start the HAP core */
    hap_start();

    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);

    // Cleanup timer_handler
    esp_timer_create(&oneshot_timer_args, &oneshot_timer);
    //esp_timer_start_once(oneshot_timer, DOOR_MOVING_MAXTIME);

    uint32_t io_num = DOOR_SENSOR_GPIO;

    //
    // Set initial door state
    //
    current_sensor_value = read_door_sensor();
    old_sensor_value = current_sensor_value;
    ESP_LOGI(TAG, "Initial door state is: (%s)", (current_sensor_value == DOOR_OPEN)?"open":"closed");
    if(current_sensor_value == DOOR_OPEN) {
      // Door Open

      set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
      send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
      send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
    } else {
      // Door Closed
      set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
      send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
      send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
    }

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
              ESP_LOGI(TAG, "Door sensor %s, Door state: %s", (current_sensor_value == DOOR_OPEN)?"open":"closed", door_state_to_string(get_door_state()));
              old_sensor_value = current_sensor_value;
              if(current_sensor_value == DOOR_OPEN) {
                //
                // Door open, opening or closing
                //
                gpio_set_level(DOOR_LED_GPIO, 1);
                if(get_door_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING) {
                  // Door is opening on request from homekit. Set it to be open.
                  send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
                  send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
                  ESP_LOGI(TAG, "Door is opening on request from homekit. Set it to be open. (State: %s, Sensor: %s)", door_state_to_string(get_door_state()), current_sensor_value?"Open":"Closed");

                } else if(get_door_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING) {
                  // Door is closing on request from homekit.
                  send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
                  send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
                  ESP_LOGI(TAG, "Door is closing on request from homekit. Setting it to be closing. (State: %s, Sensor: %s)", door_state_to_string(get_door_state()), current_sensor_value?"Open":"Closed");

                } else if(get_door_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN) {
                  // Door is already open in homekit.
                  send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
                  send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
                  ESP_LOGI(TAG, "Door is already open in homekit. Doing nothing. Setting top open again. (State: %s, Sensor: %s)", door_state_to_string(get_door_state()), current_sensor_value?"Open":"Closed");

                } else {
                  // Door has been opened manually.
                  send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN);
                  send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);

                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
                  ESP_LOGI(TAG, "Door has been opened manually. Setting it to be open. (State: %s, Sensor: %s)", door_state_to_string(get_door_state()), current_sensor_value?"Open":"Closed");

                }
              } else {

                //
                // Door closed
                //
                gpio_set_level(DOOR_LED_GPIO, 0);
                if(get_door_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING) {
                  // Door has probably been closed manually (override).
                  send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
                  send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  ESP_LOGI(TAG, "Door has probably been closed manually (overridden) when trying to open by homekit. (State: %s, Sensor: %s)", door_state_to_string(get_door_state()), current_sensor_value?"Open":"Closed");

                } else if(get_door_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING) {
                  // Door is closing on request from homekit.
                  send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
                  send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  ESP_LOGI(TAG, "Door is closing on request from homekit. Setting it to be closed. (State: %s, Sensor: %s)", door_state_to_string(get_door_state()), current_sensor_value?"Open":"Closed");

                } else if(get_door_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN) {
                  // Door has probably been closed manually.
                  send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
                  send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  ESP_LOGI(TAG, "Door has probably been closed manually while open. Setting it to be closed. (State: %s, Sensor: %s)", door_state_to_string(get_door_state()), current_sensor_value?"Open":"Closed");

                } else {
                  // Door has probably been closed manually.
                  send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
                  send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  set_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
                  ESP_LOGI(TAG, "Door has probably been closed manually. Setting it to be closed. (State: %s, Sensor: %s)", door_state_to_string(get_door_state()), current_sensor_value?"Open":"Closed");
                }
              }
            }
        }
    }


    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}

//
// Temp and humidity read task
//
static void dht_thread_entry(void *p)
{

  while (1) {

    if (dht_read_float_data(sensor_type, DHT_SENSOR_GPIO, &current_humidity, &current_temperature) == ESP_OK)
      ESP_LOGI(TAG, "Read Temp: %0.01fC Humidity: %0.01f%% ", current_humidity, current_temperature);
    else
      ESP_LOGE(TAG, "Could not read data from DHT sensor on GPIO %d", DHT_SENSOR_GPIO);

    send_current_temperature();
    send_current_humidity();
    // Sleep for a while
    vTaskDelay(30000 / portTICK_RATE_MS);
  }

  /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
  vTaskDelete(NULL);

}

//
// Start here
//
void app_main()
{
    xTaskCreate(garage_thread_entry, GARAGE_TASK_NAME, GARAGE_TASK_STACKSIZE, NULL, GARAGE_TASK_PRIORITY, NULL);
    xTaskCreate(dht_thread_entry, DHT_TASK_NAME, DHT_TASK_STACKSIZE, NULL, DHT_TASK_PRIORITY, NULL);
}

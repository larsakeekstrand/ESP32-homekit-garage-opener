//
// hap_garage.c — HomeKit accessory and all HAP I/O.
//
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_event.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>
#include <iot_button.h>
#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#include "hap_garage.h"
#include "door_control.h"
#include "dht_sensor.h"
#include "board.h"

static const char *TAG = "HAP Garage";

static hap_acc_t *accessory;

/* Reset button timeouts. */
#define RESET_NETWORK_BUTTON_TIMEOUT    3
#define RESET_TO_FACTORY_BUTTON_TIMEOUT 10

//
// Convert door state to string
//
static char *door_state_to_string(uint8_t state) {
  switch(state) {
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN:    return "open";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED:  return "closed";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING: return "opening";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING: return "closing";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED: return "stopped";
    default:                                                 return "unknown";
  }
}

//
// Send update of current door state
//
void hap_garage_send_current_door_state(uint8_t state) {
  hap_val_t door_value = { .u = state };
  hap_serv_t *service = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_GARAGE_DOOR_OPENER);
  hap_char_t *handle  = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CURRENT_DOOR_STATE);
  ESP_LOGI(TAG, "Sending current doorstate [%s]", door_state_to_string(state));
  hap_char_update_val(handle, &door_value);
}

//
// Send update of target door state
//
void hap_garage_send_target_door_state(uint8_t state) {
  hap_val_t door_value = { .u = state };
  hap_serv_t *service = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_GARAGE_DOOR_OPENER);
  hap_char_t *handle  = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_TARGET_DOOR_STATE);
  ESP_LOGI(TAG, "Sending target doorstate [%s]", door_state_to_string(state));
  hap_char_update_val(handle, &door_value);
}

//
// Send update of current temperature
//
void hap_garage_send_current_temperature(float temperature) {
  hap_val_t v = { .f = temperature };
  hap_serv_t *s = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_TEMPERATURE_SENSOR);
  hap_char_t *h = hap_serv_get_char_by_uuid(s, HAP_CHAR_UUID_CURRENT_TEMPERATURE);
  ESP_LOGI(TAG, "Sending current temperature [%f]", temperature);
  hap_char_update_val(h, &v);
}

//
// Send update of current humidity
//
void hap_garage_send_current_humidity(float humidity) {
  hap_val_t v = { .f = humidity };
  hap_serv_t *s = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_HUMIDITY_SENSOR);
  hap_char_t *h = hap_serv_get_char_by_uuid(s, HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY);
  ESP_LOGI(TAG, "Sending current humidity [%f]", humidity);
  hap_char_update_val(h, &v);
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
static void garage_hap_event_handler(void* arg, esp_event_base_t event_base, int32_t event, void *data)
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
// Incoming read
//
static int garage_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
  hap_val_t new_val;

  if (status_code) *status_code = HAP_STATUS_SUCCESS;

  if (hap_req_get_ctrl_id(read_priv)) {
    ESP_LOGI(TAG, "Received read from %s", hap_req_get_ctrl_id(read_priv));
    ESP_LOGI(TAG, "Received hap_char_get_type_uuid: %s", hap_char_get_type_uuid(hc));
  }

  if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_TARGET_DOOR_STATE)) {

    if (door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN ||
        door_get_state() == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING) {
      new_val.u = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN;
    } else {
      new_val.u = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED;
    }

    ESP_LOGI(TAG, "Target door state : hap_char_get_val: %d", new_val.u);
    hap_char_update_val(hc, &new_val);
    *status_code = HAP_STATUS_SUCCESS;
  }

  if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_DOOR_STATE)) {
    new_val.u = door_get_state();
    ESP_LOGI(TAG, "Current door state : hap_char_get_val: %d", new_val.u);
    hap_char_update_val(hc, &new_val);
    *status_code = HAP_STATUS_SUCCESS;
  }

  if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_OBSTRUCTION_DETECTED)) {
    /* Copy the current value into a non-const local; hap_char_update_val()
     * requires a non-const pointer even when the value is passed through unchanged. */
    hap_val_t obs_val = *hap_char_get_val(hc);
    ESP_LOGI(TAG, "obstruction state : hap_char_get_val: %d", obs_val.u);
    hap_char_update_val(hc, &obs_val);
    *status_code = HAP_STATUS_SUCCESS;
  }

  return HAP_SUCCESS;
}

//
// Incoming write (delegates to door_control)
//
static int garage_write(hap_write_data_t write_data[], int count,
                        void *serv_priv, void *write_priv) {
  door_handle_homekit_request();
  return HAP_SUCCESS;
}

//
// DHT read callback — serve cached sensor values (fixes spec bug: was a no-op).
//
static int dht_read(hap_char_t *hc, hap_status_t *status_code,
                    void *serv_priv, void *read_priv) {
  float t = 0, h = 0;
  dht_sensor_get(&t, &h);
  hap_val_t v;
  const char *uuid = hap_char_get_type_uuid(hc);
  if (!strcmp(uuid, HAP_CHAR_UUID_CURRENT_TEMPERATURE)) { v.f = t; hap_char_update_val(hc, &v); }
  else if (!strcmp(uuid, HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY)) { v.f = h; hap_char_update_val(hc, &v); }
  if (status_code) *status_code = HAP_STATUS_SUCCESS;
  return HAP_SUCCESS;
}

//
// Initialize HAP accessory, services, callbacks, and start HAP (non-blocking).
// Call from the garage task before hap_garage_start_wifi().
//
void hap_garage_init(void) {
    hap_serv_t *service;
    hap_serv_t *temp_service;
    hap_serv_t *hum_service;

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
        .fw_rev = FW_VERSION,
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

    service = hap_serv_garage_door_opener_create(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN,
                                                 HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN,
                                                 false);

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

    // Add the Accessory to the HomeKit Database
    hap_add_accessory(accessory);

    // Register a common button for reset Wi-Fi network and reset to factory.
    reset_key_init(RESET_GPIO);

    // Query the controller count (just for information)
    ESP_LOGI(TAG, "Accessory is paired with %d controllers",
                hap_get_paired_controller_count());

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

}

//
// Start the HAP core (call from the garage task after door_control_init).
//
void hap_garage_start(void) {
    hap_start();
}

//
// Start Wi-Fi and block until connected (call from the garage task after hap_garage_start).
//
void hap_garage_start_wifi(void) {
    app_wifi_start(portMAX_DELAY);
}

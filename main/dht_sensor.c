//
// dht_sensor.c — DHT22/AM2301 polling task and cached readings.
//
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <dht.h>

#include "dht_sensor.h"
#include "hap_garage.h"
#include "board.h"

static const char *TAG = "HAP Garage";
static const dht_sensor_type_t sensor_type = DHT_TYPE_AM2301;

static float current_temperature = 0.0f;
static float current_humidity = 0.0f;

void dht_sensor_get(float *temperature, float *humidity) {
  *temperature = current_temperature;
  *humidity = current_humidity;
}

static void dht_thread_entry(void *p) {
  while (1) {
    if (dht_read_float_data(sensor_type, DHT_SENSOR_GPIO, &current_humidity, &current_temperature) == ESP_OK)
      ESP_LOGI(TAG, "Read Temp: %0.01fC Humidity: %0.01f%%", current_temperature, current_humidity);  // fixed arg order
    else
      ESP_LOGE(TAG, "Could not read data from DHT sensor on GPIO %d", DHT_SENSOR_GPIO);

    hap_garage_send_current_temperature(current_temperature);
    hap_garage_send_current_humidity(current_humidity);
    vTaskDelay(30000 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void dht_sensor_start(void) {
  xTaskCreate(dht_thread_entry, "hap_dht", 4*1024, NULL, 1, NULL);
}

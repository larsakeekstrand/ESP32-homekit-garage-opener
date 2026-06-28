//
// Homekit garage opener — task entry points.
// HomeKit (HAP) I/O lives in hap_garage.{c,h}.
// Door state machine, relay, and sensor live in door_control.{c,h}.
//
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "hap_garage.h"
#include "door_control.h"
#include "dht_sensor.h"

#define GARAGE_TASK_PRIORITY  1
#define GARAGE_TASK_STACKSIZE 4 * 1024
#define GARAGE_TASK_NAME      "hap_garage"

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

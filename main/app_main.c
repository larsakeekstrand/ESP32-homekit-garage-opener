//
// app_main.c — composition root: wires modules and starts the tasks.
//
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "hap_garage.h"
#include "door_control.h"
#include "dht_sensor.h"

#define GARAGE_TASK_STACKSIZE (4 * 1024)

/* Garage task: HAP + Wi-Fi lifecycle, then the reed-sensor event loop. */
static void garage_thread_entry(void *p) {
  hap_garage_init();
  door_control_init();
  hap_garage_start_wifi();   // blocking
  door_control_run_loop();   // never returns
}

void app_main(void) {
  xTaskCreate(garage_thread_entry, "hap_garage", GARAGE_TASK_STACKSIZE, NULL, 1, NULL);
  dht_sensor_start();
}

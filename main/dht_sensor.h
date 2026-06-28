//
// dht_sensor.h — DHT22/AM2301 temperature + humidity.
//
#pragma once

/* Spawn the polling task. */
void dht_sensor_start(void);

/* Copy the latest cached readings (thread-safe enough on single-core). */
void dht_sensor_get(float *temperature, float *humidity);

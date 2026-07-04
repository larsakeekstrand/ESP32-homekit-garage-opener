//
// hap_garage.h — HomeKit (HAP) accessory: services, callbacks, notifications.
//
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Current door state (mirrors HAP spec enum). */
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN    0
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED  1
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING 2
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING 3
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED 4
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN 255

/* Target door state. */
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN    0
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED  1
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN 255

/* Lifecycle. */
void hap_garage_init(void);
void hap_garage_start(void);
void hap_garage_start_wifi(void);

/* Push updates to HomeKit. */
void hap_garage_send_current_door_state(uint8_t state);
void hap_garage_send_target_door_state(uint8_t state);
void hap_garage_send_current_temperature(float temperature);
void hap_garage_send_current_humidity(float humidity);

/* Left-open Contact Sensor: true = tripped (contact "open"). */
void hap_garage_send_left_open(bool tripped);
/* Garage Door StatusFault: 0 = no fault, 1 = general fault. */
void hap_garage_send_status_fault(uint8_t fault);
/* Lockout Switch: reflect an externally-driven (web) change back to HomeKit. */
void hap_garage_send_lockout(bool on);

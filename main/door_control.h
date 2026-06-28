//
// door_control.h — door state machine, relay, reed sensor.
//
#pragma once

#include <stdint.h>

/* Door open/closed sensor logical values. */
#define DOOR_OPEN   1
#define DOOR_CLOSED 0

/* Failsafe: max time a move may take before we trust the sensor (µs). */
#define DOOR_MOVING_MAXTIME 8000000

/* Current logical door state (HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_*). */
uint8_t door_get_state(void);

/* Handle an open/close request arriving from HomeKit (kicks relay, arms timer). */
void door_handle_homekit_request(void);

/* Initialize door GPIO + ISR + queue and publish the initial state. */
void door_control_init(void);

/* Run the reed-sensor event loop forever (call from the garage task). */
void door_control_run_loop(void);

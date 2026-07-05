//
// settings.h — NVS-backed config, counters, and web password.
//
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Load config/counters/password from NVS into the RAM cache. Call once at boot
 * before any other settings_* function. */
void settings_init(void);

/* Timeouts (seconds). 0 = disabled. Setter clamps to 0 or 30..3600. */
uint32_t settings_get_auto_close_sec(void);
uint32_t settings_get_left_open_sec(void);
void     settings_set_timeouts(uint32_t auto_close_sec, uint32_t left_open_sec);

/* Vacation/lockout flag (persisted). */
bool settings_get_lockout(void);
void settings_set_lockout(bool on);

/* Auto-OTA pause flag (persisted). When true, the GitHub self-poller is
 * suspended so a hand-pushed beta is not reverted. */
bool settings_get_auto_ota_paused(void);
void settings_set_auto_ota_paused(bool paused);

/* Runtime tunables (persisted). Setters clamp to range. */
uint32_t settings_get_ota_poll_sec(void);   /* 300..86400 */
void     settings_set_ota_poll_sec(uint32_t sec);
uint32_t settings_get_door_move_sec(void);  /* 3..30 */
void     settings_set_door_move_sec(uint32_t sec);
uint32_t settings_get_relay_pulse_ms(void); /* 100..2000 */
void     settings_set_relay_pulse_ms(uint32_t ms);

/* Counters — incremented in RAM, flushed to NVS in batches to limit flash wear. */
typedef struct {
  uint32_t cycles;
  uint32_t auto_closes;
  uint32_t left_open_trips;
  uint32_t faults;
} settings_counters_t;

void settings_incr_cycle(void);
void settings_incr_auto_close(void);
void settings_incr_left_open(void);
void settings_incr_fault(void);
void settings_get_counters(settings_counters_t *out);
void settings_flush(void);   /* persist dirty counters now (call periodically) */

/* Web admin password — stored as salt(16) + SHA-256(salt||password). */
bool settings_password_is_set(void);
bool settings_password_check(const char *password);   /* constant-time */
void settings_password_set_value(const char *password);

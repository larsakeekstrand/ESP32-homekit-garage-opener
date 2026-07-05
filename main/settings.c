//
// settings.c — NVS-backed config, counters, and web admin password.
//
#include <string.h>
#include <esp_log.h>
#include <esp_random.h>
#include <nvs_flash.h>
#include <nvs.h>
#include "mbedtls/sha256.h"
#include "sdkconfig.h"
#include "settings.h"

static const char *TAG = "Settings";

#define NS          "garage"   /* NVS namespace */
#define TIMEOUT_MIN 30
#define TIMEOUT_MAX 3600
#define FLUSH_EVERY 10          /* flush counters every N increments */

static uint32_t s_auto_close_sec;
static uint32_t s_left_open_sec;
static bool     s_lockout;
static bool     s_ota_paused;
static uint32_t s_ota_poll_sec;
static uint32_t s_door_move_sec;
static uint32_t s_relay_pulse_ms;
static settings_counters_t s_counters;
static uint8_t  s_pw[48];       /* 16-byte salt + 32-byte SHA-256 */
static bool     s_pw_set;
static uint32_t s_dirty_incrs;

//
// Clamp a timeout to 0 (disabled) or the valid 30..3600 s range.
//
static uint32_t clamp_timeout(uint32_t v) {
  if (v == 0) return 0;
  if (v < TIMEOUT_MIN) return TIMEOUT_MIN;
  if (v > TIMEOUT_MAX) return TIMEOUT_MAX;
  return v;
}

static uint32_t clampu(uint32_t v, uint32_t lo, uint32_t hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

//
// Open the settings NVS namespace; sets *h to 0 on failure.
//
static void ns_open(nvs_handle_t *h, nvs_open_mode_t mode) {
  esp_err_t e = nvs_open(NS, mode, h);
  if (e != ESP_OK) { ESP_LOGW(TAG, "nvs_open: %s", esp_err_to_name(e)); *h = 0; }
}

//
// Compute salt||password SHA-256 into out[32].
//
static void hash_pw(const uint8_t salt[16], const char *pw, uint8_t out[32]) {
  mbedtls_sha256_context c;
  mbedtls_sha256_init(&c);
  mbedtls_sha256_starts(&c, 0);              /* 0 = SHA-256 (not 224) */
  mbedtls_sha256_update(&c, salt, 16);
  mbedtls_sha256_update(&c, (const uint8_t *)pw, strlen(pw));
  mbedtls_sha256_finish(&c, out);
  mbedtls_sha256_free(&c);
}

void settings_init(void) {
  esp_err_t e = nvs_flash_init();
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  /* Defaults (used until NVS overrides them). */
  s_auto_close_sec = CONFIG_AUTO_CLOSE_DEFAULT_SEC;
  s_left_open_sec  = CONFIG_LEFT_OPEN_DEFAULT_SEC;
  s_lockout = false;
  s_ota_paused = false;
  s_ota_poll_sec   = CONFIG_OTA_POLL_INTERVAL_SEC;
  s_door_move_sec  = CONFIG_DOOR_MOVE_TIMEOUT_SEC;
  s_relay_pulse_ms = CONFIG_RELAY_PULSE_MS;
  memset(&s_counters, 0, sizeof(s_counters));
  s_pw_set = false;

  nvs_handle_t h;
  ns_open(&h, NVS_READONLY);
  if (h) {
    nvs_get_u32(h, "auto_close", &s_auto_close_sec);
    nvs_get_u32(h, "left_open",  &s_left_open_sec);
    uint8_t lk = 0;
    if (nvs_get_u8(h, "lockout", &lk) == ESP_OK) s_lockout = lk;
    uint8_t op = 0;
    if (nvs_get_u8(h, "ota_paused", &op) == ESP_OK) s_ota_paused = op;
    nvs_get_u32(h, "ota_poll",  &s_ota_poll_sec);
    nvs_get_u32(h, "door_move", &s_door_move_sec);
    nvs_get_u32(h, "relay_ms",  &s_relay_pulse_ms);
    size_t csz = sizeof(s_counters);
    nvs_get_blob(h, "counters", &s_counters, &csz);
    size_t psz = sizeof(s_pw);
    if (nvs_get_blob(h, "pw", s_pw, &psz) == ESP_OK && psz == sizeof(s_pw)) s_pw_set = true;
    nvs_close(h);
  }
  ESP_LOGI(TAG, "loaded: auto_close=%u left_open=%u lockout=%d ota_paused=%d pw_set=%d",
           s_auto_close_sec, s_left_open_sec, s_lockout, s_ota_paused, s_pw_set);
}

uint32_t settings_get_auto_close_sec(void) { return s_auto_close_sec; }
uint32_t settings_get_left_open_sec(void)  { return s_left_open_sec; }

void settings_set_timeouts(uint32_t ac, uint32_t lo) {
  s_auto_close_sec = clamp_timeout(ac);
  s_left_open_sec  = clamp_timeout(lo);
  nvs_handle_t h;
  ns_open(&h, NVS_READWRITE);
  if (h) {
    nvs_set_u32(h, "auto_close", s_auto_close_sec);
    nvs_set_u32(h, "left_open",  s_left_open_sec);
    nvs_commit(h);
    nvs_close(h);
  }
  ESP_LOGI(TAG, "timeouts set: auto_close=%u left_open=%u", s_auto_close_sec, s_left_open_sec);
}

bool settings_get_lockout(void) { return s_lockout; }

void settings_set_lockout(bool on) {
  s_lockout = on;
  nvs_handle_t h;
  ns_open(&h, NVS_READWRITE);
  if (h) { nvs_set_u8(h, "lockout", on ? 1 : 0); nvs_commit(h); nvs_close(h); }
  ESP_LOGI(TAG, "lockout = %d", on);
}

bool settings_get_auto_ota_paused(void) { return s_ota_paused; }

void settings_set_auto_ota_paused(bool paused) {
  s_ota_paused = paused;
  nvs_handle_t h;
  ns_open(&h, NVS_READWRITE);
  if (h) { nvs_set_u8(h, "ota_paused", paused ? 1 : 0); nvs_commit(h); nvs_close(h); }
  ESP_LOGI(TAG, "auto_ota_paused = %d", paused);
}

uint32_t settings_get_ota_poll_sec(void)   { return s_ota_poll_sec; }
uint32_t settings_get_door_move_sec(void)  { return s_door_move_sec; }
uint32_t settings_get_relay_pulse_ms(void) { return s_relay_pulse_ms; }

static void save_u32(const char *key, uint32_t val) {
  nvs_handle_t h;
  ns_open(&h, NVS_READWRITE);
  if (h) { nvs_set_u32(h, key, val); nvs_commit(h); nvs_close(h); }
}

void settings_set_ota_poll_sec(uint32_t sec) {
  s_ota_poll_sec = clampu(sec, 300, 86400);
  save_u32("ota_poll", s_ota_poll_sec);
  ESP_LOGI(TAG, "ota_poll_sec = %u", s_ota_poll_sec);
}
void settings_set_door_move_sec(uint32_t sec) {
  s_door_move_sec = clampu(sec, 3, 30);
  save_u32("door_move", s_door_move_sec);
  ESP_LOGI(TAG, "door_move_sec = %u", s_door_move_sec);
}
void settings_set_relay_pulse_ms(uint32_t ms) {
  s_relay_pulse_ms = clampu(ms, 100, 2000);
  save_u32("relay_ms", s_relay_pulse_ms);
  ESP_LOGI(TAG, "relay_pulse_ms = %u", s_relay_pulse_ms);
}

//
// Write the counters blob to NVS and reset the dirty count.
//
static void counters_flush(void) {
  nvs_handle_t h;
  ns_open(&h, NVS_READWRITE);
  if (h) {
    nvs_set_blob(h, "counters", &s_counters, sizeof(s_counters));
    nvs_commit(h);
    nvs_close(h);
    s_dirty_incrs = 0;   /* only clear after a successful persist */
  }
}

/* Concurrency: counters + the dirty count are owned by the door task (the only
 * caller of settings_incr_* / settings_flush). Config setters/getters and the
 * password run on the httpd task. NVS has its own internal locking and the
 * config u32s/bool are atomic on this 32-bit target, so no mutex is needed;
 * the only cross-task effect is a possible cosmetic torn read of the counters
 * struct in settings_get_counters, which never corrupts state. */
static void bump(uint32_t *ctr) {
  (*ctr)++;
  if (++s_dirty_incrs >= FLUSH_EVERY) counters_flush();
}

void settings_incr_cycle(void)      { bump(&s_counters.cycles); }
void settings_incr_auto_close(void) { bump(&s_counters.auto_closes); }
void settings_incr_left_open(void)  { bump(&s_counters.left_open_trips); }
void settings_incr_fault(void)      { bump(&s_counters.faults); }
void settings_get_counters(settings_counters_t *out) { *out = s_counters; }
void settings_flush(void) { if (s_dirty_incrs) counters_flush(); }

bool settings_password_is_set(void) { return s_pw_set; }

void settings_password_set_value(const char *pw) {
  esp_fill_random(s_pw, 16);           /* fresh salt */
  hash_pw(s_pw, pw, s_pw + 16);
  nvs_handle_t h;
  ns_open(&h, NVS_READWRITE);
  if (h) { nvs_set_blob(h, "pw", s_pw, sizeof(s_pw)); nvs_commit(h); nvs_close(h); }
  s_pw_set = true;
  ESP_LOGI(TAG, "web password set");
}

bool settings_password_check(const char *pw) {
  if (!s_pw_set) return false;
  uint8_t calc[32];
  hash_pw(s_pw, pw, calc);
  uint8_t diff = 0;
  for (int i = 0; i < 32; i++) diff |= calc[i] ^ s_pw[16 + i];   /* constant-time */
  return diff == 0;
}

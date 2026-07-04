# Garage Safety Features + Local Status Web Page — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add auto-close, a "door left open" HomeKit alert, an absolute vacation/lockout switch, and an authenticated local status/config web page — with auto-close fault detection so a stuck reed sensor cannot oscillate the door.

**Architecture:** A new `settings` module owns all NVS state (config, counters, web password). `door_control` gains a ~1 Hz tick (driven by a timeout on the existing event-loop queue) that evaluates auto-close, left-open, and fault detection. `hap_garage` gains a Contact Sensor, a Lockout Switch, and a StatusFault characteristic. A new `web_server` module serves an authenticated status/config page over HTTP. All new logic is single-threaded within the existing tasks — no new task, no shared-state races with the reed ISR.

**Tech Stack:** ESP-IDF v5.5.2, esp-homekit-sdk (HAP), `esp_http_server`, NVS, mbedTLS (SHA-256 + base64), FreeRTOS (single-core).

## Global Constraints

Copied verbatim from the spec / CLAUDE.md — every task implicitly includes these:

- ESP-IDF **v5.5.2**, target **esp32** only.
- App sources compile with **`-Werror`** — any warning fails the build.
- **No test framework** exists. "Verify" = `idf.py build` succeeds **and** the behavior is confirmed on hardware via `idf.py flash monitor`.
- Final image must fit the **1600 KB** OTA slot (CI checks this).
- Web server is **plain HTTP, port 80, LAN-only, no HTTPS**.
- **No secret is baked into the image** — the web password lives only in NVS (salted SHA-256).
- Single-core FreeRTOS: HTTP handlers must be **short and non-blocking** (serve from cached state).
- Use the `HOMEKIT_CHARACTERISTIC_*` macros (in `hap_garage.h`), **not** magic numbers.
- Match the existing style: C block comments delimiting each function; `ESP_LOGI/W/E` with the module's tag.
- **Bench-build note:** a local build defaults to `FW_VERSION=0.0.0-dev`, which the OTA poller treats as *older* than the latest release and will auto-downgrade the device. For bench testing, flash with `-DFW_VERSION=9.9.9` so the device keeps your build.

## File Structure

| File | Create/Modify | Responsibility |
|------|---------------|----------------|
| `main/settings.h` / `settings.c` | Create | NVS-backed config (`auto_close_sec`, `left_open_sec`, `lockout`), buffered counters, and salted-hash web password. Single owner of the `"garage"` NVS namespace. |
| `main/web_server.h` / `web_server.c` | Create | `esp_http_server` on :80. Basic-Auth gate, TOFU create-password flow, `/`, `/api/status`, `/api/settings`, `/api/lockout`, `/api/fault-reset`, `/api/password`. |
| `main/door_control.h` / `door_control.c` | Modify | Periodic tick (auto-close, left-open, fault detection), `door_open_since` tracking, counters, new getters. |
| `main/hap_garage.h` / `hap_garage.c` | Modify | Contact Sensor + Lockout Switch services, StatusFault char, notify helpers, lockout enforcement in `garage_write`. |
| `main/app_main.c` | Modify | `settings_init()` first; `web_server_start()` after Wi-Fi. |
| `main/Kconfig.projbuild` | Modify | Compile-time **defaults** for the two timeouts. |
| `main/CMakeLists.txt` | Modify | Add the two new source files to `SRCS`. |
| `README.md` | Modify | Document the web page, TOFU password, lockout, and HomeKit re-add caveat. |

**Task order & dependencies:** Task 1 (settings) is foundational. Task 2 (HAP services) depends on Task 1 and produces the notify helpers Task 3 needs. Task 3 (door tick) depends on 1 & 2. Task 4 (web server + wiring) depends on 1–3. Task 5 (docs) depends on all.

---

### Task 1: `settings` module — NVS config, counters, web password

**Files:**
- Create: `main/settings.h`, `main/settings.c`
- Modify: `main/Kconfig.projbuild` (add timeout defaults)
- Modify: `main/CMakeLists.txt` (add `settings.c`)

**Interfaces:**
- Consumes: NVS, mbedTLS SHA-256, `esp_random`. Kconfig `CONFIG_AUTO_CLOSE_DEFAULT_SEC`, `CONFIG_LEFT_OPEN_DEFAULT_SEC`.
- Produces (used by Tasks 2–4):
  - `void settings_init(void);`
  - `uint32_t settings_get_auto_close_sec(void);` / `uint32_t settings_get_left_open_sec(void);`
  - `void settings_set_timeouts(uint32_t auto_close_sec, uint32_t left_open_sec);` (clamps to 0 or 30–3600)
  - `bool settings_get_lockout(void);` / `void settings_set_lockout(bool on);`
  - `void settings_incr_cycle/auto_close/left_open/fault(void);`
  - `typedef struct { uint32_t cycles, auto_closes, left_open_trips, faults; } settings_counters_t;`
  - `void settings_get_counters(settings_counters_t *out);` / `void settings_flush(void);`
  - `bool settings_password_is_set(void);` / `bool settings_password_check(const char *password);` / `void settings_password_set_value(const char *password);`

- [ ] **Step 1: Add Kconfig defaults**

In `main/Kconfig.projbuild`, add before the final `endmenu`:

```
    config AUTO_CLOSE_DEFAULT_SEC
        int "Auto-close timeout default (seconds, 0 = disabled)"
        default 600
        help
            Seed value for the auto-close timeout on a fresh device. Runtime
            changes are stored in NVS and take precedence.

    config LEFT_OPEN_DEFAULT_SEC
        int "Door-left-open alert default (seconds, 0 = disabled)"
        default 300
        help
            Seed value for the left-open alert threshold on a fresh device.
            Runtime changes are stored in NVS and take precedence.
```

- [ ] **Step 2: Write `main/settings.h`**

```c
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
```

- [ ] **Step 3: Write `main/settings.c`**

```c
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
  memset(&s_counters, 0, sizeof(s_counters));
  s_pw_set = false;

  nvs_handle_t h;
  ns_open(&h, NVS_READONLY);
  if (h) {
    nvs_get_u32(h, "auto_close", &s_auto_close_sec);
    nvs_get_u32(h, "left_open",  &s_left_open_sec);
    uint8_t lk = 0;
    if (nvs_get_u8(h, "lockout", &lk) == ESP_OK) s_lockout = lk;
    size_t csz = sizeof(s_counters);
    nvs_get_blob(h, "counters", &s_counters, &csz);
    size_t psz = sizeof(s_pw);
    if (nvs_get_blob(h, "pw", s_pw, &psz) == ESP_OK && psz == sizeof(s_pw)) s_pw_set = true;
    nvs_close(h);
  }
  ESP_LOGI(TAG, "loaded: auto_close=%u left_open=%u lockout=%d pw_set=%d",
           s_auto_close_sec, s_left_open_sec, s_lockout, s_pw_set);
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
  }
  s_dirty_incrs = 0;
}

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
```

- [ ] **Step 4: Add `settings.c` to the build**

In `main/CMakeLists.txt`, extend the `SRCS` list:

```cmake
idf_component_register(SRCS "app_main.c" "hap_garage.c" "dht_sensor.c" "door_control.c" "ota_update.c" "settings.c"
                       INCLUDE_DIRS ".")
```

- [ ] **Step 5: Build**

Run:
```bash
. ~/esp/esp-idf/export.sh
idf.py build
```
Expected: compiles clean (no `-Werror` failures). `settings.c` is compiled and linked. (Nothing calls it yet, so no behavior change — this task is a pure foundation.)

- [ ] **Step 6: Commit**

```bash
git add main/settings.h main/settings.c main/Kconfig.projbuild main/CMakeLists.txt
git commit -m "feat(settings): NVS-backed config, counters, and web password module"
```

---

### Task 2: HAP services — Contact Sensor, Lockout Switch, StatusFault

**Files:**
- Modify: `main/hap_garage.h` (declare notify helpers)
- Modify: `main/hap_garage.c` (create services, helpers, lockout enforcement)

**Interfaces:**
- Consumes (from Task 1): `settings_get_lockout`, `settings_set_lockout`.
- Produces (used by Tasks 3–4):
  - `void hap_garage_send_left_open(bool tripped);`   (Contact Sensor: 1 = tripped/open)
  - `void hap_garage_send_status_fault(uint8_t fault);` (0 = no fault, 1 = general fault)
  - `void hap_garage_send_lockout(bool on);`          (syncs the Home-app Switch)

- [ ] **Step 1: Declare the new helpers in `main/hap_garage.h`**

Add `#include <stdbool.h>` near the top, and add after the existing `hap_garage_send_*` declarations:

```c
/* Left-open Contact Sensor: true = tripped (contact "open"). */
void hap_garage_send_left_open(bool tripped);
/* Garage Door StatusFault: 0 = no fault, 1 = general fault. */
void hap_garage_send_status_fault(uint8_t fault);
/* Lockout Switch: reflect an externally-driven (web) change back to HomeKit. */
void hap_garage_send_lockout(bool on);
```

- [ ] **Step 2: Include settings + add the notify helpers in `main/hap_garage.c`**

Add near the other includes:
```c
#include "settings.h"
```

Add these three helpers after `hap_garage_send_current_humidity()` (follow the existing find-by-UUID pattern):

```c
//
// Send update of the left-open Contact Sensor (1 = tripped / "contact not detected").
//
void hap_garage_send_left_open(bool tripped) {
  hap_val_t v = { .u = tripped ? 1 : 0 };
  hap_serv_t *s = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_CONTACT_SENSOR);
  hap_char_t *h = hap_serv_get_char_by_uuid(s, HAP_CHAR_UUID_CONTACT_SENSOR_STATE);
  ESP_LOGI(TAG, "Sending left-open contact [%s]", tripped ? "tripped" : "clear");
  hap_char_update_val(h, &v);
}

//
// Send update of the Garage Door StatusFault (0 = none, 1 = general fault).
//
void hap_garage_send_status_fault(uint8_t fault) {
  hap_val_t v = { .u = fault };
  hap_serv_t *s = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_GARAGE_DOOR_OPENER);
  hap_char_t *h = hap_serv_get_char_by_uuid(s, HAP_CHAR_UUID_STATUS_FAULT);
  ESP_LOGI(TAG, "Sending status fault [%d]", fault);
  hap_char_update_val(h, &v);
}

//
// Reflect a web-initiated lockout change back to the HomeKit Switch tile.
//
void hap_garage_send_lockout(bool on) {
  hap_val_t v = { .b = on };
  hap_serv_t *s = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_SWITCH);
  hap_char_t *h = hap_serv_get_char_by_uuid(s, HAP_CHAR_UUID_ON);
  ESP_LOGI(TAG, "Sending lockout switch [%s]", on ? "ON" : "OFF");
  hap_char_update_val(h, &v);
}
```

- [ ] **Step 3: Add the Lockout Switch write callback**

Add this function just above `garage_write()` in `main/hap_garage.c`:

```c
//
// Write callback for the Lockout Switch. Persists to NVS; the door logic reads
// settings_get_lockout() on each command / tick.
//
static int lockout_write(hap_write_data_t write_data[], int count,
                         void *serv_priv, void *write_priv) {
  for (int i = 0; i < count; i++) {
    hap_write_data_t *w = &write_data[i];
    if (!strcmp(hap_char_get_type_uuid(w->hc), HAP_CHAR_UUID_ON)) {
      settings_set_lockout(w->val.b);
      hap_char_update_val(w->hc, &w->val);
      if (w->status) *(w->status) = HAP_STATUS_SUCCESS;
      ESP_LOGI(TAG, "Lockout %s (via HomeKit)", w->val.b ? "ON" : "OFF");
    } else {
      if (w->status) *(w->status) = HAP_STATUS_RES_ABSENT;
    }
  }
  return HAP_SUCCESS;
}
```

- [ ] **Step 4: Enforce lockout in `garage_write()`**

Replace the existing `garage_write()` body with:

```c
//
// Incoming write (delegates to door_control, unless lockout is engaged).
//
static int garage_write(hap_write_data_t write_data[], int count,
                        void *serv_priv, void *write_priv) {
  if (settings_get_lockout()) {
    ESP_LOGW(TAG, "Door command refused — lockout is ON");
    /* Snap the target back to current so the Home app shows no pending command. */
    uint8_t cur = door_get_state();
    hap_garage_send_target_door_state(
        cur == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN
            ? HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN
            : HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
    for (int i = 0; i < count; i++)
      if (write_data[i].status) *(write_data[i].status) = HAP_STATUS_SUCCESS;
    return HAP_SUCCESS;
  }
  door_handle_homekit_request();
  return HAP_SUCCESS;
}
```

- [ ] **Step 5: Create the two services + StatusFault char in `hap_garage_init()`**

In `main/hap_garage.c`, immediately after the humidity service block (after `hap_acc_add_serv(accessory, hum_service);` and before `hap_add_accessory(accessory);`), add:

```c
    // Left-open Contact Sensor (trips when the door is open past the threshold).
    hap_serv_t *contact_service = hap_serv_contact_sensor_create(0);  // 0 = contact detected (closed)
    hap_serv_add_char(contact_service, hap_char_name_create("Garage Left Open"));
    hap_acc_add_serv(accessory, contact_service);

    // Lockout Switch (absolute freeze of all door movement when ON).
    hap_serv_t *lockout_service = hap_serv_switch_create(settings_get_lockout());
    hap_serv_add_char(lockout_service, hap_char_name_create("Lockout"));
    hap_serv_set_write_cb(lockout_service, lockout_write);
    hap_acc_add_serv(accessory, lockout_service);

    // StatusFault characteristic on the Garage Door service (auto-close fault indicator).
    hap_serv_add_char(service, hap_char_status_fault_create(0));
```

(`service` here is the garage-door service variable already declared at the top of `hap_garage_init()`.)

- [ ] **Step 6: Build**

Run: `idf.py build`
Expected: compiles clean. If a HAP symbol name mismatches (e.g. `hap_char_status_fault_create`), grep the SDK headers to confirm the exact name:
```bash
grep -rn "contact_sensor_create\|switch_create\|status_fault_create\|CONTACT_SENSOR_STATE\|STATUS_FAULT\|SERV_UUID_SWITCH" ~/esp/esp-homekit-sdk/components/homekit/esp_hap_apple_profiles/include
```
Adjust the identifiers to match the SDK, then rebuild.

- [ ] **Step 7: Flash and verify in the Home app**

Run: `idf.py -DFW_VERSION=9.9.9 flash monitor`
Expected on hardware:
- The accessory now shows a **"Garage Left Open"** contact sensor and a **"Lockout"** switch (you may need to remove and re-add the accessory in the Home app to see new tiles).
- Toggle **Lockout ON**, then try to open/close from the Home app: the door does **not** move and the serial log prints `Door command refused — lockout is ON`.
- Toggle **Lockout OFF**: door commands work again.
- The lockout state survives a reboot (serial log `loaded: ... lockout=1`).

- [ ] **Step 8: Commit**

```bash
git add main/hap_garage.h main/hap_garage.c
git commit -m "feat(hap): add left-open contact sensor, lockout switch, and status-fault char"
```

---

### Task 3: `door_control` — periodic tick, auto-close, left-open, fault detection

**Files:**
- Modify: `main/door_control.h` (new getters)
- Modify: `main/door_control.c` (tick, timestamp tracking, fault logic, counters)

**Interfaces:**
- Consumes: Task 1 (`settings_*`), Task 2 (`hap_garage_send_left_open`, `hap_garage_send_status_fault`).
- Produces (used by Task 4):
  - `bool door_auto_close_faulted(void);`
  - `void door_auto_close_fault_reset(void);`
  - `int64_t door_open_duration_us(void);`

- [ ] **Step 1: Declare the getters in `main/door_control.h`**

Add `#include <stdbool.h>` near the top, and add before the final line:

```c
/* True when auto-close has latched off after repeated failed attempts. */
bool door_auto_close_faulted(void);
/* Manually clear a latched auto-close fault (web reset button). */
void door_auto_close_fault_reset(void);
/* Microseconds the door has been continuously OPEN, or 0 if not open. */
int64_t door_open_duration_us(void);
```

- [ ] **Step 2: Add includes, constants, and state to `main/door_control.c`**

Add near the includes:
```c
#include "settings.h"
```

Add after the existing `static ...` declarations (below `oneshot_timer`):
```c
/* Auto-close fault detection + backstops. */
#define AUTO_CLOSE_MAX_FAILS    3       /* consecutive failed closes → latch off */
#define AUTO_CLOSE_MAX_PER_HOUR 10      /* hard cap on auto-close pulses per hour */
#define AUTO_CLOSE_MIN_GAP_US   60000000LL  /* >= 60 s between auto-close pulses */

static int64_t  door_open_since_us;        /* set when OPEN, 0 when CLOSED */
static bool     left_open_tripped;
static bool     auto_close_faulted;
static uint32_t auto_close_fail_count;
static int64_t  auto_close_verify_deadline_us;  /* 0 = no verify pending */
static int64_t  last_auto_close_pulse_us;
static int64_t  hour_window_start_us;
static uint32_t attempts_this_hour;
```

- [ ] **Step 3: Expand `door_set_state()` to manage the open timestamp + recovery**

Replace the one-line `door_set_state`:
```c
static void door_set_state(uint8_t s) { door_state = s; }
```
with:
```c
//
// Set the logical door state and maintain derived state: the open-since
// timestamp, the auto-close fault latch (auto-clears on a genuine CLOSED),
// and the left-open alert reset.
//
static void door_set_state(uint8_t s) {
  if (s == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN) {
    if (door_open_since_us == 0) door_open_since_us = esp_timer_get_time();
  } else if (s == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED) {
    door_open_since_us = 0;
    if (auto_close_faulted) {                 /* genuine close → clear the fault */
      auto_close_faulted = false;
      auto_close_fail_count = 0;
      auto_close_verify_deadline_us = 0;
      hap_garage_send_status_fault(0);
      ESP_LOGI(TAG, "Auto-close fault cleared (door reported CLOSED)");
    }
    if (left_open_tripped) {
      left_open_tripped = false;
      hap_garage_send_left_open(false);
    }
  }
  door_state = s;
}
```

- [ ] **Step 4: Add the periodic tick + public getters**

Add this block just above `door_control_run_loop()`:

```c
//
// Public getters for the web/HAP layers.
//
bool door_auto_close_faulted(void) { return auto_close_faulted; }

void door_auto_close_fault_reset(void) {
  auto_close_faulted = false;
  auto_close_fail_count = 0;
  auto_close_verify_deadline_us = 0;
  ESP_LOGI(TAG, "Auto-close fault reset (manual)");
}

int64_t door_open_duration_us(void) {
  return door_open_since_us ? (esp_timer_get_time() - door_open_since_us) : 0;
}

//
// Periodic evaluator (~1 Hz from the event loop). Runs auto-close, the left-open
// alert, and auto-close fault detection. Single-threaded within the loop, so it
// shares no locks with the reed ISR.
//
static void door_control_tick(void) {
  int64_t now = esp_timer_get_time();

  /* Verify a pending auto-close: did the door actually reach CLOSED? */
  if (auto_close_verify_deadline_us != 0 && now >= auto_close_verify_deadline_us) {
    auto_close_verify_deadline_us = 0;
    if (read_door_sensor() == DOOR_CLOSED) {
      auto_close_fail_count = 0;               /* success */
    } else {
      auto_close_fail_count++;
      settings_incr_fault();
      ESP_LOGE(TAG, "Auto-close attempt failed (%u/%u) — sensor still OPEN",
               auto_close_fail_count, AUTO_CLOSE_MAX_FAILS);
      if (auto_close_fail_count >= AUTO_CLOSE_MAX_FAILS && !auto_close_faulted) {
        auto_close_faulted = true;
        ESP_LOGE(TAG, "Auto-close LATCHED OFF — reed sensor may be stuck");
        hap_garage_send_status_fault(1);
      }
    }
  }

  /* Coarse periodic flush so counters survive power loss (~5 min). */
  static int64_t last_flush_us = 0;
  if (now - last_flush_us > 300000000LL) { last_flush_us = now; settings_flush(); }

  if (door_open_since_us == 0) return;         /* door closed → nothing to time */
  int64_t open_us = now - door_open_since_us;

  /* Left-open alert — fires regardless of lockout. */
  uint32_t left = settings_get_left_open_sec();
  if (left > 0 && !left_open_tripped && open_us >= (int64_t)left * 1000000LL) {
    left_open_tripped = true;
    settings_incr_left_open();
    ESP_LOGW(TAG, "Door left open > %u s — tripping alert", left);
    hap_garage_send_left_open(true);
  }

  /* Auto-close — suspended by lockout (absolute) and by the fault latch. */
  if (settings_get_lockout() || auto_close_faulted) return;
  uint32_t ac = settings_get_auto_close_sec();
  if (ac == 0 || open_us < (int64_t)ac * 1000000LL) return;
  if (auto_close_verify_deadline_us != 0) return;          /* attempt in flight */

  /* Backstops: min gap between pulses + hourly cap. */
  if (now - last_auto_close_pulse_us < AUTO_CLOSE_MIN_GAP_US) return;
  if (now - hour_window_start_us > 3600000000LL) { hour_window_start_us = now; attempts_this_hour = 0; }
  if (attempts_this_hour >= AUTO_CLOSE_MAX_PER_HOUR) return;

  if (read_door_sensor() != DOOR_OPEN) return;             /* nothing to close */

  ESP_LOGI(TAG, "Auto-closing door after %lld s open", open_us / 1000000LL);
  last_auto_close_pulse_us = now;
  attempts_this_hour++;
  settings_incr_auto_close();
  kick_door_relay();
  door_set_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
  hap_garage_send_current_door_state(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
  hap_garage_send_target_door_state(HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED);
  esp_timer_start_once(oneshot_timer, DOOR_MOVING_MAXTIME);   /* existing 8 s failsafe */
  auto_close_verify_deadline_us = now + DOOR_MOVING_MAXTIME;
}
```

- [ ] **Step 5: Drive the tick from the event loop + count cycles**

In `door_control_run_loop()`, change the queue wait so it wakes ~1 Hz, and count a cycle on each genuine open edge.

Replace:
```c
    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY) == pdFALSE) {
      ESP_LOGI(TAG, "Door trigger FAIL");
      continue;
    }
```
with:
```c
    if (xQueueReceive(gpio_evt_queue, &io_num, pdMS_TO_TICKS(1000)) == pdFALSE) {
      door_control_tick();   /* periodic: auto-close / left-open / fault checks */
      continue;
    }
```

Then, in the same loop, in the `if (sensor == DOOR_OPEN)` branch, add a cycle count as the first line inside the branch:
```c
    if (sensor == DOOR_OPEN) {
      settings_incr_cycle();
      gpio_set_level(DOOR_LED_GPIO, 1);
```

- [ ] **Step 6: Build**

Run: `idf.py build`
Expected: compiles clean. (`pdMS_TO_TICKS` comes from FreeRTOS, already included.)

- [ ] **Step 7: Flash and verify on hardware**

Run: `idf.py -DFW_VERSION=9.9.9 flash monitor`

Verify each behavior:
- **Auto-close:** set a short timeout for testing via the web page later, or temporarily set `CONFIG_AUTO_CLOSE_DEFAULT_SEC=30` in menuconfig. Open the door; after the timeout the log prints `Auto-closing door after N s open` and the relay pulses.
- **Left-open:** with the door open past `CONFIG_LEFT_OPEN_DEFAULT_SEC`, the log prints `Door left open > N s — tripping alert` and the Home-app contact sensor trips; it clears when the door closes.
- **Fault detection (key test):** simulate a stuck-open reed — disconnect the reed magnet (or jumper `DOOR_SENSOR_GPIO` so it reads OPEN). Trigger auto-close. Confirm the log shows at most 3 `Auto-close attempt failed` lines, then `Auto-close LATCHED OFF` and **the relay stops pulsing** (no oscillation). The Home-app StatusFault indicator turns on.
- **Auto-clear:** reconnect the reed so the door reports CLOSED → log prints `Auto-close fault cleared`.

- [ ] **Step 8: Commit**

```bash
git add main/door_control.h main/door_control.c
git commit -m "feat(door): auto-close, left-open alert, and stuck-sensor fault detection"
```

---

### Task 4: `web_server` module + app wiring

**Files:**
- Create: `main/web_server.h`, `main/web_server.c`
- Modify: `main/app_main.c` (call `settings_init()` and `web_server_start()`)
- Modify: `main/CMakeLists.txt` (add `web_server.c`)

**Interfaces:**
- Consumes: Task 1 (`settings_*`), Task 2 (`hap_garage_send_lockout`, `hap_garage_send_status_fault`), Task 3 (`door_auto_close_faulted`, `door_auto_close_fault_reset`, `door_open_duration_us`), plus `door_get_state`, `dht_sensor_get`.
- Produces: `void web_server_start(void);`

- [ ] **Step 1: Write `main/web_server.h`**

```c
//
// web_server.h — authenticated local status/config web page (HTTP, LAN-only).
//
#pragma once

/* Start the HTTP server on port 80. Call after Wi-Fi is connected. */
void web_server_start(void);
```

- [ ] **Step 2: Write `main/web_server.c`**

```c
//
// web_server.c — authenticated local status/config web page (HTTP :80, LAN-only).
//
#include <string.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_timer.h>
#include "mbedtls/base64.h"

#include "web_server.h"
#include "settings.h"
#include "door_control.h"
#include "hap_garage.h"
#include "dht_sensor.h"

static const char *TAG = "WebServer";
static httpd_handle_t server = NULL;

// ---- static pages --------------------------------------------------------

// First-run "create password" page (served while no password is set).
static const char SETUP_HTML[] =
"<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Garage setup</title>"
"<style>body{font-family:system-ui;max-width:32rem;margin:2rem auto;padding:0 1rem}"
"input,button{font-size:1rem;padding:.5rem;margin:.25rem 0;width:100%;box-sizing:border-box}</style>"
"<h1>Set admin password</h1>"
"<p>This device has no password yet. Set one now (min 4 characters). "
"Do this promptly — anyone on this network can set it until you do.</p>"
"<input id=pw type=password placeholder='New password'>"
"<button onclick=\"fetch('/api/password',{method:'POST',body:'password='+encodeURIComponent(pw.value)})"
".then(r=>r.ok?location.reload():r.text().then(t=>alert(t)))\">Save</button>";

// Main status/config page. Values are filled by JS from /api/status.
static const char PAGE_HTML[] =
"<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Garage</title>"
"<style>body{font-family:system-ui;max-width:36rem;margin:1.5rem auto;padding:0 1rem}"
"h1{font-size:1.4rem}.card{border:1px solid #ccc;border-radius:8px;padding:1rem;margin:1rem 0}"
"tr td:first-child{color:#666;padding-right:1rem}#fault{display:none;background:#c0392b;color:#fff;"
"padding:1rem;border-radius:8px}input,button{font-size:1rem;padding:.4rem}</style>"
"<h1>Garage Opener</h1>"
"<div id=fault>⚠ Auto-close FAULTED — reed sensor may be stuck. "
"<button onclick=\"post('/api/fault-reset','')\">Reset</button></div>"
"<div class=card><h3>Status</h3><table>"
"<tr><td>Door</td><td id=door>…</td></tr>"
"<tr><td>Open for</td><td id=openfor>…</td></tr>"
"<tr><td>Lockout</td><td><input type=checkbox id=lockout onchange=\"post('/api/lockout','on='+(this.checked?1:0))\"></td></tr>"
"<tr><td>Wi-Fi</td><td id=wifi>…</td></tr>"
"<tr><td>Temp</td><td id=temp>…</td></tr>"
"<tr><td>Humidity</td><td id=hum>…</td></tr>"
"<tr><td>Uptime</td><td id=uptime>…</td></tr></table></div>"
"<div class=card><h3>System</h3><table>"
"<tr><td>Firmware</td><td id=fw>…</td></tr>"
"<tr><td>Free heap</td><td id=heap>…</td></tr>"
"<tr><td>MAC</td><td id=mac>…</td></tr>"
"<tr><td>IP</td><td id=ip>…</td></tr>"
"<tr><td>Reset reason</td><td id=rst>…</td></tr></table></div>"
"<div class=card><h3>Counters</h3><table>"
"<tr><td>Door cycles</td><td id=cycles>…</td></tr>"
"<tr><td>Auto-closes</td><td id=ac>…</td></tr>"
"<tr><td>Left-open trips</td><td id=lo>…</td></tr>"
"<tr><td>Faults</td><td id=flt>…</td></tr></table></div>"
"<div class=card><h3>Settings</h3>"
"<p>Auto-close after <input id=ac_sec type=number min=0 max=3600 style=width:6rem> s (0 = off)</p>"
"<p>Left-open alert after <input id=lo_sec type=number min=0 max=3600 style=width:6rem> s (0 = off)</p>"
"<p id=hint style=color:#c0392b></p>"
"<button onclick=\"post('/api/settings','auto_close_sec='+ac_sec.value+'&left_open_sec='+lo_sec.value)\">Save</button>"
"</div>"
"<script>"
"function post(u,b){return fetch(u,{method:'POST',body:b}).then(r=>{if(!r.ok)r.text().then(t=>alert(t));setTimeout(refresh,300)})}"
"function fmt(s){s=Math.floor(s);let h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h '+m+'m '+(s%60)+'s'}"
"function refresh(){fetch('/api/status').then(r=>r.json()).then(d=>{"
"door.textContent=d.door_state;openfor.textContent=d.door_open_seconds?fmt(d.door_open_seconds):'—';"
"lockout.checked=d.lockout;wifi.textContent=d.rssi+' dBm ('+d.rssi_quality+')';"
"temp.textContent=d.temperature.toFixed(1)+' °C';hum.textContent=d.humidity.toFixed(0)+' %';"
"uptime.textContent=fmt(d.uptime_seconds);fw.textContent=d.fw_version;heap.textContent=d.free_heap+' B';"
"mac.textContent=d.mac;ip.textContent=d.ip;rst.textContent=d.reset_reason;"
"cycles.textContent=d.cycles;ac.textContent=d.auto_closes;lo.textContent=d.left_open_trips;flt.textContent=d.faults;"
"fault.style.display=d.auto_close_faulted?'block':'none';"
"if(document.activeElement!=ac_sec)ac_sec.value=d.auto_close_sec;"
"if(document.activeElement!=lo_sec)lo_sec.value=d.left_open_sec;"
"hint.textContent=(d.left_open_sec&&d.auto_close_sec&&d.left_open_sec>=d.auto_close_sec)?"
"'Left-open >= auto-close: the alert may never fire.':'';"
"})}refresh();setInterval(refresh,3000);</script>";

// ---- helpers -------------------------------------------------------------

static const char *reset_reason_str(void) {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "int-wdt";
    case ESP_RST_TASK_WDT:  return "task-wdt";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_DEEPSLEEP: return "deep-sleep";
    default:                return "other";
  }
}

static const char *door_state_str(uint8_t s) {
  switch (s) {
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN:    return "open";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED:  return "closed";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING: return "opening";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING: return "closing";
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED: return "stopped";
    default:                                                 return "unknown";
  }
}

// Basic Auth. Returns true if authorized. (TOFU: caller checks password_is_set first.)
static bool check_auth(httpd_req_t *req) {
  char hdr[160];
  if (httpd_req_get_hdr_value_str(req, "Authorization", hdr, sizeof(hdr)) != ESP_OK) return false;
  if (strncmp(hdr, "Basic ", 6) != 0) return false;
  unsigned char dec[120];
  size_t olen = 0;
  if (mbedtls_base64_decode(dec, sizeof(dec) - 1, &olen,
                            (const unsigned char *)hdr + 6, strlen(hdr + 6)) != 0) return false;
  dec[olen] = '\0';
  char *colon = strchr((char *)dec, ':');
  if (!colon) return false;
  *colon = '\0';
  if (strcmp((char *)dec, "admin") != 0) return false;
  return settings_password_check(colon + 1);
}

// Enforce auth; on failure send 401 and return ESP_FAIL.
static esp_err_t require_auth(httpd_req_t *req) {
  if (settings_password_is_set() && check_auth(req)) return ESP_OK;
  if (!settings_password_is_set()) return ESP_OK;   /* TOFU window: allow */
  httpd_resp_set_status(req, "401 Unauthorized");
  httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Garage\"");
  httpd_resp_send(req, "Authentication required", HTTPD_RESP_USE_STRLEN);
  return ESP_FAIL;
}

// Read a request body into buf (NUL-terminated). Returns length or -1.
static int read_body(httpd_req_t *req, char *buf, size_t len) {
  int total = req->content_len;
  if (total >= (int)len) total = len - 1;
  int off = 0;
  while (off < total) {
    int r = httpd_req_recv(req, buf + off, total - off);
    if (r <= 0) return -1;
    off += r;
  }
  buf[off] = '\0';
  return off;
}

// ---- handlers ------------------------------------------------------------

static esp_err_t root_get(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  if (!settings_password_is_set()) {
    httpd_resp_send(req, SETUP_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  if (require_auth(req) != ESP_OK) return ESP_OK;
  httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t status_get(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;

  float t = 0, h = 0;
  dht_sensor_get(&t, &h);

  int rssi = 0;
  const char *q = "n/a";
  wifi_ap_record_t ap;
  if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
    rssi = ap.rssi;
    q = rssi >= -55 ? "excellent" : rssi >= -67 ? "good" : rssi >= -75 ? "fair" : "weak";
  }

  esp_netif_ip_info_t ip = {0};
  esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (nif) esp_netif_get_ip_info(nif, &ip);

  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  settings_counters_t c;
  settings_get_counters(&c);

  char buf[720];
  int n = snprintf(buf, sizeof(buf),
    "{\"door_state\":\"%s\",\"door_open_seconds\":%lld,\"lockout\":%s,"
    "\"rssi\":%d,\"rssi_quality\":\"%s\",\"temperature\":%.1f,\"humidity\":%.1f,"
    "\"uptime_seconds\":%lld,\"fw_version\":\"%s\",\"free_heap\":%u,"
    "\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"ip\":\"" IPSTR "\",\"reset_reason\":\"%s\","
    "\"cycles\":%u,\"auto_closes\":%u,\"left_open_trips\":%u,\"faults\":%u,"
    "\"auto_close_faulted\":%s,\"auto_close_sec\":%u,\"left_open_sec\":%u}",
    door_state_str(door_get_state()), door_open_duration_us() / 1000000LL,
    settings_get_lockout() ? "true" : "false",
    rssi, q, t, h,
    esp_timer_get_time() / 1000000LL, FW_VERSION, (unsigned)esp_get_free_heap_size(),
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], IP2STR(&ip.ip), reset_reason_str(),
    c.cycles, c.auto_closes, c.left_open_trips, c.faults,
    door_auto_close_faulted() ? "true" : "false",
    settings_get_auto_close_sec(), settings_get_left_open_sec());

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, buf, n);
  return ESP_OK;
}

static esp_err_t settings_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  char buf[128];
  if (read_body(req, buf, sizeof(buf)) < 0) { httpd_resp_send_500(req); return ESP_OK; }
  char v[16];
  uint32_t ac = settings_get_auto_close_sec(), lo = settings_get_left_open_sec();
  if (httpd_query_key_value(buf, "auto_close_sec", v, sizeof(v)) == ESP_OK) ac = atoi(v);
  if (httpd_query_key_value(buf, "left_open_sec",  v, sizeof(v)) == ESP_OK) lo = atoi(v);
  settings_set_timeouts(ac, lo);   /* clamps internally */
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

static esp_err_t lockout_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  char buf[32];
  if (read_body(req, buf, sizeof(buf)) < 0) { httpd_resp_send_500(req); return ESP_OK; }
  char v[8];
  bool on = settings_get_lockout();
  if (httpd_query_key_value(buf, "on", v, sizeof(v)) == ESP_OK) on = (atoi(v) != 0);
  settings_set_lockout(on);
  hap_garage_send_lockout(on);     /* sync the Home-app Switch tile */
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

static esp_err_t fault_reset_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  door_auto_close_fault_reset();
  hap_garage_send_status_fault(0);
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

static esp_err_t password_post(httpd_req_t *req) {
  bool first = !settings_password_is_set();
  if (!first && require_auth(req) != ESP_OK) return ESP_OK;
  char buf[96];
  if (read_body(req, buf, sizeof(buf)) < 0) { httpd_resp_send_500(req); return ESP_OK; }
  char pw[64];
  if (httpd_query_key_value(buf, "password", pw, sizeof(pw)) != ESP_OK || strlen(pw) < 4) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "password too short (min 4)");
    return ESP_OK;
  }
  settings_password_set_value(pw);
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

// ---- lifecycle -----------------------------------------------------------

void web_server_start(void) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 8;
  config.lru_purge_enable = true;
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG, "httpd start failed");
    return;
  }
  httpd_uri_t uris[] = {
    { .uri = "/",               .method = HTTP_GET,  .handler = root_get },
    { .uri = "/api/status",     .method = HTTP_GET,  .handler = status_get },
    { .uri = "/api/settings",   .method = HTTP_POST, .handler = settings_post },
    { .uri = "/api/lockout",    .method = HTTP_POST, .handler = lockout_post },
    { .uri = "/api/fault-reset",.method = HTTP_POST, .handler = fault_reset_post },
    { .uri = "/api/password",   .method = HTTP_POST, .handler = password_post },
  };
  for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
    httpd_register_uri_handler(server, &uris[i]);
  ESP_LOGI(TAG, "Web server started on port 80");
}
```

- [ ] **Step 3: Wire into `main/app_main.c`**

Add the includes:
```c
#include "settings.h"
#include "web_server.h"
```
Update `garage_thread_entry()`:
```c
static void garage_thread_entry(void *p) {
  settings_init();          // load NVS config/counters/password before anything reads them
  hap_garage_init();        // create accessory + services, app_wifi_init, register handlers
  door_control_init();      // door GPIO + queue + failsafe timer + initial state publish
  hap_garage_start();       // start the HAP core
  hap_garage_start_wifi();  // blocking
  web_server_start();       // Wi-Fi up → serve the local status page
  door_control_run_loop();  // never returns
}
```

- [ ] **Step 4: Add `web_server.c` to the build**

In `main/CMakeLists.txt`:
```cmake
idf_component_register(SRCS "app_main.c" "hap_garage.c" "dht_sensor.c" "door_control.c" "ota_update.c" "settings.c" "web_server.c"
                       INCLUDE_DIRS ".")
```

- [ ] **Step 5: Build**

Run: `idf.py build`
Expected: compiles clean. Check the image still fits the OTA slot (CI enforces 1600 KB; `idf.py size` shows app size).

- [ ] **Step 6: Flash and verify in a browser**

Run: `idf.py -DFW_VERSION=9.9.9 flash monitor`, note the device IP from the log (`sta ip: ...`).
- Browse to `http://<device-ip>/` on the same LAN → **create-password** screen appears (fresh NVS). Set a password.
- Page reloads → browser prompts for Basic Auth (`admin` + your password). Status, System, Counters render and refresh every 3 s.
- Change the timeouts, Save → reboot the device → values persist (loaded from NVS).
- Toggle the **Lockout** checkbox → the Home-app switch flips too, and door commands are refused.
- Force a fault (stuck sensor, per Task 3) → the red banner appears; click **Reset** → banner clears.
- Wrong password → 401; correct password → access.

- [ ] **Step 7: Commit**

```bash
git add main/web_server.h main/web_server.c main/app_main.c main/CMakeLists.txt
git commit -m "feat(web): authenticated local status/config page on port 80"
```

---

### Task 5: Documentation

**Files:**
- Modify: `README.md`

**Interfaces:** none (docs only).

- [ ] **Step 1: Document the new features in `README.md`**

Add a section covering:
- **Auto-close** — default 10 min, configurable on the web page (0 = off).
- **Door-left-open alert** — HomeKit Contact Sensor "Garage Left Open"; enable notifications on it in the Home app. Default 5 min.
- **Vacation/lockout switch** — HomeKit Switch "Lockout"; absolute freeze (opens, closes, and auto-close). Editable from the Home app **and** the web page.
- **Local status page** — `http://<device-ip>/`. First visit sets an `admin` password (TOFU — set it promptly, since anyone on the LAN can set it until you do). Shows live status, system/FW info, counters, and the settings form. Plain HTTP, LAN-only.
- **Auto-close fault** — if the reed sensor sticks open, auto-close latches off after 3 failed attempts (no oscillation), shows a web banner + HomeKit fault indicator, and clears on a genuine close or the web Reset button.
- **HomeKit caveat** — after this update you may need to remove and re-add the accessory in the Home app to see the new Contact Sensor and Lockout tiles.

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: document safety features and local status web page"
```

---

## Self-Review

**Spec coverage** (each spec section → task):
- Settings/NVS + counters + password → Task 1. ✓
- Auto-close (§2) → Task 3 tick. ✓
- Auto-close fault detection (§2b) incl. backstops, latch, auto-clear + manual reset, all-three surfacing → Task 3 (logic + StatusFault + ESP_LOGE) & Task 4 (web banner + reset). ✓
- Left-open Contact Sensor (§3), fires under lockout → Task 2 (service) + Task 3 (trip logic). ✓
- Lockout absolute (§4), two surfaces/one source of truth → Task 2 (switch + garage_write) + Task 4 (`/api/lockout` + `hap_garage_send_lockout`). ✓
- Web page + API (§5), all endpoints + status fields → Task 4. ✓
- Auth Basic + TOFU + salted hash (§6) → Task 1 (hash/check) + Task 4 (Basic Auth + TOFU). ✓
- Kconfig defaults, edge-case hint (left ≥ auto), re-pair caveat → Tasks 1, 4 (JS hint), 5. ✓
- Verification per feature → each task's hardware step. ✓

**Placeholder scan:** No TBD/TODO; every code step contains complete code. The only "fill in prose" step is Task 5 (README), which lists exact points to document.

**Type consistency:** `settings_*`, `door_auto_close_faulted/_fault_reset`, `door_open_duration_us`, `hap_garage_send_left_open/_status_fault/_lockout`, `settings_counters_t` fields (`cycles`/`auto_closes`/`left_open_trips`/`faults`) match across the Interfaces blocks, the module code, and the `/api/status` JSON keys. Contact-sensor "tripped" = value 1 consistently. StatusFault 0/1 consistent.

**Risk to watch during Task 2:** exact HAP SDK identifiers (`hap_serv_contact_sensor_create`, `hap_serv_switch_create`, `hap_char_status_fault_create`, `HAP_SERV_UUID_CONTACT_SENSOR`, `HAP_CHAR_UUID_CONTACT_SENSOR_STATE`, `HAP_SERV_UUID_SWITCH`, `HAP_CHAR_UUID_ON`, `HAP_CHAR_UUID_STATUS_FAULT`) — Step 6 includes a grep to confirm against the pinned SDK and adjust if needed.

# Design: Safety features + local status web page

**Date:** 2026-07-04
**Status:** Approved (brainstorming), pending implementation plan
**Target:** ESP32-homekit-garage-opener firmware (ESP-IDF v5.5.2)

## Overview

Add four safety/convenience features to the garage opener firmware:

1. **Auto-close timer** — automatically close the door after it has been open too long.
2. **"Door left open" alert** — a HomeKit Contact Sensor that trips when the door is
   open past a threshold, so the Home app can push a notification.
3. **Vacation/lockout switch** — a HomeKit Switch that freezes all door movement.
4. **Local status web page** — an authenticated LAN web page showing live status,
   system/firmware info, persisted counters, and an editable settings form.

Plus a critical **auto-close fault-detection** mechanism so a stuck reed sensor cannot
drive the door into an open→close→open loop.

## Goals

- Auto-close and left-open thresholds are runtime-configurable and persist across reboot.
- A stuck-open reed sensor never causes repeated relay pulses (no oscillation).
- Vacation lockout is absolute: it refuses all user commands and suspends auto-close.
- The web page is authenticated; no secret is baked into the public OTA image.
- Preserve the project's module boundaries and coding conventions.

## Non-goals

- No cloud connectivity, MQTT, or external integrations.
- No HTTPS on the web server (plain HTTP, LAN-only).
- No multi-user auth or roles (single `admin` credential).
- No changes to the OTA, Wi-Fi provisioning, or DHT subsystems beyond reading their state.

## Architecture

### Module changes

| Module | Change | Responsibility |
|--------|--------|----------------|
| `settings.{c,h}` | **new** | Single owner of all NVS-backed state: config (`auto_close_sec`, `left_open_sec`, `lockout`), the web password (+ `password_set` flag), and counters. Typed getters/setters; buffered flush to limit flash wear. Loaded once at boot. |
| `web_server.{c,h}` | **new** | `esp_http_server` on port 80. Basic-Auth gate, TOFU create-password flow, `GET /`, `GET /api/status`, `POST /api/settings`, `POST /api/fault-reset`, `POST /api/password`. Pure presentation over `settings` + live getters. |
| `door_control.{c,h}` | extend | Periodic timer evaluator (auto-close, left-open, fault detection), lockout enforcement, counter increments, `door_open_since` tracking. New public helpers for the web/HAP layers to query fault + timing state. |
| `hap_garage.{c,h}` | extend | Two new HAP services — **Contact Sensor** ("Garage Left Open") and **Switch** ("Lockout") — plus `StatusFault` on the Garage Door service. New notify helpers (incl. `hap_garage_send_lockout(bool)` so a web-initiated change reflects in the Home app); `garage_write` consults lockout before acting. |
| `app_main.c` | extend | Start `web_server` after Wi-Fi/HAP are up. |
| `Kconfig.projbuild` | extend | Compile-time **defaults** only (auto-close 600 s, left-open 300 s); runtime values live in NVS. |

### Data flow

- **Reed edges** → ISR → queue → `door_control_run_loop()` (unchanged path for state updates).
- **Periodic tick**: `door_control_run_loop()` currently blocks on `xQueueReceive(..., portMAX_DELAY)`.
  Change the wait to a ~1 s timeout so the loop wakes periodically and calls a new
  `door_control_tick()` that evaluates auto-close / left-open / fault logic. This keeps **all**
  door logic single-threaded inside the existing consolidated loop — no new task, no shared-state
  races with the ISR/queue.
- **HomeKit open/close** → `garage_write` (in `hap_garage.c`) → checks lockout → if allowed,
  `door_handle_homekit_request()`; if refused, snap the target-state characteristic back to current.
- **Web requests** → `web_server` reads `settings` + live getters (`door_get_state`, RSSI, DHT
  cache, heap, uptime) and writes `settings` on POST.

## Detailed design

### 1. Settings + counters (`settings.{c,h}`)

Owns one NVS namespace (e.g. `"garage"`). API sketch:

```c
void     settings_init(void);              // load from NVS into RAM cache at boot
uint32_t settings_get_auto_close_sec(void);
uint32_t settings_get_left_open_sec(void);
void     settings_set_timeouts(uint32_t auto_close_sec, uint32_t left_open_sec); // validates + persists
bool     settings_get_lockout(void);
void     settings_set_lockout(bool on);    // persists

// Counters: incremented in RAM, flushed periodically / on graceful events.
void     settings_incr_cycle(void);
void     settings_incr_auto_close(void);
void     settings_incr_left_open(void);
void     settings_incr_fault(void);
void     settings_counters_get(...);       // for the web page
void     settings_flush(void);             // force write dirty values

// Web auth (see section 6)
bool     settings_password_set(void);
bool     settings_password_check(const char *user, const char *pass);
void     settings_password_set_value(const char *pass); // stores salted hash
```

- **Config** (`auto_close_sec`, `left_open_sec`, `lockout`) is written immediately on change
  (rare, user-initiated — flash wear is a non-issue).
- **Counters** live in RAM and are marked dirty on increment; flushed on a coarse cadence
  (every N increments or a periodic timer, and on settings change) — never per door cycle.
- Defaults on first boot come from the Kconfig values; `lockout` defaults off; counters start at 0.

### 2. Auto-close (`door_control.c`)

- Track `int64_t door_open_since_us` — set when the door transitions to `OPEN` (in the run-loop
  open branch and in `door_control_init` if it boots open); cleared on `CLOSED`.
- In `door_control_tick()`: if `auto_close_sec > 0` **and** not locked out **and** not faulted
  **and** the door currently reads `OPEN` for longer than `auto_close_sec`, initiate a close.
- Initiating a close reuses the relay path (an internal `door_auto_close()` that pulses the relay
  only when the sensor reads `OPEN`, sets state `CLOSING`, arms the existing 8 s move timer, and
  records an `auto_close_attempt` with a verification deadline).
- `0` disables auto-close. Default 600 s (10 min).

### 2b. Auto-close fault detection (the anti-oscillation guard)

Garage openers are **toggle** devices, so pulsing the relay on a door that is actually closed
(but reads open due to a stuck reed) physically **opens** it — repeated pulses oscillate the door.
Guard with a bounded, verified retry:

- After an auto-close pulse, wait the move window (`DOOR_MOVING_MAXTIME`, 8 s). In `door_control_tick()`,
  once the deadline passes, read the sensor:
  - `CLOSED` → **success**: reset `auto_close_fail_count = 0`.
  - still `OPEN` → **failure**: `auto_close_fail_count++`.
- After **`AUTO_CLOSE_MAX_FAILS` (default 3)** consecutive failures, latch
  `auto_close_faulted = true`: stop initiating auto-close entirely (this breaks the loop),
  `settings_incr_fault()`, log `ESP_LOGE`, and raise HomeKit `StatusFault`.
- **Backstops** (independent of the counter, always enforced): never pulse the relay for auto-close
  more than **once per minute**, and cap auto-close attempts per rolling hour. Even a logic bug
  cannot rapidly oscillate the door.
- **Recovery (auto-clear + manual reset):** the latch clears when
  (a) the door genuinely reports `CLOSED` again (via any path), **or**
  (b) the user hits the web-page reset (`POST /api/fault-reset`), **or**
  (c) reboot. On clear: `auto_close_fail_count = 0`, `auto_close_faulted = false`, clear `StatusFault`.
- **Surfacing (all three):** web-page banner + fault counter, HomeKit `StatusFault` on the Garage
  Door service, and `ESP_LOGE` with the `"HAP Garage"` tag.

Public helpers exposed for the web/HAP layers:

```c
bool     door_auto_close_faulted(void);
void     door_auto_close_fault_reset(void);   // manual reset
int64_t  door_open_duration_us(void);         // for the web page ("open for 4m12s")
```

### 3. Left-open alert (`door_control.c` + `hap_garage.c`)

- Independent threshold `left_open_sec` (default 300 s, `0` = off).
- In `door_control_tick()`: if the door has read `OPEN` longer than `left_open_sec`, set the
  Contact Sensor to *Open* (contact detected/"tripped") via a new
  `hap_garage_send_left_open(bool)` and `settings_incr_left_open()` (once per trip, edge-triggered).
- When the door closes, reset the Contact Sensor to *Closed*.
- **Fires regardless of lockout** — a door stuck open while you're away should still alert you.
- Edge case: if `left_open_sec >= auto_close_sec` the alert may never fire (the door auto-closes
  first). The settings form displays a hint; this is not an error.

### 4. Vacation/lockout (absolute) (`hap_garage.c` + `web_server.c`)

- New HAP **Switch** service ("Lockout"). Its write callback → `settings_set_lockout(on)`.
- On boot, restore the switch value from `settings_get_lockout()`.
- **Two control surfaces, one source of truth.** Lockout is editable from **both** the Home-app
  Switch and the web page; `settings` (NVS) is the single source of truth. Changing it from either
  surface must update the other:
  - HAP Switch write callback → `settings_set_lockout(on)` (the web page picks it up on its next poll).
  - Web `POST /api/lockout` → `settings_set_lockout(on)` **and** `hap_garage_send_lockout(on)` so the
    Home-app Switch tile reflects the change immediately.
- When lockout is **ON**:
  - `garage_write` refuses **both** open and close: it does **not** call
    `door_handle_homekit_request()`, and it re-sends the current door state as the target so the
    Home-app tile does not show a stuck pending command.
  - `door_control_tick()` skips auto-close entirely (absolute — the safety timer is suspended).
  - The left-open Contact Sensor still updates (reporting only).

### 5. Local status web page (`web_server.{c,h}`)

`esp_http_server` on port 80, plain HTTP, LAN-only. Endpoints (all except the TOFU create-password
flow require Basic Auth — see section 6):

| Method / path | Purpose |
|---------------|---------|
| `GET /` | Self-contained HTML page (inline CSS/JS, no external assets). Renders live status, system/FW info, counters, settings form, a **lockout on/off toggle**, and the fault banner when latched. |
| `GET /api/status` | JSON snapshot the page polls every few seconds. |
| `POST /api/settings` | Validate + persist `auto_close_sec`, `left_open_sec` to NVS; returns the stored values. |
| `POST /api/lockout` | Set lockout on/off → `settings_set_lockout()` + `hap_garage_send_lockout()` (syncs the Home-app Switch). |
| `POST /api/fault-reset` | Clear a latched auto-close fault. |
| `POST /api/password` | Change the admin password (authenticated); also the target of the TOFU first-run form. |

`GET /api/status` JSON fields:

- **Live:** `door_state`, `door_open_seconds`, `lockout`, `rssi`, `rssi_quality`,
  `temperature`, `humidity`, `uptime_seconds`.
- **System:** `fw_version`, `free_heap`, `mac`, `ip`, `reset_reason`.
- **Counters:** `cycles`, `auto_closes`, `left_open_trips`, `faults`.
- **Fault:** `auto_close_faulted` (drives the banner).
- **Settings:** `auto_close_sec`, `left_open_sec` (to populate the form).

Validation: timeouts are clamped to a sane range (e.g. 0 or 30–3600 s); `0` means disabled.

### 6. Web authentication (Basic Auth + TOFU)

- **Scheme:** HTTP Basic Auth. Username fixed `admin`. Password stored in NVS as a **salted hash**
  (never plaintext, never in the image). Checked on every protected request via the
  `Authorization` header.
- **First-run (TOFU):** while `settings_password_set() == false`, protected endpoints redirect to
  an unauthenticated **"create password"** screen; `POST /api/password` sets the initial password
  and flips `password_set = true`. Once set, that unauthenticated path is closed and Basic Auth is
  enforced everywhere.
- **Change password:** `POST /api/password` while authenticated updates the stored hash.
- **Caveat (documented):** a first-boot race exists (someone else on the LAN could set the password
  first). Acceptable for a home network; docs advise setting the password promptly after provisioning.
- Nothing secret is compiled into the public OTA binary.

## Configuration defaults (Kconfig)

- `CONFIG_AUTO_CLOSE_DEFAULT_SEC` = 600 (10 min; 0 = disabled)
- `CONFIG_LEFT_OPEN_DEFAULT_SEC` = 300 (5 min; 0 = disabled)
- `AUTO_CLOSE_MAX_FAILS` = 3 (compile-time constant)
- Relay backstops: min 60 s between auto-close pulses; capped attempts/hour (compile-time constants).

These seed NVS on first boot only; runtime values thereafter come from NVS.

## Edge cases & caveats

- `left_open_sec >= auto_close_sec` → left-open alert may not fire (UI hint, not an error).
- Adding two HAP services may require removing/re-adding the accessory in the Home app to see the
  new tiles (HomeKit behavior; documented in README, no code impact).
- Lockout toggled ON mid-move: auto-close evaluation is skipped on the next tick; any in-flight
  user move already commanded completes under the existing 8 s failsafe.
- Counter flush must also happen on a coarse timer so counts aren't lost on unexpected power loss
  beyond the last flush window (acceptable small loss).

## Testing / verification

No test framework exists in this repo (per CLAUDE.md); "verify" = compiles + behaves on hardware.

- Builds clean under `-Werror`; image still fits the 1600 KB OTA slot (CI check).
- **Auto-close:** door left open past timeout → closes automatically.
- **Left-open:** Contact Sensor trips at threshold; Home-app notification fires; clears on close.
- **Lockout:** switch ON refuses HomeKit open + close and suspends auto-close; tile shows no stuck
  command; switch state survives reboot.
- **Fault detection (key):** simulate a stuck-open reed (e.g. disconnect magnet / force sensor high);
  confirm auto-close makes ≤3 attempts then latches — **no oscillation** — raises `StatusFault`,
  shows the web banner, and logs `ESP_LOGE`. Confirm auto-clear on a real close and manual reset
  via the web page.
- **Web page:** loads, polls live, settings persist across reboot; Basic Auth enforced; TOFU
  create-password works on a fresh NVS and is closed afterward.

## Open risks

- **HAP service count / re-pair:** adding services to an existing paired accessory can require
  re-adding it in Home; call out in release notes.
- **Flash/heap budget:** the HTML page + esp_http_server + Basic Auth add flash and RAM; verify the
  OTA-slot fit and free-heap headroom during implementation.
- **Single-core FreeRTOS:** the web server runs on the same core as HAP/door tasks; keep request
  handlers short and non-blocking (serve from cached state, no long sensor reads inline).

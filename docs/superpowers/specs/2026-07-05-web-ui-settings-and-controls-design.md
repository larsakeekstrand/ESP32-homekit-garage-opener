# Design: More runtime settings + device controls in the web UI

**Date:** 2026-07-05
**Status:** Approved (brainstorming), pending implementation plan
**Target:** ESP32-homekit-garage-opener firmware (ESP-IDF v5.5.2)
**Builds on:** the redesigned web page (`feat/web-ui-redesign`) — this extends that page and ships with it.

## Overview

Make three currently-hardcoded values runtime-configurable from the web page, and add device
controls: a "Check for updates now" action and a "Reboot" button. The OTA poll interval, door-move
timeout, and relay pulse length move from compile-time constants into NVS-backed `settings` fields
(the pattern already used for `auto_close_sec`), with the existing Kconfig values as default seeds.
The OTA poller is reworked so an interval change takes effect within ~10 s without a reboot.

## Goals

- Edit the OTA poll interval, door-move timeout, and relay pulse length from the web page; values
  persist across reboot.
- An OTA-poll-interval change takes effect **near-immediately (~10 s), no reboot**.
- A "Check for updates now" button triggers an immediate one-off poll.
- A "Reboot" button restarts the device (confirm-guarded).
- Keep the auto-close anti-oscillation backstops compiled-in (not user-tunable) for safety.

## Non-goals

- No new persistence subsystem — reuse the `settings` NVS module.
- Do not expose the auto-close backstops (max fails / per-hour cap / min gap), debounce, or the
  DHT/RSSI intervals.
- No scheduling/cron; "check now" is a single immediate poll, not a new schedule.

## Architecture

### New NVS settings (`settings.{c,h}`)

Three fields added like the existing `auto_close_sec`, each with a getter/setter that clamps to range;
the setter persists immediately. Kconfig provides the default seed.

| Field | Type | Default (Kconfig) | Range | UI unit |
|-------|------|-------------------|-------|---------|
| `ota_poll_sec` | uint32 | `CONFIG_OTA_POLL_INTERVAL_SEC` (3600) | 300–86400 s | minutes (5–1440) |
| `door_move_sec` | uint32 | 8 (new `CONFIG_DOOR_MOVE_TIMEOUT_SEC`) | 3–30 s | seconds |
| `relay_pulse_ms` | uint32 | 300 (new `CONFIG_RELAY_PULSE_MS`) | 100–2000 ms | ms |

API (mirrors `settings_get/set_auto_close`):
```c
uint32_t settings_get_ota_poll_sec(void);   void settings_set_ota_poll_sec(uint32_t s);
uint32_t settings_get_door_move_sec(void);   void settings_set_door_move_sec(uint32_t s);
uint32_t settings_get_relay_pulse_ms(void);  void settings_set_relay_pulse_ms(uint32_t ms);
```
NVS keys: `"ota_poll"`, `"door_move"`, `"relay_ms"`. Loaded in `settings_init` with the defaults above.

### Consumers read settings at runtime

**`ota_update.c` — chunked poll + check-now.** Replace the single long `vTaskDelay(POLL_INTERVAL_MS)`
with a loop that sleeps in fixed chunks (`OTA_POLL_CHUNK_SEC` = 10 s) and, after each chunk:
- if paused (`settings_get_auto_ota_paused()`) → keep waiting (reset the elapsed accumulator);
- if a module-level **`s_check_now`** flag is set → clear it and poll immediately;
- else accumulate elapsed time; when elapsed ≥ `settings_get_ota_poll_sec()` → poll and reset.

A new `void ota_update_check_now(void)` sets `s_check_now` (called by the web handler). This makes
an interval change apply within one chunk (~10 s) and a manual check near-instant — no reboot.
`ota_update_mark_valid()` and the pushed-image pause behavior are unchanged.

**`door_control.c`.** Replace the compile-time constants with settings reads:
- The 8 s failsafe (`DOOR_MOVING_MAXTIME`, µs) → `settings_get_door_move_sec() * 1000000LL`, used
  for both `esp_timer_start_once(oneshot_timer, …)` and the auto-close verify deadline (they stay
  equal — the verify window is the move window).
- `kick_door_relay()`'s hardcoded `300` ms → `settings_get_relay_pulse_ms()`.

### Endpoints (`web_server.c`)

- Extend **`POST /api/settings`** to also parse `ota_poll_min`, `door_move_sec`, `relay_pulse_ms`
  (each optional; convert `ota_poll_min` → seconds; each clamped by its setter).
- **`POST /api/ota-check`** (auth) → `ota_update_check_now()`; respond `ok`.
- **`POST /api/reboot`** (auth) → send response, `vTaskDelay(500 ms)`, `esp_restart()` (same
  response-before-restart pattern as `/api/ota`).
- `/api/status` JSON gains `ota_poll_sec`, `door_move_sec`, `relay_pulse_ms` so the form populates.

### UI (redesigned page)

- **"Updates" card:** the Auto-update toggle (moved here from Controls), an OTA poll-interval field
  (minutes), a **"Check now"** button (`POST /api/ota-check`).
- **"Door tuning" card:** door-move-timeout (s) and relay-pulse (ms) fields.
- **"System" area:** a **"Reboot device"** button guarded by `confirm("Reboot the device now?")`,
  posting to `/api/reboot`.
- New fields save via the existing Save action (the settings form now posts all five values).
- The page keeps all existing element IDs/behavior; new IDs added for the new fields/buttons.

## Edge cases

- **Clamping:** setters enforce ranges, so a 0 ms pulse, a 1 s move timeout, or a sub-5-min poll
  interval is impossible. The 5-min poll floor prevents hammering GitHub.
- **Change while poller mid-sleep:** the chunked loop re-reads the target each chunk, so a shortened
  interval that's already exceeded by elapsed time polls on the next chunk (~10 s).
- **Reboot mid-move:** harmless — the door is a physical toggle; on boot the firmware re-reads the
  reed sensor and republishes state. Reboot drops HomeKit + the web server for ~15 s while it comes
  back up (documented next to the button via the confirm text).
- **Door-move timeout doubles as the auto-close verify window** — intended; lowering it makes both
  the failsafe and the fault-detection verify quicker.

## Testing / verification

No test framework (per CLAUDE.md); "verify" = builds + behaves on hardware.

- `idf.py build` clean under `-Werror`; image still fits the 1600 KB OTA slot.
- On hardware: change the poll interval → serial log honors the new cadence within ~10 s; "Check
  now" triggers an immediate poll (`OTA` log line); relay-pulse change visibly alters the kick;
  door-move-timeout change alters the failsafe/verify window; Reboot button restarts the device;
  all five values persist across a reboot; auto-close backstops remain fixed (not in the UI).

## Open risks

- **Reading settings on the door task hot path:** `kick_door_relay` / the tick read `settings_*`
  getters (simple RAM reads, no NVS I/O), so no latency concern.
- **UI growth:** the page gains an Updates card, a Door-tuning card, and a Reboot control; keep the
  layout coherent within the existing card grid (the redesign already established the pattern).

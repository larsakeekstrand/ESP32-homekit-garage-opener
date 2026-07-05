# Design: LAN push-OTA (firmware endpoint + Rust CLI)

**Date:** 2026-07-05
**Status:** Approved (brainstorming), pending implementation plan
**Target:** ESP32-homekit-garage-opener firmware (ESP-IDF v5.5.2) + a new Rust CLI tool

## Overview

Let the owner force a locally-built firmware image onto the device on demand (e.g. a beta for
testing) over the LAN, without cutting a GitHub release. A new authenticated `POST /api/ota`
endpoint on the device's web server streams a raw `garage.bin` directly into the inactive OTA
slot; a small Rust CLI (`garage-push`) pushes the file and reports progress. A manual push pauses
the existing hourly GitHub self-poller (persisted flag) so the pushed beta is not silently
reverted; auto-updates are re-enabled from the web page or the CLI.

## Goals

- Push a locally-built `garage.bin` to the device over the LAN with one command.
- Reuse the existing authenticated web server (Basic Auth, `admin` + web password).
- A pushed beta persists (isn't reverted by the self-poller) until auto-updates are re-enabled.
- Guard against flashing the wrong firmware onto the device.
- Leave the existing GitHub self-poller (`ota_update.c`) and rollback safety intact.

## Non-goals

- No remote/off-LAN push, no cloud backend, no MQTT/AWS IoT, no device certificates.
- No HTTPS on the device web server (plain HTTP on the LAN, as with the rest of the web page).
- No firmware signing / secure boot beyond the existing image-integrity + rollback checks.
- No changes to how normal GitHub-release OTA works when auto-updates are enabled.

## Architecture

### Firmware modules

| Module | Change | Responsibility |
|--------|--------|----------------|
| `ota_upload.{c,h}` | **new** | Receive a streamed firmware image and write it to the inactive OTA slot: `esp_ota_begin` → chunked `esp_ota_write` → app-descriptor guard → `esp_ota_end` → `esp_ota_set_boot_partition`. Exposes a small API the web handler calls; keeps all `esp_ota_*` logic out of `web_server.c`. |
| `web_server.{c,h}` | extend | Two thin new handlers: `POST /api/ota` (auth → stream body into `ota_upload` → on success set pause flag, respond, reboot) and `POST /api/auto-ota` (auth → toggle the pause flag). Add auto-update state to `/api/status` + the HTML page. |
| `settings.{c,h}` | extend | Add `auto_ota_paused` (bool, NVS-persisted) with getter/setter, mirroring the existing `lockout` flag. |
| `ota_update.c` | extend | The poll loop checks `settings_get_auto_ota_paused()` each cycle and skips the GitHub version check (and OTA) while paused. `ota_update_mark_valid()` behavior is unchanged. |

`ota_update.c` (the GitHub poller) and `ota_upload.c` (the receive-and-flash path) are deliberately
separate: the poller *pulls* over HTTPS from GitHub; the uploader *receives* a pushed image over
the LAN. They share only the `settings` pause flag.

### The Rust CLI (`tools/garage-push/`)

A standalone Cargo binary crate (not part of the ESP-IDF build). Dependencies: `clap` (arg
parsing, derive), `ureq` (blocking HTTP, minimal deps), `rpassword` (interactive password prompt).

## Detailed design

### 1. `POST /api/ota` — receive and flash (`ota_upload.c` + `web_server.c`)

Handler flow (in `web_server.c`, authenticated via the existing `require_auth`):

1. Reject if an upload is already in progress (a module-level "busy" flag in `ota_upload`) → HTTP 409.
2. `ota_upload_begin()` → `esp_ota_get_next_update_partition(NULL)` + `esp_ota_begin(part, OTA_SIZE_UNKNOWN, &handle)`.
3. Loop: `httpd_req_recv()` into a bounded stack/heap buffer (e.g. 1–4 KB) → `ota_upload_write(buf, n)`
   which calls `esp_ota_write()`. Accumulate the first ~`sizeof(esp_image_header_t) +
   sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)` bytes to validate the image
   descriptor (below) before accepting the rest.
4. **App-descriptor guard:** once enough header bytes are in, locate the `esp_app_desc_t` at
   `offset = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)`. Reject (abort via
   `esp_ota_abort`, HTTP 400) unless `magic_word == ESP_APP_DESC_MAGIC_WORD` **and**
   `strcmp(project_name, "garage") == 0` (the project name is `project(garage)` in the root
   `CMakeLists.txt`). This prevents flashing a non-garage / corrupt image.
5. On the client closing the body: `ota_upload_end()` → `esp_ota_end(handle)` (validates image
   integrity) → `esp_ota_set_boot_partition(part)`.
6. On success: `settings_set_auto_ota_paused(true)` (so the new image won't be reverted), send a
   short `200` body (e.g. `{"status":"ok","rebooting":true}`), then a brief delay + `esp_restart()`.
7. On any `esp_ota_*` error: `esp_ota_abort(handle)`, clear the busy flag, respond with a
   descriptive non-200 status. The current image keeps running (nothing was committed).

`ota_upload.{c,h}` API sketch:

```c
esp_err_t ota_upload_begin(void);                 /* pick slot, esp_ota_begin, set busy */
esp_err_t ota_upload_write(const void *buf, size_t len);  /* esp_ota_write + header capture/guard */
esp_err_t ota_upload_end(void);                   /* esp_ota_end + set_boot_partition */
void      ota_upload_abort(void);                 /* esp_ota_abort + clear busy */
bool      ota_upload_in_progress(void);
```

### 2. `auto_ota_paused` flag (`settings.{c,h}` + `ota_update.c`)

- New NVS-backed bool in `settings`, mirroring `lockout`: `settings_get_auto_ota_paused()` /
  `settings_set_auto_ota_paused(bool)`, key `"ota_paused"`, default `false`, written immediately on change.
- `ota_update.c`'s `ota_task` loop: at the top of each iteration, if
  `settings_get_auto_ota_paused()` is true, log once and `vTaskDelay` the normal interval without
  fetching `version.txt` or doing OTA. `ota_update_mark_valid()` still runs on a healthy boot
  (rollback confirmation must not be gated by the pause).

### 3. `POST /api/auto-ota` + web page (`web_server.c`)

- `POST /api/auto-ota` (authenticated): body `on=0|1`, where `on` is the *auto-update enabled*
  state — `on=1` → `settings_set_auto_ota_paused(false)` (auto-updates on); `on=0` →
  `settings_set_auto_ota_paused(true)` (paused). The web toggle and the CLI both use this same
  `on` convention.
- `/api/status` JSON gains `auto_ota_paused` (bool). The HTML page shows **Auto-update: On / PAUSED**
  with a toggle (like the lockout toggle) and continues to show the running `fw_version` so the
  owner can confirm which beta is live.

### 4. Rust CLI (`tools/garage-push/`)

`clap` subcommands:

- `garage-push push --host <ip> --file <path> [--password <pw>] [--yes]`
  Reads the file, `POST`s it to `http://<host>/api/ota` with Basic Auth (`admin:<pw>`) and an
  `application/octet-stream` body, showing an upload progress indication. Then polls
  `GET /api/status` until the device reboots and reports the pushed `fw_version` (with a timeout),
  printing the new version + that auto-update is now paused.
- `garage-push resume --host <ip> [--password <pw>]`
  `POST /api/auto-ota` to re-enable auto-updates (paused=false).
- `garage-push status --host <ip> [--password <pw>]`
  Pretty-print `/api/status`.

Password resolution order: `--password` → `$GARAGE_PASSWORD` → interactive prompt (`rpassword`).
Host is an IP (or hostname) the owner reads from the router / the web page; no mDNS discovery.

## Safety, security, edge cases

- **Rollback net (unchanged):** a pushed beta that boot-loops never reaches
  `ota_update_mark_valid()` (called ~30 s after a healthy boot), so the bootloader rolls back to the
  previous image. The pause flag does not defeat this — rollback is bootloader-level. A bad beta
  self-recovers.
- **Wrong-firmware guard:** the `project_name == "garage"` + `ESP_APP_DESC_MAGIC_WORD` check rejects
  non-garage or corrupt images before committing.
- **Auth:** the endpoint reuses Basic Auth (`admin` + the NVS web password). Flashing arbitrary
  firmware is a powerful capability, so the endpoint is auth-gated like the rest of the write API.
- **Transport:** plain HTTP on the LAN — the image and password cross the wire in cleartext on the
  local network (accepted; LAN-only, consistent with the existing web page).
- **Concurrency:** a second `/api/ota` while one is in progress returns 409; `esp_ota_write` runs in
  bounded chunks so streaming ~1.2 MB doesn't exhaust the httpd task stack/heap.
- **Reboot response race:** send the HTTP response *before* `esp_restart()` (short delay) so the CLI
  sees success rather than a dropped connection.

## Testing / verification

No unit-test framework exists (per CLAUDE.md); "verify" = builds + behaves on hardware.

- **Firmware:** `idf.py build` clean under `-Werror`; image still fits the 1600 KB OTA slot.
- **On hardware:**
  - `garage-push push` a locally-built beta → device flashes, reboots into it, `/api/status` shows
    the new `fw_version` and **Auto-update: PAUSED**; confirm the GitHub poll is skipped (serial log).
  - `garage-push resume` (or web toggle) → auto-update shows On; the next poll behaves normally.
  - Push an image built from a *different* `project()` name (or a truncated file) → rejected with a
    clear error, device keeps running the current image.
  - Push a deliberately-broken beta (e.g. panics on boot) → bootloader rolls back to the prior image.
- **CLI:** `cargo build` clean; `cargo clippy` clean; a real `push`/`resume`/`status` round-trip
  against the device.

## Open risks

- **httpd upload throughput/timeouts:** streaming ~1.2 MB through `esp_http_server` on a single core
  while HAP/door tasks run; keep `httpd` recv timeouts generous and chunks bounded. Validate on hardware.
- **esp_app_desc offset assumptions:** the app-descriptor location is derived from the standard
  image layout; confirm against the ESP-IDF headers during implementation (`esp_app_format.h`).
- **CLI as a new build artifact:** `tools/garage-push/` is a separate Cargo crate; it is a dev tool,
  not shipped in `garage.bin`, and is excluded from the ESP-IDF build.

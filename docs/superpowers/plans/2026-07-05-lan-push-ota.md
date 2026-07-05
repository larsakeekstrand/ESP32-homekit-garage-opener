# LAN Push-OTA Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the owner push a locally-built `garage.bin` onto the device over the LAN on demand (a "beta") via an authenticated `POST /api/ota`, pausing the GitHub self-poller so the beta sticks, driven by a small Rust CLI.

**Architecture:** A new `ota_upload` module wraps `esp_ota_*` to receive a streamed image into the inactive slot with a `project_name`/magic guard. The existing `web_server` gains a thin `POST /api/ota` (stream → flash → set pause flag → reboot) and `POST /api/auto-ota` (toggle pause). A persisted `auto_ota_paused` flag in `settings` gates the existing `ota_update.c` poller. A standalone Rust CLI (`tools/garage-push/`) pushes the file and re-enables auto-updates.

**Tech Stack:** ESP-IDF v5.5.2 (`esp_ota_ops`, `esp_http_server`, NVS), Rust (`clap`, `ureq`, `rpassword`, `base64`, `serde_json`).

## Global Constraints

- ESP-IDF **v5.5.2**, target **esp32**. App sources compile with **`-Werror`** — any warning fails the build.
- **No test framework** in this repo. Firmware "verify" = `idf.py build` succeeds under `-Werror` + behaves on hardware (on-hardware checks are the USER's, deferred). CLI "verify" = `cargo build` + `cargo clippy` clean.
- Final image must fit the **1600 KB** OTA slot.
- Web server is **plain HTTP, port 80, LAN-only**; endpoints reuse the existing Basic Auth (`admin` + NVS web password) via `require_auth`.
- No secret baked into the image. Single-core FreeRTOS; keep HTTP handlers bounded/non-blocking.
- The device project name is **`garage`** (`project(garage)` in the root `CMakeLists.txt`) — the upload guard checks this.
- Bench-build note: build betas with `-DFW_VERSION=<v>`; a `0.0.0-dev` build is treated as older than the latest release by the poller.
- Match existing style: C block comment per function; `ESP_LOGI/W/E` with a module tag.

## File Structure

| File | Create/Modify | Responsibility |
|------|---------------|----------------|
| `main/settings.{c,h}` | Modify | Add `auto_ota_paused` (NVS bool), mirroring `lockout`. |
| `main/ota_update.c` | Modify | Poller skips the GitHub check while `settings_get_auto_ota_paused()`. |
| `main/ota_upload.{c,h}` | **Create** | Receive-and-flash: `esp_ota_begin` → chunked `esp_ota_write` + app-desc guard → `esp_ota_end` → `esp_ota_set_boot_partition`. |
| `main/web_server.c` | Modify | `POST /api/ota`, `POST /api/auto-ota`; `auto_ota_paused` in `/api/status` + page toggle; register handlers. |
| `main/CMakeLists.txt` | Modify | Add `ota_upload.c` to `SRCS`. |
| `tools/garage-push/` | **Create** | Rust CLI (`Cargo.toml`, `src/main.rs`): `push` / `resume` / `status`. |
| `README.md` | Modify | Document push-OTA + the CLI. |
| `.gitignore` | Modify | Ignore `tools/garage-push/target/`. |

**Task order:** Task 1 (settings flag) → Task 2 (poller honors flag) → Task 3 (`ota_upload` module) → Task 4 (web endpoints + page) → Task 5 (Rust CLI) → Task 6 (docs). Tasks 1–2 and 3 are independent; Task 4 depends on 1 & 3; Task 5 depends only on the HTTP contract; Task 6 last.

---

### Task 1: `settings` — `auto_ota_paused` flag

**Files:**
- Modify: `main/settings.h`, `main/settings.c`

**Interfaces:**
- Produces (used by Tasks 2 & 4): `bool settings_get_auto_ota_paused(void);` / `void settings_set_auto_ota_paused(bool paused);`

- [ ] **Step 1: Declare the API in `main/settings.h`**

After the lockout declarations (the `void settings_set_lockout(bool on);` line), add:

```c
/* Auto-OTA pause flag (persisted). When true, the GitHub self-poller is
 * suspended so a hand-pushed beta is not reverted. */
bool settings_get_auto_ota_paused(void);
void settings_set_auto_ota_paused(bool paused);
```

- [ ] **Step 2: Add the backing state + default in `main/settings.c`**

Find the module-level statics (near `static bool s_lockout;`) and add below it:
```c
static bool s_ota_paused;
```

In `settings_init()`, in the defaults block (after `s_lockout = false;`), add:
```c
  s_ota_paused = false;
```

In `settings_init()`, in the NVS-load block (after the `if (nvs_get_u8(h, "lockout", &lk) == ESP_OK) s_lockout = lk;` line), add:
```c
    uint8_t op = 0;
    if (nvs_get_u8(h, "ota_paused", &op) == ESP_OK) s_ota_paused = op;
```

Update the load log line to include it — change:
```c
  ESP_LOGI(TAG, "loaded: auto_close=%u left_open=%u lockout=%d pw_set=%d",
           s_auto_close_sec, s_left_open_sec, s_lockout, s_pw_set);
```
to:
```c
  ESP_LOGI(TAG, "loaded: auto_close=%u left_open=%u lockout=%d ota_paused=%d pw_set=%d",
           s_auto_close_sec, s_left_open_sec, s_lockout, s_ota_paused, s_pw_set);
```

- [ ] **Step 3: Add the getter/setter in `main/settings.c`**

After `settings_set_lockout()` (right before the `counters_flush` comment block), add:
```c
bool settings_get_auto_ota_paused(void) { return s_ota_paused; }

void settings_set_auto_ota_paused(bool paused) {
  s_ota_paused = paused;
  nvs_handle_t h;
  ns_open(&h, NVS_READWRITE);
  if (h) { nvs_set_u8(h, "ota_paused", paused ? 1 : 0); nvs_commit(h); nvs_close(h); }
  ESP_LOGI(TAG, "auto_ota_paused = %d", paused);
}
```

- [ ] **Step 4: Build**

Run:
```bash
. ~/esp/esp-idf/export.sh
idf.py build
```
Expected: compiles clean under `-Werror`. (Nothing calls the new API yet — pure foundation.)

- [ ] **Step 5: Commit**

```bash
git add main/settings.h main/settings.c
git commit -m "feat(settings): add persisted auto_ota_paused flag"
```

---

### Task 2: `ota_update` poller honors the pause flag

**Files:**
- Modify: `main/ota_update.c`

**Interfaces:**
- Consumes (Task 1): `settings_get_auto_ota_paused()`.

- [ ] **Step 1: Include settings in `main/ota_update.c`**

Add to the includes (after `#include "ota_update.h"`):
```c
#include "settings.h"
```

- [ ] **Step 2: Skip the GitHub check while paused**

In `ota_task()`, inside the `while (1)` loop, as the FIRST statements of the loop body (before `char remote[32];`), add:
```c
        if (settings_get_auto_ota_paused()) {
            ESP_LOGI(TAG, "Auto-OTA paused; skipping GitHub check");
            vTaskDelay(POLL_INTERVAL_MS / portTICK_PERIOD_MS);
            continue;
        }
```

(Ordering is safe: `ota_task` sleeps `STABLE_BOOT_DELAY_MS` = 30 s before its first loop iteration, and `settings_init()` runs early in the garage task, so the flag is loaded well before the first read. `ota_update_mark_valid()` still runs before the loop, so rollback confirmation is never gated by the pause.)

- [ ] **Step 3: Build**

Run: `idf.py build`
Expected: clean under `-Werror`.

- [ ] **Step 4: Commit**

```bash
git add main/ota_update.c
git commit -m "feat(ota): self-poller skips GitHub check while auto-OTA is paused"
```

---

### Task 3: `ota_upload` module — receive and flash

**Files:**
- Create: `main/ota_upload.h`, `main/ota_upload.c`
- Modify: `main/CMakeLists.txt`

**Interfaces:**
- Produces (used by Task 4):
  - `esp_err_t ota_upload_begin(void);`
  - `esp_err_t ota_upload_write(const void *data, size_t len);`
  - `esp_err_t ota_upload_end(void);`
  - `void ota_upload_abort(void);`
  - `bool ota_upload_in_progress(void);`

- [ ] **Step 1: Write `main/ota_upload.h`**

```c
//
// ota_upload.h — receive a streamed firmware image and flash the inactive OTA slot.
//
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/* Begin an upload: pick the next OTA partition and open an esp_ota session.
 * Returns ESP_ERR_INVALID_STATE if one is already in progress. */
esp_err_t ota_upload_begin(void);

/* Feed the next chunk of the image. Once enough header bytes have arrived it
 * validates the app descriptor (magic + project_name == "garage") and returns
 * ESP_ERR_INVALID_ARG on mismatch. */
esp_err_t ota_upload_write(const void *data, size_t len);

/* Finalize: validate the image (esp_ota_end) and set it as the boot partition.
 * The caller reboots afterwards. */
esp_err_t ota_upload_end(void);

/* Abort an in-progress upload (esp_ota_abort) and clear the busy state. */
void ota_upload_abort(void);

/* True while an upload session is open. */
bool ota_upload_in_progress(void);
```

- [ ] **Step 2: Write `main/ota_upload.c`**

```c
//
// ota_upload.c — receive a streamed firmware image and flash the inactive OTA slot.
//
#include <string.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_app_format.h>
#include <esp_image_format.h>

#include "ota_upload.h"

static const char *TAG = "OTA-Upload";

/* The app descriptor sits right after the image header + first segment header. */
#define APP_DESC_OFFSET (sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t))
#define HEADER_CAPTURE  (APP_DESC_OFFSET + sizeof(esp_app_desc_t))
#define EXPECT_PROJECT  "garage"

static bool                 s_busy;
static esp_ota_handle_t     s_handle;
static const esp_partition_t *s_part;
static uint8_t              s_hdr[HEADER_CAPTURE];
static size_t               s_hdr_len;
static bool                 s_hdr_checked;

bool ota_upload_in_progress(void) { return s_busy; }

esp_err_t ota_upload_begin(void) {
  if (s_busy) return ESP_ERR_INVALID_STATE;
  s_part = esp_ota_get_next_update_partition(NULL);
  if (!s_part) { ESP_LOGE(TAG, "no OTA partition available"); return ESP_FAIL; }
  esp_err_t e = esp_ota_begin(s_part, OTA_SIZE_UNKNOWN, &s_handle);
  if (e != ESP_OK) { ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(e)); return e; }
  s_hdr_len = 0;
  s_hdr_checked = false;
  s_busy = true;
  ESP_LOGI(TAG, "OTA upload begin -> partition '%s'", s_part->label);
  return ESP_OK;
}

//
// Validate the captured app descriptor: magic word + project name.
//
static esp_err_t check_header(void) {
  const esp_app_desc_t *d = (const esp_app_desc_t *)(s_hdr + APP_DESC_OFFSET);
  if (d->magic_word != ESP_APP_DESC_MAGIC_WORD) {
    ESP_LOGE(TAG, "bad app-desc magic 0x%08x", (unsigned)d->magic_word);
    return ESP_ERR_INVALID_ARG;
  }
  if (strncmp(d->project_name, EXPECT_PROJECT, sizeof(d->project_name)) != 0) {
    ESP_LOGE(TAG, "wrong project_name '%.*s' (expected '%s')",
             (int)sizeof(d->project_name), d->project_name, EXPECT_PROJECT);
    return ESP_ERR_INVALID_ARG;
  }
  ESP_LOGI(TAG, "image accepted: project='%s' version='%.*s'",
           EXPECT_PROJECT, (int)sizeof(d->version), d->version);
  s_hdr_checked = true;
  return ESP_OK;
}

esp_err_t ota_upload_write(const void *data, size_t len) {
  if (!s_busy) return ESP_ERR_INVALID_STATE;

  /* Capture the leading header bytes for the app-desc guard (a copy — the full
   * chunk is still written below). Validate as soon as we have enough. */
  if (s_hdr_len < HEADER_CAPTURE) {
    size_t n = HEADER_CAPTURE - s_hdr_len;
    if (n > len) n = len;
    memcpy(s_hdr + s_hdr_len, data, n);
    s_hdr_len += n;
    if (s_hdr_len >= HEADER_CAPTURE && !s_hdr_checked) {
      esp_err_t e = check_header();
      if (e != ESP_OK) return e;
    }
  }
  return esp_ota_write(s_handle, data, len);
}

esp_err_t ota_upload_end(void) {
  if (!s_busy) return ESP_ERR_INVALID_STATE;
  if (!s_hdr_checked) {          /* image too small to even contain a descriptor */
    ESP_LOGE(TAG, "image too small / descriptor never validated");
    ota_upload_abort();
    return ESP_ERR_INVALID_SIZE;
  }
  esp_err_t e = esp_ota_end(s_handle);
  if (e != ESP_OK) { ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(e)); s_busy = false; return e; }
  e = esp_ota_set_boot_partition(s_part);
  if (e != ESP_OK) ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(e));
  s_busy = false;
  return e;
}

void ota_upload_abort(void) {
  if (s_busy) { esp_ota_abort(s_handle); s_busy = false; ESP_LOGW(TAG, "OTA upload aborted"); }
}
```

- [ ] **Step 3: Add `ota_upload.c` to the build**

In `main/CMakeLists.txt`, append `"ota_upload.c"` to the `SRCS` list:
```cmake
idf_component_register(SRCS "app_main.c" "hap_garage.c" "dht_sensor.c" "door_control.c" "ota_update.c" "settings.c" "web_server.c" "ota_upload.c"
                       INCLUDE_DIRS ".")
```

- [ ] **Step 4: Build**

Run: `idf.py build`
Expected: clean under `-Werror`. (Public API is declared in the header, so no unused-symbol warnings even though nothing calls it yet.) If `esp_image_header_t` / `esp_image_segment_header_t` aren't found, confirm the include is `#include "esp_image_format.h"` (ESP-IDF v5.5) and that `esp_app_format.h` provides `esp_app_desc_t` + `ESP_APP_DESC_MAGIC_WORD`; adjust includes to match the installed headers and note it.

- [ ] **Step 5: Commit**

```bash
git add main/ota_upload.h main/ota_upload.c main/CMakeLists.txt
git commit -m "feat(ota): add ota_upload module (receive + flash with project-name guard)"
```

---

### Task 4: `web_server` — upload + pause endpoints + page toggle

**Files:**
- Modify: `main/web_server.c`

**Interfaces:**
- Consumes: Task 1 (`settings_get/set_auto_ota_paused`), Task 3 (`ota_upload_*`).

- [ ] **Step 1: Add includes to `main/web_server.c`**

After the existing includes block (after `#include "dht_sensor.h"`), add:
```c
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ota_upload.h"
```

- [ ] **Step 2: Add `auto_ota_paused` to the status JSON**

In `status_get()`: enlarge the buffer and add the field. Change `static char buf[720];` to:
```c
  static char buf[800];
```
In the `snprintf` format string, change the tail
```c
    "\"auto_close_faulted\":%s,\"auto_close_sec\":%u,\"left_open_sec\":%u}",
```
to:
```c
    "\"auto_close_faulted\":%s,\"auto_close_sec\":%u,\"left_open_sec\":%u,"
    "\"auto_ota_paused\":%s}",
```
and change the final argument line
```c
    (unsigned)settings_get_auto_close_sec(), (unsigned)settings_get_left_open_sec());
```
to:
```c
    (unsigned)settings_get_auto_close_sec(), (unsigned)settings_get_left_open_sec(),
    settings_get_auto_ota_paused() ? "true" : "false");
```

- [ ] **Step 3: Add the two handlers**

Add these two functions after `password_post()` (before the `// ---- lifecycle` comment):

```c
//
// POST /api/ota — stream a firmware image into the inactive slot, then reboot.
//
static esp_err_t ota_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  if (ota_upload_in_progress()) {
    httpd_resp_set_status(req, "409 Conflict");
    httpd_resp_sendstr(req, "an OTA upload is already in progress");
    return ESP_OK;
  }
  if (ota_upload_begin() != ESP_OK) { httpd_resp_send_500(req); return ESP_OK; }

  char buf[1024];
  int remaining = req->content_len;
  esp_err_t err = ESP_OK;
  while (remaining > 0) {
    int want = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
    int r = httpd_req_recv(req, buf, want);
    if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (r <= 0) { err = ESP_FAIL; break; }
    err = ota_upload_write(buf, r);
    if (err != ESP_OK) break;
    remaining -= r;
  }
  if (err != ESP_OK) {
    ota_upload_abort();
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "OTA write/validation failed (wrong or corrupt image?)");
    return ESP_OK;
  }
  if (ota_upload_end() != ESP_OK) {
    ota_upload_abort();
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "OTA finalize failed (image invalid or too small)");
    return ESP_OK;
  }

  settings_set_auto_ota_paused(true);   /* so the poller won't revert the beta */
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"status\":\"ok\",\"rebooting\":true}");
  ESP_LOGI(TAG, "OTA upload applied; rebooting into new image");
  vTaskDelay(pdMS_TO_TICKS(500));       /* let the response flush before reboot */
  esp_restart();
  return ESP_OK;                        /* not reached */
}

//
// POST /api/auto-ota — enable/disable the GitHub self-poller. Body: on=0|1
// (on=1 => auto-updates enabled/paused=false; on=0 => paused=true).
//
static esp_err_t auto_ota_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  char buf[32];
  if (read_body(req, buf, sizeof(buf)) < 0) { httpd_resp_send_500(req); return ESP_OK; }
  char v[8];
  bool enabled = !settings_get_auto_ota_paused();
  if (httpd_query_key_value(buf, "on", v, sizeof(v)) == ESP_OK) enabled = (atoi(v) != 0);
  settings_set_auto_ota_paused(!enabled);
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}
```

- [ ] **Step 4: Register the handlers**

In `web_server_start()`, raise the handler cap — change `config.max_uri_handlers = 8;` to:
```c
  config.max_uri_handlers = 10;
```
and add two rows to the `uris[]` array (after the `/api/password` row):
```c
    { .uri = "/api/ota",        .method = HTTP_POST, .handler = ota_post },
    { .uri = "/api/auto-ota",   .method = HTTP_POST, .handler = auto_ota_post },
```

- [ ] **Step 5: Add the Auto-update toggle to the page**

In `PAGE_HTML`, add a row to the System card — change:
```c
"<tr><td>Reset reason</td><td id=rst>…</td></tr></table></div>"
```
to:
```c
"<tr><td>Reset reason</td><td id=rst>…</td></tr>"
"<tr><td>Auto-update</td><td><input type=checkbox id=autoota onchange=\"post('/api/auto-ota','on='+(this.checked?1:0))\"> <span id=autoota_l></span></td></tr></table></div>"
```
In the `refresh()` JS, after the `rst.textContent=d.reset_reason;` line, add:
```c
"autoota.checked=!d.auto_ota_paused;autoota_l.textContent=d.auto_ota_paused?'PAUSED':'on';"
```

- [ ] **Step 6: Build**

Run: `idf.py build`
Expected: clean under `-Werror`; image still fits the 1600 KB slot (`idf.py size`).

- [ ] **Step 7: On-hardware verification (USER)**

Flash with a bumped version so the poller doesn't immediately act:
```bash
idf.py -DFW_VERSION=9.9.9 flash monitor
```
- Browse to `http://<device-ip>/` → the System card shows **Auto-update: on**. Toggle it → serial logs `auto_ota_paused = 1`, page shows PAUSED; poller logs `Auto-OTA paused; skipping GitHub check` on its next cycle.
- (Full upload path is exercised end-to-end with the CLI in Task 5.)

- [ ] **Step 8: Commit**

```bash
git add main/web_server.c
git commit -m "feat(web): POST /api/ota upload + /api/auto-ota toggle + status/page auto-update state"
```

---

### Task 5: Rust CLI `tools/garage-push/`

**Files:**
- Create: `tools/garage-push/Cargo.toml`, `tools/garage-push/src/main.rs`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: the device HTTP contract — `POST /api/ota` (octet-stream body, Basic Auth), `POST /api/auto-ota` (`on=0|1`), `GET /api/status` (JSON with `fw_version`, `auto_ota_paused`, `uptime_seconds`).

- [ ] **Step 1: Write `tools/garage-push/Cargo.toml`**

```toml
[package]
name = "garage-push"
version = "0.1.0"
edition = "2021"
description = "Push a locally-built firmware image to the ESP32 garage opener over the LAN"

[dependencies]
clap = { version = "4", features = ["derive"] }
ureq = "2"
rpassword = "7"
base64 = "0.22"
serde_json = "1"

[[bin]]
name = "garage-push"
path = "src/main.rs"
```

- [ ] **Step 2: Write `tools/garage-push/src/main.rs`**

```rust
//! garage-push — push a locally-built firmware image to the ESP32 garage opener
//! over the LAN, and manage the device's auto-update pause flag.

use base64::Engine;
use clap::{Parser, Subcommand};
use std::time::{Duration, Instant};

#[derive(Parser)]
#[command(name = "garage-push", about = "Push firmware to the ESP32 garage opener over LAN")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Upload a firmware image and reboot the device into it (pauses auto-updates).
    Push {
        #[arg(long)]
        host: String,
        #[arg(long)]
        file: String,
        #[arg(long)]
        password: Option<String>,
    },
    /// Re-enable auto-updates (the GitHub self-poller).
    Resume {
        #[arg(long)]
        host: String,
        #[arg(long)]
        password: Option<String>,
    },
    /// Print the device status JSON.
    Status {
        #[arg(long)]
        host: String,
        #[arg(long)]
        password: Option<String>,
    },
}

fn resolve_password(opt: Option<String>) -> String {
    if let Some(p) = opt {
        return p;
    }
    if let Ok(p) = std::env::var("GARAGE_PASSWORD") {
        return p;
    }
    rpassword::prompt_password("Device password: ").expect("failed to read password")
}

fn auth_header(pw: &str) -> String {
    let cred = base64::engine::general_purpose::STANDARD.encode(format!("admin:{pw}"));
    format!("Basic {cred}")
}

fn main() {
    let cli = Cli::parse();
    let result = match cli.cmd {
        Cmd::Push { host, file, password } => push(&host, &file, resolve_password(password)),
        Cmd::Resume { host, password } => resume(&host, resolve_password(password)),
        Cmd::Status { host, password } => status(&host, resolve_password(password)),
    };
    if let Err(e) = result {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn push(host: &str, file: &str, pw: String) -> Result<(), String> {
    let data = std::fs::read(file).map_err(|e| format!("reading {file}: {e}"))?;
    let auth = auth_header(&pw);
    let url = format!("http://{host}/api/ota");
    eprintln!("Uploading {} ({} bytes) to {url} ...", file, data.len());

    match ureq::post(&url)
        .set("Authorization", &auth)
        .set("Content-Type", "application/octet-stream")
        .send_bytes(&data)
    {
        Ok(resp) => {
            let body = resp.into_string().unwrap_or_default();
            eprintln!("Device accepted the image: {body}");
        }
        Err(ureq::Error::Status(code, resp)) => {
            let body = resp.into_string().unwrap_or_default();
            return Err(format!("device rejected upload (HTTP {code}): {body}"));
        }
        Err(e) => return Err(format!("upload failed: {e}")),
    }

    eprintln!("Waiting for the device to reboot into the new image ...");
    wait_for_reboot(host, &auth)
}

fn resume(host: &str, pw: String) -> Result<(), String> {
    let auth = auth_header(&pw);
    let url = format!("http://{host}/api/auto-ota");
    ureq::post(&url)
        .set("Authorization", &auth)
        .send_string("on=1")
        .map_err(|e| format!("resume failed: {e}"))?;
    println!("Auto-updates re-enabled.");
    Ok(())
}

fn status(host: &str, pw: String) -> Result<(), String> {
    let auth = auth_header(&pw);
    let json = fetch_status(host, &auth)?;
    println!("{}", serde_json::to_string_pretty(&json).unwrap_or_else(|_| json.to_string()));
    Ok(())
}

fn fetch_status(host: &str, auth: &str) -> Result<serde_json::Value, String> {
    let url = format!("http://{host}/api/status");
    let body = ureq::get(&url)
        .set("Authorization", auth)
        .call()
        .map_err(|e| format!("status request failed: {e}"))?
        .into_string()
        .map_err(|e| format!("reading status body: {e}"))?;
    serde_json::from_str(&body).map_err(|e| format!("parsing status JSON: {e}"))
}

/// Poll /api/status until the device comes back up, then print its version.
fn wait_for_reboot(host: &str, auth: &str) -> Result<(), String> {
    // Give the device a moment to actually drop the connection and reboot.
    std::thread::sleep(Duration::from_secs(3));
    let deadline = Instant::now() + Duration::from_secs(90);
    while Instant::now() < deadline {
        if let Ok(json) = fetch_status(host, auth) {
            let ver = json.get("fw_version").and_then(|v| v.as_str()).unwrap_or("?");
            let paused = json
                .get("auto_ota_paused")
                .and_then(|v| v.as_bool())
                .unwrap_or(false);
            println!("Device is up. Running firmware {ver}. Auto-updates: {}.",
                     if paused { "PAUSED" } else { "on" });
            println!("Run `garage-push resume --host {host}` to re-enable auto-updates.");
            return Ok(());
        }
        std::thread::sleep(Duration::from_secs(2));
    }
    Err("device did not come back online within 90s (check it manually)".into())
}
```

- [ ] **Step 3: Ignore the Cargo build dir**

Append to `.gitignore`:
```
# Rust CLI build output
tools/garage-push/target/
```

- [ ] **Step 4: Build + lint the CLI**

Run (Rust toolchain required; if `cargo` is absent, report NEEDS_CONTEXT):
```bash
cd tools/garage-push
cargo build
cargo clippy -- -D warnings
```
Expected: builds clean; clippy reports no warnings.

- [ ] **Step 5: On-hardware round-trip (USER)**

```bash
cd tools/garage-push
cargo run -- push --host <device-ip> --file ../../build/garage.bin
# ... device flashes, reboots, prints new version + "Auto-updates: PAUSED"
cargo run -- status --host <device-ip>
cargo run -- resume --host <device-ip>
```

- [ ] **Step 6: Commit**

```bash
git add tools/garage-push/Cargo.toml tools/garage-push/src/main.rs .gitignore
git commit -m "feat(tools): add garage-push Rust CLI (push/resume/status)"
```

---

### Task 6: Documentation

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Document push-OTA + the CLI in `README.md`**

Read `README.md` and add a subsection (fitting the existing style) covering:
- **Forced/beta OTA over LAN:** `garage-push push --host <ip> --file build/garage.bin` uploads a locally-built image to `POST /api/ota` (authenticated with the web password); the device validates it's a `garage` image, flashes the inactive slot, and reboots into it. A bad image is rejected; a boot-looping image rolls back automatically.
- **Auto-update pause:** a push automatically pauses the hourly GitHub self-poller (so the beta isn't reverted). Re-enable with `garage-push resume --host <ip>` or the **Auto-update** toggle on the web page.
- **CLI:** located at `tools/garage-push/` (Rust; `cargo build`). Subcommands `push`, `resume`, `status`. Password via `--password`, `$GARAGE_PASSWORD`, or prompt.
- **Build a beta with a version:** `idf.py -DFW_VERSION=<v> build` (a `0.0.0-dev` build is treated as older than the latest release by the poller).

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: document LAN push-OTA and the garage-push CLI"
```

---

## Self-Review

**Spec coverage:**
- `POST /api/ota` receive+flash with `esp_ota_*` → Task 3 (module) + Task 4 (handler). ✓
- App-descriptor guard (`project_name == "garage"` + magic) → Task 3 `check_header`. ✓
- Set `auto_ota_paused` on success + reboot after response → Task 4 `ota_post`. ✓
- `auto_ota_paused` flag persisted, mirrors lockout → Task 1. ✓
- Poller honors the flag; `mark_valid` not gated → Task 2. ✓
- `POST /api/auto-ota` + status field + page toggle → Task 4. ✓
- Rust CLI (`push`/`resume`/`status`, `ureq`, password resolution) → Task 5. ✓
- 409 on concurrent upload; bounded chunks; response-before-reboot → Task 4. ✓
- Rollback net unchanged (no edit to `ota_update_mark_valid`) → confirmed (untouched). ✓
- Docs → Task 6. ✓

**Placeholder scan:** No TBD/TODO; every code step has complete code. Task 6 lists exact points to document.

**Type consistency:** `settings_get/set_auto_ota_paused` (bool) consistent across settings.h, ota_update.c, web_server.c. `ota_upload_*` signatures match between ota_upload.h and the Task 4 caller. JSON key `auto_ota_paused` matches between `status_get`, the page JS, and the CLI (`fetch_status`/`wait_for_reboot`). The `/api/auto-ota` `on` convention (1=enabled) matches between `auto_ota_post`, the page toggle, and the CLI `resume` (`on=1`).

**Risk to watch (Task 3):** the `esp_app_desc_t` offset and header names (`esp_image_header_t`, `esp_image_segment_header_t`, `ESP_APP_DESC_MAGIC_WORD`) — Step 4 says to confirm against the ESP-IDF v5.5 headers and adjust includes if the build can't find them.

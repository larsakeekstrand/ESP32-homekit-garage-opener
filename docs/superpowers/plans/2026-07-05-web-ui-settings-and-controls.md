# Web UI Settings + Device Controls Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the OTA poll interval, door-move timeout, and relay pulse length editable from the web page (persisted, near-immediate), and add "Check for updates now" + "Reboot" controls.

**Architecture:** Three new NVS fields in the `settings` module (Kconfig defaults as seeds); consumers (`ota_update.c`, `door_control.c`) read them at runtime. `ota_update` switches to a chunked poll loop so interval changes apply within ~10 s without reboot, plus an `ota_update_check_now()` trigger. `web_server` gains extended `/api/settings` parsing and `/api/ota-check` + `/api/reboot` endpoints. The embedded web page moves to a maintainable source (`tools/web/*.html`) regenerated into `web_server.c` by a converter script.

**Tech Stack:** ESP-IDF v5.5.2 (NVS, esp_http_server, esp_timer, FreeRTOS), Python 3 (page generator, dev-only).

## Global Constraints

- ESP-IDF **v5.5.2**, target **esp32**, `-Werror` — any warning fails the build.
- **No test framework** — firmware "verify" = `idf.py build` clean + on-hardware behavior (on-hardware steps are the USER's, deferred). The web page is verified by rendering the served bytes.
- Final image must fit the **1600 KB** OTA slot.
- Web server is HTTP :8080, LAN-only; endpoints reuse the existing Basic Auth (`require_auth`) and `read_body` helpers.
- Ranges (clamped by setters): OTA poll **300–86400 s** (UI minutes 5–1440), door-move **3–30 s**, relay pulse **100–2000 ms**.
- Keep the auto-close backstops (max 3 fails / 10 per hour / 60 s gap) **compiled-in** — not exposed.
- This work rides on the `feat/web-ui-redesign` branch (the redesigned page); the redesign + these settings ship together.
- Match existing style: C block comment per function; `ESP_LOGI/W/E` with the module tag; `HOMEKIT_CHARACTERISTIC_*` macros not magic numbers.

## File Structure

| File | Change | Responsibility |
|------|--------|----------------|
| `main/settings.{c,h}` | Modify | Add `ota_poll_sec`, `door_move_sec`, `relay_pulse_ms` (NVS, clamped getters/setters). |
| `main/Kconfig.projbuild` | Modify | Add `CONFIG_DOOR_MOVE_TIMEOUT_SEC` (8), `CONFIG_RELAY_PULSE_MS` (300) defaults. |
| `main/ota_update.{c,h}` | Modify | Chunked poll honoring `ota_poll_sec`; `ota_update_check_now()`. |
| `main/door_control.c` | Modify | Read door-move timeout + relay pulse from settings. |
| `main/web_server.c` | Modify | Extend `/api/settings`; add `/api/ota-check`, `/api/reboot`; 3 new status fields; regenerated page. |
| `tools/gen_web_page.py` | Create | Regenerate the embedded page strings in `web_server.c` from `tools/web/*.html`. |
| `tools/web/page.html`, `tools/web/setup.html` | Create | Canonical HTML source for the two pages (edited as real HTML). |
| `README.md` | Modify | Document the new settings + controls + the page-regeneration workflow. |

**Task order:** 1 settings → 2 ota_update → 3 door_control → 4 web_server backend → 5 web UI (source + converter + new controls) → 6 docs. Tasks 1–3 are independent; 4 depends on 1 & 2; 5 depends on 4; 6 last.

---

### Task 1: `settings` — three runtime tunables

**Files:** Modify `main/settings.h`, `main/settings.c`, `main/Kconfig.projbuild`

**Interfaces — Produces (used by Tasks 2–4):**
```c
uint32_t settings_get_ota_poll_sec(void);   void settings_set_ota_poll_sec(uint32_t sec);
uint32_t settings_get_door_move_sec(void);   void settings_set_door_move_sec(uint32_t sec);
uint32_t settings_get_relay_pulse_ms(void);  void settings_set_relay_pulse_ms(uint32_t ms);
```

- [ ] **Step 1: Kconfig defaults**

In `main/Kconfig.projbuild`, before the final `endmenu`, add:
```
    config DOOR_MOVE_TIMEOUT_SEC
        int "Door-move timeout / auto-close verify window (seconds)"
        default 8
        help
            How long a move may take before the failsafe trusts the sensor. Seed
            value; runtime changes are stored in NVS and take precedence.

    config RELAY_PULSE_MS
        int "Relay pulse length (milliseconds)"
        default 300
        help
            How long the relay is held to 'kick' the door. Seed value; runtime
            changes are stored in NVS and take precedence.
```

- [ ] **Step 2: Declare the API in `main/settings.h`**

After the `auto_ota_paused` declarations, add:
```c
/* Runtime tunables (persisted). Setters clamp to range. */
uint32_t settings_get_ota_poll_sec(void);   /* 300..86400 */
void     settings_set_ota_poll_sec(uint32_t sec);
uint32_t settings_get_door_move_sec(void);  /* 3..30 */
void     settings_set_door_move_sec(uint32_t sec);
uint32_t settings_get_relay_pulse_ms(void); /* 100..2000 */
void     settings_set_relay_pulse_ms(uint32_t ms);
```

- [ ] **Step 3: Backing state, defaults, load in `main/settings.c`**

Add a generic clamp helper near `clamp_timeout`:
```c
static uint32_t clampu(uint32_t v, uint32_t lo, uint32_t hi) {
  return v < lo ? lo : v > hi ? hi : v;
}
```
Add module statics (near `s_ota_paused`):
```c
static uint32_t s_ota_poll_sec;
static uint32_t s_door_move_sec;
static uint32_t s_relay_pulse_ms;
```
In `settings_init()` defaults block (after `s_ota_paused = false;`):
```c
  s_ota_poll_sec   = CONFIG_OTA_POLL_INTERVAL_SEC;
  s_door_move_sec  = CONFIG_DOOR_MOVE_TIMEOUT_SEC;
  s_relay_pulse_ms = CONFIG_RELAY_PULSE_MS;
```
In `settings_init()` NVS-load block (after the `ota_paused` load):
```c
    nvs_get_u32(h, "ota_poll",  &s_ota_poll_sec);
    nvs_get_u32(h, "door_move", &s_door_move_sec);
    nvs_get_u32(h, "relay_ms",  &s_relay_pulse_ms);
```

- [ ] **Step 4: Getters/setters in `main/settings.c`**

After `settings_set_auto_ota_paused()`, add:
```c
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
```

- [ ] **Step 5: Build**

`. ~/esp/esp-idf/export.sh && idf.py build` → clean under `-Werror`. (Ignore clangd false positives.)

- [ ] **Step 6: Commit**
```bash
git add main/settings.h main/settings.c main/Kconfig.projbuild
git commit -m "feat(settings): add ota_poll_sec, door_move_sec, relay_pulse_ms tunables"
```

---

### Task 2: `ota_update` — chunked poll + check-now

**Files:** Modify `main/ota_update.h`, `main/ota_update.c`

**Interfaces:**
- Consumes (Task 1): `settings_get_ota_poll_sec()`, `settings_get_auto_ota_paused()`.
- Produces (Task 4): `void ota_update_check_now(void);`

- [ ] **Step 1: Declare `ota_update_check_now` in `main/ota_update.h`**

Add:
```c
/* Request an immediate one-off version check on the next poll chunk (~<=10 s).
 * No effect while auto-updates are paused. */
void ota_update_check_now(void);
```

- [ ] **Step 2: Chunked poll loop in `main/ota_update.c`**

Add near the top (after the `POLL_INTERVAL_MS` define — which is now unused; you may leave or remove it):
```c
#define OTA_POLL_CHUNK_SEC 10           /* granularity for interval changes / check-now */
static volatile bool s_check_now = false;

void ota_update_check_now(void) { s_check_now = true; }
```
Replace the entire `while (1) { ... }` body of `ota_task()` with:
```c
    uint32_t elapsed = 0;               /* seconds accumulated since the last check */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(OTA_POLL_CHUNK_SEC * 1000));

        if (settings_get_auto_ota_paused()) {
            elapsed = 0;
            s_check_now = false;        /* check-now is ignored while paused */
            continue;
        }

        bool due;
        if (s_check_now) { s_check_now = false; due = true; }
        else { elapsed += OTA_POLL_CHUNK_SEC; due = elapsed >= settings_get_ota_poll_sec(); }
        if (!due) continue;
        elapsed = 0;

        char remote[32];
        if (fetch_latest_version(remote, sizeof(remote)) == ESP_OK) {
            if (ota_semver_cmp(remote, FW_VERSION) > 0) {
                ESP_LOGI(TAG, "Update available: running %s, latest %s", FW_VERSION, remote);
                do_ota();
            } else {
                ESP_LOGI(TAG, "Up to date (running %s, latest %s)", FW_VERSION, remote);
            }
        } else {
            ESP_LOGW(TAG, "Version check failed; will retry");
        }
    }
```
(The `vTaskDelay(STABLE_BOOT_DELAY_MS ...)` + `ota_update_mark_valid()` before the loop are unchanged.)

- [ ] **Step 3: Build** — `idf.py build` clean under `-Werror`.

- [ ] **Step 4: Commit**
```bash
git add main/ota_update.h main/ota_update.c
git commit -m "feat(ota): chunked poll honoring ota_poll_sec + check-now trigger"
```

---

### Task 3: `door_control` — runtime door-move timeout + relay pulse

**Files:** Modify `main/door_control.c` (and `main/door_control.h` to drop the now-unused macro)

**Interfaces — Consumes (Task 1):** `settings_get_door_move_sec()`, `settings_get_relay_pulse_ms()`.

- [ ] **Step 1: Add a µs helper in `main/door_control.c`**

`#include "settings.h"` is already present. Add near the top (after the statics):
```c
/* Door-move / auto-close-verify window in microseconds, from settings. */
static int64_t door_move_us(void) {
  return (int64_t)settings_get_door_move_sec() * 1000000LL;
}
```

- [ ] **Step 2: Use the settings values**

In `kick_door_relay()`, replace:
```c
  vTaskDelay(300 / portTICK_PERIOD_MS);
```
with:
```c
  vTaskDelay(pdMS_TO_TICKS(settings_get_relay_pulse_ms()));
```
Replace **both** occurrences of `esp_timer_start_once(oneshot_timer, DOOR_MOVING_MAXTIME)` (in `door_handle_homekit_request` and the auto-close block) with:
```c
  esp_timer_start_once(oneshot_timer, door_move_us());
```
Replace `auto_close_verify_deadline_us = now + DOOR_MOVING_MAXTIME;` with:
```c
  auto_close_verify_deadline_us = now + door_move_us();
```

- [ ] **Step 3: Remove the now-unused macro from `main/door_control.h`**

Delete the line `#define DOOR_MOVING_MAXTIME 8000000` (and its comment). Grep to confirm no remaining users:
```bash
grep -rn DOOR_MOVING_MAXTIME main/
```
Expected: no matches.

- [ ] **Step 4: Build** — `idf.py build` clean under `-Werror`.

- [ ] **Step 5: Commit**
```bash
git add main/door_control.c main/door_control.h
git commit -m "feat(door): read door-move timeout and relay pulse from settings"
```

---

### Task 4: `web_server` — endpoints + status fields

**Files:** Modify `main/web_server.c`

**Interfaces — Consumes:** Task 1 (`settings_set/get_ota_poll_sec/door_move_sec/relay_pulse_ms`), Task 2 (`ota_update_check_now`).

- [ ] **Step 1: Include ota_update.h**

Add to the includes (after `#include "ota_upload.h"`):
```c
#include "ota_update.h"
```

- [ ] **Step 2: Add the 3 fields to `/api/status`**

In `status_get()`, enlarge the buffer: change `static char buf[800];` to `static char buf[960];`.
In the `snprintf` format string, change the tail
```c
    "\"auto_ota_paused\":%s}",
```
to:
```c
    "\"auto_ota_paused\":%s,"
    "\"ota_poll_sec\":%u,\"door_move_sec\":%u,\"relay_pulse_ms\":%u}",
```
and change the final argument line
```c
    settings_get_auto_ota_paused() ? "true" : "false");
```
to:
```c
    settings_get_auto_ota_paused() ? "true" : "false",
    (unsigned)settings_get_ota_poll_sec(), (unsigned)settings_get_door_move_sec(),
    (unsigned)settings_get_relay_pulse_ms());
```

- [ ] **Step 3: Extend `settings_post`**

In `settings_post()`, enlarge the body buffer: change `char buf[128];` to `char buf[192];`.
After the `settings_set_timeouts(ac, lo);` line, add:
```c
  if (httpd_query_key_value(buf, "ota_poll_min", v, sizeof(v)) == ESP_OK)
    settings_set_ota_poll_sec((uint32_t)atoi(v) * 60);
  if (httpd_query_key_value(buf, "door_move_sec", v, sizeof(v)) == ESP_OK)
    settings_set_door_move_sec((uint32_t)atoi(v));
  if (httpd_query_key_value(buf, "relay_pulse_ms", v, sizeof(v)) == ESP_OK)
    settings_set_relay_pulse_ms((uint32_t)atoi(v));
```

- [ ] **Step 4: Add the ota-check + reboot handlers**

After `auto_ota_post()`, add:
```c
//
// POST /api/ota-check — trigger an immediate version check (no effect while paused).
//
static esp_err_t ota_check_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  if (settings_get_auto_ota_paused()) {
    httpd_resp_sendstr(req, "auto-update is paused — resume first");
    return ESP_OK;
  }
  ota_update_check_now();
  httpd_resp_sendstr(req, "checking for updates...");
  return ESP_OK;
}

//
// POST /api/reboot — restart the device (response is sent before the restart).
//
static esp_err_t reboot_post(httpd_req_t *req) {
  if (require_auth(req) != ESP_OK) return ESP_OK;
  httpd_resp_sendstr(req, "rebooting...");
  ESP_LOGW(TAG, "Reboot requested via web");
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_restart();
  return ESP_OK;   /* not reached */
}
```

- [ ] **Step 5: Register the two endpoints**

In `web_server_start()`, raise `config.max_uri_handlers = 10;` to `config.max_uri_handlers = 12;` and add to the `uris[]` array (after the `/api/auto-ota` row):
```c
    { .uri = "/api/ota-check",  .method = HTTP_POST, .handler = ota_check_post },
    { .uri = "/api/reboot",     .method = HTTP_POST, .handler = reboot_post },
```

- [ ] **Step 6: Build** — `idf.py build` clean; image still fits the OTA slot (`idf.py size`).

- [ ] **Step 7: Commit**
```bash
git add main/web_server.c
git commit -m "feat(web): status fields + /api/settings tunables + /api/ota-check + /api/reboot"
```

---

### Task 5: Web UI — maintainable source + new controls  *(controller-executed inline; needs visual verification)*

**Files:** Create `tools/gen_web_page.py`, `tools/web/page.html`, `tools/web/setup.html`; Modify `main/web_server.c` (regenerated page strings).

**Note:** This task is best executed by the controller inline (not a blind subagent) because it requires rendering the page to verify it looks right. It consumes the `/api/status` fields + endpoints from Task 4.

- [ ] **Step 1: Add the page generator `tools/gen_web_page.py`**

```python
#!/usr/bin/env python3
"""Regenerate the embedded web pages in main/web_server.c from tools/web/*.html.
Workflow: edit tools/web/page.html or setup.html, then run `python3 tools/gen_web_page.py`."""
import re, os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def clit(name, s):
    s = s.replace("\\", "\\\\").replace('"', '\\"')
    body = "\n".join('"%s\\n"' % ln for ln in s.split("\n"))
    return "static const char %s[] =\n%s;\n" % (name, body)

setup = open(os.path.join(ROOT, "tools/web/setup.html"), encoding="utf-8").read().rstrip("\n")
page  = open(os.path.join(ROOT, "tools/web/page.html"),  encoding="utf-8").read().rstrip("\n")
ws_path = os.path.join(ROOT, "main/web_server.c")
ws = open(ws_path, encoding="utf-8").read()
region = re.compile(r"(// ---- static pages[^\n]*\n\n)(.*?)(\n// ---- helpers)", re.S)
m = region.search(ws)
assert m, "static-pages region not found in web_server.c"
mid = ('\n// First-run "create password" page (served while no password is set).\n' + clit("SETUP_HTML", setup)
       + '\n// Main status/config page. Values are filled by JS from /api/status.\n' + clit("PAGE_HTML", page))
ws = ws[:m.start()] + m.group(1) + mid.rstrip("\n") + "\n" + m.group(3) + ws[m.end():]
open(ws_path, "w", encoding="utf-8").write(ws)
print("regenerated PAGE_HTML(%d) SETUP_HTML(%d)" % (len(page), len(setup)))
```

- [ ] **Step 2: Seed the canonical source from the current strings**

Run once to extract the current pages into `tools/web/`:
```bash
cd /Users/laek/source/ESP32-homekit-garage-opener
python3 - <<'PY'
import re, os
ws = open("main/web_server.c", encoding="utf-8").read()
def extract(name):
    seg = ws[ws.index("static const char %s[] =" % name):]
    seg = seg[:seg.index(";\n")]
    lits = re.findall(r'"((?:[^"\\]|\\.)*)"', seg)
    return "".join(lits).replace("\\n", "\n").replace('\\"', '"').rstrip("\n")
os.makedirs("tools/web", exist_ok=True)
open("tools/web/page.html", "w", encoding="utf-8").write(extract("PAGE_HTML"))
open("tools/web/setup.html", "w", encoding="utf-8").write(extract("SETUP_HTML"))
print("seeded tools/web/page.html + setup.html")
PY
python3 tools/gen_web_page.py
git diff --stat main/web_server.c   # expect no or only trivial whitespace change
idf.py build                        # still builds clean
```

- [ ] **Step 3: Edit `tools/web/page.html` — add the new controls**

Make these edits to `tools/web/page.html` (plain HTML — no C escaping):

1. **Remove** the Auto-update row from the Controls card (the `<div class="sw">…id="autoota"…</div>` line), leaving Lockout there.
2. **Add** an "Updates" card and a "Door tuning" card. Insert after the Controls card's closing `</div>`:
```html
    <div class="card">
      <h3>Updates</h3>
      <div class="sw"><div class="t">Auto-update</div><span class="st" id="autoota_l">on</span><input type="checkbox" class="tog" id="autoota" onchange="post('/api/auto-ota','on='+(this.checked?1:0))"></div>
      <div class="field"><label>Poll every</label><span class="num-in"><input type="number" id="ota_min" min="5" max="1440"> min</span></div>
      <button class="save" style="margin-top:10px;background:#20232c;color:var(--ink)" onclick="post('/api/ota-check','')">Check for updates now</button>
    </div>

    <div class="card">
      <h3>Door tuning</h3>
      <div class="field"><label>Move timeout</label><span class="num-in"><input type="number" id="move_sec" min="3" max="30"> s</span></div>
      <div class="field"><label>Relay pulse</label><span class="num-in"><input type="number" id="pulse_ms" min="100" max="2000"> ms</span></div>
    </div>
```
3. **Add** a Reboot button. Insert just before the closing `</div>` of `.wrap` (after the `.grid` closes):
```html
  <div style="margin-top:16px;text-align:center">
    <button style="background:#3a1d1d;color:#ffb4b4;border:1px solid rgba(255,106,106,.4)" onclick="if(confirm('Reboot the device now? HomeKit and the web page drop for ~15 s.'))post('/api/reboot','')">Reboot device</button>
  </div>
```
4. **Extend the Save action** — the Timers card's Save button posts the settings. Change its `onclick` body to also send the new fields:
```html
onclick="post('/api/settings','auto_close_sec='+ac_sec.value+'&left_open_sec='+lo_sec.value+'&ota_poll_min='+ota_min.value+'&door_move_sec='+move_sec.value+'&relay_pulse_ms='+pulse_ms.value)"
```
5. **Populate the new fields in `paint(d)`** — add these lines inside `paint`, guarded like the existing timer inputs (so typing isn't clobbered):
```js
if(document.activeElement.id!='ota_min')document.getElementById('ota_min').value=Math.round(d.ota_poll_sec/60);
if(document.activeElement.id!='move_sec')document.getElementById('move_sec').value=d.door_move_sec;
if(document.activeElement.id!='pulse_ms')document.getElementById('pulse_ms').value=d.relay_pulse_ms;
```

- [ ] **Step 4: Regenerate + build**
```bash
python3 tools/gen_web_page.py
idf.py build
```
Expected: clean under `-Werror`; image fits the OTA slot.

- [ ] **Step 5: Render-verify the served page (visual)**

Extract the regenerated `PAGE_HTML` back out of `web_server.c`, inject a mock `/api/status`, serve it locally, and render it (Playwright/browser) to confirm: the new Updates / Door-tuning cards, the poll/move/pulse fields (populated from mock: 60 min, 8 s, 300 ms), the Check-now and Reboot buttons all appear and the page has no JS errors. Confirm mobile (single-column) still looks right.

- [ ] **Step 6: Commit**
```bash
git add tools/gen_web_page.py tools/web/page.html tools/web/setup.html main/web_server.c
git commit -m "feat(web): editable OTA poll / door-move / relay settings + check-now + reboot UI"
```

---

### Task 6: Documentation

**Files:** Modify `README.md`

- [ ] **Step 1: Document in `README.md`**

Add to the web-page section:
- The page now edits **OTA poll interval** (minutes, 5–1440), **door-move timeout** (3–30 s), and **relay pulse** (100–2000 ms), alongside the auto-close/left-open timers. All persist in NVS.
- **Check for updates now** button triggers an immediate GitHub check (no effect while auto-updates are paused).
- **Reboot device** button restarts the device (HomeKit + web drop for ~15 s).
- A poll-interval change takes effect within ~10 s (no reboot).
- **Editing the page:** the web UI source lives in `tools/web/page.html` / `setup.html`; after editing, run `python3 tools/gen_web_page.py` to regenerate the embedded strings in `main/web_server.c`, then rebuild.

- [ ] **Step 2: Commit**
```bash
git add README.md
git commit -m "docs: document runtime settings, check-now, reboot, and page regeneration"
```

---

## Self-Review

**Spec coverage:**
- 3 NVS settings + Kconfig defaults + clamped ranges → Task 1. ✓
- Chunked poll (near-immediate, no reboot) + check-now trigger → Task 2. ✓
- door-move timeout (failsafe + verify window) + relay pulse from settings → Task 3. ✓
- Extended `/api/settings`, `/api/ota-check`, `/api/reboot`, 3 status fields → Task 4. ✓
- UI: Updates card (auto-update toggle moved + poll interval + check-now), Door-tuning card, Reboot (confirm-guarded), populate/save new fields → Task 5. ✓
- Auto-close backstops NOT exposed → confirmed (not touched). ✓
- Maintainable page source + regenerator → Task 5. ✓
- Docs → Task 6. ✓

**Placeholder scan:** No TBD/TODO; every code step has complete code. Task 5 fragments are complete HTML/JS; Task 6 lists exact points.

**Type consistency:** `settings_get/set_ota_poll_sec` / `_door_move_sec` / `_relay_pulse_ms` (uint32) consistent across settings.{c,h}, web_server.c, and status JSON. `ota_update_check_now(void)` matches between ota_update.h and web_server.c. JSON keys `ota_poll_sec`/`door_move_sec`/`relay_pulse_ms` match between `status_get` and the page JS. POST key `ota_poll_min` (minutes) → converted to seconds in `settings_post`; the page's `ota_min` field ↔ `ota_poll_sec/60`. Element IDs `ota_min`/`move_sec`/`pulse_ms` match between the HTML and `paint`.

**Watch during Task 4:** the status buffer bump to 960 must cover the enlarged payload (current ~620 B + ~55 B new fields) — comfortably fits.

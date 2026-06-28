# CI/CD + Self-Updating OTA Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add GitHub Actions CI (build on push/PR), a manually-dispatched release workflow that publishes an OTA image, and a self-polling GitHub-release OTA updater on the device that replaces the HomeKit firmware-upgrade trigger.

**Architecture:** Firmware version is injected at build time (`FW_VERSION` → `fw_rev` and the OTA baseline). A new `ota_update` module runs a low-priority task that hourly fetches `version.txt` from the repo's latest GitHub Release, semver-compares it to the running version, and `esp_https_ota()`s from the release's `garage.bin` (trusting GitHub via the IDF cert bundle), with bootloader rollback protecting against bad images. CI builds on push/PR; a `workflow_dispatch` release workflow tags + publishes `garage.bin` + `version.txt`.

**Tech Stack:** C, ESP-IDF v5.5.2, FreeRTOS, `esp_https_ota`/`esp_http_client`/`app_update`/`esp_crt_bundle`, Espressif HomeKit SDK, GitHub Actions (`espressif/esp-idf-ci-action`), CMake + ESP Component Manager.

## Testing Note (read before starting)

This firmware has **no automated test harness** (CLAUDE.md: "no tests, linters, or CI" — this plan adds CI, not unit tests). Per the approved spec, verification is build-clean + on-device. So:
- **Per task:** a clean `idf.py build` (zero errors) is the gate. `main/CMakeLists.txt` compiles our own component with `-Werror` (added in Task 1), so any warning in our code fails the build — this is also what makes CI a real gate.
- **One exception:** the semver comparison (Task 3) is pure logic; that task includes a throwaway host-`cc` check of just that function (run, confirm, discard — not committed, no harness added).
- **GitHub Actions** (Tasks 5–6) cannot be run locally; verify by YAML-validating and dry-running the underlying build commands. The first push/dispatch is the real proof (the user will do that).

**Build command throughout:**
```bash
. ~/esp/esp-idf/export.sh && cd ~/source/ESP32-homekit-garage-opener && idf.py build
```
Note `sdkconfig` is gitignored (regenerated from `sdkconfig.defaults`); after editing `sdkconfig.defaults` run `idf.py reconfigure` (or just `idf.py build`).

## Global Constraints

- **ESP-IDF v5.5.2**, target **esp32** only.
- **Repo slug:** `larsakeekstrand/ESP32-homekit-garage-opener`; OTA base URL `https://github.com/larsakeekstrand/ESP32-homekit-garage-opener`.
- **esp-homekit-sdk** pinned commit `676fabac4a4a05184be020611cb069faa0016411` (CI clones it; locally it's the `~/esp/esp-homekit-sdk` peer).
- **OTA partition:** 1600K (1638400 bytes) per slot; the app must fit. App was ~98% full before this work — `-Os` (Task 2) creates headroom.
- **Release asset names:** `garage.bin` (the app image, ESP-IDF names it after `project(garage)`) and `version.txt`.
- **Default poll interval:** 3600 s (hourly), menuconfig-configurable.
- **Default `FW_VERSION`:** `0.0.0-dev` (local/CI builds); releases pass the real version.
- **Logging tag:** keep the `"HAP Garage"` style; OTA module may use its own tag `"OTA"`.
- **No new features beyond the spec.** Remove `hap_fw_upgrade`.
- **Commit after every task.** Branch `cicd-self-updating-ota` (already created). Stage files explicitly (the tree is otherwise clean).

---

### Task 1: Build-time `FW_VERSION` injection + `-Werror` for our component

Wire a compile-time version string into the firmware and lock in warning-free builds.

**Files:**
- Modify: `main/CMakeLists.txt`
- Modify: `main/hap_garage.c` (the `.fw_rev` line)

**Interfaces:**
- Consumes: nothing.
- Produces: a preprocessor macro `FW_VERSION` (a string literal, e.g. `"0.0.0-dev"`) available to all `main/` sources, overridable via `idf.py -DFW_VERSION=<v> build`.

- [ ] **Step 1: Update `main/CMakeLists.txt`**

Replace its contents with:
```cmake
idf_component_register(SRCS "app_main.c" "hap_garage.c" "dht_sensor.c" "door_control.c"
                       INCLUDE_DIRS ".")

# Firmware version: overridable with `idf.py -DFW_VERSION=1.2.3 build`.
if(NOT DEFINED FW_VERSION OR FW_VERSION STREQUAL "")
  set(FW_VERSION "0.0.0-dev")
endif()
message(STATUS "Building firmware version: ${FW_VERSION}")
target_compile_definitions(${COMPONENT_LIB} PRIVATE FW_VERSION="${FW_VERSION}")

# Treat warnings in our own code as errors (does not affect the SDK/third-party components).
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)
```
(Note: `ota_update.c` is added to SRCS in Task 3.)

- [ ] **Step 2: Use `FW_VERSION` for `fw_rev` in `main/hap_garage.c`**

Find `.fw_rev = "1.0.0",` (around line 272) and change it to:
```c
        .fw_rev = FW_VERSION,
```

- [ ] **Step 3: Build clean (default version)**

```bash
. ~/esp/esp-idf/export.sh && cd ~/source/ESP32-homekit-garage-opener && idf.py build 2>&1 | tail -15
```
Expected: `Project build complete.`, zero errors, and a `Building firmware version: 0.0.0-dev` line earlier in the configure output.

- [ ] **Step 4: Verify the override path works**

```bash
idf.py -DFW_VERSION=9.9.9 reconfigure 2>&1 | grep "Building firmware version"
```
Expected: `Building firmware version: 9.9.9`. Then reset back: `idf.py -DFW_VERSION=0.0.0-dev reconfigure >/dev/null`.

- [ ] **Step 5: Commit**

```bash
git add main/CMakeLists.txt main/hap_garage.c
git commit -m "feat: inject FW_VERSION at build time; -Werror for app sources

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: Config — cert bundle, OTA rollback, size optimization, Kconfig options

Enable the platform features the OTA module needs and add its tunables.

**Files:**
- Modify: `sdkconfig.defaults`
- Modify: `main/Kconfig.projbuild`

**Interfaces:**
- Consumes: nothing.
- Produces: Kconfig symbols `CONFIG_OTA_POLL_INTERVAL_SEC` (int) and `CONFIG_OTA_GITHUB_BASE_URL` (string), plus the cert-bundle / rollback / `-Os` build settings.

- [ ] **Step 1: Append OTA/build settings to `sdkconfig.defaults`**

Add these lines to `sdkconfig.defaults`:
```
# HTTPS OTA: trust standard CAs (incl. GitHub) via the bundled cert store
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEFAULT_CMN=y
# OTA rollback: auto-revert if a new image never confirms a healthy boot
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
# Optimize for size to keep the image within the 1600K OTA slot
CONFIG_COMPILER_OPTIMIZATION_SIZE=y
```

- [ ] **Step 2: Add OTA options to `main/Kconfig.projbuild`**

Inside the existing `menu "Example Configuration"` … `endmenu` block (before `endmenu`), add:
```
    config OTA_POLL_INTERVAL_SEC
        int "OTA poll interval (seconds)"
        default 3600
        help
            How often the device checks GitHub Releases for a newer firmware.

    config OTA_GITHUB_BASE_URL
        string "GitHub repo base URL for OTA"
        default "https://github.com/larsakeekstrand/ESP32-homekit-garage-opener"
        help
            Base URL of the GitHub repo. The updater fetches
            <base>/releases/latest/download/version.txt and .../garage.bin
```

- [ ] **Step 3: Reconfigure + build clean**

```bash
. ~/esp/esp-idf/export.sh && cd ~/source/ESP32-homekit-garage-opener
rm -rf build && idf.py reconfigure 2>&1 | tail -5 && idf.py build 2>&1 | tail -20
```
Expected: `Project build complete.`, zero errors. Confirm the new Kconfig symbols resolve (no "unknown symbol" warnings). Note the app binary size line; with `-Os` it should be comfortably below 1600K (1638400 bytes).

- [ ] **Step 4: Sanity-check the size headroom**

```bash
SIZE=$(stat -f%z build/garage.bin 2>/dev/null || stat -c%s build/garage.bin); echo "garage.bin = $SIZE bytes (limit 1638400)"; [ "$SIZE" -le 1638400 ] && echo "FITS" || echo "OVERFLOW"
```
Expected: `FITS`. If `OVERFLOW`, STOP and report — the cert bundle (~tens of KB) outweighed the `-Os` savings and we must revisit (e.g. partition resize), which is out of this task's scope.

- [ ] **Step 5: Commit**

```bash
git add sdkconfig.defaults main/Kconfig.projbuild
git commit -m "build: enable cert bundle, OTA rollback, size opt; add OTA Kconfig

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: The `ota_update` module (poller + version compare + HTTPS OTA)

Create the self-updating OTA module. Build it but don't wire it in yet (Task 4 does that), so this task's deliverable is a clean-compiling module with a host-verified version comparator.

**Files:**
- Create: `main/ota_update.h`, `main/ota_update.c`
- Modify: `main/CMakeLists.txt` (add `ota_update.c` to SRCS + `REQUIRES`)

**Interfaces:**
- Consumes: `CONFIG_OTA_POLL_INTERVAL_SEC`, `CONFIG_OTA_GITHUB_BASE_URL`, `FW_VERSION` (from Tasks 1–2).
- Produces (in `ota_update.h`):
  - `void ota_update_start(void);` — spawn the updater task.
  - `void ota_update_mark_valid(void);` — confirm the running image (cancel pending rollback).

- [ ] **Step 1: Create `main/ota_update.h`**

```c
//
// ota_update.h — self-polling GitHub-release OTA updater.
//
#pragma once

/* Spawn the background task that periodically checks GitHub Releases and
 * applies a newer firmware via HTTPS OTA. */
void ota_update_start(void);

/* Confirm the currently-running image after a healthy boot, cancelling any
 * pending rollback. Safe to call when rollback is not pending (no-op). */
void ota_update_mark_valid(void);
```

- [ ] **Step 2: Create `main/ota_update.c`**

```c
//
// ota_update.c — periodically poll GitHub Releases and OTA-update if newer.
//
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_http_client.h>
#include <esp_https_ota.h>
#include <esp_crt_bundle.h>
#include <esp_ota_ops.h>
#include <esp_system.h>

#include "ota_update.h"

static const char *TAG = "OTA";

#define OTA_BASE_URL      CONFIG_OTA_GITHUB_BASE_URL
#define VERSION_URL       OTA_BASE_URL "/releases/latest/download/version.txt"
#define FIRMWARE_URL      OTA_BASE_URL "/releases/latest/download/garage.bin"
#define POLL_INTERVAL_MS  (CONFIG_OTA_POLL_INTERVAL_SEC * 1000)
#define FIRST_CHECK_DELAY_MS 60000   /* let Wi-Fi/HAP settle and mark-valid run first */

/* Parse up to major.minor.patch from a version string. A non-numeric/missing
 * component (e.g. the "-dev" suffix) parses as 0. */
static void parse_ver(const char *s, int out[3]) {
    out[0] = out[1] = out[2] = 0;
    sscanf(s, "%d.%d.%d", &out[0], &out[1], &out[2]);
}

/* Return >0 if a is newer than b, <0 if older, 0 if equal. */
int ota_semver_cmp(const char *a, const char *b) {
    int va[3], vb[3];
    parse_ver(a, va);
    parse_ver(b, vb);
    for (int i = 0; i < 3; i++) {
        if (va[i] != vb[i]) return (va[i] > vb[i]) ? 1 : -1;
    }
    return 0;
}

/* Fetch the latest release's version.txt into buf (NUL-terminated, trimmed).
 * Returns ESP_OK on success. */
static esp_err_t fetch_latest_version(char *buf, size_t buf_len) {
    esp_http_client_config_t config = {
        .url = VERSION_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return ESP_FAIL;

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open %s: %s", VERSION_URL, esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }
    esp_http_client_fetch_headers(client);
    int read = esp_http_client_read_response(client, buf, buf_len - 1);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    if (read <= 0) return ESP_FAIL;

    buf[read] = '\0';
    /* trim trailing whitespace/newline */
    while (read > 0 && (buf[read - 1] == '\n' || buf[read - 1] == '\r' || buf[read - 1] == ' ')) {
        buf[--read] = '\0';
    }
    return ESP_OK;
}

/* Download and apply the latest firmware; reboots on success. */
static void do_ota(void) {
    ESP_LOGI(TAG, "Starting OTA from %s", FIRMWARE_URL);
    esp_http_client_config_t http_config = {
        .url = FIRMWARE_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };
    esp_err_t err = esp_https_ota(&ota_config);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA successful; rebooting into new firmware");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
    }
}

static void ota_task(void *arg) {
    vTaskDelay(FIRST_CHECK_DELAY_MS / portTICK_PERIOD_MS);
    while (1) {
        char remote[32];
        if (fetch_latest_version(remote, sizeof(remote)) == ESP_OK) {
            if (ota_semver_cmp(remote, FW_VERSION) > 0) {
                ESP_LOGI(TAG, "Update available: running %s, latest %s", FW_VERSION, remote);
                do_ota();   /* reboots on success; falls through to retry on failure */
            } else {
                ESP_LOGI(TAG, "Up to date (running %s, latest %s)", FW_VERSION, remote);
            }
        } else {
            ESP_LOGW(TAG, "Version check failed; will retry");
        }
        vTaskDelay(POLL_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

void ota_update_start(void) {
    xTaskCreate(ota_task, "ota_update", 8 * 1024, NULL, 2, NULL);
}

void ota_update_mark_valid(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG, "Marking running image valid (cancelling rollback)");
        esp_ota_mark_app_valid_cancel_rollback();
    }
}
```

- [ ] **Step 3: Register the source + dependencies in `main/CMakeLists.txt`**

Update the `idf_component_register` call to add `ota_update.c` and the OTA-related `REQUIRES`:
```cmake
idf_component_register(SRCS "app_main.c" "hap_garage.c" "dht_sensor.c" "door_control.c" "ota_update.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_https_ota esp_http_client app_update esp-tls mbedtls)
```
(Leave the `FW_VERSION` and `-Werror` lines from Task 1 unchanged.)

- [ ] **Step 4: Host-verify the version comparator (throwaway, not committed)**

```bash
cat > /tmp/semver_check.c <<'EOF'
#include <stdio.h>
#include <assert.h>
static void parse_ver(const char *s, int out[3]){out[0]=out[1]=out[2]=0;sscanf(s,"%d.%d.%d",&out[0],&out[1],&out[2]);}
int ota_semver_cmp(const char *a,const char *b){int va[3],vb[3];parse_ver(a,va);parse_ver(b,vb);for(int i=0;i<3;i++){if(va[i]!=vb[i])return va[i]>vb[i]?1:-1;}return 0;}
int main(void){
  assert(ota_semver_cmp("1.1.0","1.0.0")>0);
  assert(ota_semver_cmp("1.10.0","1.9.0")>0);   /* numeric, not lexical */
  assert(ota_semver_cmp("1.0.0","1.0.0")==0);
  assert(ota_semver_cmp("0.0.0-dev","1.0.0")<0);/* dev is oldest */
  assert(ota_semver_cmp("2.0.0","1.9.9")>0);
  assert(ota_semver_cmp("1.0.1","1.0.0")>0);
  printf("semver OK\n"); return 0;
}
EOF
cc /tmp/semver_check.c -o /tmp/semver_check && /tmp/semver_check && rm -f /tmp/semver_check /tmp/semver_check.c
```
Expected: `semver OK`. (This mirrors the exact `parse_ver`/`ota_semver_cmp` logic from `ota_update.c`. It is not added to the repo.)

- [ ] **Step 5: Build clean**

```bash
. ~/esp/esp-idf/export.sh && cd ~/source/ESP32-homekit-garage-opener && idf.py build 2>&1 | tail -20
```
Expected: `Project build complete.`, zero errors. (`ota_update.c` compiles with `-Werror`; `esp_crt_bundle.h`/`esp_https_ota.h` resolve via the `REQUIRES`.) Confirm `garage.bin` still fits (re-run the Task 2 Step 4 size check).

- [ ] **Step 6: Commit**

```bash
git add main/ota_update.c main/ota_update.h main/CMakeLists.txt
git commit -m "feat: add self-polling GitHub-release OTA updater module

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: Wire OTA into `app_main`; remove `hap_fw_upgrade`

Start the updater, confirm healthy boots, and drop the obsolete HomeKit upgrade service.

**Files:**
- Modify: `main/app_main.c`
- Modify: `main/hap_garage.c`

**Interfaces:**
- Consumes: `ota_update_start()`, `ota_update_mark_valid()` (Task 3).
- Produces: nothing new.

- [ ] **Step 1: Wire the updater into `main/app_main.c`**

Add the include and update the task body + `app_main`:
```c
#include "ota_update.h"
```
In `garage_thread_entry`, call `ota_update_mark_valid()` once the device is healthy (after Wi-Fi/HAP are up, before the event loop):
```c
static void garage_thread_entry(void *p) {
  hap_garage_init();
  door_control_init();
  hap_garage_start();
  hap_garage_start_wifi();   // blocking; returns once Wi-Fi/HAP are up
  ota_update_mark_valid();   // healthy boot confirmed → cancel any pending rollback
  door_control_run_loop();   // never returns
}
```
In `app_main`, spawn the updater task alongside the others:
```c
void app_main(void) {
  xTaskCreate(garage_thread_entry, "hap_garage", GARAGE_TASK_STACKSIZE, NULL, 1, NULL);
  dht_sensor_start();
  ota_update_start();
}
```

- [ ] **Step 2: Remove `hap_fw_upgrade` from `main/hap_garage.c`**

Delete all of these:
- the include: `#include <hap_fw_upgrade.h>`
- the global: `char server_cert[] = {};` (and its comment)
- in `hap_garage_init`, the FW-upgrade service block — the `hap_fw_upgrade_config_t ota_config = { .server_cert_pem = server_cert };`, the `service = hap_serv_fw_upgrade_create(&ota_config);`, and the matching `hap_acc_add_serv(accessory, service);` for that service. (Keep the Garage Door Opener, Temperature, and Humidity service additions intact.)

- [ ] **Step 3: Build clean**

```bash
. ~/esp/esp-idf/export.sh && cd ~/source/ESP32-homekit-garage-opener && idf.py build 2>&1 | tail -20
```
Expected: `Project build complete.`, zero errors, no unused-variable/`-Werror` failures from the removed code. Confirm `garage.bin` still fits (Task 2 Step 4 check).

- [ ] **Step 4: Commit**

```bash
git add main/app_main.c main/hap_garage.c
git commit -m "feat: start OTA updater + mark-valid; remove hap_fw_upgrade service

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: CI build workflow

Build the firmware on every push to `main` and every PR to `main`.

**Files:**
- Create: `.github/workflows/build.yml`

**Interfaces:** none (CI config).

- [ ] **Step 1: Create `.github/workflows/build.yml`**

```yaml
name: Build

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Clone pinned esp-homekit-sdk
        run: |
          git clone https://github.com/espressif/esp-homekit-sdk.git "$GITHUB_WORKSPACE/esp-homekit-sdk"
          git -C "$GITHUB_WORKSPACE/esp-homekit-sdk" checkout 676fabac4a4a05184be020611cb069faa0016411
          git -C "$GITHUB_WORKSPACE/esp-homekit-sdk" submodule update --init --recursive

      - name: Build firmware
        uses: espressif/esp-idf-ci-action@v1
        with:
          esp_idf_version: v5.5.2
          target: esp32
          command: |
            export HOMEKIT_PATH="$GITHUB_WORKSPACE/esp-homekit-sdk"
            idf.py set-target esp32 build

      - name: Check image fits the OTA partition
        run: |
          SIZE=$(stat -c%s build/garage.bin)
          LIMIT=$((1600 * 1024))
          echo "garage.bin = $SIZE bytes (limit $LIMIT)"
          test "$SIZE" -le "$LIMIT"

      - name: Upload firmware artifact
        uses: actions/upload-artifact@v4
        with:
          name: garage-bin
          path: build/garage.bin
```

- [ ] **Step 2: Validate the YAML**

```bash
cd ~/source/ESP32-homekit-garage-opener
python3 -c "import yaml,sys; yaml.safe_load(open('.github/workflows/build.yml')); print('YAML OK')"
command -v actionlint >/dev/null && actionlint .github/workflows/build.yml || echo "(actionlint not installed; skipped)"
```
Expected: `YAML OK` (and actionlint clean if present). The build commands mirror the locally-proven `idf.py set-target esp32 build`; warnings-as-errors comes from the `-Werror` added in Task 1. First-push run on GitHub is the real proof.

- [ ] **Step 3: Commit**

```bash
git add .github/workflows/build.yml
git commit -m "ci: build firmware on push and PR to main

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: Release workflow

Manually-dispatched workflow that builds a versioned image and publishes a GitHub Release.

**Files:**
- Create: `.github/workflows/release.yml`

**Interfaces:** none (CI config).

- [ ] **Step 1: Create `.github/workflows/release.yml`**

```yaml
name: Release

on:
  workflow_dispatch:
    inputs:
      version:
        description: "Release version (e.g. 1.1.0, no leading v)"
        required: true
        type: string

permissions:
  contents: write

jobs:
  release:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Clone pinned esp-homekit-sdk
        run: |
          git clone https://github.com/espressif/esp-homekit-sdk.git "$GITHUB_WORKSPACE/esp-homekit-sdk"
          git -C "$GITHUB_WORKSPACE/esp-homekit-sdk" checkout 676fabac4a4a05184be020611cb069faa0016411
          git -C "$GITHUB_WORKSPACE/esp-homekit-sdk" submodule update --init --recursive

      - name: Build firmware (versioned)
        uses: espressif/esp-idf-ci-action@v1
        with:
          esp_idf_version: v5.5.2
          target: esp32
          command: |
            export HOMEKIT_PATH="$GITHUB_WORKSPACE/esp-homekit-sdk"
            idf.py -DFW_VERSION="${{ inputs.version }}" set-target esp32 build

      - name: Check image fits the OTA partition
        run: |
          SIZE=$(stat -c%s build/garage.bin)
          LIMIT=$((1600 * 1024))
          echo "garage.bin = $SIZE bytes (limit $LIMIT)"
          test "$SIZE" -le "$LIMIT"

      - name: Write version.txt
        run: printf '%s' "${{ inputs.version }}" > version.txt

      - name: Create GitHub Release
        uses: softprops/action-gh-release@v2
        with:
          tag_name: v${{ inputs.version }}
          name: v${{ inputs.version }}
          generate_release_notes: true
          files: |
            build/garage.bin
            version.txt
```

- [ ] **Step 2: Validate the YAML + the versioned build locally**

```bash
cd ~/source/ESP32-homekit-garage-opener
python3 -c "import yaml; yaml.safe_load(open('.github/workflows/release.yml')); print('YAML OK')"
command -v actionlint >/dev/null && actionlint .github/workflows/release.yml || echo "(actionlint not installed; skipped)"
. ~/esp/esp-idf/export.sh && idf.py -DFW_VERSION="1.2.3" reconfigure 2>&1 | grep "Building firmware version"
idf.py -DFW_VERSION=0.0.0-dev reconfigure >/dev/null   # reset
```
Expected: `YAML OK` and `Building firmware version: 1.2.3` (confirms the release build's version injection works). GitHub-side run is the real proof (the user dispatches it once).

- [ ] **Step 3: Commit**

```bash
git add .github/workflows/release.yml
git commit -m "ci: add manual release workflow publishing garage.bin + version.txt

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: Documentation

Document the CI, the release process, and the self-updating OTA behavior.

**Files:**
- Modify: `README.md`
- Modify: `CLAUDE.md`

**Interfaces:** none.

- [ ] **Step 1: Update `README.md`**

Add a "Continuous Integration & Releases" section covering:
- CI builds every push/PR to `main` (link/badge optional: `![Build](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/actions/workflows/build.yml/badge.svg)`).
- Cutting a release: in the GitHub **Actions** tab, run the **Release** workflow (`workflow_dispatch`) and enter the version (e.g. `1.1.0`). It tags `v1.1.0` and publishes a Release with `garage.bin` + `version.txt`.
- OTA: the device checks `releases/latest` hourly and auto-updates to a newer version; a bad image is rolled back automatically by the bootloader. Interval/repo configurable via `idf.py menuconfig` (Example Configuration → OTA options).

- [ ] **Step 2: Update `CLAUDE.md`**

- Architecture: add `ota_update.{c,h}` to the module table (self-polling GitHub-release OTA: hourly version check, `esp_https_ota` with cert bundle, rollback) and note `hap_fw_upgrade` was removed (HomeKit no longer carries an upgrade service).
- Note the `app_main` startup now also calls `ota_update_start()` and `ota_update_mark_valid()` (after Wi-Fi/HAP up).
- Configuration: mention `FW_VERSION` build injection (default `0.0.0-dev`; releases set it), the new `sdkconfig.defaults` entries (cert bundle, rollback, `-Os`), and the OTA Kconfig options.
- Commands/CI: mention the two workflows and that the build uses `-Werror` for app sources.

- [ ] **Step 3: Build remains green + commit**

```bash
. ~/esp/esp-idf/export.sh && cd ~/source/ESP32-homekit-garage-opener && idf.py build 2>&1 | tail -5
git add README.md CLAUDE.md
git commit -m "docs: document CI, releases, and self-updating OTA

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

- [ ] **Step 4: On-device end-to-end OTA test (user runs, after merge + first release)**

This is the real proof of the OTA path and can only be done by the user with hardware:
1. Flash a build at a low version: `idf.py -DFW_VERSION=1.0.0 flash monitor`.
2. Dispatch the Release workflow with a higher version (e.g. `1.0.1`); wait for the Release to publish.
3. Watch the monitor: within the poll interval the `OTA` task logs "Update available… running 1.0.0, latest 1.0.1", downloads, and reboots into 1.0.1 (verify via `fw_rev` in the Home app or the boot log).
4. (Optional) Confirm rollback by publishing a deliberately-broken image and verifying the bootloader reverts.

---

## Self-Review

**Spec coverage:**
- CI build on push/PR + warnings-as-errors + partition-fit + artifact → Task 5 (+ `-Werror` in Task 1). ✓
- Release `workflow_dispatch` w/ version → tag + Release w/ `garage.bin` + `version.txt` → Task 6. ✓
- Build-time `FW_VERSION` → `fw_rev` + OTA baseline → Task 1. ✓
- `ota_update` module: hourly poll, `version.txt` fetch, semver compare, `esp_https_ota` + cert bundle, reboot → Task 3. ✓
- Rollback enable + mark-valid after healthy boot → Task 2 (config) + Task 3 (`ota_update_mark_valid`) + Task 4 (call site). ✓
- Remove `hap_fw_upgrade` + `server_cert` → Task 4. ✓
- `sdkconfig.defaults` cert bundle / rollback / `-Os`; Kconfig interval + base URL → Task 2. ✓
- Docs → Task 7. ✓
- Verification (build, YAML, on-device) → per-task + Task 7 Step 4. ✓
- Out of scope (tests, signed OTA, multi-target, CI-push) → not present. ✓

**Placeholder scan:** No TBD/TODO. All code blocks are complete. The host semver check is concrete and runnable.

**Type consistency:** `ota_update_start(void)` / `ota_update_mark_valid(void)` declared in `ota_update.h` (Task 3) and called in `app_main.c` (Task 4) — match. `ota_semver_cmp(const char*, const char*)` internal to Task 3 and mirrored exactly in the host check. `FW_VERSION` produced in Task 1, consumed in Tasks 3–4. Kconfig symbols `CONFIG_OTA_POLL_INTERVAL_SEC`/`CONFIG_OTA_GITHUB_BASE_URL` produced in Task 2, consumed in Task 3. Asset names `garage.bin`/`version.txt` consistent across Tasks 3/5/6. SDK pin `676fabac…` consistent across Tasks 5/6 and Global Constraints.

**Note on CI env (`HOMEKIT_PATH` inside the container):** the `esp-idf-ci-action` runs the `command` in the IDF container with the workspace mounted; `$GITHUB_WORKSPACE/esp-homekit-sdk` is exported inside that command. If the first CI run shows the SDK isn't found at that path, the fix is to adjust the exported path to the in-container workspace mount (reported by the action's log) — flagged here so the implementer checks the first run rather than assuming.

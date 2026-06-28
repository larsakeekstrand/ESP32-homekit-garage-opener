# CI/CD Pipeline + Self-Updating OTA — Design

**Date:** 2026-06-28
**Status:** Approved (pending spec review)
**Scope:** Add GitHub Actions CI (build on push/PR), a manually-triggered release workflow that
publishes an OTA image, and replace the HomeKit-triggered firmware upgrade with a self-polling
GitHub-release OTA updater.

## Goal

1. **CI:** every push to `main` and every PR to `main` builds the firmware on ESP-IDF v5.5.2 and
   fails on warnings or if the image won't fit the OTA partition.
2. **Release:** a manually-dispatched GitHub Actions workflow builds a versioned image and publishes
   it as a GitHub Release.
3. **OTA:** the device polls GitHub Releases on a schedule and auto-updates itself to the newest
   release, with rollback protection.

## Context & Constraints (from the existing codebase)

- ESP-IDF **v5.5.2**, target **esp32** only. Repo at `~/source/ESP32-homekit-garage-opener`
  (sibling of `~/esp/`); builds via `idf.py` after `. ~/esp/esp-idf/export.sh`.
- Dependencies: `esp-homekit-sdk` is a cloned git peer (no registry package), pinned commit
  **`676fabac4a4a05184be020611cb069faa0016411`**; the DHT driver is fetched by the Component
  Manager (`main/idf_component.yml`).
- Modules: `board.h`, `hap_garage.{c,h}`, `door_control.{c,h}`, `dht_sensor.{c,h}`, thin
  `app_main.c`.
- Partitions (`partitions_hap.csv`): dual OTA slots `ota_0`/`ota_1` at **1600K each**, plus
  `otadata`, `factory_nvs`, `nvs_keys`. The app image is currently **~98% of 1600K**.
- Current OTA: the HomeKit SDK `hap_fw_upgrade` service (hidden, custom UUID) with an empty
  `server_cert[]`. It is controller-triggered (a controller writes a URL characteristic) and the
  Apple Home app cannot drive it. This is being **replaced**.
- Repo GitHub slug: **`larsakeekstrand/ESP32-homekit-garage-opener`** (public).
- No automated test harness exists; the build is the verification gate.

## Key Decisions

- CI "test" = **build-only gate** (warnings-as-errors + partition-fit check). No unit tests.
- Release trigger = **manual `workflow_dispatch`** with a `version` input. Version is injected into
  the firmware and drives OTA comparison.
- OTA = **device self-polls hourly and auto-applies** newer releases (no HomeKit trigger, no
  manual step). Interval is menuconfig-configurable.
- The HomeKit `hap_fw_upgrade` service is **removed**.

## Design

### 1. CI build workflow — `.github/workflows/build.yml`

- **Triggers:** `push` to `main`; `pull_request` targeting `main`.
- **Job steps:**
  1. `actions/checkout`.
  2. Clone `esp-homekit-sdk` at the pinned commit `676fabac` into the runner and export
     `HOMEKIT_PATH` to it.
  3. Build with `espressif/esp-idf-ci-action@v1`, `esp_idf_version: v5.5.2`, target `esp32`,
     running `idf.py set-target esp32 build`. The Component Manager fetches the DHT driver
     (network available in CI).
  4. **Warnings-as-errors:** enabled for the project's own sources (via a build flag /
     `idf.py`-level setting), so new warnings fail CI. SDK/third-party warnings are not promoted.
  5. **Partition-fit gate:** read the produced app image size and fail the job if it exceeds the
     1600K `ota_0` slot. (Belt-and-suspenders: the linker already fails on overflow, but this gives
     a clear, early signal and guards the tight headroom.)
  6. Upload `build/garage.bin` as a CI artifact (inspection only; not a release).
- **No secrets required.**

### 2. Release workflow — `.github/workflows/release.yml`

- **Trigger:** `workflow_dispatch` with a required input `version` (e.g. `1.1.0`,
  no leading `v`).
- **Permissions:** `contents: write` (create tag + release).
- **Steps:**
  1. `actions/checkout`.
  2. Clone pinned `esp-homekit-sdk`; export `HOMEKIT_PATH`.
  3. Build via `esp-idf-ci-action` (v5.5.2, esp32) with the version injected:
     `idf.py -DFW_VERSION=<version> build` (see §3).
  4. Produce `version.txt` containing exactly `<version>`.
  5. Create git tag `v<version>` at the workflow's commit.
  6. Create a **GitHub Release** named `v<version>` with auto-generated notes, attaching **two
     assets**: `garage.bin` and `version.txt`.
- The release's assets are reachable at the stable URLs
  `https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/releases/latest/download/garage.bin`
  and `.../releases/latest/download/version.txt`.

### 3. Build-time firmware version

- `CMakeLists.txt` (or `main/CMakeLists.txt`) reads a `FW_VERSION` CMake cache variable, defaulting
  to `0.0.0-dev` when unset (local/CI builds), and adds it as a compile definition
  (e.g. `-DFW_VERSION="0.0.0-dev"`).
- `hap_garage.c` uses `FW_VERSION` for `hap_acc_cfg_t.fw_rev` instead of the literal `"1.0.0"`.
- The OTA module (§4) uses the same `FW_VERSION` as its local-version baseline for comparison.

### 4. Self-updating OTA module — `main/ota_update.{c,h}`

- **Interface:**
  - `void ota_update_start(void);` — spawn the updater task.
  - `void ota_update_mark_valid(void);` — call after a healthy boot to confirm the running image
    and cancel pending rollback.
- **Task behavior** (low priority; loops):
  1. Sleep for the configured interval (Kconfig, default **3600 s**).
  2. HTTPS-GET `<base>/releases/latest/download/version.txt` into a small buffer, using an
     `esp_http_client`/`esp_https_ota` config with `crt_bundle_attach = esp_crt_bundle_attach`
     (trusts GitHub's CA chain via the IDF cert bundle; follows the `latest/download` redirect to
     the asset CDN).
  3. **Semver compare** the fetched version to `FW_VERSION` (parse `major.minor.patch`; ignore a
     `-dev` suffix → treat as oldest). If the remote is strictly newer, proceed; else loop.
  4. Run `esp_https_ota()` against `<base>/releases/latest/download/garage.bin` with the same cert
     bundle config. On success, log and `esp_restart()` into the new slot. On failure, log and
     continue the loop (retry next interval).
- `<base>` = `https://github.com/larsakeekstrand/ESP32-homekit-garage-opener` (defined via Kconfig
  so the owner/repo is configurable).
- **Rollback safety:**
  - Enable `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` (§6).
  - After boot reaches a healthy state (Wi-Fi connected + HomeKit started), `app_main`/the garage
    task calls `ota_update_mark_valid()` → `esp_ota_mark_app_valid_cancel_rollback()`. If a freshly
    OTA'd image never reaches that point (boot loop/crash), the bootloader reverts to the previous
    slot on the next reset.
- Spawned from `app_main` alongside the existing garage + DHT tasks.

### 5. Remove the HomeKit firmware-upgrade service

- In `hap_garage.c`: remove `#include <hap_fw_upgrade.h>`, the `server_cert[]` global, the
  `hap_fw_upgrade_config_t` + `hap_serv_fw_upgrade_create` call, and the corresponding
  `hap_acc_add_serv` for that service. The accessory keeps Garage Door Opener + Temperature +
  Humidity services only.

### 6. Configuration

- **`sdkconfig.defaults`:**
  - `CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y` — CA bundle for HTTPS to GitHub.
  - `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` — auto-revert bad OTA images.
  - `CONFIG_COMPILER_OPTIMIZATION_SIZE=y` — `-Os` for flash headroom (replaces the regenerated
    default `-Og`).
- **`main/Kconfig.projbuild`:** add
  - `OTA_POLL_INTERVAL_SEC` (int, default `3600`).
  - `OTA_GITHUB_BASE_URL` (string, default the repo's
    `https://github.com/larsakeekstrand/ESP32-homekit-garage-opener`).

### 7. Documentation

- Update `README.md` and `CLAUDE.md`: CI/release badges/notes, how to cut a release (dispatch the
  workflow with a version), and how the device self-updates (hourly poll of latest release;
  rollback behavior). Note the new `ota_update` module in the architecture list and that
  `hap_fw_upgrade` was removed.

## Verification

1. **Workflows:** lint the YAML and dry-run the equivalent build commands locally; confirm the
   build succeeds with `FW_VERSION` injected. (Cannot run GitHub Actions locally.)
2. **Build:** `idf.py build` clean with the new module + config; CI partition-fit gate passes.
3. **Release (user):** run the dispatch workflow once; confirm a `v<version>` Release with
   `garage.bin` + `version.txt`.
4. **OTA end-to-end (user, the real proof):** flash a build, publish a higher-versioned release,
   and confirm the device pulls + flashes + reboots into it within the poll interval, and that a
   deliberately-broken image rolls back.

## Out of Scope

- Unit tests / test harness.
- Signed OTA, secure boot, anti-rollback fuses (trust model: repo-release publishers can push code
  to the device — acceptable for a personal repo).
- Multi-target builds (esp32 only).
- Pushing updates to the device from CI (impossible across the LAN boundary; device-pull only).

## Risks

1. **`esp_https_ota` + GitHub `latest/download` redirect + cert bundle** — the make-or-break path.
   Mitigation: verify on-device early; if the redirect-to-CDN cert doesn't validate via the bundle,
   fall back to resolving the asset's direct URL or hosting elsewhere. The on-device OTA test is the
   proof.
2. **Partition headroom (~98%)** — adding the OTA module may overflow 1600K. Mitigation: `-Os`
   (expected to free ample space) + CI partition-fit gate. If still tight, revisit (e.g. resize OTA
   slots — larger change, would be flagged before doing it).
3. **Auto-update of a bad release** — mitigated by rollback (§4). The mark-valid placement must be
   correct (only after a genuinely healthy boot) or rollback won't protect.
4. **Unauthenticated `version.txt`/binary** — transport is authenticated (cert bundle) and only
   repo owners publish releases; no payload signature (documented, out of scope).

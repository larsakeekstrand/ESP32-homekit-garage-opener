# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A HomeKit (Apple HAP) garage door opener firmware for the ESP32, built on Espressif's ESP-IDF v5.5.2.
A single relay kicks the door, a reed (magnetic) sensor reports open/closed, and a DHT22/AM2301
sensor exposes temperature + humidity as additional HomeKit services. The firmware is split across
five modules under `main/` (see Architecture below).

## Build environment (read this first)

This project is **not self-contained**. Two external dependencies must be present:

1. **`esp-homekit-sdk`** — cloned git peer (provides `hap*`, `app_wifi`, `examples/common`).
2. **DHT22/AM2301 driver** — fetched automatically by the ESP Component Manager from the registry
   (declared in `main/idf_component.yml` as `esp-idf-lib/dht ^1.0.0`, resolved into
   `managed_components/`). No manual clone of `esp-idf-lib` is required.

The repo lives at `~/source/ESP32-homekit-garage-opener`; its parent `~/source/` is a **sibling** of `~/esp/`:

```
~/
  esp/
    esp-idf            # Espressif ESP-IDF v5.5.2 (provides idf.py, FreeRTOS, drivers)
    esp-homekit-sdk    # Espressif HomeKit SDK (provides hap*, app_wifi, examples/common)
  source/
    ESP32-homekit-garage-opener   # this repo
```

`CMakeLists.txt` honors the `HOMEKIT_PATH` env var as an override; otherwise it falls back to
`../../esp/esp-homekit-sdk` (which resolves to `~/esp/esp-homekit-sdk` from this checkout).
`EXTRA_COMPONENT_DIRS` pulls in the HomeKit `components/` + `components/homekit` and
`esp-homekit-sdk/examples/common` (the source of `app_wifi` and `app_hap_setup_payload`).

## Commands

```bash
. ~/esp/esp-idf/export.sh      # activate the ESP-IDF v5.5.2 toolchain (once per shell)
idf.py set-target esp32        # run once before the first build (target is esp32 only)
idf.py menuconfig              # set Wi-Fi SSID/key (App Wi-Fi config) and HomeKit setup code/id
idf.py build
idf.py flash monitor           # flash over serial and open the log monitor
```

The build is **CMake only** via `idf.py`. The legacy GNU-make build has been removed.

There are **no tests, linters, or CI** in this repo. "Verify" means it compiles and behaves
correctly on hardware via `idf.py flash monitor`.

## Configuration that matters

- **`sdkconfig.defaults`** — pins the build to a custom partition table (`partitions_hap.csv`),
  forces single-core FreeRTOS (`CONFIG_FREERTOS_UNICORE=y`), 4 MB flash, and disables the task
  watchdog. Edit this for persistent config; `sdkconfig` is the generated working copy.
- **`partitions_hap.csv`** — dual OTA app slots (`ota_0`/`ota_1`, 1600K each) plus a `factory_nvs`
  partition. HomeKit setup info is expected in `factory_nvs` for production; see below.
- **`main/Kconfig.projbuild`** — defines `EXAMPLE_USE_HARDCODED_SETUP_CODE` (default `y`),
  `EXAMPLE_SETUP_CODE` (default `111-22-333`), and `EXAMPLE_SETUP_ID` (default `ES32`). With the
  hardcoded option on, `hap_garage.c` calls `hap_set_setup_code`/`hap_set_setup_id` directly. For
  production, generate setup info with `factory_nvs_gen` and flash it to `factory_nvs` instead.

## Hardware pin map (defined as macros in `main/board.h`)

| Function            | GPIO | Notes |
|---------------------|------|-------|
| Reset/factory button| 0    | "Boot" button — short hold resets Wi-Fi, long hold resets to factory |
| Running LED         | 2    | on while firmware runs |
| Door state LED      | 16   | mirrors door open/closed |
| Door relay          | 5    | pulsed ~300 ms to "kick" the door |
| Reed door sensor    | 4    | input, pull-up, any-edge interrupt; logic is inverted (`^1`) |
| DHT22 / AM2301      | 22   | temperature + humidity |

## Architecture

The firmware is split into five modules under `main/`:

| File | Responsibility |
|------|---------------|
| `board.h` | GPIO pin macros and board-level constants. |
| `hap_garage.{c,h}` | HomeKit accessory + services (Garage Door Opener, Temperature, Humidity, FW-Upgrade/OTA), `garage_read`/`garage_write` HAP callbacks, the four `hap_garage_send_*` notify helpers, and the HAP + Wi-Fi lifecycle (`hap_garage_init`, `hap_garage_start_wifi`). |
| `door_control.{c,h}` | Door state machine, relay (300 ms kick), reed-sensor ISR + queue with ~50 ms debounce, 8 s failsafe timer, initial-state publish, and the consolidated event loop (`door_control_run_loop`). |
| `dht_sensor.{c,h}` | DHT polling task (30 s interval), cached readings, `dht_sensor_get(float*, float*)`. |
| `app_main.c` | Thin composition root: spawns the garage task and starts the DHT task. |

Two FreeRTOS tasks run on the single configured core:

1. **`garage_thread_entry`** — calls `hap_garage_init()` → `door_control_init()` →
   `hap_garage_start_wifi()` (blocking Wi-Fi/HAP bring-up) → `door_control_run_loop()` (never
   returns).
2. **DHT task** (spawned by `dht_sensor_start()`) — polls the DHT sensor every 30 s, caches the
   readings, and pushes temperature/humidity to HomeKit.

### The door state machine

Two sources of truth are reconciled:

- **physical**: `read_door_sensor()` reads the reed switch (inverted: returns `DOOR_OPEN`/`DOOR_CLOSED`).
- **logical**: the module-local `door_state` holds a HomeKit *current door state* enum
  (OPEN/CLOSED/OPENING/CLOSING/STOPPED/UNKNOWN), accessed via `door_get_state()`.

Flow:
- A HomeKit open/close request lands in `garage_write()`, which calls
  `door_handle_homekit_request()`: pulses the relay (`kick_door_relay()`), sets `door_state` to
  OPENING/CLOSING, and arms a one-shot timer (`DOOR_MOVING_MAXTIME`, 8 s).
- The reed sensor's edge interrupt enqueues the GPIO number; `door_control_run_loop()` dequeues it,
  applies a ~50 ms debounce, and on a genuine edge emits `hap_garage_send_current_door_state` /
  `hap_garage_send_target_door_state` updates.
- If the sensor never confirms the move, `timer_callback()` fires after 8 s and forces `door_state`
  to match whatever the sensor currently reads (failsafe).

`hap_garage_send_current_door_state` / `hap_garage_send_target_door_state` /
`hap_garage_send_current_temperature` / `hap_garage_send_current_humidity` are the bridge to
HomeKit: they look up the service + characteristic by UUID on the global accessory and call
`hap_char_update_val`.

On-demand reads (`garage_read`) for temperature/humidity return the values cached by
`dht_sensor_get()` rather than doing a live sensor read.

### HomeKit accessory composition

Built in `hap_garage_init()`: a Garage Door Opener service (with `garage_read`/`garage_write`
callbacks), a Temperature Sensor service, a Humidity Sensor service, and the firmware-upgrade (OTA)
service from `hap_fw_upgrade`. OTA cert verification is disabled (`server_cert[]` is empty).

## Conventions

- C-style block comments delimiting each function, `ESP_LOGI`/`ESP_LOGE` with the `"HAP Garage"`
  tag for all tracing. Match this style when extending.
- Door-state and target-state values use the `HOMEKIT_CHARACTERISTIC_*` macros defined in
  `hap_garage.h` (which mirror the HAP spec enum), not magic numbers.
- Build artifacts (`build/`, `managed_components/`, `.cache/`, `sdkconfig.old`) are excluded by
  `.gitignore`.

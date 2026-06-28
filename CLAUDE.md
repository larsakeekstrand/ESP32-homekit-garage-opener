# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A HomeKit (Apple HAP) garage door opener firmware for the ESP32, built on Espressif's ESP-IDF.
A single relay kicks the door, a reed (magnetic) sensor reports open/closed, and a DHT22/AM2301
sensor exposes temperature + humidity as additional HomeKit services. All logic lives in one file:
`main/app_main.c`.

## Build environment (read this first)

This project is **not self-contained**. The build resolves three external dependencies via paths
**two directories up** (`../../`), so the repo must live inside an `esp/` workspace laid out exactly
like this:

```
esp/
  esp-idf            # Espressif ESP-IDF (provides idf.py, FreeRTOS, drivers)
  esp-idf-lib        # UncleRus esp-idf-lib (provides the dht component)
  esp-homekit-sdk    # Espressif HomeKit SDK (provides hap*, app_wifi, examples/common)
  source/
    ESP32-homekit-garage-opener   # this repo
```

`CMakeLists.txt` honors the `HOMEKIT_PATH` env var as an override; otherwise it falls back to the
relative layout above. `EXTRA_COMPONENT_DIRS` pulls in `esp-idf-lib/components`, the HomeKit
`components/` + `components/homekit`, and `esp-homekit-sdk/examples/common` (the source of
`app_wifi` and `app_hap_setup_payload`).

## Commands

```bash
idf.py set-target esp32        # or esp32s2 / esp32c3 — run once before first build
idf.py menuconfig              # set Wi-Fi SSID/key and HomeKit setup code/id
idf.py build
idf.py flash monitor           # flash over serial and open the log monitor
```

The CMake build (`idf.py`) is the current path. A legacy GNU-make build (`Makefile` +
`main/component.mk`, project name `garage`) still exists but `idf.py` is preferred.

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
  hardcoded option on, `app_main.c` calls `hap_set_setup_code`/`hap_set_setup_id` directly. For
  production, generate setup info with `factory_nvs_gen` and flash it to `factory_nvs` instead.

## Hardware pin map (defined as macros in `app_main.c`)

| Function            | GPIO | Notes |
|---------------------|------|-------|
| Reset/factory button| 0    | "Boot" button — short hold resets Wi-Fi, long hold resets to factory |
| Running LED         | 2    | on while firmware runs |
| Door state LED      | 16   | mirrors door open/closed |
| Door relay          | 5    | pulsed ~300 ms to "kick" the door |
| Reed door sensor    | 4    | input, pull-up, any-edge interrupt; logic is inverted (`^1`) |
| DHT22 / AM2301      | 22   | temperature + humidity |

## Architecture

Two FreeRTOS tasks are spawned from `app_main()`, both on the single configured core:

1. **`garage_thread_entry`** — sets up GPIO + the HAP accessory, then runs an infinite loop that
   blocks on `gpio_evt_queue` (fed by the reed-sensor ISR). This is the heart of the state machine.
2. **`dht_thread_entry`** — polls the DHT sensor every 30 s and pushes temperature/humidity to
   HomeKit.

### The door state machine

Two sources of truth are reconciled:

- **physical**: `read_door_sensor()` reads the reed switch (inverted: returns `DOOR_OPEN`/`DOOR_CLOSED`).
- **logical**: the global `door_state` holds a HomeKit *current door state* enum
  (OPEN/CLOSED/OPENING/CLOSING/STOPPED/UNKNOWN), accessed via `get_door_state()`/`set_door_state()`.

Flow:
- A HomeKit open/close request lands in `garage_write()`, which pulses the relay
  (`kick_door_relay()`), sets `door_state` to OPENING/CLOSING, and arms a one-shot timer
  (`DOOR_MOVING_MAXTIME`, 8 s).
- The reed sensor's edge interrupt wakes the main loop, which compares the new sensor value against
  the expected `door_state` to decide whether movement was **HomeKit-initiated** or **manual**, then
  emits the appropriate `send_current_door_state` / `send_target_door_state` updates.
- If the sensor never confirms the move, `timer_callback()` fires after 8 s and forces `door_state`
  to match whatever the sensor currently reads (failsafe).

`send_current_door_state` / `send_target_door_state` / `send_current_temperature` /
`send_current_humidity` are the bridge to HomeKit: they look up the service + characteristic by UUID
on the global `accessory` and call `hap_char_update_val`.

### HomeKit accessory composition

Built in `garage_thread_entry`: a Garage Door Opener service (with `garage_read`/`garage_write`
callbacks), a Temperature Sensor service, a Humidity Sensor service, and the firmware-upgrade (OTA)
service from `hap_fw_upgrade`. OTA cert verification is disabled (`server_cert[]` is empty).

## Conventions

- Single C file, C-style block comments delimiting each function, `ESP_LOGI`/`ESP_LOGE` with the
  `"HAP Garage"` tag for all tracing. Match this style when extending.
- Door-state and target-state values use the local `HOMEKIT_CHARACTERISTIC_*` macros (which mirror
  the HAP spec enum), not magic numbers.

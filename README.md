# ESP32 Homekit garage door opener
## Introduction
This is a homekit garage opener for the ESP32. It's a bit a work in progress, but it should mostly work.

Features:
* Controls a relay that opens/closes the door.
* Uses a reed sensor to detect if the door is open/closed.
* Contains a DHT22 temperature/humidity sensor.

Schematics for the hardware can be found [here](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/blob/478886ba8ccb8f31c0db583b10b282738d822965/docs/Schematic_GarageOpener.pdf). The schematics is made using [EasyEDA](https://easyeda.com/)

STL files for the housing can be found [here](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/blob/2dc0cb2f88ffee0f719a3f48c10d991301077dc2/docs/GarageOpenerTop.stl), [here](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/blob/2dc0cb2f88ffee0f719a3f48c10d991301077dc2/docs/GarageOpenerBottom.stl) and [here](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/blob/d89407ebe4e6d4afd1c8a517a600cc80013ba3aa/docs/GarageOpenerBottomButton.stl).

An overview of how to assemble the housing can be found [here](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/blob/d89407ebe4e6d4afd1c8a517a600cc80013ba3aa/docs/GarageOpenerCompleteNotPrintable.stl).



## Getting started
### Setup host environment

Set up the host environment and **Espressif ESP-IDF v5.5.2** using the instructions given [here](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html).

Set up the Espressif esp-homekit-sdk using the instructions given [here](https://github.com/espressif/esp-homekit-sdk).

The DHT22/AM2301 driver is fetched automatically by the **ESP Component Manager** (declared in `main/idf_component.yml`) — no separate clone of `esp-idf-lib` is required.


### Get ESP32-homekit-garage-opener

Clone this repository **alongside** (not inside) your `~/esp/` directory:

```
cd ~/source    # ~/source is a sibling of ~/esp
git clone https://github.com/larsakeekstrand/ESP32-homekit-garage-opener
```

After the installation the directory structure should look like this:

```
~/
  esp/
    esp-idf            # ESP-IDF v5.5.2
    esp-homekit-sdk    # Espressif HomeKit SDK (cloned peer)
  source/
    ESP32-homekit-garage-opener   # this repo
```

`CMakeLists.txt` locates `esp-homekit-sdk` at `../../esp/esp-homekit-sdk` relative to the repo root, or via the `HOMEKIT_PATH` environment variable.

### Configure
Source the ESP-IDF environment, then run `idf.py menuconfig` to set Wi-Fi credentials and the HomeKit setup code before flashing:
```bash
. ~/esp/esp-idf/export.sh
cd ~/source/ESP32-homekit-garage-opener
idf.py menuconfig
```

Set the Wi-Fi SSID and password under **App Wi-Fi configuration**. Set the HomeKit setup code and ID under **HAP Setup** (or leave the hardcoded defaults for development).

### Compile and Flash

```bash
idf.py set-target esp32
idf.py build
idf.py flash monitor
```

## Continuous Integration & Releases

![Build](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/actions/workflows/build.yml/badge.svg)

### CI (automatic)

Every push to `main` and every pull request targeting `main` triggers the **Build** workflow
(`.github/workflows/build.yml`). It:

- Builds the firmware with ESP-IDF v5.5.2 and target `esp32`.
- Verifies the image fits the 1600 KB OTA partition.
- Uploads `garage.bin` as a downloadable artifact.

Warnings in the app sources are treated as errors (`-Werror`), so a warning will fail the build.

### Cutting a release (manual)

1. Go to the **Actions** tab in GitHub.
2. Select the **Release** workflow.
3. Click **Run workflow**, enter a version string (e.g. `1.1.0`, no leading `v`), and confirm.

The workflow builds with that version baked in as `FW_VERSION`, tags the commit `v1.1.0`, and
publishes a GitHub Release with two assets: `garage.bin` and `version.txt`.

### Self-updating OTA

The device checks `releases/latest` on GitHub **every hour** (default). If `version.txt` reports a
version newer than the running firmware, the device downloads `garage.bin` via HTTPS and reboots
into the new image. A bad image that never completes initialization is **automatically rolled back**
by the bootloader.

The poll interval and repo base URL are configurable via `idf.py menuconfig` →
**Example Configuration** → OTA options (`CONFIG_OTA_POLL_INTERVAL_SEC`,
`CONFIG_OTA_GITHUB_BASE_URL`).

> **Security note:** anyone who can publish a GitHub Release to this repo can push firmware to the
> device. Images are not cryptographically signed — acceptable for a personal repo.

### Wi-Fi credentials & OTA

Wi-Fi credentials are stored in the **`nvs` flash partition**, which OTA never touches (OTA only
rewrites the inactive app slot). So **credentials persist across updates** — you don't re-enter them
when the device self-updates.

The release images built by CI are **provisioning-mode** (no credentials baked in). Don't put your
SSID/password into the build or into GitHub — provision the device instead:

- **First-time / after a reset:** when the device is unprovisioned it advertises a BLE service named
  `PROV_XXYYZZ` (last 3 bytes of its MAC) and prints a **QR code + proof-of-possession (POP)** to the
  serial monitor. Install Espressif's **ESP BLE Provisioning** app (iOS/Android), scan the QR code
  (or pick `PROV_XXYYZZ` and enter the POP), choose your Wi-Fi, and enter the password. The creds are
  saved to NVS over an encrypted BLE link.
- **Re-provision:** short-press the boot/reset button (~3 s) to reset the network → the device
  re-advertises for provisioning. A long press (~10 s) is a full factory reset (also unpairs
  HomeKit).

> A locally-built **hardcoded** image (`menuconfig` → App Wi-Fi → *Use hardcoded credentials*) is fine
> for bench testing, but the first OTA replaces it with a provisioning image. Because esp-wifi caches
> the hardcoded credentials in NVS, the provisioning image reconnects using them — but from then on
> the device is provisioning-based, so re-provision over BLE if you ever clear the network config.

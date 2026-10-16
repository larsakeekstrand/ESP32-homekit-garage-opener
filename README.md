# ESP32 Homekit garage door opener
## Introduction
This is a homekit garage opener for the ESP32. It's a bit a work in progress, but it should mostly work.

Features:
* Controls a relay that opens/closes the door.
* Uses a reed sensor to detect if the door is open/closed.
* Contains a DHT22 temperature/humidity sensor.
* Auto-close timer (default 10 min, configurable, 0 = off).
* Door-left-open HomeKit alert via a Contact Sensor accessory.
* Vacation/lockout switch to refuse all open and close commands.
* Local status and configuration web page at `http://<device-ip>:8080/`.

Schematics for the hardware are in [docs/Schematic_GarageOpener.pdf](docs/Schematic_GarageOpener.pdf), drawn in [KiCad](https://www.kicad.org/) — the project (rev 1 carrier PCB for an ESP32 DevKitC V4) is in [hardware/garage-opener-pcb](hardware/garage-opener-pcb), with a bill of materials in [docs/BOM_GarageOpener.pdf](docs/BOM_GarageOpener.pdf). The routed board and Aisler-ready Gerbers are in [hardware/garage-opener-pcb/fab](hardware/garage-opener-pcb/fab), and a 3D-printable wall-mounted enclosure for it is in [hardware/enclosure](hardware/enclosure/README.md).

The STL files below are for the original hand-wired prototype housing.

STL files for the housing can be found [here](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/blob/2dc0cb2f88ffee0f719a3f48c10d991301077dc2/docs/GarageOpenerTop.stl), [here](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/blob/2dc0cb2f88ffee0f719a3f48c10d991301077dc2/docs/GarageOpenerBottom.stl) and [here](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/blob/d89407ebe4e6d4afd1c8a517a600cc80013ba3aa/docs/GarageOpenerBottomButton.stl).

An overview of how to assemble the housing can be found [here](https://github.com/larsakeekstrand/ESP32-homekit-garage-opener/blob/d89407ebe4e6d4afd1c8a517a600cc80013ba3aa/docs/GarageOpenerCompleteNotPrintable.stl).



## Safety features

### Auto-close timer

When the door has been open for more than **10 minutes** (default), the firmware automatically
sends a close command. The timeout is configurable on the local web page (see below); set it to
`0` to disable auto-close entirely.

### Door-left-open alert

A HomeKit **Contact Sensor** named **"Garage Left Open"** fires when the door has been open for
more than **5 minutes** (default). Enable notifications on it in the Home app to receive an alert.
Set the threshold to `0` on the web page to disable the alert. The sensor still reports even when
the lockout switch is ON.

### Vacation / lockout switch

A HomeKit **Switch** named **"Lockout"** puts the opener into an absolute freeze: while it is ON,
HomeKit open and close commands are refused and the auto-close timer is suspended. It can be
toggled from both the Home app and the local web page — they stay in sync.

### Local status and configuration web page

The device serves a status and config page at `http://<device-ip>:8080/` on the local network (plain
HTTP, no HTTPS, LAN only). Port **8080** is used because the HomeKit (HAP) server already occupies
port 80. Find the device's IP from your router, or via mDNS — it advertises over Bonjour, e.g.
`dns-sd -B _hap._tcp` shows it as `GarageOpener-XXXXXX`.

**First-time setup — set a password promptly.** On the very first visit the device has no password
and shows a one-time "create password" screen (trust-on-first-use). Anyone on the same LAN can set
the password until you do, so set it immediately after first access. The firmware-flash endpoint
(`POST /api/ota`) is equally open during this window — since it can replace the running firmware,
set the password promptly, especially on an untrusted LAN (uploaded images are validated against the
`garage` app descriptor and the endpoint is LAN-only, but the credential gate is the main barrier).

After the password is set, the page requires **HTTP Basic Auth** (username `admin`, your chosen
password). From there you can:

* View live status: door state, lockout, Wi-Fi RSSI, temperature, humidity, uptime.
* View system info: firmware version, free heap, MAC address, IP address, reset reason.
* View persisted counters: door cycles, auto-closes, door-left-open trips, faults.
* Edit the auto-close timeout, door-left-open threshold, **OTA poll interval** (5–1440 min),
  **door-move timeout** (3–30 s), and **relay pulse length** (100–2000 ms). All settings persist
  in NVS. The OTA poll interval takes effect within ~10 s — no reboot needed.
* Trigger an immediate GitHub version check with **Check for updates now** (no effect while
  auto-updates are paused — resume first).
* Restart the device with **Reboot device** (asks for confirmation; HomeKit and the web page drop
  for ~15 s while it boots).
* Change the admin password.
* Reset a latched auto-close fault (see below).

**Editing the web page source.** The page HTML and JavaScript live in `tools/web/page.html`
(main status/config page) and `tools/web/setup.html` (first-time password setup). After editing
either file, regenerate the embedded C string literals:

```bash
python3 tools/gen_web_page.py
```

This rewrites the relevant sections of `main/web_server.c`. Rebuild and flash as normal.
Edit only `tools/web/*.html` — the `PAGE_HTML` / `SETUP_HTML` C strings in `web_server.c` are
generated and will be silently overwritten by the next `gen_web_page.py` run.

### Auto-close fault detection

If the reed sensor remains stuck reporting "open" after a close command, the firmware makes at most
**3 attempts** then latches auto-close off, so it never oscillates the door. A minimum of 60
seconds between pulses and a per-hour cap serve as additional backstops.

The fault is surfaced three ways:

1. A banner on the local web page with a **Reset** button to clear it manually.
2. A HomeKit **StatusFault** indicator on the Garage Door Opener accessory.
3. A log message at error severity.

The fault clears automatically when the door genuinely reports closed, or immediately when you
press **Reset** on the web page.

### HomeKit caveat after updating

If you are updating from a firmware version that does not include these features, you may need to
**remove and re-add the accessory** in the Home app to make the new **"Garage Left Open"** Contact
Sensor and **"Lockout"** Switch tiles visible.

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

### LAN push-OTA (forced / beta updates)

You can push a locally-built image directly to the device over the local network — useful for
testing a beta build without cutting a GitHub release.

**Build a versioned beta first.** A build with the default `FW_VERSION=0.0.0-dev` is treated as
*older* than the latest release by the hourly poller, so it would be reverted on the next poll.
Pass a real version to avoid that:

```bash
idf.py -DFW_VERSION=1.2.0-beta build
```

**Upload with `garage-push`.** The CLI lives at `tools/garage-push/` (Rust). Build and run it with
`cargo`:

```bash
cd tools/garage-push
cargo build                            # or: cargo run -- <subcommand>
```

Subcommands:

| Subcommand | What it does |
|------------|-------------|
| `push --host <ip> --file build/garage.bin` | Uploads the image to the device, waits for it to reboot into the new firmware |
| `resume --host <ip>` | Re-enables the hourly GitHub auto-update poller |
| `status --host <ip>` | Prints the device status JSON |

The `push` subcommand authenticates with HTTP Basic Auth (username `admin`, your web page password).
The password is resolved in order: `--password <pw>` flag → `$GARAGE_PASSWORD` environment variable
→ interactive prompt.

The device validates the uploaded image before flashing: it checks the app descriptor for the
correct project name and magic word. A wrong or corrupt image is rejected and the running firmware
is untouched. An image that does boot but then crashes or hangs is **automatically rolled back** by
the bootloader (same rollback mechanism as GitHub OTA).

**Auto-update pause.** A successful `push` automatically pauses the hourly GitHub poller; the pause
flag is stored in NVS and survives reboots. Consequently, if the pushed beta fails to boot and the
bootloader rolls back to the previous image, that restored image comes back with auto-updates still
paused — run `garage-push resume --host <ip>` (or toggle the web-page **Auto-update** switch) to
re-enable them. Re-enable auto-updates with:

```bash
cargo run -- resume --host <ip>
```

Or toggle the **Auto-update** switch on the local web page — it shows **on** or **PAUSED**.

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

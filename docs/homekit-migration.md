# Moving the HomeKit pairing to a new board

How to switch the Home app from an old ESP32 board to a replacement one (e.g.
after rebuilding on an ESP32-WROOM-32U — see
[`wiring-wroom-32u.html`](wiring-wroom-32u.html)).

## The one thing to know first

HomeKit **cannot transfer a pairing between two physical boards.** To HomeKit the
new board is a completely separate accessory, so this is a *remove-old +
add-new*, not a swap.

Why: a HAP accessory's identity is its **device ID**, derived from the ESP32's
**Wi-Fi MAC address**, which is burned into each chip and differs between boards.
The Home app therefore sees the new board as a new device even though it runs
identical firmware.

Both boards share the same **setup code** (`111-22-333`, from
`EXAMPLE_SETUP_CODE` in `main/Kconfig.projbuild`) — that's just the pairing PIN,
not the identity, so there's no conflict in two boards having the same code.

**Consequence:** the room assignment, name, scenes, and automations attached to
the old accessory do **not** carry over. You re-create them on the new one.
There is no DIY-HomeKit way around this.

## Steps

### 1. Get firmware + Wi-Fi onto the new board first

Do this *before* touching the Home app. Flash over serial:

```bash
. ~/esp/esp-idf/export.sh
idf.py flash monitor
```

Then the board needs to join Wi-Fi. Which path depends on how the image was
built:

- **Provisioning image** (the CI/release `garage.bin`, built with
  `CONFIG_ENABLE_UNIFIED_PROVISIONING=y`): on first boot it advertises over BLE
  as `PROV_XXYYZZ` (last 3 MAC bytes) and prints a QR code + PoP to the serial
  log. Use Espressif's **ESP BLE Provisioning** app to hand it your Wi-Fi
  credentials.
- **Hardcoded-credentials build** (SSID/key set via `idf.py menuconfig`): it
  connects automatically.

It's ready once the monitor shows it obtained an IP and HAP started. The
periodic `WiFi RSSI: … dBm` line (logged every 30 s) is a good "it's alive and
connected" signal.

### 2. Add the new accessory in the Home app

1. Home app → **+** → **Add Accessory**.
2. Scan the **QR code** printed in the serial monitor, or tap
   **"More options…" / "I don't have a code"** and enter the setup code manually:
   **`111-22-333`**.
3. It is not MFi-certified, so you'll get an **"Uncertified Accessory"** warning
   → **Add Anyway**.
4. Assign it to a room and name it.

The accessory advertises as `GarageOpener-XXYYZZ` (the name is made unique per
board via `UNIQUE_NAME`), so it won't be confused with the old one.

### 3. Remove the old accessory

1. In the Home app, tap the old garage opener → scroll down →
   **Remove Accessory**.
2. Power down / disconnect the old board.

**Recommended order:** add the new board and confirm it opens/closes the door
**before** removing the old one, so you're never left without a working opener.

## Gotcha: "the new board won't pair"

If the new board was **ever paired during testing**, HomeKit will refuse to
advertise for setup until it's reset. Clear it with the **Boot button**:

| Hold time | Action | Effect |
|-----------|--------|--------|
| ~3 s (short) | `hap_reset_network` | Clears Wi-Fi only → re-advertises for BLE provisioning |
| ~10 s (long) | `hap_reset_to_factory` | Unpairs HomeKit **and** clears Wi-Fi → fully fresh, pairable |

For a stuck pairing, use the **long (~10 s) hold** to return the board to a
fresh, pairable state, then repeat from step 1.

## Quick reference

| Item | Value |
|------|-------|
| Setup code (pairing PIN) | `111-22-333` |
| Setup ID | `ES32` |
| Advertised name | `GarageOpener-XXYYZZ` |
| BLE provisioning name | `PROV_XXYYZZ` |
| Re-pair reset | Boot button, long hold (~10 s) |

Setup code and ID come from `main/Kconfig.projbuild`
(`EXAMPLE_USE_HARDCODED_SETUP_CODE` is on by default). For a production build the
setup info would instead live in the `factory_nvs` partition.

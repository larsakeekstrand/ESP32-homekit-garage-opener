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

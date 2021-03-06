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

Set up the host environment and Espressif esp-idf using the instructions given [here](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html).

Set up the Espressif esp-homekit-sdk using the instructions given [here](https://github.com/espressif/esp-homekit-sdk).

Set up the esp-idf-lib using the instructions given [here](https://github.com/UncleRus/esp-idf-lib).


### Get ESP32-homekit-garage-opener

Clone this repository using the command below:

```
cd esp
mkdir source
cd source
git clone https://github.com/larsakeekstrand/ESP32-homekit-garage-opener
```

After the installation the directory structure should look like this:

```
esp/
  esp-idf
  esp-idf-lib
  esp-homekit-sdk
  source/
    ESP32-homekit-garage-opener
```

### Configure
You will need to set wifi SSID/key using the esp-idf config tool:
```
cd /path/to/ESP32-homekit-garage-opener
make menuconfig  
```
### Compile and Flash

```
$ cd /path/to/ESP32-homekit-garage-opener
$ idf.py set-target <esp32/esp32s2/esp32c3>
$ idf.py build
$ idf.py flash monitor
```

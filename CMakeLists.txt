# ESP32 HomeKit garage door opener — ESP-IDF v5.5.2
cmake_minimum_required(VERSION 3.16)

if(DEFINED ENV{HOMEKIT_PATH})
  set(HOMEKIT_PATH $ENV{HOMEKIT_PATH})
else()
  # This repo lives at ~/source/ESP32-homekit-garage-opener, a sibling of ~/esp/,
  # so the SDK (cloned next to esp-idf) is at ../../esp/esp-homekit-sdk.
  set(HOMEKIT_PATH ${CMAKE_CURRENT_LIST_DIR}/../../esp/esp-homekit-sdk)
endif()

set(EXTRA_COMPONENT_DIRS
    ${HOMEKIT_PATH}/components
    ${HOMEKIT_PATH}/components/homekit
    ${HOMEKIT_PATH}/examples/common)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(garage)

idf_component_register(SRCS "app_main.c" "hap_garage.c" "dht_sensor.c" "door_control.c" "ota_update.c"
                       INCLUDE_DIRS ".")

# Firmware version: overridable with `idf.py -DFW_VERSION=1.2.3 build`.
if(NOT DEFINED FW_VERSION OR FW_VERSION STREQUAL "")
  set(FW_VERSION "0.0.0-dev")
endif()
message(STATUS "Building firmware version: ${FW_VERSION}")
target_compile_definitions(${COMPONENT_LIB} PRIVATE FW_VERSION="${FW_VERSION}")

# Treat warnings in our own code as errors (does not affect the SDK/third-party components).
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)

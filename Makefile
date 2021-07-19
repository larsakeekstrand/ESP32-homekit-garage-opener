#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

HOMEKIT_PATH ?= $(abspath $(shell pwd)/../../esp-homekit-sdk)
IDF_LIB_PATH ?= $(abspath $(shell pwd)/../../esp-idf-lib)
COMMON_COMPONENT_PATH ?= $(abspath $(shell pwd)/../../esp-homekit-sdk/examples/common)

PROJECT_NAME := garage
EXTRA_COMPONENT_DIRS += $(HOMEKIT_PATH)/components/
EXTRA_COMPONENT_DIRS += $(HOMEKIT_PATH)/components/homekit
EXTRA_COMPONENT_DIRS += $(IDF_LIB_PATH)/components
EXTRA_COMPONENT_DIRS += $(COMMON_COMPONENT_PATH)

include $(IDF_PATH)/make/project.mk

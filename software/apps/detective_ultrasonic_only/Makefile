# nRF application makefile
PROJECT_NAME = $(shell basename "$(realpath ./)")

# Configurations
NRF_IC = nrf52832
SDK_VERSION = 15
SOFTDEVICE_MODEL = s132

# Source and header files
APP_HEADER_PATHS += .
APP_SOURCE_PATHS += .
APP_SOURCES = $(notdir $(wildcard ./*.c))

APP_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
APP_SOURCE_PATHS += $(wildcard $(APP_DIR)/../../libraries/*/)
APP_HEADER_PATHS += $(wildcard $(APP_DIR)/../../libraries/*/)
APP_SOURCES += $(notdir $(wildcard $(APP_DIR)/../../libraries/*/*.c))

# Path to base of nRF52-base repo
NRF_BASE_DIR = ../../buckler/software/nrf5x-base/

# Include board Makefile (if any)
include ../../buckler/software/boards/buckler_revB/Board.mk

# Include main Makefile
include $(NRF_BASE_DIR)make/AppMakefile.mk


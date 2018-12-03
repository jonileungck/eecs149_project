# Board-specific configurations for the Berkeley Buckler

# Ensure that this file is only included once
# ifndef BOARD_MAKEFILE
# BOARD_MAKEFILE = 1

# Board-specific configurations
# BOARD = Buckler_revB
# USE_BLE = 1

# Get directory of this makefile
LIB_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

# Include any files in this directory in the build process
LIB_SOURCE_PATHS = $(LIB_DIR)/.
LIB_SOURCE_PATHS += $(wildcard $(LIB_DIR)/*/)
LIB_HEADER_PATHS = $(LIB_DIR)/.
LIB_HEADER_PATHS += $(wildcard $(LIB_DIR)/*/)
LIB_LINKER_PATHS = $(LIB_DIR)/.
LIB_SOURCES = $(notdir $(wildcard $(LIB_DIR)/./*.c))
LIB_SOURCES += $(notdir $(wildcard $(LIB_DIR)/*/*.c))
BOARD_AS = $(notdir $(wildcard $(LIB_DIR)/./*.s))



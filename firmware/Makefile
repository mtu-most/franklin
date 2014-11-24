MONITOR_PORT = /dev/ttyACM* /dev/ttyUSB*
RESET_CMD=:

CPPFLAGS = -g -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2 -Wshadow -fno-strict-aliasing
EXTRA_FLAGS = --param=ssp-buffer-size=4
OPTIMIZATION_LEVEL = 5
CPPFLAGS += -DARCH_INCLUDE=\"arch-avr.h\"

ARDUINO_LIBS = EEPROM

ifeq (${TARGET}, mini)
BOARD_TAG = atmega328
CPPFLAGS += -DDYNAMIC_MEM_SIZE=0x100
else
ifeq (${TARGET}, melzi0)
BOARD_TAG = mighty_opt
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
BOARDS_TXT = /usr/share/arduino/hardware/mighty-1284p/boards.txt
CPPFLAGS += -DDYNAMIC_MEM_SIZE=0x600
else
ifeq (${TARGET}, melzi1)
BOARD_TAG = mighty_opt
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
BOARDS_TXT = /usr/share/arduino/hardware/mighty-1284p/boards.txt
CPPFLAGS += -DUSE_SERIAL1
CPPFLAGS += -DDYNAMIC_MEM_SIZE=0x600
else
ifeq (${TARGET}, mega)
BOARD_TAG = mega
CPPFLAGS += -DRAMSTART=0x200
CPPFLAGS += -DDYNAMIC_MEM_SIZE=0x600
else
ifeq (${TARGET}, ramps)
BOARD_TAG = mega2560
CPPFLAGS += -DRAMSTART=0x200
CPPFLAGS += -DDYNAMIC_MEM_SIZE=0x600
else
$(error Invalid TARGET)
endif
endif
endif
endif
endif
include /usr/share/arduino/Arduino.mk

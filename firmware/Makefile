MONITOR_PORT = /dev/ttyACM* /dev/ttyUSB*
RESET_CMD=:

CPPFLAGS = -g -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2 -Wshadow -fno-strict-aliasing
EXTRA_FLAGS = --param=ssp-buffer-size=4
OPTIMIZATION_LEVEL = 5
CPPFLAGS += -DARCH_INCLUDE=\"arch-avr.h\"

ARDUINO_LIBS = EEPROM

ifeq (${TARGET}, mini)
BOARD_TAG = atmega328
CPPFLAGS += -DNUM_MOTORS=4 -DNUM_BUFFERS=4 -DFRAGMENTS_PER_BUFFER=16 -DBYTES_PER_FRAGMENT=32
else
ifeq (${TARGET}, melzi0)
BOARD_TAG = mighty_opt
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
BOARDS_TXT = /usr/share/arduino/hardware/mighty-1284p/boards.txt
CPPFLAGS += -DNUM_MOTORS=6 -DNUM_BUFFERS=6 -DFRAGMENTS_PER_BUFFER=32 -DBYTES_PER_FRAGMENT=32
else
ifeq (${TARGET}, melzi1)
BOARD_TAG = mighty_opt
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
BOARDS_TXT = /usr/share/arduino/hardware/mighty-1284p/boards.txt
CPPFLAGS += -DUSE_SERIAL1
CPPFLAGS += -DNUM_MOTORS=6 -DNUM_BUFFERS=6 -DFRAGMENTS_PER_BUFFER=32 -DBYTES_PER_FRAGMENT=32
else
ifeq (${TARGET}, mega)
BOARD_TAG = mega
CPPFLAGS += -DNUM_MOTORS=6 -DNUM_BUFFERS=6 -DFRAGMENTS_PER_BUFFER=32 -DBYTES_PER_FRAGMENT=32
else
ifeq (${TARGET}, ramps)
BOARD_TAG = mega2560
CPPFLAGS += -DNUM_MOTORS=6 -DNUM_BUFFERS=6 -DFRAGMENTS_PER_BUFFER=32 -DBYTES_PER_FRAGMENT=32
else
$(error Invalid TARGET)
endif
endif
endif
endif
endif
include /usr/share/arduino/Arduino.mk

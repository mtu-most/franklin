MELZI=true
#MINI=true

MONITOR_PORT = /dev/ttyACM* /dev/ttyUSB*

EXTRA_FLAGS = -g -Os --param=ssp-buffer-size=4 -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2

ifdef MINI
BOARD_TAG = atmega328
include /usr/share/arduino/Arduino.mk
else
ifdef MELZI
BOARD_TAG = mighty_opt
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
BOARDS_TXT = /usr/share/arduino/hardware/mighty-1284p/boards.txt
include Arduino.mk
else
BOARD_TAG = mega2560
include Arduino.mk
#include /usr/share/arduino/Arduino.mk
endif
endif

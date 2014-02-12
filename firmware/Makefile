#MELZI=true

MONITOR_PORT = /dev/ttyACM* /dev/ttyUSB*

EXTRA_FLAGS = -g -O2 --param=ssp-buffer-size=4 -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2

ifdef MELZI
BOARD_TAG = mighty_opt
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
BOARDS_TXT = /usr/share/arduino/hardware/mighty-1284p/boards.txt
include Arduino.mk
else
BOARD_TAG = mega2560
include /usr/share/arduino/Arduino.mk
endif

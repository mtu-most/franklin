MELZI=true
ifdef MELZI
BOARD_TAG = mighty_opt
ARDUINO_CORE_PATH = /usr/share/arduino/hardware/mighty-1284p/cores/standard
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
PARSE_BOARD_OPTS = --boards_txt=/usr/share/arduino/hardware/mighty-1284p/boards.txt
else
BOARD_TAG = mega2560
endif

EXTRA_FLAGS = -g -O2 --param=ssp-buffer-size=4 -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2
#include /usr/share/arduino/Arduino.mk
include Arduino.mk

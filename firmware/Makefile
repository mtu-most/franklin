#BOARD_TAG = mighty
#PARSE_BOARD_OPTS = --boards_txt=/usr/share/arduino/hardware/mighty-1284p/boards.txt
BOARD_TAG = mega2560

include /usr/share/arduino/Arduino.mk

%.gui: %.gui.in
	xmlgen < $< > $@

#PI=true
MELZI=true
#MEGA=true
#MINI=true

MONITOR_PORT = /dev/ttyACM* /dev/ttyUSB*

CPPFLAGS = -g -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2
SOURCES = axis.cpp debug.cpp extruder.cpp gpio.cpp motor.cpp move.cpp packet.cpp serial.cpp settings.cpp setup.cpp storage.cpp temp.cpp
HEADERS = configuration.h firmware.h

ifdef PI
HEADERS += arch-pi.h
CPPFLAGS += -DARCH_INCLUDE=\"arch-pi.h\"
OBJECTS = $(patsubst %.cpp,%.o,$(SOURCES))
franklin: $(OBJECTS) Makefile
	g++ $(LDFLAGS) $(OBJECTS) -o $@
%.o: %.cpp $(HEADERS) Makefile
	g++ $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

else

EXTRA_FLAGS = --param=ssp-buffer-size=4
# -Os currently crashes gcc.
OPTIMIZATION_LEVEL = 5
CPPFLAGS += -DARCH_INCLUDE=\"arch-avr.h\"

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
ifdef MEGA
BOARD_TAG = mega
include Arduino.mk
else
BOARD_TAG = mega2560
include Arduino.mk
#include /usr/share/arduino/Arduino.mk
endif
endif
endif
endif

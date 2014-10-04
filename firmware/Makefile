MONITOR_PORT = /dev/ttyACM* /dev/ttyUSB*

CPPFLAGS = -g -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2 -Wshadow
SOURCES = debug.cpp globals.cpp gpio.cpp move.cpp packet.cpp serial.cpp setup.cpp space.cpp storage.cpp temp.cpp type-carthesian.cpp type-delta.cpp base.cpp
HEADERS = configuration.h firmware.h

ifeq (${TARGET}, bbb)
HEADERS += arch-bbb.h
CPPFLAGS += -DARCH_INCLUDE=\"arch-bbb.h\"
OBJECTS = $(patsubst %.cpp,%.o,$(SOURCES))
firmware: $(OBJECTS) Makefile
	g++ $(LDFLAGS) $(OBJECTS) -o $@
%.o: %.cpp $(HEADERS) Makefile
	g++ $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@
base.o: firmware.ino

clean:
	rm -rf build-* $(OBJECTS) firmware

else

EXTRA_FLAGS = --param=ssp-buffer-size=4
# -Os currently crashes gcc.
OPTIMIZATION_LEVEL = 5
CPPFLAGS += -DARCH_INCLUDE=\"arch-avr.h\"

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
#include /usr/share/arduino/Arduino.mk
else
$(error Invalid TARGET)
endif
endif
endif
endif
endif
include Arduino.mk
endif

MONITOR_PORT = /dev/ttyACM* /dev/ttyUSB*
RESET_CMD=:
TARGET ?= melzi
#TARGET ?= ramps

CPPFLAGS = -g -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2 -Wshadow -fno-strict-aliasing -Wall -Wextra -Werror
EXTRA_FLAGS = --param=ssp-buffer-size=4

ifeq (${TARGET}, sim)
SOURCES = arch-avr.h arch-sim.h firmware.h firmware.ino packet.cpp serial.cpp setup.cpp timer.cpp
CPPFLAGS += -DARCH_INCLUDE=\"arch-sim.h\" -DBBB
CPPFLAGS += -DNUM_MOTORS=5 -DFRAGMENTS_PER_MOTOR_BITS=3 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=9
build-sim/sim.elf: $(patsubst %.cpp,build-sim/%.o,$(filter %.cpp,$(SOURCES) firmware.cpp))
	g++ $(CPPFLAGS) $(LDFLAGS) $^ -o $@ $(LIBS)
build-sim/%.o: %.cpp $(filter %.h,$(SOURCES)) Makefile build-sim/stamp
	g++ $(CPPFLAGS) -c $< -o $@
build-sim/%.o: %.ino $(filter %.h,$(SOURCES)) Makefile build-sim/stamp
	g++ $(CPPFLAGS) -xc++ -c $< -o $@
build-sim/stamp:
	mkdir -p build-sim
	touch $@
else

OPTIMIZATION_LEVEL = 1
CPPFLAGS += -DARCH_INCLUDE=\"arch-avr.h\" -DWATCHDOG
ARDUINO_LIBS = EEPROM

# To avoid loading HardwareSerial.cpp, disable the core.
NO_CORE = true
# All these are for some reason not parsed when NO_CORE is set.
VARIANT = $(call PARSE_BOARD,$(BOARD_TAG),build.variant)
MCU   = $(call PARSE_BOARD,$(BOARD_TAG),build.mcu)
F_CPU = $(call PARSE_BOARD,$(BOARD_TAG),build.f_cpu)
AVRDUDE_ARD_PROGRAMMER = $(call PARSE_BOARD,$(BOARD_TAG),upload.protocol)
AVRDUDE_ARD_BAUDRATE = $(call PARSE_BOARD,$(BOARD_TAG),upload.speed)
ISP_LOCK_FUSE_PRE = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.unlock_bits)
ISP_HIGH_FUSE = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.high_fuses)
ISP_LOW_FUSE = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.low_fuses)
ISP_EXT_FUSE = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.extended_fuses)
BOOTLOADER_PATH = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.path)
BOOTLOADER_FILE = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.file)
ISP_LOCK_FUSE_POST = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.lock_bits)
HEX_MAXIMUM_SIZE  = $(call PARSE_BOARD,$(BOARD_TAG),upload.maximum_size)
# Avoid including Arduino.h
ARDUINO_HEADER = firmware.h

ifeq (${TARGET}, mini)
BOARD_TAG = uno
CPPFLAGS += -DNUM_MOTORS=7 -DFRAGMENTS_PER_MOTOR_BITS=2 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=7
CPPFLAGS += -DUSART0_RX_vect=USART_RX_vect -DNO_DEBUG
else
ifeq (${TARGET}, melzi)
BOARD_TAG = mighty_opt
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
BOARDS_TXT = /usr/share/arduino/hardware/mighty-1284p/boards.txt
CPPFLAGS += -DNUM_MOTORS=6 -DFRAGMENTS_PER_MOTOR_BITS=5 -DBYTES_PER_FRAGMENT=32 -DSERIAL_SIZE_BITS=11
else
ifeq (${TARGET}, mega)
BOARD_TAG = mega
CPPFLAGS += -DNUM_MOTORS=6 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=10
else
ifeq (${TARGET}, ramps)
BOARD_TAG = mega2560
CPPFLAGS += -DNUM_MOTORS=6 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=10
else
$(error Invalid TARGET)
endif
endif
endif
endif
include /usr/share/arduino/Arduino.mk
endif

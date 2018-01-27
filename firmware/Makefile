# Makefile - build rules for Franklin
# vim: set foldmethod=marker :
# Copyright 2014-2016 Michigan Technological University
# Copyright 2016 Bas Wijnen <wijnen@debian.org>
# Copyright 2017 Lorin Edwin Parker <lorin.parker@hive13.org>
# Author: Bas Wijnen <wijnen@debian.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

MONITOR_PORT ?= /dev/ttyACM* /dev/ttyUSB*
RESET_CMD=:
TARGET ?= atmega1284p
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
AVRDUDE_CONF = /etc/avrdude.conf

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

ifeq (${TARGET}, atmega328p)
BOARD_TAG = uno
CPPFLAGS += -DNUM_MOTORS=3 -DFRAGMENTS_PER_MOTOR_BITS=3 -DBYTES_PER_FRAGMENT=8 -DSERIAL_SIZE_BITS=9
CPPFLAGS += -DUSART0_RX_vect=USART_RX_vect -DNO_DEBUG
AVRDUDE_OPTS = -q -q -D

else
ifeq (${TARGET}, atmega1284p)
BOARD_TAG = mighty_opt
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
BOARDS_TXT = /usr/share/arduino/hardware/mighty-1284p/boards.txt
CPPFLAGS += -DNUM_MOTORS=10 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=10
AVRDUDE_OPTS = -q -q -D

else
ifeq (${TARGET}, atmega1280)
BOARD_TAG = mega
CPPFLAGS += -DNUM_MOTORS=10 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=10
AVRDUDE_OPTS = -q -q -D

else
ifeq (${TARGET}, atmega2560)
BOARD_TAG = mega2560
CPPFLAGS += -DNUM_MOTORS=7 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=10
AVRDUDE_OPTS = -q -q -D

else
ifeq (${TARGET}, atmega1281)
BOARD_TAG = 1281
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/MegaCore/variants
BOARDS_TXT = /usr/share/arduino/hardware/MegaCore/boards.txt
CPPFLAGS += -DNUM_MOTORS=7 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=10
AVRDUDE_OPTS = -q -q -D

else
ifeq (${TARGET}, atmega644p)
BOARD_TAG = atmega644p
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/sanguino/variants
BOARDS_TXT = /usr/share/arduino/hardware/sanguino/boards.txt
CPPFLAGS += -DNUM_MOTORS=4 -DFRAGMENTS_PER_MOTOR_BITS=3 -DBYTES_PER_FRAGMENT=8 -DSERIAL_SIZE_BITS=9 -DNO_DEBUG
AVRDUDE_OPTS = -q -q -D

else
ifeq (${TARGET}, atmega168)
BOARD_TAG = nano
CPPFLAGS += -DNUM_MOTORS=1 -DFRAGMENTS_PER_MOTOR_BITS=1 -DBYTES_PER_FRAGMENT=4 -DSERIAL_SIZE_BITS=5
CPPFLAGS += -DUSART0_RX_vect=USART_RX_vect -DNO_DEBUG
AVRDUDE_OPTS = -q -q -D

else
$(error Invalid TARGET)
endif
endif
endif
endif
endif
endif
endif
include /usr/share/arduino/Arduino.mk
endif

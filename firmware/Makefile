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

MONITOR_PORT ?= $(firstword $(wildcard /dev/ttyACM* /dev/ttyUSB*))
RESET_CMD=:
TARGET ?= atmega1284p
#TARGET ?= ramps

CPPFLAGS = -g -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2 -Wshadow -fno-strict-aliasing -Wall -Wextra -Werror
EXTRA_FLAGS = --param=ssp-buffer-size=4

ifeq (${TARGET}, sim)
SOURCES = arch-avr.h arch-sim.h firmware.h firmware.cpp packet.cpp serial.cpp setup.cpp timer.cpp
CPPFLAGS += -DARCH_INCLUDE=\"arch-sim.h\" -DBBB
CPPFLAGS += -DNUM_MOTORS=5 -DFRAGMENTS_PER_MOTOR_BITS=3 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=9
build-sim/sim.elf: $(patsubst %.cpp,build-sim/%.o,$(filter %.cpp,$(SOURCES) firmware.cpp))
	g++ $(CPPFLAGS) $(LDFLAGS) $^ -o $@ $(LIBS)
build-sim/%.o: %.cpp $(filter %.h,$(SOURCES)) Makefile build-sim/stamp
	g++ $(CPPFLAGS) -c $< -o $@
build-sim/%.o: %.cpp $(filter %.h,$(SOURCES)) Makefile build-sim/stamp
	g++ $(CPPFLAGS) -xc++ -c $< -o $@
build-sim/stamp:
	mkdir -p build-sim
	touch $@
else

OPTIMIZATION_LEVEL = 1
CPPFLAGS += -DARCH_INCLUDE=\"arch-avr.h\" -DWATCHDOG
ARDUINO_LIBS = EEPROM
AVRDUDE_CONF = /etc/avrdude.conf

ifeq (${TARGET}, atmega328p)
BOARD_TAG = uno
CPPFLAGS += -DNUM_MOTORS=2 -DFRAGMENTS_PER_MOTOR_BITS=3 -DBYTES_PER_FRAGMENT=4 -DSERIAL_SIZE_BITS=8
CPPFLAGS += -DUSART0_RX_vect=USART_RX_vect -DNO_DEBUG
AVRDUDE_OPTS = -q -q -D

else
ifeq (${TARGET}, atmega1284p)
BOARD_TAG = mighty_opt
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
CPPFLAGS += -DNUM_MOTORS=10 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=10
AVRDUDE_OPTS = -q -q -D

else
ifeq (${TARGET}, atmega1284p-12MHz)
BOARD_TAG = mighty_opt
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/mighty-1284p/variants
CPPFLAGS += -DNUM_MOTORS=10 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=10
AVRDUDE_OPTS = -q -q -D
F_CPU = 12000000
OBJDIR = build-atmega1284p-12MHz

else
ifeq (${TARGET}, atmega1280)
BOARD_TAG = mega
CPPFLAGS += -DNUM_MOTORS=8 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=8
AVRDUDE_OPTS = -q -q -D

else
#ifeq (${TARGET}, atmega2560)
#BOARD_TAG = mega2560
#CPPFLAGS += -DNUM_MOTORS=8 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=8
#AVRDUDE_OPTS = -q -q -D

#else
ifeq (${TARGET}, atmega1281)
BOARD_TAG = 1281
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/MegaCore/variants
CPPFLAGS += -DNUM_MOTORS=8 -DFRAGMENTS_PER_MOTOR_BITS=4 -DBYTES_PER_FRAGMENT=16 -DSERIAL_SIZE_BITS=8
AVRDUDE_OPTS = -q -q -D

else
ifeq (${TARGET}, atmega644p)
BOARD_TAG = atmega644p
ARDUINO_VAR_PATH = /usr/share/arduino/hardware/sanguino/variants
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
#endif
endif
endif
endif

# This used to include Arduino.mk, but that doesn't work with the new arduino files.
# So it's reimplemented here (with less functionality).

ALL_BOARDS = $(wildcard /usr/share/arduino/hardware/*/boards.txt /usr/share/arduino/hardware/*/*/boards.txt)
BOARDSDIR = $(patsubst %/boards.txt,%,$(firstword $(shell grep -E '^${BOARD_TAG}(\.menu\..*)?\.build\.mcu=' ${ALL_BOARDS} | cut -f1 -d:)))
get = $(firstword $(shell grep -E '^${BOARD_TAG}(\.menu\..*)?\.${1}=' ${BOARDSDIR}/boards.txt | cut -f2 -d=))

MCU ?= $(call get,build.mcu)
F_CPU ?= $(call get,build.f_cpu)
VARIANT ?= $(call get,build.variant)
SPEED ?= $(call get,upload.speed)
PROTOCOL ?= $(call get,upload.protocol)
PINDIR ?= ${BOARDSDIR}/variants/${VARIANT}
OBJDIR ?= build-${MCU}

ALL_SOURCES = arch-avr.h firmware.h packet.cpp serial.cpp setup.cpp timer.cpp firmware.cpp
SOURCES = $(filter %.cpp,${ALL_SOURCES})
OTHER_SOURCES = $(filter-out %.cpp,${ALL_SOURCES})
OBJS = $(patsubst %.cpp,${OBJDIR}/%.o,${SOURCES})

all: ${OBJDIR}/${MCU}.hex

print:
	echo ${BOARDSDIR} ${BOARD_TAG} ${OBJDIR}

clean:
	rm -rf ${OBJDIR}/

upload: ${OBJDIR}/${MCU}.hex Makefile
	avrdude ${AVRDUDE_OPTS} -c ${PROTOCOL} -b ${SPEED} -p ${MCU} -P ${MONITOR_PORT} -U flash:w:$<:i

${OBJDIR}/%.hex: ${OBJDIR}/%.elf ${OBJDIR}/%.sizeok Makefile
	avr-objcopy -O ihex -R .eeprom $< $@
${OBJDIR}/${MCU}.elf: ${OBJS} Makefile
	avr-g++ -mmcu=${MCU} ${LDFLAGS} ${OBJS} ${LDADD} -o $@
	avr-size --format=avr --mcu=${MCU} $@
${OBJDIR}/%.o: %.cpp ${OTHER_SOURCES} ${OBJDIR}/stamp
	avr-g++ -mmcu=${MCU} -I ${PINDIR} -DF_CPU=${F_CPU} ${CPPFLAGS} ${CXXFLAGS} $< -o $@ -c
${OBJDIR}/stamp:
	mkdir -p ${OBJDIR}
	touch $@
${OBJDIR}/%.sizeok: ${OBJDIR}/%.elf Makefile
	[ "`avr-size --format=avr --mcu=${MCU} $< | grep -E '\d\d\d(\.\d*)?%'`" = "" ]
	touch $@

.PHONY: all clean upload
.PRECIOUS: ${OBJDIR}/stamp ${OBJDIR}/${MCU}.sizeok
endif

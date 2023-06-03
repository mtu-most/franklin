# Makefile - build rules for Franklin
# Copyright and license {{{
# Copyright 2014-2016 Michigan Technological University
# Copyright 2016-2022 Bas Wijnen <wijnen@debian.org>
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
# }}}

ifndef MCU
MCU = atmega1284p
#MCU = atmega168
endif

ifeq (${MCU}, atmega168) # {{{
F_CPU ?= 16000000
PROTOCOL ?= arduino
BAUD ?= 19200

EXTRA_CPPFLAGS ?= -DNO_DEBUG -DNO_ADC -DNO_FOLLOWER -DNO_PIN_MOTOR -DNO_TEMP_HOLD -DNO_TEMP_LIMIT -DNO_SPI -DNO_PATTERN -DNO_TIMEOUT -DNO_LED -DNO_PWM
NUM_MOTORS ?= 1
FRAGMENTS_PER_MOTOR_BITS ?= 1
BYTES_PER_FRAGMENT ?= 2
SERIAL_SIZE_BITS ?= 7	# Minimum: 7 (with minimal fragment size)
endif # }}}

ifeq (${MCU}, atmega328p) # {{{
F_CPU ?= 16000000
PROTOCOL ?= arduino
BAUD ?= 57600

EXTRA_CPPFLAGS ?= -DNO_DEBUG -DNO_ADC -DNO_FOLLOWER -DNO_PIN_MOTOR -DNO_TEMP_HOLD -DNO_TEMP_LIMIT -DNO_SPI -DNO_PATTERN -DNO_TIMEOUT -DNO_LED -DNO_PWM
NUM_MOTORS ?= 1
FRAGMENTS_PER_MOTOR_BITS ?= 1
BYTES_PER_FRAGMENT ?= 2
SERIAL_SIZE_BITS ?= 7	# Minimum: 7 (with minimal fragment size)
endif # }}}

ifeq (${MCU}, atmega1284p) # {{{
F_CPU ?= 16000000
PROTOCOL ?= arduino
BAUD ?= 115200

NUM_MOTORS ?= 8
FRAGMENTS_PER_MOTOR_BITS ?= 4
BYTES_PER_FRAGMENT ?= 16
SERIAL_SIZE_BITS ?= 10	# Minimum: 7 (with minimal fragment size)
endif # }}}

ifeq (${MCU}, atmega2560) # {{{
F_CPU ?= 16000000
PROTOCOL ?= wiring
BAUD ?= 115200

NUM_MOTORS ?= 6
FRAGMENTS_PER_MOTOR_BITS ?= 4
BYTES_PER_FRAGMENT ?= 16
SERIAL_SIZE_BITS ?= 9	# Minimum: 7 (with minimal fragment size)
endif # }}}

TARGET_ARCH ?= avr

#AVRDUDE_CONF = /etc/avrdude.conf
#AVRDUDE_OPTS = -q -q -D

EXTRA_CPPFLAGS ?=
EXTRA_CPPFLAGS += \
	-DWATCHDOG \
	-DNUM_MOTORS=${NUM_MOTORS} \
	-DFRAGMENTS_PER_MOTOR_BITS=${FRAGMENTS_PER_MOTOR_BITS} \
	-DBYTES_PER_FRAGMENT=${BYTES_PER_FRAGMENT} \
	-DSERIAL_SIZE_BITS=${SERIAL_SIZE_BITS} \
	-I../arch/${TARGET_ARCH}

TARGETS = franklin

franklin_SOURCES = \
	      arch/${TARGET_ARCH}/arch-firmware-defs.h \
	      arch/${TARGET_ARCH}/arch-firmware.h \
	      arch/${TARGET_ARCH}/arch-firmware.cc \
	      firmware.h \
	      packet.cc \
	      serial.cc \
	      setup.cc \
	      timer.cc \
	      firmware.cc

include amat.mk

# vim: set foldmethod=marker :

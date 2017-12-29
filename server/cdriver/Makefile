# Makefile - cdriver build rules for Franklin
# Copyright 2014-2016 Michigan Technological University
# Copyright 2016 Bas Wijnen <wijnen@debian.org>
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

#TARGET ?= orangepizero

PROFILE = #-pg

CPPFLAGS ?= -ggdb3 -O3 -Wall -Wextra -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2 -Wshadow $(PROFILE)
LDFLAGS ?= $(PROFILE)

all: franklin-cdriver

ifeq (${TARGET}, orangepizero)
ARCH_HEADER = arch-multicore.h
CPPFLAGS += -DORANGEPIZERO
else
ifeq (${TARGET}, pine64)
ARCH_HEADER = arch-multicore.h
CPPFLAGS += -DPINE64
else
ARCH_HEADER = arch-avr.h
endif
endif

SOURCES = \
	base.cpp \
	debug.cpp \
	globals.cpp \
	gpio.cpp \
	hostserial.cpp \
	move.cpp \
	packet.cpp \
	parse.cpp \
	run.cpp \
	serial.cpp \
	setup.cpp \
	space.cpp \
	storage.cpp \
	temp.cpp \
	type-cartesian.cpp \
	type-delta.cpp \
	type-polar.cpp

HEADERS = \
	configuration.h \
	cdriver.h \
	${ARCH_HEADER}

CPPFLAGS += -DARCH_INCLUDE=\"${ARCH_HEADER}\" -std=c++0x

OBJECTS = $(addprefix build/,$(patsubst %.cpp,%.o,$(SOURCES)))

franklin-cdriver: $(OBJECTS) Makefile
	g++ $(LDFLAGS) $(OBJECTS) $(LIBS) -o $@

build/stamp:
	mkdir -p build
	touch $@

build/%.o: %.cpp $(HEADERS) build/stamp Makefile
	g++ $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJECTS) build franklin-cdriver $(DTBO)

# Makefile - cdriver build rules for Franklin
# Copyright 2018 Bas Wijnen <wijnen@debian.org>
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

# The values after ?= are defaults; the ones after += are required.
CPPFLAGS ?= -Wall -Wextra -Wformat -Werror -Wdate-time -D_FORTIFY_SOURCE=2
CXXFLAGS ?= -ggdb3 -O3 -fstack-protector-strong
CXXFLAGS += -fPIC -rdynamic
LDFLAGS ?= -Wall -Wextra -Wformat -Werror ${CXXFLAGS}
LDFLAGS += -export-dynamic -fPIC -rdynamic
LIBS ?=
LIBS += -ldl
TARGET_ARCH ?= avr

CPPFLAGS += -Iarch/${TARGET_ARCH} -I.

all: module/build/stamp franklin-cdriver

module/build/stamp: module/module.cpp module/module.h module/parse.cpp module/setup.py Makefile
	cd module && python3 setup.py build
	touch $@

SOURCES = \
	arch/${TARGET_ARCH}/arch-host.cpp \
	base.cpp \
	debug.cpp \
	globals.cpp \
	gpio.cpp \
	move.cpp \
	packet.cpp \
	run.cpp \
	serial.cpp \
	setup.cpp \
	space.cpp \
	temp.cpp

OBJS = $(patsubst %.cpp,build/%.o,${SOURCES})

HEADERS = \
	arch/${TARGET_ARCH}/arch-host.h \
	cdriver.h \
	configuration.h \
	module/module.h

DEPENDS = \
	  Makefile

franklin-cdriver: ${OBJS} ${HEADERS} ${DEPENDS}
	g++ ${LDFLAGS} ${OBJS} -o $@ ${LIBS}

build/%.o: %.cpp ${HEADERS} ${DEPENDS}
	mkdir -p $(dir $@)
	g++ -std=c++11 -c ${CPPFLAGS} ${CXXFLAGS} $< -o $@

clean:
	rm -rf build module/build franklin-cdriver

.PHONY: clean

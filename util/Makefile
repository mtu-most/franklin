#!/usr/bin/python3
# vim: set foldmethod=marker :
# Makefile - build rules for Franklin
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

all: joystick.py

%.py: %.elf
	./$< > $@

%.elf: %.c
	gcc -Wall -Wextra -Werror $< -o $@

%.gui: %.gui.in
	xmlgen <$< >$@

install: joystick.py
	install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/ joystick.py
	install -D -t ${DESTDIR}/usr/lib/franklin/ franklin_joystick.py

clean:
	rm -rf __pycache__ joystick.py c457a-ui.gui *.pyc

# Makefile - test rules for Franklin
# Copyright 2014 Michigan Technological University
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

# Support {foo,bar} notation.
SHELL=/bin/bash

all: links check.stamp index.html

check.stamp: server.js main.js templates.js ui.js
	! which gjslint || gjslint --disable 5,110,220 $^
	touch $@

links:
	rm -f rpc.js builders.js
	ln -s /usr/share/python3-websocketd/rpc.js .
	ln -s /usr/share/python3-websocketd/builders.js .

clean:
	rm -f rpc.js builders.js index.html check.stamp

files := $(wildcard type/*.js)
header := $(foreach file,${files},<script type="application/javascript" src="${file}"></script>)

index.html: index.html.template
	sed 's#<!-- TYPE-INCLUDES -->#${header}#' < $< > $@

install: index.html
	install -m 644 -t $(DESTDIR)/usr/lib/franklin/html/ -D index.html {main,server,templates,ui}.js {style,ui}.css

.PHONY: index.html

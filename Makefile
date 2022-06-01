# Makefile - build rules for Franklin
# Copyright 2015 Michigan Technological University
# Copyright 2016-2022 Bas Wijnen
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

# This Makefile has 3 goals:
# 1. Provide "make install" for Debian package build.
# 2. Build and install Debian package through "make".
# 3. Build code for non-Debian systems through "make franklin".

# Note that "make franklin" does not run the service.

# Default rule.
package:

# Dependencies to be installed.
rundeps = python3-serial, avrdude, adduser, lsb-base, apache2
franklindeps = debhelper, python3-all, dh-python, gcc-avr, arduino-mighty-1284p, arduino-mk, python3-all-dev, python3-network, python3-websocketd, python3-fhs
depends:
	depends=`dpkg-checkbuilddeps 2>&1 -d 'devscripts, git, wget, sudo, fakeroot, ${rundeps}, ${franklindeps}'`; depends="$${depends##*:}"; if [ "$${depends}" ]; then echo "Installing $$depends"; sudo apt install $$depends ; fi
	dpkg-checkbuilddeps

mkdeb:
	# Get mkdeb to build Debian package.
	wget https://people.debian.org/~wijnen/mkdeb
	chmod a+x mkdeb

package: depends mkdeb
	# Build and install the Debian package.
	git pull
	sudo a2enmod proxy proxy_html proxy_http proxy_wstunnel
	./mkdeb $(MKDEB_ARG)
	sudo dpkg -i $(wildcard /tmp/franklin*.deb)

install:
	# Note: This target is called by debhelper to install files into the Debian package.
	make -C server install
	make -C util install

franklin:
	# Compile everything, but don't install or set up a service.
	pip install -r requirements.txt
	cd server/html && rm -f builders.js rpc.js && python3 -m websocketd copy
	make -C server
	make -C util

clean:
	# This target is used by the Debian package, but can also be called manually.
	rm -f mkdeb
	make -C firmware clean
	make -C server clean
	make -C util clean

.PHONY: install-all install build clean depends

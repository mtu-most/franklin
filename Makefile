# Makefile - build rules for Franklin
# Copyright 2015 Michigan Technological University
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

MODULES = python-fhs python-network python-websocketd

install-all: build
	sudo dpkg -i $(wildcard $(patsubst %,/tmp/python3-%*.deb,$(subst python-,,$(MODULES))) /tmp/franklin*.deb)

# This target is meant for producing release packages for Athena.
# Some preparations are required to make ti work:
# - A beaglebone must be attached to a USB port, and accessible on the address given below.
# - It must have a user with password as given below.
# - All build dependencies must be installed on it. (Note that some come from backports or jessie (or newer).)
# - DEBFULLNAME and DEBEMAIL should be set.
# - On the host system, FRANKLIN_PASSPHRASE must be set and the secret key given below must be available.
BB ?= debian@192.168.7.2
BB_PASS ?= reprap
UPGRADE_KEY ?= 46BEB154
SSHPASS ?= sshpass -p'${BB_PASS}'
zip: armhf
	cd zipdir && for f in * ; do echo "$$FRANKLIN_PASSPHRASE" | gpg --local-user $(UPGRADE_KEY) --passphrase-fd 0 --sign $$f ; rm $$f ; done
	changelog="`dpkg-parsechangelog`" && name="`echo "$$changelog" | grep '^Source: ' | cut -b9-`" && fullversion="`echo "$$changelog" | grep '^Version: ' | cut -b10-`" && version="$${fullversion%-*}" && rm -f $$name-$$version.zip && cd zipdir && zip ../$$name-$$version.zip *

armhf:
	test ! -d zipdir || rm -r zipdir
	mkdir zipdir
	$(SSHPASS) ssh $(BB) sudo ip route del default || true
	$(SSHPASS) ssh $(BB) sudo ip route add default via 192.168.7.1
	$(SSHPASS) ssh $(BB) sudo ntpdate -u time.mtu.edu
	$(SSHPASS) ssh $(BB) rm -rf franklin '/tmp/*.{dsc,changes,tar.gz,tar.xz,deb}'
	cd zipdir && git clone .. franklin
	tar cf - -C zipdir franklin | $(SSHPASS) ssh $(BB) tar xf -
	rm -rf zipdir/franklin
	$(SSHPASS) ssh $(BB) git -C franklin remote set-url origin https://github.com/mtu-most/franklin
	$(SSHPASS) ssh $(BB) make -C franklin build
	$(SSHPASS) scp $(BB):/tmp/*deb zipdir/
	test ! "$$DINSTALL" -o ! "$$DINSTALL_DIR" -o ! "$$DINSTALL_INCOMING" || $(SSHPASS) scp $(BB):'/tmp/*.{dsc,changes,tar.gz,tar.xz,deb}' "$$DINSTALL_INCOMING" ; cd "$$DINSTALL_DIR" && $$DINSTALL && $$DINSTALL

bb: armhf
	rm -r zipdir

install:
	# Fake target to make debhelper happy.
	:

build: basedeps mkdeb $(addprefix module-,$(MODULES))
	git pull
	git submodule foreach git pull
	git submodule foreach ../mkdeb $(MKDEB_ARG)
	./mkdeb $(MKDEB_ARG)

mkdeb:
	wget https://people.debian.org/~wijnen/mkdeb
	chmod a+x mkdeb

module-%: base = $(patsubst module-%,%,$@)
module-%:
	git submodule add https://github.com/wijnen/$(base)
	touch $@

clean-%: base = $(patsubst clean-%,%,$@)
clean-%:
	git submodule deinit -f $(base) || :
	rm -rf $(base) || :
	git rm -f $(base) || :
	rm -rf .git/modules/$(base)

clean: $(addprefix clean-,$(MODULES))
	git rm -f .gitmodules || :
	rm -f mkdeb .gitmodules
	rm -rf zipdir
	rm -f $(addprefix module-,$(MODULES))

basedeps:
	dpkg-checkbuilddeps -d 'devscripts, git, wget, sudo'
	dpkg-checkbuilddeps

.PHONY: install build clean zip bb basedeps
	
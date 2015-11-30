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
	sudo dpkg -i $(wildcard $(patsubst %,/tmp/%*.deb,$(MODULES)) /tmp/franklin*.deb)

BB = debian@192.168.7.2
zip:
	rm -rf zipdir
	mkdir zipdir
	sshpass -preprap ssh $(BB) rm -rf franklin
	sshpass -preprap ssh $(BB) mkdir franklin || :
	git archive HEAD | sshpass -preprap ssh $(BB) tar xf - -C franklin
	sshpass -preprap ssh $(BB) make -C franklin build
	sshpass -preprap ssh $(BB) cd /tmp \; aptitude download avrdude
	sshpass -preprap scp $(BB):/tmp/*deb zipdir/
	cd zipdir && aptitude download arduino-mighty-1284p
	for f in avrdude* ; do mv $$f 1-$$f ; done
	for f in python-fhs* ; do mv $$f 1-$$f ; done
	for f in python-network* ; do mv $$f 2-$$f ; done
	for f in python-websocketd* ; do mv $$f 3-$$f ; done
	for f in python-franklin* ; do mv $$f 4-$$f ; done
	# Prepare script.
	echo '#!/bin/sh' > zipdir/0-prepare
	echo 'ip route del default' >> zipdir/0-prepare
	echo 'dpkg --purge repetier-server' >> zipdir/0-prepare
	echo "sed -i -e 's/apt-get update -f/#apt-get install -f/' /etc/rc.local" >> zipdir/0-prepare
	echo 'rm /etc/default/franklin' >> zipdir/0-prepare
	# Finalize script.
	echo '#!/bin/sh' > zipdir/9-finalize
	echo 'cat > /etc/default/franklin <<EOF' >> zipdir/9-finalize
	echo "ATEXIT='sudo shutdown -h now'" >> zipdir/9-finalize
	echo "TLS=False" >> zipdir/9-finalize
	echo 'EOF' >> zipdir/9-finalize
	echo 'shutdown -h now' >> zipdir/9-finalize
	cd zipdir && echo -n 'Passphrase for signing key: ' && read passphrase && for f in * ; do echo "$$passphrase" | gpg --local-user 46BEB154 --passphrase-fd 0 --detach-sign $$f ; done
	cd zipdir && zip ../franklin.zip *

install:
	# Fake target to make debhelper happy.
	:

build: mkdeb $(addprefix module-,$(MODULES))
	./mkdeb

mkdeb:
	wget http://people.debian.org/~wijnen/mkdeb
	chmod a+x mkdeb

module-%: base = $(patsubst module-%,%,$@)
module-%: mkdeb
	git submodule add https://github.com/wijnen/$(base) || git submodule update $(base)
	cd $(base) && ../mkdeb
	touch $@

clean-%: base = $(patsubst clean-%,%,$@)
clean-%:
	git submodule deinit -f $(base) || :
	git rm -f $(base) || :
	rm -rf .git/modules/$(base)
	rm -f module-$(base)

clean: $(addprefix clean-,$(MODULES))
	rm -f mkdeb .gitmodules
	rm -rf zipdir

.PHONY: install build clean zip

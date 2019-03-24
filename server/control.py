#!/usr/bin/python3
# control.py - USB hotplug handling for Franklin
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

import network
import os
import sys

action = os.getenv('ACTION')
dev = os.getenv('DEVNAME')

users = []
if os.path.exists('/run/user'):
	for u in os.listdir('/run/user'):
		try:
			n = int(u)
		except:
			continue
		users.append(n)
	users.sort()
dirs = ['/run'] + ['/run/user/{}'.format(u) for u in users]

for d in dirs:
	try:
		s = network.Socket(d + '/franklin/udev.socket')
		s.sendline('{} {}\n'.format(action, dev))
		break
	except:
		continue
else:
	sys.stderr.write('Failed to handle udev event for Franklin\n')
	sys.exit(1)

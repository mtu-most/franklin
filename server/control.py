#!/usr/bin/python
# control.py - USB hotplug handling for Franklin
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

import websocketd
import os
import sys

port = 8000
tls = True

with open('/etc/default/franklin') as f:
	for l in f.readlines():
		l = l.strip()
		if l == '' or l.startswith('#') or not '=' in l:
			continue
		key, value = l.split('=', 1)
		if key == 'PORT':
			# Leave it as a string because it need not be numerical.
			port = value.strip()
		if key == 'TLS':
			tls = value.lower().strip() == 'true'

try:
	p = websocketd.RPC(port, tls = tls)
	action = os.getenv('ACTION')
	dev = os.getenv('DEVNAME')
	if action == 'add':
		p.add_port(dev)
	elif action == 'remove':
		p.remove_port(dev)
except:
	sys.stderr.write('Failed to handle serial port event for Franklin')

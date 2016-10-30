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

import websocketd
import os
import sys

port = 8000
tls = False
user = None
password = None
current_level = 0

def credentials(level, value):
	global current_level
	if level < current_level:
		return
	current_level = level
	if ':' in value:
		user, password = value.split(':', 1)
	else:
		user = 'admin'
		password = value

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
			tls = value.lower().strip() in ('1', 'true')
		if key == 'USER':
			credentials(0, value.strip())
		if key == 'EXPERT':
			credentials(1, value.strip())
		if key == 'ADMIN':
			credentials(2, value.strip())

try:
	p = websocketd.RPC(port, tls = tls, url = '/admin', user = user, password = password)
	action = os.getenv('ACTION')
	dev = os.getenv('DEVNAME')
	if action == 'add':
		p.add_port(dev)
	elif action == 'remove':
		p.remove_port(dev)
except:
	sys.stderr.write('Failed to handle serial port event for Franklin')

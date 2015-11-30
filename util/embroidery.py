#!/usr/bin/python3
# vim: set foldmethod=marker :
# embroidery.py - embroidery support for Franklin
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
import time
import fhs

config = fhs.init({'port': 8000, 'tls': 'False', 'sensor': 4, 'motor': 3})
motor = int(config['motor'])
sensor = int(config['sensor'])
tls = config['tls'].lower() != 'false'

p = websocketd.RPC(config['port'], tls = tls)

'''
Procedure:
	-> wait for confirm
	motor on
	needle down -> wait for gpio off
	needle up -> wait for gpio on
	motor off
	-> confirm
	move
	repeat
'''

while True:
	id, message = p.wait_confirm()
	msg = message.split()
	if len(msg) != 2 or msg[0] != 'EMBROIDER':
		#websocketd.log('skip %s' % repr(msg))
		continue
	if msg[1] == 'stitch':
		#websocketd.log('do %s' % repr(msg))
		p.set_gpio(motor, state = 1)
		p.wait_gpio(sensor, 0)
		time.sleep(.01)
		p.wait_gpio(sensor, 1)
		p.set_gpio(motor, state = 0)
		p.confirm(None)
		continue
	websocketd.log('unrecognized embroidery command %s' % msg[1])

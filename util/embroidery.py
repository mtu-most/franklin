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

"""Embroidery helper for Franklin
This script facilitates using Franklin as an embroidery machine.
For generating G-Code, use inkscape with the extension at
https://github.com/phidiasllc/inkscape-embroidery
This script should be running while the G-Code is being run.

Required machine setup:
	2 Gpio controls must be created; one for the motor and one for the
	sensor.  By default Gpio 3 and 4 are used respectively (leaving room
	for limit switch monitor Gpios on 0-2).  These defaults can be changed
	by passing --sensor and --motor arguments.

	The sensor pin must be an input which is active when the needle is up
	(so it is safe to make a move).  It has been tested with a magnet and a
	hall effect sensor.

	The motor must be connected to a relay (preferably solid state, because
	it switches a lot) which turns the motor on when active.

	The sewing machine should be set up with very low pressure on the foot,
	so the fabric can move underneath it without lifting the foot.  If
	possible, there should be no transport from below.  The stitch width
	and length should be set to 0.

Make sure that the embroidery loop moves properly over the surface of the
sewing machine.
"""

import websocketd
import time
import fhs

config = fhs.init({'port': '8000', 'tls': 'False', 'motor': 3, 'sensor': 4})
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
	Franklin moves
	repeat
'''

while True:
	# Wait for confirm; pick up pending confirmation request, if any.
	id, message = p.wait_confirm()
	msg = message.split()
	while len(msg) != 2 or msg[0] != 'EMBROIDER':
		# Wrong message; wait for next confirm.
		#websocketd.log('skip %s' % repr(msg))
		id, message = p.wait_confirm(pending = False)
		msg = message.split()
	if msg[1] == 'stitch':
		#websocketd.log('do %s' % repr(msg))
		# Enable motor.
		p.set_gpio(motor, state = 1)
		# Wait for needle to be down.
		p.wait_gpio(sensor, 0)
		time.sleep(.01)
		# Wait for needle to be up.
		p.wait_gpio(sensor, 1)
		# Disable motor.
		p.set_gpio(motor, state = 0)
		time.sleep(.1)
		# Confirm; Franklin moves.
		p.confirm(id)
		# Repeat.
	else:
		websocketd.log('unrecognized embroidery command %s' % msg[1])

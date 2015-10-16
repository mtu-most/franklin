#!/usr/bin/python3

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
	id, message = p.wait_confirm(False)
	msg = message.split()
	if len(msg) != 2 or msg[0] != 'EMBROIDERY':
		continue
	if msg[1] == 'stitch':
		p.set_gpio(motor, state = 1)
		p.wait_gpio(sensor, 0)
		time.sleep(.01)
		p.wait_gpio(sensor, 1)
		p.set_gpio(motor, state = 0)
		p.confirm(None)
		continue
	websocketd.log('unrecognized embroidery command %s' % msg[1])

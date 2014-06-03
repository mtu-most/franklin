#!/usr/bin/python

import websockets
import os

tls = True
port = 8080

with open ('/etc/default/franklin') as f:
	for l in f.readlines ():
		l = l.strip ()
		if l == '' or l.startswith ('#') or not '=' in l:
			continue
		key, value = l.split ('=', 1)
		if key == 'PORT':
			# Leave it as a string because it need not be numerical.
			port = value.strip ()
		if key == 'TLS' and value.strip () == '':
			tls = False

p = websockets.RPC (port, tls = tls)
action = os.getenv ('ACTION')
dev = os.getenv ('DEVNAME')
if action == 'add':
	p.add_port (dev)
elif action == 'remove':
	p.remove_port (dev)

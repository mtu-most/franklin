#!/usr/bin/python

import cgi
import network
import sys
import json

query = cgi.FieldStorage ()

request = query.getfirst ('request', None)
if request is None:
	sys.stdout.write ('Content-Type: text/html;charset=UTF-8\n\n')
	sys.stdout.write (open ('reprap.html').read ())
	sys.exit (0)


sys.stdout.write ('Content-Type: text/plain;charset=UTF-8\n\n')

reprap = network.RPCSocket ('reprap|')

if request == 'move':
	reprap.goto (x = float (query.getfirst ('x')), y = float (query.getfirst ('y')), z = float (query.getfirst ('z')))
	reprap.wait ()
	print ('ok')
if request == 'goto':
	reprap.goto (x = float (query.getfirst ('x')), y = float (query.getfirst ('y')), z = float (query.getfirst ('z')))
	reprap.wait ()
	print ('ok')
elif request == 'home':
	reprap.home (query.getfirst ('z', None) is not None)
	reprap.wait ()
	print ('ok')
elif request == 'sleep':
	reprap.sleep ()
	reprap.wait ()
	print ('ok')
elif request == 'extrude':
	reprap.extrude (float (query.getfirst ('dir')), float (query.getfirst ('extruder', 0)))
	reprap.wait ()
	print ('ok')
elif request == 'feedfactor':
	reprap.feedfactor (float (query.getfirst ('factor')))
	print ('ok')
elif request == 'feedmin':
	reprap.feedmin (float (query.getfirst ('value')))
	print ('ok')
elif request == 'feedmax':
	reprap.feedmax (float (query.getfirst ('value')))
	print ('ok')
elif request == 'flowrate':
	reprap.flowrate (float (query.getfirst ('factor')), float (query.getfirst ('extruder', 0)))
	print ('ok')
elif request == 'bed':
	reprap.bed_temperature (float (query.getfirst ('temperature')))
	reprap.wait ()
	print ('ok')
elif request == 'temperature':
	reprap.temperature (float (query.getfirst ('temperature')), float (query.getfirst ('extruder', 0)))
	reprap.wait ()
	print ('ok')
elif request == 'config':
	print json.dumps (reprap.get_config ())
elif request == 'state':
	print json.dumps (reprap.get_state ())
elif request == 'print':
	reprap.fileprint (query['file'].file.read ())
	print ('ok')
elif request == 'sdprint':
	reprap.sdsend (query['file'].file.read ())
	reprap.sdprint ()
	print ('ok')
elif request == 'stop':
	reprap.stop ()
	print ('ok')
else:
	print ('unknown command')

#!/usr/bin/python3
# vim: foldmethod=marker :
# Documentation. {{{
# server.py - machine multiplexing for Franklin {{{
# Copyright 2014-2016 Michigan Technological University
# Copyright 2016 Bas Wijnen <wijnen@debian.org>
# Copyright 2017 Lorin Edwin Parker <lorin.parker@hive13.org>
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
# }}}

# File documentation. {{{
'''@file
This is the main program.  It runs a WebSockets server and starts driver.py
processes when new machines are detected.  It sends any requests it doesn't
handle itself to those processes.

This file is installed as "franklin" in the executable path.  When run, it
accepts the following options (prefix with "--" on the commandline, or use as
is in a configuration file):
* port: which port to listen on for requests.  Default: 8000
* address: which address to bind to.  If set to empty, it binds the both
  0.0.0.0 and ::1, so it responds to both IPv4 and IPv6 requests.  If IPv6 is
  not supported (this is currently the case on the Raspberry Pi), you need to
  explicitly set it to 0.0.0.0 or the server will not start.  This can also be
  used to listen only on one interface, by setting it to the local address of
  that interface.  Default: ''
* machine: default machine for new client connections.  Leave empty to use the
  first detected machine.  Default: ''
* blacklist: regular expression of serial ports that detection should not be
  attempted on.  This should normally not be used.  add-blacklist should be
  used instead.
* add-blacklist: same as blacklist.  However, add-blacklist has a empty
  default, so it can be used to add ports to the standard blacklist, as opposed
  to replacing it.
* noautodetect: if not set, communication is attempted on newly detected ports.
  This is normally good, but it can be disabled if autodetection prevents
  flashing new firmware, or if it is unclear which ports should be blacklisted
  and Franklin must not interfere with some ports.  Default: True
* predetect: system command that is run before detection is attempted.  The
  substring #PORT# is replaced with the port.  Use this to set up the port.
  Default: ``stty -F #PORT# raw 115200 -echo -echoe -echok -echoke -echonl -echoprt``
* allow-system: regular expression for commands that are allowed to be run from
  G-Code.  Default: '^$'
* admin: Credentials for accessing the /admin interface.  This can either be a
  password, or a username:password pair.  If only a password is supplied, any
  username is accepted with that password.  Default: ''
* expert: Credentials for accessing the /expert interface.  This can either be a
  password, or a username:password pair.  If only a password is supplied, any
  username is accepted with that password.  Default: ''
* user: Credentials for accessing the default interface.  This can either be a
  password, or a username:password pair.  If only a password is supplied, any
  username is accepted with that password.  Default: ''
* done: system command to run after completing a job.  Default: ''
* log: passed on to the websockets server, which uses it to create a log file
  with websockets traffic.
* tls: passed on to the websockets server, which uses it to enable encryption.
  Default: True
'''
# }}}

# Main page documentation. {{{
'''@mainpage
This is the documentation for Franklin that was generated from its source code.
It is meant to help people make changes to the code, and as a reference for
people who write programs that access Franklin with a WebSocket.

For the latter purpose, most of this documentation should be ignored.  Two
classes are useful: Connection and Machine.  Functions from those classes can
be called using RPC requests.  In Machine, functions with a role prefix must be
called by a connections with at least those permissions.  The prefix must not
be part of the RPC request.
'''
# }}}
# }}}

# Imports and config. {{{
import re
import os
import sys
import math
import random
import network
import websocketd
from websocketd import log
import fhs
import subprocess
import crypt
import time
import serial
import json
import traceback
import fcntl
import protocol

fhs.option('port', 'Port to listen on', default = '8000')
fhs.option('address', 'Address to listen on. Set to 0.0.0.0 to force IPv4 only', default = '')
fhs.option('whitelist', 'If set, only allow serial ports that match this regular expression', default = '')
fhs.option('blacklist', 'Disallow serial ports that match this regular expression', default = r'/dev/(input/.*|ptmx|console|tty(printk|(GS)?\d*))$')
fhs.option('add-blacklist', 'Also disallow serial ports that match this regular expression', default = r'^$')
fhs.option('noautodetect', 'Do not autodetect machines on new serial ports', argtype = bool)
fhs.option('predetect', 'Run this command prior to detecting a machine on a serial port', default = 'stty -F $PORT raw 115200 -echo -echoe -echok -echoke -echonl -echoprt')
fhs.option('controller', 'Run this command to handle a controller on a serial port', default = '/usr/lib/franklin/controller.py --dev "$PORT"')
fhs.option('allow-system', 'Only allow system commands that match this regular expression', default = '^$')
fhs.option('admin', 'Admin password', default = '')
fhs.option('expert', 'Expert user password', default = '')
fhs.option('user', 'Local user password', default = '')
fhs.option('remote', 'Remote user password', default = '')
fhs.option('done', 'Run this command when a job is done', default = '')
fhs.option('log', 'Enable logging to a given logfile')
fhs.option('tls', 'Enable TLS. It is recommended to let Apache handle this', argtype = bool)
config = fhs.init(packagename = 'franklin')
# }}}

# Global variables. {{{
httpd = None
ports = {}
autodetect = not config['noautodetect']
tls = config['tls']
machines = {}
log('whitelist: %s' % config['whitelist'])
# }}}

class Server(websocketd.RPChttpd): # {{{
	def auth_message(self, connection, is_websocket):
		path = connection.address.path
		for extra in ('/', '/websocket'):
			if path.endswith(extra):
				path = path[:-len(extra)]
		if path.endswith('/benjamin'):
			connection.data['role'] = 'benjamin'
			escalate = ()
			down = ('admin', 'expert', 'user', 'remote',)
		elif path.endswith('/admin'):
			connection.data['role'] = 'admin'
			escalate = ()
			down = ('expert', 'user', 'remote',)
		elif path.endswith('/expert'):
			connection.data['role'] = 'expert'
			escalate = ('admin',)
			down = ('user', 'remote',)
		elif path.endswith('/user'):
			connection.data['role'] = 'user'
			escalate = ('expert', 'admin',)
			down = ('remote',)
		else:
			connection.data['role'] = 'remote'
			escalate = ('user', 'expert', 'admin',)
			down = ()
		for role in escalate:
			if config[role]:
				break
			connection.data['role'] = role
		connection.data['pwd'] = config[connection.data['role'] if connection.data['role'] != 'benjamin' else 'admin']
		if not connection.data['pwd']:
			for role in down:
				if config[role]:
					connection.data['pwd'] = config[role]
					break
		return 'Please identify yourself for %s access' % connection.data['role'] if connection.data['pwd'] else None
	def authenticate(self, connection):
		if ':' in connection.data['pwd']:
			return [connection.data['user'], connection.data['password']] == connection.data['pwd'].split(':', 1)
		else:
			return connection.data['password'] == connection.data['pwd']
	def page(self, connection):
		if 'machine' in connection.query:
			# Export request.
			machine = connection.query['machine'][0]
			if machine not in machines or not isinstance(machines[machine], Machine):
				self.reply(connection, 404)
			else:
				def export_reply(success, message):
					self.reply(connection, 200, message.encode('utf-8'), 'text/plain;charset=utf-8')
					connection.socket.close()
				machines[machine].call('export_settings', (connection.data['role'],), {}, export_reply)
				return True
		elif connection.address.path.endswith('/adc'):
			filename = '/tmp/franklin-adc-dump'	# FIXME
			if os.path.exists(filename):
				message = open(filename, 'rb').read()
				os.unlink(filename)
			else:
				message = b''
			self.reply(connection, 200, message, 'text/plain;charset=utf-8')
		elif any(connection.address.path.endswith('/' + x) for x in ('benjamin', 'admin', 'expert', 'user')):
			websocketd.RPChttpd.page(self, connection, path = connection.address.path[:connection.address.path.rfind('/') + 1])
		else:
			websocketd.RPChttpd.page(self, connection)
	def post(self, connection):
		# Add to queue (POST).
		if 'file' not in connection.post[1] or 'machine' not in connection.post[0] or 'action' not in connection.post[0]:
			log('invalid post: {}'.format(connection.post))
			self.reply(connection, 400)
			return False
		machine = connection.post[0]['machine'][0]
		action = connection.post[0]['action'][0]
		if machine not in machines or not isinstance(machines[machine], Machine):
			log('machine not found: %s' % machine)
			self.reply(connection, 404)
			return False
		# Count files, so we know when the connection should be closed.
		# Use a list to make it accessible from the callback.
		num = [len(connection.post[1]['file'])]
		for post in connection.post[1].pop('file'):
			def cb(success, ret, filename):
				self.reply(connection, 200 if success else 400, b'' if ret is None else ret.encode('utf-8'), 'text/plain;charset=utf-8')
				os.unlink(filename)
				num[0] -= 1
				if num[0] == 0:
					connection.socket.close()
			def cbwrap(filename):
				'''This function makes sure that filename gets its own scope and is not changed by the for loop.'''
				return lambda success, ret: cb(success, ret, filename)
			if action == 'queue_add':
				machines[machine].call('queue_add_POST', [connection.data['role'], post[0], post[1]], {}, cbwrap(post[0]))
			elif action == 'probe_add':
				machines[machine].call('probe_add_POST', [connection.data['role'], post[0], post[1]], {}, cbwrap(post[0]))
			elif action == 'audio_add':
				machines[machine].call('audio_add_POST', [connection.data['role'], post[0], post[1]], {}, cbwrap(post[0]))
			elif action == 'import':
				machines[machine].call('import_POST', [connection.data['role'], post[0], post[1]], {}, cbwrap(post[0]))
			else:
				cb(false, 'invalid POST action', post[0])
		return True
# }}}

class Connection: # {{{
	'''Object to handle a single network connection.
	This class is used with the WebSocket RPC server.  Functions in it can
	be called from the remote end using RPC requests.
	'''
	## Currently active connections.  Keys are their id, an int for internal use.
	connections = {}
	## Id for next connection.
	nextid = 0
	def __init__(self, socket): # {{{
		'''Constructor, as required by python-websockets.
		@param socket: remote end of RPC connection.
		'''
		socket.initialized = False
		socket.monitor = False
		socket.connection = self
		self.socket = socket
		self.id = Connection.nextid
		Connection.nextid += 1
		Connection.connections[self.id] = self
		# Done with setup; activate connection.
		self.socket()
	# }}}
	def get_ports(self): # {{{
		return tuple(ports.keys())
	# }}}
	def get_machines(self): # {{{
		return tuple(machines.keys())
	# }}}
	def detect(self, port): # {{{
		return detect(port)
	# }}}
	def disable(self, machine, reason = 'disabled by user'): # {{{
		assert self.socket.data['role'] in ('benjamin', 'admin', 'expert')
		assert machine in machines
		assert machines[machine].port
		return disable(machine, reason)
	# }}}
	def remove_machine(self, machine): # {{{
		assert self.socket.data['role'] in ('benjamin', 'admin')
		assert machine in machines
		machines[machine].remove_machine()
	# }}}
	def _get_command(self, board, port): # {{{
		if board == 'bbbmelzi ':
			return ('sudo', fhs.read_data(os.path.join('bb', 'flash-bb-0'), opened = False), fhs.read_data(os.path.join('bb', 'avrdude.conf'), opened = False), fhs.read_data(os.path.join('firmware', 'atmega1284p' + os.extsep + 'hex'), opened = False))
		if board == 'bb4melzi ':
			return ('sudo', fhs.read_data(os.path.join('bb', 'flash-bb-4'), opened = False), fhs.read_data(os.path.join('bb', 'avrdude.conf'), opened = False), fhs.read_data(os.path.join('firmware', 'atmega1284p' + os.extsep + 'hex'), opened = False))
		if board == 'opi ':
			return ('sudo', fhs.read_data(os.path.join('opi', 'flash-opi'), opened = False), fhs.read_data(os.path.join('opi', 'avrdude.conf'), opened = False), fhs.read_data(os.path.join('firmware', 'atmega1284p-12MHz' + os.extsep + 'hex'), opened = False))
		boards = read_boards()
		if board not in boards:
			raise ValueError('board type not supported')
		filename = fhs.read_data(os.path.join('firmware', boards[board]['build.mcu'] + os.extsep + 'hex'), opened = False)
		if filename is None:
			raise NotImplementedError('Firmware is not available')
		return ('avrdude', '-D', '-q', '-q', '-p', boards[board]['build.mcu'], '-C', '/etc/avrdude.conf', '-b', boards[board]['upload.speed'], '-c', boards[board]['upload.protocol'], '-P', port, '-U', 'flash:w:' + filename + ':i')
	# }}}
	def upload(self, port, board): # {{{
		wake = (yield)
		assert self.socket.data['role'] in ('benjamin', 'admin')
		assert port in ports
		if ports[port]:
			disable(ports[port], 'disabled for upload')
		def cancel():
			# Waking the generator kills the process.
			wake('Aborted')
		ports[port] = cancel
		command = self._get_command(board, port)
		data = ['']
		log('Flashing firmware: ' + ' '.join(command))
		broadcast(None, 'port_state', port, 3)
		process = subprocess.Popen(command, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.STDOUT, close_fds = True)
		def output():
			d = ''
			try:
				d = process.stdout.read().decode('utf-8')
			except:
				data[0] += '\nError writing %s firmware: ' % board + traceback.format_exc()
				log(repr(data[0]))
				wake(data[0])
				return False
			if d != '':
				#broadcast(None, 'message', port, '\n'.join(data[0].split('\n')[-4:]))
				data[0] += d
				return True
			wake(data[0])
			return False
		def error():
			data[0] += '\nError writing %s firmware: ' % board
			log(repr(data[0]))
			wake(data[0])
			return False
		fl = fcntl.fcntl(process.stdout.fileno(), fcntl.F_GETFL)
		fcntl.fcntl(process.stdout.fileno(), fcntl.F_SETFL, fl | os.O_NONBLOCK)
		websocketd.add_read(process.stdout, output, error)
		broadcast(None, 'uploading', port, 'uploading firmware for %s' % board)
		#broadcast(None, 'message', port, '')
		d = (yield)
		try:
			process.kill()	# In case it wasn't dead yet.
		except:
			pass
		try:
			process.communicate()	# Clean up.
		except:
			pass
		broadcast(None, 'uploading', port, None)
		#broadcast(None, 'message', port, '')
		broadcast(None, 'port_state', port, 0)
		ports[port] = None
		if autodetect:
			websocketd.call(None, detect, port)
		if d:
			return 'firmware upload for %s: ' % board + d
		else:
			return 'firmware for %s successfully uploaded' % board
	# }}}
	def upload_options(self, port): # {{{
		return upload_options(port)
	# }}}
	def create_machine(self): # {{{
		create_machine()
	# }}}
	def set_monitor(self, value): # {{{
		if value:
			self.socket.initialized = False
			self.socket.autodetect.event(autodetect)
			for p in ports:
				self.socket.new_port.event(p, self.upload_options(p))
				if ports[p] is None:
					self.socket.port_state.event(p, 0)
				elif not isinstance(ports[p], str):	# TODO: distinguish initial detect from flashing.
					self.socket.port_state.event(p, 3)
				else:
					self.socket.port_state.event(p, 2)
			for p in machines:
				machines[p].call('send_machine', [self.socket.data['role'], self.id], {}, lambda success, data: None)
			self.socket.initialized = True
		self.socket.monitor = value
	# }}}
	def get_monitor(self): # {{{
		return self.socket.monitor
	# }}}
	def get_role(self): # {{{
		return self.socket.data['role']
	# }}}
	def _call(self, name, a, ka): # {{{
		wake = (yield)
		#log('other: %s %s %s' % (name, repr(a), repr(ka)))
		if 'machine' in ka:
			machine = ka.pop('machine')
		else:
			machine = None
		if machine not in machines:
			if len(machines) == 1:
				machine = tuple(machines.keys())[0]
			else:
				options = [m for m in machines if machines[m].port is not None]
				if len(options) == 1:
					machine = options[0]
				else:
					log('No active machine')
					return ('error', 'No active machine')
		if name.endswith('_POST'):
			log('refusing to call function only meant for POST')
			return ('error', 'Invalid function name')
		def reply(success, ret):
			if success:
				wake(ret)
			else:
				log('machine errors')
				wake(('error', ret))
				#disable(machine, 'machine replied with error to wake up')
		machines[machine].call(name, (self.socket.data['role'],) + tuple(a), ka, reply)
		return (yield)
	# }}}
	def __getattr__ (self, attr): # {{{
		return lambda *a, **ka: self._call(attr, a, ka)
	# }}}
# }}}

def read_boards(): # {{{
	boards = {}
	for d in fhs.read_data('hardware', packagename = 'arduino', dir = True, multiple = True):
		for board in os.listdir(d):
			boards_txt = os.path.join(d, board, 'boards' + os.extsep + 'txt')
			if not os.path.exists(boards_txt):
				continue
			with open(boards_txt) as b:
				for line in b:
					if line.startswith('#') or line.strip() == '':
						continue
					parse = re.match('([^.=]+)\.([^=]+)=(.*)$', line.strip())
					if parse is None:
						log('Warning: invalid line in %s: %s' % (boards_txt, line.strip()))
						continue
					tag, option, value = parse.groups()
					if tag not in boards:
						boards[tag] = {}
					if option in boards[tag]:
						if boards[tag][option] != value:
							log('%s: duplicate tag %s.%s with different value (%s != %s); using %s' % (boards_txt, tag, option, value, boards[tag][option], boards[tag][option]))
							continue
					boards[tag][option] = value
	for tag in tuple(boards.keys()):
		if 'name' not in boards[tag]:
			boards[tag]['name'] = tag
		if any(x not in boards[tag] for x in ('upload.protocol', 'upload.speed', 'build.mcu', 'upload.maximum_size')):
			#log('skipping %s because hardware information is incomplete (%s)' % (boards[tag]['name'], repr(boards[tag])))
			del boards[tag]
			continue
		if int(boards[tag]['upload.maximum_size']) < 30000:
			# Not enough memory; don't complain about skipping this board.
			del boards[tag]
			continue
		if fhs.read_data(os.path.join('firmware', boards[tag]['build.mcu'] + os.extsep + 'hex'), opened = False) is None:
			#log('skipping %s because firmware for %s is not installed' % (boards[tag]['name'], boards[tag]['build.mcu']))
			del boards[tag]
			continue
	return boards
# }}}

def upload_options(port): # {{{
	ret = []
	if port in ('/dev/ttyS0', '/dev/ttyO0'):
		ret += [('bbbmelzi ', 'Melzi from BeagleBone (atmega1284p, bridgeboard v1)')]
	elif port in ('/dev/ttyS4', '/dev/ttyO4'):
		ret += [('bb4melzi ', 'Melzi from BeagleBone (atmega1284p, bridgeboard v2)')]
	elif port  == '/dev/ttyS1':
		ret += [('opi ', 'Athena on OrangePi Zero (atmega1284p at 12MHz)')]
	boards = read_boards()
	ret += list((tag, '%s (%s, %s, %d baud)' % (boards[tag]['name'], boards[tag]['build.mcu'], boards[tag]['upload.protocol'], int(boards[tag]['upload.speed']))) for tag in boards)
	ret.sort(key = lambda x: (boards[x[0]]['build.mcu'] if x[0] in boards else '', x[1]))
	return ret
# }}}

def broadcast(target, name, *args): # {{{
	if target is not None:
		#log('broadcasting to target %d' % target)
		if target not in Connection.connections:
			log('ignoring targeted broadcast of %s to missing connection %s' % (repr((name, args)), target))
			return
		target = Connection.connections[target].socket
		if target.monitor:
			#log('%s %s' % (name, repr(args)))
			getattr(target, name).event(*args)
		else:
			log("not broadcasting to target, because it isn't set to monitor")
	elif httpd:
		#log('broadcasting to all: %s' % repr((name, args)))
		for c in httpd.websockets:
			if c.monitor and c.initialized:
				#log('broadcasting to one')
				getattr(c, name).event(*args)
# }}}

class Machine: # {{{
	def __init__(self, port, process, run_id, send = True): # {{{
		'''Create a new Machine object.
		This can be called for several reasons:
		- At startup, every saved machine is started.  In this case, port is None.
		- When a new machine with an unknown uuid is detected on a port.  In this case, port and run_id are set.
		'''
		if port is not None:
			self.detecting = True
			broadcast(None, 'port_state', port, 1)
		self.name = None
		self.uuid = None
		self.port = port
		self.run_id = run_id
		self.process = process
		self.waiters = ({}, {}, {})
		self.next_mid = 0
		self.buffer = b''
		fl = fcntl.fcntl(process.stdout.fileno(), fcntl.F_GETFL)
		fcntl.fcntl(process.stdout.fileno(), fcntl.F_SETFL, fl | os.O_NONBLOCK)
		self.input_handle = websocketd.add_read(process.stdout, self.machine_input, self.machine_error)
		def get_vars(success, vars, cb = None):
			if not success:
				log('failed to get vars')
				return
			if self.uuid is None:
				log('new uuid:' + repr(vars['uuid']))
				self.uuid = vars['uuid']
			else:
				assert self.uuid == vars['uuid']
			self.detecting = False
			self.call('send_machine', ['admin', None], {}, lambda success, data: broadcast(None, 'port_state', port, 2))
			if cb is not None:
				cb()
		if send:
			self.call('get_globals', ('admin',), {}, get_vars)
		else:
			def finish(cb):
				self.call('get_globals', ('admin',), {}, lambda success, vars: get_vars(success, vars, cb))
				del self.finish
			self.finish = finish
	# }}}
	def call(self, name, args, kargs, cb): # {{{
		#log('calling {}'.format(repr((name, args, kargs))))
		data = json.dumps([self.next_mid, name, args, kargs]) + '\n'
		#log('calling %s on %d' % (repr(data), self.process.stdin.fileno()))
		try:
			self.process.stdin.write(data.encode('utf-8'))
			self.process.stdin.flush()
		except:
			log('killing machine handle because of error')
			#traceback.print_exc()
			def kill():
				cb(False, None)
				disable(self.uuid, 'error from machine')
			# Schedule this as a callback, so the generator isn't called recursively.
			websocketd.add_idle(kill)
			return
		#def debug_cb(success, ret):
		#	log('call {} returned: {}: {}'.format(name, success, ret))
		#	cb(success, ret)
		self.waiters[0][self.next_mid] = cb
		self.next_mid += 1
	# }}}
	def movewait(self, cb): # {{{
		self.waiters[1][self.next_mid] = cb
		self.next_mid += 1
	# }}}
	def tempwait(self, cb): # {{{
		self.waiters[2][self.next_mid] = cb
		self.next_mid += 1
	# }}}
	def machine_error(self): # {{{
		log('%s died from error; removing port.' % self.name)
		self.die('from error')
		del ports[self.port]
		return False
	# }}}
	def die(self, reason = 'at request'): # {{{
		log('{} died {}.'.format(self.uuid, reason))
		websocketd.remove_read(self.input_handle)
		try:
			self.process.kill()
		except:
			pass
		try:
			self.process.communicate()
		except:
			pass
		self.process = None
		for t in range(3):
			for w in self.waiters[t]:
				self.waiters[t][w](False, 'Machine {} died {}'.format(self.uuid, reason))
		if self.uuid in machines:
			del machines[self.uuid]
	# }}}
	def machine_input(self): # {{{
		while self.process is not None:
			data = self.process.stdout.read()
			if data is None:
				#log('%s: no data now' % self.name)
				# No more data.
				return True
			if data == b'':
				# Connection closed.
				self.die('because there was an error')
				return False
			self.buffer += data
			#log('machine input:' + repr(data))
			while b'\n'[0] in self.buffer:
				pos = self.buffer.index(b'\n'[0])
				line = self.buffer[:pos]
				self.buffer = self.buffer[pos + 1:]
				data = json.loads(line.decode('utf-8'))
				#log('machine command input:' + repr(data))
				if data[1] == 'broadcast':
					broadcast(data[2], data[3], self.uuid, *(data[4:]))
				elif data[1] == 'disconnect':
					port = self.port
					ports[self.port] = None
					broadcast(None, 'port_state', port, 0)
					#if autodetect:
					#	websocketd.call(None, detect, port)
				elif data[1] == 'error':
					if data[0] is None:
						# Error on command without id.
						log('error on command without id: %s' % repr(data))
					else:
						self.waiters[0].pop(data[0])(False, data[2])
				elif data[1] == 'return':
					self.waiters[0].pop(data[0])(True, data[2])
				elif data[1] == 'movecb':
					self.waiters[1].pop(data[0])(True, data[2])
				elif data[1] == 'tempcb':
					self.waiters[2].pop(data[0])(True, data[2])
				else:
					raise AssertionError('invalid reply from machine process: %s' % repr(data))
	# }}}
	def remove_machine(self): # {{{
		def finish():
			broadcast(None, 'del_machine', self.uuid)
			del machines[self.uuid]
			self.call('die', ['admin', 'Machine is removed'], {}, lambda success, ret: self.die('because it was removed'))
		if self.port and ports[self.port]:
			assert ports[self.port] == self.uuid
			self.call('disconnect', ['admin'], {}, lambda success, ret: finish())
		else:
			finish()
	# }}}
# }}}

def nextid(): # {{{
	global last_id
	# 0x23456789 is an arbitrary number with bits set in every nybble, that is
	# odd(so it doesn't visit the same number twice until it did all of
	# them, because it loops at 2**32, which is not divisible by anything
	# except 2).
	last_id = (last_id + 0x23456789) & 0xffffffff
	return bytes([id_map[(last_id >> (4 * c)) & 0xf] for c in range(8)])
last_id = random.randrange(1 << 32)
# Parity table is [0x8b, 0x2d, 0x1e]; half of these codes overlap with codes from the single command map; those single commands are not used.
id_map = [0x40, 0xe1, 0xd2, 0x73, 0x74, 0xd5, 0xe6, 0x47, 0xf8, 0x59, 0x6a, 0xcb, 0xcc, 0x6d, 0x5e, 0xff]
# }}}

# TODO: see if this should be used again.
def job_done(port, completed, reason): # {{{
	broadcast(None, 'running', port.port, False)
	if config['done']:
		cmd = config['done']
		cmd = cmd.replace('[[STATE]]', 'completed' if completed else 'aborted').replace('[[REASON]]', reason)
		log('running %s' % cmd)
		p = subprocess.Popen(cmd, stdout = subprocess.PIPE, shell = True, close_fds = True)
		def process_done():
			data = p.stdout.read()
			if data:
				log('Data from completion callback: %s' % repr(data))
				return True
			log('Callback for job completion done; return: %s' % repr(p.wait()))
			return False
		def process_error():
			log('Job completion process returned error.')
			return False
		websocketd.add_read(p.stdout, process_done, process_error)
# }}}

def disable(uuid, reason): # {{{
	if uuid is not None and not isinstance(uuid, str):
		uuid()
		return
	if uuid not in machines:
		log('not disabling nonexistent machine %s' % uuid)
		return
	p = machines[uuid]
	if p.port not in ports:
		log("not disabling machine which isn't enabled")
		return
	p.call('disconnect', ('admin', reason), {}, lambda success, ret: None)
	port = p.port
	ports[port] = None
	p.port = None
	broadcast(None, 'port_state', port, 0)
# }}}

class Admin_Connection: # {{{
	def __init__(self, remote): # {{{
		self.remote = remote
		remote.readlines(self.read)
		remote.disconnect_cb(lambda connection, data: data)
	# }}}
	def read(self, line): # {{{
		if line.strip() == '':
			return
		try:
			action, dev = line.split(None, 1)
		except:
			log('invalid command on admin socket: %s' % line)
			return
		if action == 'add':
			add_port(dev.strip())
		elif action == 'remove':
			remove_port(dev.strip())
		else:
			log('invalid action on admin socket: %s' % line)
	# }}}
# }}}

def remove_port(port): # {{{
	log('removing port %s' % port)
	if port not in ports:
		return
	if ports[port]:
		disable(ports[port], 'port is removed')
	del ports[port]
	broadcast(None, 'del_port', port)
# }}}

def add_port(port): # {{{
	if port in ports:
		log('already existing port %s cannot be added' % port)
		return
	if not re.match(config['whitelist'], port) or re.match(config['blacklist'], port) or re.match(config['add-blacklist'], port):
		#log('skipping blacklisted or non-whitelisted port %s' % port)
		return
	ports[port] = None
	broadcast(None, 'new_port', port, upload_options(port))
	broadcast(None, 'port_state', port, 0)
	if autodetect:
		websocketd.call(None, detect, port)
# }}}

def detect(port): # {{{
	log('detecting machine on %s' % port)
	if port not in ports:
		log('port does not exist')
		return
	if ports[port] != None:
		# Abort detection in progress.
		if ports[port]:
			disable(ports[port], 'disabled to prepare for detection')
		if ports[port] != None:
			# This should never happen.
			log('BUG: port is not in detectable state. Please report this.')
			return
	broadcast(None, 'port_state', port, 1)
	if port == '-' or port.startswith('!'):
		run_id = nextid()
		process = subprocess.Popen((fhs.read_data('driver.py', opened = False), '--uuid', '-', '--allow-system', config['allow-system']) + (('--system',) if fhs.is_system else ()), stdin = subprocess.PIPE, stdout = subprocess.PIPE, close_fds = True)
		machines[port] = Machine(port, process, run_id)
		ports[port] = port
		return False
	if not os.path.exists(port):
		log("not detecting on %s, because file doesn't exist." % port)
		return False
	if config['predetect']:
		env = os.environ.copy()
		env['PORT'] = port
		subprocess.call(config['predetect'], env = env, shell = True)
	try:
		machine = serial.Serial(port, baudrate = 115200, timeout = 0)
	except serial.SerialException as e:
		log('failed to open serial port %s (%s).' % (port, str(e)))
		del ports[port]
		#traceback.print_exc()
		return False
	# We need to get the machine id first.  If the machine is booting, this can take a while.
	id = [None, None, None, None]	# data, timeouts, had data
	# Wait to make sure the command is interpreted as a new packet.
	def part2():
		id[0] = b''
		id[1] = 0
		id[2] = False
		def timeout():
			id[1] += 1
			if id[1] >= 30:
				# Timeout.  Give up.
				websocketd.remove_read(watcher)
				machine.close()
				log('Timeout waiting for machine on port %s; giving up.' % port)
				ports[port] = None
				broadcast(None, 'port_state', port, 0)
				return
			if not id[2]:
				machine.write(protocol.single['ID'])
			else:
				id[2] = False
			timeout_handle[0] = websocketd.add_timeout(time.time() + .5, timeout)
		def boot_machine_input():
			id[2] = True
			ids = [protocol.single[code][0] for code in ('ID', 'STARTUP')]
			# CMD:1 ID:8 + 16 Checksum:9 Total: 34
			while len(id[0]) < 34:
				try:
					data = machine.read(34 - len(id[0]))
				except OSError:
					continue
				except IOError:
					continue
				id[0] += data
				#log('incomplete id: ' + id[0])
				if len(id[0]) < 34:
					if len(id[0]) > 0 and id[0][0] == protocol.single['CONTROLLER'][0]:
						# This is a controller. Spawn the process, then cancel this detection.
						websocketd.remove_timeout(timeout_handle[0])
						machine.close()
						ports[port] = None
						broadcast(None, 'port_state', port, 0)
						log('Starting controller driver on ' + port)
						env = os.environ.copy()
						env['PORT'] = port
						subprocess.Popen(config['controller'], env = env, shell = True)
						return False
					return True
				if id[0][0] not in ids or not protocol.check(id[0]):
					log('skip non-id: %s (%s)' % (''.join('%02x' % x for x in id[0]), repr(id[0])))
					f = len(id[0])
					for start in ids:
						if start in id[0][1:]:
							p = id[0].index(bytes((start,)), 1)
							if p < f:
								f = p
							log('Keeping some')
					if f == 0:
						f = 1
					id[0] = id[0][f:]
					return True
			# We have something to handle; cancel the timeout, but keep the serial port open to avoid a reset. (I don't think this even works, but it doesn't hurt.)
			websocketd.remove_timeout(timeout_handle[0])
			# This machine was running and tried to send an id.  Check the id.
			uuid = id[0][9:9 + 16]
			if (uuid[7] & 0xf0) != 0x40 or (uuid[9] & 0xc0) != 0x80:
				# Broken uuid; create a new one and set it.
				log('broken uuid: ' + repr(uuid))
				uuid = None
			else:
				uuid = ''.join('%02x' % x for x in uuid[:16])
				uuid = uuid[:8] + '-' + uuid[8:12] + '-' + uuid[12:16] + '-' + uuid[16:20] + '-' + uuid[20:32]
				id[0] = id[0][1:9]
				running_machine = [p for p in machines if machines[p].run_id == id[0]]
				assert len(running_machine) < 2
				if len(running_machine) > 0:
					p = running_machine[0]
					assert p.uuid == uuid
					if p.port is not None:
						disable(p.uuid, 'disabled machine which was detected on different port')
					log('rediscovered machine %s on %s' % (''.join('%02x' % x for x in id[0]), port))
					ports[port] = p.uuid
					p.port = port
					def close_port(success, data):
						log('reconnect complete; closing server port')
						machine.close()
					p.call('reconnect', ['admin', port], {}, lambda success, ret: (ports[port].call('send_machine', ['admin', None], {}, close_port) if success else close_port()))
					broadcast(None, 'port_state', port, 2)
					return False
			run_id = nextid()
			# Find uuid or create new Machine object.
			if uuid in machines:
				log('accepting known machine on port %s (uuid %s)' % (port, uuid))
				machines[uuid].port = port
				ports[port] = uuid
				log('connecting %s to port %s' % (uuid, port))
				machines[uuid].call('connect', ['admin', port, [chr(x) for x in run_id]], {}, lambda success, ret: None)
			else:
				log('accepting unknown machine on port %s' % port)
				# Close detect port so it doesn't interfere.
				machine.close()
				#log('machines: %s' % repr(tuple(machines.keys())))
				process = subprocess.Popen((fhs.read_data('driver.py', opened = False), '--uuid', uuid if uuid is not None else '', '--allow-system', config['allow-system']) + (('--system',) if fhs.is_system else ()), stdin = subprocess.PIPE, stdout = subprocess.PIPE, close_fds = True)
				new_machine = Machine(port, process, run_id, send = False)
				def finish():
					log('finish detect %s' % repr(uuid))
					ports[port] = uuid
					machines[uuid] = new_machine
					log('connecting new machine %s to port %s' % (uuid, port))
					new_machine.call('connect', ['admin', port, [chr(x) for x in run_id]], {}, lambda success, ret: None)
				if uuid is None:
					def prefinish(success, uuid):
						assert success
						new_machine.uuid = uuid
						new_machine.finish(finish)
					new_machine.call('reset_uuid', ['admin'], {}, prefinish)
				else:
					new_machine.finish(finish)
			return False
		def boot_machine_error():
			log('error during machine detection on port %s.' % port)
			websocketd.remove_timeout(timeout_handle[0])
			machine.close()
			ports[port] = None
			broadcast(None, 'port_state', port, 0)
			return False
		machine.write(protocol.single['ID'])
		timeout_handle = [websocketd.add_timeout(time.time() + .5, timeout)]
		watcher = websocketd.add_read(machine, boot_machine_input, boot_machine_error)
		def cancel():
			websocketd.remove_timeout(timeout_handle[0])
			websocketd.remove_read(watcher)
			machine.close()
			ports[port] = None
		ports[port] = cancel
	# Wait at least a second before sending anything, otherwise the bootloader thinks we might be trying to reprogram it.
	handle = websocketd.add_timeout(time.time() + 1.5, part2)
	def cancel():
		websocketd.remove_timeout(handle)
		ports[port] = None
	ports[port] = cancel
# }}}

# Main loop. {{{
def _disconnect(socket, data):
	del Connection.connections[socket.connection.id]
try:
	httpd = Server(config['port'], Connection, disconnect_cb = _disconnect, httpdirs = fhs.read_data('html', dir = True, multiple = True), address = config['address'], log = config['log'], tls = tls)
	udevsocket = fhs.write_runtime('udev.socket', packagename = 'franklin', opened = False)
	os.makedirs(os.path.dirname(udevsocket), exist_ok = True)
	if os.path.exists(udevsocket):
		os.unlink(udevsocket)
	udevserver = network.Server(udevsocket, Admin_Connection)
except OSError:
	log('failed to start server: %s' % sys.exc_info()[1])
	sys.exit(1)
# }}}

# Initialization. {{{
def create_machine(uuid = None): # {{{
	if uuid is None:
		uuid = protocol.new_uuid()
	process = subprocess.Popen((fhs.read_data('driver.py', opened = False), '--uuid', uuid, '--allow-system', config['allow-system']) + (('--system',) if fhs.is_system else ()), stdin = subprocess.PIPE, stdout = subprocess.PIPE, close_fds = True)
	machines[uuid] = Machine(None, process, None)
	return uuid
# }}}

# Start known machine drivers.
for d in fhs.read_data('.', dir = True, opened = False, multiple = True):
	for uuid in os.listdir(d):
		if uuid in machines:
			continue
		if not os.path.isdir(os.path.join(d, uuid, 'profiles')):
			continue
		log('starting machine %s' % uuid)
		create_machine(uuid = uuid)

# Detect serial ports. {{{
# Assume a GNU/Linux system; if you have something else, you need to come up with a way to iterate over all your serial ports and implement it here.  Patches welcome, especially if they are platform-independent.
try:
	# Try Linux sysfs.
	for tty in os.listdir('/sys/class/tty'):
		add_port('/dev/' + tty)
except:
	# Try more generic approach.  Don't use this by default, because it doesn't detect all ports on GNU/Linux.
	try:
		import serial.tools.list_ports
		for tty in serial.tools.list_ports.comports():
			add_port(tty[0])
	except:
		traceback.print_exc()
		log('Not probing serial ports, because an error occurred: %s' % sys.exc_info()[1])
# }}}
# }}}

log('Franklin server is running')
print('Franklin server is running')
websocketd.fgloop()


'''
ports is a dict with ports as keys and uuids as values, or None if no machine
is active on the port.
machines is a dict with uuids as keys and Machine objects as values.

As startup, all saved machines are loaded in the machines object, and all ports
are first found and inserted into the ports object, then machines are detected
on them.  If found, the machine is enabled and the dicts are updated.

When a port is removed that has a machine attached, it is first disabled.

Machines can also be disabled manually, and detection on a port can be
requested manually as well.
'''

#!/usr/bin/python3
# vim: foldmethod=marker :
# server.py - printer multiplexing for Franklin {{{
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
* printer: default printer for new client connections.  Leave empty to use the
  first detected printer.  Default: ''
* blacklist: regular expression of serial ports that detection should not be
  attempted on.  This should normally not be used.  add-blacklist should be
  used instead.
* add-blacklist: same as blacklist.  However, add-blacklist has a empty
  default, so it can be used to add ports to the standard blacklist, as opposed
  to replacing it.
* autodetect: if true, communication is attempted on newly detected ports.
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
* local: Internal use only.  Do not use.
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
classes are useful: Connection and Printer.  Functions from those classes can
be called using RPC requests.  In Printer, functions with a role prefix must be
called by a connections with at least those permissions.  The prefix must not
be part of the RPC request.
'''
# }}}

# Imports and config. {{{
import re
import os
import sys
import math
import random
import websocketd
from websocketd import log
import fhs
try:
	from gi.repository import GLib
except ImportError:
	import glib as GLib
import subprocess
import crypt
import time
import serial
import json
import traceback
import fcntl
import protocol

config = fhs.init(packagename = 'franklin', config = {
		'port': '8000',
		'address': '',
		'printer': '',
		'blacklist': '/dev/(input/.*|ptmx|console|tty(printk|(GS)?\\d*))$',
		'add-blacklist': '$',
		'autodetect': 'True',
		'predetect': 'stty -F #PORT# raw 115200 -echo -echoe -echok -echoke -echonl -echoprt',
		'allow-system': '^$',
		'admin': '',
		'expert': '',
		'user': '',
		'done': '',
		'local': '',
		'log': '',
		'tls': 'True',
	})
# }}}

# Global variables. {{{
httpd = None
default_printer = (None, None)
ports = {}
autodetect = config['autodetect'].lower() == 'true'
tls = config['tls'].lower() == 'true'
orphans = {}
scripts = {}
# }}}

# Load scripts. {{{
for d in fhs.read_data('scripts', dir = True, multiple = True):
	for s in os.listdir(d):
		name, ext = os.path.splitext(s)
		if ext != os.extsep + 'js' or name in scripts:
			continue
		dataname = os.path.join(d, name + os.extsep + 'dat')
		scripts[name] = [open(os.path.join(d, s)).read(), open(dataname).read() if os.path.exists(dataname) else None]
nextscriptname = 0
while '%04d' % nextscriptname in scripts:
	nextscriptname += 1
# }}}

class Server(websocketd.RPChttpd): # {{{
	def auth_message(self, connection, is_websocket):
		path = connection.address.path
		for extra in ('/', '/websocket'):
			if path.endswith(extra):
				path = path[:-len(extra)]
		if path.endswith('/benjamin'):
			connection.data['role'] = 'benjamin'
			connection.data['pwd'] = config['admin'] or config['expert'] or config['user']
		elif path.endswith('/admin'):
			connection.data['role'] = 'admin'
			connection.data['pwd'] = config['admin'] or config['expert'] or config['user']
		elif path.endswith('/expert'):
			connection.data['role'] = 'expert'
			connection.data['pwd'] = config['expert'] or config['user']
		else:
			connection.data['role'] = 'user'
			connection.data['pwd'] = config['user']
		return 'Please identify yourself for %s access' % connection.data['role'] if connection.data['pwd'] else None
	def authenticate(self, connection):
		if ':' in connection.data['pwd']:
			return [connection.data['user'], connection.data['password']] == connection.data['pwd'].split(':', 1)
		else:
			return connection.data['password'] == connection.data['pwd']
	def page(self, connection):
		if 'port' in connection.query:
			# Export request.
			port = connection.query['port'][0]
			if port not in ports or not ports[port] or ports[port].detecting:
				self.reply(connection, 404)
			else:
				def export_reply(success, message):
					self.reply(connection, 200, message.encode('utf-8'), 'text/plain;charset=utf8')
					connection.socket.close()
				ports[port].call('export_settings', (connection.data['role'],), {}, export_reply)
				return True
		elif connection.address.path.endswith('/adc'):
			filename = '/tmp/franklin-adc-dump'
			if os.path.exists(filename):
				message = open(filename, 'rb').read()
				os.unlink(filename)
			else:
				message = ''
			self.reply(connection, 200, message, 'text/plain;charset=utf8')
		elif any(connection.address.path.endswith('/' + x) for x in ('benjamin', 'admin', 'expert', 'user')):
			websocketd.RPChttpd.page(self, connection, path = connection.address.path[:connection.address.path.rfind('/') + 1])
		else:
			websocketd.RPChttpd.page(self, connection)
	def post(self, connection):
		# Add to queue (POST).
		if 'file' not in connection.post[1] or 'port' not in connection.post[0] or 'action' not in connection.post[0]:
			self.reply(connection, 400)
			return False
		port = connection.post[0]['port'][0]
		action = connection.post[0]['action'][0]
		if port not in ports or not ports[port] or ports[port].detecting:
			log('port not found: %s' % port)
			self.reply(connection, 404)
			return False
		post = connection.post[1].pop('file')
		def cb(success, ret):
			self.reply(connection, 200 if success else 400, b'' if ret is None else ret.encode('utf8'), 'text/plain;charset=utf8')
			os.unlink(post[0])
			connection.socket.close()
		if action == 'queue_add':
			ports[port].call('queue_add_file', [connection.data['role'], post[0], post[1]], {}, cb)
		elif action == 'audio_add':
			ports[port].call('audio_add_file', [connection.data['role'], post[0], post[1]], {}, cb)
		elif action == 'import':
			ports[port].call('import_file', [connection.data['role'], post[0], post[1]], {}, cb)
		else:
			os.unlink(post[0])
			self.reply(connection, 400)
			return False
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
		self.printer = self.find_printer(*default_printer)
		self.id = Connection.nextid
		Connection.nextid += 1
		Connection.connections[self.id] = self
		# Done with setup; activate connection.
		self.socket()
	# }}}
	def find_printer(self, uuid = None, port = None): # {{{
		'''Detect a printer by uuid or port.
		If both uuid and port are None, return any connected printer.
		@param uuid: requested uuid, or None to allow any uuid.
		@param port: regular expression for port, or None to allow any port.
		@return The printer name, or None if no printer matches the request.
		'''
		for p in ports:
			if not ports[p] or ports[p].detecting:
				continue
			if (not uuid or uuid == ports[p].uuid) and (not port or re.match(port, p)):
				return p
		return None
	# }}}
	def set_autodetect(self, detect): # {{{
		'''Enable or disable autodetection on new ports.
		If autodetection is enabled, detection will be attempted for
		any newly detected port, both at boot time and later.  Changing
		to autodetect does not trigger a detection on ports that were
		already known to Franklin.
		@param detect: New autodetect state.
		@return: None.
		'''
		global autodetect
		autodetect = detect
		broadcast(None, 'autodetect', autodetect)
	# }}}
	def get_autodetect(self): # {{{
		'''Get autodetection status.
		@return: Autodetection status.
		'''
		return autodetect
	# }}}
	def detect(self, port): # {{{
		detect(port, self.socket.data['role'])
	# }}}
	def detect_all(self): # {{{
		for p in ports:
			if ports[p] is not None:
				continue
			self.detect(p)
	# }}}
	def add_port(self, port): # {{{
		assert self.socket.data['role'] in ('benjamin', 'admin')
		add_port(port)
	# }}}
	def remove_port(self, port): # {{{
		assert self.socket.data['role'] in ('benjamin', 'admin')
		remove_port(port)
	# }}}
	def get_ports(self): # {{{
		return tuple(ports.keys())
	# }}}
	def set_default_printer(self, name = None, port = None): # {{{
		global default_printer
		default_printer = (name, port)
	# }}}
	def get_default_printer(self): # {{{
		return default_printer
	# }}}
	def new_script(self, code): # {{{
		global nextscriptname
		name = '%04d' % nextscriptname
		scripts[name] = [code, None]
		while '%04d' % nextscriptname in scripts:
			nextscriptname += 1
		f = fhs.write_data(os.path.join('scripts', name + os.extsep + 'js'), text = False)
		f.write(code)
		f.close()
		broadcast(None, 'new_script', name, code, None)
		return []
	# }}}
	def del_script(self, name): # {{{
		del scripts[name]
		for e in('js', 'dat'):
			filename = fhs.write_data(os.path.join('scripts', name + os.extsep + e), opened = False)
			if os.path.exists(filename):
				os.unlink(filename)
		broadcast(None, 'del_script', name)
	# }}}
	def set_data(self, name, data): # {{{
		scripts[name][1] = data
		filename = fhs.write_data(os.path.join('scripts', name + os.extsep + 'dat'), opened = False)
		if data is None:
			if os.path.exists(filename):
				os.unlink(filename)
		else:
			with open(filename, 'wb') as f:
				f.write(data)
		broadcast(None, 'new_data', name, data)
	# }}}
	def disable(self, port): # {{{
		return disable(self.socket.data['role'], port)
	# }}}
	def _read_boards(self): # {{{
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
				log('skipping %s because hardware information is incomplete (%s)' % (boards[tag]['name'], repr(boards[tag])))
				del boards[tag]
				continue
			if int(boards[tag]['upload.maximum_size']) < 32000:
				# Not enough memory; don't complain about skipping this board.
				del boards[tag]
				continue
			if fhs.read_data(os.path.join('firmware', boards[tag]['build.mcu'] + os.extsep + 'hex'), opened = False) is None:
				log('skipping %s because firmware for %s is not installed' % (boards[tag]['name'], boards[tag]['build.mcu']))
				del boards[tag]
				continue
		return boards
	# }}}
	def upload_options(self, port): # {{{
		ret = []
		if port in ('/dev/ttyS0', '/dev/ttyO0'):
			ret += [('bbbmelzi ', 'Melzi from BeagleBone (atmega1284p, bridgeboard v1)')]
		elif port in ('/dev/ttyS4', '/dev/ttyO4'):
			ret += [('bb4melzi ', 'Melzi from BeagleBone (atmega1284p, bridgeboard v2)')]
		boards = self._read_boards()
		ret += list((tag, '%s (%s, %s, %d baud)' % (boards[tag]['name'], boards[tag]['build.mcu'], boards[tag]['upload.protocol'], int(boards[tag]['upload.speed']))) for tag in boards)
		ret.sort(key = lambda x: (boards[x[0]]['build.mcu'] if x[0] in boards else '', x[1]))
		return ret
	# }}}
	def _get_command(self, board, port): # {{{
		if board == 'bbbmelzi ':
			return ('sudo', fhs.read_data('flash-bbb', opened = False), fhs.read_data('avrdude.conf', opened = False), fhs.read_data(os.path.join('firmware', 'atmega1284p' + os.extsep + 'hex'), opened = False))
		if board == 'bb4melzi ':
			return ('sudo', fhs.read_data('flash-bb-O4', opened = False), fhs.read_data('avrdude.conf', opened = False), fhs.read_data(os.path.join('firmware', 'atmega1284p' + os.extsep + 'hex'), opened = False))
		boards = self._read_boards()
		if board not in boards:
			raise ValueError('board type not supported')
		filename = fhs.read_data(os.path.join('firmware', boards[board]['build.mcu'] + os.extsep + 'hex'), opened = False)
		if filename is None:
			raise NotImplementedError('Firmware is not available')
		return ('avrdude', '-q', '-q', '-p', boards[board]['build.mcu'], '-b', boards[board]['upload.speed'], '-c', boards[board]['upload.protocol'], '-P', port, '-U', 'flash:w:' + filename + ':i')
	# }}}
	def upload(self, port, board): # {{{
		wake = (yield)
		assert self.socket.data['role'] in ('benjamin', 'admin')
		assert ports[port] is None
		ports[port] = 0
		command = self._get_command(board, port)
		data = ['']
		log('Flashing firmware: ' + ' '.join(command))
		broadcast(None, 'port_state', port, 3)
		process = subprocess.Popen(command, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.STDOUT, close_fds = True)
		def output(fd, cond):
			d = ''
			try:
				d = process.stdout.read().decode('utf-8')
			except:
				data[0] += '\nError writing %s firmware: ' % board + traceback.format_exc()
				log(repr(data[0]))
				wake(data[0])
				return False
			if d != '':
				broadcast(None, 'message', port, '\n'.join(data[0].split('\n')[-4:]))
				data[0] += d
				return True
			wake(data[0])
			return False
		fl = fcntl.fcntl(process.stdout.fileno(), fcntl.F_GETFL)
		fcntl.fcntl(process.stdout.fileno(), fcntl.F_SETFL, fl | os.O_NONBLOCK)
		GLib.io_add_watch(process.stdout, GLib.IO_IN | GLib.IO_PRI | GLib.IO_HUP, output)
		broadcast(None, 'blocked', port, 'uploading firmware for %s' % board)
		broadcast(None, 'message', port, '')
		d = (yield)
		try:
			process.kill()	# In case it wasn't dead yet.
		except OSError:
			pass
		process.communicate()	# Clean up.
		broadcast(None, 'blocked', port, None)
		broadcast(None, 'message', port, '')
		broadcast(None, 'port_state', port, 0)
		ports[port] = None
		if autodetect:
			websocketd.call(None, self.detect, port)
		if d:
			return 'firmware upload for %s: ' % board + d
		else:
			return 'firmware for %s successfully uploaded' % board
	# }}}
	def set_printer(self, printer = None, port = None): # {{{
		self.printer = self.find_printer(printer, port)
	# }}}
	def get_printer(self): # {{{
		return self.printer
	# }}}
	def set_monitor(self, value): # {{{
		if value:
			self.socket.initialized = False
			self.socket.autodetect.event(autodetect)
			for p in ports:
				self.socket.new_port.event(p)
				if ports[p] is None:
					self.socket.port_state.event(p, 0)
				elif ports[p] == 0:
					self.socket.port_state.event(p, 3)
				elif ports[p] is False or ports[p].detecting:
					self.socket.port_state.event(p, 1)
				else:
					self.socket.port_state.event(p, 2)
					ports[p].call('send_printer', [self.socket.data['role'], self.id], {}, lambda success, data: None)
			for s in scripts:
				broadcast(self.id, 'new_script', s, scripts[s][0], scripts[s][1])
			self.socket.initialized = True
		self.socket.monitor = value
	# }}}
	def get_monitor(self): # {{{
		return self.socket.monitor
	# }}}
	def get_role(self): # {{{
		return self.socket.data['role']
	# }}}
	def _call (self, name, a, ka): # {{{
		wake = (yield)
		#log('other: %s %s %s' % (name, repr(a), repr(ka)))
		if not self.printer or self.printer not in ports or not ports[self.printer] or ports[self.printer].detecting:
			self.printer = self.find_printer()
			if not self.printer:
				log('No printer found')
				return ('error', 'No printer found')
		def reply(success, ret):
			if success:
				wake(ret)
			else:
				log('printer errors')
				wake(None)
				#disable('admin', self.printer)
		ports[self.printer].call(name, (self.socket.data['role'],) + tuple(a), ka, reply)
		return (yield)
	# }}}
	def __getattr__ (self, attr): # {{{
		return lambda *a, **ka: self._call(attr, a, ka)
	# }}}
# }}}

def broadcast(target, name, *args): # {{{
	if target is not None:
		#log('broadcasting to target %d' % target)
		if target not in Connection.connections:
			log('ignoring targeted broadcast of %s to missing connection %d' % (repr((name, args)), target))
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

class Port: # {{{
	def __init__(self, port, process, detectport, run_id): # {{{
		self.detecting = True
		broadcast(None, 'port_state', port, 1)
		self.name = None
		self.uuid = None
		self.port = port
		self.run_id = run_id
		self.process = process
		self.waiters = ({}, {}, {})
		self.next_mid = 0
		self.input_handle = GLib.io_add_watch(process.stdout.fileno(), GLib.IO_IN | GLib.IO_HUP, self.printer_input)
		old = []
		def get_settings(success, settings):
			if not success:
				log('failed to get settings')
				return
			self.call('import_settings', ['admin', settings], {}, lambda success, ret: None)
			GLib.source_remove(orphans[self.run_id].input_handle)
			orphans[self.run_id].call('die', ('admin', 'replaced by new connection',), {}, lambda success, ret: None)
			try:
				orphans[self.run_id].process.kill()
				orphans[self.run_id].process.communicate()
			except OSError:
				pass
			del orphans[self.run_id]
			self.detecting = False
			self.call('send_printer', ['admin', None], {}, lambda success, data: broadcast(None, 'port_state', port, 2))
		def get_vars(success, vars):
			if not success:
				log('failed to get vars')
				return
			# The child has opened the port now; close our handle.
			if detectport is not None:
				log('Driver started; closing server port')
				detectport.close()
			self.uuid = vars['uuid']
			# Copy settings from orphan with the same run_id, then kill the orphan.
			if self.run_id in orphans and orphans[self.run_id].uuid == self.uuid:
				orphans[self.run_id].call('export_settings', ('admin',), {}, get_settings)
			else:
				self.detecting = False
				self.call('send_printer', ['admin', None], {}, lambda success, data: broadcast(None, 'port_state', port, 2))
		self.call('get_globals', ('admin',), {}, get_vars)
	# }}}
	def call(self, name, args, kargs, cb): # {{{
		data = json.dumps([self.next_mid, name, args, kargs]) + '\n'
		#log('calling %s on %d' % (repr(data), self.process.stdin.fileno()))
		try:
			self.process.stdin.write(data.encode('utf-8'))
			self.process.stdin.flush()
		except:
			log('killing printer handle because of IOError')
			#traceback.print_exc()
			cb(False, None)
			disable('admin', self.port)
			return
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
	def printer_input(self, fd, cond): # {{{
		line = self.process.stdout.readline()
		if line == b'':
			log('%s died.' % self.name)
			self.process.communicate()	# Clean up the zombie.
			for t in range(3):
				for w in self.waiters[t]:
					self.waiters[t][w](False, 'Printer died')
			disable('admin', self.port)
			return False
		data = json.loads(line.decode('utf-8'))
		#log('printer input:' + repr(data))
		if data[1] == 'broadcast':
			broadcast(data[2], data[3], self.port, *(data[4:]))
		elif data[1] == 'disconnect':
			# Don't remember a printer that hasn't sent its name yet.
			port = self.port
			ports[self.port] = None
			broadcast(None, 'port_state', port, 0)
			self.make_orphan()
			if autodetect:
				detect(self.port, 'admin')
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
			raise AssertionError('invalid reply from printer process: %s' % repr(data))
		return True
	# }}}
	def make_orphan(self): # {{{
		if ports[self.port]:
			self.call('disconnect', ['admin'], {}, lambda cd, arg: None)
		# If there already is an orphan with the same uuid, kill the old orphan.
		for o in [x for x in orphans if orphans[x].uuid == self.uuid]:
			# This for loop always runs 0 or 1 times, never more.
			log('killing duplicate orphan')
			orphans[o].call('die', ('admin', 'replaced by connection with same uuid',), {}, lambda success, ret: None)
			del orphans[o]
		orphans[self.run_id] = self
		broadcast(None, 'del_printer', self.port)
	# }}}
# }}}

def detect(port, role): # {{{
	log('detecting printer on %s' % port)
	if port not in ports:
		log('port does not exist')
	if ports[port] != None:
		# Abort detection in progress.
		disable(role, port)
		if ports[port] != None:
			log('port is not in detectable state')
			return
	ports[port] = False
	broadcast(None, 'port_state', port, 1)
	if port == '-' or port.startswith('!'):
		run_id = nextid()
		process = subprocess.Popen((fhs.read_data('driver.py', opened = False), '--cdriver', config['local'] or fhs.read_data('franklin-cdriver', opened = False), '--port', port, '--run-id', run_id, '--allow-system', config['allow-system']) + (('--system',) if fhs.is_system else ()), stdin = subprocess.PIPE, stdout = subprocess.PIPE, close_fds = True)
		ports[port] = Port(port, process, None, run_id)
		return False
	if not os.path.exists(port):
		log("not detecting on %s, because file doesn't exist." % port)
		return False
	if config['predetect']:
		subprocess.call(config['predetect'].replace('#PORT#', port), shell = True)
	try:
		printer = serial.Serial(port, baudrate = 115200, timeout = 0)
	except serial.SerialException as e:
		log('failed to open serial port %s (%s).' % (port, e.message))
		#traceback.print_exc()
		return False
	# We need to get the printer id first.  If the printer is booting, this can take a while.
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
				GLib.source_remove(watcher)
				printer.close()
				log('Timeout waiting for printer on port %s; giving up.' % port)
				ports[port] = None
				broadcast(None, 'port_state', port, 0)
				return False
			if not id[2]:
				printer.write(protocol.single['ID'])
			else:
				id[2] = False
			return True
		def boot_printer_input(fd, cond):
			id[2] = True
			ids = [protocol.single[code][0] for code in ('ID', 'STARTUP')]
			# CMD:1 ID:8 Checksum:3
			while len(id[0]) < 12:
				try:
					data = printer.read(12 - len(id[0]))
				except OSError:
					continue
				except IOError:
					continue
				id[0] += data
				#log('incomplete id: ' + id[0])
				if len(id[0]) < 12:
					return True
				if id[0][0] not in ids or not protocol.check(id[0]):
					log('skip non-id: %s (%s)' % (''.join('%02x' % x for x in id[0]), repr(id[0])))
					f = len(id[0])
					for start in ids:
						if start in id[0][1:]:
							p = id[0].index(bytes((start,)))
							if p < f:
								f = p
							log('Keeping some')
					if f == 0:
						f = 1
					id[0] = id[0][f:]
					return True
			# We have something to handle; cancel the timeout, but keep the serial port open to avoid a reset. (I don't think this even works, but it doesn't hurt.)
			GLib.source_remove(timeout_handle)
			# This printer was running and tried to send an id.  Check the id.
			id[0] = id[0][1:9]
			if id[0] in orphans:
				log('accepting orphan %s on %s' % (''.join('%02x' % x for x in id[0]), port))
				ports[port] = orphans.pop(id[0])
				ports[port].port = port
				def close_port(success, data):
					log('reconnect complete; closing server port')
					printer.close()
				log('reconnecting %s' % port)
				ports[port].call('reconnect', ['admin', port], {}, lambda success, ret: ports[port].call('send_printer', ['admin', None], {}, close_port) if success else close_port)
				broadcast(None, 'port_state', port, 2)
				return False
			run_id = nextid()
			log('accepting unknown printer on port %s (id %s)' % (port, ''.join('%02x' % x for x in run_id)))
			#log('orphans: %s' % repr(tuple(orphans.keys())))
			process = subprocess.Popen((fhs.read_data('driver.py', opened = False), '--cdriver', fhs.read_data('franklin-cdriver', opened = False), '--port', port, '--run-id', run_id, '--allow-system', config['allow-system']) + (('--system',) if fhs.is_system else ()), stdin = subprocess.PIPE, stdout = subprocess.PIPE, close_fds = True)
			ports[port] = Port(port, process, printer, run_id)
			return False
		printer.write(protocol.single['ID'])
		timeout_handle = GLib.timeout_add(500, timeout)
		watcher = GLib.io_add_watch(printer.fileno(), GLib.IO_IN, boot_printer_input)
	# Wait at least a second before sending anything, otherwise the bootloader thinks we might be trying to reprogram it.
	# This is only a problem for RAMPS; don't wait for ports that cannot be RAMPS.
	if 'ACM' in port:
		GLib.timeout_add(1500, part2)
	else:
		part2()
# }}}

def add_port(port): # {{{
	if port in ports:
		log('already existing port %s cannot be added' % port)
		return
	if re.match(config['blacklist'], port) or re.match(config['add-blacklist'], port):
		#log('skipping blacklisted port %s' % port)
		return
	ports[port] = None
	broadcast(None, 'new_port', port)
	broadcast(None, 'port_state', port, 0)
	if autodetect:
		detect(port, 'admin')
# }}}

def remove_port(port): # {{{
	log('removing port %s' % port)
	if port not in ports:
		return
	if ports[port]:
		ports[port].make_orphan()
	del ports[port]
	broadcast(None, 'del_port', port)
# }}}

def disable(role, port): # {{{
	if port not in ports:
		log('not disabling nonexistent port %s' % port)
		return
	p = ports[port]
	if p is None:
		log('not disabling inactive port %s' % port)
		return
	if p is False or p.detecting or p == 0:
		# TODO: Kill things that are in progress.
		pass
	# Forget the printer.  First tell the printer to die
	ports[port] = None
	if p:
		def done(success, ret):
			GLib.source_remove(p.input_handle)
			try:
				p.process.kill()
			except OSError:
				pass
			try:
				p.process.communicate()
			except:
				pass
		p.call('die', (role, 'disabled by user',), {}, done)
	if p not in (None, False):
		broadcast(None, 'del_printer', port)
	broadcast(None, 'port_state', port, 0)
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

def print_done(port, completed, reason): # {{{
	broadcast(None, 'printing', port.port, False)
	if config['done']:
		cmd = config['done']
		cmd = cmd.replace('[[STATE]]', 'completed' if completed else 'aborted').replace('[[REASON]]', reason)
		log('running %s' % cmd)
		p = subprocess.Popen(cmd, stdout = subprocess.PIPE, shell = True, close_fds = True)
		def process_done(fd, cond):
			data = p.stdout.read()
			if data:
				log('Data from completion callback: %s' % repr(data))
				return True
			log('Callback for print completion done; return: %s' % repr(p.wait()))
			return False
		GLib.io_add_watch(p.stdout.fileno(), GLib.IO_IN, process_done)
# }}}

if config['local'] != '':
	add_port('-')

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

# Set default printer. {{{
if ' ' in config['printer']:
	default_printer = config['printer'].rsplit(' ', 1)
else:
	default_printer = (config['printer'], None)
# }}}

def _disconnect(socket, data):
	del Connection.connections[socket.connection.id]
httpd = Server(config['port'], Connection, disconnect_cb = _disconnect, httpdirs = fhs.read_data('html', dir = True, multiple = True), address = config['address'], log = config['log'], tls = tls)

log('Franklin server is running')
websocketd.fgloop()

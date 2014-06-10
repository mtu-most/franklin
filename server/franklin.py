#!/usr/bin/python
# vim: foldmethod=marker :

# Imports and config. {{{
import re
import os
import sys
import math
import random
import websockets
from websockets import log
import xdgbasedir
import glib
import subprocess
import crypt
import time
import serial
import json
import traceback

config = xdgbasedir.config_load(packagename = 'franklin', defaults = {
		'port': 8080,
		'address': '',
		'printer': '',
		'audiodir': xdgbasedir.cache_filename_write(packagename = 'franklin', filename = 'audio', makedirs = False),
		'blacklist': r'/dev/(ptmx|console|tty(printk|S?\d*))$',
		'autodetect': 'True',
		'avrdude': '/usr/bin/avrdude',
		'login': '',
		'passwordfile': '',
		'done': '',
		'printercmd': (tuple(xdgbasedir.data_files_read(packagename = 'franklin', filename = 'printer.py')) + ('',))[0]
	})
# }}}

# Global variables. {{{
httpd = None
default_printer = (None, None)
ports = {}
autodetect = config['autodetect'].lower() == 'true'
blacklist = config['blacklist']
orphans = {}
scripts = {}
queue = {}
laser = serial.Serial('/dev/ttyUSB0', 38400)
# These are defined in printer, but ID is required here.
single = { 'NACK': '\x80', 'ACK0': '\xb3', 'ACKWAIT0': '\xb4', 'STALL': '\x87', 'ACKWAIT1': '\x99', 'ID': '\xaa', 'ACK1': '\xad', 'DEBUG': '\x9e' }
# }}}

# Load scripts. {{{
for d in xdgbasedir.data_files_read('scripts', packagename = 'franklin'):
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

class Server(websockets.RPChttpd): # {{{
	def auth_message(self, connection, is_websocket):
		return 'Please identify yourself' if config['passwordfile'] or config['login'] else None
	def authenticate(self, connection):
		if config['login']:
			if ':' in config['login']:
				user, password = config['login'].split(':', 1)
				if user == connection.data['user'] and password == connection.data['password']:
					return True
			else:
				if connection.data['user'] == config['login']:
					return True
		if config['passwordfile']:
			with open(config['passwordfile']) as f:
				for l in f:
					if ':' not in l:
						continue
					user, password = l.split(':')[:2]
					if user == connection.data['user'] and(password[1:] == connection.data['password'] if password.startswith(':') else password == crypt.crypt(connection.data['password'], password)):
						return True
		return False
	def page(self, connection):
		if 'port' in connection.query:
			port = connection.query['port'][0]
			if port not in ports or not ports[port]:
				self.reply(connection, 404)
			else:
				def export_reply(success, message):
					self.reply(connection, 200, message, 'text/plain;charset=utf8')
					connection.socket.close()
				ports[port].call('export_settings', (), {}, export_reply)
				return True
		else:
			websockets.RPChttpd.page(self, connection)
# }}}

class Connection: # {{{
	connections = {}
	nextid = 0
	def __init__(self, socket): # {{{
		socket.monitor = False
		self.socket = socket
		self.printer = self.find_printer(*default_printer)
		self.id = Connection.nextid
		Connection.nextid += 1
		Connection.connections[self.id] = self
	# }}}
	@classmethod
	def disconnect(cls, socket, data): # {{{
		del connections[id]
	# }}}
	@classmethod
	def _broadcast(cls, target, name, *args): # {{{
		if name == 'spindle':
			laser.write('laser:output %d\nbeep 2\n' % args[0])
			return
		if target is not None:
			#log('broadcasting to target %d' % target)
			if target not in Connection.connections:
				log('ignoring targeted broadcast to missing connection %d' % target)
				return
			target = Connection.connections[target].socket
			if target.monitor:
				#log('%s %s' % (name, repr(args)))
				getattr(target, name).event(*args)
			else:
				log("not broadcasting to target, because it isn't set to monitor")
		elif httpd:
			#log('broadcasting to all')
			for c in httpd.websockets:
				if c.monitor:
					#log('broadcasting to one')
					getattr(c, name).event(*args)
	# }}}
	@classmethod
	def upload(cls, port, board): # {{{
		resumeinfo = [(yield), None]
		cls.disable(port)
		if board == 'melzi':
			protocol = 'arduino'
			baudrate = '115200'
			mcu = 'atmega1284p'
			filename = xdgbasedir.data_files_read(os.path.join('firmware', 'mighty_opt.hex'), packagename = 'franklin')[0]
		elif board == 'ramps':
			protocol = 'wiring'
			baudrate = '115200'
			mcu = 'atmega2560'
			filename = xdgbasedir.data_files_read(os.path.join('firmware', 'mega2560.hex'), packagename = 'franklin')[0]
		elif board == 'mega':
			protocol = 'arduino'
			baudrate = '57600'
			mcu = 'atmega1280'
			filename = xdgbasedir.data_files_read(os.path.join('firmware', 'mega.hex'), packagename = 'franklin')[0]
		else:
			raise ValueError('board type not supported')
		data = ['']
		process = subprocess.Popen([config['avrdude'], '-q', '-q', '-V', '-c', protocol, '-b', baudrate, '-p', mcu, '-P', port, '-U', 'flash:w:' + filename], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.STDOUT, close_fds = True)
		def output(fd, cond):
			data[0] += process.stdout.read()
			if data[0] != '':
				return True
			resumeinfo[0]()
			return False
		glib.io_add_watch(process.stdout, glib.IO_IN | glib.IO_PRI | glib.IO_HUP, output)
		yield websockets.WAIT
		process.communicate()[0]
		if autodetect:
			c = websockets.call(None, cls.detect, port)()
		yield data[0] or 'firmware successfully uploaded'
	# }}}
	@classmethod
	def find_printer(cls, name = None, port = None): # {{{
		for p in ports:
			if not ports[p] or ports[p].name is None:
				continue
			if (name is None or re.match(name, ports[p].name)) and (port is None or re.match(port, p)):
				return p
		return None
	# }}}
	@classmethod
	def set_autodetect(cls, detect): # {{{
		global autodetect
		autodetect = detect
		cls._broadcast(None, 'autodetect', autodetect)
	# }}}
	@classmethod
	def get_autodetect(cls): # {{{
		return autodetect
	# }}}
	@classmethod
	def disable(cls, port): # {{{
		if port not in ports or not ports[port]:
			#log('port is not enabled')
			return
		glib.source_remove(ports[port].input_handle)
		# Forget the printer.  First tell the printer to die
		p = ports[port]
		ports[port] = None
		p.call('die', (), {}, lambda success, ret: None)
		cls._broadcast(None, 'del_printer', port)
	# }}}
	@classmethod
	def detect(cls, port): # {{{
		resumeinfo = [(yield), None]
		#log('detecting printer on %s' % port)
		if port not in ports or ports[port] != None:
			#log('port is not in detectable state')
			return
		ports[port] = False
		websockets.call(resumeinfo, detect, port)
	# }}}
	@classmethod
	def detect_all(cls): # {{{
		resumeinfo = [(yield), None]
		for p in ports:
			if ports[p] is not None:
				continue
			c = websockets.call(resumeinfo, cls.detect, p)
			while c(): c.args = (yield websockets.WAIT)
	# }}}
	@classmethod
	def add_port(cls, port): # {{{
		resumeinfo = [(yield), None]
		if port in ports:
			log('already existing port %s cannot be added' % port)
			return
		if re.match(blacklist, port):
			#log('skipping blacklisted port %s' % port)
			return
		ports[port] = None
		cls._broadcast(None, 'new_port', port);
		if autodetect:
			c = websockets.call(resumeinfo, cls.detect, port)
			while c(): c.args = (yield websockets.WAIT)
	# }}}
	@classmethod
	def remove_port(cls, port): # {{{
		log('removing port %s' % port)
		if port not in ports:
			return
		if ports[port]:
			# Close serial port, in case it still exists.
			cls.disable(port)
		del ports[port]
		cls._broadcast(None, 'del_port', port)
	# }}}
	@classmethod
	def get_ports(cls): # {{{
		return [(p, ports[p].name if ports[p] else None) for p in ports]
	# }}}
	@classmethod
	def set_default_printer(cls, name = None, port = None): # {{{
		global default_printer
		default_printer = (name, port)
	# }}}
	@classmethod
	def get_default_printer(cls): # {{{
		return default_printer
	# }}}
	@classmethod
	def set_blacklist(cls, newlist): # {{{
		global blacklist
		blacklist = newlist
		cls._broadcast(None, 'blacklist', blacklist)
	# }}}
	@classmethod
	def get_blacklist(cls): # {{{
		return blacklist
	# }}}
	@classmethod
	def new_script(cls, code): # {{{
		global nextscriptname
		name = '%04d' % nextscriptname
		scripts[name] = [code, None]
		while '%04d' % nextscriptname in scripts:
			nextscriptname += 1
		with open(xdgbasedir.data_filename_write(os.path.join('scripts', name + os.extsep + 'js'), packagename = 'franklin'), 'wb') as f:
			f.write(code)
		cls._broadcast(None, 'new_script', name, code, None)
		return []
	# }}}
	@classmethod
	def del_script(cls, name): # {{{
		del scripts[name]
		for e in('js', 'dat'):
			filename = xdgbasedir.data_filename_write(os.path.join('scripts', name + os.extsep + e), packagename = 'franklin')
			if os.path.exists(filename):
				os.unlink(filename)
		cls._broadcast(None, 'del_script', name)
	# }}}
	@classmethod
	def set_data(cls, name, data): # {{{
		scripts[name][1] = data
		filename = xdgbasedir.data_filename_write(os.path.join('scripts', name + os.extsep + 'dat'), packagename = 'franklin')
		if data is None:
			if os.path.exists(filename):
				os.unlink(filename)
		else:
			with open(filename, 'wb') as f:
				f.write(data)
		cls._broadcast(None, 'new_data', name, data)
	# }}}
	@classmethod
	def queue_add(cls, data, name): # {{{
		assert name not in queue
		parsed = cls.gcode_parse(data)
		queue[name] = (parsed, cls.gcode_bbox(parsed))
		cls._broadcast(None, 'queue', [(q, queue[q][1]) for q in queue])
		return ''
	# }}}
	@classmethod
	def queue_remove(cls, name): # {{{
		assert name in queue
		del queue[name]
		cls._broadcast(None, 'queue', [(q, queue[q][1]) for q in queue])
	# }}}
	@classmethod
	def gcode_parse(cls, code): # {{{
		mode = None
		message = None
		ret = []
		unit = 1
		rel = False
		erel = None
		pos = [[float('nan'), float('nan'), float('nan')], [0.], float('inf')]
		current_extruder = 0
		for lineno, origline in enumerate(code.split('\n')):
			line = origline.strip()
			if line.startswith('N'):
				r = re.match('N\d+\s*(.*?)\*\d+$')
				if not r:
					# Invalid line; ignore it.
					log('%d:ignoring invalid gcode: %s' % (lineno, origline))
					continue
				line = r.group(1)
			comment = ''
			while '(' in line:
				b = line.index('(')
				e = line.find(')', b)
				if e < 0:
					log('%d:ignoring unterminated comment: %s' % (lineno, origline))
					continue
				comment = line[b + 1:e].strip()
				line = line[:b] + line[e + 1:].strip()
			if ';' in line:
				p = line.index(';')
				comment = line[p + 1:].strip()
				line = line[:p].strip()
			if comment.upper().startswith('MSG,'):
				message = comment[4:].strip()
			if line == '':
				continue
			line = line.split()
			if mode is None or line[0][0] in 'GMT':
				if len(line[0]) < 2:
					log('%d:ignoring unparsable line: %s' % (lineno, origline))
					continue
				try:
					cmd = line[0][0], int(line[0][1:])
				except:
					log('%d:parse error in line: %s' % (lineno, origline))
					traceback.print_exc()
					continue
				line = line[1:]
			else:
				cmd = mode
			args = {}
			for a in line:
				try:
					args[a[0]] = float(a[1:])
				except:
					log('%d:ignoring invalid gcode: %s' % (lineno, origline))
					break
			else:
				if cmd == ('M', 2):
					# Program end.
					break
				elif cmd[0] == 'T':
					target = cmd[1]
					if target >= len(pos[1]):
						pos[1].extend([0.] * (target - len(pos[1]) + 1))
					current_extruder = target
					continue
				elif cmd == ('G', 20):
					unit = 25.4
					continue
				elif cmd == ('G', 21):
					unit = 1
					continue
				elif cmd == ('G', 90):
					rel = False
					continue
				elif cmd == ('G', 91):
					rel = True
					continue
				elif cmd == ('M', 82):
					erel = False
					continue
				elif cmd == ('M', 83):
					erel = True
					continue
				elif cmd == ('G', 92):
					which = {}
					for w in args:
						if w in 'XYZ':
							# Ignore; this is only abused.
							pass
						elif w == 'E':
							pos[1][current_extruder] = args[w]
					continue
				elif cmd[0] == 'M' and cmd[1] in(104, 109, 116):
					args['E'] = current_extruder
				if not((cmd[0] == 'G' and cmd[1] in(0, 1, 4, 28, 81, 94)) or(cmd[0] == 'M' and cmd[1] in(0, 3, 4, 5, 6, 9, 42, 84, 104, 106, 107, 109, 116, 140, 190)) or(cmd[0] in('S', 'T'))):
					log('%d:invalid gcode command %s' % (lineno, repr((cmd, args))))
				elif cmd == ('G', 28):
					ret.append((cmd, args, message))
					for a in range(len(pos[0])):
						pos[0][a] = float('nan')
				elif cmd[0] == 'G' and cmd[1] in(0, 1, 81):
					if cmd[1] != 0:
						mode = cmd
					components = {'X': None, 'Y': None, 'Z': None, 'E': None, 'F': None, 'R': None}
					for c in args:
						assert c in components and components[c] is None
						components[c] = args[c]
					f0 = pos[2]
					if components['F'] is not None:
						pos[2] = components['F'] * unit
					if cmd[1] != 81:
						if components['E'] is not None:
							if erel or(erel is None and rel):
								estep = components['E'] * unit
							else:
								estep = components['E'] * unit - pos[1][current_extruder]
							pos[1][current_extruder] += estep
						else:
							estep = 0
					else:
						estep = 0
						if components['R'] is not None:
							if rel:
								r = pos[0][2] + components['R'] * unit
							else:
								r = components['R'] * unit
					oldpos = pos[0][:]
					for axis in range(3):
						value = components[chr(ord('X') + axis)]
						if value is not None:
							if rel:
								pos[0][axis] += value * unit
							else:
								pos[0][axis] = value * unit
							if axis == 2:
								z = pos[0][2]
					if cmd[1] != 81:
						dist = sum([(pos[0][x] - oldpos[x]) ** 2 for x in range(3)]) ** .5
						if dist == 0:
							dist = abs(estep)
						if dist > 0:
							#if f0 is None:
							#	f0 = pos[1][current_extruder]
							f0 = pos[2]	# Always use new value.
							if f0 == 0:
								f0 = float('inf')
						if math.isnan(dist):
							dist = 0
						args = {'X': pos[0][0], 'Y': pos[0][1], 'Z': pos[0][2], 'E': estep, 'f': f0 / dist / 60 if dist != 0 and cmd[1] == 1 else float('inf'), 'F': pos[2] / dist / 60 if dist != 0 and cmd[1] == 1 else float('inf')}
						cmd = ('G', 1)
						ret.append((cmd, args, message))
					else:
						# Drill cycle.
						# Only support OLD_Z (G90) retract mode; don't support repeats(L).
						# goto x,y
						ret.append((('G', 1), {'X': pos[0][0], 'Y': pos[0][1], 'Z': oldpos[2], 'E': 0, 'f': float('inf'), 'F': float('inf')}, message))
						# goto r
						ret.append((('G', 1), {'X': pos[0][0], 'Y': pos[0][1], 'Z': r, 'E': 0, 'f': float('inf'), 'F': float('inf')}, None))
						# goto z; this is always straight down, because the move before and after it are also vertical.
						if z != r:
							f0 = pos[2] / abs(z - r) / 60
							ret.append((('G', 1), {'X': pos[0][0], 'Y': pos[0][1], 'Z': z, 'E': 0, 'f': f0, 'F': f0}, None))
						# retract; this is always straight up, because the move before and after it are also non-horizontal.
						ret.append((('G', 1), {'X': pos[0][0], 'Y': pos[0][1], 'Z': oldpos[2], 'E': 0, 'f': float('inf'), 'F': float('inf')}, None))
						# empty move; this makes sure the previous move is entirely vertical.
						ret.append((('G', 1), {'X': pos[0][0], 'Y': pos[0][1], 'Z': oldpos[2], 'E': 0, 'f': float('inf'), 'F': float('inf')}, None))
						# Set up current z position so next G81 will work.
						pos[0][2] = oldpos[2]
				else:
					ret.append((cmd, args, message))
				message = None
		return ret
	# }}}
	@classmethod
	def gcode_bbox(cls, code): # {{{
		ret = [[None, None], [None, None], [None, None]]
		def inspect(value, box):
			if math.isnan(value):
				return
			if box[0] is None or value < box[0]:
				box[0] = value
			if box[1] is None or value > box[1]:
				box[1] = value
		for cmd, args, message in code:
			if tuple(cmd) != ('G', 1):
				continue
			inspect(args['X'], ret[0])
			inspect(args['Y'], ret[1])
			inspect(args['Z'], ret[2])
		return ret
	# }}}

	def set_printer(self, printer = None, port = None): # {{{
		self.printer = self.find_printer(printer, port)
	# }}}
	def get_printer(self): # {{{
		return self.printer.name if self.printer is not None else None
	# }}}
	def status(self): # {{{
		resumeinfo = [(yield), None]
		assert self.printer is not None
		c = websockets.call(resumeinfo, self.printer.printer.readtemp_temp, 0)
		while c(): c.args = (yield websockets.WAIT)
		t = c.ret()
		c = websockets.call(resumeinfo, self.printer.printer.readtemp_extruder, 0)
		while c(): c.args = (yield websockets.WAIT)
		yield(t, c.ret())
	# }}}
	def set_monitor(self, value): # {{{
		self.socket.monitor = value
		if self.socket.monitor:
			self.socket.autodetect.event(autodetect)
			self.socket.blacklist.event(blacklist)
			for p in ports:
				self.socket.new_port.event(p)
				if ports[p]:
					ports[p].call('send_printer', [self.id], {}, lambda success, data: None)
			for s in scripts:
				Connection._broadcast(self.id, 'new_script', s, scripts[s][0], scripts[s][1])
			Connection._broadcast(self.id, 'queue', [(q, queue[q][1]) for q in queue])
	# }}}
	def get_monitor(self): # {{{
		return self.socket.monitor
	# }}}
	def queue_print(self, names, ref = (0, 0), angle = 0, probemap = None): # {{{
		resumeinfo = [(yield), None]
		assert self.printer is not None
		Connection._broadcast(None, 'printing', self.printer, True)
		for n in names:
			c = websockets.call(resumeinfo, self.gcode_run, queue[n][0], ref, angle, probemap)
			while c(): c.args = (yield websockets.WAIT)
		print_done(ports[self.printer], *c.ret())
		if c.ret():
			yield c.ret()
	# }}}
	def queue_probe(self, names, ref = (0, 0), angle = 0, density = (4, 4)): # {{{
		resumeinfo = [(yield), None]
		assert self.printer is not None
		bbox = [float('nan'), (float('nan')), float('nan'), float('nan')]
		for n in names:
			bb = queue[n][1]
			if not bbox[0] < bb[0]:
				bbox[0] = bb[0]
			if not bbox[1] < bb[1]:
				bbox[1] = bb[1]
			if not bbox[2] > bb[2]:
				bbox[2] = bb[2]
			if not bbox[3] > bb[3]:
				bbox[3] = bb[3]
			c = websockets.call(resumeinfo, self.gcode_run, queue[n][0], ref, angle, probemap)
			while c(): c.args = (yield websockets.WAIT)
			if c.ret():
				yield c.ret()
	# }}}
	def _call (self, name, a, ka): # {{{
		resumeinfo = [(yield), None]
		assert self.printer is not None
		#log('other: %s' % attr)
		ports[self.printer].call(name, a, ka, lambda success, ret: resumeinfo[0](ret))
		yield (yield websockets.WAIT)
	# }}}
	def __getattr__ (self, attr): # {{{
		return lambda *a, **ka: self._call(attr, a, ka)
	# }}}
# }}}

class Port: # {{{
	def __init__(self, port, process, id): # {{{
		self.name = None
		self.port = port
		self.process = process
		self.id = id
		self.waiters = ({}, {}, {})
		self.next_mid = 0
		self.input_handle = glib.io_add_watch(process.stdout.fileno(), glib.IO_IN | glib.IO_HUP, self.printer_input)
		old = []
		def get_settings(success, settings):
			self.call('import_settings', [settings], {}, lambda success, ret: None)
			glib.source_remove(orphans[old[0]].input_handle)
			orphans[old[0]].call('die', (), {}, lambda success, ret: None)
			del orphans[old[0]]
		def get_vars(success, vars):
			self.name = vars['name']
			# Copy settings from orphan with the same name, then kill the orphan.
			old[:] = [x for x in orphans if orphans[x].name == self.name]
			if len(old) > 0:
				orphans[old[0]].call('export_settings', (), {}, get_settings)
		self.call('send_printer', [None], {}, lambda success, data: self.call('get_variables', (), {}, get_vars))
	# }}}
	def call(self, name, args, kargs, cb): # {{{
		data = json.dumps([self.next_mid, name, args, kargs]) + '\n'
		#log('calling %s on %d' % (repr(data), self.process.stdin.fileno()))
		try:
			self.process.stdin.write(data)
		except IOError:
			log('killing printer handle because of IOError')
			traceback.print_exc()
			Connection.disable(self.port)
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
		if line == '':
			log('%s died.' % self.name)
			self.process.wait()	# Clean up the zombie.
			for t in range(3):
				for w in self.waiters[t]:
					self.waiters[t][w](False, 'Printer died')
			Connection.disable(self.port)
			if autodetect:
				websockets.call(None, Connection.detect, self.port)()
			return False
		data = json.loads(line)
		#log('printer input:' + repr(data))
		if data[1] == 'broadcast':
			Connection._broadcast(data[2], data[3], self.port, *(data[4:]))
		elif data[1] == 'disconnect':
			# Don't remember a printer that hasn't sent its name yet.
			port = self.port
			ports[self.port] = None
			if self.name is not None:
				# If there already is an orphan with the same name, kill the old orphan.
				for o in [x for x in orphans if orphans[x].name == self.name]:
					# This for loop always runs 0 or 1 times, never more.
					del orphans[x]
				orphans[self.id] = self
			Connection._broadcast(None, 'del_printer', port)
			if autodetect:
				websockets.call(None, Connection.detect, self.port)()
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
# }}}

def detect(port): # {{{
	if not os.path.exists(port):
		log("not detecting on %s, because file doesn't exist." % port)
		return False
	log('detecting on %s' % port)
	try:
		printer = serial.Serial(port, baudrate = 115200, timeout = 0)
	except serial.SerialException:
		return False
	# flush buffer.
	while printer.read() != '':
		pass
	# We need to get the printer id first.  If the printer is booting, this can take a while.
	id = [None, None]
	# Wait to make sure the command is interpreted as a new packet.
	def part2():
		id[0] = ''
		id[1] = 10
		# How many times to try before accepting an invalid id.
		# This is required, because the ID is not protected with a checksum.
		def timeout():
			# Timeout.  Give up.
			glib.source_remove(watcher)
			log('Timeout waiting for printer on port %s; giving up.' % port)
			return False
		def boot_printer_input(fd, cond):
			while len(id[0]) < 9:
				id[0] += printer.read(9 - len(id[0]))
				if len(id[0]) < 9:
					printer.write(single['ID'])
					return True
				if id[0][0] != single['ID']:
					log('skip non-id: %s %s' % (' '.join(['%02x' % ord(x) for x in id[0]]), id[0]))
					if single['ID'] in id[0]:
						id[0] = id[0][id[0].index(single['ID']):]
						log('keeping some: %s %s' % (' '.join(['%02x' % ord(x) for x in id[0]]), id[0]))
					else:
						id[0] = ''
						log('discarding all')
			if not all([x in str_id_map for x in id[0][1:]]) and not all([ord(x) == 0 for x in id[0][1:]]):
				log('received invalid id: %s' % id[0]) #' '.join(['%02x' % ord(x) for x in id[0][1:]]))
				if id[1] > 0:
					id[1] -= 1
					id[0] = id[0][9:]
					printer.write(single['ID'])
					return True
				else:
					log('accepting it anyway')
			# We have something to handle; cancel the timeout and close the serial port.
			glib.source_remove(timeout_handle)
			printer.close()
			# This printer was running and tried to send an id.  Check the id.
			id[0] = id[0][1:]
			if id[0] in orphans:
				log('accepting orphan %s on %s' % (repr(id[0]), port))
				ports[port] = orphans[id[0]]
				ports[port].call('reconnect', [port], {}, lambda success, ret: ports[port].call('send_printer', [None], {}, lambda success, data: None))
				return False
			log('accepting unknown printer on port %s (id was %s)' % (port, repr(id[0])))
			id[0] = nextid()
			log('new id %s' % repr(id[0]))
			process = subprocess.Popen((config['printercmd'], port, config['audiodir'], id[0]), stdin = subprocess.PIPE, stdout = subprocess.PIPE, close_fds = True)
			ports[port] = Port(port, process, id[0])
			# TODO: restore settings of orphan with the same name.
			return False
		printer.write(single['ID'])
		timeout_handle = glib.timeout_add(15000, timeout)
		watcher = glib.io_add_watch(printer.fileno(), glib.IO_IN, boot_printer_input)
	glib.timeout_add(150, part2)
# }}}

def nextid(): # {{{
	global last_id
	# 0x23456789 is an arbitrary number with bits set in every nybble, that
	# is odd(so it doesn't visit the same number twice until it did all of
	# them, because it loops at 2**32, which is not divisible by anything
	# except 2).
	last_id = (last_id + 0x23456789) & 0xffffffff
	return ''.join([chr(id_map[(last_id >> (4 * c)) & 0xf]) for c in range(8)])
last_id = random.randrange(1 << 32)
# Parity table is [0x8b, 0x2d, 0x1e]; half of these codes overlap with codes from the single command map; those single commands are not used.
id_map = [0x40, 0xe1, 0xd2, 0x73, 0x74, 0xd5, 0xe6, 0x47, 0xf8, 0x59, 0x6a, 0xcb, 0xcc, 0x6d, 0x5e, 0xff]
str_id_map = ''.join([chr(x) for x in id_map])
# }}}

def print_done(port, completed, reason): # {{{
	Connection._broadcast(None, 'printing', port.port, False)
	if config['done']:
		cmd = config['done']
		cmd = cmd.replace('[[STATE]]', 'completed' if completed else 'aborted').replace('[[REASON]]', reason)
		log('running %s' % cmd)
		p = subprocess.Popen(cmd, stdout = subprocess.PIPE, shell = True, close_fds = True)
		def process_done(fd, cond):
			data = p.stdout.read()
			if data:
				return True
			log('Flashing done; return: %s' % repr(p.wait()))
			return False
		glib.io_add_watch(p.stdout.fileno(), glib.IO_IN, process_done)
# }}}

if autodetect: # {{{
	# Assume a GNU/Linux system; if you have something else, you need to come up with a way to iterate over all your serial ports and implement it here.  Patches welcome, especially if they are platform-independent.
	if os.path.exists('/sys/class/tty'):
		for tty in os.listdir('/sys/class/tty'):
			websockets.call(None, Connection.add_port, '/dev/' + tty) ()
# }}}

# Set default printer. {{{
if ' ' in config['printer']:
	default_printer = config['printer'].rsplit(' ', 1)
else:
	default_printer = (config['printer'], None)
# }}}

httpd = Server(config['port'], Connection, disconnect_cb = Connection.disconnect, httpdirs = xdgbasedir.data_files_read('html', packagename = 'franklin'), address = config['address'])

log('running')
websockets.fgloop()

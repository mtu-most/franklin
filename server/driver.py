#!/usr/bin/python3
# vim: set foldmethod=marker fileencoding=utf8 redrawtime=5000 :
# Python parts of the host side driver for Franklin. {{{
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
# }}}

show_own_debug = False
#show_own_debug = True

# Imports.  {{{
import fhs
import websocketd
from websocketd import log
import serial
import time
import math
import struct
import os
import re
import sys
import wave
import sys
import io
import base64
import json
import fcntl
import select
import subprocess
import traceback
import signal
import atexit
import protocol
import mmap
import random
import errno
import shutil
import cdriver
import numpy
# }}}

# Constants {{{
C0 = 273.15	# Conversion between K and °C
WAIT = object()	# Sentinel for blocking functions.
NUM_SPACES = 3	# Position, extruders, followers
record_format = '=Bi' + 'd' * 11 + 'q' # type, tool, X[3], h[3], Jg, tf, v0, E, time, line
# Space types
type_names = []
# }}}

fhs.option('allow-system', 'Regular expression of allowed system commands', default = '')
fhs.option('uuid', 'Machine uuid')
config = fhs.init(packagename = 'franklin')

# Load space type modules {{{
modulepath = fhs.read_data(os.path.join('type', 'types.txt'), opened = False)
moduledir = os.path.dirname(modulepath)
typeinfo = {}
#typeinfo['h_bot'] = {
#		'title': 'H-Bot',
#		'name': (['r', None], ['θ', '°'], ['z', None]),	// Axis names and units. None means default units.
#		'min-axes': 3
#		'space': [{
#			'name': 'radius',
#			'title': Radius',
#			'unit': None,	# None means default units.
#			'type': 'float',
#			'default': 0.,
#			'digits': 2,
#			'scale': 1.,
#			'help': 'Rotation of the machine'}
#		],
#		'axis': [],
#		'motor': [],
#	}

# Config file format:
'''
# Global values must be specified before any parameters.
# This is the default. Set to different value to allow using a comma as part of the name or unit.
seperator = ,
motor-names = r,θ,z
# Values which are not specified use the system default unit (normally mm).
motor-units = ,°,

# Define a parameter. This is for a property that has one instance.
# All possible paramters are specified here. Only title and help are required.
# Everything else has sensible defaults.
# Type can be float (the default), int, string, or motor.
# For the motor type, unit, default, digits and scale are ignored.
[space]
name = Angle
help = Rotation of the machine.
unit = °
type = float
default = 0
digits = 2
scale = 1000

# More space parameters can follow. Every parameter definition starts with a name.

[motor]
title = Radius
help = Distance from the center to this apex.
default = 125
digits = 1

[axis]
title = Offset
help = ...
'''

# If a value contains only [-0-9], it is an int; otherwise it is a float. 'inf' and 'nan' are supported.
# Digits are counted and used for displaying in the interface.
# Scale is parsed and used for displaying in the interface; default value is also multiplied by it.

for m in open(modulepath):
	#log('reading info for type ' + m.strip())
	separator = ','
	title = m.strip()
	module = re.sub('[^a-z0-9]', '_', title.lower())
	with open(os.path.join(moduledir, module, module + os.extsep + 'ini')) as f:
		section = None
		ret = {'title': title, 'space': [], 'axis': [], 'motor': [], 'name': [], 'min-axes': None}
		for ln in f:
			if ln.strip() == '' or ln.strip().startswith('#'):
				continue
			r = re.match(r'^\s*(?:\[(space|axis|motor)\]|(separator|min-axes|motor-names|motor-units|title|name|help|unit|type|default|digits|scale)\s*=\s*(.*?))\s*$', ln)
			assert r is not None
			# 1: space|axis|motor		new section
			# 2: separator|...|scale	key
			# 3: .*?			value
			if r.group(1) is not None:
				# New section.
				section = r.group(1)
				continue
			key = r.group(2)
			value = r.group(3).strip()
			if section is None:
				if key == 'separator':
					assert len(value) > 0
					separator = value
				elif key == 'motor-names':
					names = value.split(separator)
					if len(ret['name']) < len(names):
						ret['name'] += [[None, None] for _ in range(len(names) - len(ret['name']))]
					for m in range(len(names)):
						ret['name'][m][0] = names[m].strip()
				elif key == 'motor-units':
					units = value.split(separator)
					if len(ret['name']) < len(units):
						ret['name'] += [[None, None] for _ in range(len(units) - len(ret['name']))]
					for m in range(len(units)):
						ret['name'][m][1] = units[m].strip()
				elif key == 'min-axes':
					assert ret['min-axes'] is None
					ret['min-axes'] = int(value)
				else:
					raise AssertionError('Invalid key in global section')
			else:
				if key == 'title':
					assert len(ret[section]) == 0 or 'help' in ret[section][-1]
					ret[section].append({'unit': None, 'type': 'float', 'default': 0, 'digits': 0, 'scale': 1, 'name': value.lower()})
				assert len(ret[section]) > 0
				if key in ('name', 'title', 'help', 'unit', 'type'):
					if key == 'type':
						assert value in ('int', 'float', 'motor', 'string')
					ret[section][-1][key] = value
				elif key == 'digits':
					ret[section][-1][key] = int(value)
				elif key == 'scale':
					ret[section][-1][key] = float(value)
				elif key == 'default':
					if re.match(r'^[-0-9]+$', value):
						ret[section][-1][key] = int(value)
					else:
						ret[section][-1][key] = float(value)
				else:
					raise AssertionError('Invalid key in section %s' % section)
	typeinfo[module] = ret
	type_names.append(module)
# }}}

cdriver.init(fhs.read_data('franklin-cdriver', opened = False).encode('utf-8'), (moduledir + os.sep).encode('utf-8'))

# Enable code trace. {{{
if False:
	def trace(frame, why, arg):
		if why == 'call':
			code = frame.f_code
			log('call: %d %s' % (code.co_firstlineno, code.co_name))
	sys.settrace(trace)
# }}}

# Traceback at hangup. {{{
if False:
	def handle_exit(signum, stack):
		#time.sleep(1)
		log('exit from signal %d' % signum)
		traceback.print_stack()
	for sig in dir(signal):
		value = getattr(signal, sig)
		if sig.startswith('SIG') and not sig.startswith('SIG_') and isinstance(value, int):
			if sig in ('SIGSTOP', 'SIGKILL'):
				log('not handling %s' % sig)
				continue
			log('setting signal handler for %s: %d' % (sig, value))
			signal.signal(value, handle_exit)
		else:
			log('not setting signal handler for %s: %s' % (sig, repr(value)))
	atexit.register(handle_exit, 0, None)
# }}}

fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

def dprint(x, data): # {{{
	if show_own_debug:
		log('%s: %s' % (x, ' '.join(['%02x' % c for c in data])))
# }}}

# Decorator for functions which block.
def delayed(f): # {{{
	def ret(self, *a, **ka):
		#log('delayed called with args %s,%s' % (repr(a), repr(ka)))
		def wrap(id):
			#log('wrap called with id %s' % (repr(id)))
			return f(self, id, *a, **ka)
		return (WAIT, wrap)
	return ret
# }}}

# Reading and writing pins to and from ini files. {{{
def read_pin(machine, pin):
	extra = 0
	if pin.startswith('X'):
		pin = pin[1:]
		if pin == '':
			return 0
	else:
		extra += 256
	if pin.startswith('-'):
		extra += 512
		pin = pin[1:]
	try:
		pin = int(pin)
	except:
		log('incorrect pin %s' % pin)
		return 0
	if pin >= len(machine.pin_names):
		machine.pin_names.extend([[0xf, '(Pin %d)' % i] for i in range(len(machine.pin_names), pin + 1)])
	return pin + extra

def write_pin(pin):
	if pin == 0:
		return 'X'
	ret = ''
	if pin >= 512:
		ret += '-'
		pin -= 512
	if pin >= 256:
		pin -= 256
	else:
		ret = 'X' + ret
	return ret + '%d' % pin
# }}}

class Machine: # {{{
	# Internal stuff.  {{{
	def _read_data(self, data): # {{{
		cmd, s, m, e, f = struct.unpack('=BLLLd', data[:21])
		return cmd, s, m, f, e, data[21:]
	# }}}
	def _send(self, *data): # {{{
		#log('writing to server: %s' % repr(data))
		sys.stdout.write(json.dumps(data) + '\n')
		sys.stdout.flush()
	# }}}
	def _refresh_queue(self, target = None): # {{{
		if self.uuid is None:
			return
		spool = fhs.read_spool(self.uuid, dir = True, opened = False)
		if spool is None:
			return
		gcode = os.path.join(spool, 'gcode')
		audio = os.path.join(spool, 'audio')
		if os.path.isdir(gcode):
			self.jobqueue = {}
			for filename in os.listdir(gcode):
				name, ext = os.path.splitext(filename)
				if ext != os.extsep + 'bin':
					log('skipping %s' % filename)
					continue
				try:
					#log('opening %s' % filename)
					with open(os.path.join(gcode, filename), 'rb') as f:
						f.seek(-8 * 7, os.SEEK_END)
						self.jobqueue[name] = struct.unpack('=' + 'd' * 7, f.read())
				except:
					traceback.print_exc()
					log('failed to open gcode file %s' % os.path.join(gcode, filename))
					os.unlink(os.path.join(gcode, filename))
			sortable_queue = [(q, self.jobqueue[q]) for q in self.jobqueue]
			sortable_queue.sort()
			self._broadcast(target, 'queue', sortable_queue)
		if os.path.isdir(audio):
			self.audioqueue = {}
			for filename in os.listdir(audio):
				name, ext = os.path.splitext(filename)
				if ext != os.extsep + 'bin':
					log('skipping %s' % filename)
					continue
				try:
					#log('opening audio %s' % filename)
					self.audioqueue[name] = os.stat(os.path.join(audio, filename)).st_size
				except:
					traceback.print_exc()
					log('failed to stat audio file %s' % os.path.join(audio, filename))
			sortable_queue = list(self.audioqueue.keys())
			sortable_queue.sort()
			self._broadcast(target, 'audioqueue', sortable_queue)
	# }}}
	def __init__(self, allow_system): # {{{
		self.initialized = False
		self.connected = False
		self.uuid = config['uuid']
		# Break string in parts to avoid fold markers.
		self.user_interface = '{Dv2m(Blocker:){Dv2m(No Connection:){dv3m{dv3m{dv3m[0:*Controls:{Dh57.1%{Dv18.9m{Dv8.8m{dh17m(Job Control:)(Buttons:)}(Position:)}{Dh85%(XY Map:)(Z Map:)}}{Dv4.4m(Abort:){Dv8.3m(Multipliers:){Dv3.6m(Gpios:){Dv7.4m(Temps:)(Temp Graph:)}}' + '}}' + '}Setup:{Dv2m(Save Profile:)[2:Profile:(Profile Setup:)Probe:(Probe Setup:)*Globals:(Globals Setup:)Axes:(Axis Setup:)Motors:(Motor Setup:)Type:{Dv3m(Type Setup:){Dh50%(Delta Setup:)(Polar Setup:)}}Extruder:(Extruder Setup:)Follower:(Follower Setup:)GPIO:(Gpio Setup:)Temps:(Temp Setup:)]}](Confirmation:)}(Message:)}(State:)}}' + '}'
		self.pin_names = []
		self.allow_system = allow_system
		self.job_current = None
		self.job_id = None
		self.confirm_id = 0
		self.confirm_message = None
		self.confirm_axes = None
		self.confirmer = None
		self.position_valid = False
		self.probing = False
		self.parking = False
		self.home_phase = None
		self.home_order = None
		self.home_target = None
		self.home_cb = self._do_home
		self.gcode_file = False
		self.gcode_map = None
		self.gcode_id = None
		self.gcode_waiting = 0
		self.audio_id = None
		self.confirm_waits = set()
		self.gpio_waits = {}
		self.total_time = float('nan')
		self.resuming = False
		self.debug_buffer = None
		self.command_buffer = ''
		self.bed_id = -1
		self.fan_id = -1
		self.spindle_id = -1
		self.probe_enable = False
		self.probe_points = []
		self.probemap = None
		self.probe_pending = False
		self.probe_cb = None
		self.probe_speed = 3.
		self.probe_dist = 1000
		self.probe_offset = 0
		self.probe_safe_dist = 10
		self.unit_name = 'mm'
		self.park_after_job = True
		self.sleep_after_job = True
		self.cool_after_job = True
		self.spi_setup = []
		# Set up state.
		self.spaces = [self.Space(self, i) for i in range(NUM_SPACES)]
		self.temps = []
		self.gpios = []
		self.sending = False
		self.paused = False
		self.limits = [{} for s in self.spaces]
		self.moving = False
		self.movecb = []
		self.tempcb = []
		self.alarms = set()
		self.targetangle = 0.
		self.store_adc = False
		self.temp_scale_min = 0
		self.temp_scale_max = 250
		self.multipliers = []
		self.current_extruder = 0
		try:
			assert self.uuid is not None # Don't try reading if there is no uuid given.
			with fhs.read_data(os.path.join(self.uuid, 'info' + os.extsep + 'txt')) as pfile:
				self.name = pfile.readline().rstrip('\n')
				self.profile = pfile.readline().rstrip('\n')
			#log('profile is %s' % self.profile)
		except:
			#log("No default profile; using 'default'.")
			self.name = self.uuid
			self.profile = 'default'
		profiles = self.list_profiles()
		if self.profile not in profiles and len(profiles) > 0:
			self.profile = profiles[0]
			#log('Profile does not exist; using %s instead' % self.profile)
		self.default_profile = self.profile
		# Globals.
		self.queue_length = 0
		self.num_pins = 0
		self.led_pin = 0
		self.stop_pin = 0
		self.probe_pin = 0
		self.spiss_pin = 0
		self.pattern_step_pin = 0
		self.pattern_dir_pin = 0
		self.timeout = float('inf')
		self.bed_id = -1
		self.fan_id = -1
		self.spindle_id = -1
		self.feedrate = 1
		self.max_deviation = 0
		self.max_v = 100
		self.max_a = 10000
		self.max_J = 10000
		self.adjust_speed = 1
		self.current_extruder = 0
		# Other things don't need to be initialized, because num_* == 0.
		# Fill job queue.
		self.jobqueue = {}
		self.audioqueue = {}
		self._refresh_queue()
		try:
			self.user_load(update = False)
		except:
			log('Failed to import initial settings')
			traceback.print_exc()
		global show_own_debug
		if show_own_debug is None:
			show_own_debug = True
	# }}}
	# Constants.  {{{
	# Single-byte commands.
	single = {'OK': b'\xb3', 'WAIT': b'\xad' }
	# }}}
	def _broadcast(self, *a): # {{{
		self._send(None, 'broadcast', *a)
	# }}}
	def _close(self, notify = True): # {{{
		log('disconnecting')
		self.connected = False
		if notify:
			self._send(None, 'disconnect')
		self._globals_update()
	# }}}
	def _command_input(self): # {{{
		data = sys.stdin.read()
		if data == '':
			log('End of file detected on command input; exiting.')
			traceback.print_stack()
			sys.exit(0)
		self.command_buffer += data
		die = None
		#log('cmd buf %s' % repr(self.command_buffer))
		while '\n' in self.command_buffer:
			pos = self.command_buffer.index('\n')
			id, func, a, ka = json.loads(self.command_buffer[:pos])
			self.command_buffer = self.command_buffer[pos + 1:]
			try:
				#log('command: %s(%s %s)' % (func, a, ka))
				assert not any(func.startswith(x + '_') for x in ('benjamin', 'admin', 'expert', 'user', 'remote'))
				role = a.pop(0) + '_'
				if hasattr(self, role + func):
					func = role + func
				elif role == 'benjamin_' and hasattr(self, 'admin_' + func):
					func = 'admin_' + func
				elif role in ('benjamin_', 'admin_') and hasattr(self, 'expert_' + func):
					func = 'expert_' + func
				elif role in ('benjamin_', 'admin_', 'expert_') and hasattr(self, 'user_' + func):
					func = 'user_' + func
				ret = getattr(self, func)(*a, **ka)
				if isinstance(ret, tuple) and len(ret) == 2 and ret[0] is WAIT:
					# The function blocks; it will send its own reply later.
					if ret[1] is WAIT:
						# Special case: request to die.
						die = id
					else:
						ret[1](id)
						continue
			except SystemExit:
				log('exiting as requested by server (1)')
				traceback.print_stack()
				sys.exit(0)
			except:
				log('error handling command input')
				traceback.print_exc()
				self._send(id, 'error', repr(sys.exc_info()))
				continue
			if ret != (WAIT, WAIT):
				#log('returning %s' % repr(ret))
				self._send(id, 'return', ret)
		if die is not None:
			self._send(die, 'return', None)
			log('exiting as requested by server (2)')
			traceback.print_stack()
			sys.exit(0)
	# }}}
	def _trigger_movewaits(self, done = True): # {{{
		#log('movewaits triggered: %s' % repr(self.movecb))
		call_queue.extend([(x, [done]) for x in self.movecb])
		self.movecb = []
		self.moving = False
	# }}}
	def _machine_input(self, reply = False): # {{{
		cmd = cdriver.get_interrupt()
		#log('received interrupt: %s' % repr(cmd))
		if cmd['type'] == 'move-cb':
			self._trigger_movewaits()
		elif cmd['type'] == 'temp-cb':
			self.alarms.add(cmd['temp'])
			t = 0
			while t < len(self.tempcb):
				if self.tempcb[t][0] is None or self.tempcb[t][0] in self.alarms:
					call_queue.append((self.tempcb.pop(t)[1], []))
				else:
					t += 1
		elif cmd['type'] == 'limit':
			if self.home_phase is None:
				if not 0 <= cmd['space'] < NUM_SPACES or not 0 <= cmd['motor'] < len(self.spaces[cmd['space']].motor):
					button = 'the emergency stop button'
				else:
					button = 'the limit switch for motor %s' % self.spaces[cmd['space']].motor_name(cmd['motor'])
				log('Move stopped because %s was hit.' % button)
				self._broadcast(None, 'message', 'Move stopped because %s was hit.' % button)
			if cmd['space'] < len(self.spaces) and cmd['motor'] < len(self.spaces[cmd['space']].motor):
				self.limits[cmd['space']][cmd['motor']] = cmd['pos']
			self._trigger_movewaits(False)
		elif cmd['type'] == 'timeout':
			self.position_valid = False
			call_queue.append((self._globals_update, ()))
			for i, t in enumerate(self.temps):
				if not math.isnan(t.value):
					t.value = float('nan')
					call_queue.append((self._temp_update, (i,)))
			for i, g in enumerate(self.gpios):
				if g.state != g.reset:
					g.state = g.reset
					call_queue.append((self._gpio_update, (i,)))
		elif cmd['type'] == 'pinchange':
			self.gpios[cmd['pin']].value = cmd['state']
			call_queue.append((self._gpio_update, (cmd['pin'],)))
			if cmd['pin'] in self.gpio_waits:
				for id in self.gpio_waits[cmd['pin']]:
					self._send(id, 'return', None)
				del self.gpio_waits[cmd['pin']]
		elif cmd['type'] == 'homed':
			for i in range(len(self.homed_pos)):
				self.homed_pos[i] += cmd['position'][i]
			call_queue.append((self._do_home, [True]))
		elif cmd['type'] == 'disconnect':
			self._broadcast(None, 'message', 'Machine disconnected. Reason: ' + ('unknown' if len(cmd['reason']) == 0 else cmd['reason']))
			self._close()
		elif cmd['type'] == 'update-temp':
			if cmd['temp'] < len(self.temps):
				self.temps[cmd['temp']].value = cmd['value'] - C0
				self._temp_update(cmd['temp'])
			else:
				log('Ignoring updated invalid temp %d' % cmd['temp'])
		elif cmd['type'] == 'update-pin':
			self.gpios[cmd['pin']].state = cmd['state']
			call_queue.append((self._gpio_update, (cmd['pin'],)))
		elif cmd['type'] == 'confirm':
			if cmd['tool-changed'] and self.probemap is not None and self.probe_enable:
				self.probe_pending = True
			call_queue.append((self.user_request_confirmation(cmd['message'] or 'Continue?')[1], (False,)))
		elif cmd['type'] == 'park':
			call_queue.append((self.user_park(cb = cdriver.resume)[1], (None,)))
		elif cmd['type'] == 'file-done':
			call_queue.append((self._job_done, (True, 'completed')))
		elif cmd['type'] == 'pinname':
			if cmd['pin'] >= len(self.pin_names):
				self.pin_names.extend([[0xf, '(Pin %d)' % i] for i in range(len(self.pin_names), cmd['pin'] + 1)])
			self.pin_names[cmd['pin']] = [cmd['mode'], cmd['name'].decode('utf-8', 'replace')]
			#log('pin name {} = {}'.format(cmd['pin'], self.pin_names[cmd['pin']]))
		elif cmd['type'] == 'connected':
			def sync():
				# Initialize the machine.
				self._write_globals(update = False)
				for i, s in enumerate(self.spaces):
					s.write_info()
					for a in range(len(s.axis)):
						s.write_axis(a)
					for m in range(len(s.motor)):
						s.write_motor(m)
				for i, t in enumerate(self.temps):
					t.write()
					# Disable heater.
					self.user_settemp(i, float('nan'), update = False)
					# Disable heater alarm.
					self.user_waittemp(i, None, None)
				for i, g in enumerate(self.gpios):
					g.write()
				# The machine may still be doing things.  Pause it and send a move; this will discard the queue.
				self.user_pause(True, False, update = False)[1](None)
				if self.spi_setup:
					self._spi_send(self.spi_setup)
				self.connected = True
				self._globals_update()
			call_queue.append((sync, ()))
		else:
			log('unexpected packet %02x' % cmd)
			raise AssertionError('Received unknown interrupt packet type')
	# }}}
	def _read_globals(self, update = True): # {{{
		data = cdriver.read_globals()
		num_temps = data.pop('num_temps')
		num_gpios = data.pop('num_gpios')
		#log('read globals:', data)
		for k in data:
			setattr(self, k, data[k])
		while len(self.temps) < num_temps:
			self.temps.append(self.Temp(len(self.temps)))
			if update:
				self.temps[-1].read()
		self.temps = self.temps[:num_temps]
		while len(self.gpios) < num_gpios:
			self.gpios.append(self.Gpio(len(self.gpios)))
			if update:
				self.gpios[-1].read()
		self.gpios = self.gpios[:num_gpios]
		return True
	# }}}
	def _write_globals(self, nt = None, ng = None, update = True): # {{{
		if nt is None:
			nt = len(self.temps)
		if ng is None:
			ng = len(self.gpios)
		dt = nt - len(self.temps)
		dg = ng - len(self.gpios)
		data = {'num_temps': nt, 'num_gpios': ng}
		data.update({x:getattr(self, x) for x in ('led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'pattern_step_pin', 'pattern_dir_pin', 'probe_enable', 'bed_id', 'fan_id', 'spindle_id', 'feedrate', 'max_deviation', 'max_v', 'max_a', 'max_J', 'adjust_speed', 'timeout', 'current_extruder', 'targetangle', 'store_adc')})
		#log('writing globals: %s' % repr(data))
		cdriver.write_globals(data)
		self._read_globals(update = True)
		if update:
			self._globals_update()
			for t in range(dt):
				self._temp_update(nt - dt + t)
			for g in range(dg):
				self._gpio_update(ng - dg + g)
		return True
	# }}}
	def _mangle_spi(self): # {{{
		ret = []
		for bits, data in self.spi_setup:
			ret.append('%d:%s' % (bits, ','.join('%02x' % x for x in data)))
		return ';'.join(ret)
	# }}}
	def _unmangle_spi(self, data): # {{{
		ret = []
		if len(data) > 0:
			for p in data.split(';'):
				bits, data = p.split(':')
				bits = int(bits)
				data = [int(x, 16) for x in data.split(',')]
				ret.append([bits, data])
		return ret
	# }}}
	def _globals_update(self, target = None): # {{{
		if not self.initialized:
			return
		attrnames = ('name', 'profile', 'user_interface', 'pin_names', 'led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'pattern_step_pin', 'pattern_dir_pin', 'probe_dist', 'probe_offset', 'probe_safe_dist', 'bed_id', 'fan_id', 'spindle_id', 'unit_name', 'probe_enable', 'feedrate', 'max_deviation', 'max_v', 'max_a', 'max_J', 'adjust_speed', 'timeout', 'targetangle', 'store_adc', 'park_after_job', 'sleep_after_job', 'cool_after_job', 'temp_scale_min', 'temp_scale_max', 'probe_points', 'connected')
		attrs = {n: getattr(self, n) for n in attrnames}
		attrs['num_temps'] = len(self.temps)
		attrs['num_gpios'] = len(self.gpios)
		attrs['spi_setup'] = self._mangle_spi()
		# Status is True if printing, False if paused, None if there is no current job.
		attrs['status'] = not self.paused and (self.gcode_file or None)
		self._broadcast(target, 'globals_update', attrs)
	# }}}
	def _space_update(self, which, target = None): # {{{
		if not self.initialized:
			return
		if which >= len(self.spaces):
			# This can happen if this function is scheduled before changing the number of spaces.
			return
		self._broadcast(target, 'space_update', which, self.spaces[which].export())
	# }}}
	def _temp_update(self, which, target = None): # {{{
		if not self.initialized:
			return
		if which >= len(self.temps):
			# This can happen if this function is scheduled before changing the number of temps.
			return
		self._broadcast(target, 'temp_update', which, self.temps[which].export())
	# }}}
	def _gpio_update(self, which, target = None): # {{{
		if not self.initialized:
			return
		if which >= len(self.gpios):
			# This can happen if this function is scheduled before changing the number of gpios.
			return
		self._broadcast(target, 'gpio_update', which, self.gpios[which].export())
	# }}}
	def _gcode_close(self): # {{{
		self.gcode_strings = []
		self.gcode_map.close()
		os.close(self.gcode_fd)
		self.gcode_map = None
		self.gcode_file = False
		self.gcode_fd = -1
	# }}}
	def _job_done(self, complete, reason): # {{{
		cdriver.run_file()
		if self.gcode_file:
			log('job done: ' + reason)
			self._gcode_close()
		#traceback.print_stack()
		if self.gcode_id is not None:
			log('Job done (%d): %s' % (complete, reason))
			self._send(self.gcode_id, 'return', (complete, reason))
			self.gcode_id = None
		if self.audio_id is not None:
			log('Audio done (%d): %s' % (complete, reason))
			self._send(self.audio_id, 'return', (complete, reason))
		self.audio_id = None
		if self.job_current is not None:
			if self.job_id is not None:
				self._send(self.job_id, 'return', (complete, reason))
			self.job_id = None
			self.job_current = None
			if complete:
				self._finish_done()
		if self.home_phase is not None:
			#log('killing homer')
			self.home_phase = None
			self.expert_set_space(0, type = self.home_orig_type, module = self.home_orig_module[0])
			for a, ax in enumerate(self.spaces[0].axis):
				self.expert_set_axis((0, a), min = self.home_limits[a][0], max = self.home_limits[a][1], module = self.home_orig_module[1][a])
			for m, mtr in enumerate(self.spaces[0].motor):
				self.expert_set_motor((0, m), module = self.home_orig_module[2][m])
			if self.home_cb in self.movecb:
				self.movecb.remove(self.home_cb)
				if self.home_id is not None:
					self._send(self.home_id, 'return', None)
		if self.probe_cb in self.movecb:
			#log('killing prober')
			self.movecb.remove(self.probe_cb)
			self.probe_cb(None)
		self._globals_update()
	# }}}
	def _finish_done(self): # {{{
		if self.cool_after_job:
			for t in range(len(self.temps)):
				self.user_settemp(t, float('nan'))
		def maybe_sleep():
			if self.sleep_after_job:
				self.user_sleep()
		if self.park_after_job:
			self.user_park(cb = maybe_sleep)[1](None)
		else:
			maybe_sleep()
	# }}}
	def _unpause(self): # {{{
		'''Stop being paused. Don't resume.'''
		self.paused = False
		cdriver.unpause()
		self._globals_update()
	# }}}
	def _queue_add(self, filename, name): # {{{
		name = os.path.splitext(os.path.split(name)[1])[0]
		origname = name
		i = 0
		while name == '' or name in self.jobqueue:
			name = '%s-%d' % (origname, i)
			i += 1
		infilename = filename.encode('utf-8', 'replace')
		outfiledir = fhs.write_spool(os.path.join(self.uuid, 'gcode'), dir = True)
		if not os.path.isdir(outfiledir):
			os.makedirs(outfiledir)
		outfilename = os.path.join(outfiledir, name + os.path.extsep + 'bin').encode('utf-8', 'replace')
		self._broadcast(None, 'blocked', 'Parsing g-code')
		errors = cdriver.parse_gcode(infilename, outfilename)
		self._refresh_queue()
		self._broadcast(None, 'blocked', None)
		return name + '\n' + json.dumps(errors)
	# }}}
	def _audio_add(self, f, name): # {{{
		name = os.path.splitext(os.path.split(name)[1])[0]
		origname = name
		i = 0
		while name == '' or name in self.audioqueue:
			name = '%s-%d' % (origname, i)
			i += 1
		try:
			wav = wave.open(f)
		except:
			return 'Unable to open audio file'
		rate = wav.getframerate()
		channels = wav.getnchannels()
		self._broadcast(None, 'blocked', 'Parsing audio')
		data = wav.readframes(wav.getnframes())
		# Data is 16 bit signed ints per channel, but it is read as bytes.  First convert it to 16 bit numbers.
		data = [(h << 8) + l if h < 128 else(h << 8) + l -(1 << 16) for l, h in zip(data[::2 * channels], data[1::2 * channels])]
		bit = 0
		byte = 0
		with fhs.write_spool(os.path.join(self.uuid, 'audio', name + os.path.extsep + 'bin'), text = False) as dst:
			dst.write(struct.pack('@d', rate))
			for t, sample in enumerate(data):
				if sample > 0:
					byte |= 1 << bit
				bit += 1
				if bit >= 8:
					dst.write(bytes((byte,)))
					byte = 0
					bit = 0
		self.audioqueue[os.path.splitext(name)[0]] = wav.getnframes()
		self._broadcast(None, 'blocked', '')
		self._broadcast(None, 'audioqueue', list(self.audioqueue.keys()))
		return ''
	# }}}
	def _do_home(self, done = None): # {{{
		'''Home the machine.
		Steps to do:
		None: ignore; return (may happen after home is aborted).
		'start': Set up everything
		For each order id that is present, in ascending order:
			move position to switch, including followers with switch on the same side; wait for limit: 'main-limit', or
			if a leader or follower with its partners on the same side hit a switch, move only its partners until they also hits their switches: 'follow-limit'
			continue until all switches are hit
		Move all motors away from their switches: 'calibrate'
		Synchronize followers: 'synchronize'
		Move to opposite followers: 'opposite'
		Move away from switches: 'follow-calibrate'
		Synchronize followers: 'follow-synchronize'
		Move to second home position: 'home2'
		Move head within limits: 'finish'
		Finish up.
		'''

		#log('home %s %s %s' % (self.home_phase, repr(self.home_order), done))
		home_v = 50 / self.feedrate
		if self.home_phase is None:
			#log('_do_home ignored because home_phase is None')
			return
		if self.home_phase == 'start':
			# Initial call; start homing.
			# If it is currently moving, doing the things below without pausing causes stall responses.
			self.user_pause(True, False)[1](None)
			self.user_sleep(False)
			self.homed_pos = [-self.spaces[0].get_current_pos(m)[1] for m in range(len(self.spaces[0].motor))]
			for l in self.limits:
				l.clear()
			gpio_motors = set()
			for g, gpio in enumerate(self.gpios):
				space = gpio.leader & 0xf
				motor = gpio.leader >> 4
				if 0 <= space <= 1 and 0 <= motor < len(self.spaces[space].motor):
					gpio_motors.add((space, motor))
			# Set all extruders to 0, except those which have a gpio linked to them (those are moved to the position later).
			for i, e in enumerate(self.spaces[1].axis):
				if (1, i) not in gpio_motors:
					self.user_set_axis_pos(1, i, 0)
			self.home_limits = [(a['min'], a['max']) for a in self.spaces[0].axis]
			for a, ax in enumerate(self.spaces[0].axis):
				self.expert_set_axis((0, a), min = float('-inf'), max = float('inf'))
			self.home_orig_type = self.spaces[0].type
			self.home_orig_module = [self.get_space(0)['module'], [self.get_axis(0, a)['module'] for a in range(len(self.spaces[0].axis))], [self.get_motor(0, m)['module'] for m in range(len(self.spaces[0].motor))]]
			self.expert_set_space(0, type = type_names[0])
			self.home_order = {'standard': {}, 'opposite': [], 'homing': {}, 'single': []}
			followers_used = {'standard': set(), 'opposite': []}
			for m, mtr in enumerate(self.spaces[0].motor):
				can_use_pos = self._pin_valid(mtr['limit_max_pin'])
				can_use_neg = self._pin_valid(mtr['limit_min_pin'])
				if can_use_pos or can_use_neg:
					# gpio_motors is a list of motors that should move to their home position without a limit switch, so remove motors with a limit switch from the list.
					if (0, m) in gpio_motors:
						gpio_motors.remove((0, m))
					followers = []
					for fm, fmtr in enumerate(self.spaces[2].motor):
						if self.spaces[2].follower[fm]['leader'] == m << 4:
							followers.append({'leader': m, 'motor': (2, fm), 'min': self._pin_valid(fmtr['limit_min_pin']), 'max': self._pin_valid(fmtr['limit_max_pin'])})
					if not can_use_neg:
						use_pos = True
					elif not can_use_pos:
						use_pos = False
					else:
						num_pos = sum(f['max'] for f in followers)
						num_neg = sum(f['min'] for f in followers)
						use_pos = num_pos >= num_neg
					if mtr['home_order'] not in self.home_order['standard']:
						self.home_order['standard'][mtr['home_order']] = {}
					# Use f.update() or f to add the field to the dict, but also use the dict as the expression.
					use_followers = {f['motor']: f.update({'positive': use_pos}) or f for f in followers if f['max' if use_pos else 'min']}
					self.home_order['standard'][mtr['home_order']][m] = {'motor': (0, m), 'positive': use_pos, 'followers': use_followers}
					followers_used['standard'].update(use_followers[f]['motor'][1] for f in use_followers)
					for f in followers:
						if f['motor'][1] in followers_used['standard']:
							continue
						if any(self._pin_valid(f[limit]) for limit in ('limit_min_pin', 'limit_max_pin')):
							self.home_order['opposite'].append({'motor': f['motor'], 'positive': self._pin_valid(f['limit_max_pin']), 'leader': m})
							followers_used['opposite'].append(f['motor'][1])
				else:
					if (0, m) not in gpio_motors:
						self.user_set_axis_pos(0, m, mtr['home_pos'])
			self.home_phase = 'main-limit-start'
			if len(gpio_motors) > 0:
				# Move motors with a gpio follower to their home position.
				target = [{}, {}]
				for s, m in gpio_motors:
					target[s][m] = self.spaces[s].motor[m]['home_pos']
				if len(target[1]) == 1:
					# move position and single extruder.
					self.movecb.append(self.home_cb)
					self.user_line(target[0], tool = target[1].keys()[0], e = target[1].values()[0], force = True)[1](None)
					# Immediately fall through to main-limit.
					return
				else:
					# move position.
					self.movecb.append(self.home_cb)
					self.user_line(target[0], force = True)[1](None)
					# move each extruder.
					if len(target[1]) > 0:
						self.home_phase = 'gpio'
						self.home_gpio = target[1]
					return
			# Fall through (to main-limit).
		if self.home_phase == 'gpio':
			m = self.home_gpio.pop()
			if len(self.home_gpio) == 0:
				self.home_phase = 'main-limit-start'
			self.movecb.append(self.home_cb)
			self.user_line(tool = m, e = self.spaces[1].motor[m]['home_pos'], force = True)[1](None)
			return
		while True:	# Allow code below to repeat from here.
			if self.home_phase == 'main-limit-start':
				done = False	# Fake incomplete move to avoid aborting home.
				self.home_phase = 'main-limit'
			if self.home_phase == 'main-limit':
				# Find out what the situation is. Which motors should still be moved, and should we now move all or a single motor?
				if len(self.home_order['standard']) > 0:
					current = self.home_order['standard'][min(self.home_order['standard'])]
					if done is True:
						# We moved dist and still didn't arrive at any switch. Give up.
						log('Warning: limits were not hit during home: {}'.format(current))
						# No targets left to move to.
						self.home_phase = 'calibrate'
					elif done is False:
						#log('hit limit %s %s current %s' % (self.limits[0], self.limits[2], current))
						# A limit was hit. Find out which one and start single moves if so needed.
						for m in self.limits[0]:
							# Complete recording of move until limit switch (was started when homing began).
							self.homed_pos[m] += self.spaces[0].get_current_pos(m)[1]
							# Set new position.
							self.user_set_axis_pos(0, m, self.spaces[0].motor[m]['home_pos'])
							# Start recording move away from limit switch (finished when HOMED is received).
							self.homed_pos[m] -= self.spaces[0].get_current_pos(m)[1]
							if m in current:
								if len(current[m]['followers']) > 0:
									self.home_order['single'] = current[m]['followers']
									self.home_order['leader'] = m
									self.home_phase = 'follow-limit'
									#log('leader hit first; single = {}'.format(self.home_order['single']))
								#else:
									#log('solo hit')
								self.home_order['homing'][(0, m)] = current.pop(m)
						self.limits[0].clear()
						for fm in self.limits[2]:
							self.user_set_axis_pos(2, fm, self.spaces[2].motor[fm]['home_pos'])
							for m in current:
								if (2, fm) in current[m]['followers']:
									self.home_order['homing'][(2, fm)] = current[m]['followers'][(2, fm)]
									self.home_order['single'] = {(0, m): current[m]}
									self.home_order['single'].update(current[m]['followers'])
									self.home_order['leader'] = m
									self.home_order['homing'][(2, fm)] = self.home_order['single'].pop((2, fm))
									self.home_phase = 'follow-limit'
									#log('follower hit first; single = {}'.format(self.home_order['single']))
						self.limits[2].clear()
					else:
						# This is the first time we are here. Start the move.
						pass
					# If the home phase is still 'main-limit', move all motors.
					if self.home_phase == 'main-limit':
						dist = 10000 #TODO: use better value.
						self.home_target = {m: dist if current[m]['positive'] else -dist for m in current}
						if len(self.home_target) > 0:
							self.movecb.append(self.home_cb)
							self.user_line(self.home_target, v = home_v * len(self.home_target) ** .5, single = False, force = True, relative = True)[1](None)
							return
						else:
							# Done with this phase.
							self.home_phase = 'calibrate'
							break
				else:
					# No targets left to move to.
					self.home_phase = 'calibrate'
					break
				# Fall through.
			if self.home_phase == 'follow-limit':
				for m in self.limits[0]:
					self.user_set_axis_pos(0, m, self.spaces[0].motor[m]['home_pos'])
					if (0, m) in self.home_order['single']:
						self.home_order['homing'][(0, m)] = self.home_order['single'].pop((0, m))
						current = self.home_order['standard'][min(self.home_order['standard'])]
						current.pop(m)
				self.limits[0].clear()
				for m in self.limits[2]:
					self.user_set_axis_pos(2, m, self.spaces[2].motor[m]['home_pos'])
					if (2, m) in self.home_order['single']:
						self.home_order['homing'][(2, m)] = self.home_order['single'].pop((2, m))
				self.limits[2].clear()
				if len(self.home_order['single']) > 0:
					target = tuple(self.home_order['single'])[0]
					target_obj = self.home_order['single'][target]
					self.movecb.append(self.home_cb)
					if target[0] == 0:
						self.user_line({target[1]: self.home_target[target[1]]}, relative = True, force = True, single = True)[1](None)
					else:
						self.user_line(e = self.home_target[target_obj['leader']], tool = ~target[1], relative = False, force = True, single = True)[1](None)
					return
				# Done with these followers, clean up and move on.
				self.home_phase = 'main-limit'
				continue	# Restart at phase 'main-limit'.
			break
		if self.home_phase == 'calibrate':
			# Move all motors away from their switches
			data = []
			num = 0
			for m, mtr in enumerate(self.spaces[0].motor):
				if (0, m) not in self.home_order['homing']:
					data.append(0)
				else:
					num += 1
					data.append(-1 if self.home_order['homing'][(0, m)]['positive'] else 1)
			data.extend([0] * len(self.spaces[1].motor))
			for m, mtr in enumerate(self.spaces[2].motor):
				if (2, m) not in self.home_order['homing']:
					data.append(0)
				else:
					num += 1
					data.append(-1 if self.home_order['homing'][(2, m)]['positive'] else 1)
			self.home_phase = 'synchronize'
			if num > 0:
				cdriver.home(*data)
				return
			# Fall through.
		if self.home_phase == 'synchronize':
			# Homing finished; set motor positions.
			self.home_order['sync'] = {}
			for s, m in self.home_order['homing']:
				self.user_set_axis_pos(s, m, self.spaces[s].motor[m]['home_pos'])
				if s == 0:
					leader = m
				else:
					leader = self.spaces[s].follower[m]['leader'] >> 4
				if leader not in self.home_order['sync']:
					self.home_order['sync'][leader] = {}
				self.home_order['sync'][leader][(s, m)] = self.spaces[s].motor[m]
			# Move motors to be synchronized.
			for leader in self.home_order['sync']:
				if self.home_order['homing'][(0, leader)]['positive']:
					self.home_order['sync'][leader][None] = min(self.home_order['sync'][leader][x]['home_pos'] for x in self.home_order['sync'][leader])
				else:
					self.home_order['sync'][leader][None] = max(self.home_order['sync'][leader][x]['home_pos'] for x in self.home_order['sync'][leader])
			self.home_order['leader'] = [[l, list(x for x in self.home_order['sync'][l] if x is not None and self.home_order['sync'][l][x]['home_pos'] != self.home_order['sync'][l][None])] for l in list(self.home_order['sync'])]
			#log('cleanup: {}'.format(self.home_order['leader']))
			while len(self.home_order['leader']) > 0 and len(self.home_order['leader'][0][1]) == 0:
				self.home_order['leader'].pop(0)
			self.home_phase = 'opposite'
			if len(self.home_order['leader']) > 0:
				target = self.home_order['sync'][self.home_order['leader'][0][0]][None]
				m = self.home_order['leader'][0][1].pop(0)
				self.movecb.append(self.home_cb)
				#log('m={}'.format(m))
				if m[0] == 0:
					self.user_line({m[1]: target}, relative = False, force = True, single = True)[1](None)
				else:
					self.user_line(tool = ~target[1], e = target, relative = False, force = True, single = True)[1](None)
				return
			# Fall through.
		if self.home_phase == 'opposite':
			while len(self.home_order['leader']) > 0 and len(self.home_order['leader'][0][1]) == 0:
				self.home_order['leader'].pop(0)
			if len(self.home_order['leader']) > 0:
				target = self.home_order['sync'][self.home_order['leader']][0][0]
				m = self.home_order['leader'][0][1].pop(0)
				self.movecb.append(self.home_cb)
				if m[0] == 0:
					self.user_line({m[1]: target}, relative = False, force = True, single = True)[1](None)
				else:
					self.user_line(tool = ~target[1], e = target, relative = False, force = True, single = True)[1](None)
				return
			# Move to opposite followers
			# for m in self.home_order['opposite']: TODO
			# Move away from switches: 5

			# Reset space type and move to pos2.
			self.expert_set_space(0, type = self.home_orig_type, module = self.home_orig_module[0])
			for a, ax in enumerate(self.spaces[0].axis):
				self.expert_set_axis((0, a), min = self.home_limits[a][0], max = self.home_limits[a][1], module = self.home_orig_module[1][a])
			for m, mtr in enumerate(self.spaces[0].motor):
				self.expert_set_motor((0, m), module = self.home_orig_module[2][m])
			target = {}
			for i, a in enumerate(self.spaces[0].axis):
				if not math.isnan(a['home_pos2']):
					target[i] = a['home_pos2'] - a['offset']
			self.home_phase = 'home2'
			if len(target) > 0:
				self.movecb.append(self.home_cb)
				self.user_line(target, force = True)[1](None)
				return
			# Fall through.
		if self.home_phase == 'home2':
			# Move within bounds.
			target = {}
			for i, a in enumerate(self.spaces[0].axis):
				current = self.spaces[0].get_current_pos(i)[0]
				if current > a['max'] - a['offset']:
					target[i] = a['max'] - a['offset']
				elif current < a['min'] - a['offset']:
					target[i] = a['min'] - a['offset']
			self.home_phase = 'finish'
			if len(target) > 0:
				self.movecb.append(self.home_cb)
				self.user_line(target, force = True)[1](None)
				return
			# Fall through.
		if self.home_phase == 'finish':
			self.home_phase = None
			self.position_valid = True
			if self.home_id is not None:
				orig_motor_pos = [m['home_pos'] - self.homed_pos[i] for i, m in enumerate(self.spaces[0].motor)]
				orig_pos = self.motors2xyz(orig_motor_pos)
				self._send(self.home_id, 'return', orig_pos)
			if self.home_done_cb is not None:
				call_queue.append((self.home_done_cb, []))
				self.home_done_cb = None
			return
		log('Internal error: invalid home phase %s' % self.home_phase)
	# }}}
	def _handle_one_probe(self, good): # {{{
		if good is None:
			return
		pos = self.get_axis_pos(0)
		cdriver.adjust_probe(*(pos[a] + self.spaces[0].axis[a].offset for a in range(3)))
		self.probe_cb = lambda good: self.user_request_confirmation("Continue?")[1](False) if good is not None else None
		self.movecb.append(self.probe_cb)
		self.user_line({2: self.probe_safe_dist}, relative = True)[1](None)
	# }}}
	def _one_probe(self): # {{{
		self.probe_cb = self._handle_one_probe
		self.movecb.append(self.probe_cb)
		z = self.get_axis_pos(0, 2)
		z_low = self.spaces[0].axis[2]['min']
		self.user_line({2: z_low}, f0 = float(self.probe_speed) / (z - z_low) if z > z_low else float('inf'), probe = True)[1](None)
	# }}}
	def _gcode_run(self, src, abort = True, paused = False): # {{{
		if self.parking:
			return
		self.gcode_angle = math.sin(self.targetangle), math.cos(self.targetangle)
		if 0 <= self.bed_id < len(self.temps):
			self.btemp = self.temps[self.bed_id].value
		else:
			self.btemp = float('nan')
		if abort:
			self._unpause()
			self._job_done(False, 'aborted by starting new job')
		# Disable all alarms.
		for i in range(len(self.temps)):
			self.user_waittemp(i, None, None)
		self.user_sleep(False)
		if len(self.spaces) > 1:
			for e in range(len(self.spaces[1].axis)):
				self.user_set_axis_pos(1, e, 0)
		filename = fhs.read_spool(os.path.join(self.uuid, 'gcode', src + os.extsep + 'bin'), text = False, opened = False)
		self.total_time = self.jobqueue[src][-1]
		self.gcode_fd = os.open(filename, os.O_RDONLY)
		self.gcode_map = mmap.mmap(self.gcode_fd, 0, prot = mmap.PROT_READ)
		filesize = os.fstat(self.gcode_fd).st_size
		bboxsize = 7 * struct.calcsize('=d')
		def unpack(format, pos):
			return struct.unpack(format, self.gcode_map[pos:pos + struct.calcsize(format)])
		num_strings = unpack('=I', filesize - bboxsize - struct.calcsize('=I'))[0]
		self.gcode_strings = []
		sizes = [unpack('=I', filesize - bboxsize - struct.calcsize('=I') * (num_strings + 1 - x))[0] for x in range(num_strings)]
		first_string = filesize - bboxsize - struct.calcsize('=I') * (num_strings + 1) - sum(sizes)
		pos = 0
		for x in range(num_strings):
			self.gcode_strings.append(self.gcode_map[first_string + pos:first_string + pos + sizes[x]].decode('utf-8', 'replace'))
			pos += sizes[x]
		self.gcode_num_records = first_string / struct.calcsize(record_format)
		self.gcode_file = True
		log('running %s %f %f' % (filename, self.gcode_angle[0], self.gcode_angle[1]))
		cdriver.run_file(filename.encode('utf-8'), 1 if not paused and self.confirmer is None else 0, self.gcode_angle[0], self.gcode_angle[1])
		self.paused = paused
		self._globals_update()
	# }}}
	def _reset_extruders(self, axes): # {{{
		for i, sp in enumerate(axes):
			for a, pos in enumerate(sp):
				# Assume motor[a] corresponds to axis[a] if it exists.
				if len(self.spaces[i].motor) > a and not self._pin_valid(self.spaces[i].motor[a]['limit_max_pin']) and not self._pin_valid(self.spaces[i].motor[a]['limit_min_pin']):
					self.user_set_axis_pos(i, a, pos)
	# }}}
	def _pin_valid(self, pin):	# {{{
		return (pin & 0x100) != 0
	# }}}
	def _spi_send(self, data): # {{{
		for bits, p in data:
			shift = (8 - bits % 8) % 8
			if shift > 0:
				p = [(p[b] << shift | p[b + 1] >> (8 - shift)) & 0xff for b in range(len(p) - 1)] + [(p[-1] << shift) & 0xff]
			cdriver.spi(bits, b''.join(struct.pack('=B', b) for b in p))
	# }}}
	def admin_connect(self, port, run_id): # {{{
		cdriver.connect_machine(bytes([ord(x) for x in run_id]), port.encode('utf-8'))
		# The rest happens in response to the CONNECTED reply.
	# }}}
	def admin_reconnect(self, port): # {{{
		pass
	# }}}
	# Subclasses.  {{{
	class Space: # {{{
		def __init__(self, machine, id):
			self.name = ['position', 'extruders', 'followers'][id]
			self.type = type_names[id]
			self.machine = machine
			self.id = id
			self.axis = []
			self.motor = []
			self.module = {item['name']: item['default'] for item in typeinfo[self.type]['space']}
			self.extruder = []
			self.follower = []
		def parse_info(self, data, info):
			'''Decode data from cdriver
			data is ([ints], [floats], [strings])'''
			int_num = 0
			float_num = 0
			string_num = 0
			ret = {}
			for item in info:
				if item['type'] == 'float':
					ret[item['name']] = data[1][float_num]
					float_num += 1
				elif item['type'] == 'string':
					ret[item['name']] = data[2][string_num]
					string_num += 1
				else:
					ret[item['name']] = data[0][int_num]
					int_num += 1
			return ret
		def build_module_data(self, data, info):
			'''Prepare data for sending to cdriver
			data is a single list, in the order of info'''
			ret = []
			for item in info:
				if item['name'] in data:
					value = data[item['name']]
				else:
					value = item['default']
				if item['type'] == 'float':
					ret.append(float(value))
				elif item['type'] == 'string':
					ret.append(str(value))
				else:
					ret.append(int(value))
			return ret
		def read(self):
			self.read_info()
			for a in range(len(self.axis)):
				self.read_axis(a)
			for m in range(len(self.motor)):
				self.read_motor(m)
		def read_info(self):
			data = cdriver.read_space_info(self.id)
			old_type = self.type
			self.type = type_names[data.pop('type')]
			if old_type != self.type:
				self.module = self.parse_info(data['module'], typeinfo[self.type]['space'])
			num_axes = data.pop('num_axes')
			num_motors = data.pop('num_motors')
			if self.id == 1:
				self.machine.multipliers = (self.machine.multipliers + [1.] * num_axes)[:num_axes]
				while len(self.extruder) < num_axes:
					self.extruder.append({})
				self.extruder = self.extruder[:num_axes]
			if self.id == 2:
				while len(self.follower) < num_axes:
					self.follower.append({})
				self.follower = self.follower[:num_axes]
			if num_axes > len(self.axis):
				def nm(i):
					if self.id == 0:
						if i < 3:
							return chr(ord('x') + i)
						elif i < 6:
							return chr(ord('a') + i - 3)
						else:
							return 'Axis %d' % i
					elif self.id == 1:
						return 'extruder %d' % i
					else:
						return 'follower %d' % i
				for i in range(len(self.axis), num_axes):
					self.axis.append({'name': nm(i), 'park': float('nan'), 'park_order': 0, 'min': float('nan'), 'max': float('nan'), 'offset': 0., 'home_pos2': float('nan')})
					if old_type == self.type:
						self.axis[i]['module'] = {item['name']: item['default'] for item in typeinfo[self.type]['axis']}
			else:
				while len(self.axis) > num_axes:
					self.axis.pop(-1)
			if old_type != self.type:
				for a, axis in enumerate(self.axis):
					self.axis[a]['module'] = {item['name']: item['default'] for item in typeinfo[self.type]['axis']}
			if num_motors > len(self.motor):
				for i in range(len(self.motor), num_motors):
					self.motor.append({'step_pin': 0, 'dir_pin': 0, 'enable_pin': 0, 'limit_min_pin': 0, 'limit_max_pin': 0, 'steps_per_unit': 100., 'home_pos': 0., 'limit_v': float('inf'), 'limit_a': float('inf'), 'home_order': 0, 'unit': self.machine.unit_name})
					if old_type == self.type:
						self.motor[i]['module'] = {item['name']: item['default'] for item in typeinfo[self.type]['motor']}
			else:
				while len(self.motor) > num_motors:
					self.motor.pop(-1)
			if old_type != self.type:
				for m, motor in enumerate(self.motor):
					self.motor[m]['module'] = {item['name']: item['default'] for item in typeinfo[self.type]['motor']}
			module_data = data.pop('module')
			self.module = self.parse_info(module_data, typeinfo[self.type]['space'])
		def read_axis(self, a):
			data = cdriver.read_space_axis(self.id, a, type_names.index(self.type))
			module_data = data.pop('module')
			self.axis[a]['module'] = self.parse_info(module_data, typeinfo[self.type]['axis'])
			for k in data:
				self.axis[a][k] = data[k]
		def read_motor(self, m):
			data = cdriver.read_space_motor(self.id, m, type_names.index(self.type))
			module_data = data.pop('module')
			self.motor[m]['module'] = self.parse_info(module_data, typeinfo[self.type]['motor'])
			for k in data:
				self.motor[m][k] = data[k]
			if self.id == 1 and m < len(self.machine.multipliers):
				self.motor[m]['steps_per_unit'] /= self.machine.multipliers[m]
		def write_info(self, num_axes = None):
			if num_axes is None:
				num_axes = len(self.axis)
			if typeinfo[self.type]['min-axes'] is not None and num_axes < typeinfo[self.type]['min-axes']:
				num_axes = typeinfo[self.type]['min-axes']
			data = {'type': type_names.index(self.type) if self.type in type_names else 0, 'num_axes': num_axes}
			data['module'] = self.build_module_data(self.module, typeinfo[self.type]['space'])
			cdriver.write_space_info(self.id, data)
		def write_axis(self, axis):
			data = {'park_order': 0, 'park': float('nan'), 'min': float('-inf'), 'max': float('inf'), 'offset': 0.}
			if self.id == 0:
				for k in data.keys():
					data[k] = self.axis[axis][k]
			data['module'] = self.build_module_data(self.axis[axis]['module'], typeinfo[self.type]['axis'])
			#log('write space axis', self.id, axis, data, type_names.index(self.type))
			cdriver.write_space_axis(self.id, axis, data, type_names.index(self.type))
		def write_motor(self, motor):
			if self.id == 2:
				leader = self.follower[motor]['leader']
				if leader is None or (leader & 0xf) >= len(self.machine.spaces) or (leader >> 4) >= len(self.machine.spaces[self.follower[motor]['space']].motor):
					#log('write motor for follower %d with fake base' % motor)
					base = {'steps_per_unit': 1, 'limit_v': float('inf'), 'limit_a': float('inf')}
				else:
					#log('write motor for follower %d with base %s' % (motor, self.machine.spaces[0].motor))
					base = self.machine.spaces[leader & 0xf].motor[leader >> 4]
			else:
				base = self.motor[motor]
			# don't include unit, because cdriver doesn't use it.
			data = {x: self.motor[motor][x] for x in ('step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'home_pos', 'home_order')}
			data.update({x: base[x] for x in ('limit_v', 'limit_a', 'steps_per_unit')})
			data['module'] = self.build_module_data(self.motor[motor]['module'], typeinfo[self.type]['motor'])
			if self.id == 1 and motor < len(self.machine.multipliers):
				data['steps_per_unit'] *= self.machine.multipliers[motor]
			cdriver.write_space_motor(self.id, motor, data, type_names.index(self.type))
		def set_current_pos(self, axis, pos):
			#log('setting pos of %d %d to %f' % (self.id, axis, pos))
			cdriver.setpos(self.id, axis, pos)
		def get_current_pos(self, axis):
			#log('getting current pos %d %d' % (self.id, axis))
			if not self.machine.connected:
				return (float('nan'), float('nan'))
			return cdriver.getpos(self.id, axis)
		def motor_name(self, i):
			if i < len(typeinfo[self.type]['name']):
				name = typeinfo[self.type]['name'][i][0]
				if name is not None:
					return name
			if i < len(self.axis):
				return self.axis[i]['name']
			return 'motor %d' % i
		def export(self):
			std = [self.name, self.type, [[a['name'], a['park'], a['park_order'], a['min'], a['max'], a['offset'], a['home_pos2']] for a in self.axis], [[self.motor_name(i), m['step_pin'], m['dir_pin'], m['enable_pin'], m['limit_min_pin'], m['limit_max_pin'], m['steps_per_unit'], m['home_pos'], m['limit_v'], m['limit_a'], m['home_order'], m['unit']] for i, m in enumerate(self.motor)], None if self.id != 1 else self.machine.multipliers]
			for a, data in enumerate(std[2]):
				data.append(self.axis[a]['module'])
			for m, data in enumerate(std[3]):
				data.append(self.motor[m]['module'])
			return std + [self.module]
		def export_settings(self):
			# Things to handle specially while homing:
			# * self.home_limits = [(a['min'], a['max']) for a in self.spaces[0].axis]
			# * self.home_orig_type = self.spaces[0].type
			ret = '[space %d]\r\n' % self.id
			type = self.type if self.id != 0 or self.machine.home_phase is None else self.machine.home_orig_type
			if self.id == 0:
				ret += 'type = %s\r\n' % type
			ret += 'num_axes = %d\r\n' % len(self.axis)
			for key in self.module:
				value = self.module[key]
				ret += '%s-%s = %s\r\n' % (self.type, key, value)
			for i, a in enumerate(self.axis):
				ret += '[axis %d %d]\r\n' % (self.id, i)
				ret += 'name = %s\r\n' % a['name']
				if self.id == 0:
					ret += ''.join(['%s = %f\r\n' % (x, a[x]) for x in ('park', 'offset', 'home_pos2')])
					ret += 'park_order = %d\r\n' % a['park_order']
					if self.machine.home_phase is None:
						ret += ''.join(['%s = %f\r\n' % (x, a[x]) for x in ('min', 'max')])
					else:
						ret += ''.join(['%s = %f\r\n' % (x, y) for x, y in zip(('min', 'max'), self.machine.home_limits[self.id])])
				for key in a['module']:
					value = a['module'][key]
					ret += '%s-%s = %s\r\n' % (self.type, key, value)
			for i, m in enumerate(self.motor):
				ret += '[motor %d %d]\r\n' % (self.id, i)
				ret += ''.join(['%s = %s\r\n' % (x, write_pin(m[x])) for x in ('step_pin', 'dir_pin', 'enable_pin')])
				if self.id != 1:
					ret += ''.join(['%s = %s\r\n' % (x, write_pin(m[x])) for x in ('limit_min_pin', 'limit_max_pin')])
					ret += ''.join(['%s = %f\r\n' % (x, m[x]) for x in ('home_pos',)])
					ret += ''.join(['%s = %d\r\n' % (x, m[x]) for x in ('home_order',)])
				if self.id != 2:
					ret += ''.join(['%s = %s\r\n' % (x, m[x]) for x in ('unit',)])
					ret += ''.join(['%s = %f\r\n' % (x, m[x]) for x in ('steps_per_unit', 'limit_v', 'limit_a')])
				for key in m['module']:
					value = m['module'][key]
					ret += '%s-%s = %s\r\n' % (self.type, key, value)
			return ret
	# }}}
	class Temp: # {{{
		def __init__(self, id):
			self.name = 'temp %d' % id
			self.id = id
			self.value = float('nan')
		def read(self):
			data = cdriver.read_temp(self.id)
			try:
				self.Rc = math.exp(data.pop('logRc'))
			except:
				self.Rc = float('nan')
			if not math.isnan(data['fan_duty']):
				self.fan_duty = data.pop('fan_duty')
			for k in data:
				setattr(self, k, data[k])
			self.Tc -= C0
			self.heater_limit_l -= C0
			self.heater_limit_h -= C0
			self.fan_limit_l -= C0
			self.fan_limit_h -= C0
			self.fan_temp -= C0
			self.fan_pin ^= 0x200
		def write(self):
			attrnames = ('R0', 'R1', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time', 'P', 'I', 'D')
			data = {n: getattr(self, n) for n in attrnames}
			try:
				data['logRc'] = math.log(self.Rc)
			except:
				data['logRc'] = float('nan')
			data['fan_pin'] ^= 0x200
			for key in ('Tc', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'fan_temp'):
				data[key] += C0
			#log('writing temp: %s' % repr(data))
			cdriver.write_temp(self.id, data)
		def export(self):
			attrnames = ('name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time', 'P', 'I', 'D', 'value')
			return {n: getattr(self, n) for n in attrnames}
		def export_settings(self):
			ret = '[temp %d]\r\n' % self.id
			ret += 'name = %s\r\n' % self.name
			ret += ''.join(['%s = %s\r\n' % (x, write_pin(getattr(self, x))) for x in ('heater_pin', 'fan_pin', 'thermistor_pin')])
			ret += ''.join(['%s = %f\r\n' % (x, getattr(self, x)) for x in ('fan_temp', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time', 'P', 'I', 'D')])
			return ret
	# }}}
	class Gpio: # {{{
		def __init__(self, id):
			self.name = 'gpio %d' % id
			self.id = id
			self.state = 3
			self.reset = 3
			self.value = False
			self.duty = 1.
			self.leader = None
			self.ticks = 1
		def read(self):
			data = cdriver.read_gpio(self.id)
			self.pin = data['pin']
			self.duty = data['duty']
			self.state = data['state'] & 0x3
			self.reset = (data['state'] >> 2) & 0x3
			self.leader = data['leader']
			self.ticks = data['ticks']
		def write(self):
			attrnames = ('pin', 'duty', 'leader', 'ticks')
			data = {n: getattr(self, n) for n in attrnames}
			data['state'] = self.state | (self.reset << 2)
			if data['leader'] is None:
				data['leader'] = -1
			cdriver.write_gpio(self.id, data)
		def export(self):
			attrnames = ('name', 'pin', 'state', 'reset', 'duty', 'leader', 'ticks')
			attrs = {n: getattr(self, n) for n in attrnames}
			attrs['value'] = self.value if self.state >= 2 else self.state == 1
			if attrs['leader'] < 0:
				attrs['leader'] = None
			return attrs
		def export_settings(self):
			ret = '[gpio %d]\r\n' % self.id
			ret += 'name = %s\r\n' % self.name
			ret += 'pin = %s\r\n' % write_pin(self.pin)
			ret += 'reset = %d\r\n' % self.reset
			ret += 'duty = %f\r\n' % self.duty
			ret += 'leader = %s\r\n' % (self.leader if self.leader is not None else '-')
			ret += 'ticks = %d\r\n' % self.ticks
			return ret
	# }}}
	# }}}
	# }}}
	# Useful commands.  {{{

	# Setup and shutdown.
	def admin_reset_uuid(self): # {{{
		uuid = protocol.new_uuid(string = False)
		cdriver.set_uuid(bytes(uuid))
		self.uuid = protocol.new_uuid(uuid = uuid, string = True)
		if not self.name:
			self.name = self.uuid
		return self.uuid
	# }}}
	def get_typeinfo(self): # {{{
		return typeinfo
	# }}}
	def expert_die(self, reason): # {{{
		'''Kill this machine, including all files on disk.
		'''
		log('%s dying as requested by host (%s).' % (self.uuid, reason))
		# Clean up spool.
		dirname = fhs.write_spool(self.uuid, dir = True, opened = False)
		if os.path.isdir(dirname):
			try:
				shutil.rmtree(dirname, ignore_errors = False)
			except:
				log('Could not remove %s' % dirname)
		# Clean up profiles.
		for dirname in fhs.read_data(self.uuid, dir = True, multiple = True, opened = False):
			try:
				shutil.rmtree(dirname, ignore_errors = False)
			except:
				log('Could not remove %s' % dirname)
		return (WAIT, WAIT)
	# }}}
	def send_machine(self, target): # {{{
		'''Return all settings about a machine.
		'''
		self.initialized = True
		self._broadcast(target, 'new_machine', [self.queue_length])
		self._globals_update(target)
		for i, s in enumerate(self.spaces):
			self._space_update(i, target)
		for i, t in enumerate(self.temps):
			self._temp_update(i, target)
		for i, g in enumerate(self.gpios):
			self._gpio_update(i, target)
		self._refresh_queue(target)
		if self.confirmer is not None:
			self._broadcast(target, 'confirm', self.confirm_id, self.confirm_message)
	# }}}
	def admin_disconnect(self, reason = None): # {{{
		cdriver.force_disconnect()
		self._close(False)
	# }}}

	# Normal operation.
	@delayed
	def user_line(self, id, moves = (), e = None, tool = None, v = None, relative = False, probe = False, single = False, force = False): # {{{
		'''Move the tool in a straight line; return when done.
		'''
		if not force and self.home_phase is not None:
			log('ignoring line during home')
			if id is not None:
				self._send(id, 'return', None)
			return
		if tool is None:
			tool = self.current_extruder
		if e is None:
			e = float('nan')
		# Turn sequences into a dict.
		if isinstance(moves, (list, tuple)):
			mdict = {}
			for s, data in enumerate(moves):
				mdict[s] = data
			moves = mdict
		# Make sure the keys are ints.
		moves = {int(k): moves[k] for k in moves}
		# Turn into list of floats.
		moves = [moves[i] if i in moves else float('nan') for i in range(6)]
		# Set current tool.
		if tool is not None:
			self.current_extruder = tool
		# Set defaults for feedrates.
		if all(math.isnan(x) for x in moves) and self.current_extruder < len(self.spaces[1].motor) and (v is None or not 0 < v <= self.spaces[1].motor[self.current_extruder]['limit_v']):
			v = self.spaces[1].motor[self.current_extruder]['limit_v']
		elif v is None or not 0 < v <= self.max_v:
			v = self.max_v
		self.moving = True
		#log('move to ' + repr(moves))
		cdriver.move(*([self.current_extruder] + moves + [e, v]), single = single, probe = probe, relative = relative)
		if id is not None:
			self.wait_for_cb()[1](id)
	# }}}
	def user_move_target(self, dx, dy): # {{{
		'''Move the target position.
		Using this function avoids a round trip to the driver.
		'''
		self.user_set_globals(targetx = self.targetx + dx, targety = self.targety + dy)
	# }}}
	def user_sleep(self, sleeping = True, update = True, force = False): # {{{
		'''Put motors to sleep, or wake them up.
		'''
		if sleeping:
			if self.home_phase is not None or (not self.paused and self.gcode_file):
				return
			self.position_valid = False
			if update:
				self._globals_update()
		cdriver.sleep(sleeping, force)
	# }}}
	def user_settemp(self, channel, temp, update = True): # {{{
		'''Set target temperature.
		'''
		channel = int(channel)
		self.temps[channel].value = temp
		if update:
			self._temp_update(channel)
		cdriver.settemp(channel, temp + C0 if not math.isnan(self.temps[channel].beta) else temp)
		if self.gcode_waiting > 0 and any(channel == x[0] for x in self.tempcb):
			self.user_waittemp(channel, temp)
	# }}}
	def user_waittemp(self, channel, min, max = None): # {{{
		'''Set temperature alarm values.
		Note that this function returns immediately; it does not wait
		for the temperature to be reached.
		'''
		channel = int(channel)
		if min is None:
			min = float('nan')
		if max is None:
			max = float('nan')
		cdriver.waittemp(channel, min + C0 if not math.isnan(self.temps[channel].beta) else min, max + C0 if not math.isnan(self.temps[channel].beta) else max)
	# }}}
	def temp_value(self, channel): # {{{
		'''Read current temperature.
		'''
		if not self.connected:
			return float('nan')
		channel = int(channel)
		if channel >= len(self.temps):
			log('Trying to read invalid temp %d' % channel)
			return float('nan')
		return cdriver.temp_value(channel) - (C0 if not math.isnan(self.temps[channel].beta) else 0)
	# }}}
	def power_value(self, channel): # {{{
		'''Read power recordings.
		The return value is a tuple of the time it has been on since
		this function was last called, and the current time, both in
		milliseconds.
		To use, this function must be called at least twice; the first
		call only the time is recorded.  The second call the new time
		is recorded and the elapsed time is computed and used in
		combination with the time it was on.
		'''
		channel = int(channel)
		if channel >= len(self.temps):
			log('Trying to read invalid power %d' % channel)
			return None, None
		return cdriver.power_value(channel)
	# }}}
	def pin_value(self, pin): # {{{
		'''Read current value of a gpio pin.
		'''
		return cdriver.pin_value(pin)
	# }}}
	def user_abort(self): # {{{
		'''Abort the current job.
		'''
		for t, temp in enumerate(self.temps):
			self.user_settemp(t, float('nan'))
		for g, gpio in enumerate(self.gpios):
			self.user_set_gpio(g, state = gpio.reset)
		if self.paused:
			self._unpause()
		self._job_done(False, 'aborted by user')
		# Sleep doesn't work as long as home_phase is non-None, so do it after _job_done.
		self.user_sleep(force = True)
	# }}}
	@delayed
	def user_pause(self, id, pausing = True, store = True, update = True): # {{{
		'''Pause or resume the machine.
		'''
		if pausing == self.paused:
			if id is not None:
				self._send(id, 'return', None)
			return
		self._trigger_movewaits(False)
		self.paused = self.gcode_file and pausing
		if pausing:
			cdriver.pause()
		else:
			cdriver.resume()
		if update:
			self._globals_update()
		if pausing and id is not None and self.moving:
			self.movecb.append(lambda done: self._send(id, 'return', None))
		elif id is not None:
			self._send(id, 'return', None)
	# }}}
	def queued(self): # {{{
		'''Get the number of currently queued segments.
		'''
		return cdriver.queued(False)
	# }}}
	@delayed
	def user_home(self, id, speed = 5, cb = None): # {{{
		'''Recalibrate the position with its limit switches.
		'''
		if not self.connected:
			if id is not None:
				self._send(id, 'return', None)
			return
		if self.home_phase is not None:
			log("ignoring request to home because we're already homing")
			if id is not None:
				self._send(id, 'return', None)
			return
		self.home_phase = 'start'
		self.home_id = id
		self.home_speed = speed
		self.home_done_cb = cb
		self._do_home()
	# }}}
	@delayed
	def user_park(self, id, cb = None, order = 0, aborted = False): # {{{
		'''Go to the park position.
		Home first if the position is unknown.
		'''
		if aborted:
			if id is not None:
				self._send(id, 'error', 'aborted')
			return
		#log('parking with cb %s' % repr(cb))
		self.parking = True
		if not self.position_valid:
			#log('homing')
			self.user_home(cb = lambda: self.user_park(cb)[1](id))[1](None)
			return
		next_order = None
		topark = [a['park_order'] for a in self.spaces[0].axis if not math.isnan(a['park']) and a['park_order'] >= order]
		if len(topark) > 0 and (next_order is None or min(topark) > next_order):
			next_order = min(topark)
		if next_order is None:
			#log('done parking; cb = %s' % repr(cb))
			self.parking = False
			if cb:
				def wrap_cb(done):
					call_queue.append((cb, []))
					if id is not None:
						self._send(id, 'return', None)
				self.movecb.append(wrap_cb)
				self.user_line()[1](None)
			else:
				if id is not None:
					self._send(id, 'return', None)
			return
		#log('not done parking: ' + repr((next_order)))
		self.movecb.append(lambda done: self.user_park(cb, order = next_order + 1, aborted = not done)[1](id))
		self.user_line([a['park'] - a['offset'] if a['park_order'] == next_order else float('nan') for ai, a in enumerate(self.spaces[0].axis)])[1](None)
	# }}}
	def wait_for_cb(self, id): # {{{
		'''Block until the move queue is empty.
		'''
		ret = lambda w: id is None or self._send(id, 'return', w)
		if self.moving or (self.gcode_file and not self.paused):
			self.movecb.append(ret)
		else:
			ret(True)
	# }}}
	def waiting_for_cb(self): # {{{
		'''Check if any process is waiting for the move queue to be empty.
		'''
		return self.moving or (self.gcode_file and not self.paused)
	# }}}
	@delayed
	def wait_for_temp(self, id, which = None): # {{{
		'''Wait for a temp to trigger its alarm.
		'''
		def cb():
			if id is not None:
				self._send(id, 'return', None)
				return
			self.gcode_waiting -= 1
		if(which is None and len(self.alarms) > 0) or which in self.alarms:
			cb()
		else:
			self.tempcb.append((which, cb))
	# }}}
	def user_clear_alarm(self, which = None): # {{{
		'''Disable a temp alarm.
		If which is None, disable all temp alarms.
		'''
		if which is None:
			self.alarms.clear()
		else:
			self.alarms.discard(which)
	# }}}
	def get_limits(self, space, motor = None):	# {{{
		'''Return all limits that were hit since they were cleared.
		'''
		if motor is None:
			return self.limits[space]
		if motor in self.limits[space]:
			return self.limits[space][motor]
		return None
	# }}}
	def user_clear_limits(self):	# {{{
		'''Clear all recorded limits.
		'''
		for s in range(len(self.spaces)):
			self.limits[s].clear()
	# }}}
	def valid(self):	# {{{
		'''Return whether the position of the motors is known.
		'''
		return self.position_valid
	# }}}
	@delayed
	def user_gcode_run(self, id, code, paused = False): # {{{
		'''Run a string of g-code.
		'''
		with fhs.write_temp(text = False) as f:
			f.write(code)
			f.seek(0)
			self.gcode_id = id
			# Break this in two, otherwise tail recursion may destroy f before call is done?
			ret = self._gcode_run(f.filename, paused = paused)
			return ret
	# }}}
	@delayed
	def user_request_confirmation(self, id, message): # {{{
		'''Wait for confirmation.
		The return value is True if confirmation is given, False if
		not.
		'''
		# Abort pending confirmation, if any.
		if self.confirmer not in (False, None):
			self._send(self.confirmer, 'return', False)
		self.confirmer = id
		self.confirm_id += 1
		self.confirm_axes = [[s.get_current_pos(a)[0] for a in range(len(s.axis))] for s in self.spaces]
		self.confirm_message = message
		self._broadcast(None, 'confirm', self.confirm_id, self.confirm_message)
		for c in self.confirm_waits:
			self._send(c, 'return', (self.confirm_id, self.confirm_message))
		self.confirm_waits.clear()
	# }}}
	def get_confirm_id(self): # {{{
		'''Return id of current confirmation request, if any.
		'''
		return self.confirm_id, self.confirm_message
	# }}}
	@delayed
	def wait_confirm(self, id, pending = True): # {{{
		'''Block until confirmation is requested.
		If pending is False, ignore the current request, if any.
		'''
		if pending and self.confirmer is not None:
			self._send(id, 'return', (self.confirm_id, self.confirm_message))
			return
		self.confirm_waits.add(id)
	# }}}
	def user_confirm(self, confirm_id, success = True): # {{{
		'''Respond to a confirmation request.
		If confirm_id is not None, it must be equal to the current id
		or the confirmation is ignored.
		Success is passed to the requester.  If it is requested by
		g-code, passing False will abort the job.
		'''
		if confirm_id not in (self.confirm_id, None) or self.confirm_axes is None:
			# Confirmation was already sent, or never reguested.
			#log('no confirm %s' % repr((confirm_id, self.confirm_id)))
			return False
		id = self.confirmer
		self.confirmer = None
		self.confirm_message = None
		self._broadcast(None, 'confirm', None)
		self._reset_extruders(self.confirm_axes)
		self.confirm_axes = None
		if id not in (False, None):
			self._send(id, 'return', success)
		else:
			if self.probing: # FIXME: remove this?
				call_queue.append((self.probe_cb, [False if success else None]))
			else:
				if not success:
					self.probe_pending = False
					self._job_done(False, 'aborted by failed confirmation')
				else:
					if self.probe_enable and self.probe_pending and self._pin_valid(self.probe_pin):
						self.probe_pending = False
						call_queue.append((self._one_probe, []))
					else:
						self.probe_pending = False
						cdriver.resume()
		return True
	# }}}
	def get_machine_state(self): # {{{
		'''Return current machine state.
		Return value is a tuple of a human readable string describing
		the state, NaN or the elapsed time, NaN or the total time for
		the current job.
		Note that the times are computed from the requested speeds.
		These are generally too low, because they don't account for
		acceleration and velocity limits.
		'''
		pos = self.tp_get_position()
		context = self.tp_get_context(position = pos[0])
		if self.paused:
			state = 'Paused'
		elif self.gcode_file:
			state = 'Running'
		else:
			return 'Idle', float('nan'), float('nan'), pos[0], pos[1], context
		return state, cdriver.get_time(), self.total_time / self.feedrate, pos[0], pos[1], context
	# }}}
	def motors2xyz(self, motors): # {{{
		return cdriver.motors2xyz(*motors)
	# }}}

	# Profile management.
	def user_load(self, profile = None, update = True): # {{{
		'''Load a profile.
		'''
		filenames = fhs.read_data(os.path.join(self.uuid, 'profiles', ((profile and profile.strip()) or self.profile) + os.extsep + 'ini'), opened = False, multiple = True)
		if profile and self.profile != profile.strip():
			#log('setting profile to %s' % profile.strip())
			self.profile = profile.strip()
			if update:
				self._globals_update()
		if len(filenames) > 0:
			with open(filenames[0]) as f:
				log('loading profile {}'.format(filenames[0]))
				self.expert_import_settings(f.read(), update = update)
		else:
			log('not loading nonexistent profile')
	# }}}
	def admin_save(self, profile = None): # {{{
		'''Save a profile.
		If the profile name is not given, it saves the current profile.
		'''
		if profile and self.profile != profile.strip():
			log('setting profile to %s' % profile.strip())
			self.profile = profile.strip()
			self._globals_update()
		with fhs.write_data(os.path.join(self.uuid, 'profiles', (profile.strip() or self.profile) + os.extsep + 'ini')) as f:
			f.write(self.export_settings())
	# }}}
	def list_profiles(self): # {{{
		'''Get a list of all available profiles.
		'''
		dirnames = fhs.read_data(os.path.join(self.uuid, 'profiles'), dir = True, multiple = True, opened = False)
		ret = []
		for d in dirnames:
			for f in os.listdir(d):
				name = os.path.splitext(f)[0].strip()
				if name not in ret:
					ret.append(name)
		ret.sort()
		return ret
	# }}}
	def admin_remove_profile(self, profile): # {{{
		'''Remove a profile.
		'''
		filename = fhs.write_data(os.path.join(self.uuid, 'profiles', (profile.strip() or self.profile) + os.extsep + 'ini'), opened = False)
		if os.path.exists(filename):
			os.unlink(filename)
			return True
		return False
	# }}}
	def admin_set_default_profile(self, profile): # {{{
		'''Set a profile as default.
		'''
		self.default_profile = profile
		with fhs.write_data(os.path.join(self.uuid, 'info' + os.extsep + 'txt')) as f:
			f.write(self.name + '\n')
			f.write(profile + '\n')
	# }}}
	def export_settings(self): # {{{
		'''Export the current settings.
		The resulting string can be imported back.
		'''
		message = '[general]\r\n'
		for t in ('temps', 'gpios'):
			message += 'num_%s = %d\r\n' % (t, len(getattr(self, t)))
		message += 'pin_names = %s\r\n' % ','.join(('%d' % p[0]) + p[1] for p in self.pin_names)
		message += 'unit_name = %s\r\n' % self.unit_name
		message += 'spi_setup = %s\r\n' % self._mangle_spi()
		message += ''.join(['%s = %s\r\n' % (x, write_pin(getattr(self, x))) for x in ('led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'pattern_step_pin', 'pattern_dir_pin')])
		message += ''.join(['%s = %d\r\n' % (x, getattr(self, x)) for x in ('bed_id', 'fan_id', 'spindle_id', 'park_after_job', 'sleep_after_job', 'cool_after_job', 'probe_enable')])
		message += ''.join(['%s = %f\r\n' % (x, getattr(self, x)) for x in ('probe_dist', 'probe_offset', 'probe_safe_dist', 'temp_scale_min', 'temp_scale_max', 'max_deviation', 'max_v', 'max_a', 'max_J', 'adjust_speed', 'timeout')])
		message += 'user_interface = %s\r\n' % self.user_interface
		for i, s in enumerate(self.spaces):
			message += s.export_settings()
		for i, t in enumerate(self.temps):
			message += t.export_settings()
		for i, g in enumerate(self.gpios):
			message += g.export_settings()
		return message
	# }}}
	def expert_import_settings(self, settings, filename = None, update = True): # {{{
		'''Import new settings.
		settings is a string as created by export_settings.
		The filename is ignored.
		'''
		self._broadcast(None, 'blocked', 'importing settings')
		self.user_sleep(update = update)
		section = 'general'
		index = None
		obj = None
		regexp = re.compile(r'\s*\[(general|(space|temp|gpio|(extruder|axis|motor|follower)\s+(\d+))\s+(\d+))\]\s*$|\s*(?:(\w+)-)?(\w+)\s*=\s*(.*?)\s*$|\s*(?:#.*)?$')
		#1: (general|(space|temp|gpio|(axis|motor)\s+(\d+))\s+(\d+))	1 section
		#2: (space|temp|gpio|(extruder|axis|motor)\s+(\d+))		2 section with index
		#3: (extruder|axis|motor)					3 sectionname with two indices
		#4: (\d+)							4 index of space
		#5: (\d+)							5 only or component index
		#6: (\w+)							6 module prefix
		#7: (\w+)							7 identifier
		#8: (.*?)							8 value
		errors = []
		globals_changed = True
		changed = {'space': set(), 'temp': set(), 'gpio': set(), 'axis': set(), 'motor': set(), 'extruder': set(), 'follower': set()}
		keys = {
				'general': {'num_temps', 'num_gpios', 'user_interface', 'pin_names', 'led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'pattern_step_pin', 'pattern_dir_pin', 'probe_dist', 'probe_offset', 'probe_safe_dist', 'bed_id', 'fan_id', 'spindle_id', 'unit_name', 'probe_enable', 'temp_scale_min', 'temp_scale_max', 'park_after_job', 'sleep_after_job', 'cool_after_job', 'spi_setup', 'max_deviation', 'max_v', 'max_a', 'max_J', 'adjust_speed', 'timeout'},
				'space': {'type', 'num_axes'},
				'temp': {'name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time', 'P', 'I', 'D'},
				'gpio': {'name', 'pin', 'state', 'reset', 'duty', 'leader', 'ticks'},
				'axis': {'name', 'park', 'park_order', 'min', 'max', 'offset', 'home_pos2'},
				'motor': {'step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'steps_per_unit', 'home_pos', 'limit_v', 'limit_a', 'home_order', 'unit'},
				'extruder': {'dx', 'dy', 'dz'},
				'follower': {'leader'}
			}
		for l in settings.split('\n'):
			r = regexp.match(l)
			if not r:
				errors.append((l, 'syntax error'))
				continue
			if r.group(1) is not None:
				# New section.
				if r.group(2) is not None:
					# At least one index.
					#log("At least one index")
					if r.group(3) is not None:
						# Two indices: axis, motor, extruder, follower.
						#log("Two indices")
						index = (int(r.group(4)), int(r.group(5)))
						section = r.group(3)
						if index[0] >= len(self.spaces) or index[1] >= len(getattr(self.spaces[index[0]], section)):
							log('index out of range for %s; %s %s' % (index, len(self.spaces), len(getattr(self.spaces[index[0]], section)) if index[0] < len(self.spaces) else 'x'))
							errors.append((l, 'index out of range'))
							obj = None
							continue
						obj = getattr(self.spaces[index[0]], section)[index[1]]
					else:
						#log("One index")
						# One index: space, temp, gpio.
						index = int(r.group(5))
						section = r.group(2)
						if index >= len(getattr(self, section + 's')):
							errors.append((l, 'index out of range'))
							obj = None
							continue
						obj = getattr(self, section + 's')[index]
					changed[section].add(index)
				else:
					#log("No index")
					# No indices: general.
					section = r.group(1)
					index = None
					obj = self
					globals_changed = True
				continue
			elif obj is None:
				# Ignore settings for incorrect section.
				continue
			if not r.group(7):
				# Comment or empty line.
				continue
			module = r.group(6)
			key = r.group(7)
			value = r.group(8)
			if module is None:
				try:
					if key == 'pin_names':
						if len(self.pin_names) > 0:
							# Don't override hardware-provided names.
							continue
						if value.strip() == '':
							# Avoid errors when empty.
							continue
						value = [[int(x[0]), x[1:]] for x in value.split(',')]
					elif 'name' in key or key in ('user_interface', 'type', 'unit'):
						pass	# Keep strings as they are.
					elif key == 'spi_setup':
						value = self._unmangle_spi(value)
					elif key.endswith('pin'):
						value = read_pin(self, value)
						#log('pin imported as {} for {}'.format(value, key))
					elif key == 'leader':
						value = None if value == '-' else int(value)
					elif key.startswith('num') or key.endswith('order') or key.endswith('_id') or key in ('ticks', 'probe_enable'):
						value = int(value)
					else:
						value = float(value)
				except ValueError:
					errors.append((l, 'invalid value for %s' % key))
					continue
				if key not in keys[section] or (section == 'motor' and ((key in ('home_pos', 'home_order') and index[0] == 1) or (key in ('steps_per_unit', 'limit_v', 'limit_a') and index[0] == 2))):
					errors.append((l, 'invalid key for section %s' % section))
					continue
			else:
				if index is None:
					errors.append((l, 'module keys are not allowed in the general section'))
					continue
				if section == 'space':
					if module != obj.type:
						errors.append((l, 'invalid module type: %s != %s' % (module, obj.type)))
						continue
					module_data = obj.module
				elif section not in ('motor', 'axis'):
					errors.append((l, 'invalid section %s for module key' % section))
					continue
				else:
					if module != self.spaces[index[0]].type:
						errors.append((l, 'invalid module type: %s != %s' % (module, self.spaces[index[0]].type)))
						continue
					module_data = obj['module']
				if key not in module_data:
					errors.append((l, 'invalid module key %s-%s' % (module, key)))
					continue
				# Cast to correct type.
				module_data[key] = type(module_data[key])(value)
			# If something critical is changed, update instantly.
			if module is None and (key.startswith('num') or key == 'type'):
				#log('setting now for %s:%s=%s' % (section, key, value))
				if index is None:
					self.expert_set_globals(**{key: value})
				else:
					if section == 'space':
						for i in changed['motor']:
							if i[0] == index:
								self.expert_set_motor(i, readback = False)
						for i in changed['axis']:
							if i[0] == index:
								self.expert_set_axis(i, readback = False)
					getattr(self, 'expert_set_' + section)(index, **{key: value})
			else:
				if isinstance(index, tuple):
					#log('setting later %s' % repr((section, key, value)))
					obj[key] = value
				else:
					#log('setting later other %s' % repr((section, key, value)))
					if section == 'extruder':
						obj[ord[key[1]] - ord['x']] = value
					else:
						setattr(obj, key, value)
		# Update values in the machine by calling the expert_set_* functions with no new settings.
		if globals_changed:
			#log('setting globals')
			self.expert_set_globals()
		for index in changed['extruder']:
			changed['space'].add(index[0])
		for index in changed['follower']:
			changed['space'].add(index[0])
		for section in changed:
			for index in changed[section]:
				if not isinstance(index, tuple):
					continue
				if section not in ('follower', 'extruder'):
					#log('setting non-follower %s %s' % (section, index))
					getattr(self, 'expert_set_' + section)(index, readback = False)
				changed['space'].add(index[0])
		for section in changed:
			for index in changed[section]:
				if isinstance(index, tuple):
					continue
				#log('setting %s' % repr((section, index)))
				getattr(self, 'expert_set_' + section)(index)
		self._broadcast(None, 'blocked', None)
		return errors
	# }}}
	def expert_import_POST(self, filename, name): # {{{
		'''Import settings using a POST request.
		Note that this function can only be called using POST; not with the regular websockets system.
		'''
		errors = ['%s (%s)' % (msg, ln) for ln, msg in self.expert_import_settings(open(filename).read(), name)]
		return filename + '\n' + json.dumps(errors)
	# }}}

	# Audio (currently not working).
	@delayed
	def benjamin_audio_play(self, id, name, motor = 2): # {{{
		self.audio_id = id
		self.user_sleep(False)
		filename = fhs.read_spool(os.path.join(self.uuid, 'audio', name + os.extsep + 'bin'), opened = False)
		cdriver.run_file(filename.encode('utf-8'), b'', 1, 0, 0, motor)
	# }}}
	def benjamin_audio_add_POST(self, filename, name): # {{{
		with open(filename, 'rb') as f:
			self._audio_add(f, name)
		return filename + '\n' + json.dumps([])
	# }}}
	def benjamin_audio_del(self, name): # {{{
		assert name in self.audioqueue
		filename = fhs.read_spool(os.path.join(self.uuid, 'audio', name + os.extsep + 'bin'), opened = False)
		os.unlink(filename)
		del self.audioqueue[name]
		self._broadcast(None, 'audioqueue', tuple(self.audioqueue.keys()))
	# }}}
	def audio_list(self): # {{{
		return self.audioqueue
	# }}}
	@delayed

	# Queue management.
	def queue_add(self, data, name): # {{{
		'''Add code to the queue as a string.
		'''
		with fhs.write_temp() as f:
			f.write(data)
			f.seek(0)
			return self._queue_add(f.filename, name)
	# }}}
	def queue_add_POST(self, filename, name): # {{{
		'''Add g-code to queue using a POST request.
		Note that this function can only be called using POST; not with the regular websockets system.
		'''
		return self._queue_add(filename, name)
	# }}}
	def queue_remove(self, name, audio = False): # {{{
		'''Remove an entry from the queue.
		'''
		#log('removing %s' % name)
		if audio:
			assert name in self.audioqueue
			filename = fhs.read_spool(os.path.join(self.uuid, 'audio', name + os.extsep + 'bin'), opened = False)
			del self.audioqueue[name]
		else:
			assert name in self.jobqueue
			filename = fhs.read_spool(os.path.join(self.uuid, 'gcode', name + os.extsep + 'bin'), opened = False)
			del self.jobqueue[name]
		try:
			os.unlink(filename)
		except:
			log('unable to unlink %s' % filename)
		self._refresh_queue()
	# }}}
	def queue_list(self): # {{{
		return self.jobqueue
	# }}}
	@delayed
	def user_queue_run(self, id, name, paused = False): # {{{
		'''Start a new job.
		'''
		if self.job_current is not None and not self.paused:
			log('ignoring run request while job is in progress: %s ' % repr(self.job_current) + str(self.paused))
			if id is not None:
				self._send(id, 'return', None)
			return
		#log('set active jobs to %s' % names)
		self.job_current = name
		if self.job_current is not None:
			self.job_id = id
			self.gcode_id = None
			# Set all extruders to 0.
			for i, e in enumerate(self.spaces[1].axis):
				self.user_set_axis_pos(1, i, 0)
			def cb():
				#log('start job %s' % self.job_current)
				self._gcode_run(self.job_current, abort = False, paused = paused)
			if not self.position_valid:
				self.user_park(cb = cb)[1](None)
			else:
				cb()
		elif id is not None:
			self._send(id, 'return', None)
	# }}}

	# Commands for handling the toolpath.
	def tp_get_position(self): # {{{
		'''Get current toolpath position.
		@return position, total toolpath length.'''
		if not self.gcode_file:
			return 0, 0
		return cdriver.tp_getpos(), self.gcode_num_records
	# }}}
	def user_tp_set_position(self, position): # {{{
		'''Set current toolpath position.
		It is an error to call this function while not paused.
		@param position: new toolpath position.
		@return None.'''
		assert self.gcode_file
		assert 0 <= position < self.gcode_num_records
		assert self.paused
		cdriver.tp_setpos(position)
	# }}}
	def tp_get_context(self, num = None, position = None): # {{{
		'''Get context around a position.
		@param num: number of lines context on each side.
		@param position: center of the returned region, or None for current position.
		@return first position of returned region (normally position - num), list of lines+arcs+specials'''
		if not self.gcode_file:
			return 0, []
		if num is None:
			num = 100;
		if position is None:
			position = self.tp_get_position()[0]
		position = int(position)
		def parse_record(num):
			s = struct.calcsize(record_format)
			type, tool, X, Y, Z, hx, hy, hz, Jg, tf, v0, E, time, line = struct.unpack(record_format, self.gcode_map[num * s:(num + 1) * s])
			#log('get context type %d' % type)

			return {'type': tuple(x for x in protocol.parsed if protocol.parsed[x] == type)[0], 'X': (X, Y, Z), 'h': (hx, hy, hz), 'Jg': Jg, 'tf': tf, 'v0': v0, 'E': E, 'time': time, 'line': line}
		return max(0, position - num), [parse_record(x) for x in range(position - num, position + num + 1) if 0 <= x < self.gcode_num_records]
	# }}}
	def tp_get_string(self, num): # {{{
		'''Get string from toolpath.
		@param num: index of the string.
		@return the string.'''
		return self.gcode_strings[num]
	# }}}
	def tp_find_position(self, x = None, y = None, z = None): # {{{
		'''Find toolpath position closest to coordinate.
		Inputs may be None, in that case that coordinate is ignored.
		@param x: X coordinate of target or None.
		@param y: Y coordinate of target or None.
		@param z: Z coordinate of target or None.
		@return toolpath position.'''
		assert self.gcode_file
		return cdriver.tp_findpos(*(a if a is not None else float('nan') for a in (x, y, z)))
	# }}}
	# }}}
	# Accessor functions. {{{
	# Globals. {{{
	def get_globals(self): # {{{
		#log('getting globals')
		ret = {'num_temps': len(self.temps), 'num_gpios': len(self.gpios)}
		for key in ('name', 'user_interface', 'pin_names', 'uuid', 'queue_length', 'num_pins', 'led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'pattern_step_pin', 'pattern_dir_pin', 'probe_dist', 'probe_offset', 'probe_safe_dist', 'bed_id', 'fan_id', 'spindle_id', 'unit_name', 'probe_enable', 'feedrate', 'targetangle', 'store_adc', 'temp_scale_min', 'temp_scale_max', 'probe_points', 'paused', 'park_after_job', 'sleep_after_job', 'cool_after_job', 'spi_setup', 'max_deviation', 'max_v', 'max_a', 'max_J', 'adjust_speed', 'timeout'):
			ret[key] = getattr(self, key)
		return ret
	# }}}
	def expert_set_globals(self, update = True, **ka): # {{{
		#log('setting variables with %s' % repr(ka))
		nt = ka.pop('num_temps') if 'num_temps' in ka else None
		ng = ka.pop('num_gpios') if 'num_gpios' in ka else None
		if 'store_adc' in ka:
			self.store_adc = bool(ka.pop('store_adc'))
		if 'name' in ka:
			name = ka.pop('name')
			if name != self.name:
				self.name = name
				self.admin_set_default_profile(self.default_profile)
		for key in ('unit_name', 'user_interface', 'pin_names'):
			if key in ka:
				setattr(self, key, ka.pop(key))
		if 'spi_setup' in ka:
			self.spi_setup = self._unmangle_spi(ka.pop('spi_setup'))
			if self.spi_setup:
				self._spi_send(self.spi_setup)
		for key in ('led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'pattern_step_pin', 'pattern_dir_pin', 'bed_id', 'fan_id', 'spindle_id', 'park_after_job', 'sleep_after_job', 'cool_after_job', 'probe_enable'):
			if key in ka:
				setattr(self, key, int(ka.pop(key)))
		for key in ('probe_dist', 'probe_offset', 'probe_safe_dist', 'feedrate', 'targetangle', 'temp_scale_min', 'temp_scale_max', 'max_deviation', 'max_v', 'max_a', 'max_J', 'adjust_speed', 'timeout'):
			if key in ka:
				setattr(self, key, float(ka.pop(key)))
		if 'probe_points' in ka:
			self.probe_points = ka.pop('probe_points')
			# Build probe map.
			points = numpy.array(self.probe_points, dtype = float)	# points = [[x, y, z], ...], so points[0,:] is the first point, points[:,0] is all x coordinates.
			mean = numpy.mean(points[:, 2])
			points[:, 2] -= mean
			bbox = numpy.array([numpy.min(points, 0), numpy.max(points, 0)])
			# determine number of points on map grid
			grid = numpy.array(numpy.ceil((bbox[1] - bbox[0]) / self.probe_dist), dtype = int)
			# For each point on the grid, we need to find the weights of all probe points based on their distance.
			# Then using those weights, we compute the offset.
			# Computation for single grid point g using probe points p:
			# dists = [sum((p[c] - g[c]) ** 2 for c in range(2)) ** .5 for p in probe_points]
			# weights = [1 / d ** 2 for d in dists]
			# total = sum(weights, 0)
			# infs = isinf(total)
			# numinf = sum(infs)
			# if numinf > 0:
			#	return sum(weights * infs) / numinf
			# return sum(p[2] * w for p in probe_points) / total
			x = numpy.linspace(bbox[0][0], bbox[1][0], grid[0]).reshape((-1, 1, 1))
			y = numpy.linspace(bbox[0][1], bbox[1][1], grid[1]).reshape((1, -1, 1))
			dx = points[:, 0].reshape((1, 1, -1)) - x
			dy = points[:, 1].reshape((1, 1, -1)) - y
			dists = (dx ** 2 + dy ** 2) ** .5
			infs = dists == 0
			numinf = numpy.sum(infs, 2)
			dists[infs] = 1	# Avoid divide by zero warnings; value is forced to sample later.
			weights = 1 / (dists ** 2)
			total = numpy.sum(weights, 2)
			map_data = numpy.sum(points[:, 2].reshape((1, 1, -1)) * weights, 2) / total
			have_inf = numinf > 0
			map_data[have_inf] = numpy.sum(points[:, 2] * infs, 2)[have_inf]
			self.probe_map = {'origin': tuple(bbox[0, :2]), 'step': tuple(grid[:2]), 'data': tuple(tuple(d) for d in map_data)}
			cdriver.write_probe_map(self.probe_map)
		self._write_globals(nt, ng, update = update)
		if len(ka) > 0:
			log('Warning: unrecognized keyword arguments ignored: %s' % repr(ka))
		assert len(ka) == 0
	# }}}
	def user_set_globals(self, update = True, **ka): # {{{
		real_ka = {}
		for key in ('feedrate', 'targetangle'):
			if key in ka:
				real_ka[key] = ka.pop(key)
		assert len(ka) == 0
		return self.expert_set_globals(update = update, **real_ka)
	# }}}
	# }}}
	# Space {{{
	def get_axis_pos(self, space, axis = None): # {{{
		if space >= len(self.spaces) or (axis is not None and axis >= len(self.spaces[space].axis)):
			log('request for invalid axis position %d %d' % (space, axis))
			return float('nan')
		if axis is None:
			return [self.spaces[space].get_current_pos(a)[0] for a in range(len(self.spaces[space].axis))]
		else:
			return self.spaces[space].get_current_pos(axis)[0]
	# }}}
	def get_motor_pos(self, space, motor = None): # {{{
		if space >= len(self.spaces) or (motor is not None and motor >= len(self.spaces[space].motor)):
			log('request for invalid motor position %d %d' % (space, motor))
			return float('nan')
		if motor is None:
			return [self.spaces[space].get_current_pos(m)[1] for m in range(len(self.spaces[space].motor))]
		else:
			return self.spaces[space].get_current_pos(motor)[1]
	# }}}
	def user_set_axis_pos(self, space, axis, pos): # {{{
		if space >= len(self.spaces) or (axis is not None and axis >= len(self.spaces[space].axis)):
			log('request to set invalid axis position %d %d' % (space, axis))
			return False
		return self.spaces[space].set_current_pos(axis, pos)
	# }}}
	def get_space(self, space): # {{{
		ret = {'name': self.spaces[space].name, 'num_axes': len(self.spaces[space].axis), 'num_motors': len(self.spaces[space].motor)}
		ret['module'] = {'type': self.spaces[space].type}
		ret['module'].update(self.spaces[space].module)
		return ret
	# }}}
	def get_axis(self, space, axis): # {{{
		ret = {'name': self.spaces[space].axis[axis]['name']}
		if space == 1:
			ret['multiplier'] = self.multipliers[axis]
		if space == 0:
			for key in ('park', 'park_order', 'min', 'max', 'offset', 'home_pos2'):
				ret[key] = self.spaces[space].axis[axis][key]
		ret['module'] = {'type': self.spaces[space].type}
		ret['module'].update(self.spaces[space].axis[axis]['module'])
		return ret
	# }}}
	def get_motor(self, space, motor): # {{{
		ret = {'name': self.spaces[space].motor_name(motor)}
		for key in ('step_pin', 'dir_pin', 'enable_pin'):
			ret[key] = self.spaces[space].motor[motor][key]
		if space != 1:
			for key in ('limit_min_pin', 'limit_max_pin', 'home_pos', 'home_order'):
				ret[key] = self.spaces[space].motor[motor][key]
		if space != 2:
			for key in ('steps_per_unit', 'limit_v', 'limit_a', 'unit'):
				ret[key] = self.spaces[space].motor[motor][key]
		ret['module'] = {'type': self.spaces[space].type}
		ret['module'].update(self.spaces[space].motor[motor]['module'])
		return ret
	# }}}
	def expert_set_space(self, space, readback = True, update = True, **ka): # {{{
		old_type = self.spaces[space].type
		if space == 0 and 'type' in ka:
			self.spaces[space].type = ka.pop('type').strip()
			if self.spaces[space].type not in type_names:
				self.spaces[space].type = type_names[0]
		if 'num_axes' in ka:
			num_axes = int(ka.pop('num_axes'))
		else:
			num_axes = len(self.spaces[space].axis)
		num_motors = num_axes
		if old_type != self.spaces[space].type or not hasattr(self.spaces[space], 'module'):
			self.spaces[space].module = {item['name']: item['default'] for item in typeinfo[self.spaces[space].type]['space']}
		if 'module' in ka:
			module_data = ka.pop('module')
			assert module_data.pop('type') == self.spaces[space].type
			for key in module_data:
				assert key in self.spaces[space].module
				self.spaces[space].module[key] = type(self.spaces[space].module[key])(module_data[key])
		self.spaces[space].write_info(num_axes)
		if readback:
			self.spaces[space].read()
			if update:
				self._space_update(space)
		if len(ka) != 0:
			log('invalid input ignored: %s' % repr(ka))
	# }}}
	def expert_set_axis(self, spaceaxis, readback = True, update = True, **ka): # {{{
		space, axis = spaceaxis
		if 'name' in ka:
			self.spaces[space].axis[axis]['name'] = ka.pop('name')
		if space == 0:
			for key in ('park', 'park_order', 'min', 'max', 'offset', 'home_pos2'):
				if key in ka:
					self.spaces[space].axis[axis][key] = ka.pop(key)
		if space == 1 and 'multiplier' in ka and axis < len(self.spaces[space].motor):
			assert(ka['multiplier'] > 0)
			self.multipliers[axis] = ka.pop('multiplier')
			self.expert_set_motor((space, axis), readback, update)
		if 'module' not in self.spaces[space].axis[axis]:
			self.spaces[space].axis[axis]['module'] = {item['name']: item['default'] for item in typeinfo[self.spaces[space].type]['axis']}
		if 'module' in ka:
			module_data = ka.pop('module')
			assert module_data.pop('type') == self.spaces[space].type
			for key in module_data:
				assert key in self.spaces[space].axis[axis]['module']
				self.spaces[space].axis[axis]['module'][key] = type(self.spaces[space].axis[axis]['module'][key])(module_data[key])
		self.spaces[space].write_axis(axis)
		if readback:
			self.spaces[space].read()
			if update:
				self._space_update(space)
		assert len(ka) == 0
	# }}}
	def expert_set_motor(self, spacemotor, readback = True, update = True, **ka): # {{{
		space, motor = spacemotor
		for key in ('step_pin', 'dir_pin', 'enable_pin'):
			if key in ka:
				self.spaces[space].motor[motor][key] = ka.pop(key)
		for key in ('home_pos', 'limit_min_pin', 'limit_max_pin'):
			if space != 1 and key in ka:
				self.spaces[space].motor[motor][key] = ka.pop(key)
		if space != 1 and 'home_order' in ka:
			self.spaces[space].motor[motor]['home_order'] = ka.pop('home_order')
		for key in ('steps_per_unit', 'limit_v', 'limit_a'):
			if space != 2 and key in ka:
				self.spaces[space].motor[motor][key] = ka.pop(key)
		if space == 1 and 'unit' in ka:
			self.spaces[space].motor[motor]['unit'] = ka.pop('unit')
		if 'module' not in self.spaces[space].motor[motor]:
			log('new module')
			self.spaces[space].motor[motor]['module'] = {item['name']: item['default'] for item in typeinfo[self.spaces[space].type]['motor']}
		if 'module' in ka:
			module_data = ka.pop('module')
			assert module_data.pop('type') == self.spaces[space].type
			for key in module_data:
				assert key in self.spaces[space].motor[motor]['module']
				self.spaces[space].motor[motor]['module'][key] = type(self.spaces[space].motor[motor]['module'][key])(module_data[key])
		self.spaces[space].write_motor(motor)
		followers = False
		for m, mt in enumerate(self.spaces[2].motor):
			leader = self.spaces[2].follower[m]['leader']
			if leader is not None:
				if (leader & 0xf) == space and (leader >> 4) == motor:
					followers = True
					self.spaces[2].write_motor(m)
		if readback:
			self.spaces[space].read()
			if update:
				self._space_update(space)
			if followers:
				self.spaces[2].read()
				if update:
					self._space_update(2)
		assert len(ka) == 0
	# }}}
	# }}}
	# Temp {{{
	def get_temp(self, temp): # {{{
		ret = {}
		for key in ('name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time', 'P', 'I', 'D', 'value'):
			ret[key] = getattr(self.temps[temp], key)
		return ret
	# }}}
	def expert_set_temp(self, temp, update = True, **ka): # {{{
		ret = {}
		for key in ('name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time', 'P', 'I', 'D'):
			if key in ka:
				setattr(self.temps[temp], key, ka.pop(key))
		self.temps[temp].write()
		self.temps[temp].read()
		if update:
			self._temp_update(temp)
		if len(ka) != 0:
			log('problem: %s' % repr(ka))
		assert len(ka) == 0
	# }}}
	def user_set_temp(self, temp, update = True, **ka): # {{{
		real_ka = {}
		if 'fan_duty' in ka:
			real_ka['fan_duty'] = ka.pop('fan_duty')
		assert len(ka) == 0
		return self.expert_set_temp(temp, update = update, **real_ka)
	# }}}
	# }}}
	# Gpio {{{
	@delayed
	def wait_gpio(self, id, gpio, value = 1): # {{{
		assert gpio < len(self.gpios)
		if int(value) == int(self.gpios[gpio].value):
			self._send(id, 'return', None)
			return
		if gpio not in self.gpio_waits:
			self.gpio_waits[gpio] = []
		self.gpio_waits[gpio].append(id)
	# }}}
	def get_gpio(self, gpio): # {{{
		ret = {}
		for key in ('name', 'pin', 'state', 'reset', 'duty', 'leader', 'ticks', 'value'):
			ret[key] = getattr(self.gpios[gpio], key)
		return ret
	# }}}
	def expert_set_gpio(self, gpio, update = True, **ka): # {{{
		for key in ('name', 'pin', 'state', 'reset', 'duty', 'leader', 'ticks'):
			if key in ka:
				setattr(self.gpios[gpio], key, ka.pop(key))
		self.gpios[gpio].state = int(self.gpios[gpio].state)
		self.gpios[gpio].reset = int(self.gpios[gpio].reset)
		if self.gpios[gpio].reset >= 2 or (self.gpios[gpio].reset < 2 and self.gpios[gpio].state >= 2):
			self.gpios[gpio].state = self.gpios[gpio].reset
		#log('gpio %d reset %d' % (gpio, self.gpios[gpio].reset))
		self.gpios[gpio].write()
		self.gpios[gpio].read()
		if update:
			self._gpio_update(gpio)
		assert len(ka) == 0
	# }}}
	def user_set_gpio(self, gpio, update = True, **ka): # {{{
		real_ka = {}
		if 'state' in ka:
			real_ka['state'] = ka.pop('state')
		assert len(ka) == 0
		return self.expert_set_gpio(gpio, update = update, **real_ka)
	# }}}
	# }}}
	# }}}
# }}}

call_queue = []
machine = Machine(config['allow-system'])
fd = cdriver.fileno()

while True: # {{{
	while len(call_queue) > 0:
		f, a = call_queue.pop(0)
		#log('calling %s' % repr((f, a)))
		f(*a)
	fds = [sys.stdin, fd]
	found = select.select(fds, [], fds, None)
	if sys.stdin in found[0] or sys.stdin in found[2]:
		#log('command')
		machine._command_input()
	if fd in found[0] or fd in found[2]:
		#log('machine')
		machine._machine_input()
# }}}

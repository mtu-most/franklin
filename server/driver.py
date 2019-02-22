#!/usr/bin/python3
# vim: set foldmethod=marker fileencoding=utf8 :
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

# Constants {{{
C0 = 273.15	# Conversion between K and °C
WAIT = object()	# Sentinel for blocking functions.
NUM_SPACES = 3
# Space types
TYPE_CARTESIAN = 0
TYPE_DELTA = 1
TYPE_POLAR = 2
TYPE_EXTRUDER = 3
TYPE_FOLLOWER = 4
record_format = '=Bidddddddddddd' # type, tool, X, Y, Z, Bx, By, Bz, E, v0, v1, time, dist, r
# }}}

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
# }}}

config = fhs.init(packagename = 'franklin', config = { # {{{
	'allow-system': None,
	'uuid': None,
	'local': False,
	'arc': True
	})
# }}}

cdriver.init(fhs.read_data('franklin-cdriver', opened = False).encode('utf-8'))

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
	def handle_exit(*args):
		time.sleep(1)
		log('exit from signal: %s' % repr(args))
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
	atexit.register(handle_exit, 'exit')
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
		probe = fhs.read_spool(os.path.join(self.uuid, 'probe' + os.extsep + 'bin'), text = False)
		if probe is not None:
			try:
				# Map = [[targetx, targety, x0, y0, w, h], [nx, ny], [[...], [...], ...]]
				size = struct.calcsize('@ddddddddLLd')
				targetx, targety, x0, y0, w, h, sina, cosa, nx, ny, self.targetangle = struct.unpack('@ddddddddLLd', probe.read(size))
				self.gcode_angle = math.sin(self.targetangle), math.cos(self.targetangle)
				sina, cosa = self.gcode_angle
				limits = [targetx, targety, x0, y0, w, h]
				nums = [nx, ny, self.targetangle]
				if not (0 < nx < 1000 and 0 < ny < 1000):
					raise ValueError('probe map too large; probably invalid')
				probes = [[None for x in range(nx + 1)] for y in range(ny + 1)]
				for y in range(ny + 1):
					for x in range(nx + 1):
						probes[y][x] = struct.unpack('@d', probe.read(struct.calcsize('@d')))[0]
				self.probemap = [limits, nums, probes]
			except:
				log('Failed to load probe map')
			self._globals_update()
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
						f.seek(-8 * 8, os.SEEK_END)
						self.jobqueue[name] = struct.unpack('=' + 'd' * 8, f.read())
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
		self.user_interface = '{Dv2m(Blocker:){Dv2m(No Connection:){dv3m{dv3m{dv3m[0:*Controls:{Dh60%{Dv12m{Dv5m{dh11m(Job Control:)(Buttons:)}(Position:)}{Dh85%(XY Map:)(Z Map:)}}{Dv4m(Abort:){Dv6m(Multipliers:){Dv2m(Gpios:){Dv9m(Temps:)(Temp Graph:)}}' + '}}' + '}Setup:{Dv2m(Save Profile:)[0:*Profile:(Profile Setup:)Probe:(Probe Setup:)Globals:(Globals Setup:)Axes:(Axis Setup:)Motors:(Motor Setup:)Type:{Dv3m(Type Setup:){Dh50%(Cartesian Setup:){Dh50%(Delta Setup:)(Polar Setup:)}}' + '}Extruder:(Extruder Setup:)Follower:(Follower Setup:)GPIO:(Gpio Setup:)Temps:(Temp Setup:)]}](Confirmation:)}(Message:)}(State:)}}' + '}'
		self.pin_names = []
		self.allow_system = allow_system
		self.probemap = None
		self.job_current = None
		self.job_id = None
		self.confirm_id = 0
		self.confirm_message = None
		self.confirm_axes = None
		self.confirmer = None
		self.position_valid = False
		self.probing = False
		self.probe_pending = False
		self.parking = False
		self.home_phase = None
		self.home_target = None
		self.home_cb = [False, self._do_home]
		self.probe_cb = [False, None]
		self.probe_speed = 3.
		self.gcode_file = False
		self.gcode_map = None
		self.gcode_id = None
		self.gcode_waiting = 0
		self.audio_id = None
		self.queue = []
		self.queue_pos = 0
		self.queue_info = None
		self.confirm_waits = set()
		self.gpio_waits = {}
		self.total_time = [float('nan'), float('nan')]
		self.resuming = False
		self.flushing = False
		self.debug_buffer = None
		self.command_buffer = ''
		self.bed_id = -1
		self.fan_id = -1
		self.spindle_id = -1
		self.probe_dist = 1000
		self.probe_offset = 0
		self.probe_safe_dist = 10
		self.num_probes = 1
		self.unit_name = 'mm'
		self.park_after_job = True
		self.sleep_after_job = True
		self.cool_after_job = True
		self.spi_setup = []
		# Set up state.
		self.spaces = [self.Space(self, i) for i in range(NUM_SPACES)]
		self.temps = []
		self.gpios = []
		self.probe_time_dist = [float('nan'), float('nan')]
		self.sending = False
		self.paused = False
		self.limits = [{} for s in self.spaces]
		self.wait = False
		self.movewait = 0
		self.movecb = []
		self.tempcb = []
		self.alarms = set()
		self.targetx = 0.
		self.targety = 0.
		self.targetangle = 0.
		self.zoffset = 0.
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
		self.timeout = 0
		self.bed_id = -1
		self.fan_id = -1
		self.spindle_id = -1
		self.feedrate = 1
		self.max_deviation = 0
		self.max_v = 100
		self.max_a = 10000
		self.current_extruder = 0
		self.targetx = 0.
		self.targety = 0.
		self.targetangle = 0.
		self.zoffset = 0.
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
	def _trigger_movewaits(self, num, done = True): # {{{
		#traceback.print_stack()
		#log('trigger %s' % repr(self.movecb))
		#log('movecbs: %d/%d' % (num, self.movewait))
		if self.movewait < num:
			log('More cbs received than requested!')
			self.movewait = 0
		else:
			#log('movewait %d/%d' % (num, self.movewait))
			self.movewait -= num
		if self.movewait == 0:
			#log('running cbs: %s' % repr(self.movecb))
			call_queue.extend([(x[1], [done]) for x in self.movecb])
			self.movecb = []
			if self.flushing and self.queue_pos >= len(self.queue):
				#log('done flushing')
				self.flushing = 'done'
		#else:
		#	log('cb seen, but waiting for more')
	# }}}
	def _machine_input(self, reply = False): # {{{
		cmd = cdriver.get_interrupt()
		#log('received interrupt: %s' % repr(cmd))
		if cmd['type'] == 'move-cb':
			#log('movecb %d/%d (%d in queue)' % (s, self.movewait, len(self.movecb)))
			self._trigger_movewaits(cmd['num'])
		elif cmd['type'] == 'temp-cb':
			self.alarms.add(cmd['temp'])
			t = 0
			while t < len(self.tempcb):
				if self.tempcb[t][0] is None or self.tempcb[t][0] in self.alarms:
					call_queue.append((self.tempcb.pop(t)[1], []))
				else:
					t += 1
		elif cmd['type'] == 'continue':
			# Move continue.
			self.wait = False
			#log('resuming queue %d' % len(self.queue))
			call_queue.append((self._do_queue, []))
			if self.flushing is None:
				self.flushing = False
		elif cmd['type'] == 'limit':
			if cmd['space'] < len(self.spaces) and cmd['motor'] < len(self.spaces[cmd['space']].motor):
				self.limits[cmd['space']][cmd['motor']] = cmd['pos']
			self.wait = False	# queue is flushed when limit switch is hit.
			#log('limit; %d waits' % e)
			self._trigger_movewaits(self.movewait, False)
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
			call_queue.append((self._do_home, [True]))
		elif cmd['type'] == 'disconnect':
			self._close()
			# _close returns after reconnect.
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
			if cmd['tool-changed'] and self.probemap is not None:
				self.probe_pending = True
			call_queue.append((self.user_request_confirmation(cmd['message'] or 'Continue?')[1], (False,)))
		elif cmd['type'] == 'park':
			call_queue.append((self.user_park(cb = cdriver.resume, abort = False)[1], (None,)))
		elif cmd['type'] == 'file-done':
			call_queue.append((self._job_done, (True, 'completed')))
		elif cmd['type'] == 'pinname':
			if cmd['pin'] >= len(self.pin_names):
				self.pin_names.extend([[0xf, '(Pin %d)' % i] for i in range(len(self.pin_names), cmd['pin'] + 1)])
			self.pin_names[cmd['pin']] = [cmd['mode'], cmd['name']]
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
				self.user_pause(True, False, update = False)
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
		data.update({x:getattr(self, x) for x in ('led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'timeout', 'bed_id', 'fan_id', 'spindle_id', 'feedrate', 'max_deviation', 'max_v', 'max_a', 'current_extruder', 'targetx', 'targety', 'targetangle', 'zoffset', 'store_adc')})
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
		attrnames = ('name', 'profile', 'user_interface', 'pin_names', 'led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'probe_dist', 'probe_offset', 'probe_safe_dist', 'bed_id', 'fan_id', 'spindle_id', 'unit_name', 'timeout', 'feedrate', 'max_deviation', 'max_v', 'max_a', 'targetx', 'targety', 'targetangle', 'zoffset', 'store_adc', 'park_after_job', 'sleep_after_job', 'cool_after_job', 'temp_scale_min', 'temp_scale_max', 'probemap', 'connected')
		attrs = {n: getattr(self, n) for n in attrnames}
		attrs['num_temps'] = len(self.temps)
		attrs['num_gpios'] = len(self.gpios)
		attrs['spi_setup'] = self._mangle_spi()
		attrs['status'] = not self.paused and (None if self.gcode_map is None and not self.gcode_file else True)
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
		self.gcode_fd = -1
	# }}}
	def _job_done(self, complete, reason): # {{{
		cdriver.run_file()
		if self.gcode_map is not None:
			log('job done: ' + reason)
			self._gcode_close()
		self.gcode_file = False
		#traceback.print_stack()
		if self.queue_info is None and self.gcode_id is not None:
			log('Job done (%d): %s' % (complete, reason))
			self._send(self.gcode_id, 'return', (complete, reason))
			self.gcode_id = None
		if self.audio_id is not None:
			log('Audio done (%d): %s' % (complete, reason))
			self._send(self.audio_id, 'return', (complete, reason))
		self.audio_id = None
		if self.queue_info is None and self.job_current is not None:
			if self.job_id is not None:
				self._send(self.job_id, 'return', (complete, reason))
			self.job_id = None
			self.job_current = None
			if complete:
				self._finish_done()
		while self.queue_pos < len(self.queue):
			move, e, tool, v, probe, single, rel = self.queue[self.queue_pos]
			self.queue_pos += 1
			if id is not None:
				self._send(id, 'error', 'aborted')
		self.queue = []
		self.queue_pos = 0
		if self.home_phase is not None:
			#log('killing homer')
			self.home_phase = None
			self.expert_set_space(0, type = self.home_orig_type)
			for a, ax in enumerate(self.spaces[0].axis):
				self.expert_set_axis((0, a), min = self.home_limits[a][0], max = self.home_limits[a][1])
			if self.home_cb in self.movecb:
				self.movecb.remove(self.home_cb)
				if self.home_id is not None:
					self._send(self.home_id, 'return', None)
		if self.probe_cb in self.movecb:
			#log('killing prober')
			self.movecb.remove(self.probe_cb)
			self.probe_cb[1](None)
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
		if self.gcode_file:
			cdriver.resume() # Just in case.
		if self.queue_info is None:
			return
		#log('doing resume to %d/%d' % (self.queue_info[0], len(self.queue_info[2])))
		self.queue = self.queue_info[2]
		self.queue_pos = self.queue_info[0]
		self.movecb = self.queue_info[3]
		self.flushing = self.queue_info[4]
		self.resuming = False
		self.queue_info = None
		self.paused = False
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
		cdriver.parse_gcode(infilename, outfilename)
		self._refresh_queue()
		self._broadcast(None, 'blocked', None)
		return name
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
	def _do_queue(self): # {{{
		#log('queue %s' % repr((self.queue_pos, len(self.queue), self.resuming, self.wait)))
		if self.paused and not self.resuming and len(self.queue) == 0:
			#log('queue is empty')
			return
		while not self.wait and (self.queue_pos < len(self.queue) or self.resuming):
			#log('queue not empty %s' % repr((self.queue_pos, len(self.queue), self.resuming, self.wait)))
			if self.queue_pos >= len(self.queue):
				self._unpause()
				#log('unpaused, %d %d' % (self.queue_pos, len(self.queue)))
				if self.queue_pos >= len(self.queue):
					break
			move, e, tool, v, probe, single, rel = self.queue[self.queue_pos]
			if tool is None:
				tool = self.current_extruder
			if e is None:
				e = float('nan')
			self.queue_pos += 1
			# Turn sequences into a dict.
			if isinstance(move, (list, tuple)):
				mdict = {}
				for s, data in enumerate(move):
					mdict[s] = data
				move = mdict
			# Make sure the keys are ints.
			move = {int(k): move[k] for k in move}
			# Turn into list of floats.
			move = [move[i] if i in move else float('nan') for i in range(6)]
			# Set current tool.
			if tool is not None:
				self.current_extruder = tool
			# Set defaults for feedrates.
			if all(math.isnan(x) for x in move) and self.current_extruder < len(self.spaces[1].motor) and (v is None or not 0 < v <= self.spaces[1].motor[self.current_extruder]['limit_v']):
				v = self.spaces[1].motor[self.current_extruder]['limit_v']
			elif v is None or not 0 < v <= self.max_v:
				v = self.max_v
			self.movewait += 1
			#log('movewait +1 -> %d' % self.movewait)
			#log('move args: ' + repr((self.current_extruder, move, e, v)))
			self.wait = cdriver.move(*([self.current_extruder] + move + [0, 0, 0, e, v]), single = single, probe = probe, relative = rel)
		if self.flushing is None and not self.wait:
			self.flushing = False
		#log('queue done %s' % repr((self.queue_pos, len(self.queue), self.resuming, self.wait)))
	# }}}
	def _do_home(self, done = None): # {{{
		#log('do_home: %s %s' % (self.home_phase, done))
		# 0: Prepare for next order.
		# 1: Move to limits. (enter from loop after 2).
		#  : If leader or follower hits limit: move only partner(s) until all hit limit.
		# 2: Finish moving to limits; loop home_order; move slowly away from switch.
		# 3: Set current position; move delta and followers.
		# 4: Move within limits.
		# 5: Return.
		#log('home %s %s' % (self.home_phase, repr(self.home_target)))
		#traceback.print_stack()
		home_v = 50 / self.feedrate
		if self.home_phase is None:
			#log('_do_home ignored because home_phase is None')
			return
		if self.home_phase == 0:
			if done is not None:
				# Continuing call received after homing was aborted; ignore.
				return
			# Initial call; start homing.
			self.home_phase = 1
			# If it is currently moving, doing the things below without pausing causes stall responses.
			self.user_pause(True, False)
			self.user_sleep(False)
			for i, e in enumerate(self.spaces[1].axis):
				self.user_set_axis_pos(1, i, 0)
			self.home_limits = [(a['min'], a['max']) for a in self.spaces[0].axis]
			for a, ax in enumerate(self.spaces[0].axis):
				self.expert_set_axis((0, a), min = float('-inf'), max = float('inf'))
			self.home_orig_type = self.spaces[0].type
			self.expert_set_space(0, type = TYPE_CARTESIAN)
			n = set()
			for m, mtr in enumerate(self.spaces[0].motor):
				if self._pin_valid(mtr['limit_min_pin']) or self._pin_valid(mtr['limit_max_pin']):
					n.add(mtr['home_order'])
				else:
					self.user_set_axis_pos(0, m, 0)
			if len(n) == 0:
				self.home_phase = 4
			else:
				self.home_order = min(n)
			# Fall through.
		if self.home_phase == 1:
			# Move to limit.
			self.home_phase = 2
			self.home_motors = []
			for i, m in enumerate(self.spaces[0].motor):
				if (self._pin_valid(m['limit_min_pin']) or self._pin_valid(m['limit_max_pin'])) and m['home_order'] == self.home_order:
					self.home_motors.append((i, self.spaces[0].axis[i], m))
			self.limits[0].clear()
			self.home_target = {}
			dist = 1000 #TODO: use better value.
			for i, a, m in self.home_motors:
				self.spaces[0].set_current_pos(i, 0)
				if self._pin_valid(m['limit_max_pin']):
					self.home_target[i] = dist - (0 if i != 2 else self.zoffset)
				else:
					self.home_target[i] = -dist - (0 if i != 2 else self.zoffset)
			if len(self.home_target) > 0:
				self.home_cb[0] = list(self.home_target.keys())
				if self.home_cb not in self.movecb:
					self.movecb.append(self.home_cb)
				#log("home phase %d target %s" % (self.home_phase, self.home_target))
				self.user_line(self.home_target, v = home_v, single = True, force = True)[1](None)
				return
			# Fall through.
		if self.home_phase == 2:
			# Continue moving to find limit switch.
			found_limits = False
			for a in self.limits[0].keys():
				if a in self.home_target:
					#log('found limit %d' % a)
					self.home_target.pop(a)
					found_limits = True
					# Make sure no attempt is made to move through the limit switch (not even by rounding errors).
					self.spaces[0].set_current_pos(a, self.spaces[0].get_current_pos(a))
			# Repeat until move is done, or all limits are hit.
			if (not done or found_limits) and len(self.home_target) > 0:
				self.home_cb[0] = list(self.home_target.keys())
				if self.home_cb not in self.movecb:
					self.movecb.append(self.home_cb)
				#log("0 t %s" % (self.home_target))
				k = tuple(self.home_target.keys())[0]
				dist = abs(self.home_target[k] - self.spaces[0].get_current_pos(k))
				if dist > 0:
					#log("home phase %d target %s" % (self.home_phase, self.home_target))
					self.user_line(self.home_target, v = home_v, single = True, force = True)[1](None)
					return
				# Fall through.
			if len(self.home_target) > 0:
				log('Warning: not all limits were found during homing')
			n = set()
			for m in self.spaces[0].motor:
				if (self._pin_valid(m['limit_min_pin']) or self._pin_valid(m['limit_max_pin'])) and m['home_order'] > self.home_order:
					n.add(m['home_order'])
			if len(n) > 0:
				self.home_phase = 1
				self.home_order = min(n)
				return self._do_home()
			# Move away slowly.
			data = []
			num = 0
			for m in self.spaces[0].motor:
				if self._pin_valid(m['limit_max_pin']):
					data.append(-1)
					num += 1
				elif self._pin_valid(m['limit_min_pin']):
					data.append(1)
					num += 1
				else:
					data.append(0)
			self.home_phase = 3
			if num > 0:
				dprint('homing', data)
				cdriver.home(*data)
				return
			# Fall through.
		if self.home_phase == 3:
			# Move followers and delta into alignment.
			self.home_return = []
			for i, m in enumerate(self.spaces[0].motor):
				if i in self.limits[0]:
					if not math.isnan(m['home_pos']):
						#log('set %d %d %f' % (0, i, m['home_pos']))
						self.home_return.append(m['home_pos'] - self.spaces[0].get_current_pos(i))
						self.spaces[0].set_current_pos(i, m['home_pos'])
					else:
						#log('limited zeroset %d %d' % (0, i))
						self.home_return.append(-self.spaces[0].get_current_pos(i))
						self.spaces[0].set_current_pos(i, 0)
				else:
					if (self._pin_valid(m['limit_min_pin']) or self._pin_valid(m['limit_max_pin'])) and not math.isnan(m['home_pos']):
						#log('defset %d %d %f' % (0, i, m['home_pos']))
						self.home_return.append(m['home_pos'] - self.spaces[0].get_current_pos(i))
						self.spaces[0].set_current_pos(i, m['home_pos'])
					else:
						#log('unlimited zeroset %d %d' % (0, i))
						self.home_return.append(-self.spaces[0].get_current_pos(i))
						self.spaces[0].set_current_pos(i, 0)
			self.home_phase = 4
			# FIXME: support homing of followers.
			'''
			# Align followers.
			for i, m in enumerate(self.spaces[2].motor):
				fs = self.spaces[2].follower[i]['space']
				fm = self.spaces[2].follower[i]['motor']
				# Use 2, not len(self.spaces), because following followers is not supported.
				if not 0 <= fs < 2 or not 0 <= fm < len(self.spaces[fs].motor):
					continue
				if self._pin_valid(m['limit_max_pin']):
					if not self._pin_valid(self.spaces[fs].motor[fm]['limit_max_pin']) and self._pin_valid(self.spaces[fs].motor[fm]['limit_min_pin']):
						# Opposite limit pin: don't compare values.
						groups[2].append((2, i))
						continue
					for g in groups[1]:
						if (fs, fm) in g:
							g.append((2, i))
							break
					else:
						groups[1].append([(2, i), (fs, fm)])
				elif self._pin_valid(m['limit_min_pin']):
					if self._pin_valid(self.spaces[fs].motor[fm]['limit_max_pin']):
						# Opposite limit pin: don't compare values.
						groups[2].append((2, i))
						continue
					for g in groups[0]:
						if (fs, fm) in g:
							g.append((2, i))
							break
					else:
						groups[0].append([(2, i), (fs, fm)])
			self.home_target = {}
			for g in groups[0]:
				target = max(g, key = lambda x: self.spaces[x[0]].motor[x[1]]['home_pos'])
				target = self.spaces[target[0]].motor[target[1]]['home_pos']
				for s, m in g:
					if target != self.spaces[s].motor[m]['home_pos']:
						offset = (0 if s != 0 or m != 2 else self.zoffset)
						self.home_target[(s, m)] = target - offset
			for g in groups[1]:
				target = min(g, key = lambda x: self.spaces[x[0]].motor[x[1]]['home_pos'])
				target = self.spaces[target[0]].motor[target[1]]['home_pos']
				for s, m in g:
					if target != self.spaces[s].motor[m]['home_pos']:
						offset = (0 if s != 0 or m != 2 else self.zoffset)
						self.home_target[(s, m)] = target - offset
			for s, m in groups[2]:
				fs = self.spaces[s].follower[m]['space']
				fm = self.spaces[s].follower[m]['motor']
				if self.spaces[fs].motor[fm]['home_pos'] != self.spaces[s].motor[m]['home_pos']:
					offset = (0 if s != 0 or m != 2 else self.zoffset)
					self.home_target[(s, m)] = self.spaces[fs].motor[fm]['home_pos'] - offset
			if len(self.home_target) > 0:
				self.home_cb[0] = False
				if self.home_cb not in self.movecb:
					self.movecb.append(self.home_cb)
				#log("home phase %d target %s" % (self.home_phase, self.home_target))
				self.user_line(self.home_target, single = True, force = True)[1](None)
				return
			'''
			# Fall through.
		if self.home_phase == 4:
			# Reset space type and move to pos2.
			self.expert_set_space(0, type = self.home_orig_type)
			for a, ax in enumerate(self.spaces[0].axis):
				self.expert_set_axis((0, a), min = self.home_limits[a][0], max = self.home_limits[a][1])
			target = {}
			for i, a in enumerate(self.spaces[0].axis):
				if not math.isnan(a['home_pos2']):
					offset = (0 if i != 2 else self.zoffset)
					target[i] = a['home_pos2'] - offset
			self.home_phase = 5
			if len(target) > 0:
				self.home_cb[0] = False
				if self.home_cb not in self.movecb:
					self.movecb.append(self.home_cb)
					#log("home phase %d target %s" % (self.home_phase, target))
				self.user_line(target, force = True)[1](None)
				return
			# Fall through.
		if self.home_phase == 5:
			# Move within bounds.
			target = {}
			for i, a in enumerate(self.spaces[0].axis):
				current = self.spaces[0].get_current_pos(i)
				offset = (0 if i != 2 else self.zoffset)
				if current > a['max'] - offset:
					target[i] = a['max'] - offset
				elif current < a['min'] - offset:
					target[i] = a['min'] - offset
			self.home_phase = 6
			if len(target) > 0:
				self.home_cb[0] = False
				if self.home_cb not in self.movecb:
					self.movecb.append(self.home_cb)
				#log("home phase %d target %s" % (self.home_phase, target))
				self.user_line(target, force = True)[1](None)
				#log('movecb: ' + repr(self.movecb))
				return
			# Fall through.
		if self.home_phase == 6:
			self.home_phase = None
			self.position_valid = True
			if self.home_id is not None:
				self._send(self.home_id, 'return', self.home_return)
				self.home_return = None
			if self.home_done_cb is not None:
				call_queue.append((self.home_done_cb, []))
				self.home_done_cb = None
			return
		log('Internal error: invalid home phase')
	# }}}
	def _handle_one_probe(self, good): # {{{
		if good is None:
			return
		pos = self.get_axis_pos(0)
		cdriver.adjust_probe(pos[0], pos[1], pos[2] + self.zoffset)
		self.probe_cb[1] = lambda good: self.user_request_confirmation("Continue?")[1](False) if good is not None else None
		self.movecb.append(self.probe_cb)
		self.user_line({2: self.probe_safe_dist}, relative = True)[1](None)
	# }}}
	def _one_probe(self): # {{{
		self.probe_cb[1] = self._handle_one_probe
		self.movecb.append(self.probe_cb)
		z = self.get_axis_pos(0, 2)
		z_low = self.spaces[0].axis[2]['min']
		self.user_line({2: z_low}, f0 = float(self.probe_speed) / (z - z_low) if z > z_low else float('inf'), probe = True)[1](None)
	# }}}
	def _do_probe(self, id, x, y, z, phase = 0, good = True): # {{{
		#log('probe %d %s' % (phase, good))
		# Map = [[x0, y0, x1, y1], [nx, ny, angle], [[...], [...], ...]]
		if good is None:
			# This means the probe has been aborted.
			#log('abort probe')
			self.probing = False
			if id is not None:
				self._send(id, 'error', 'aborted')
			#self._job_done(False, 'Probe aborted')
			return
		self.probing = True
		if not self.position_valid:
			self.user_home(cb = lambda: self._do_probe(id, x, y, z, phase, True), abort = False)[1](None)
			return
		p = self.probemap
		if phase == 0:
			if y > p[1][1]:
				# Done.
				self.probing = False
				self._check_probemap()
				if id is not None:
					self._send(id, 'return', p)
				for y, c in enumerate(p[2]):
					for x, o in enumerate(c):
						log('map %f %f %f' % (p[0][0] + p[0][2] * x / p[1][0], p[0][1] + p[0][3] * y / p[1][1], o))
					sys.stderr.write('\n')
				return
			# Goto x,y
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, z, 1, good)
			self.movecb.append(self.probe_cb)
			px = p[0][2] + p[0][4] * x / p[1][0]
			py = p[0][3] + p[0][5] * y / p[1][1]
			log(repr((p, px, py, x, y, self.gcode_angle)))
			self.user_line([p[0][0] + px * self.gcode_angle[1] - py * self.gcode_angle[0], p[0][1] + py * self.gcode_angle[1] + px * self.gcode_angle[0]])[1](None)
		elif phase == 1:
			# Probe
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, z, 2, good)
			if self._pin_valid(self.probe_pin):
				self.movecb.append(self.probe_cb)
				z_low = self.spaces[0].axis[2]['min']
				self.user_line({2: z_low}, f0 = float(self.probe_speed) / (z - z_low) if z > z_low else float('inf'), probe = True)[1](None)
			else:
				#log('confirm probe')
				self.user_request_confirmation('Please move the tool to the surface')[1](False)
		else:
			# Record result
			if good:
				log('Warning: probe did not hit anything')
			z = self.spaces[0].get_current_pos(2)
			p[2][y][x].append(z + self.zoffset)
			if len(p[2][y][x]) >= self.num_probes:
				p[2][y][x].sort()
				trash = self.num_probes // 3
				if trash == 0:
					p[2][y][x] = sum(p[2][y][x]) / len(p[2][y][x])
				else:
					p[2][y][x] = sum(p[2][y][x][trash:-trash]) / (len(p[2][y][x]) - 2 * trash)
				if y & 1:
					x -= 1
					if x < 0:
						x = 0
						y += 1
				else:
					x += 1
					if x > p[1][0]:
						x = p[1][0]
						y += 1
			z += self.probe_safe_dist
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, z, 0, good)
			self.movecb.append(self.probe_cb)
			# Retract
			self.user_line({2: z})[1](None)
	# }}}
	def _check_probemap(self): # {{{
		'''Check the probemap, and save it if it is valid; discard it otherwise.
		@returns: True if the probemap is valid, False otherwise.'''
		if not isinstance(self.probemap, (list, tuple)) or len(self.probemap) != 3:
			log('probemap check failed: not a list of length 3')
			self.probemap = None
			self._globals_update()
			return False
		limits, nums, probes = self.probemap
		if not isinstance(limits, (list, tuple)) or not isinstance(nums, (list, tuple)) or len(limits) != 6 or len(nums) != 3:
			log('probemap check failed: first lists are not length 6 and 3')
			self.probemap = None
			self._globals_update()
			return False
		if not all(isinstance(e, (float, int)) and not math.isnan(e) for e in limits):
			log('probemap check failed: limits must be numbers')
			self.probemap = None
			self._globals_update()
			return False
		if not all(isinstance(e, t) and not math.isnan(e) for e, t in zip(nums, (int, int, (float, int)))):
			log('probemap check failed: nums and angle must be numbers')
			self.probemap = None
			self._globals_update()
			return False
		nx, ny, angle = nums
		if len(probes) != ny + 1 or not all(isinstance(e, (list, tuple)) and len(e) == nx + 1 for e in probes):
			log('probemap check failed: probe map is incorrect size')
			self.probemap = None
			self._globals_update()
			return False
		if not all(all(isinstance(e, (float, int)) and not math.isnan(e) for e in f) for f in probes):
			log('probemap check failed: probe points must all be numbers')
			self.probemap = None
			self._globals_update()
			return False
		with fhs.write_spool(os.path.join(self.uuid, 'probe' + os.extsep + 'bin'), text = False) as probemap_file:
			# Map = [[x, y, w, h], [nx, ny], [[...], [...], ...]]
			sina, cosa = self.gcode_angle
			targetx, targety, x0, y0, w, h = self.probemap[0]
			probemap_file.write(struct.pack('@ddddddddLLd', targetx, targety, x0, y0, w, h, sina, cosa, *self.probemap[1]))
			for y in range(self.probemap[1][1] + 1):
				for x in range(self.probemap[1][0] + 1):
					probemap_file.write(struct.pack('@d', self.probemap[2][y][x]))
		self._globals_update()
		return True
	# }}}
	def _start_job(self, paused): # {{{
		# Set all extruders to 0.
		for i, e in enumerate(self.spaces[1].axis):
			self.user_set_axis_pos(1, i, 0)
		def cb():
			#log('start job %s' % self.job_current)
			self._gcode_run(self.job_current, abort = False, paused = paused)
		if not self.position_valid:
			self.user_park(cb = cb, abort = False)[1](None)
		else:
			cb()
		self.gcode_id = None
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
		self.queue_info = None
		# Disable all alarms.
		for i in range(len(self.temps)):
			self.user_waittemp(i, None, None)
		self.paused = paused
		self._globals_update()
		self.user_sleep(False)
		if len(self.spaces) > 1:
			for e in range(len(self.spaces[1].axis)):
				self.user_set_axis_pos(1, e, 0)
		filename = fhs.read_spool(os.path.join(self.uuid, 'gcode', src + os.extsep + 'bin'), text = False, opened = False)
		self.total_time = self.jobqueue[src][-2:]
		self.gcode_fd = os.open(filename, os.O_RDONLY)
		self.gcode_map = mmap.mmap(self.gcode_fd, 0, prot = mmap.PROT_READ)
		filesize = os.fstat(self.gcode_fd).st_size
		bboxsize = 8 * struct.calcsize('=d')
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
		if self.probemap is None:
			encoded_probemap_filename = b''
		else:
			encoded_probemap_filename = fhs.read_spool(os.path.join(self.uuid, 'probe' + os.extsep + 'bin'), text = False, opened = False).encode('utf-8')
		self.gcode_file = True
		self._globals_update()
		log('running %s %f %f' % (filename, self.gcode_angle[0], self.gcode_angle[1]))
		cdriver.run_file(filename.encode('utf-8'), encoded_probemap_filename, 1 if not paused and self.confirmer is None else 0, self.gcode_angle[0], self.gcode_angle[1], -1)
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
			self.type = [TYPE_CARTESIAN, TYPE_EXTRUDER, TYPE_FOLLOWER][id]
			self.machine = machine
			self.id = id
			self.axis = []
			self.motor = []
			self.delta = [{'axis_min': 0., 'axis_max': 0., 'rodlength': 0., 'radius': 0.} for t in range(3)]
			self.delta_angle = 0
			self.polar_max_r = float('inf')
			self.extruder = []
			self.follower = []
		def read(self):
			self.read_info()
			for a in range(len(self.axis)):
				self.read_axis(a)
			for m in range(len(self.motor)):
				self.read_motor(m)
		def read_info(self):
			data = cdriver.read_space_info(self.id)
			self.type = data.pop('type')
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
				self.axis += [{'name': nm(i), 'home_pos2': float('nan')} for i in range(len(self.axis), num_axes)]
			else:
				self.axis[num_axes:] = []
			if num_motors > len(self.motor):
				self.motor += [{'unit': self.machine.unit_name} if self.id == 1 else {} for i in range(len(self.motor), num_motors)]
			else:
				self.motor[num_motors:] = []
			if self.type == TYPE_CARTESIAN:
				for m in self.motor:
					m['unit'] = self.machine.unit_name
			elif self.type == TYPE_DELTA:
				for m in self.motor:
					m['unit'] = self.machine.unit_name
				for a, d in enumerate(data.pop('delta')):
					for k in d:
						self.delta[a][k] = d[k]
				self.delta_angle = data['delta_angle']
			elif self.type == TYPE_POLAR:
				for i, m in enumerate(self.motor):
					m['unit'] = self.machine.unit_name if i != 1 else '°'
				self.polar_max_r = data['max_r']
			elif self.type == TYPE_EXTRUDER:
				for m in self.motor:
					if 'unit' not in m:
						m['unit'] = self.machine.unit_name
				offset = data.pop('offset')
				for a in range(num_axes):
					for i, k in enumerate(('dx', 'dy', 'dz')):
						self.extruder[a][k] = offset[a][i]
			elif self.type == TYPE_FOLLOWER:
				follow = data.pop('follow')
				for a in range(num_axes):
					for i, x in enumerate(('space', 'motor')):
						self.follower[a][x] = follow[a][i]
					s, m = follow[a]
					self.motor[a]['unit'] = self.machine.unit_name if not 0 <= s <= 1 or not 0 <= m < len(self.machine.spaces[s].motor) else self.machine.spaces[s].motor[m]['unit']
			else:
				log('invalid type')
				raise AssertionError('invalid space type')
		def read_axis(self, a):
			data = cdriver.read_space_axis(self.id, a)
			for k in data:
				self.axis[a][k] = data[k]
		def read_motor(self, m):
			data = cdriver.read_space_motor(self.id, m)
			for k in data:
				self.motor[m][k] = data[k]
			if self.id == 1 and m < len(self.machine.multipliers):
				self.motor[m]['steps_per_unit'] /= self.machine.multipliers[m]
		def write_info(self, num_axes = None):
			if num_axes is None:
				num_axes = len(self.axis)
			data = {'type': self.type, 'num_axes': num_axes}
			if self.type == TYPE_CARTESIAN:
				pass
			elif self.type == TYPE_DELTA:
				data['delta'] = [{k: self.delta[a][k] for k in ('axis_min', 'axis_max', 'rodlength', 'radius')} for a in range(3)]
				data['delta_angle'] = self.delta_angle
			elif self.type == TYPE_POLAR:
				data['max_r'] = self.polar_max_r
			elif self.type == TYPE_EXTRUDER:
				data['offset'] = [tuple(self.extruder[a][x] for x in ('dx', 'dy', 'dz')) if a < len(self.extruder) else (0, 0, 0) for a in range(num_axes)]
			elif self.type == TYPE_FOLLOWER:
				data['follow'] = [tuple(self.follower[a][x] for x in ('space', 'motor')) if a < len(self.follower) else (-1, -1) for a in range(num_axes)]
			else:
				log('invalid type')
				raise AssertionError('invalid space type')
			#log('writing info: %s' % repr(data))
			cdriver.write_space_info(self.id, data)
		def write_axis(self, axis):
			data = {'park_order': 0, 'park': float('nan'), 'min': float('-inf'), 'max': float('inf')}
			if self.id == 0:
				for k in data.keys():
					data[k] = self.axis[axis][k]
			cdriver.write_space_axis(self.id, axis, data)
		def write_motor(self, motor):
			if self.id == 2:
				if self.follower[motor]['space'] >= len(self.machine.spaces) or self.follower[motor]['motor'] >= len(self.machine.spaces[self.follower[motor]['space']].motor):
					#log('write motor for follower %d with fake base' % motor)
					base = {'steps_per_unit': 1, 'limit_v': float('inf'), 'limit_a': float('inf')}
				else:
					#log('write motor for follower %d with base %s' % (motor, self.machine.spaces[0].motor))
					base = self.machine.spaces[self.follower[motor]['space']].motor[self.follower[motor]['motor']]
			else:
				base = self.motor[motor]
			# don't include unit, because cdriver doesn't use it.
			data = {x: self.motor[motor][x] for x in ('step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'home_pos', 'home_order')}
			data.update({x: base[x] for x in ('limit_v', 'limit_a', 'steps_per_unit')})
			if self.id == 1 and motor < len(self.machine.multipliers):
				data['steps_per_unit'] *= self.machine.multipliers[motor]
			cdriver.write_space_motor(self.id, motor, data)
		def set_current_pos(self, axis, pos):
			#log('setting pos of %d %d to %f' % (self.id, axis, pos))
			cdriver.setpos(self.id, axis, pos)
		def get_current_pos(self, axis):
			#log('getting current pos %d %d' % (self.id, axis))
			if not self.machine.connected:
				return float('nan')
			return cdriver.getpos(self.id, axis)
		def motor_name(self, i):
			if self.type in (TYPE_CARTESIAN, TYPE_EXTRUDER, TYPE_FOLLOWER):
				return self.axis[i]['name']
			elif self.type == TYPE_DELTA:
				return chr(ord('u') + i)
			elif self.type == TYPE_POLAR:
				return ['r', 'θ', 'z'][i]
			else:
				log('invalid type')
				raise AssertionError('invalid space type')
		def export(self):
			std = [self.name, self.type, [[a['name'], a['park'], a['park_order'], a['min'], a['max'], a['home_pos2']] for a in self.axis], [[self.motor_name(i), m['step_pin'], m['dir_pin'], m['enable_pin'], m['limit_min_pin'], m['limit_max_pin'], m['steps_per_unit'], m['home_pos'], m['limit_v'], m['limit_a'], m['home_order'], m['unit']] for i, m in enumerate(self.motor)], None if self.id != 1 else self.machine.multipliers]
			if self.type == TYPE_CARTESIAN:
				return std
			elif self.type == TYPE_DELTA:
				return std + [[[a['axis_min'], a['axis_max'], a['rodlength'], a['radius']] for a in self.delta] + [self.delta_angle]]
			elif self.type == TYPE_POLAR:
				return std + [self.polar_max_r]
			elif self.type == TYPE_EXTRUDER:
				return std + [[[a['dx'], a['dy'], a['dz']] for a in self.extruder]]
			elif self.type == TYPE_FOLLOWER:
				return std + [[[a['space'], a['motor']] for a in self.follower]]
			else:
				log('invalid type')
				raise AssertionError('invalid space type')
		def export_settings(self):
			# Things to handle specially while homing:
			# * self.home_limits = [(a['min'], a['max']) for a in self.spaces[0].axis]
			# * self.home_orig_type = self.spaces[0].type
			ret = '[space %d]\r\n' % self.id
			type = self.type if self.id != 0 or self.machine.home_phase is None else self.machine.home_orig_type
			if self.id == 0:
				ret += 'type = %d\r\n' % type
			if type == TYPE_CARTESIAN:
				ret += 'num_axes = %d\r\n' % len(self.axis)
			elif type == TYPE_DELTA:
				ret += 'delta_angle = %f\r\n' % self.delta_angle
				for i in range(3):
					ret += '[delta %d %d]\r\n' % (self.id, i)
					ret += ''.join(['%s = %f\r\n' % (x, self.delta[i][x]) for x in ('rodlength', 'radius', 'axis_min', 'axis_max')])
			elif type == TYPE_POLAR:
				ret += 'polar_max_r = %f\r\n' % self.polar_max_r
			elif type == TYPE_EXTRUDER:
				ret += 'num_axes = %d\r\n' % len(self.axis)
				for i in range(len(self.extruder)):
					ret += '[extruder %d %d]\r\n' % (self.id, i)
					ret += ''.join(['%s = %f\r\n' % (x, self.extruder[i][x]) for x in ('dx', 'dy', 'dz')])
			elif type == TYPE_FOLLOWER:
				ret += 'num_axes = %d\r\n' % len(self.axis)
				for i in range(len(self.follower)):
					ret += '[follower %d %d]\r\n' % (self.id, i)
					ret += ''.join(['%s = %d\r\n' % (x, self.follower[i][x]) for x in ('space', 'motor')])
			else:
				log('invalid type')
				raise AssertionError('invalid space type')
			for i, a in enumerate(self.axis):
				ret += '[axis %d %d]\r\n' % (self.id, i)
				ret += 'name = %s\r\n' % a['name']
				if self.id == 0:
					ret += ''.join(['%s = %f\r\n' % (x, a[x]) for x in ('park', 'park_order', 'home_pos2')])
					if self.machine.home_phase is None:
						ret += ''.join(['%s = %f\r\n' % (x, a[x]) for x in ('min', 'max')])
					else:
						ret += ''.join(['%s = %f\r\n' % (x, y) for x, y in zip(('min', 'max'), self.machine.home_limits[self.id])])
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
			attrnames = ('R0', 'R1', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time')
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
			attrnames = ('name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time', 'value')
			return {n: getattr(self, n) for n in attrnames}
		def export_settings(self):
			ret = '[temp %d]\r\n' % self.id
			ret += 'name = %s\r\n' % self.name
			ret += ''.join(['%s = %s\r\n' % (x, write_pin(getattr(self, x))) for x in ('heater_pin', 'fan_pin', 'thermistor_pin')])
			ret += ''.join(['%s = %f\r\n' % (x, getattr(self, x)) for x in ('fan_temp', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time')])
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
		def read(self):
			data = cdriver.read_gpio(self.id)
			self.pin = data['pin']
			self.duty = data['duty']
			self.state = data['state'] & 0x3
			self.reset = (data['state'] >> 2) & 0x3
		def write(self):
			attrnames = ('pin', 'duty')
			data = {n: getattr(self, n) for n in attrnames}
			data['state'] = self.state | (self.reset << 2)
			cdriver.write_gpio(self.id, data)
		def export(self):
			attrnames = ('name', 'pin', 'state', 'reset', 'duty')
			attrs = {n: getattr(self, n) for n in attrnames}
			attrs['value'] = self.value if self.state >= 2 else self.state == 1
			return attrs
		def export_settings(self):
			ret = '[gpio %d]\r\n' % self.id
			ret += 'name = %s\r\n' % self.name
			ret += 'pin = %s\r\n' % write_pin(self.pin)
			ret += 'reset = %d\r\n' % self.reset
			ret += 'duty = %f\r\n' % self.duty
			return ret
	# }}}
	# }}}
	# }}}
	# Useful commands.  {{{
	def admin_reset_uuid(self): # {{{
		uuid = protocol.new_uuid(string = False)
		cdriver.set_uuid(bytes(uuid))
		self.uuid = protocol.new_uuid(uuid = uuid, string = True)
		if not self.name:
			self.name = self.uuid
		return self.uuid
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
				log('Could not remove %d' % dirname)
		# Clean up profiles.
		for dirname in fhs.read_data(self.uuid, dir = True, multiple = True, opened = False):
			try:
				shutil.rmtree(dirname, ignore_errors = False)
			except:
				log('Could not remove %d' % dirname)
		return (WAIT, WAIT)
	# }}}
	@delayed
	def flush(self, id): # {{{
		'''Wait for currently scheduled moves to finish.
		'''
		#log('flush start')
		def cb(w):
			#log('flush done')
			if id is not  None:
				self._send(id, 'return', w)
		self.movecb.append((False, cb))
		if self.flushing is not True:
			self.user_line()[1](None)
		#log('end flush preparation')
	# }}}
	@delayed
	def user_probe(self, id, area, speed = 3.): # {{{
		'''Run a probing routine.
		This moves over the given area and probes a grid of points less
		than max_probe_distance apart.
		If the probe pin is valid, it will be used for the probe.
		If it is invalid, a confirmation is required for every point.
		'''
		if area is None:
			try:
				fhs.remove_spool(os.path.join(self.uuid, 'probe' + os.extsep + 'bin'))
			except:
				log('Failed to remove probe file.')
				traceback.print_exc()
			self.probemap = None
			self._globals_update()
			if id is not None:
				self._send(id, 'return', None)
			return
		if len(self.spaces[0].axis) < 3 or not self.probe_safe_dist > 0:
			if id is not None:
				self._send(id, 'return', None)
			return
		log(repr(area))
		density = [int(area[t + 4] / self.probe_dist) + 1 for t in range(2)] + [self.targetangle]
		self.probemap = [area, density, [[[] for x in range(density[0] + 1)] for y in range(density[1] + 1)]]
		self.gcode_angle = math.sin(self.targetangle), math.cos(self.targetangle)
		self.probe_speed = speed
		self._do_probe(id, 0, 0, self.get_axis_pos(0, 2))
	# }}}
	@delayed
	def user_line(self, id, moves = (), e = None, tool = None, v = None, relative = False, probe = False, single = False, force = False): # {{{
		'''Move the tool in a straight line; return when done.
		'''
		if not force and self.home_phase is not None and not self.paused:
			log('ignoring line during home')
			if id is not None:
				self._send(id, 'return', None)
			return
		self.queue.append((moves, e, tool, v, probe, single, relative))
		if not self.wait:
			#log('doing queue now')
			self._do_queue()
		else:
			#log('waiting for queue to be ready')
			pass
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
			if self.home_phase is not None or (not force and not self.paused and (self.gcode_map is not None or self.gcode_file)):
				return
			self.position_valid = False
			if update:
				self._globals_update()
		cdriver.sleep(sleeping)
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
	def user_abort(self): # {{{
		'''Abort the current job.
		'''
		for t, temp in enumerate(self.temps):
			self.user_settemp(t, float('nan'))
		self.user_pause(store = False)
		for g, gpio in enumerate(self.gpios):
			self.user_set_gpio(g, state = gpio.reset)
		self._job_done(False, 'aborted by user')
		# Sleep doesn't work as long as home_phase is non-None, so do it after _job_done.
		self.user_sleep(force = True)
	# }}}
	def user_pause(self, pausing = True, store = True, update = True): # {{{
		'''Pause or resume the machine.
		'''
		was_paused = self.paused
		if pausing:
			num_in_queue = cdriver.queued(True)
			self.movewait = 0
			self.wait = False
		self.paused = pausing
		if not self.paused:
			if was_paused:
				# Go back to pausing position.
				# First reset all axes that don't have a limit switch.
				if self.queue_info is not None:
					self._reset_extruders(self.queue_info[1])
					self.user_line(self.queue_info[1])[1](None)
				# TODO: adjust extrusion of current segment to shorter path length.
				#log('resuming')
				self.resuming = True
			#log('sending resume')
			cdriver.resume()
			self._do_queue()
		else:
			#log('pausing')
			if not was_paused:
				#log('pausing %d %d %d %d %d' % (store, self.queue_info is None, len(self.queue), self.queue_pos, s))
				if store and self.queue_info is None and ((len(self.queue) > 0 and self.queue_pos - num_in_queue >= 0) or self.gcode_file):
					if self.home_phase is not None:
						#log('killing homer')
						self.home_phase = None
						self.expert_set_space(0, type = self.home_orig_type)
						for a, ax in enumerate(self.spaces[0].axis):
							self.expert_set_axis((0, a), min = self.home_limits[a][0], max = self.home_limits[a][1])
						if self.home_cb in self.movecb:
							self.movecb.remove(self.home_cb)
							if self.home_id is not None:
								self._send(self.home_id, 'return', None)
						store = False
					if self.probe_cb in self.movecb:
						self.movecb.remove(self.probe_cb)
						self.probe_cb[1](None)
						store = False
					#log('pausing gcode %d/%d/%d' % (self.queue_pos, num_in_queue, len(self.queue)))
					if self.flushing is None:
						self.flushing = False
					if store:
						self.queue_info = [len(self.queue) if self.gcode_file else self.queue_pos - num_in_queue, [[s.get_current_pos(a) for a in range(len(s.axis))] for s in self.spaces], self.queue, self.movecb, self.flushing]
				else:
					#log('stopping')
					self.paused = False
					if self.probe_cb in self.movecb:
						self.movecb.remove(self.probe_cb)
						self.probe_cb[1](None)
					if len(self.movecb) > 0:
						call_queue.extend([(x[1], [False]) for x in self.movecb])
				self.queue = []
				self.movecb = []
				self.flushing = False
				self.queue_pos = 0
		if update:
			self._globals_update()
	# }}}
	def queued(self): # {{{
		'''Get the number of currently queued segments.
		'''
		return cdriver.queued(False)
	# }}}
	@delayed
	def user_home(self, id, speed = 5, cb = None, abort = True): # {{{
		'''Recalibrate the position with its limit switches.
		'''
		if self.home_phase is not None and not self.paused:
			log("ignoring request to home because we're already homing")
			if id is not None:
				self._send(id, 'return', None)
			return
		# Abort only if it is requested, and the job is not paused.
		if abort and self.queue_info is None:
			self._job_done(False, 'aborted by homing')
		self.home_phase = 0
		self.home_id = id
		self.home_return = None
		self.home_speed = speed
		self.home_done_cb = cb
		self._do_home()
	# }}}
	@delayed
	def user_park(self, id, cb = None, abort = True, order = 0, aborted = False): # {{{
		'''Go to the park position.
		Home first if the position is unknown.
		'''
		if aborted:
			if id is not None:
				self._send(id, 'error', 'aborted')
			return
		#log('parking with cb %s' % repr(cb))
		if abort and self.queue_info is None:
			self._job_done(False, 'aborted by parking')
		self.parking = True
		if not self.position_valid:
			#log('homing')
			self.user_home(cb = lambda: self.user_park(cb, abort = False)[1](id), abort = False)[1](None)
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
				self.movecb.append((False, wrap_cb))
				self.user_line()[1](None)
			else:
				if id is not None:
					self._send(id, 'return', None)
			return
		#log('not done parking: ' + repr((next_order)))
		self.movecb.append((False, lambda done: self.user_park(cb, abort = False, order = next_order + 1, aborted = not done)[1](id)))
		self.user_line([a['park'] - (0 if ai != 2 else self.zoffset) if a['park_order'] == next_order else float('nan') for ai, a in enumerate(self.spaces[0].axis)])[1](None)
	# }}}
	@delayed
	def benjamin_audio_play(self, id, name, motor = 2): # {{{
		self.audio_id = id
		self.user_sleep(False)
		filename = fhs.read_spool(os.path.join(self.uuid, 'audio', name + os.extsep + 'bin'), opened = False)
		cdriver.run_file(filename.encode('utf-8'), b'', 1, 0, 0, motor, 0)
	# }}}
	def benjamin_audio_add_POST(self, filename, name): # {{{
		with open(filename, 'rb') as f:
			self._audio_add(f, name)
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
	def wait_for_cb(self, id): # {{{
		'''Block until the move queue is empty.
		'''
		ret = lambda w: id is None or self._send(id, 'return', w)
		if self.movewait == 0:
			#log('not delaying with wait_for_cb, because there is no cb waiting')
			ret(self.movewait == 0)
		else:
			#log('waiting for cb')
			self.movecb.append((True, ret))
	# }}}
	def waiting_for_cb(self): # {{{
		'''Check if any process is waiting for the move queue to be empty.
		'''
		return self.movewait > 0
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
		message += ''.join(['%s = %s\r\n' % (x, write_pin(getattr(self, x))) for x in ('led_pin', 'stop_pin', 'probe_pin', 'spiss_pin')])
		message += ''.join(['%s = %d\r\n' % (x, getattr(self, x)) for x in ('bed_id', 'fan_id', 'spindle_id', 'park_after_job', 'sleep_after_job', 'cool_after_job', 'timeout')])
		message += ''.join(['%s = %f\r\n' % (x, getattr(self, x)) for x in ('probe_dist', 'probe_offset', 'probe_safe_dist', 'temp_scale_min', 'temp_scale_max', 'max_deviation', 'max_v', 'max_a')])
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
		regexp = re.compile('\s*\[(general|(space|temp|gpio|(extruder|axis|motor|delta|follower)\s+(\d+))\s+(\d+))\]\s*$|\s*(\w+)\s*=\s*(.*?)\s*$|\s*(?:#.*)?$')
		#1: (general|(space|temp|gpio|(axis|motor|delta)\s+(\d+))\s+(\d+))	1 section
		#2: (space|temp|gpio|(extruder|axis|motor|delta)\s+(\d+))		2 section with index
		#3: (extruder|axis|motor|delta)						3 sectionname with two indices
		#4: (\d+)								4 index of space
		#5: (\d+)								5 only or component index
		#6: (\w+)								6 identifier
		#7: (.*?)								7 value
		errors = []
		globals_changed = True
		changed = {'space': set(), 'temp': set(), 'gpio': set(), 'axis': set(), 'motor': set(), 'extruder': set(), 'delta': set(), 'follower': set()}
		keys = {
				'general': {'num_temps', 'num_gpios', 'user_interface', 'pin_names', 'led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'probe_dist', 'probe_offset', 'probe_safe_dist', 'bed_id', 'fan_id', 'spindle_id', 'unit_name', 'timeout', 'temp_scale_min', 'temp_scale_max', 'park_after_job', 'sleep_after_job', 'cool_after_job', 'spi_setup', 'max_deviation', 'max_v', 'max_a'},
				'space': {'type', 'num_axes', 'delta_angle', 'polar_max_r'},
				'temp': {'name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time'},
				'gpio': {'name', 'pin', 'state', 'reset', 'duty'},
				'axis': {'name', 'park', 'park_order', 'min', 'max', 'home_pos2'},
				'motor': {'step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'steps_per_unit', 'home_pos', 'limit_v', 'limit_a', 'home_order', 'unit'},
				'extruder': {'dx', 'dy', 'dz'},
				'delta': {'axis_min', 'axis_max', 'rodlength', 'radius'},
				'follower': {'space', 'motor'}
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
						# Two indices: axis, motor, extruder, delta, follower.
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
			if not r.group(6):
				# Comment or empty line.
				continue
			key = r.group(6)
			value = r.group(7)
			try:
				if key == 'pin_names':
					if len(self.pin_names) > 0:
						# Don't override hardware-provided names.
						continue
					if value.strip() == '':
						# Avoid errors when empty.
						continue
					value = [[int(x[0]), x[1:]] for x in value.split(',')]
				elif 'name' in key or key == 'user_interface':
					pass	# Keep strings as they are.
				elif key == 'spi_setup':
					value = self._unmangle_spi(value)
				elif key.endswith('pin'):
					value = read_pin(self, value)
					#log('pin imported as {} for {}'.format(value, key))
				elif key.startswith('num') or section == 'follower' or key.endswith('_id'):
					value = int(value)
				else:
					value = float(value)
			except ValueError:
				errors.append((l, 'invalid value for %s' % key))
				continue
			if key not in keys[section] or (section == 'motor' and ((key in ('home_pos', 'home_order') and index[0] == 1) or (key in ('steps_per_unit', 'limit_v', 'limit_a') and index[0] == 2))):
				errors.append((l, 'invalid key for section %s' % section))
				continue
			# If something critical is changed, update instantly.
			if key.startswith('num') or key == 'type':
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
						for i in changed['delta']:
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
		for index in changed['delta']:
			changed['space'].add(index[0])
		for section in changed:
			for index in changed[section]:
				if not isinstance(index, tuple):
					continue
				if section not in ('follower', 'delta', 'extruder'):
					#log('setting non-{delta,follower} %s %s' % (section, index))
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
		return ', '.join('%s (%s)' % (msg, ln) for ln, msg in self.expert_import_settings(open(filename).read(), name))
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
		self.confirm_axes = [[s.get_current_pos(a) for a in range(len(s.axis))] for s in self.spaces]
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
			if self.probing:
				call_queue.append((self.probe_cb[1], [False if success else None]))
			else:
				if not success:
					self.probe_pending = False
					self._job_done(False, 'aborted by failed confirmation')
				else:
					if self.probe_pending and self._pin_valid(self.probe_pin):
						self.probe_pending = False
						call_queue.append((self._one_probe, []))
					else:
						self.probe_pending = False
						cdriver.resume()
		return True
	# }}}
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
	def probe_add_POST(self, filename, name): # {{{
		'''Set probe map using a POST request.
		Note that this function can only be called using POST; not with the regular websockets system.
		'''
		with open(filename) as f:
			self.probemap = json.loads(f.read().strip())
		return '' if self._check_probemap() else 'Invalid probemap'
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
	@delayed
	def user_queue_run(self, id, name, paused = False): # {{{
		'''Start a new job.
		'''
		if self.probing:
			log('ignoring run request while probe is in progress')
			if id is not None:
				self._send(id, 'return', None)
			return
		if self.job_current is not None and not self.paused:
			log('ignoring run request while job is in progress: %s ' % repr(self.job_current) + str(self.paused))
			if id is not None:
				self._send(id, 'return', None)
			return
		#log('set active jobs to %s' % names)
		self.job_current = name
		if self.job_current is not None:
			self.job_id = id
			self._start_job(paused)
		elif id is not None:
			self._send(id, 'return', None)
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
		elif self.gcode_map is not None or self.gcode_file:
			state = 'Running'
		else:
			return 'Idle', float('nan'), float('nan'), pos[0], pos[1], context
		return state, cdriver.get_time(), (self.total_time[0] + (0 if len(self.spaces) < 1 else self.total_time[1] / self.max_v)) / self.feedrate, pos[0], pos[1], context
	# }}}
	def send_machine(self, target): # {{{
		'''Return all settings about a machine.
		'''
		log('sending machine ' + repr(self.uuid))
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
	# Commands for handling the toolpath.
	def tp_get_position(self): # {{{
		'''Get current toolpath position.
		@return position, total toolpath length.'''
		if self.gcode_map is None:
			return 0, 0
		return cdriver.tp_getpos(), self.gcode_num_records
	# }}}
	def user_tp_set_position(self, position): # {{{
		'''Set current toolpath position.
		It is an error to call this function while not paused.
		@param position: new toolpath position.
		@return None.'''
		assert self.gcode_map is not None
		assert 0 <= position < self.gcode_num_records
		assert self.paused
		if self.queue_info is not None:
			self.queue_info[1] = []	# Don't restore extruder position on resume.
		cdriver.tp_setpos(position)
	# }}}
	def tp_get_context(self, num = None, position = None): # {{{
		'''Get context around a position.
		@param num: number of lines context on each side.
		@param position: center of the returned region, or None for current position.
		@return first position of returned region (normally position - num), list of lines+arcs+specials'''
		if self.gcode_map is None:
			return 0, []
		if num is None:
			num = 100;	# TODO: make configurable.
		if position is None:
			position = self.tp_get_position()[0]
		position = int(position)
		def parse_record(num):
			s = struct.calcsize(record_format)
			type, tool, X, Y, Z, Bx, By, Bz, E, v0, v1, time, dist, r = struct.unpack(record_format, self.gcode_map[num * s:(num + 1) * s])
			#log('get context type %d' % type)
			return tuple(protocol.parsed.keys())[tuple(protocol.parsed.values()).index(type)], tool, X, Y, Z, Bx, By, Bz, E, v0, v1, time, dist, r
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
		assert self.gcode_map is not None
		return cdriver.tp_findpos(*(a if a is not None else float('nan') for a in (x, y, z)))
	# }}}
	# }}}
	# Accessor functions. {{{
	# Globals. {{{
	def get_globals(self): # {{{
		#log('getting globals')
		ret = {'num_temps': len(self.temps), 'num_gpios': len(self.gpios)}
		for key in ('name', 'user_interface', 'pin_names', 'uuid', 'queue_length', 'num_pins', 'led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'probe_dist', 'probe_offset', 'probe_safe_dist', 'bed_id', 'fan_id', 'spindle_id', 'unit_name', 'timeout', 'feedrate', 'targetx', 'targety', 'targetangle', 'zoffset', 'store_adc', 'temp_scale_min', 'temp_scale_max', 'probemap', 'paused', 'park_after_job', 'sleep_after_job', 'cool_after_job', 'spi_setup', 'max_deviation', 'max_v', 'max_a'):
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
		if 'probemap' in ka:
			self.probemap = ka.pop('probemap')
			self._check_probemap()
		for key in ('unit_name', 'user_interface', 'pin_names'):
			if key in ka:
				setattr(self, key, ka.pop(key))
		if 'spi_setup' in ka:
			self.spi_setup = self._unmangle_spi(ka.pop('spi_setup'))
			if self.spi_setup:
				self._spi_send(self.spi_setup)
		for key in ('led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'bed_id', 'fan_id', 'spindle_id', 'park_after_job', 'sleep_after_job', 'cool_after_job', 'timeout'):
			if key in ka:
				setattr(self, key, int(ka.pop(key)))
		for key in ('probe_dist', 'probe_offset', 'probe_safe_dist', 'feedrate', 'targetx', 'targety', 'targetangle', 'zoffset', 'temp_scale_min', 'temp_scale_max', 'max_deviation', 'max_v', 'max_a'):
			if key in ka:
				setattr(self, key, float(ka.pop(key)))
		self._write_globals(nt, ng, update = update)
		assert len(ka) == 0
	# }}}
	def user_set_globals(self, update = True, **ka): # {{{
		real_ka = {}
		for key in ('feedrate', 'targetx', 'targety', 'targetangle', 'zoffset'):
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
			return [self.spaces[space].get_current_pos(a) for a in range(len(self.spaces[space].axis))]
		else:
			return self.spaces[space].get_current_pos(axis)
	# }}}
	def user_set_axis_pos(self, space, axis, pos): # {{{
		if space >= len(self.spaces) or (axis is not None and axis >= len(self.spaces[space].axis)):
			log('request to set invalid axis position %d %d' % (space, axis))
			return False
		return self.spaces[space].set_current_pos(axis, pos)
	# }}}
	def get_space(self, space): # {{{
		ret = {'name': self.spaces[space].name, 'num_axes': len(self.spaces[space].axis), 'num_motors': len(self.spaces[space].motor)}
		if self.spaces[space].type == TYPE_CARTESIAN:
			pass
		elif self.spaces[space].type == TYPE_DELTA:
			delta = []
			for i in range(3):
				d = {}
				for key in ('axis_min', 'axis_max', 'rodlength', 'radius'):
					d[key] = self.spaces[space].delta[i][key]
				delta.append(d)
			delta.append(self.spaces[space].delta_angle)
			ret['delta'] = delta
		elif self.spaces[space].type == TYPE_POLAR:
			ret['polar_max_r'] = self.spaces[space].polar_max_r
		elif self.spaces[space].type == TYPE_EXTRUDER:
			ret['extruder'] = []
			for a in range(len(self.spaces[space].axis)):
				ret['extruder'].append({})
				for key in ('dx', 'dy', 'dz'):
					ret['extruder'][-1][key] = self.spaces[space].extruder[a][key]
		elif self.spaces[space].type == TYPE_FOLLOWER:
			ret['follower'] = []
			for a in range(len(self.spaces[space].axis)):
				ret['follower'].append({})
				for key in ('space', 'motor'):
					ret['follower'][-1][key] = self.spaces[space].follower[a][key]
		else:
			log('invalid type')
		return ret
	# }}}
	def get_axis(self, space, axis): # {{{
		ret = {'name': self.spaces[space].axis[axis]['name']}
		if space == 1:
			ret['multiplier'] = self.multipliers[axis]
		if space == 0:
			for key in ('park', 'park_order', 'min', 'max', 'home_pos2'):
				ret[key] = self.spaces[space].axis[axis][key]
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
		return ret
	# }}}
	def expert_set_space(self, space, readback = True, update = True, **ka): # {{{
		old_type = self.spaces[space].type
		if space == 0 and 'type' in ka:
			self.spaces[space].type = int(ka.pop('type'))
		current_pos = None if self.spaces[space].type != old_type else self.get_axis_pos(space)
		if self.spaces[space].type == TYPE_EXTRUDER:
			if 'extruder' in ka:
				e = ka.pop('extruder')
				for ei, ee in e.items():
					i = int(ei)
					for key in ('dx', 'dy', 'dz'):
						if key in ee:
							self.spaces[space].extruder[i][key] = ee.pop(key)
					assert len(ee) == 0
		if self.spaces[space].type == TYPE_FOLLOWER:
			if 'follower' in ka:
				f = ka.pop('follower')
				for fi, ff in f.items():
					fi = int(fi)
					for key in ('space', 'motor'):
						if key in ff:
							self.spaces[space].follower[fi][key] = int(ff.pop(key))
						log('follower set to {}'.format(self.spaces[space].follower))
				assert len(ff) == 0
		if self.spaces[space].type in (TYPE_CARTESIAN, TYPE_EXTRUDER, TYPE_FOLLOWER):
			if 'num_axes' in ka:
				num_axes = int(ka.pop('num_axes'))
			else:
				num_axes = len(self.spaces[space].axis)
			num_motors = num_axes
		elif self.spaces[space].type == TYPE_DELTA:
			num_axes = 3
			num_motors = 3
			if 'delta' in ka:
				d = ka.pop('delta')
				if isinstance(d, (list, tuple)):
					d = {i: item for i, item in enumerate(d)}
				for di, dd in d.items():
					i = int(di)
					assert 0 <= i < 3
					for key in ('axis_min', 'axis_max', 'rodlength', 'radius'):
						if key in dd:
							self.spaces[space].delta[i][key] = dd.pop(key)
					assert len(dd) == 0
			if 'delta_angle' in ka:
				self.spaces[space].delta_angle = ka.pop('delta_angle')
		elif self.spaces[space].type == TYPE_POLAR:
			num_axes = 3
			num_motors = 3
			if 'polar_max_r' in ka:
				self.spaces[space].polar_max_r = ka.pop('polar_max_r')
		self.spaces[space].write_info(num_axes)
		if readback:
			self.spaces[space].read()
			if update:
				self._space_update(space)
		if len(ka) != 0:
			log('invalid input ignored: %s' % repr(ka))
		if space == 0 and current_pos is not None and not all(math.isnan(x) for x in current_pos) and (self.paused or (self.home_phase is None and not self.gcode_file and self.gcode_map is None)):
			self.user_line(current_pos)[1](None)
		#else:
		#	log(repr(('not going to pos:', current_pos, self.paused, self.home_phase, self.gcode_file, self.gcode_map)))
	# }}}
	def expert_set_axis(self, spaceaxis, readback = True, update = True, **ka): # {{{
		space, axis = spaceaxis
		if 'name' in ka:
			self.spaces[space].axis[axis]['name'] = ka.pop('name')
		if space == 0:
			for key in ('park', 'park_order', 'min', 'max', 'home_pos2'):
				if key in ka:
					self.spaces[space].axis[axis][key] = ka.pop(key)
		if space == 1 and 'multiplier' in ka and axis < len(self.spaces[space].motor):
			assert(ka['multiplier'] > 0)
			self.multipliers[axis] = ka.pop('multiplier')
			self.expert_set_motor((space, axis), readback, update)
		self.spaces[space].write_axis(axis)
		if readback:
			self.spaces[space].read()
			if update:
				self._space_update(space)
		assert len(ka) == 0
	# }}}
	def expert_set_motor(self, spacemotor, readback = True, update = True, **ka): # {{{
		space, motor = spacemotor
		current_pos = self.get_axis_pos(space, motor)
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
		self.spaces[space].write_motor(motor)
		followers = False
		for m, mt in enumerate(self.spaces[2].motor):
			fs = self.spaces[2].follower[m]['space']
			fm = self.spaces[2].follower[m]['motor']
			if fs == space and fm == motor:
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
		if not math.isnan(current_pos) and (self.paused or (self.home_phase is None and not self.gcode_file and self.gcode_map is None)):
			self.user_line({motor: current_pos})[1](None)
	# }}}
	# }}}
	# Temp {{{
	def get_temp(self, temp): # {{{
		ret = {}
		for key in ('name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time'):
			ret[key] = getattr(self.temps[temp], key)
		return ret
	# }}}
	def expert_set_temp(self, temp, update = True, **ka): # {{{
		ret = {}
		for key in ('name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit_l', 'heater_limit_h', 'fan_limit_l', 'fan_limit_h', 'hold_time'):
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
		for key in ('name', 'pin', 'state', 'reset', 'duty', 'value'):
			ret[key] = getattr(self.gpios[gpio], key)
		return ret
	# }}}
	def expert_set_gpio(self, gpio, update = True, **ka): # {{{
		for key in ('name', 'pin', 'state', 'reset', 'duty'):
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
	#log('waiting; movewait = %d' % machine.movewait)
	found = select.select(fds, [], fds, None)
	if sys.stdin in found[0] or sys.stdin in found[2]:
		#log('command')
		machine._command_input()
	if fd in found[0] or fd in found[2]:
		#log('machine')
		machine._machine_input()
# }}}

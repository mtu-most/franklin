#! /usr/bin/python3
# vim: set foldmethod=marker fileencoding=utf8 :
# driver.py - high level driver implementation for Franklin {{{
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
import protocol
import mmap
import random
import errno
# }}}

config = fhs.init(packagename = 'franklin', config = { # {{{
	'cdriver': None,
	'port': None,
	'run-id': None,
	'allow-system': None
	})
# }}}

# Enable code trace. {{{
if False:
	def trace(frame, why, arg):
		if why == 'call':
			code = frame.f_code
			log('call: %s' % code.co_name)
	sys.settrace(trace)
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

# Call cdriver running on same machine.
class Driver: # {{{
	def __init__(self, port, run_id):
		#log(repr(config))
		self.driver = subprocess.Popen((config['cdriver'], port, run_id), stdin = subprocess.PIPE, stdout = subprocess.PIPE, close_fds = True)
		fcntl.fcntl(self.driver.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
		self.buffer = b''
	def available(self):
		return len(self.buffer) > 0
	def write(self, data):
		self.driver.stdin.write(data)
		self.driver.stdin.flush()
	def read(self, length):
		while True:
			if len(self.buffer) >= length:
				ret = self.buffer[:length]
				self.buffer = self.buffer[length:]
				return ret
			try:
				r = os.read(self.driver.stdout.fileno(), 4096)
			except IOError:
				r = self.buffer[:length]
				self.buffer = self.buffer[length:]
				return r
			except OSError as exc:
				if exc.errno == errno.EAGAIN:
					r = self.buffer[:length]
					self.buffer = self.buffer[length:]
					return r
				raise
			if r == b'':
				log('EOF!')
				self.close()
			self.buffer += r
	def close(self):
		log('Closing printer driver; exiting.')
		sys.exit(0)
	def fileno(self):
		return self.driver.stdout.fileno()
# }}}

# Reading and writing pins to and from ini files. {{{
def read_pin(pin):
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
		return int(pin) + extra
	except:
		log('incorrect pin %s' % pin)
		return 0

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

class Printer: # {{{
	# Internal stuff.  {{{
	def _read_data(self, data): # {{{
		cmd, s, m, e, f = struct.unpack('=BLLLd', data[:21])
		return cmd, s, m, f, e, data[21:]
	# }}}
	def _send(self, *data): # {{{
		#sys.stderr.write(repr(data) + '\n')
		sys.stdout.write(json.dumps(data) + '\n')
		sys.stdout.flush()
	# }}}
	def __init__(self, port, run_id, allow_system): # {{{
		self.initialized = False
		self.pin_names = None
		self.printer = Driver(port, run_id)
		self.allow_system = allow_system
		self.job_output = ''
		self.jobs_active = []
		self.jobs_angle = 0
		self.probemap = None
		self.job_current = 0
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
		self.printer_buffer = ''
		self.command_buffer = ''
		self.bed_id = 255
		self.fan_id = 255
		self.spindle_id = 255
		self.probe_dist = 1000
		self.probe_safe_dist = 10
		self.num_probes = 1
		self.unit_name = 'mm'
		self.park_after_print = True
		self.sleep_after_print = True
		self.cool_after_print = True
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
		self.run_id = run_id
		self.zoffset = 0.
		self.targetx = 0.
		self.targety = 0.
		self.store_adc = False
		self.temp_scale_min = 0
		self.temp_scale_max = 250
		self.multipliers = []
		self.current_extruder = 0
		# Get the printer state.
		self._read_globals(False)
		for i, s in enumerate(self.spaces):
			s.read(self._read('SPACE', i))
		for i, t in enumerate(self.temps):
			t.read(self._read('TEMP', i))
			# Disable heater.
			self.settemp(i, float('nan'), update = False)
			# Disable heater alarm.
			self.waittemp(i, None, None)
		for i, g in enumerate(self.gpios):
			g.read(self._read('GPIO', i))
		# The printer may still be doing things.  Pause it and send a move; this will discard the queue.
		self.pause(True, False, update = False)
		if port != '-' and not port.startswith('!'):
			self._send_packet(struct.pack('=B', protocol.command['GET_UUID']))
			cmd, s, m, f, e, uuid = self._get_reply()
			if (uuid[7] & 0xf0) != 0x40 or (uuid[9] & 0xc0) != 0x80:
				# Broken uuid; create a new one and set it.
				log('old uuid: ' + repr(uuid))
				uuid = [random.randrange(256) for i in range(16)]
				uuid[7] &= 0x0f
				uuid[7] |= 0x40
				uuid[9] &= 0x3f
				uuid[9] |= 0x80
				log('new uuid: ' + repr(uuid))
				self._send_packet(struct.pack('=B', protocol.command['SET_UUID']) + bytes(uuid))
			uuid = ''.join('%02x' % x for x in uuid[:16])
			self.uuid = uuid[:8] + '-' + uuid[8:12] + '-' + uuid[12:16] + '-' + uuid[16:20] + '-' + uuid[20:32]
			assert cmd == protocol.rcommand['UUID']
		else:
			self.uuid = 'local'
		try:
			with fhs.read_data(os.path.join(self.uuid, 'profile')) as pfile:
				self.profile = pfile.readline().strip()
			log('profile is %s' % self.profile)
		except:
			log("No default profile; using 'default'.")
			self.profile = 'default'
		profiles = self.list_profiles()
		if self.profile not in profiles and len(profiles) > 0:
			log('Default profile does not exist; using %s instead' % self.profile)
			self.profile = profiles[0]
		# Fill job queue.
		self.jobqueue = {}
		self.audioqueue = {}
		spool = fhs.read_spool(self.uuid, dir = True, opened = False)
		if spool is not None:
			gcode = os.path.join(spool, 'gcode')
			audio = os.path.join(spool, 'audio')
			if os.path.isdir(gcode):
				for filename in os.listdir(gcode):
					name, ext = os.path.splitext(filename)
					if ext != os.extsep + 'bin':
						log('skipping %s' % filename)
						continue
					try:
						log('opening %s' % filename)
						with open(os.path.join(gcode, filename), 'rb') as f:
							f.seek(-8 * 8, os.SEEK_END)
							self.jobqueue[name] = struct.unpack('=' + 'd' * 8, f.read())
					except:
						traceback.print_exc()
						log('failed to open gcode file %s' % os.path.join(gcode, filename))
			if os.path.isdir(audio):
				for filename in os.listdir(audio):
					name, ext = os.path.splitext(filename)
					if ext != os.extsep + 'bin':
						log('skipping %s' % filename)
						continue
					try:
						log('opening audio %s' % filename)
						self.audioqueue[name] = os.stat(os.path.join(audio, filename)).st_size
					except:
						traceback.print_exc()
						log('failed to stat audio file %s' % os.path.join(audio, filename))
		try:
			self.load(update = False)
			if self.spi_setup:
				self._spi_send(self.spi_setup)
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
		if notify:
			self._send(None, 'disconnect')
		waiting_commands = ''
		while True:
			s = select.select([sys.stdin], [], [sys.stdin])
			if len(s[2]) > 0:
				log('Error on standard input; exiting.')
				sys.exit(0)
			data = sys.stdin.read()
			if data == '':
				log('EOF on standard input; exiting.')
				sys.exit(0)
			self.command_buffer += data
			while '\n' in self.command_buffer:
				pos = self.command_buffer.index('\n')
				ln = self.command_buffer[:pos]
				self.command_buffer = self.command_buffer[pos + 1:]
				id, func, a, ka = json.loads(ln)
				if func == 'reconnect':
					log('reconnect %s' % a[1])
					self._send_packet(bytes((protocol.command['RECONNECT'],)) + a[1].encode('utf-8') + b'\x00')
					self.command_buffer = waiting_commands + self.command_buffer
					self._send(id, 'return', None)
					return
				elif func == 'disconnect':
					# Ignore disconnect requests while disconnected.
					pass
				elif func in ('export_settings', 'die'):
					role = a.pop(0) + '_'
					# TODO: Use role.
					ret = getattr(self, ('expert_' if func == 'die' else '') + func)(*a, **ka)
					if ret == (WAIT, WAIT):
						# Special case: request to die.
						self._send(id, 'return', None)
						sys.exit(0)
					else:
						self._send(id, 'return', ret)
				else:
					waiting_commands += ln + '\n'
	# }}}
	def _printer_read(self, *a, **ka): # {{{
		while True:
			try:
				return self.printer.read(*a, **ka)
			except:
				log('error reading')
				traceback.print_exc()
				sys.exit(0)
	# }}}
	def _printer_write(self, data): # {{{
		#log('writing %s' % ' '.join(['%02x' % x for x in data]))
		while True:
			try:
				self.printer.write(data)
				return
			except:
				log('error writing')
				traceback.print_exc()
				sys.exit(0)
	# }}}
	def _command_input(self): # {{{
		data = sys.stdin.read()
		if data == '':
			log('End of file detected on command input; exiting.')
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
				assert not any(func.startswith(x + '_') for x in ('benjamin', 'admin', 'expert', 'user'))
				role = a.pop(0) + '_'
				if hasattr(self, role + func):
					func = role + func
				elif role == 'benjamin_' and hasattr(self, 'admin_' + func):
					func = 'admin_' + func
				elif role in ('benjamin_', 'admin_') and hasattr(self, 'expert_' + func):
					func = 'expert_' + func
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
			call_queue.extend([(x[1], [done]) for x in self.movecb])
			self.movecb = []
			if self.flushing and self.queue_pos >= len(self.queue):
				#log('done flushing')
				self.flushing = 'done'
		#else:
		#	log('cb seen, but waiting for more')
	# }}}
	def _printer_input(self, reply = False): # {{{
		while True:
			if len(self.printer_buffer) == 0:
				r = self._printer_read(1)
				dprint('(1) read', r)
				if r == b'':
					return ('no data', None)
				if r == self.single['WAIT']:
					return ('wait', None)
				if r == self.single['OK']:
					return ('ok', None)
				# Regular packet.
				self.printer_buffer = r
			packet_len = self.printer_buffer[0]
			while True:
				r = self._printer_read(packet_len - len(self.printer_buffer))
				dprint('rest of packet read', r)
				if r == '':
					return (None, None)
				self.printer_buffer += r
				if len(self.printer_buffer) >= packet_len:
					break
				if not self.printer.available():
					#log('waiting for more data (%d/%d)' % (len(self.printer_buffer), packet_len))
					ret = select.select([self.printer], [], [self.printer], 1)
					if self.printer not in ret[0]:
						log('broken packet?')
						return (None, None)
			self.printer.write(self.single['OK'])
			cmd, s, m, f, e, data = self._read_data(self.printer_buffer[1:])
			self.printer_buffer = ''
			# Handle the asynchronous events.
			if cmd == protocol.rcommand['MOVECB']:
				#log('movecb %d/%d (%d in queue)' % (s, self.movewait, len(self.movecb)))
				self._trigger_movewaits(s)
				continue
			if cmd == protocol.rcommand['TEMPCB']:
				self.alarms.add(s)
				t = 0
				while t < len(self.tempcb):
					if self.tempcb[t][0] is None or self.tempcb[t][0] in self.alarms:
						call_queue.append((self.tempcb.pop(t)[1], []))
					else:
						t += 1
				continue
			elif cmd == protocol.rcommand['CONTINUE']:
				# Move continue.
				self.wait = False
				#log('resuming queue %d' % len(self.queue))
				call_queue.append((self._do_queue, []))
				if self.flushing is None:
					self.flushing = False
				continue
			elif cmd == protocol.rcommand['LIMIT']:
				if s < len(self.spaces) and m < len(self.spaces[s].motor):
					self.limits[s][m] = f
				#log('limit; %d waits' % e)
				self._trigger_movewaits(self.movewait, False)
				continue
			elif cmd == protocol.rcommand['TIMEOUT']:
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
				continue
			elif cmd == protocol.rcommand['PINCHANGE']:
				self.gpios[s].value = m
				call_queue.append((self._gpio_update, (s,)))
				if s in self.gpio_waits:
					for id in self.gpio_waits[s]:
						self._send(id, 'return', None)
					del self.gpio_waits[s]
				continue
			elif cmd == protocol.rcommand['HOMED']:
				call_queue.append((self._do_home, [True]))
				continue
			elif cmd == protocol.rcommand['DISCONNECT']:
				self._close()
				# _close returns after reconnect.
				continue
			elif cmd == protocol.rcommand['UPDATE_TEMP']:
				if s < len(self.temps):
					self.temps[s].value = f - C0
					self._temp_update(s)
				else:
					log('Ignoring updated invalid temp %d' % s)
				continue
			elif cmd == protocol.rcommand['UPDATE_PIN']:
				self.gpios[s].state = m
				call_queue.append((self._gpio_update, (s,)))
				continue
			elif cmd == protocol.rcommand['CONFIRM']:
				if s and self.probemap is not None:
					self.probe_pending = True
				call_queue.append((self.request_confirmation(data.decode('utf-8', 'replace') or 'Continue?')[1], (False,)))
				continue
			elif cmd == protocol.rcommand['PARKWAIT']:
				def cb():
					self._send_packet(bytes((protocol.command['RESUME'],)))
				call_queue.append((self.park(cb = cb, abort = False)[1], (None,)))
				continue
			elif cmd == protocol.rcommand['FILE_DONE']:
				call_queue.append((self._print_done, (True, 'completed')))
				continue
			elif cmd == protocol.rcommand['PINNAME']:
				self.pin_names[s] = data
				continue
			if reply:
				return ('packet', (cmd, s, m, f, e, data))
			log('unexpected packet %02x' % cmd)
			raise AssertionError('Received unexpected reply packet')
	# }}}
	def _send_packet(self, data, move = False): # {{{
		data = bytes((len(data) + 1,)) + data
		dprint('(1) writing', data);
		self._printer_write(data)
		if not move:
			return True
		start_time = time.time()
		while True:
			if not self.printer.available():
				ret = select.select([self.printer], [], [self.printer], 1)
				if self.printer not in ret[0] and self.printer not in ret[2]:
					# No response; keep waiting.
					log('no response yet: %s' % repr(ret))
					assert time.time() - start_time < 10
					continue
			ret = self._printer_input()
			if ret[0] == 'wait':
				#log('wait')
				self.wait = True
				return True
			elif ret[0] == 'ok':
				return True
			#log('no response yet')
	# }}}
	def _get_reply(self, cb = False): # {{{
		#traceback.print_stack()
		while True:
			if not self.printer.available():
				ret = select.select([self.printer], [], [self.printer], 3)
				if len(ret[0]) == 0 and len(ret[2]) == 0:
					log('no reply received')
					#traceback.print_stack()
					continue
			ret = self._printer_input(reply = True)
			if ret[0] == 'packet' or (cb and ret[0] == 'no data'):
				return ret[1]
			#log('no response yet waiting for reply')
	# }}}
	def _read(self, cmd, channel, sub = None): # {{{
		if cmd == 'SPACE':
			info = self._read('SPACE_INFO', channel)
			self.spaces[channel].type = struct.unpack('=B', info[:1])[0]
			info = info[1:]
			if self.spaces[channel].type == TYPE_CARTESIAN:
				num_axes = struct.unpack('=B', info)[0]
				num_motors = num_axes
			elif self.spaces[channel].type == TYPE_DELTA:
				self.spaces[channel].delta = [{}, {}, {}]
				for a in range(3):
					self.spaces[channel].delta[a]['axis_min'], self.spaces[channel].delta[a]['axis_max'], self.spaces[channel].delta[a]['rodlength'], self.spaces[channel].delta[a]['radius'] = struct.unpack('=dddd', info[32 * a:32 * (a + 1)])
				self.spaces[channel].delta_angle = struct.unpack('=d', info[32 * 3:])[0]
				num_axes = 3
				num_motors = 3
			elif self.spaces[channel].type == TYPE_POLAR:
				self.spaces[channel].polar_max_r = struct.unpack('=d', info)[0]
				num_axes = 3
				num_motors = 3
			elif self.spaces[channel].type == TYPE_EXTRUDER:
				num_axes = struct.unpack('=B', info[:1])[0]
				num_motors = num_axes
				self.spaces[channel].extruder = []
				for a in range(num_axes):
					dx, dy, dz = struct.unpack('=ddd', info[1 + 24 * a:1 + 24 * (a + 1)])
					self.spaces[channel].extruder.append({'dx': dx, 'dy': dy, 'dz': dz})
			elif self.spaces[channel].type == TYPE_FOLLOWER:
				num_axes = struct.unpack('=B', info[:1])[0]
				num_motors = num_axes
				self.spaces[channel].follower = []
				for a in range(num_axes):
					space, motor = struct.unpack('=BB', info[1 + 2 * a:1 + 2 * (a + 1)])
					self.spaces[channel].follower.append({'space': space, 'motor': motor})
			else:
				log('invalid type %s' % repr(self.spaces[channel].type))
				raise AssertionError('invalid space type')
			return ([self._read('SPACE_AXIS', channel, axis) for axis in range(num_axes)], [self._read('SPACE_MOTOR', channel, motor) for motor in range(num_motors)])
		if cmd == 'GLOBALS':
			packet = struct.pack('=B', protocol.command['READ_' + cmd])
		elif sub is not None and cmd.startswith('SPACE'):
			packet = struct.pack('=BBB', protocol.command['READ_' + cmd], channel, sub)
		else:
			packet = struct.pack('=BB', protocol.command['READ_' + cmd], channel)
		self._send_packet(packet)
		cmd, s, m, f, e, data = self._get_reply()
		assert cmd == protocol.rcommand['DATA']
		return data
	# }}}
	def _read_globals(self, update = True): # {{{
		data = self._read('GLOBALS', None)
		if data is None:
			return False
		self.queue_length, self.num_pins, num_temps, num_gpios = struct.unpack('=BBBB', data[:4])
		if self.pin_names is None:
			self.pin_names = [''] * self.num_pins
		self.led_pin, self.stop_pin, self.probe_pin, self.spiss_pin, self.timeout, self.bed_id, self.fan_id, self.spindle_id, self.feedrate, self.max_deviation, self.max_v, self.current_extruder, self.targetx, self.targety, self.zoffset, self.store_adc = struct.unpack('=HHHHHhhhdddBddd?', data[4:])
		while len(self.temps) < num_temps:
			self.temps.append(self.Temp(len(self.temps)))
			if update:
				data = self._read('TEMP', len(self.temps) - 1)
				self.temps[-1].read(data)
		self.temps = self.temps[:num_temps]
		while len(self.gpios) < num_gpios:
			self.gpios.append(self.Gpio(len(self.gpios)))
			if update:
				data = self._read('GPIO', len(self.gpios) - 1)
				self.gpios[-1].read(data)
		self.gpios = self.gpios[:num_gpios]
		return True
	# }}}
	def _write_globals(self, nt, ng, update = True): # {{{
		if nt is None:
			nt = len(self.temps)
		if ng is None:
			ng = len(self.gpios)
		dt = nt - len(self.temps)
		dg = ng - len(self.gpios)
		data = struct.pack('=BBHHHHHhhhdddBddd?', nt, ng, self.led_pin, self.stop_pin, self.probe_pin, self.spiss_pin, int(self.timeout), self.bed_id, self.fan_id, self.spindle_id, self.feedrate, self.max_deviation, self.max_v, self.current_extruder, self.targetx, self.targety, self.zoffset, self.store_adc)
		self._send_packet(struct.pack('=B', protocol.command['WRITE_GLOBALS']) + data)
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
		self._broadcast(target, 'globals_update', [self.profile, len(self.temps), len(self.gpios), self.led_pin, self.stop_pin, self.probe_pin, self.spiss_pin, self.probe_dist, self.probe_safe_dist, self.bed_id, self.fan_id, self.spindle_id, self.unit_name, self.timeout, self.feedrate, self.max_deviation, self.max_v, self.targetx, self.targety, self.zoffset, self.store_adc, self.park_after_print, self.sleep_after_print, self.cool_after_print, self._mangle_spi(), self.temp_scale_min, self.temp_scale_max, not self.paused and (None if self.gcode_map is None and not self.gcode_file else True)])
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
	def _use_probemap(self, x, y, z): # {{{
		'''Return corrected z according to self.probemap.'''
		# Map = [[x, y, w, h], [nx, ny], [[...], [...], ...]]
		if self.probemap is None or any(math.isnan(t) for t in (x, y, z)):
			return z
		p = self.probemap
		x -= p[0][0]
		y -= p[0][1]
		x /= p[0][2] / p[1][0]
		y /= p[0][3] / p[1][1]
		if x < 0:
			x = 0
		ix = int(x)
		if x >= p[1][0]:
			x = p[1][0]
			ix = int(x) - 1
		if y < 0:
			y = 0
		iy = int(y)
		if y >= p[1][1]:
			y = p[1][1]
			iy = int(y) - 1
		fx = x - ix
		fy = y - iy
		l = p[2][iy][ix] * (1 - fy) + p[2][iy + 1][ix] * fy
		r = p[2][iy][ix + 1] * (1 - fy) + p[2][iy + 1][ix + 1] * fy
		return z + l * (1 - fx) + r * fx
	# }}}
	def _gcode_close(self): # {{{
		self.gcode_strings = []
		self.gcode_map.close()
		os.close(self.gcode_fd)
		self.gcode_map = None
		self.gcode_fd = -1
	# }}}
	def _print_done(self, complete, reason): # {{{
		self._send_packet(struct.pack('=BBddBB', protocol.command['RUN_FILE'], 0, 0, 0, 0xff, 0))
		if self.gcode_map is not None:
			log(reason)
			self._gcode_close()
		self.gcode_file = False
		#traceback.print_stack()
		if self.queue_info is None and self.gcode_id is not None:
			log('Print done (%d): %s' % (complete, reason))
			self._send(self.gcode_id, 'return', (complete, reason))
			self.gcode_id = None
		if self.audio_id is not None:
			log('Audio done (%d): %s' % (complete, reason))
			self._send(self.audio_id, 'return', (complete, reason))
		self.audio_id = None
		#log('job %s' % repr(self.jobs_active))
		if self.queue_info is None and len(self.jobs_active) > 0:
			if complete:
				if self.job_current >= len(self.jobs_active) - 1:
					#log('job queue done')
					if self.job_id is not None:
						self._send(self.job_id, 'return', (True, reason))
					self.job_id = None
					self.jobs_active = []
					self._finish_done()
				else:
					#log('job part done; moving on')
					self._next_job()
			else:
				#log('job aborted')
				if self.job_id is not None:
					self._send(self.job_id, 'return', (False, reason))
				self.job_id = None
				self.jobs_active = []
		while self.queue_pos < len(self.queue):
			axes, e, f0, f1, v0, v1, which, single, rel = self.queue[self.queue_pos]
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
		if self.cool_after_print:
			for t in range(len(self.temps)):
				self.settemp(t, float('nan'))
		def maybe_sleep():
			if self.sleep_after_print:
				self.sleep()
		if self.park_after_print:
			self.park(cb = maybe_sleep)[1](None)
		else:
			maybe_sleep()
	# }}}
	def _unpause(self): # {{{
		if self.gcode_file:
			self._send_packet(bytes((protocol.command['RESUME'],)))	# Just in case.
		if self.queue_info is None:
			return
		log('doing resume to %d/%d' % (self.queue_info[0], len(self.queue_info[2])))
		self.queue = self.queue_info[2]
		self.queue_pos = self.queue_info[0]
		self.movecb = self.queue_info[3]
		self.flushing = self.queue_info[4]
		self.resuming = False
		self.queue_info = None
		self.paused = False
		self._globals_update()
	# }}}
	def _queue_add(self, f, name): # {{{
		name = os.path.split(name)[1]
		origname = name
		i = 0
		while name == '' or name in self.jobqueue:
			name = '%s-%d' % (origname, i)
			i += 1
		bbox, errors = self._gcode_parse(f, name)
		for e in errors:
			log(e)
		if bbox is None:
			return errors
		self.jobqueue[os.path.splitext(name)[0]] = bbox
		self._broadcast(None, 'queue', [(q, self.jobqueue[q]) for q in self.jobqueue])
		return errors
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
		with fhs.write_spool(os.path.join(self.uuid, 'audio', name + os.path.extsep + 'bin')) as dst:
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
				log('unpaused, %d %d' % (self.queue_pos, len(self.queue)))
				if self.queue_pos >= len(self.queue):
					break
			axes, f0, f1, v0, v1, probe, single, rel = self.queue[self.queue_pos]
			#log('queueing %s' % repr((axes, f0, f1, probe)))
			self.queue_pos += 1
			# Turn sequences into a dict.
			if isinstance(axes, (list, tuple)):
				adict = {}
				for s, data in enumerate(axes):
					adict[s] = data
				axes = adict
			# Make sure the keys are ints.
			adict = {}
			#log('axes: %s' % repr(axes))
			for k in axes:
				adict[int(k)] = axes[k]
			axes = adict
			a = {}
			a0 = 0
			for i, sp in enumerate(self.spaces):
				# Only handle spaces that are specified.
				if i not in axes or axes[i] is None:
					a0 += len(sp.axis)
					continue
				# Handle sequences.
				if isinstance(axes[i], (list, tuple)):
					for ij, axis in enumerate(axes[i]):
						if ij >= len(sp.axis):
							log('ignoring nonexistent axis %d %d' % (i, ij))
							continue
						if axis is not None and not math.isnan(axis):
							if i == 1 and ij != self.current_extruder:
								#log('setting current extruder to %d' % ij)
								self.current_extruder = ij
								self._write_globals()
							if rel:
								axis += sp.get_current_pos(ij)
							# Limit values for axis.
							if axis > sp.axis[ij]['max'] - (0 if i != 0 or ij != 2 else self.zoffset):
								log('limiting %d %d to %f because it exceeds max' % (i, ij, axis))
								axis = sp.axis[ij]['max'] - (0 if i != 0 or ij != 2 else self.zoffset)
							if axis < sp.axis[ij]['min'] - (0 if i != 0 or ij != 2 else self.zoffset):
								log('limiting %d %d to %f because it exceeds min' % (i, ij, axis))
								axis = sp.axis[ij]['min'] - (0 if i != 0 or ij != 2 else self.zoffset)
							a[a0 + ij] = axis
				else:
					for j, axis in tuple(axes[i].items()):
						ij = int(j)
						if ij >= len(sp.axis):
							log('ignoring nonexistent axis %d %d' % (i, ij))
							continue
						if axis is not None and not math.isnan(axis):
							if i == 1 and ij != self.current_extruder:
								log('Setting current extruder to %d' % ij)
								self.current_extruder = ij
								self._write_globals(len(self.temps), len(self.gpios))
							if rel:
								axis += sp.get_current_pos(ij)
							# Limit values for axis.
							if axis > sp.axis[ij]['max'] - (0 if i != 0 or ij != 2 else self.zoffset):
								log('limiting %d %d to %f because it exceeds max' % (i, ij, axis))
								axis = sp.axis[ij]['max'] - (0 if i != 0 or ij != 2 else self.zoffset)
							if axis < sp.axis[ij]['min'] - (0 if i != 0 or ij != 2 else self.zoffset):
								log('limiting %d %d to %f because it exceeds min' % (i, ij, axis))
								axis = sp.axis[ij]['min'] - (0 if i != 0 or ij != 2 else self.zoffset)
							a[a0 + ij] = axis
				a0 += len(sp.axis)
			targets = [0] * (((2 + a0 - 1) >> 3) + 1)
			axes = a
			args = b''
			# Set defaults for feedrates.
			if v0 is not None:
				assert f0 is None
				f0 = -v0
			elif f0 is None:
				f0 = float('inf')
			if v1 is not None:
				assert f1 is None
				f1 = -v1
			elif f1 is None:
				f1 = f0
			assert f0 != 0 or f1 != 0
			# If feedrates are equal to firmware defaults, don't send them.
			if f0 != float('inf'):
				targets[0] |= 1 << 0
				args += struct.pack('=d', f0)
			if f1 != f0:
				targets[0] |= 1 << 1
				args += struct.pack('=d', f1)
			a = list(axes.keys())
			a.sort()
			#log('f0: %f f1: %f' %(f0, f1))
			for axis in a:
				if math.isnan(axes[axis]):
					continue
				targets[(axis + 2) >> 3] |= 1 << ((axis + 2) & 0x7)
				args += struct.pack('=d', axes[axis])
				#log('axis %d: %f' %(axis, axes[axis]))
			if probe:
				p = bytes((protocol.command['PROBE'],))
			elif single:
				p = bytes((protocol.command['SINGLE'],))
			else:
				p = bytes((protocol.command['LINE'],))
			#log('movewait +1 -> %d' % self.movewait)
			self.movewait += 1
			#log('queueing %s' % repr((axes, f0, f1, self.flushing)))
			self._send_packet(p + bytes(targets) + args, move = True)
			if self.flushing is None:
				self.flushing = False
		#log('queue done %s' % repr((self.queue_pos, len(self.queue), self.resuming, self.wait)))
	# }}}
	def _do_home(self, done = None): # {{{
		#log('do_home: %s %s' % (self.home_phase, done))
		# 0: Prepare for next order.
		# 1: Move to limits. (enter from loop after 2).
		# 2: Finish moving to limits; loop home_order; move slowly away from switch.
		# 3: Set current position; move delta and followers.
		# 4: Move within limits.
		# 5: Return.
		#log('home %s %s' % (self.home_phase, repr(self.home_target)))
		#traceback.print_stack()
		home_v = 50 / self.feedrate
		def mktarget():
			ret = {}
			for s, m in self.home_target:
				if s not in ret:
					ret[s] = {}
				ret[s][m] = self.home_target[(s, m)]
			return ret
		if self.home_phase is None:
			log('_do_home ignored because home_phase is None')
			return
		if self.home_phase == 0:
			if done is not None:
				# Continuing call received after homing was aborted; ignore.
				return
			# Initial call; start homing.
			self.home_phase = 1
			# If it is currently moving, doing the things below without pausing causes stall responses.
			self.pause(True, False)
			self.sleep(False)
			self.home_limits = [(a['min'], a['max']) for a in self.spaces[0].axis]
			for a, ax in enumerate(self.spaces[0].axis):
				self.expert_set_axis((0, a), min = float('-inf'), max = float('inf'))
			self.home_orig_type = self.spaces[0].type
			self.expert_set_space(0, type = TYPE_CARTESIAN)
			n = set()
			for s in self.spaces:
				for m in s.motor:
					if self._pin_valid(m['limit_min_pin']) or self._pin_valid(m['limit_max_pin']):
						n.add(m['home_order'])
			if len(n) == 0:
				self.home_phase = 4
			else:
				self.home_order = min(n)
			# Fall through.
		if self.home_phase == 1:
			# Move to limit.
			self.home_phase = 2
			self.home_motors = []
			for s, sp in enumerate(self.spaces):
				for i, m in enumerate(sp.motor):
					if (self._pin_valid(m['limit_min_pin']) or self._pin_valid(m['limit_max_pin'])) and m['home_order'] == self.home_order:
						self.home_motors.append((s, i, sp.axis[i], m))
				self.limits[s].clear()
			self.home_target = {}
			dist = 1000 #TODO: use better value.
			for s, i, a, m in self.home_motors:
				self.spaces[s].set_current_pos(i, 0)
				if self._pin_valid(m['limit_max_pin']):
					self.home_target[(s, i)] = dist - (0 if s != 0 or i != 2 else self.zoffset)
				else:
					self.home_target[(s, i)] = -dist - (0 if s != 0 or i != 2 else self.zoffset)
			if len(self.home_target) > 0:
				self.home_cb[0] = [(s, k) for s, k in self.home_target.keys()]
				if self.home_cb not in self.movecb:
					self.movecb.append(self.home_cb)
				#log("N t %s" % (self.home_target))
				self.line(mktarget(), f0 = home_v / dist, force = True, single = True)
				return
			# Fall through.
		if self.home_phase == 2:
			# Continue moving to find limit switch.
			found_limits = False
			for s, sp in enumerate(self.spaces):
				for a in self.limits[s].keys():
					if (s, a) in self.home_target:
						#log('found limit %d %d' % (s, a))
						self.home_target.pop((s, a))
						found_limits = True
						sp.set_current_pos(a, sp.get_current_pos(a))
			# Repeat until move is done, or all limits are hit.
			if (not done or found_limits) and len(self.home_target) > 0:
				self.home_cb[0] = list(self.home_target.keys())
				if self.home_cb not in self.movecb:
					self.movecb.append(self.home_cb)
				#log("0 t %s" % (self.home_target))
				k = tuple(self.home_target.keys())[0]
				dist = abs(self.home_target[k] - self.spaces[k[0]].get_current_pos(k[1]))
				if dist > 0:
					self.line(mktarget(), f0 = home_v / dist, force = True, single = True)
					return
				# Fall through.
			if len(self.home_target) > 0:
				log('Warning: not all limits were found during homing')
			n = set()
			for s in self.spaces:
				for m in s.motor:
					if (self._pin_valid(m['limit_min_pin']) or self._pin_valid(m['limit_max_pin'])) and m['home_order'] > self.home_order:
						n.add(m['home_order'])
			if len(n) > 0:
				self.home_phase = 1
				self.home_order = min(n)
				return self._do_home()
			# Move away slowly.
			data = b''
			num = 0
			for s, spc in enumerate(self.spaces):
				for m in spc.motor:
					if self._pin_valid(m['limit_max_pin']):
						data += b'\xff'
						num += 1
					elif self._pin_valid(m['limit_min_pin']):
						data += b'\x01'
						num += 1
					else:
						data += b'\x00'
			self.home_phase = 3
			if num > 0:
				dprint('homing', data)
				self._send_packet(bytes((protocol.command['HOME'],)) + data)
				return
			# Fall through.
		if self.home_phase == 3:
			# set current position.
			for s, sp in enumerate(self.spaces):
				for i, m in enumerate(sp.motor):
					if i in self.limits[s]:
						if not math.isnan(m['home_pos']):
							#log('set %d %d %f' % (s, i, m['home_pos']))
							sp.set_current_pos(i, m['home_pos'])
					else:
						if (self._pin_valid(m['limit_min_pin']) or self._pin_valid(m['limit_max_pin'])) and not math.isnan(m['home_pos']):
							#log('defset %d %d %f' % (s, i, m['home_pos']))
							sp.set_current_pos(i, m['home_pos'])
						else:
							#log('zeroset %d %d' % (s, i))
							sp.set_current_pos(i, 0)
			# Pre-insert delta axes as followers to align.
			groups = ([], [], [])	# min limits; max limits; just move.
			if self.home_orig_type == TYPE_DELTA:
				groups[1].append([])
				for i, m in enumerate(self.spaces[0].motor):
					groups[1][-1].append((0, i))
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
						groups[1].append([(2, i), (fs, fm)])
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
			self.home_phase = 4
			if len(self.home_target) > 0:
				self.home_cb[0] = False
				if self.home_cb not in self.movecb:
					self.movecb.append(self.home_cb)
				self.line(mktarget(), force = True, single = True)
				return
			# Fall through.
		if self.home_phase == 4:
			for s, sp in enumerate(self.spaces):
				for i, m in enumerate(sp.motor):
					if not self._pin_valid(m['limit_min_pin']) and not self._pin_valid(m['limit_max_pin']):
						if self.home_orig_type != TYPE_DELTA or s != 0:
							#log('set %d %d to 0' % (s, i))
							sp.set_current_pos(i, 0 if s == 1 else m['home_pos'])
			self.expert_set_space(0, type = self.home_orig_type)
			for a, ax in enumerate(self.spaces[0].axis):
				self.expert_set_axis((0, a), min = self.home_limits[a][0], max = self.home_limits[a][1])
			target = {}
			for s, sp in enumerate(self.spaces[:2]):
				for i, a in enumerate(sp.axis):
					current = sp.get_current_pos(i)
					offset = (0 if s != 0 or i != 2 else self.zoffset)
					if current > a['max'] - offset:
						if s not in target:
							target[s] = {}
						target[s][i] = a['max'] - offset
					elif current < a['min'] - offset:
						if s not in target:
							target[s] = {}
						target[s][i] = a['min'] - offset
			self.home_phase = 5
			if len(target) > 0:
				self.home_cb[0] = False
				if self.home_cb not in self.movecb:
					self.movecb.append(self.home_cb)
				self.line(target, force = True)
				return
			# Fall through.
		if self.home_phase == 5:
			self.home_phase = None
			self.position_valid = True
			if self.home_id is not None:
				self._send(self.home_id, 'return', None)
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
		self._send_packet(struct.pack('=Bddd', protocol.command['ADJUSTPROBE'], pos[0], pos[1], pos[2]))
		self.probe_cb[1] = lambda good: self.request_confirmation("Continue?")[1](False) if good is not None else None
		self.movecb.append(self.probe_cb)
		self.line([{2: self.probe_safe_dist}], relative = True)
	# }}}
	def _one_probe(self): # {{{
		self.probe_cb[1] = self._handle_one_probe
		self.movecb.append(self.probe_cb)
		z = self.get_axis_pos(0, 2)
		z_low = self.spaces[0].axis[2]['min']
		self.line([{2: z_low}], f0 = float(self.probe_speed) / (z - z_low) if z > z_low else float('inf'), probe = True)
	# }}}
	def _do_probe(self, id, x, y, z, angle, phase = 0, good = True): # {{{
		#log('probe %d' % phase)
		# Map = [[x0, y0, x1, y1], [nx, ny], [[...], [...], ...]]
		if good is None:
			# This means the probe has been aborted.
			log('abort probe')
			self.probing = False
			if id is not None:
				self._send(id, 'error', 'aborted')
			#self._print_done(False, 'Probe aborted')
			return
		self.probing = True
		if not self.position_valid:
			self.home(cb = lambda: self._do_probe(id, x, y, z, angle, phase, True), abort = False)[1](None)
			return
		p = self.probemap
		if phase == 0:
			if y > p[1][1]:
				# Done.
				self.probing = False
				if id is not None:
					self._send(id, 'return', self.probemap)
				else:
					for y, c in enumerate(p[2]):
						for x, o in enumerate(c):
							log('map %f %f %f' % (p[0][0] + p[0][2] * x / p[1][0], p[0][1] + p[0][3] * y / p[1][1], o))
					#log('result: %s' % repr(self.probemap))
					if len(self.jobs_active) == 1:
						def cb():
							self.request_confirmation("Probing done; prepare for job.")[1](False)
							self._next_job()
						self.park(cb = cb, abort = False)[1](None)
					else:
						self._next_job()
				return
			# Goto x,y
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, z, angle, 1, good)
			self.movecb.append(self.probe_cb)
			px = p[0][0] + p[0][2] * x / p[1][0]
			py = p[0][1] + p[0][3] * y / p[1][1]
			self.line([[self.targetx + px * self.gcode_angle[1] - py * self.gcode_angle[0], self.targety + py * self.gcode_angle[1] + px * self.gcode_angle[0]]])
		elif phase == 1:
			# Probe
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, z, angle, 2, good)
			if self._pin_valid(self.probe_pin):
				self.movecb.append(self.probe_cb)
				z_low = self.spaces[0].axis[2]['min']
				self.line([{2: z_low}], f0 = float(self.probe_speed) / (z - z_low) if z > z_low else float('inf'), probe = True)
			else:
				#log('confirm probe')
				self.request_confirmation('Please move the tool to the surface')[1](False)
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
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, z, angle, 0, good)
			self.movecb.append(self.probe_cb)
			# Retract
			self.line([{2: z}])
	# }}}
	def _next_job(self): # {{{
		# Set all extruders to 0.
		for i, e in enumerate(self.spaces[1].axis):
			self.set_axis_pos(1, i, 0)
		self.job_current += 1
		if self.job_current >= len(self.jobs_active):
			self._print_done(True, 'Queue finished')
			return
		def cb():
			if len(self.jobs_active) > 1:
				self.request_confirmation("Prepare for job '%s'." % self.jobs_active[self.job_current])[1](False)
			self._gcode_run(self.jobs_active[self.job_current], self.jobs_angle, abort = False)
		if not self.position_valid or len(self.jobs_active) > 1:
			self.park(cb = cb, abort = False)[1](None)
		else:
			cb()
		self.gcode_id = None
	# }}}
	def _gcode_run(self, src, angle = 0, abort = True): # {{{
		if self.parking:
			return
		angle = math.radians(angle)
		self.gcode_angle = math.sin(angle), math.cos(angle)
		if self.bed_id < len(self.temps):
			self.btemp = self.temps[self.bed_id].value
		else:
			self.btemp = float('nan')
		if abort:
			self._unpause()
			self._print_done(False, 'aborted by starting new print')
		self.queue_info = None
		# Disable all alarms.
		for i in range(len(self.temps)):
			self.waittemp(i, None, None)
		self.paused = False
		self._globals_update()
		self.sleep(False)
		if len(self.spaces) > 1:
			for e in range(len(self.spaces[1].axis)):
				self.set_axis_pos(1, e, 0)
		filename = fhs.read_spool(os.path.join(self.uuid, 'gcode', src + os.extsep + 'bin'), text = False, opened = False)
		self.total_time = self.jobqueue[src][-2:]
		if self.probemap is not None:
			self.gcode_file = True
			self._globals_update()
			encoded_filename = filename.encode('utf8')
			with fhs.write_spool(os.path.join(self.uuid, 'probe', src + os.extsep + 'bin'), text = False) as probemap_file:
				encoded_probemap_filename = probemap_file.name.encode('utf8')
				# Map = [[x, y, w, h], [nx, ny], [[...], [...], ...]]
				sina, cosa = self.gcode_angle
				x, y, w, h = self.probemap[0]
				# Transform origin because only rotation is done by cdriver.
				x, y = cosa * x - sina * y, cosa * y + sina * x
				x += self.targetx
				y += self.targety
				x, y = cosa * x + sina * y, cosa * y - sina * x
				probemap_file.write(struct.pack('@ddddddLL', x, y, w, h, sina, cosa, *self.probemap[1]))
				for y in range(self.probemap[1][1] + 1):
					for x in range(self.probemap[1][0] + 1):
						probemap_file.write(struct.pack('@d', self.probemap[2][y][x]))
			self._send_packet(struct.pack('=BBddBB', protocol.command['RUN_FILE'], 1 if self.confirmer is None else 0, self.gcode_angle[0], self.gcode_angle[1], 0xff, len(encoded_probemap_filename)) + encoded_filename + encoded_probemap_filename)
		else:
			# Let cdriver do the work.
			self.gcode_file = True
			self._globals_update()
			self._send_packet(struct.pack('=BBddBB', protocol.command['RUN_FILE'], 1 if self.confirmer is None else 0, self.gcode_angle[0], self.gcode_angle[1], 0xff, 0) + filename.encode('utf8'))
	# }}}
	def _gcode_parse(self, src, name): # {{{
		assert len(self.spaces) > 0
		self._broadcast(None, 'blocked', 'parsing g-code')
		errors = []
		mode = None
		message = None
		bbox = [None] * 6
		bbox_last = [None] * 6
		strings = ['']
		unit = 1.
		arc_normal = (0, 0, 1)
		rel = False
		erel = None
		pos = [[float('nan') for a in range(6)], [0., 0.], float('inf')]
		time_dist = [0., 0.]
		pending = []
		tool_changed = False
		def add_timedist(type, nums):
			if type == protocol.parsed['LINE']:
				if nums[-2] == float('inf'):
					extra = sum((nums[2 * i + 1] - nums[2 * i + 2]) ** 2 for i in range(3)) ** .5
					if not math.isnan(extra):
						time_dist[1] += extra
				else:
					extra = 2 / (nums[-2] + nums[-1])
					if not math.isnan(extra):
						time_dist[0] += extra
			elif type == protocol.parsed['ARC']:
				pass	# TODO: add time+dist.
			return nums + time_dist
		with fhs.write_spool(os.path.join(self.uuid, 'gcode', os.path.splitext(name)[0] + os.path.extsep + 'bin'), text = False) as dst:
			def add_record(type, nums = None, force = False):
				if nums is None:
					nums = []
				if isinstance(nums, dict):
					nums = [nums['T'], nums['X'], nums['Y'], nums['Z'], nums['E'], nums['f'], nums['F']]
				nums += [0] * (7 - len(nums))
				if not force and type == protocol.parsed['LINE']:
					for i in range(3):
						value = nums[i + 1]
						if math.isnan(value):
							continue
						if bbox[2 * i] is None or value < bbox[2 * i]:
							#log('new min bbox %f: %f from %f' % (i, value / 25.4, float('nan' if bbox[2 * i] is None else bbox[2 * i] / 25.4)))
							bbox[2 * i] = value
						if bbox[2 * i + 1] is None or value > bbox[2 * i + 1]:
							#log('new max bbox %f: %f from %f' % (i, value / 25.4, float('nan' if bbox[2 * i + 1] is None else bbox[2 * i + 1] / 25.4)))
							bbox[2 * i + 1] = value
					# Analyze this move in combination with pending moves.
					if len(pending) == 0:
						pending.append([0, pos[0][0], pos[0][1], pos[0][2], pos[1][nums[0]], pos[2], pos[2]])
					if len(pending) < 2:
						pending.append(nums)
					if len(pending) == 3:
						# If the points are not on a circle with equal angles, or the angle is too large, push pending[1] through to output.
						# Otherwise, record settings.
						def center(a, b, c):
							tb = ((b[0] - c[0]) * (c[0] - a[0]) / 2 - (b[1] - c[1]) * (c[1] - a[1]) / 2) / ((b[0] - a[0]) * (c[1] - a[1]) - (b[0] - c[0]) * (c[0] - a[0]))
							ctr = (b[0] - a[0] / 2 + tb * (b[1] - a[1]), b[1] - a[1] / 2 + tb * (b[0] - a[0])) 
							r = ((a[0] - ctr[0]) ** 2 + (a[1] - ctr[1]) ** 2) ** .5
							diff = 0
							for o in b, c:
								mid = (o[0] - a[0] / 2, o[1] - a[1] / 2)
								d = ((mid[0] - ctr[0]) ** 2 + (mid[1] - ctr[1]) ** 2) ** .5
								if abs(d - r) > diff:
									diff = abs(d - r)
							return (ctr, r, diff)
						#TODO
						#d12 = (pending[1][1] - pending[0][1]) ** 2 + (pending[1][2] - pending[0][2]) ** 2
						#d22 = (pending[2][1] - pending[1][1]) ** 2 + (pending[2][2] - pending[1][2]) ** 2
						#de1 = pending[1][4] - pending[0][4]
						#de2 = pending[2][4] - pending[1][4]
						#f = pending[0][5]
						#current_arc = ...
						pending.append(nums)
					if len(pending) > 3:
						# TODO: If new point doesn't fit on circle, push pending as circle to output.  Otherwise, add current point.
						pending.append(nums)
					flush_pending()
					return
				else:
					flush_pending()
				dst.write(struct.pack('=Bl' + 'd' * 8, type, *add_timedist(type, nums)))
			def flush_pending():
				tmp = pending[1:]
				pending[:] = []
				for p in tmp:
					add_record(protocol.parsed['LINE'], p, True)
			def add_string(string):
				if string is None:
					return 0
				if string not in strings:
					strings.append(string)
				return strings.index(string)
			current_extruder = 0
			for lineno, origline in enumerate(src):
				line = origline.strip()
				#log('parsing %s' % line)
				# Get rid of line numbers and checksums.
				if line.startswith('N'):
					r = re.match(r'N(\d+)\s+(.*?)\*\d+\s*$', line)
					if not r:
						r = re.match(r'N(\d+)\s+(.*?)\s*$', line)
						if not r:
							# Invalid line; ignore it.
							errors.append('%d:ignoring invalid gcode: %s' % (lineno, origline))
							continue
					lineno = int(r.group(1))
					line = r.group(2)
				else:
					lineno += 1
				comment = ''
				while '(' in line:
					b = line.index('(')
					e = line.find(')', b)
					if e < 0:
						errors.append('%d:ignoring line with unterminated comment: %s' % (lineno, origline))
						continue
					comment = line[b + 1:e].strip()
					line = line[:b] + ' ' + line[e + 1:].strip()
				if ';' in line:
					p = line.index(';')
					comment = line[p + 1:].strip()
					line = line[:p].strip()
				if comment.upper().startswith('MSG,'):
					message = comment[4:].strip()
				elif comment.startswith('SYSTEM:'):
					if not re.match(self.allow_system, comment[7:]):
						errors.append('Warning: system command %s is forbidden and will not be run' % comment[7:])
					add_record(protocol.parsed['SYSTEM'], [add_string(comment[7:])])
					continue
				if line == '':
					continue
				line = line.split()
				while len(line) > 0:
					if mode is None or line[0][0] in 'GMTDS':
						if len(line[0]) < 2:
							errors.append('%d:ignoring unparsable line: %s' % (lineno, origline))
							break
						try:
							cmd = line[0][0], int(line[0][1:])
						except:
							errors.append('%d:parse error in line: %s' % (lineno, origline))
							traceback.print_exc()
							break
						line = line[1:]
					else:
						cmd = mode
					args = {}
					success = True
					for i, a in enumerate(line):
						if a[0] in 'GMD':
							line = line[i:]
							break
						try:
							args[a[0]] = float(a[1:])
						except:
							errors.append('%d:ignoring invalid gcode: %s' % (lineno, origline))
							success = False
							break
					else:
						line = []
					if not success:
						break
					if cmd == ('M', 2):
						# Program end.
						break
					elif cmd[0] == 'T':
						target = cmd[1]
						if target >= len(pos[1]):
							pos[1].extend([0.] * (target - len(pos[1]) + 1))
						current_extruder = target
						# Force update of extruder.
						add_record(protocol.parsed['LINE'], {'X': pos[0][0], 'Y': pos[0][1], 'Z': pos[0][2], 'E': pos[1][current_extruder], 'f': float('inf'), 'F': float('inf'), 'T': current_extruder})
						continue
					elif cmd == ('G', 17):
						arc_normal = (0, 0, 1)
						continue
					elif cmd == ('G', 18):
						arc_normal = (0, 1, 0)
						continue
					elif cmd == ('G', 19):
						arc_normal = (1, 0, 0)
						continue
					elif cmd == ('G', 20):
						unit = 25.4
						continue
					elif cmd == ('G', 21):
						unit = 1.
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
					elif cmd == ('M', 84):
						for e in range(len(pos[1])):
							pos[1][e] = 0.
					elif cmd == ('G', 92):
						if 'E' not in args:
							continue
						args['E'] *= unit
						pos[1][current_extruder] = args['E']
					elif cmd[0] == 'M' and cmd[1] in (104, 109, 116):
						args['E'] = int(args['T']) if 'T' in args else current_extruder
					if cmd == ('M', 140):
						cmd = ('M', 104)
						args['E'] = -2
					elif cmd == ('M', 190):
						cmd = ('M', 109)
						args['E'] = -2
					elif cmd == ('M', 6):
						# Tool change: park and remember to probe.
						cmd = ('G', 28)
						tool_changed = True
					if cmd == ('G', 28):
						nums = [current_extruder]
						if len(self.spaces) > 1 and len(self.spaces[1].axis) > current_extruder:
							pos[1][current_extruder] = 0.
						add_record(protocol.parsed['PARK'])
						for a in range(len(pos[0])):
							if len(self.spaces[0].axis) > a and not math.isnan(self.spaces[0].axis[a]['park']):
								pos[0][a] = float('nan')
					elif cmd[0] == 'G' and cmd[1] in (0, 1, 81):
						if cmd[1] != 0:
							mode = cmd
						components = {'X': None, 'Y': None, 'Z': None, 'A': None, 'B': None, 'C': None, 'E': None, 'F': None, 'R': None}
						for c in args:
							if c not in components:
								errors.append('%d:invalid component %s' % (lineno, c))
								continue
							assert components[c] is None
							components[c] = args[c]
						f0 = pos[2]
						if components['F'] is not None:
							pos[2] = components['F'] * unit / 60
						oldpos = pos[0][:], pos[1][:]
						if cmd[1] != 81:
							if components['E'] is not None:
								if erel or (erel is None and rel):
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
						for axis in range(6):
							value = components['XYZABC'[axis]]
							if value is not None:
								if rel:
									pos[0][axis] += value * unit
								else:
									pos[0][axis] = value * unit
								if axis == 2:
									z = pos[0][2]
						if cmd[1] != 81:
							dist = sum([0] + [(pos[0][x] - oldpos[0][x]) ** 2 for x in range(3) if not math.isnan(pos[0][x] - oldpos[0][x])]) ** .5
							if dist > 0:
								#if f0 is None:
								#	f0 = pos[1][current_extruder]
								f0 = pos[2]	# Always use new value.
								if f0 == 0:
									f0 = float('inf')
							if math.isnan(dist):
								dist = 0
							if all((math.isnan(pos[0][i]) and math.isnan(oldpos[0][i])) or pos[0][i] == oldpos[0][i] for i in range(3, 6)):
								add_record(protocol.parsed['LINE'], {'X': pos[0][0], 'Y': pos[0][1], 'Z': pos[0][2], 'E': pos[1][current_extruder], 'f': f0 / dist if dist > 0 and cmd[1] == 1 else float('inf'), 'F': pos[2] / dist if dist > 0 and cmd[1] == 1 else float('inf'), 'T': current_extruder})
							else:
								add_record(protocol.parsed['PRE_LINE'], {'X': pos[0][3], 'Y': pos[0][4], 'Z': pos[0][5], 'E': float('NaN'), 'f': float('NaN'), 'F': float('NaN'), 'T': current_extruder})
								add_record(protocol.parsed['LINE'], {'X': pos[0][0], 'Y': pos[0][1], 'Z': pos[0][2], 'E': pos[1][current_extruder], 'f': f0 / dist if dist > 0 and cmd[1] == 1 else float('inf'), 'F': pos[2] / dist if dist > 0 and cmd[1] == 1 else float('inf'), 'T': current_extruder})
						else:
							# If old pos is unknown, use safe distance.
							if math.isnan(oldpos[0][2]):
								oldpos[0][2] = r
							# Drill cycle.
							# Only support OLD_Z (G90) retract mode; don't support repeats(L).
							# goto x,y
							add_record(protocol.parsed['LINE'], {'X': pos[0][0], 'Y': pos[0][1], 'Z': oldpos[0][2], 'E': 0, 'f': float('inf'), 'F': float('inf'), 'T': current_extruder})
							# goto r
							add_record(protocol.parsed['LINE'], {'X': pos[0][0], 'Y': pos[0][1], 'Z': r, 'E': 0, 'f': float('inf'), 'F': float('inf'), 'T': current_extruder})
							# goto z; this is always straight down, because the move before and after it are also vertical.
							if z != r:
								f0 = pos[2] / abs(z - r)
								if math.isnan(f0):
									f0 = float('inf')
								add_record(protocol.parsed['LINE'], {'X': pos[0][0], 'Y': pos[0][1], 'Z': z, 'E': 0, 'f': f0, 'F': f0, 'T': current_extruder})
							# retract; this is always straight up, because the move before and after it are also non-horizontal.
							add_record(protocol.parsed['LINE'], {'X': pos[0][0], 'Y': pos[0][1], 'Z': oldpos[0][2], 'E': 0, 'f': float('inf'), 'F': float('inf'), 'T': current_extruder})
							# empty move; this makes sure the previous move is entirely vertical.
							add_record(protocol.parsed['LINE'], {'X': pos[0][0], 'Y': pos[0][1], 'Z': oldpos[0][2], 'E': 0, 'f': float('inf'), 'F': float('inf'), 'T': current_extruder})
							# Set up current z position so next G81 will work.
							pos[0][2] = oldpos[0][2]
					elif cmd[0] == 'G' and cmd[1] in (2, 3):
						# Arc.
						mode = cmd
						components = {'X': None, 'Y': None, 'Z': None, 'E': None, 'F': None, 'I': None, 'J': None, 'K': None}
						for c in args:
							if c not in components:
								errors.append('%d:invalid arc component %s' % (lineno, c))
								continue
							assert components[c] is None
							components[c] = args[c]
						f0 = pos[2]
						if components['F'] is not None:
							pos[2] = components['F'] * unit / 60
						oldpos = pos[0][:], pos[1][:]
						if components['E'] is not None:
							if erel or (erel is None and rel):
								estep = components['E'] * unit
							else:
								estep = components['E'] * unit - pos[1][current_extruder]
							pos[1][current_extruder] += estep
						else:
							estep = 0
						center = [None] * 3
						for axis in range(3):
							value = components[chr(b'X'[0] + axis)]
							if value is not None:
								if rel:
									pos[0][axis] += value * unit
								else:
									pos[0][axis] = value * unit
								if axis == 2:
									z = pos[0][2]
							value = components[chr(b'I'[0] + axis)]
							if value is not None:
								center[axis] = oldpos[0][axis] + value
							else:
								center[axis] = oldpos[0][axis]
						s = -1 if cmd[1] == 2 else 1
						add_record(protocol.parsed['PRE_ARC'], {'X': center[0], 'Y': center[1], 'Z': center[2], 'E': s * arc_normal[0], 'f': s * arc_normal[1], 'F': s * arc_normal[2], 'T': 0})
						add_record(protocol.parsed['ARC'], {'X': pos[0][0], 'Y': pos[0][1], 'Z': pos[0][2], 'E': pos[1][current_extruder], 'f': -f0, 'F': -pos[2], 'T': current_extruder})
					elif cmd == ('G', 4):
						add_record(protocol.parsed['WAIT'], [0, float(args['P']) / 1000 if 'P' in args else 0])
					elif cmd == ('G', 92):
						add_record(protocol.parsed['SETPOS'], [current_extruder, args['E']])
					elif cmd == ('G', 94):
						# Set feedrate to units per minute; this is always used, and it shouldn't raise an error.
						pass
					elif cmd == ('M', 0):
						add_record(protocol.parsed['CONFIRM'], [add_string(message), 1 if tool_changed else 0])
						tool_changed = False
					elif cmd == ('M', 3):
						# Spindle on, clockwise.
						add_record(protocol.parsed['GPIO'], [-3, 1])
					elif cmd == ('M', 4):
						# Spindle on, counterclockwise.
						add_record(protocol.parsed['GPIO'], [-3, 1])
					elif cmd == ('M', 5):
						add_record(protocol.parsed['GPIO'], [-3, 0])
					elif cmd == ('M', 9):
						# Coolant off: ignore.
						pass
					elif cmd == ('M', 42):
						if 'P' in args and 'S' in args:
							add_record(protocol.parsed['GPIO'], [args['P'], args.get('S')])
						else:
							errors.append('%d:invalid M42 request (needs P and S)' % lineno)
					elif cmd == ('M', 84):
						# Don't sleep, but set all extruder positions to 0.
						for e in range(len(pos[1])):
							add_record(protocol.parsed['SETPOS'], [e, 0])
					elif cmd == ('M', 104):
						if args['E'] >= len(self.temps):
							errors.append('ignoring M104 for invalid temp %d' % args['E'])
						elif 'S' not in args:
							errors.append('ignoring M104 without S')
						else:
							add_record(protocol.parsed['SETTEMP'], [args['E'], args['S'] + C0])
					elif cmd == ('M', 106):
						add_record(protocol.parsed['GPIO'], [-2, 1])
					elif cmd == ('M', 107):
						add_record(protocol.parsed['GPIO'], [-2, 0])
					elif cmd == ('M', 109):
						if 'S' in args:
							add_record(protocol.parsed['SETTEMP'], [args['E'], args['S'] + C0])
						add_record(protocol.parsed['WAITTEMP'], [args['E']])
					elif cmd == ('M', 116):
						add_record(protocol.parsed['WAITTEMP'], [-2])
					elif cmd[0] == 'S':
						# Spindle speed; not supported, but shouldn't error.
						pass
					else:
						errors.append('%d:invalid gcode command %s' % (lineno, repr((cmd, args))))
					message = None
			flush_pending()
			stringmap = []
			size = 0
			for s in strings:
				us = s.encode('utf-8')
				stringmap.append(len(us))
				dst.write(us)
				size += len(us)
			for s in stringmap:
				dst.write(struct.pack('=L', s))
			ret = bbox
			if any(x is None for x in bbox[:4]):
				bbox = bbox_last
				ret = bbox
				if any(x is None for x in bbox[:4]):
					bbox = [0] * 6
					ret = None
			if any(x is None for x in bbox):
				for t, b in enumerate(bbox):
					if b is None:
						bbox[t] = 0;
			dst.write(struct.pack('=L' + 'd' * 8, len(strings), *(bbox + time_dist)))
		self._broadcast(None, 'blocked', None)
		return ret and ret + time_dist, errors
	# }}}
	def _reset_extruders(self, axes): # {{{
		for i, sp in enumerate(axes):
			for a, pos in enumerate(sp):
				# Assume motor[a] corresponds to axis[a] if it exists.
				if len(self.spaces[i].motor) > a and not self._pin_valid(self.spaces[i].motor[a]['limit_max_pin']) and not self._pin_valid(self.spaces[i].motor[a]['limit_min_pin']):
					self.set_axis_pos(i, a, pos)
	# }}}
	def _pin_valid(self, pin):	# {{{
		return(pin & 0x100) != 0
	# }}}
	def _spi_send(self, data): # {{{
		for bits, p in data:
			shift = (8 - bits % 8) % 8
			if shift > 0:
				p = [(p[b] << shift | p[b + 1] >> (8 - shift)) & 0xff for b in range(len(p) - 1)] + [(p[-1] << shift) & 0xff]
			self._send_packet(struct.pack('=BB', protocol.command['SPI'], bits) + b''.join(struct.pack('=B', b) for b in p))
	# }}}
	# Subclasses.  {{{
	class Space: # {{{
		def __init__(self, printer, id):
			self.name = ['position', 'extruders', 'followers'][id]
			self.printer = printer
			self.id = id
			self.axis = []
			self.motor = []
			self.delta = [{'axis_min': 0., 'axis_max': 0., 'rodlength': 0., 'radius': 0.} for t in range(3)]
			self.delta_angle = 0
			self.polar_max_r = float('inf')
			self.extruder = []
		def read(self, data):
			axes, motors = data
			if self.id == 1:
				self.printer.multipliers = (self.printer.multipliers + [1.] * len(axes))[:len(axes)]
			if len(axes) > len(self.axis):
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
				self.axis += [{'name': nm(i)} for i in range(len(self.axis), len(axes))]
			else:
				self.axis[len(axes):] = []
			for a in range(len(axes)):
				self.axis[a]['park'], self.axis[a]['park_order'], self.axis[a]['min'], self.axis[a]['max'] = struct.unpack('=dBdd', axes[a])
			if len(motors) > len(self.motor):
				self.motor += [{} for i in range(len(self.motor), len(motors))]
			else:
				self.motor[len(motors):] = []
			for m in range(len(motors)):
				self.motor[m]['step_pin'], self.motor[m]['dir_pin'], self.motor[m]['enable_pin'], self.motor[m]['limit_min_pin'], self.motor[m]['limit_max_pin'], self.motor[m]['steps_per_unit'], self.motor[m]['home_pos'], self.motor[m]['limit_v'], self.motor[m]['limit_a'], self.motor[m]['home_order'] = struct.unpack('=HHHHHddddB', motors[m])
				if self.id == 1 and m < len(self.printer.multipliers):
					self.motor[m]['steps_per_unit'] /= self.printer.multipliers[m]
		def write_info(self, num_axes = None):
			data = struct.pack('=B', self.type)
			if self.type == TYPE_CARTESIAN:
				data += struct.pack('=B', num_axes if num_axes is not None else len(self.axis))
			elif self.type == TYPE_DELTA:
				for a in range(3):
					data += struct.pack('=dddd', self.delta[a]['axis_min'], self.delta[a]['axis_max'], self.delta[a]['rodlength'], self.delta[a]['radius'])
				data += struct.pack('=d', self.delta_angle)
			elif self.type == TYPE_POLAR:
				data += struct.pack('=d', self.polar_max_r)
			elif self.type == TYPE_EXTRUDER:
				num = num_axes if num_axes is not None else len(self.axis)
				data += struct.pack('=B', num)
				for a in range(num):
					if a < len(self.extruder):
						data += struct.pack('=ddd', self.extruder[a]['dx'], self.extruder[a]['dy'], self.extruder[a]['dz'])
					else:
						data += struct.pack('=ddd', 0, 0, 0)
			elif self.type == TYPE_FOLLOWER:
				num = num_axes if num_axes is not None else len(self.axis)
				data += struct.pack('=B', num)
				for a in range(num):
					if a < len(self.follower):
						data += struct.pack('=BB', self.follower[a]['space'], self.follower[a]['motor'])
					else:
						data += struct.pack('=BB', 0xff, 0xff)
			else:
				log('invalid type')
				raise AssertionError('invalid space type')
			return data
		def write_axis(self, axis):
			if self.id == 0:
				return struct.pack('=dBdd', self.axis[axis]['park'], int(self.axis[axis]['park_order']), self.axis[axis]['min'], self.axis[axis]['max'])
			else:
				return struct.pack('=dBdd', float('nan'), 0, float('-inf'), float('inf'))
		def write_motor(self, motor):
			return struct.pack('=HHHHHddddB', self.motor[motor]['step_pin'], self.motor[motor]['dir_pin'], self.motor[motor]['enable_pin'], self.motor[motor]['limit_min_pin'], self.motor[motor]['limit_max_pin'], self.motor[motor]['steps_per_unit'] * (1. if self.id != 1 or motor >= len(self.printer.multipliers) else self.printer.multipliers[motor]), self.motor[motor]['home_pos'], self.motor[motor]['limit_v'], self.motor[motor]['limit_a'], int(self.motor[motor]['home_order']))
		def set_current_pos(self, axis, pos):
			#log('setting pos of %d %d to %f' % (self.id, axis, pos))
			self.printer._send_packet(struct.pack('=BBBd', protocol.command['SETPOS'], self.id, axis, pos))
		def get_current_pos(self, axis):
			#log('getting current pos %d %d' % (self.id, axis))
			self.printer._send_packet(struct.pack('=BBB', protocol.command['GETPOS'], self.id, axis))
			cmd, s, m, f, e, data = self.printer._get_reply()
			assert cmd == protocol.rcommand['POS']
			#log('get current pos %d %d: %f' % (self.id, axis, f))
			return f
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
			std = [self.name, self.type, [[a['name'], a['park'], a['park_order'], a['min'], a['max']] for a in self.axis], [[self.motor_name(i), m['step_pin'], m['dir_pin'], m['enable_pin'], m['limit_min_pin'], m['limit_max_pin'], m['steps_per_unit'], m['home_pos'], m['limit_v'], m['limit_a'], m['home_order']] for i, m in enumerate(self.motor)], None if self.id != 1 else self.printer.multipliers]
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
			type = self.type if self.id != 0 or self.printer.home_phase is None else self.printer.home_orig_type
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
					ret += ''.join(['%s = %f\r\n' % (x, a[x]) for x in ('park', 'park_order')])
					if self.printer.home_phase is None:
						ret += ''.join(['%s = %f\r\n' % (x, a[x]) for x in ('min', 'max')])
					else:
						ret += ''.join(['%s = %f\r\n' % (x, y) for x, y in zip(('min', 'max'), self.printer.home_limits[self.id])])
			for i, m in enumerate(self.motor):
				ret += '[motor %d %d]\r\n' % (self.id, i)
				ret += ''.join(['%s = %s\r\n' % (x, write_pin(m[x])) for x in ('step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin')])
				ret += ''.join(['%s = %d\r\n' % (x, m[x]) for x in ('home_order',)])
				ret += ''.join(['%s = %f\r\n' % (x, m[x]) for x in ('steps_per_unit', 'limit_v', 'limit_a')])
				if self.id != 1:
					ret += 'home_pos = %f\r\n' % m['home_pos']
			return ret
	# }}}
	class Temp: # {{{
		def __init__(self, id):
			self.name = 'temp %d' % id
			self.id = id
			self.value = float('nan')
		def read(self, data):
			self.R0, self.R1, logRc, Tc, self.beta, self.heater_pin, self.fan_pin, self.thermistor_pin, fan_temp, self.fan_duty, heater_limit, fan_limit = struct.unpack('=dddddHHHdddd', data)
			try:
				self.Rc = math.exp(logRc)
			except:
				self.Rc = float('nan')
			self.Tc = Tc - C0
			self.heater_limit = heater_limit - C0
			self.fan_limit = fan_limit - C0
			self.fan_temp = fan_temp - C0
			self.fan_pin ^= 0x200
		def write(self):
			try:
				logRc = math.log(self.Rc)
			except:
				logRc = float('nan')
			return struct.pack('=dddddHHHdddd', self.R0, self.R1, logRc, self.Tc + C0, self.beta, self.heater_pin, self.fan_pin ^ 0x200, self.thermistor_pin, self.fan_temp + C0, self.fan_duty, self.heater_limit + C0, self.fan_limit + C0)
		def export(self):
			return [self.name, self.R0, self.R1, self.Rc, self.Tc, self.beta, self.heater_pin, self.fan_pin, self.thermistor_pin, self.fan_temp, self.fan_duty, self.heater_limit, self.fan_limit, self.value]
		def export_settings(self):
			ret = '[temp %d]\r\n' % self.id
			ret += 'name = %s\r\n' % self.name
			ret += ''.join(['%s = %s\r\n' % (x, write_pin(getattr(self, x))) for x in ('heater_pin', 'fan_pin', 'thermistor_pin')])
			ret += ''.join(['%s = %f\r\n' % (x, getattr(self, x)) for x in ('fan_temp', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'fan_duty', 'heater_limit', 'fan_limit')])
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
		def read(self, data):
			self.pin, state, self.duty = struct.unpack('=HBd', data)
			self.state = state & 0x3
			self.reset = (state >> 2) & 0x3
		def write(self):
			return struct.pack('=HBd', self.pin, self.state | (self.reset << 2), self.duty)
		def export(self):
			return [self.name, self.pin, self.state, self.reset, self.duty, self.value if self.state >= 2 else self.state == 1]
		def export_settings(self):
			ret = '[gpio %d]\r\n' % self.id
			ret += 'name = %s\r\n' % self.name
			ret += 'pin = %s\r\n' % write_pin(self.pin)
			ret += 'reset = %d\r\n' % self.reset
			ret += 'duty = %d\r\n' % self.duty
			return ret
	# }}}
	# }}}
	# }}}
	# Useful commands.  {{{
	def expert_die(self, reason): # {{{
		'''Terminate the driver process.
		'''
		log('%s dying as requested by host (%s).' % (self.uuid, reason))
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
			self.line()
		#log('end flush preparation')
	# }}}
	@delayed
	def probe(self, id, area, angle = 0, speed = 3.): # {{{
		'''Run a probing routine.
		This moves over the given area and probes a grid of points less
		than max_probe_distance apart.
		If the probe pin is valid, it will be used for the probe.
		If it is invalid, a confirmation is required for every point.
		'''
		if len(self.spaces[0].axis) < 3 or not self.probe_safe_dist > 0:
			if id is not None:
				self._send(id, 'return', None)
			return
		density = [int(area[t + 2] / self.probe_dist) + 1 for t in range(2)]
		self.probemap = [area, density, [[[] for x in range(density[0] + 1)] for y in range(density[1] + 1)]]
		angle = math.radians(angle)
		self.gcode_angle = math.sin(angle), math.cos(angle)
		self.probe_speed = speed
		self._do_probe(id, 0, 0, self.get_axis_pos(0, 2), angle)
	# }}}
	def line(self, moves = (), f0 = None, f1 = None, v0 = None, v1 = None, relative = False, probe = False, single = False, force = False): # {{{
		'''Move the tool in a straight line.
		'''
		#log('line %s %s %s %d %d' % (repr(moves), f0, f1, probe))
		#log('speed %s' % f0)
		#traceback.print_stack()
		if not force and self.home_phase is not None and not self.paused:
			log('ignoring line during home')
			return
		self.queue.append((moves, f0, f1, v0, v1, probe, single, relative))
		if not self.wait:
			self._do_queue()
	# }}}
	@delayed
	def line_cb(self, id, moves = (), f0 = None, f1 = None, v0 = None, v1 = None, relative = False, probe = False, single = False): # {{{
		'''Move the tool in a straight line; return when done.
		'''
		if self.home_phase is not None and not self.paused:
			log('ignoring linecb during home')
			if id is not None:
				self._send(id, 'return', None)
			return
		self.line(moves, f0, f1, v0, v1, relative, probe, single)
		self.wait_for_cb()[1](id)
	# }}}
	def move_target(self, dx, dy): # {{{
		'''Move the target position.
		Using this function avoids a round trip to the driver.
		'''
		self.set_globals(targetx = self.targetx + dx, targety = self.targety + dy)
	# }}}
	def sleep(self, sleeping = True, update = True, force = False): # {{{
		'''Put motors to sleep, or wake them up.
		'''
		if sleeping:
			if self.home_phase is not None or (not force and not self.paused and (self.gcode_map is not None or self.gcode_file)):
				return
			self.position_valid = False
			if update:
				self._globals_update()
		self._send_packet(struct.pack('=BB', protocol.command['SLEEP'], sleeping))
	# }}}
	def settemp(self, channel, temp, update = True): # {{{
		'''Set target temperature.
		'''
		channel = int(channel)
		self.temps[channel].value = temp
		if update:
			self._temp_update(channel)
		self._send_packet(struct.pack('=BBd', protocol.command['SETTEMP'], channel, temp + C0 if not math.isnan(self.temps[channel].beta) else temp))
		if self.gcode_waiting > 0 and any(channel == x[0] for x in self.tempcb):
			self.waittemp(channel, temp)
	# }}}
	def waittemp(self, channel, min, max = None): # {{{
		'''Set temperature alarm values.
		Note that this function returns immediately; it does not wait
		for the temperature to be reached.
		'''
		channel = int(channel)
		if min is None:
			min = float('nan')
		if max is None:
			max = float('nan')
		self._send_packet(struct.pack('=BBdd', protocol.command['WAITTEMP'], channel, min + C0 if not math.isnan(self.temps[channel].beta) else min, max + C0 if not math.isnan(self.temps[channel].beta) else max))
	# }}}
	def readtemp(self, channel): # {{{
		'''Read current temperature.
		'''
		channel = int(channel)
		if channel >= len(self.temps):
			log('Trying to read invalid temp %d' % channel)
			return float('nan')
		self._send_packet(struct.pack('=BB', protocol.command['READTEMP'], channel))
		cmd, s, m, f, e, data = self._get_reply()
		assert cmd == protocol.rcommand['TEMP']
		return f - (C0 if not math.isnan(self.temps[channel].beta) else 0)
	# }}}
	def readpower(self, channel): # {{{
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
			return float('nan')
		self._send_packet(struct.pack('=BB', protocol.command['READPOWER'], channel))
		cmd, s, m, f, e, data = self._get_reply()
		assert cmd == protocol.rcommand['POWER']
		return s, m
	# }}}
	def readpin(self, pin): # {{{
		'''Read current value of a gpio pin.
		'''
		self._send_packet(struct.pack('=BB', protocol.command['READPIN'], pin))
		cmd, s, m, f, e, data = self._get_reply()
		assert cmd == protocol.rcommand['PIN']
		return bool(s)
	# }}}
	def load(self, profile = None, update = True): # {{{
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
				self.expert_import_settings(f.read(), update = update)
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
		with fhs.write_data(os.path.join(self.uuid, 'profile')) as f:
			f.write(profile.strip() + '\n')
	# }}}
	def abort(self): # {{{
		'''Abort the current job.
		'''
		for t, temp in enumerate(self.temps):
			self.settemp(t, float('nan'))
		self.pause(store = False)
		self.sleep(force = True);
		for g, gpio in enumerate(self.gpios):
			self.set_gpio(g, state = gpio.reset)
		self._print_done(False, 'aborted by user')
	# }}}
	def pause(self, pausing = True, store = True, update = True): # {{{
		'''Pause or resume the machine.
		'''
		was_paused = self.paused
		if pausing:
			self._send_packet(struct.pack('=BB', protocol.command['QUEUED'], True))
			cmd, s, m, f, e, data = self._get_reply()
			if cmd != protocol.rcommand['QUEUE']:
				log('invalid reply to queued command')
				return
			self.movewait = 0
			self.wait = False
		self.paused = pausing
		if not self.paused:
			if was_paused:
				# Go back to pausing position.
				# First reset all axes that don't have a limit switch.
				self._reset_extruders(self.queue_info[1])
				self.line(self.queue_info[1])
				# TODO: adjust extrusion of current segment to shorter path length.
				#log('resuming')
				self.resuming = True
			#log('sending resume')
			self._send_packet(bytes((protocol.command['RESUME'],)))
			self._do_queue()
		else:
			#log('pausing')
			if not was_paused:
				#log('pausing %d %d %d %d %d' % (store, self.queue_info is None, len(self.queue), self.queue_pos, s))
				if store and self.queue_info is None and ((len(self.queue) > 0 and self.queue_pos - s >= 0) or self.gcode_file):
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
						#log('killing prober')
						self.movecb.remove(self.probe_cb)
						self.probe_cb[1](None)
						store = False
					#log('pausing gcode %d/%d/%d' % (self.queue_pos, s, len(self.queue)))
					if self.flushing is None:
						self.flushing = False
					if store:
						self.queue_info = [len(self.queue) if self.gcode_file else self.queue_pos - s, [[s.get_current_pos(a) for a in range(len(s.axis))] for s in self.spaces], self.queue, self.movecb, self.flushing]
				else:
					#log('stopping')
					self.paused = False
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
		self._send_packet(struct.pack('=BB', protocol.command['QUEUED'], False))
		cmd, s, m, f, e, data = self._get_reply()
		if cmd != protocol.rcommand['QUEUE']:
			log('invalid reply to queued command')
			return None
		return s
	# }}}
	@delayed
	def home(self, id, speed = 5, cb = None, abort = True): # {{{
		'''Recalibrate the position with its limit switches.
		'''
		if self.home_phase is not None and not self.paused:
			log("ignoring request to home because we're already homing")
			if id is not None:
				self._send(id, 'return', None)
			return
		# Abort only if it is requested, and the job is not paused.
		if abort and self.queue_info is None:
			self._print_done(False, 'aborted by homing')
		self.home_phase = 0
		self.home_id = id
		self.home_speed = speed
		self.home_done_cb = cb
		for i, e in enumerate(self.spaces[1].axis):
			self.set_axis_pos(1, i, 0)
		self._do_home()
	# }}}
	@delayed
	def park(self, id, cb = None, abort = True, order = 0, aborted = False): # {{{
		'''Go to the park position.
		Home first if the position is unknown.
		'''
		if aborted:
			if id is not None:
				self._send(id, 'error', 'aborted')
			return
		#log('parking')
		if abort:
			self._print_done(False, 'aborted by parking')
		self.parking = True
		if not self.position_valid:
			#log('homing')
			self.home(cb = lambda: self.park(cb, abort = False)[1](id), abort = False)[1](None)
			return
		next_order = None
		for s in self.spaces:
			topark = [a['park_order'] for a in s.axis if not math.isnan(a['park']) and a['park_order'] >= order]
			if len(topark) > 0 and (next_order is None or min(topark) > next_order):
				next_order = min(topark)
		if next_order is None:
			self.parking = False
			if cb:
				def wrap_cb(done):
					call_queue.append((cb, []))
					if id is not None:
						self._send(id, 'return', None)
				self.movecb.append((False, wrap_cb))
				self.line()
			else:
				if id is not None:
					self._send(id, 'return', None)
			return
		self.movecb.append((False, lambda done: self.park(cb, False, next_order + 1, not done)[1](id)))
		self.line([[a['park'] - (0 if si != 0 or ai != 2 else self.zoffset) if a['park_order'] == next_order else float('nan') for ai, a in enumerate(s.axis)] for si, s in enumerate(self.spaces)])
	# }}}
	@delayed
	def benjamin_audio_play(self, id, name, motor = 0): # {{{
		self.audio_id = id
		self.sleep(False)
		filename = fhs.read_spool(os.path.join(self.uuid, 'audio', name + os.extsep + 'bin'), opened = False)
		self._send_packet(struct.pack('=BBddBB', protocol.command['RUN_FILE'], 1, 0, 0, motor, 0) + filename.encode('utf8'))
	# }}}
	def benjamin_audio_add_file(self, filename, name): # {{{
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
	def clear_alarm(self, which = None): # {{{
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
	def clear_limits(self):	# {{{
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
		message += 'unit_name=%s\r\n' % self.unit_name
		message += 'spi_setup=%s\r\n' % self._mangle_spi()
		message += ''.join(['%s = %s\r\n' % (x, write_pin(getattr(self, x))) for x in ('led_pin', 'stop_pin', 'probe_pin', 'spiss_pin')])
		message += ''.join(['%s = %d\r\n' % (x, getattr(self, x)) for x in ('bed_id', 'fan_id', 'spindle_id', 'park_after_print', 'sleep_after_print', 'cool_after_print', 'timeout')])
		message += ''.join(['%s = %f\r\n' % (x, getattr(self, x)) for x in ('probe_dist', 'probe_safe_dist', 'temp_scale_min', 'temp_scale_max', 'max_deviation', 'max_v')])
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
		self.sleep(update = update)
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
				'general': {'num_temps', 'num_gpios', 'led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'probe_dist', 'probe_safe_dist', 'bed_id', 'fan_id', 'spindle_id', 'unit_name', 'timeout', 'temp_scale_min', 'temp_scale_max', 'park_after_print', 'sleep_after_print', 'cool_after_print', 'spi_setup', 'max_deviation', 'max_v'},
				'space': {'type', 'num_axes', 'delta_angle', 'polar_max_r'},
				'temp': {'name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit', 'fan_limit'},
				'gpio': {'name', 'pin', 'state', 'reset', 'duty'},
				'axis': {'name', 'park', 'park_order', 'min', 'max'},
				'motor': {'step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'steps_per_unit', 'home_pos', 'limit_v', 'limit_a', 'home_order'},
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
				# Ignore settings for incorrect setion.
				continue
			if not r.group(6):
				# Comment or empty line.
				continue
			key = r.group(6)
			value = r.group(7)
			if key.endswith('name'):
				pass
			elif key == 'spi_setup':
				value = self._unmangle_spi(value)
			elif key.endswith('pin'):
				value = read_pin(value)
			elif key.startswith('num') or section == 'follower' or key.endswith('_id'):
				value = int(value)
			else:
				value = float(value)
			if key not in keys[section] or (section == 'motor' and key == 'home_pos' and index[0] == 1):
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
		# Update values in the printer by calling the expert_set_* functions with no new settings.
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
	def expert_import_file(self, filename, name): # {{{
		'''Import settings using a POST request.
		Note that this function can only be called using POST; not with the regular websockets system.
		'''	# FIXME: At the moment there is no protection against calling it from the websocket.  This is a security risk.
		return ', '.join('%s (%s)' % (msg, ln) for ln, msg in self.expert_import_settings(open(filename).read(), name))
	# }}}
	@delayed
	def gcode_run(self, id, code, angle = 0, probemap = None): # {{{
		'''Run a string of g-code.
		'''
		self.probemap = probemap
		with fhs.write_temp(text = False) as f:
			f.write(code)
			f.seek(0)
			self.gcode_id = id
			# Break this in two, otherwise tail recursion may destroy f before call is done?
			ret = self._gcode_run(f.filename, angle)
			return ret
	# }}}
	@delayed
	def request_confirmation(self, id, message): # {{{
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
	def confirm(self, confirm_id, success = True): # {{{
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
					self._print_done(False, 'aborted by failed confirmation')
				else:
					if self.probe_pending and self._pin_valid(self.probe_pin):
						self.probe_pending = False
						call_queue.append((self._one_probe, []))
					else:
						self.probe_pending = False
						self._send_packet(bytes((protocol.command['RESUME'],)))
		return True
	# }}}
	def queue_add(self, data, name): # {{{
		'''Add code to the queue as a string.
		'''
		with fhs.write_temp() as f:
			f.write(data)
			f.seek(0)
			return self._queue_add(f, name)
	# }}}
	def queue_add_file(self, filename, name): # {{{
		'''Add g-code to queue using a POST request.
		Note that this function can only be called using POST; not with the regular websockets system.
		'''	# FIXME: At the moment there is no protection against calling it from the websocket.  This is a security risk.
		log('post queue add')
		with open(filename) as f:
			return ', '.join(self._queue_add(f, name))
	# }}}
	def queue_remove(self, name, audio = False): # {{{
		'''Remove an entry from the queue.
		'''
		assert name in self.jobqueue
		#log('removing %s' % name)
		if audio:
			filename = fhs.read_spool(os.path.join(self.uuid, 'audio', name + os.extsep + 'bin'), opened = False)
			del self.audioqueue[name]
			self._broadcast(None, 'audioqueue', tuple(self.audioqueue.keys()))
		else:
			filename = fhs.read_spool(os.path.join(self.uuid, 'gcode', name + os.extsep + 'bin'), opened = False)
			del self.jobqueue[name]
			self._broadcast(None, 'queue', [(q, self.jobqueue[q]) for q in self.jobqueue])
		try:
			os.unlink(filename)
		except:
			log('unable to unlink %s' % filename)
	# }}}
	@delayed
	def queue_print(self, id, names, angle = 0, probemap = None): # {{{
		'''Run one or more new jobs.
		'''
		if len(self.jobs_active) > 0 and not self.paused:
			log('ignoring probe request while print is in progress: %s' % repr(self.jobs_active) + str(self.paused))
			if id is not None:
				self._send(id, 'return', None)
			return
		self.job_output = ''
		self.jobs_active = names
		self.jobs_angle = angle
		self.probemap = probemap
		self.job_current = -1	# next_job will make it start at 0.
		self.job_id = id
		if not self.probing:
			self._next_job()
	# }}}
	@delayed
	def queue_probe(self, id, names, angle = 0, speed = 3): # {{{
		'''Run one or more new jobs after probing the affected area.
		'''
		if len(self.jobs_active) > 0 and not self.paused:
			log('ignoring probe request while print is in progress: %s' % repr(self.jobs_active) + str(self.paused))
			if id is not None:
				self._send(id, 'return', None)
			return
		bbox = [float('nan'), float('nan'), float('nan'), float('nan')]
		for n in names:
			bb = self.jobqueue[n]
			if not bbox[0] < bb[0]:
				bbox[0] = bb[0]
			if not bbox[1] < bb[2]:
				bbox[1] = bb[2]
			if not bbox[2] > bb[1]:
				bbox[2] = bb[1]
			if not bbox[3] > bb[3]:
				bbox[3] = bb[3]
		self.probe((bbox[0], bbox[1], bbox[2] - bbox[0], bbox[3] - bbox[1]), angle, speed)[1](None)
		# Pass probemap to make sure it doesn't get overwritten.
		self.queue_print(names, angle, self.probemap)[1](id)
	# }}}
	def get_print_state(self): # {{{
		'''Return current print state.
		Return value is a tuple of a human readable string describing
		the state, NaN or the elapsed time, NaN or the total time for
		the current job.
		Note that the times are computed from the requested speeds.
		These are generally too low, because they don't account for
		acceleration and velocity limits.
		'''
		if self.paused:
			state = 'Paused'
		elif self.gcode_map is not None or self.gcode_file:
			state = 'Printing'
		else:
			return 'Idle', float('nan'), float('nan')
		if self.gcode_map:
			f = (self.probe_time_dist[0] + (0 if len(self.spaces) > 0 else self.probe_time_dist[1] / self.max_v)) / self.feedrate
		else:
			self._send_packet(struct.pack('=B', protocol.command['GETTIME']))
			cmd, s, m, f, e, data = self._get_reply()
			if cmd != protocol.rcommand['TIME']:
				log('invalid reply to gettime command')
				return 'Error', float('nan'), float('nan')
		return state, f, (self.total_time[0] + (0 if len(self.spaces) < 1 else self.total_time[1] / self.max_v)) / self.feedrate
	# }}}
	def send_printer(self, target): # {{{
		'''Return all settings about a machine.
		'''
		self.initialized = True
		self._broadcast(target, 'new_printer', [self.uuid, self.queue_length, [(x[0], x[1:].decode('utf-8')) for x in self.pin_names]])
		self._globals_update(target)
		for i, s in enumerate(self.spaces):
			self._space_update(i, target)
		for i, t in enumerate(self.temps):
			self._temp_update(i, target)
		for i, g in enumerate(self.gpios):
			self._gpio_update(i, target)
		self._broadcast(target, 'queue', [(q, self.jobqueue[q]) for q in self.jobqueue])
		self._broadcast(target, 'audioqueue', tuple(self.audioqueue.keys()))
		if self.confirmer is not None:
			self._broadcast(target, 'confirm', self.confirm_id, self.confirm_message)
	# }}}
	def admin_disconnect(self):
		self._send_packet(struct.pack('=B', protocol.command['FORCE_DISCONNECT']))
		self._close(False)
	# }}}
	# Accessor functions. {{{
	# Globals. {{{
	def get_globals(self): # {{{
		ret = {'num_temps': len(self.temps), 'num_gpios': len(self.gpios)}
		for key in ('uuid', 'queue_length', 'num_pins', 'led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'probe_dist', 'probe_safe_dist', 'bed_id', 'fan_id', 'spindle_id', 'unit_name', 'timeout', 'feedrate', 'targetx', 'targety', 'zoffset', 'store_adc', 'temp_scale_min', 'temp_scale_max', 'paused', 'park_after_print', 'sleep_after_print', 'cool_after_print', 'spi_setup', 'max_deviation', 'max_v'):
			ret[key] = getattr(self, key)
		return ret
	# }}}
	def expert_set_globals(self, update = True, **ka): # {{{
		#log('setting variables with %s' % repr(ka))
		nt = ka.pop('num_temps') if 'num_temps' in ka else None
		ng = ka.pop('num_gpios') if 'num_gpios' in ka else None
		if 'store_adc' in ka:
			self.store_adc = bool(ka.pop('store_adc'))
		if 'unit_name' in ka:
			self.unit_name = ka.pop('unit_name')
		if 'spi_setup' in ka:
			self.spi_setup = self._unmangle_spi(ka.pop('spi_setup'))
			if self.spi_setup:
				self._spi_send(self.spi_setup)
		for key in ('led_pin', 'stop_pin', 'probe_pin', 'spiss_pin', 'bed_id', 'fan_id', 'spindle_id', 'park_after_print', 'sleep_after_print', 'cool_after_print', 'timeout'):
			if key in ka:
				setattr(self, key, int(ka.pop(key)))
		for key in ('probe_dist', 'probe_safe_dist', 'feedrate', 'targetx', 'targety', 'zoffset', 'temp_scale_min', 'temp_scale_max', 'max_deviation', 'max_v'):
			if key in ka:
				setattr(self, key, float(ka.pop(key)))
		self._write_globals(nt, ng, update = update)
		assert len(ka) == 0
	# }}}
	def set_globals(self, update = True, **ka): # {{{
		real_ka = {}
		for key in ('feedrate', 'targetx', 'targety', 'zoffset'):
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
	def set_axis_pos(self, space, axis, pos): # {{{
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
			for key in ('park', 'park_order', 'min', 'max'):
				ret[key] = self.spaces[space].axis[axis][key]
		return ret
	# }}}
	def get_motor(self, space, motor): # {{{
		ret = {'name': self.spaces[space].motor_name(motor)}
		if space != 1:
			ret['home_pos'] = self.spaces[space].motor[motor]['home_pos']
		for key in ('step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'steps_per_unit', 'limit_v', 'limit_a', 'home_order'):
			ret[key] = self.spaces[space].motor[motor][key]
		return ret
	# }}}
	def expert_set_space(self, space, readback = True, update = True, **ka): # {{{
		if space == 0 and 'type' in ka:
			self.spaces[space].type = int(ka.pop('type'))
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
				assert len(ff) == 0
		if self.spaces[space].type in (TYPE_CARTESIAN, TYPE_EXTRUDER, TYPE_FOLLOWER):
			if 'num_axes' in ka:
				num_axes = int(ka.pop('num_axes'))
			else:
				num_axes = len(self.spaces[space].axis)
			num_motors = num_axes
		elif self.spaces[space].type == TYPE_DELTA:
			num_axes = 3;
			num_motors = 3;
			if 'delta' in ka:
				d = ka.pop('delta')
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
			num_axes = 3;
			num_motors = 3;
			if 'polar_max_r' in ka:
				self.spaces[space].polar_max_r = ka.pop('polar_max_r')
		self._send_packet(struct.pack('=BB', protocol.command['WRITE_SPACE_INFO'], space) + self.spaces[space].write_info(num_axes))
		if readback:
			self.spaces[space].read(self._read('SPACE', space))
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
			for key in ('park', 'park_order', 'min', 'max'):
				if key in ka:
					self.spaces[space].axis[axis][key] = ka.pop(key)
		if space == 1 and 'multiplier' in ka and axis < len(self.spaces[space].motor):
			assert(ka['multiplier'] > 0)
			self.multipliers[axis] = ka.pop('multiplier')
			self.expert_set_motor((space, axis), readback, update)
		self._send_packet(struct.pack('=BBB', protocol.command['WRITE_SPACE_AXIS'], space, axis) + self.spaces[space].write_axis(axis))
		if readback:
			self.spaces[space].read(self._read('SPACE', space))
			if update:
				self._space_update(space)
		assert len(ka) == 0
	# }}}
	def expert_set_motor(self, spacemotor, readback = True, update = True, **ka): # {{{
		space, motor = spacemotor
		for key in ('step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'steps_per_unit', 'limit_v', 'limit_a', 'home_order'):
			if key in ka:
				self.spaces[space].motor[motor][key] = ka.pop(key)
		if space != 1 and 'home_pos' in ka:
			self.spaces[space].motor[motor]['home_pos'] = ka.pop('home_pos')
		self._send_packet(struct.pack('=BBB', protocol.command['WRITE_SPACE_MOTOR'], space, motor) + self.spaces[space].write_motor(motor))
		if readback:
			self.spaces[space].read(self._read('SPACE', space))
			if update:
				self._space_update(space)
		assert len(ka) == 0
	# }}}
	# }}}
	# Temp {{{
	def get_temp(self, temp): # {{{
		ret = {}
		for key in ('name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit', 'fan_limit'):
			ret[key] = getattr(self.temps[temp], key)
		return ret
	# }}}
	def expert_set_temp(self, temp, update = True, **ka): # {{{
		ret = {}
		for key in ('name', 'R0', 'R1', 'Rc', 'Tc', 'beta', 'heater_pin', 'fan_pin', 'thermistor_pin', 'fan_temp', 'fan_duty', 'heater_limit', 'fan_limit'):
			if key in ka:
				setattr(self.temps[temp], key, ka.pop(key))
		self._send_packet(struct.pack('=BB', protocol.command['WRITE_TEMP'], temp) + self.temps[temp].write())
		self.temps[temp].read(self._read('TEMP', temp))
		if update:
			self._temp_update(temp)
		if len(ka) != 0:
			log('problem: %s' % repr(ka))
		assert len(ka) == 0
	# }}}
	def set_temp(self, temp, update = True, **ka): # {{{
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
		self._send_packet(struct.pack('=BB', protocol.command['WRITE_GPIO'], gpio) + self.gpios[gpio].write())
		self.gpios[gpio].read(self._read('GPIO', gpio))
		if update:
			self._gpio_update(gpio)
		assert len(ka) == 0
	# }}}
	def set_gpio(self, gpio, update = True, **ka): # {{{
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
printer = Printer(config['port'], config['run-id'], config['allow-system'])
if printer.printer is None:
	sys.exit(0)

while True: # {{{
	while len(call_queue) > 0:
		f, a = call_queue.pop(0)
		#log('calling %s' % repr((f, a)))
		f(*a)
	while printer.printer.available():
		printer._printer_input()
	if len(call_queue) > 0:
		continue	# Handle this first.
	fds = [sys.stdin, printer.printer]
	#log('waiting; movewait = %d' % printer.movewait)
	found = select.select(fds, [], fds, None)
	if sys.stdin in found[0] or sys.stdin in found[2]:
		#log('command')
		printer._command_input()
	if printer.printer in found[0] or printer.printer in found[2]:
		#log('printer')
		printer._printer_input()
# }}}

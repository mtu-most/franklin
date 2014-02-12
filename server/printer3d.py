# vim: set foldmethod=marker :

show_own_debug = False
show_firmware_debug = True

# Imports.  {{{
import glib
import websockets
from websockets import log
import serial
import time
import math
import struct
import os
import re
import sys
import wave
import sys
import StringIO
import base64
# }}}

def dprint (x, data): # {{{
	if show_own_debug:
		log ('%s: %s' % (x, ' '.join (['%02x' % ord (c) for c in data])))
# }}}

class Printer: # {{{
	# Internal stuff.  {{{
	# Constants.  {{{
	# Serial timeout for normal communication in seconds.
	default_timeout = 5	# TODO: this is for debugging; really should be .005
	long_timeout = 3	# Timeout when waiting for the printer to boot.  May need 15 for non-optiboot.
	# Masks for computing checksums.
	mask = [	[0xc0, 0xc3, 0xff, 0x09],
			[0x38, 0x3a, 0x7e, 0x13],
			[0x26, 0xb5, 0xb9, 0x23],
			[0x95, 0x6c, 0xd5, 0x43],
			[0x4b, 0xdc, 0xe2, 0x83]]
	# Single-byte commands.
	single = { 'NACK': '\x80', 'ACK': '\xe1', 'ACKWAIT': '\xd2', 'STALL': '\xb3', 'UNUSED1': '\xf4', 'INIT': '\x95', 'UNUSED2': '\xa6', 'DEBUG': '\xc7' }
	command = {
			'BEGIN': 0x00,
			'GOTO': 0x01,
			'GOTOCB': 0x02,
			'RUN': 0x03,
			'SLEEP': 0x04,
			'SETTEMP': 0x05,
			'WAITTEMP': 0x06,
			'READTEMP': 0x07,
			'SETPOS': 0x08,
			'GETPOS': 0x09,
			'LOAD': 0x0a,
			'SAVE': 0x0b,
			'READ': 0x0c,
			'WRITE': 0x0d,
			'PAUSE': 0x0e,
			'PING': 0x0f,
			'READPIN': 0x10,
			'AUDIO_SETUP': 0x11,
			'AUDIO_DATA': 0x12}
	rcommand = {
			'START': '\x13',
			'TEMP': '\x14',
			'POS': '\x15',
			'DATA': '\x16',
			'PONG': '\x17',
			'PIN': '\x18',
			'MOVECB': '\x19',
			'TEMPCB': '\x1a',
			'CONTINUE': '\x1b',
			'LIMIT': '\x1c',
			'SENSE': '\x1d'}
	# }}}
	def _set_waiter (self, type, resumeinfo, condition = True): # {{{
		#log ('adding waiter for ' + type)
		item = (resumeinfo[0], condition)
		self.waiters[type].append (item)
		return lambda: self.waiters.remove (item) if item in self.waiters else None
	# }}}
	def _trigger (self, type, arg = None, all = True): # {{{
		#log ('triggering ' + type)
		if all:
			waiters = self.waiters[type]
			self.waiters[type] = []
			for w in waiters:
				if w[1] is True or w[1] (arg):
					w[0] (arg)
				else:
					self.waiters[type].append (w)
		else:
			for i, w in enumerate (self.waiters[type]):
				if w[1] is True or w[1] (arg):
					break
			else:
				#log ('no triggers for %s' % type)
				return
			self.waiters[type].pop (i)[0] (arg)
	# }}}
	def _init (self, port): # {{{
		resumeinfo = [(yield), None]
		self.debug_buffer = None
		self.buffer = ''
		self.port = port
		self.audiodir = '.'
		self.audiofile = None
		self.pos = []
		# Set up state.
		self.ff_in = False
		self.ff_out = False
		self.limits = {}
		self.sense = {}
		self.wait = False
		self.waitaudio = False
		self.movewait = 0
		self.tempwait = set ()
		self.waiters = {'send': [], 'ack': [], 'queue': [], 'move': [], 'audio': [], 'limit': [], 'temp': [], 'reply': [], 'init': [], 'sense': []}
		self.printer = serial.Serial (port, baudrate = 115200, timeout = 0)
		self.printer.readall ()	# flush buffer.
		self.last_time = float ('nan')
		glib.io_add_watch (self.printer.fd, glib.IO_IN, self._printer_input)
		# Reset printer.
		self.printer.setDTR (False)
		glib.timeout_add (100, lambda: resumeinfo[0] () is None and False)	# Make sure the lambda function returns False.
		yield websockets.WAIT
		self.printer.setDTR (True)
		# Wait for firmware to boot.
		self._set_waiter ('init', resumeinfo)
		yield websockets.WAIT
		c = websockets.call (resumeinfo, self._begin)
		while c (): c.args = (yield websockets.WAIT)
		c = websockets.call (resumeinfo, self._read, 0)
		while c (): c.args = (yield websockets.WAIT)
		self.namelen, self.maxaxes, self.maxextruders, self.maxtemps, self.audio_fragments, self.audio_fragment_size, self.num_pins, self.num_digital_pins = struct.unpack ('<BBBBBBBB', c.ret ())
		c = websockets.call (resumeinfo, self._read_variables)
		while c (): c.args = (yield websockets.WAIT)
		self.axis = [Printer.Axis (self, t) for t in range (self.maxaxes)]
		for a in range (self.maxaxes):
			c = websockets.call (resumeinfo, self._read, 2 + a)
			while c (): c.args = (yield websockets.WAIT)
			self.axis[a].read (c.ret ())
		self.extruder = [Printer.Extruder () for t in range (self.maxextruders)]
		for e in range (self.maxextruders):
			c = websockets.call (resumeinfo, self._read, 2 + self.maxaxes + e)
			while c (): c.args = (yield websockets.WAIT)
			self.extruder[e].read (c.ret ())
		self.temp = [Printer.Temp () for t in range (self.maxtemps)]
		for t in range (self.maxtemps):
			c = websockets.call (resumeinfo, self._read, 2 + self.maxaxes + self.maxextruders + t)
			while c (): c.args = (yield websockets.WAIT)
			self.temp[t].read (c.ret ())
		global show_own_debug
		if show_own_debug is None:
			show_own_debug = True
	# }}}
	def _printer_input (self, who, what): # {{{	
		if time.time () - self.last_time > .1:
			dprint ('writing (5)', self.single['NACK']);
			self.printer.write (self.single['NACK'])
			self.last_time = float ('nan')
			self.buffer = ''
			if self.debug_buffer is not None:
				if show_firmware_debug:
					log ('Debug (partial): %s' % self.debug_buffer)
				self.debug_buffer = None
		while True:
			while self.debug_buffer is not None:
				r = self.printer.read (1)
				if r == '':
					self.last_time = time.time ()
					return True
				if r == '\x00':
					if show_firmware_debug:
						log ('Debug: %s' % self.debug_buffer)
					self.debug_buffer = None
				else:
					self.debug_buffer += r
			if len (self.buffer) == 0:
				r = self.printer.read (1)
				if r != self.single['DEBUG']:
					dprint ('(1) read', r)
				if r == '':
					self.last_time = float ('nan')
					return True
				if r == self.single['DEBUG']:
					self.debug_buffer = ''
					continue
				if r == self.single['ACK']:
					# Ack is special in that if it isn't expected, it is not an error.
					self._trigger ('ack', 'ack', False)
					continue
				if r == self.single['NACK']:
					# Nack is special in that it should only be handled if an ack is expected.
					self._trigger ('ack', 'nack', False)
					continue
				if r == self.single['ACKWAIT']:
					self._trigger ('ack', 'wait', False)
					continue
				if r == self.single['INIT']:
					self._trigger ('init')
					continue
				if r == self.single['STALL']:
					# There is a problem we can't solve; kill the generator that registered to see the ack.
					self._trigger ('ack', 'stall', False)
				if (ord (r) & 0x80) != 0 or ord (r) in (0, 1, 2, 4):
					dprint ('writing (1)', self.single['NACK']);
					self.printer.write (self.single['NACK'])
					continue
				# Regular packet.
				self.buffer = r
			packet_len = ord (self.buffer[0]) + (ord (self.buffer[0]) + 2) / 3
			r = self.printer.read (packet_len - len (self.buffer))
			dprint ('rest of packet read', r)
			self.buffer += r
			if len (self.buffer) < packet_len:
				if len (r) == 0:
					self.last_time = time.time ()
				else:
					self.last_time = float ('nan')
				return True
			packet = self._parse_packet (self.buffer)
			self.buffer = ''
			self.last_time = float ('nan')
			if packet is None:
				dprint ('writing (4)', self.single['NACK']);
				self.printer.write (self.single['NACK'])
				continue	# Start over.
			if bool (ord (packet[0]) & 0x80) != self.ff_in:
				# This was a retry of the previous packet; accept it and retry.
				dprint ('(2) writing', self.single['ACK']);
				self.printer.write (self.single['ACK'])
				continue	# Start over.
			# Packet successfully received.
			dprint ('(3) writing', self.single['ACK']);
			self.printer.write (self.single['ACK'])
			# Clear the flip flop.
			packet = chr (ord (packet[0]) & ~0x80) + packet[1:]
			# Flip it.
			self.ff_in = not self.ff_in
			# Handle the asynchronous events here; don't bother the caller with them.
			if packet[0] == self.rcommand['MOVECB']:
				num = ord (packet[1])
				#log ('movecb %d/%d' % (num, self.movewait))
				assert self.movewait >= num
				self.movewait -= num
				if self.movewait == 0:
					self._trigger ('move')
				continue
			elif packet[0] == self.rcommand['TEMPCB']:
				t = ord (packet[1])
				assert ord (t) in self.tempwait
				self.tempwait.remove (t)
				self._trigger ('temp', t)
			elif packet[0] == self.rcommand['CONTINUE']:
				if packet[1] == '\x00':
					# Move continue.
					self.wait = False
					self._trigger ('queue', None, False)
				else:
					# Audio continue.
					self.waitaudio = False
					self._trigger ('audio', None, False)
			elif packet[0] == self.rcommand['LIMIT']:
				which = ord (packet[1])
				self.limits[which] = struct.unpack ('<f', packet[2:])[0]
				if self.waiters['limit'] is not None:
					self._trigger ('limit', which)
			elif packet[0] == self.rcommand['SENSE']:
				w = ord (packet[1])
				which = w & 0x7f
				state = bool (w & 0x80)
				pos = struct.unpack ('<f', packet[2:])[0]
				if which not in self.sense:
					self.sense[which] = []
				self.sense[which].append ((state, pos))
				self._trigger ('sense', which)
			else:
				self._trigger ('reply', packet)
	# }}}
	def _make_packet (self, data): # {{{
		data = chr (len (data) + 1) + data
		for t in range ((len (data) + 2) / 3):
			check = t & 0x7
			for bit in range (5):
				sum = check & Printer.mask[bit][3]
				for byte in range (3):
					sum ^= ord (data[3 * t + byte]) & Printer.mask[bit][byte]
				sum ^= sum >> 4
				sum ^= sum >> 2
				sum ^= sum >> 1
				if sum & 1:
					check |= 1 << (bit + 3)
			data += chr (check)
		#log (' '.join (['%02x' % ord (x) for x in data]))
		return data
	# }}}
	def _parse_packet (self, data): # {{{
		#log (' '.join (['%02x' % ord (x) for x in data]))
		if ord (data[0]) + (ord (data[0]) + 2) / 3 != len (data):
			return None
		length = ord (data[0])
		checksum = data[length:]
		for t in range (len (checksum)):
			r = data[3 * t:3 * t + 3] + checksum[t]
			if (ord (checksum[t]) & 0x7) != (t & 7):
				return None
			for bit in range (5):
				sum = 0
				for byte in range (4):
					sum ^= ord (r[byte]) & Printer.mask[bit][byte]
				sum ^= sum >> 4
				sum ^= sum >> 2
				sum ^= sum >> 1
				if (sum & 1) != 0:
					return None
		return data[1:length]
	# }}}
	def _send_packet (self, data, audio = False): # {{{
		resumeinfo = [(yield), None]
		if len (self.waiters['send']) > 0:
			self.waiters.send.append (resumeinfo[0])
			yield websockets.WAIT
		if self.ff_out:
			data = chr (ord (data[0]) | 0x80) + data[1:]
		self.ff_out = not self.ff_out
		data = self._make_packet (data)
		try:
			while True:
				dprint ('(1) writing', data);
				self.printer.write (data)
				self._set_waiter ('ack', resumeinfo)
				ack = yield websockets.WAIT
				if ack == 'nack':
					continue
				elif ack == 'ack':
					break
				elif ack == 'wait':
					if audio:
						self.waitaudio = True
					else:
						self.wait = True
					break
				else:
					# ack == 'stall'
					raise ValueError ('Printer sent stall')
		finally:
			self._trigger ('send', None, False)
	# }}}
	def _begin (self): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BL', self.command['BEGIN'], 0))
		while c (): c.args = (yield websockets.WAIT)
		self._set_waiter ('reply', resumeinfo)
		reply = yield websockets.WAIT
		assert struct.unpack ('<BL', reply) == (ord (self.rcommand['START']), 0)
	# }}}
	def _read (self, channel): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['READ'], channel))
		while c (): c.args = (yield websockets.WAIT)
		self._set_waiter ('reply', resumeinfo)
		reply = yield websockets.WAIT
		assert reply[0] == self.rcommand['DATA']
		yield reply[1:]
	# }}}
	def _read_variables (self): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._read, 1)
		while c (): c.args = (yield websockets.WAIT)
		data = c.ret ()
		self.name = data[:self.namelen].rstrip ('\0')
		self.num_axes, self.num_extruders, self.num_temps, self.printer_type, self.led_pin, self.room_T, self.motor_limit, self.temp_limit, self.feedrate = struct.unpack ('<BBBBHfLLf', data[self.namelen:])
		self.pos = (self.pos + [float ('nan')] * self.num_axes)[:self.num_axes]
	# }}}
	def _write_variables (self): # {{{
		resumeinfo = [(yield), None]
		data = (self.name + chr (0) * self.namelen)[:self.namelen] + struct.pack ('<BBBBHfLLf', self.num_axes, self.num_extruders, self.num_temps, self.printer_type, self.led_pin, self.room_T, self.motor_limit, self.temp_limit, self.feedrate)
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['WRITE'], 1) + data)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def _write_axis (self, which): # {{{
		resumeinfo = [(yield), None]
		data = self.axis[which].write ()
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['WRITE'], 2 + which) + data)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def _write_extruder (self, which): # {{{
		resumeinfo = [(yield), None]
		data = self.extruder[which].write ()
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['WRITE'], 2 + self.maxaxes + which) + data)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def _write_temp (self, which): # {{{
		resumeinfo = [(yield), None]
		data = self.temp[which].write ()
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['WRITE'], 2 + self.maxaxes + self.maxextruders + which) + data)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def _send_audio (self): # {{{
		resumeinfo = [(yield), None]
		while not self.waitaudio and self.audiofile is not None:
			data = self.audiofile.read (self.audio_fragment_size)
			if len (data) < self.audio_fragment_size:
				self.audiofile = None
				return
			c = websockets.call (resumeinfo, self._send_packet, chr (self.command['AUDIO_DATA']) + data, audio = True)
			while c (): c.args = (yield websockets.WAIT)
	# }}}
	# Subclasses.  {{{
	class Temp: # {{{
		def read (self, data):
			self.alpha, self.beta, self.core_C, self.shell_C, self.transfer, self.radiation, self.power, self.power_pin, self.thermistor_pin = struct.unpack ('<fffffffHH', data[:32])
			return data[32:]
		def write (self):
			return struct.pack ('<fffffffHH', self.alpha, self.beta, self.core_C, self.shell_C, self.transfer, self.radiation, self.power, self.power_pin, self.thermistor_pin)
	# }}}
	class Motor: # {{{
		def read (self, data):
			self.step_pin, self.dir_pin, self.enable_pin, self.steps_per_mm, self.max_v_neg, self.max_v_pos, self.max_a = struct.unpack ('<HHHffff', data[:22])
			return data[22:]
		def write (self):
			return struct.pack ('<HHHffff', self.step_pin, self.dir_pin, self.enable_pin, self.steps_per_mm, self.max_v_neg, self.max_v_pos, self.max_a)
	# }}}
	class Axis: # {{{
		def __init__ (self, printer, id):
			self.printer = printer
			self.id = id
			self.motor = Printer.Motor ()
		def read (self, data):
			data = self.motor.read (data)
			self.limit_min_pin, self.limit_max_pin, self.sense_pin, self.limit_min_pos, self.limit_max_pos, self.delta_length, self.delta_radius, self.offset = struct.unpack ('<HHHfffff', data)
		def write (self):
			return self.motor.write () + struct.pack ('<HHHfffff', self.limit_min_pin, self.limit_max_pin, self.sense_pin, self.limit_min_pos, self.limit_max_pos, self.delta_length, self.delta_radius, self.offset)
		def set_current_pos (self, pos):
			resumeinfo = [(yield), None]
			c = websockets.call (resumeinfo, self.printer._send_packet, struct.pack ('<BBf', self.printer.command['SETPOS'], 2 + self.id, pos))
			while c (): c.args = (yield websockets.WAIT)
		def get_current_pos (self):
			resumeinfo = [(yield), None]
			c = websockets.call (resumeinfo, self.printer._send_packet, struct.pack ('<BB', self.printer.command['GETPOS'], 2 + self.id))
			while c (): c.args = (yield websockets.WAIT)
			self.printer._set_waiter ('reply', resumeinfo)
			ret = yield websockets.WAIT
			assert ret[0] == self.printer.rcommand['POS']
			yield struct.unpack ('<ff', ret[1:])
	# }}}
	class Extruder: # {{{
		def __init__ (self):
			self.motor = Printer.Motor ()
			self.temp = Printer.Temp ()
		def read (self, data):
			data = self.motor.read (data)
			data = self.temp.read (data)
			self.filament_heat, self.nozzle_size, self.filament_size = struct.unpack ('<fff', data)
		def write (self):
			return self.motor.write () + self.temp.write () + struct.pack ('<fff', self.filament_heat, self.nozzle_size, self.filament_size)
	# }}}
	# }}}
	# }}}
	# Useful commands.  {{{
	def goto (self, axes = {}, e = None, f0 = None, f1 = None, which = 0, cb = False): # {{{
		resumeinfo = [(yield), None]
		if isinstance (axes, (list, tuple)):
			a = {}
			for i, axis in enumerate (axes):
				a[i] = axis
			axes = a
		else:
			a = {}
			for i, axis in axes.items ():
				a[int (i)] = axis
			axes = a
		targets = [0] * (((2 + self.num_axes + self.num_extruders - 1) >> 3) + 1)
		args = ''
		if f0 is None:
			f0 = float ('inf')
		if f1 is None:
			f1 = f0
		targets[0] |= 1 << 0
		targets[0] |= 1 << 1
		args += struct.pack ('<f', f0)
		args += struct.pack ('<f', f1)
		a = axes.keys ()
		a.sort ()
		for axis in a:
			assert axis < self.num_axes
			if math.isnan (axes[axis]):
				continue
			self.pos[axis] = axes[axis]
			targets[(axis + 2) >> 3] |= 1 << ((axis + 2) & 0x7)
			args += struct.pack ('<f', axes[axis])
		if e is not None:
			targets[(2 + self.num_axes + which) >> 3] |= 1 << ((2 + self.num_axes + which) & 0x7)
			args += struct.pack ('<f', e)
		while self.wait:
			self._set_waiter ('queue', resumeinfo)
			yield websockets.WAIT
		if cb:
			self.movewait += 1
			#log ('movewait +1 -> %d' % self.movewait)
			p = chr (self.command['GOTOCB'])
		else:
			p = chr (self.command['GOTO'])
		c = websockets.call (resumeinfo, self._send_packet, p + ''.join ([chr (t) for t in targets]) + args)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def run_axis (self, which, speed): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.run, 2 + which, speed)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def run_extruder (self, which, speed): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.run, 2 + self.maxaxes + which, speed)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def run (self, channel, speed):	# speed: float; 0 means off. # {{{
		resumeinfo = [(yield), None]
		if channel >= 2 and channel < 2 + self.num_axes and not math.isnan (speed):
			self.pos[channel - 2] = float ('nan')
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BBf', self.command['RUN'], channel, speed))
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def sleep_axis (self, which, sleeping = True): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.sleep, 2 + which, sleeping)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def sleep_extruder (self, which, sleeping = True): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.sleep, 2 + self.maxaxes + which, sleeping)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def sleep (self, channel, sleeping = True): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['SLEEP'], (channel & 0x7f) | (0x80 if sleeping else 0)))
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def settemp_extruder (self, which, temp):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.settemp ,2 + self.maxaxes + which, temp)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def settemp_temp (self, which, temp):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.settemp, 2 + self.maxaxes + self.maxextruders + which, temp)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def settemp (self, channel, temp): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BBf', self.command['SETTEMP'], channel, temp))
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def waittemp_extruder (self, which, min, max):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.waittemp, 2 + self.maxaxes + which, min, max)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def waittemp_temp (self, which, min, max):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.waittemp, 2 + self.maxaxes + self.maxextruders + which, min, max)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def waittemp (self, channel, min, max): # {{{
		resumeinfo = [(yield), None]
		if min is None:
			min = float ('nan')
		if max is None:
			max = float ('nan')
		if math.isnan (min) and math.isnan (max):
			self.tempwait.discard (channel)
		else:
			self.tempwait.add (channel)
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BBff', self.command['WAITTEMP'], channel, min, max))
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def readtemp_extruder (self, which):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.readtemp, 2 + self.maxaxes + which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	# }}}
	def readtemp_temp (self, which):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.readtemp, 2 + self.maxaxes + self.maxextruders + which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	# }}}
	def readtemp (self, channel): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['READTEMP'], channel))
		while c (): c.args = (yield websockets.WAIT)
		self._set_waiter ('reply', resumeinfo)
		ret = yield websockets.WAIT
		assert ret[0] == self.rcommand['TEMP']
		yield struct.unpack ('<f', ret[1:])[0]
	# }}}
	def readpin (self, pin): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['READPIN'], pin))
		while c (): c.args = (yield websockets.WAIT)
		self._set_waiter ('reply', resumeinfo)
		ret = yield websockets.WAIT
		assert ret[0] == self.rcommand['PIN']
		yield bool (ord (ret[1]))
	# }}}
	def load_variables (self): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.load, 1)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def load_axis (self, which): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.load, 2 + which)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def load_extruder (self, which): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.load, 2 + self.maxaxes + which)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def load_temp (self, which): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.load, 2 + self.maxaxes + self.maxextruders + which)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def load (self, channel): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['LOAD'], channel))
		while c (): c.args = (yield websockets.WAIT)
		if channel == 1:
			c = websockets.call (resumeinfo, self._read_variables, )
			while c (): c.args = (yield websockets.WAIT)
		elif 2 <= channel < 2 + self.maxaxes:
			c = websockets.call (resumeinfo, self.axis[channel - 2].read, self._read (channel))
			while c (): c.args = (yield websockets.WAIT)
		elif 2 + self.maxaxes <= channel < 2 + self.maxaxes + self.maxextruders:
			c = websockets.call (resumeinfo, self.extruder[channel - 2 - self.maxaxes].read, self._read (channel))
			while c (): c.args = (yield websockets.WAIT)
		else:
			assert channel < 2 + self.maxaxes + self.maxextruders + self.maxtemps
			c = websockets.call (resumeinfo, self.temp[channel - 2 - self.maxaxes - self.maxextruders].read, self._read (channel))
			while c (): c.args = (yield websockets.WAIT)
	# }}}
	def load_all (self): # {{{
		resumeinfo = [(yield), None]
		for i in range (1, 2 + self.maxaxes + self.maxextruders + self.maxtemps):
			log ('loading %d' % i)
			c = websockets.call (resumeinfo, self.load, i)
			while c (): c.args = (yield websockets.WAIT)
	# }}}
	def save_variables (self): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.save, 1)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def save_axis (self, which): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.save, 2 + which)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def save_extruder (self, which): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.save, 2 + self.maxaxes + which)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def save_temp (self, which): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.save, 2 + self.maxaxes + self.maxextruders + which)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def save (self, channel): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['SAVE'], channel))
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def save_all (self): # {{{
		resumeinfo = [(yield), None]
		for i in range (1, 2 + self.maxaxes + self.maxextruders + self.maxtemps):
			log ('saving %d' % i)
			c = websockets.call (resumeinfo, self.save, i)
			while c (): c.args = (yield websockets.WAIT)
	# }}}
	def pause (self, pausing = True): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['PAUSE'], pausing))
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def ping (self, arg = 0): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['PING'], arg))
		while c (): c.args = (yield websockets.WAIT)
		self._set_waiter ('reply', resumeinfo)
		reply = yield websockets.WAIT
		assert struct.unpack ('<BB', reply) == (ord (self.rcommand['PONG']), arg)
	# }}}
	def home (self, axes = (0, 1, 2), speed = 5): # {{{
		resumeinfo = [(yield), None]
		if self.printer_type != 1:
			# Find the switches.
			self.limits.clear ()
			for axis in axes:
				s = 3200. / self.axis[axis].motor.steps_per_mm
				c = websockets.call (resumeinfo, self.run_axis, axis, -s if self.pin_valid (self.axis[axis].limit_min_pin) else s)
				while c (): c.args = (yield websockets.WAIT)
			c = websockets.call (resumeinfo, self.wait_for_limits, len (axes))
			while c (): c.args = (yield websockets.WAIT)
			# Back off a bit.
			axesmove = {}
			for axis in axes:
				c = websockets.call (resumeinfo, self.axis[axis].set_current_pos, 0)
				while c (): c.args = (yield websockets.WAIT)
				axesmove[axis] = 3 if self.pin_valid (self.axis[axis].limit_min_pin) else -3
			c = websockets.call (resumeinfo, self.goto, axes = axesmove, cb = True)
			while c (): c.args = (yield websockets.WAIT)
			c = websockets.call (resumeinfo, self.wait_for_cb, )
			while c (): c.args = (yield websockets.WAIT)
			# Move there again, but slowly.
			self.limits.clear ()
			for axis in axes:
				c = websockets.call (resumeinfo, self.run_axis, axis, -1 if self.pin_valid (self.axis[axis].limit_min_pin) else 1)
				while c (): c.args = (yield websockets.WAIT)
			c = websockets.call (resumeinfo, self.wait_for_limits, len (axes))
			while c (): c.args = (yield websockets.WAIT)
			# Current position is 0.
			ret = [self.axis[axis].limit_min_pos if self.pin_valid (self.axis[axis].limit_min_pin) else self.axis[axis].limit_max_pos for axis in axes]
			for i, axis in enumerate (axes):
				c = websockets.call (resumeinfo, self.axis[axis].set_current_pos, ret[i])
				while c (): c.args = (yield websockets.WAIT)
			for i, a in enumerate (axes):
				self.pos[a] = ret[i] / self.axis[a].motor.steps_per_mm
			return
		else:
			# Compute home position close to limit switches.
			if self.pin_valid (self.axis[0].limit_max_pin):
				pos = (0, 0, min ([self.axis[i].limit_max_pos for i in range (3)]) + self.axis[i].offset - 10)
			elif self.pin_valid (self.axis[0].limit_min_pin):
				pos = [0, 0, max ([self.axis[i].limit_min_pos for i in range (3)]) + self.axis[i].offset + 10]
			else:
				try:
					c = websockets.call (resumeinfo, self.set_printer_type, 0)
					while c (): c.args = (yield websockets.WAIT)
					for a in range (3):
						c = websockets.call (resumeinfo, self.axis[a].set_current_pos, 0)
						while c (): c.args = (yield websockets.WAIT)
					self.clear_sense ()
					dist = 50
					sense = {}
					while len (sense) < 3:
						c = websockets.call (resumeinfo, self.goto, [dist if a not in sense else sense[a] for a in range (3)], cb = True)
						while c (): c.args = (yield websockets.WAIT)
						c = websockets.call (resumeinfo, self.wait_for_cb, True)
						while c (): c.args = (yield websockets.WAIT)
						if not c.ret ():
							c = websockets.call (resumeinfo, self.pause, True)
							while c (): c.args = (yield websockets.WAIT)
							for a in self.sense:
								if a not in sense:
									sense[a] = self.sense[a][0][1]
							self.clear_sense ()
							continue
						if len (sense) == 3:
							break
						while True:
							c = websockets.call (resumeinfo, self.goto, [-dist if a not in sense else sense[a] for a in range (3)], cb = True)
							while c (): c.args = (yield websockets.WAIT)
							c = websockets.call (resumeinfo, self.wait_for_cb, True)
							while c (): c.args = (yield websockets.WAIT)
							if not c.ret ():
								c = websockets.call (resumeinfo, self.pause, True)
								while c (): c.args = (yield websockets.WAIT)
								for a in self.sense:
									if a not in sense:
										sense[a] = self.sense[a][0][1]
								self.clear_sense ()
								continue
							break
						assert dist < 300
						dist += 30
					c = websockets.call (resumeinfo, self.goto, [sense[a] - 10 for a in range (3)], cb = True)
					while c (): c.args = (yield websockets.WAIT)
					c = websockets.call (resumeinfo, self.wait_for_cb, )
					while c (): c.args = (yield websockets.WAIT)
					for a in range (3):
						c = websockets.call (resumeinfo, self.axis[a].set_current_pos, 0)
						while c (): c.args = (yield websockets.WAIT)
						self.clear_sense ()
						c = websockets.call (resumeinfo, self.goto, {a: 20}, f0 = speed / 20., cb = True)
						while c (): c.args = (yield websockets.WAIT)
						c = websockets.call (resumeinfo, self.wait_for_cb, True)
						while c (): c.args = (yield websockets.WAIT)
						assert a in self.sense
						c = websockets.call (resumeinfo, self.pause, True)
						while c (): c.args = (yield websockets.WAIT)
						sense[a] = self.sense[a][0][1]
						c = websockets.call (resumeinfo, self.goto, {a: 0}, cb = True)
						while c (): c.args = (yield websockets.WAIT)
						c = websockets.call (resumeinfo, self.wait_for_cb, )
						while c (): c.args = (yield websockets.WAIT)
					c = websockets.call (resumeinfo, self.goto, [sense[a] for a in range (3)], cb = True)
					while c (): c.args = (yield websockets.WAIT)
					c = websockets.call (resumeinfo, self.wait_for_cb, )
					while c (): c.args = (yield websockets.WAIT)
					for a in range (3):
						c = websockets.call (resumeinfo, self.axis[a].set_current_pos, -self.axis[a].limit_min_pos)
						while c (): c.args = (yield websockets.WAIT)
					c = websockets.call (resumeinfo, self.goto, [min ([-self.axis[i].limit_min_pos for i in range (3)]) for a in range (3)], cb = True)
					while c (): c.args = (yield websockets.WAIT)
					c = websockets.call (resumeinfo, self.wait_for_cb, )
					while c (): c.args = (yield websockets.WAIT)
				finally:
					c = websockets.call (resumeinfo, self.set_printer_type, 1)
					while c (): c.args = (yield websockets.WAIT)
				return
			# Go to limit switches.
			self.limits.clear ()
			s = 1 if self.pin_valid (self.axis[0].limit_max_pin) else -1
			for i in range (3):
				c = websockets.call (resumeinfo, self.run_axis, i, s * 50)
				while c (): c.args = (yield websockets.WAIT)
			c = websockets.call (resumeinfo, self.wait_for_limits, 3)
			while c (): c.args = (yield websockets.WAIT)
			# Set positions to limit switch positions.
			for i in range (3):
				c = websockets.call (resumeinfo, self.axis[i].set_current_pos, self.axis[i].limit_max_pos if self.pin_valid (self.axis[0].limit_max_pin) else self.axis[i].limit_min_pos)
				while c (): c.args = (yield websockets.WAIT)
			# Go to home position.
			c = websockets.call (resumeinfo, self.goto, pos, cb = True)
			while c (): c.args = (yield websockets.WAIT)
			c = websockets.call (resumeinfo, self.wait_for_cb, )
			while c (): c.args = (yield websockets.WAIT)
			# Slowly go to limit switches.
			self.limits.clear ()
			for i in range (3):
				c = websockets.call (resumeinfo, self.run_axis, i, s * 10)
				while c (): c.args = (yield websockets.WAIT)
			c = websockets.call (resumeinfo, self.wait_for_limits, 3)
			while c (): c.args = (yield websockets.WAIT)
			self.limits.clear ()
			# Set positions to improved values.
			for i in range (3):
				c = websockets.call (resumeinfo, self.axis[i].set_current_pos, self.axis[i].limit_max_pos if self.pin_valid (self.axis[0].limit_max_pin) else self.axis[i].limit_min_pos)
				while c (): c.args = (yield websockets.WAIT)
			# Go to home position.
			c = websockets.call (resumeinfo, self.goto, pos, cb = True)
			while c (): c.args = (yield websockets.WAIT)
			c = websockets.call (resumeinfo, self.wait_for_cb, )
			while c (): c.args = (yield websockets.WAIT)
	# }}}
	def audio_play (self, name, axes = None, extruders = None): # {{{
		resumeinfo = [(yield), None]
		assert os.path.basename (name) == name
		self.audiofile = open (os.path.join (self.audiodir, name), 'rb')
		channels = [0] * (((2 + self.num_axes + self.num_extruders - 1) >> 3) + 1)
		for axis in range (self.num_axes):
			if axes is None or axis in axes:
				channels[(axis + 2) >> 3] |= 1 << ((axis + 2) & 0x7)
		for extruder in range (self.num_extruders):
			if extruders is None or extruder in extruders:
				channels[(2 + self.num_axes + extruder) >> 3] |= 1 << ((2 + self.num_axes + extruder) & 0x7)
		us_per_bit = self.audiofile.read (2)
		c = websockets.call (resumeinfo, self._send_packet, chr (self.command['AUDIO_SETUP']) + us_per_bit + ''.join ([chr (x) for x in channels]))
		while c (): c.args = (yield websockets.WAIT)
		c = websockets.call (resumeinfo, self._send_audio)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def audio_load (self, name, data): # {{{
		assert os.path.basename (name) == name
		wav = wave.open (StringIO.StringIO (base64.b64decode (data)))
		assert wav.getnchannels () == 1
		data = [ord (x) for x in wav.readframes (wav.getnframes ())]
		data = [(h << 8) + l if h < 128 else (h << 8) + l - (1 << 16) for l, h in zip (data[::2], data[1::2])]
		minimum = min (data)
		maximum = max (data)
		level = (minimum + maximum) / 2
		s = ''
		for t in range (0, len (data) - 7, 8):
			c = 0
			for b in range (8):
				if data[t + b] > level:
					c += 1 << b
			s += chr (c)
		if not os.path.exists (self.audiodir):
			os.makedirs (self.audiodir)
		with open (os.path.join (self.audiodir, name), 'wb') as f:
			f.write (struct.pack ('<H', 1000000 / wav.getframerate ()) + s)
	# }}}
	def audio_list (self): # {{{
		ret = []
		for x in os.listdir (self.audiodir):
			try:
				with open (os.path.join (self.audiodir, x), 'rb') as f:
					us_per_bit = struct.unpack ('<H', f.read (2))[0]
					bits = (os.fstat (f.fileno ()).st_size - 2) * 8
					t = bits * us_per_bit * 1e-6
					ret.append ((x, t))
			except:
				pass
		return ret
	# }}}
	def wait_for_cb (self, sense = False): # {{{
		resumeinfo = [(yield), None]
		if sense in self.sense:
			yield self.movewait == 0
		if sense is not False:
			rm1 = self._set_waiter ('sense', resumeinfo, lambda s: sense is True or s in sense)
		else:
			rm1 = lambda: None
		rm2 = self._set_waiter ('move', resumeinfo)
		yield websockets.WAIT
		rm1 ()
		rm2 ()
		yield self.movewait == 0
	# }}}
	def wait_for_limits (self, num = 1): # {{{
		resumeinfo = [(yield), None]
		while len (self.limits) < num:
			self._set_waiter ('limit', resumeinfo)
			yield websockets.WAIT
	# }}}
	def wait_for_temp (self, which = None): # {{{
		resumeinfo = [(yield), None]
		while len (self.tempwait) > 0:
			self._set_waiter ('temp', resumeinfo, lambda t: which is None or t in which)
			yield websockets.WAIT
	# }}}
	def is_waiting (self):	# {{{
		return self.wait or len (self.tempwait) > 0
	# }}}
	def get_limits (self, axis = None):	# {{{
		if axis is None:
			return self.limits
		if axis in self.limits:
			return self.limits[axis]
		return None
	# }}}
	def clear_limits (self):	# {{{
		self.limits.clear ()
	# }}}
	def get_sense (self, axis = None):	# {{{
		if axis is None:
			return self.sense
		if axis in self.sense:
			return self.sense[axis]
		return []
	# }}}
	def clear_sense (self):	# {{{
		self.sense.clear ()
	# }}}
	def get_position (self):	# {{{
		return self.pos
	# }}}
	def pin_valid (self, pin):	# {{{
		return (pin & 0x100) == 0
	# }}}
	# }}}
	# Accessor functions. {{{
	# Constants. {{{
	def get_namelen (self):	# {{{
		return self.namelen
	# }}}
	def get_maxaxes (self):	# {{{
		return self.maxaxes
	# }}}
	def get_audio_fragments (self):	# {{{
		return self.audio_fragments
	# }}}
	def get_audio_fragment_size (self):	# {{{
		return self.audio_fragment_size
	# }}}
	def get_maxextruders (self):	# {{{
		return self.maxextruders
	# }}}
	def get_maxtemps (self):	# {{{
		return self.maxtemps
	# }}}
	def get_num_pins (self):	# {{{
		return self.num_pins
	# }}}
	def get_num_digital_pins (self):	# {{{
		return self.num_digital_pins
	# }}}
	# }}}
	# Variables. {{{
	def set_name (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.name = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
	def get_name (self):
		return self.name
	# }}}
	def set_num_axes (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.num_axes = value
		self.pos = (self.pos + [float ('nan')] * value)[:value]
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
	def get_num_axes (self):
		return self.num_axes
	# }}}
	def set_num_extruders (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.num_extruders = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
	def get_num_extruders (self):
		return self.num_extruders
	# }}}
	def set_num_temps (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.num_temps = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
	def get_num_temps (self):
		return self.num_temps
	# }}}
	def set_printer_type (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.printer_type = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
	def get_printer_type (self):
		return self.printer_type
	# }}}
	def set_led_pin (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.led_pin = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
	def get_led_pin (self):
		return self.led_pin
	# }}}
	def set_room_T (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.room_T = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
	def get_room_T (self):
		return self.room_T
	# }}}
	def set_motor_limit (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.motor_limit = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
	def get_motor_limit (self):
		return self.motor_limit
	# }}}
	def set_temp_limit (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp_limit = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
	def get_temp_limit (self):
		return self.temp_limit
	# }}}
	def set_feedrate (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.feedrate = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
	def get_feedrate (self):
		return self.feedrate
	# }}}
	# }}}
	# Temp {{{
	def temp_set_alpha (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].alpha = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
	def temp_get_alpha (self, which):
		return self.temp[which].alpha
	# }}}
	def temp_set_beta (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].beta = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
	def temp_get_beta (self, which):
		return self.temp[which].beta
	# }}}
	def temp_set_core_C (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].core_C = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
	def temp_get_core_C (self, which):
		return self.temp[which].core_C
	# }}}
	def temp_set_shell_C (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].shell_C = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
	def temp_get_shell_C (self, which):
		return self.temp[which].shell_C
	# }}}
	def temp_set_transfer (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].transfer = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
	def temp_get_transfer (self, which):
		return self.temp[which].transfer
	# }}}
	def temp_set_radiation (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].radiation = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
	def temp_get_radiation (self, which):
		return self.temp[which].radiation
	# }}}
	def temp_set_power (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].power = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
	def temp_get_power (self, which):
		return self.temp[which].power
	# }}}
	def temp_set_power_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].power_pin = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
	def temp_get_power_pin (self, which):
		return self.temp[which].power_pin
	# }}}
	def temp_set_thermistor_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].thermistor_pin = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
	def temp_get_thermistor_pin (self, which):
		return self.temp[which].thermistor_pin
	# }}}
	# }}}
	# Axis {{{
	def axis_motor_set_step_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.step_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_motor_get_step_pin (self, which):
		return self.axis[which].motor.step_pin
	# }}}
	def axis_motor_set_dir_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.dir_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_motor_get_dir_pin (self, which):
		return self.axis[which].motor.dir_pin
	# }}}
	def axis_motor_set_enable_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.enable_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_motor_get_enable_pin (self, which):
		return self.axis[which].motor.enable_pin
	# }}}
	def axis_motor_set_steps_per_mm (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.steps_per_mm = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_motor_get_steps_per_mm (self, which):
		return self.axis[which].motor.steps_per_mm
	# }}}
	def axis_motor_set_max_v_neg (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.max_v_neg = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_motor_get_max_v_neg (self, which):
		return self.axis[which].motor.max_v_neg
	# }}}
	def axis_motor_set_max_v_pos (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.max_v_pos = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_motor_get_max_v_pos (self, which):
		return self.axis[which].motor.max_v_pos
	# }}}
	def axis_motor_set_max_a (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.max_a = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_motor_get_max_a (self, which):
		return self.axis[which].motor.max_a
	# }}}
	def axis_set_limit_min_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].limit_min_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_get_limit_min_pin (self, which):
		return self.axis[which].limit_min_pin
	# }}}
	def axis_set_limit_max_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].limit_max_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_get_limit_max_pin (self, which):
		return self.axis[which].limit_max_pin
	# }}}
	def axis_set_sense_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].sense_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_get_sense_pin (self, which):
		return self.axis[which].sense_pin
	# }}}
	def axis_set_limit_min_pos (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].limit_min_pos = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_get_limit_min_pos (self, which):
		return self.axis[which].limit_min_pos
	# }}}
	def axis_set_limit_max_pos (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].limit_max_pos = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_get_limit_max_pos (self, which):
		return self.axis[which].limit_max_pos
	# }}}
	def axis_set_delta_length (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].delta_length = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_get_delta_length (self, which):
		return self.axis[which].delta_length
	# }}}
	def axis_set_delta_radius (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].delta_radius = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_get_delta_radius (self, which):
		return self.axis[which].delta_radius
	# }}}
	def axis_set_offset (self, which, offset):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].offset = offset
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
	def axis_get_offset (self, which):
		return self.axis[which].offset
	# }}}
	def axis_set_current_pos (self, which, pos):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.axis[which].set_current_pos, pos)
		while c (): c.args = (yield websockets.WAIT)
	def axis_get_current_pos (self, which):
		return self.axis[which].get_current_pos ()
	# }}}
	# }}}
	# Extruder {{{
	def extruder_temp_set_alpha (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.alpha = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_temp_get_alpha (self, which):
		return self.extruder[which].temp.alpha
	# }}}
	def extruder_temp_set_beta (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.beta = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_temp_get_beta (self, which):
		return self.extruder[which].temp.beta
	# }}}
	def extruder_temp_set_core_C (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.core_C = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_temp_get_core_C (self, which):
		return self.extruder[which].temp.core_C
	# }}}
	def extruder_temp_set_shell_C (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.shell_C = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_temp_get_shell_C (self, which):
		return self.extruder[which].temp.shell_C
	# }}}
	def extruder_temp_set_transfer (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.transfer = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_temp_get_transfer (self, which):
		return self.extruder[which].temp.transfer
	# }}}
	def extruder_temp_set_radiation (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.radiation = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_temp_get_radiation (self, which):
		return self.extruder[which].temp.radiation
	# }}}
	def extruder_temp_set_power (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.power = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_temp_get_power (self, which):
		return self.extruder[which].temp.power
	# }}}
	def extruder_temp_set_power_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.power_pin = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_temp_get_power_pin (self, which):
		return self.extruder[which].temp.power_pin
	# }}}
	def extruder_temp_set_thermistor_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.thermistor_pin = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_temp_get_thermistor_pin (self, which):
		return self.extruder[which].temp.thermistor_pin
	# }}}
	def extruder_motor_set_step_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.step_pin = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_motor_get_step_pin (self, which):
		return self.extruder[which].motor.step_pin
	# }}}
	def extruder_motor_set_dir_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.dir_pin = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_motor_get_dir_pin (self, which):
		return self.extruder[which].motor.dir_pin
	# }}}
	def extruder_motor_set_enable_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.enable_pin = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_motor_get_enable_pin (self, which):
		return self.extruder[which].motor.enable_pin
	# }}}
	def extruder_motor_set_steps_per_mm (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.steps_per_mm = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_motor_get_steps_per_mm (self, which):
		return self.extruder[which].motor.steps_per_mm
	# }}}
	def extruder_motor_set_max_v_neg (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.max_v_neg = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_motor_get_max_v_neg (self, which):
		return self.extruder[which].motor.max_v_neg
	# }}}
	def extruder_motor_set_max_v_pos (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.max_v_pos = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_motor_get_max_v_pos (self, which):
		return self.extruder[which].motor.max_v_pos
	# }}}
	def extruder_motor_set_max_a (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.max_a = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_motor_get_max_a (self, which):
		return self.extruder[which].motor.max_a
	# }}}
	def extruder_set_filament_heat (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].filament_heat = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_get_filament_heat (self, which):
		return self.extruder[which].filament_heat
	# }}}
	def extruder_set_nozzle_size (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].nozzle_size = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_get_nozzle_size (self, which):
		return self.extruder[which].nozzle_size
	# }}}
	def extruder_set_filament_size (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].filament_size = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
	def extruder_get_filament_size (self, which):
		return self.extruder[which].filament_size
	# }}}
	# }}}
	# }}}
# }}}

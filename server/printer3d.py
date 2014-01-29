# vim: set foldmethod=marker :

show_own_debug = None
show_firmware_debug = True

# Imports.  {{{
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
		print ('%s: %s' % (x, ' '.join (['%02x' % ord (c) for c in data])))
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
	single = { 'ACK': '\x80', 'NACK': '\xe1', 'ACKWAIT': '\xd2', 'STALL': '\xb3', 'RESET': '\xf4', 'INIT': '\x95', 'ACKRESET': '\xa6', 'DEBUG': '\xc7' }
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
			'AUDIO_SETUP': 0x10,
			'AUDIO_DATA': 0x11}
	rcommand = {
			'START': '\x12',
			'TEMP': '\x13',
			'POS': '\x14',
			'DATA': '\x15',
			'PONG': '\x16',
			'MOVECB': '\x17',
			'TEMPCB': '\x18',
			'CONTINUE': '\x19',
			'LIMIT': '\x1a',
			'MESSAGE': '\x1b'}
	# }}}
	def __init__ (self, name = None): # {{{
		# Assume a GNU/Linux system; if you have something else, you need to come up with a way to iterate over all your serial ports and implement it here.  Patches welcome, especially if they are platform-independent.
		self.audiodir = '.'
		self.audiofile = None
		self.continue_cb = None
		self.continue_audio_cb = None
		self.pos = []
		blacklist = 'ptmx$|console$|ttyS?\d*$'
		found_ports = []
		found_printers = []
		for p in os.listdir ('/sys/class/tty'):
			if re.match (blacklist, p):
				continue
			try:
				# Set up state.
				self.ff_in = False
				self.ff_out = False
				self.limits = {}
				self.wait = False
				self.waitaudio = False
				self.movewait = 0
				self.tempwait = set ()
				self.printer = serial.Serial ('/dev/' + p, baudrate = 115200, timeout = .01)
				found_ports.append (p)
				self.printer.readall ()	# flush buffer.
				# Reset printer.
				self.printer.setDTR (False)
				time.sleep (.1)
				self.printer.setDTR (True)
				time.sleep (.1)
				# Wait for firmware to boot.
				self.printer.setTimeout (Printer.long_timeout)
				r = self.printer.read ()
				dprint ('read (0)', r)
				if len (r) == 0:
					continue
				self.printer.setTimeout (Printer.default_timeout)
				while True:
					if r == self.single['DEBUG']:
						self._handle_debug ()
						r = self.printer.read ()
						dprint ('read (0)', r)
						continue
					if r == self.single['INIT']:
						break
					dprint ('unexpected data', r)
					r = self.printer.read ()
					dprint ('read (0)', r)
				self.printer.setTimeout (Printer.default_timeout)
				self._begin ()
				self.namelen, self.maxaxes, self.maxextruders, self.maxtemps, self.audio_fragments, self.audio_fragment_size = struct.unpack ('<BBBBBB', self._read (0))
				self.load (1)
				if not name or re.match (name, self.name):
					self.axis = [Printer.Axis (self, t) for t in range (self.maxaxes)]
					for a in range (self.maxaxes):
						self.axis[a].read (self._read (2 + a))
					self.extruder = [Printer.Extruder () for t in range (self.maxextruders)]
					for e in range (self.maxextruders):
						self.extruder[e].read (self._read (2 + self.maxaxes + e))
					self.temp = [Printer.Temp () for t in range (self.maxtemps)]
					for t in range (self.maxtemps):
						self.temp[t].read (self._read (2 + self.maxaxes + self.maxextruders + t))
					global show_own_debug
					if show_own_debug is None:
						show_own_debug = True
					return
				else:
					found_printers.append ((p, self.name))
			except:
				print ('Not using %s: %s' % (p, sys.exc_value))
				pass
		sys.stderr.write ('Printer not found.  Usable ports: %s, Printers: %s\n' % (', '.join (found_ports), ', '.join (['%s (%s)' % (x[1], x[0]) for x in found_printers])))
		raise ValueError ('printer not found')
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
		#print ' '.join (['%02x' % ord (x) for x in data])
		return data
	# }}}
	def _parse_packet (self, data): # {{{
		#print ' '.join (['%02x' % ord (x) for x in data])
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
		if self.ff_out:
			data = chr (ord (data[0]) | 0x80) + data[1:]
		self.ff_out = not self.ff_out
		data = self._make_packet (data)
		events = 0
		ready = 0
		while True:
			dprint ('(1) writing', data);
			self.printer.write (data)
			while True:
				r = self.printer.read (1)
				if r != self.single['DEBUG']:
					dprint ('(1) read', r)
				if r == '':
					break	# Packet was not received.  Break from this loop to resend it.
				if r == self.single['DEBUG']:
					self._handle_debug ()
					continue
				if r == self.single['ACK']:
					return	# Done.
				if r == self.single['NACK']:
					break	# Break from this loop, so the packet is sent again.
				if r == self.single['ACKWAIT']:
					if audio:
						self.waitaudio = True
					else:
						self.wait = True
					return	# Done.
				assert r != self.single['STALL']	# This must not happen and if it does, there's no way to handle it, so raise an exception.
				assert r != self.single['RESET']	# This should only be sent to the firmware, not to the host.
				assert r != self.single['INIT']		# This means the printer has reset; that shouldn't happen (but should be handled better; TODO).
				assert r != self.single['ACKRESET']	# This should only happen after we request it.
				if (ord (r) & 0x80) != 0:
					dprint ('writing (1)', self.single['NACK']);
					self.printer.write (self.single['NACK'])
					continue
				# Regular packet received.  Must be asynchronous; handle it.
				#print repr (r)
				reply = self._recv_packet (r, True)
				#print repr (reply)
				assert reply == ''
	# }}}
	def _recv_packet (self, buffer = '', want_any = False, end_time = None): # {{{
		if end_time:
			self.printer.setTimeout (end_time - time.time ())
		while True:
			while len (buffer) < 1:
				r = self.printer.read (1)
				if r == self.single['DEBUG']:
					self._handle_debug ()
					continue
				else:
					dprint ('(3) read', r)
				if r == '':
					if end_time and time.time () >= end_time:
						self.printer.setTimeout (Printer.default_timeout)
						return ''
					if want_any == False:
						dprint ('writing (2)', self.single['NACK']);
						self.printer.write (self.single['NACK'])
					continue
				assert r != self.single['INIT']	# This should be handled more gracefully.  TODO.
				assert (ord (r) & 0x80) == 0	# This must not happen.
				buffer += r
			length = ord (buffer[0]) + (ord (buffer[0]) + 2) / 3
			while len (buffer) < length:
				r = self.printer.read (length - len (buffer))
				dprint ('read (%d/%d)' % (len (r) + len (buffer), length), r)
				if r == '':
					dprint ('writing (3)', self.single['NACK']);
					self.printer.write (self.single['NACK'])
					buffer = ''
					break	# Start over.
				buffer += r
			else:
				# Entire buffer has been received.
				data = self._parse_packet (buffer)
				if data is None:
					dprint ('writing (4)', self.single['NACK']);
					self.printer.write (self.single['NACK'])
					buffer = ''
					continue	# Start over.
				if bool (ord (data[0]) & 0x80) != self.ff_in:
					# This was a retry of the previous packet; accept it and retry.
					dprint ('(2) writing', self.single['ACK']);
					self.printer.write (self.single['ACK'])
					buffer = ''
					continue	# Start over.
				# Packet successfully received.
				dprint ('(3) writing', self.single['ACK']);
				self.printer.write (self.single['ACK'])
				# Clear the flip flop.
				data = chr (ord (data[0]) & ~0x80) + data[1:]
				# Flip it.
				self.ff_in = not self.ff_in
				# Handle the asynchronous events here; don't bother the caller with them.
				if data[0] == self.rcommand['MOVECB']:
					num = ord (data[1])
					#print ('movecb %d/%d' % (num, self.movewait))
					assert self.movewait >= num
					self.movewait -= num
					if want_any:
						if end_time:
							self.printer.setTimeout (Printer.default_timeout)
						return ''
					buffer = ''
					continue	# Start over.
				if data[0] == self.rcommand['TEMPCB']:
					assert ord (data[1]) in self.tempwait
					self.tempwait.remove (ord (data[1]))
					if want_any:
						if end_time:
							self.printer.setTimeout (Printer.default_timeout)
						return ''
					buffer = ''
					continue	# Start over.
				if data[0] == self.rcommand['CONTINUE']:
					if data[1] == '\x00':
						# Move continue.
						self.wait = False
						if self.continue_cb:
							self.continue_cb ()
					else:
						# Audio continue.
						self.waitaudio = False
						self._send_audio ()
					if want_any:
						if end_time:
							self.printer.setTimeout (Printer.default_timeout)
						return ''
					buffer = ''
					continue	# Start over.
				if data[0] == self.rcommand['LIMIT']:
					self.limits[ord (data[1])] = struct.unpack ('<l', data[2:])[0]
					if want_any:
						if end_time:
							self.printer.setTimeout (Printer.default_timeout)
						return ''
					buffer = ''
					continue	# Start over.
				if data[0] == self.rcommand['MESSAGE']:
					self.messages.append ((struct.unpack ('<l', data[1:5])[0], data[5:]))
					print ('Message: %d %s' % self.messages[-1])
					continue	# Start over.
				if end_time:
					self.printer.setTimeout (Printer.default_timeout)
				return data
	# }}}
	def _handle_debug (self): # {{{
		s = ''
		while True:
			r = self.printer.read (1)
			if r in ('', '\0'):
				break
			s += r
		if show_firmware_debug:
			print ('Debug: %s' % s)
	def _begin (self): # {{{
		self._send_packet (struct.pack ('<Bf', self.command['BEGIN'], 0))
		assert struct.unpack ('<Bf', self._recv_packet ()) == (ord (self.rcommand['START']), 0.0)
	# }}}
	def _read (self, channel): # {{{
		self._send_packet (struct.pack ('<BB', self.command['READ'], channel))
		p = self._recv_packet ()
		assert p[0] == self.rcommand['DATA']
		return p[1:]
	# }}}
	# }}}
	def _write_variables (self): # {{{
		data = (self.name + chr (0) * self.namelen)[:self.namelen] + struct.pack ('<BBBBHfLLf', self.num_axes, self.num_extruders, self.num_temps, self.printer_type, self.led_pin, self.room_T, self.motor_limit, self.temp_limit, self.feedrate)
		self._send_packet (struct.pack ('<BB', self.command['WRITE'], 1) + data)
	# }}}
	def _write_axis (self, which): # {{{
		data = self.axis[which].write ()
		self._send_packet (struct.pack ('<BB', self.command['WRITE'], 2 + which) + data)
	# }}}
	def _write_extruder (self, which): # {{{
		data = self.extruder[which].write ()
		self._send_packet (struct.pack ('<BB', self.command['WRITE'], 2 + self.maxaxes + which) + data)
	# }}}
	def _write_temp (self, which): # {{{
		data = self.temp[which].write ()
		self._send_packet (struct.pack ('<BB', self.command['WRITE'], 2 + self.maxaxes + self.maxextruders + which) + data)
	# }}}
	def _send_audio (self): # {{{
		while not self.waitaudio and self.audiofile is not None:
			data = self.audiofile.read (self.audio_fragment_size)
			if len (data) < self.audio_fragment_size:
				self.audiofile = None
				return
			self._send_packet (chr (self.command['AUDIO_DATA']) + data, audio = True)
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
			self.limit_min_pin, self.limit_max_pin, self.limit_min_pos, self.limit_max_pos, self.delta_length, self.delta_radius, self.offset = struct.unpack ('<HHllfff', data)
		def write (self):
			return self.motor.write () + struct.pack ('<HHllfff', self.limit_min_pin, self.limit_max_pin, self.limit_min_pos, self.limit_max_pos, self.delta_length, self.delta_radius, self.offset)
		def set_current_pos (self, pos):
			self.printer._send_packet (struct.pack ('<BBl', self.printer.command['SETPOS'], 2 + self.id, pos))
		def get_current_pos (self):
			self.printer._send_packet (struct.pack ('<BB', self.printer.command['GETPOS'], 2 + self.id))
			ret = self.printer._recv_packet ()
			assert ret[0] == self.printer.rcommand['POS']
			return struct.unpack ('<lf', ret[1:])
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
		while self.wait:
			self.block ()
		if cb:
			self.movewait += 1
			#print ('movewait +1 -> %d' % self.movewait)
			p = chr (self.command['GOTOCB'])
		else:
			p = chr (self.command['GOTO'])
		if isinstance (axes, (list, tuple)):
			a = {}
			for i, axis in enumerate (axes):
				a[i] = axis
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
		self._send_packet (p + ''.join ([chr (t) for t in targets]) + args)
	# }}}
	def run_axis (self, which, speed): # {{{
		self.run (2 + which, speed)
	# }}}
	def run_extruder (self, which, speed): # {{{
		self.run (2 + self.maxaxes + which, speed)
	# }}}
	def run (self, channel, speed):	# speed: float; 0 means off. # {{{
		if channel >= 2 and channel < 2 + self.num_axes and not math.isnan (speed):
			self.pos[channel - 2] = float ('nan')
		self._send_packet (struct.pack ('<BBf', self.command['RUN'], channel, speed))
	# }}}
	def sleep_axis (self, which, sleeping = True): # {{{
		self.sleep (2 + which, sleeping)
	# }}}
	def sleep_extruder (self, which, sleeping = True): # {{{
		self.sleep (2 + self.maxaxes + which, sleeping)
	# }}}
	def sleep (self, channel, sleeping = True): # {{{
		self._send_packet (struct.pack ('<BB', self.command['SLEEP'], (channel & 0x7f) | (0x80 if sleeping else 0)))
	# }}}
	def settemp_extruder (self, which, temp):	# {{{
		self.settemp (2 + self.maxaxes + which, temp)
	# }}}
	def settemp_temp (self, which, temp):	# {{{
		self.settemp (2 + self.maxaxes + self.maxextruders + which, temp)
	# }}}
	def settemp (self, channel, temp): # {{{
		self._send_packet (struct.pack ('<BBf', self.command['SETTEMP'], channel, temp))
	# }}}
	def waittemp_extruder (self, which, min, max):	# {{{
		self.waittemp (2 + self.maxaxes + which, min, max)
	# }}}
	def waittemp_temp (self, which, min, max):	# {{{
		self.waittemp (2 + self.maxaxes + self.maxextruders + which, min, max)
	# }}}
	def waittemp (self, channel, min, max): # {{{
		if min is None:
			min = float ('nan')
		if max is None:
			max = float ('nan')
		self._send_packet (struct.pack ('<BBff', self.command['WAITTEMP'], channel, min, max))
		if math.isnan (min) and math.isnan (max):
			self.tempwait.discard (channel)
		else:
			self.tempwait.add (channel)
	# }}}
	def blocktemps (self): # {{{
		while len (self.tempwait) > 0:
			self.block ()
	# }}}
	def readtemp_extruder (self, which):	# {{{
		return self.readtemp (2 + self.maxaxes + which)
	# }}}
	def readtemp_temp (self, which):	# {{{
		return self.readtemp (2 + self.maxaxes + self.maxextruders + which)
	# }}}
	def readtemp (self, channel): # {{{
		self._send_packet (struct.pack ('<BB', self.command['READTEMP'], channel))
		ret = self._recv_packet ()
		assert ret[0] == self.rcommand['TEMP']
		return struct.unpack ('<f', ret[1:])[0]
	# }}}
	def load_variables (self): # {{{
		self.load (1)
	# }}}
	def load_axis (self, which): # {{{
		self.load (2 + which)
	# }}}
	def load_extruder (self, which): # {{{
		self.load (2 + self.maxaxes + which)
	# }}}
	def load_temp (self, which): # {{{
		self.load (2 + self.maxaxes + self.maxextruders + which)
	# }}}
	def load (self, channel): # {{{
		self._send_packet (struct.pack ('<BB', self.command['LOAD'], channel))
		if channel == 1:
			data = self._read (1)
			self.name = data[:self.namelen]
			self.num_axes, self.num_extruders, self.num_temps, self.printer_type, self.led_pin, self.room_T, self.motor_limit, self.temp_limit, self.feedrate = struct.unpack ('<BBBBHfLLf', data[self.namelen:])
			self.pos = (self.pos + [float ('nan')] * self.num_axes)[:self.num_axes]
		elif 2 <= channel < 2 + self.maxaxes:
			self.axis[channel - 2].read (self._read (channel))
		elif 2 + self.maxaxes <= channel < 2 + self.maxaxes + self.maxextruders:
			self.extruder[channel - 2 - self.maxaxes].read (self._read (channel))
		else:
			assert channel < 2 + self.maxaxes + self.maxextruders + self.maxtemps
			self.temp[channel - 2 - self.maxaxes - self.maxextruders].read (self._read (channel))
	# }}}
	def load_all (self): # {{{
		for i in range (1, 2 + self.maxaxes + self.maxextruders + self.maxtemps):
			print ('loading %d' % i)
			self.load (i)
	# }}}
	def save_variables (self): # {{{
		self.save (1)
	# }}}
	def save_axis (self, which): # {{{
		self.save (2 + which)
	# }}}
	def save_extruder (self, which): # {{{
		self.save (2 + self.maxaxes + which)
	# }}}
	def save_temp (self, which): # {{{
		self.save (2 + self.maxaxes + self.maxextruders + which)
	# }}}
	def save (self, channel): # {{{
		self._send_packet (struct.pack ('<BB', self.command['SAVE'], channel))
	# }}}
	def save_all (self): # {{{
		for i in range (1, 2 + self.maxaxes + self.maxextruders + self.maxtemps):
			print ('saving %d' % i)
			self.save (i)
	# }}}
	def pause (self, pausing = True): # {{{
		self._send_packet (struct.pack ('<BB', self.command['PAUSE'], pausing))
	# }}}
	def ping (self, arg = 0): # {{{
		self._send_packet (struct.pack ('<BB', self.command['PING'], arg))
		assert struct.unpack ('<BB', self._recv_packet ()) == (ord (self.rcommand['PONG']), arg)
	# }}}
	def home (self, axes = (0, 1, 2)): # {{{
		if self.printer_type != 1:
			# Find the switches.
			self.limits.clear ()
			for axis in axes:
				s = 3200. / self.axis[axis].motor.steps_per_mm
				self.run_axis (axis, -s if self.pin_valid (self.axis[axis].limit_min_pin) else s)
			self.wait_for_limits (len (axes))
			# Back off a bit.
			axesmove = {}
			for axis in axes:
				self.axis[axis].set_current_pos (0)
				axesmove[axis] = 3 if self.pin_valid (self.axis[axis].limit_min_pin) else -3
			self.goto (axes = axesmove, cb = True)
			self.wait_for_cb ()
			# Move there again, but slowly.
			self.limits.clear ()
			for axis in axes:
				self.run_axis (axis, -1 if self.pin_valid (self.axis[axis].limit_min_pin) else 1)
			self.wait_for_limits (len (axes))
			# Current position is 0.
			ret = [self.axis[axis].limit_min_pos if self.pin_valid (self.axis[axis].limit_min_pin) else self.axis[axis].limit_max_pos for axis in axes]
			for i, axis in enumerate (axes):
				self.axis[axis].set_current_pos (ret[i])
			for i, a in enumerate (axes):
				self.pos[a] = ret[i] / self.axis[a].motor.steps_per_mm
			return [self.pos[p] for p in axes]
		else:
			# Compute home position close to limit switches.
			if self.pin_valid (self.axis[0].limit_max_pin):
				pos = (0, 0, min ([self.axis[i].limit_max_pos / self.axis[i].motor.steps_per_mm for i in range (3)]) - 10)
			else:
				pos = [0, 0, max ([self.axis[i].limit_min_pos / self.axis[i].motor.steps_per_mm for i in range (3)]) + 10]
			# Go to limit switches.
			self.limits.clear ()
			s = 1 if self.pin_valid (self.axis[0].limit_max_pin) else -1
			for i in range (3):
				self.run_axis (i, s * 50)
			self.wait_for_limits (3)
			# Set positions to limit switch positions.
			for i in range (3):
				self.axis[i].set_current_pos (self.axis[i].limit_max_pos if self.pin_valid (self.axis[0].limit_max_pin) else self.axis[i].limit_min_pos)
			# Go to home position.
			self.goto (pos, cb = True)
			self.wait_for_cb ()
			# Slowly go to limit switches.
			self.limits.clear ()
			for i in range (3):
				self.run_axis (i, s * 10)
			self.wait_for_limits (3)
			self.limits.clear ()
			# Set positions to improved values.
			for i in range (3):
				self.axis[i].set_current_pos (self.axis[i].limit_max_pos if self.pin_valid (self.axis[0].limit_max_pin) else self.axis[i].limit_min_pos)
			# Go to home position.
			self.goto (pos, cb = True)
			self.wait_for_cb ()
			# Return current nozzle position.
			return [pos[a] for a in axes]
	# }}}
	def audio_play (self, name, axes = None, extruders = None): # {{{
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
		self._send_packet (chr (self.command['AUDIO_SETUP']) + us_per_bit + ''.join ([chr (x) for x in channels]))
		self._send_audio ()
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
					us_per_bit = struct.unpack ('<H', f.read (2))
					bits = (os.fstat (f.fileno ()).st_size - 2) * 8
					t = bits * us_per_bit * 1e-6
					ret.append ((x, t))
			except:
				pass
		return ret
	# }}}
	def wait_for_cb (self): # {{{
		while self.movewait > 0:
			self._recv_packet (want_any = True)
	# }}}
	def wait_for_limits (self, num = 1): # {{{
		while len (self.limits) < num:
			self._recv_packet (want_any = True)
	# }}}
	def wait_for_temp (self): # {{{
		while len (self.tempwait) > 0:
			self._recv_packet (want_any = True)
	# }}}
	def block (self, timeout = 30, probe = True): # {{{
		self.printer.setTimeout (timeout)
		while True:
			r = self.printer.read (1)
			if r != self.single['DEBUG']:
				dprint ('(1) read', r)
			if r == '':
				assert probe
				break
			if r == self.single['DEBUG']:
				self._handle_debug ()
				if probe:
					break
				continue
			if r == self.single['ACK']:
				if probe:
					break
				continue
			if r == self.single['NACK']:
				if probe:
					break
				continue	# ignore.
			if r == self.single['ACKWAIT']:
				if probe:
					break
				continue	# ignore.
			assert r != self.single['STALL']	# This must not happen and if it does, there's no way to handle it, so raise an exception.
			assert r != self.single['RESET']	# This should only be sent to the firmware, not to the host.
			assert r != self.single['INIT']		# This means the printer has reset; that shouldn't happen (but should be handled better; TODO).
			assert r != self.single['ACKRESET']	# This should only happen after we request it.
			if (ord (r) & 0x80) != 0:
				dprint ('writing (1)', self.single['NACK']);
				self.printer.write (self.single['NACK'])
				if probe:
					break
				continue
			# Regular packet received.  Must be asynchronous; handle it.
			#print repr (r)
			self.printer.setTimeout (Printer.default_timeout)
			reply = self._recv_packet (r, True)
			#print repr (reply)
			assert reply == ''
			break
	# }}}
	def is_waiting (self):	# {{{
		return self.wait or len (self.tempwait) > 0
	# }}}
	def get_limits (self):	# {{{
		return self.limits
	# }}}
	def clear_limits (self):	# {{{
		self.limits.clear ()
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
	# }}}
	# Variables. {{{
	def set_name (self, value):	# {{{
		self.name = value
		self._write_variables ()
	def get_name (self):
		return self.name
	# }}}
	def set_num_axes (self, value):	# {{{
		self.num_axes = value
		self.pos = (self.pos + [float ('nan')] * value)[:value]
		self._write_variables ()
	def get_num_axes (self):
		return self.num_axes
	# }}}
	def set_num_extruders (self, value):	# {{{
		self.num_extruders = value
		self._write_variables ()
	def get_num_extruders (self):
		return self.num_extruders
	# }}}
	def set_num_temps (self, value):	# {{{
		self.num_temps = value
		self._write_variables ()
	def get_num_temps (self):
		return self.num_temps
	# }}}
	def set_printer_type (self, value):	# {{{
		self.printer_type = value
		self._write_variables ()
	def get_printer_type (self):
		return self.printer_type
	# }}}
	def set_led_pin (self, value):	# {{{
		self.led_pin = value
		self._write_variables ()
	def get_led_pin (self):
		return self.led_pin
	# }}}
	def set_room_T (self, value):	# {{{
		self.room_T = value
		self._write_variables ()
	def get_room_T (self):
		return self.room_T
	# }}}
	def set_motor_limit (self, value):	# {{{
		self.motor_limit = value
		self._write_variables ()
	def get_motor_limit (self):
		return self.motor_limit
	# }}}
	def set_temp_limit (self, value):	# {{{
		self.temp_limit = value
		self._write_variables ()
	def get_temp_limit (self):
		return self.temp_limit
	# }}}
	def set_feedrate (self, value):	# {{{
		self.feedrate = value
		self._write_variables ()
	def get_feedrate (self):
		return self.feedrate
	# }}}
	# }}}
	# Temp {{{
	def temp_set_alpha (self, which, value):	# {{{
		self.temp[which].alpha = value
		self._write_temp (which)
	def temp_get_alpha (self, which):
		return self.temp[which].alpha
	# }}}
	def temp_set_beta (self, which, value):	# {{{
		self.temp[which].beta = value
		self._write_temp (which)
	def temp_get_beta (self, which):
		return self.temp[which].beta
	# }}}
	def temp_set_core_C (self, which, value):	# {{{
		self.temp[which].core_C = value
		self._write_temp (which)
	def temp_get_core_C (self, which):
		return self.temp[which].core_C
	# }}}
	def temp_set_shell_C (self, which, value):	# {{{
		self.temp[which].shell_C = value
		self._write_temp (which)
	def temp_get_shell_C (self, which):
		return self.temp[which].shell_C
	# }}}
	def temp_set_transfer (self, which, value):	# {{{
		self.temp[which].transfer = value
		self._write_temp (which)
	def temp_get_transfer (self, which):
		return self.temp[which].transfer
	# }}}
	def temp_set_radiation (self, which, value):	# {{{
		self.temp[which].radiation = value
		self._write_temp (which)
	def temp_get_radiation (self, which):
		return self.temp[which].radiation
	# }}}
	def temp_set_power (self, which, value):	# {{{
		self.temp[which].power = value
		self._write_temp (which)
	def temp_get_power (self, which):
		return self.temp[which].power
	# }}}
	def temp_set_power_pin (self, which, value):	# {{{
		self.temp[which].power_pin = value
		self._write_temp (which)
	def temp_get_power_pin (self, which):
		return self.temp[which].power_pin
	# }}}
	def temp_set_thermistor_pin (self, which, value):	# {{{
		self.temp[which].thermistor_pin = value
		self._write_temp (which)
	def temp_get_thermistor_pin (self, which):
		return self.temp[which].thermistor_pin
	# }}}
	# }}}
	# Axis {{{
	def axis_motor_set_step_pin (self, which, value):	# {{{
		self.axis[which].motor.step_pin = value
		self._write_axis (which)
	def axis_motor_get_step_pin (self, which):
		return self.axis[which].motor.step_pin
	# }}}
	def axis_motor_set_dir_pin (self, which, value):	# {{{
		self.axis[which].motor.dir_pin = value
		self._write_axis (which)
	def axis_motor_get_dir_pin (self, which):
		return self.axis[which].motor.dir_pin
	# }}}
	def axis_motor_set_enable_pin (self, which, value):	# {{{
		self.axis[which].motor.enable_pin = value
		self._write_axis (which)
	def axis_motor_get_enable_pin (self, which):
		return self.axis[which].motor.enable_pin
	# }}}
	def axis_motor_set_steps_per_mm (self, which, value):	# {{{
		self.axis[which].motor.steps_per_mm = value
		self._write_axis (which)
	def axis_motor_get_steps_per_mm (self, which):
		return self.axis[which].motor.steps_per_mm
	# }}}
	def axis_motor_set_max_v_neg (self, which, value):	# {{{
		self.axis[which].motor.max_v_neg = value
		self._write_axis (which)
	def axis_motor_get_max_v_neg (self, which):
		return self.axis[which].motor.max_v_neg
	# }}}
	def axis_motor_set_max_v_pos (self, which, value):	# {{{
		self.axis[which].motor.max_v_pos = value
		self._write_axis (which)
	def axis_motor_get_max_v_pos (self, which):
		return self.axis[which].motor.max_v_pos
	# }}}
	def axis_motor_set_max_a (self, which, value):	# {{{
		self.axis[which].motor.max_a = value
		self._write_axis (which)
	def axis_motor_get_max_a (self, which):
		return self.axis[which].motor.max_a
	# }}}
	def axis_set_limit_min_pin (self, which, value):	# {{{
		self.axis[which].limit_min_pin = value
		self._write_axis (which)
	def axis_get_limit_min_pin (self, which):
		return self.axis[which].limit_min_pin
	# }}}
	def axis_set_limit_max_pin (self, which, value):	# {{{
		self.axis[which].limit_max_pin = value
		self._write_axis (which)
	def axis_get_limit_max_pin (self, which):
		return self.axis[which].limit_max_pin
	# }}}
	def axis_set_limit_min_pos (self, which, value):	# {{{
		self.axis[which].limit_min_pos = value
		self._write_axis (which)
	def axis_get_limit_min_pos (self, which):
		return self.axis[which].limit_min_pos
	# }}}
	def axis_set_limit_max_pos (self, which, value):	# {{{
		self.axis[which].limit_max_pos = value
		self._write_axis (which)
	def axis_get_limit_max_pos (self, which):
		return self.axis[which].limit_max_pos
	# }}}
	def axis_set_delta_length (self, which, value):	# {{{
		self.axis[which].delta_length = value
		self._write_axis (which)
	def axis_get_delta_length (self, which):
		return self.axis[which].delta_length
	# }}}
	def axis_set_delta_radius (self, which, value):	# {{{
		self.axis[which].delta_radius = value
		self._write_axis (which)
	def axis_get_delta_radius (self, which):
		return self.axis[which].delta_radius
	# }}}
	def axis_set_offset (self, which, offset):	# {{{
		self.axis[which].offset = offset
		self._write_axis (which)
	def axis_get_offset (self, which):
		return self.axis[which].offset
	# }}}
	def axis_set_current_pos (self, which, pos):	# {{{
		self.axis[which].set_current_pos (pos)
	def axis_get_current_pos (self, which):
		return self.axis[which].get_current_pos ()
	# }}}
	# }}}
	# Extruder {{{
	def extruder_temp_set_alpha (self, which, value):	# {{{
		self.extruder[which].temp.alpha = value
		self._write_temp (which)
	def extruder_temp_get_alpha (self, which):
		return self.extruder[which].temp.alpha
	# }}}
	def extruder_temp_set_beta (self, which, value):	# {{{
		self.extruder[which].temp.beta = value
		self._write_temp (which)
	def extruder_temp_get_beta (self, which):
		return self.extruder[which].temp.beta
	# }}}
	def extruder_temp_set_core_C (self, which, value):	# {{{
		self.extruder[which].temp.core_C = value
		self._write_temp (which)
	def extruder_temp_get_core_C (self, which):
		return self.extruder[which].temp.core_C
	# }}}
	def extruder_temp_set_shell_C (self, which, value):	# {{{
		self.extruder[which].temp.shell_C = value
		self._write_temp (which)
	def extruder_temp_get_shell_C (self, which):
		return self.extruder[which].temp.shell_C
	# }}}
	def extruder_temp_set_transfer (self, which, value):	# {{{
		self.extruder[which].temp.transfer = value
		self._write_temp (which)
	def extruder_temp_get_transfer (self, which):
		return self.extruder[which].temp.transfer
	# }}}
	def extruder_temp_set_radiation (self, which, value):	# {{{
		self.extruder[which].temp.radiation = value
		self._write_temp (which)
	def extruder_temp_get_radiation (self, which):
		return self.extruder[which].temp.radiation
	# }}}
	def extruder_temp_set_power (self, which, value):	# {{{
		self.extruder[which].temp.power = value
		self._write_temp (which)
	def extruder_temp_get_power (self, which):
		return self.extruder[which].temp.power
	# }}}
	def extruder_temp_set_power_pin (self, which, value):	# {{{
		self.extruder[which].temp.power_pin = value
		self._write_temp (which)
	def extruder_temp_get_power_pin (self, which):
		return self.extruder[which].temp.power_pin
	# }}}
	def extruder_temp_set_thermistor_pin (self, which, value):	# {{{
		self.extruder[which].temp.thermistor_pin = value
		self._write_temp (which)
	def extruder_temp_get_thermistor_pin (self, which):
		return self.extruder[which].temp.thermistor_pin
	# }}}
	def extruder_motor_set_step_pin (self, which, value):	# {{{
		self.extruder[which].motor.step_pin = value
		self._write_extruder (which)
	def extruder_motor_get_step_pin (self, which):
		return self.extruder[which].motor.step_pin
	# }}}
	def extruder_motor_set_dir_pin (self, which, value):	# {{{
		self.extruder[which].motor.dir_pin = value
		self._write_extruder (which)
	def extruder_motor_get_dir_pin (self, which):
		return self.extruder[which].motor.dir_pin
	# }}}
	def extruder_motor_set_enable_pin (self, which, value):	# {{{
		self.extruder[which].motor.enable_pin = value
		self._write_extruder (which)
	def extruder_motor_get_enable_pin (self, which):
		return self.extruder[which].motor.enable_pin
	# }}}
	def extruder_motor_set_steps_per_mm (self, which, value):	# {{{
		self.extruder[which].motor.steps_per_mm = value
		self._write_extruder (which)
	def extruder_motor_get_steps_per_mm (self, which):
		return self.extruder[which].motor.steps_per_mm
	# }}}
	def extruder_motor_set_max_v_neg (self, which, value):	# {{{
		self.extruder[which].motor.max_v_neg = value
		self._write_extruder (which)
	def extruder_motor_get_max_v_neg (self, which):
		return self.extruder[which].motor.max_v_neg
	# }}}
	def extruder_motor_set_max_v_pos (self, which, value):	# {{{
		self.extruder[which].motor.max_v_pos = value
		self._write_extruder (which)
	def extruder_motor_get_max_v_pos (self, which):
		return self.extruder[which].motor.max_v_pos
	# }}}
	def extruder_motor_set_max_a (self, which, value):	# {{{
		self.extruder[which].motor.max_a = value
		self._write_extruder (which)
	def extruder_motor_get_max_a (self, which):
		return self.extruder[which].motor.max_a
	# }}}
	def extruder_set_filament_heat (self, which, value):	# {{{
		self.extruder[which].filament_heat = value
		self._write_extruder (which)
	def extruder_get_filament_heat (self, which):
		return self.extruder[which].filament_heat
	# }}}
	def extruder_set_nozzle_size (self, which, value):	# {{{
		self.extruder[which].nozzle_size = value
		self._write_extruder (which)
	def extruder_get_nozzle_size (self, which):
		return self.extruder[which].nozzle_size
	# }}}
	def extruder_set_filament_size (self, which, value):	# {{{
		self.extruder[which].filament_size = value
		self._write_extruder (which)
	def extruder_get_filament_size (self, which):
		return self.extruder[which].filament_size
	# }}}
	# }}}
	# }}}
# }}}

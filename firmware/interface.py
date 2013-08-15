# vim: set foldmethod=marker :

# Imports.  {{{
import serial
import time
import math
import struct
# }}}

def dprint (x, data):
	#print ('%s: %s' % (x, ' '.join (['%02x' % ord (c) for c in data])))
	pass

class Printer: # {{{
	# Internal stuff.  {{{
	# Constants.  {{{
	# Serial timeout for normal communication in seconds.
	default_timeout = 5	# TODO: this is for debugging; really should be .005
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
			'LOAD': 0x08,
			'SAVE': 0x09,
			'READ': 0x0a,
			'WRITE': 0x0b,
			'PAUSE': 0x0c,
			'PING': 0x0d}
	rcommand = {
			'START': '\x0e',
			'TEMP': '\x0f',
			'DATA': '\x10',
			'PONG': '\x11',
			'MOVECB': '\x12',
			'TEMPCB': '\x13',
			'CONTINUE': '\x14',
			'LIMIT': '\x15',
			'MESSAGE': '\x16'}
	# }}}
	def __init__ (self, port = '/dev/ttyACM0'): # {{{
		self.printer = serial.Serial (port, baudrate = 115200, timeout = .01)
		self.printer.readall ()	# flush buffer.
		# Wait for firmware to boot.
		self.printer.setTimeout (2)
		r = self.printer.read ()
		dprint ('read (0)', r)
		self.printer.setTimeout (Printer.default_timeout)
		while True:
			if r == self.single['DEBUG']:
				self.handle_debug ()
				r = self.printer.read ()
				dprint ('read (0)', r)
				continue
			assert r == self.single['INIT']
			break
		self.printer.setTimeout (Printer.default_timeout)
		# Set up state.
		self.ff_in = False
		self.ff_out = False
		self.limits = {}
		self.wait = False
		self.movewait = 0
		self.tempwait = set ()
		self.begin ()
		self.maxaxes, self.maxextruders, self.maxtemps = struct.unpack ('<BBB', self.read (0))
		self.num_axes, self.num_extruders, self.num_temps, self.room_T = struct.unpack ('<BBBf', self.read (1))
		self.axis = [Printer.Axis (self, t) for t in range (self.maxaxes)]
		for a in range (self.maxaxes):
			self.axis[a].read (self.read (2 + a))
		self.extruder = [Printer.Extruder () for t in range (self.maxextruders)]
		for e in range (self.maxextruders):
			self.extruder[e].read (self.read (2 + self.maxaxes + e))
		self.temp = [Printer.Temp () for t in range (self.maxtemps)]
		for t in range (self.maxtemps):
			self.temp[t].read (self.read (2 + self.maxaxes + self.maxextruders + t))
	# }}}
	def make_packet (self, data): # {{{
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
	def parse_packet (self, data): # {{{
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
	def send_packet (self, data): # {{{
		if self.ff_out:
			data = chr (ord (data[0]) | 0x80) + data[1:]
		self.ff_out = not self.ff_out
		data = self.make_packet (data)
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
					self.handle_debug ()
					continue
				if r == self.single['ACK']:
					return	# Done.
				if r == self.single['NACK']:
					break	# Break from this loop, so the packet is sent again.
				if r == self.single['ACKWAIT']:
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
				reply = self.recv_packet (r, True)
				#print repr (reply)
				assert reply == ''
	# }}}
	def recv_packet (self, buffer = '', want_any = False): # {{{
		while True:
			while len (buffer) < 1:
				r = self.printer.read (1)
				if r != self.single['DEBUG']:
					dprint ('(3) read', r)
				if r == '':
					dprint ('writing (2)', self.single['NACK']);
					self.printer.write (self.single['NACK'])
					continue
				if r == self.single['DEBUG']:
					self.handle_debug ()
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
				data = self.parse_packet (buffer)
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
					for i in range (ord (data[1])):
						assert self.movewait > 0
						self.movewait -= 1
					if want_any:
						return ''
					buffer = ''
					continue	# Start over.
				if data[0] == self.rcommand['TEMPCB']:
					assert ord (data[0]) in self.tempwait
					self.tempwait.remove (ord (data[0]))
					if want_any:
						return ''
					buffer = ''
					continue	# Start over.
				if data[0] == self.rcommand['CONTINUE']:
					self.wait = False
					if want_any:
						return ''
					buffer = ''
					continue	# Start over.
				if data[0] == self.rcommand['LIMIT']:
					self.limits[ord (data[1])] = struct.unpack ('<l', data[2:])[0]
					if want_any:
						return ''
					buffer = ''
					continue	# Start over.
				if data[0] == self.rcommand['MESSAGE']:
					self.messages.append ((struct.unpack ('<l', data[1:5])[0], data[5:]))
					print ('Message: %d %s' % self.messages[-1])
					continue	# Start over.
				return data
	# }}}
	def block (self, timout = 30): # {{{
		self.printer.setTimeout (timeout)
		r = self.printer.read ()
		dprint ('(2) read', r)
		assert r != ''
		self.printer.setTimeout (Printer.default_timeout)
		assert self.recv_packet (r, True) == ''
	# }}}
	def handle_debug (self): # {{{
		s = ''
		while True:
			r = self.printer.read (1)
			if r in ('', '\0'):
				break
			s += r
		print ('Debug: %s' % s)
	# }}}
	# Config stuff.  {{{
	class Temp: # {{{
		def read (self, data):
			self.alpha, self.beta, self.radiation, self.power, self.power_pin, self.thermistor_pin = struct.unpack ('<ffffBB', data[:18])
			return data[18:]
		def write (self):
			return struct.pack ('<ffffBB', self.alpha, self.beta, self.radiation, self.power, self.power_pin, self.thermistor_pin)
	# }}}
	class Motor: # {{{
		def read (self, data):
			self.step_pin, self.dir_pin, self.enable_pin, self.steps_per_mm, self.max_f_neg, self.max_f_pos = struct.unpack ('<BBBfff', data[:11])
			return data[11:]
		def write (self):
			return struct.pack ('<BBBfff', self.step_pin, self.dir_pin, self.enable_pin, self.steps_per_mm, self.max_f_neg, self.max_f_pos)
	# }}}
	class Axis: # {{{
		def __init__ (self, printer, id):
			self.printer = printer
			self.id = id
			self.motor = Printer.Motor ()
		def read (self, data):
			data = self.motor.read (data)
			self.limit_min_pin, self.limit_max_pin, self.current_pos = struct.unpack ('<BBl', data)
		def write (self):
			return self.motor.write () + struct.pack ('<BBl', self.limit_min_pin, self.limit_max_pin, self.current_pos)
		def set_current_pos (self, pos):
			self.current_pos = pos
			self.printer.write_axis (self.id)
		def get_current_pos (self):
			# Don't update the stored value; it may be in the middle of a move.
			return struct.unpack ('<BBl', self.printer.read (2 + self.id)[-6:])[2]
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
	# Internal commands.  {{{
	def begin (self): # {{{
		self.send_packet (struct.pack ('<Bf', self.command['BEGIN'], 0))
		assert struct.unpack ('<Bf', self.recv_packet ()) == (ord (self.rcommand['START']), 0.0)
	# }}}
	def read (self, channel): # {{{
		self.send_packet (struct.pack ('<BB', self.command['READ'], channel))
		p = self.recv_packet ()
		assert p[0] == self.rcommand['DATA']
		return p[1:]
	# }}}
	# }}}
	# }}}
	# Useful commands.  {{{
	def goto (self, axes = {}, e = None, f0 = None, f1 = None, which = 0, cb = False): # {{{
		while self.wait:
			self.block ()
		if cb:
			self.movewait += 1
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
			targets[(axis + 2) >> 3] |= 1 << ((axis + 2) & 0x7)
			args += struct.pack ('<f', axes[axis])
			self.axis[axis].current_pos = axes[axis]	# Assume move has completed.
		if e is not None:
			targets[(2 + self.num_axes + which) >> 3] |= 1 << ((2 + self.num_axes + which) & 0x7)
			args += struct.pack ('<f', e)
		self.send_packet (p + ''.join ([chr (t) for t in targets]) + args)
	# }}}
	def run (self, channel, speed):	# speed: float; 0 means off. # {{{
		self.send_packet (struct.pack ('<BBf', self.command['RUN'], channel, speed))
	# }}}
	def sleep (self, channel, sleeping = True): # {{{
		self.send_packet (struct.pack ('<BB', self.command['SLEEP'], (channel & 0x7f) | (0x80 if sleeping else 0)))
	# }}}
	def settemp (self, channel, temp): # {{{
		self.send_packet (struct.pack ('<BBf', self.command['SETTEMP'], channel, temp))
	# }}}
	def waittemp (self, channel, min, max): # {{{
		if min is None:
			min = float ('nan')
		if max is None:
			max = float ('nan')
		self.send_packet (struct.pack ('<BBff', self.command['WAITTEMP'], channel, min, max))
		if math.isnan (min) and math.isnan (max):
			self.tempwait.discard (channel)
		else:
			self.tempwait.add (channel)
	# }}}
	def blocktemps (self): # {{{
		while len (self.tempwait) > 0:
			self.block ()
	# }}}
	def readtemp (self, channel): # {{{
		self.send_packet (struct.pack ('<BB', self.command['READTEMP'], channel))
		ret = self.recv_packet ()
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
		self.send_packet (struct.pack ('<BB', self.command['LOAD'], channel))
		if channel == 1:
			self.num_axes, self.num_extruders, self.num_temps, self.room_T = struct.unpack ('<BBBf', self.read (1))
		elif 2 <= channel < 2 + self.maxaxes:
			self.axis[channel - 2].read (self.read (channel))
		elif 2 + self.maxaxes <= channel < 2 + self.maxaxes + self.maxextruders:
			self.extruder[channel - 2 - self.maxaxes].read (self.read (channel))
		else:
			assert channel < 2 + self.maxaxes + self.maxextruders + self.maxtemps
			self.temp[channel - 2 - self.maxaxes - self.maxextruders].read (self.read (channel))
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
		self.send_packet (struct.pack ('<BB', self.command['SAVE'], channel))
	# }}}
	def save_all (self): # {{{
		for i in range (1, 2 + self.maxaxes + self.maxextruders + self.maxtemps):
			print ('saving %d' % i)
			self.save (i)
	# }}}
	def write_variables (self): # {{{
		data = struct.pack ('<BBBf', self.num_axes, self.num_extruders, self.num_temps, self.room_T)
		self.send_packet (struct.pack ('<BB', self.command['WRITE'], 1) + data)
	# }}}
	def write_axis (self, which): # {{{
		data = self.axis[which].write ()
		self.send_packet (struct.pack ('<BB', self.command['WRITE'], 2 + which) + data)
	# }}}
	def write_extruder (self, which): # {{{
		data = self.extruder[which].write ()
		self.send_packet (struct.pack ('<BB', self.command['WRITE'], 2 + self.maxaxes + which) + data)
	# }}}
	def write_temp (self, which): # {{{
		data = self.temp[which].write ()
		self.send_packet (struct.pack ('<BB', self.command['WRITE'], 2 + self.maxaxes + self.maxextruders + which) + data)
	# }}}
	def write (self, channel): # {{{
		if channel == 1:
			self.write_variables ()
		elif 2 <= channel < 2 + self.maxaxes:
			self.write_axis (channel - 2)
		elif 2 + self.maxaxes <= channel < 2 + self.maxaxes + self.maxextruders:
			self.write_extruder (channel - 2 - self.maxaxes)
		else:
			self.write_temp (channel - 2 - self.maxaxes - self.maxextruders)
	# }}}
	def write_all (self): # {{{
		for i in range (1, 2 + self.maxaxes + self.maxextruders + self.maxtemps):
			print ('(4) writing %d' % i)
			self.write (i)
	# }}}
	def pause (self, pausing = True): # {{{
		self.send_packet (struct.pack ('<BB', self.command['PAUSE'], pausing))
	# }}}
	def ping (self, arg = 0): # {{{
		self.send_packet (struct.pack ('<BB', self.command['PING'], arg))
		assert struct.unpack ('<BB', self.recv_packet ()) == (ord (self.rcommand['PONG']), arg)
	# }}}
	# }}}
	# Presets.  {{{
	def set_ramps_pins (self, min_limits = None, max_limits = None): # {{{
		if min_limits is None and max_limits is None:
			min_limits = True
			max_limits = False
		elif min_limits is None:
			min_limits = not max_limits
		elif max_limits is None:
			max_limits = not min_limits
		self.num_axes = 3
		p.num_extruders = 1	# Default to 1 extruder, heated bed and fan.
		p.num_temps = 2
		self.axis[0].limit_min_pin = 3 if min_limits else 255
		self.axis[0].limit_max_pin = 2 if max_limits else 255
		self.axis[0].motor.step_pin = 54
		self.axis[0].motor.dir_pin = 55
		self.axis[0].motor.enable_pin = 38
		self.axis[1].limit_min_pin = 14 if min_limits else 255
		self.axis[1].limit_max_pin = 15 if max_limits else 255
		self.axis[1].motor.step_pin = 60
		self.axis[1].motor.dir_pin = 61
		self.axis[1].motor.enable_pin = 56
		self.axis[2].limit_min_pin = 18 if min_limits else 255
		self.axis[2].limit_max_pin = 19 if max_limits else 255
		self.axis[2].motor.step_pin = 46
		self.axis[2].motor.dir_pin = 48
		self.axis[2].motor.enable_pin = 62
		for a in range (3, self.maxaxes):
			self.axis[a].limit_min_pin = 255
			self.axis[a].limit_max_pin = 255
			self.axis[a].motor.step_pin = 255
			self.axis[a].motor.dir_pin = 255
			self.axis[a].motor.enable_pin = 255
		self.extruder[0].temp.power_pin = 10
		self.extruder[0].temp.thermistor_pin = 13
		self.extruder[0].motor.step_pin = 26
		self.extruder[0].motor.dir_pin = 28
		self.extruder[0].motor.enable_pin = 24
		self.extruder[1].temp.power_pin = 9
		self.extruder[1].temp.thermistor_pin = 15
		self.extruder[1].motor.step_pin = 36
		self.extruder[1].motor.dir_pin = 34
		self.extruder[1].motor.enable_pin = 30
		for e in range (2, self.maxextruders):
			self.extruder[e].temp.power_pin = 255
			self.extruder[e].temp.thermistor_pin = 255
			self.extruder[e].R0 = float ('nan')
			self.extruder[e].motor.step_pin = 255
			self.extruder[e].motor.dir_pin = 255
			self.extruder[e].motor.enable_pin = 255
		self.temp[0].power_pin = 8
		self.temp[0].thermistor_pin = 14
		for t in range (1, self.maxtemps):
			self.temp[t].power_pin = 255
			self.temp[t].thermistor_pin = 255
	# }}}
	def set_melzi_pins (self, min_limits = None, max_limits = None): # {{{
		if min_limits is None and max_limits is None:
			min_limits = True
			max_limits = False
		elif min_limits is None:
			min_limits = not max_limits
		elif max_limits is None:
			max_limits = not min_limits
		assert not (max_limits and min_limits)
		self.axis[0].limit_min_pin = 18 if min_limits else 255
		self.axis[0].limit_max_pin = 18 if max_limits else 255
		self.axis[0].motor.step_pin = 15
		self.axis[0].motor.dir_pin = 21
		self.axis[0].motor.enable_pin = 14
		self.axis[1].limit_min_pin = 19 if min_limits else 255
		self.axis[1].limit_max_pin = 19 if max_limits else 255
		self.axis[1].motor.step_pin = 22
		self.axis[1].motor.dir_pin = 23
		self.axis[1].motor.enable_pin = 14
		self.axis[2].limit_min_pin = 20 if min_limits else 255
		self.axis[2].limit_max_pin = 20 if max_limits else 255
		self.axis[2].motor.step_pin = 3
		self.axis[2].motor.dir_pin = 2
		self.axis[2].motor.enable_pin = 26
		for a in range (3, self.maxaxes):
			self.axis[a].limit_min_pin = 255
			self.axis[a].limit_max_pin = 255
			self.axis[a].motor.step_pin = 255
			self.axis[a].motor.dir_pin = 255
			self.axis[a].motor.enable_pin = 255
		self.extruder[0].temp.power_pin = 13
		self.extruder[0].temp.thermistor_pin = 7
		self.extruder[0].motor.step_pin = 1
		self.extruder[0].motor.dir_pin = 0
		self.extruder[0].motor.enable_pin = 14
		for e in range (1, self.maxextruders):
			self.extruder[e].temp.power_pin = 255
			self.extruder[e].temp.thermistor_pin = 255
			self.extruder[e].motor.step_pin = 255
			self.extruder[e].motor.dir_pin = 255
			self.extruder[e].motor.enable_pin = 255
		self.temp[0].power_pin = 10
		self.temp[0].thermistor_pin = 6
		for t in range (1, self.maxtemps):
			self.temp[t].power_pin = 255
			self.temp[t].thermistor_pin = 255
	# }}}
	# }}}
# }}}

if __name__ == '__main__': # {{{
	p = Printer ()
	if False:
		# Set everything up for calibration.
		p.set_ramps_pins ()
		p.num_extruders = 2	# Override default.
		p.num_temps = 1
		p.room_T = 25.
		for a in range (p.maxaxes):
			p.axis[a].motor.max_f_neg = float ('inf')
			p.axis[a].motor.max_f_pos = float ('inf')
			p.axis[a].motor.steps_per_mm = float ('nan')
		# X and Y: 5 mm per tooth; 12 teeth per revolution; 200 steps per revolution; 16 microsteps per step.
		p.axis[0].motor.steps_per_mm = (200 * 16.) / (5 * 12)	# [steps/rev] / ([mm/t] * [t/rev]) = [steps/rev] / [mm/rev] = [steps/mm]
		p.axis[1].motor.steps_per_mm = (200 * 16.) / (5 * 12)
		# Z: 1.25 mm per revolution; 200 steps per revolution; 16 microsteps per step.
		p.axis[2].motor.steps_per_mm = (200 * 16.) / 1.25
		for e in range (p.maxextruders):
			p.extruder[e].temp.beta = 3885.0342279785623
			p.extruder[e].temp.alpha = 10.056909432214743
			p.extruder[e].temp.radiation = float ('nan')
			p.extruder[e].temp.power = 12. ** 2 / 5.4
			# Different per hobbed bolt and possibly per filament; must be measured.
			# However, as an estimate:
			# Small gear has 9 teeth; large gear 47.  Radius of hobbing is approximately 3.2 mm (20/2pi).
			p.extruder[e].motor.steps_per_mm = 835 #(200 * 16.) / ((9. / 47) * 20)
			p.extruder[e].motor.max_f_neg = float ('inf')
			p.extruder[e].motor.max_f_pos = float ('inf')
		for t in range (p.maxtemps):
			p.temp[t].beta = float ('nan')
			p.temp[t].alpha = float ('nan')
			p.temp[t].radiation = float ('nan')
			p.temp[t].power = float ('nan')
		p.temp[0].beta = 3700.
		p.temp[0].alpha = 11.7
		p.temp[0].radiation = float ('nan')
		p.temp[0].power = 12. ** 2 / 1.6
		p.write_all ()
		p.save_all ()
	else:
		# Display current settings.
		print ('num axes: %d' % p.num_axes)
		print ('num extruders: %d' % p.num_extruders)
		print ('num temps: %d' % p.num_temps)
		print ('room temperature: %f' % p.room_T)
		for a in range (p.maxaxes):
			print ('Axis %d:' % a)
			print ('\tlimit pins: %d %d' % (p.axis[a].limit_min_pin, p.axis[a].limit_max_pin))
			print ('\tmotor pins: %d %d %d' % (p.axis[a].motor.step_pin, p.axis[a].motor.dir_pin, p.axis[a].motor.enable_pin))
			print ('\tmotor steps per mm: %f' % p.axis[a].motor.steps_per_mm)
			print ('\tmotor max steps per ms: %f, %f' % p.axis[a].motor.max_f_neg, p.axis[a].motor.max_f_pos)
		for e in range (p.maxextruders):
			print ('extruder %d' % e)
			print ('\ttemp pins: %d %d' % (p.extruder[e].temp.power_pin, p.extruder[e].temp.thermistor_pin))
			print ('\ttemp settings: %f %f %f %f' % (p.extruder[e].temp.alpha, p.extruder[e].temp.beta, p.extruder[e].temp.radiation, p.extruder[e].temp.power))
			print ('\tmotor pins: %d %d %d' % (p.extruder[e].motor.step_pin, p.extruder[e].motor.dir_pin, p.extruder[e].motor.enable_pin))
			print ('\tmotor steps per mm: %f' % p.extruder[e].motor.steps_per_mm)
			print ('\tmotor max steps per ms: %f, %f' % p.extruder[e].motor.max_f_neg, p.extruder[e].motor.max_f_pos)
			#p.extruder[e].temp.alpha = float ('nan')
		for t in range (p.maxtemps):
			print ('temp %d' % t)
			print ('\tpins: %d %d' % (p.temp[t].power_pin, p.temp[t].thermistor_pin))
			print ('\tsettings: %f %f %f %f' % (p.temp[t].alpha, p.temp[t].beta, p.temp[t].radiation, p.temp[t].power))
			#p.temp[t].alpha = float ('nan')
		for i in range (2 + p.maxaxes, 2 + p.maxaxes + p.maxextruders + p.maxtemps):
			p.write (i)
			print ('temp %d: %f' % (i, p.readtemp (i)))
		p.goto (f0 = 3.5, axes = [-100], cb = True)
		while p.movewait > 0:
		#p.run (2, -100)
		#while len (p.limits) == 0:
			p.recv_packet (want_any = True)
		#for i in range (20):
		#	p.sleep (2, True)
		#	time.sleep (1)
		#	p.sleep (2, False)
		#	time.sleep (1)
# }}}

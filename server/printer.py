#! /usr/bin/python
# vim: set foldmethod=marker fileencoding=utf8 :

show_own_debug = False
#show_own_debug = True
show_firmware_debug = True

# Imports.  {{{
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
import io
import base64
import json
import fcntl
import select
import traceback
# }}}

fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

def dprint(x, data): # {{{
	if show_own_debug:
		log('%s: %s' %(x, ' '.join(['%02x' % ord(c) for c in data])))
# }}}

# Constants
C0 = 273.15	# Conversion between K and °C
WAIT = object()	# Sentinel for blocking functions.
# Space types
TYPE_EXTRUDER = 0
TYPE_CARTESIAN = 1
TYPE_DELTA = 2

# Decorator for functions which block.
class delayed: # {{{
	def __init__(self, f):
		self.f = f
	def __call__(self, *a, **ka):
		#log('delayed called with args %s,%s' % (repr(a), repr(ka)))
		def wrap(id):
			# HACK: self is lost because the decorator gets the unbound function.  Pass in the only instance that is ever used.
			#log('wrap called with id %s' % (repr(id)))
			return self.f(printer, id, *a, **ka)
		return (WAIT, wrap)
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
	def _send(self, *data): # {{{
		#sys.stderr.write(repr(data) + '\n')
		sys.stdout.write(json.dumps(data) + '\n')
		sys.stdout.flush()
	# }}}
	def __init__(self, port, audiodir, newid): # {{{
		# HACK: this variable needs to have its value before init is done, to make the above HACK in the generator work.
		global printer
		printer = self
		self.printer = serial.Serial(port, baudrate = 115200, timeout = 0)
		# Discard pending data.
		while self.printer.read() != '':
			pass
		self.jobqueue = {}
		self.job_output = ''
		self.jobs_active = []
		self.jobs_ref = None
		self.jobs_angle = 0
		self.jobs_probemap = None
		self.job_current = 0
		self.job_id = None
		self.status = None
		self.confirm_id = 0
		self.position_valid = False
		self.probing = False
		self.home_phase = None
		self.home_cb = [False, self._do_home]
		self.probe_cb = [False, None]
		self.gcode = None
		self.gcode_id = None
		self.gcode_wait = None
		self.gcode_parking = False
		self.gcode_waiting = 0
		self.confirmer = None
		self.queue = []
		self.queue_pos = 0
		self.queue_info = None
		self.resuming = False
		self.flushing = False
		self.initialized = False
		self.debug_buffer = None
		self.printer_buffer = ''
		self.command_buffer = ''
		self.audiofile = None
		# Set up state.
		self.sending = False
		self.paused = False
		self.ff_in = False
		self.ff_out = False
		self.limits = []
		self.sense = []
		self.wait = False
		self.waitaudio = False
		self.audioid = None
		self.movewait = 0
		self.movecb = []
		self.tempcb = []
		self.alarms = set()
		self.audiodir = audiodir
		# The printer may be resetting.  If it is, it will send data
		# when it's done.  If not, request some data.
		for i in range(15):
			#log('writing ID')
			self.printer.write(self.single['ID'])
			ret = select.select([self.printer], [], [self.printer], 1)
			#log(repr(ret))
			if len(ret[0]) != 0:
				break
		else:
			log("Printer doesn't respond: giving up.")
			sys.exit(0)
		time.sleep(.150)	# Make sure data is received.
		# Discard pending data.
		while self.printer.read() != '':
			pass
		# Send several ping commands, to get the flip flops synchronized.
		# If the output flipflop is out of sync, the first ping will not generate a reply.
		# If the input flipflop is out of sync, the first reply will be ignored.
		# Whatever happens, the third ping must be recognized by both sides.
		# But first send an ack, so any transmission in progress is considered finished.
		# This does no harm, because spurious acks are ignored (and the current state is not precious).
		self.printer.write(self.single['ACK0'])
		self.printer.write(self.single['ACK1'])
		self._send_packet(struct.pack('<BB', self.command['PING'], 0))
		self._send_packet(struct.pack('<BB', self.command['PING'], 1))
		self._send_packet(struct.pack('<BB', self.command['PING'], 2))
		t = time.time()
		while time.time() - t < 15:
			reply = self._get_reply()
			if reply is None:
				log('No ping reply: giving up.')
				sys.exit(0)
			if reply[0] == self.rcommand['PONG'] and ord(reply[1]) == 2:
				break
		else:
			log('Timeout waiting for pongs: giving up.')
			sys.exit(0)
		# Now we're in sync.
		# Get the printer state.
		self.printerid = newid
		if not self._begin(newid):
			log('Incorrect reply to BEGIN, probably due to version mismatch.  Giving up.')
			sys.exit(0)
		self.spaces = []
		self.temps = []
		self.gpios = []
		self._read_globals(False)
		for i, s in enumerate(self.spaces):
			s.read(self._read('SPACE', i))
		for i, t in enumerate(self.temps):
			t.read(self._read('TEMP', i))
			# Disable heater.
			self.settemp(i, float('nan'))
			# Disable heater alarm.
			self.waittemp(i, None, None)
		for i, g in enumerate(self.gpios):
			g.read(self._read('GPIO', i))
		# The printer may still be doing things.  Pause it and send a move; this will discard the queue.
		self.pause(True, False)
		#self.load()
		self.initialized = True
		global show_own_debug
		if show_own_debug is None:
			show_own_debug = True
	# }}}
	# Constants.  {{{
	# Masks for computing checksums.
	mask = [	[0xc0, 0xc3, 0xff, 0x09],
			[0x38, 0x3a, 0x7e, 0x13],
			[0x26, 0xb5, 0xb9, 0x23],
			[0x95, 0x6c, 0xd5, 0x43],
			[0x4b, 0xdc, 0xe2, 0x83]]
	# Single-byte commands.  See firmware/serial.cpp for an explanation of the codes.
	single = { 'NACK': '\x80', 'ACK0': '\xb3', 'ACKWAIT0': '\xb4', 'STALL': '\x87', 'ACKWAIT1': '\x99', 'ID': '\xaa', 'ACK1': '\xad', 'DEBUG': '\x9e' }
	command = {
			'BEGIN': 0x00,
			'PING': 0x01,
			'RESET': 0x02,
			'GOTO': 0x03,
			'GOTOCB': 0x04,
			'SLEEP': 0x05,
			'SETTEMP': 0x06,
			'WAITTEMP': 0x07,
			'READTEMP': 0x08,
			'READPOWER': 0x09,
			'SETPOS': 0x0a,
			'GETPOS': 0x0b,
			'LOAD': 0x0c,
			'SAVE': 0x0d,
			'READ_GLOBALS': 0x0e,
			'WRITE_GLOBALS': 0x0f,
			'READ_SPACE_INFO': 0x10,
			'READ_SPACE_AXIS': 0x11,
			'READ_SPACE_MOTOR': 0x12,
			'WRITE_SPACE_INFO': 0x13,
			'WRITE_SPACE_AXIS': 0x14,
			'WRITE_SPACE_MOTOR': 0x15,
			'READ_TEMP': 0x16,
			'WRITE_TEMP': 0x17,
			'READ_GPIO': 0x18,
			'WRITE_GPIO': 0x19,
			'QUEUED': 0x1a,
			'READPIN': 0x1b,
			'AUDIO_SETUP': 0x1c,
			'AUDIO_DATA': 0x1d}
	rcommand = {
			'START': '\x1e',
			'TEMP': '\x1f',
			'POWER': '\x20',
			'POS': '\x21',
			'DATA': '\x22',
			'PONG': '\x23',
			'PIN': '\x24',
			'QUEUE': '\x25',
			'MOVECB': '\x26',
			'TEMPCB': '\x27',
			'CONTINUE': '\x28',
			'LIMIT': '\x29',
			'AUTOSLEEP': '\x2a',
			'SENSE': '\x2b'}
	# }}}
	def _broadcast(self, *a): # {{{
		self._send(None, 'broadcast', *a)
	# }}}
	def _close(self): # {{{
		self.printer.close()
		log('disconnecting')
		self._send(None, 'disconnect')
		waiting_commands = ''
		while True:
			select.select([sys.stdin], [], [sys.stdin])
			self.command_buffer += sys.stdin.read()
			while '\n' in self.command_buffer:
				pos = self.command_buffer.index('\n')
				ln = self.command_buffer[:pos]
				self.command_buffer = self.command_buffer[pos + 1:]
				id, func, a, ka = json.loads(ln)
				if func == 'reconnect':
					self.printer = serial.Serial(a[0], baudrate = 115200, timeout = 0)
					self.command_buffer = waiting_commands + self.command_buffer
					self.printer.write(self.single['NACK'])	# Just to be sure.
					self._send(id, 'return', None)
					return
				elif func in ('export_settings', 'die'):
					ret = getattr(self, func)(*a, **ka)
					if ret == (WAIT, WAIT):
						# Special case: request to die.
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
				self._close()
				# _close will return when the connection returns.
	# }}}
	def _printer_write(self, *a, **ka): # {{{
		while True:
			try:
				return self.printer.write(*a, **ka)
			except:
				log('error writing')
				traceback.print_exc()
				self._close()
				# _close will return when the connection returns.
	# }}}
	def _command_input(self): # {{{
		data = sys.stdin.read()
		if data == '':
			log('End of file detected on command input; exiting.')
			sys.exit(0)
		self.command_buffer += data
		die = False
		while '\n' in self.command_buffer:
			pos = self.command_buffer.index('\n')
			id, func, a, ka = json.loads(self.command_buffer[:pos])
			#log('command: %s' % repr((id, func, a, ka)))
			self.command_buffer = self.command_buffer[pos + 1:]
			try:
				ret = getattr(self, func)(*a, **ka)
				if isinstance(ret, tuple) and len(ret) == 2 and ret[0] is WAIT:
					# The function blocks; it will send its own reply later.
					if ret[1] is WAIT:
						# Special case: request to die.
						die = True
					else:
						ret[1](id)
						continue
			except:
				traceback.print_exc()
				self._send(id, 'error', repr(sys.exc_info()))
				continue
			if not die:
				self._send(id, 'return', ret)
		if die:
			sys.exit(0)
	# }}}
	def _printer_input(self, ack = False, reply = False): # {{{
		for input_limit in range(50):
			while self.debug_buffer is not None:
				r = self._printer_read(1)
				if r == '':
					return ('no data', None)
				if r == '\x00':
					if show_firmware_debug:
						for ln in self.debug_buffer.split(';'):
							log('Debug: %s' % ln)
					self.debug_buffer = None
				else:
					self.debug_buffer += r
			if len(self.printer_buffer) == 0:
				r = self._printer_read(1)
				if r != self.single['DEBUG']:
					dprint('(1) read', r)
				if r == '':
					return ('no data', None)
				if r == self.single['DEBUG']:
					self.debug_buffer = ''
					continue
				if r == self.single['ACK0']:
					# Ack is special in that if it isn't expected, it is not an error.
					if ack:
						return ('ack', 'ack0')
					continue
				if r == self.single['ACK1']:
					# Ack is special in that if it isn't expected, it is not an error.
					if ack:
						return ('ack', 'ack1')
					continue
				if r == self.single['NACK']:
					# Nack is special in that it should only be handled if an ack is expected.
					if ack:
						return ('ack', 'nack')
					continue
				if r == self.single['ACKWAIT0']:
					if ack:
						return ('ack', 'wait0')
					continue
				if r == self.single['ACKWAIT1']:
					if ack:
						return ('ack', 'wait1')
					continue
				if r == self.single['STALL']:
					# There is a problem we can't solve.
					if ack:
						return ('ack', 'stall')
					else:
						log('printer sent unsolicited stall')
						sys.exit(0)
				if r == self.single['ID']:
					# The printer has reset, or some extra data was received after reconnect.
					# Check which by reading the id.
					buffer = ''
					while len(buffer) < 8:
						ret = select.select([self.printer], [], [self.printer], .100)
						if self.printer not in ret[0]:
							# Something was wrong with the packet; ignore all.
							break
						ret = self.printer.read(1)
						if ret == '':
							break
						buffer += ret
					else:
						# ID has been received.
						if buffer == '\x00' * 8:
							# Printer has reset.
							self._broadcast(None, 'reset')
							log('printer reset')
							sys.exit(0)
						# Regular id has been sent; ignore.
						continue
				if(ord(r) & 0x80) != 0 or ord(r) in (0, 1):
					dprint('writing(1)', self.single['NACK'])
					self._printer_write(self.single['NACK'])
					continue
				# Regular packet.
				self.printer_buffer = r
			packet_len = ord(self.printer_buffer[0]) + (ord(self.printer_buffer[0]) + 2) / 3
			while True:
				r = self._printer_read(packet_len - len(self.printer_buffer))
				dprint('rest of packet read', r)
				if r == '':
					return (None, None)
				self.printer_buffer += r
				if len(self.printer_buffer) >= packet_len:
					break
				ret = select.select([self.printer], [], [self.printer], .100)
				if self.printer not in ret[0]:
					dprint('writing(5)', self.single['NACK']);
					self._printer_write(self.single['NACK'])
					self.printer_buffer = ''
					if self.debug_buffer is not None:
						if show_firmware_debug:
							log('Debug(partial): %s' % self.debug_buffer)
						self.debug_buffer = None
					return (None, None)
			packet = self._parse_packet(self.printer_buffer)
			self.printer_buffer = ''
			if packet is None:
				dprint('writing(4)', self.single['NACK']);
				self._printer_write(self.single['NACK'])
				continue	# Start over.
			if bool(ord(packet[0]) & 0x80) != self.ff_in:
				# This was a retry of the previous packet; accept it and retry.
				dprint('(2) writing', self.single['ACK0'] if self.ff_in else self.single['ACK1']);
				self._printer_write(self.single['ACK0'] if self.ff_in else self.single['ACK1'])
				continue	# Start over.
			# Packet successfully received.
			dprint('(3) writing', self.single['ACK1'] if self.ff_in else self.single['ACK0']);
			self._printer_write(self.single['ACK1'] if self.ff_in else self.single['ACK0'])
			# Clear the flip flop.
			packet = chr(ord(packet[0]) & ~0x80) + packet[1:]
			# Flip it.
			self.ff_in = not self.ff_in
			# Handle the asynchronous events.
			if packet[0] == self.rcommand['MOVECB']:
				num = ord(packet[1])
				#log('movecb %d/%d (%d in queue)' % (num, self.movewait, len(self.movecb)))
				if self.initialized:
					if self.movewait < num:
						log('More cbs received than requested!')
						self.movewait = 0
					else:
						self.movewait -= num
					if self.movewait == 0:
						call_queue.extend([(x[1], [True]) for x in self.movecb])
						self.movecb = []
						if self.flushing and self.queue_pos >= len(self.queue):
							#log('done flushing')
							self.flushing = 'done'
							call_queue.append((self._do_gcode, []))
					else:
						log('cb seen, but waiting for more')
				else:
					self.movewait = 0
				continue
			if packet[0] == self.rcommand['TEMPCB']:
				if not self.initialized:
					# Ignore this while connecting.
					continue
				t = ord(packet[1])
				self.alarms.add(t)
				t = 0
				while t < len(self.tempcb):
					if self.tempcb[t][0] is None or self.tempcb[t][0] in self.alarms:
						call_queue.append((self.tempcb.pop(t)[1], []))
					else:
						t += 1
				continue
			elif packet[0] == self.rcommand['CONTINUE']:
				if not self.initialized:
					# Ignore this while connecting.
					continue
				if packet[1] == '\x00':
					# Move continue.
					self.wait = False
					#log('resuming queue %d' % len(self.queue))
					call_queue.append((self._do_queue, []))
					if self.flushing is None:
						self.flushing = False
						call_queue.append((self._do_gcode, []))
				else:
					# Audio continue.
					self.waitaudio = False
					self._audio_play()
				continue
			elif packet[0] == self.rcommand['LIMIT']:
				if not self.initialized:
					# Ignore this while connecting.
					continue
				space = ord(packet[1])
				motor = ord(packet[2])
				self.limits[space][motor] = struct.unpack('<f', packet[3:])[0]
				continue
			elif packet[0] == self.rcommand['AUTOSLEEP']:
				if not self.initialized:
					# Ignore this while connecting.
					continue
				what = ord(packet[1])
				if what & 1:
					self.position_valid = False
					self._globals_update()
				if what & 2:
					for t in range(len(self.temps)):
						if not math.isnan(self.temps[t].value):
							self.temps[t].value = float('nan')
							self._temp_update(t)
				continue
			elif packet[0] == self.rcommand['SENSE']:
				if not self.initialized:
					# Ignore this while connecting.
					continue
				w = ord(packet[1])
				which = w & 0x7f
				motor = ord(packet[2])
				state = bool(w & 0x80)
				pos = struct.unpack('<f', packet[3:])[0]
				if motor not in self.sense[which]:
					self.sense[which][motor] = []
				self.sense[which][motor].append((state, pos))
				s = 0
				tocall = []
				while s < len(self.movecb):
					if self.movecb[s][0] is not False and any(x[1] in self.sense[x[0]] for x in self.movecb[s][0]):
						call_queue.append((self.movecb.pop(s)[1], [self.movewait == 0]))
					else:
						s += 1
				continue
			elif not self.initialized and packet[0] == self.rcommand['PONG'] and ord(packet[1]) < 2:
				# PONGs during initialization are possible.
				dprint('ignore pong', packet)
				continue
			if reply:
				return ('packet', packet)
			dprint('unexpected packet', packet)
			if self.initialized:
				raise AssertionError('Received unexpected reply packet')
	# }}}
	def _make_packet(self, data): # {{{
		checklen = len(data) / 3 + 1
		fulldata = list(chr(len(data) + 1) + data + '\x00' * checklen)
		for t in range(checklen):
			check = t & 0x7
			for bit in range(5):
				sum = check & Printer.mask[bit][3]
				for byte in range(3):
					sum ^= ord(fulldata[3 * t + byte]) & Printer.mask[bit][byte]
				sum ^= sum >> 4
				sum ^= sum >> 2
				sum ^= sum >> 1
				if sum & 1:
					check |= 1 <<(bit + 3)
			fulldata[len(data) + 1 + t] = chr(check)
		#log(' '.join(['%02x' % ord(x) for x in fulldata]))
		return ''.join(fulldata)
	# }}}
	def _parse_packet(self, data): # {{{
		#log(' '.join(['%02x' % ord(x) for x in data]))
		if ord(data[0]) + (ord(data[0]) + 2) / 3 != len(data):
			return None
		length = ord(data[0])
		checksum = data[length:]
		for t in range(len(checksum)):
			r = data[3 * t:3 * t + 3] + checksum[t]
			if length in (2, 4):
				r = r[0:2] + '\x00' + r[3:]
			if(ord(checksum[t]) & 0x7) != (t & 7):
				return None
			for bit in range(5):
				sum = 0
				for byte in range(4):
					sum ^= ord(r[byte]) & Printer.mask[bit][byte]
				sum ^= sum >> 4
				sum ^= sum >> 2
				sum ^= sum >> 1
				if(sum & 1) != 0:
					return None
		return data[1:length]
	# }}}
	def _send_packet(self, data, audio = False): # {{{
		if self.ff_out:
			data = chr(ord(data[0]) | 0x80) + data[1:]
		self.ff_out = not self.ff_out
		data = self._make_packet(data)
		maxtries = 10
		while maxtries > 0:
			dprint('(1) writing', data);
			self._printer_write(data)
			while True:
				ack = None
				ret = select.select([self.printer], [], [self.printer], .300)
				if self.printer not in ret[0] and self.printer not in ret[2]:
					# No response; retry.
					log('no response waiting for ack')
					maxtries -= 1
					break
				ack = self._printer_input(ack = True)
				if ack[0] != 'ack':
					#log('no response yet waiting for ack')
					maxtries -= 1
					continue
				if ack[1] == 'nack':
					log('nack response waiting for ack')
					maxtries -= 1
					continue
				elif (not self.ff_out and ack[1] in ('ack0', 'wait0')) or (self.ff_out and ack[1] in ('ack1', 'wait1')):
					log('wrong ack response waiting for ack')
					traceback.print_stack()
					maxtries -= 1
					continue
				elif ack[1] in ('ack0', 'ack1'):
					return True
				elif ack[1] in ('wait0', 'wait1'):
					#log('ackwait response waiting for ack')
					if audio:
						self.waitaudio = True
					else:
						self.wait = True
					return True
				else: # ack[1] == 'stall'
					log('stall response waiting for ack')
					self._unpause()
					self._print_done(False, 'printer sent stall')
					return False
		return False
	# }}}
	def _get_reply(self, cb = False): # {{{
		while self.printer is not None:
			ret = select.select([self.printer], [], [self.printer], .150)
			if len(ret[0]) == 0:
				log('no reply received')
				return None
			ret = self._printer_input(reply = True)
			if ret[0] == 'packet' or (cb and ret[0] == 'no data'):
				return ret[1]
			#log('no response yet waiting for reply')
	# }}}
	def _begin(self, newid): # {{{
		if not self._send_packet(struct.pack('<BL', self.command['BEGIN'], 0) + newid):
			return False
		return struct.unpack('<BL', self._get_reply()) == (ord(self.rcommand['START']), 0)
	# }}}
	def _read(self, cmd, channel, sub = None): # {{{
		if cmd == 'SPACE':
			info = self._read('SPACE_INFO', channel)
			self.spaces[channel].type, self.spaces[channel].max_deviation = struct.unpack('<Bf', info[:5])
			if self.spaces[channel].type == TYPE_EXTRUDER:
				self.spaces[channel].extruder[0], self.spaces[channel].extruder[1], self.spaces[channel].extruder[2], num_axes = struct.unpack('<fffB', info[5:])
				num_motors = num_axes
			elif self.spaces[channel].type == TYPE_CARTESIAN:
				num_axes = struct.unpack('<B', info[5:])[0]
				num_motors = num_axes
			elif self.spaces[channel].type == TYPE_DELTA:
				self.spaces[channel].delta = [{}, {}, {}]
				for a in range(3):
					self.spaces[channel].delta[a]['axis_min'], self.spaces[channel].delta[a]['axis_max'], self.spaces[channel].delta[a]['rodlength'], self.spaces[channel].delta[a]['radius'] = struct.unpack('<ffff', info[5 + 16 * a:5 + 16 * (a + 1)])
				self.spaces[channel].delta_angle = struct.unpack('<f', info[5 + 16 * 3:])[0]
				num_axes = 3
				num_motors = 3
			else:
				log('invalid type')
				raise AssertionError('invalid space type')
			self.spaces[channel].axis = []
			return ([self._read('SPACE_AXIS', channel, axis) for axis in range(num_axes)], [self._read('SPACE_MOTOR', channel, motor) for motor in range(num_motors)])
		if cmd == 'GLOBALS':
			packet = struct.pack('<B', self.command['READ_' + cmd])
		elif sub is not None and cmd.startswith('SPACE'):
			packet = struct.pack('<BBB', self.command['READ_' + cmd], channel, sub)
		else:
			packet = struct.pack('<BB', self.command['READ_' + cmd], channel)
		if not self._send_packet(packet):
			return None
		reply = self._get_reply()
		assert reply[0] == self.rcommand['DATA']
		return reply[1:]
	# }}}
	def _read_globals(self, update = True): # {{{
		data = self._read('GLOBALS', None)
		if data is None:
			return False
		#log('%d' % len(data[self.namelen:]))
		self.queue_length, self.audio_fragments, self.audio_fragment_size, self.num_pins, self.num_digital_pins, num_spaces, num_temps, num_gpios, namelen = struct.unpack('<BBBBBBBBB', data[:9])
		self.name = unicode(data[9:9 + namelen], 'utf-8', 'replace')
		self.led_pin, self.probe_pin, self.probe_dist, self.probe_safe_dist, self.motor_limit, self.temp_limit, self.feedrate = struct.unpack('<HHfffff', data[9 + namelen:])
		while len(self.spaces) < num_spaces:
			self.spaces.append(self.Space(self, len(self.spaces)))
			if update:
				data = self._read('SPACE', len(self.spaces) - 1)
				self.spaces[-1].read(data)
		self.spaces = self.spaces[:num_spaces]
		while len(self.temps) < num_temps:
			self.temps.append(self.Temp())
			if update:
				data = self._read('TEMP', len(self.temps) - 1)
				self.temps[-1].read(data)
		self.temps = self.temps[:num_temps]
		while len(self.gpios) < num_gpios:
			self.gpios.append(self.Gpio())
			if update:
				data = self._read('GPIO', len(self.gpios) - 1)
				self.gpios[-1].read(data)
		self.gpios = self.gpios[:num_gpios]
		while len(self.limits) < num_spaces:
			self.limits.append({})
			self.sense.append({})
		while len(self.limits) > num_spaces:
			self.limits.pop()
			self.sense.pop()
		return True
	# }}}
	def _write_globals(self, ns, nt, ng): # {{{
		name = self.name.encode('utf-8')
		data = struct.pack('<BBBB', ns if ns is not None else len(self.spaces), nt if nt is not None else len(self.temps), ng if ng is not None else len(self.gpios), len(name)) + name + struct.pack('<HHfffff', self.led_pin, self.probe_pin, self.probe_dist, self.probe_safe_dist, self.motor_limit, self.temp_limit, self.feedrate)
		if not self._send_packet(struct.pack('<B', self.command['WRITE_GLOBALS']) + data):
			return False
		self._read_globals()
		self._globals_update()
		return True
	# }}}
	def _globals_update(self, target = None): # {{{
		if not self.initialized:
			return
		self._broadcast(target, 'globals_update', [len(self.spaces), len(self.temps), len(self.gpios), self.name, self.led_pin, self.probe_pin, self.probe_dist, self.probe_safe_dist, self.motor_limit, self.temp_limit, self.feedrate, not self.paused and (None if self.gcode is None else True)])
	# }}}
	def _space_update(self, which, target = None): # {{{
		if not self.initialized:
			return
		self._broadcast(target, 'space_update', which, self.spaces[which].export())
	# }}}
	def _temp_update(self, which, target = None): # {{{
		if not self.initialized:
			return
		self._broadcast(target, 'temp_update', which, self.temps[which].export())
	# }}}
	def _gpio_update(self, which, target = None): # {{{
		if not self.initialized:
			return
		self._broadcast(target, 'gpio_update', which, self.gpios[which].export())
	# }}}
	def _use_probemap(self, x, y, z): # {{{
		'''Return corrected z according to self.gcode_probemap.'''
		# Map = [[x0, y0, x1, y1], [nx, ny], [[...], [...], ...]]
		if self.gcode_probemap is None or any(math.isnan(t) for t in (x, y, z)):
			return z
		p = self.gcode_probemap
		x -= p[0][0]
		y -= p[0][1]
		x /= (p[0][2] - p[0][0]) / p[1][0]
		y /= (p[0][3] - p[0][1]) / p[1][1]
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
	def _do_gcode(self): # {{{
		if self.gcode is None:
			return
		flushed = False
		if self.flushing == 'done':
			self.flushing = False
			flushed = True
		while len(self.gcode) > 0:
			if self.gcode_parking or self.gcode_waiting > 0 or self.confirmer is not None or self.gcode_wait is not None or self.flushing or self.paused:
				#log('gcode waiting: %s' % repr((self.gcode_parking, self.gcode_waiting, self.confirmer, self.gcode_wait)))
				# Wait until done.
				return
			cmd, args, message = self.gcode[0]
			cmd = tuple(cmd)
			#log('Running %s %s' % (cmd, args))
			if cmd[0] == 'S':
				# Spindle speed; not supported, but shouldn't error.
				pass
			elif cmd == ('G', 94):
				# Set feed rate mode to units per minute; we don't support anything else, but shouldn't error on this request.
				pass
			elif cmd == ('M', 9):
				# Coolant off.  We don't support coolant, but turning it off is not an error.
				pass
			elif cmd == ('G', 1):
				if self.flushing is None:
					#log('not filling; waiting for queue space')
					return
				#log(repr(args))
				sina, cosa = self.gcode_angle
				target = cosa * args['X'] - sina * args['Y'] + self.gcode_ref[0], cosa * args['Y'] + sina * args['X'] + self.gcode_ref[1]
				if self._use_probemap and self.gcode_probemap:
					if args['x'] is not None:
						source = cosa * args['x'] - sina * args['y'] + self.gcode_ref[0], cosa * args['y'] + sina * args['x'] + self.gcode_ref[1]
						#log('gcode %s' % repr([(target[t], source[t], self.gcode_probemap[0][t + 2], self.gcode_probemap[0][t], self.gcode_probemap[1][t]) for t in range(2)]))
						nums = [abs(target[t] - source[t]) / (abs(self.gcode_probemap[0][t + 2] - self.gcode_probemap[0][t]) / self.gcode_probemap[1][t]) for t in range(2) if not math.isnan(target[t]) and self.gcode_probemap[0][t] != self.gcode_probemap[0][t + 2]]
						#log('num %s' % nums)
						if len(nums) == 0 or all(math.isnan(num) for num in nums):
							num = 1
						else:
							num = int(max(n for n in nums if not math.isnan(n))) + 1
						if num == 1:
							#log('debugpart: %.2f %.2f %.2f %.2f' % (target[0] * 1e3, target[1] * 1e3, args['f'], args['F']))
							z = self._use_probemap(target[0], target[1], args['Z'])
							log('go to one %f %f %f' % (target[0], target[1], z))
							self.goto([[target[0], target[1], z], [args['E']]], f0 = args['f'], f1 = args['F'])[1](None)
						else:
							for t in range(num):
								targetpart = [source[tt] + (target[tt] - source[tt]) * (t + 1.) / num for tt in range(2)]
								#log('debugpart: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f' % (target[0] * 1e3, target[1] * 1e3, source[0] * 1e3, source[1] * 1e3, targetpart[0] * 1e3, targetpart[1] * 1e3, args['f'], args['F']))
								z = self._use_probemap(targetpart[0], targetpart[1], args['Z'])
								log('go to part %f %f %f' % (targetpart[0], targetpart[1], z))
								self.goto([[targetpart[0], targetpart[1], z], [args['E']]], f0 = args['f'] * num, f1 = args['F'] * num)[1](None)
					else:
						self.goto([[target[0], target[1], self._use_probemap(target[0], target[1], args['Z'])], [args['E']]], f0 = args['f'], f1 = args['F'])[1](None)
				else:
					self.goto([[target[0], target[1], args['Z']], [args['E']]], f0 = args['f'], f1 = args['F'])[1](None)
				if self.wait and self.flushing is False:
					#log('stop filling; wait for queue space')
					self.flushing = None
					self.gcode.pop(0)
					continue
			else:
				if not flushed:
					self.flush()[1](None)
					self.flushing = True
					return
				flushed = False
				if cmd in (('G', 28), ('M', 6)):
					# Home or tool change; same response: park
					self.gcode.pop(0)
					self.gcode_parking = True
					return self.park(self._do_gcode, abort = False)[1](None)
				elif cmd == ('G', 4):
					if 'P' in args:
						self.gcode.pop(0)
						self.gcode_wait = time.time() + float(args['P']) / 1000
						return
				elif cmd == ('G', 92):
					# Don't allow g-code to disable motors.
					#self.sleep()
					# But do set the position to 0, so things work as expected.
					self.set_axis_pos(1, args['e'], args['E'])
				elif cmd == ('M', 0):
					# Wait.
					self.gcode.pop(0)
					self.request_confirmation(message or 'Continue?')[1](False)
					return
				elif cmd[0] == 'M' and cmd[1] in (3, 4):
					# Start spindle; we don't support speed or direction, so M3 and M4 are equal.
					if len(self.gpios) > 1:
						self.set_gpio(1, state = 1)
				elif cmd == ('M', 5):
					# Stop spindle.
					if len(self.gpios) > 1:
						self.set_gpio(1, state = 0)
				elif cmd == ('M', 42):
					g = int(args['P'])
					if len(self.gpios) > g:
						value = args['S']
						self.set_gpio(int(g), state = int(value))
				elif cmd == ('M', 84):
					# Don't allow g-code to disable motors.
					#self.sleep()
					# But do set the position to 0, so things work as expected.
					for e in range(len(self.spaces[1].axis)):
						self.set_axis_pos(1, e, 0)
				elif cmd == ('M', 104):
					self.settemp(1 + args['E'], args['S'])
				elif cmd == ('M', 106):	# Fan on
					if len(self.gpios) > 0:
						self.set_gpio(0, state = 1)
				elif cmd == ('M', 107): # Fan off
					if len(self.gpios) > 0:
						self.set_gpio(0, state = 0)
				elif cmd == ('M', 109):
					e = args['E']
					if 'S' in args:
						self.settemp(1 + e, args['S'])
					else:
						args['S'] = self.temps[1 + e].value
					if not math.isnan(args['S']) and self.pin_valid(self.temps[1 + e].thermistor_pin):
						self.clear_alarm()
						self.waittemp(1 + e, args['S'])
						self.gcode.pop(0)
						self.gcode_waiting += 1
						return self.wait_for_temp(1 + e)[1](None)
				elif cmd == ('M', 116):
					e = args['E']
					etemp = self.temps[1 + e].value
					self.clear_alarm()
					if not math.isnan(etemp) and self.pin_valid(self.temps[1 + e].thermistor_pin):
						self.waittemp(1 + e, etemp)
						self.gcode.pop(0)
						self.gcode_waiting += 1
						self.wait_for_temp(1 + e)[1](None)
					if not math.isnan(self.btemp) and self.pin_valid(self.temps[0].thermistor_pin):
						self.waittemp(0, self.btemp)
						self.gcode.pop(0)
						self.gcode_waiting += 1
						self.wait_for_temp(0)[1](None)
					if self.gcode_waiting > 0:
						return
				elif cmd == ('M', 140):
					self.btemp = args['S']
					self.settemp(0, self.btemp)
				elif cmd == ('M', 190):
					if 'S' in args:
						self.btemp = args['S']
						self.settemp(0, self.btemp)
					if not math.isnan(self.btemp) and self.pin_valid(self.temps[0].thermistor_pin):
						self.gcode.pop(0)
						self.waittemp(0, self.btemp)
						self.gcode_waiting = True
						return self.wait_for_temp(0)[1](None)
				elif cmd == ('SYSTEM', 0):
					log('running system command: %s' % message)
					os.system(message)
			if len(self.gcode) > 0:
				self.gcode.pop(0)
			else:
				# Printing was aborted; don't call print_done, or we'll abort whatever aborted us.
				return
		if not flushed:
			self.flush()[1](None)
			self.flushing = True
			return
		if len(self.spaces) > 1:
			for e in range(len(self.spaces[1].axis)):
				self.set_axis_pos(1, e, 0)
		self._print_done(True, 'completed')
	# }}}
	def _print_done(self, complete, reason): # {{{
		log('abort: %d %s' % (complete, reason))
		self.gcode = None
		traceback.print_stack()
		if self.queue_info is None and self.gcode_id is not None:
			log('Print done (%d): %s' % (complete, reason))
			self._send(self.gcode_id, 'return', (complete, reason))
		self.gcode_id = None
		self.gcode_wait = None
		#log('job %s' % repr(self.jobs_active))
		if self.queue_info is None and len(self.jobs_active) > 0:
			if complete:
				if self.job_current >= len(self.jobs_active) - 1:
					#log('job queue done')
					self._send(self.job_id, 'return', (True, reason))
					self.job_id = None
					self.jobs_active = []
				else:
					#log('job part done; moving on')
					self._next_job()
			else:
				#log('job aborted')
				self._send(self.job_id, 'return', (False, reason))
				self.job_id = None
				self.jobs_active = []
		while self.queue_pos < len(self.queue):
			id, axes, e, f0, f1, which, cb = self.queue[self.queue_pos]
			self.queue_pos += 1
			if id is not None:
				self._send(id, 'error', 'aborted')
		self.queue = []
		self.queue_pos = 0
		if self.home_phase is not None:
			#log('killing homer')
			self.home_phase = None
			self.set_space(self.home_space, type = self.home_orig_type)
			for i in range(len(self.spaces[self.home_space].axis)):
				self.set_axis((self.home_space, i), min = self.home_orig_limits[i][0], max = self.home_orig_limits[i][1])
				self.set_motor((self.home_space, i), max_steps = self.home_orig_limits[1][i])
			if self.home_cb in self.movecb:
				self.movecb.remove(self.home_cb)
				self._send(self.home_id, 'return', None)
		if self.probe_cb in self.movecb:
			#log('killing prober')
			self.movecb.remove(self.probe_cb)
			self.probe_cb(False)
	# }}}
	def _unpause(self): # {{{
		if self.queue_info is None:
			return
		#log('doing resume to %d/%d' % (self.queue_info[0], len(self.queue_info[2])))
		self.queue = self.queue_info[2]
		self.queue_pos = self.queue_info[0]
		self.movecb = self.queue_info[3]
		self.flushing = self.queue_info[4]
		self.gcode_wait = self.queue_info[5]
		self.resuming = False
		self.queue_info = None
		self.paused = False
		self._globals_update()
		call_queue.append((self._do_gcode, []))
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
				if self.queue_pos >= len(self.queue):
					break
			id, axes, f0, f1, cb = self.queue[self.queue_pos]
			#log('queueing %s' % repr((id, axes, f0, f1, cb)))
			self.queue_pos += 1
			a = {}
			a0 = 0
			i = 0
			if isinstance(axes, (list, tuple)):
				adict = {}
				for s, data in enumerate(axes):
					adict[s] = data
				axes = adict
			adict = {}
			for k in axes:
				adict[int(k)] = axes[k]
			axes = adict
			for i, sp in enumerate(self.spaces):
				if i not in axes or axes[i] is None:
					a0 += len(sp.axis)
					continue
				if isinstance(axes[i], (list, tuple)):
					assert len(axes[i]) <= len(sp.axis)
					for j, axis in enumerate(axes[i]):
						ij = int(j)
						if axis is not None and not math.isnan(axis):
							# Limit values for axis.
							if axis > sp.axis[ij]['max'] - sp.axis[ij]['offset']:
								axis = sp.axis[ij]['max'] - sp.axis[ij]['offset']
							if axis < sp.axis[ij]['min'] - sp.axis[ij]['offset']:
								axis = sp.axis[ij]['min'] - sp.axis[ij]['offset']
							a[a0 + ij] = axis
							#log('current pos: %f' % self.spaces[i].get_current_pos(j))
				else:
					for j, axis in tuple(axes[i].items()):
						ij = int(j)
						assert ij <= len(sp.axis)
						if axis is not None and not math.isnan(axis):
							# Limit values for axis.
							if axis > sp.axis[ij]['max'] - sp.axis[ij]['offset']:
								axis = sp.axis[ij]['max'] - sp.axis[ij]['offset']
							if axis < sp.axis[ij]['min'] - sp.axis[ij]['offset']:
								axis = sp.axis[ij]['min'] - sp.axis[ij]['offset']
							a[a0 + ij] = axis
				a0 += len(sp.axis)
			targets = [0] * (((2 + a0 - 1) >> 3) + 1)
			axes = a
			args = ''
			if f0 is None:
				f0 = float('inf')
			if f1 is None:
				f1 = f0
			if f0 != float('inf'):
				targets[0] |= 1 << 0
				args += struct.pack('<f', f0)
			if f1 != f0:
				targets[0] |= 1 << 1
				args += struct.pack('<f', f1)
			a = list(axes.keys())
			a.sort()
			#log('f0: %f f1: %f' %(f0, f1))
			for axis in a:
				if math.isnan(axes[axis]):
					continue
				targets[(axis + 2) >> 3] |= 1 <<((axis + 2) & 0x7)
				args += struct.pack('<f', axes[axis])
				#log('axis %d: %f' %(axis, axes[axis]))
			if cb:
				self.movewait += 1
				#log('movewait +1 -> %d' % self.movewait)
				p = chr(self.command['GOTOCB'])
			else:
				p = chr(self.command['GOTO'])
			#log('queueing %s' % repr((axes, f0, f1, cb, self.flushing)))
			self._send_packet(p + ''.join([chr(t) for t in targets]) + args)
			if id is not None:
				self._send(id, 'return', None)
			if self.flushing is None:
				self.flushing = False
				call_queue.append((self._do_gcode, []))
		#log('queue done %s' % repr((self.queue_pos, len(self.queue), self.resuming, self.wait)))
	# }}}
	def _do_home(self, done = None): # {{{
		#log('do_home: %s %s' % (self.home_phase, done))
		# 0: Prepare for next order.
		# 1: Move to max limit switch or find sense switch.
		# 2: Move to min limit switch or find sense switch.
		# 3: Back off.
		# 4: Move slowly to switch.
		# 5: Repeat until all home orders are done.
		# 6: Set current position; move to center (delta only).
		small_dist = .005
		if self.home_phase is None:
			# Initial call; start homing.
			self.home_phase = 0
			self.home_orig_type = self.spaces[self.home_space].type
			self.home_orig_limits = [[(a['min'], a['max']) for a in self.spaces[self.home_space].axis], [m['max_steps'] for m in self.spaces[self.home_space].motor]]
			self.set_space(self.home_space, type = TYPE_CARTESIAN)
			for i, m in enumerate(self.spaces[self.home_space].motor):
				self.set_motor((self.home_space, i), max_steps = 1)
				self.set_axis((self.home_space, i), min = m['motor_min'], max = m['motor_max'])
			# If it is currently moving, doing the things below without pausing causes stall responses.
			self.pause(True, False)
			self.sleep(False)
			n = [m['home_order'] for m in self.spaces[self.home_space].motor]
			if len(n) == 0:
				self.home_phase = 6
			else:
				self.home_order = min(n)
			# Fall through.
		if self.home_phase == 0:
			# Move up to find limit or sense switch.
			self.home_phase = 1
			self.home_motors = [(i, m) for i, m in enumerate(self.spaces[self.home_space].motor) if m['home_order'] == self.home_order]
			self.home_target = {}
			self.limits[self.home_space].clear()
			self.sense[self.home_space].clear()
			for i, m in self.home_motors:
				if self.pin_valid(m['limit_max_pin']) or (not self.pin_valid(m['limit_min_pin']) and self.pin_valid(m['sense_pin'])):
					dist = m['home_pos'] + .01 - m['motor_min']
					self.spaces[self.home_space].set_current_pos(i, m['motor_max'] - dist - self.spaces[self.home_space].axis[i]['offset'])
					self.home_target[i] = m['motor_max'] - self.spaces[self.home_space].axis[i]['offset']
			if len(self.home_target) > 0:
				self.home_cb[0] = [(self.home_space, k) for k in self.home_target.keys()]
				self.movecb.append(self.home_cb)
				#log("N t %s" % (self.home_target))
				self.goto({self.home_space: self.home_target}, cb = True)[1](None)
				return
			# Fall through.
		if self.home_phase == 1:
			# Continue moving up to find limit or sense switch.
			self.pause(True, False)	# Needed if we hit a sense switch.
			found_limits = False
			for a in self.limits[self.home_space].keys() + self.sense[self.home_space].keys():
				if a in self.home_target:
					self.home_target.pop(a)
					found_limits = True
			# Repeat until move is done, or all limits are hit.
			if (not done or found_limits) and len(self.home_target) > 0:
				self.home_cb[0] = [(self.home_space, k) for k in self.home_target.keys()]
				self.movecb.append(self.home_cb)
				#log("0 t %s" % (self.home_target))
				self.goto({self.home_space: self.home_target}, cb = True)[1](None)
				return
			self.home_phase = 2
			# Move down to find limit or sense switch.
			for i, m in self.home_motors:
				if (i not in self.limits[self.home_space] and self.pin_valid(m['limit_min_pin'])) or (i not in self.sense[self.home_space] and self.pin_valid(m['sense_pin'])):
					dist = m['motor_max'] - m['home_pos'] - .01
					self.spaces[self.home_space].set_current_pos(i, m['motor_min'] + dist)
					self.home_target[i] = m['motor_min'] - self.spaces[self.home_space].axis[i]['offset']
			if len(self.home_target) > 0:
				self.home_cb[0] = [(self.home_space, k) for k in self.home_target.keys()]
				self.movecb.append(self.home_cb)
				#log("1 t %s" % (self.home_target))
				self.goto({self.home_space: self.home_target}, cb = True)[1](None)
				return
			# Fall through.
		if self.home_phase == 2:
			# Continue moving down to find limit or sense switch.
			self.pause(True, False)	# Needed if we hit a sense switch.
			found_limits = False
			for a in self.limits[self.home_space].keys() + self.sense[self.home_space].keys():
				if a in self.home_target:
					self.home_target.pop(a)
					found_limits = True
			# Repeat until move is done, or all limits are hit.
			if (not done or found_limits) and len(self.home_target) > 0:
				self.home_cb[0] = [(self.home_space, k) for k in self.home_target.keys()]
				self.movecb.append(self.home_cb)
				#log("2 t %s" % (self.home_target))
				self.goto({self.home_space: self.home_target}, cb = True)[1](None)
				return
			if len(self.home_target) > 0:
				log('Warning: not all limits were found during homing')
			# Back off.
			self.home_phase = 3
			self.home_target = {}
			for i, m in self.home_motors:
				if self.pin_valid(m['limit_max_pin']) or (not self.pin_valid(m['limit_min_pin']) and self.pin_valid(m['sense_pin'])):
					self.spaces[self.home_space].set_current_pos(i, m['motor_min'] + small_dist)
					self.home_target[i] = m['motor_min'] - self.spaces[self.home_space].axis[i]['offset']
				elif self.pin_valid(m['limit_min_pin']):
					self.spaces[self.home_space].set_current_pos(i, m['motor_max'] - small_dist)
					self.home_target[i] = m['motor_max'] - self.spaces[self.home_space].axis[i]['offset']
			if len(self.home_target) > 0:
				self.home_cb[0] = False
				self.movecb.append(self.home_cb)
				#log("back t %s" % (self.home_target))
				self.goto({self.home_space: self.home_target}, cb = True)[1](None)
				return
			# Fall through
		if self.home_phase == 3:
			# Goto switch slowly.
			self.home_phase = 4
			self.home_target = {}
			self.limits[self.home_space].clear()
			self.sense[self.home_space].clear()
			small_dist = .01
			for i, m in self.home_motors:
				if self.pin_valid(m['limit_max_pin']) or (not self.pin_valid(m['limit_min_pin']) and self.pin_valid(m['sense_pin'])):
					self.spaces[self.home_space].set_current_pos(i, m['motor_min'] - 2 * small_dist)
					self.home_target[i] = m['motor_min'] - self.spaces[self.home_space].axis[i]['offset']
				elif self.pin_valid(m['limit_min_pin']):
					self.spaces[self.home_space].set_current_pos(i, m['motor_max'] + 2 * small_dist)
					self.home_target[i] = m['motor_max'] - self.spaces[self.home_space].axis[i]['offset']
			if len(self.home_target) > 0:
				self.home_cb[0] = [(self.home_space, k) for k in self.home_target.keys()]
				self.movecb.append(self.home_cb)
				#log("sp %s t %s" % (self.home_speed, self.home_target))
				self.goto({self.home_space: self.home_target}, f0 = self.home_speed / (2 * small_dist), cb = True)[1](None)
				return
			# Fall through
		if self.home_phase == 4:
			# Continue going to switch slowly.
			self.pause(True, False)	# Needed if we hit a sense switch.
			found_limits = False
			for a in self.limits[self.home_space].keys() + self.sense[self.home_space].keys():
				if a in self.home_target:
					self.home_target.pop(a)
					found_limits = True
			# Repeat until move is done, or all limits are hit.
			if (not done or found_limits) and len(self.home_target) > 0:
				self.home_cb[0] = [(self.home_space, k) for k in self.home_target.keys()]
				self.movecb.append(self.home_cb)
				key = self.home_target.keys()[0]
				dist = abs(self.home_target[key] - self.spaces[self.home_space].get_current_pos(key))
				#log("sp %s d %s t %s" % (self.home_speed, dist, self.home_target))
				self.goto({self.home_space: self.home_target}, f0 = self.home_speed / dist, cb = True)[1](None)
				return
			if len(self.home_target) > 0:
				log('Warning: not all limits were hit during homing')
			self.home_phase = 5
			# Move delta to center.
			if self.home_orig_type == TYPE_DELTA:
				# Goto center.  Since this is only for deltas, assume that switches are not at minimum travel.
				self.home_delta_target = float('nan')
				for i, m in self.home_motors:
					if math.isnan(self.home_delta_target) or m['home_pos'] < self.home_delta_target:
						self.home_delta_target = m['home_pos']
				target = {}
				for i, m in self.home_motors:
					self.spaces[self.home_space].set_current_pos(i, m['motor_min'] + m['home_pos'] - self.home_delta_target)
					target[i] = m['motor_min'] - self.spaces[self.home_space].axis[i]['offset']
				if len(target) > 0:
					self.home_cb[0] = False
					self.movecb.append(self.home_cb)
					self.goto({self.home_space: target}, cb = True)[1](None)
					return
			# Fall through.
		if self.home_phase == 5:
			# set current position.
			for i, m in self.home_motors:
				if i in self.limits[self.home_space]:
					if self.home_orig_type == TYPE_DELTA:
						self.spaces[self.home_space].set_current_pos(i, self.home_delta_target)
					else:
						self.spaces[self.home_space].set_current_pos(i, m['home_pos'])
				elif i in self.sense[self.home_space]:
					# Correct for possible extra steps that were done because pausing happened later than hitting the sensor.
					if self.home_orig_type == TYPE_DELTA:
						self.spaces[self.home_space].set_current_pos(i, m['home_pos'] + self.sense[self.home_space][i][-1][1] - self.spaces[self.home_space].get_current_pos(i))
					else:
						self.spaces[self.home_space].set_current_pos(i, self.home_delta_target + self.sense[self.home_space][i][-1][1] - self.spaces[self.home_space].get_current_pos(i))
			n = [m['home_order'] for m in self.spaces[self.home_space].motor if m['home_order'] > self.home_order]
			if len(n) > 0:
				self.home_phase = 0
				self.home_order = min(n)
				return self._do_home()
			self.home_phase = 6
			# Fall through
		if self.home_phase == 6:
			#log('home 6')
			self.set_space(self.home_space, type = self.home_orig_type)
			for i in range(len(self.spaces[self.home_space].motor)):
				self.set_axis((self.home_space, i), min = self.home_orig_limits[0][i][0], max = self.home_orig_limits[0][i][1])
				self.set_motor((self.home_space, i), max_steps = self.home_orig_limits[1][i])
			target = {}
			for i, a in enumerate(self.spaces[self.home_space].axis):
				current = self.spaces[self.home_space].get_current_pos(i)
				if current > a['max'] - a['offset']:
					target[i] = a['max'] - a['offset']
				elif current < a['min'] - a['offset']:
					target[i] = a['min'] - a['offset']
			self.home_phase = 7
			if len(target) > 0:
				self.home_cb[0] = False
				self.movecb.append(self.home_cb)
				#log('target: %s' % repr(target))
				self.goto({self.home_space: target}, cb = True)[1](None)
				return
			# Fall through.
		if self.home_phase == 7:
			#log('home 7')
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
	def _do_probe(self, id, x, y, z, angle, speed, phase = 0, good = True): # {{{
		# Map = [[x0, y0, x1, y1], [nx, ny], [[...], [...], ...]]
		if not good:
			# This means the probe has been aborted.
			for m in range(len(self.spaces[0].motor)):
				self.set_motor((0, m), limit_min_pin = self.probe_orig[m][0], max_steps = self.probe_orig[m][1])
			self.probing = False
			if id is not None:
				self._send(id, 'error', 'aborted')
			return
		self.probing = True
		p = self.jobs_probemap
		if phase == 0:
			if y > p[1][1]:
				# Done.
				self.probing = False
				for m in range(len(self.spaces[0].motor)):
					self.set_motor((0, m), limit_min_pin = self.probe_orig[m][0], max_steps = self.probe_orig[m][1])
				if id is not None:
					self._send(id, 'return', self.jobs_probemap)
				else:
					for y, c in enumerate(p[2]):
						for x, o in enumerate(c):
							log('map %f %f %f' % (p[0][0] + (p[0][2] - p[0][0]) * x / p[1][0], p[0][1] + (p[0][3] - p[0][1]) * y / p[1][1], o))
					#log('result: %s' % repr(self.jobs_probemap))
					if len(self.jobs_active) == 1:
						def cb():
							self.request_confirmation("Probing done; prepare for job.")[1](False)
							self._next_job()
						self.park(cb = cb, abort = False)[1](None)
					else:
						self._next_job()
				return
			# Goto x,y
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, z, angle, speed, 1, good)
			self.movecb.append(self.probe_cb)
			px = p[0][0] + (p[0][2] - p[0][0]) * x / p[1][0]
			py = p[0][1] + (p[0][3] - p[0][1]) * y / p[1][1]
			self.goto([[px * self.gcode_angle[1] - py * self.gcode_angle[0], py * self.gcode_angle[1] + px * self.gcode_angle[0]]], cb = True)[1](None)
		elif phase == 1:
			# Probe
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, z, angle, speed, 2, good)
			self.movecb.append(self.probe_cb)
			z_low = self.spaces[0].motor[2]['motor_min']
			self.goto([{2: z_low}], f0 = float(speed) / (z - z_low), cb = True)[1](None)
		else:
			# Record result
			z = self.spaces[0].get_current_pos(2)
			p[2][y][x] = z + self.spaces[0].axis[2]['offset']
			z += self.probe_safe_dist
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
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, z, angle, speed, 0, good)
			self.movecb.append(self.probe_cb)
			# Retract
			self.goto([{2: z}], cb = True)[1](None)
	# }}}
	def _next_job(self): # {{{
		self.job_current += 1
		if self.job_current >= len(self.jobs_active):
			self._print_done(True, 'Queue finished')
			return
		if len(self.jobs_active) > 1:
			def cb():
				self.request_confirmation("Prepare for job '%s'." % self.jobs_active[self.job_current])[1](False)
			self.gcode_parking = True
			self.park(cb = cb, abort = False)[1](None)
		self.gcode_run(self.jobqueue[self.jobs_active[self.job_current]][0][:], self.jobs_ref, self.jobs_angle, self.jobs_probemap, abort = False)[1](None)
	# }}}
	def _audio_play(self): # {{{
		if self.audiofile is None:
			if self.audioid is not None:
				self._send(id, 'return', True)
			return
		while not self.waitaudio:
			data = self.audiofile.read(self.audio_fragment_size)
			if len(data) < self.audio_fragment_size:
				self.audiofile = None
				if self.audioid is not None:
					self._send(id, 'return', True)
				return
			if not self._send_packet(chr(self.command['AUDIO_DATA']) + data, audio = True):
				self.audiofile = None
				if self.audioid is not None:
					self._send(id, 'return', False)
				return
	# }}}
	# Subclasses.  {{{
	class Space: # {{{
		def __init__(self, printer, id):
			self.printer = printer
			self.id = id
			self.axis = []
			self.motor = []
			self.delta = [{'axis_min': 0., 'axis_max': 0., 'rodlength': 0., 'radius': 0.} for t in range(3)]
			self.delta_angle = 0
			self.extruder = [0., 0., 0.]
		def read(self, data):
			axes, motors = data
			self.axis = []
			self.motor = []
			for a in range(len(axes)):
				self.axis.append({})
				self.axis[-1]['offset'], self.axis[-1]['park'], self.axis[-1]['park_order'], self.axis[-1]['max_v'], self.axis[-1]['min'], self.axis[-1]['max'] = struct.unpack('<ffBfff', axes[a])
			for m in range(len(motors)):
				self.motor.append({})
				self.motor[-1]['step_pin'], self.motor[-1]['dir_pin'], self.motor[-1]['enable_pin'], self.motor[-1]['limit_min_pin'], self.motor[-1]['limit_max_pin'], self.motor[-1]['sense_pin'], self.motor[-1]['steps_per_m'], self.motor[-1]['max_steps'], self.motor[-1]['home_pos'], self.motor[-1]['motor_min'], self.motor[-1]['motor_max'], self.motor[-1]['limit_v'], self.motor[-1]['limit_a'], self.motor[-1]['home_order'] = struct.unpack('<HHHHHHfBfffffB', motors[m])
				log('received min %f max %f v %f a %f' % tuple(self.motor[-1][x] for x in ('motor_min', 'motor_max', 'limit_v', 'limit_a')))
		def write_info(self, num_axes = None):
			data = struct.pack('<Bf', self.type, self.max_deviation)
			if self.type == TYPE_EXTRUDER:
				data += struct.pack('<fffB', self.extruder[0], self.extruder[1], self.extruder[2], num_axes if num_axes is not None else len(self.axis))
			elif self.type == TYPE_CARTESIAN:
				data += struct.pack('<B', num_axes if num_axes is not None else len(self.axis))
			elif self.type == TYPE_DELTA:
				for a in range(3):
					data += struct.pack('<ffff', self.delta[a]['axis_min'], self.delta[a]['axis_max'], self.delta[a]['rodlength'], self.delta[a]['radius'])
				data += struct.pack('<f', self.delta_angle)
			else:
				log('invalid type')
				raise AssertionError('invalid space type')
			return data
		def write_axis(self, axis):
			return struct.pack('<ffBfff', self.axis[axis]['offset'], self.axis[axis]['park'], self.axis[axis]['park_order'], self.axis[axis]['max_v'], self.axis[axis]['min'], self.axis[axis]['max'])
		def write_motor(self, motor):
			return struct.pack('<HHHHHHfBfffffB', self.motor[motor]['step_pin'], self.motor[motor]['dir_pin'], self.motor[motor]['enable_pin'], self.motor[motor]['limit_min_pin'], self.motor[motor]['limit_max_pin'], self.motor[motor]['sense_pin'], self.motor[motor]['steps_per_m'], self.motor[motor]['max_steps'], self.motor[motor]['home_pos'], self.motor[motor]['motor_min'], self.motor[motor]['motor_max'], self.motor[motor]['limit_v'], self.motor[motor]['limit_a'], self.motor[motor]['home_order'])
		def set_current_pos(self, axis, pos):
			log('setting pos of %d %d to %f' % (self.id, axis, pos))
			if not self.printer._send_packet(struct.pack('<BBBf', self.printer.command['SETPOS'], self.id, axis, pos)):
				return False
			return True
		def get_current_pos(self, axis):
			if not self.printer._send_packet(struct.pack('<BBB', self.printer.command['GETPOS'], self.id, axis)):
				return None
			ret = self.printer._get_reply()
			assert ret[0] == self.printer.rcommand['POS']
			return struct.unpack('<f', ret[1:])[0]
		def export(self):
			std = [self.type, self.max_deviation, [[a['offset'], a['park'], a['park_order'], a['max_v'], a['min'], a['max']] for a in self.axis], [[m['step_pin'], m['dir_pin'], m['enable_pin'], m['limit_min_pin'], m['limit_max_pin'], m['sense_pin'], m['steps_per_m'], m['max_steps'], m['home_pos'], m['motor_min'], m['motor_max'], m['limit_v'], m['limit_a'], m['home_order']] for m in self.motor]]
			if self.type == TYPE_EXTRUDER:
				return std + [[e for e in self.extruder]]
			elif self.type == TYPE_CARTESIAN:
				return std
			elif self.type == TYPE_DELTA:
				return std + [[[a['axis_min'], a['axis_max'], a['rodlength'], a['radius']] for a in self.delta] + [self.delta_angle]]
			else:
				log('invalid type')
				raise AssertionError('invalid space type')
		def export_settings(self, num):
			ret = '[space %d]\r\n' % num
			ret += 'type = %d\r\n' % self.type
			ret += 'max_deviation = %d\r\n' % self.max_deviation
			if self.type == TYPE_EXTRUDER:
				ret += 'num_axes = %d\r\n' % len(self.axis)
				ret += '[extruder %d]\r\n' % num
				ret += ''.join(['%s = %f\r\n' % (e, self.extruder[i]) for i, e in enumerate(('dx', 'dy', 'dz'))])
			elif self.type == TYPE_CARTESIAN:
				ret += 'num_axes = %d\r\n' % len(self.axis)
			elif self.type == TYPE_DELTA:
				ret += 'delta_angle = %f\r\n' % self.delta_angle
				for i in range(3):
					ret += '[delta %d %d]\r\n' % (num, i)
					ret += ''.join(['%s = %f\r\n' % (x, self.delta[i][x]) for x in ('axis_min', 'axis_max', 'rodlength', 'radius')])
			for i, a in enumerate(self.axis):
				ret += '[axis %d %d]\r\n' % (num, i)
				ret += ''.join(['%s = %f\r\n' % (x, a[x]) for x in ('offset', 'park', 'park_order', 'max_v', 'min', 'max')])
			for i, m in enumerate(self.motor):
				ret += '[motor %d %d]\r\n' % (num, i)
				ret += ''.join(['%s = %s\r\n' % (x, write_pin(m[x])) for x in ('step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'sense_pin')])
				ret += ''.join(['%s = %d\r\n' % (x, m[x]) for x in ('max_steps', 'home_order')])
				ret += ''.join(['%s = %f\r\n' % (x, m[x]) for x in ('steps_per_m', 'home_pos', 'motor_min', 'motor_max', 'limit_v', 'limit_a')])
			return ret
	# }}}
	class Temp: # {{{
		def __init__(self):
			self.value = float('nan')
		def read(self, data):
			self.R0, self.R1, logRc, Tc, self.beta, self.power_pin, self.thermistor_pin = struct.unpack('<fffffHH', data)
			try:
				self.Rc = math.exp(logRc)
			except:
				self.Rc = float('nan')
			self.Tc = Tc - C0
		def write(self):
			try:
				logRc = math.log(self.Rc)
			except:
				logRc = float('nan')
			return struct.pack('<fffffHH', self.R0, self.R1, logRc, self.Tc + C0, self.beta, self.power_pin, self.thermistor_pin)
		def export(self):
			return [self.R0, self.R1, self.Rc, self.Tc, self.beta, self.power_pin, self.thermistor_pin, self.value]
		def export_settings(self, num):
			ret = '[temp %d]\r\n' % num
			ret += ''.join(['%s = %s\r\n' % (x, write_pin(getattr(self, x))) for x in ('power_pin', 'thermistor_pin')])
			ret += ''.join(['%s = %f\r\n' % (x, getattr(self, x)) for x in ('R0', 'R1', 'Rc', 'Tc', 'beta')])
			return ret
	# }}}
	class Gpio: # {{{
		def read(self, data):
			self.pin, self.state, self.master, value = struct.unpack('<HBBf', data)
			self.value = value - C0
		def write(self):
			return struct.pack('<HBBf', self.pin, self.state, self.master, self.value + C0)
		def export(self):
			return [self.pin, self.state, self.master, self.value]
		def export_settings(self, num):
			ret = '[gpio %d]\r\n' % num
			ret += 'pin = %s\r\n' % write_pin(self.pin)
			ret += ''.join(['%s = %d\r\n' % (x, getattr(self, x)) for x in ('state', 'master')])
			ret += 'value = %f\r\n' % self.value
			return ret
	# }}}
	# }}}
	# }}}
	# Useful commands.  {{{
	def reset(self): # {{{
		log('%s resetting and dying.' % self.name)
		return self._send_packet(struct.pack('<BB', self.command['RESET'], 0))
	# }}}
	def die(self, reason): # {{{
		log('%s dying as requested by host (%s).' % (self.name, reason))
		return (WAIT, WAIT)
	# }}}
	@delayed
	def flush(self, id): # {{{
		def cb(w):
			if id is not  None:
				self._send(id, 'return', w)
			self._do_gcode()
		self.movecb.append((False, cb))
		if self.flushing is not True:
			self.goto(cb = True)[1](None)
	# }}}
	@delayed
	def probe(self, id, area, angle = 0, speed = .003): # {{{
		log('area:%s' % repr(area))
		if not self.pin_valid(self.probe_pin) or len(self.spaces) < 1 or len(self.spaces[0].axis) < 3 or not self.probe_safe_dist > 0:
			if id is not None:
				self._send(id, 'return', None)
			return
		density = [int(abs(area[t + 2] - area[t]) / self.probe_dist) + 1 for t in range(2)]
		self.jobs_probemap = [area, density, [[None for x in range(density[0] + 1)] for y in range(density[1] + 1)]]
		self.gcode_angle = math.sin(angle), math.cos(angle)
		#log(repr(self.jobs_probemap))
		self.probe_orig = [(m['limit_min_pin'], m['max_steps']) for m in self.spaces[0].motor]
		for m in range(len(self.spaces[0].motor)):
			self.set_motor((0, m), limit_min_pin = self.probe_pin, max_steps = 1, readback = False)
		self._do_probe(id, 0, 0, self.get_axis_pos(0, 2), angle, speed)
	# }}}
	@delayed
	def goto(self, id, moves = (), f0 = None, f1 = None, cb = False): # {{{
		#log('goto %s %s %s' % (repr(moves), f0, f1))
		#log('speed %s' % f0)
		#traceback.print_stack()
		self.queue.append((id, moves, f0, f1, cb))
		if not self.wait:
			self._do_queue()
	# }}}
	def sleep(self, sleeping = True): # {{{
		if sleeping:
			self.position_valid = False
			self._globals_update()
		return self._send_packet(struct.pack('<BB', self.command['SLEEP'], sleeping))
	# }}}
	def settemp(self, channel, temp): # {{{
		channel = int(channel)
		self.temps[channel].value = temp
		self._temp_update(channel)
		return self._send_packet(struct.pack('<BBf', self.command['SETTEMP'], channel, temp + C0 if not math.isnan(self.temps[channel].beta) else temp))
	# }}}
	def waittemp(self, channel, min, max = None): # {{{
		channel = int(channel)
		if min is None:
			min = float('nan')
		if max is None:
			max = float('nan')
		return self._send_packet(struct.pack('<BBff', self.command['WAITTEMP'], channel, min + C0 if not math.isnan(self.temps[channel].beta) else min, max + C0 if not math.isnan(self.temps[channel].beta) else max))
	# }}}
	def readtemp(self, channel): # {{{
		channel = int(channel)
		if channel >= len(self.temps):
			log('Trying to read invalid temp %d' % channel)
			return float('nan')
		if not self._send_packet(struct.pack('<BB', self.command['READTEMP'], channel)):
			return None
		ret = self._get_reply()
		assert ret[0] == self.rcommand['TEMP']
		return struct.unpack('<f', ret[1:])[0] - (C0 if not math.isnan(self.temps[channel].beta) else 0)
	# }}}
	def readpower(self, channel): # {{{
		channel = int(channel)
		if channel >= len(self.temps):
			log('Trying to read invalid power %d' % channel)
			return float('nan')
		if not self._send_packet(struct.pack('<BB', self.command['READPOWER'], channel)):
			return None
		ret = self._get_reply()
		assert ret[0] == self.rcommand['POWER']
		return struct.unpack('<LL', ret[1:])
	# }}}
	def readpin(self, pin): # {{{
		if not self._send_packet(struct.pack('<BB', self.command['READPIN'], pin)):
			return None
		ret = self._get_reply()
		assert ret[0] == self.rcommand['PIN']
		return bool(ord(ret[1]))
	# }}}
	def load(self): # {{{
		if not self._send_packet(struct.pack('<B', self.command['LOAD'])):
			return False
		if not self._read_globals(False):
			return False
		self._globals_update()
		for i, s in enumerate(self.spaces):
			self.spaces[i].read(self._read('SPACE', i))
			self._space_update(i)
		for i, t in enumerate(self.temps):
			self.temps[i].read(self._read('TEMP', i))
			self._temp_update(i)
		for i, g in enumerate(self.gpios):
			self.gpios[i].read(self._read('GPIO', i))
			self._gpio_update(i)
		return True
	# }}}
	def save(self): # {{{
		self._broadcast(None, 'blocked', 'saving')
		ret = self._send_packet(struct.pack('<B', self.command['SAVE']))
		self._broadcast(None, 'blocked', None)
		return ret
	# }}}
	def pause(self, pausing = True, store = True): # {{{
		was_paused = self.paused
		if pausing:
			if not self._send_packet(struct.pack('<BB', self.command['QUEUED'], True)):
				log('failed to pause printer')
				return False
			self.movewait = 0
			reply = struct.unpack('<BB', self._get_reply())
			if reply[0] != ord(self.rcommand['QUEUE']):
				log('invalid reply to queued command')
				return False
			self.wait = False
		self.paused = pausing
		if not self.paused:
			if was_paused:
				# Go back to pausing position.
				self.goto(self.queue_info[1])
				# TODO: adjust extrusion of current segment to shorter path length.
				#log('resuming')
				self.resuming = True
			self._do_queue()
		else:
			#log('pausing')
			if not was_paused:
				#log('pausing %d %d %d %d %d' % (store, self.queue_info is None, len(self.queue), self.queue_pos, reply[1]))
				if store and self.queue_info is None and len(self.queue) > 0 and self.queue_pos - reply[1] >= 0:
					if self.home_phase is not None:
						#log('killing homer')
						self.home_phase = None
						self.set_space(self.home_space, type = self.home_orig_type)
						for i in range(len(self.spaces[self.home_space].axis)):
							self.set_axis((self.home_space, i), min = self.home_orig_limits[i][0], max = self.home_orig_limits[i][1])
							self.set_motor((self.home_space, i), max_steps = self.home_orig_limits[1][i])
						if self.home_cb in self.movecb:
							self.movecb.remove(self.home_cb)
							self._send(self.home_id, 'return', None)
					if self.probe_cb in self.movecb:
						#log('killing prober')
						self.movecb.remove(self.probe_cb)
						self.probe_cb(False)
					#log('pausing gcode %d/%d/%d' % (self.queue_pos, reply[1], len(self.queue)))
					if self.flushing is None:
						self.flushing = False
					self.queue_info = [self.queue_pos - reply[1], [[s.get_current_pos(a) for a in range(len(s.axis))] for s in self.spaces], self.queue, self.movecb, self.flushing, self.gcode_wait]
				else:
					#log('stopping')
					self.paused = False
					if len(self.movecb) > 0:
						call_queue.extend([(x[1], [True]) for x in self.movecb])
				self.queue = []
				self.movecb = []
				self.flushing = False
				self.gcode_wait = None
				self.queue_pos = 0
		self._globals_update()
	# }}}
	def queued(self): # {{{
		if not self._send_packet(struct.pack('<BB', self.command['QUEUED'], False)):
			return None
		reply = struct.unpack('<BB', self._get_reply())
		if reply[0] != ord(self.rcommand['QUEUE']):
			log('invalid reply to queued command')
			return None
		return reply[1]
	# }}}
	def ping(self, arg = 0): # {{{
		if not self._send_packet(struct.pack('<BB', self.command['PING'], arg)):
			return False
		reply = self._get_reply()
		return struct.unpack('<BB', reply) == (ord(self.rcommand['PONG']), arg)
	# }}}
	@delayed
	def home(self, id, space = 0, speed = .005, cb = None, abort = True): # {{{
		#log('homing')
		if self.home_phase is not None and not self.paused:
			log("ignoring request to home because we're already homing")
			self._send(id, 'return', None)
			return
		# Abort only if it is requested, and the job is not paused.
		if abort and self.queue_info is None:
			self._print_done(False, 'aborted by homing')
		self.home_id = id
		self.home_speed = speed
		self.home_done_cb = cb
		self.home_space = space
		self._do_home()
	# }}}
	@delayed
	def park(self, id, cb = None, abort = True, order = 0, aborted = False): # {{{
		if aborted:
			if id is not None:
				self._send(id, 'error', 'aborted')
			return
		#log('parking')
		if abort:
			self._print_done(False, 'aborted by parking')
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
			self.gcode_parking = False
			if cb:
				call_queue.append((cb, []))
			if id is not None:
				self._send(id, 'return', None)
			return
		self.movecb.append((False, lambda done: self.park(cb, False, next_order + 1, not done)[1](id)))
		self.goto([[a['park'] - a['offset'] if a['park_order'] == next_order else float('nan') for a in s.axis] for s in self.spaces], cb = True)[1](None)
	# }}}
	@delayed
	def audio_play(self, id, name, motors = None): # {{{
		assert os.path.basename(name) == name
		self.audiofile = open(os.path.join(self.audiodir, name), 'rb')
		channels = [0] *(((sum([len(s.motor) for s in self.spaces]) - 1) >> 3) + 1)
		m0 = 0
		for s in self.spaces:
			for m in range(len(s.motor)):
				if motors is None or m in motors:
					channels[(m0 + m) >> 3] |= 1 <<((m0 + m) & 0x7)
			m0 += len(s.motor)
		us_per_bit = self.audiofile.read(2)
		if not self._send_packet(chr(self.command['AUDIO_SETUP']) + us_per_bit + ''.join([chr(x) for x in channels])):
			return False
		self.audio_id = id
		self._audio_play()
	# }}}
	def audio_load(self, name, data): # {{{
		assert os.path.basename(name) == name
		wav = wave.open(io.BytesIO(base64.b64decode(data)))
		assert wav.getnchannels() == 1
		data = [ord(x) for x in wav.readframes(wav.getnframes())]
		data = [(h << 8) + l if h < 128 else(h << 8) + l -(1 << 16) for l, h in zip(data[::2], data[1::2])]
		minimum = min(data)
		maximum = max(data)
		level = (minimum + maximum) / 2
		s = ''
		for t in range(0, len(data) - 7, 8):
			c = 0
			for b in range(8):
				if data[t + b] > level:
					c += 1 << b
			s += chr(c)
		if not os.path.exists(self.audiodir):
			os.makedirs(self.audiodir)
		with open(os.path.join(self.audiodir, name), 'wb') as f:
			f.write(struct.pack('<H', 1000000 / wav.getframerate()) + s)
	# }}}
	def audio_list(self): # {{{
		ret = []
		for x in os.listdir(self.audiodir):
			try:
				with open(os.path.join(self.audiodir, x), 'rb') as f:
					us_per_bit = struct.unpack('<H', f.read(2))[0]
					bits = (os.fstat(f.fileno()).st_size - 2) * 8
					t = bits * us_per_bit * 1e-6
					ret.append((x, t))
			except:
				pass
		return ret
	# }}}
	@delayed
	def wait_for_cb(self, id, sense = False): # {{{
		ret = lambda w: id is None or self._send(id, 'return', w)
		if self.movewait == 0 or sense is not False and sense[1] in self.sense[sense[0]]:
			ret(self.movewait == 0)
		else:
			self.movecb.append((sense, ret))
	# }}}
	@delayed
	def wait_for_temp(self, id, which = None): # {{{
		def cb():
			if id is not None:
				self._send(id, 'return', None)
			self.gcode_waiting -= 1
			self._do_gcode()
		if(which is None and len(self.alarms) > 0) or which in self.alarms:
			cb()
		else:
			self.tempcb.append((which, cb))
	# }}}
	def clear_alarm(self, which = None): # {{{
		if which is None:
			self.alarms.clear()
		else:
			self.alarms.discard(which)
	# }}}
	def get_limits(self, space, motor = None):	# {{{
		if motor is None:
			return self.limits[space]
		if motor in self.limits[space]:
			return self.limits[space][motor]
		return None
	# }}}
	def clear_limits(self):	# {{{
		for s in range(len(self.spaces)):
			self.limits[s].clear()
	# }}}
	def get_sense(self, space, motor = None):	# {{{
		if motor is None:
			return self.sense[space]
		if motor in self.sense[space]:
			return self.sense[space][motor]
		return []
	# }}}
	def clear_sense(self):	# {{{
		for s in range(len(self.spaces)):
			self.sense[s].clear()
	# }}}
	def pin_valid(self, pin):	# {{{
		return(pin & 0x100) != 0
	# }}}
	def valid(self):	# {{{
		return self.position_valid
	# }}}
	def export_settings(self): # {{{
		message = '[general]\r\n'
		# Don't export the name.
		#message += 'name=' + self.name.replace('\\', '\\\\').replace('\n', '\\n') + '\r\n'
		for t in ('spaces', 'temps', 'gpios'):
			message += 'num_%s = %d\r\n' % (t, len(getattr(self, t)))
		message += ''.join(['%s = %s\r\n' % (x, write_pin(getattr(self, x))) for x in ('led_pin', 'probe_pin')])
		message += ''.join(['%s = %f\r\n' % (x, getattr(self, x)) for x in ('probe_dist', 'probe_safe_dist', 'feedrate', 'motor_limit', 'temp_limit')])
		for i, s in enumerate(self.spaces):
			message += s.export_settings(i)
		for i, t in enumerate(self.temps):
			message += t.export_settings(i)
		for i, g in enumerate(self.gpios):
			message += g.export_settings(i)
		return message
	# }}}
	def import_settings(self, settings, filename = None): # {{{
		section = 'general'
		index = None
		obj = None
		regexp = re.compile('\s*\[(general|(space|temp|gpio|extruder|(axis|motor|delta)\s+(\d+))\s+(\d+))\]\s*$|\s*(\w+)\s*=\s*(.*?)\s*$|\s*(?:#.*)?$')
		#1: (general|(space|temp|gpio|(axis|motor|delta)\s+(\d+))\s+(\d+))	1 section
		#2: (space|temp|gpio|extruder|(axis|motor|delta)\s+(\d+))		2 section with index
		#3: (axis|motor|delta)							3 sectionname with two indices
		#4: (\d+)								4 index of space
		#5: (\d+)								5 only or component index
		#6: (\w+)								6 identifier
		#7: (.*?)								7 value
		errors = []
		globals_changed = False
		changed = {'space': set(), 'temp': set(), 'gpio': set(), 'axis': set(), 'motor': set(), 'extruder': set(), 'delta': set()}
		keys = {
				'general': {'num_spaces', 'num_temps', 'num_gpios', 'led_pin', 'probe_pin', 'probe_dist', 'probe_safe_dist', 'feedrate', 'motor_limit', 'temp_limit'},
				'space': {'type', 'max_deviation', 'num_axes', 'delta_angle'},
				'temp': {'R0', 'R1', 'Rc', 'Tc', 'beta', 'power_pin', 'thermistor_pin'},
				'gpio': {'pin', 'state', 'master', 'value'},
				'axis': {'offset', 'park', 'park_order', 'max_v', 'min', 'max'},
				'motor': {'step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'sense_pin', 'steps_per_m', 'max_steps', 'home_pos', 'motor_min', 'motor_max', 'limit_v', 'limit_a', 'home_order'},
				'extruder': {'dx', 'dy', 'dz'},
				'delta': {'axis_min', 'axis_max', 'rodlength', 'radius'}
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
					if r.group(3) is not None:
						# Two indices.
						index = (int(r.group(4)), int(r.group(5)))
						section = r.group(3)
						if index[0] >= len(self.spaces) or index[1] >= len(getattr(self.spaces[index[0]], section)):
							log('index out of range for %s; %s %s' % (index, len(self.spaces), len(getattr(self.spaces[index[0]], section)) if index[0] < len(self.spaces) else 'x'))
							errors.append((l, 'index out of range'))
							obj = None
							continue
						obj = getattr(self.spaces[index[0]], section)[index[1]]
					else:
						# One index.
						index = int(r.group(5))
						section = r.group(2)
						if section == 'extruder':
							if index >= len(self.spaces):
								errors.append((l, 'index out of range'))
								obj = None
								continue
							obj = self.spaces[index].extruder
						else:
							if index >= len(getattr(self, section + 's')):
								errors.append((l, 'index out of range'))
								obj = None
								continue
							obj = getattr(self, section + 's')[index]
					changed[section].add(index)
				else:
					# No indices.
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
			if key == 'name':
				pass
			elif key.endswith('pin'):
				value = read_pin(value)
			elif key.startswith('num'):
				value = int(value)
			else:
				value = float(value)
			if key not in keys[section]:
				errors.append((l, 'invalid key for section %s' % section))
				continue
			# If the number of things is changed, update instantly.
			if key.startswith('num') or key == 'type':
				#log('setting now for %s:%s=%s' % (section, key, value))
				if index is None:
					self.set_globals(**{key: value})
					globals_changed = False
				else:
					getattr(self, 'set_' + section)(index, **{key: value})
					changed[section].discard(index)
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
		# Update values in the printer by calling the set_* functions with no new settings.
		if globals_changed:
			#log('setting globals')
			self.set_globals()
		for section in changed:
			if section == 'extruder':
				changed['space'].add(index)
				continue
			for index in changed[section]:
				if not isinstance(index, tuple):
					continue
				if section != 'delta':
					#log('setting non-delta %s' % repr(section))
					getattr(self, 'set_' + section)(index, readback = False)
				changed['space'].add(index[0])
		for section in changed:
			if section == 'extruder':
				continue
			for index in changed[section]:
				if isinstance(index, tuple):
					continue
				#log('setting %s' % repr((section, index)))
				getattr(self, 'set_' + section)(index)
		return errors
	# }}}
	@delayed
	def gcode_run(self, id, code, ref = (0, 0), angle = 0, probemap = None, abort = True): # {{{
		self.gcode_ref = ref
		angle = math.radians(angle)
		self.gcode_angle = math.sin(angle), math.cos(angle)
		self.gcode_probemap = probemap
		ret = lambda complete, reason: id is None or self._send(id, 'return', complete, reason)	#TODO: use this?
		if len(self.temps) > 0:
			self.btemp = self.temps[0].value
		else:
			self.btemp = float('nan')
		if abort:
			self._unpause()
			self._print_done(False, 'aborted by starting new print')
		self.queue_info = None
		self.gcode = code
		self.gcode_id = id
		if len(self.gcode) <= 0:
			log('nothing to run')
			self.gcode = None
			ret(False, 'nothing to run')
			self.paused = None
			self._globals_update()
			return
		self.paused = False
		self._globals_update()
		self.sleep(False)
		if len(self.spaces) > 1:
			for e in range(len(self.spaces[1].axis)):
				self.set_axis_pos(1, e, 0)
		self._broadcast(None, 'printing', True)
		self._do_gcode()
	# }}}
	@delayed
	def request_confirmation(self, id, message): # {{{
		self.confirmer = id
		self.confirm_id += 1
		self._broadcast(None, 'confirm', self.confirm_id, message)
	# }}}
	def confirm(self, confirm_id, success = True): # {{{
		if confirm_id != self.confirm_id:
			# Confirmation was already sent.
			return False
		id = self.confirmer
		self.confirmer = None
		if id not in (False, None):
			self._send(id, 'return', success)
		else:
			if not success:
				self._unpause()
				self._print_done(False, 'aborted by failed confirmation')
			else:
				self._do_gcode()
		return True
	# }}}
	def queue_add(self, data, name): # {{{
		assert name not in self.jobqueue
		self._broadcast(None, 'blocked', 'parsing g-code')
		parsed = self.gcode_parse(data)
		self.jobqueue[name] = (parsed, self.gcode_bbox(parsed))
		self._broadcast(None, 'queue', [(q, self.jobqueue[q][1]) for q in self.jobqueue])
		self._broadcast(None, 'blocked', None)
		return ''
	# }}}
	def queue_remove(self, name): # {{{
		assert name in self.jobqueue
		del self.jobqueue[name]
		self._broadcast(None, 'queue', [(q, self.jobqueue[q][1]) for q in self.jobqueue])
	# }}}
	def gcode_parse(self, code): # {{{
		mode = None
		message = None
		ret = []
		unit = .001
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
					log('%d:ignoring line with unterminated comment: %s' % (lineno, origline))
					continue
				comment = line[b + 1:e].strip()
				line = line[:b] + line[e + 1:].strip()
			if ';' in line:
				p = line.index(';')
				comment = line[p + 1:].strip()
				line = line[:p].strip()
			if comment.upper().startswith('MSG,'):
				message = comment[4:].strip()
			elif comment.startswith('SYSTEM:'):
				if not re.match(config['allow-system'], comment[7:]):
					log('refusing to run forbidden system command')
				else:
					ret.append((('SYSTEM', 0), {}, comment[7:]))
				continue
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
					unit = .0254
					continue
				elif cmd == ('G', 21):
					unit = .001
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
					args['e'] = current_extruder
				elif cmd[0] == 'M' and cmd[1] in (104, 109, 116):
					args['E'] = int(args['T']) if 'T' in args else current_extruder
				if not((cmd[0] == 'G' and cmd[1] in (0, 1, 4, 28, 81, 92, 94)) or (cmd[0] == 'M' and cmd[1] in (0, 3, 4, 5, 6, 9, 42, 84, 104, 106, 107, 109, 116, 140, 190)) or (cmd[0] in ('S', 'T'))):
					log('%d:invalid gcode command %s' % (lineno, repr((cmd, args))))
				elif cmd == ('G', 28):
					ret.append((cmd, args, message))
					for a in range(len(pos[0])):
						if not math.isnan(self.spaces[0].axis[a]['park']):
							pos[0][a] = self.spaces[0].axis[a]['park']
				elif cmd[0] == 'G' and cmd[1] in (0, 1, 81):
					if cmd[1] != 0:
						mode = cmd
					components = {'X': None, 'Y': None, 'Z': None, 'E': None, 'F': None, 'R': None}
					for c in args:
						assert c in components and components[c] is None
						components[c] = args[c]
					f0 = pos[2]
					if components['F'] is not None:
						pos[2] = components['F'] * unit
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
						dist = sum([(pos[0][x] - oldpos[0][x]) ** 2 for x in range(3)]) ** .5
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
						args = {'x': oldpos[0][0], 'y': oldpos[0][1], 'z': oldpos[0][2], 'X': pos[0][0], 'Y': pos[0][1], 'Z': pos[0][2], 'e': oldpos[1][current_extruder], 'E': pos[1][current_extruder], 'f': f0 / dist / 60 if dist != 0 and cmd[1] == 1 else float('inf'), 'F': pos[2] / dist / 60 if dist != 0 and cmd[1] == 1 else float('inf')}
						cmd = ('G', 1)
						ret.append((cmd, args, message))
					else:
						# Drill cycle.
						# Only support OLD_Z (G90) retract mode; don't support repeats(L).
						# goto x,y
						ret.append((('G', 1), {'x': oldpos[0][0], 'y': oldpos[0][1], 'z': oldpos[0][2], 'X': pos[0][0], 'Y': pos[0][1], 'Z': oldpos[0][2], 'e': 0, 'E': 0, 'f': float('inf'), 'F': float('inf')}, message))
						# goto r
						ret.append((('G', 1), {'x': pos[0][0], 'y': pos[0][1], 'z': oldpos[0][2], 'X': pos[0][0], 'Y': pos[0][1], 'Z': r, 'e': 0, 'E': 0, 'f': float('inf'), 'F': float('inf')}, None))
						# goto z; this is always straight down, because the move before and after it are also vertical.
						if z != r:
							f0 = pos[2] / abs(z - r) / 60
							ret.append((('G', 1), {'x': pos[0][0], 'y': pos[0][1], 'z': r, 'X': pos[0][0], 'Y': pos[0][1], 'Z': z, 'e': 0, 'E': 0, 'f': f0, 'F': f0}, None))
						# retract; this is always straight up, because the move before and after it are also non-horizontal.
						ret.append((('G', 1), {'x': pos[0][0], 'y': pos[0][1], 'z': z, 'X': pos[0][0], 'Y': pos[0][1], 'Z': oldpos[0][2], 'e': 0, 'E': 0, 'f': float('inf'), 'F': float('inf')}, None))
						# empty move; this makes sure the previous move is entirely vertical.
						ret.append((('G', 1), {'x': pos[0][0], 'y': pos[0][1], 'z': oldpos[0][2], 'X': pos[0][0], 'Y': pos[0][1], 'Z': oldpos[0][2], 'e': 0, 'E': 0, 'f': float('inf'), 'F': float('inf')}, None))
						# Set up current z position so next G81 will work.
						pos[0][2] = oldpos[0][2]
				else:
					ret.append((cmd, args, message))
				message = None
		return ret
	# }}}
	def gcode_bbox(self, code): # {{{
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
			inspect(args['x'], ret[0])
			inspect(args['y'], ret[1])
			inspect(args['z'], ret[2])
			inspect(args['X'], ret[0])
			inspect(args['Y'], ret[1])
			inspect(args['Z'], ret[2])
		return ret
	# }}}
	@delayed
	def queue_print(self, id, names, ref = (0, 0), angle = 0, probemap = None): # {{{
		self._broadcast(None, 'printing', True)
		self.job_output = ''
		self.jobs_active = names
		self.jobs_ref = ref
		self.jobs_angle = angle
		self.jobs_probemap = probemap
		self.job_current = -1	# next_job will make it start at 0.
		self.job_id = id
		if not self.probing:
			self._next_job()
	# }}}
	@delayed
	def queue_probe(self, id, names, ref = (0, 0), angle = 0, speed = .003): # {{{
		bbox = [float('nan'), (float('nan')), float('nan'), float('nan')]
		for n in names:
			bb = self.jobqueue[n][1]
			if not bbox[0] < bb[0][0]:
				bbox[0] = bb[0][0]
			if not bbox[1] < bb[1][0]:
				bbox[1] = bb[1][0]
			if not bbox[2] > bb[0][1]:
				bbox[2] = bb[0][1]
			if not bbox[3] > bb[1][1]:
				bbox[3] = bb[1][1]
		self.probe((bbox[0] + ref[0], bbox[1] + ref[1], bbox[2] + ref[0], bbox[3] + ref[1]), angle, speed)[1](None)
		# Pass jobs_probemap to make sure it doesn't get overwritten.
		self.queue_print(names, ref, angle, self.jobs_probemap)[1](id)
	# }}}
	# }}}
	# Accessor functions. {{{
	# Globals. {{{
	def get_globals(self):
		ret = {'num_spaces': len(self.spaces), 'num_temps': len(self.temps), 'num_gpios': len(self.gpios)}
		for key in ('name', 'queue_length', 'audio_fragments', 'audio_fragment_size', 'num_pins', 'num_digital_pins', 'led_pin', 'probe_pin', 'probe_dist', 'probe_safe_dist', 'motor_limit', 'temp_limit', 'feedrate', 'paused'):
			ret[key] = getattr(self, key)
		return ret
	def set_globals(self, **ka):
		#log('setting variables with %s' % repr(ka))
		if 'name' in ka:
			self.name = str(ka.pop('name'))
		ns = ka.pop('num_spaces') if 'num_spaces' in ka else None
		nt = ka.pop('num_temps') if 'num_temps' in ka else None
		ng = ka.pop('num_gpios') if 'num_gpios' in ka else None
		for key in ('led_pin', 'probe_pin'):
			if key in ka:
				setattr(self, key, int(ka.pop(key)))
		for key in ('probe_dist', 'probe_safe_dist', 'motor_limit', 'temp_limit', 'feedrate'):
			if key in ka:
				setattr(self, key, float(ka.pop(key)))
		self._write_globals(ns, nt, ng)
		assert len(ka) == 0
	# }}}
	# Space {{{
	def get_axis_pos(self, space, axis):
		if space >= len(self.spaces) or axis >= len(self.spaces[space].axis):
			log('request for invalid axis position %d %d' % (space, axis))
			return float('nan')
		return self.spaces[space].get_current_pos(axis)
	def set_axis_pos(self, space, axis, pos):
		return self.spaces[space].set_current_pos(axis, pos)
	def get_space(self, space):
		ret = {'num_axes': len(self.spaces[space].axis), 'num_motors': len(self.spaces[space].motor), 'type': self.spaces[space].type, 'max_deviation': self.spaces[space].max_deviation}
		if self.spaces[space].type == TYPE_EXTRUDER:
			ret['extruder'] = {}
			for i, key in enumerate(('dx', 'dy', 'dz')):
				ret['extruder'][key] = self.spaces[space].extruder[i]
		elif self.spaces[space].type == TYPE_CARTESIAN:
			pass
		elif self.spaces[space].type == TYPE_DELTA:
			delta = []
			for i in range(3):
				d = {}
				for key in ('axis_min', 'axis_max', 'rodlength', 'radius'):
					d[key] = self.spaces[space].delta[i][key]
				delta.append(d)
			delta.append(self.delta_angle)
			ret['delta'] = delta
		else:
			log('invalid type')
		return ret
	def get_axis(self, space, axis):
		ret = {}
		for key in ('offset', 'park', 'park_order', 'max_v', 'min', 'max'):
			ret[key] = self.spaces[space].axis[axis][key]
		return ret
	def get_motor(self, space, motor):
		ret = {}
		for key in ('step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'sense_pin', 'steps_per_m', 'max_steps', 'home_pos', 'motor_min', 'motor_max', 'limit_v', 'limit_a', 'home_order'):
			ret[key] = self.spaces[space].motor[motor][key]
		return ret
	def set_space(self, space, readback = True, **ka):
		if 'type' in ka:
			self.spaces[space].type = int(ka.pop('type'))
		if 'max_deviation' in ka:
			self.spaces[space].max_deviation = float(ka.pop('max_deviation'))
		if self.spaces[space].type == TYPE_EXTRUDER:
			if 'extruder' in ka:
				e = ka.pop('extruder')
				for i, key in enumerate(('dx', 'dy', 'dz')):
					if key in e:
						self.spaces[space].extruder[i] = e.pop(key)
				assert len(e) == 0
		if self.spaces[space].type in (TYPE_CARTESIAN, TYPE_EXTRUDER):
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
				for dd, di in d.items():
					i = int(dd)
					assert 0 <= i < 3
					for key in ('axis_min', 'axis_max', 'rodlength', 'radius'):
						if key in di:
							self.spaces[space].delta[i][key] = di.pop(key)
					assert len(di) == 0
			if 'delta_angle' in ka:
				self.spaces[space].delta_angle = ka.pop('delta_angle')
		self._send_packet(struct.pack('<BB', self.command['WRITE_SPACE_INFO'], space) + self.spaces[space].write_info(num_axes))
		self.spaces[space].axis = []
		self.spaces[space].motor = []
		if readback:
			self.spaces[space].read(self._read('SPACE', space))
			self._space_update(space)
		assert len(ka) == 0
	def set_axis(self, (space, axis), readback = True, **ka):
		for key in ('offset', 'park', 'park_order', 'max_v', 'min', 'max'):
			if key in ka:
				self.spaces[space].axis[axis][key] = ka.pop(key)
		self._send_packet(struct.pack('<BBB', self.command['WRITE_SPACE_AXIS'], space, axis) + self.spaces[space].write_axis(axis))
		if readback:
			self.spaces[space].read(self._read('SPACE', space))
			self._space_update(space)
		assert len(ka) == 0
	def set_motor(self, (space, motor), readback = True, **ka):
		for key in ('step_pin', 'dir_pin', 'enable_pin', 'limit_min_pin', 'limit_max_pin', 'sense_pin', 'steps_per_m', 'max_steps', 'home_pos', 'motor_min', 'motor_max', 'limit_v', 'limit_a', 'home_order'):
			if key in ka:
				self.spaces[space].motor[motor][key] = ka.pop(key)
		self._send_packet(struct.pack('<BBB', self.command['WRITE_SPACE_MOTOR'], space, motor) + self.spaces[space].write_motor(motor))
		if readback:
			self.spaces[space].read(self._read('SPACE', space))
			self._space_update(space)
		assert len(ka) == 0
	# }}}
	# Temp {{{
	def get_temp(self, temp):
		ret = {}
		for key in ('R0', 'R1', 'Rc', 'Tc', 'beta', 'power_pin', 'thermistor_pin'):
			ret[key] = getattr(self.temps[temp], key)
		return ret
	def set_temp(self, temp, **ka):
		ret = {}
		for key in ('R0', 'R1', 'Rc', 'Tc', 'beta', 'power_pin', 'thermistor_pin'):
			if key in ka:
				setattr(self.temps[temp], key, ka.pop(key))
		self._send_packet(struct.pack('<BB', self.command['WRITE_TEMP'], temp) + self.temps[temp].write())
		self.temps[temp].read(self._read('TEMP', temp))
		self._temp_update(temp)
		assert len(ka) == 0
	# }}}
	# Gpio {{{
	def get_gpio(self, gpio):
		ret = {}
		for key in ('pin', 'state', 'master', 'value'):
			ret[key] = getattr(self.gpios[gpio], key)
		return ret
	def set_gpio(self, gpio, **ka):
		for key in ('pin', 'state', 'master', 'value'):
			if key in ka:
				setattr(self.gpios[gpio], key, ka.pop(key))
		self._send_packet(struct.pack('<BB', self.command['WRITE_GPIO'], gpio) + self.gpios[gpio].write())
		self.gpios[gpio].read(self._read('GPIO', gpio))
		self._gpio_update(gpio)
		assert len(ka) == 0
	# }}}
	def send_printer(self, target): # {{{
		self._broadcast(target, 'new_printer', [self.queue_length, self.audio_fragments, self.audio_fragment_size, self.num_digital_pins, self.num_pins])
		self._globals_update(target)
		for i, s in enumerate(self.spaces):
			self._space_update(i, target)
		for i, t in enumerate(self.temps):
			self._temp_update(i, target)
		for i, g in enumerate(self.gpios):
			self._gpio_update(i, target)
		self._broadcast(None, 'queue', [(q, self.jobqueue[q][1]) for q in self.jobqueue])
		if self.gcode is not None:
			self._broadcast(target, 'printing', True)
	# }}}
	# }}}
# }}}

call_queue = []
printer = Printer(*sys.argv[1:])
if printer.printer is None:
	sys.exit(0)

while True: # {{{
	while len(call_queue) > 0:
		f, a = call_queue.pop(0)
		f(*a)
	now = time.time()
	fds = [sys.stdin, printer.printer]
	found = select.select(fds, [], fds, printer.gcode_wait and max(0, printer.gcode_wait - now))
	#log(repr(found))
	if len(found[0]) == 0:
		#log('timeout')
		# Timeout.
		printer.gcode_wait = None
		printer._do_gcode()
		continue
	if sys.stdin in found[0] or sys.stdin in found[2]:
		#log('command')
		printer._command_input()
	if printer.printer in found[0] or printer.printer in found[2]:
		#log('printer')
		printer._printer_input()
# }}}

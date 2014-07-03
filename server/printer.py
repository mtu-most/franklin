#! /usr/bin/python
# vim: set foldmethod=marker :

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

# Sentinel for blocking functions.
WAIT = object()

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
		self.status = None
		self.confirm_id = 0
		self.home_phase = None
		self.home_cb = [False, self._do_home]
		self.probe_cb = [False, None]
		self.gcode = None
		self.gcode_id = None
		self.gcode_wait = None
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
		self.pos = []
		# Set up state.
		self.sending = False
		self.paused = False
		self.ff_in = False
		self.ff_out = False
		self.limits = {}
		self.sense = {}
		self.wait = False
		self.waitaudio = False
		self.movewait = 0
		self.movecb = []
		self.limitcb = []
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
		self._begin(newid)
		self.namelen, self.queue_length, self.maxaxes, self.maxextruders, self.maxtemps, self.maxgpios, self.audio_fragments, self.audio_fragment_size, self.num_pins, self.num_digital_pins = struct.unpack('<BBBBBBBBBB', self._read(0))
		self._read_variables()
		self.axis = [Printer.Axis(self, t) for t in range(self.maxaxes)]
		for a in range(self.maxaxes):
			self.axis[a].read(self._read(2 + a))
			self.sleep_axis(a)
		self.extruder = [Printer.Extruder() for t in range(self.maxextruders)]
		for e in range(self.maxextruders):
			self.extruder[e].read(self._read(2 + self.maxaxes + e))
			# Disable heater.
			self.settemp_extruder(e, float('nan'))
			# Disable heater alarm.
			self.waittemp_extruder(e, None, None)
			# Sleep motor.
			self.sleep_extruder(e)
		self.temp = [Printer.Temp() for t in range(self.maxtemps)]
		for t in range(self.maxtemps):
			self.temp[t].read(self._read(2 + self.maxaxes + self.maxextruders + t))
			# Disable heater.
			self.settemp_temp(t, float('nan'))
			# Disable heater alarm.
			self.waittemp_temp(t, None, None)
		self.gpio = [Printer.Gpio() for g in range(self.maxgpios)]
		for g in range(self.maxgpios):
			self.gpio[g].read(self._read(2 + self.maxaxes + self.maxextruders + self.maxtemps + g))
		# The printer may still be doing things.  Pause it and send a move; this will discard the queue.
		self.pause(True, False)
		self.goto()[1](None)	# This flushes the buffer, possibly causing movecbs to be sent.
		self.goto(cb = True)[1](None)	# This sends one new movecb, to be sure the rest is received if they were sent.
		self._get_reply(cb = True)
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
			'RUN': 0x05,
			'SLEEP': 0x06,
			'SETTEMP': 0x07,
			'WAITTEMP': 0x08,
			'READTEMP': 0x09,
			'READPOWER': 0x0a,
			'SETPOS': 0x0b,
			'GETPOS': 0x0c,
			'LOAD': 0x0d,
			'SAVE': 0x0e,
			'READ': 0x0f,
			'WRITE': 0x10,
			'QUEUED': 0x11,
			'READGPIO': 0x12,
			'AUDIO_SETUP': 0x13,
			'AUDIO_DATA': 0x14,
			'SETSERIAL': 0x15,
			'SERIAL_TX': 0x16}
	rcommand = {
			'START': '\x17',
			'TEMP': '\x18',
			'POWER': '\x19',
			'POS': '\x1a',
			'DATA': '\x1b',
			'PONG': '\x1c',
			'PIN': '\x1d',
			'QUEUE': '\x1e',
			'MOVECB': '\x1f',
			'TEMPCB': '\x20',
			'CONTINUE': '\x21',
			'LIMIT': '\x22',
			'AUTOSLEEP': '\x23',
			'SENSE': '\x24',
			'SERIAL_RX': '\x25'}
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
		while '\n' in self.command_buffer:
			pos = self.command_buffer.index('\n')
			id, func, a, ka = json.loads(self.command_buffer[:pos])
			#log('command: %s (rest %s)' % (repr((id, func, a, ka)), repr(self.command_buffer)))
			self.command_buffer = self.command_buffer[pos + 1:]
			die = False
			try:
				ret = getattr(self, func)(*a, **ka)
				if isinstance(ret, tuple) and len(ret) == 2 and ret[0] is WAIT:
					# The function blocks; it will send its own reply later.
					if ret[1] is WAIT:
						# Special case: request to die.
						die = True
					else:
						ret[1](id)
						return True
			except:
				traceback.print_exc()
				self._send(id, 'error', repr(sys.exc_info()))
				return True
			if die:
				sys.exit(0)
			self._send(id, 'return', ret)
	# }}}
	def _printer_input(self, ack = False, reply = False): # {{{
		for input_limit in range(50):
			while self.debug_buffer is not None:
				r = self._printer_read(1)
				if r == '':
					return ('no data', None)
				if r == '\x00':
					if show_firmware_debug:
						log('Debug: %s' % self.debug_buffer)
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
				if(ord(r) & 0x80) != 0 or ord(r) in (0, 1, 2, 4):
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
					assert self.movewait >= num
					self.movewait -= num
					if self.movewait == 0:
						call_queue.extend([(x[1], [True]) for x in self.movecb])
						self.movecb = []
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
				else:
					# Audio continue.
					self.waitaudio = False
				continue
			elif packet[0] == self.rcommand['LIMIT']:
				if not self.initialized:
					# Ignore this while connecting.
					continue
				which = ord(packet[1])
				if not math.isnan(self.axis[which].motor.runspeed):
					self.axis[which].motor.runspeed = float('nan')
					self._axis_update(which)
				self.limits[which] = struct.unpack('<f', packet[2:])[0]
				l = 0
				while l < len(self.limitcb):
					if self.limitcb[l][0] <= len(self.limits):
						call_queue.append((self.limitcb.pop(l)[1], []))
					else:
						l += 1
				continue
			elif packet[0] == self.rcommand['AUTOSLEEP']:
				if not self.initialized:
					# Ignore this while connecting.
					continue
				what = ord(packet[1])
				changed = [set(), set(), set()]
				if what & 1:
					for a in range(self.maxaxes):
						self.axis[a].valid = False
						if not math.isnan(self.axis[a].motor.runspeed):
							self.axis[a].motor.runspeed = float('nan')
							changed[0].add(a)
						if not self.axis[a].motor.sleeping:
							self.axis[a].motor.sleeping = True
							changed[0].add(a)
					for e in range(self.maxextruders):
						if not math.isnan(self.extruder[e].motor.runspeed):
							self.extruder[e].motor.runspeed = float('nan')
							changed[1].add(e)
						if not self.extruder[e].motor.sleeping:
							self.extruder[e].motor.sleeping = True
							changed[1].add(e)
				if what & 2:
					for e in range(self.maxextruders):
						if not math.isnan(self.extruder[e].temp.value):
							self.extruder[e].temp.value = float('nan')
							changed[1].add(e)
					for t in range(self.maxtemps):
						if not math.isnan(self.temp[t].value):
							self.temp[t].value = float('nan')
							changed[2].add(t)
				for a in changed[0]:
					self._axis_update(a)
				for e in changed[1]:
					self._extruder_update(e)
				for t in changed[2]:
					self._temp_update(t)
				continue
			elif packet[0] == self.rcommand['SENSE']:
				if not self.initialized:
					# Ignore this while connecting.
					continue
				w = ord(packet[1])
				which = w & 0x7f
				state = bool(w & 0x80)
				pos = struct.unpack('<f', packet[2:])[0]
				if which not in self.sense:
					self.sense[which] = []
				self.sense[which].append((state, pos))
				s = 0
				tocall = []
				while s < len(self.movecb):
					if self.movecb[s][0] is not False and ((self.movecb[s][0] is True and len(self.sense) > 0) or any(x in self.sense for x in self.movecb[s][0])):
						call_queue.append((self.movecb.pop(s)[1], [self.movewait == 0]))
					else:
						s += 1
				continue
			elif packet[0] == self.rcommand['SERIAL_RX']:
				port = ord(packet[1])
				data = packet[2:]
				log('Serial data received on port %d: %s' % (port, data))
				self._broadcast(None, 'serial', [port, data])
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
				r[2] = '\x00'
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
				else:
					log('stall response waiting for ack')
					# ack[1] == 'stall'
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
	def _read(self, channel): # {{{
		if not self._send_packet(struct.pack('<BB', self.command['READ'], channel)):
			return None
		reply = self._get_reply()
		assert reply[0] == self.rcommand['DATA']
		return reply[1:]
	# }}}
	def _read_variables(self): # {{{
		data = self._read(1)
		if data is None:
			return False
		#log('%d' % len(data[self.namelen:]))
		self.name = unicode(data[:self.namelen].rstrip('\0'), 'utf-8', 'replace')
		self.num_axes, self.num_extruders, self.num_temps, self.num_gpios, self.printer_type, self.led_pin, self.probe_pin, self.room_T, self.motor_limit, self.temp_limit, self.feedrate, angle = struct.unpack('<BBBBBHHfLLff', data[self.namelen:])
		self.pos = (self.pos + [float('nan')] * self.num_axes)[:self.num_axes]
		self.angle = math.degrees(angle)
		return True
	# }}}
	def _write_variables(self): # {{{
		data = (self.name.encode('utf-8') + chr(0) * self.namelen)[:self.namelen] + struct.pack('<BBBBBHHfLLff', self.num_axes, self.num_extruders, self.num_temps, self.num_gpios, self.printer_type, self.led_pin, self.probe_pin, self.room_T, self.motor_limit, self.temp_limit, self.feedrate, math.radians(self.angle))
		if not self._send_packet(struct.pack('<BB', self.command['WRITE'], 1) + data):
			return False
		self._variables_update()
		return True
	# }}}
	def _variables_update(self, target = None): # {{{
		if not self.initialized:
			return
		self._broadcast(target, 'variables_update', [self.name, self.num_axes, self.num_extruders, self.num_temps, self.num_gpios, self.printer_type, self.led_pin, self.probe_pin, self.room_T, self.motor_limit, self.temp_limit, self.feedrate, self.angle, self.paused])
	# }}}
	def _write_axis(self, which): # {{{
		data = self.axis[which].write()
		if not self._send_packet(struct.pack('<BB', self.command['WRITE'], 2 + which) + data):
			return False
		self._axis_update(which)
		return True
	# }}}
	def _axis_update(self, which, target = None): # {{{
		if not self.initialized:
			return
		self._broadcast(target, 'axis_update', which, [
			[self.axis[which].motor.step_pin, self.axis[which].motor.dir_pin, self.axis[which].motor.enable_pin, self.axis[which].motor.steps_per_mm, self.axis[which].motor.limit_v, self.axis[which].motor.max_v, self.axis[which].motor.max_a, self.axis[which].motor.max_steps, self.axis[which].motor.runspeed, self.axis[which].motor.sleeping],
			self.axis[which].limit_min_pin, self.axis[which].limit_max_pin, self.axis[which].sense_pin, self.axis[which].limit_pos, self.axis[which].axis_min, self.axis[which].axis_max, self.axis[which].motor_min, self.axis[which].motor_max, self.axis[which].park, self.axis[which].delta_length, self.axis[which].delta_radius, self.axis[which].offset])
	# }}}
	def _write_extruder(self, which): # {{{
		data = self.extruder[which].write()
		if not self._send_packet(struct.pack('<BB', self.command['WRITE'], 2 + self.maxaxes + which) + data):
			return False
		self._extruder_update(which)
		return True
	# }}}
	def _extruder_update(self, which, target = None): # {{{
		if not self.initialized:
			return
		self._broadcast(target, 'extruder_update', which, [
			[self.extruder[which].motor.step_pin, self.extruder[which].motor.dir_pin, self.extruder[which].motor.enable_pin, self.extruder[which].motor.steps_per_mm, self.extruder[which].motor.limit_v, self.extruder[which].motor.max_v, self.extruder[which].motor.max_a, self.extruder[which].motor.max_steps, self.extruder[which].motor.runspeed, self.extruder[which].motor.sleeping],
			[self.extruder[which].temp.power_pin, self.extruder[which].temp.thermistor_pin, self.extruder[which].temp.R0, self.extruder[which].temp.R1, self.extruder[which].temp.Rc, self.extruder[which].temp.Tc, self.extruder[which].temp.beta, self.extruder[which].temp.core_C, self.extruder[which].temp.shell_C, self.extruder[which].temp.transfer, self.extruder[which].temp.radiation, self.extruder[which].temp.power, self.extruder[which].temp.value],
			self.extruder[which].filament_heat, self.extruder[which].nozzle_size, self.extruder[which].filament_size])
	# }}}
	def _write_temp(self, which): # {{{
		data = self.temp[which].write()
		if not self._send_packet(struct.pack('<BB', self.command['WRITE'], 2 + self.maxaxes + self.maxextruders + which) + data):
			return False
		self._temp_update(which)
		return True
	# }}}
	def _temp_update(self, which, target = None): # {{{
		if not self.initialized:
			return
		self._broadcast(target, 'temp_update', which, [self.temp[which].power_pin, self.temp[which].thermistor_pin, self.temp[which].R0, self.temp[which].R1, self.temp[which].Rc, self.temp[which].Tc, self.temp[which].beta, self.temp[which].core_C, self.temp[which].shell_C, self.temp[which].transfer, self.temp[which].radiation, self.temp[which].power, self.temp[which].value])
	# }}}
	def _write_gpio(self, which): # {{{
		data = self.gpio[which].write()
		if not self._send_packet(struct.pack('<BB', self.command['WRITE'], 2 + self.maxaxes + self.maxextruders + self.maxtemps + which) + data):
			return False
		self._gpio_update(which)
		return True
	# }}}
	def _gpio_update(self, which, target = None): # {{{
		if not self.initialized:
			return
		self._broadcast(target, 'gpio_update', which, [self.gpio[which].pin, self.gpio[which].state, self.gpio[which].master, self.gpio[which].value])
	# }}}
	def _export_motor(self, name, obj, index): # {{{
		return '[%s %d]\r\n' % (name, index) + ''.join(['%s = %d\r\n' % (x, getattr(obj, x)) for x in ('step_pin', 'dir_pin', 'enable_pin')]) + ''.join(['%s = %f\r\n' % (x, getattr(obj, x)) for x in ('steps_per_mm', 'limit_v', 'max_v', 'max_a', 'max_steps')])
	# }}}
	def _export_temp(self, name, obj, index): # {{{
		return '[%s %d]\r\n' % (name, index) + ''.join(['%s = %d\r\n' % (x, getattr(obj, x)) for x in ('power_pin', 'thermistor_pin')]) + ''.join(['%s = %f\r\n' % (x, getattr(obj, x)) for x in ('R0', 'R1', 'Rc', 'Tc', 'beta', 'core_C', 'shell_C', 'transfer', 'radiation', 'power')])
	# }}}
	def _use_probemap(self, x, y, z): # {{{
		'''Return corrected z according to self.gcode_probemap.'''
		# Map = [[x0, y0, x1, y1], [nx, ny], [[...], [...], ...]]
		if self.gcode_probemap is None:
			return z
		p = self.gcode_probemap
		x -= p[0][0]
		y -= p[0][1]
		x /= (p[0][2] - p[0][0]) / p[1][0]
		y /= (p[0][3] - p[0][1]) / p[1][1]
		if x < 0:
			x = 0
		if x >= p[1][0]:
			x = p[1][0] - 1
		if y < 0:
			y = 0
		if y >= p[1][1]:
			y = p[1][1] - 1
		ix = int(x)
		iy = int(y)
		fx = x - ix
		fy = y - iy
		l = p[2][iy][ix] * (1 - fy) + p[2][iy + 1][ix] * fy
		r = p[2][iy][ix + 1] * (1 - fy) + p[2][iy + 1][ix + 1] * fy
		return z + l * (1 - fx) + r * fx
	# }}}
	def _do_gcode(self): # {{{
		while len(self.gcode) > 0:
			cmd, args, message = self.gcode[0]
			cmd = tuple(cmd)
			#log('Running %s %s' % (cmd, args))
			if cmd[0] == 'S':
				# Spindle speed; not supported, but shouldn't error.
				pass
			elif cmd == ('G', 1):
				#log(repr(args))
				sina, cosa = self.gcode_angle
				target = cosa * args['X'] - sina * args['Y'] + self.gcode_ref[0], cosa * args['Y'] + sina * args['X'] + self.gcode_ref[1]
				if self._use_probemap and self.gcode_probemap:
					source = cosa * args['x'] - sina * args['y'] + self.gcode_ref[0], cosa * args['y'] + sina * args['x'] + self.gcode_ref[1]
					if args[x] is not None:
						num = max([abs(target[t] - source[t]) / (abs(self.gcode_probemap[0][t + 2] - self.gcode_probemap[0][t]) / self.gcode_probemap[1][t]) for t in range(2)])
					else:
						num = 1
					for t in range(num):
						target = [source[tt] + (target[tt] - source[tt]) * (t + 1.) / num for tt in range(2)]
					self.goto([target[0], target[1], self._use_probemap(target[0], target[1], args['Z'])], e = args['E'], f0 = args['f'], f1 = args['F'])[1](None)
				self.goto([target[0], target[1], args['Z']], e = args['E'], f0 = args['f'], f1 = args['F'])[1](None)
			elif cmd in (('G', 28), ('M', 6)):
				# Home or tool change; same response: park
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				self.gcode.pop(0)
				return self.park(True, self._do_gcode)[1](None)
			elif cmd == ('G', 4):
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				if 'P' in args:
					self.gcode.pop(0)
					self.gcode_wait = time.time() + float(args['P']) / 1000
					return
			elif cmd == ('G', 94):
				# Set feed rate mode to units per minute; we don't support anything else, but shouldn't error on this request.
				pass
			elif cmd == ('M', 0):
				# Wait.
				self.gcode.pop(0)
				return self.request_confirmation(message)[1](None)
			elif cmd[0] == 'M' and cmd[1] in (3, 4):
				# Start spindle; we don't support speed or direction, so M3 and M4 are equal.
				self.set_gpio(1, state = 1)
			elif cmd == ('M', 5):
				# Stop spindle.
				self.set_gpio(1, state = 0)
			elif cmd == ('M', 9):
				# Coolant off.  We don't support coolant, but turning it off is not an error.
				pass
			elif cmd == ('M', 42):
				g = args['P']
				value = args['S']
				self.set_gpio(int(g), state = int(value))
			elif cmd == ('M', 84):
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				self.sleep_all()
			elif cmd == ('M', 104):
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				self.settemp_extruder(args['E'], args['S'])
			elif cmd == ('M', 106):	# Fan on
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				self.set_gpio(0, state = 1)
			elif cmd == ('M', 107): # Fan off
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				self.set_gpio(0, state = 0)
			elif cmd == ('M', 109):
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				e = args['E']
				if 'S' in args:
					self.settemp_extruder(e, args['S'])
				else:
					args['S'] = self.extruder[e].temp.value
				if not math.isnan(args['S']) and self.pin_valid(self.extruder[e].temp.thermistor_pin):
					self.clear_alarm()
					self.waittemp_extruder(e, args['S'])
					self.gcode.pop(0)
					return self.wait_for_temp_extruder(e)[1](None)
			elif cmd == ('M', 116):
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				e = args['E']
				etemp = self.extruder[e].temp.value
				if not math.isnan(etemp) and self.pin_valid(self.extruder[e].temp.thermistor_pin):
					self.clear_alarm()
					self.waittemp_extruder(e, etemp)
					self.gcode.pop(0) # TODO: wait for extruder AND bed.
					return self.wait_for_temp_extruder(e)[1](None)
				if not math.isnan(self.btemp) and self.pin_valid(self.temp[0].thermistor_pin):
					self.clear_alarm()
					self.waittemp_temp(0, self.btemp)
					self.gcode.pop(0)
					return self.wait_for_temp_temp(0)[1](None)
			elif cmd == ('M', 140):
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				self.btemp = args['S']
				self.settemp_temp(0, self.btemp)
			elif cmd == ('M', 190):
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				if 'S' in args:
					self.btemp = args['S']
					self.settemp_temp(0, self.btemp)
				if not math.isnan(self.btemp) and self.pin_valid(self.temp[0].thermistor_pin):
					self.gcode.pop(0)
					self.waittemp_temp(0, self.btemp)
					return self.wait_for_temp_temp(0)[1](None)
			elif cmd == ('SYSTEM', 0):
				if not self.flushing:
					self.flushing = True
					return self.flush()[1](None)
				log('running system command: %s' % message)
				os.system(message)
			self.flushing = False
			if len(self.gcode) > 0:
				self.gcode.pop(0)
		if self.gcode_id is not None and not self.flushing:
			self.flushing = True
			return self.flush()[1](None)
		self._print_done(True, 'completed')
	# }}}
	def _print_done(self, complete, reason): # {{{
		if self.queue_info is None and self.gcode_id is not None:
			log('done %d %s' % (complete, reason))
			self._send(self.gcode_id, 'return', (complete, reason))
			self.gcode_id = None
			self.gcode = []
			self.flushing = False
			self.gcode_wait = None
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
			self.set_variables(printer_type = self.orig_printer_type)
			if self.home_cb in self.movecb:
				self.movecb.remove(self.home_cb)
				self._send(self.home_id, 'return', None)
		if self.probe_cb in self.movecb:
			#log('killing prober')
			self.movecb.remove(self.probe_cb)
			self.probe_cb(False)
	# }}}
	def _do_queue(self): # {{{
		#log('queue %s' % repr((self.queue_pos, len(self.queue), self.resuming, self.wait)))
		while not self.wait and (self.queue_pos < len(self.queue) or self.resuming):
			if self.queue_pos >= len(self.queue):
				#log('doing resume to %d/%d' % (self.queue_info[0], len(self.queue_info[2])))
				self.queue = self.queue_info[2]
				self.queue_pos = self.queue_info[0]
				self.movecb = self.queue_info[3]
				self.flushing = self.queue_info[4]
				self.gcode_wait = self.queue_info[5]
				self.resuming = False
				self.queue_info = None
				if self.queue_pos >= len(self.queue):
					break
			id, axes, e, f0, f1, which, cb = self.queue[self.queue_pos]
			self.queue_pos += 1
			a = {}
			if isinstance(axes,(list, tuple)):
				for i, axis in enumerate(axes):
					if not math.isnan(axis):
						a[i] = axis
			else:
				for i, axis in tuple(axes.items()):
					if not math.isnan(axis):
						a[int(i)] = axis
			axes = a
			targets = [0] *(((2 + self.num_axes + self.num_extruders - 1) >> 3) + 1)
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
				assert axis < self.num_axes
				if math.isnan(axes[axis]):
					continue
				if self.axis[axis].motor.sleeping:
					self.axis[axis].motor.sleeping = False
					self._axis_update(axis)
				self.pos[axis] = axes[axis]
				targets[(axis + 2) >> 3] |= 1 <<((axis + 2) & 0x7)
				args += struct.pack('<f', axes[axis])
				#log('axis %d: %f' %(axis, axes[axis]))
			if e is not None:
				targets[(2 + self.num_axes + which) >> 3] |= 1 <<((2 + self.num_axes + which) & 0x7)
				args += struct.pack('<f', e)
				if self.extruder[which].motor.sleeping:
					self.extruder[which].motor.sleeping = False
					self._extruder_update(which)
			if cb:
				self.movewait += 1
				#log('movewait +1 -> %d' % self.movewait)
				p = chr(self.command['GOTOCB'])
			else:
				p = chr(self.command['GOTO'])
			self._send_packet(p + ''.join([chr(t) for t in targets]) + args)
			if id is not None:
				self._send(id, 'return', None)
		if not self.wait:
			self.queue = []
			self.queue_pos = 0
	# }}}
	def _do_home(self, done = None): # {{{
		#log('do_home: %s %s' % (self.home_phase, done))
		# 0: move to max limit switch or find sense switch.
		# 1: move to min limit switch or find sense switch.
		# 2: back off.
		# 3: move slowly to switch.
		# 4: set current position; move to center (delta only).
		if self.home_phase is None:
			self.home_phase = 0
			# Initial call; start homing.
			self.orig_printer_type = self.printer_type
			self.set_variables(printer_type = 0)
			# If it is currently moving, doing the things below without pausing causes stall responses.
			self.pause(True, False)
			for a in range(self.num_axes):
				self.sleep_axis(a, False)
				self.axis[a].set_current_pos(self.axis[a].motor_min)
			self.home_target = {}
			self.limits.clear()
			self.sense.clear()
			for a in range(self.num_axes):
				if self.pin_valid(self.axis[a].limit_max_pin) or (not self.pin_valid(self.axis[a].limit_min_pin) and self.pin_valid(self.axis[a].sense_pin)):
					self.home_target[a] = self.axis[a].limit_pos + 1 - self.axis[a].offset
			if len(self.home_target) > 0:
				self.home_cb[0] = self.home_target.keys()
				self.movecb.append(self.home_cb)
				self.goto(self.home_target, cb = True)[1](None)
				return
			# Fall through.
		if self.home_phase == 0:
			self.pause(True, False)	# Needed if we hit a sense switch.
			found_limits = False
			for a in self.limits.keys() + self.sense.keys():
				if a in self.home_target:
					self.home_target.pop(a)
					found_limits = True
			# Repeat until move is done, or all limits are hit.
			if (not done or found_limits) and len(self.home_target) > 0:
				self.home_cb[0] = self.home_target.keys()
				self.movecb.append(self.home_cb)
				self.goto(self.home_target, cb = True)[1](None)
				return
			self.home_phase = 1
			for a in range(self.num_axes):
				if a not in self.limits and (self.pin_valid(self.axis[a].limit_min_pin) or self.pin_valid(self.axis[a].sense_pin)):
					self.axis[a].set_current_pos(self.axis[a].motor_max)
					self.home_target[a] = self.axis[a].limit_pos - 1 - self.axis[a].offset
			if len(self.home_target) > 0:
				self.home_cb[0] = self.home_target.keys()
				self.movecb.append(self.home_cb)
				self.goto(self.home_target, cb = True)[1](None)
				return
			# Fall through.
		if self.home_phase == 1:
			self.pause(True, False)	# Needed if we hit a sense switch.
			found_limits = False
			for a in self.limits.keys() + self.sense.keys():
				if a in self.home_target:
					self.home_target.pop(a)
					found_limits = True
			# Repeat until move is done, or all limits are hit.
			if (not done or found_limits) and len(self.home_target) > 0:
				self.home_cb[0] = self.home_target.keys()
				self.movecb.append(self.home_cb)
				self.goto(self.home_target, cb = True)[1](None)
				return
			if len(self.home_target) > 0:
				log('Warning: not all limits were found during homing')
			self.home_phase = 2
			# Back off.
			self.home_target = {}
			for a in range(self.num_axes):
				self.axis[a].set_current_pos(self.axis[a].limit_pos)
				if self.pin_valid(self.axis[a].limit_max_pin) or (not self.pin_valid(self.axis[a].limit_min_pin) and self.pin_valid(self.axis[a].sense_pin)):
					self.home_target[a] = self.axis[a].limit_pos - 10 - self.axis[a].offset
				elif self.pin_valid(self.axis[a].limit_min_pin):
					self.home_target[a] = self.axis[a].limit_pos + 10 - self.axis[a].offset
			if len(self.home_target) > 0:
				self.home_cb[0] = False
				self.movecb.append(self.home_cb)
				self.goto(self.home_target, cb = True)[1](None)
				return
			# Fall through
		if self.home_phase == 2:
			self.home_phase = 3
			# Goto switch slowly.
			self.home_target = {}
			self.limits.clear()
			self.sense.clear()
			for a in range(self.num_axes):
				if self.pin_valid(self.axis[a].limit_max_pin) or (not self.pin_valid(self.axis[a].limit_min_pin) and self.pin_valid(self.axis[a].sense_pin)):
					self.home_target[a] = self.axis[a].limit_pos + 10 - self.axis[a].offset
				elif self.pin_valid(self.axis[a].limit_min_pin):
					self.home_target[a] = self.axis[a].limit_pos - 10 - self.axis[a].offset
			if len(self.home_target) > 0:
				self.home_cb[0] = self.home_target.keys()
				self.movecb.append(self.home_cb)
				self.goto(self.home_target, f0 = self.home_speed / 20., cb = True)[1](None)
				return
			# Fall through
		if self.home_phase == 3:
			self.pause(True, False)	# Needed if we hit a sense switch.
			found_limits = False
			for a in self.limits.keys() + self.sense.keys():
				if a in self.home_target:
					self.home_target.pop(a)
					found_limits = True
			# Repeat until move is done, or all limits are hit.
			if (not done or found_limits) and len(self.home_target) > 0:
				self.home_cb[0] = self.home_target.keys()
				self.movecb.append(self.home_cb)
				self.goto(self.home_target, cb = True)[1](None)
				return
			if len(self.home_target) > 0:
				log('Warning: not all limits were hit during homing')
			self.home_phase = 4
			# set current position.
			for a in range(self.num_axes):
				if a in self.limits:
					self.axis[a].set_current_pos(self.axis[a].limit_pos)
				elif a in self.sense:
					# Correct for possible extra steps that were done because pausing happened later than hitting the sensor.
					self.axis[a].set_current_pos(self.axis[a].limit_pos + self.sense[a][-1][1] - self.axis[a].get_current_pos()[0])
			if self.orig_printer_type == 1:
				# Goto center.  Since this is only for deltas, assume that switches are not at minimum travel.
				target = float('nan')
				for a in range(3):	# Delta only works for the first 3 axes.
					# Use not to handle nan correctly.
					if not (self.axis[a].limit_pos > target):
						target = self.axis[a].limit_pos
				if not math.isnan(target):
					self.home_cb[0] = False
					self.movecb.append(self.home_cb)
					self.goto([target - self.axis[a].offset for a in range(self.num_axes)], cb = True)[1](None)
					return
			# Fall through.
		if self.home_phase == 4:
			self.set_variables(printer_type = self.orig_printer_type)
			# Same target computation as above.
			if self.orig_printer_type == 1:
				target = float('nan')
				for a in range(3):
					if not (self.axis[a].limit_pos > target):
						target = self.axis[a].limit_pos
				target = [target] * self.num_axes
			else:
				target = [self.axis[a].limit_pos for a in range(self.num_axes)]
			for a in range(self.num_axes):
				self.axis[a].set_current_pos(target[a])
			if self.home_id is not None:
				self._send(self.home_id, 'return', None)
			self.home_phase = None
			if self.home_done_cb is not None:
				call_queue.append((self.home_done_cb, []))
				self.home_done_cb = None
			return
		log('Internal error: invalid home phase')
	# }}}
	def _do_probe(self, id, x, y, angle, speed, phase = 0, good = True): # {{{
		# Map = [[x0, y0, x1, y1], [nx, ny], [[...], [...], ...]]
		if not good:
			self.set_axis(2, limit_min_pin = self.probe_orig_pin)
			self._send(id, 'error', 'aborted')
			return
		p = self.gcode_probemap
		if phase == 0:
			if y >= p[1][1]:
				# Done.
				self._send(id, 'return', self.gcode_probemap)
				return
			# Goto x,y
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, angle, speed, 1, good)
			self.movecb.append(self.probe_cb)
			px = p[0][0] + (p[0][2] - p[0][0]) * x / p[1][0]
			py = p[0][1] + (p[0][3] - p[0][1]) * y / p[1][1]
			self.goto([px * self.gcode_angle[1] - py * self.gcode_angle[0], py * self.gcode_angle[1] + px * self.gcode_angle[0], 3], cb = True)[1](None)
		elif phase == 1:
			# Probe
			self.probe_orig_pin = self.axis[2].limit_min_pin
			self.set_axis(2, limit_min_pin = self.probe_pin)
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, angled, speed, 2, good)
			self.movecb.append(self.probe_cb)
			self.goto({2: -10}, f0 = float(speed) / 13, cb = True)[1](None)
		else:
			self.set_axis(2, limit_min_pin = self.probe_orig_pin)
			# Record result
			p[2][y][x] = self.axis[2].get_current_pos()[1]
			# Retract
			x += 1
			if x >= p[1][0]:
				x = 0
				y += 1
			self.probe_cb[1] = lambda good: self._do_probe(id, x, y, angle, speed, 0, good)
			self.movecb.append(self.probe_cb)
			self.goto({2: 3}, cb = True)[1](None)
	# }}}
	# Subclasses.  {{{
	class Temp: # {{{
		def __init__(self):
			self.value = float('nan')
		def read(self, data):
			self.R0, self.R1, self.Rc, self.Tc, self.beta, self.core_C, self.shell_C, self.transfer, self.radiation, self.power, self.power_pin, self.thermistor_pin = struct.unpack('<ffffffffffHH', data[:44])
			return data[44:]
		def write(self):
			return struct.pack('<ffffffffffHH', self.R0, self.R1, self.Rc, self.Tc, self.beta, self.core_C, self.shell_C, self.transfer, self.radiation, self.power, self.power_pin, self.thermistor_pin)
	# }}}
	class Motor: # {{{
		def __init__(self):
			self.runspeed = 0
			self.sleeping = True
		def read(self, data):
			self.step_pin, self.dir_pin, self.enable_pin, self.steps_per_mm, self.limit_v, self.max_v, self.max_a, self.max_steps = struct.unpack('<HHHffffB', data[:23])
			return data[23:]
		def write(self):
			return struct.pack('<HHHffffB', self.step_pin, self.dir_pin, self.enable_pin, self.steps_per_mm, self.limit_v, self.max_v, self.max_a, self.max_steps)
	# }}}
	class Axis: # {{{
		def __init__(self, printer, id):
			self.printer = printer
			self.id = id
			self.valid = False
			self.motor = Printer.Motor()
		def read(self, data):
			data = self.motor.read(data)
			self.limit_min_pin, self.limit_max_pin, self.sense_pin, self.limit_pos, self.axis_min, self.axis_max, self.motor_min, self.motor_max, self.park, self.delta_length, self.delta_radius, self.offset = struct.unpack('<HHHfffffffff', data)
		def write(self):
			return self.motor.write() + struct.pack('<HHHfffffffff', self.limit_min_pin, self.limit_max_pin, self.sense_pin, self.limit_pos, self.axis_min, self.axis_max, self.motor_min, self.motor_max, self.park, self.delta_length, self.delta_radius, self.offset)
		def set_current_pos(self, pos):
			#log('setting pos of %d to %f' % (self.id, pos))
			if not self.printer._send_packet(struct.pack('<BBf', self.printer.command['SETPOS'], 2 + self.id, pos)):
				return False
			self.valid = True
			return True
		def get_current_pos(self):
			if not self.printer._send_packet(struct.pack('<BB', self.printer.command['GETPOS'], 2 + self.id)):
				return None
			ret = self.printer._get_reply()
			assert ret[0] == self.printer.rcommand['POS']
			return struct.unpack('<ff', ret[1:])
	# }}}
	class Extruder: # {{{
		def __init__(self):
			self.motor = Printer.Motor()
			self.temp = Printer.Temp()
		def read(self, data):
			data = self.motor.read(data)
			data = self.temp.read(data)
			self.filament_heat, self.nozzle_size, self.filament_size = struct.unpack('<fff', data)
		def write(self):
			return self.motor.write() + self.temp.write() + struct.pack('<fff', self.filament_heat, self.nozzle_size, self.filament_size)
	# }}}
	class Gpio: # {{{
		def read(self, data):
			self.pin, self.state, self.master, self.value = struct.unpack('<HBBf', data)
		def write(self):
			return struct.pack('<HBBf', self.pin, self.state, self.master, self.value)
	# }}}
	# }}}
	# }}}
	# Useful commands.  {{{
	def reset(self): # {{{
		log('%s resetting and dying.' % self.name)
		return self._send_packet(struct.pack('<BB', self.command['RESET'], 0))
	# }}}
	def die(self): # {{{
		log('%s dying as requested by host.' % self.name)
		return (WAIT, WAIT)
	# }}}
	@delayed
	def flush(self, id): # {{{
		self.movecb.append((False, lambda w: (id is None or self._send(id, 'return', w) or True) and self._do_gcode()))
		self.goto(cb = True)[1](None)
	# }}}
	@delayed
	def probe(self, id, area, density, angle = 0, speed = 3): # {{{
		if not self.pin_valid(self.probe_pin):
			self._send(id, 'return', None)
			return
		self.gcode_probemap = [area, density, [[None for x in range(density[0])] for y in range(density[1])]]
		self.gcode_angle = math.sin(angle), math.cos(angle)
		self._do_probe(id, 0, 0, angle, speed)
	# }}}
	@delayed
	def goto(self, id, axes = {}, e = None, f0 = None, f1 = None, which = 0, cb = False): # {{{
		#log('goto %s' % repr(axes))
		self.queue.append((id, axes, e, f0, f1, which, cb))
		if not self.wait:
			self._do_queue()
	# }}}
	def run(self, channel, speed):	# speed: float; 0 means off. # {{{
		channel = int(channel)
		if channel >= 2 and channel < 2 + self.num_axes:
			if speed != 0 and not math.isnan(speed):
				#log(repr(self.pos))
				self.pos[channel - 2] = float('nan')
				self.axis[channel - 2].valid = False
			self.axis[channel - 2].motor.runspeed = speed
			self.axis[channel - 2].motor.sleeping = False
			self._axis_update(channel - 2)
		else:
			self.extruder[channel - 2 - self.maxaxes].motor.runspeed = speed
			self.extruder[channel - 2 - self.maxaxes].motor.sleeping = False
			self._extruder_update(channel - 2 - self.maxaxes)
		return self._send_packet(struct.pack('<BBf', self.command['RUN'], channel, speed))
	def run_axis(self, which, speed): # {{{
		return self.run(2 + which, speed)
	# }}}
	def run_extruder(self, which, speed): # {{{
		return self.run(2 + self.maxaxes + which, speed)
	# }}}
	# }}}
	def sleep(self, channel, sleeping = True): # {{{
		channel = int(channel)
		if channel >= 2 and channel < 2 + self.maxaxes:
			self.axis[channel - 2].motor.sleeping = sleeping
			if sleeping:
				self.axis[channel - 2].valid = False
			self._axis_update(channel - 2)
		else:
			self.extruder[channel - 2 - self.maxaxes].motor.sleeping = sleeping
			self._extruder_update(channel - 2 - self.maxaxes)
		return self._send_packet(struct.pack('<BB', self.command['SLEEP'],(channel & 0x7f) |(0x80 if sleeping else 0)))
	def sleep_axis(self, which, sleeping = True): # {{{
		return self.sleep(2 + which, sleeping)
	# }}}
	def sleep_extruder(self, which, sleeping = True): # {{{
		return self.sleep(2 + self.maxaxes + which, sleeping)
	# }}}
	def sleep_all(self): # {{{
		for axis in range(self.num_axes):
			self.sleep_axis(axis)
		for extruder in range(self.num_extruders):
			self.sleep_extruder(extruder)
	# }}}
	# }}}
	def settemp(self, channel, temp): # {{{
		channel = int(channel)
		if channel < 2 + self.maxaxes + self.maxextruders:
			self.extruder[channel - 2 - self.maxaxes].temp.value = temp
			self._extruder_update(channel - 2 - self.maxaxes)
		else:
			self.temp[channel - 2 - self.maxaxes - self.maxextruders].value = temp
			self._temp_update(channel - 2 - self.maxaxes - self.maxextruders)
		return self._send_packet(struct.pack('<BBf', self.command['SETTEMP'], channel, temp))
	def settemp_extruder(self, which, temp):	# {{{
		return self.settemp(2 + self.maxaxes + which, temp)
	# }}}
	def settemp_temp(self, which, temp):	# {{{
		return self.settemp(2 + self.maxaxes + self.maxextruders + which, temp)
	# }}}
	# }}}
	def waittemp(self, channel, min, max = None): # {{{
		channel = int(channel)
		if min is None:
			min = float('nan')
		if max is None:
			max = float('nan')
		return self._send_packet(struct.pack('<BBff', self.command['WAITTEMP'], channel, min, max))
	def waittemp_extruder(self, which, min, max = None):	# {{{
		return self.waittemp(2 + self.maxaxes + which, min, max)
	# }}}
	def waittemp_temp(self, which, min, max = None):	# {{{
		return self.waittemp(2 + self.maxaxes + self.maxextruders + which, min, max)
	# }}}
	# }}}
	def readtemp(self, channel): # {{{
		channel = int(channel)
		if not self._send_packet(struct.pack('<BB', self.command['READTEMP'], channel)):
			return None
		ret = self._get_reply()
		assert ret[0] == self.rcommand['TEMP']
		return struct.unpack('<f', ret[1:])[0]
	def readtemp_extruder(self, which):	# {{{
		return self.readtemp(2 + self.maxaxes + which)
	# }}}
	def readtemp_temp(self, which):	# {{{
		return self.readtemp(2 + self.maxaxes + self.maxextruders + which)
	# }}}
	# }}}
	def readpower(self, channel): # {{{
		channel = int(channel)
		if not self._send_packet(struct.pack('<BB', self.command['READPOWER'], channel)):
			return None
		ret = self._get_reply()
		assert ret[0] == self.rcommand['POWER']
		return struct.unpack('<LL', ret[1:])
	def readpower_extruder(self, which):	# {{{
		return self.readpower(2 + self.maxaxes + which)
	# }}}
	def readpower_temp(self, which):	# {{{
		return self.readpower(2 + self.maxaxes + self.maxextruders + which)
	# }}}
	# }}}
	def readpin(self, pin): # {{{
		if not self._send_packet(struct.pack('<BB', self.command['READPIN'], pin)):
			return None
		ret = self._get_reply()
		assert ret[0] == self.rcommand['PIN']
		return bool(ord(ret[1]))
	# }}}
	def load(self, channel): # {{{
		channel = int(channel)
		if not self._send_packet(struct.pack('<BB', self.command['LOAD'], channel)):
			return False
		if channel == 1:
			if not self._read_variables():
				return False
			self._variables_update()
		else:
			data = self._read(channel)
			if data is None:
				return False
			if 2 <= channel < 2 + self.maxaxes:
				self.axis[channel - 2].read(data)
				self._axis_update(channel - 2)
			elif 2 + self.maxaxes <= channel < 2 + self.maxaxes + self.maxextruders:
				self.extruder[channel - 2 - self.maxaxes].read(data)
				self._extruder_update(channel - 2 - self.maxaxes)
			elif 2 + self.maxaxes + self.maxextruders <= channel < 2 + self.maxaxes + self.maxextruders + self.maxtemps:
				self.temp[channel - 2 - self.maxaxes - self.maxextruders].read(data)
				self._temp_update(channel - 2 - self.maxaxes - self.maxextruders)
			else:
				assert 2 + self.maxaxes + self.maxextruders + self.maxtemps <= channel < 2 + self.maxaxes + self.maxextruders + self.maxtemps + self.maxgpios
				self.gpio[channel - 2 - self.maxaxes - self.maxextruders - self.maxtemps].read(data)
				self._gpio_update(channel - 2 - self.maxaxes - self.maxextruders - self.maxtemps)
		return True
	def load_variables(self): # {{{
		return self.load(1)
	# }}}
	def load_axis(self, which): # {{{
		return self.load(2 + which)
	# }}}
	def load_extruder(self, which): # {{{
		return self.load(2 + self.maxaxes + which)
	# }}}
	def load_temp(self, which): # {{{
		return self.load(2 + self.maxaxes + self.maxextruders + which)
	# }}}
	def load_all(self): # {{{
		for i in range(1, 2 + self.maxaxes + self.maxextruders + self.maxtemps + self.maxgpios):
			log('loading %d' % i)
			if not self.load(i):
				return False
		return True
	# }}}
	# }}}
	def save(self, channel): # {{{
		self._broadcast(None, 'blocked', 'saving')
		ret = self._send_packet(struct.pack('<BB', self.command['SAVE'], int(channel)))
		self._broadcast(None, 'blocked', None)
		return ret
	def save_variables(self): # {{{
		return self.save(1)
	# }}}
	def save_axis(self, which): # {{{
		return self.save(2 + which)
	# }}}
	def save_extruder(self, which): # {{{
		return self.save(2 + self.maxaxes + which)
	# }}}
	def save_temp(self, which): # {{{
		return self.save(2 + self.maxaxes + self.maxextruders + which)
	# }}}
	def save_all(self): # {{{
		for i in range(1, 2 + self.maxaxes + self.maxextruders + self.maxtemps + self.maxgpios):
			log('saving %d' % i)
			self.save(i)
	# }}}
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
		self._variables_update()
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
				#log('pausing %d %d %d %d' % (self.queue_info is None, len(self.queue), self.queue_pos, reply[1]))
				if store and self.queue_info is None and len(self.queue) > 0 and self.queue_pos - reply[1] >= 0:
					#log('pausing gcode %d/%d/%d' % (self.queue_pos, reply[1], len(self.queue)))
					self.queue_info = [self.queue_pos - reply[1], [a.get_current_pos() for a in self.axis], self.queue, self.movecb, self.flushing, self.gcode_wait]
				else:
					#log('stopping')
					while len(self.movecb) > 0:
						call_queue.extend([(x[1], [True]) for x in self.movecb])
					self.paused = False
				self.queue = []
				self.movecb = []
				self.flushing = False
				self.gcode_wait = None
				self.queue_pos = 0
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
	def home(self, id, speed = 5, cb = None): # {{{
		#log('homing')
		self.home_id = id
		self.home_speed = speed
		self.home_done_cb = cb
		self._do_home()
	# }}}
	@delayed
	def park(self, id, keep_gcode = False, cb = None): # {{{
		#log('parking')
		if not keep_gcode:
			#log('dumping gcode')
			self.gcode = []
		if any(self.axis[a].motor.sleeping for a in range(self.num_axes)):
			#log('homing')
			self.home(cb = lambda: self.park(keep_gcode, cb)[1](id))[1](None)
			return
		def park_cb(w):
			#log('park cb')
			if cb:
				call_queue.append((cb, []))
			if id is not None:
				self._send(id, 'return', None)
		self.movecb.append((False, park_cb))
		self.goto([self.axis[a].park for a in range(self.num_axes)], cb = True)[1](None)
	# }}}
	def audio_play(self, name, axes = None, extruders = None): # {{{
		assert os.path.basename(name) == name
		self.audiofile = open(os.path.join(self.audiodir, name), 'rb')
		channels = [0] *(((2 + self.num_axes + self.num_extruders - 1) >> 3) + 1)
		for axis in range(self.num_axes):
			if axes is None or axis in axes:
				channels[(axis + 2) >> 3] |= 1 <<((axis + 2) & 0x7)
		for extruder in range(self.num_extruders):
			if extruders is None or extruder in extruders:
				channels[(2 + self.num_axes + extruder) >> 3] |= 1 <<((2 + self.num_axes + extruder) & 0x7)
		us_per_bit = self.audiofile.read(2)
		if not self._send_packet(chr(self.command['AUDIO_SETUP']) + us_per_bit + ''.join([chr(x) for x in channels])):
			return False
		while self.audiofile is not None:
			while self.waitaudio:
				glib.Mainloop.iteration() #TODO
			data = self.audiofile.read(self.audio_fragment_size)
			if len(data) < self.audio_fragment_size:
				self.audiofile = None
				return True
			if not self._send_packet(chr(self.command['AUDIO_DATA']) + data, audio = True):
				return False
		return True
	# }}}
	def audio_load(self, name, data): # {{{
		assert os.path.basename(name) == name
		wav = wave.open(io.StringIO(base64.b64decode(data)))
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
		if sense is not False and((sense is True and len(self.sense) > 0) or sense in self.sense):
			ret()
		else:
			self.movecb.append((sense, ret))
	# }}}
	@delayed
	def wait_for_limits(self, id, num = 1): # {{{
		ret = lambda: (id is None or self._send(id, 'return', None) or True) and self._do_gcode()
		if len(self.limits) >= num:
			ret()
		else:
			self.limitcb.append((num, ret))
	# }}}
	def wait_for_temp_extruder(self, which = None): # {{{
		return self.wait_for_temp(2 + self.maxaxes + which if which is not None else None)
	# }}}
	def wait_for_temp_temp(self, which = None): # {{{
		return self.wait_for_temp(2 + self.maxaxes + self.maxextruders + which if which is not None else None)
	# }}}
	@delayed
	def wait_for_temp(self, id, which = None): # {{{
		ret = lambda: (id is None or self._send(id, 'return', None) or True) and self._do_gcode()
		if(which is None and len(self.alarms) > 0) or which in self.alarms:
			ret()
		else:
			self.tempcb.append((which, ret))
	# }}}
	def clear_alarm(self, which = None): # {{{
		if which is None:
			self.alarms.clear()
		else:
			self.alarms.discard(which)
	# }}}
	def get_limits(self, axis = None):	# {{{
		if axis is None:
			return self.limits
		if axis in self.limits:
			return self.limits[axis]
		return None
	# }}}
	def clear_limits(self):	# {{{
		self.limits.clear()
	# }}}
	def get_sense(self, axis = None):	# {{{
		if axis is None:
			return self.sense
		if axis in self.sense:
			return self.sense[axis]
		return []
	# }}}
	def clear_sense(self):	# {{{
		self.sense.clear()
	# }}}
	def get_position(self):	# {{{
		return self.pos
	# }}}
	def pin_valid(self, pin):	# {{{
		return(pin & 0x100) == 0
	# }}}
	def axis_valid(self, axis):	# {{{
		return self.axis[axis].valid
	# }}}
	def export_settings(self): # {{{
		message = '[general]\r\n'
		# Don't export the name.
		#message += 'name=' + self.name.replace('\\', '\\\\').replace('\n', '\\n') + '\r\n'
		message += ''.join(['%s = %d\r\n' % (x, getattr(self, x)) for x in ('num_axes', 'num_extruders', 'num_temps', 'printer_type', 'led_pin', 'probe_pin', 'motor_limit', 'temp_limit')])
		message += ''.join(['%s = %f\r\n' % (x, getattr(self, x)) for x in ('room_T', 'feedrate', 'angle')])
		for a in range(self.maxaxes):
			message += '[axis %d]\r\n' % a
			message += ''.join(['%s = %d\r\n' % (x, getattr(self.axis[a], x)) for x in ('limit_min_pin', 'limit_max_pin', 'sense_pin')]) + ''.join(['%s = %f\r\n' % (x, getattr(self.axis[a], x)) for x in ('limit_pos', 'axis_min', 'axis_max', 'motor_min', 'motor_max', 'park', 'delta_length', 'delta_radius', 'offset')])
			message += self._export_motor('axis_motor', self.axis[a].motor, a)
		for e in range(self.maxextruders):
			message += '[extruder %d]\r\n' % e
			message += ''.join(['%s = %f\r\n' % (x, getattr(self.extruder[e], x)) for x in ('filament_heat', 'nozzle_size', 'filament_size')])
			message += self._export_motor('extruder_motor', self.extruder[e].motor, e)
			message += self._export_temp('extruder_temp', self.extruder[e].temp, e)
		for t in range(self.maxtemps):
			message += self._export_temp('temp', self.temp[t], t)
		for g in range(self.maxgpios):
			message += '[gpio %d]\r\n' % g
			message += ''.join(['%s = %d\r\n' % (x, getattr(self.gpio[g], x)) for x in ('pin', 'state', 'master')])
			message += ''.join(['%s = %f\r\n' % (x, getattr(self.gpio[g], x)) for x in ('value',)])
		return message
	# }}}
	def import_settings(self, settings, filename = None): # {{{
		section = 'general'
		index = None
		regexp = re.compile('\s*\[(general|(axis|axis_motor|extruder|extruder_motor|extruder_temp|temp|gpio)\s+(\d+))\]\s*$|\s*(\w+)\s*=\s*(.*?)\s*$|\s*(?:#.*)?$')
		errors = []
		var_changed = False
		changed = {
				'axis': [False] * self.maxaxes,
				'extruder': [False] * self.maxextruders,
				'temp': [False] * self.maxtemps,
				'gpio': [False] * self.maxgpios}
		for l in settings.split('\n'):
			r = regexp.match(l)
			if not r:
				errors.append((l, 'syntax error'))
				continue
			if r.group(1):
				# New section.
				if r.group(3):
					section = r.group(2)
					index = int(r.group(3))
					type = section.split('_')[0]
					if index >= len(changed[type]):
						errors.append((l, 'index out of range'))
						continue
					changed[type][index] = True
				else:
					section = r.group(1)
					index = None
					var_changed = True
				continue
			if not r.group(4):
				# Comment.
				continue
			key = r.group(4)
			value = r.group(5)
			if key != 'name':
				if key.endswith('limit') or key.endswith('pin') or key.startswith('num'):
					value = int(value)
				else:
					value = float(value)
			try:
				if section == 'general':
					setattr(self, key, value)
				else:
					if '_' in section:
						main, part = section.split('_')
						setattr(getattr(getattr(self, main)[index], part), key, value)
					else:
						setattr(getattr(self, section)[index], key, value)
			except:
				errors.append((l, str(sys.exc_info()[1])))
		# Update values in the printer by calling the set_* functions with no new settings.
		if var_changed:
			self.set_variables()
		for type in changed:
			for num, ch in enumerate(changed[type]):
				if ch:
					getattr(self, 'set_' + type)(num)
		return errors
	# }}}
	@delayed
	def gcode_run(self, id, code, ref = (0, 0), angle = 0, probemap = None): # {{{
		self.gcode_ref = ref
		angle = math.radians(angle)
		self.gcode_angle = math.sin(angle), math.cos(angle)
		self.gcode_probemap = probemap
		ret = lambda complete, reason: id is None or self._send(id, 'return', complete, reason)
		if len(self.temp) > 0:
			self.btemp = self.temp[0].value
		else:
			self.btemp = float('nan')
		self._print_done(False, 'aborted by starting new print')
		self.queue_info = None
		self.gcode = code
		self.gcode_id = id
		if len(self.gcode) <= 0:
			log('nothing to run')
			ret(complete, 'nothing to run')
			return
		self._broadcast(None, 'printing', True)
		self._do_gcode()
	# }}}
	@delayed
	def request_confirmation(self, id, message): # {{{
		self.confirm_id += 1
		self._broadcast(None, 'confirm', self.confirm_id, message)
		self.confirmer = id
	# }}}
	def confirm(self, confirm_id, success = True): # {{{
		if confirm_id != self.confirm_id:
			# Confirmation was already sent.
			return False
		id = self.confirmer
		self.confirmer = None
		if id is not None:
			self._send(id, 'return', success)
		else:
			if not success:
				self._print_done(False, 'aborted by failed confirmation')
			else:
				self._do_gcode()
		return True
	# }}}
	def serial_enable(self, port, baud = 115200): # {{{
		return self._send_packet(struct.pack('<BBl', self.command['SETSERIAL'], port, baud))
	# }}}
	def serial_send(self, port, data): # {{{
		# data is unicode; that messes things up.
		return self._send_packet(struct.pack('<BB', self.command['SERIAL_TX'], port) + data.encode('utf-8'))
	# }}}
	# }}}
	# Accessor functions. {{{
	# Motor subtype {{{
	def _get_motor(self, obj):
		ret = {}
		for key in ('step_pin', 'dir_pin', 'enable_pin', 'steps_per_mm', 'limit_v', 'max_v', 'max_a', 'max_steps'):
			ret[key] = getattr(obj, key)
		return ret
	def _set_motor(self, obj, ka):
		for key in ('step_pin', 'dir_pin', 'enable_pin', 'max_steps'):
			if key in ka:
				setattr(obj, key, int(ka.pop(key)))
		for key in ('steps_per_mm', 'limit_v', 'max_v', 'max_a'):
			if key in ka:
				setattr(obj, key, float(ka.pop(key)))
		assert len(ka) == 0
	# }}}
	# Temp subtype {{{
	def _get_temp(self, obj):
		ret = {}
		for key in ('R0', 'R1', 'Rc', 'Tc', 'beta', 'core_C', 'shell_C', 'transfer', 'radiation', 'power', 'power_pin', 'thermistor_pin'):
			ret[key] = getattr(obj, key)
		return ret
	def _set_temp(self, obj, ka):
		for key in ('power_pin', 'thermistor_pin'):
			if key in ka:
				setattr(obj, key, int(ka.pop(key)))
		for key in ('R0', 'R1', 'Rc', 'Tc', 'beta', 'core_C', 'shell_C', 'transfer', 'radiation', 'power'):
			if key in ka:
				setattr(obj, key, float(ka.pop(key)))
		assert len(ka) == 0
	# }}}
	# Constants and variables. {{{
	def get_constants(self):
		ret = {}
		for key in ('namelen', 'queue_length', 'audio_fragments', 'audio_fragment_size', 'maxaxes', 'maxextruders', 'maxtemps', 'maxgpios', 'num_pins', 'num_digital_pins'):
			ret[key] = getattr(self, key)
		return ret
	def get_variables(self):
		ret = {}
		for key in ('name', 'num_axes', 'num_extruders', 'num_temps', 'num_gpios', 'printer_type', 'led_pin', 'probe_pin', 'room_T', 'motor_limit', 'temp_limit', 'feedrate', 'angle'):
			ret[key] = getattr(self, key)
		return ret
	def set_variables(self, **ka):
		#log('setting variables with %s' % repr(ka))
		if 'name' in ka:
			self.name = str(ka.pop('name'))
		for key in ('num_axes', 'num_extruders', 'num_temps', 'num_gpios', 'printer_type', 'led_pin', 'probe_pin'):
			if key in ka:
				setattr(self, key, int(ka.pop(key)))
		for key in ('room_T', 'motor_limit', 'temp_limit', 'feedrate', 'angle'):
			if key in ka:
				setattr(self, key, float(ka.pop(key)))
		self._write_variables()
		self.pos = (self.pos + [float('nan')] * self.num_axes)[:self.num_axes]
		assert len(ka) == 0
	# }}}
	# Temp {{{
	def get_temp(self, index):
		ret = {}
		return self._get_temp(self.temp[index])
	def set_temp(self, index, **ka):
		self._set_temp(self.temp[index], ka)
		self._write_temp(index)
		assert len(ka) == 0
	# }}}
	# Axis {{{
	def get_axis_pos(self, index):
		return self.axis[index].get_current_pos()
	def set_axis_pos(self, index, pos):
		return self.axis[index].set_current_pos(pos)
	def get_axis(self, index):
		ret = {'motor': self._get_motor(self.axis[index].motor)}
		for key in ('limit_min_pin', 'limit_max_pin', 'sense_pin', 'limit_pos', 'axis_min', 'axis_max', 'motor_min', 'motor_max', 'park', 'delta_length', 'delta_radius', 'offset'):
			ret[key] = getattr(self.axis[index], key)
		return ret
	def set_axis(self, index, **ka):
		if 'motor' in ka:
			self._set_motor(self.axis[index].motor, ka.pop('motor'))
		for key in ('limit_min_pin', 'limit_max_pin', 'sense_pin'):
			if key in ka:
				setattr(self.axis[index], key, int(ka.pop(key)))
		for key in ('limit_pos', 'axis_min', 'axis_max', 'motor_min', 'motor_max', 'park', 'delta_length', 'delta_radius', 'offset'):
			if key in ka:
				setattr(self.axis[index], key, float(ka.pop(key)))
		self._write_axis(index)
		assert len(ka) == 0
	# }}}
	# Extruder {{{
	def get_extruder(self, index):
		ret = {'motor': self._get_motor(self.extruder[index].motor), 'temp': self._get_temp(self.extruder[index].temp)}
		for key in ('filament_heat', 'nozzle_size', 'filament_size'):
			ret[key] = getattr(self.extruder[index], key)
		return ret
	def set_extruder(self, index, **ka):
		if 'motor' in ka:
			self._set_motor(self.extruder[index].motor, ka.pop('motor'))
		if 'temp' in ka:
			self._set_temp(self.extruder[index].temp, ka.pop('temp'))
		for key in ('filament_heat', 'nozzle_size', 'filament_size'):
			if key in ka:
				setattr(self.extruder[index], key, float(ka.pop(key)))
		self._write_extruder(index)
		assert len(ka) == 0
	# }}}
	# Gpio {{{
	def get_gpio(self, index):
		ret = {}
		for key in ('pin', 'state', 'master', 'value'):
			ret[key] = getattr(self.gpio[index], key)
		return ret
	def set_gpio(self, index, **ka):
		for key in ('pin', 'state', 'master'):
			if key in ka:
				setattr(self.gpio[index], key, int(ka.pop(key)))
		if 'value' in ka:
			self.gpio[index].value = float(ka.pop('value'))
		self._write_gpio(index)
		assert len(ka) == 0
	# }}}
	def send_printer(self, target): # {{{
		self._broadcast(target, 'new_printer', [self.namelen, self.queue_length, self.maxaxes, self.maxextruders, self.maxtemps, self.maxgpios, self.audio_fragments, self.audio_fragment_size, self.num_digital_pins, self.num_pins])
		self._variables_update(target)
		for a in range(self.maxaxes):
			self._axis_update(a, target)
		for e in range(self.maxextruders):
			self._extruder_update(e, target)
		for t in range(self.maxtemps):
			self._temp_update(t, target)
		for g in range(self.maxgpios):
			self._gpio_update(g, target)
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

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
	# Masks for computing checksums.
	mask = [	[0xc0, 0xc3, 0xff, 0x09],
			[0x38, 0x3a, 0x7e, 0x13],
			[0x26, 0xb5, 0xb9, 0x23],
			[0x95, 0x6c, 0xd5, 0x43],
			[0x4b, 0xdc, 0xe2, 0x83]]
	# Single-byte commands.  See firmware/serial.cpp for an explanation of the codes.
	single = { 'NACK': '\x80', 'ACK': '\xb3', 'ACKWAIT': '\xb4', 'STALL': '\x87', 'INIT': '\x99', 'GETID': '\xaa', 'SENDID': '\xad', 'DEBUG': '\x9e' }
	command = {
			'BEGIN': 0x00,
			'PING': 0x01,
			'GOTO': 0x02,
			'GOTOCB': 0x03,
			'RUN': 0x04,
			'SLEEP': 0x05,
			'SETTEMP': 0x06,
			'WAITTEMP': 0x07,
			'READTEMP': 0x08,
			'READPOWER': 0x09,
			'SETPOS': 0x0a,
			'GETPOS': 0x0b,
			'LOAD': 0x0c,
			'SAVE': 0x0d,
			'READ': 0x0e,
			'WRITE': 0x0f,
			'PAUSE': 0x10,
			'READGPIO': 0x11,
			'AUDIO_SETUP': 0x12,
			'AUDIO_DATA': 0x13}
	rcommand = {
			'START': '\x14',
			'TEMP': '\x15',
			'POWER': '\x16',
			'POS': '\x17',
			'DATA': '\x18',
			'PONG': '\x19',
			'PIN': '\x1a',
			'MOVECB': '\x1b',
			'TEMPCB': '\x1c',
			'CONTINUE': '\x1d',
			'LIMIT': '\x1e',
			'AUTOSLEEP': '\x1f',
			'SENSE': '\x20'}
	# }}}
	def _set_waiter (self, type, resumeinfo, condition = True): # {{{
		if show_own_debug:
			log ('adding waiter for %s: %s' % (type, repr (resumeinfo)))
		assert self.waiters[type][0] >= 2 or len (self.waiters[type][1]) == 0
		item = (resumeinfo[0], condition)
		self.waiters[type][1].append (item)
		return lambda: self.waiters[type][1].remove (item) if item in self.waiters[type][1] else None
	# }}}
	def _trigger (self, type, arg = None): # {{{
		if show_own_debug:
			log ('triggering %s: %s (%s)' % (type, repr (self.waiters[type][1]), arg))
		waiters = self.waiters[type][1][:]
		if self.waiters[type][0] < 2:
			if len (self.waiters[type][1]) != 1:
				if self.waiters[type][0] != 0:
					log ('no waiters for %s' % type)
			else:
				self.waiters[type][1].pop (0)[0] (arg)
		elif self.waiters[type][0] == 2:
			if len (self.waiters[type][1]) > 0:
				self.waiters[type][1].pop (0)[0] (arg)
		else:
			self.waiters[type][1][:] = []
			for w in waiters:
				if w[1] is True or w[1] (arg):
					w[0] (arg)
				else:
					self.waiters[type][1].append (w)
		return len (waiters) > len (self.waiters[type][1])
	# }}}
	def __init__ (self): # {{{
		# This must be done before _init, because it is set before that is called.
		# (So it doesn't really need to be done at all, at least not if printer3d-server is used.)
		self.audiodir = '.'
	# }}}
	def _init (self, port, broadcast, death, orphans, newid): # {{{
		resumeinfo = [(yield), None]
		self.pong = 0
		self.id_buffer = None
		self.debug_buffer = None
		self.buffer = ''
		self.port = port
		self._broadcast = broadcast
		self._death = death
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
		self.alarms = set ()
		# Type: 0 = only one waiter allowed; 1: like 0, and must have a waiter when triggered; 2: queue; 3: broadcast.
		self.waiters = {'id': (0, []), 'connect': (3, []), 'send': (2, []), 'ack': (0, []), 'queue': (2, []), 'move': (3, []), 'audio': (0, []), 'limit': (3, []), 'temp': (3, []), 'reply': (1, []), 'init': (1, []), 'sense': (3, []), 'pong': (3, [])}
		self.printer = serial.Serial (port, baudrate = 115200, timeout = 0)
		#self.printer.setDTR (False)
		#time.sleep (.1)
		#self.printer.setDTR (True)
		self.disconnected = False
		self.last_time = float ('nan')
		self.printer.readall ()	# flush buffer.
		killer = self._set_waiter ('init', resumeinfo)
		self.watcher = glib.io_add_watch (self.printer.fd, glib.IO_IN, self._printer_input)
		# We need to get the printer id first.  If the printer is booting, this can take a while.
		id = None
		# Wait to make sure the command is interpreted as a new packet.
		glib.timeout_add (150, lambda: resumeinfo[0] () is None and False)	# Make sure the lambda function returns False.
		yield websockets.WAIT
		# How many times to try before accepting an invalid id.
		self.id_tries = 10
		handle = glib.timeout_add_seconds (10, lambda: resumeinfo[0] (False) is None and False)	# Make sure the lambda function returns False.
		while id is None:
			self.printer.write (self.single['GETID'])
			getid_waiter = self._set_waiter ('id', resumeinfo)
			id = (yield websockets.WAIT)
			getid_waiter ()
			if id is False:
				# Timeout.  Give up.
				getid_waiter ()
				glib.source_remove (self.watcher)
				yield False
			# This printer was running and tried to send an id.  Remove the timeout and check the id.
			glib.source_remove (handle)
			if id in orphans:
				log ('accepting orphan')
				killer ()
				self.disconnected = True
				glib.source_remove (self.watcher)
				yield orphans[id]
			if id is None:
				# An invalid id was received, or an init notification.
				continue
			# A valid id was received, but we don't know it.  This will be treated as no id.
			log ("Printer has valid, but unknown id")
		log ('accepting printer')
		killer ()
		# Send several ping commands, to get the flip flops synchronized.
		# If the output flipflop is out of sync, the first ping will not generate a reply.
		# If the input flipflop is out of sync, the first reply will not generate a trigger.
		# Whatever happens, the third ping must be recognized by both sides.
		# But first send an ack, so any transmission in progress is considered finished.
		# This does no harm, because spurious acks are ignored.
		self.printer.write (self.single['ACK'])
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['PING'], 0))
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['PING'], 1))
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['PING'], 2))
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		while self.pong != 2:
			self._set_waiter ('pong', resumeinfo)
			yield websockets.WAIT
		# Now we're in sync.
		# Get the printer state.
		self.printerid = newid
		c = websockets.call (resumeinfo, self._begin, newid)
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		c = websockets.call (resumeinfo, self._read, 0)
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		self.namelen, self.maxaxes, self.maxextruders, self.maxtemps, self.maxgpios, self.audio_fragments, self.audio_fragment_size, self.num_pins, self.num_digital_pins = struct.unpack ('<BBBBBBBBB', c.ret ())
		c = websockets.call (resumeinfo, self._read_variables)
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		self.axis = [Printer.Axis (self, t) for t in range (self.maxaxes)]
		for a in range (self.maxaxes):
			c = websockets.call (resumeinfo, self._read, 2 + a)
			while c (): c.args = (yield websockets.WAIT)
			if not c.ret ():
				yield False
			self.axis[a].read (c.ret ())
			# Sleep motor.
			c = websockets.call (resumeinfo, self.sleep_axis, a)
			while c (): c.args = (yield websockets.WAIT)
		self.extruder = [Printer.Extruder () for t in range (self.maxextruders)]
		for e in range (self.maxextruders):
			c = websockets.call (resumeinfo, self._read, 2 + self.maxaxes + e)
			while c (): c.args = (yield websockets.WAIT)
			if not c.ret ():
				yield False
			self.extruder[e].read (c.ret ())
			# Disable heater.
			c = websockets.call (resumeinfo, self.settemp_extruder, e, float ('nan'))
			while c (): c.args = (yield websockets.WAIT)
			# Disable heater alarm.
			c = websockets.call (resumeinfo, self.waittemp_extruder, e, None, None)
			while c (): c.args = (yield websockets.WAIT)
			# Sleep motor.
			c = websockets.call (resumeinfo, self.sleep_extruder, e)
			while c (): c.args = (yield websockets.WAIT)
		self.temp = [Printer.Temp () for t in range (self.maxtemps)]
		for t in range (self.maxtemps):
			c = websockets.call (resumeinfo, self._read, 2 + self.maxaxes + self.maxextruders + t)
			while c (): c.args = (yield websockets.WAIT)
			if not c.ret ():
				yield False
			self.temp[t].read (c.ret ())
			# Disable heater.
			c = websockets.call (resumeinfo, self.settemp_temp, t, float ('nan'))
			while c (): c.args = (yield websockets.WAIT)
			# Disable heater alarm.
			c = websockets.call (resumeinfo, self.waittemp_temp, t, None, None)
			while c (): c.args = (yield websockets.WAIT)
		self.gpio = [Printer.Gpio () for g in range (self.maxgpios)]
		for g in range (self.maxgpios):
			c = websockets.call (resumeinfo, self._read, 2 + self.maxaxes + self.maxextruders + self.maxtemps + g)
			while c (): c.args = (yield websockets.WAIT)
			if not c.ret ():
				yield False
			self.gpio[g].read (c.ret ())
		# The printer may still be doing things.  Pause it and send a move; this will discard the queue.
		c = websockets.call (resumeinfo, self.pause, True)
		while c (): c.args = (yield websockets.WAIT)
		c = websockets.call (resumeinfo, self.goto)	# This flushes the buffer, possibly causing movecbs to be sent.
		while c (): c.args = (yield websockets.WAIT)
		c = websockets.call (resumeinfo, self.goto, cb = True)	# This sends one new movecb, to be sure the rest is received if they were sent.
		while c (): c.args = (yield websockets.WAIT)
		self._set_waiter ('move', resumeinfo)
		yield websockets.WAIT
		self.pong = 3	# Signal to allow all commands and send updates.
		global show_own_debug
		if show_own_debug is None:
			show_own_debug = True
		yield True
	# }}}
	def _reconnect (self, port, printer): # {{{
		self.port = port
		self.printer = printer
		self.disconnected = False
		self.watcher = glib.io_add_watch (self.printer.fd, glib.IO_IN, self._printer_input)
		# expect a missed packet.
		self.printer.write (self.single['NACK'])
		# both ways.
		self._trigger ('ack', 'nack')
		self._trigger ('connect')
	# }}}
	def _close (self): # {{{
		glib.source_remove (self.watcher)
		self.printer.close ()
		log ('disconnecting')
		self.disconnected = True
	# }}}
	def _printer_input (self, who, what): # {{{	
		if self.disconnected:
			return False
		if time.time () - self.last_time > .1:
			dprint ('writing (5)', self.single['NACK']);
			self.printer.write (self.single['NACK'])
			self.last_time = float ('nan')
			self.buffer = ''
			if self.debug_buffer is not None:
				if show_firmware_debug:
					log ('Debug (partial): %s' % self.debug_buffer)
				self.debug_buffer = None
			if self.id_buffer is not None:
				log ('Partial id: %s' % ' '.join (['%02x' % ord (x) for x in self.id_buffer]))
				self._trigger ('id', None)
				self.id_buffer = None
		while True:
			if self.disconnected:
				return False
			while self.id_buffer is not None or self.debug_buffer is not None:
				if self.disconnected:
					return False
				try:
					r = self.printer.read (1)
				except:
					# Ignore the exception now; it will be triggered again below.
					log ('exception during id or debug read')
					r = None
				if r == '':
					self.last_time = time.time ()
					return True
				if r is not None and self.id_buffer is not None:
					self.id_buffer += r
					if len (self.id_buffer) >= 8:
						if not all ([x in self.id_map for x in self.id_buffer]) and not all ([ord (x) == 0 for x in self.id_buffer]):
							log ('received invalid id: %s' % ' '.join (['%02x' % ord (x) for x in self.id_buffer]))
							if self.id_tries > 0:
								self.id_tries -= 1
								self.id_buffer = None
							else:
								log ('accepting it anyway')
						self._trigger ('id', self.id_buffer)
						self.id_buffer = None
					continue
				else:
					if r is None or r == '\x00':
						if show_firmware_debug:
							log ('Debug: %s' % self.debug_buffer)
						self.debug_buffer = None
					else:
						self.debug_buffer += r
					continue
			if self.disconnected:
				return False
			if len (self.buffer) == 0:
				try:
					r = self.printer.read (1)
				except:
					self._close ()
					self._death (self.port)
					return False
				if r != self.single['DEBUG']:
					dprint ('(1) read', r)
				if r == '':
					self.last_time = float ('nan')
					return True
				if r == self.single['DEBUG']:
					self.debug_buffer = ''
					continue
				if r == self.single['SENDID']:
					self.id_buffer = ''
					continue
				if r == self.single['ACK']:
					# Ack is special in that if it isn't expected, it is not an error.
					self._trigger ('ack', 'ack')
					continue
				if r == self.single['NACK']:
					# Nack is special in that it should only be handled if an ack is expected.
					self._trigger ('ack', 'nack')
					continue
				if r == self.single['ACKWAIT']:
					self._trigger ('ack', 'wait')
					continue
				if r == self.single['INIT']:
					if not self._trigger ('init'):
						self._close ()
						self._broadcast (None, 'reset', self.port)
						self._death (self.port)
						return False
					continue
				if r == self.single['STALL']:
					# There is a problem we can't solve; kill the generator that registered to see the ack.
					self._trigger ('ack', 'stall')
				if (ord (r) & 0x80) != 0 or ord (r) in (0, 1, 2, 4):
					dprint ('writing (1)', self.single['NACK']);
					self.printer.write (self.single['NACK'])
					continue
				# Regular packet.
				self.buffer = r
			packet_len = ord (self.buffer[0]) + (ord (self.buffer[0]) + 2) / 3
			try:
				r = self.printer.read (packet_len - len (self.buffer))
			except serial.SerialException:
				self.buffer = ''
				self._close ()
				self._death (self.port)
				return False
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
			if self.pong != 3 and packet[0] == self.rcommand['PONG']:
				# During startup, this is handled as asynchronous event.
				self.pong = ord (packet[1])
				self._trigger ('pong')
				continue
			if packet[0] == self.rcommand['MOVECB']:
				num = ord (packet[1])
				#log ('movecb %d/%d' % (num, self.movewait))
				if self.pong == 3:
					assert self.movewait >= num
					self.movewait -= num
				else:
					self.movewait = 0
				if self.movewait == 0:
					self._trigger ('move')
				continue
			if packet[0] == self.rcommand['TEMPCB']:
				if self.pong != 3:
					# Ignore this while connecting.
					continue
				t = ord (packet[1])
				self.alarms.add (t)
				self._trigger ('temp', t)
				continue
			elif packet[0] == self.rcommand['CONTINUE']:
				if self.pong != 3:
					# Ignore this while connecting.
					continue
				if packet[1] == '\x00':
					# Move continue.
					self.wait = False
					self._trigger ('queue', True)
				else:
					# Audio continue.
					self.waitaudio = False
					self._trigger ('audio', None)
				continue
			elif packet[0] == self.rcommand['LIMIT']:
				if self.pong != 3:
					# Ignore this while connecting.
					continue
				which = ord (packet[1])
				if not math.isnan (self.axis[which].motor.runspeed):
					self.axis[which].motor.runspeed = float ('nan')
					self._axis_update (which)
				self.limits[which] = struct.unpack ('<f', packet[2:])[0]
				self._trigger ('limit', which)
				continue
			elif packet[0] == self.rcommand['AUTOSLEEP']:
				if self.pong != 3:
					# Ignore this while connecting.
					continue
				what = ord (packet[1])
				changed = [set (), set (), set ()]
				if what & 1:
					for a in range (self.maxaxes):
						self.axis[a].valid = False
						if not math.isnan (self.axis[a].motor.runspeed):
							self.axis[a].motor.runspeed = float ('nan')
							changed[0].add (a)
						if not self.axis[a].motor.sleeping:
							self.axis[a].motor.sleeping = True
							changed[0].add (a)
					for e in range (self.maxextruders):
						if not math.isnan (self.extruder[e].motor.runspeed):
							self.extruder[e].motor.runspeed = float ('nan')
							changed[1].add (e)
						if not self.extruder[e].motor.sleeping:
							self.extruder[e].motor.sleeping = True
							changed[1].add (e)
				if what & 2:
					for e in range (self.maxextruders):
						if not math.isnan (self.extruder[e].temp.value):
							self.extruder[e].temp.value = float ('nan')
							changed[1].add (e)
					for t in range (self.maxtemps):
						if not math.isnan (self.temp[t].value):
							self.temp[t].value = float ('nan')
							changed[2].add (t)
				for a in changed[0]:
					self._axis_update (a)
				for e in changed[1]:
					self._extruder_update (e)
				for t in changed[2]:
					self._temp_update (t)
				continue
			elif packet[0] == self.rcommand['SENSE']:
				if self.pong != 3:
					# Ignore this while connecting.
					continue
				w = ord (packet[1])
				which = w & 0x7f
				state = bool (w & 0x80)
				pos = struct.unpack ('<f', packet[2:])[0]
				if which not in self.sense:
					self.sense[which] = []
				self.sense[which].append ((state, pos))
				self._trigger ('sense', which)
				continue
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
		if self.sending:
			self._set_waiter ('send', resumeinfo)
			yield websockets.WAIT
		self.sending = True
		if self.ff_out:
			data = chr (ord (data[0]) | 0x80) + data[1:]
		self.ff_out = not self.ff_out
		data = self._make_packet (data)
		while True:
			dprint ('(1) writing', data);
			try:
				self.printer.write (data)
			except:
				self._close ()
				self._death (self.port)
				# Fall through to waiting for ack; at reconnect, a nack event will be generated.
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
				self.sending = False
				self._trigger ('send', None)
				yield False
		self.sending = False
		self._trigger ('send', None)
		yield True
	# }}}
	def _begin (self, newid): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BL', self.command['BEGIN'], 0) + newid)
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		self._set_waiter ('reply', resumeinfo)
		reply = yield websockets.WAIT
		yield struct.unpack ('<BL', reply) == (ord (self.rcommand['START']), 0)
	# }}}
	def _read (self, channel): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['READ'], channel))
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield None
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
		if data is None:
			yield False
		self.name = unicode (data[:self.namelen].rstrip ('\0'), 'utf-8', 'replace')
		self.num_axes, self.num_extruders, self.num_temps, self.num_gpios, self.printer_type, self.led_pin, self.probe_pin, self.room_T, self.motor_limit, self.temp_limit, self.feedrate = struct.unpack ('<BBBBBHHfLLf', data[self.namelen:])
		self.pos = (self.pos + [float ('nan')] * self.num_axes)[:self.num_axes]
		yield True
	# }}}
	def _write_variables (self): # {{{
		resumeinfo = [(yield), None]
		data = (self.name.encode ('utf-8') + chr (0) * self.namelen)[:self.namelen] + struct.pack ('<BBBBBHHfLLf', self.num_axes, self.num_extruders, self.num_temps, self.num_gpios, self.printer_type, self.led_pin, self.probe_pin, self.room_T, self.motor_limit, self.temp_limit, self.feedrate)
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['WRITE'], 1) + data)
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		self._variables_update ()
		yield True
	# }}}
	def _variables_update (self, target = None): # {{{
		if self.pong != 3:
			return
		self._broadcast (target, 'variables_update', self.port, [self.name, self.num_axes, self.num_extruders, self.num_temps, self.num_gpios, self.printer_type, self.led_pin, self.probe_pin, self.room_T, self.motor_limit, self.temp_limit, self.feedrate, self.paused])
	# }}}
	def _write_axis (self, which): # {{{
		resumeinfo = [(yield), None]
		data = self.axis[which].write ()
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['WRITE'], 2 + which) + data)
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		self._axis_update (which)
		yield True
	# }}}
	def _axis_update (self, which, target = None): # {{{
		if self.pong != 3:
			return
		self._broadcast (target, 'axis_update', self.port, which, [
			[self.axis[which].motor.step_pin, self.axis[which].motor.dir_pin, self.axis[which].motor.enable_pin, self.axis[which].motor.steps_per_mm, self.axis[which].motor.max_v_pos, self.axis[which].motor.max_v_neg, self.axis[which].motor.max_a, self.axis[which].motor.runspeed, self.axis[which].motor.sleeping],
			self.axis[which].limit_min_pin, self.axis[which].limit_max_pin, self.axis[which].sense_pin, self.axis[which].limit_pos, self.axis[which].axis_min, self.axis[which].axis_max, self.axis[which].motor_min, self.axis[which].motor_max, self.axis[which].park, self.axis[which].delta_length, self.axis[which].delta_radius, self.axis[which].offset])
	# }}}
	def _write_extruder (self, which): # {{{
		resumeinfo = [(yield), None]
		data = self.extruder[which].write ()
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['WRITE'], 2 + self.maxaxes + which) + data)
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		self._extruder_update (which)
		yield True
	# }}}
	def _extruder_update (self, which, target = None): # {{{
		if self.pong != 3:
			return
		self._broadcast (target, 'extruder_update', self.port, which, [
			[self.extruder[which].motor.step_pin, self.extruder[which].motor.dir_pin, self.extruder[which].motor.enable_pin, self.extruder[which].motor.steps_per_mm, self.extruder[which].motor.max_v_pos, self.extruder[which].motor.max_v_neg, self.extruder[which].motor.max_a, self.extruder[which].motor.runspeed, self.extruder[which].motor.sleeping],
			[self.extruder[which].temp.power_pin, self.extruder[which].temp.thermistor_pin, self.extruder[which].temp.R0, self.extruder[which].temp.R1, self.extruder[which].temp.Rc, self.extruder[which].temp.Tc, self.extruder[which].temp.beta, self.extruder[which].temp.core_C, self.extruder[which].temp.shell_C, self.extruder[which].temp.transfer, self.extruder[which].temp.radiation, self.extruder[which].temp.power, self.extruder[which].temp.value],
			self.extruder[which].filament_heat, self.extruder[which].nozzle_size, self.extruder[which].filament_size])
	# }}}
	def _write_temp (self, which): # {{{
		resumeinfo = [(yield), None]
		data = self.temp[which].write ()
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['WRITE'], 2 + self.maxaxes + self.maxextruders + which) + data)
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		self._temp_update (which)
		yield True
	# }}}
	def _temp_update (self, which, target = None): # {{{
		if self.pong != 3:
			return
		self._broadcast (target, 'temp_update', self.port, which, [self.temp[which].power_pin, self.temp[which].thermistor_pin, self.temp[which].R0, self.temp[which].R1, self.temp[which].Rc, self.temp[which].Tc, self.temp[which].beta, self.temp[which].core_C, self.temp[which].shell_C, self.temp[which].transfer, self.temp[which].radiation, self.temp[which].power, self.temp[which].value])
	# }}}
	def _write_gpio (self, which): # {{{
		resumeinfo = [(yield), None]
		data = self.gpio[which].write ()
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['WRITE'], 2 + self.maxaxes + self.maxextruders + self.maxtemps + which) + data)
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		self._gpio_update (which)
		yield True
	# }}}
	def _gpio_update (self, which, target = None): # {{{
		if self.pong != 3:
			return
		self._broadcast (target, 'gpio_update', self.port, which, [self.gpio[which].pin, self.gpio[which].state, self.gpio[which].master, self.gpio[which].value])
	# }}}
	# Subclasses.  {{{
	class Temp: # {{{
		def __init__ (self):
			self.value = float ('nan')
		def read (self, data):
			self.R0, self.R1, self.Rc, self.Tc, self.beta, self.core_C, self.shell_C, self.transfer, self.radiation, self.power, self.power_pin, self.thermistor_pin = struct.unpack ('<ffffffffffHH', data[:44])
			return data[44:]
		def write (self):
			return struct.pack ('<ffffffffffHH', self.R0, self.R1, self.Rc, self.Tc, self.beta, self.core_C, self.shell_C, self.transfer, self.radiation, self.power, self.power_pin, self.thermistor_pin)
	# }}}
	class Motor: # {{{
		def __init__ (self):
			self.runspeed = 0
			self.sleeping = True
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
			self.valid = False
			self.motor = Printer.Motor ()
		def read (self, data):
			data = self.motor.read (data)
			self.limit_min_pin, self.limit_max_pin, self.sense_pin, self.limit_pos, self.axis_min, self.axis_max, self.motor_min, self.motor_max, self.park, self.delta_length, self.delta_radius, self.offset = struct.unpack ('<HHHfffffffff', data[:26])
		def write (self):
			return self.motor.write () + struct.pack ('<HHHfffffffff', self.limit_min_pin, self.limit_max_pin, self.sense_pin, self.limit_pos, self.axis_min, self.axis_max, self.motor_min, self.motor_max, self.park, self.delta_length, self.delta_radius, self.offset)
		def set_current_pos (self, pos):
			resumeinfo = [(yield), None]
			c = websockets.call (resumeinfo, self.printer._send_packet, struct.pack ('<BBf', self.printer.command['SETPOS'], 2 + self.id, pos))
			while c (): c.args = (yield websockets.WAIT)
			if not c.ret ():
				yield False
			self.valid = True
			yield True
		def get_current_pos (self):
			resumeinfo = [(yield), None]
			c = websockets.call (resumeinfo, self.printer._send_packet, struct.pack ('<BB', self.printer.command['GETPOS'], 2 + self.id))
			while c (): c.args = (yield websockets.WAIT)
			if not c.ret ():
				yield None
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
	class Gpio: # {{{
		def read (self, data):
			self.pin, self.state, self.master, self.value = struct.unpack ('<HBBf', data)
		def write (self):
			return struct.pack ('<HBBf', self.pin, self.state, self.master, self.value)
	# }}}
	# }}}
	# }}}
	# Useful commands.  {{{
	def goto (self, axes = {}, e = None, f0 = None, f1 = None, which = 0, cb = False): # {{{
		resumeinfo = [(yield), None]
		a = {}
		if isinstance (axes, (list, tuple)):
			for i, axis in enumerate (axes):
				a[i] = axis
		else:
			for i, axis in axes.items ():
				a[int (i)] = axis
		axes = a
		targets = [0] * (((2 + self.num_axes + self.num_extruders - 1) >> 3) + 1)
		args = ''
		if f0 is None:
			f0 = float ('inf')
		if f1 is None:
			f1 = f0
		if f0 != float ('inf'):
			targets[0] |= 1 << 0
			args += struct.pack ('<f', f0)
		if f1 != f0:
			targets[0] |= 1 << 1
			args += struct.pack ('<f', f1)
		a = axes.keys ()
		a.sort ()
		#log ('f0: %f f1: %f' % (f0, f1))
		for axis in a:
			assert axis < self.num_axes
			if math.isnan (axes[axis]):
				continue
			if self.axis[axis].motor.sleeping:
				self.axis[axis].motor.sleeping = False
				self._axis_update (axis)
			self.pos[axis] = axes[axis]
			targets[(axis + 2) >> 3] |= 1 << ((axis + 2) & 0x7)
			args += struct.pack ('<f', axes[axis])
			#log ('axis %d: %f' % (axis, axes[axis]))
		if e is not None:
			targets[(2 + self.num_axes + which) >> 3] |= 1 << ((2 + self.num_axes + which) & 0x7)
			args += struct.pack ('<f', e)
			if self.extruder[which].motor.sleeping:
				self.extruder[which].motor.sleeping = False
				self._extruder_update (which)
		# Check the queue only after we're allowed to send, because the last ack may be a wait.
		if self.sending:
			self._set_waiter ('send', resumeinfo)
			yield websockets.WAIT
		while self.wait and not self.paused:
			self._set_waiter ('queue', resumeinfo)
			if not (yield websockets.WAIT):
				return
		if cb:
			self.movewait += 1
			#log ('movewait +1 -> %d' % self.movewait)
			p = chr (self.command['GOTOCB'])
		else:
			p = chr (self.command['GOTO'])
		if self.paused:
			self._trigger ('queue', False)
			self.paused = False
			self._variables_update ()
		c = websockets.call (resumeinfo, self._send_packet, p + ''.join ([chr (t) for t in targets]) + args)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
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
		channel = int (channel)
		if self.paused:
			self.paused = False
			self._variables_update ()
		if channel >= 2 and channel < 2 + self.maxaxes:
			if speed != 0 and not math.isnan (speed):
				self.pos[channel - 2] = float ('nan')
				self.axis[channel - 2].valid = False
			self.axis[channel - 2].motor.runspeed = speed
			self.axis[channel - 2].motor.sleeping = False
			self._axis_update (channel - 2)
		else:
			self.extruder[channel - 2 - self.maxaxes].motor.runspeed = speed
			self.extruder[channel - 2 - self.maxaxes].motor.sleeping = False
			self._extruder_update (channel - 2 - self.maxaxes)
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BBf', self.command['RUN'], channel, speed))
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
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
		channel = int (channel)
		if self.paused:
			self.paused = False
			self._variables_update ()
		if channel >= 2 and channel < 2 + self.maxaxes:
			self.axis[channel - 2].motor.sleeping = sleeping
			if sleeping:
				self.axis[channel - 2].valid = False
			self._axis_update (channel - 2)
		else:
			self.extruder[channel - 2 - self.maxaxes].motor.sleeping = sleeping
			self._extruder_update (channel - 2 - self.maxaxes)
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['SLEEP'], (channel & 0x7f) | (0x80 if sleeping else 0)))
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
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
		channel = int (channel)
		if channel < 2 + self.maxaxes + self.maxextruders:
			self.extruder[channel - 2 - self.maxaxes].temp.value = temp
			self._extruder_update (channel - 2 - self.maxaxes)
		else:
			self.temp[channel - 2 - self.maxaxes - self.maxextruders].value = temp
			self._temp_update (channel - 2 - self.maxaxes - self.maxextruders)
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BBf', self.command['SETTEMP'], channel, temp))
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	# }}}
	def waittemp_extruder (self, which, min, max = None):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.waittemp, 2 + self.maxaxes + which, min, max)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def waittemp_temp (self, which, min, max = None):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.waittemp, 2 + self.maxaxes + self.maxextruders + which, min, max)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def waittemp (self, channel, min, max = None): # {{{
		resumeinfo = [(yield), None]
		channel = int (channel)
		if min is None:
			min = float ('nan')
		if max is None:
			max = float ('nan')
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BBff', self.command['WAITTEMP'], channel, min, max))
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
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
		channel = int (channel)
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['READTEMP'], channel))
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield None
		self._set_waiter ('reply', resumeinfo)
		ret = yield websockets.WAIT
		assert ret[0] == self.rcommand['TEMP']
		yield struct.unpack ('<f', ret[1:])[0]
	# }}}
	def readpower_extruder (self, which):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.readpower, 2 + self.maxaxes + which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	# }}}
	def readpower_temp (self, which):	# {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.readpower, 2 + self.maxaxes + self.maxextruders + which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	# }}}
	def readpower (self, channel): # {{{
		resumeinfo = [(yield), None]
		channel = int (channel)
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['READPOWER'], channel))
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield None
		self._set_waiter ('reply', resumeinfo)
		ret = yield websockets.WAIT
		assert ret[0] == self.rcommand['POWER']
		yield struct.unpack ('<LL', ret[1:])
	# }}}
	def readpin (self, pin): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['READPIN'], pin))
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield None
		self._set_waiter ('reply', resumeinfo)
		ret = yield websockets.WAIT
		assert ret[0] == self.rcommand['PIN']
		yield bool (ord (ret[1]))
	# }}}
	def load_variables (self): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.load, 1)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	# }}}
	def load_axis (self, which): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.load, 2 + which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	# }}}
	def load_extruder (self, which): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.load, 2 + self.maxaxes + which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	# }}}
	def load_temp (self, which): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.load, 2 + self.maxaxes + self.maxextruders + which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	# }}}
	def load (self, channel): # {{{
		resumeinfo = [(yield), None]
		channel = int (channel)
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['LOAD'], channel))
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		if channel == 1:
			c = websockets.call (resumeinfo, self._read_variables, )
			while c (): c.args = (yield websockets.WAIT)
			if not c.ret ():
				yield False
			self._variables_update ()
		else:
			c = websockets.call (resumeinfo, self._read, channel)
			while c (): c.args = (yield websockets.WAIT)
			data = c.ret ()
			if data is None:
				yield False
			if 2 <= channel < 2 + self.maxaxes:
				c = websockets.call (resumeinfo, self.axis[channel - 2].read, data)
				while c (): c.args = (yield websockets.WAIT)
				self._axis_update (channel - 2)
			elif 2 + self.maxaxes <= channel < 2 + self.maxaxes + self.maxextruders:
				c = websockets.call (resumeinfo, self.extruder[channel - 2 - self.maxaxes].read, data)
				while c (): c.args = (yield websockets.WAIT)
				self._extruder_update (channel - 2 - self.maxaxes)
			elif 2 + self.maxaxes + self.maxextruders <= channel < 2 + self.maxaxes + self.maxextruders + self.maxtemps:
				c = websockets.call (resumeinfo, self.temp[channel - 2 - self.maxaxes - self.maxextruders].read, data)
				while c (): c.args = (yield websockets.WAIT)
				self._temp_update (channel - 2 - self.maxaxes - self.maxextruders)
			else:
				assert 2 + self.maxaxes + self.maxextruders + self.maxtemps <= channel < 2 + self.maxaxes + self.maxextruders + self.maxtemps + self.maxgpios
				c = websockets.call (resumeinfo, self.gpio[channel - 2 - self.maxaxes - self.maxextruders - self.maxtemps].read, data)
				while c (): c.args = (yield websockets.WAIT)
				self._gpio_update (channel - 2 - self.maxaxes - self.maxextruders - self.maxtemps)
			yield True
	# }}}
	def load_all (self): # {{{
		resumeinfo = [(yield), None]
		for i in range (1, 2 + self.maxaxes + self.maxextruders + self.maxtemps + self.maxgpios):
			log ('loading %d' % i)
			c = websockets.call (resumeinfo, self.load, i)
			while c (): c.args = (yield websockets.WAIT)
			if not c.ret ():
				yield False
		yield True
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
		channel = int (channel)
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['SAVE'], channel))
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	# }}}
	def save_all (self): # {{{
		resumeinfo = [(yield), None]
		for i in range (1, 2 + self.maxaxes + self.maxextruders + self.maxtemps + self.maxgpios):
			log ('saving %d' % i)
			c = websockets.call (resumeinfo, self.save, i)
			while c (): c.args = (yield websockets.WAIT)
	# }}}
	def pause (self, pausing = True): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['PAUSE'], pausing))
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		self.paused = pausing
		self._variables_update ()
		yield True
	# }}}
	def ping (self, arg = 0): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self._send_packet, struct.pack ('<BB', self.command['PING'], arg))
		while c (): c.args = (yield websockets.WAIT)
		if not c.ret ():
			yield False
		self._set_waiter ('reply', resumeinfo)
		reply = yield websockets.WAIT
		yield struct.unpack ('<BB', reply) == (ord (self.rcommand['PONG']), arg)
	# }}}
	def home (self, axes = (0, 1, 2), speed = 5): # {{{
		resumeinfo = [(yield), None]
		self._trigger ('queue', False)
		if self.printer_type != 1:
			# Find the switches.
			self.limits.clear ()
			for axis in axes:
				s = 3200. / self.axis[axis].motor.steps_per_mm
				c = websockets.call (resumeinfo, self.sleep_axis, axis, False)
				while c (): c.args = (yield websockets.WAIT)
				c = websockets.call (resumeinfo, self.axis[axis].set_current_pos, 0)
				while c (): c.args = (yield websockets.WAIT)
				c = websockets.call (resumeinfo, self.run_axis, axis, -s if self.pin_valid (self.axis[axis].limit_min_pin) else s)
				while c (): c.args = (yield websockets.WAIT)
			c = websockets.call (resumeinfo, self.wait_for_limits, len (axes))
			while c (): c.args = (yield websockets.WAIT)
			# Back off a bit.
			axesmove = {}
			for axis in axes:
				c = websockets.call (resumeinfo, self.axis[axis].set_current_pos, 0)
				while c (): c.args = (yield websockets.WAIT)
				axesmove[axis] = -self.axis[axis].offset + 3 if self.pin_valid (self.axis[axis].limit_min_pin) else -self.axis[axis].offset - 3
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
			# Current position is limit_pos.
			for axis in axes:
				p = self.axis[axis].limit_min_pos if self.pin_valid (self.axis[axis].limit_min_pin) else self.axis[axis].limit_max_pos
				c = websockets.call (resumeinfo, self.axis[axis].set_current_pos, p)
				while c (): c.args = (yield websockets.WAIT)
				self.pos[axis] = p + self.axis[axis].offset
			return
		else:
			# Allow motors to move.
			for axis in range (3):
				c = websockets.call (resumeinfo, self.sleep_axis, axis, False)
				while c (): c.args = (yield websockets.WAIT)
			# Compute home position close to limit switches.
			if all ([self.pin_valid (self.axis[a].limit_max_pin) for a in range (3)]):
				pos = [self.axis[i].limit_max_pos - self.axis[i].offset - 10 for i in range (3)]
			elif all ([self.pin_valid (self.axis[a].limit_min_pin) for a in range (3)]):
				pos = [self.axis[i].limit_min_pos - self.axis[i].offset + 10 for i in range (3)]
			elif all ([self.pin_valid (self.axis[a].sense_pin) for a in range (3)]):
				# Only sense pins are available; search for them.
				try:
					# Use carthesian commands while searching.
					c = websockets.call (resumeinfo, self.set_printer_type, 0)
					while c (): c.args = (yield websockets.WAIT)
					# Set starting position as original 0.
					for a in range (3):
						c = websockets.call (resumeinfo, self.axis[a].set_current_pos, 0)
						while c (): c.args = (yield websockets.WAIT)
					# Go back and forth until the sensor is seen.
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
					# All sensors are found.
					# Move away, then one by one slowly back.
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
						c = websockets.call (resumeinfo, self.wait_for_cb, a)
						while c (): c.args = (yield websockets.WAIT)
						assert a in self.sense
						c = websockets.call (resumeinfo, self.pause, True)
						while c (): c.args = (yield websockets.WAIT)
						sense[a] = self.sense[a][0][1]
						c = websockets.call (resumeinfo, self.goto, {a: 0}, cb = True)
						while c (): c.args = (yield websockets.WAIT)
						c = websockets.call (resumeinfo, self.wait_for_cb)
						while c (): c.args = (yield websockets.WAIT)
					# All sensors are found with better precision.  Go to the sensor position.
					c = websockets.call (resumeinfo, self.goto, [sense[a] for a in range (3)], cb = True)
					while c (): c.args = (yield websockets.WAIT)
					c = websockets.call (resumeinfo, self.wait_for_cb)
					while c (): c.args = (yield websockets.WAIT)
					# Set current position to be -limit_min_pos.
					for a in range (3):
						c = websockets.call (resumeinfo, self.axis[a].set_current_pos, -self.axis[a].limit_min_pos)
						while c (): c.args = (yield websockets.WAIT)
					# Move the extruder to the center: all axes must have the same coordinate.
					# This is required to allow the printer to determine the position of the extruder.
					pos = min ([-self.axis[i].limit_min_pos for i in range (3)])
					c = websockets.call (resumeinfo, self.goto, [pos for a in range (3)], cb = True)
					while c (): c.args = (yield websockets.WAIT)
					c = websockets.call (resumeinfo, self.wait_for_cb)
					while c (): c.args = (yield websockets.WAIT)
				finally:
					c = websockets.call (resumeinfo, self.set_printer_type, 1)
					while c (): c.args = (yield websockets.WAIT)
				# Current positions are discarded on set_printer_type, so set them again.
				for a in range (3):
					c = websockets.call (resumeinfo, self.axis[a].set_current_pos, pos)
					while c (): c.args = (yield websockets.WAIT)
				return
			else:
				log ("Can't home without any sensors")
				return
			try:
				# Use carthesian commands while searching.
				c = websockets.call (resumeinfo, self.set_printer_type, 0)
				while c (): c.args = (yield websockets.WAIT)
				for axis in range (3):
					c = websockets.call (resumeinfo, self.axis[axis].set_current_pos, 0)
					while c (): c.args = (yield websockets.WAIT)
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
				# Move the extruder to the center: all axes must have the same coordinate.
				# This is required to allow the printer to determine the position of the extruder.
				homepos = min ([self.axis[i].limit_max_pos - 10 if self.pin_valid (self.axis[i].limit_max_pin) else self.axis[i].limit_min_pos + 10 for i in range (3)])
				c = websockets.call (resumeinfo, self.goto, [homepos - self.axis[a].offset for a in range (3)], cb = True)
				while c (): c.args = (yield websockets.WAIT)
				c = websockets.call (resumeinfo, self.wait_for_cb)
				while c (): c.args = (yield websockets.WAIT)
			finally:
				c = websockets.call (resumeinfo, self.set_printer_type, 1)
				while c (): c.args = (yield websockets.WAIT)
			# Go to home position.
			for a in range (3):
				c = websockets.call (resumeinfo, self.axis[a].set_current_pos, homepos)
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
		if not c.ret ():
			yield False
		while self.audiofile is not None:
			while self.waitaudio:
				self._set_waiter ('audio', resumeinfo)
				yield websockets.WAIT
			data = self.audiofile.read (self.audio_fragment_size)
			if len (data) < self.audio_fragment_size:
				self.audiofile = None
				yield
			c = websockets.call (resumeinfo, self._send_packet, chr (self.command['AUDIO_DATA']) + data, audio = True)
			while c (): c.args = (yield websockets.WAIT)
			if not c.ret ():
				yield False
		yield True
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
		if self.movewait == 0:
			yield True
		if sense is not False:
			if sense in self.sense:
				yield self.movewait == 0
			rm1 = self._set_waiter ('sense', resumeinfo, lambda s: sense is True or s == sense)
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
	def wait_for_temp_extruder (self, which = None): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.wait_for_temp, 2 + self.maxaxes + which)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def wait_for_temp_temp (self, which = None): # {{{
		resumeinfo = [(yield), None]
		c = websockets.call (resumeinfo, self.wait_for_temp, 2 + self.maxaxes + self.maxextruders + which)
		while c (): c.args = (yield websockets.WAIT)
	# }}}
	def wait_for_temp (self, which = None): # {{{
		resumeinfo = [(yield), None]
		while len (self.alarms) == 0 and (which is None or which not in self.alarms):
			self._set_waiter ('temp', resumeinfo, lambda t: which is None or t == which)
			yield websockets.WAIT
	# }}}
	def clear_alarm (self, which = None): # {{{
		if which is None:
			self.alarms.clear ()
		else:
			self.alarms.discard (which)
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
	def axis_valid (self, axis):	# {{{
		return self.axis[axis].valid
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
	def get_maxgpios (self):	# {{{
		return self.maxgpios
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
		if not isinstance (value, unicode):
			value = unicode (value, 'utf-8', 'replace')
		self.name = unicode (value.encode ('utf-8')[:self.namelen], 'utf-8', 'replace')
		c = websockets.call (resumeinfo, self._write_variables)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_name (self):
		return self.name
	# }}}
	def set_num_axes (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.num_axes = value
		self.pos = (self.pos + [float ('nan')] * value)[:value]
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_num_axes (self):
		return self.num_axes
	# }}}
	def set_num_extruders (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.num_extruders = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_num_extruders (self):
		return self.num_extruders
	# }}}
	def set_num_temps (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.num_temps = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_num_temps (self):
		return self.num_temps
	# }}}
	def set_num_gpios (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.num_gpios = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_num_gpios (self):
		return self.num_gpios
	# }}}
	def set_printer_type (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.printer_type = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_printer_type (self):
		return self.printer_type
	# }}}
	def set_led_pin (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.led_pin = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_led_pin (self):
		return self.led_pin
	# }}}
	def set_probe_pin (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.probe_pin = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_probe_pin (self):
		return self.probe_pin
	# }}}
	def set_room_T (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.room_T = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_room_T (self):
		return self.room_T
	# }}}
	def set_motor_limit (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.motor_limit = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_motor_limit (self):
		return self.motor_limit
	# }}}
	def set_temp_limit (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp_limit = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_temp_limit (self):
		return self.temp_limit
	# }}}
	def set_feedrate (self, value):	# {{{
		resumeinfo = [(yield), None]
		self.feedrate = value
		c = websockets.call (resumeinfo, self._write_variables, )
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def get_feedrate (self):
		return self.feedrate
	# }}}
	# }}}
	# Temp {{{
	def temp_set_R0 (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].R0 = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_R0 (self, which):
		return self.temp[which].R0
	# }}}
	def temp_set_R1 (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].R1 = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_R1 (self, which):
		return self.temp[which].R1
	# }}}
	def temp_set_Rc (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].Rc = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_Rc (self, which):
		return self.temp[which].Rc
	# }}}
	def temp_set_Tc (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].Tc = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_Tc (self, which):
		return self.temp[which].Tc
	# }}}
	def temp_set_beta (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].beta = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_beta (self, which):
		return self.temp[which].beta
	# }}}
	def temp_set_core_C (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].core_C = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_core_C (self, which):
		return self.temp[which].core_C
	# }}}
	def temp_set_shell_C (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].shell_C = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_shell_C (self, which):
		return self.temp[which].shell_C
	# }}}
	def temp_set_transfer (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].transfer = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_transfer (self, which):
		return self.temp[which].transfer
	# }}}
	def temp_set_radiation (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].radiation = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_radiation (self, which):
		return self.temp[which].radiation
	# }}}
	def temp_set_power (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].power = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_power (self, which):
		return self.temp[which].power
	# }}}
	def temp_set_power_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].power_pin = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def temp_get_power_pin (self, which):
		return self.temp[which].power_pin
	# }}}
	def temp_set_thermistor_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.temp[which].thermistor_pin = value
		c = websockets.call (resumeinfo, self._write_temp, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
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
		yield c.ret ()
	def axis_motor_get_step_pin (self, which):
		return self.axis[which].motor.step_pin
	# }}}
	def axis_motor_set_dir_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.dir_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_motor_get_dir_pin (self, which):
		return self.axis[which].motor.dir_pin
	# }}}
	def axis_motor_set_enable_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.enable_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_motor_get_enable_pin (self, which):
		return self.axis[which].motor.enable_pin
	# }}}
	def axis_motor_set_steps_per_mm (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.steps_per_mm = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_motor_get_steps_per_mm (self, which):
		return self.axis[which].motor.steps_per_mm
	# }}}
	def axis_motor_set_max_v_neg (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.max_v_neg = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_motor_get_max_v_neg (self, which):
		return self.axis[which].motor.max_v_neg
	# }}}
	def axis_motor_set_max_v_pos (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.max_v_pos = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_motor_get_max_v_pos (self, which):
		return self.axis[which].motor.max_v_pos
	# }}}
	def axis_motor_set_max_a (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor.max_a = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_motor_get_max_a (self, which):
		return self.axis[which].motor.max_a
	# }}}
	def axis_set_limit_min_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].limit_min_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_limit_min_pin (self, which):
		return self.axis[which].limit_min_pin
	# }}}
	def axis_set_limit_max_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].limit_max_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_limit_max_pin (self, which):
		return self.axis[which].limit_max_pin
	# }}}
	def axis_set_sense_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].sense_pin = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_sense_pin (self, which):
		return self.axis[which].sense_pin
	# }}}
	def axis_set_limit_pos (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].limit_pos = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_limit_pos (self, which):
		return self.axis[which].limit_pos
	# }}}
	def axis_set_axis_min (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].axis_min = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_axis_min (self, which):
		return self.axis[which].axis_min
	# }}}
	def axis_set_axis_max (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].axis_max = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_axis_max (self, which):
		return self.axis[which].axis_max
	# }}}
	def axis_set_motor_min (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor_min = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_motor_min (self, which):
		return self.axis[which].motor_min
	# }}}
	def axis_set_motor_max (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].motor_max = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_motor_max (self, which):
		return self.axis[which].motor_max
	# }}}
	def axis_set_park (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].park = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_park (self, which):
		return self.axis[which].park
	# }}}
	def axis_set_delta_length (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].delta_length = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_delta_length (self, which):
		return self.axis[which].delta_length
	# }}}
	def axis_set_delta_radius (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].delta_radius = value
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def axis_get_delta_radius (self, which):
		return self.axis[which].delta_radius
	# }}}
	def axis_set_offset (self, which, offset):	# {{{
		resumeinfo = [(yield), None]
		self.axis[which].offset = offset
		c = websockets.call (resumeinfo, self._write_axis, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
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
	def extruder_temp_set_R0 (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.R0 = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_R0 (self, which):
		return self.extruder[which].temp.R0
	# }}}
	def extruder_temp_set_R1 (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.R1 = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_R1 (self, which):
		return self.extruder[which].temp.R1
	# }}}
	def extruder_temp_set_Rc (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.Rc = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_Rc (self, which):
		return self.extruder[which].temp.Rc
	# }}}
	def extruder_temp_set_Tc (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.Tc = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_Tc (self, which):
		return self.extruder[which].temp.Tc
	# }}}
	def extruder_temp_set_beta (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.beta = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_beta (self, which):
		return self.extruder[which].temp.beta
	# }}}
	def extruder_temp_set_core_C (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.core_C = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_core_C (self, which):
		return self.extruder[which].temp.core_C
	# }}}
	def extruder_temp_set_shell_C (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.shell_C = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_shell_C (self, which):
		return self.extruder[which].temp.shell_C
	# }}}
	def extruder_temp_set_transfer (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.transfer = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_transfer (self, which):
		return self.extruder[which].temp.transfer
	# }}}
	def extruder_temp_set_radiation (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.radiation = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_radiation (self, which):
		return self.extruder[which].temp.radiation
	# }}}
	def extruder_temp_set_power (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.power = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_power (self, which):
		return self.extruder[which].temp.power
	# }}}
	def extruder_temp_set_power_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.power_pin = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_power_pin (self, which):
		return self.extruder[which].temp.power_pin
	# }}}
	def extruder_temp_set_thermistor_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].temp.thermistor_pin = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_temp_get_thermistor_pin (self, which):
		return self.extruder[which].temp.thermistor_pin
	# }}}
	def extruder_motor_set_step_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.step_pin = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_motor_get_step_pin (self, which):
		return self.extruder[which].motor.step_pin
	# }}}
	def extruder_motor_set_dir_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.dir_pin = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_motor_get_dir_pin (self, which):
		return self.extruder[which].motor.dir_pin
	# }}}
	def extruder_motor_set_enable_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.enable_pin = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_motor_get_enable_pin (self, which):
		return self.extruder[which].motor.enable_pin
	# }}}
	def extruder_motor_set_steps_per_mm (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.steps_per_mm = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_motor_get_steps_per_mm (self, which):
		return self.extruder[which].motor.steps_per_mm
	# }}}
	def extruder_motor_set_max_v_neg (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.max_v_neg = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_motor_get_max_v_neg (self, which):
		return self.extruder[which].motor.max_v_neg
	# }}}
	def extruder_motor_set_max_v_pos (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.max_v_pos = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_motor_get_max_v_pos (self, which):
		return self.extruder[which].motor.max_v_pos
	# }}}
	def extruder_motor_set_max_a (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].motor.max_a = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_motor_get_max_a (self, which):
		return self.extruder[which].motor.max_a
	# }}}
	def extruder_set_filament_heat (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].filament_heat = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_get_filament_heat (self, which):
		return self.extruder[which].filament_heat
	# }}}
	def extruder_set_nozzle_size (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].nozzle_size = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_get_nozzle_size (self, which):
		return self.extruder[which].nozzle_size
	# }}}
	def extruder_set_filament_size (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.extruder[which].filament_size = value
		c = websockets.call (resumeinfo, self._write_extruder, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def extruder_get_filament_size (self, which):
		return self.extruder[which].filament_size
	# }}}
	# }}}
	# Gpio {{{
	def gpio_set_pin (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.gpio[which].pin = value
		c = websockets.call (resumeinfo, self._write_gpio, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def gpio_get_pin (self, which):
		return self.gpio[which].pin
	# }}}
	def gpio_set_state (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.gpio[which].state = int (value)
		c = websockets.call (resumeinfo, self._write_gpio, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def gpio_get_state (self, which):
		return self.gpio[which].state
	# }}}
	def gpio_set_master (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.gpio[which].master = int (value)
		c = websockets.call (resumeinfo, self._write_gpio, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def gpio_get_master (self, which):
		return self.gpio[which].master
	# }}}
	def gpio_set_value (self, which, value):	# {{{
		resumeinfo = [(yield), None]
		self.gpio[which].value = value
		c = websockets.call (resumeinfo, self._write_gpio, which)
		while c (): c.args = (yield websockets.WAIT)
		yield c.ret ()
	def gpio_get_value (self, which):
		return self.gpio[which].value
	# }}}
	# }}}
	# }}}
# }}}

#

import serial
import time


class printer:
	# Masks for computing checksums.
	mask = [	[0xc0, 0xc3, 0xff, 0x09],
			[0x38, 0x3a, 0x7e, 0x13],
			[0x26, 0xb5, 0xb9, 0x23],
			[0x95, 0x6c, 0xd5, 0x43],
			[0x4b, 0xdc, 0xe2, 0x83]]
	# Single-byte commands.
	single = { 'ACK': '\x80', 'NACK': '\xe1', 'ACKWAIT': '\xd2', 'STALL': '\xb3', 'RESET': '\xf4', 'INIT': '\x95', 'ACKRESET': '\xa6', 'UNUSED': '\xc7' }
	command = {
			'BEGIN': '\x00',
			'GOTO': '\x01',
			'GOTOCB': '\x02',
			'RUN': '\x03',
			'SLEEP': '\x04',
			'SETTEMP': '\x05',
			'WAITTEMP': '\x06',
			'READTEMP': '\x07',
			'LOAD': '\x08',
			'SAVE': '\x09',
			'READ': '\x0a',
			'WRITE': '\x0b',
			'PAUSE': '\x0c',
			'PING': '\x0d'}
	rcommand = {
			'START': '\x0e',
			'TEMP': '\x0f',
			'DATA': '\x10',
			'PONG': '\x11',
			'MOVECB': '\x12',
			'TEMPCB': '\x13',
			'CONTINUE': '\x14'}
	def __init__ (self, port = '/dev/ttyACM0'):
		self.printer = serial.Serial (port, baudrate = 115200, timeout = .01)
		self.printer.readall ()
		self.printer.setTimeout (2)
		assert self.printer.read () == self.single['INIT']
		self.printer.setTimeout (.01)
		assert self.begin (0) == 0
	def make_packet (data):
		data = chr (len (data) + 1) + data
		for t in range ((len (data) + 2) / 3):
			check = t & 0x7
			for bit in range (5):
				sum = check & mask[bit][3]
				for byte in range (3):
					sum ^= ord (data[3 * t + byte]) & mask[bit][byte]
				sum ^= sum >> 4
				sum ^= sum >> 2
				sum ^= sum >> 1
				if sum & 1:
					check |= 1 << (bit + 3)
			data += chr (check)
		#print ' '.join (['%02x' % ord (x) for x in data])
		return data
	def parse_packet (data):
		#print ' '.join (['%02x' % ord (x) for x in data])
		if (ord (data[0]) + 2) / 3 != (len (data) + 3) / 4:
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
					sum ^= ord (r[byte]) & mask[bit][byte]
				sum ^= sum >> 4
				sum ^= sum >> 2
				sum ^= sum >> 1
				if (sum & 1) != 0:
					return None
		return data[1:length]
	def begin (self, version):
		self.send_packet (struct.pack ('<Bf', self.command['BEGIN'], version))
		assert struct.unpack ('<Bf', self.recv_packet ()) == [0, 0.0]
	def goto (self):
		pass	# TODO: goto
	def gotocb (self):
		pass	# TODO: goto
	def run (self, channel, speed):	# speed: float; 0 means off.
		self.send_packet (struct.pack ('<BBf', self.command['RUN'], channel, speed)
	def sleep (self, channel, sleeping):
		self.send_packet (struct.pack ('<BB', self.command['SLEEP'], (channel & 0x7f) | (0x80 if sleeping else 0)))
	def settemp (self, channel, temp):
		self.send_packet (struct.pack ('<BBf', self.command['SETTEMP'], channel, temp))
	def waittemp (self):
		pass	# TODO: waittemp
	def readtemp (self, channel):
		self.send_packet (struct.pack ('<BB', self.command['READTEMP'], channel))
		return struct.unpack ('<f', self.recv_packet ())[0]
	def load (self, channel):
		self.send_packet (struct.pack ('<BB', self.command['LOAD'], channel))
	def save (self, channel):
		self.send_packet (struct.pack ('<BB', self.command['SAVE'], channel))
	def read (self, channel):
		self.send_packet (struct.pack ('<BB', self.command['READ'], channel))
		return self.recv_packet ()
	def write (self, channel, data):
		self.send_packet (struct.pack ('<BB', self.command['WRITE'], channel) + data)
	def send_packet (self, data):
		data = self.make_packet (data)
		events = 0
		ready = 0
		while True:
			self.printer.write (data)
			while True:
				r = self.printer.read (1)
				if r == self.single['ACK']:
					return	# Done.
				if r == self.single['NACK']:
					break	# Break from this loop, so the packet is sent again.
				if r == self.single['ACKWAIT']:
					self.wait = True
					return	# Done.
				assert r != self.single['STALL']	# This must not happen and if it does, there's no way to handle it, so raise an exception.
				assert r != self.single['RESET']	# This should only be sent to the firmware, not to the host.
				assert r != self.single['INIT']		# This means the printer has reset; that shouldn't happen.
				assert r != self.single['ACKRESET']	# This should only happen after we request it.
				assert r != self.single['UNUSED']	# This must not be used at all.
				if (ord (r) & 0x80) != 0:
					self.printer.write (self.single['NACK'])
					continue
				# Regular packet received; handle it.
				p = self.recv_packet (r)
	def recv_packet (self, buffer = '', want_continue = False):
		while True:
			while len (buffer) < 1:
				r = self.printer.read (1)
				if r == '' or (ord (r) & 0x80) != 0:
					self.printer.write (self.single['NACK'])
					continue
				buffer += r
			length = ((ord (buffer[0]) + 2) / 3) * 4
			while len (buffer) < length:
				r = self.printer.read (length - len (buffer))
				if r == '':
					self.printer.write (self.single['NACK'])
					buffer = ''
					break	# Start over.
				buffer += r
			else:
				# Entire buffer has been received.
				data = self.parse_packet (buffer)
				if data is None:
					self.printer.write (self.single['NACK'])
					buffer = ''
					break	# Start over.
				if (ord (data[0]) & 0x80) != self.ff_in:
					# This was a retry of the previous packet; accept it and retry.
					self.printer.write (self.single['ACK'])
					buffer = ''
					break	# Start over.
				# Packet successfully received.
				# Clear the flip flop.
				data = chr (ord (data[0]) & ~0x80) + data[1:]
				# Flip it.
				self.ff_in = chr (ord (self.ff_in) ^ 0x80)
				# Handle the asynchronous events here; don't bother the caller with them.
				if data[0] == self.rcommand['MOVECB']:
					assert len (self.movecb) > 0
					self.movecb.pop (0) ()
					buffer = ''
					break	# Start over.
				if data[0] == self.rcommand['TEMPCB']:
					assert len (self.movecb) > 0
					self.movecb.pop (0) (ord (buffer[1]))
					buffer = ''
					break	# Start over.
				if data[0] == self.rcommand['CONTINUE']:
					self.wait = False
					if want_continue:
						return ''
					buffer = ''
					break	# Start over.
				return data

#

import serial
import time


class printer:
	# Masks for computing checksums.
	mask = [[0xc0, 0xc3, 0xff, 0x09],
			[0x38, 0x3a, 0x7e, 0x13],
			[0x26, 0xb5, 0xb9, 0x23],
			[0x95, 0x6c, 0xd5, 0x43],
			[0x4b, 0xdc, 0xe2, 0x83]]
	# Single-byte commands.
	single = { 'ACK': '\x40', 'NACK': '\xe1', 'EVENT': '\xd2', 'PAUSE': '\x73', 'CONTINUE': '\xf4', 'STALL': '\x55', 'SYNC': '\x66', 'UNUSED': '\xc7' }
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
			'WRITE': '\x0b' }
	def __init__ (self, port = '/dev/ttyACM0'):
		self.printer = serial.Serial (port, baudrate = 115200, timeout = .01)
		self.printer.readall ()
		self.printer.setTimeout (2)
		assert self.printer.read () == self.single['SYNC']
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
		print ' '.join (['%02x' % ord (x) for x in data])
		length = ((len (data) + 3) / 4)
		assert (ord (data[0]) + 2) / 3 == length
		length = ord (data[0])
		checksum = data[length:]
		for t in range (len (checksum)):
			r = data[3 * t:3 * t + 3] + checksum[t]
			assert (ord (checksum[t]) & 0x7) == (t & 7)
			for bit in range (5):
				sum = 0
				for byte in range (4):
					sum ^= ord (r[byte]) & mask[bit][byte]
				sum ^= sum >> 4
				sum ^= sum >> 2
				sum ^= sum >> 1
				assert (sum & 1) == 0
		return data[1:length]
	def begin (self, version):
		self.send_packet (struct.pack ('<Bf', self.command['BEGIN'], version))
		assert struct.unpack ('<f', self.recv_packet ())[0] == 0
	def goto (self):
		pass	# TODO: goto
	def gotocb (self):
		pass	# TODO: goto
	def run (self, channel, direction):	# direction: True/False/None
		self.send_packet (struct.pack ('<BB', self.command['RUN'], (channel & 3f) | (0 if direction is None else 0xc0 if direction else 0x80)))
	def sleep (self, channel, sleeping):
		self.send_packet (struct.pack ('<BB', self.command['SLEEP'], (channel & 3f) | (0x80 if sleeping else 0)))
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
					return True
				if r == self.single['PAUSE']:
					return False
				if r == self.single['NACK']:
					break
				if r == self.single['SYNC']:
					continue
				if r == self.single['EVENT']:
					self.printer.write (self.single['EVENT'])
					events += 1
					continue
				if r == self.single['CONTINUE']:
					ready += 1
					continue
				assert r != self.single['STALL']	# This must not happen and if it does, there's no way to handle it, so raise an exception.
				if r == self.single['UNUSED']:
					continue

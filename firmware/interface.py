#

import serial
import time

mask = [[0xc0, 0xc3, 0xff, 0x09],
		[0x38, 0x3a, 0x7e, 0x13],
		[0x26, 0xb5, 0xb9, 0x23],
		[0x95, 0x6c, 0xd5, 0x43],
		[0x4b, 0xdc, 0xe2, 0x83]]

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

single = { 'ACK': '\x40', 'NACK': '\xe1', 'EVENT': '\xd2', 'PAUSE': '\x73', 'CONTINUE': '\xf4', 'STALL': '\x55', 'SYNC': '\x66', 'UNUSED': '\xc7' }

printer = serial.Serial ('/dev/ttyACM0', baudrate = 115200, timeout = .01)
printer.readall ()
printer.setTimeout (2)
printer.read ()
printer.setTimeout (.01)

printer.write (make_packet ('\x00\x00\x00\x00\x00'))
data = printer.readall ()
print ' '.join (['%02x' % ord (x) for x in parse_packet (data[1:])])

#parse_packet (make_packet ('\x00\x00\x00\x00\x00'))

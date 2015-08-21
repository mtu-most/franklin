# Module for the communication protocol.

from websockets import log

single = {
	'NACK0': '\xf0',       # Incorrect packet; please resend.
	'NACK1': '\x91',       # Incorrect packet; please resend.
	'NACK2': '\xa2',       # Incorrect packet; please resend.
	'NACK3': '\xc3',       # Incorrect packet; please resend.
	'ACK0': '\xc4',        # Packet properly received and accepted; ready for next command.  Reply follows if it should.
	'ACK1': '\xa5',        # Packet properly received and accepted; ready for next command.  Reply follows if it should.
	'ACK2': '\x96',        # Packet properly received and accepted; ready for next command.  Reply follows if it should.
	'ACK3': '\xf7',        # Packet properly received and accepted; ready for next command.  Reply follows if it should.
	'STALL0': '\x88',      # Packet properly received, but not accepted; don't resend packet unmodified.
	'STALL1': '\xe9',      # Packet properly received, but not accepted; don't resend packet unmodified.
	'STALL2': '\xda',      # Packet properly received, but not accepted; don't resend packet unmodified.
	'STALL3': '\xbb',      # Packet properly received, but not accepted; don't resend packet unmodified.
	'ID': '\xbc',          # Request/reply printer ID code.
	'DEBUG': '\xdd',       # Debug message; a nul-terminated message follows (no checksum; no resend).
	'STARTUP': '\xee',     # Starting up.
	'STALLACK': '\x8f'     # Clear stall.
	}

command = {
	'SET_UUID': 0x00,
	'GET_UUID': 0x01,
	'LINE': 0x02,
	'RUN_FILE': 0x03,
	'PROBE': 0x04,
	'SLEEP': 0x05,
	'SETTEMP': 0x06,
	'WAITTEMP': 0x07,
	'READTEMP': 0x08,
	'READPOWER': 0x09,
	'SETPOS': 0x0a,
	'GETPOS': 0x0b,
	'READ_GLOBALS': 0x0c,
	'WRITE_GLOBALS': 0x0d,
	'READ_SPACE_INFO': 0x0e,
	'READ_SPACE_AXIS': 0x0f,
	'READ_SPACE_MOTOR': 0x10,
	'WRITE_SPACE_INFO': 0x11,
	'WRITE_SPACE_AXIS': 0x12,
	'WRITE_SPACE_MOTOR': 0x13,
	'READ_TEMP': 0x14,
	'WRITE_TEMP': 0x15,
	'READ_GPIO': 0x16,
	'WRITE_GPIO': 0x17,
	'QUEUED': 0x18,
	'READPIN': 0x19,
	'HOME': 0x1a,
	'RECONNECT': 0x1b,
	'RESUME': 0x1c,
	'GETTIME': 0x1d,
	'SPI': 0x1e,
	}

rcommand = {
	'UUID': 0x40,
	'TEMP': 0x41,
	'POWER': 0x42,
	'POS': 0x43,
	'DATA': 0x44,
	'PIN': 0x45,
	'QUEUE': 0x46,
	'HOMED': 0x47,
	'TIME': 0x48,
	'MOVECB': 0x49,
	'TEMPCB': 0x4a,
	'CONTINUE': 0x4b,
	'LIMIT': 0x4c,
	'TIMEOUT': 0x4d,
	'SENSE': 0x4e,
	'DISCONNECT': 0x4f,
	'PINCHANGE': 0x50,
	'UPDATE_TEMP': 0x51,
	'UPDATE_PIN': 0x52,
	'CONFIRM': 0x53,
	'FILE_DONE': 0x54,
	'PARKWAIT': 0x55,
	}

parsed = {
	'SYSTEM': 0,
	'LINE': 1,
	'GPIO': 2,
	'SETTEMP': 3,
	'WAITTEMP': 4,
	'SETPOS': 5,
	'WAIT': 6,
	'CONFIRM': 7,
	'PARK': 8
}

mask = [[0xc0, 0xc3, 0xff, 0x09],
	[0x38, 0x3a, 0x7e, 0x13],
	[0x26, 0xb5, 0xb9, 0x23],
	[0x95, 0x6c, 0xd5, 0x43],
	[0x4b, 0xdc, 0xe2, 0x83]]

def build(packet):
	l = len(packet)
	num = l // 3
	packet += [0] * (num + 1)
	for t in range(num):
		s = t & 7
		for bit in range(5):
			check = 0
			for p in range(3):
				check ^= packet[3 * t + p] & mask[bit][p]
			check ^= s & mask[bit][3]
			check ^= check >> 4
			check ^= check >> 2
			check ^= check >> 1
			packet[l + t]
			if check & 1:
				packet[num + t] ^= 1 << (bit + 3)
	return ''.join(map(chr, packet))

def check(packet):
	num = (len(packet) + 3) // 4
	l = len(packet) - num
	for t in range(num):
		s = packet[l + t]
		if s & 7 != t:
			log('bad index %x %x %x' % (s, l, t))
			return False
		for bit in range(5):
			check = 0
			for p in range(3):
				check ^= (packet[3 * t + p] if 3 * t + p < l + t else 0) & mask[bit][p]
			check ^= s & mask[bit][3]
			check ^= check >> 4
			check ^= check >> 2
			check ^= check >> 1
			if check & 1 != 0:
				log('bad checksum')
				return False
	log('good')
	return True

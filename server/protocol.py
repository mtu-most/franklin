# Module for the communication protocol.

single = {
	'NACK': '\x80',
	'ACK0': '\xb3',
	'STALL0': '\x87',
	'STALL1': '\x9e',
	'ID': '\xaa',
	'ACK1': '\xad',
	'DEBUG': '\xb4',
	'STARTUP': '\x99',
	}

ID_MAGIC = 'e1d5e6cb'	# 4 random characters from the unused single codes.

command = {
	'RESET': 0x00,
	'GOTO': 0x01,
	'GOTOCB': 0x02,
	'PROBE': 0x03,
	'SLEEP': 0x04,
	'SETTEMP': 0x05,
	'WAITTEMP': 0x06,
	'READTEMP': 0x07,
	'READPOWER': 0x08,
	'SETPOS': 0x09,
	'GETPOS': 0x0a,
	'READ_GLOBALS': 0x0b,
	'WRITE_GLOBALS': 0x0c,
	'READ_SPACE_INFO': 0x0d,
	'READ_SPACE_AXIS': 0x0e,
	'READ_SPACE_MOTOR': 0x0f,
	'WRITE_SPACE_INFO': 0x10,
	'WRITE_SPACE_AXIS': 0x11,
	'WRITE_SPACE_MOTOR': 0x12,
	'READ_TEMP': 0x13,
	'WRITE_TEMP': 0x14,
	'READ_GPIO': 0x15,
	'WRITE_GPIO': 0x16,
	'QUEUED': 0x17,
	'READPIN': 0x18,
	'HOME': 0x19,
	'RECONNECT': 0x1a,
	'AUDIO_SETUP': 0x1b,
	'AUDIO_DATA': 0x1c,
	}

rcommand = {
	'TEMP': 0x1d,
	'POWER': 0x1e,
	'POS': 0x1f,
	'DATA': 0x20,
	'PIN': 0x21,
	'QUEUE': 0x22,
	'HOMED': 0x23,
	'MOVECB': 0x24,
	'TEMPCB': 0x25,
	'CONTINUE': 0x26,
	'LIMIT': 0x27,
	'TIMEOUT': 0x28,
	'SENSE': 0x29,
	'DISCONNECT': 0x2a,
	}

parsed = {
	'SYSTEM': 0,
	'GOTO': 1,
	'GPIO': 2,
	'SETTEMP': 3,
	'WAITTEMP': 4,
	'PARK': 5,
	'SETPOS': 6,
	'WAIT': 7,
	'CONFIRM': 8,
}

#include "firmware.h"

//#define DEBUG_SERIAL
//#define DEBUG_FF

// Commands which can be sent:
// Packet: n*4 bytes, of which n*3 bytes are data.
// Ack: 1 byte.
// Nack: 1 byte.

// The first byte of a packet is the length: 0lllllll
// The second byte of a packet is the flipflop and the command (fccccccc)
// All other commands have bit 7 set, so they cannot be mistaken for a packet.
// They have 4 bit data and 3 bit parity: 1pppdddd
// static const uint8_t MASK1[3] = {0x4b, 0x2d, 0x1e}
// Codes (low nybble is data): 80 (e1 d2) b3 b4 (d5 e6) 87 (f8) 99 aa (cb cc) ad 9e (ff)
// Codes which have duplicates in printer id codes are not used.
// These are defined in firmware.h.

static uint8_t ff_in = 0;
static uint8_t ff_out = 0;
static bool had_stall = true;
static uint16_t last_millis;

// Parity masks for decoding.
static const uint8_t MASK[5][4] = {
	{0xc0, 0xc3, 0xff, 0x09},
	{0x38, 0x3a, 0x7e, 0x13},
	{0x26, 0xb5, 0xb9, 0x23},
	{0x95, 0x6c, 0xd5, 0x43},
	{0x4b, 0xdc, 0xe2, 0x83}
};

static inline uint16_t fullpacketlen() {
	if ((command[0] & ~0x10) == CMD_BEGIN) {
		return command[1];
	}
	else if ((command[0] & ~0x10) == CMD_CONTROL) {
		return 2 + command[1] * 2;
	}
	else if ((command[0] & ~0x10) == CMD_HOME) {
		return 5 + active_motors;
	}
	else if ((command[0] & ~0x10) == CMD_MOVE) {
		return 3 + current_len;
	}
	else
		return minpacketlen();
}

// There may be serial data available.
void serial()
{
	uint16_t milliseconds = millis();
	if (!had_data && command_end > 0 && milliseconds - last_millis >= 100)
	{
		// Command not finished; ignore it and wait for next.
		watchdog_reset();
		debug("fail %d %x %ld %ld", command_end, command[0], F(last_millis), F(milliseconds));
		command_end = 0;
		if (serial_overflow)
			clear_overflow();
	}
	had_data = false;
	while (command_end == 0)
	{
		if (!serial_available()) {
			watchdog_reset();
			if (serial_overflow)
				clear_overflow();
			return;
		}
		had_data = true;
		command[0] = serial_read();
#ifdef DEBUG_SERIAL
		debug("received: %x", command[0]);
#endif
		// If this is a 1-byte command, handle it.
		switch (command[0])
		{
		case CMD_ACK0:
		case CMD_ACK1:
			// Ack: everything was ok; flip the flipflop.
			if (out_busy && ((!ff_out) ^ (command[0] == CMD_ACK1))) {	// Only if we expected it and it is the right type.
				ff_out ^= 0x10;
#ifdef DEBUG_FF
				debug("new ff_out: %d", ff_out);
#endif
				out_busy = false;
				if ((pending_packet[0] & ~0x10) == CMD_LIMIT) {
					stopping = false;
				}
			}
			continue;
		case CMD_NACK:
			// Nack: the host didn't properly receive the packet: resend.
			// Unless the last packet was already received; in that case ignore the NACK.
			if (out_busy)
				send_packet();
			continue;
		case CMD_ID:
			// Request printer id.  This is called when the host
			// connects to the printer.  This may be a reconnect,
			// and can happen at any time.
			// Response is to send the printer id, and temporarily disable all temperature readings.
			serial_write(CMD_ID);
			for (uint8_t i = 0; i < ID_SIZE; ++i)
				serial_write(printerid[i]);
			continue;
		default:
			break;
		}
		if ((command[0] & 0xe0) != 0x40) {
			// This cannot be a good packet.
			debug("invalid command %x %d", command[0], serial_buffer_tail);
			// Fake a serial overflow.
			serial_overflow = true;
			continue;
		}
		command_end = 1;
		last_millis = millis();
	}
	uint16_t len = serial_available();
	if (len == 0)
	{
#ifdef DEBUG_SERIAL
		debug("no more data available now");
#endif
		watchdog_reset();
		// If an out packet is waiting for ACK for too long, assume it didn't arrive and resend it.
		if (out_busy && millis() - out_time >= 1000) {
#ifdef DEBUG_SERIAL
			debug("resending packet");
#endif
			send_packet();
		}
		return;
	}
	had_data = true;
	if (len + command_end > MAX_COMMAND_LEN)
		len = MAX_COMMAND_LEN - command_end;
	uint16_t cmd_len = minpacketlen();
	if (command_end < cmd_len) {
		uint16_t num = min(cmd_len - command_end, len);
		for (uint16_t n = 0; n < num; ++n)
			command[command_end + n] = serial_read();
		command_end += num;
		len -= num;
#ifdef DEBUG_SERIAL
		debug("preread %d", num);
#endif
		if (command_end < cmd_len) {
			watchdog_reset();
#ifdef DEBUG_SERIAL
			debug("minimum length %d not available", cmd_len);
#endif
			return;
		}
	}
	uint16_t fulllen = fullpacketlen();
	//debug("len %d %d", cmd_len, fulllen);
	cmd_len = fulllen + (fulllen + 2) / 3;
	if (command_end + len > cmd_len) {
		len = cmd_len - command_end;
#ifdef DEBUG_SERIAL
		debug("new len %d = %d - %d", len, cmd_len, command_end);
#endif
	}
	if (len > 0) {
		for (uint16_t n = 0; n < len; ++n)
			command[command_end++] = serial_read();
	}
#ifdef DEBUG_SERIAL
	debug("check %d %x %x", fulllen, command[0], command[fulllen - 1], command[fulllen]);
#endif
	last_millis = millis();
	if (command_end < cmd_len)
	{
#ifdef DEBUG_SERIAL
		debug("not done yet; %d of %d received.", command_end, cmd_len);
#endif
		watchdog_reset();
		return;
	}
	command_end = 0;
	watchdog_reset();
	// Check packet integrity.
	// Checksum must be good.
	for (uint16_t t = 0; t < (fulllen + 2) / 3; ++t)
	{
		uint8_t sum = command[fulllen + t];
		if (fulllen == 1) {
			command[1] = 0;
			command[2] = 0;
		}
		else if (fulllen == 2 || (fulllen == 4 && t == 1))
			command[fulllen + t] = 0;
		if ((sum & 0x7) != (t & 0x7))
		{
			debug("incorrect extra bit %d %d %x %x %x %d", fulllen, t, command[0], sum, command[fulllen - 1], serial_buffer_tail);
			debug_dump();
			// Fake a serial overflow.
			command_end = 1;
			serial_overflow = true;
			return;
		}
		for (uint8_t bit = 0; bit < 5; ++bit)
		{
			uint8_t check = sum & MASK[bit][3];
			for (uint8_t p = 0; p < 3; ++p)
				check ^= command[3 * t + p] & MASK[bit][p];
			check ^= check >> 4;
			check ^= check >> 2;
			check ^= check >> 1;
			if (check & 1)
			{
				debug("incorrect checksum %d %d %d %x %x %d %d %x", fulllen, t, bit, command[0], command[fulllen - 1], command[fulllen + t], serial_buffer_tail, sum);
				debug_dump();
				// Fake a serial overflow.
				command_end = 1;
				serial_overflow = true;
				return;
			}
		}
	}
	// Packet is good.
#ifdef DEBUG_SERIAL
	debug("good");
#endif
	// Flip-flop must have good state.
	if ((command[0] & 0x10) != ff_in)
	{
		// Wrong: this must be a retry to send the previous packet, so our ack was lost.
		// Resend the ack (or stall), but don't do anything (the action has already been taken).
#ifdef DEBUG_SERIAL
		debug("duplicate");
#endif
#ifdef DEBUG_FF
		debug("old ff_in: %d", ff_in);
#endif
		if (had_stall) {
			debug("repeating stall");
			serial_write(ff_in ? CMD_STALL0 : CMD_STALL1);
		}
		else {
			debug("repeating ack");
			serial_write(ff_in ? CMD_ACK0 : CMD_ACK1);
		}
		return;
	}
	// Right: update flip-flop and send ack.
	ff_in ^= 0x10;
#ifdef DEBUG_FF
	debug("new ff_in: %d", ff_in);
#endif
	// Clear flag for easier parsing.
	command[0] &= ~0x10;
	packet();
}

// Command sending method:
// When sending a command:
// - fill appropriate command buffer
// - set flag to signal data is ready
//
// When ACK is received:
//
// When NACK is received:
// - call send_packet to resend the last packet

// Set checksum bytes.
static void prepare_packet(uint16_t len)
{
#ifdef DEBUG_SERIAL
	debug("prepare");
#endif
	if (len >= MAX_COMMAND_LEN)
	{
		debug("packet is too large: %d > %d", len, MAX_COMMAND_LEN);
		return;
	}
	// Set flipflop bit.
	pending_packet[0] &= ~0x10;
	pending_packet[0] ^= ff_out;
#ifdef DEBUG_FF
	debug("use ff_out: %d", ff_out);
#endif
	// Compute the checksums.  This doesn't work for size in (1, 2, 4), so
	// the protocol requires an initial 0 at positions 1, 2 and 5 to make
	// the checksum of those packets work.  For size % 3 != 0, the first
	// checksums are part of the data for the last checksum.  This means
	// they must have been filled in at that point.  (This is also the
	// reason (1, 2, 4) need special handling.)
	if (len == 1) {
		pending_packet[1] = 0;
		pending_packet[2] = 0;
	}
	else if (len == 2)
		pending_packet[2] = 0;
	else if (len == 4)
		pending_packet[5] = 0;
	for (uint16_t t = 0; t < (len + 2) / 3; ++t)
	{
		uint8_t sum = t & 7;
		for (uint8_t bit = 0; bit < 5; ++bit)
		{
			uint8_t check = 0;
			for (uint8_t p = 0; p < 3; ++p)
				check ^= pending_packet[3 * t + p] & MASK[bit][p];
			check ^= sum & MASK[bit][3];
			check ^= check >> 4;
			check ^= check >> 2;
			check ^= check >> 1;
			if (check & 1)
				sum ^= 1 << (bit + 3);
		}
		pending_packet[len + t] = sum;
	}
	pending_len = len + (len + 2) / 3;
}

// Send packet to host.
void send_packet()
{
#ifdef DEBUG_SERIAL
	debug("send");
#endif
	out_busy = true;
	for (uint16_t t = 0; t < pending_len; ++t)
		serial_write(pending_packet[t]);
	out_time = millis();
}

// Call send_packet if we can.
void try_send_next()
{
#ifdef DEBUG_SERIAL
	//debug("try send");
#endif
	if (out_busy)
	{
#ifdef DEBUG_SERIAL
		//debug("still busy");
#endif
		// Still busy sending other packet.
		return;
	}
	for (uint8_t m = 0; m < active_motors; ++m) {
		if (motor[m].flags & (Motor::SENSE0 | Motor::SENSE1)) {
#ifdef DEBUG_SERIAL
			debug("sense %d", m);
#endif
			uint8_t type = (motor[m].flags & Motor::SENSE1 ? 1 : 0);
			pending_packet[0] = (type ? CMD_SENSE1 : CMD_SENSE0);
			pending_packet[1] = m;
			for (uint8_t mi = 0; mi < active_motors; ++mi)
				*reinterpret_cast <int32_t *>(&pending_packet[2 + 4 * mi]) = motor[mi].sense_pos[type];
			motor[m].flags &= ~(type ? Motor::SENSE1 : Motor::SENSE0);
			prepare_packet(2 + 4 * active_motors);
			send_packet();
			return;
		}
	}
	for (uint8_t m = 0; m < active_motors; ++m) {
		if (motor[m].flags & Motor::LIMIT) {
#ifdef DEBUG_SERIAL
			debug("limit %d", m);
#endif
			pending_packet[0] = CMD_LIMIT;
			pending_packet[1] = m;
			pending_packet[2] = limit_fragment_pos;
			arch_write_current_pos(3);
			motor[m].flags &= ~Motor::LIMIT;
			prepare_packet(3 + 4 * active_motors);
			send_packet();
			return;
		}
	}
	if (timeout) {
		pending_packet[0] = CMD_TIMEOUT;
		timeout = false;
		prepare_packet(1);
		send_packet();
		return;
	}
	if (notified_current_fragment != current_fragment && (!underrun || stopped)) {
		if (underrun)
			pending_packet[0] = CMD_UNDERRUN;
		else
			pending_packet[0] = CMD_DONE;
		uint8_t num = (current_fragment - notified_current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
		pending_packet[1] = num;
		//debug("done %d %d %d", current_fragment, notified_current_fragment, last_fragment);
		notified_current_fragment = (notified_current_fragment + num) % FRAGMENTS_PER_BUFFER;
		pending_packet[2] = (last_fragment - notified_current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
		if (underrun) {
			arch_write_current_pos(3);
			prepare_packet(3 + 4 * active_motors);
		}
		else
			prepare_packet(3);
		send_packet();
		return;
	}
	if (reply_ready)
	{
#ifdef DEBUG_SERIAL
		debug("reply %x %d", reply[1], reply[0]);
#endif
		for (uint8_t i = 0; i < reply_ready; ++i)
			pending_packet[i] = reply[i];
		prepare_packet(reply_ready);
		reply_ready = 0;
		send_packet();
		return;
	}
	if (home_step_time > 0 && homers == 0 && stopped)
	{
		pending_packet[0] = CMD_HOMED;
		arch_write_current_pos(1);
		home_step_time = 0;
		prepare_packet(1 + 4 * active_motors);
		send_packet();
		return;
	}
	if (ping != 0)
	{
#ifdef DEBUG_SERIAL
		debug("pong %d", ping);
#endif
		for (uint8_t b = 0; b < 8; ++b)
		{
			if (ping & (1 << b))
			{
				pending_packet[0] = CMD_PONG;
				pending_packet[1] = b;
				ping &= ~(1 << b);
				prepare_packet(2);
				send_packet();
				return;
			}
		}
	}
	if (adcreply_ready) // This is pretty much always true, so make it the least important (nothing below this will ever be sent).
	{
#ifdef DEBUG_SERIAL
		debug("adcreply %x %d", adcreply[1], adcreply[0]);
#endif
		for (uint8_t i = 0; i < adcreply_ready; ++i)
			pending_packet[i] = adcreply[i];
		prepare_packet(adcreply_ready);
		adcreply_ready = 0;
		send_packet();
		return;
	}
}

void write_ack()
{
	//debug("acking");
	had_stall = false;
	serial_write(ff_in ? CMD_ACK0 : CMD_ACK1);
}

void write_stall()
{
	//debug("stalling");
	had_stall = true;
	serial_write(ff_in ? CMD_STALL0 : CMD_STALL1);
}

#include "firmware.h"
// vim: set foldmethod=marker :

#if 0
#define sdebug debug
#else
#define sdebug(...) do {} while (0)
#endif
//#define DEBUG_FF

// Protocol explanation.  {{{
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
// }}}

// Static variables. {{{
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
// }}}

static inline uint16_t fullpacketlen() { // {{{
	if ((command(0) & ~0x10) == CMD_BEGIN) {
		return command(1);
	}
	else if ((command(0) & ~0x10) == CMD_CONTROL) {
		return 2 + command(1) * 2;
	}
	else if ((command(0) & ~0x10) == CMD_HOME) {
		return 5 + active_motors;
	}
	else if ((command(0) & ~0x10) == CMD_MOVE) {
		return 2 + last_len;
	}
	else
		return minpacketlen();
}
// }}}

static void clear_overflow() { // {{{
	command_end = 0;
	serial_buffer_head = 0;
	serial_buffer_tail = 0;
	serial_overflow = false;
	debug("serial flushed after overflow");
	debug_dump();
	serial_write(CMD_NACK);
}
// }}}

static uint16_t serial_available() { // {{{
	if (serial_overflow)
		return 0;
	cli();
	uint16_t ret = (serial_buffer_head - serial_buffer_tail) & SERIAL_MASK;
	sei();
	return ret;
}
// }}}

static inline void inc_tail(uint16_t amount) { // {{{
	serial_buffer_tail = (serial_buffer_tail + amount) & SERIAL_MASK;
}
// }}}

// There may be serial data available.
void serial() { // {{{
	uint16_t milliseconds = millis();
	if (milliseconds - last_millis >= 2000) {
		if (serial_overflow)
			clear_overflow();
		if (!had_data && command_end > 0)
		{
			// Command not finished; ignore it and wait for next.
			watchdog_reset();
			debug("fail %d %x %ld %ld", command_end, command(0), F(last_millis), F(milliseconds));
			command_end = 0;
		}
	}
	had_data = false;
	while (command_end == 0)
	{
		if (!serial_available()) {
			watchdog_reset();
			return;
		}
		had_data = true;
		uint8_t firstbyte = serial_buffer[serial_buffer_tail];
		sdebug("received: %x", firstbyte);
		// If this is a 1-byte command, handle it.
		switch (firstbyte)
		{
		case CMD_ACK0:
		case CMD_ACK1:
			// Ack: everything was ok; flip the flipflop.
			if (out_busy && ((!ff_out) ^ (firstbyte == CMD_ACK1))) {	// Only if we expected it and it is the right type.
				ff_out ^= 0x10;
#ifdef DEBUG_FF
				debug("new ff_out: %d", ff_out);
#endif
				out_busy = false;
				if ((pending_packet[0] & ~0x10) == CMD_LIMIT) {
					stopping = false;
				}
			}
			inc_tail(1);
			continue;
		case CMD_NACK:
			// Nack: the host didn't properly receive the packet: resend.
			// Unless the last packet was already received; in that case ignore the NACK.
			if (out_busy)
				send_packet();
			inc_tail(1);
			continue;
		case CMD_ID:
			// Request printer id.  This is called when the host
			// connects to the printer.  This may be a reconnect,
			// and can happen at any time.
			// Response is to send the printer id, and temporarily disable all temperature readings.
			serial_write(CMD_ID);
			for (uint8_t i = 0; i < ID_SIZE; ++i)
				serial_write(printerid[i]);
			inc_tail(1);
			continue;
		default:
			break;
		}
		if ((firstbyte & 0xe0) != 0x40) {
			// This cannot be a good packet.
			debug("invalid command %x %d", firstbyte, serial_buffer_tail);
			// Fake a serial overflow.
			serial_overflow = true;
			continue;
		}
		command_end = 1;
		last_millis = millis();
	}
	uint16_t len = serial_available() - command_end;
	if (len == 0)
	{
		sdebug("no more data available now");
		watchdog_reset();
		// If an out packet is waiting for ACK for too long, assume it didn't arrive and resend it.
		//if (out_busy && millis() - out_time >= 1000) {
		//	sdebug("resending packet");
		//	// Don't resend, because it stops the beaglebone from booting; we still resend on NACK.
		//	//send_packet();
		//}
		return;
	}
	had_data = true;
	uint16_t cmd_len = minpacketlen();
	if (command_end < cmd_len) {
		uint16_t num = min(cmd_len - command_end, len);
		command_end += num;
		len -= num;
		sdebug("preread %d", num);
		if (command_end < cmd_len) {
			watchdog_reset();
			sdebug("minimum length %d not available", cmd_len);
			return;
		}
	}
	uint16_t fulllen = fullpacketlen();
	//debug("len %d %d", cmd_len, fulllen);
	cmd_len = fulllen + (fulllen + 2) / 3;
	if (command_end + len > cmd_len) {
		len = cmd_len - command_end;
		sdebug("new len %d = %d - %d", len, cmd_len, command_end);
	}
	if (len > 0)
		command_end += len;
	//sdebug("check %d %x %x", fulllen, command(0), command(fulllen - 1), command(fulllen));
	last_millis = millis();
	if (command_end < cmd_len)
	{
		sdebug("not done yet; %d of %d received.", command_end, cmd_len);
		watchdog_reset();
		return;
	}
	command_end = 0;
	watchdog_reset();
	// This may take long, so check limit switches.
	handle_motors();
	// Check packet integrity.
	// Checksum must be good.
	for (uint16_t t = 0; t < (fulllen + 2) / 3; ++t)
	{
		uint8_t sum = command(fulllen + t);
		if ((sum & 0x7) != (t & 0x7))
		{
			debug("incorrect extra bit %d %d %x %x %x %d", fulllen, t, command(0), sum, command(fulllen - 1), serial_buffer_tail);
			debug_dump();
			// Fake a serial overflow.
			command_end = 1;
			serial_overflow = true;
			inc_tail(cmd_len);
			return;
		}
		for (uint8_t bit = 0; bit < 5; ++bit)
		{
			uint8_t check = sum & MASK[bit][3];
			for (uint8_t p = 0; p < 3; ++p) {
				uint16_t pos = 3 * t + p;
				if ((fulllen != 1 || (pos != 1 && pos != 2)) && ((fulllen != 2 && (fulllen != 4 || t != 1)) || pos != fulllen + t))
					check ^= command(3 * t + p) & MASK[bit][p];
			}
			check ^= check >> 4;
			check ^= check >> 2;
			check ^= check >> 1;
			if (check & 1)
			{
				debug("incorrect checksum %d %d %x %x %x %x mask %x %x %x %x", t, bit, command(3 * t), command(3 * t + 1), command(3 * t + 2), command(fulllen + t), MASK[bit][0], MASK[bit][1], MASK[bit][2], MASK[bit][3]);
				debug_dump();
				// Fake a serial overflow.
				command_end = 1;
				serial_overflow = true;
				inc_tail(cmd_len);
				return;
			}
		}
	}
	// Packet is good.
	sdebug("good");
	// Flip-flop must have good state.
	if ((command(0) & 0x10) != ff_in)
	{
		// Wrong: this must be a retry to send the previous packet, so our ack was lost.
		// Resend the ack (or stall), but don't do anything (the action has already been taken).
		sdebug("duplicate");
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
		inc_tail(cmd_len);
		return;
	}
	// Right: update flip-flop and send ack.
	ff_in ^= 0x10;
#ifdef DEBUG_FF
	debug("new ff_in: %d", ff_in);
#endif
	// Clear flag for easier parsing.
	serial_buffer[serial_buffer_tail] &= ~0x10;
	// This may take long, so check limit switches.
	handle_motors();
	packet();
	inc_tail(cmd_len);
}
// }}}

// Set checksum bytes. {{{
static void prepare_packet(uint16_t len)
{
	sdebug("prepare");
	if (len >= (MAX_REPLY_LEN > 6 ? MAX_REPLY_LEN : 6))
	{
		debug("packet is too large: %d > %d", len, MAX_REPLY_LEN);
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
// }}}

// Send packet to host. {{{
void send_packet()
{
	sdebug("send");
	out_busy = true;
	for (uint16_t t = 0; t < pending_len; ++t)
		serial_write(pending_packet[t]);
	out_time = millis();
}
// }}}

void try_send_next() { // Call send_packet if we can. {{{
	sdebug("try send");
	if (out_busy) { // {{{
		sdebug("still busy");
		// Still busy sending other packet.
		return;
	} // }}}
	for (uint8_t m = 0; m < active_motors; ++m) {
		if (motor[m].flags & (Motor::SENSE0 | Motor::SENSE1)) { // {{{
			sdebug("sense %d", m);
			uint8_t type = (motor[m].flags & Motor::SENSE1 ? 1 : 0);
			pending_packet[0] = (type ? CMD_SENSE1 : CMD_SENSE0);
			pending_packet[1] = m;
			for (uint8_t mi = 0; mi < active_motors; ++mi)
				*reinterpret_cast <int32_t *>(&pending_packet[2 + 4 * mi]) = motor[mi].sense_pos[type];
			motor[m].flags &= ~(type ? Motor::SENSE1 : Motor::SENSE0);
			prepare_packet(2 + 4 * active_motors);
			send_packet();
			return;
		} // }}}
	}
	for (uint8_t m = 0; m < active_motors; ++m) {
		if (motor[m].flags & Motor::LIMIT) { // {{{
			sdebug("limit %d", m);
			pending_packet[0] = CMD_LIMIT;
			pending_packet[1] = m;
			pending_packet[2] = limit_fragment_pos;
			arch_write_current_pos(3);
			motor[m].flags &= ~Motor::LIMIT;
			prepare_packet(3 + 4 * active_motors);
			send_packet();
			return;
		} // }}}
	}
	if (timeout) { // {{{
		pending_packet[0] = CMD_TIMEOUT;
		timeout = false;
		prepare_packet(1);
		send_packet();
		return;
	} // }}}
	uint8_t cf = current_fragment;
	if (notified_current_fragment != cf) { // {{{
		if (step_state == 1) {
			pending_packet[0] = CMD_UNDERRUN;
			//debug_dump();
		}
		else
			pending_packet[0] = CMD_DONE;
		cli();
		uint8_t num = (cf - notified_current_fragment) & FRAGMENTS_PER_MOTOR_MASK;
		//debug("done %ld %d %d %d", &motor[0].current_pos, cf, notified_current_fragment, last_fragment);
		sei();
		pending_packet[1] = num;
		notified_current_fragment = (notified_current_fragment + num) & FRAGMENTS_PER_MOTOR_MASK;
		pending_packet[2] = (last_fragment - notified_current_fragment) & FRAGMENTS_PER_MOTOR_MASK;
		if (pending_packet[0] == CMD_UNDERRUN) {
			arch_write_current_pos(3);
			prepare_packet(3 + 4 * active_motors);
		}
		else
			prepare_packet(3);
		send_packet();
		return;
	} // }}}
	if (reply_ready) { // {{{
		sdebug("reply %x %d", reply[1], reply[0]);
		for (uint8_t i = 0; i < reply_ready; ++i)
			pending_packet[i] = reply[i];
		prepare_packet(reply_ready);
		reply_ready = 0;
		send_packet();
		return;
	} // }}}
	if (home_step_time > 0 && homers == 0 && step_state == 1) { // {{{
		pending_packet[0] = CMD_HOMED;
		arch_write_current_pos(1);
		home_step_time = 0;
		prepare_packet(1 + 4 * active_motors);
		send_packet();
		return;
	} // }}}
	if (ping != 0) { // {{{
		sdebug("pong %d", ping);
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
	} // }}}
	// This is pretty much always true, so make it the least important (nothing below this will ever be sent).
	if (adcreply_ready) { // {{{
		sdebug("adcreply %x %d", adcreply[1], adcreply[0]);
		for (uint8_t i = 0; i < adcreply_ready; ++i)
			pending_packet[i] = adcreply[i];
		prepare_packet(adcreply_ready);
		adcreply_ready = 0;
		send_packet();
		return;
	} // }}}
	sdebug("Nothing to send %d %d %x", cf, notified_current_fragment, debug_value);
}
// }}}

// Respond with proper ff. {{{
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
// }}}

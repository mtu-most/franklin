#include "firmware.h"
// vim: set foldmethod=marker :

//#define sdebug(fmt, ...) debug("buf %x %x %x " fmt, serial_buffer_head, serial_buffer_tail, command_end, ##__VA_ARGS__)
#define sdebug(...) do {} while (0)
//#define sdebug2 debug
#define sdebug2(...) do {} while (0)
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

static inline int16_t fullpacketlen() { // {{{
	if ((command(0) & ~0x10) == CMD_BEGIN) {
		return command(1);
	}
	else if ((command(0) & ~0x10) == CMD_CONTROL) {
		return 2 + command(1) * 2;
	}
	else if ((command(0) & ~0x10) == CMD_HOME) {
		return 5 + active_motors;
	}
	else if ((command(0) & ~0x10) == CMD_MOVE || (command(0) & ~0x10) == CMD_AUDIO) {
		return 2 + last_len;
	}
	else
		return minpacketlen();
}
// }}}

static void clear_overflow() { // {{{
	BUFFER_CHECK(serial_buffer, serial_buffer_tail);
	debug("serial flushed after overflow (%x)", serial_buffer[serial_buffer_tail]);
	command_end = 0;
	serial_buffer_head = 0;
	serial_buffer_tail = 0;
	serial_overflow = false;
	debug_dump();
	arch_serial_write(CMD_NACK);
}
// }}}

static int16_t serial_available() { // {{{
	cli();
	int16_t ret = (serial_buffer_head - serial_buffer_tail) & SERIAL_MASK;
	sei();
	return ret;
}
// }}}

static inline void inc_tail(int16_t amount) { // {{{
	cli();
	serial_buffer_tail = (serial_buffer_tail + amount) & SERIAL_MASK;
	if (serial_overflow && serial_buffer_head == serial_buffer_tail)
		clear_overflow();
	sei();
}
// }}}

// Check if there is serial data available.  This is not running from an interrupt, because it must 
void serial() { // {{{
	uint16_t milliseconds = millis();
	sdebug2("command end %d", command_end);
	if (milliseconds - last_millis >= 100) {
		if (serial_overflow)
			clear_overflow();
		if (!had_data && command_end > 0)
		{
			// Command not finished; ignore it and wait for next.
			arch_watchdog_reset();
			debug("fail %d %x %ld %ld", command_end, command(0), F(last_millis), F(milliseconds));
			clear_overflow();
			return;
		}
	}
	had_data = false;
	while (command_end == 0)
	{
		if (!serial_available()) {
			arch_watchdog_reset();
			return;
		}
		had_data = true;
		BUFFER_CHECK(serial_buffer, serial_buffer_tail);
		uint8_t firstbyte = serial_buffer[serial_buffer_tail];
		sdebug2("received: %x", firstbyte);
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
					current_fragment = (current_fragment + 1) & FRAGMENTS_PER_MOTOR_MASK;
					last_fragment = current_fragment;
					notified_current_fragment = current_fragment;
					filling = 0;
					stopping = -1;
					for (uint8_t m = 0; m < active_motors; ++m) {
						if (motor[m].flags & Motor::LIMIT) {
							stopping = m;
							break;
						}
					}
				}
			}
			inc_tail(1);
			continue;
		case CMD_NACK:
			// Nack: the host didn't properly receive the packet: resend.
			// Unless the last packet was already received; in that case ignore the NACK.
			if (out_busy) {
				debug("resend at request %x", pending_packet[0] & 0xff);
				send_packet();
			}
			else
				debug("no resend at request");
			inc_tail(1);
			continue;
		case CMD_ID:
			// Request printer id.  This is called when the host
			// connects to the printer.  This may be a reconnect,
			// and can happen at any time.
			// Response is to send the printer id, and temporarily disable all temperature readings.
			send_id(CMD_ID);
			inc_tail(1);
			continue;
		default:
			break;
		}
		if ((firstbyte & 0xe0) != 0x40) {
			// This cannot be a good packet.
			debug("invalid command %x %d", firstbyte, serial_buffer_tail);
			inc_tail(1);
			continue;
		}
		command_end = 1;
		last_millis = millis();
	}
	int16_t sa = serial_available();
	int16_t len = sa - command_end;
	sdebug("get len %d %d", sa, len);
	if (len == 0)
	{
		sdebug2("no more data available now");
		if  (serial_overflow)
			clear_overflow();
		arch_watchdog_reset();
		return;
	}
	had_data = true;
	int16_t cmd_len = minpacketlen();
	if (command_end < cmd_len) {
		int16_t num = min(cmd_len - command_end, len);
		command_end += num;
		len -= num;
		sdebug("preread %d %d", num, len);
		if (command_end < cmd_len) {
			arch_watchdog_reset();
			sdebug("minimum length %d not available (%d)", cmd_len, command_end);
			if  (serial_overflow)
				clear_overflow();
			return;
		}
	}
	int16_t fulllen = fullpacketlen();
	cmd_len = fulllen + (fulllen + 2) / 3;
	sdebug("len %d %d %d", len, cmd_len, fulllen);
	if (command_end + len > cmd_len) {
		len = cmd_len - command_end;
		sdebug("new len %d = %d - %d", len, cmd_len, command_end);
	}
	if (len > 0)
		command_end += len;
	//sdebug2("check %d %x %x", fulllen, command(0), command(fulllen - 1), command(fulllen));
	last_millis = millis();
	if (command_end < cmd_len)
	{
		sdebug("not done yet; %d of %d received.", command_end, cmd_len);
		if  (serial_overflow)
			clear_overflow();
		arch_watchdog_reset();
		return;
	}
	sdebug("packet %x", command(0) & 0xff);
	command_end = 0;
	arch_watchdog_reset();
	// Check packet integrity.
	// Checksum must be good.
	for (int16_t t = 0; t < (fulllen + 2) / 3; ++t)
	{
		uint8_t sum = command(fulllen + t);
		if ((sum & 0x7) != (t & 0x7))
		{
			debug("incorrect extra bit %d %d %x %x %x %d", fulllen, t, command(0), sum, command(fulllen - 1), serial_buffer_tail);
			debug_dump();
			inc_tail(cmd_len);
			return;
		}
		for (uint8_t bit = 0; bit < 5; ++bit)
		{
			uint8_t check = sum & MASK[bit][3];
			for (uint8_t p = 0; p < 3; ++p) {
				int16_t pos = 3 * t + p;
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
				inc_tail(cmd_len);
				return;
			}
		}
	}
	// Packet is good.
	sdebug2("good");
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
			arch_serial_write(ff_in ? CMD_STALL0 : CMD_STALL1);
		}
		else {
			debug("repeating ack");
			arch_serial_write(ff_in ? CMD_ACK0 : CMD_ACK1);
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
	BUFFER_CHECK(serial_buffer, serial_buffer_tail);
	serial_buffer[serial_buffer_tail] &= ~0x10;
	packet();
	inc_tail(cmd_len);
}
// }}}

// Set checksum bytes. {{{
static int16_t prepare_packet(int16_t len, uint8_t *packet = pending_packet)
{
	sdebug2("prepare");
	if (len > MAX_REPLY_LEN)
	{
		debug("packet is too large: %d > %d", len, MAX_REPLY_LEN);
		return 0;
	}
	if (packet == pending_packet) {
		// Set flipflop bit.
		packet[0] &= ~0x10;
		packet[0] ^= ff_out;
#ifdef DEBUG_FF
		debug("use ff_out: %d", ff_out);
#endif
	}
	// Compute the checksums.  This doesn't work for size in (1, 2, 4), so
	// the protocol requires an initial 0 at positions 1, 2 and 5 to make
	// the checksum of those packets work.  For size % 3 != 0, the first
	// checksums are part of the data for the last checksum.  This means
	// they must have been filled in at that point.  (This is also the
	// reason (1, 2, 4) need special handling.)
	if (len == 1) {
		packet[1] = 0;
		packet[2] = 0;
	}
	else if (len == 2)
		packet[2] = 0;
	else if (len == 4)
		packet[5] = 0;
	for (int16_t t = 0; t < (len + 2) / 3; ++t)
	{
		uint8_t sum = t & 7;
		for (uint8_t bit = 0; bit < 5; ++bit)
		{
			uint8_t check = 0;
			for (uint8_t p = 0; p < 3; ++p) {
				check ^= packet[3 * t + p] & MASK[bit][p];
			}
			check ^= sum & MASK[bit][3];
			check ^= check >> 4;
			check ^= check >> 2;
			check ^= check >> 1;
			if (check & 1)
				sum ^= 1 << (bit + 3);
		}
		packet[len + t] = sum;
	}
	return len + (len + 2) / 3;
}
// }}}

// Send packet to host. {{{
void send_packet()
{
	sdebug2("send");
	out_busy = true;
	for (int16_t t = 0; t < pending_len; ++t) {
		BUFFER_CHECK(pending_packet, t);
		arch_serial_write(pending_packet[t]);
	}
}
// }}}

void send_id(uint8_t cmd) { // {{{
	printerid[0] = cmd;
	int16_t len = prepare_packet(ID_SIZE + 1, printerid);
	for (uint8_t i = 0; i < len; ++i)
		arch_serial_write(printerid[i]);
} // }}}

void try_send_next() { // Call send_packet if we can. {{{
	sdebug2("try send");
	if (out_busy) { // {{{
		//debug("still busy %x", pending_packet[0] & 0xff);
		// Still busy sending other packet.
		return;
	} // }}}
	for (uint8_t m = 0; m < active_motors; ++m) {
		if (motor[m].flags & (Motor::SENSE0 | Motor::SENSE1)) { // {{{
			sdebug2("sense %d", m);
			uint8_t type = (motor[m].flags & Motor::SENSE1 ? 1 : 0);
			pending_packet[0] = (type ? CMD_SENSE1 : CMD_SENSE0);
			pending_packet[1] = m;
			for (uint8_t mi = 0; mi < active_motors; ++mi) {
				BUFFER_CHECK(pending_packet, 2 + 4 * mi);
				BUFFER_CHECK(motor, mi);
				*reinterpret_cast <int32_t *>(&pending_packet[2 + 4 * mi]) = motor[mi].sense_pos[type];
			}
			motor[m].flags &= ~(type ? Motor::SENSE1 : Motor::SENSE0);
			pending_len = prepare_packet(2 + 4 * active_motors);
			send_packet();
			return;
		} // }}}
	}
	if (pin_events > 0) { // {{{
		for (uint8_t p = 0; p < NUM_DIGITAL_PINS; ++p) {
			if (!pin[p].event())
				continue;
			pending_packet[0] = CMD_PINCHANGE;
			pending_packet[1] = p;
			pending_packet[2] = pin[p].state & CTRL_VALUE ? 1 : 0;
			pin[p].clear_event();
			pending_len = prepare_packet(3);
			send_packet();
			return;
		}
	} // }}}
	if (timeout) { // {{{
		pending_packet[0] = CMD_TIMEOUT;
		timeout = false;
		pending_len = prepare_packet(1);
		send_packet();
		return;
	} // }}}
	uint8_t cf = current_fragment;
	if (notified_current_fragment != cf) { // {{{
		if (step_state == 1 && stopping < 0) {
			pending_packet[0] = CMD_UNDERRUN;
			debug_add(0x101);
			debug_add(cf);
			debug_add(last_fragment);
			debug_add(notified_current_fragment);
			//debug_dump();
		}
		else {
			pending_packet[0] = CMD_DONE;
			debug_add(0x102);
			debug_add(cf);
			debug_add(last_fragment);
			debug_add(notified_current_fragment);
		}
		cli();
		uint8_t num = (cf - notified_current_fragment) & FRAGMENTS_PER_MOTOR_MASK;
		sei();
		sdebug2("done %ld %d %d %d", &motor[0].current_pos, cf, notified_current_fragment, last_fragment);
		pending_packet[1] = num;
		notified_current_fragment = (notified_current_fragment + num) & FRAGMENTS_PER_MOTOR_MASK;
		pending_packet[2] = (last_fragment - notified_current_fragment) & FRAGMENTS_PER_MOTOR_MASK;
		if (pending_packet[0] == CMD_UNDERRUN) {
			write_current_pos(3);
			pending_len = prepare_packet(3 + 4 * active_motors);
		}
		else
			pending_len = prepare_packet(3);
		send_packet();
		return;
	} // }}}
	if (stopping >= 0) { // {{{
		sdebug2("limit %d", stopping);
		pending_packet[0] = CMD_LIMIT;
		pending_packet[1] = stopping;
		pending_packet[2] = limit_fragment_pos;
		write_current_pos(3);
		if (stopping < active_motors)
			motor[stopping].flags &= ~Motor::LIMIT;
		pending_len = prepare_packet(3 + 4 * active_motors);
		send_packet();
		debug_add(0x100);
		//debug_dump();
		return;
	} // }}}
	if (reply_ready) { // {{{
		sdebug2("reply %x %d", reply[1], reply[0]);
		for (uint8_t i = 0; i < reply_ready; ++i) {
			BUFFER_CHECK(reply, i);
			BUFFER_CHECK(pending_packet, i);
			pending_packet[i] = reply[i];
		}
		pending_len = prepare_packet(reply_ready);
		reply_ready = 0;
		send_packet();
		return;
	} // }}}
	if (home_step_time > 0 && homers == 0 && step_state == 1) { // {{{
		pending_packet[0] = CMD_HOMED;
		write_current_pos(1);
		home_step_time = 0;
		pending_len = prepare_packet(1 + 4 * active_motors);
		send_packet();
		return;
	} // }}}
	if (ping != 0) { // {{{
		sdebug2("pong %d", ping);
		for (uint8_t b = 0; b < 8; ++b)
		{
			if (ping & (1 << b))
			{
				pending_packet[0] = CMD_PONG;
				pending_packet[1] = b;
				ping &= ~(1 << b);
				pending_len = prepare_packet(2);
				send_packet();
				return;
			}
		}
	} // }}}
	// This is pretty much always true, so make it the least important (nothing below this will ever be sent).
	if (adcreply_ready) { // {{{
		sdebug2("adcreply %x %d", adcreply[1], adcreply[0]);
		for (uint8_t i = 0; i < adcreply_ready; ++i) {
			BUFFER_CHECK(pending_packet, i);
			BUFFER_CHECK(adcreply, i);
			pending_packet[i] = adcreply[i];
		}
		pending_len = prepare_packet(adcreply_ready);
		adcreply_ready = 0;
		send_packet();
		return;
	} // }}}
	sdebug2("Nothing to send %d %d", cf, notified_current_fragment);
}
// }}}

// Respond with proper ff. {{{
void write_ack()
{
	//debug("acking");
	had_stall = false;
	arch_serial_write(ff_in ? CMD_ACK0 : CMD_ACK1);
}

void write_stall()
{
	//debug("stalling");
	had_stall = true;
	arch_serial_write(ff_in ? CMD_STALL0 : CMD_STALL1);
}
// }}}

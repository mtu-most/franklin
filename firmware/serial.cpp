/* serial.cpp - serial data handling for Franklin
 * vim: set foldmethod=marker :
 * Copyright 2014 Michigan Technological University
 * Author: Bas Wijnen <wijnen@debian.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "firmware.h"

//#define sdebug(fmt, ...) debug("buf %x %x %x " fmt, serial_buffer_head, serial_buffer_tail, command_end, ##__VA_ARGS__)
#define sdebug(...) do {} while (0)
//#define sdebug2 debug
#define sdebug2(...) do {} while (0)
//#define senddebug debug
#define senddebug(...) do {} while (0)
//#define DEBUG_FF

// Protocol explanation.  {{{
// Commands which can be sent:
// Packet: n*4 bytes, of which n*3 bytes are data.
// Ack: 1 byte.
// Nack: 1 byte.

// The first byte of a packet is the serial and the command (0ssdcccc (serial, direction, command)); 
// All 1-byte commands have bit 7 set, so they cannot be mistaken for a packet.
// They have 4 bit data and 3 bit parity: 1pppdddd.
// parity is odd to avoid near-conflicts with commands.
// static const uint8_t MASK1[3] = {0x4b, 0x2d, 0x1e}
// Codes (low nybble is data): f0 91 a2 c3 c4 a5 96 f7 88 e9 da bb bc dd ee (8f)
// These are defined in firmware.h.
// }}}

// Static variables. {{{
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

static const uint8_t cmd_ack[4] = { CMD_ACK0, CMD_ACK1, CMD_ACK2, CMD_ACK3 };
static const uint8_t cmd_nack[4] = { CMD_NACK0, CMD_NACK1, CMD_NACK2, CMD_NACK3 };
static const uint8_t cmd_stall[4] = { CMD_STALL0, CMD_STALL1, CMD_STALL2, CMD_STALL3 };
// }}}

static inline int16_t fullpacketlen() { // {{{
	if ((command(0) & 0x1f) == CMD_BEGIN) {
		return command(1);
	}
	else if ((command(0) & 0x1f) == CMD_HOME) {
		return 5 + active_motors;
	}
	else if ((command(0) & 0x1f) == CMD_MOVE || (command(0) & 0x1f) == CMD_MOVE_SINGLE) {
		return 2 + last_len;
	}
	else if ((command(0) & 0x1f) == CMD_SPI) {
		return 2 + ((command(1) + 7) >> 3);
	}
	else
		return minpacketlen();
}
// }}}

static void clear_overflow() { // {{{
	debug("serial flushed after overflow");
	command_end = 0;
	serial_buffer_head = serial_buffer;
	serial_buffer_tail = serial_buffer;
	serial_overflow = false;
	debug_dump();
	arch_serial_write(cmd_nack[ff_in]);
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
	if (amount <= 0)
		amount = 1;
	cli();
	serial_buffer_tail = (volatile uint8_t *)(((uint16_t(serial_buffer_tail) + amount) & SERIAL_MASK) | uint16_t(serial_buffer));
	if (serial_overflow && serial_buffer_head == serial_buffer_tail)
		clear_overflow();
	sei();
}
// }}}

// Check if there is serial data available.  This is not running from an interrupt, because it must 
void serial() { // {{{
	uint16_t milliseconds = millis();
	sdebug("command end %d", command_end);
	if (milliseconds - last_millis >= 500) {
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
		uint8_t firstbyte = *serial_buffer_tail;
		//debug_add(firstbyte);
		sdebug2("received: %x", firstbyte);
		// If this is a 1-byte command, handle it.
		uint8_t which = 0;
		switch (firstbyte)
		{
		case CMD_ACK3:
			which += 1;
		case CMD_ACK2:
			which += 1;
		case CMD_ACK1:
			which += 1;
		case CMD_ACK0:
		{
			// Ack: everything was ok; flip the flipflop.
			//debug("a%d out %d busy %d", which, ff_out, out_busy);
			//debug("a%d", ff_out);
			if (out_busy > 0 && ((ff_out - out_busy) & 3) == which) {	// Only if we expected it and it is the right type.
				out_busy -= 1;
				if ((pending_packet[which][0] & 0x1f) == CMD_LIMIT) {
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
		}
		case CMD_NACK3:
			which += 1;
		case CMD_NACK2:
			which += 1;
		case CMD_NACK1:
			which += 1;
		case CMD_NACK0:
		{
			arch_claim_serial();
			// Nack: the host didn't properly receive the packet: resend.
			//debug("n%d out %d busy %d", which, ff_out, out_busy);
			//debug_dump();
			// Unless the last packet was already received; in that case ignore the NACK.
			uint8_t amount = (ff_out - which) & 3;
			if (amount <= out_busy) {
				//debug("resend at request %x", pending_packet[ff_out][0] & 0xff);
				ff_out = (ff_out - amount) & 3;
				out_busy -= amount;
				while (amount--) {
					ff_out = (ff_out + 1) & 3;
					send_packet();
				}
			}
			//else
				//debug("no resend at request");
			inc_tail(1);
			continue;
		}
		case CMD_ID:
			// Request printer id.  This is called when the host
			// connects to the printer.  This may be a reconnect,
			// and can happen at any time.
			// Response is to send the printer id, and temporarily disable all temperature readings.
			arch_claim_serial();
			send_id(CMD_ID);
			inc_tail(1);
			continue;
		case CMD_STALLACK:
			had_stall = false;
			inc_tail(1);
			continue;
		default:
			break;
		}
		if ((firstbyte & 0x80) != 0x00) {
			// This cannot be a good packet.
			debug("invalid command %x", firstbyte);
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
			debug("incorrect extra bit %d %d %x %x %x", fulllen, t, command(0), sum, command(fulllen - 1));
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
				debug("incorrect checksum %d %d %x %x %x %x %d", t, bit, command(3 * t), command(3 * t + 1), command(3 * t + 2), command(fulllen + t), fulllen);
				debug_dump();
				inc_tail(cmd_len);
				return;
			}
		}
	}
	// Packet is good.
	sdebug2("good");
	if (had_stall) {
		debug("repeating stall");
		arch_serial_write(cmd_stall[ff_in]);
		inc_tail(cmd_len);
		return;
	}
	// Flip-flop must have good state.
	uint8_t which = (command(0) >> 5) & 3;
	if (which != ff_in)
	{
		// Wrong: this must be a retry to send the previous packet, so our ack was lost.
		// Resend the ack, but don't do anything (the action has already been taken).
		debug("duplicate %d %d len: %d", ff_in, which, cmd_len);
#ifdef DEBUG_FF
		debug("old ff_in: %d", ff_in);
#endif
		inc_tail(cmd_len);
		arch_serial_write(cmd_ack[which]);
		return;
	}
#ifdef DEBUG_FF
	debug("new ff_in: %d", ff_in);
#endif
	// Clear flag for easier parsing.
	*serial_buffer_tail &= 0x1f;
	//debug(">%x", command(0));
	packet();
	inc_tail(cmd_len);
}
// }}}

// Set checksum bytes. {{{
static int16_t prepare_packet(int16_t len, uint8_t *packet = 0)
{
	if (len > MAX_REPLY_LEN)
	{
		debug("packet is too large: %d > %d", len, MAX_REPLY_LEN);
		return 0;
	}
	int16_t *packetlen;
	if (!packet) {
		packet = pending_packet[ff_out];
		packetlen = &pending_len[ff_out];
		packet[0] |= ff_out << 5;
		ff_out = (ff_out + 1) & 3;
#ifdef DEBUG_FF
		debug("use ff_out: %d", ff_out);
#endif
	}
	else
		packetlen = 0;
	//debug("p%x", packet[0]);
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
	if (packetlen)
		*packetlen = len + (len + 2) / 3;
	return len + (len + 2) / 3;
}
// }}}

// Send packet to host. {{{
void send_packet()
{
	sdebug2("send");
	out_busy += 1;
	uint8_t which = (ff_out - 1) & 3;
	for (int16_t t = 0; t < pending_len[which]; ++t) {
		BUFFER_CHECK(pending_packet[which], t);
		arch_serial_write(pending_packet[which][t]);
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
	sdebug("try send");
	if (out_busy >= 3) { // {{{
		//debug("still busy %x", pending_packet[0] & 0xff);
		// Still busy sending other packet.
		return;
	} // }}}
	while (out_busy < 3) {
		if (pin_events > 0) { // {{{
			senddebug("pin");
			for (uint8_t p = 0; p < NUM_DIGITAL_PINS; ++p) {
				if (!pin[p].event())
					continue;
				pending_packet[ff_out][0] = CMD_PINCHANGE;
				pending_packet[ff_out][1] = p;
				pending_packet[ff_out][2] = pin[p].state & CTRL_VALUE ? 1 : 0;
				pin[p].clear_event();
				prepare_packet(3);
				send_packet();
				break;
			}
			continue;
		} // }}}
		if (timeout) { // {{{
			senddebug("timeout");
			pending_packet[ff_out][0] = CMD_TIMEOUT;
			timeout = false;
			prepare_packet(1);
			send_packet();
			continue;
		} // }}}
		uint8_t cf = current_fragment;
		if (notified_current_fragment != cf) { // {{{
			senddebug("done/underrun");
			uint8_t offset;
			// If a limit was hit, ignore underrun and send done instead.
			if (step_state == 1 && stopping < 0) {
				pending_packet[ff_out][0] = CMD_UNDERRUN;
				pending_packet[ff_out][1] = active_motors;
				offset = 2;
				//debug_add(0x101);
				//debug_add(cf);
				//debug_add(last_fragment);
				//debug_add(notified_current_fragment);
				//debug_dump();
			}
			else {
				pending_packet[ff_out][0] = CMD_DONE;
				offset = 1;
				//debug_add(0x102);
				//debug_add(cf);
				//debug_add(last_fragment);
				//debug_add(notified_current_fragment);
			}
			uint8_t num = (cf - notified_current_fragment) & FRAGMENTS_PER_MOTOR_MASK;
			sdebug2("done %ld %d %d %d", &motor[0].current_pos, cf, notified_current_fragment, last_fragment);
			pending_packet[ff_out][offset] = num;
			notified_current_fragment = (notified_current_fragment + num) & FRAGMENTS_PER_MOTOR_MASK;
			pending_packet[ff_out][offset + 1] = (last_fragment - notified_current_fragment) & FRAGMENTS_PER_MOTOR_MASK;
			if (pending_packet[ff_out][0] == CMD_UNDERRUN) {
				write_current_pos(4);
				prepare_packet(4 + 4 * active_motors);
			}
			else
				prepare_packet(3);
			send_packet();
			continue;
		} // }}}
		if (stopping >= 0 && (stopping == active_motors || (stopping < active_motors && motor[stopping].flags & Motor::LIMIT))) { // {{{
			senddebug("limit");
			sdebug2("limit %d", stopping);
			pending_packet[ff_out][0] = CMD_LIMIT;
			pending_packet[ff_out][1] = active_motors;
			pending_packet[ff_out][2] = stopping;
			pending_packet[ff_out][3] = limit_fragment_pos;
			write_current_pos(4);
			if (stopping < active_motors)
				motor[stopping].flags &= ~Motor::LIMIT;
			else
				stopping += 1;	// Prevent triggering another notification.
			prepare_packet(4 + 4 * active_motors);
			send_packet();
			//debug_add(0x100);
			//debug_dump();
			continue;
		} // }}}
		if (reply_ready) { // {{{
			senddebug("reply");
			sdebug2("reply %x %d", reply[1], reply[0]);
			for (uint8_t i = 0; i < reply_ready; ++i) {
				BUFFER_CHECK(reply, i);
				BUFFER_CHECK(pending_packet[ff_out], i);
				pending_packet[ff_out][i] = reply[i];
			}
			prepare_packet(reply_ready);
			reply_ready = 0;
			send_packet();
			continue;
		} // }}}
		if (home_step_time > 0 && homers == 0 && step_state == 1) { // {{{
			senddebug("homed");
			pending_packet[ff_out][0] = CMD_HOMED;
			pending_packet[ff_out][1] = active_motors;
			write_current_pos(2);
			home_step_time = 0;
			prepare_packet(2 + 4 * active_motors);
			send_packet();
			continue;
		} // }}}
		if (ping != 0) { // {{{
			senddebug("pong");
			//debug("P%x", ping);
			for (uint8_t b = 0; b < 8; ++b)
			{
				if (ping & (1 << b))
				{
					pending_packet[ff_out][0] = CMD_PONG;
					pending_packet[ff_out][1] = b;
					ping &= ~(1 << b);
					prepare_packet(2);
					send_packet();
					break;
				}
			}
			continue;
		} // }}}
		// This is pretty much always true, so make it the least important (nothing below this will ever be sent).
		if (adcreply_ready) { // {{{
			senddebug("adc");
			//debug("adcreply %x %d", adcreply[1], adcreply[0]);
			for (uint8_t i = 0; i < adcreply_ready; ++i) {
				BUFFER_CHECK(pending_packet[ff_out], i);
				BUFFER_CHECK(adcreply, i);
				pending_packet[ff_out][i] = adcreply[i];
			}
			prepare_packet(adcreply_ready);
			adcreply_ready = 0;
			send_packet();
			continue;
		} // }}}
		sdebug("Nothing to send %d %d", cf, notified_current_fragment);
		return;
	}
}
// }}}

// Respond with proper ff. {{{
void write_ack()
{
	//debug("acking %d", out_busy);
	//debug_dump();
	had_stall = false;
	arch_serial_write(cmd_ack[ff_in]);
	ff_in = (ff_in + 1) & 3;
}

void write_stall()
{
	//debug("stalling");
	had_stall = true;
	arch_serial_write(cmd_stall[ff_in]);
}
// }}}

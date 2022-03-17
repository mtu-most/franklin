/* serial.cpp - serial port data handling for Franklin {{{
 * vim: set foldmethod=marker :
 * Copyright 2014-2016 Michigan Technological University
 * Copyright 2016 Bas Wijnen <wijnen@debian.org>
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
 * }}} */

#include "cdriver.h"

// Nothing in this file is useful if there serial ports are not used for communicating.
#ifdef SERIAL

//#define DEBUG_DATA
//#define DEBUG_SERIAL
//#define DEBUG_FF

// Commands which can be sent: {{{
// Packet: n*4 bytes, of which n*3 bytes are data.
// Ack: 1 byte.
// Nack: 1 byte.

// The first byte of a packet is the length: 0lllllll
// The second byte of a packet is the flipflop and the command (fccccccc)
// All other commands have bit 7 set, so they cannot be mistaken for a packet.
// They have 4 bit data and 3 bit parity: 1pppdddd
// static const uint8_t MASK1[3] = {0x4b, 0x2d, 0x1e}
// Codes (low nybble is data): 80 (e1 d2) b3 b4 (d5 e6) 87 (f8) 99 aa (cb cc) ad 9e (ff)
// Codes which have duplicates in machine id codes are not used.
// These are defined in cdriver.h.
// }}}

// Globals. {{{
static bool had_data = false;
static bool doing_debug = false;
static uint8_t need_id = 0;
// }}}

// Parity masks for decoding. {{{
static const uint8_t MASK[5][4] = {
	{0xc0, 0xc3, 0xff, 0x09},
	{0x38, 0x3a, 0x7e, 0x13},
	{0x26, 0xb5, 0xb9, 0x23},
	{0x95, 0x6c, 0xd5, 0x43},
	{0x4b, 0xdc, 0xe2, 0x83}};
// }}}

// Constants. {{{
SingleByteCommands cmd_ack[4] = { CMD_ACK0, CMD_ACK1, CMD_ACK2, CMD_ACK3 };
SingleByteCommands cmd_nack[4] = { CMD_NACK0, CMD_NACK1, CMD_NACK2, CMD_NACK3 };
SingleByteCommands cmd_stall[4] = { CMD_STALL0, CMD_STALL1, CMD_STALL2, CMD_STALL3 };
// }}}

static void command_cancel() { // {{{
	while (true) {
		command_end -= 1;
		memmove(command, &command[1], command_end);
		if (command_end == 0)
			break;
		int len = 0;
		if (hwpacketsize(command_end, &len) <= command_end)
			break;
	}
} // }}}

static void resend(int amount) { // {{{
	// Unless the last packet was already received; in that case ignore the NACK.
	//debug("nack%d ff %d busy %d", which, ff_out, out_busy);
	if (out_busy >= amount) {
		ff_out = (ff_out - amount) & 3;
		out_busy -= amount;
		while (amount--) {
			ff_out = (ff_out + 1) & 3;
			send_packet();
		}
	}
} // }}}

static void try_pending() {
	if (out_busy < 3 && change_pending)
		arch_motors_change();
	if (out_busy < 3 && start_pending) {
		arch_start_move(0);
	}
	if (out_busy < 3 && stop_pending) {
		//debug("do pending stop");
		arch_stop();
	}
	if (out_busy < 3 && discard_pending)
		arch_do_discard();
	if (!sending_fragment && !stopping && arch_running()) {
		run_file_fill_queue();
		buffer_refill();
	}
}

// There may be serial data available.
bool serial(bool allow_pending) { // {{{
	while (true) { // Loop until all data is handled.
		// Handle timeouts on serial line.
		int32_t utm = utime();
		if (int32_t(utm - last_micros) >= 100000)
		{
			if (command_end > 0) {
				if (!had_data) {
					// Command not finished; ignore it and wait for next.
					debug("Ignoring unfinished command");
					command_cancel();
					last_micros = utm;
				}
				//debug("Silence, but handle data first");
			}
			else {
				debug("Too much silence; request packet to be sure");
				write_nack();
				resend(out_busy);
				last_micros = utm;
			}
		}
		had_data = false;
		if (!serialdev->available()) {
			if (allow_pending)
				try_pending();
			return false;
		}
		while (command_end == 0) { // First byte. {{{
			if (!serialdev->available()) {
				//debug("serial done");
				if (allow_pending)
					try_pending();
				return true;
			}
			command[0] = serialdev->read();
#ifdef DEBUG_SERIAL
			debug("received: %x", command[0]);
#endif
			had_data = true;
			if (doing_debug) {
				if (command[0] == 0) {
					END_DEBUG();
					doing_debug = false;
					continue;
				}
				DO_DEBUG(command[0]);
				continue;
			}
			if (need_id) {
				need_id -= 1;
				if (!need_id) {
					// Firmware has reset.
					//arch_reset();
					// avr_* should not be used outside arch_avr.h, but this is just for debugging...
					if (avr_pong == 255)
						debug("firmware sent id");
				}
				continue;
			}
			// If this is a 1-byte command, handle it.
			int which = 0;
			switch (command[0])
			{
			case CMD_DEBUG:
				doing_debug = true;
				START_DEBUG();
				continue;
			case CMD_STALL3:
				which += 1;
				// Fall through.
			case CMD_STALL2:
				which += 1;
				// Fall through.
			case CMD_STALL1:
				which += 1;
				// Fall through.
			case CMD_STALL0:
				debug("received stall!");
				ff_out = which;
				out_busy = 0;
				serialdev->write(CMD_STALLACK);
				which += 1;
				abort();
				// Fall through.
			case CMD_ACK3:
				which += 1;
				// Fall through.
			case CMD_ACK2:
				which += 1;
				// Fall through.
			case CMD_ACK1:
				which += 1;
				// Fall through.
			case CMD_ACK0:
				//debug("ack%d ff %d busy %d alllow pending %d discard pending %d", which, ff_out, out_busy, allow_pending, discard_pending);
				which &= 3;
				// Ack: flip the flipflop.
				if (out_busy > 0 && ((ff_out - out_busy) & 3) == which) { // Only if we expected it and it is the right type.
					out_busy -= 1;
					void (*cb)() = serial_cb[0];
					for (int i = 0; i < out_busy; ++i)
						serial_cb[i] = serial_cb[i + 1];
					serial_cb[out_busy] = NULL;
					if (cb)
						cb();
				}
				if (allow_pending) {
					try_pending();
				}
				if (!preparing)
					arch_had_ack();
				continue;
			case CMD_NACK3:
				which += 1;
				// Fall through.
			case CMD_NACK2:
				which += 1;
				// Fall through.
			case CMD_NACK1:
				which += 1;
				// Fall through.
			case CMD_NACK0:
			{
				// Nack: the host didn't properly receive the packet: resend.
				//debug("nack received");
				int amount = ((ff_out - which - 1) & 3) + 1;
				resend(amount);
				continue;
			}
			case CMD_ID:
			case CMD_STARTUP:
				// Request machine id.  This is called when the host
				// connects to the machine.  This may be a reconnect,
				// and can happen at any time.
				// Response to host is to send the machine id; from machine this is handled after receiving the id.
				need_id = ID_SIZE + UUID_SIZE + (1 + ID_SIZE + UUID_SIZE + 2) / 3;
				continue;
			default:
				if (!expected_replies)
					last_micros = utime();
				break;
			}
			if ((command[0] & 0x90) != 0x10) {
				// These lengths are not allowed; this cannot be a good packet.
				debug("invalid firmware command code %02x", command[0]);
				//abort();
				write_nack();
				continue;
			}
			command_end = 1;
		} // }}}
		int len = serialdev->available();
		// Get at least two bytes in buffer.
		if (command_end == 1) {
			if (len == 0)
				break;
			command[1] = serialdev->read();
#ifdef DEBUG_SERIAL
			debug("second byte: %x", command[1]);
#endif
			len -= 1;
			command_end += 1;
		}
		had_data = true;
		if (!expected_replies)
			last_micros = utime();
		if (len + command_end > COMMAND_SIZE) {
			debug("clip size! %d %d %d", len, command_end, COMMAND_SIZE);
			len = COMMAND_SIZE - command_end;
			if (len == 0) {
				debug("canceling command");
				command_cancel();
				continue;
			}
		}
		int cmd_len;
		cmd_len = hwpacketsize(command_end, &len);
		cmd_len += (cmd_len + 2) / 3;
		if (cmd_len > FULL_COMMAND_SIZE) {
			// This command does not fit in the buffer, so it cannot be parsed.  Reject it.
			debug("Command length %d larger than buffer; rejecting", cmd_len);
			command_cancel();
			continue;
		}
		if (command_end + len > cmd_len)
			len = cmd_len - command_end;
		serialdev->readBytes(reinterpret_cast <char *> (&command[command_end]), len);
#ifdef DEBUG_SERIAL // {{{
		debug("read %d bytes:", len);
		for (uint8_t i = 0; i < len; ++i)
			fprintf(stderr, " %02x", command[command_end + i]);
		fprintf(stderr, "\n");
#endif // }}}
		last_micros = utime();
		command_end += len;
		if (command_end < cmd_len) {
#ifdef DEBUG_SERIAL // {{{
			debug("not done yet; %d of %d received.", command_end, cmd_len);
#endif // }}}
			return true;
		}
#ifdef DEBUG_DATA
		char const *recvname[0x10] = {"ready", "pong", "homed", "pin", "stopped", "named-pin", "done", "underrun", "adc", "limit", "timeout", "pinchange", "c", "d", "e", "f"};
		if ((command[0] & 0xf) != 8) {
			fprintf(stderr, "recv: %s ", recvname[command[0] & 0xf]);
			for (int i = 0; i < command_end; ++i) {
				if (i == command_end - (command_end + 3) / 4)
					fprintf(stderr, "\t\t*");
				fprintf(stderr, " %02x", command[i]);
			}
			fprintf(stderr, "\n");
		}
#endif
		int end = command_end;
		// Check packet integrity.
		// Checksum must be good.
		len = hwpacketsize(end, NULL);
		for (uint8_t t = 0; t < (len + 2) / 3; ++t) {
			uint8_t sum = command[len + t];
			if (len == 1)
				command[2] = 0;
			if (len <= 2 || (len == 4 && t == 1))
				command[len + t] = 0;
			if ((sum & 0x7) != (t & 0x7))
			{
				debug("incorrect extra bit, size = %d t = %d", len, t);
				//abort();
				command_cancel();
				if (command_end == 0)
					write_nack();
				else
					continue;
				return true;
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
					debug("incorrect checksum byte %d bit %d", t, bit);
					//abort();
		//fprintf(stderr, "err (%d %d):", len, t);
		//for (uint8_t i = 0; i < len + (len + 2) / 3; ++i)
		//	fprintf(stderr, " %02x", command[i]);
		//fprintf(stderr, "\n");
					command_cancel();
					if (command_end == 0)
						write_nack();
					else
						continue;
					return true;
				}
			}
		}
		// Packet is good.
		//debug("good");
		// Flip-flop must have good state.
		int which = (command[0] >> 5) & 3;
		if (which != ff_in)
		{
			// Wrong: this must be a retry to send the previous packet, so our ack was lost.
			// Resend the ack, but don't do anything (the action has already been taken).
			//debug("duplicate %x", command[0]);
#ifdef DEBUG_FF
			debug("old ff_in: %d", ff_in);
#endif
			serialdev->write(cmd_ack[which]);
			command_end = 0;
			continue;
		}
#ifdef DEBUG_FF
		debug("new ff_in: %d", ff_in);
#endif
		// Clear flag for easier parsing.
		command[0] &= 0x1f;
		command_end = 0;
		if (hwpacket(cmd_len)) {
			void (*cb)() = wait_for_reply[0];
			expected_replies -= 1;
			for (int i = 0; i < expected_replies; ++i)
				wait_for_reply[i] = wait_for_reply[i + 1];
			wait_for_reply[expected_replies] = NULL;
			cb();
			break;
		}
	}
	return true;
} // }}}

// This is run in a loop until some event happened.
void serial_wait(int timeout) { // {{{
	poll(&pollfds[BASE_FDS], 1, timeout);
	serial(false);
} // }}}

// Command sending method:
// When sending a command:
// - fill appropriate command buffer
// - set flag to signal data is ready
// - call try_send_next
//
// When ACK is received:
// - call try_send_next
//
// When NACK is received:
// - call send_packet to resend the last packet

// Set checksum bytes.
bool prepare_packet(char *the_packet, int size) { // {{{
	//debug("prepare %d %d %d", size, ff_out, out_busy);
	if (size >= COMMAND_SIZE)
	{
		debug("packet is too large: %d > %d", size, COMMAND_SIZE);
		return false;
	}
	if (preparing) {
		debug("Prepare_packet is called recursively.  Aborting.");
		abort();
	}
	// Wait for room in the queue.  This is required to avoid a stall being received in between prepare and send.
	preparing = true;
	while (out_busy >= 3)
		serial_wait();
	preparing = false;	// Not yet, but there are no further interruptions.
	if (stopping)
		return false;
	// Set flipflop bit.
	the_packet[0] &= 0x1f;
	the_packet[0] |= ff_out << 5;
#ifdef DEBUG_FF
	debug("use ff_out: %d", ff_out);
#endif
	// Compute the checksums.  This doesn't work for size in (1, 2, 4), so
	// the protocol requires initial 0's at checksum positions.
	// For size % 3 != 0, the first checksums are part of the data for the
	// last checksum.  This means they must have been filled in at that
	// point.  (This is also the reason (1, 2, 4) cause trouble.)
	if (size == 1) {
		the_packet[1] = 0;
		the_packet[2] = 0;
	}
	else if (size == 2)
		the_packet[2] = 0;
	else if (size == 4)
		the_packet[5] = 0;
	for (uint8_t t = 0; t < (size + 2) / 3; ++t)
	{
		uint8_t sum = t & 7;
		for (uint8_t bit = 0; bit < 5; ++bit)
		{
			uint8_t check = 0;
			for (uint8_t p = 0; p < 3; ++p) {
				//debug("c %d %d %x %x %x", t, p, the_packet[3 * t + p], MASK[bit][p], the_packet[3 * t + p] & MASK[bit][p]);
				check ^= the_packet[3 * t + p] & MASK[bit][p];
			}
			check ^= sum & MASK[bit][3];
			check ^= check >> 4;
			check ^= check >> 2;
			check ^= check >> 1;
			if (check & 1)
				sum ^= 1 << (bit + 3);
		}
		the_packet[size + t] = sum;
	}
	pending_len[ff_out] = size + (size + 2) / 3;
#ifdef DEBUG_SERIAL
	fprintf(stderr, "prepare %p:", the_packet);
#endif
	for (uint8_t i = 0; i < pending_len[ff_out]; ++i) {
		pending_packet[ff_out][i] = the_packet[i];
#ifdef DEBUG_SERIAL
		fprintf(stderr, " %02x", int(uint8_t(pending_packet[ff_out][i])));
#endif
	}
#ifdef DEBUG_SERIAL
	fprintf(stderr, "\n");
#endif
	ff_out = (ff_out + 1) & 3;
	return true;
} // }}}

// Send packet to firmware.
void send_packet() { // {{{
	int which = (ff_out - 1) & 3;
#ifdef DEBUG_DATA
	char const *sendname[0x20] = {"begin", "ping", "set-uuid", "setup", "control", "msetup", "asetup", "home", "start-move", "start-probe", "move", "move-single", "pattern", "start", "stop", "abort", "discard", "getpin", "spi", "pinname", "14", "15", "16", "17", "18", "19", "1a", "1b", "1c", "1d", "1e", "1f"};
	fprintf(stderr, "send (%d): %s ", which, sendname[pending_packet[which][0] & 0x1f]);
	for (uint8_t i = 0; i < pending_len[which]; ++i) {
		if (i == pending_len[which] - (pending_len[which] + 3) / 4)
			fprintf(stderr, "\t\t/");
		fprintf(stderr, " %02x", int(uint8_t(pending_packet[which][i])));
	}
	fprintf(stderr, "\n");
#endif
	for (uint8_t t = 0; t < pending_len[which]; ++t)
		serialdev->write(pending_packet[which][t]);
	out_busy += 1;
	out_time = utime();
} // }}}

void write_ack() { // {{{
//#ifdef DEBUG_DATA
	//debug("wack %d", ff_in);
//#endif
	serialdev->write(cmd_ack[ff_in]);
	ff_in = (ff_in + 1) & 3;
} // }}}

void write_nack() { // {{{
//#ifdef DEBUG_DATA
	//debug("wnack %d", ff_in);
//#endif
	serialdev->write(cmd_nack[ff_in]);
} // }}}

#endif

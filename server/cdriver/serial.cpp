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

//#define DEBUG_DATA
//#define DEBUG_HOST
//#define DEBUG_ALL_HOST
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

struct Queuerecord { // {{{
	Queuerecord *next;
	unsigned len;
	char cmd;
	int32_t s, m, e;
	double f;
}; // }}}

// Globals. {{{
static bool sending_to_host = false;
static Queuerecord *hostqueue_head = NULL;
static Queuerecord *hostqueue_tail = NULL;
#ifdef SERIAL
static bool had_data = false;
static bool doing_debug = false;
static uint8_t need_id = 0;
#endif
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
const SingleByteCommands cmd_ack[4] = { CMD_ACK0, CMD_ACK1, CMD_ACK2, CMD_ACK3 };
const SingleByteCommands cmd_nack[4] = { CMD_NACK0, CMD_NACK1, CMD_NACK2, CMD_NACK3 };
const SingleByteCommands cmd_stall[4] = { CMD_STALL0, CMD_STALL1, CMD_STALL2, CMD_STALL3 };
// }}}

static void send_to_host() { // {{{
	sending_to_host = true;
	//debug("sending");
	Queuerecord *r = hostqueue_head;
	hostqueue_head = r->next;
	if (!hostqueue_head)
		hostqueue_tail = NULL;
#ifdef DEBUG_HOST
#ifndef DEBUG_ALL_HOST
	if (r->cmd != CMD_POS && r->cmd != CMD_TIME && r->cmd != CMD_TEMP)
#endif
	{
		debug("**** host send cmd %02x s %08x m %08x e %08x f %f data len %d", r->cmd, r->s, r->m, r->e, r->f, r->len);
		for (uint8_t i = 0; i < r->len; ++i)
			fprintf(stderr, " %02x", reinterpret_cast <unsigned char *>(r)[sizeof(Queuerecord) + i]);
		fprintf(stderr, "\n");
	}
#endif
	serialdev[0]->write(22 + r->len);
	serialdev[0]->write(r->cmd);
	for (unsigned i = 0; i < sizeof(int32_t); ++i)
		serialdev[0]->write(reinterpret_cast <char *>(&r->s)[i]);
	for (unsigned i = 0; i < sizeof(int32_t); ++i)
		serialdev[0]->write(reinterpret_cast <char *>(&r->m)[i]);
	for (unsigned i = 0; i < sizeof(int32_t); ++i)
		serialdev[0]->write(reinterpret_cast <char *>(&r->e)[i]);
	for (unsigned i = 0; i < sizeof(double); ++i)
		serialdev[0]->write(reinterpret_cast <char *>(&r->f)[i]);
	for (unsigned i = 0; i < r->len; ++i)
		serialdev[0]->write(reinterpret_cast <char *>(r)[sizeof(Queuerecord) + i]);
	if (r->cmd == CMD_LIMIT)
		stopping = 1;
	free(r);
} // }}}

#ifdef SERIAL
static void command_cancel() { // {{{
	while (true) {
		command_end[1] -= 1;
		memmove(command[1], &command[1][1], command_end[1]);
		if (command_end[1] == 0)
			break;
		int len = 0;
		if (hwpacketsize(command_end[1], &len) <= command_end[1])
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
#endif

// There may be serial data available.
bool serial(uint8_t channel) { // {{{
	while (true) { // Loop until all data is handled.
#ifdef SERIAL // Handle timeouts on serial line. {{{
		if (channel == 1) {
			int32_t utm = utime();
			if (int32_t(utm - last_micros) >= 100000)
			{
				if (command_end[channel] > 0) {
					if (!had_data) {
						// Command not finished; ignore it and wait for next.
						debug("Ignoring unfinished command");
						command_cancel();
						last_micros = utm;
					}
					//debug("Silence, but handle data first");
				}
				else {
					//debug("Too much silence; request packet to be sure");
					write_nack();
					resend(out_busy);
					last_micros = utm;
				}
			}
			had_data = false;
		}
#endif // }}}
		if (channel == 0) { // Ignore host data while blocking it.
			if (host_block)
				return false;
		}
		if (!serialdev[channel]->available())
			return false;
		while (command_end[channel] == 0) { // First byte. {{{
			if (!serialdev[channel]->available()) {
				//debug("serial %d done", channel);
				return true;
			}
			command[channel][0] = serialdev[channel]->read();
#ifdef DEBUG_SERIAL
			debug("received on %d: %x", channel, command[channel][0]);
#endif
#ifdef SERIAL // {{{
			if (channel == 1) {
				had_data = true;
				if (doing_debug) {
					if (command[channel][0] == 0) {
						END_DEBUG();
						doing_debug = false;
						continue;
					}
					DO_DEBUG(command[channel][0]);
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
				switch (command[channel][0])
				{
				case CMD_DEBUG:
					if (channel == 0) {
						debug("wtf?");
						abort();
					}
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
					serialdev[1]->write(CMD_STALLACK);
					which += 1;
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
					//debug("ack%d ff %d busy %d", which, ff_out, out_busy);
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
					if (channel == 0) {
						debug("ID request from host");
						continue;
					}
					need_id = ID_SIZE + UUID_SIZE + (1 + ID_SIZE + UUID_SIZE + 2) / 3;
					continue;
				default:
					if (!expected_replies)
						last_micros = utime();
					break;
				}
				if ((command[1][0] & 0x90) != 0x10) {
					// These lengths are not allowed; this cannot be a good packet.
					debug("invalid firmware command code %02x", command[1][0]);
					//abort();
					write_nack();
					continue;
				}
			}
			else {
#endif // }}}
				// Message received.
				if (command[channel][0] == OK) {
					if (sending_to_host) {
						//debug("no longer sending");
						sending_to_host = false;
						//debug("received OK; sending next to host (if any)");
						if (stopping == 1) {
							//debug("done stopping");
							stopping = 0;
							sending_fragment = 0;
						}
						if (hostqueue_head) {
							//debug("sending next");
							send_to_host();
						}
					}
					else {
						debug("received unexpected OK");
						abort();
					}
					continue;
				}
				else if (command[channel][0] & 0x80) {
					debug("invalid first byte from host: 0x%02x", command[channel][0] & 0xff);
					abort();
				}
#ifdef SERIAL // {{{
			}
#endif // }}}
			command_end[channel] = 1;
		} // }}}
		int len = serialdev[channel]->available();
		// Get at least two bytes in buffer.
		if (command_end[channel] == 1) {
			if (len == 0)
				break;
			command[channel][1] = serialdev[channel]->read();
#ifdef DEBUG_SERIAL
			debug("second byte on %d: %x", channel, command[channel][1]);
#endif
			len -= 1;
			command_end[channel] += 1;
		}
#ifdef SERIAL // {{{
		if (channel == 1) {
			had_data = true;
			if (!expected_replies)
				last_micros = utime();
			if (len + command_end[channel] > COMMAND_SIZE) {
				debug("clip size! %d %d %d", len, command_end[channel], COMMAND_SIZE);
				len = COMMAND_SIZE - command_end[channel];
				if (len == 0) {
					debug("canceling command");
					command_cancel();
					continue;
				}
			}
		}
		else
#endif // }}}
			if (len + command_end[channel] > FULL_COMMAND_SIZE[channel]) {
				debug("clip size! %d %d %d", len, command_end[channel], FULL_COMMAND_SIZE[channel]);
				len = FULL_COMMAND_SIZE[channel] - command_end[channel];
			}
		int cmd_len;
#ifdef SERIAL // {{{
		if (channel == 1) {
			cmd_len = hwpacketsize(command_end[channel], &len);
			cmd_len += (cmd_len + 2) / 3;
		}
		else
#endif // }}}
			cmd_len = ((command[channel][0] & 0xff) << 8) | (command[channel][1] & 0xff);
		if (cmd_len > FULL_COMMAND_SIZE[channel]) {
			// This command does not fit in the buffer, so it cannot be parsed.  Reject it.
			debug("Command length %d larger than buffer; rejecting", cmd_len);
#ifdef SERIAL // {{{
			if (channel == 1)
				command_cancel();
#endif // }}}
			continue;
		}
		if (command_end[channel] + len > cmd_len)
			len = cmd_len - command_end[channel];
		serialdev[channel]->readBytes(reinterpret_cast <char *> (&command[channel][command_end[channel]]), len);
#ifdef DEBUG_SERIAL // {{{
		debug("read %d bytes on %d:", len, channel);
		for (uint8_t i = 0; i < len; ++i)
			fprintf(stderr, " %02x", command[channel][command_end[channel] + i]);
		fprintf(stderr, "\n");
#endif // }}}
#ifdef SERIAL // {{{
		if (channel == 1)
			if (!expected_replies)
				last_micros = utime();
#endif // }}}
		command_end[channel] += len;
		if (command_end[channel] < cmd_len)
		{
#ifdef DEBUG_SERIAL // {{{
			debug("%d not done yet; %d of %d received.", channel, command_end[channel], cmd_len);
#endif // }}}
			return true;
		}
#ifdef SERIAL // {{{
#ifdef DEBUG_DATA
		if (channel == 1 && (command[1][0] & 0xf) != 8) {
			fprintf(stderr, "recv:");
			for (uint8_t i = 0; i < command_end[channel]; ++i)
				fprintf(stderr, " %02x", command[channel][i]);
			fprintf(stderr, "\n");
		}
#endif
#endif // }}}
#ifdef DEBUG_HOST // {{{
		if (channel == 0) {
#ifndef DEBUG_ALL_HOST
			if (command[channel][2] != CMD_GETPOS && command[channel][2] != CMD_GETTIME && command[channel][2] != CMD_READTEMP)
#endif
			{
				fprintf(stderr, "**** host recv:");
				for (uint8_t i = 0; i < command_end[channel]; ++i)
					fprintf(stderr, " %02x", command[channel][i]);
				fprintf(stderr, "\n");
			}
		}
#endif // }}}
#ifdef SERIAL // {{{
		int end = command_end[channel];
		// Check packet integrity.
		if (channel == 1) {
			// Checksum must be good.
			len = hwpacketsize(end, NULL);
			for (uint8_t t = 0; t < (len + 2) / 3; ++t)
			{
				uint8_t sum = command[channel][len + t];
				if (len == 1)
					command[channel][2] = 0;
				if (len <= 2 || (len == 4 && t == 1))
					command[channel][len + t] = 0;
				if ((sum & 0x7) != (t & 0x7))
				{
					debug("incorrect extra bit, size = %d t = %d", len, t);
					//abort();
					command_cancel();
					if (command_end[channel] == 0)
						write_nack();
					else
						continue;
					return true;
				}
				for (uint8_t bit = 0; bit < 5; ++bit)
				{
					uint8_t check = sum & MASK[bit][3];
					for (uint8_t p = 0; p < 3; ++p)
						check ^= command[channel][3 * t + p] & MASK[bit][p];
					check ^= check >> 4;
					check ^= check >> 2;
					check ^= check >> 1;
					if (check & 1)
					{
						debug("incorrect checksum byte %d bit %d", t, bit);
						//abort();
			//fprintf(stderr, "err %d (%d %d):", channel, len, t);
			//for (uint8_t i = 0; i < len + (len + 2) / 3; ++i)
			//	fprintf(stderr, " %02x", command[channel][i]);
			//fprintf(stderr, "\n");
						command_cancel();
						if (command_end[channel] == 0)
							write_nack();
						else
							continue;
						return true;
					}
				}
			}
			// Packet is good.
			//debug("%d good", channel);
			// Flip-flop must have good state.
			int which = (command[channel][0] >> 5) & 3;
			if (which != ff_in)
			{
				// Wrong: this must be a retry to send the previous packet, so our ack was lost.
				// Resend the ack, but don't do anything (the action has already been taken).
				//debug("%d duplicate %x", channel, command[channel][0]);
#ifdef DEBUG_FF
				debug("old ff_in: %d", ff_in);
#endif
				serialdev[channel]->write(cmd_ack[which]);
				command_end[channel] = 0;
				continue;
			}
#ifdef DEBUG_FF
			debug("new ff_in: %d", ff_in);
#endif
			// Clear flag for easier parsing.
			command[channel][0] &= 0x1f;
			command_end[channel] = 0;
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
		else {
#endif // }}}
			command_end[channel] = 0;
			packet();
#ifdef SERIAL // {{{
		}
#endif // }}}
	}
	return true;
} // }}}

#ifdef SERIAL
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
	while (out_busy >= 3) {
		poll(&pollfds[2], 1, -1);
		serial(1);
	}
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
	fprintf(stderr, "send (%d): ", out_busy);
	for (uint8_t i = 0; i < pending_len[which]; ++i)
		fprintf(stderr, " %02x", int(uint8_t(pending_packet[which][i])));
	fprintf(stderr, "\n");
#endif
	for (uint8_t t = 0; t < pending_len[which]; ++t)
		serialdev[1]->write(pending_packet[which][t]);
	out_busy += 1;
	out_time = utime();
} // }}}

void write_ack() { // {{{
//#ifdef DEBUG_DATA
	//debug("wack %d", ff_in);
//#endif
	serialdev[1]->write(cmd_ack[ff_in]);
	ff_in = (ff_in + 1) & 3;
} // }}}

void write_nack() { // {{{
//#ifdef DEBUG_DATA
	//debug("wnack %d", ff_in);
//#endif
	serialdev[1]->write(cmd_nack[ff_in]);
} // }}}
#endif

void send_host(char cmd, int s, int m, double f, int e, unsigned len) { // {{{
	//debug("queueing for host cmd %x", cmd);
	// Use malloc, not mem_alloc, because there are multiple pointers to the same memory and mem_alloc cannot handle that.
	Queuerecord *record = reinterpret_cast <Queuerecord *>(malloc(sizeof(Queuerecord) + len));
	if (hostqueue_head)
		hostqueue_tail->next = record;
	else
		hostqueue_head = record;
	hostqueue_tail = record;
	record->s = s;
	record->m = m;
	record->f = f;
	record->e = e;
	record->cmd = cmd;
	record->len = len;
	record->next = NULL;
	for (unsigned i = 0; i < len; ++i)
		reinterpret_cast <char *>(record)[sizeof(Queuerecord) + i] = datastore[i];
	if (!sending_to_host) {
		//debug("immediately sending");
		send_to_host();
	}
	//else
	//	debug("queueing host cmd");
} // }}}

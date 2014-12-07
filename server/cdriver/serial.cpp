#include "cdriver.h"

//#define DEBUG_DATA
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
// These are defined in cdriver.h.

struct Record {
	int32_t s, m, e;
	float f;
};

struct Queuerecord {
	Record r;
	Queuerecord *next;
	int len;
	char cmd;
};

static bool sending_to_host = false;
static Queuerecord *hostqueue_head = NULL;
static Queuerecord *hostqueue_tail = NULL;
static uint32_t last_micros = 0;
static bool had_data = false;
static bool had_stall = false;
static bool doing_debug = false;
static uint8_t need_id = 0;
static char the_id[ID_SIZE];
static char out_buffer[16];
static uint8_t ff_in = 0;
static uint8_t ff_out = 0;

// Parity masks for decoding.
static const uint8_t MASK[5][4] = {
	{0xc0, 0xc3, 0xff, 0x09},
	{0x38, 0x3a, 0x7e, 0x13},
	{0x26, 0xb5, 0xb9, 0x23},
	{0x95, 0x6c, 0xd5, 0x43},
	{0x4b, 0xdc, 0xe2, 0x83}};

static void send_to_host() {
	sending_to_host = true;
	Queuerecord *r = hostqueue_head;
	hostqueue_head = r->next;
	if (!hostqueue_head)
		hostqueue_tail = NULL;
	//debug("sending to host cmd %x", r->cmd);
	serialdev[0]->write(18 + r->len);
	serialdev[0]->write(r->cmd);
	for (unsigned i = 0; i < sizeof(Record); ++i)
		serialdev[0]->write(reinterpret_cast <char *>(&r->r)[i]);
	for (unsigned i = 0; i < r->len; ++i)
		serialdev[0]->write(reinterpret_cast <char *>(r)[sizeof(Queuerecord) + i]);
	if (r->cmd == CMD_LIMIT)
		stopping = 1;
	free(r);
}

// There may be serial data available.
void serial(uint8_t which)
{
	while (serialdev[which]->available()) {
		if (which == 1) {
			if (!had_data && command_end[which] > 0 && utime() - last_micros >= 100000)
			{
				// Command not finished; ignore it and wait for next.
				command_end[which] = 0;
			}
			had_data = false;
		}
		while (command_end[which] == 0)
		{
			if (!serialdev[which]->available()) {
				//debug("serial %d done", which);
				return;
			}
			command[which][0] = serialdev[which]->read();
#ifdef DEBUG_SERIAL
			debug("received on %d: %x", which, command[which][0]);
#endif
			if (which == 1) {
				had_data = true;
				if (doing_debug) {
					if (command[which][0] == 0) {
						END_DEBUG();
						doing_debug = false;
						continue;
					}
					DO_DEBUG(command[which][0]);
					continue;
				}
				if (need_id) {
					the_id[ID_SIZE - need_id] = command[which][0];
					need_id -= 1;
					if (!need_id) {
						// Firmware has reset.
						//arch_reset();
						debug("firmware has reset");
					}
					continue;
				}
				// If this is a 1-byte command, handle it.
				switch (command[which][0])
				{
				case CMD_DEBUG:
					if (which == 0) {
						debug("wtf?");
						reset();
					}
					doing_debug = true;
					START_DEBUG();
					continue;
				case CMD_ACK0:
				case CMD_ACK1:
					// Ack: everything was ok; flip the flipflop.
					if (out_busy && ((!ff_out) ^ (command[which][0] == CMD_ACK1))) {	// Only if we expected it and it is the right type.
						ff_out ^= 0x10;
#ifdef DEBUG_FF
						debug("new ff_out: %d", ff_out[which]);
#endif
						out_busy = false;
						if (free_fragments > 0 && moving && !stopping)
							buffer_refill();
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
					// Response to host is to send the printer id; from printer this is handled after receiving the id.
					if (which == 0) {
						debug("ID request from host");
						continue;
					}
					need_id = ID_SIZE;
					continue;
				default:
					break;
				}
				if ((command[1][0] & 0xe0) != 0x60) {
					// These lengths are not allowed; this cannot be a good packet.
					debug("invalid firmware command code %02x", command[1][0]);
					serialdev[which]->write(CMD_NACK);
					continue;
				}
				last_micros = utime();
			}
			else {
				// Message received.
				if (command[which][0] == OK) {
					if (sending_to_host) {
						sending_to_host = false;
						//debug("received OK; sending next to host (if any)");
						if (stopping == 1) {
							debug("done stopping");
							stopping = 0;
						}
						if (hostqueue_head)
							send_to_host();
					}
					else
						debug("received unexpected OK");
					continue;
				}
			}
			command_end[which] = 1;
		}
		int len = serialdev[which]->available();
		if (len == 0)
		{
#ifdef DEBUG_SERIAL
			//debug("no more data available on %d now", which);
#endif
			// If an out packet is waiting for ACK for too long, assume it didn't arrive and resend it.
			if (which == 1 && out_busy && utime() - out_time >= 200000) {
#ifdef DEBUG_SERIAL
				debug("resending packet on %d", which);
#endif
				send_packet();
			}
			return;
		}
		if (which == 1)
			had_data = true;
		if (len + command_end[which] > COMMAND_SIZE)
			len = COMMAND_SIZE - command_end[which];
		uint8_t cmd_len;
		if (which == 1) {
			cmd_len = hwpacketsize(command_end[which], &len);
			cmd_len += (cmd_len + 2) / 3;
		}
		else
			cmd_len = command[which][0];
		if (command_end[which] + len > cmd_len)
			len = cmd_len - command_end[which];
		serialdev[which]->readBytes(reinterpret_cast <char *> (&command[which][command_end[which]]), len);
#ifdef DEBUG_SERIAL
		debug("read %d bytes on %d:", len, which);
		for (uint8_t i = 0; i < len; ++i)
			fprintf(stderr, " %02x", command[which][command_end[which] + i]);
		fprintf(stderr, "\n");
#endif
		if (which == 1)
			last_micros = utime();
		command_end[which] += len;
		if (command_end[which] < cmd_len)
		{
#ifdef DEBUG_SERIAL
			debug("%d not done yet; %d of %d received.", which, command_end[which], cmd_len);
#endif
			return;
		}
#ifdef DEBUG_DATA
		if (which == 1) {
			fprintf(stderr, "recv %d:", which);
			for (uint8_t i = 0; i < command_end[which]; ++i)
				fprintf(stderr, " %02x", command[which][i]);
			fprintf(stderr, "\n");
		}
#endif
		int end = command_end[which];
		command_end[which] = 0;
		// Check packet integrity.
		if (which == 1) {
			// Checksum must be good.
			len = hwpacketsize(end, NULL);
			for (uint8_t t = 0; t < (len + 2) / 3; ++t)
			{
				uint8_t sum = command[which][len + t];
				if (len == 2 || (len == 4 && t == 1))
					command[which][len + t] = 0;
				if ((sum & 0x7) != (t & 0x7))
				{
					debug("incorrect extra bit, size = %d t = %d", len, t);
					serialdev[which]->write(CMD_NACK);
					return;
				}
				for (uint8_t bit = 0; bit < 5; ++bit)
				{
					uint8_t check = sum & MASK[bit][3];
					for (uint8_t p = 0; p < 3; ++p)
						check ^= command[which][3 * t + p] & MASK[bit][p];
					check ^= check >> 4;
					check ^= check >> 2;
					check ^= check >> 1;
					if (check & 1)
					{
						debug("incorrect checksum byte %d bit %d", t, bit);
						exit(0);
			//fprintf(stderr, "err %d (%d %d):", which, len, t);
			//for (uint8_t i = 0; i < len + (len + 2) / 3; ++i)
			//	fprintf(stderr, " %02x", command[which][i]);
			//fprintf(stderr, "\n");
						serialdev[which]->write(CMD_NACK);
						return;
					}
				}
			}
			// Packet is good.
#ifdef DEBUG_SERIAL
			debug("%d good", which);
#endif
			// Flip-flop must have good state.
			if ((command[which][0] & 0x10) != ff_in)
			{
				// Wrong: this must be a retry to send the previous packet, so our ack was lost.
				// Resend the ack, but don't do anything (the action has already been taken).
#ifdef DEBUG_SERIAL
				debug("%d duplicate", which);
#endif
#ifdef DEBUG_FF
				debug("old ff_in: %d", ff_in);
#endif
				if (had_stall)
					serialdev[which]->write(ff_in ? CMD_STALL0 : CMD_STALL1);
				else
					serialdev[which]->write(ff_in ? CMD_ACK0 : CMD_ACK1);
				continue;
			}
			// Right: update flip-flop and send ack.
			ff_in ^= 0x10;
#ifdef DEBUG_FF
			debug("new ff_in: %d", ff_in);
#endif
			// Clear flag for easier parsing.
			command[which][0] &= ~0x10;
			hwpacket(cmd_len);
		}
		else
			packet();
	}
}

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
void prepare_packet(char *the_packet, int size)
{
#ifdef DEBUG_SERIAL
	debug("prepare");
#endif
	if (size >= COMMAND_SIZE)
	{
		debug("packet is too large: %d > %d", size, COMMAND_SIZE);
		return;
	}
	// Set flipflop bit.
	the_packet[0] &= ~0x10;
	the_packet[0] ^= ff_out;
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
			for (uint8_t p = 0; p < 3; ++p)
				check ^= the_packet[3 * t + p] & MASK[bit][p];
			check ^= sum & MASK[bit][3];
			check ^= check >> 4;
			check ^= check >> 2;
			check ^= check >> 1;
			if (check & 1)
				sum ^= 1 << (bit + 3);
		}
		the_packet[size + t] = sum;
	}
	pending_len = size + (size + 2) / 3;
#ifdef DEBUG_DATA
	fprintf(stderr, "prepare %d:", 1);
#endif
	for (uint8_t i = 0; i < pending_len; ++i) {
		pending_packet[i] = the_packet[i];
#ifdef DEBUG_DATA
		fprintf(stderr, " %02x", int(uint8_t(pending_packet[i])));
#endif
	}
#ifdef DEBUG_DATA
	fprintf(stderr, "\n");
#endif
}

// Send packet to firmware.
void send_packet()
{
#ifdef DEBUG_SERIAL
	fprintf(stderr, "send %d:", 1);
	for (uint8_t i = 0; i < pending_len; ++i)
		fprintf(stderr, " %02x", int(uint8_t(pending_packet[i])));
	fprintf(stderr, "\n");
#endif
	for (uint8_t t = 0; t < pending_len; ++t)
		serialdev[1]->write(pending_packet[t]);
	out_busy = true;
	out_time = utime();
}

void write_ack()
{
	had_stall = false;
	serialdev[1]->write(ff_in ? CMD_ACK0 : CMD_ACK1);
}

void write_stall()
{
	had_stall = true;
	serialdev[1]->write(ff_in ? CMD_STALL0 : CMD_STALL1);
}

void send_host(char cmd, int s, int m, float f, int e, int len)
{
	//debug("queueing for host cmd %x", cmd);
	// Use malloc, not mem_alloc, because there are multiple pointers to the same memory and mem_alloc cannot handle that.
	Queuerecord *record = reinterpret_cast <Queuerecord *>(malloc(sizeof(Queuerecord) + len));
	if (hostqueue_head)
		hostqueue_tail->next = record;
	else
		hostqueue_head = record;
	hostqueue_tail = record;
	record->r.s = s;
	record->r.m = m;
	record->r.f = f;
	record->r.e = e;
	record->cmd = cmd;
	record->len = len;
	record->next = NULL;
	for (unsigned i = 0; i < len; ++i)
		reinterpret_cast <char *>(record)[sizeof(Queuerecord) + i] = datastore[i];
	if (!sending_to_host)
		send_to_host();
}

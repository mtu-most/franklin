#include "firmware.hh"

// Commands which can be sent:
// Packet: n*4 bytes, of which n*3 bytes are data.
// Ack: 1 byte.
// Nack: 1 byte.

// The first byte of a packet is the length: 0lllllll
// The second byte of a packet is the flipflop and the command (fccccccc)
// All other commands have bit 7 set, so they cannot be mistaken for a packet.
// They have 4 bit data and 3 bit parity: 1pppdddd
// Codes (defined in firmware.hh):
// 	0000	-> 0x80	1000
// 	0001	-> 0xe1	1110
// 	0010	-> 0xd2	1101
// 	0011	-> 0xb3	1011
// 	0100	-> 0xf4	1111
// 	0101	-> 0x95	1001
// 	0110	-> 0xa6	1010
// 	0111	-> 0xc7	1100
// static const uint8_t MASK1[3] = {0x4f, 0x2d, 0x1e}

static uint8_t ff_in = 0;
static uint8_t ff_out = 0;
static unsigned long last_micros = 0;
static bool had_data = false;

// Parity masks for decoding.
static const uint8_t MASK[5][4] = {
	{0xc0, 0xc3, 0xff, 0x09},
	{0x38, 0x3a, 0x7e, 0x13},
	{0x26, 0xb5, 0xb9, 0x23},
	{0x95, 0x6c, 0xd5, 0x43},
	{0x4b, 0xdc, 0xe2, 0x83}};

// There may be serial data available.
void serial ()
{
	if (!had_data && command_end > 0 && micros () >= last_micros + 100000)
	{
		// Command not finished; ignore it and wait for next.
		command_end = 0;
	}
	had_data = false;
	while (command_end == 0)
	{
		if (!Serial.available ())
			return;
		had_data = true;
		command[0] = Serial.read ();
		//debug ("received: %x", command[0]);
		// If this is a 1-byte command, handle it.
		switch (command[0])
		{
		case CMD_ACK:
			// Ack: everything was ok; flip the flipflop.
			ff_out ^= 0x80;
			out_busy = false;
			try_send_next ();
			continue;
		case CMD_NACK:
			// Nack: the host didn't properly receive the notification: resend.
			// Unless the last packet was already received; in that case ignore the NACK.
			if (out_busy)
				send_packet (last_packet);
			continue;
		case CMD_RESET:
			// Emergency reset.
			queue_start = 0;
			queue_end = 0;
			for (uint8_t o = 0; o < MAXOBJECT; ++o)
			{
				if (motors[o])
				{
					motors[o]->steps_total = 0;
					motors[o]->continuous = false;
					SET (motors[o]->enable_pin);
				}
				if (temps[o])
				{
					temps[o]->target = NAN;
					if (temps[o]->is_on)
					{
						RESET (temps[o]->power_pin);
						temps[o]->is_on = false;
					}
				}
			}
			Serial.write (CMD_ACKRESET);
			continue;
		default:
			break;
		}
		command_end = 1;
		last_micros = micros ();
	}
	int len = Serial.available ();
	if (len == 0)
	{
		//debug ("no more data available now");
		return;
	}
	had_data = true;
	if (len + command_end > COMMAND_SIZE)
		len = COMMAND_SIZE - command_end;
	uint8_t cmd_len = command[0] & COMMAND_LEN_MASK;
	cmd_len += (cmd_len + 2) / 3;
	if (command_end + len > cmd_len)
		len = cmd_len - command_end;
	Serial.readBytes (reinterpret_cast <char *> (&command[command_end]), len);
	//debug ("read %d bytes", len);
	last_micros = micros ();
	command_end += len;
	if (command_end < cmd_len)
	{
		//debug ("not done yet; %d of %d received.", command_end, cmd_len);
		return;
	}
	command_end = 0;
	// Check packet integrity.
	// Size must be good.
	if (command[0] < 2 || command[0] == 4)
	{
		Serial.write (CMD_NACK);
		return;
	}
	// Checksum must be good.
	len = command[0] & ~0x80;
	for (uint8_t t = 0; t < (len + 2) / 3; ++t)
	{
		uint8_t sum = command[len + t];
		if ((sum & 0x7) != (t & 0x7))
		{
			Serial.write (CMD_NACK);
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
				Serial.write (CMD_NACK);
				return;
			}
		}
	}
	// Packet is good.
	//debug ("good");
	// Flip-flop must have good state.
	if ((command[1] & 0x80) != ff_in)
	{
		// Wrong: this must be a retry to send the previous packet, so our ack was lost.
		// Resend the ack, but don't do anything (the action has already been taken).
		//debug ("duplicate");
		Serial.write (CMD_ACK);
		return;
	}
	// Right: update flip-flop and send ack.
	ff_in ^= 0x80;
	// Clear flag for easier parsing.
	command[1] &= ~0x80;
	packet ();
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
static void prepare_packet (char *the_packet)
{
	//debug ("prepare");
	// Set flipflop bit.
	the_packet[1] &= ~0x80;
	the_packet[1] ^= ff_out;
	// Compute the checksums.  This doesn't work for size in (1, 2, 4), so
	// the protocol doesn't allow those.
	// For size % 3 != 0, the first checksums are part of the data for the
	// last checksum.  This means they must have been filled in at that
	// point.  (This is also the reason (1, 2, 4) are not allowed.)
	uint8_t cmd_len = the_packet[0] & COMMAND_LEN_MASK;
	for (uint8_t t = 0; t < (cmd_len + 2) / 3; ++t)
	{
		uint8_t sum = t & 7;
		for (unsigned bit = 0; bit < 5; ++bit)
		{
			uint8_t check = 0;
			for (uint8_t p = 0; p < 3; ++p)
				check ^= the_packet[3 * t + p] & MASK[bit][p];
			check ^= sum & MASK[bit][3];
			check ^= check >> 4;
			check ^= check >> 2;
			check ^= check >> 1;
			if (check & 1)
				sum |= 1 << (bit + 3);
		}
		the_packet[cmd_len + t] = sum;
	}
}

// Send packet to host.
void send_packet (char *the_packet)
{
	//debug ("send");
	last_packet = the_packet;
	uint8_t cmd_len = the_packet[0] & COMMAND_LEN_MASK;
	for (uint8_t t = 0; t < cmd_len + (cmd_len + 2) / 3; ++t)
		Serial.write (the_packet[t]);
	out_busy = true;
}

// Call send_packet if we can.
void try_send_next ()
{
	//debug ("try send");
	if (out_busy)
	{
		//debug ("still busy");
		// Still busy sending other packet.
		return;
	}
	if (have_msg)
	{
		prepare_packet (msg_buffer);
		send_packet (msg_buffer);
		have_msg = false;
		return;
	}
	for (uint8_t w = 0; w < MAXAXES; ++w)
	{
		if (limits_pos[w] != MAXLONG)
		{
			//debug ("limit %d", w);
			limitcb_buffer[2] = w;
			ReadFloat f;
			f.l = limits_pos[w];
			limitcb_buffer[3] = f.b[0];
			limitcb_buffer[4] = f.b[1];
			limitcb_buffer[5] = f.b[2];
			limitcb_buffer[6] = f.b[3];
			limits_pos[w] = MAXLONG;
			prepare_packet (limitcb_buffer);
			send_packet (limitcb_buffer);
			return;
		}
	}
	if (num_movecbs > 0)
	{
		//debug ("movecb %d", num_movecbs);
		movecb_buffer[2] = num_movecbs;
		prepare_packet (movecb_buffer);
		send_packet (movecb_buffer);
		num_movecbs = 0;
		return;
	}
	if (which_tempcbs != 0)
	{
		//debug ("tempcb %d", which_tempcbs);
		for (uint8_t w = 0; w < EXTRUDER0 + num_extruders; ++w)
		{
			if (which_tempcbs & (1 << w))
			{
				tempcb_buffer[2] = w;
				which_tempcbs &= ~(1 << w);
				break;
			}
		}
		prepare_packet (tempcb_buffer);
		send_packet (tempcb_buffer);
		return;
	}
	if (reply_ready)
	{
		//debug ("reply %x %d", reply[1], reply[0]);
		prepare_packet (reply);
		send_packet (reply);
		reply_ready = false;
		return;
	}
	if (continue_cb)
	{
		//debug ("continue");
		prepare_packet (continue_buffer);
		send_packet (continue_buffer);
		continue_cb = false;
		return;
	}
	//debug ("nothing to send?!");
}

#include "firmware.hh"

static uint8_t get_which ()
{
	return command[2] & 0x3f;
}

static float get_float (uint8_t offset)
{
	ReadFloat ret;
	for (uint8_t t = 0; t < sizeof (float); ++t)
		ret.b[t] = command[offset + t];
	return ret.f;
}

void packet ()
{
	// command[0] is the length not including checksum bytes.
	// command[1] is the command.
	uint8_t which;
	uint16_t addr;
	switch (command[1])
	{
	case CMD_BEGIN:	// begin: request response
	{
		Serial.write (CMD_ACK);
		reply[0] = 6;
		reply[1] = CMD_START;
		reply[2] = 0;
		reply[3] = 0;
		reply[4] = 0;
		reply[5] = 0;
		reply_ready = true;
		try_send_next ();
		return;
	}
	case CMD_GOTO:	// goto
	case CMD_GOTOCB:	// goto with callback
	{
		if (((queue_end + 1) & QUEUE_LENGTH_MASK) == queue_start)
		{
			Serial.write (CMD_STALL);
			return;
		}
		uint8_t const num = FLAG_EXTRUDER0 + num_extruders;
		uint8_t const offset = 2 + ((num - 1) >> 3) + 1;
		uint8_t t = 0;
		for (uint8_t ch = 0; ch < num; ++ch)
		{
			if (command[2 + (ch >> 3)] & (1 << (ch & 0x7)))
			{
				ReadFloat f;
				for (uint8_t i = 0; i < sizeof (float); ++i)
					f.b[i] = command[offset + i + t * sizeof (float)];
				queue[queue_end].data[ch] = f.f;
				++t;
			}
			else
				queue[queue_end].data[ch] = NAN;
		}
		// Set cb in next record, because it will be read when queue_start has already been incremented.
		queue[(queue_end + 1) & QUEUE_LENGTH_MASK].cb = command[1] == CMD_GOTOCB;
		queue_end = (queue_end + 1) & QUEUE_LENGTH_MASK;
		if (motors_busy == 0)
			next_move ();
		break;
	}
	case CMD_RUN:	// run motor
	{
		which = get_which ();
		if (!motors[which])
		{
			Serial.write (CMD_STALL);
			return;
		}
		if (command[2] & 0x80)
		{
			motors[which]->positive = command[2] & 0x40;
			digitalWrite (motors[which]->dir_pin, motors[which]->positive ? HIGH : LOW);
			digitalWrite (motors[which]->sleep_pin, HIGH);
			motors[which]->continuous = true;
		}
		else
			motors[which]->continuous = false;
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_SLEEP:	// disable motor current
	{
		which = get_which ();
		if (!motors[which])
		{
			Serial.write (CMD_STALL);
			return;
		}
		digitalWrite (motors[which]->sleep_pin, command[2] & 0x80 ? LOW : HIGH);
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_SETTEMP:	// set target temperature and enable control
	{
		which = get_which ();
		if (!temps[which])
		{
			Serial.write (CMD_STALL);
			return;
		}
		temps[which]->target = get_float (3);
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_WAITTEMP:	// wait for a temperature sensor to reach a target range
	{
		uint8_t const num = FLAG_EXTRUDER0 + num_extruders;
		uint8_t ch = command[2];
		if (ch >= num || !temps[ch])
		{
			Serial.write (CMD_STALL);
			return;
		}
		ReadFloat min, max;
		for (uint8_t i = 0; i < sizeof (float); ++i)
		{
			min.b[i] = command[3 + i];
			max.b[i] = command[3 + i + sizeof (float)];
		}
		temps[ch]->min_alarm = min.f;
		temps[ch]->max_alarm = max.f;
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_READTEMP:	// read temperature
	{
		which = get_which ();
		if (!temps[which])
		{
			Serial.write (CMD_STALL);
			return;
		}
		Serial.write (CMD_ACK);
		return;
		ReadFloat f;
		f.f = temps[which]->read ();
		reply[0] = 2 + sizeof (float);
		reply[1] = CMD_TEMP;
		for (uint8_t b = 0; b < sizeof (float); ++b)
			reply[2 + b] = f.b[b];
		reply_ready = true;
		try_send_next ();
		return;
	}
	case CMD_LOAD:	// reload settings from eeprom
	{
		which = get_which ();
		if (!objects[which])
		{
			Serial.write (CMD_STALL);
			return;
		}
		addr = objects[which]->address;
		objects[which]->load (addr, true);
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_SAVE:	// save settings to eeprom
	{
		which = get_which ();
		if (!objects[which])
		{
			Serial.write (CMD_STALL);
			return;
		}
		addr = objects[which]->address;
		objects[which]->save (addr, true);
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_READ:	// reply settings to host
	{
		which = get_which ();
		if (!objects[which])
		{
			Serial.write (CMD_STALL);
			return;
		}
		addr = 2;
		if (which == FLAG_BED)
			bed_save (addr, false);
		else
			objects[which]->save (addr, false);
		reply[0] = addr;
		reply[1] = CMD_DATA;
		Serial.write (CMD_ACK);
		reply_ready = true;
		try_send_next ();
		return;
	}
	case CMD_WRITE:	// change settings from host
	{
		which = get_which ();
		if (!objects[which])
		{
			Serial.write (CMD_STALL);
			return;
		}
		addr = 3;
		if (which == FLAG_BED)
			bed_load (addr, false);
		else
			objects[which]->load (addr, false);
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_PAUSE:
	{
		pause_all = command[2] != 0;
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_PING:
	{
		reply[0] = 3;
		reply[1] = CMD_PONG;
		reply[2] = command[2];
		reply_ready = true;
		try_send_next ();
	}
	default:
	{
		Serial.write (CMD_STALL);
		return;
	}
	}
}

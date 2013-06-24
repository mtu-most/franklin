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
	case BEGIN:	// begin: request response
		Serial.write (ACK);
		outcommand[0] = 5;
		outcommand[1] = 0;
		outcommand[2] = 0;
		outcommand[3] = 0;
		outcommand[4] = 0;
		send_packet ();
		return;
	case GOTO:	// goto
	case GOTOCB:	// goto with callback
		// TODO: implement goto
		if (motors_busy == 0)
			next_move ();
		break;
	case RUN:	// run motor
		which = get_which ();
		if (!motors[which])
		{
			Serial.write (STALL);
			return;
		}
		if (command[2] & 0x80)
		{
			motors[which]->positive = command[2] & 0x40;
			digitalWrite (motors[which]->dir_pin, motors[which]->positive ? HIGH : LOW);
			digitalWrite (motors[which]->sleep_pin, LOW);
			motors[which]->continuous = true;
		}
		else
			motors[which]->continuous = false;
		Serial.write (ACK);
		return;
	case SLEEP:	// disable motor current
		which = get_which ();
		if (!motors[which])
		{
			Serial.write (STALL);
			return;
		}
		digitalWrite (motors[which]->sleep_pin, command[2] & 0x80 ? HIGH : LOW);
		Serial.write (ACK);
		return;
	case SETTEMP:	// set target temperature and enable control
		which = get_which ();
		if (!temps[which])
		{
			Serial.write (STALL);
			return;
		}
		temps[which]->target = get_float (3);
		Serial.write (ACK);
		return;
	case WAITTEMP:	// wait for one or more temperature sensors to reach their target
		// TODO: implement waittemp.
		Serial.write (PAUSE);
		return;
	case READTEMP:	// read temperature
		which = get_which ();
		if (!temps[which])
		{
			Serial.write (STALL);
			return;
		}
		Serial.write (ACK);
		return;
		ReadFloat f;
		f.f = temps[which]->read ();
		outcommand[0] = 1 + sizeof (float);
		for (uint8_t b = 0; b < sizeof (float); ++b)
			outcommand[1 + b] = f.b[b];
		send_packet ();
		return;
	case LOAD:	// reload settings from eeprom
		which = get_which ();
		if (!objects[which])
		{
			Serial.write (STALL);
			return;
		}
		addr = objects[which]->address;
		objects[which]->load (addr, true);
		Serial.write (ACK);
		return;
	case SAVE:	// save settings to eeprom
		which = get_which ();
		if (!objects[which])
		{
			Serial.write (STALL);
			return;
		}
		addr = objects[which]->address;
		objects[which]->save (addr, true);
		Serial.write (ACK);
		return;
	case READ:	// reply settings to host
		which = get_which ();
		if (!objects[which])
		{
			Serial.write (STALL);
			return;
		}
		addr = 3;
		objects[which]->load (addr, false);
		Serial.write (ACK);
		return;
	case WRITE:	// change settings from host
		which = get_which ();
		if (!objects[which])
		{
			Serial.write (STALL);
			return;
		}
		addr = 3;
		objects[which]->save (addr, false);
		Serial.write (ACK);
		return;
	default:
		Serial.write (STALL);
		return;
	}
}

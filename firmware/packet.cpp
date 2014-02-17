#include "firmware.h"

//#define DEBUG_CMD

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

static uint16_t get_uint16 (uint8_t offset)
{
	return ((uint16_t)command[offset] & 0xff) | (uint16_t)command[offset + 1] << 8;
}

void packet ()
{
	// command[0] is the length not including checksum bytes.
	// command[1] is the command.
	uint8_t which;
	int16_t addr;
	switch (command[1])
	{
	case CMD_BEGIN:	// begin: request response
	{
#ifdef DEBUG_CMD
		debug ("CMD_BEGIN");
#endif
		// A server is running; start the watchdog.
		wdt_reset ();
		wdt_enable (WDTO_120MS);
		for (uint8_t i = 0; i < ID_SIZE; ++i)
			printerid[i] = command[6 + i];
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
#ifdef DEBUG_CMD
		debug ("CMD_GOTO(CB)");
#endif
		if (!pause_all && ((queue_end + 1) % QUEUE_LENGTH) == queue_start)
		{
			debug ("Host ignores wait request");
			Serial.write (CMD_STALL);
			return;
		}
		uint8_t const num = TEMP0;
		uint8_t const offset = 2 + ((num - 1) >> 3) + 1;	// Bytes from start of command where values are.
		uint8_t t = 0;
		for (uint8_t ch = 0; ch < num; ++ch)
		{
			if (command[2 + (ch >> 3)] & (1 << (ch & 0x7)))
			{
				ReadFloat f;
				for (uint8_t i = 0; i < sizeof (float); ++i)
					f.b[i] = command[offset + i + t * sizeof (float)];
				queue[queue_end].data[ch] = f.f;
				//debug ("goto %d %f", ch, &f.f);
				++t;
			}
			else
				queue[queue_end].data[ch] = NAN;
		}
		// f0 and f1 must be present and valid.
		float f0 = queue[queue_end].data[F0];
		float f1 = queue[queue_end].data[F1];
		if (isnan (f0) || isnan (f1) || f0 < 0 || f1 < 0 || (f0 == 0 && f1 == 0))
		{
			debug ("Invalid f0 or f1: %f %f", &f0, &f1);
			Serial.write (CMD_STALL);
			return;
		}
		// If a new goto is sent while paused; we discard the current buffer.
		if (pause_all) {
			pause_all = false;
			abort_move ();
			audio_start += micros () - pause_time;
		}
		queue[queue_end].cb = command[1] == CMD_GOTOCB;
		queue_end = (queue_end + 1) % QUEUE_LENGTH;
		if (((queue_end + 1) % QUEUE_LENGTH) == queue_start)
			Serial.write (CMD_ACKWAIT);
		else
			Serial.write (CMD_ACK);
		if (!moving)
			next_move ();
		//else
			//debug ("waiting with move");
		break;
	}
	case CMD_RUN:	// run motor
	{
#ifdef DEBUG_CMD
		debug ("CMD_RUN");
#endif
		which = get_which ();
		ReadFloat f;
		for (uint8_t i = 0; i < sizeof (float); ++i) {
			f.b[i] = command[3 + i];
		}
		// Motor must exist, must not be doing a move.
		if (!motors[which])
		{
			debug ("Running invalid motor %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		// If a run is sent while paused; we discard the current buffer.
		if (pause_all) {
			pause_all = false;
			abort_move ();
			audio_start += micros () - pause_time;
		}
		if (moving)
		{
			if (((printer_type != 1 || which > 2 + 3) && !isnan (motors[which]->dist)) || (printer_type == 1 && which <= 2 + 3 && (!isnan (motors[2]->dist) || !isnan (motors[3]->dist) || !isnan (motors[4]->dist))))
			{
				debug ("Running moving motor %d", which);
				Serial.write (CMD_STALL);
				return;
			}
		}
		Serial.write (CMD_ACK);
		if (which < 2 + MAXAXES) {
			for (uint8_t a = 0; a < 3; ++a)
				axis[a].source = NAN;
		}
		float speed = isnan (f.f) ? 0 : f.f;
		if (speed > 0) {
			if (speed > motors[which]->max_v_pos)
				speed = motors[which]->max_v_pos;
		}
		else if (speed < 0) {
			if (speed < -motors[which]->max_v_neg)
				speed = -motors[which]->max_v_neg;
		}
		if (motors[which]->f != 0) {
		       	if ((motors[which]->positive && speed < 0) || (!motors[which]->positive && speed > 0))
				motors[which]->continuous_steps_per_s = -abs (speed) * motors[which]->steps_per_mm;
			else
				motors[which]->continuous_steps_per_s = abs (speed) * motors[which]->steps_per_mm;
		}
		else {
			if (speed > 0) {
				SET (motors[which]->dir_pin);
				motors[which]->continuous_steps_per_s = speed * motors[which]->steps_per_mm;
				motors[which]->positive = true;
			}
			else {
				RESET (motors[which]->dir_pin);
				motors[which]->continuous_steps_per_s = -speed * motors[which]->steps_per_mm;
				motors[which]->positive = false;
			}
			//debug ("initial positive %d", motors[which]->positive);
			motors[which]->continuous_last_time = micros ();
			SET (motors[which]->enable_pin);
			motors[which]->continuous_steps = 0;
			motors_busy = true;
		}
		return;
	}
	case CMD_SLEEP:	// disable motor current
	{
#ifdef DEBUG_CMD
		debug ("CMD_SLEEP");
#endif
		which = get_which ();
		if (!motors[which])
		{
			debug ("Sleeping invalid motor %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		if (command[2] & 0x80)
			RESET (motors[which]->enable_pin);
		else {
			SET (motors[which]->enable_pin);
			motors_busy = true;
		}
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_SETTEMP:	// set target temperature and enable control
	{
#ifdef DEBUG_CMD
		debug ("CMD_SETTEMP");
#endif
		which = get_which ();
		float target = get_float (3);
		if (!temps[which] || (temps[which]->thermistor_pin.invalid () && (target < 0 || !isinf (target)) && !isnan (target)))
		{
			debug ("Setting invalid temp %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		temps[which]->target = target + 273.15;
		if (isnan (temps[which]->target)) {
			// loop () doesn't handle it anymore, so it isn't disabled there.
			if (temps[which]->is_on) {
				RESET (temps[which]->power_pin);
				temps[which]->is_on = false;
				--temps_busy;
			}
			last_active = millis ();
		}
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_WAITTEMP:	// wait for a temperature sensor to reach a target range
	{
#ifdef DEBUG_CMD
		debug ("CMD_WAITTEMP");
#endif
		which = get_which ();
		if (!temps[which])
		{
			debug ("Waiting for invalid temp %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		ReadFloat min, max;
		for (uint8_t i = 0; i < sizeof (float); ++i)
		{
			min.b[i] = command[3 + i];
			max.b[i] = command[3 + i + sizeof (float)];
		}
		temps[which]->min_alarm = min.f + 273.15;
		temps[which]->max_alarm = max.f + 273.15;
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_READTEMP:	// read temperature
	{
#ifdef DEBUG_CMD
		debug ("CMD_READTEMP");
#endif
		which = get_which ();
		if (!temps[which])
		{
			debug ("Reading invalid temp %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		Serial.write (CMD_ACK);
		ReadFloat f;
		f.f = temps[which]->read () - 273.15;
		//debug ("read temp %f", f.f);
		reply[0] = 2 + sizeof (float);
		reply[1] = CMD_TEMP;
		for (uint8_t b = 0; b < sizeof (float); ++b)
			reply[2 + b] = f.b[b];
		reply_ready = true;
		try_send_next ();
		return;
	}
	case CMD_READPOWER:	// read used power
	{
#ifdef DEBUG_CMD
		debug ("CMD_READPOWER");
#endif
		which = get_which ();
		if (!temps[which])
		{
			debug ("Reading power of invalid temp %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		Serial.write (CMD_ACK);
		reply[0] = 10;
		reply[1] = CMD_POWER;
		ReadFloat on, current;
		unsigned long t = micros ();
		if (temps[which]->is_on) {
			// This causes an insignificant error in the model, but when using this you probably aren't using the model anyway, and besides you won't notice the error even if you do.
			temps[which]->time_on += t - temps[which]->last_time;
			temps[which]->last_time = t;
		}
		on.u = temps[which]->time_on;
		current.u = t;
		for (uint8_t b = 0; b < sizeof (unsigned long); ++b)
		{
			reply[2 + b] = on.b[b];
			reply[6 + b] = current.b[b];
		}
	}
	case CMD_SETPOS:	// Set current position
	{
#ifdef DEBUG_CMD
		debug ("CMD_SETPOS");
#endif
		which = get_which ();
		if (which < 2 || which > 2 + MAXAXES)
		{
			debug ("Setting position of invalid axis %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		for (uint8_t a = 0; a < 3; ++a)
			axis[a].source = NAN;
		Serial.write (CMD_ACK);
		axis[which - 2].current_pos = int32_t (get_float (3) * axis[which - 2].motor.steps_per_mm);
		return;
	}
	case CMD_GETPOS:	// Get current position
	{
#ifdef DEBUG_CMD
		debug ("CMD_GETPOS");
#endif
		which = get_which ();
		if (which < 2 || which > 2 + MAXAXES)
		{
			debug ("Getting position of invalid axis %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		Serial.write (CMD_ACK);
		if (isnan (axis[0].source))
			reset_pos ();
		ReadFloat pos, current;
		pos.f = axis[which - 2].current_pos / axis[which - 2].motor.steps_per_mm;
		current.f = axis[which - 2].current;
		reply[0] = 2 + sizeof (int32_t) + sizeof (float);
		reply[1] = CMD_POS;
		for (uint8_t b = 0; b < sizeof (int32_t); ++b)
			reply[2 + b] = pos.b[b];
		for (uint8_t b = 0; b < sizeof (float); ++b)
			reply[2 + sizeof (int32_t) + b] = current.b[b];
		reply_ready = true;
		try_send_next ();
		return;
	}
	case CMD_LOAD:	// reload settings from eeprom
	{
#ifdef DEBUG_CMD
		debug ("CMD_LOAD");
#endif
		which = get_which ();
		if (which < 1 || which >= MAXOBJECT)
		{
			debug ("Loading invalid object %d", which);
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
#ifdef DEBUG_CMD
		debug ("CMD_SAVE");
#endif
		which = get_which ();
		if (which < 1 || which >= MAXOBJECT)
		{
			debug ("Saving invalid object %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		if (moving)
		{
			debug ("Saving %d while moving would disrupt move", which);
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
#ifdef DEBUG_CMD
		debug ("CMD_READ");
#endif
		which = get_which ();
		if (which < 0 || which >= MAXOBJECT)
		{
			debug ("Reading invalid object %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		addr = 2;
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
#ifdef DEBUG_CMD
		debug ("CMD_WRITE %d", which);
#endif
		if (which < 1 || which >= MAXOBJECT)
		{
			debug ("Writing invalid object %d", which);
			Serial.write (CMD_STALL);
			return;
		}
		addr = 3;
		objects[which]->load (addr, false);
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_PAUSE:
	{
#ifdef DEBUG_CMD
		debug ("CMD_PAUSE");
#endif
		if (command[2]) {
			if (!pause_all)
				pause_time = micros ();
		}
		else {
			if (pause_all) {
				unsigned long diff = micros () - pause_time;
				start_time += diff;
				audio_start += diff;
				for (uint8_t m = 0; m < MAXOBJECT; ++m) {
					if (!motors[m])
						continue;
					motors[m]->continuous_last_time += diff;
				}
			}
		}
		pause_all = command[2] != 0;
		Serial.write (CMD_ACK);
		return;
	}
	case CMD_PING:
	{
#ifdef DEBUG_CMD
		debug ("CMD_PING");
#endif
		Serial.write (CMD_ACK);
		ping |= 1 << command[2];
		try_send_next ();
		return;
	}
	case CMD_READPIN:
	{
#ifdef DEBUG_CMD
		debug ("CMD_READPIN");
#endif
		if (command[2] >= NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS)
		{
			debug ("Readpin can only handle %d pins; not %d", NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS, command[2]);
			Serial.write (CMD_STALL);
			return;
		}
		Serial.write (CMD_ACK);
		reply[0] = 3;
		reply[1] = CMD_PIN;
		reply[2] = digitalRead (command[2]) ? 1 : 0;
		reply_ready = true;
		try_send_next ();
		return;
	}
	case CMD_AUDIO_SETUP:
	{
#ifdef DEBUG_CMD
		debug ("CMD_AUDIO_SETUP");
#endif
		audio_us_per_bit = get_uint16 (2);
		for (uint8_t a = 0; a < num_axes; ++a)
		{
			if (command[4 + ((a + 2) >> 3)] & (1 << ((a + 2) & 0x7))) {
				axis[a].motor.audio_flags |= Motor::PLAYING;
				SET (axis[a].motor.enable_pin);
				motors_busy = true;
			}
			else
				axis[a].motor.audio_flags &= ~Motor::PLAYING;
		}
		for (uint8_t e = 0; e < num_extruders; ++e)
		{
			if (command[4 + ((e + 2 + num_axes) >> 3)] & (1 << ((e + 2 + num_axes) & 0x7))) {
				extruder[e].motor.audio_flags |= Motor::PLAYING;
				SET (extruder[e].motor.enable_pin);
				motors_busy = true;
			}
			else
				extruder[e].motor.audio_flags &= ~Motor::PLAYING;
		}
		// Abort any currently playing sample.
		audio_head = 0;
		audio_tail = 0;
		Serial.write (CMD_ACK);
		last_active = millis ();
		return;
	}
	case CMD_AUDIO_DATA:
	{
#ifdef DEBUG_CMD
		debug ("CMD_AUDIO_DATA");
#endif
		if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
		{
			debug ("Audio buffer is full");
			Serial.write (CMD_STALL);
			return;
		}
		for (uint8_t i = 0; i < AUDIO_FRAGMENT_SIZE; ++i)
			audio_buffer[audio_tail][i] = command[2 + i];
		if (audio_tail == audio_head)
			audio_start = micros ();
		audio_tail = (audio_tail + 1) % AUDIO_FRAGMENTS;
		if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
			Serial.write (CMD_ACKWAIT);
		else
			Serial.write (CMD_ACK);
		last_active = millis ();
		return;
	}
	default:
	{
		debug ("Invalid command %x %x %x %x", command[0], command[1], command[2], command[3]);
		Serial.write (CMD_STALL);
		return;
	}
	}
}

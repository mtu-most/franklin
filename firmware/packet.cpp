#include "firmware.h"

//#define DEBUG_CMD

static uint8_t get_which()
{
	return command[2] & 0x3f;
}

#if defined(HAVE_TEMPS) || defined(HAVE_SPACES)
static float get_float(uint8_t offset)
{
	ReadFloat ret;
	for (uint8_t t = 0; t < sizeof(float); ++t)
		ret.b[t] = command[offset + t];
	return ret.f;
}
#endif

#if defined(HAVE_AUDIO)
static int16_t get_int16(uint8_t offset)
{
	return ((uint16_t)command[offset] & 0xff) | (uint16_t)command[offset + 1] << 8;
}
#endif

void packet()
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
		debug("CMD_BEGIN");
#endif
		// A server is running; start the watchdog.
		watchdog_enable();
		for (uint8_t i = 0; i < 4; ++i) {
			if (command[2 + i] != 0) {
				debug("Server version is not supported");
				Serial.write(CMD_STALL);
				return;
			}
		}
		for (uint8_t i = 0; i < ID_SIZE; ++i)
			printerid[i] = command[6 + i];
		write_ack();
		reply[0] = 6;
		reply[1] = CMD_START;
		reply[2] = 0;
		reply[3] = 0;
		reply[4] = 0;
		reply[5] = 0;
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_RESET: // reset controller; used before reprogramming flash.
	{
#ifdef DEBUG_CMD
		debug("CMD_RESET");
#endif
		write_ack();
		Serial.flush();
		reset();
	}
#ifdef HAVE_SPACES
	case CMD_GOTO:	// goto
	case CMD_GOTOCB:	// goto with callback
	{
#ifdef DEBUG_CMD
		debug("CMD_GOTO(CB)");
#endif
		last_active = millis();
		if (queue_full)
		{
			debug("Host ignores wait request");
			Serial.write(CMD_STALL);
			return;
		}
		uint8_t num = 2;
		for (uint8_t t = 0; t < num_spaces; ++t)
			num += spaces[t].num_axes;
		uint8_t const offset = 2 + ((num - 1) >> 3) + 1;	// Bytes from start of command where values are.
		uint8_t t = 0;
		for (uint8_t ch = 0; ch < num; ++ch)
		{
			if (command[2 + (ch >> 3)] & (1 << (ch & 0x7)))
			{
				ReadFloat f;
				for (uint8_t i = 0; i < sizeof(float); ++i)
					f.b[i] = command[offset + i + t * sizeof(float)];
				if (ch < 2) {
					queue[queue_end].f[ch] = f.f;
					//debug("goto %d %f", ch, F(f.f));
				}
				else {
					queue[queue_end].data[ch - 2] = f.f;
					//debug("goto %d %f", ch, F(f.f));
				}
				initialized = true;
				++t;
			}
			else {
				if (ch < 2)
					queue[queue_end].f[ch] = NAN;
				else
					queue[queue_end].data[ch - 2] = NAN;
				//debug("goto %d -", ch);
			}
		}
		if (!(command[2] & 0x1) || isnan(queue[queue_end].f[0]))
			queue[queue_end].f[0] = INFINITY;
		if (!(command[2] & 0x2) || isnan(queue[queue_end].f[1]))
			queue[queue_end].f[1] = queue[queue_end].f[0];
		// f0 and f1 must be valid.
		float f0 = queue[queue_end].f[0];
		float f1 = queue[queue_end].f[1];
		if (isnan(f0) || isnan(f1) || f0 < 0 || f1 < 0 || (f0 == 0 && f1 == 0))
		{
			debug("Invalid f0 or f1: %f %f", F(f0), F(f1));
			Serial.write(CMD_STALL);
			return;
		}
		queue[queue_end].cb = command[1] == CMD_GOTOCB;
		queue_end = (queue_end + 1) % QUEUE_LENGTH;
		if (queue_end == queue_start) {
			queue_full = true;
			write_ackwait();
		}
		else
			write_ack();
		if (!moving)
			next_move();
		//else
			//debug("waiting with move");
		break;
	}
	case CMD_SLEEP:	// disable motor current
	{
#ifdef DEBUG_CMD
		debug("CMD_SLEEP");
#endif
		last_active = millis();
		if (moving)
		{
			debug("Sleeping while moving");
			Serial.write(CMD_STALL);
			return;
		}
		if (command[2]) {
			for (uint8_t t = 0; t < num_spaces; ++t) {
				for (uint8_t m = 0; m < spaces[t].num_motors; ++m) {
					RESET(spaces[t].motor[m]->enable_pin);
					spaces[t].motor[m]->current_pos = NAN;
				}
				for (uint8_t a = 0; a < spaces[t].num_axes; ++a) {
					spaces[t].axis[a]->source = NAN;
					spaces[t].axis[a]->current = NAN;
				}
			}
			motors_busy = false;
		}
		else {
			for (uint8_t t = 0; t < num_spaces; ++t) {
				for (uint8_t m = 0; m < spaces[t].num_motors; ++m)
					SET(spaces[t].motor[m]->enable_pin);
			}
			motors_busy = true;
		}
		write_ack();
		return;
	}
#endif
#ifdef HAVE_TEMPS
	case CMD_SETTEMP:	// set target temperature and enable control
	{
#ifdef DEBUG_CMD
		debug("CMD_SETTEMP");
#endif
		last_active = millis();
		which = get_which();
		if (which >= num_temps)
		{
			debug("Setting invalid temp %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		temps[which].target = get_float(3);
		temps[which].adctarget = temps[which].toadc(temps[which].target);
		//debug("adc target %d from %d", temps[which]->adctarget, int16_t(temps[which]->target / 1024));
		if (temps[which].adctarget >= MAXINT) {
			// loop() doesn't handle it anymore, so it isn't disabled there.
			//debug("Temp %d disabled", which);
			if (temps[which].is_on) {
				RESET(temps[which].power_pin);
				temps[which].is_on = false;
				--temps_busy;
			}
		}
		else if (temps[which].adctarget < 0) {
			// loop() doesn't handle it anymore, so it isn't enabled there.
			//debug("Temp %d enabled", which);
			if (!temps[which].is_on) {
				SET(temps[which].power_pin);
				temps[which].is_on = true;
				++temps_busy;
			}
		}
		else {
			//debug("Temp %d set to %f", which, F(target));
			initialized = true;
		}
		adc_phase = 1;
		write_ack();
		return;
	}
	case CMD_WAITTEMP:	// wait for a temperature sensor to reach a target range
	{
#ifdef DEBUG_CMD
		debug("CMD_WAITTEMP");
#endif
		initialized = true;
		which = get_which();
		if (which >= num_temps)
		{
			debug("Waiting for invalid temp %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		ReadFloat min, max;
		for (uint8_t i = 0; i < sizeof(float); ++i)
		{
			min.b[i] = command[3 + i];
			max.b[i] = command[3 + i + sizeof(float)];
		}
		temps[which].min_alarm = min.f;
		temps[which].max_alarm = max.f;
		temps[which].adcmin_alarm = temps[which].toadc(temps[which].min_alarm);
		temps[which].adcmax_alarm = temps[which].toadc(temps[which].max_alarm);
		write_ack();
		adc_phase = 1;
		return;
	}
	case CMD_READTEMP:	// read temperature
	{
#ifdef DEBUG_CMD
		debug("CMD_READTEMP");
#endif
		which = get_which();
		if (which >= num_temps)
		{
			debug("Reading invalid temp %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		requested_temp = which;
		adc_phase = 1;
		write_ack();
		return;
	}
	case CMD_READPOWER:	// read used power
	{
#ifdef DEBUG_CMD
		debug("CMD_READPOWER");
#endif
		which = get_which();
		if (which >= num_temps)
		{
			debug("Reading power of invalid temp %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		write_ack();
		reply[0] = 10;
		reply[1] = CMD_POWER;
		ReadFloat on, current;
		unsigned long t = micros();
		if (temps[which].is_on) {
			// This causes an insignificant error in the model, but when using this you probably aren't using the model anyway, and besides you won't notice the error even if you do.
			temps[which].time_on += t - temps[which].last_time;
			temps[which].last_time = t;
		}
		on.ui = temps[which].time_on;
		current.ui = t;
		for (uint8_t b = 0; b < sizeof(unsigned long); ++b)
		{
			reply[2 + b] = on.b[b];
			reply[6 + b] = current.b[b];
		}
	}
#endif
#ifdef HAVE_SPACES
	case CMD_SETPOS:	// Set current position
	{
#ifdef DEBUG_CMD
		debug("CMD_SETPOS");
#endif
		last_active = millis();
		which = get_which();
		uint8_t t = command[3];
		if (which >= num_spaces || t >= spaces[which].num_axes)
		{
			debug("Invalid axis for setting position: %d %d", which, t);
			Serial.write(CMD_STALL);
			return;
		}
		if (!motors_busy)
		{
			debug("Setting position while motors are sleeping");
			Serial.write(CMD_STALL);
			return;
		}
		if (moving)
		{
			debug("Setting position while moving");
			Serial.write(CMD_STALL);
			return;
		}
		for (uint8_t a = 0; a < spaces[which].num_axes; ++a) {
			spaces[which].axis[a]->source = NAN;
			spaces[which].axis[a]->current = NAN;
		}
		write_ack();
		spaces[which].motor[t]->current_pos = get_float(4);
		return;
	}
	case CMD_GETPOS:	// Get current position
	{
#ifdef DEBUG_CMD
		debug("CMD_GETPOS");
#endif
		which = get_which();
		uint8_t t = command[3];
		if (which >= num_spaces || t >= spaces[which].num_axes)
		{
			debug("Getting position of invalid axis %d %d", which, t);
			Serial.write(CMD_STALL);
			return;
		}
		write_ack();
		if (isnan(spaces[which].axis[t]->source))
			space_types[spaces[which].type].reset_pos(&spaces[which]);
			for (uint8_t a = 0; a < spaces[which].num_axes; ++a)
				spaces[which].axis[a]->current = spaces[which].axis[a]->source;
		ReadFloat pos;
		pos.f = spaces[which].axis[t]->current - spaces[which].axis[t]->offset;
		reply[0] = 2 + sizeof(float);
		reply[1] = CMD_POS;
		for (uint8_t b = 0; b < sizeof(float); ++b)
			reply[2 + b] = pos.b[b];
		reply_ready = true;
		try_send_next();
		return;
	}
#endif
	case CMD_LOAD:	// reload settings from eeprom
	{
#ifdef DEBUG_CMD
		debug("CMD_LOAD");
#endif
		load_all();
		write_ack();
		return;
	}
	case CMD_SAVE:	// save settings to eeprom
	{
#ifdef DEBUG_CMD
		debug("CMD_SAVE");
#endif
		if (moving)
		{
			debug("Saving while moving would disrupt move");
			Serial.write(CMD_STALL);
			return;
		}
		addr = 0;
		globals_save(addr, true);
#ifdef HAVE_SPACES
		for (uint8_t t = 0; t < num_spaces; ++t) {
			spaces[t].save_info(addr, true);
			for (uint8_t a = 0; a < spaces[t].num_axes; ++a)
				spaces[t].save_axis(a, addr, true);
			for (uint8_t m = 0; m < spaces[t].num_motors; ++m)
				spaces[t].save_motor(m, addr, true);
		}
#endif
#ifdef HAVE_TEMPS
		for (uint8_t t = 0; t < num_temps; ++t)
			temps[t].save(addr, true);
#endif
#ifdef HAVE_GPIOS
		for (uint8_t t = 0; t < num_gpios; ++t)
			gpios[t].save(addr, true);
#endif
		write_ack();
		return;
	}
	case CMD_READ_GLOBALS:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_GLOBALS");
#endif
		addr = 2;
		globals_save(addr, false);
		reply[0] = addr;
		reply[1] = CMD_DATA;
		write_ack();
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_WRITE_GLOBALS:
	{
#ifdef DEBUG_CMD
		debug("CMD_WRITE_GLOBALS");
#endif
		addr = 2;
		globals_load(addr, false);
		write_ack();
		return;
	}
#ifdef HAVE_SPACES
	case CMD_READ_SPACE_INFO:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_SPACE_INFO");
#endif
		addr = 2;
		which = get_which();
		if (which >= num_spaces) {
			debug("Reading invalid space %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		spaces[which].save_info(addr, false);
		reply[0] = addr;
		reply[1] = CMD_DATA;
		write_ack();
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_READ_SPACE_AXIS:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_SPACE_AXIS");
#endif
		addr = 2;
		which = get_which();
		uint8_t axis = command[3];
		if (which >= num_spaces || axis >= spaces[which].num_axes) {
			debug("Reading invalid axis %d %d", which, axis);
			Serial.write(CMD_STALL);
			return;
		}
		spaces[which].save_axis(axis, addr, false);
		reply[0] = addr;
		reply[1] = CMD_DATA;
		write_ack();
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_READ_SPACE_MOTOR:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_SPACE_MOTOR");
#endif
		addr = 2;
		which = get_which();
		uint8_t motor = command[3];
		if (which >= num_spaces || motor >= spaces[which].num_motors) {
			debug("Reading invalid motor %d %d", which, motor);
			Serial.write(CMD_STALL);
			return;
		}
		spaces[which].save_motor(motor, addr, false);
		reply[0] = addr;
		reply[1] = CMD_DATA;
		write_ack();
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_WRITE_SPACE_INFO:
	{
		which = get_which();
#ifdef DEBUG_CMD
		debug("CMD_WRITE_SPACE_INFO");
#endif
		if (which >= num_spaces) {
			debug("Writing invalid space %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		addr = 3;
		spaces[which].load_info(addr, false);
		write_ack();
		return;
	}
	case CMD_WRITE_SPACE_AXIS:
	{
		which = get_which();
		uint8_t axis = command[3];
#ifdef DEBUG_CMD
		debug("CMD_WRITE_SPACE_MOTOR");
#endif
		if (which >= num_spaces || axis >= spaces[which].num_axes) {
			debug("Writing invalid axis %d %d", which, axis);
			Serial.write(CMD_STALL);
			return;
		}
		addr = 4;
		spaces[which].load_axis(axis, addr, false);
		write_ack();
		return;
	}
	case CMD_WRITE_SPACE_MOTOR:
	{
		which = get_which();
		uint8_t motor = command[3];
#ifdef DEBUG_CMD
		debug("CMD_WRITE_SPACE_MOTOR");
#endif
		if (which >= num_spaces || motor >= spaces[which].num_motors) {
			debug("Writing invalid motor %d %d", which, motor);
			Serial.write(CMD_STALL);
			return;
		}
		addr = 4;
		spaces[which].load_motor(motor, addr, false);
		write_ack();
		return;
	}
#endif
#ifdef HAVE_TEMPS
	case CMD_READ_TEMP:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_TEMP");
#endif
		addr = 2;
		which = get_which();
		if (which >= num_temps) {
			debug("Reading invalid temp %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		temps[which].save(addr, false);
		reply[0] = addr;
		reply[1] = CMD_DATA;
		write_ack();
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_WRITE_TEMP:
	{
		which = get_which();
#ifdef DEBUG_CMD
		debug("CMD_WRITE_TEMP");
#endif
		if (which >= num_temps) {
			debug("Writing invalid temp %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		addr = 3;
		temps[which].load(addr, false);
		write_ack();
		return;
	}
#endif
#ifdef HAVE_GPIOS
	case CMD_READ_GPIO:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_GPIO");
#endif
		addr = 2;
		which = get_which();
		if (which >= num_gpios) {
			debug("Reading invalid gpio %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		gpios[which].save(addr, false);
		reply[0] = addr;
		reply[1] = CMD_DATA;
		write_ack();
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_WRITE_GPIO:
	{
		which = get_which();
#ifdef DEBUG_CMD
		debug("CMD_WRITE_GPIO");
#endif
		if (which >= num_gpios) {
			debug("Writing invalid gpio %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		addr = 3;
		gpios[which].load(addr, false);
		write_ack();
		return;
	}
#endif
	case CMD_QUEUED:
	{
#ifdef DEBUG_CMD
		debug("CMD_QUEUED");
#endif
		last_active = millis();
		reply[0] = 3;
		reply[1] = CMD_QUEUE;
		reply[2] = queue_full ? QUEUE_LENGTH : (queue_end - queue_start + QUEUE_LENGTH) % QUEUE_LENGTH;
		if (command[2]) {
			queue_start = 0;
			queue_end = 0;
			queue_full = false;
			//debug("aborting at request");
			abort_move();
		}
		write_ack();
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_PING:
	{
#ifdef DEBUG_CMD
		debug("CMD_PING");
#endif
		write_ack();
		ping |= 1 << command[2];
		try_send_next();
		return;
	}
#ifdef HAVE_GPIOS
	case CMD_READPIN:
	{
#ifdef DEBUG_CMD
		debug("CMD_READGPIO");
#endif
		which = get_which();
		if (which >= num_gpios)
		{
			debug("Reading invalid gpio %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		write_ack();
		reply[0] = 3;
		reply[1] = CMD_PIN;
		reply[2] = GET(gpios[which].pin, false) ? 1 : 0;
		reply_ready = true;
		try_send_next();
		return;
	}
#endif
#ifdef HAVE_AUDIO
	case CMD_AUDIO_SETUP:
	{
#ifdef DEBUG_CMD
		debug("CMD_AUDIO_SETUP");
#endif
		last_active = millis();
		audio_us_per_bit = get_int16(2);
		uint8_t m0 = 0;
		for (uint8_t t = 0; t < num_spaces; ++t) {
			for (uint8_t m = 0; m < spaces[t].num_motors; ++m) {
				if (command[4 + ((m0 + m) >> 3)] & (1 << ((m0 + m) & 0x7))) {
					spaces[t].motor[m]->audio_flags |= Motor::PLAYING;
					SET(spaces[t].motor[m]->enable_pin);
					motors_busy = true;
				}
				else
					spaces[t].motor[m]->audio_flags &= ~Motor::PLAYING;
			}
			m0 += spaces[t].num_motors;
		}
		// Abort any currently playing sample.
		audio_head = 0;
		audio_tail = 0;
		write_ack();
		return;
	}
	case CMD_AUDIO_DATA:
	{
#ifdef DEBUG_CMD
		debug("CMD_AUDIO_DATA");
#endif
		initialized = true;
		last_active = millis();
		if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
		{
			debug("Audio buffer is full");
			Serial.write(CMD_STALL);
			return;
		}
		for (uint8_t i = 0; i < AUDIO_FRAGMENT_SIZE; ++i)
			audio_buffer[audio_tail][i] = command[2 + i];
		if (audio_tail == audio_head)
			audio_start = micros();
		audio_tail = (audio_tail + 1) % AUDIO_FRAGMENTS;
		if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
			write_ackwait();
		else
			write_ack();
		return;
	}
#endif
	default:
	{
		debug("Invalid command %x %x %x %x", command[0], command[1], command[2], command[3]);
		Serial.write(CMD_STALL);
		return;
	}
	}
}

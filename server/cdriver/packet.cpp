#include "cdriver.h"

//#define DEBUG_CMD

static uint8_t get_which()
{
	return command[0][2] & 0x3f;
}

static float get_float(uint8_t offset)
{
	ReadFloat ret;
	for (uint8_t t = 0; t < sizeof(float); ++t)
		ret.b[t] = command[0][offset + t];
	return ret.f;
}

#if defined(HAVE_AUDIO)
static int16_t get_int16(uint8_t offset)
{
	return ((uint16_t)command[0][offset] & 0xff) | (uint16_t)command[0][offset + 1] << 8;
}
#endif

void packet()
{
	// command[0][0] is the length not including checksum bytes.
	// command[0][1] is the command.
	uint8_t which;
	int32_t addr;
	switch (command[0][1])
	{
	case CMD_RESET: // reset controller; used before reprogramming flash.
	{
#ifdef DEBUG_CMD
		debug("CMD_RESET");
#endif
		reset();
	}
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
			return;
		}
		uint8_t num = 2;
		for (uint8_t t = 0; t < num_spaces; ++t)
			num += spaces[t].num_axes;
		uint8_t const offset = 2 + ((num - 1) >> 3) + 1;	// Bytes from start of command where values are.
		uint8_t t = 0;
		for (uint8_t ch = 0; ch < num; ++ch)
		{
			if (command[0][2 + (ch >> 3)] & (1 << (ch & 0x7)))
			{
				ReadFloat f;
				for (uint8_t i = 0; i < sizeof(float); ++i)
					f.b[i] = command[0][offset + i + t * sizeof(float)];
				if (ch < 2)
					queue[queue_end].f[ch] = f.f;
				else
					queue[queue_end].data[ch - 2] = f.f;
				//debug("goto (%d) %d %f", queue_end, ch, F(f.f));
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
		if (!(command[0][2] & 0x1) || isnan(queue[queue_end].f[0]))
			queue[queue_end].f[0] = INFINITY;
		if (!(command[0][2] & 0x2) || isnan(queue[queue_end].f[1]))
			queue[queue_end].f[1] = queue[queue_end].f[0];
		// F0 and F1 must be valid.
		float F0 = queue[queue_end].f[0];
		float F1 = queue[queue_end].f[1];
		if (isnan(F0) || isnan(F1) || F0 < 0 || F1 < 0 || (F0 == 0 && F1 == 0))
		{
			debug("Invalid F0 or F1: %f %f", F(F0), F(F1));
			return;
		}
		queue[queue_end].cb = command[0][1] == CMD_GOTOCB;
		queue_end = (queue_end + 1) % QUEUE_LENGTH;
		if (queue_end == queue_start) {
			queue_full = true;
			serialdev[0]->write(WAIT);
		}
		else
			serialdev[0]->write(OK);
		if (!moving) {
			int num_movecbs = next_move();
			if (num_movecbs > 0)
				send_host(CMD_MOVECB, num_movecbs);
			buffer_refill();
		}
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
		if (command[0][2]) {
			//debug("sleeping");
			if (moving)
			{
				debug("Sleeping while moving");
				return;
			}
			for (uint8_t t = 0; t < num_spaces; ++t) {
				for (uint8_t m = 0; m < spaces[t].num_motors; ++m) {
					RESET(spaces[t].motor[m]->enable_pin);
					spaces[t].motor[m]->settings[current_fragment].current_pos = 0;
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
		return;
	}
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
			return;
		}
		temps[which].target = get_float(3);
		temps[which].adctarget = temps[which].toadc(temps[which].target);
		//debug("adc target %d from %d", temps[which]->adctarget, int32_t(temps[which]->target / 1024));
		if (temps[which].adctarget >= MAXINT) {
			// main loop doesn't handle it anymore, so it isn't disabled there.
			//debug("Temp %d disabled", which);
			if (temps[which].is_on) {
				RESET(temps[which].power_pin);
				temps[which].is_on = false;
				--temps_busy;
			}
		}
		else if (temps[which].adctarget < 0) {
			// main loop doesn't handle it anymore, so it isn't enabled there.
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
		arch_setup_temp(which, temps[which].thermistor_pin.pin, true, temps[which].power_pin.valid() ? temps[which].power_pin.pin : ~0, temps[which].power_pin.inverted(), temps[which].adctarget);
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
			return;
		}
		ReadFloat min, max;
		for (uint8_t i = 0; i < sizeof(float); ++i)
		{
			min.b[i] = command[0][3 + i];
			max.b[i] = command[0][3 + i + sizeof(float)];
		}
		temps[which].min_alarm = min.f;
		temps[which].max_alarm = max.f;
		temps[which].adcmin_alarm = temps[which].toadc(temps[which].min_alarm);
		temps[which].adcmax_alarm = temps[which].toadc(temps[which].max_alarm);
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
			return;
		}
		if (!temps[which].thermistor_pin.valid()) {
			debug("Reading temp %d with invalid thermistor", which);
			send_host(CMD_TEMP);
			return;
		}
		requested_temp = which;
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
			return;
		}
		uint32_t t = utime();
		if (temps[which].is_on) {
			// This causes an insignificant error in the model, but when using this you probably aren't using the model anyway, and besides you won't notice the error even if you do.
			temps[which].time_on += t - temps[which].last_temp_time;
			temps[which].last_temp_time = t;
		}
		send_host(CMD_POWER, temps[which].time_on, t);
		temps[which].time_on = 0;
	}
	case CMD_SETPOS:	// Set current position
	{
#ifdef DEBUG_CMD
		debug("CMD_SETPOS");
#endif
		last_active = millis();
		which = get_which();
		uint8_t t = command[0][3];
		if (which >= num_spaces || t >= spaces[which].num_axes)
		{
			debug("Invalid axis for setting position: %d %d", which, t);
			return;
		}
		if (!motors_busy)
		{
			for (uint8_t s = 0; s < num_spaces; ++s) {
				for (uint8_t m = 0; m < spaces[s].num_motors; ++m)
					SET(spaces[s].motor[m]->enable_pin);
			}
			motors_busy = true;
		}
		if (moving)
		{
			debug("Setting position while moving");
			return;
		}
		for (uint8_t a = 0; a < spaces[which].num_axes; ++a) {
			spaces[which].axis[a]->source = NAN;
			spaces[which].axis[a]->current = NAN;
		}
		float f = get_float(4);
		int32_t diff = int32_t(f * spaces[which].motor[t]->steps_per_m + (f > 0 ? .49 : -.49)) - spaces[which].motor[t]->settings[current_fragment].current_pos;
		spaces[which].motor[t]->settings[current_fragment].current_pos += diff;
		spaces[which].motor[t]->settings[current_fragment].hwcurrent_pos += diff;
		//debug("cp4 %d %d", spaces[which].motor[t]->settings[current_fragment].current_pos, diff);
		arch_addpos(which, t, diff);
		//debug("setpos %d %d %f", which, t, F(spaces[which].motor[t]->current_pos));
		return;
	}
	case CMD_GETPOS:	// Get current position
	{
#ifdef DEBUG_CMD
		debug("CMD_GETPOS");
#endif
		which = get_which();
		uint8_t t = command[0][3];
		if (which >= num_spaces || t >= spaces[which].num_axes)
		{
			debug("Getting position of invalid axis %d %d", which, t);
			return;
		}
		if (isnan(spaces[which].axis[t]->source)) {
			space_types[spaces[which].type].reset_pos(&spaces[which]);
			for (uint8_t a = 0; a < spaces[which].num_axes; ++a)
				spaces[which].axis[a]->current = spaces[which].axis[a]->source;
		}
		send_host(CMD_POS, which, t, spaces[which].axis[t]->current - spaces[which].axis[t]->offset);
		return;
	}
	case CMD_READ_GLOBALS:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_GLOBALS");
#endif
		addr = 0;
		globals_save(addr);
		send_host(CMD_DATA, 0, 0, 0, 0, addr);
		return;
	}
	case CMD_WRITE_GLOBALS:
	{
#ifdef DEBUG_CMD
		debug("CMD_WRITE_GLOBALS");
#endif
		addr = 2;
		globals_load(addr);
		return;
	}
	case CMD_READ_SPACE_INFO:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_SPACE_INFO");
#endif
		addr = 0;
		which = get_which();
		if (which >= num_spaces) {
			debug("Reading invalid space %d", which);
			return;
		}
		spaces[which].save_info(addr);
		send_host(CMD_DATA, 0, 0, 0, 0, addr);
		return;
	}
	case CMD_READ_SPACE_AXIS:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_SPACE_AXIS");
#endif
		addr = 0;
		which = get_which();
		uint8_t axis = command[0][3];
		if (which >= num_spaces || axis >= spaces[which].num_axes) {
			debug("Reading invalid axis %d %d", which, axis);
			return;
		}
		spaces[which].save_axis(axis, addr);
		send_host(CMD_DATA, 0, 0, 0, 0, addr);
		return;
	}
	case CMD_READ_SPACE_MOTOR:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_SPACE_MOTOR");
#endif
		addr = 0;
		which = get_which();
		uint8_t motor = command[0][3];
		if (which >= num_spaces || motor >= spaces[which].num_motors) {
			debug("Reading invalid motor %d %d > %d %d", which, motor, num_spaces, which < num_spaces ? spaces[which].num_motors : -1);
			return;
		}
		spaces[which].save_motor(motor, addr);
		send_host(CMD_DATA, 0, 0, 0, 0, addr);
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
			return;
		}
		addr = 3;
		spaces[which].load_info(addr);
		if (which == 0)
			for (uint8_t s = 0; s < num_spaces; ++s)
				space_types[spaces[s].type].change0(&spaces[s]);
		return;
	}
	case CMD_WRITE_SPACE_AXIS:
	{
		which = get_which();
		uint8_t axis = command[0][3];
#ifdef DEBUG_CMD
		debug("CMD_WRITE_SPACE_MOTOR");
#endif
		if (which >= num_spaces || axis >= spaces[which].num_axes) {
			debug("Writing invalid axis %d %d", which, axis);
			return;
		}
		addr = 4;
		spaces[which].load_axis(axis, addr);
		return;
	}
	case CMD_WRITE_SPACE_MOTOR:
	{
		which = get_which();
		uint8_t motor = command[0][3];
#ifdef DEBUG_CMD
		debug("CMD_WRITE_SPACE_MOTOR");
#endif
		if (which >= num_spaces || motor >= spaces[which].num_motors) {
			debug("Writing invalid motor %d %d", which, motor);
			return;
		}
		addr = 4;
		spaces[which].load_motor(motor, addr);
		return;
	}
	case CMD_READ_TEMP:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_TEMP");
#endif
		addr = 0;
		which = get_which();
		if (which >= num_temps) {
			debug("Reading invalid temp %d", which);
			return;
		}
		temps[which].save(addr);
		send_host(CMD_DATA, 0, 0, 0, 0, addr);
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
			return;
		}
		addr = 3;
		temps[which].load(addr, which);
		return;
	}
	case CMD_READ_GPIO:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_GPIO");
#endif
		addr = 0;
		which = get_which();
		if (which >= num_gpios) {
			debug("Reading invalid gpio %d", which);
			return;
		}
		gpios[which].save(addr);
		send_host(CMD_DATA, 0, 0, 0, 0, addr);
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
			return;
		}
		addr = 3;
		gpios[which].load(which, addr);
		return;
	}
	case CMD_QUEUED:
	{
#ifdef DEBUG_CMD
		debug("CMD_QUEUED");
#endif
		last_active = millis();
		if (command[0][2]) {
			queue_start = 0;
			queue_end = 0;
			queue_full = false;
			cbs_after_current_move = 0;
			arch_stop();
		}
		send_host(CMD_QUEUE, queue_full ? QUEUE_LENGTH : (queue_end - queue_start + QUEUE_LENGTH) % QUEUE_LENGTH);
		return;
	}
	case CMD_HOME:
	{
#ifdef DEBUG_CMD
		debug("CMD_HOME");
#endif
		arch_home();
		return;
	}
	case CMD_READPIN:
	{
#ifdef DEBUG_CMD
		debug("CMD_READGPIO");
#endif
		which = get_which();
		if (which >= num_gpios)
		{
			debug("Reading invalid gpio %d", which);
			return;
		}
		send_host(CMD_PIN, GET(gpios[which].pin, false) ? 1 : 0);
		return;
	}
#ifdef HAVE_AUDIO
	case CMD_AUDIO_SETUP:
	{
#ifdef DEBUG_CMD
		debug("CMD_AUDIO_SETUP");
#endif
		last_active = millis();
		audio_us_per_sample = get_int16(2);
		uint8_t m0 = 0;
		for (uint8_t t = 0; t < num_spaces; ++t) {
			for (uint8_t m = 0; m < spaces[t].num_motors; ++m) {
				if (command[0][4 + ((m0 + m) >> 3)] & (1 << ((m0 + m) & 0x7))) {
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
			return;
		}
		for (uint8_t i = 0; i < AUDIO_FRAGMENT_SIZE; ++i)
			audio_buffer[audio_tail][i] = command[0][2 + i];
		if (audio_tail == audio_head)
			audio_start = utime();
		audio_tail = (audio_tail + 1) % AUDIO_FRAGMENTS;
		if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
			serialdev[0]->write(WAIT);
		else
			serialdev[0]->write(OK);
		return;
	}
#endif
	default:
	{
		debug("Invalid command %x %x %x %x", command[0][0], command[0][1], command[0][2], command[0][3]);
		return;
	}
	}
}

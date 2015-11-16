#include "cdriver.h"

//#define DEBUG_CMD

static uint8_t get_which()
{
	return command[0][2] & 0x3f;
}

static double get_float(uint8_t offset)
{
	ReadFloat ret;
	for (uint8_t t = 0; t < sizeof(double); ++t)
		ret.b[t] = command[0][offset + t];
	return ret.f;
}

void settemp(int which, double target) {
	if (which < 0 || which >= num_temps)
	{
		debug("Setting invalid temp %d", which);
		//abort();
		return;
	}
	temps[which].target[0] = target;
	temps[which].adctarget[0] = temps[which].toadc(target, MAXINT);
	//debug("adc target %d from %f", temps[which].adctarget[0], temps[which].target[0]);
	if (temps[which].adctarget[0] >= MAXINT) {
		// main loop doesn't handle it anymore, so it isn't disabled there.
		//debug("Temp %d disabled", which);
		if (temps[which].is_on[0]) {
			RESET(temps[which].power_pin[0]);
			temps[which].is_on[0] = false;
			--temps_busy;
		}
	}
	else if (temps[which].adctarget[0] < 0) {
		// main loop doesn't handle it anymore, so it isn't enabled there.
		//debug("Temp %d enabled", which);
		if (!temps[which].is_on[0]) {
			SET(temps[which].power_pin[0]);
			temps[which].is_on[0] = true;
			++temps_busy;
		}
	}
	else {
		//debug("Temp %d set to %f", which, target);
		initialized = true;
	}
	if (temps[which].thermistor_pin.valid())
		arch_setup_temp(which, temps[which].thermistor_pin.pin, true, temps[which].power_pin[0].valid() ? temps[which].power_pin[0].pin : ~0, temps[which].power_pin[0].inverted(), temps[which].adctarget[0], temps[which].power_pin[1].valid() ? temps[which].power_pin[1].pin : ~0, temps[which].power_pin[1].inverted(), temps[which].adctarget[1]);
}

void waittemp(int which, double mintemp, double maxtemp) {
	if (which >= num_temps)
	{
		debug("Waiting for invalid temp %d", which);
		//abort();
		return;
	}
	temps[which].min_alarm = mintemp;
	temps[which].max_alarm = maxtemp;
	temps[which].adcmin_alarm = temps[which].toadc(temps[which].min_alarm, -1);
	temps[which].adcmax_alarm = temps[which].toadc(temps[which].max_alarm, MAXINT);
}

void setpos(int which, int t, double f) {
	if (!motors_busy)
	{
		for (uint8_t s = 0; s < NUM_SPACES; ++s) {
			for (uint8_t m = 0; m < spaces[s].num_motors; ++m)
				SET(spaces[s].motor[m]->enable_pin);
		}
		motors_busy = true;
	}
	for (uint8_t a = 0; a < spaces[which].num_axes; ++a) {
		spaces[which].axis[a]->settings.source = NAN;
		spaces[which].axis[a]->settings.current = NAN;
	}
	//debug("setting pos for %d %d", which, t);
	double diff = f * spaces[which].motor[t]->steps_per_unit - spaces[which].motor[t]->settings.current_pos;
	int32_t ipos = int(spaces[which].motor[t]->settings.current_pos);
	spaces[which].motor[t]->settings.current_pos += diff;
	int32_t idiff = int(spaces[which].motor[t]->settings.current_pos) - ipos;
	for (int fragment = 0; fragment < FRAGMENTS_PER_BUFFER; ++fragment)
		spaces[which].motor[t]->history[fragment].current_pos += diff;
	if (isnan(spaces[which].axis[t]->settings.current)) {
		space_types[spaces[which].type].reset_pos(&spaces[which]);
		for (uint8_t a = 0; a < spaces[which].num_axes; ++a)
			spaces[which].axis[a]->settings.current = spaces[which].axis[a]->settings.source;
	}
	arch_addpos(which, t, idiff);
	cpdebug(which, t, "setpos diff %d", diff);
	//arch_stop();
	space_types[spaces[which].type].reset_pos(&spaces[which]);
	for (uint8_t a = 0; a < spaces[which].num_axes; ++a)
		spaces[which].axis[a]->settings.current = spaces[which].axis[a]->settings.source;
	/*for (uint8_t a = 0; a < spaces[which].num_axes; ++a)
		debug("setpos done source %f", spaces[which].axis[a]->settings.source);
	// */
}

static void get_cb(bool value) {
	send_host(CMD_PIN, value ? 1 : 0);
}

void packet()
{
	// command[0][0] is the length not including checksum bytes.
	// command[0][1] is the command.
	uint8_t which;
	int32_t addr;
	switch (command[0][1])
	{
#ifdef SERIAL
	case CMD_SET_UUID: // Program a new uuid into the flash.
	{
#ifdef DEBUG_CMD
		debug("CMD_SET_UUID");
#endif
		for (int i = 0; i < UUID_SIZE; ++i) {
			uuid[i] = command[0][2 + i];
			debug("uuid %d: %x", i, uuid[i]);
		}
		arch_set_uuid();
		break;
	}
	case CMD_GET_UUID: // get uuid as received from firmware.
	{
#ifdef DEBUG_CMD
		debug("CMD_GET_UUID");
#endif
		memcpy(datastore, uuid, 16);
		send_host(CMD_UUID, 0, 0, 0, 0, 16);
		break;
	}
#endif
	case CMD_LINE:	// line
	case CMD_SINGLE:	// line without followers
	case CMD_PROBE:	// probe
	{
#ifdef DEBUG_CMD
		debug("CMD_LINE/PROBE");
#endif
		last_active = millis();
		if (settings.queue_full)
		{
			debug("Host ignores wait request");
			abort();
			return;
		}
		uint8_t num = 2;
		for (uint8_t t = 0; t < NUM_SPACES; ++t)
			num += spaces[t].num_axes;
		queue[settings.queue_end].probe = command[0][1] == CMD_PROBE;
		queue[settings.queue_end].single = command[0][1] == CMD_SINGLE;
		uint8_t const offset = 2 + ((num - 1) >> 3) + 1;	// Bytes from start of command where values are.
		uint8_t t = 0;
		for (uint8_t ch = 0; ch < num; ++ch)
		{
			if (command[0][2 + (ch >> 3)] & (1 << (ch & 0x7)))
			{
				ReadFloat f;
				for (uint8_t i = 0; i < sizeof(double); ++i)
					f.b[i] = command[0][offset + i + t * sizeof(double)];
				if (ch < 2)
					queue[settings.queue_end].f[ch] = f.f;
				else
					queue[settings.queue_end].data[ch - 2] = f.f;
				//debug("line (%d) %d %f", settings.queue_end, ch, f.f);
				initialized = true;
				++t;
			}
			else {
				if (ch < 2)
					queue[settings.queue_end].f[ch] = NAN;
				else
					queue[settings.queue_end].data[ch - 2] = NAN;
				//debug("line %d -", ch);
			}
		}
		if (!(command[0][2] & 0x1) || isnan(queue[settings.queue_end].f[0]))
			queue[settings.queue_end].f[0] = INFINITY;
		if (!(command[0][2] & 0x2) || isnan(queue[settings.queue_end].f[1]))
			queue[settings.queue_end].f[1] = queue[settings.queue_end].f[0];
		// F0 and F1 must be valid.
		double F0 = queue[settings.queue_end].f[0];
		double F1 = queue[settings.queue_end].f[1];
		if (isnan(F0) || isnan(F1) || (F0 == 0 && F1 == 0))
		{
			debug("Invalid F0 or F1: %f %f", F0, F1);
			abort();
			return;
		}
		queue[settings.queue_end].cb = true;
		queue[settings.queue_end].arc = false;
		settings.queue_end = (settings.queue_end + 1) % QUEUE_LENGTH;
		if (settings.queue_end == settings.queue_start) {
			settings.queue_full = true;
			serialdev[0]->write(WAIT);
		}
		else
			serialdev[0]->write(OK);
		if (!computing_move) {
			//debug("starting move");
			int num_movecbs = next_move();
			if (num_movecbs > 0) {
				if (arch_running()) {
					//debug("adding %d cbs after current move", num_movecbs);
					cbs_after_current_move += num_movecbs;
				}
				else
					send_host(CMD_MOVECB, num_movecbs);
			}
			//debug("no movecbs to add (prev %d)", settings[(current_fragment - 1 + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER].cbs);
			buffer_refill();
		}
		//else
		//	debug("waiting with move");
		break;
	}
	case CMD_RUN_FILE: // Run commands from a file.
	{
#ifdef DEBUG_CMD
		debug("CMD_RUN_FILE");
#endif
		ReadFloat args[2];
		for (int i = 0; i < sizeof(double); ++i)
		{
			for (int j = 0; j < 2; ++j)
				args[j].b[i] = command[0][3 + i + j * sizeof(double)];
		}
		run_file(command[0][0] - 21 - command[0][20], reinterpret_cast<char const *>(&command[0][21]), command[0][20], reinterpret_cast<char const *>(&command[0][21 + command[0][20]]), command[0][2], args[0].f, args[1].f, uint8_t(command[0][19]) == 0xff ? -1 : command[0][19]);
		break;
	}
	case CMD_SLEEP:	// Enable or disable motor current
	{
#ifdef DEBUG_CMD
		debug("CMD_SLEEP");
#endif
		last_active = millis();
		if (command[0][2]) {
			//debug("sleeping");
			if (arch_running() && !stop_pending)
			{
				debug("Sleeping while moving");
				//abort();
				return;
			}
			for (uint8_t t = 0; t < NUM_SPACES; ++t) {
				for (uint8_t m = 0; m < spaces[t].num_motors; ++m) {
					//debug("resetting %d %d %x", t, m, spaces[t].motor[m]->enable_pin.write());
					RESET(spaces[t].motor[m]->enable_pin);
				}
				for (uint8_t a = 0; a < spaces[t].num_axes; ++a) {
					spaces[t].axis[a]->settings.source = NAN;
					spaces[t].axis[a]->settings.current = NAN;
				}
			}
			motors_busy = false;
		}
		else {
			for (uint8_t t = 0; t < NUM_SPACES; ++t) {
				for (uint8_t m = 0; m < spaces[t].num_motors; ++m) {
					//debug("setting %d %d %x", t, m, spaces[t].motor[m]->enable_pin.write());
					SET(spaces[t].motor[m]->enable_pin);
				}
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
		double target = get_float(3);
		settemp(which, target);
		return;
	}
	case CMD_WAITTEMP:	// wait for a temperature sensor to reach a target range
	{
#ifdef DEBUG_CMD
		debug("CMD_WAITTEMP");
#endif
		initialized = true;
		which = get_which();
		ReadFloat min_temp, max_temp;
		for (uint8_t i = 0; i < sizeof(double); ++i)
		{
			min_temp.b[i] = command[0][3 + i];
			max_temp.b[i] = command[0][3 + i + sizeof(double)];
		}
		waittemp(which, min_temp.f, max_temp.f);
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
			//abort();
			return;
		}
		if (!temps[which].thermistor_pin.valid()) {
			debug("Reading temp %d with invalid thermistor", which);
			//abort();
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
			//abort();
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
		return;
	}
	case CMD_SETPOS:	// Set current position
	{
#ifdef DEBUG_CMD
		debug("CMD_SETPOS");
#endif
		last_active = millis();
		which = get_which();
		uint8_t t = command[0][3];
		if (which >= NUM_SPACES || t >= spaces[which].num_axes)
		{
			debug("Invalid axis for setting position: %d %d", which, t);
			abort();
			return;
		}
		if (arch_running() && !stop_pending)
		{
			debug("Setting position while moving");
			//abort();
			return;
		}
		double f = get_float(4);
		setpos(which, t, f);
		return;
	}
	case CMD_GETPOS:	// Get current position
	{
#ifdef DEBUG_CMD
		debug("CMD_GETPOS");
#endif
		which = get_which();
		uint8_t t = command[0][3];
		if (which >= NUM_SPACES || t >= spaces[which].num_axes)
		{
			debug("Getting position of invalid axis %d %d", which, t);
			abort();
			return;
		}
		if (!motors_busy) {
			send_host(CMD_POS, which, t, NAN);
			return;
		}
		if (isnan(spaces[which].axis[t]->settings.source)) {
			//debug("resetting space %d for getpos; %f", which, spaces[0].axis[0]->settings.current);
			space_types[spaces[which].type].reset_pos(&spaces[which]);
			for (uint8_t a = 0; a < spaces[which].num_axes; ++a)
				spaces[which].axis[a]->settings.current = spaces[which].axis[a]->settings.source;
		}
		double value = spaces[which].axis[t]->settings.current;
		if (which == 0) {
			for (int s = 0; s < NUM_SPACES; ++s) {
				value = space_types[spaces[s].type].unchange0(&spaces[s], t, value);
			}
			if (t == 2)
				value -= zoffset;
		}
		send_host(CMD_POS, which, t, value);
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
		discarding = true;
		arch_discard();
		addr = 2;
		globals_load(addr);
		discarding = false;
		buffer_refill();
		return;
	}
	case CMD_READ_SPACE_INFO:
	{
#ifdef DEBUG_CMD
		debug("CMD_READ_SPACE_INFO");
#endif
		addr = 0;
		which = get_which();
		if (which >= NUM_SPACES) {
			debug("Reading invalid space %d", which);
			abort();
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
		if (which >= NUM_SPACES || axis >= spaces[which].num_axes) {
			debug("Reading invalid axis %d %d", which, axis);
			abort();
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
		if (which >= NUM_SPACES || motor >= spaces[which].num_motors) {
			debug("Reading invalid motor %d %d > %d", which, motor, which < NUM_SPACES ? spaces[which].num_motors : -1);
			abort();
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
		if (which >= NUM_SPACES) {
			debug("Writing invalid space %d", which);
			abort();
			return;
		}
		discarding = true;
		arch_discard();
		addr = 3;
		spaces[which].load_info(addr);
		discarding = false;
		buffer_refill();
		return;
	}
	case CMD_WRITE_SPACE_AXIS:
	{
		which = get_which();
		uint8_t axis = command[0][3];
#ifdef DEBUG_CMD
		debug("CMD_WRITE_SPACE_MOTOR");
#endif
		if (which >= NUM_SPACES || axis >= spaces[which].num_axes) {
			debug("Writing invalid axis %d %d", which, axis);
			abort();
			return;
		}
		discarding = true;
		arch_discard();
		addr = 4;
		spaces[which].load_axis(axis, addr);
		discarding = false;
		buffer_refill();
		return;
	}
	case CMD_WRITE_SPACE_MOTOR:
	{
		which = get_which();
		uint8_t motor = command[0][3];
#ifdef DEBUG_CMD
		debug("CMD_WRITE_SPACE_MOTOR");
#endif
		if (which >= NUM_SPACES || motor >= spaces[which].num_motors) {
			debug("Writing invalid motor %d %d", which, motor);
			abort();
			return;
		}
		discarding = true;
		arch_discard();
		addr = 4;
		spaces[which].load_motor(motor, addr);
		discarding = false;
		buffer_refill();
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
			abort();
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
			abort();
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
			abort();
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
			abort();
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
		send_host(CMD_QUEUE, settings.queue_full ? QUEUE_LENGTH : (settings.queue_end - settings.queue_start + QUEUE_LENGTH) % QUEUE_LENGTH);
		if (command[0][2]) {
			if (run_file_map)
				run_file_wait += 1;
			else
				cbs_after_current_move = 0;
			arch_stop();
			settings.queue_start = 0;
			settings.queue_end = 0;
			settings.queue_full = false;
		}
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
			abort();
			return;
		}
		GET(gpios[which].pin, false, get_cb);
		return;
	}
#ifdef SERIAL
	case CMD_RECONNECT:
	{
#ifdef DEBUG_CMD
		debug("CMD_RECONNECT");
#endif
		if (arch_fds() != 0) {
			debug("Unexpected reconnect");
			abort();
			return;
		}
		arch_reconnect(reinterpret_cast <char *>(&command[0][2]));
		return;
	}
#endif
	case CMD_RESUME:
	{
#ifdef DEBUG_CMD
		debug("CMD_RESUME");
#endif
		if (run_file_wait)
			run_file_wait -= 1;
		run_file_fill_queue();
		return;
	}
	case CMD_GETTIME:
	{
#ifdef DEBUG_CMD
		debug("CMD_GETTIME");
#endif
		send_host(CMD_TIME, 0, 0, (history[running_fragment].run_time + history[running_fragment].run_dist / max_v) / feedrate + settings.hwtime / 1e6);
		return;
	}
	case CMD_SPI:
	{
#ifdef DEBUG_CMD
		debug("CMD_SPI");
#endif
		arch_send_spi(command[0][2], &command[0][3]);
		return;
	}
	default:
	{
		debug("Invalid command %x %x %x %x", command[0][0], command[0][1], command[0][2], command[0][3]);
		abort();
		return;
	}
	}
}

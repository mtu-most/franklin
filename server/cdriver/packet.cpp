/* packet.cpp - command packet handling for Franklin
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
 */

#include "cdriver.h"

//#define DEBUG_CMD

static int get_which()
{
	return command[0][3] & 0x3f;
}

static double get_float(int offset)
{
	ReadFloat ret;
	for (unsigned t = 0; t < sizeof(double); ++t)
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
	if (temps[which].thermistor_pin.valid()) {
		int llh = temps[which].adclimit[0][0];
		int lhh = temps[which].adclimit[0][1];
		int llf = temps[which].adclimit[1][0];
		int lhf = temps[which].adclimit[1][1];
		arch_setup_temp(which, temps[which].thermistor_pin.pin, true, temps[which].power_pin[0].valid() ? temps[which].power_pin[0].pin : ~0, temps[which].power_pin[0].inverted(), temps[which].adctarget[0], llh, lhh, temps[which].power_pin[1].valid() ? temps[which].power_pin[1].pin : ~0, temps[which].power_pin[1].inverted(), temps[which].adctarget[1], llf, lhf, temps[which].hold_time);
	}
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
		debug("Error: Setting position while motors are not busy!");
		for (int s = 0; s < NUM_SPACES; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m)
				SET(spaces[s].motor[m]->enable_pin);
		}
		motors_busy = true;
	}
	if (isnan(spaces[which].motor[t]->steps_per_unit)) {
		debug("Error: NaN steps per unit");
		abort();
	}
	for (int a = 0; a < spaces[which].num_axes; ++a) {
		spaces[which].axis[a]->settings.source = NAN;
		spaces[which].axis[a]->settings.current = NAN;
	}
	//debug("setting pos for %d %d to %f", which, t, f);
	double diff;
	if (!isnan(spaces[which].motor[t]->settings.current_pos)) {
		diff = f * spaces[which].motor[t]->steps_per_unit - arch_round_pos(which, t, spaces[which].motor[t]->settings.current_pos);
		spaces[which].motor[t]->settings.current_pos += diff;
		//debug("non nan %f %f %f", spaces[which].motor[t]->settings.current_pos, diff, f);
		//debug("setpos non-nan %d %d %f", which, t, diff);
	}
	else {
		diff = f * spaces[which].motor[t]->steps_per_unit;
		spaces[which].motor[t]->settings.current_pos = diff;
		//debug("setpos nan %d %d %f", which, t, diff);
	}
	for (int fragment = 0; fragment < FRAGMENTS_PER_BUFFER; ++fragment) {
		if (!isnan(spaces[which].motor[t]->history[fragment].current_pos))
			spaces[which].motor[t]->history[fragment].current_pos += diff;
		else
			spaces[which].motor[t]->history[fragment].current_pos = diff;
	}
	if (isnan(spaces[which].axis[t]->settings.current)) {
		reset_pos(&spaces[which]);
		for (int a = 0; a < spaces[which].num_axes; ++a)
			spaces[which].axis[a]->settings.current = spaces[which].axis[a]->settings.source;
	}
	arch_addpos(which, t, diff);
	cpdebug(which, t, "setpos diff %d", diff);
	//arch_stop();
	reset_pos(&spaces[which]);
	for (int a = 0; a < spaces[which].num_axes; ++a)
		spaces[which].axis[a]->settings.current = spaces[which].axis[a]->settings.source;
	/*for (int a = 0; a < spaces[which].num_axes; ++a)
		debug("setpos done source %f", spaces[which].axis[a]->settings.source);
	// */
}

static void get_cb(bool value) {
	send_host(CMD_PIN, value ? 1 : 0);
}

void packet()
{
	// command[0][0:1] is the length not including checksum bytes.
	// command[0][2] is the command.
	uint8_t which;
	int32_t addr;
	switch (command[0][2])
	{
#ifdef SERIAL
	case CMD_SET_UUID: // Program a new uuid into the flash.
	{
#ifdef DEBUG_CMD
		debug("CMD_SET_UUID");
#endif
		for (int i = 0; i < UUID_SIZE; ++i) {
			uuid[i] = command[0][3 + i];
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
		int num = 2;
		for (int t = 0; t < NUM_SPACES; ++t)
			num += spaces[t].num_axes;
		queue[settings.queue_end].probe = command[0][2] == CMD_PROBE;
		queue[settings.queue_end].single = command[0][2] == CMD_SINGLE;
		int const offset = 3 + ((num - 1) >> 3) + 1;	// Bytes from start of command where values are.
		int t = 0;
		for (int ch = 0; ch < num; ++ch)
		{
			if (command[0][3 + (ch >> 3)] & (1 << (ch & 0x7)))
			{
				ReadFloat f;
				for (unsigned i = 0; i < sizeof(double); ++i)
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
		if (!(command[0][3] & 0x1) || isnan(queue[settings.queue_end].f[0]))
			queue[settings.queue_end].f[0] = INFINITY;
		if (!(command[0][3] & 0x2) || isnan(queue[settings.queue_end].f[1]))
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
					cbs_after_current_move += num_movecbs;
					//debug("adding %d cbs after current move to %d", num_movecbs, cbs_after_current_move);
				}
				else {
					send_host(CMD_MOVECB, num_movecbs);
					//debug("sent immediate %d cbs", num_movecbs);
				}
			}
			//debug("no movecbs to add (prev %d)", history[(current_fragment - 1 + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER].cbs);
			buffer_refill();
		}
		//else
		//	debug("waiting with move");
		break;
	}
	case CMD_PARSE_GCODE: // Convert a file of G-Code into a machine readable file.
	{
		int fulllen = ((command[0][0] & 0xff) << 8) | (command[0][1] & 0xff);
		int namelen = *reinterpret_cast <short *>(&command[0][3]);
		parse_gcode(std::string((char *)&command[0][5], namelen), std::string((char *)&command[0][5 + namelen], fulllen - namelen - 5));
		send_host(CMD_PARSED_GCODE);
		break;
	}
	case CMD_RUN_FILE: // Run commands from a file.
	{
#ifdef DEBUG_CMD
		debug("CMD_RUN_FILE");
#endif
		ReadFloat args[2];
		for (unsigned i = 0; i < sizeof(double); ++i)
		{
			for (unsigned j = 0; j < 2; ++j)
				args[j].b[i] = command[0][4 + i + j * sizeof(double)];
		}
		int namelen = (((command[0][0] & 0xff) << 8) | (command[0][1] & 0xff)) - 22 - command[0][21];
		run_file(namelen, reinterpret_cast<char const *>(&command[0][22]), command[0][21], reinterpret_cast<char const *>(&command[0][22 + namelen]), command[0][3], args[0].f, args[1].f, uint8_t(command[0][20]) == 0xff ? -1 : command[0][20]);
		break;
	}
	case CMD_SLEEP:	// Enable or disable motor current
	{
#ifdef DEBUG_CMD
		debug("CMD_SLEEP");
#endif
		last_active = millis();
		if (command[0][3]) {
			//debug("sleeping");
			if (arch_running() && !stop_pending)
			{
				debug("Sleeping while moving");
				//abort();
				return;
			}
			for (int t = 0; t < NUM_SPACES; ++t) {
				for (int m = 0; m < spaces[t].num_motors; ++m) {
					//debug("resetting %d %d %x", t, m, spaces[t].motor[m]->enable_pin.write());
					RESET(spaces[t].motor[m]->enable_pin);
				}
				for (int a = 0; a < spaces[t].num_axes; ++a) {
					spaces[t].axis[a]->settings.source = NAN;
					spaces[t].axis[a]->settings.current = NAN;
				}
			}
			motors_busy = false;
		}
		else {
			for (int t = 0; t < NUM_SPACES; ++t) {
				for (int m = 0; m < spaces[t].num_motors; ++m) {
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
		double target = get_float(4);
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
		for (unsigned i = 0; i < sizeof(double); ++i)
		{
			min_temp.b[i] = command[0][4 + i];
			max_temp.b[i] = command[0][4 + i + sizeof(double)];
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
			send_host(CMD_TEMP, 0, 0, NAN);
			return;
		}
		if (!temps[which].thermistor_pin.valid()) {
			// Before first connection, NUM_PINS is 0; don't break on that.
			if (NUM_PINS > 0) {
				debug("Reading temp %d with invalid thermistor", which);
				//abort();
			}
			send_host(CMD_TEMP, 0, 0, NAN);
			return;
		}
		arch_request_temp(which);
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
		int32_t t = utime();
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
		uint8_t t = command[0][4];
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
		double f = get_float(5);
		setpos(which, t, f);
		return;
	}
	case CMD_GETPOS:	// Get current position
	{
#ifdef DEBUG_CMD
		debug("CMD_GETPOS");
#endif
		which = get_which();
		uint8_t t = command[0][4];
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
		if (isnan(spaces[which].axis[t]->settings.current)) {
			//debug("resetting space %d for getpos; %f", which, spaces[0].axis[0]->settings.current);
			reset_pos(&spaces[which]);
			for (int a = 0; a < spaces[which].num_axes; ++a)
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
		addr = 3;
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
		uint8_t axis = command[0][4];
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
		uint8_t motor = command[0][4];
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
		debug("CMD_WRITE_SPACE_INFO %d", which);
#endif
		if (which >= NUM_SPACES) {
			debug("Writing invalid space %d", which);
			abort();
			return;
		}
		discarding = true;
		arch_discard();
		addr = 4;
		spaces[which].load_info(addr);
		discarding = false;
		buffer_refill();
		return;
	}
	case CMD_WRITE_SPACE_AXIS:
	{
		which = get_which();
		uint8_t axis = command[0][4];
#ifdef DEBUG_CMD
		debug("CMD_WRITE_SPACE_AXIS");
#endif
		if (which >= NUM_SPACES || axis >= spaces[which].num_axes) {
			debug("Writing invalid axis %d %d", which, axis);
			abort();
			return;
		}
		discarding = true;
		arch_discard();
		addr = 5;
		spaces[which].load_axis(axis, addr);
		discarding = false;
		buffer_refill();
		return;
	}
	case CMD_WRITE_SPACE_MOTOR:
	{
		which = get_which();
		uint8_t motor = command[0][4];
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
		addr = 5;
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
		addr = 4;
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
		addr = 4;
		gpios[which].load(addr);
		return;
	}
	case CMD_QUEUED:
	{
#ifdef DEBUG_CMD
		debug("CMD_QUEUED");
#endif
		last_active = millis();
		send_host(CMD_QUEUE, settings.queue_full ? QUEUE_LENGTH : (settings.queue_end - settings.queue_start + QUEUE_LENGTH) % QUEUE_LENGTH);
		if (command[0][3]) {
			if (run_file_map)
				run_file_wait += 1;
			else {
				//debug("clearing %d cbs after current move for abort", cbs_after_current_move);
				cbs_after_current_move = 0;
			}
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
	case CMD_FORCE_DISCONNECT:
	{
#ifdef DEBUG_CMD
		debug("CMD_FORCE_DISCONNECT");
#endif
		disconnect(false);
		return;
	}
	case CMD_CONNECT:
	{
#ifdef DEBUG_CMD
		debug("CMD_CONNECT");
#endif
		if (arch_fds() != 0) {
			debug("Unexpected connect");
			abort();
			return;
		}
		arch_connect(reinterpret_cast <char *>(&command[0][3]), reinterpret_cast <char *>(&command[0][3 + ID_SIZE]));
		return;
	}
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
		arch_reconnect(reinterpret_cast <char *>(&command[0][3]));
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
		arch_send_spi(command[0][3], &command[0][4]);
		return;
	}
	case CMD_ADJUSTPROBE:
	{
#ifdef DEBUG_CMD
		debug("CMD_ADJUSTPROBE");
#endif
		double pos[3];
		for (int i = 0; i < 3; ++i) {
			pos[i] = get_float(3 + i * sizeof(double));
		}
		run_adjust_probe(pos[0], pos[1], pos[2]);
		return;
	}
	case CMD_TP_GETPOS:
	{
#ifdef DEBUG_CMD
		debug("CMD_TP_GETPOS");
#endif
		// TODO: Send actual current position, not next queued.  Include fraction.
		send_host(CMD_TP_POS, 0, 0, settings.run_file_current);
		return;
	}
	case CMD_TP_SETPOS:
	{
#ifdef DEBUG_CMD
		debug("CMD_TP_SETPOS");
#endif
		double pos = get_float(3);
		int ipos = int(pos);
		if (ipos > 0 && ipos < run_file_num_records && (run_file_map[ipos - 1].type == RUN_PRE_ARC || run_file_map[ipos - 1].type == RUN_PRE_LINE))
			ipos -= 1;
		discarding = true;
		arch_discard();
		settings.run_file_current = int(pos);
		// Hack to force TP_GETPOS to return the same value; this is only called when paused, so it does no harm.
		history[running_fragment].run_file_current = int(pos);
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings.source = NAN;
		}
		// TODO: Use fraction.
		discarding = false;
		buffer_refill();
		return;
	}
	case CMD_TP_FINDPOS:
	{
#ifdef DEBUG_CMD
		debug("CMD_TP_FINDPOS");
#endif
		double pos[3];
		for (int i = 0; i < 3; ++i) {
			pos[i] = get_float(3 + i * sizeof(double));
		}
		send_host(CMD_TP_POS, 0, 0, run_find_pos(pos));
		return;
	}
	case CMD_MOTORS2XYZ:
	{
#ifdef DEBUG_CMD
		debug("CMD_MOTORS2XYZ");
#endif
		which = get_which();
		double motors[spaces[which].num_motors];
		double xyz[spaces[which].num_axes];
		for (int m = 0; m < spaces[which].num_motors; ++m)
			motors[m] = get_float(4 + m * sizeof(double));
		space_types[spaces[which].type].motors2xyz(&spaces[which], motors, xyz);
		for (int a = 0; a < spaces[which].num_axes; ++a)
			send_host(CMD_XYZ, which, a, xyz[a]);
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

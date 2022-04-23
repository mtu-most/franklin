/* packet.cpp - command packet handling for Franklin
 * Copyright 2014-2016 Michigan Technological University
 * Copyright 2016-2018 Bas Wijnen <wijnen@debian.org>
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

static void get_cb(bool value) {
	shmem->ints[1] = value ? 1 : 0;
	delayed_reply();
}

#if 0
#define CASE(x) case x: debug("request " # x " current fragment pos = %d", current_fragment_pos);
#define CASE2(x) case x: debug("request " # x);
#else
#define CASE(x) case x:
#define CASE2(x) case x:
#endif

void request(int req) {
	switch (req) {
	CASE(CMD_SET_UUID)
		arch_set_uuid();
		break;
	CASE(CMD_FORCE_DISCONNECT)
		disconnect(false, NULL);
		break;
	CASE(CMD_CONNECT)
		if (arch_fds() != 0) {
			debug("Unexpected connect");
			abort();
			return;
		}
		arch_connect(const_cast<const char *>(shmem->strs[0]), const_cast<const char *>(shmem->strs[1]));
		debug("Finished connecting");
		break;
	CASE(CMD_RECONNECT)
		if (arch_fds() != 0) {
			debug("Unexpected reconnect");
			abort();
			return;
		}
		arch_reconnect(const_cast<const char *>(shmem->strs[0]));
		break;
	CASE(CMD_MOVE)
		// Ignore move while stopping.
		if (stopping)
			break;
		// Ignore move while running.
		if (run_file_map != NULL && !pausing && !parkwaiting)
			break;
		//debug("moving to (%f,%f,%f), tool %d e %f v %f", shmem->move.target[0], shmem->move.target[1], shmem->move.target[2], shmem->move.tool, shmem->move.e, shmem->move.v0);
		last_active = millis();
		initialized = true;
		shmem->ints[1] = go_to(shmem->ints[0], const_cast <MoveCommand const *>(&shmem->move));
		if (!computing_move)
			cb_pending = true;
		delayed_reply();
		buffer_refill();
		return;
	CASE(CMD_RUN)
		last_active = millis();
		if (!run_file(const_cast<const char *>(shmem->strs[0]), shmem->ints[0], shmem->floats[0], shmem->floats[1]))
			delayed_reply();
		return;
	CASE(CMD_SLEEP)
		last_active = millis();
		if (shmem->ints[0]) {
			//debug("sleeping");
			if (arch_running() && !stop_pending)
			{
				if (shmem->ints[1]) {
					// Forced sleep; stop moving.
					abort_move(current_fragment_pos);
					arch_stop();
				}
				else {
					debug("Sleeping while moving");
					abort();
					break;
				}
			}
			for (int t = 0; t < NUM_SPACES; ++t) {
				for (int m = 0; m < spaces[t].num_motors; ++m) {
					//debug("resetting %d %d %x", t, m, spaces[t].motor[m]->enable_pin.write());
					RESET(spaces[t].motor[m]->enable_pin);
				}
				for (int a = 0; a < spaces[t].num_axes; ++a) {
					spaces[t].axis[a]->settings.source = NAN;
					spaces[t].axis[a]->current = NAN;
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
		break;
	CASE(CMD_SETTEMP)
		last_active = millis();
		settemp(shmem->ints[0], shmem->floats[0]);
		break;
	CASE(CMD_WAITTEMP)
		initialized = true;
		waittemp(shmem->ints[0], shmem->floats[0], shmem->floats[1]);
		break;
	CASE2(CMD_TEMP_VALUE)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= num_temps)
		{
			debug("Reading invalid temp %d", shmem->ints[0]);
			//abort();
			shmem->floats[0] = NAN;
			break;
		}
		if (!temps[shmem->ints[0]].thermistor_pin.valid()) {
			// Before first connection, NUM_PINS is 0; don't break on that.
			if (NUM_PINS > 0) {
				debug("Reading temp %d with invalid thermistor", shmem->ints[0]);
				//abort();
			}
			shmem->floats[0] = NAN;
			break;
		}
		arch_request_temp(shmem->ints[0]);
		return;
	CASE(CMD_POWER_VALUE)
	{
		if (shmem->ints[0] >= num_temps)
		{
			debug("Reading power of invalid temp %d", shmem->ints[0]);
			//abort();
			shmem->floats[0] = NAN;
			break;
		}
		int32_t t = utime();
		if (temps[shmem->ints[0]].is_on) {
			// This causes an insignificant error in the model, but when using this you probably aren't using the model anyway, and besides you won't notice the error even if you do.
			temps[shmem->ints[0]].time_on += t - temps[shmem->ints[0]].last_temp_time;
			temps[shmem->ints[0]].last_temp_time = t;
		}
		shmem->ints[1] = t;
		shmem->ints[2] = temps[shmem->ints[0]].time_on;
		temps[shmem->ints[0]].time_on = 0;
		break;
	}
	CASE(CMD_SETPOS)
		last_active = millis();
		if (shmem->ints[0] >= NUM_SPACES || shmem->ints[1] >= spaces[shmem->ints[0]].num_axes) {
			debug("Invalid axis for setting position: %d %d", shmem->ints[0], shmem->ints[1]);
			abort();
			return;
		}
		if (arch_running() && !stop_pending) {
			debug("Setting position while moving");
			//abort();
			break;
		}
		setpos(shmem->ints[0], shmem->ints[1], shmem->floats[0], true);
		break;
	CASE2(CMD_GETPOS)
		if (shmem->ints[0] >= NUM_SPACES || shmem->ints[1] >= spaces[shmem->ints[0]].num_axes) {
			debug("Getting position of invalid axis %d %d", shmem->ints[0], shmem->ints[1]);
			abort();
			return;
		}
		if (!motors_busy) {
			shmem->floats[0] = NAN;
			break;
		}
		if (std::isnan(spaces[shmem->ints[0]].axis[shmem->ints[1]]->current)) {
			reset_pos(&spaces[shmem->ints[0]]);
			for (int a = 0; a < spaces[shmem->ints[0]].num_axes; ++a) {
				//debug("setting %d %d source to %f for non-NaN.", shmem->ints[0], a, spaces[shmem->ints[0]].axis[a]->current);
				spaces[shmem->ints[0]].axis[a]->settings.source = spaces[shmem->ints[0]].axis[a]->current;
			}
		}
		shmem->floats[0] = spaces[shmem->ints[0]].axis[shmem->ints[1]]->current;
		shmem->floats[1] = spaces[shmem->ints[0]].motor[shmem->ints[1]]->settings.current_pos;
		//debug("getpos %d %d %f", shmem->ints[0], shmem->ints[1], shmem->floats[0]);
		break;
	CASE(CMD_READ_GLOBALS)
		globals_save();
		break;
	CASE(CMD_WRITE_GLOBALS)
		discard();
		globals_load();
		break;
	CASE(CMD_READ_SPACE_INFO)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= NUM_SPACES) {
			debug("invalid space for read info");
			abort();
			break;
		}
		spaces[shmem->ints[0]].save_info();
		break;
	CASE(CMD_READ_SPACE_AXIS)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= NUM_SPACES || shmem->ints[1] < 0 || shmem->ints[1] >= spaces[shmem->ints[0]].num_axes) {
			debug("Reading invalid axis %d %d", shmem->ints[0], shmem->ints[1]);
			abort();
			return;
		}
		spaces[shmem->ints[0]].save_axis(shmem->ints[1]);
		break;
	CASE(CMD_READ_SPACE_MOTOR)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= NUM_SPACES || shmem->ints[1] < 0 || shmem->ints[1] >= spaces[shmem->ints[0]].num_motors) {
			debug("Reading invalid motor %d %d", shmem->ints[0], shmem->ints[1]);
			abort();
			return;
		}
		spaces[shmem->ints[0]].save_motor(shmem->ints[1]);
		break;
	CASE(CMD_WRITE_SPACE_INFO)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= NUM_SPACES) {
			debug("Writing invalid space %d", shmem->ints[0]);
			abort();
			return;
		}
		discard();
		spaces[shmem->ints[0]].load_info();
		break;
	CASE(CMD_WRITE_SPACE_AXIS)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= NUM_SPACES || shmem->ints[1] < 0 || shmem->ints[1] >= spaces[shmem->ints[0]].num_axes) {
			debug("Writing invalid axis %d %d", shmem->ints[0], shmem->ints[1]);
			abort();
			return;
		}
		discard();
		spaces[shmem->ints[0]].load_axis(shmem->ints[1]);
		break;
	CASE(CMD_WRITE_SPACE_MOTOR)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= NUM_SPACES || shmem->ints[1] < 0 || shmem->ints[1] >= spaces[shmem->ints[0]].num_motors) {
			debug("Writing invalid motor %d %d", shmem->ints[0], shmem->ints[1]);
			abort();
			return;
		}
		discard();
		spaces[shmem->ints[0]].load_motor(shmem->ints[1]);
		break;
	CASE(CMD_READ_TEMP)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= num_temps) {
			debug("Reading invalid temp %d", shmem->ints[0]);
			abort();
			return;
		}
		temps[shmem->ints[0]].save();
		break;
	CASE(CMD_WRITE_TEMP)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= num_temps) {
			debug("Writing invalid temp %d", shmem->ints[0]);
			abort();
			return;
		}
		temps[shmem->ints[0]].load(shmem->ints[0]);
		break;
	CASE(CMD_READ_GPIO)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= num_gpios) {
			debug("Reading invalid gpio %d", shmem->ints[0]);
			abort();
			return;
		}
		gpios[shmem->ints[0]].save();
		break;
	CASE(CMD_WRITE_GPIO)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= num_gpios) {
			debug("Writing invalid gpio %d", shmem->ints[0]);
			abort();
			return;
		}
		gpios[shmem->ints[0]].load();
		break;
	CASE(CMD_QUEUED)
		last_active = millis();
		shmem->ints[1] = settings.queue_end - settings.queue_start;
		break;
	CASE(CMD_HOME)
		arch_home();
		break;
	CASE(CMD_PIN_VALUE)
		if (shmem->ints[0] >= num_gpios)
		{
			debug("Reading invalid gpio %d", shmem->ints[0]);
			abort();
			return;
		}
		GET(gpios[shmem->ints[0]].pin, false, get_cb);
		return;
	CASE(CMD_PAUSE)
	{
		if (!motors_busy)
			break;
		bool store = !pausing && run_file_wait == 0;
		pausing = true;
		if (run_file_map)
			run_file_wait += 1;
		// Store resume info.
		double x[6], v[6], a[6];
		bool done = compute_current_pos(x, v, a, store);
		if (store) {
			for (int i = 0; i < 6; ++i) {
				resume.x[i] = x[i];
				resume.v[i] = v[i];
				resume.a[i] = a[i];
			}
			//debug("storing resume settings in fragment %d, times: %d, %d", current_fragment, settings.hwtime, settings.end_time);
			memcpy(&resume.settings, &settings, sizeof(History));
		}
		if (done) {
			break;
		}
		// Bring a to 0.
		int q = prepare_retarget(0, -1, x, v, a);
		smooth_stop(q, x, v);
		break;
	}
	CASE(CMD_RESUME)
		if (!run_file_map)
			break;
		else if (computing_move)
			break;
		else if (pausing)
			do_resume();
		else if (parkwaiting) {
			parkwaiting = false;
			if (run_file_wait > 0)
				run_file_wait -= 1;
			run_file_next_command(settings.hwtime);
		}
		buffer_refill();
		break;
	CASE(CMD_UNPAUSE)
		pausing = false;
		break;
	CASE2(CMD_GET_TIME)
		shmem->floats[0] = history[running_fragment].run_time / feedrate + settings.hwtime / 1e6;
		break;
	CASE(CMD_SPI)
		arch_send_spi(shmem->ints[0], reinterpret_cast<const uint8_t *>(const_cast<const char *>(shmem->strs[0])));
		break;
	CASE2(CMD_TP_GETPOS)
		shmem->floats[0] = history[running_fragment].run_file_current + (history[running_fragment].hwtime / 1e6) / (history[running_fragment].end_time / 1e6);
		break;
	CASE(CMD_TP_SETPOS)
	{
		int ipos = int(shmem->floats[0]);
		discard();
		settings.run_file_current = ipos;
		// Hack to force TP_GETPOS to return the same value; this is only called when paused, so it does no harm.
		history[running_fragment].run_file_current = ipos;
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings.source = NAN;
		}
		// TODO: Use fraction.
		break;
	}
	CASE(CMD_TP_FINDPOS)
		shmem->floats[3] = run_find_pos(const_cast<const double *>(shmem->floats));
		break;
	CASE(CMD_MOTORS2XYZ)
		spaces[0].motors2xyz(const_cast<const double *>(shmem->floats), const_cast<double *>(&shmem->floats[shmem->ints[0]]));
		break;
	CASE(CMD_WRITE_PROBE_MAP)
	{
		if (shmem->ints[1] != probe_nx || shmem->ints[2] != probe_ny) {
			// Size changed; reallocate storage.
			if (probe_data)
				delete[] probe_data;
			probe_nx = shmem->ints[1];
			probe_ny = shmem->ints[2];
			if (probe_nx * probe_ny > 0)
				probe_data = new double[probe_nx * probe_ny];
			else
				probe_data = NULL;
		}
		int base = shmem->ints[0];
		for (int n = base; n < base + 400 && n < probe_nx * probe_ny; ++n) {
			int y = n / probe_nx;
			int x = n % probe_nx;
			probe_data[y * probe_nx + x] = shmem->floats[100 + n - base];
		}
		break;
	}
	CASE(CMD_READ_PROBE_MAP)
	{
		int base = shmem->ints[0];
		shmem->ints[1] = probe_nx;
		shmem->ints[2] = probe_ny;
		shmem->floats[0] = probe_origin[0];
		shmem->floats[1] = probe_origin[1];
		shmem->floats[2] = probe_step[0];
		shmem->floats[3] = probe_step[1];
		for (int n = base; n < base + 400 && n < probe_nx * probe_ny; ++n) {
			int y = n / probe_nx;
			int x = n % probe_nx;
			shmem->floats[100 + n - base] = probe_data[y * probe_nx + x];
		}
		break;
	}
	default:
		debug("unknown packet received: %x", req);
	}
	// Not delayed, but use the same system.
	delayed_reply();
}

void delayed_reply() {
	char cmd = 0;
	if (write(toserver, &cmd, 1) != 1) {
		debug("failed to send delayed reply");
		abort();
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

void settemp(int which, double target) {
	if (which < 0 || which >= num_temps)
	{
		debug("Setting invalid temp %d", which);
		//abort();
		return;
	}
	temps[which].target[0] = target;
	// Set target. Add 10% if hold_time == 0 and PID is set up.
	temps[which].adctarget[0] = temps[which].toadc(target * (temps[which].hold_time > 0 && !std::isinf(temps[which].P) ? 1 : 1.02), MAXINT);
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
	if (!std::isnan(temps[which].min_alarm))
		waittemp(which, temps[which].target[0], temps[which].max_alarm);
}

void setpos(int which, int t, double f, bool reset) {
	if (!motors_busy) {
		//debug("Error: Setting position while motors are not busy!");
		for (int s = 0; s < NUM_SPACES; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m)
				SET(spaces[s].motor[m]->enable_pin);
		}
		motors_busy = true;
	}
	//debug("setting pos for %d %d to %f", which, t, f);
	double old = spaces[which].motor[t]->settings.current_pos;
	if (std::isnan(old))
		old = 0;
	spaces[which].motor[t]->settings.current_pos = f;
	arch_addpos(which, t, f - old);
	//arch_stop();
	if (reset) {
		reset_pos(&spaces[which]);
		for (int a = 0; a < spaces[which].num_axes; ++a) {
			//debug("setting source and endpos %d %d to %f for setpos", which, a, spaces[which].axis[a]->current);
			spaces[which].axis[a]->settings.source = spaces[which].axis[a]->current;
			spaces[which].axis[a]->settings.endpos = spaces[which].axis[a]->current;
		}
	}
	cpdebug(which, t, "setpos new %f old %f diff %f", f, old, f - old);
	// */
}

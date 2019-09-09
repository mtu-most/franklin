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

#define CASE(x) case x: //debug("request " # x);

int go_to(bool relative, MoveCommand const *move, bool cb) {
	//debug("goto (%f,%f,%f)->(%f,%f,%f) at speed %f, e %f->%f", spaces[0].axis[0]->settings.current, spaces[0].axis[1]->settings.current, spaces[0].axis[2]->settings.current, move->target[0], move->target[1], move->target[2], move->v0, spaces[1].axis[move->tool]->settings.current, move->e);
	// This is a manual move or the start of a job; set hwtime step to default.
	settings.hwtime_step = default_hwtime_step;
	if (computing_move || settings.queue_full || settings.queue_end != settings.queue_start) {
		// This is not allowed; signal error to Python glue.
		debug("move error %d %d %d %d", computing_move, settings.queue_full, settings.queue_start, settings.queue_end);
		return 1;
	}
	settings.queue_start = 0;
	settings.queue_end = 3;
	double vmax = NAN;
	double amax = NAN;
	double dist = NAN;
	double unit[3] = {0, 0, 0};
	double target[3];
	for (int a = 0; a < 3; ++a) {
		if (spaces[0].num_axes <= a)
			break;
		double pos = spaces[0].axis[a]->settings.current;
		//debug("prepare move, pos[%d]: %f -> %f", a, pos, move->target[a]);
		if (std::isnan(pos))
			continue;
		if (std::isnan(move->target[a]))
			target[a] = (relative ? 0 : pos) - (a == 2 ? zoffset : 0);
		else
			target[a] = move->target[a];
		double d = target[a] + (a == 2 ? zoffset : 0) - (relative ? 0 : pos);
		unit[a] = d;
		if (std::isnan(dist))
			dist = d * d;
		else
			dist += d * d;
	}
	if (!std::isnan(dist) && dist >= 1e-10) {
		dist = std::sqrt(dist);
		vmax = max_v;
		amax = max_a;
		for (int i = 0; i < 3; ++i)
			unit[i] /= dist;
		//debug("dist = %f", dist);
	}
	else {
		for (int i = 0; i < 3; ++i)
			unit[i] = 0;
		for (int a = 3; a < 6; ++a) {
			if (a >= spaces[0].num_axes)
				break;
			double pos = spaces[0].axis[a]->settings.current;
			if (std::isnan(pos))
				continue;
			double d = std::fabs(target[a] - (relative ? 0 : pos));
			if (std::isnan(dist) || dist < d) {
				dist = d;
				vmax = spaces[0].motor[a]->limit_v;
				amax = spaces[0].motor[a]->limit_a;
			}
		}
	}
	int tool = std::isnan(move->tool) ? current_extruder : move->tool;
	double e;
	if (tool >= 0 && tool < spaces[1].num_axes)
		e = spaces[1].axis[tool]->settings.current;
	else if (move->single && tool < 0 && ~tool < spaces[2].num_axes)
		e = spaces[2].axis[~tool]->settings.current;
	else
		e = NAN;
	if (!std::isnan(e)) {
		if (std::isnan(dist) || dist < 1e-10) {
			dist = std::fabs(move->e - (relative ? 0 : e));
			if (tool >= 0) {
				vmax = spaces[1].motor[tool]->limit_v;
				amax = spaces[1].motor[tool]->limit_a;
			}
			else {
				// use leader limits with fallback to global limits.
				int sm = space_types[spaces[2].type].follow(&spaces[2], ~tool);
				int fs = sm >> 8;
				int fm = sm & 0xff;
				if (fs >= 0 && fs < NUM_SPACES && fm >= 0 && fm < spaces[fs].num_motors) {
					vmax = spaces[fs].motor[fm]->limit_v;
					amax = spaces[fs].motor[fm]->limit_a;
				}
				else {
					vmax = max_v;
					amax = max_a;
				}
			}
		}
	}
	if (std::isnan(dist) || dist < 1e-10) {
		// No moves requested.
		//debug("dist is %f", dist);
		if (cb)
			num_movecbs += 1;
		//debug("adding 1 move cb for manual move");
		settings.queue_end = 0;
		return 1;
	}

	double reachable_v = compute_max_v(dist / 2, 0, max_J, amax);
	//debug("vmax = %f, reachable = %f, dist=%f", vmax, reachable_v, dist);
	vmax = min(vmax, reachable_v);
	double max_ramp_dv = amax * amax / (2 * max_J);
	// Initialize the queue. At most 7 items will be used.
	for (int i = 0; i < 7; ++i) {
		queue[i].cb = false;
		queue[i].probe = false;
		queue[i].single = false;
		queue[i].reverse = false;
		queue[i].tool = move->tool;
		for (int j = 0; j < 3; ++j) {
			queue[i].h[j] = 0;
			queue[i].unitg[j] = unit[j];
			queue[i].unith[j] = 0;
			queue[i].abc[j] = 0;
		}
		queue[i].Jh = 0;
		queue[i].time = move->time;
		queue[i].gcode_line = -1;
		queue[i].pattern_size = 0;
	}
	double dv_ramp, t_max_a;
	if (max_ramp_dv * 2 >= vmax) {
		// ramp up, ramp down, const v, ramp down, ramp up.
		dv_ramp = vmax / 2;
		t_max_a = 0;
	}
	else {
		// ramp up, const a, ramp down, const v, ramp down, const a, ramp up.
		dv_ramp = max_ramp_dv;
		t_max_a = (vmax - 2 * dv_ramp) / amax;
	}
	int q = 0;
	double t_ramp = std::sqrt(2 * dv_ramp / max_J);
	double s_ramp_up = max_J / 6 * t_ramp * t_ramp * t_ramp;
	double s_ramp_down = -max_J / 6 * t_ramp * t_ramp * t_ramp + vmax * t_ramp;
	double s_const_a = amax / 2 * t_max_a * t_max_a;
	double s_const_v = dist - 2 * (s_ramp_up + s_ramp_down + s_const_a);
	double t_const_v = s_const_v / vmax;

	double s[7] = {s_ramp_up, s_const_a, s_ramp_down, s_const_v, s_ramp_down, s_const_a, s_ramp_up};
	double t[7] = {t_ramp, t_max_a, t_ramp, t_const_v, t_ramp, t_max_a, t_ramp};
	double J[7] =     {max_J,       0, -max_J,     0, -max_J,              0, max_J};
	double v0[7] =    {    0, dv_ramp,   vmax,  vmax,   vmax, vmax - dv_ramp,     0};
	double a0[7] =    {    0,    amax,      0,     0,      0,          -amax,     0};
	bool reverse[7] = {false,   false,   true, false,  false,          false,  true};
	double X[3];
	for (int i = 0; i < 3; ++i)
		X[i] = i < spaces[0].num_axes ? spaces[0].axis[i]->settings.current : 0;
	double current_s = 0;
	for (int part = 0; part < 7; ++part) {
		current_s += s[part];
		if (s[part] < 1e-10)
			continue;
		for (int i = 0; i < 3; ++i) {
			X[i] += unit[i] * s[part];
			queue[q].target[i] = X[i];
		}
		//debug("target %d: %f,%f,%f", part, X[0], X[1], X[2]);
		queue[q].Jg = J[part];
		queue[q].tf = t[part];
		queue[q].v0 = v0[part];
		queue[q].a0 = a0[part];
		queue[q].reverse = reverse[part];
		queue[q].e = move->e * current_s / dist;
		q += 1;
	}
	settings.queue_end = q;

	if (cb) {
		queue[q - 1].cb = true;
		//debug("adding cb for queue %d", q - 1);
	}
#if 0
	debug("goto dir=%f,%f,%f, dist=%f, tool=%d e=%f single=%d", unit[0], unit[1], unit[2], dist, tool, move->e, move->single);
	debug("goto s %f,%f,%f,%f,%f,%f,%f", s[0], s[1], s[2], s[3], s[4], s[5], s[6]);
	debug("goto t %f,%f,%f,%f,%f,%f,%f", t[0], t[1], t[2], t[3], t[4], t[5], t[6]);
	debug("goto J %f,%f,%f,%f,%f,%f,%f", J[0], J[1], J[2], J[3], J[4], J[5], J[6]);
	debug("goto v0 %f,%f,%f,%f,%f,%f,%f", v0[0], v0[1], v0[2], v0[3], v0[4], v0[5], v0[6]);
#endif
	int new_num_movecbs = next_move(settings.hwtime);
	bool ret = 0;
	if (new_num_movecbs > 0) {
		if (arch_running()) {
			cbs_after_current_move += new_num_movecbs;
			//debug("adding %d cbs after current move to %d", new_num_movecbs, cbs_after_current_move);
		}
		else {
			ret = 1;
			num_movecbs += new_num_movecbs;
			//debug("adding %d cbs because move is immediately done", new_num_movecbs);
			//debug("sent immediate %d cbs", new_num_movecbs);
		}
	}
	//debug("no movecbs to add (prev %d)", history[(current_fragment - 1 + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER].cbs);
	buffer_refill();
	return ret;
}

void request(int req) {
	switch (req) {
	CASE(CMD_SET_UUID)
		arch_set_uuid();
		break;
	CASE(CMD_FORCE_DISCONNECT)
		disconnect(false);
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
		//debug("moving to (%f,%f,%f)", shmem->move.g[0], shmem->move.g[1], shmem->move.g[2]);
		last_active = millis();
		initialized = true;
		shmem->ints[1] = go_to(shmem->ints[0], const_cast <MoveCommand const *>(&shmem->move), true);
		break;
	CASE(CMD_RUN)
		last_active = millis();
		run_file(const_cast<const char *>(shmem->strs[0]), const_cast<const char *>(shmem->strs[1]), shmem->ints[0], shmem->floats[0], shmem->floats[1]);
		break;
	CASE(CMD_SLEEP)
		last_active = millis();
		if (shmem->ints[0]) {
			//debug("sleeping");
			if (arch_running() && !stop_pending)
			{
				debug("Sleeping while moving");
				abort();
				break;
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
		break;
	CASE(CMD_SETTEMP)
		last_active = millis();
		settemp(shmem->ints[0], shmem->floats[0]);
		break;
	CASE(CMD_WAITTEMP)
		initialized = true;
		waittemp(shmem->ints[0], shmem->floats[0], shmem->floats[1]);
		break;
	CASE(CMD_TEMP_VALUE)
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
		setpos(shmem->ints[0], shmem->ints[1], shmem->floats[0]);
		break;
	CASE(CMD_GETPOS)
		if (shmem->ints[0] >= NUM_SPACES || shmem->ints[1] >= spaces[shmem->ints[0]].num_axes) {
			debug("Getting position of invalid axis %d %d", shmem->ints[0], shmem->ints[1]);
			abort();
			return;
		}
		if (!motors_busy) {
			shmem->floats[0] = NAN;
			break;
		}
		if (std::isnan(spaces[shmem->ints[0]].axis[shmem->ints[1]]->settings.current)) {
			reset_pos(&spaces[shmem->ints[0]]);
			for (int a = 0; a < spaces[shmem->ints[0]].num_axes; ++a)
				spaces[shmem->ints[0]].axis[a]->settings.current = spaces[shmem->ints[0]].axis[a]->settings.source;
		}
		shmem->floats[0] = spaces[shmem->ints[0]].axis[shmem->ints[1]]->settings.current;
		//debug("getpos %d %d %f", shmem->ints[0], shmem->ints[1], shmem->floats[0]);
		if (shmem->ints[0] == 0) {
			for (int s = 0; s < NUM_SPACES; ++s) {
				shmem->floats[0] = space_types[spaces[s].type].unchange0(&spaces[s], shmem->ints[1], shmem->floats[0]);
			}
			if (shmem->ints[1] == 2)
				shmem->floats[0] -= zoffset;
		}
		break;
	CASE(CMD_READ_GLOBALS)
		globals_save();
		break;
	CASE(CMD_WRITE_GLOBALS)
		discarding = true;
		arch_discard();
		globals_load();
		discarding = false;
		buffer_refill();
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
		discarding = true;
		arch_discard();
		spaces[shmem->ints[0]].load_info();
		discarding = false;
		buffer_refill();
		break;
	CASE(CMD_WRITE_SPACE_AXIS)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= NUM_SPACES || shmem->ints[1] < 0 || shmem->ints[1] >= spaces[shmem->ints[0]].num_axes) {
			debug("Writing invalid axis %d %d", shmem->ints[0], shmem->ints[1]);
			abort();
			return;
		}
		discarding = true;
		arch_discard();
		spaces[shmem->ints[0]].load_axis(shmem->ints[1]);
		discarding = false;
		buffer_refill();
		break;
	CASE(CMD_WRITE_SPACE_MOTOR)
		if (shmem->ints[0] < 0 || shmem->ints[0] >= NUM_SPACES || shmem->ints[1] < 0 || shmem->ints[1] >= spaces[shmem->ints[0]].num_motors) {
			debug("Writing invalid motor %d %d", shmem->ints[0], shmem->ints[1]);
			abort();
			return;
		}
		discarding = true;
		arch_discard();
		spaces[shmem->ints[0]].load_motor(shmem->ints[1]);
		discarding = false;
		buffer_refill();
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
		shmem->ints[1] = settings.queue_full ? QUEUE_LENGTH : (settings.queue_end - settings.queue_start + QUEUE_LENGTH) % QUEUE_LENGTH;
		if (shmem->ints[0]) {
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
	CASE(CMD_RESUME)
		if (run_file_wait)
			run_file_wait -= 1;
		run_file_fill_queue();
		break;
	CASE(CMD_GET_TIME)
		shmem->floats[0] = history[running_fragment].run_time / feedrate + settings.hwtime / 1e6;
		break;
	CASE(CMD_SPI)
		arch_send_spi(shmem->ints[0], reinterpret_cast<const uint8_t *>(const_cast<const char *>(shmem->strs[0])));
		break;
	CASE(CMD_ADJUST_PROBE)
		run_adjust_probe(shmem->floats[0], shmem->floats[1], shmem->floats[2]);
		break;
	CASE(CMD_TP_GETPOS)
		// TODO: Send actual current position, not next queued.  Include fraction.
		shmem->floats[0] = settings.run_file_current;
		break;
	CASE(CMD_TP_SETPOS)
	{
		int ipos = int(shmem->floats[0]);
		discarding = true;
		arch_discard();
		settings.run_file_current = ipos;
		// Hack to force TP_GETPOS to return the same value; this is only called when paused, so it does no harm.
		history[running_fragment].run_file_current = ipos;
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings.source = NAN;
		}
		// TODO: Use fraction.
		discarding = false;
		buffer_refill();
		break;
	}
	CASE(CMD_TP_FINDPOS)
		shmem->floats[3] = run_find_pos(const_cast<const double *>(shmem->floats));
		break;
	CASE(CMD_MOTORS2XYZ)
		space_types[spaces[shmem->ints[0]].type].motors2xyz(&spaces[shmem->ints[0]], const_cast<const double *>(shmem->floats), const_cast<double *>(&shmem->floats[shmem->ints[1]]));
		break;
	default:
		debug("unknown packet received: %x", req);
	}
	// Not delayed, but use the same system.
	delayed_reply();
}

void delayed_reply() {
	char cmd = 0;
	if (write(toserver, &cmd, 1) != 1)
		abort();
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
	if (!motors_busy) {
		//debug("Error: Setting position while motors are not busy!");
		for (int s = 0; s < NUM_SPACES; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m)
				SET(spaces[s].motor[m]->enable_pin);
		}
		motors_busy = true;
	}
	for (int a = 0; a < spaces[which].num_axes; ++a) {
		spaces[which].axis[a]->settings.source = NAN;
		spaces[which].axis[a]->settings.current = NAN;
	}
	//debug("setting pos for %d %d to %f", which, t, f);
	double old = spaces[which].motor[t]->settings.current_pos;
	if (std::isnan(old))
		old = 0;
	spaces[which].motor[t]->settings.current_pos = f;
	arch_addpos(which, t, f - old);
	//arch_stop();
	reset_pos(&spaces[which]);
	for (int a = 0; a < spaces[which].num_axes; ++a) {
		spaces[which].axis[a]->settings.current = spaces[which].axis[a]->settings.source;
		spaces[which].axis[a]->settings.endpos = spaces[which].axis[a]->settings.source;
	}
	cpdebug(which, t, "setpos new %f old %f diff %f", f, old, f - old);
	// */
}

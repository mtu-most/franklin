/* move.cpp - movement planning for Franklin {{{
 * vim: set foldmethod=marker :
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
 * }}} */

#include "cdriver.h"

//#define mdebug(...) debug(__VA_ARGS__)
#define debug_abort() abort()

#ifndef mdebug
#define mdebug(...) do {} while (0)
#endif

#ifndef debug_abort
#define debug_abort() do {} while(0)
#endif

#define warning debug

static void send_fragment() { // {{{
	if (host_block) {
		current_fragment_pos = 0;
		return;
	}
	if (current_fragment_pos <= 0 || stopping || sending_fragment) {
		//debug("no send fragment %d %d %d", current_fragment_pos, stopping, sending_fragment);
		return;
	}
	if (num_active_motors == 0) {
		if (current_fragment_pos < 1) {
			// TODO: find out why this is attempted and avoid it.
			warning("not sending short fragment for 0 motors; %d %d", current_fragment, running_fragment);
			// TODO: Maybe the move cb is not sent in this case?
			current_fragment_pos = 0;
			return;
		}
		else {
			//debug("sending fragment for 0 motors at position %d", current_fragment_pos);
		}
	}
	//debug("sending %d", current_fragment);
	if (aborting || arch_send_fragment()) {
		current_fragment = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
		current_fragment_pos = 0;
		//debug("current_fragment = (current_fragment + 1) %% FRAGMENTS_PER_BUFFER; %d", current_fragment);
		//debug("current send -> %x", current_fragment);
		if (!aborting && (current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER >= MIN_BUFFER_FILL && !stopping) {
			arch_start_move(0);
		}
	}
	else {
		// Reset current_fragment_pos anyway, otherwise store_settings will complain.
		current_fragment_pos = 0;
	}
	while (sending_fragment) {
		//debug("waiting for sending fragment to be 0 from %d", sending_fragment);
		serial_wait();
	}
	store_settings();
} // }}}

// For documentation about variables used here, see struct History in cdriver.h
void next_move(int32_t start_time) { // {{{
	if (stopping) {
		//debug("ignoring move while stopping");
		return;
	}
	//debug("next move, computing=%d, start time=%d, current time=%d, queue %d -> %d", computing_move, start_time, settings.hwtime, settings.queue_start, settings.queue_end);
	factor = 0;
	if (settings.queue_start == settings.queue_end) {
		if (resume_pending) {
			memcpy(&settings, &resume.settings, sizeof(History));
			factor = (settings.hwtime / 1e6) / (settings.end_time / 1e6);
			if (factor < 0)
				factor = 0;
			if (factor > 1)
				factor = 1;
			for (int s = 0; s < NUM_SPACES; ++s) {
				for (int a = 0; a < spaces[s].num_axes; ++a) {
					Axis *ax = spaces[s].axis[a];
					mdebug("settings source for %d to %.2f for resume", a, ax->resume_start);
					ax->settings.source = ax->resume_start;
					ax->settings.endpos = ax->resume_end;
					if (s == 1)
						setpos(s, a, ax->settings.source + factor * (ax->settings.endpos - ax->settings.source), false);
				}
			}
			reset_pos(&spaces[1]);
			computing_move = true;
			resume_pending = false;
			mdebug("finished resume. axes = %f,%f,%f motors = %f,%f,%f, extruder[0] = %f", spaces[0].axis[0]->current, spaces[0].axis[1]->current, spaces[0].axis[2]->current, spaces[0].motor[0]->settings.current_pos, spaces[0].motor[1]->settings.current_pos, spaces[0].motor[2]->settings.current_pos, spaces[1].axis[0]->current);
			return;
		}
		//debug("no next move");
		computing_move = false;
		return;
	}
	// Clean up state before starting the move (but not if there is no next move; in that case finish the current tick before sending).
	if (current_fragment_pos > 0) {
		//debug("sending because new move is started");
		while (sending_fragment)
			serial_wait();
		if (host_block || stopping || discard_pending || stop_pending) {
			//debug("not moving yet");
			return;
		}
		send_fragment();
		if (host_block || stopping || discard_pending || stop_pending) {
			//debug("not moving more");
			return;
		}
	}
	//debug("Next move; queue start = %d, end = %d", settings.queue_start, settings.queue_end);
	// Set everything up for running queue[settings.queue_start].
	int q = settings.queue_start;
	single = queue[q].single;
	probing = queue[q].probe;
	settings.gcode_line = queue[q].gcode_line;

	// Make sure machine state is good. {{{
	// If the source is unknown, determine it from current_pos.
	for (int s = 0; s < NUM_SPACES; ++s) {
		if (s == 1) {
			// Extruders are handled later.
			continue;
		}
		Space &sp = spaces[s];
		for (int a = 0; a < sp.num_axes; ++a) {
			if (std::isnan(sp.axis[a]->settings.source)) {
				if (!std::isnan(sp.axis[a]->current)) {
					mdebug("settings source for %d to %.2f for non-move", a, sp.axis[a]->current);
					sp.axis[a]->settings.source = sp.axis[a]->current;
					continue;
				}
				reset_pos(&sp);
				for (int aa = 0; aa < sp.num_axes; ++aa) {
					mdebug("settings source for %d %d to %.2f for non-move after reset", s, aa, sp.axis[aa]->current);
					sp.axis[aa]->settings.source = sp.axis[aa]->current;
				}
				break;
			}
			else
				mdebug("non-nan: %d %d %f %f", s, a, sp.axis[a]->settings.source, sp.motor[a]->settings.current_pos);
		}
		// Followers don't have good motor data, so initialize them here.
		for (int a = 0; a < sp.num_axes; ++a) {
			if (s == 2)
				sp.motor[a]->settings.current_pos = sp.axis[a]->settings.source;
		}
	}
	// }}}

	// Fill unspecified coordinates with previous values. {{{
	Space &sp0 = spaces[0];
	for (int a = 0; a < 3; ++a) {
		if (a >= sp0.num_axes)
			continue;
		if ((!std::isnan(queue[q].target[a]) || (q + 1 != settings.queue_end && !std::isnan(queue[q + 1].target[a]))) && std::isnan(sp0.axis[a]->settings.source)) {
			warning("Motor position for axis %d is not known, so move cannot take place; aborting move and removing it from the queue: q1=%f q2=%f src=%f", a, queue[q].target[a], queue[q + 1].target[a], sp0.axis[a]->settings.source);
			// This possibly removes one move too many, but it shouldn't happen anyway.
			settings.queue_start += 1;
			abort_move(current_fragment_pos);
			return;
		}
		// If position is NaN, don't move.
		if (std::isnan(queue[q].target[a]))
			queue[q].target[a] = sp0.axis[a]->settings.source;
	}
	// }}}
	// Reset time. {{{
	if (computing_move) {
		settings.adjust_start_time -= start_time;
		settings.hwtime -= start_time;
		//debug("time -= %d, now %d", start_time, settings.hwtime);
	}
	else {
		settings.adjust_start_time -= settings.hwtime;
		settings.hwtime = 0;
	}
	// }}}

	if (!computing_move) {	// Set up source if this is a new move. {{{
		mdebug("starting new move");
		//debug("current %d running %d", current_fragment, running_fragment);
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a) {
				if (!std::isnan(sp.axis[a]->current)) {
					mdebug("setting source for %d %d to current %f (was %f)", s, a, sp.axis[a]->current, sp.axis[a]->settings.source);
					sp.axis[a]->settings.source = sp.axis[a]->current;
				}
			}
		}
	} // }}}

	// TODO: use feedrate.
	settings.end_time = queue[q].tf * 1e6;
	settings.run_time = queue[q].time;
	settings.Jh = 0;
	double leng = 0;
	for (int i = 0; i < 3; ++i) {
		if (i >= sp0.num_axes) {
			settings.unitg[i] = 0;
			settings.unith[i] = 0;
			continue;
		}
		settings.unitg[i] = queue[q].target[i] - spaces[0].axis[i]->settings.source;
		settings.unith[i] = queue[q].unith[i];
		leng += settings.unitg[i] * settings.unitg[i];
	}
	settings.Jh = queue[q].Jh;
	leng = std::sqrt(leng);
	for (int i = 0; i < 3; ++i) {
		settings.unitg[i] /= leng;
	}
	double t = settings.end_time / 1e6;
	double t2 = t * t;
	double t3 = t2 * t;
	if (queue[q].reverse) {
		//debug("set reverse from J %f v %f", queue[q].Jg, queue[q].v0);
		settings.x0h = queue[q].Jh / 6 * t3;
		settings.v0h = -queue[q].Jh / 2 * t2;
		settings.a0h = queue[q].Jh * t;
		settings.Jh = -queue[q].Jh;
		settings.x0g = leng - (queue[q].Jg / 6 * t3 + queue[q].v0 * t);
		settings.v0g = queue[q].v0 + queue[q].Jg / 2 * t2;
		settings.a0g = -queue[q].Jg * t;
		settings.Jg = queue[q].Jg;
	}
	else {
		settings.Jg = queue[q].Jg;
		settings.a0g = queue[q].a0;
		settings.v0g = queue[q].v0;
		settings.x0g = 0;
		settings.a0h = 0;
		settings.v0h = 0;
		settings.x0h = 0;
	}

	// Info for 3-D robot
	if (spaces[0].num_axes >= 3)
		mdebug("move ln %" LONGFMT ", from=(%.2f,%.2f,%.2f) (current %.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f), g=(%.2f,%.2f,%.2f) h=(%.2f,%.2f,%.2f), e=%.2f, Jg=%.2f a0g=%.2f v0g=%.2f x0g=%.2f end time=%.4f, single=%d, Jh=%.2f, a0h=%.2f, v0h=%.2f, x0h=%.2f", settings.gcode_line, spaces[0].axis[0]->settings.source, spaces[0].axis[1]->settings.source, spaces[0].axis[2]->settings.source, spaces[0].axis[0]->current, spaces[0].axis[1]->current, spaces[0].axis[2]->current, queue[q].target[0], queue[q].target[1], queue[q].target[2], settings.unitg[0], settings.unitg[1], settings.unitg[2], settings.unith[0], settings.unith[1], settings.unith[2], queue[q].e, settings.Jg, settings.a0g, settings.v0g, settings.x0g, settings.end_time / 1e6, queue[q].single, settings.Jh, settings.a0h, settings.v0h, settings.x0h);
	// Info for 2-D robot
	else if (spaces[0].num_axes >= 2)
		mdebug("move ln %" LONGFMT ", from=(%.2f,%.2f) (current %.2f,%.2f) target=(%.2f,%.2f,%.2f), g=(%.2f,%.2f,%.2f) h=(%.2f,%.2f,%.2f), e=%.2f, Jg=%.2f a0g=%.2f v0g=%.2f x0g=%.2f end time=%.4f, single=%d, Jh=%.2f, a0h=%.2f, v0h=%.2f, x0h=%.2f", settings.gcode_line, spaces[0].axis[0]->settings.source, spaces[0].axis[1]->settings.source, spaces[0].axis[0]->current, spaces[0].axis[1]->current, queue[q].target[0], queue[q].target[1], queue[q].target[2], settings.unitg[0], settings.unitg[1], settings.unitg[2], settings.unith[0], settings.unith[1], settings.unith[2], queue[q].e, settings.Jg, settings.a0g, settings.v0g, settings.x0g, settings.end_time / 1e6, queue[q].single, settings.Jh, settings.a0h, settings.v0h, settings.x0h);
	else if (spaces[0].num_axes >= 1)
		mdebug("move ln %" LONGFMT ", from=%.2f (current %.2f) target=(%.2f,%.2f,%.2f), g=(%.2f,%.2f,%.2f) h=(%.2f,%.2f,%.2f), e=%.2f, Jg=%.2f a0g=%.2f v0g=%.2f x0g=%.2f end time=%.4f, single=%d, Jh=%.2f, a0h=%.2f, v0h=%.2f, x0h=%.2f", settings.gcode_line, spaces[0].axis[0]->settings.source, spaces[0].axis[0]->current, queue[q].target[0], queue[q].target[1], queue[q].target[2], settings.unitg[0], settings.unitg[1], settings.unitg[2], settings.unith[0], settings.unith[1], settings.unith[2], queue[q].e, settings.Jg, settings.a0g, settings.v0g, settings.x0g, settings.end_time / 1e6, queue[q].single, settings.Jh, settings.a0h, settings.v0h, settings.x0h);
	// Short info for 2-D robot
	//mdebug("move (%.2f,%.2f) -> (%.2f,%.2f)", spaces[0].axis[0]->settings.source, spaces[0].axis[1]->settings.source, queue[q].target[0], queue[q].target[1]);

	int const na = spaces[0].num_axes;
	double check_x = 0, check_v = 0, check_a = 0;
	for (int i = 0; i < 6; ++i) {
		if (i < na) {
			double dx = spaces[0].axis[i]->settings.source + settings.unitg[i] * settings.x0g + settings.unith[i] * settings.x0h - final_x[i];
			check_x += dx * dx;
		}

		double dv = settings.v0g * settings.unitg[i] + settings.v0h * settings.unith[i] - final_v[i];
		check_v += dv * dv;

		double da = settings.a0g * settings.unitg[i] + settings.a0h * settings.unith[i] - final_a[i];
		check_a += da * da;
	}
	if (check_x > 1e-2) {
		warning("Warning: %" LONGFMT " (%" LONGFMT ") final x %f,%f,%f != start x %f,%f,%f", settings.gcode_line, settings.run_file_current, final_x[0], final_x[1], final_x[2], na > 0 ? spaces[0].axis[0]->settings.source + settings.unitg[0] * settings.x0g + settings.unith[0] * settings.x0h : NAN, na > 1 ? spaces[0].axis[1]->settings.source + settings.unitg[1] * settings.x0g + settings.unith[1] * settings.x0h : NAN, na > 2 ? spaces[0].axis[2]->settings.source + settings.unitg[2] * settings.x0g + settings.unith[2] * settings.x0h : NAN);
		debug_abort();
	}
	if (check_v > 1e-2) {
		warning("Warning: %" LONGFMT " (%" LONGFMT ") final v %f,%f,%f != start v %f,%f,%f", settings.gcode_line, settings.run_file_current, final_v[0], final_v[1], final_v[2], na > 0 ? settings.v0g * settings.unitg[0] + settings.v0h * settings.unith[0] : NAN, na > 1 ? settings.v0g * settings.unitg[1] + settings.v0h * settings.unith[1] : NAN, na > 2 ? settings.v0g * settings.unitg[2] + settings.v0h * settings.unith[2] : NAN);
		debug_abort();
	}
	if (check_a > 1e2) {
		warning("Warning: %" LONGFMT " (%" LONGFMT ") final a %f,%f,%f != start a %f,%f,%f", settings.gcode_line, settings.run_file_current, final_a[0], final_a[1], final_a[2], na > 0 ? settings.a0g * settings.unitg[0] + settings.a0h * settings.unith[0] : NAN, na > 1 ? settings.a0g * settings.unitg[1] + settings.a0h * settings.unith[1] : NAN, na > 2 ? settings.a0g * settings.unitg[2] + settings.a0h * settings.unith[2] : NAN);
		debug_abort();
	}
	if (queue[q].tool >= 0 && queue[q].tool < spaces[1].num_axes && !std::isnan(queue[q].e)) {
		spaces[1].axis[queue[q].tool]->settings.endpos = queue[q].e;
		mdebug("move extruder to %f", queue[q].e);
	}
	else if (queue[q].single && queue[q].tool < 0 && ~queue[q].tool < spaces[2].num_axes && !std::isnan(queue[q].e)) {
		spaces[2].axis[~queue[q].tool]->settings.endpos = queue[q].e;
		mdebug("move follower to %f, current=%f source=%f current_pos=%f", queue[q].e, spaces[2].axis[~queue[q].tool]->current, spaces[2].axis[~queue[q].tool]->settings.source, spaces[2].motor[~queue[q].tool]->settings.current_pos);
	}
	auto last_hwtime_step = settings.hwtime_step;
	if (queue[q].pattern_size > 0) {
		memcpy(settings.pattern, queue[q].pattern, queue[q].pattern_size);
		settings.hwtime_step = settings.end_time / (queue[q].pattern_size * 8);
		if (settings.hwtime_step < min_hwtime_step)
			settings.hwtime_step = min_hwtime_step;
	}
	settings.pattern_size = queue[q].pattern_size;
	for (int a = 0; a < min(6, spaces[0].num_axes); ++a) {
		mdebug("setting endpos %d %d to target %f", 0, a, queue[q].target[a]);
		spaces[0].axis[a]->settings.endpos = queue[q].target[a];
	}
	store_settings();
	if (settings.hwtime_step != last_hwtime_step)
		arch_globals_change();
	settings.queue_start += 1;
	first_fragment = current_fragment;	// Do this every time, because otherwise the queue must be regenerated.	TODO: send partial fragment to make sure this hack actually works, or fix it properly.
	for (int i = 0; i < 6; ++i) {
		if (i >= spaces[0].num_axes)
			break;
		double t = settings.end_time / 1e6;
		double t2 = t * t;
		double t3 = t2 * t;
		final_x[i] = spaces[0].axis[i]->settings.source + settings.unitg[i] * (settings.x0g + settings.v0g * t + settings.a0g / 2 * t2 + settings.Jg / 6 * t3) + settings.unith[i] * (settings.x0h + settings.v0h * t + settings.a0h / 2 * t2 + settings.Jh / 6 * t3);
		final_v[i] = settings.unitg[i] * (settings.v0g + settings.a0g * t + settings.Jg / 2 * t2) + settings.unith[i] * (settings.v0h + settings.a0h * t + settings.Jh / 2 * t2);
		mdebug("final x,v[%d] = %f,%f", i, final_x[i], final_v[i]);
		final_a[i] = settings.unitg[i] * (settings.a0g + settings.Jg * t) + settings.unith[i] * (settings.a0h + settings.Jh * t);
	}
	//debug("computing for next move, stopping=%d", stopping);
	computing_move = true;
} // }}}

static void check_distance(int sp, int mt, Motor *mtr, Motor *limit_mtr, double distance, double dt, double &factor) { // {{{
	// Check motor limits for motor (sp,mt), which is mtr. For followers, limit_mtr is set to its leader unless settings.single is true.
	// distance is the distance to travel; dt is the time, factor is lowered if needed.
	//debug("check distance %d %d dist %f t %f current %f time step %d", sp, mt, distance, dt, mtr->settings.current_pos, settings.hwtime_step);
	if (dt == 0) {
		warning("check distance for 0 time");
		factor = 0;
		return;
	}
	if (std::isnan(distance) || distance == 0) {
		mtr->last_v = 0;
		return;
	}
	double orig_distance = distance;
	//debug("cd %f %f", distance, dt);
	double target_v = distance / dt;
	double v = std::fabs(target_v);
	int s = (target_v < 0 ? -1 : 1);
	// When turning around, ignore limits (they shouldn't have been violated anyway).
	if (mtr->last_v * s < 0) {
		//debug("direction change");
		mtr->last_v = 0;
	}
	// Limit v.
	if (v > limit_mtr->limit_v * (1 + 1e-3)) {
		warning("line %" LONGFMT " motor %d %d v %f limit %f dist %f dt %f current %f factor %f", settings.gcode_line, sp, mt, v, limit_mtr->limit_v, distance, dt, mtr->settings.current_pos, factor);
		distance = (s * limit_mtr->limit_v) * dt;
		v = std::fabs(distance / dt);
	}
	// Limit a+.
	double limit_dv = limit_mtr->limit_a * dt;
	if (v - mtr->last_v * s > limit_dv * (1 + 1e-3)) {
		warning("line %" LONGFMT " a+ %d %d target v %f limit dv %f last v %f s %d current %f factor %f", settings.gcode_line, sp, mt, target_v, limit_dv, mtr->last_v, s, mtr->settings.current_pos, factor);
		distance = (limit_dv * s + mtr->last_v) * dt;
		v = std::fabs(distance / dt);
	}
	int steps = round((arch_round_pos(sp, mt, mtr->settings.current_pos + distance) - arch_round_pos(sp, mt, mtr->settings.current_pos)) * mtr->steps_per_unit);
	int targetsteps = steps;
	//cpdebug(s, m, "cf %d value %d", current_fragment, value);
	if (probing && steps)
		steps = s;
	else {
		int max = 0x1c0 * 0xf0;	// Don't use 0xff, because limiting isn't accurate.
		if (abs(steps) > max) {
			warning("overflow %d from cp %f dist %f steps/mm %f dt %f s %d max %d", steps, mtr->settings.current_pos, distance, mtr->steps_per_unit, dt, s, max);
			steps = max * s;
		}
	}
	if (abs(steps) < abs(targetsteps)) {
		distance = arch_round_pos(sp, mt, mtr->settings.current_pos) + (steps + s * .5) / mtr->steps_per_unit - mtr->settings.current_pos;
		v = std::fabs(distance / dt);
		//warning("new distance = %f, old = %f", distance, orig_distance);
	}
	//debug("=============");
	double f = distance / orig_distance;
	if (f < factor) {
		//debug("checked %d %d src %f target %f target dist %f new dist %f old factor %f new factor %f", sp, mt, mtr->settings.current_pos, mtr->settings.target_pos, orig_distance, distance, factor, f);
		//factor = f;
	}
	// Store adjusted value of v.
	//target_v = s * v;
	mtr->last_v = target_v;
} // }}}

static double move_axes(Space *s) { // {{{
	s->xyz2motors();
	double factor = 1;
	//*
	for (int m = 0; m < s->num_motors; ++m) {
		//if (s->id == 0 && m == 0)
		mdebug("check move %d %d time %f target %f current %f", s->id, m, settings.hwtime / 1e6, s->motor[m]->target_pos, s->motor[m]->settings.current_pos);
		double distance = std::fabs(s->motor[m]->target_pos - s->motor[m]->settings.current_pos);
		Motor *limit_mtr;
		if (single || s->id != 2)
			limit_mtr = s->motor[m];
		else {
			FollowerMotorData *data = reinterpret_cast <FollowerMotorData *>(s->motor[m]->type_data);
			if (data->space < 0 || data->space > 1 || data->motor < 0 || data->motor >= spaces[data->space].num_motors)
				limit_mtr = s->motor[m];
			else
				limit_mtr = spaces[data->space].motor[data->motor];
		}
		check_distance(s->id, m, s->motor[m], limit_mtr, distance, settings.hwtime_step / 1e6, factor);
	}
	// */
	return factor;
} // }}}

static void do_steps(double old_factor) { // {{{
	// Do the steps to arrive at the correct position. Update axis and motor current positions.
	// Set new current position.
	for (int s = 0; s < NUM_SPACES; ++s) {
		if (!single && s == 2)
			continue;
		Space &sp = spaces[s];
		for (int a = 0; a < sp.num_axes; ++a) {
			if (!std::isnan(sp.axis[a]->target))
				sp.axis[a]->current = sp.axis[a]->target;
		}
	}
	// Move the motors.
	mdebug("start move");
	for (int s = 0; s < NUM_SPACES; ++s) {
		if (!single && s == 2)
			continue;
		Space &sp = spaces[s];
		for (int m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			double target = mtr.target_pos;
			if (std::isnan(target)) {
				mdebug("target %d %d is NaN", s, m);
				continue;
			}
			cpdebug(s, m, "ccp3 stopping %d target %f frag %d", stopping, target, current_fragment);
			int current_hw = mtr.settings.hw_pos;
			int new_hw = arch_pos2hw(s, m, target);
			if (current_hw != new_hw) {
				if (!mtr.active) {
					mtr.active = true;
					num_active_motors += 1;
					//debug("activating motor %d %d", s, m);
				}
				int diff = new_hw - current_hw;
				int max_steps = 0x1ff;
				if (diff > max_steps) {
					warning("Error on line %" LONGFMT ": %d %d trying to send more than 0x1ff steps: %x  from %x to %x (time %d)", settings.gcode_line, s, m, diff, current_hw, new_hw, settings.hwtime);
					debug_abort();
					int adjust = diff - max_steps;
					if (settings.hwtime_step > settings.hwtime)
						settings.hwtime = 0;
					else
						settings.hwtime -= settings.hwtime_step;
					factor = old_factor;
					diff = max_steps;
					target -= adjust / mtr.steps_per_unit;
					mtr.target_pos = target;
				}
				if (diff < -max_steps) {
					warning("Error on line %" LONGFMT ": %d %d trying to send more than -0x1ff steps: %x  from %x to %x (time %d)", settings.gcode_line, s, m, -diff, current_hw, new_hw, settings.hwtime);
					debug_abort();
					int adjust = diff + max_steps;
					if (settings.hwtime_step > settings.hwtime)
						settings.hwtime = 0;
					else
						settings.hwtime -= settings.hwtime_step;
					factor = old_factor;
					diff = -max_steps;
					target -= adjust / mtr.steps_per_unit;
					mtr.target_pos = target;
				}
				// Encode bits.
				int sign = (diff < 0) ? -1 : 1;
				int hwsign = ((diff < 0) ^ spaces[s].motor[m]->dir_pin.inverted()) ? -1 : 1;
				diff = abs(diff);
				int count = diff >> 6;
				int head = ((1 << count) - 1) << (7 - count);
				int value = (hwsign < 0 ? 0x80 : 0) | head | ((diff & 0x3f) >> count);
				mdebug("sending %d %d steps %d*%x (coded to %x) at %d", s, m, hwsign, diff, value, current_fragment_pos);
				DATA_SET(s, m, value != 0x80 ? value : 0);	// Send 0 instead of -0, because 0x80 is special.
				// When count == 7, an extra bit is shifted out so the computation fails. There's no mantissa in that case, so the replacement is a constant.
				int sent = diff >= 0x1c0 ? 0x1c0 : diff & ~((1 << count) - 1);
				mtr.settings.hw_pos += sign * sent;
			}
			//debug("new cp: %d %d %f %d", s, m, target, current_fragment_pos);
			if (!single) {
				for (int mm = 0; mm < spaces[2].num_motors; ++mm) {
					FollowerMotorData *data = reinterpret_cast <FollowerMotorData *>(spaces[2].motor[mm]->type_data);
					if (data->space != s || data->motor != m)
						continue;
					spaces[2].motor[mm]->settings.current_pos += target - mtr.settings.current_pos;
				}
			}
			mtr.settings.current_pos = target;
		}
	}
	int pattern_pos = (settings.hwtime - settings.hwtime_step / 2) / settings.hwtime_step;
	//debug("time %d step %d pattern pos %d size %d", settings.hwtime, settings.hwtime_step, pattern_pos, settings.pattern_size);
	if (pattern_pos < settings.pattern_size * 8) {
		if (!pattern.active) {
			pattern.active = true;
			num_active_motors += 1;
			//debug("activating pattern");
		}
		//debug("set pattern for %d at %d to %d", current_fragment, current_fragment_pos, settings.pattern[pattern_pos]);
		PATTERN_SET(settings.pattern[pattern_pos]);
	}
	current_fragment_pos += 1;
	//debug("current fragment pos -> %d", current_fragment_pos);
	if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
		mdebug("sending because fragment full (normal) %d %d %d %d", computing_move, current_fragment_pos, BYTES_PER_FRAGMENT, stopping);
		send_fragment();
	}
} // }}}

static double set_targets(double factor) { // {{{
	// Set motor targets for the requested factor. If limits are exceeded, return largest acceptable factor.
	if (spaces[0].num_axes > 0) {
		// Only handle positional move if there are positional axes.
		double t = factor * settings.end_time / 1e6;
		double t2 = t * t;
		double t3 = t2 * t;
		double xg = settings.Jg * t3 / 6 + settings.a0g * t2 / 2 + settings.v0g * t + settings.x0g;
		double xh = settings.Jh * t3 / 6 + settings.a0h * t2 / 2 + settings.v0h * t + settings.x0h;
		for (int i = 0; i < 3; ++i) {
			if (i >= spaces[0].num_axes)
				continue;
			spaces[0].axis[i]->target = spaces[0].axis[i]->settings.source + xg * settings.unitg[i] + xh * settings.unith[i];
		}
		mdebug("targets %f,%f,%f -> %f,%f,%f src %f,%f,%f current %f,%f,%f adjust %f: %f,%f,%f time %f", spaces[0].axis[0]->target, spaces[0].axis[1]->target, spaces[0].axis[2]->target, spaces[0].motor[0]->settings.current_pos, spaces[0].motor[1]->settings.current_pos, spaces[0].motor[2]->settings.current_pos, spaces[0].axis[0]->settings.source, spaces[0].axis[1]->settings.source, spaces[0].axis[2]->settings.source, spaces[0].axis[0]->current, spaces[0].axis[1]->current, spaces[0].axis[2]->current, settings.adjust, spaces[0].axis[0]->settings.adjust, spaces[0].axis[1]->settings.adjust, spaces[0].axis[2]->settings.adjust, settings.adjust_time / 1e6);
	}
	// Set all other axes with linear interpolation and compute motor positions, returning maximum allowed factor.
	double max_f = 1;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		if (s == 2 && !single)
			continue;
		for (int a = (s == 0 ? 3 : 0); a < sp.num_axes; ++a) {
			auto ax = sp.axis[a];
			if (std::isnan(ax->settings.source)) {
				ax->settings.source = 0;
				mdebug("setting axis %d %d source from nan to 0", s, a);
			}
			ax->target = ax->settings.source + factor * (ax->settings.endpos - ax->settings.source);
			//if (s == 1)
			//	debug("setting target for %d %d to %f (%f -> %f) adjust %f", s, a, ax->target, ax->settings.source, ax->settings.endpos, ax->settings.adjust);
		}
		double f = move_axes(&sp);
		if (max_f > f)
			max_f = f;
	}
	return max_f;
} // }}}

static void apply_tick() { // {{{
	// Move motors to position for next time tick.
	// If it exceeds limits, adjust hwtime so that it's acceptable.
	mdebug("tick");
	if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
		// Fragment is already full. This shouldn't normally happen.
		debug("aborting apply_tick, because fragment is already full.");
		return;
	}
	// Check for move.
	if (!(computing_move || settings.adjust > 0)) {
		debug("apply tick called, but not moving");
		return;
	}
	settings.hwtime += settings.hwtime_step;
	if (settings.adjust > 0) {
		settings.adjust = 1 - ((settings.hwtime - settings.adjust_start_time) * 1. / settings.adjust_time);
	}
	while (true) {
		// This loop is normally only run once, but when a move is complete it is rerun for the next move.
		int empty_fragments = (running_fragment - 1 - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
		if (stopping || discarding || discard_pending || empty_fragments <= (FRAGMENTS_PER_BUFFER > 4 ? 4 : FRAGMENTS_PER_BUFFER - 2)) {
			// Abort attempt to fill buffer. Adjust time so next call it will retry current time.
			settings.hwtime -= settings.hwtime_step;
			break;
		}
		//debug("tick time %d step %d frag %d pos %d", settings.hwtime, settings.hwtime_step, current_fragment, current_fragment_pos);
		double t = settings.hwtime / 1e6;
		double target_factor;	// Factor of current move that should be completed at this time.
		if (settings.hwtime >= settings.end_time)
			target_factor = 1;
		else if (settings.hwtime <= 0)
			target_factor = 0;
		else {
			target_factor = t / (settings.end_time / 1e6);
			if (target_factor > 1)
				target_factor = 1;
			if (target_factor < 0)
				target_factor = 0;
		}
		//debug("target factor: %f (t=%f end=%f)", target_factor, t, settings.end_time / 1e6);
		// Go straight to the next move if the distance was 0 (so target_factor is NaN).
		double old_factor = factor;
		if (!std::isnan(target_factor)) {
			double f = set_targets(target_factor);
			if (!std::isnan(factor) && f < 1) {
				//double old = target_factor;
				if (factor > 0 || settings.hwtime <= 0)
					target_factor = factor + f * (target_factor - factor);
				else {
					// factor == 0, so we're at the start of a move.
					// settings.hwtime > 0, so this must be a continuation.
					// Only the new part can be limited, so to limit the same distance, it needs to be a larger fraction.
					// Example:
					// hwtime_step = 10
					// hwtime = 6
					// -> newpart = 0.6
					// f = 0.9
					// target move = 100
					// so (1-0.9)*100=10 of target move must be removed
					// that is (1-0.9)/0.6 of new part.
					double newpart = settings.hwtime * 1. / settings.hwtime_step;
					//debug("update f from %f", f);
					f = 1 - (1 - f) / newpart;
					target_factor = f * target_factor;
				}
				if (target_factor < 0)
					target_factor = 0;
				else if (target_factor > 1)
					target_factor = 1;
				//debug("adjust target factor (%f) from %f to %f because f=%f", factor, old, target_factor, f);
				set_targets(target_factor);
				//debug("adjusting time with f = %f, old factor = %f, old time = %f", f, old_factor, settings.hwtime / 1e6);
				//adjust_time(target_factor);
			}
			mdebug("target factor %f time 0 -> %d -> %d v0g %f", target_factor, settings.hwtime, settings.end_time, settings.v0g);
			factor = target_factor;
			if (factor < 1) {
				do_steps(old_factor);
				return;
			}
		}
		mdebug("next segment");
		// Set new sources for all axes.
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a) {
				auto ax = sp.axis[a];
				mdebug("setting source for %d %d to endpos %f (target is %f)", s, a, ax->settings.endpos, ax->target);
				ax->settings.source = ax->settings.endpos;
			}
		}
		// start new move; adjust time.
		// This will call next_move, even if there is no run file.
		run_file_next_command(settings.end_time);
		if (stopping || discarding || discard_pending)
			break;
		if (!computing_move) {
			// There is no next move.
			continue_event = true;
			do_steps(old_factor);
			if (current_fragment_pos > 0 && !sending_fragment) {
				mdebug("sending final fragment");
				send_fragment();
			}
			return;
		}
		mdebug("try again");
	}
	mdebug("tick done");
	//if (spaces[0].num_axes >= 2)
		//debug("move z %d %d %f %f %f", current_fragment, current_fragment_pos, spaces[0].axis[2]->current, spaces[0].motor[0]->settings.current_pos, spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0]);
} // }}}

void store_settings() { // {{{
	if (current_fragment_pos != 0) {
		warning("store settings called with non-empty send buffer");
		abort();
	}
	num_active_motors = 0;
	if (FRAGMENTS_PER_BUFFER == 0)
		return;
	for (int i = 0; i < 3; ++i) {
		history[current_fragment].unitg[i] = settings.unitg[i];
		history[current_fragment].unith[i] = settings.unith[i];
	}
	history[current_fragment].Jg = settings.Jg;
	history[current_fragment].Jh = settings.Jh;
	history[current_fragment].a0g = settings.a0g;
	history[current_fragment].a0h = settings.a0h;
	history[current_fragment].v0g = settings.v0g;
	history[current_fragment].v0h = settings.v0h;
	history[current_fragment].x0g = settings.x0g;
	history[current_fragment].x0h = settings.x0h;
	history[current_fragment].hwtime = settings.hwtime;
	history[current_fragment].hwtime_step = settings.hwtime_step;
	history[current_fragment].end_time = settings.end_time;
	history[current_fragment].adjust_start_time = settings.adjust_start_time;
	history[current_fragment].adjust_time = settings.adjust_time;
	history[current_fragment].run_time = settings.run_time;
	history[current_fragment].pattern_size = settings.pattern_size;
	history[current_fragment].gcode_line = settings.gcode_line;
	history[current_fragment].adjust = settings.adjust;
	history[current_fragment].run_file_current = settings.run_file_current;
	history[current_fragment].queue_start = settings.queue_start;
	history[current_fragment].queue_end = settings.queue_end;
	for (int i = 0; i < PATTERN_MAX; ++i)
		history[current_fragment].pattern[i] = settings.pattern[i];
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->active = false;
			sp.motor[m]->history[current_fragment].current_pos = sp.motor[m]->settings.current_pos;
			sp.motor[m]->history[current_fragment].hw_pos = sp.motor[m]->settings.hw_pos;
			cpdebug(s, m, "store");
		}
		for (int a = 0; a < sp.num_axes; ++a) {
			mdebug("setting history %d of source for %d %d to %f; current is %f", current_fragment, s, a, sp.axis[a]->settings.source, sp.axis[a]->current);
			sp.axis[a]->history[current_fragment].source = sp.axis[a]->settings.source;
			sp.axis[a]->history[current_fragment].endpos = sp.axis[a]->settings.endpos;
			sp.axis[a]->history[current_fragment].adjust = sp.axis[a]->settings.adjust;
		}
	}
	pattern.active = false;
	DATA_CLEAR();
} // }}}

void restore_settings() { // {{{
	current_fragment_pos = 0;
	num_active_motors = 0;
	if (FRAGMENTS_PER_BUFFER == 0)
		return;
	for (int i = 0; i < 3; ++i) {
		settings.unitg[i] = history[current_fragment].unitg[i];
		settings.unith[i] = history[current_fragment].unith[i];
	}
	settings.Jg = history[current_fragment].Jg;
	//debug("restored Jg to %f from history %d", settings.Jg, current_fragment);
	settings.Jh = history[current_fragment].Jh;
	settings.a0g = history[current_fragment].a0g;
	settings.a0h = history[current_fragment].a0h;
	settings.v0g = history[current_fragment].v0g;
	settings.v0h = history[current_fragment].v0h;
	settings.x0g = history[current_fragment].x0g;
	settings.x0h = history[current_fragment].x0h;
	settings.hwtime = history[current_fragment].hwtime;
	settings.hwtime_step = history[current_fragment].hwtime_step;
	settings.adjust_start_time = history[current_fragment].adjust_start_time;
	settings.adjust_time = history[current_fragment].adjust_time;
	settings.end_time = history[current_fragment].end_time;
	settings.run_time = history[current_fragment].run_time;
	settings.pattern_size = history[current_fragment].pattern_size;
	settings.gcode_line = history[current_fragment].gcode_line;
	settings.adjust = history[current_fragment].adjust;
	settings.run_file_current = history[current_fragment].run_file_current;
	settings.queue_start = history[current_fragment].queue_start;
	settings.queue_end = history[current_fragment].queue_end;
	for (int i = 0; i < PATTERN_MAX; ++i)
		settings.pattern[i] = history[current_fragment].pattern[i];
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->active = false;
			sp.motor[m]->settings.current_pos = sp.motor[m]->history[current_fragment].current_pos;
			sp.motor[m]->settings.hw_pos = sp.motor[m]->history[current_fragment].hw_pos;
			cpdebug(s, m, "restore");
		}
		for (int a = 0; a < sp.num_axes; ++a) {
			mdebug("setting source for %d %d to history %d of %f", s, a, current_fragment, sp.axis[a]->history[current_fragment].source);
			sp.axis[a]->settings.source = sp.axis[a]->history[current_fragment].source;
			sp.axis[a]->settings.endpos = sp.axis[a]->history[current_fragment].endpos;
			sp.axis[a]->settings.adjust = sp.axis[a]->history[current_fragment].adjust;
		}
	}
	pattern.active = false;
	DATA_CLEAR();
} // }}}

void buffer_refill() { // {{{
	// Try to fill the buffer. This is called at any time that the buffer may be refillable.
	//mdebug("refill");
	if (aborting || preparing || FRAGMENTS_PER_BUFFER == 0) {
		mdebug("no refill because prepare or no buffer yet");
		return;
	}
	if (!(computing_move || settings.adjust > 0) || refilling || stopping || discarding != 0) {
		//mdebug("no refill due to block: not computing %d adjust %f refilling %d stopping %d discarding %d", !computing_move, settings.adjust, refilling, stopping, discarding);
		return;
	}
	refilling = true;
	// send_fragment in the previous refill may have failed; try it again.
	/*if (current_fragment_pos > 0) {
		debug("sending because data pending frag=%d pos=%d", current_fragment, current_fragment_pos);
		send_fragment();
	}*/
	mdebug("refill start running %d current %d sending %d computing %d aborting %d stopping %d discarding %d discard_pending %d", running_fragment, current_fragment, sending_fragment, computing_move, aborting, stopping, discarding, discard_pending);
	// Keep one free fragment, because we want to be able to rewind and use the buffer before the one currently active.
	while ((computing_move || settings.adjust > 0) && !aborting && !stopping && discarding == 0 && !discard_pending && (running_fragment - 1 - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER > (FRAGMENTS_PER_BUFFER > 4 ? 4 : FRAGMENTS_PER_BUFFER - 2) && !sending_fragment) {
		mdebug("refill %d %d %f", current_fragment, current_fragment_pos, spaces[0].motor[0]->settings.current_pos);
		// fill fragment until full.
		apply_tick();
		mdebug("refill2 %d %f", current_fragment, spaces[0].motor[0]->settings.current_pos);
		if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
			debug("sending because fragment full (weird) %d %d %d", computing_move, current_fragment_pos, BYTES_PER_FRAGMENT);
			send_fragment();
		}
	}
	if (aborting || stopping || discarding != 0 || discard_pending) {
		mdebug("aborting refill for stopping");
		refilling = false;
		return;
	}
	if (!(computing_move || settings.adjust > 0) && current_fragment_pos > 0) {
		mdebug("sending because move ended");
		send_fragment();
	}
	refilling = false;
	arch_start_move(0);
} // }}}

void abort_move(int pos) { // {{{
	aborting = true;
	//debug("abort pos %d", pos);
	//debug("abort; cf %d rf %d first %d computing_move %d fragments, regenerating %d ticks", current_fragment, running_fragment, first_fragment, computing_move, pos);
	//debug("try aborting move");
	current_fragment = running_fragment;
	//debug("current_fragment = running_fragment; %d", current_fragment);
	//debug("current abort -> %x", current_fragment);
	while (pos < 0) {
		if (current_fragment == first_fragment) {
			pos = 0;
		}
		else {
			current_fragment = (current_fragment + FRAGMENTS_PER_BUFFER - 1) % FRAGMENTS_PER_BUFFER;
			//debug("current_fragment = (current_fragment + FRAGMENTS_PER_BUFFER - 1) %% FRAGMENTS_PER_BUFFER; %d", current_fragment);
			pos += SAMPLES_PER_FRAGMENT;
			running_fragment = current_fragment;
		}
	}
	restore_settings();
	//debug("free abort reset");
	current_fragment_pos = 0;
	computing_move = true;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int a = 0; a < sp.num_axes; ++a) {
			sp.axis[a]->current = sp.axis[a]->settings.source;
		}
	}
	while (computing_move && current_fragment_pos < unsigned(pos)) {
		mdebug("abort reconstruct %d %d", current_fragment_pos, pos);
		apply_tick();
	}
	mdebug("done reconstructing");
	for (int i = 0; i < 6; ++i) {
		final_x[i] = NAN;
		final_v[i] = 0;
		final_a[i] = 0;
	}
	if (spaces[0].num_axes > 0)
		cpdebug(0, 0, "ending hwpos %f", arch_round_pos(0, 0, spaces[0].motor[0]->settings.current_pos) + avr_pos_offset[0]);
	// Flush queue.
	settings.queue_start = 0;
	settings.queue_end = 0;
	// Copy settings back to previous fragment.
	current_fragment_pos = 0;
	settings.adjust = 0;
	store_settings();
	//debug("no compute because abort");
	computing_move = false;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int a = 0; a < sp.num_axes; ++a) {
			mdebug("setting axis %d source to %f for abort", a, sp.axis[a]->current);
			if (!std::isnan(sp.axis[a]->current))
				sp.axis[a]->settings.source = sp.axis[a]->current;
		}
	}
	mdebug("aborted move, current fragment pos is %d", current_fragment_pos);
	aborting = false;
} // }}}

static double inner(double A[6], double B[6]) { // {{{
	double ret = 0;
	for (int i = 0; i < 6; ++i)
		ret += A[i] * B[i];
	return ret;
} // }}}

static void mul(double dst[6], double A[6], double f) { // {{{
	for (int i = 0; i < 6; ++i)
		dst[i] = A[i] * f;
} // }}}

static void deviation(double dst[6], double main[6], double deviant[6]) { // {{{
	double projection[6];
	double len_main = std::sqrt(inner(main, main));
	mul(projection, main, inner(deviant, main) / len_main);
	for (int i = 0; i < 6; ++i)
		dst[i] = deviant[i] - projection[i];
} // }}}

static double s_dv(double v1, double v2) { // {{{
	// Compute length that is required for changing speed from v1 to v2.
	double dv = std::fabs(v2 - v1);
	double v0 = min(v1, v2);
	double max_ramp_dv = max_a * max_a / (2 * max_J);
	if (max_ramp_dv >= dv / 2) {
		// max a is not reached.
		double t = std::sqrt(dv / max_J);
		return max_J * t * t * t + 2 * v0 * t;
	}
	else {
		// max a is reached.
		double t = max_a / max_J;
		double dv_a = dv - max_J * t * t;
		double t_a = dv_a / max_a;
		return v0 * (2 * t + t_a) + max_ramp_dv * t_a + max_a / 2 * t_a * t_a + max_a / 2 * t * t + (dv - max_ramp_dv) * t;
	}
} // }}}

static int add_to_queue(int q, int64_t gcode_line, int time, int tool, double pos[6], double tf, double v0, double a0, double e, double target[6], double Jg, double *h = NULL, double Jh = 0, bool reverse = false, bool probing = false) { // {{{
	queue[q].probe = probing;
	queue[q].single = false;
	queue[q].reverse = reverse;
	queue[q].tool = tool;
	double leng = 0, lenh = 0;
	double g[6];
	for (int i = 0; i < 6; ++i) {
		queue[q].target[i] = target[i];
		g[i] = target[i] - pos[i];
		leng += g[i] * g[i];
		lenh += h ? h[i] * h[i] : 0;
	}
	leng = std::sqrt(leng);
	if (tf < 1e-6)
		return q;
	lenh = std::sqrt(lenh);
	mdebug("adding to queue: %.2f,%.2f,%.4f -> %.2f,%.2f,%.4f time %.2f v0 %.2f a0 %.2f Jg %.2f g %.2f,%.2f,%.2f h %.2f,%.2f,%.2f Jh %.2f reverse %d", pos[0], pos[1], pos[2], target[0], target[1], target[2], tf, v0, a0, Jg, g[0], g[1], g[2], h ? h[0] : 0, h ? h[1] : 0, h ? h[2] : 0, Jh, reverse);
#if 0
	// Compute and report [vaJ][gh](tf)
	if (!reverse) {
		double x0 = 0;
		for (int i = 0; i < 3; ++i)
			x0 += (target[i] - pos[i]) * (target[i] - pos[i]);
		x0 = -std::sqrt(x0);
		double xgt = x0 + Jg / 6 * tf * tf * tf + a0 / 2 * tf * tf + v0 * tf;
		double xht = Jh / 6 * tf * tf * tf;
		double vgt = Jg / 2 * tf * tf + a0 * tf + v0;
		double vht = Jh / 2 * tf * tf;
		double agt = Jg * tf + a0;
		double aht = Jh * tf;
		debug("dxh/dxg(tf)=%f/%f=%f=1/%f", xht, xgt, xht / xgt, xgt / xht);
		debug("vh/vg(tf)=%f/%f=%f=1/%f", vht, vgt, vht / vgt, vgt / vht);
		debug("ah/ag(tf)=%f/%f=%f=1/%f", aht, agt, aht / agt, agt / aht);
		debug("Jh/Jg=%f/%f=%f=1/%f", Jh, Jg, Jh / Jg, Jg / Jh);
	}
#endif
	for (int i = 0; i < 6; ++i) {
		queue[q].unitg[i] = g[i] / leng;
		queue[q].unith[i] = lenh == 0 ? 0 : h[i] / lenh;
		// Set x to new target.
		pos[i] = target[i];
	}
	queue[q].Jh = Jh;
	queue[q].time = time;
	queue[q].gcode_line = gcode_line;
	queue[q].pattern_size = 0;
	queue[q].Jg = Jg;
	queue[q].tf = tf;
	queue[q].v0 = v0;
	queue[q].a0 = a0;
	queue[q].e = e;
	return q + 1;
} // }}}

static int queue_speed_change(int q, int tool, double x[6], double unitv[6], double old_v, double new_v) { // {{{
	mdebug("speed change from %f to %f", old_v, new_v);
	int s = new_v < old_v ? -1 : 1;
	double dv = std::fabs(new_v - old_v);
	if (dv < 1e-5)
		return q;
	double max_ramp_dv = max_a * max_a / (2 * max_J);
	if (dv > 2 * max_ramp_dv) {
		// With const a.
		double t_ramp = max_a / max_J;
		double t_ramp2 = t_ramp * t_ramp;
		double t_ramp3 = t_ramp2 * t_ramp;
		double dv_a = dv - max_J * t_ramp * t_ramp;
		double t_max_a = dv_a / max_a;
		double dv_ramp = max_ramp_dv;
		double target[6];
		for (int i = 0; i < 6; ++i)
			target[i] = x[i] + unitv[i] * (old_v * t_ramp + max_J * s / 6 * t_ramp3);
		q = add_to_queue(q, -1, 0, tool, x, t_ramp, old_v, 0, NAN, target, max_J * s);
		double extra = (old_v + dv_ramp * s) * t_max_a + max_a * s / 2 * t_max_a * t_max_a;
		for (int i = 0; i < 6; ++i)
			target[i] += unitv[i] * extra;
		mdebug("ramps+ old v %f new v %f t_ramp %f s %d max J %f", old_v, new_v, t_ramp, s, max_J);
		q = add_to_queue(q, -1, 0, tool, x, t_max_a, old_v + dv_ramp * s, max_a * s, NAN, target, 0);
		extra = new_v * t_ramp - max_J * s / 6 * t_ramp3;
		for (int i = 0; i < 6; ++i)
			target[i] += unitv[i] * extra;
		q = add_to_queue(q, -1, 0, tool, x, t_ramp, new_v, 0, NAN, target, -max_J * s, NULL, 0, true);
	}
	else {
		// Only ramps.
		double t_ramp = std::sqrt(dv / max_J);
		double t_ramp2 = t_ramp * t_ramp;
		double t_ramp3 = t_ramp2 * t_ramp;
		// Compute both targets before adding anything to queue, so the reference is the same.
		double target[2][6];
		for (int i = 0; i < 6; ++i) {
			target[0][i] = x[i] + unitv[i] * (s * max_J / 6 * t_ramp3 + old_v * t_ramp);
			target[1][i] = x[i] + unitv[i] * (s * max_J * t_ramp3 + 2 * old_v * t_ramp);
		}
		mdebug("ramps old v %f new v %f t_ramp %f s %d max J %f", old_v, new_v, t_ramp, s, max_J);
		q = add_to_queue(q, -1, 0, tool, x, t_ramp, old_v, 0, NAN, target[0], max_J * s);
		q = add_to_queue(q, -1, 0, tool, x, t_ramp, new_v, 0, NAN, target[1], -max_J * s, NULL, 0, true);
	}
	return q;
} // }}}

bool compute_current_pos(double x[6], double v[6], double a[6], bool store) { // {{{
	// discard buffer
	discard();
	for (int i = 0; i < 6; ++i) {
		if (i < spaces[0].num_motors)
			spaces[0].motor[i]->last_v = NAN;
	}
	settings.hwtime_step = default_hwtime_step;
	if (store) {
		for (int s = 0; s < NUM_SPACES; ++s) {
			for (int a = 0; a < spaces[s].num_axes; ++a) {
				mdebug("storing %d,%d: %f, %f", s, a, spaces[s].axis[a]->settings.source, spaces[s].axis[a]->settings.endpos);
				spaces[s].axis[a]->resume_start = spaces[s].axis[a]->settings.source;
				spaces[s].axis[a]->resume_end = spaces[s].axis[a]->settings.endpos;
			}
		}
	}
	if (!computing_move) {
		// Not running, simply fill parameters with current static position.
		for (int i = 0; i < 6; ++i) {
			if (i < spaces[0].num_axes)
				x[i] = spaces[0].axis[i]->current;
			v[i] = 0;
			a[i] = 0;
		}
		return true;
	}
	mdebug("flush queue for discard at compute current");
	settings.queue_start = 0;
	settings.queue_end = 0;
	double t = settings.hwtime / 1e6;
	double t2 = t * t;
	double t3 = t2 * t;
	double ag = settings.a0g + settings.Jg * t;
	double ah = settings.a0h + settings.Jh * t;
	double vg = settings.v0g + settings.a0g * t + settings.Jg / 2 * t2;
	double vh = settings.v0h + settings.a0h * t + settings.Jh / 2 * t2;
	double xg = settings.x0g + settings.v0g * t + settings.a0g / 2 * t2 + settings.Jg / 6 * t3;
	double xh = settings.x0h + settings.v0h * t + settings.a0h / 2 * t2 + settings.Jh / 6 * t3;
	mdebug("agh=%.2f,%.2f vgh=%.2f,%.2f xgh=%.2f,%.2f g %.2f,%.2f,%.2f Jg %f src=%.2f,%.2f,%.2f", ag, ah, vg, vh, xg, xh, settings.unitg[0], settings.unitg[1], settings.unitg[2], settings.Jg, spaces[0].axis[0]->settings.source, spaces[0].axis[1]->settings.source, spaces[0].axis[2]->settings.source);
	double factor = t / (settings.end_time / 1e6);
	for (int s = 1; s < NUM_SPACES; ++s) {
		for (int i = 0; i < spaces[s].num_axes; ++i) {
			double src = spaces[s].axis[i]->settings.source;
			double dst = spaces[s].axis[i]->settings.endpos;
			spaces[s].axis[i]->current = src + factor * (dst - src);
			mdebug("setting axis %d source to %f for discard", i, spaces[s].axis[i]->current);
			spaces[s].axis[i]->settings.source = spaces[s].axis[i]->current;
			spaces[s].axis[i]->settings.endpos = spaces[s].axis[i]->current;
		}
	}
	for (int i = 0; i < 6; ++i) {
		x[i] = (i < spaces[0].num_axes ? spaces[0].axis[i]->settings.source : 0) + xg * settings.unitg[i] + xh * settings.unith[i];
		v[i] = vg * settings.unitg[i] + vh * settings.unith[i];
		a[i] = ag * settings.unitg[i] + ah * settings.unith[i];
		// Update "final" settings.
		if (i < spaces[0].num_axes) {
			final_x[i] = x[i];
			final_v[i] = v[i];
			final_a[i] = a[i];
		}
		// Update current position.
		if (i < spaces[0].num_axes)
			spaces[0].axis[i]->current = x[i];
	}
	mdebug("current pos computed (t=%f): %f,%f,%f v %f,%f,%f a %f,%f,%f", t, x[0], x[1], x[2], v[0], v[1], v[2], a[0], a[1], a[2]);
	return false;
} // }}}

int prepare_retarget(int q, int tool, double x[6], double v[6], double a[6], bool resuming) { // {{{
	if (resuming)
		mul(v, v, -1);
	double lena = std::sqrt(inner(a, a));
	double lenv = std::sqrt(inner(v, v));
	mdebug("retargeting%s, x=(%.2f,%.2f,%.2f) v=(%.2f,%.2f,%.2f), a=(%.2f,%.2f,%.2f)", resuming ? " (for resume)" : "", x[0], x[1], x[2], v[0], v[1], v[2], a[0], a[1], a[2]);
	// add segment to bring a to zero
	mdebug("pull a from %f to zero", lena);
	// J -> -a, t = a/J, g -> v, h -> (a - (a_g))
	double target_x[6], target_v[6];
	double t = lena / max_J;
	double t2 = t * t;
	double t3 = t2 * t;
	double J[6];
	for (int i = 0; i < 6; ++i) {
		J[i] = lena > 0 ? -a[i] / lena * max_J : 0;
		target_x[i] = J[i] / 6 * t3 + a[i] / 2 * t2 + v[i] * t + x[i];
		target_v[i] = J[i] / 2 * t2 + a[i] * t + v[i];
	}
	double len_target_v = std::sqrt(inner(target_v, target_v));

	// Compute unitg and Jg
	double unitg[6], h[6];
	if (len_target_v > 1e-5)
		mul(unitg, target_v, 1 / len_target_v);
	else
		mul(unitg, v, 1 / lenv);
	double Jg = inner(J, unitg);

	// Compute h and Jh
	deviation(h, a, unitg);
	double lenh = std::sqrt(inner(h, h));
	mul(h, h, lenh > 0 ? 1 / lenh : 0);
	double Jh = inner(J, h);
	mul(h, h, Jh);

	// Set source.
	double sh = Jh / 6 * t3;
	double sg = sh / std::tan(M_PI / 2 - std::acos(inner(unitg, v) / lenv));
	mdebug("retargeting, g=%.2f,%.2f,%.2f sh=%.2f sg=%.2f Jg=%.2f", unitg[0], unitg[1], unitg[2], sh, sg, Jh);
	for (int i = 0; i < 6; ++i) {
		if (i < spaces[0].num_axes) {
			spaces[0].axis[i]->settings.source = x[i] - h[i] / Jh * sh - unitg[i] * sg;
			mdebug("setting axis %d source to %f for retarget", i, spaces[0].axis[i]->settings.source);
		}
	}
	// Add segment to queue.
	if (resuming) { // {{{
		// target_x, -target_v is the last point before the resume point.
		double s = s_dv(0, len_target_v);
		MoveCommand move;
		move.single = 0;
		move.probe = false;
		move.v0 = max_v;
		move.tool = 0;
		move.e = spaces[1].num_axes > 0 ? spaces[1].axis[0]->current : 0;
		move.gcode_line = 0;
		move.time = 0;
		double start[6];
		mdebug("go to resuming");
		if (s > 1e-5) {
			for (int i = 0; i < 6; ++i) {
				start[i] = target_x[i] + target_v[i] / len_target_v * s;
				move.target[i] = start[i];
			}
			q = go_to(false, &move, true);
			mul(unitg, unitg, -1);
			q = queue_speed_change(q, -1, start, unitg, 0, len_target_v);
		}
		else {
			for (int i = 0; i < 6; ++i)
				move.target[i] = target_x[i];
			q = go_to(false, &move, true);
		}
		if (lena > 1e-5)
			q = add_to_queue(q, -1, 0, tool, target_x, t, len_target_v, 0, NAN, x, Jg, h, Jh, false);
	} // }}}
	else {
		if (lena > 1e-5) {
			mdebug("bring a to 0");
			q = add_to_queue(q, -1, 0, tool, x, t, len_target_v, 0, NAN, target_x, Jg, h, Jh, true);
		}
		else {
			mdebug("a was already 0: %f", lena);
			for (int i = 0; i < 6; ++i) {
				if (i < spaces[0].num_axes) {
					spaces[0].axis[i]->settings.source = x[i];
					mdebug("setting axis %d source to %f for retarget correction", i, spaces[0].axis[i]->settings.source);
				}
			}
		}
		// Update movement variables.
		for (int i = 0; i < 6; ++i) {
			x[i] = target_x[i];
			v[i] = target_v[i];
		}
		//debug("Try from here, x=(%f,%f,%f) v=(%f,%f,%f)", x[0], x[1], x[2], v[0], v[1], v[2]);
	}
	return q;
} // }}}

void smooth_stop(int q, double x[6], double v[6]) { // {{{
	mdebug("pausing. axes = %f,%f,%f motors = %f,%f,%f", spaces[0].axis[0]->current, spaces[0].axis[1]->current, spaces[0].axis[2]->current, spaces[0].motor[0]->settings.current_pos, spaces[0].motor[1]->settings.current_pos, spaces[0].motor[2]->settings.current_pos);
	double lenv = std::sqrt(inner(v, v));
	mul(v, v, 1 / lenv);
	settings.queue_end = queue_speed_change(q, -1, x, v, lenv, 0);
	next_move(settings.hwtime);
} // }}}

void do_resume() { // {{{
	if (pausing && run_file_wait > 0)
		run_file_wait -= 1;
	pausing = false;
	resume_pending = true;
	mdebug("new queue for resume");
	settings.queue_start = 0;
	settings.queue_end = prepare_retarget(0, -1, resume.x, resume.v, resume.a, true);
	next_move(settings.hwtime);
	buffer_refill();
} // }}}

int go_to(bool relative, MoveCommand const *move, bool queue_only, bool unprobe) { // {{{
	mdebug("goto (%.2f,%.2f,%.2f) %s at speed %.2f, e %.2f", move->target[0], move->target[1], move->target[2], relative ? "rel" : "abs", move->v0, move->e);
	mdebug("new queue for goto");
	settings.queue_start = 0;
	int q = 0;
	double x[6];
	for (int a = 0; a < 6; ++a) {
		if (a < spaces[0].num_axes)
			x[a] = spaces[0].axis[a]->current;
		else
			x[a] = 0;
	}
	double target[6];
	for (int a = 0; a < 6; ++a) {
		if (a >= spaces[0].num_axes)
			break;
		if (std::isnan(x[a]))
			continue;
		if (std::isnan(move->target[a]))
			target[a] = x[a];
		else
			target[a] = (relative ? x[a] : 0) + move->target[a];
		spaces[0].axis[a]->last_target = target[a];
	}
	if (unprobe)
		target[2] -= probe_value(0, target[0], target[1]);
	if (computing_move) {
		// Reset target of current move to given values.
		double v[6], a[6];
		compute_current_pos(x, v, a, false);
		q = prepare_retarget(q, move->tool, x, v, a);
		double lenv = std::sqrt(inner(v, v));
		mul(v, v, 1 / lenv);
		// add segment to slow down to min(current_v, requested_v) {{{
		if (move->v0 < lenv) {
			mdebug("slow down from %f to %f", lenv, move->v0);
			// Add segments to queue
			q = queue_speed_change(q, move->tool, x, v, lenv, move->v0);
			// Update movement variables.
			lenv = move->v0;
		} // }}}
		// Only do a curve if current v is not 0, otherwise do a normal goto.
		if (lenv > 1e-10) {
			mdebug("current v %f > 0", lenv);
			// Compute P, g, unitPF, h, EF {{{
			// v = J/2t^2
			double s_stop = s_dv(lenv, 0);
			double g[6], h[6], P[6], unitPF[6];
			double lenPF = 0;
			for (int i = 0; i < 6; ++i) {
				g[i] = v[i];
				P[i] = x[i] + g[i] * s_stop;
				if (std::isnan(move->target[i])) {
					if (i < spaces[0].num_axes && !std::isnan(spaces[0].axis[i]->last_target))
						unitPF[i] = spaces[0].axis[i]->last_target - P[i];
					else
						unitPF[i] = 0;
				}
				else
					unitPF[i] = target[i] - P[i];
				lenPF += unitPF[i] * unitPF[i];
			}
			lenPF = sqrt(lenPF);
			mul(unitPF, unitPF, 1 / lenPF);
			deviation(h, g, unitPF);
			double h2[6];
			deviation(h2, unitPF, g);
			double s_speedup = s_dv(lenv, move->v0);
			double s_slowdown = s_dv(move->v0, 0);
			// }}}
			bool done = true;
			if (lenPF < 2 * s_stop) {
				// target is closer to P than tool: stop and goto target.
				q = queue_speed_change(q, move->tool, x, v, lenv, 0);
				lenv = 0;
				done = false; // Fall through to goto handling.
			}
			else {
				// make curve
				double theta = std::acos(inner(g, unitPF));
				double dir = std::tan(theta / 2);	// direction along curve at midpoint.
				double t = s_stop / lenv;
				mdebug("retarget curve, x=(%f,%f,%f), P=(%f,%f,%f), unitPF=(%f,%f,%f), theta=%f, dir=%f, v0=%f, x0=-%f", x[0], x[1], x[2], P[0], P[1], P[2], unitPF[0], unitPF[1], unitPF[2], theta, dir, lenv, s_stop);
				double Jh = 2 * lenv * dir / ((dir * dir + 1) * t * t);
				double Jg = -dir * Jh;
				double subtarget[6];
				for (int i = 0; i < 6; ++i)
					subtarget[i] = P[i] + unitPF[i] * s_stop;
				q = add_to_queue(q, -1, 0, move->tool, x, t, lenv, 0, NAN, P, Jg, h, Jh, false);
				q = add_to_queue(q, -1, 0, move->tool, x, t, lenv, 0, NAN, subtarget, Jg, h2, Jh, true);
				mul(v, unitPF, 1);
				double v_top;
				if (lenPF - s_stop < s_speedup + s_slowdown) {
					//debug("target is too close for reaching requested v");
					// target is too close to reach requested v. Go as fast as possible for the segment.
					// Don't care enough about this case to optimize. Pretend that the space needs to be used to get to current speed as well.
					v_top = compute_max_v((lenPF - s_stop) / 2, 0, max_J, max_a);
					if (v_top <= lenv) {
						// The safe max v is lower than the current v. Don't try to speed up at all.
						v_top = lenv;
						s_speedup = 0;
						s_slowdown = s_stop;
					}
					s_speedup = s_dv(lenv, v_top);
					s_slowdown = s_dv(v_top, 0);
				}
				else {
					// Everything fits.
					v_top = move->v0;
				}
				// speed up
				q = queue_speed_change(q, move->tool, x, v, lenv, v_top);
				// constant v
				double dist = lenPF - s_stop - s_speedup - s_slowdown;
				if (dist < 0) {
					warning("Warning: dist < 0 for retarget?! dist=%f, lenPF=%f, s_stop=%f, s_speedup=%f, s_slowdown=%f, v0=%f, v=%f", dist, lenPF, s_stop, s_speedup, s_slowdown, lenv, v_top);
				}
				for (int i = 0; i < 6; ++i)
					subtarget[i] = x[i] + v[i] * dist;
				q = add_to_queue(q, -1, 0, move->tool, x, dist / v_top, v_top, 0, NAN, subtarget, 0);
				// slow down to 0
				q = queue_speed_change(q, move->tool, x, v, v_top, 0);
			}
			settings.queue_end = q;
			if (done && !stopping) {
				next_move(settings.hwtime);
				return 0;
			}
		}
		//debug("retarget fall through to regular goto");
	}
	// This is a manual move or the start of a job; set hwtime step to default.
	mdebug("goto (%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f) at speed %.2f, e %.2f->%.2f", x[0], x[1], x[2], target[0], target[1], target[2], move->v0, move->tool >= 0 && move->tool < spaces[1].num_axes ? spaces[1].axis[move->tool]->current : 0, move->e);
	settings.hwtime_step = default_hwtime_step;
	double vmax = NAN;
	double amax = NAN;
	double dist = NAN;
	double unit[6] = {0, 0, 0, 0, 0, 0};
	for (int a = 0; a < 6; ++a) {
		if (spaces[0].num_axes <= a)
			break;
		//debug("prepare move, pos[%d]: %f -> %f", a, pos, move->target[a]);
		double d = target[a] - x[a];
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
		for (int i = 0; i < 6; ++i)
			unit[i] /= dist;
		mdebug("normal goto, dist = %f", dist);
	}
	else {
		mdebug("no dist");
		for (int i = 0; i < 6; ++i)
			unit[i] = 0;
	}
	int tool = move->tool;
	double e;
	if (tool >= 0 && tool < spaces[1].num_axes)
		e = spaces[1].axis[tool]->current;
	else if (move->single && tool < 0 && ~tool < spaces[2].num_axes)
		e = spaces[2].axis[~tool]->current;
	else
		e = NAN;
	if (!std::isnan(e)) {
		if (std::isnan(dist) || dist < 1e-10) {
			dist = std::fabs(move->e - (relative ? 0 : e));
			mdebug("dist for e %d = %f", tool, dist);
			if (tool >= 0) {
				vmax = spaces[1].motor[tool]->limit_v;
				amax = spaces[1].motor[tool]->limit_a;
			}
			else {
				// use leader limits with fallback to global limits.
				if (~tool < spaces[2].num_motors) {
					FollowerMotorData *data = reinterpret_cast <FollowerMotorData *>(spaces[2].motor[~tool]->type_data);
					if (data->space >= 0 && data->space <= 1 && data->motor >= 0 && data->motor < spaces[data->space].num_motors) {
						vmax = spaces[data->space].motor[data->motor]->limit_v;
						amax = spaces[data->space].motor[data->motor]->limit_a;
					}
					else {
						vmax = max_v;
						amax = max_a;
					}
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
		//debug("not moving, because dist is %f", dist);
		settings.queue_end = q;
		return queue_only ? 0 : 1;
	}

	double reachable_v = compute_max_v(dist / 2, 0, max_J, amax);
	mdebug("building queue for goto with vmax = %f, reachable = %f, dist=%f", vmax, reachable_v, dist);
	vmax = min(vmax, reachable_v);
	vmax = min(vmax, move->v0);
	double max_ramp_dv = amax * amax / (2 * max_J);
	// Initialize the queue. At most 7 items will be used.
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
	double t_ramp = std::sqrt(2 * dv_ramp / max_J);
	double s_ramp_up = max_J / 6 * t_ramp * t_ramp * t_ramp;
	double s_ramp_down = -max_J / 6 * t_ramp * t_ramp * t_ramp + vmax * t_ramp;
	double s_const_a = amax / 2 * t_max_a * t_max_a + dv_ramp * t_max_a;
	double s_const_v = dist - 2 * (s_ramp_up + s_ramp_down + s_const_a);
	double t_const_v = s_const_v / vmax;

	double s[7] = {s_ramp_up, s_const_a, s_ramp_down, s_const_v, s_ramp_down,      s_const_a, s_ramp_up};
	double t[7] =    {t_ramp,   t_max_a,      t_ramp, t_const_v,      t_ramp,        t_max_a,    t_ramp};
	double J[7] =     {max_J,         0,      -max_J,         0,      -max_J,              0,     max_J};
	double v0[7] =    {    0,   dv_ramp,        vmax,      vmax,        vmax, vmax - dv_ramp,         0};
	double a0[7] =    {    0,      amax,           0,         0,           0,          -amax,         0};
	bool reverse[7] = {false,     false,        true,     false,       false,          false,      true};
	double X[6];
	for (int i = 0; i < 6; ++i)
		X[i] = i < spaces[0].num_axes ? x[i] : 0;
	double current_s = 0;
	double e0;
	if (tool >= 0 && tool < spaces[1].num_axes)
		e0 = spaces[1].axis[tool]->current;
	else if (tool < 0 && ~tool < spaces[2].num_axes)
		e0 = spaces[2].axis[~tool]->current;
	else
		e0 = NAN;
	for (int part = 0; part < 7; ++part) {
		current_s += s[part];
		if (s[part] < 1e-10)
			continue;
		double subtarget[6];
		for (int i = 0; i < 6; ++i)
			subtarget[i] = X[i] + unit[i] * s[part];
		if (spaces[0].num_axes >= 3)
			mdebug("adding to queue: X2=%.2f target2=%.2f", X[2], subtarget[2]);
		q = add_to_queue(q, move->gcode_line, move->time, move->tool, X, t[part], v0[part], a0[part], e0 + (move->e - e0) * current_s / dist, subtarget, J[part], NULL, 0, reverse[part], move->probe);
	}
	settings.queue_end = q;

#if 0
	debug("goto dir=%f,%f,%f, dist=%f, tool=%d e=%f single=%d", unit[0], unit[1], unit[2], dist, tool, move->e, move->single);
	debug("goto s %f,%f,%f,%f,%f,%f,%f", s[0], s[1], s[2], s[3], s[4], s[5], s[6]);
	debug("goto t %f,%f,%f,%f,%f,%f,%f", t[0], t[1], t[2], t[3], t[4], t[5], t[6]);
	debug("goto J %f,%f,%f,%f,%f,%f,%f", J[0], J[1], J[2], J[3], J[4], J[5], J[6]);
	debug("goto v0 %f,%f,%f,%f,%f,%f,%f", v0[0], v0[1], v0[2], v0[3], v0[4], v0[5], v0[6]);
#endif

	if (queue_only)
		return q;

	next_move(settings.hwtime);
	return 0;
} // }}}

void discard_finals() { // {{{
	for (int i = 0; i < 6; ++i) {
		final_x[i] = NAN;
		final_v[i] = NAN;
		final_a[i] = NAN;
	}
} // }}}

void discard() { // {{{
	// Discard much of the buffer, so the upcoming change will be used almost immediately.
	if (!avr_running || stopping || avr_homing || !computing_move || discarding != 0)
		return;
	//debug("discard start current = %d, running = %d, sending = %d", current_fragment, running_fragment, sending_fragment);
	discard_pending = true;
	// Clear final target of current move.
	discard_finals();
	// Compute fragments in buffer; if there are enough, discard some.
	int fragments = (current_fragment + (transmitting_fragment ? 1 : 0) - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
	if (fragments <= 3) {
		discard_pending = false;
		return;
	}
	discarding = fragments - 3;
	// Discard the fragments and reload old state as current.
	current_fragment = (current_fragment - discarding + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
	restore_settings();
	// Send instruction to firmware. If this cannot be done now, it is done later.
	arch_discard();
	// We're in the middle of a move again, so make sure the computation is restarted.
	computing_move = true;
	buffer_refill();
	//debug("discard end current = %d, running = %d, sending = %d", current_fragment, running_fragment, sending_fragment);
} // }}}

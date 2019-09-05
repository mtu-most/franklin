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

#ifndef mdebug
#define mdebug(...) do {} while (0)
#endif

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
			debug("not sending short fragment for 0 motors; %d %d", current_fragment, running_fragment);
			if (history[current_fragment].cbs) {
				if (settings.queue_start == settings.queue_end && !settings.queue_full) {
					// Send cbs immediately.
					if (!host_block) {
						history[(current_fragment + 1) % FRAGMENTS_PER_BUFFER].cbs += history[current_fragment].cbs;
						//debug("adding %d cbs in send_fragment", history[current_fragment].cbs);
						history[current_fragment].cbs = 0;
					}
				}
			}
			current_fragment_pos = 0;
			return;
		}
		else {
			//debug("sending fragment for 0 motors at position %d", current_fragment_pos);
		}
		//abort();
	}
	//debug("sending %d prevcbs %d", current_fragment, history[(current_fragment + FRAGMENTS_PER_BUFFER - 1) % FRAGMENTS_PER_BUFFER].cbs);
	if (aborting || arch_send_fragment()) {
		current_fragment = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
		current_fragment_pos = 0;
		//debug("current_fragment = (current_fragment + 1) %% FRAGMENTS_PER_BUFFER; %d", current_fragment);
		//debug("current send -> %x", current_fragment);
		store_settings();
		if (!aborting && (current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER >= MIN_BUFFER_FILL && !stopping) {
			arch_start_move(0);
		}
	}
	else {
		// Reset current_fragment_pos anyway, otherwise store_settings will complain.
		current_fragment_pos = 0;
	}
} // }}}

static void change0(int qpos) { // {{{
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int a = 0; a < 3; ++a) {
			if (a >= spaces[s].num_axes)
				continue;
			//debug("change %d %d %d %f", s, a, sp.type, queue[qpos].X[a]);
			queue[qpos].target[a] = space_types[sp.type].change0(&sp, a, queue[qpos].target[a]);	// TODO: this should be checked.
		}
	}
} // }}}

// For documentation about variables used here, see struct History in cdriver.h
int next_move(int32_t start_time) { // {{{
	settings.probing = false;
	settings.factor = 0;
	int num_cbs = 0;
	run_file_fill_queue();
	if (settings.queue_start == settings.queue_end && !settings.queue_full) {
		//debug("no next move");
		computing_move = false;
		return num_cbs;
	}
	// Clean up state before starting the move (but not if there is no next move; in that case finish the current tick before sending).
	if (current_fragment_pos > 0) {
		//debug("sending because new move is started");
		send_fragment();
	}
	mdebug("Next move; queue start = %d, end = %d", settings.queue_start, settings.queue_end);
	// Set everything up for running queue[settings.queue_start].
	int q = settings.queue_start;
	int n = (settings.queue_start + 1) % QUEUE_LENGTH;
	settings.single = queue[q].single;

	if (queue[q].cb) {
		++num_cbs;
		//debug("getting %d cbs from queue %d, which makes cbs = %d", queue[q].cb, q, num_cbs);
	}

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
				if (!std::isnan(sp.axis[a]->settings.current)) {
					sp.axis[a]->settings.source = sp.axis[a]->settings.current;
					continue;
				}
				reset_pos(&sp);
				for (int aa = 0; aa < sp.num_axes; ++aa)
					sp.axis[aa]->settings.current = sp.axis[aa]->settings.source;
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

	change0(q);
	// Fill unspecified coordinates with previous values. {{{
	Space &sp0 = spaces[0];
	for (int a = 0; a < 3; ++a) {
		if (a >= sp0.num_axes)
			continue;
		if ((!std::isnan(queue[q].target[a]) || (n != settings.queue_end && !std::isnan(queue[n].target[a]))) && std::isnan(sp0.axis[a]->settings.source)) {
			debug("Motor position for axis %d is not known, so move cannot take place; aborting move and removing it from the queue: q1=%f q2=%f src=%f", a, queue[q].target[a], queue[n].target[a], sp0.axis[a]->settings.source);
			// This possibly removes one move too many, but it shouldn't happen anyway.
			settings.queue_start = n;
			settings.queue_full = false;
			abort_move(current_fragment_pos);
			return num_cbs;
		}
		// If position is NaN, don't move.
		if (std::isnan(queue[q].target[a]))
			queue[q].target[a] = sp0.axis[a]->settings.source;
	}
	// }}}
	// Reset time. {{{
	if (computing_move) {
		settings.hwtime -= start_time;
		//debug("time -= %d, now %d", start_time, settings.hwtime);
	}
	else {
		settings.hwtime = 0;
	}
	// }}}

	if (!computing_move) {	// Set up source if this is a new move. {{{
		mdebug("starting new move");
		//debug("current %d running %d", current_fragment, running_fragment);
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a) {
				if (!std::isnan(sp.axis[a]->settings.current)) {
					mdebug("setting source for %d %d to current %f (was %f)", s, a, sp.axis[a]->settings.current, sp.axis[a]->settings.source);
					sp.axis[a]->settings.source = sp.axis[a]->settings.current;
				}
			}
		}
	} // }}}

	double r = feedrate;
	double r2 = r * r;
	double r3 = r2 * r;
	settings.end_time = queue[q].tf * 1e6 / r;
	settings.run_time = queue[q].time / r;
	settings.Jh = 0;
	double leng = 0;
	for (int i = 0; i < 3; ++i) {
		if (i >= sp0.num_axes)
			continue;
		settings.g[i] = queue[q].target[i] - spaces[0].axis[i]->settings.source;
		settings.h[i] = queue[q].h[i];
		leng += settings.g[i] * settings.g[i];
		settings.Jh += settings.h[i] * settings.h[i] * r3;
	}
	settings.Jh = std::sqrt(settings.Jh);
	leng = std::sqrt(leng);
	for (int i = 0; i < 3; ++i) {
		settings.unitg[i] = settings.g[i] / leng;
		settings.unith[i] = settings.Jh == 0 ? 0 : settings.h[i] / settings.Jh;
	}
	double t = settings.end_time / 1e6;
	double t2 = t * t;
	double t3 = t2 * t;
	if (queue[q].reverse) {
		mdebug("set reverse from J %f v %f", queue[q].Jg, queue[q].v0);
		settings.x0h = queue[q].Jh / 6 * t3;
		settings.v0h = r * -queue[q].Jh / 2 * t2;
		settings.a0h = r2 * queue[q].Jh * t;
		settings.Jh = r3 * -queue[q].Jh;
		settings.x0g = leng - (queue[q].Jg / 6 * t3 + queue[q].v0 * t);
		settings.v0g = r * (queue[q].Jg / 2 * t2 + queue[q].v0);
		settings.a0g = r2 * -queue[q].Jg * t;
		settings.Jg = r3 * queue[q].Jg;
	}
	else {
		settings.Jg = r3 * queue[q].Jg;
		settings.a0g = r2 * queue[q].a0;
		settings.v0g = r * queue[q].v0;
		settings.x0g = 0;
		settings.a0h = 0;
		settings.v0h = 0;
		settings.x0h = 0;
	}
	settings.dist = ((settings.Jg * t / 6 + settings.a0g) * t + settings.v0g) * t;
	if (std::isnan(settings.dist) || std::fabs(settings.dist) < 1e-10) {
		//debug("no space dist, using other system. dist=%f a=%f ab=%f b=%f", settings.dist, norma, normab, normb);
		if (queue[q].tool >= 0 && queue[q].tool < spaces[1].num_axes)
			settings.dist = std::fabs(queue[q].e - spaces[1].axis[queue[q].tool]->settings.source);
		else if (queue[q].single && queue[q].tool < 0 && ~queue[q].tool < spaces[2].num_axes)
			settings.dist = std::fabs(queue[q].e - spaces[2].axis[~queue[q].tool]->settings.source);
	}
	if (queue[q].tool >= 0 && queue[q].tool < spaces[1].num_axes && !std::isnan(queue[q].e)) {
		spaces[1].axis[queue[q].tool]->settings.endpos = queue[q].e;
		mdebug("move extruder to %f", queue[q].e);
	}
	else if (queue[q].single && queue[q].tool < 0 && ~queue[q].tool < spaces[2].num_axes && !std::isnan(queue[q].e)) {
		spaces[2].axis[~queue[q].tool]->settings.endpos = queue[q].e;
		mdebug("move follower to %f, current=%f source=%f current_pos=%f", queue[q].e, spaces[2].axis[~queue[q].tool]->settings.current, spaces[2].axis[~queue[q].tool]->settings.source, spaces[2].motor[~queue[q].tool]->settings.current_pos);
	}
	auto last_hwtime_step = settings.hwtime_step;
	if (queue[q].pattern_size > 0) {
		memcpy(settings.pattern, queue[q].pattern, queue[q].pattern_size);
		settings.hwtime_step = settings.end_time / (queue[q].pattern_size * 8);
		if (settings.hwtime_step < min_hwtime_step)
			settings.hwtime_step = min_hwtime_step;
	}
	settings.pattern_size = queue[q].pattern_size;
	if (settings.hwtime_step != last_hwtime_step)
		arch_globals_change();
	//debug("move prepared, from=(%f,%f,%f) target=(%f,%f,%f), h=(%f,%f,%f), dist=%f, e=%f, Jg=%f a0g=%f v0g=%f x0g=%f end time=%f, single=%d", spaces[0].axis[0]->settings.source, spaces[0].axis[1]->settings.source, spaces[0].axis[2]->settings.source, queue[q].target[0], queue[q].target[1], queue[q].target[2], settings.h[0], settings.h[1], settings.h[2], settings.dist, queue[q].e, settings.Jg, settings.a0g, settings.v0g, settings.x0g, settings.end_time / 1e6, queue[q].single);

	settings.queue_start = n;
	first_fragment = current_fragment;	// Do this every time, because otherwise the queue must be regenerated.	TODO: send partial fragment to make sure this hack actually works, or fix it properly.
	if (!computing_move) {
		if (current_fragment_pos > 0)
			abort();
		store_settings();
	}
	computing_move = true;
	return num_cbs;
} // }}}

static void check_distance(int sp, int mt, Motor *mtr, Motor *limit_mtr, double distance, double dt, double &factor) { // {{{
	// TODO: fix and enable this.
	return;
	// Check motor limits for motor (sp,mt), which is mtr. For followers, limit_mtr is set to its leader unless settings.single is true.
	// distance is the distance to travel; dt is the time, factor is lowered if needed.
	//debug("check distance %d %d dist %f t %f current %f time step %d", sp, mt, distance, dt, mtr->settings.current_pos, settings.hwtime_step);
	if (dt == 0) {
		debug("check distance for 0 time");
		factor = 0;
		return;
	}
	if (std::isnan(distance) || distance == 0) {
		mtr->settings.last_v = 0;
		return;
	}
	double orig_distance = distance;
	//debug("cd %f %f", distance, dt);
	mtr->settings.target_v = distance / dt;
	double v = std::fabs(mtr->settings.target_v);
	int s = (mtr->settings.target_v < 0 ? -1 : 1);
	// When turning around, ignore limits (they shouldn't have been violated anyway).
	if (mtr->settings.last_v * s < 0) {
		//debug("direction change");
		mtr->settings.last_v = 0;
	}
	// Limit v.
	if (v > limit_mtr->limit_v) {
		debug("%d %d v %f limit %f dist %f dt %f current %f factor %f", sp, mt, v, limit_mtr->limit_v, distance, dt, mtr->settings.current_pos, settings.factor);
		distance = (s * limit_mtr->limit_v) * dt;
		v = std::fabs(distance / dt);
	}
	// Limit a+.
	double limit_dv = limit_mtr->limit_a * dt;
	if (v - mtr->settings.last_v * s > limit_dv) {
		debug("a+ %d %d target v %f limit dv %f last v %f s %d current %f factor %f", sp, mt, mtr->settings.target_v, limit_dv, mtr->settings.last_v, s, mtr->settings.current_pos, settings.factor);
		distance = (limit_dv * s + mtr->settings.last_v) * dt;
		v = std::fabs(distance / dt);
	}
	int steps = round((arch_round_pos(sp, mt, mtr->settings.current_pos + distance) - arch_round_pos(sp, mt, mtr->settings.current_pos)) * mtr->steps_per_unit);
	int targetsteps = steps;
	//cpdebug(s, m, "cf %d value %d", current_fragment, value);
	if (settings.probing && steps)
		steps = s;
	else {
		int max = 0x78;	// Don't use 0x7f, because limiting isn't accurate.
		if (abs(steps) > max) {
			debug("overflow %d from cp %f dist %f steps/mm %f dt %f s %d max %d", steps, mtr->settings.current_pos, distance, mtr->steps_per_unit, dt, s, max);
			steps = max * s;
		}
	}
	if (abs(steps) < abs(targetsteps)) {
		distance = arch_round_pos(sp, mt, mtr->settings.current_pos) + (steps + s * .5) / mtr->steps_per_unit - mtr->settings.current_pos;
		v = std::fabs(distance / dt);
		debug("new distance = %f, old = %f", distance, orig_distance);
	}
	//debug("=============");
	double f = distance / orig_distance;
	if (f < factor) {
		//debug("checked %d %d src %f target %f target dist %f new dist %f old factor %f new factor %f", sp, mt, mtr->settings.current_pos, mtr->settings.target_pos, orig_distance, distance, factor, f);
		factor = f;
	}
	// Store adjusted value of v.
	mtr->settings.target_v = s * v;
} // }}}

static double move_axes(Space *s) { // {{{
	bool ok = true;
	space_types[s->type].xyz2motors(s);
	// Try again if it didn't work; it should have moved target to a better location.
	if (!ok) {
		space_types[s->type].xyz2motors(s);
		mdebug("retried move");
	}
	//mdebug("ok %d", ok);
	double factor = 1;
	//*
	for (int m = 0; m < s->num_motors; ++m) {
		//if (s->id == 0 && m == 0)
		//	debug("check move %d %d time %f target %f current %f", s->id, m, settings.hwtime / 1e6, s->motor[m]->settings.target_pos, s->motor[m]->settings.current_pos);
		double distance = std::fabs(s->motor[m]->settings.target_pos - s->motor[m]->settings.current_pos);
		Motor *limit_mtr;
		if (settings.single)
			limit_mtr = s->motor[m];
		else {
			int target = space_types[s->type].follow(s, m);
			if (target < 0)
				limit_mtr = s->motor[m];
			else {
				int fs = target >> 8;
				int fa = target & 0xff;
				limit_mtr = spaces[fs].motor[fa];
			}
		}
		check_distance(s->id, m, s->motor[m], limit_mtr, distance, settings.hwtime_step / 1e6, factor);
	}
	// */
	return factor;
} // }}}

static void adjust_time(double target_factor) { // {{{
	// Adjust time.	TODO: use poly3 math
	//double old_time = settings.hwtime / 1e6;
	double x = target_factor * std::fabs(settings.dist);
	settings.hwtime = (x / settings.v0g) * 1e6;
	//debug("adjust time for factor %f: %f -> %f", target_factor, old_time, settings.hwtime / 1e6);
} // }}}

static void do_steps(double old_factor) { // {{{
	// Do the steps to arrive at the correct position. Update axis and motor current positions.
	// Set new current position.
	for (int s = 0; s < NUM_SPACES; ++s) {
		if (!settings.single && s == 2)
			continue;
		Space &sp = spaces[s];
		for (int a = 0; a < sp.num_axes; ++a) {
			if (!std::isnan(sp.axis[a]->settings.target))
				sp.axis[a]->settings.current = sp.axis[a]->settings.target;
		}
	}
	// Move the motors.
	mdebug("start move");
	for (int s = 0; s < NUM_SPACES; ++s) {
		if (!settings.single && s == 2)
			continue;
		Space &sp = spaces[s];
		for (int m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			double target = mtr.settings.target_pos;
			if (std::isnan(target)) {
				mdebug("target %d %d is NaN", s, m);
				continue;
			}
			cpdebug(s, m, "ccp3 stopping %d target %f frag %d", stopping, target, current_fragment);
			double rounded_cp = arch_round_pos(s, m, mtr.settings.current_pos);
			double rounded_new_cp = arch_round_pos(s, m, target);
			if (rounded_cp != rounded_new_cp) {
				if (!mtr.active) {
					mtr.active = true;
					num_active_motors += 1;
					//debug("activating motor %d %d", s, m);
				}
				int diff = round((rounded_new_cp - rounded_cp) * mtr.steps_per_unit);
				if (diff > 0x7f) {
					debug("Error: %d %d trying to send more than 127 steps: %d  from %f to %f (time %d)", s, m, diff, rounded_cp, rounded_new_cp, settings.hwtime);
					int adjust = diff - 0x7f;
					if (settings.hwtime_step > settings.hwtime)
						settings.hwtime = 0;
					else
						settings.hwtime -= settings.hwtime_step;
					settings.factor = old_factor;
					diff = 0x7f;
					target -= adjust / mtr.steps_per_unit;
					mtr.settings.target_pos = target;
				}
				if (diff < -0x7f) {
					debug("Error: %d %d trying to send more than -127 steps: %d  from %f to %f (time %d)", s, m, -diff, rounded_cp, rounded_new_cp, settings.hwtime);
					int adjust = diff + 0x7f;
					if (settings.hwtime_step > settings.hwtime)
						settings.hwtime = 0;
					else
						settings.hwtime -= settings.hwtime_step;
					settings.factor = old_factor;
					diff = -0x7f;
					target -= adjust / mtr.steps_per_unit;
					mtr.settings.target_pos = target;
				}
				//debug("sending %d %d steps %d at %d", s, m, diff, current_fragment_pos);
				DATA_SET(s, m, diff);
			}
			//debug("new cp: %d %d %f %d", s, m, target, current_fragment_pos);
			if (!settings.single) {
				for (int mm = 0; mm < spaces[2].num_motors; ++mm) {
					int fm = space_types[spaces[2].type].follow(&spaces[2], mm);
					if (fm < 0)
						continue;
					int fs = fm >> 8;
					fm &= 0x7f;
					if (fs != s || fm != m || (fs == 2 && fm >= mm))
						continue;
					//debug("follow %d %d %d %d %d %f %f", s, m, fs, fm, mm, target, mtr.settings.current_pos);
					spaces[2].motor[mm]->settings.current_pos += target - mtr.settings.current_pos;
				}
			}
			mtr.settings.current_pos = target;
			cpdebug(s, m, "cp three %f", target);
			mtr.settings.last_v = mtr.settings.target_v;
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
	if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
		//debug("sending because fragment full (normal) %d %d %d", computing_move, current_fragment_pos, BYTES_PER_FRAGMENT);
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
			spaces[0].axis[i]->settings.target = spaces[0].axis[i]->settings.source + xg * settings.unitg[i] + xh * settings.unith[i];
		}
		mdebug("targets %f,%f,%f src %f,%f,%f", spaces[0].axis[0]->settings.target, spaces[0].axis[1]->settings.target, spaces[0].axis[2]->settings.target, spaces[0].axis[0]->settings.source, spaces[0].axis[1]->settings.source, spaces[0].axis[2]->settings.source);
	}
	// Set all other axes with linear interpolation and compute motor positions, returning maximum allowed factor.
	double max_f = 1;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		if (s == 2 && !settings.single)
			continue;
		for (int a = (s == 0 ? 3 : 0); a < sp.num_axes; ++a) {
			auto ax = sp.axis[a];
			if (std::isnan(ax->settings.source)) {
				ax->settings.source = 0;
				mdebug("setting axis %d %d source from nan to 0", s, a);
			}
			ax->settings.target = ax->settings.source + factor * (ax->settings.endpos - ax->settings.source);
			if (s == 0)
				mdebug("setting target for %d %d to %f (%f -> %f)", s, a, ax->settings.target, ax->settings.source, ax->settings.endpos);
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
	//debug("tick");
	if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
		// Fragment is already full. This shouldn't normally happen.
		debug("aborting apply_tick, because fragment is already full.");
		return;
	}
	// Check for move.
	if (!computing_move) {
		mdebug("apply tick called, but not moving");
		return;
	}
	//mdebug("handling %d %d", computing_move, cbs_after_current_move);
	// This loop is normally only run once, but when a move is complete it is rerun for the next move.
	while ((running_fragment - 1 - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER > (FRAGMENTS_PER_BUFFER > 4 ? 4 : FRAGMENTS_PER_BUFFER - 2)) {
		settings.hwtime += settings.hwtime_step;
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
		double old_factor = settings.factor;
		if (!std::isnan(target_factor)) {
			double f = set_targets(target_factor);
			if (f < 1) {
				//double old = target_factor;
				if (settings.factor > 0 || settings.hwtime <= 0)
					target_factor = settings.factor + f * (target_factor - settings.factor);
				else {
					// settings.factor == 0, so we're at the start of a move.
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
				//debug("adjust target factor (%f) from %f to %f because f=%f", settings.factor, old, target_factor, f);
				set_targets(target_factor);
				//debug("adjusting time with f = %f, old factor = %f, old time = %f", f, old_factor, settings.hwtime / 1e6);
				adjust_time(target_factor);
			}
			mdebug("target factor %f time 0 -> %d -> %d v0g %f", target_factor, settings.hwtime, settings.end_time, settings.v0g);
			settings.factor = target_factor;
			if (settings.factor < 1) {
				do_steps(old_factor);
				return;
			}
		}
		//debug("next segment");
		// Set new sources for all axes.
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a) {
				auto ax = sp.axis[a];
				ax->settings.source = ax->settings.target;
			}
		}
		// start new move; adjust time.
		int current_cbs = cbs_after_current_move;
		//debug("adding %d cbs because move is completed", cbs_after_current_move);
		cbs_after_current_move = next_move(settings.end_time);
		//debug("new pending cbs: %d", cbs_after_current_move);
		//debug("next move prepared, cbs after this = %d", cbs_after_current_move);
		if (!computing_move) {
			// There is no next move.
			continue_event = true;
			do_steps(old_factor);
			if (current_fragment_pos > 0 && !sending_fragment) {
				//debug("sending %d cbs after fragment %d because no new move", current_cbs, current_fragment);
				history[current_fragment].cbs += current_cbs;
				send_fragment();
			}
			else {
				//debug("move done, sending %d cbs now", current_cbs);
				num_movecbs = current_cbs;
			}
			return;
		}
		history[current_fragment].cbs += current_cbs;
		mdebug("try again");
		// Next loop the time is incremented again, but in this case that shouldn't happen, so compensate.
		settings.hwtime -= settings.hwtime_step;
	}
	//if (spaces[0].num_axes >= 2)
		//debug("move z %d %d %f %f %f", current_fragment, current_fragment_pos, spaces[0].axis[2]->settings.current, spaces[0].motor[0]->settings.current_pos, spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0]);
} // }}}

void store_settings() { // {{{
	if (current_fragment_pos != 0)
		abort();
	num_active_motors = 0;
	if (FRAGMENTS_PER_BUFFER == 0)
		return;
	for (int i = 0; i < 3; ++i) {
		history[current_fragment].g[i] = settings.g[i];
		history[current_fragment].h[i] = settings.h[i];
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
	history[current_fragment].dist = settings.dist;
	history[current_fragment].hwtime = settings.hwtime;
	history[current_fragment].hwtime_step = settings.hwtime_step;
	history[current_fragment].end_time = settings.end_time;
	history[current_fragment].cbs = 0;
	history[current_fragment].queue_start = settings.queue_start;
	history[current_fragment].queue_end = settings.queue_end;
	history[current_fragment].queue_full = settings.queue_full;
	history[current_fragment].run_file_current = settings.run_file_current;
	history[current_fragment].probing = settings.probing;
	history[current_fragment].single = settings.single;
	history[current_fragment].run_time = settings.run_time;
	history[current_fragment].factor = settings.factor;
	history[current_fragment].pattern_size = settings.pattern_size;
	for (int i = 0; i < PATTERN_MAX; ++i)
		history[current_fragment].pattern[i] = settings.pattern[i];
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		sp.history[current_fragment].dist[0] = sp.settings.dist[0];
		sp.history[current_fragment].dist[1] = sp.settings.dist[1];
		for (int i = 0; i < 2; ++i) {
			sp.history[current_fragment].arc[i] = sp.settings.arc[i];
			sp.history[current_fragment].angle[i] = sp.settings.angle[i];
			sp.history[current_fragment].helix[i] = sp.settings.helix[i];
			for (int t = 0; t < 2; ++t)
				sp.history[current_fragment].radius[i][t] = sp.settings.radius[i][t];
			for (int t = 0; t < 3; ++t) {
				sp.history[current_fragment].offset[i][t] = sp.settings.offset[i][t];
				sp.history[current_fragment].e1[i][t] = sp.settings.e1[i][t];
				sp.history[current_fragment].e2[i][t] = sp.settings.e2[i][t];
				sp.history[current_fragment].normal[i][t] = sp.settings.normal[i][t];
			}
		}
		for (int m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->active = false;
			sp.motor[m]->history[current_fragment].target_v = sp.motor[m]->settings.target_v;
			sp.motor[m]->history[current_fragment].target_pos = sp.motor[m]->settings.target_pos;
			sp.motor[m]->history[current_fragment].current_pos = sp.motor[m]->settings.current_pos;
			cpdebug(s, m, "store");
		}
		for (int a = 0; a < sp.num_axes; ++a) {
			sp.axis[a]->history[current_fragment].dist[0] = sp.axis[a]->settings.dist[0];
			sp.axis[a]->history[current_fragment].dist[1] = sp.axis[a]->settings.dist[1];
			sp.axis[a]->history[current_fragment].main_dist = sp.axis[a]->settings.main_dist;
			sp.axis[a]->history[current_fragment].target = sp.axis[a]->settings.target;
			sp.axis[a]->history[current_fragment].source = sp.axis[a]->settings.source;
			sp.axis[a]->history[current_fragment].current = sp.axis[a]->settings.current;
			sp.axis[a]->history[current_fragment].endpos = sp.axis[a]->settings.endpos;
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
		settings.g[i] = history[current_fragment].g[i];
		settings.h[i] = history[current_fragment].h[i];
		settings.unitg[i] = history[current_fragment].unitg[i];
		settings.unith[i] = history[current_fragment].unith[i];
	}
	settings.Jg = history[current_fragment].Jg;
	settings.Jh = history[current_fragment].Jh;
	settings.a0g = history[current_fragment].a0g;
	settings.a0h = history[current_fragment].a0h;
	settings.v0g = history[current_fragment].v0g;
	settings.v0h = history[current_fragment].v0h;
	settings.x0g = history[current_fragment].x0g;
	settings.x0h = history[current_fragment].x0h;
	settings.dist = history[current_fragment].dist;
	settings.hwtime = history[current_fragment].hwtime;
	settings.hwtime_step = history[current_fragment].hwtime_step;
	settings.end_time = history[current_fragment].end_time;
	history[current_fragment].cbs = 0;
	settings.queue_start = history[current_fragment].queue_start;
	settings.queue_end = history[current_fragment].queue_end;
	settings.queue_full = history[current_fragment].queue_full;
	settings.run_file_current = history[current_fragment].run_file_current;
	settings.probing = history[current_fragment].probing;
	settings.single = history[current_fragment].single;
	settings.run_time = history[current_fragment].run_time;
	settings.factor = history[current_fragment].factor;
	settings.pattern_size = history[current_fragment].pattern_size;
	for (int i = 0; i < PATTERN_MAX; ++i)
		settings.pattern[i] = history[current_fragment].pattern[i];
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		sp.settings.dist[0] = sp.history[current_fragment].dist[0];
		sp.settings.dist[1] = sp.history[current_fragment].dist[1];
		for (int i = 0; i < 2; ++i) {
			sp.settings.arc[i] = sp.history[current_fragment].arc[i];
			sp.settings.angle[i] = sp.history[current_fragment].angle[i];
			sp.settings.helix[i] = sp.history[current_fragment].helix[i];
			for (int t = 0; t < 2; ++t)
				sp.settings.radius[i][t] = sp.history[current_fragment].radius[i][t];
			for (int t = 0; t < 3; ++t) {
				sp.settings.offset[i][t] = sp.history[current_fragment].offset[i][t];
				sp.settings.e1[i][t] = sp.history[current_fragment].e1[i][t];
				sp.settings.e2[i][t] = sp.history[current_fragment].e2[i][t];
				sp.settings.normal[i][t] = sp.history[current_fragment].normal[i][t];
			}
		}
		for (int m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->active = false;
			sp.motor[m]->settings.target_v = sp.motor[m]->history[current_fragment].target_v;
			sp.motor[m]->settings.target_pos = sp.motor[m]->history[current_fragment].target_pos;
			sp.motor[m]->settings.current_pos = sp.motor[m]->history[current_fragment].current_pos;
			cpdebug(s, m, "restore");
		}
		for (int a = 0; a < sp.num_axes; ++a) {
			sp.axis[a]->settings.dist[0] = sp.axis[a]->history[current_fragment].dist[0];
			sp.axis[a]->settings.dist[1] = sp.axis[a]->history[current_fragment].dist[1];
			sp.axis[a]->settings.main_dist = sp.axis[a]->history[current_fragment].main_dist;
			sp.axis[a]->settings.target = sp.axis[a]->history[current_fragment].target;
			sp.axis[a]->settings.source = sp.axis[a]->history[current_fragment].source;
			sp.axis[a]->settings.current = sp.axis[a]->history[current_fragment].current;
			sp.axis[a]->settings.endpos = sp.axis[a]->history[current_fragment].endpos;
		}
	}
	pattern.active = false;
	DATA_CLEAR();
} // }}}

void buffer_refill() { // {{{
	// Try to fill the buffer. This is called at any time that the buffer may be refillable.
	//debug("refill");
	if (aborting || preparing || FRAGMENTS_PER_BUFFER == 0) {
		//debug("no refill because prepare or no buffer yet");
		return;
	}
	if (!computing_move || refilling || stopping || discard_pending || discarding) {
		//debug("no refill due to block: %d %d %d %d %d", !computing_move, refilling, stopping, discard_pending, discarding);
		return;
	}
	refilling = true;
	// send_fragment in the previous refill may have failed; try it again.
	/*if (current_fragment_pos > 0) {
		debug("sending because data pending frag=%d pos=%d", current_fragment, current_fragment_pos);
		send_fragment();
	}*/
	//debug("refill start %d %d %d", running_fragment, current_fragment, sending_fragment);
	// Keep one free fragment, because we want to be able to rewind and use the buffer before the one currently active.
	while (computing_move && !aborting && !stopping && !discard_pending && !discarding && (running_fragment - 1 - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER > (FRAGMENTS_PER_BUFFER > 4 ? 4 : FRAGMENTS_PER_BUFFER - 2) && !sending_fragment) {
		//debug("refill %d %d %f", current_fragment, current_fragment_pos, spaces[0].motor[0]->settings.current_pos);
		// fill fragment until full.
		apply_tick();
		//debug("refill2 %d %f", current_fragment, spaces[0].motor[0]->settings.current_pos);
		if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
			//debug("sending because fragment full (weird) %d %d %d", computing_move, current_fragment_pos, BYTES_PER_FRAGMENT);
			send_fragment();
		}
	}
	if (aborting || stopping || discard_pending) {
		//debug("aborting refill for stopping");
		refilling = false;
		return;
	}
	if (!computing_move && current_fragment_pos > 0) {
		//debug("sending because move ended");
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
	while (computing_move && current_fragment_pos < unsigned(pos)) {
		//debug("abort reconstruct %d %d", current_fragment_pos, pos);
		apply_tick();
	}
	if (spaces[0].num_axes > 0)
		cpdebug(0, 0, "ending hwpos %f", arch_round_pos(0, 0, spaces[0].motor[0]->settings.current_pos) + avr_pos_offset[0]);
	// Flush queue.
	settings.queue_start = 0;
	settings.queue_end = 0;
	settings.queue_full = false;
	// Copy settings back to previous fragment.
	current_fragment_pos = 0;
	store_settings();
	computing_move = false;
	current_fragment_pos = 0;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		sp.settings.dist[0] = 0;
		sp.settings.dist[1] = 0;
		for (int a = 0; a < sp.num_axes; ++a) {
			//debug("setting axis %d source to %f", a, sp.axis[a]->settings.current);
			if (!std::isnan(sp.axis[a]->settings.current))
				sp.axis[a]->settings.source = sp.axis[a]->settings.current;
			sp.axis[a]->settings.dist[0] = NAN;
			sp.axis[a]->settings.dist[1] = NAN;
		}
	}
	mdebug("aborted move");
	aborting = false;
} // }}}

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

static void change0(int qpos) { // {{{
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int a = 0; a < spaces[0].num_axes; ++a) {
			//debug("change %d %d %d %f", s, a, sp.type, queue[qpos].X[a]);
			queue[qpos].X[a] = space_types[sp.type].change0(&sp, a, queue[qpos].X[a]);
		}
	}
} // }}}

// For documentation about variables used here, see struct History in cdriver.h
int next_move(int32_t start_time) { // {{{
	settings.probing = false;
	settings.single = false;
	moving_to_current = 0;
	int num_cbs = 0;
	run_file_fill_queue();
	if (settings.queue_start == settings.queue_end && !settings.queue_full) {
		//debug("no next move");
		prepared = false;
		computing_move = false;
		return num_cbs;
	}
	mdebug("Next move; queue start = %d, end = %d", settings.queue_start, settings.queue_end);
	// Set everything up for running queue[settings.queue_start].
	int q = settings.queue_start;
	int n = (settings.queue_start + 1) % QUEUE_LENGTH;

	if (queue[q].cb)
		++num_cbs;

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
	}
	// }}}

	change0(q);
	// Fill unspecified coordinates with previous values. {{{
	Space &sp0 = spaces[0];
	for (int a = 0; a < sp0.num_axes; ++a) {
		if (n != settings.queue_end) {
			// If only one of them is set, set the other one as well to make the rounded corner work.
			if (!std::isnan(queue[q].X[a]) && std::isnan(queue[n].X[a])) {
				queue[n].X[a] = sp0.axis[a]->settings.source + sp0.axis[a]->settings.dist[1] - (a == 2 ? zoffset : 0);
				mdebug("filling next %d with %f", a, queue[n].X[a]);
			}
			if (std::isnan(queue[q].X[a]) && !std::isnan(queue[n].X[a])) {
				queue[q].X[a] = sp0.axis[a]->settings.source;
				mdebug("filling %d with %f", a, queue[q].X[a]);
			}
		}
		if ((!std::isnan(queue[q].X[a]) || (n != settings.queue_end && !std::isnan(queue[n].X[a]))) && std::isnan(sp0.axis[a]->settings.source)) {
			debug("Motor position for axis %d is not known, so move cannot take place; aborting move and removing it from the queue: q1=%f q2=%f src=%f", a, queue[q].X[a], queue[n].X[a], sp0.axis[a]->settings.source);
			// This possibly removes one move too many, but it shouldn't happen anyway.
			settings.queue_start = n;
			settings.queue_full = false;
			abort_move(current_fragment_pos);
			return num_cbs;
		}
	}
	// }}}
	// Reset time. {{{
	settings.hwtime -= start_time;
	settings.last_time -= start_time;
	// }}}

	if (!computing_move) {	// Set up source if this is a new move. {{{
		mdebug("starting new move");
		//debug("current %d running %d", current_fragment, running_fragment);
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a) {
				if (!std::isnan(sp.axis[a]->settings.current))
					sp.axis[a]->settings.source = sp.axis[a]->settings.current;
			}
		}
		store_settings();
	} // }}}

	settings.v0 = queue[q].v0;
	settings.v1 = queue[q].v1;
	double dot = 0, norma = 0, normb = 0, normab = 0;
	for (int i = 0; i < 3; ++i) {
		bool use = i < spaces[0].num_axes;
		double p = (use ? spaces[0].axis[i]->settings.source : 0);
		settings.P[i] = (use ? (std::isnan(queue[q].X[i]) ? p : (queue[q].X[i] + p) / 2) : 0);
		settings.A[i] = settings.P[i] - p;
		settings.B[i] = (use ? queue[q].B[i] : 0);
		double ab = settings.A[i] + settings.B[i];
		dot += settings.A[i] * ab;
		norma += settings.A[i] * settings.A[i];
		normb += settings.B[i] * settings.B[i];
		normab += ab * ab;
	}
	norma = sqrt(norma);
	normb = sqrt(normb);
	normab = sqrt(normab);
	settings.alpha_max = acos(dot / (norma * normab));
	if (std::isnan(settings.alpha_max))
		settings.alpha_max = 0;
	settings.dist = (normb > 1e-5 ? norma * (normab / normb) * settings.alpha_max: norma) * 2;
	if (std::isnan(settings.dist) || abs(settings.dist) < 1e-10) {
		if (queue[q].tool >= 0 && queue[q].tool < spaces[1].num_axes)
			settings.dist = abs(queue[q].e - spaces[1].axis[queue[q].tool]->settings.source);
		else if (queue[q].single && queue[q].tool < 0 && ~queue[q].tool < spaces[2].num_axes)
			settings.dist = abs(queue[q].e - spaces[2].axis[~queue[q].tool]->settings.source);
	}
	double dt = settings.dist / ((settings.v0 + settings.v1) / 2);
	settings.end_time = (std::isnan(dt) ? 0 : 1e6 * dt);
	if (queue[q].tool >= 0 && queue[q].tool < spaces[1].num_axes && !std::isnan(queue[q].e)) {
		spaces[1].axis[queue[q].tool]->settings.endpos = queue[q].e;
		mdebug("move extruder to %f", queue[q].e);
	}
	else if (queue[q].single && queue[q].tool < 0 && ~queue[q].tool < spaces[2].num_axes && !std::isnan(queue[q].e)) {
		spaces[1].axis[~queue[q].tool]->settings.endpos = queue[q].e;
		mdebug("move follower to %f", queue[q].e);
	}
	//debug("move prepared, Q=(%f,%f,%f) P=(%f,%f,%f), A=(%f,%f,%f), B=(%f,%f,%f), max alpha=%f, dist=%f, e=%f, v0=%f, v1=%f, end time=%f", queue[q].X[0], queue[q].X[1], queue[q].X[2], settings.P[0], settings.P[1], settings.P[2], settings.A[0], settings.A[1], settings.A[2], settings.B[0], settings.B[1], settings.B[2], settings.alpha_max, settings.dist, queue[q].e, settings.v0, settings.v1, settings.end_time / 1e6);
	//debug("times end %d current %d dist %f v0 %f v1 %f", settings.end_time, settings.hwtime, settings.dist, settings.v0, settings.v1);

	settings.queue_start = n;
	first_fragment = current_fragment;	// Do this every time, because otherwise the queue must be regenerated.	TODO: send partial fragment to make sure this hack actually works, or fix it properly.
	computing_move = true;
	return num_cbs;
} // }}}

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
		if (current_fragment_pos < 2) {
			// TODO: find out why this is attempted and avoid it.
			debug("not sending short fragment for 0 motors; %d %d", current_fragment, running_fragment);
			if (history[current_fragment].cbs) {
				if (settings.queue_start == settings.queue_end && !settings.queue_full) {
					// Send cbs immediately.
					if (!host_block) {
						num_movecbs += history[current_fragment].cbs;
						//debug("adding %d cbs in send_fragment", history[current_fragment].cbs);
						history[current_fragment].cbs = 0;
					}
				}
			}
			current_fragment_pos = 0;
			return;
		}
		else
			debug("sending fragment for 0 motors at position %d", current_fragment_pos);
		//abort();
	}
	//debug("sending %d prevcbs %d", current_fragment, history[(current_fragment + FRAGMENTS_PER_BUFFER - 1) % FRAGMENTS_PER_BUFFER].cbs);
	if (arch_send_fragment()) {
		current_fragment = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
		//debug("current_fragment = (current_fragment + 1) %% FRAGMENTS_PER_BUFFER; %d", current_fragment);
		//debug("current send -> %x", current_fragment);
		store_settings();
		if ((current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER >= MIN_BUFFER_FILL && !stopping) {
			arch_start_move(0);
		}
	}
} // }}}

static void check_distance(int sp, int mt, Motor *mtr, double distance, double dt, double &factor) { // {{{
	// Check motor limits for motor (sp,mt), which is mtr. distance is the distance to travel; dt is the time, factor is lowered if needed.
	//debug("check distance %d %d dist %f t %f", sp, mt, distance, dt);
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
	double v = fabs(mtr->settings.target_v);
	int s = (mtr->settings.target_v < 0 ? -1 : 1);
	// When turning around, ignore limits (they shouldn't have been violated anyway).
	if (mtr->settings.last_v * s < 0) {
		debug("direction change");
		mtr->settings.last_v = 0;
	}
	// Limit v.
	if (v > mtr->limit_v) {
		debug("%d %d v %f limit %f dist %f t %f", sp, mt, v, mtr->limit_v, distance, dt);
		distance = (s * mtr->limit_v) * dt;
		v = fabs(distance / dt);
	}
	// Limit a+.
	double limit_dv = mtr->limit_a * dt;
	if (v - mtr->settings.last_v * s > limit_dv) {
		debug("a+ %d %d target v %f limit dv %f last v %f s %d", sp, mt, mtr->settings.target_v, limit_dv, mtr->settings.last_v, s);
		distance = (limit_dv * s + mtr->settings.last_v) * dt;
		v = fabs(distance / dt);
	}
	int steps = round((arch_round_pos(sp, mt, mtr->settings.current_pos + distance) - arch_round_pos(sp, mt, mtr->settings.current_pos)) * mtr->steps_per_unit);
	int targetsteps = steps;
	//cpdebug(s, m, "cf %d value %d", current_fragment, value);
	if (settings.probing && steps)
		steps = s;
	else {
		// Maximum depends on number of subfragments.  Be conservative and use 0x7e per subfragment; assume 128 subfragments (largest power of 2 smaller than 10000/75)
		// TODO: 10000 and 75 should follow the actual values for step_time in cdriver and TIME_PER_ISR in firmware.
		int max = 0x7e << 7;
		if (abs(steps) > max) {
			debug("overflow %d from cp %f dist %f steps/mm %f dt %f s %d max %d", steps, mtr->settings.current_pos, distance, mtr->steps_per_unit, dt, s, max);
			steps = max * s;
		}
	}
	if (abs(steps) < abs(targetsteps)) {
		distance = (arch_round_pos(sp, mt, mtr->settings.current_pos) + steps + s * .5 - mtr->settings.current_pos);
		v = fabs(distance / dt);
	}
	//debug("=============");
	double f = distance / orig_distance;
	//debug("checked %d %d src %f target %f target dist %f new dist %f old factor %f new factor %f", sp, mt, mtr->settings.current_pos, mtr->settings.target_pos, orig_distance, distance, factor, f);
	if (f < factor)
		factor = f;
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
	for (int m = 0; m < s->num_motors; ++m) {
		//if (s->id == 0 && m == 0)
			//debug("check move %d %d target %f current %f", s->id, m, s->motor[m]->settings.target_pos, s->motor[m]->settings.current_pos);
		double distance = s->motor[m]->settings.target_pos - s->motor[m]->settings.current_pos;
		check_distance(s->id, m, s->motor[m], distance, (settings.hwtime - settings.last_time) / 1e6, factor);
	}
	return factor; // */
} // }}}

static void do_steps() { // {{{
	// Do the steps to arrive at the correct position. Update axis and motor current positions.
	settings.last_time = settings.hwtime;
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
	//debug("start move");
	for (int s = 0; s < NUM_SPACES; ++s) {
		if (!settings.single && s == 2)
			continue;
		Space &sp = spaces[s];
		for (int m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			double target = mtr.settings.target_pos;
			if (std::isnan(target))
				continue;
			cpdebug(s, m, "ccp3 stopping %d target %f frag %d", stopping, target, current_fragment);
			double rounded_cp = arch_round_pos(s, m, mtr.settings.current_pos);
			double rounded_new_cp = arch_round_pos(s, m, target);
			if (rounded_cp != rounded_new_cp) {
				if (!mtr.active) {
					mtr.active = true;
					//debug("activating motor %d %d", s, m);
					num_active_motors += 1;
				}
				int diff = round((rounded_new_cp - rounded_cp) * mtr.steps_per_unit);
				mdebug("sending %d %d steps %d", s, m, diff);
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
	current_fragment_pos += 1;
	if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
		//debug("fragment full %d %d %d", computing_move, current_fragment_pos, BYTES_PER_FRAGMENT);
		send_fragment();
	}
} // }}}

static double set_targets(double factor) { // {{{
	// Set motor targets for the requested factor. If limits are exceeded, return largest acceptable factor.
	if (spaces[0].num_axes > 0) {
		// Only handle positional move if there are positional axes.
		double factor2 = 2 * factor - 1;	// convert [0,1] to [-1,1]
		double alpha = factor2 * settings.alpha_max;	// requested angle.
		// Moves are a combination of two vectors:
		// A is from the middle of the line from start to end, to the end.
		// B is from the middle of the line from start to end, to the middle of the arc.
		// (A and B are perpendicular.)
		// a and b are weights to use for A and B respectively.
		// a = sin(alpha)/sin(alpha_max). For small angles this breaks, so just use factor2.
		double denominator = sin(settings.alpha_max);
		double a = (denominator < 1e-10 ? factor2 : sin(alpha) / denominator);
		// b = cos(alpha)-cos(alpha_max)/(1-cos(alpha_max). For small angles this also breaks, so use 1-abs(factor2).
		double cmax = cos(settings.alpha_max);
		double b = (cos(alpha) - cmax) / (1 - cmax);
		if (isnan(b))
			b = 1 - std::abs(factor2);	// Doesn't really matter; B == {0, 0, 0}.
		// Set position for xyz axes.
		for (int i = 0; i < 3; ++i) {
			if (i < spaces[0].num_axes) {
				spaces[0].axis[i]->settings.target = settings.P[i] + a * settings.A[i] + b * settings.B[i];
				mdebug("target %f P %f a %f A %f B %f amax %f factor2 %f", spaces[0].axis[i]->settings.target, settings.P[i],a, settings.A[i], settings.B[i], settings.alpha_max, factor2);
			}
		}
	}
	// Set all other axes with linear interpolation and compute motor positions, returning maximum allowed factor.
	double max_f = 1;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int a = (s == 0 ? 3 : 0); a < sp.num_axes; ++a) {
			auto ax = sp.axis[a];
			if (std::isnan(ax->settings.source)) {
				ax->settings.source = 0;
				debug("setting axis %d %d source from nan to 0", s, a);
			}
			ax->settings.target = ax->settings.source + factor * (ax->settings.endpos - ax->settings.source);
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
	settings.hwtime += hwtime_step;
	mdebug("handling %d %d", computing_move, cbs_after_current_move);
	// This loop is normally only run once, but when a move is complete it is rerun for the next move.
	while ((running_fragment - 1 - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER > (FRAGMENTS_PER_BUFFER > 4 ? 4 : FRAGMENTS_PER_BUFFER - 2)) {
		double t = settings.hwtime / 1e6;
		double target_factor;	// Factor of current move that should be completed at this time.
		if (settings.hwtime >= settings.end_time)
			target_factor = 1;
		else if (settings.hwtime <= 0)
			target_factor = 0;
		else {
			target_factor = ((settings.v1 - settings.v0) / (settings.end_time / 1e6) * t * t / 2 + settings.v0 * t) / settings.dist;
			if (target_factor > 1)
				target_factor = 1;
		}
		mdebug("target factor: %f", target_factor);
		// Go straight to the next move if the distance was 0 (so target_factor is NaN).
		if (!std::isnan(target_factor)) {
			double f = set_targets(target_factor);
			if (f < 1) {
				target_factor = settings.factor + f * (target_factor - settings.factor);
				set_targets(target_factor);
				// TODO: Adjust time.
				//settings.start_time += (settings.hwtime - the_last_time) * ((1 - factor) * .99);
			}
			//debug("target factor %f time 0 -> %d -> %d v %f -> %f", target_factor, settings.hwtime, settings.end_time, settings.v0, settings.v1);
			settings.factor = target_factor;
			if (settings.factor < 1) {
				do_steps();
				return;
			}
		}
		mdebug("next segment");
		// Set new sources for all axes.
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a) {
				auto ax = sp.axis[a];
				ax->settings.source = ax->settings.target;
			}
		}
		// start new move; adjust time.
		num_movecbs = cbs_after_current_move;
		//debug("adding %d cbs because move is completed", cbs_after_current_move);
		cbs_after_current_move = next_move(settings.end_time);
		mdebug("next move prepared");
		if (!computing_move) {
			// There is no next move.
			continue_event = true;
			do_steps();
			return;
		}
		mdebug("try again");
	}
	//if (spaces[0].num_axes >= 2)
		//debug("move z %d %d %f %f %f", current_fragment, current_fragment_pos, spaces[0].axis[2]->settings.current, spaces[0].motor[0]->settings.current_pos, spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0]);
} // }}}

void store_settings() { // {{{
	current_fragment_pos = 0;
	num_active_motors = 0;
	if (FRAGMENTS_PER_BUFFER == 0)
		return;
	for (int i = 0; i < 3; ++i) {
		history[current_fragment].P[i] = settings.P[i];
		history[current_fragment].A[i] = settings.A[i];
		history[current_fragment].B[i] = settings.B[i];
	}
	history[current_fragment].v0 = settings.v0;
	history[current_fragment].v1 = settings.v1;
	history[current_fragment].dist = settings.dist;
	history[current_fragment].alpha_max = settings.alpha_max;
	history[current_fragment].hwtime = settings.hwtime;
	history[current_fragment].end_time = settings.end_time;
	history[current_fragment].last_time = settings.last_time;
	history[current_fragment].cbs = 0;
	history[current_fragment].queue_start = settings.queue_start;
	history[current_fragment].queue_end = settings.queue_end;
	history[current_fragment].queue_full = settings.queue_full;
	history[current_fragment].run_file_current = settings.run_file_current;
	history[current_fragment].probing = settings.probing;
	history[current_fragment].single = settings.single;
	history[current_fragment].run_time = settings.run_time;
	history[current_fragment].run_dist = settings.run_dist;
	history[current_fragment].factor = settings.factor;
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
	DATA_CLEAR();
} // }}}

void restore_settings() { // {{{
	current_fragment_pos = 0;
	num_active_motors = 0;
	if (FRAGMENTS_PER_BUFFER == 0)
		return;
	for (int i = 0; i < 3; ++i) {
		settings.P[i] = history[current_fragment].P[i];
		settings.A[i] = history[current_fragment].A[i];
		settings.B[i] = history[current_fragment].B[i];
	}
	settings.v0 = history[current_fragment].v0;
	settings.v1 = history[current_fragment].v1;
	settings.dist = history[current_fragment].dist;
	settings.alpha_max = history[current_fragment].alpha_max;
	settings.hwtime = history[current_fragment].hwtime;
	settings.end_time = history[current_fragment].end_time;
	settings.last_time = history[current_fragment].last_time;
	history[current_fragment].cbs = 0;
	settings.queue_start = history[current_fragment].queue_start;
	settings.queue_end = history[current_fragment].queue_end;
	settings.queue_full = history[current_fragment].queue_full;
	settings.run_file_current = history[current_fragment].run_file_current;
	settings.probing = history[current_fragment].probing;
	settings.single = history[current_fragment].single;
	settings.run_time = history[current_fragment].run_time;
	settings.run_dist = history[current_fragment].run_dist;
	settings.factor = history[current_fragment].factor;
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
	DATA_CLEAR();
} // }}}

void buffer_refill() { // {{{
	// Try to fill the buffer. This is called at any time that the buffer may be refillable.
	//debug("refill");
	if (preparing || FRAGMENTS_PER_BUFFER == 0) {
		//debug("no refill because prepare or no buffer yet");
		return;
	}
	if (!computing_move || refilling || stopping || discard_pending || discarding) {
		//debug("no refill due to block: %d %d %d %d %d", !computing_move, refilling, stopping, discard_pending, discarding);
		return;
	}
	refilling = true;
	// send_fragment in the previous refill may have failed; try it again.
	if (current_fragment_pos > 0)
		send_fragment();
	//debug("refill start %d %d %d", running_fragment, current_fragment, sending_fragment);
	// Keep one free fragment, because we want to be able to rewind and use the buffer before the one currently active.
	while (computing_move && !stopping && !discard_pending && !discarding && (running_fragment - 1 - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER > (FRAGMENTS_PER_BUFFER > 4 ? 4 : FRAGMENTS_PER_BUFFER - 2) && !sending_fragment) {
		//debug("refill %d %d %f", current_fragment, current_fragment_pos, spaces[0].motor[0]->settings.current_pos);
		// fill fragment until full.
		apply_tick();
		//debug("refill2 %d %f", current_fragment, spaces[0].motor[0]->settings.current_pos);
		if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
			//debug("fragment full %d %d %d", computing_move, current_fragment_pos, BYTES_PER_FRAGMENT);
			send_fragment();
		}
	}
	if (stopping || discard_pending) {
		//debug("aborting refill for stopping");
		refilling = false;
		return;
	}
	if (!computing_move && current_fragment_pos > 0) {
		//debug("finalize");
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
#ifdef DEBUG_MOVE
	debug("move no longer prepared");
#endif
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
	store_settings();
	computing_move = false;
	prepared = false;
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
	//debug("aborted move");
	aborting = false;
} // }}}

/* move.cpp - movement planning for Franklin
 * vim: set foldmethod=marker :
 * Copyright 2014 Michigan Technological University
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

//#define DEBUG_MOVE

// Set up:
// start_time		utime() at start of move.
// t0			time to do main part.
// tp			time to do connection.
// mtr->dist[0]		total distance of this segment (mm).
// mtr->dist[1]		total distance of next segment (mm).
// mtr->main_dist	distance of main part (mm).
// v0, vp		start and end velocity for main part. (fraction/s)
// vq			start velocity of connector part. (fraction/s)

static void change0(int qpos) { // {{{
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int a = 0; a < spaces[0].num_axes; ++a) {
			//debug("change %d %d %d %f", s, a, sp.type, queue[qpos].data[a]);
			queue[qpos].data[a] = space_types[sp.type].change0(&sp, a, queue[qpos].data[a]);
		}
	}
} // }}}

static void set_from_queue(int s, int qpos, int a0, bool next) { // {{{
	Space &sp = spaces[s];
	for (int a = 0; a < sp.num_axes; ++a) {
		sp.axis[a]->settings.endpos[1] = queue[qpos].data[a0 + a] + (s == 0 && a == 2 ? zoffset : 0);
		if (isnan(queue[qpos].data[a0 + a])) {
			sp.axis[a]->settings.dist[1] = 0;
		}
		else {
			sp.axis[a]->settings.dist[1] = queue[qpos].data[a0 + a] + (s == 0 && a == 2 ? zoffset : 0) - (next && !isnan(sp.axis[a]->settings.endpos[0]) ? sp.axis[a]->settings.endpos[0] : sp.axis[a]->settings.source);
#ifdef DEBUG_MOVE
			debug("setting dist for %d %d (+%d) queue (%d) %f next %d end %f src %f dist %f", s, a, a0, qpos, queue[qpos].data[a0 + a], next, sp.axis[a]->settings.endpos[0], sp.axis[a]->settings.source, sp.axis[a]->settings.dist[1]);
#endif
		}
	}
	if (s == 0 && queue[qpos].arc) { // {{{
		sp.settings.arc[1] = true;
		double src = 0, normal = 0, dst = 0;
		double target[3];
		double center[3];
		double source[3];
		double sn = 0, cn = 0, tn = 0;
		for (int i = 0; i < 3; ++i) {
			sp.settings.normal[1][i] = queue[qpos].normal[i];
			normal += sp.settings.normal[1][i] * sp.settings.normal[1][i];
		}
		normal = sqrt(normal);
		for (int i = 0; i < 3; ++i) {
			sp.settings.normal[1][i] /= normal;
			center[i] = queue[qpos].center[i] + (s == 0 && i == 2 ? zoffset : 0);
			source[i] = i < sp.num_axes ? next ? sp.axis[i]->settings.endpos[0] : sp.axis[i]->settings.source : 0;
			target[i] = queue[qpos].data[a0 + i] + (s == 0 && i == 2 ? zoffset : 0);
			sn += source[i] * sp.settings.normal[1][i];
			cn += center[i] * sp.settings.normal[1][i];
			tn += target[i] * sp.settings.normal[1][i];
			//debug("%d src %f target %f", i, source[i], target[i]);
		}
		sp.settings.helix[1] = tn - sn;
		//debug("helix %f %f %f", tn, sn, sp.settings.helix[1]);
		for (int i = 0; i < 3; ++i) {
			center[i] -= sp.settings.normal[1][i] * (cn - sn);
			target[i] -= sp.settings.normal[1][i] * sp.settings.helix[1];
			sp.settings.offset[1][i] = source[i] - center[i];
			src += sp.settings.offset[1][i] * sp.settings.offset[1][i];
			dst += (target[i] - center[i]) * (target[i] - center[i]);
		}
		src = sqrt(src);
		dst = sqrt(dst);
		for (int i = 0; i < 3; ++i) {
			sp.settings.e1[1][i] = sp.settings.offset[1][i] / src;
			sp.settings.normal[1][i] /= normal;
		}
		double cosa = 0, sina = 0;
		for (int i = 0; i < 3; ++i) {
			int c1 = (i + 1) % 3;
			int c2 = (i + 2) % 3;
			sp.settings.e2[1][i] = sp.settings.normal[1][c1] * sp.settings.e1[1][c2] - sp.settings.normal[1][c2] * sp.settings.e1[1][c1];
			cosa += sp.settings.e1[1][i] * (target[i] - center[i]) / dst;
			sina += sp.settings.e2[1][i] * (target[i] - center[i]) / dst;
		}
		sp.settings.angle[1] = atan2(sina, cosa);
		if (sp.settings.angle[1] <= 0)
			sp.settings.angle[1] += 2 * M_PI;
		sp.settings.radius[1][0] = src;
		sp.settings.radius[1][1] = dst;
		double d = (src + dst) / 2 * sp.settings.angle[1];
		sp.settings.dist[1] = sqrt(d * d + sp.settings.helix[1] * sp.settings.helix[1]);
	} // }}}
	else { // {{{
		sp.settings.arc[1] = false;
		sp.settings.angle[1] = NAN;
		sp.settings.helix[1] = NAN;
		for (int i = 0; i < 2; ++i) {
			sp.settings.radius[1][i] = NAN;
		}
		for (int i = 0; i < 3; ++i) {
			sp.settings.offset[1][i] = NAN;
			sp.settings.e1[1][i] = NAN;
			sp.settings.e2[1][i] = NAN;
			sp.settings.normal[1][i] = NAN;
		}
		double d = 0;
		for (int a = 0; a < min(3, sp.num_axes); ++a) {
			if (!isnan(sp.axis[a]->settings.dist[1]))
				d += sp.axis[a]->settings.dist[1] * sp.axis[a]->settings.dist[1];
		}
		sp.settings.dist[1] = sqrt(d);
	} // }}}
	if (isnan(sp.settings.dist[1]))
		sp.settings.dist[1] = 0;
} // }}}

static void copy_next(int s) { // {{{
	Space &sp = spaces[s];
	sp.settings.dist[0] = sp.settings.dist[1];
	sp.settings.dist[1] = 0;
	sp.settings.arc[0] = sp.settings.arc[1];
	sp.settings.angle[0] = sp.settings.angle[1];
	sp.settings.helix[0] = sp.settings.helix[1];
	for (int i = 0; i < 2; ++i) {
		sp.settings.radius[0][i] = sp.settings.radius[1][i];
	}
	for (int i = 0; i < 3; ++i) {
		sp.settings.offset[0][i] = sp.settings.offset[1][i];
		sp.settings.e1[0][i] = sp.settings.e1[1][i];
		sp.settings.e2[0][i] = sp.settings.e2[1][i];
		sp.settings.normal[0][i] = sp.settings.normal[1][i];
	}
	for (int a = 0; a < sp.num_axes; ++a) {
		sp.axis[a]->settings.endpos[0] = sp.axis[a]->settings.endpos[1];
		sp.axis[a]->settings.dist[0] = sp.axis[a]->settings.dist[1];
		sp.axis[a]->settings.dist[1] = 0;
#ifdef DEBUG_MOVE
		debug("Last segment distance for motor %d is %f", a, sp.axis[a]->settings.dist[0]);
#endif
	}
} // }}}

// Used from previous segment (if prepared): tp, vq.
int next_move() { // {{{
	settings.probing = false;
	settings.single = false;
	moving_to_current = 0;
	int num_cbs = 0;
	int a0;
	run_file_fill_queue();
	if (settings.queue_start == settings.queue_end && !settings.queue_full) {
		//debug("no next move");
		prepared = false;
		return num_cbs;
	}
#ifdef DEBUG_MOVE
	debug("Next move; queue start = %d, end = %d", settings.queue_start, settings.queue_end);
#endif
	// Set everything up for running queue[settings.queue_start].
	int n = (settings.queue_start + 1) % QUEUE_LENGTH;

	// Make sure printer state is good. {{{
	// If the source is unknown, determine it from current_pos.
	//for (int a = 0; a < num_axes; ++a)
	//	debug("target %d %f", a, queue[settings.queue_start].data[a]);
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int a = 0; a < sp.num_axes; ++a) {
			if (isnan(sp.axis[a]->settings.source)) {
				if (!isnan(sp.axis[a]->settings.current)) {
					sp.axis[a]->settings.source = sp.axis[a]->settings.current;
					continue;
				}
				space_types[sp.type].reset_pos(&sp);
				for (int aa = 0; aa < sp.num_axes; ++aa)
					sp.axis[aa]->settings.current = sp.axis[aa]->settings.source;
				break;
			}
#ifdef DEBUG_MOVE
			else
				debug("non-nan: %d %d %f %f", s, a, sp.axis[a]->settings.source, sp.motor[a]->settings.current_pos);
#endif
		}
	}
	// }}}

	settings.f0 = settings.fq;
	// If no move is prepared, set dist[1] from the queue; it will be used as dist[0] below. {{{
	if (!prepared) {
#ifdef DEBUG_MOVE
		debug("No move prepared.");
#endif
		settings.f0 = 0;
		a0 = 0;
		change0(settings.queue_start);
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			space_types[sp.type].check_position(&sp, &queue[settings.queue_start].data[a0]);
			sp.settings.dist[0] = 0;
			for (int a = 0; a < sp.num_axes; ++a) {
				sp.axis[a]->settings.dist[0] = 0;
				sp.axis[a]->settings.endpos[0] = sp.axis[a]->settings.source;
			}
			set_from_queue(s, settings.queue_start, a0, false);
			a0 += sp.num_axes;
		}
	}
	// }}}
	// Fill unspecified coordinates with previous values. {{{
	a0 = 0;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int a = 0; a < sp.num_axes; ++a) {
			if (n != settings.queue_end) {
				// If only one of them is set, set the other one as well to make the rounded corner work.
				if (!isnan(queue[settings.queue_start].data[a0 + a]) && isnan(queue[n].data[a0 + a])) {
					queue[n].data[a0 + a] = sp.axis[a]->settings.source + sp.axis[a]->settings.dist[1] - (s == 0 && a == 2 ? zoffset : 0);
#ifdef DEBUG_MOVE
					debug("filling next %d with %f", a0 + a, queue[n].data[a0 + a]);
#endif
				}
				if (isnan(queue[settings.queue_start].data[a]) && !isnan(queue[n].data[a])) {
					queue[settings.queue_start].data[a0 + a] = sp.axis[a]->settings.source;
#ifdef DEBUG_MOVE
					debug("filling %d with %f", a0 + a, queue[settings.queue_start].data[a0 + a]);
#endif
				}
			}
			if ((!isnan(queue[settings.queue_start].data[a0 + a]) || (n != settings.queue_end && !isnan(queue[n].data[a0 + a]))) && isnan(sp.axis[a]->settings.source)) {
				debug("Motor positions are not known, so move cannot take place; aborting move and removing it from the queue: %f %f %f", queue[settings.queue_start].data[a0 + a], queue[n].data[a0 + a], sp.axis[a]->settings.source);
				// This possibly removes one move too many, but it shouldn't happen anyway.
				if (queue[settings.queue_start].cb)
					++num_cbs;
				if (settings.queue_end == settings.queue_start)
					send_host(CMD_CONTINUE, 0);
				settings.queue_start = n;
				settings.queue_full = false;
				abort_move(current_fragment_pos);
				return num_cbs;
			}
		}
		a0 += sp.num_axes;
	}
	// }}}
	// We are prepared and can start the segment.
	bool action = false;
	double v1;
	if (n == settings.queue_end) { // There is no next segment; we should stop at the end. {{{
		prepared = false;
#ifdef DEBUG_MOVE
		debug("Building final segment.");
#endif
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			copy_next(s);
			if (sp.settings.dist[0] != 0)
				action = true;
		}
		v1 = 0;
	}
	// }}}
	else { // There is a next segment; we should connect to it. {{{
		prepared = true;
#ifdef DEBUG_MOVE
		debug("Building a connecting segment.");
#endif
		a0 = 0;
		change0(n);
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			space_types[sp.type].check_position(&sp, &queue[n].data[a0]);
			copy_next(s);
			set_from_queue(s, n, a0, true);
			if (sp.settings.dist[1] != 0 || sp.settings.dist[0] != 0)
				action = true;
			a0 += sp.num_axes;
		}
		v1 = queue[n].f[0] * feedrate;
	}
	// }}}

	double v0 = queue[settings.queue_start].f[0] * feedrate;
	double vp = queue[settings.queue_start].f[1] * feedrate;
	settings.probing = queue[settings.queue_start].probe;
	settings.single = queue[settings.queue_start].single;
	settings.run_time = queue[settings.queue_start].time;
	settings.run_dist = queue[settings.queue_start].dist;

	if (queue[settings.queue_start].cb) {
		cbs_after_current_move += 1;
		//debug("cbs after current inc'd to %d", cbs_after_current_move);
	}
	//debug("add cb to current starting at %d", current_fragment);
	if (settings.queue_end == settings.queue_start)
		send_host(CMD_CONTINUE, 0);
	settings.queue_full = false;
	settings.queue_start = n;

	if (!action) {	// Skip zero-distance move. {{{
#ifdef DEBUG_MOVE
		debug("Skipping zero-distance prepared move (cbs %d)", cbs_after_current_move);
#endif
		num_cbs += cbs_after_current_move;
		//debug("cbs after current cleared for return from next move as %d+%d", num_cbs, cbs_after_current_move);
		cbs_after_current_move = 0;
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			sp.settings.dist[0] = 0;
			for (int a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings.dist[0] = NAN;
		}
		settings.fq = 0;
		return num_cbs + next_move();
	} // }}}

	// Currently set up:
	// f0: fraction of move already done by connection.
	// v0: this move's requested starting speed.
	// vp: this move's requested ending speed.
	// v1: next move's requested starting speed.
	// cbs_after_current_move: number of cbs that should be fired after this segment is complete.
	// dist[0]: total distance of this segment (mm).
	// dist[1]: total distance of next segment (mm).
	// mtr->dist[0]: motor distance of this segment (mm).
	// mtr->dist[1]: motor distance of next segment (mm).
#ifdef DEBUG_MOVE
	debug("Set up: v0 = %f /s, vp = %f /s, vq = %f /s", v0, vp, v1);
#endif

	// Limit v0, vp, vq. {{{
	double vq = v1;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		double limit;
		if (s == 0)
			limit = max_v;
		else if (s == 1 && current_extruder < sp.num_motors)
			limit = sp.motor[current_extruder]->limit_v;
		else
			continue;
		if (isnan(limit) || isinf(limit) || limit <= 0)
			continue;
		// max_mm is the maximum speed in mm/s.
		double max_mm = settings.probing ? space_types[sp.type].probe_speed(&sp) : limit;
		double max = max_mm / sp.settings.dist[0];
#ifdef DEBUG_MOVE
		debug("limiting space %d, max_mm = %f, max = %f, limit = %f", s, max_mm, max, limit);
#endif
		if (v0 < 0)
			v0 = -v0 / sp.settings.dist[0];
		if (vp < 0)
			vp = -vp / sp.settings.dist[0];
		if (vq < 0)
			vq = -vq / sp.settings.dist[1];
		if (v0 > max)
			v0 = max;
		if (vp > max)
			vp = max;
		max = max_mm / sp.settings.dist[1];
		if (vq > max)
			vq = max;
	}
#ifdef DEBUG_MOVE
	debug("After limiting, v0 = %f /s, vp = %f /s and vq = %f /s", v0, vp, vq);
#endif
	// }}}
	// Already set up: f0, v0, vp, vq, dist[0], dist[1], mtr->dist[0], mtr->dist[1].
	// To do: start_time, t0, tp, fmain, fp, fq, mtr->main_dist
#ifdef DEBUG_MOVE
	debug("Preparation did f0 = %f", settings.f0);
#endif

	// Use maximum deviation to find fraction where to start rounded corner. {{{
	double factor = vq / vp;
	done_factor = NAN;
	if (vq == 0) {
		settings.fp = 0;
		settings.fq = 0;
	}
	else {
		settings.fp = factor > 1 ? .5 / factor : .5;
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			if (sp.num_axes < 2)
				continue;
			if (s != 0 || max_deviation == 0) {
				settings.fp = 0;
				break;
			}
			double nd = sp.settings.dist[1] * factor;
			double d = sp.settings.dist[0] - sp.settings.dist[1];
			// Calculate distances and ignore spaces which don't have two segments.
			if (nd <= 0)
				continue;
			if (sp.settings.dist[0] <= 0)
				continue;
			double done = 1 - max_deviation / sp.settings.dist[0];
			// Set it also if done_factor is NaN.
			if (!(done <= done_factor))
				done_factor = done;
			double new_fp = max_deviation / sqrt(nd / (sp.settings.dist[0] + nd) * d);
#ifdef DEBUG_MOVE
			debug("Space %d fp %f dev %f", s, settings.fp, max_deviation);
#endif
			if (new_fp < settings.fp)
				settings.fp = new_fp;
		}
		if (isnan(done_factor))
			settings.fq = 0;
		else
			settings.fq = settings.fp * factor;
	}
	if (isnan(done_factor))
		done_factor = 1;
	// }}}

	settings.t0 = (1 - settings.fp) / (fabs(v0 + vp) / 2);
	settings.tp = settings.fp / (fabs(vp) / 2);
	settings.f1 = .5 * fabs(v0) * settings.t0;
	settings.f2 = 1 - settings.fp - settings.f1;

	// Set up endpos. {{{
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int a = 0; a < sp.num_axes; ++a) {
			sp.axis[a]->settings.main_dist = sp.axis[a]->settings.dist[0] * (1 - settings.fp);
			// Fill target for filling endpos below.
			if ((sp.axis[a]->settings.dist[0] > 0 && sp.axis[a]->settings.dist[1] < 0) || (sp.axis[a]->settings.dist[0] < 0 && sp.axis[a]->settings.dist[1] > 0))
				sp.axis[a]->settings.target = sp.axis[a]->settings.source + sp.axis[a]->settings.dist[0];
			else
				sp.axis[a]->settings.target = sp.axis[a]->settings.source + sp.axis[a]->settings.dist[0] + sp.axis[a]->settings.dist[1] * settings.fq;
#ifdef DEBUG_MOVE
			debug("Axis %d %d dist %f main dist = %f, next dist = %f currentpos = %f current = %f", s, a, sp.axis[a]->settings.dist[0], sp.axis[a]->settings.main_dist, sp.axis[a]->settings.dist[1], sp.motor[a]->settings.current_pos, sp.axis[a]->settings.current);
#endif
		}
		// Using NULL as target fills endpos.
		space_types[sp.type].xyz2motors(&sp, NULL);
	}
	// }}}

	// Enable motors if they weren't. {{{
	if (!motors_busy) {
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int m = 0; m < sp.num_motors; ++m)
				SET(sp.motor[m]->enable_pin);
		}
		motors_busy = true;
	} // }}}
#ifdef DEBUG_MOVE
	debug("Segment has been set up: f0=%f fp=%f fq=%f v0=%f /s vp=%f /s vq=%f /s t0=%f s tp=%f s", settings.f0, settings.fp, settings.fq, v0, vp, vq, settings.t0, settings.tp);
#endif
	// Reset time. {{{
	settings.hwtime = 0;
	settings.last_time = 0;
	settings.last_current_time = 0;
	settings.start_time = settings.last_time - int32_t(settings.f0 / fabs(vp) * 1e6);
	// }}}

	if (!computing_move) {	// Set up source if this is a new move. {{{
#ifdef DEBUG_MOVE
		debug("starting new move");
#endif
		//debug("current %d running %d", current_fragment, running_fragment);
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings.source = sp.axis[a]->settings.current;
		}
		store_settings();
#ifdef DEBUG_PATH
		fprintf(stderr, "\n");
#endif
	} // }}}

	first_fragment = current_fragment;	// Do this every time, because otherwise the queue must be regenerated.	TODO: send partial fragment to make sure this hack actually works, or fix it properly.
	computing_move = true;
	return num_cbs;
} // }}}

void abort_move(int pos) { // {{{
	aborting = true;
	//debug("abort pos %d", pos);
	//debug("abort; cf %d rf %d first %d computing_move %d fragments, regenerating %d ticks", current_fragment, running_fragment, first_fragment, computing_move, pos);
	//debug("try aborting move");
	current_fragment = running_fragment;
	//debug("current abort -> %x", current_fragment);
	while (pos < 0) {
		if (current_fragment == first_fragment) {
			pos = 0;
		}
		else {
			current_fragment = (current_fragment + FRAGMENTS_PER_BUFFER - 1) % FRAGMENTS_PER_BUFFER;
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
			sp.axis[a]->settings.source = sp.axis[a]->settings.current;
			sp.axis[a]->settings.dist[0] = NAN;
			sp.axis[a]->settings.dist[1] = NAN;
		}
		for (int m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->settings.last_v = 0;
			//debug("setting motor %d pos to %f", m, sp.motor[m]->settings.current_pos);
		}
	}
	//debug("aborted move");
	aborting = false;
} // }}}

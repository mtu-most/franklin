/* move.cpp - movement planning for Franklin
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
 */

#include "cdriver.h"

//#define mdebug debug

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
	if ((std::isnan(settings.dist) || abs(settings.dist) < 1e-10) && queue[q].tool < spaces[1].num_axes)
		settings.dist = abs(queue[q].e - spaces[1].axis[queue[q].tool]->settings.source);
	double dt = settings.dist / ((settings.v0 + settings.v1) / 2);
	settings.end_time = (std::isnan(dt) ? 0 : 1e6 * dt);
	if (queue[q].tool < spaces[1].num_axes && !std::isnan(queue[q].e)) {
		spaces[1].axis[queue[q].tool]->settings.endpos = queue[q].e;
		mdebug("move extruder to %f", queue[q].e);
	}
	else
		spaces[1].axis[queue[q].tool]->settings.endpos = spaces[1].axis[queue[q].tool]->settings.current;
	mdebug("move prepared, Q=(%f,%f,%f) P=(%f,%f,%f), A=(%f,%f,%f), B=(%f,%f,%f), max alpha=%f, dist=%f, e=%f, v0=%f, v1=%f, end time=%f", queue[q].X[0], queue[q].X[1], queue[q].X[2], settings.P[0], settings.P[1], settings.P[2], settings.A[0], settings.A[1], settings.A[2], settings.B[0], settings.B[1], settings.B[2], settings.alpha_max, settings.dist, queue[q].e, settings.v0, settings.v1, settings.end_time / 1e6);
	//debug("times end %d current %d dist %f v0 %f v1 %f", settings.end_time, settings.hwtime, settings.dist, settings.v0, settings.v1);

	settings.queue_start = n;
	first_fragment = current_fragment;	// Do this every time, because otherwise the queue must be regenerated.	TODO: send partial fragment to make sure this hack actually works, or fix it properly.
	computing_move = true;
	return num_cbs;
} // }}}

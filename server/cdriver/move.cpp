// vim: set foldmethod=marker :
#include "cdriver.h"

//#define DEBUG_MOVE

// Set up:
// start_time		utime() at start of move.
// t0			time to do main part.
// tp			time to do connection.
// mtr->dist		total distance of this segment (mm).
// mtr->next_dist		total distance of next segment (mm).
// mtr->main_dist		distance of main part (mm).
// v0, vp		start and end velocity for main part. (fraction/s)
// vq			end velocity of connector part. (fraction/s)

static void change0(int qpos) {
	if (num_spaces < 1)
		return;
	for (int s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (int a = 0; a < spaces[0].num_axes; ++a)
			queue[qpos].data[a] = space_types[sp.type].change0(&sp, a, queue[qpos].data[a]);
	}
}

// Used from previous segment (if prepared): tp, vq.
uint8_t next_move() {
	probing = false;
	uint8_t num_cbs = 0;
	uint8_t a0;
	if (queue_start == queue_end && !queue_full) {
		stopped = true;
		prepared = false;
		return num_cbs;
	}
#ifdef DEBUG_MOVE
	debug("Next move; queue start = %d, end = %d", queue_start, queue_end);
#endif
	// Set everything up for running queue[queue_start].
	uint8_t n = (queue_start + 1) % QUEUE_LENGTH;

	// Make sure printer state is good. {{{
	// If the source is unknown, determine it from current_pos.
	//for (uint8_t a = 0; a < num_axes; ++a)
	//	debug("target %d %f", a, queue[queue_start].data[a]);
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			if (isnan(sp.axis[a]->settings[current_fragment].source)) {
				space_types[sp.type].reset_pos(&sp);
				for (uint8_t aa = 0; aa < sp.num_axes; ++aa)
					sp.axis[aa]->settings[current_fragment].current = sp.axis[aa]->settings[current_fragment].source;
				break;
			}
#ifdef DEBUG_MOVE
			else
				debug("non-nan: %d %d %f %d", s, a, sp.axis[a]->settings[current_fragment].source, sp.motor[a]->settings[current_fragment].current_pos);
#endif
		}
	}
	// }}}

	settings[current_fragment].f0 = settings[current_fragment].fq;
	// If no move is prepared, set next_dist from the queue; it will be used as dist below.
	if (!prepared) { // {{{
#ifdef DEBUG_MOVE
		debug("No move prepared.");
#endif
		settings[current_fragment].f0 = 0;
		a0 = 0;
		change0(queue_start);
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			space_types[sp.type].check_position(&sp, &queue[queue_start].data[a0]);
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if (isnan(queue[queue_start].data[a0 + a])) {
					sp.axis[a]->settings[current_fragment].next_dist = 0;
					//debug("not using object %d", mtr);
				}
				else {
					sp.axis[a]->settings[current_fragment].next_dist = queue[queue_start].data[a0 + a] - sp.axis[a]->settings[current_fragment].source + sp.axis[a]->offset;
#ifdef DEBUG_MOVE
					debug("next dist of %d = %f (%f - %f + %f)", a0 + a, sp.axis[a]->settings[current_fragment].next_dist, queue[queue_start].data[a0 + a], sp.axis[a]->settings[current_fragment].source, sp.axis[a]->offset);
#endif
				}
			}
			a0 += sp.num_axes;
		}
	}
	a0 = 0;
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			if (n != queue_end) {
				// If only one of them is set, set the other one as well to make the rounded corner work.
				if (!isnan(queue[queue_start].data[a0 + a]) && isnan(queue[n].data[a0 + a])) {
					queue[n].data[a0 + a] = sp.axis[a]->settings[current_fragment].source + sp.axis[a]->settings[current_fragment].next_dist - sp.axis[a]->offset;
#ifdef DEBUG_MOVE
					debug("filling next %d with %f", a0 + a, queue[n].data[a0 + a]);
#endif
				}
				if (isnan(queue[queue_start].data[a]) && !isnan(queue[n].data[a])) {
					queue[queue_start].data[a0 + a] = sp.axis[a]->settings[current_fragment].source - sp.axis[a]->offset;
#ifdef DEBUG_MOVE
					debug("filling %d with %f", a0 + a, queue[queue_start].data[a0 + a]);
#endif
				}
			}
			if ((!isnan(queue[queue_start].data[a0 + a]) || (n != queue_end && !isnan(queue[n].data[a0 + a]))) && isnan(sp.axis[a]->settings[current_fragment].source)) {
				debug("Motor positions are not known, so move cannot take place; aborting move and removing it from the queue: %f %f %f", queue[queue_start].data[a0 + a], queue[n].data[a0 + a], sp.axis[a]->settings[current_fragment].source);
				// This possibly removes one move too many, but it shouldn't happen anyway.
				if (queue[queue_start].cb)
					++num_cbs;
				if (queue_end == queue_start)
					send_host(CMD_CONTINUE, 0);
				queue_start = n;
				queue_full = false;
				abort_move(current_fragment_pos);
				return num_cbs;
			}
		}
		a0 += sp.num_axes;
	}
	// }}}
	// We are prepared and can start the segment. {{{
	bool action = false;
#ifdef DEBUG_MOVE
	debug("Move was prepared");
#endif
	float vq;
	if (n == queue_end) { // {{{
		// There is no next segment; we should stop at the end.
		prepared = false;
#ifdef DEBUG_MOVE
		debug("Building final segment.");
#endif
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				sp.axis[a]->settings[current_fragment].dist = sp.axis[a]->settings[current_fragment].next_dist;
				if (sp.axis[a]->settings[current_fragment].dist != 0)
					action = true;
				sp.axis[a]->settings[current_fragment].next_dist = 0;
#ifdef DEBUG_MOVE
				debug("Last segment distance for motor %d is %f", a, sp.axis[a]->settings[current_fragment].dist);
#endif
			}
		}
		vq = 0;
	}
	// }}}
	else { // {{{
		// There is a next segment; we should connect to it.
		prepared = true;
#ifdef DEBUG_MOVE
		debug("Building a connecting segment.");
#endif
		a0 = 0;
		change0(n);
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			space_types[sp.type].check_position(&sp, &queue[n].data[a0]);
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				sp.axis[a]->settings[current_fragment].dist = sp.axis[a]->settings[current_fragment].next_dist;
				if (isnan(queue[n].data[a0 + a]))
					sp.axis[a]->settings[current_fragment].next_dist = 0;
				else
					sp.axis[a]->settings[current_fragment].next_dist = queue[n].data[a0 + a] + sp.axis[a]->offset - (sp.axis[a]->settings[current_fragment].source + sp.axis[a]->settings[current_fragment].dist);
				if (sp.axis[a]->settings[current_fragment].next_dist != 0 || sp.axis[a]->settings[current_fragment].dist != 0)
					action = true;
#ifdef DEBUG_MOVE
				debug("Connecting distance for motor %d is %f, to %f = %f + %f - (%f + %f)", a, sp.axis[a]->settings[current_fragment].dist, sp.axis[a]->settings[current_fragment].next_dist, queue[n].data[a0 + a], sp.axis[a]->offset, sp.axis[a]->settings[current_fragment].source, sp.axis[a]->settings[current_fragment].dist);
#endif
			}
			a0 += sp.num_axes;
		}
		vq = queue[n].f[0] * feedrate;
	}
	// }}}
	float v0 = queue[queue_start].f[0] * feedrate;
	float vp = queue[queue_start].f[1] * feedrate;
	//debug("mv %f %f %f %f", feedrate, v0, vp, vq);
	if (queue[queue_start].cb)
		cbs_after_current_move += 1;
	if (queue_end == queue_start)
		send_host(CMD_CONTINUE, 0);
	probing = queue[queue_start].probe;
	queue_start = n;
	queue_full = false;
	if (!action) {
#ifdef DEBUG_MOVE
		debug("Skipping zero-distance prepared move");
#endif
		num_cbs += cbs_after_current_move;
		cbs_after_current_move = 0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings[current_fragment].dist = NAN;
		}
		settings[current_fragment].fq = 0;
		return num_cbs + next_move();
	}
	// }}}

	// Currently set up:
	// f0: fraction of move already done by connection.
	// v0: this move's requested starting speed.
	// vp: this move's requested ending speed.
	// vq: next move's requested starting speed.
	// cbs_after_current_move: number of cbs that should be fired after this segment is complete.
	// mtr->dist: total distance of this segment (mm).
	// mtr->next_dist: total distance of next segment (mm).
#ifdef DEBUG_MOVE
	debug("Set up: v0 = %f /s, vp = %f /s, vq = %f /s", v0, vp, vq);
#endif

	// Limit v0, vp, vq. {{{
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			if (!isnan(sp.axis[a]->max_v)) {
				float max;
				if (sp.axis[a]->settings[current_fragment].dist != 0) {
					max = sp.axis[a]->max_v / fabs(sp.axis[a]->settings[current_fragment].dist);
					if (v0 > max) {
						v0 = max;
						//debug("limited v0 to %f for max v %f dist %f so %f", v0, sp.axis[a].max_v, sp.axis[a].dist, max);
					}
					if (vp > max)
						vp = max;
				}
				if (sp.axis[a]->settings[current_fragment].next_dist != 0) {
					max = sp.axis[a]->max_v / fabs(sp.axis[a]->settings[current_fragment].next_dist);
					if (vq > max)
						vq = max;
				}
			}
		}
	}
#ifdef DEBUG_MOVE
	debug("After limiting, v0 = %f /s, vp = %f /s and vq = %f /s", v0, vp, vq);
#endif
	// }}}
	// Already set up: f0, v0, vp, vq, mtr->dist, mtr->next_dist.
	// To do: start_time, t0, tp, fmain, fp, fq, mtr->main_dist
#ifdef DEBUG_MOVE
	debug("Preparation did f0 = %f", settings[current_fragment].f0);
#endif

	// Use maximum deviation to find fraction where to start rounded corner.
	float factor = vq / vp;
	done_factor = NAN;
	if (vq == 0) {
		settings[current_fragment].fp = 0;
		settings[current_fragment].fq = 0;
	}
	else {
		settings[current_fragment].fp = factor > 1 ? .5 / factor : .5;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			if (sp.num_axes < 2)
				continue;
			if (sp.max_deviation == 0) {
				settings[current_fragment].fp = 0;
				break;
			}
			float norma2 = 0, normb2 = 0, diff2 = 0;
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				float nd = sp.axis[a]->settings[current_fragment].next_dist * factor;
				norma2 += sp.axis[a]->settings[current_fragment].dist * sp.axis[a]->settings[current_fragment].dist;
				normb2 += nd * nd;
				float d = sp.axis[a]->settings[current_fragment].dist - sp.axis[a]->settings[current_fragment].next_dist;
				diff2 += d * d;
			}
			// Calculate distances and ignore spaces which don't have two segments.
			float normb(sqrt(normb2));
			if (normb <= 0)
				continue;
			float norma(sqrt(norma2));
			if (norma <= 0)
				continue;
			float done = 1 - sp.max_deviation / norma;
			// Set it also if done_factor is NaN.
			if (!(done <= done_factor))
				done_factor = done;
			float new_fp = sp.max_deviation / sqrt(normb / (norma + normb) * diff2);
#ifdef DEBUG_MOVE
			debug("Space %d fp %f dev %f", s, settings[current_fragment].fp, sp.max_deviation);
#endif
			if (new_fp < settings[current_fragment].fp)
				settings[current_fragment].fp = new_fp;
		}
		settings[current_fragment].fq = settings[current_fragment].fp * factor;
	}
	if (isnan(done_factor))
		done_factor = 1;
	// Set up t0, tp.
	settings[current_fragment].t0 = (1 - settings[current_fragment].fp) / (fabs(v0 + vp) / 2);
	settings[current_fragment].tp = settings[current_fragment].fp / (fabs(vp) / 2);
	// Set up f1, f2.
	settings[current_fragment].f1 = .5 * fabs(v0) * settings[current_fragment].t0;
	settings[current_fragment].f2 = 1 - settings[current_fragment].fp - settings[current_fragment].f1;

	// Finish. {{{
	bool anything = false;
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		//debug("space %d axes %d motors %d", s, sp.num_axes, sp.num_motors);
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			//debug("axis %d", a);
			sp.axis[a]->settings[current_fragment].main_dist = sp.axis[a]->settings[current_fragment].dist * (1 - settings[current_fragment].fp);
			// Fill target for filling endpos below.
			if ((sp.axis[a]->settings[current_fragment].dist > 0 && sp.axis[a]->settings[current_fragment].next_dist < 0) || (sp.axis[a]->settings[current_fragment].dist < 0 && sp.axis[a]->settings[current_fragment].next_dist > 0))
				sp.axis[a]->settings[current_fragment].target = sp.axis[a]->settings[current_fragment].source + sp.axis[a]->settings[current_fragment].dist;
			else
				sp.axis[a]->settings[current_fragment].target = sp.axis[a]->settings[current_fragment].source + sp.axis[a]->settings[current_fragment].dist + sp.axis[a]->settings[current_fragment].next_dist; // * settings[current_fragment].fq;
#ifdef DEBUG_MOVE
			debug("Axis %d %d dist %f main dist = %f, next dist = %f currentpos = %d hw = %d current = %f", s, a, sp.axis[a]->settings[current_fragment].dist, sp.axis[a]->settings[current_fragment].main_dist, sp.axis[a]->settings[current_fragment].next_dist, sp.motor[a]->settings[current_fragment].current_pos, sp.motor[a]->settings[current_fragment].hwcurrent_pos, sp.axis[a]->settings[current_fragment].current);
#endif
		}
		bool ok = true;
		// Using NULL as target fills endpos.
		space_types[sp.type].xyz2motors(&sp, NULL, &ok);
		for (int m = 0; m < sp.num_motors; ++m) {
			float ep = sp.motor[m]->settings[current_fragment].endpos * sp.motor[m]->steps_per_m;
			int iep = int(ep + (ep > 0 ? .49 : -.49));
			if (sp.motor[m]->settings[current_fragment].current_pos != iep) {
				anything = true;
				break;
			}
			//debug("any %d %d %d %d %d", s, m, anything, sp.motor[m]->settings[current_fragment].current_pos, iep);
		}
	}
	if (!anything) {
		num_cbs += cbs_after_current_move;
		cbs_after_current_move = 0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings[current_fragment].dist = NAN;
		}
		settings[current_fragment].fq = 0;
		return num_cbs + next_move();
	}
	if (!motors_busy) {
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t m = 0; m < sp.num_motors; ++m)
				SET(sp.motor[m]->enable_pin);
		}
		motors_busy = true;
	}
#ifdef DEBUG_MOVE
	debug("Segment has been set up: f0=%f fp=%f fq=%f v0=%f /s vp=%f /s vq=%f /s t0=%f s tp=%f s", settings[current_fragment].f0, settings[current_fragment].fp, settings[current_fragment].fq, v0, vp, vq, settings[current_fragment].t0, settings[current_fragment].tp);
#endif
	if (stopped) {
#ifdef DEBUG_MOVE
		debug("starting new move");
#endif
		if (current_fragment_pos <= 0) {
			// Copy all settings to previous fragment, in case this fragment gets interrupted.
			copy_fragment_settings(current_fragment, (current_fragment - 1 + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER);
		}
		//debug("curf1 %d", current_fragment);
		// Reset time.
		settings[current_fragment].hwtime = 0;
		settings[current_fragment].last_time = 0;
		settings[current_fragment].last_current_time = 0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings[current_fragment].source = sp.axis[a]->settings[current_fragment].current;
		}
#ifdef DEBUG_PATH
		fprintf(stderr, "\n");
#endif
	}
	//debug("last=%ld", long(settings[current_fragment].last_time));
	first_fragment = current_fragment;	// Do this every time, because otherwise the queue must be regenerated.	TODO: send partial fragment to make sure this hack actually works, or fix it properly.
	stopped = false;
	moving = true;
	settings[current_fragment].start_time = settings[current_fragment].last_time - uint32_t(settings[current_fragment].f0 / fabs(vp) * 1e6);
	//debug("start=%ld, last-start=%ld", long(settings[current_fragment].start_time), long(settings[current_fragment].last_time - settings[current_fragment].start_time));
	// }}}
	return num_cbs;
}

void abort_move(int pos) { // {{{
	aborting = true;
	//debug("abort; cf %d ff %d first %d moving %d discarding %d fragments, regenerating %d ticks", current_fragment, free_fragments, first_fragment, moving, FRAGMENTS_PER_BUFFER - free_fragments - 2, pos);
	//debug("try aborting move");
	int prev_f = (current_fragment + free_fragments + 1) % FRAGMENTS_PER_BUFFER;	// +1 because free_fragments starts as FPB-1.
	int f = (prev_f + 1) % FRAGMENTS_PER_BUFFER;
	if (!stopped && pos < 0 && first_fragment != f) {
		f = prev_f;
		free_fragments += 1;
		//debug("free abort +1 %d", free_fragments);
		pos += settings[f].fragment_length;
		prev_f = (f - 1 + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
	}
	if (pos < 0)
		pos = 0;
	//debug("abort really regenerating %d ticks", pos);
	copy_fragment_settings(prev_f, f);
	current_fragment = f;
#ifdef DEBUG_MOVE
	debug("move no longer prepared");
#endif
	//debug("free abort reset");
	free_fragments = FRAGMENTS_PER_BUFFER - 1;
	current_fragment_pos = -1;
	moving = true;
	while (current_fragment_pos < pos)
		apply_tick();
	//debug("done restoring position");
	// Copy settings back to previous fragment.
	copy_fragment_settings(f, prev_f);
	moving = false;
	stopped = true;
	prepared = false;
	reset_dirs(f, false);
	//debug("curf3 %d", current_fragment);
	//debug("aborting move");
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			//debug("setting axis %d source to %f", a, sp.axis[a]->settings[current_fragment].current);
			sp.axis[a]->settings[current_fragment].source = sp.axis[a]->settings[current_fragment].current;
			sp.axis[a]->settings[current_fragment].dist = NAN;
			sp.axis[a]->settings[current_fragment].next_dist = NAN;
		}
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->settings[current_fragment].last_v = 0;
			sp.motor[m]->settings[current_fragment].hwcurrent_pos = sp.motor[m]->settings[current_fragment].current_pos;
			//debug("setting motor %d pos to %d", m, sp.motor[m]->settings[current_fragment].current_pos);
		}
	}
	aborting = false;
} // }}}

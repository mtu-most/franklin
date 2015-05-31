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
	run_file_fill_queue();
	if (settings.queue_start == settings.queue_end && !settings.queue_full) {
		//debug("no next move");
		stopped = true;
		prepared = false;
		return num_cbs;
	}
#ifdef DEBUG_MOVE
	debug("Next move; queue start = %d, end = %d", settings.queue_start, settings.queue_end);
#endif
	// Set everything up for running queue[settings.queue_start].
	uint8_t n = (settings.queue_start + 1) % QUEUE_LENGTH;

	// Make sure printer state is good. {{{
	// If the source is unknown, determine it from current_pos.
	//for (uint8_t a = 0; a < num_axes; ++a)
	//	debug("target %d %f", a, queue[settings.queue_start].data[a]);
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			if (isnan(sp.axis[a]->settings.source)) {
				space_types[sp.type].reset_pos(&sp);
				for (uint8_t aa = 0; aa < sp.num_axes; ++aa)
					sp.axis[aa]->settings.current = sp.axis[aa]->settings.source;
				break;
			}
#ifdef DEBUG_MOVE
			else
				debug("non-nan: %d %d %f %d", s, a, sp.axis[a]->settings.source, sp.motor[a]->settings.current_pos);
#endif
		}
	}
	// }}}

	settings.f0 = settings.fq;
	// If no move is prepared, set next_dist from the queue; it will be used as dist below.
	if (!prepared) { // {{{
#ifdef DEBUG_MOVE
		debug("No move prepared.");
#endif
		settings.f0 = 0;
		a0 = 0;
		change0(settings.queue_start);
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			space_types[sp.type].check_position(&sp, &queue[settings.queue_start].data[a0]);
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if (isnan(queue[settings.queue_start].data[a0 + a])) {
					sp.axis[a]->settings.next_dist = 0;
					//debug("not using object %d", mtr);
				}
				else {
					sp.axis[a]->settings.next_dist = queue[settings.queue_start].data[a0 + a] + (s == 0 && a == 2 ? zoffset : 0) - sp.axis[a]->settings.source;
#ifdef DEBUG_MOVE
					debug("next dist of %d = %f (%f - %f)", a0 + a, sp.axis[a]->settings.next_dist, queue[settings.queue_start].data[a0 + a] + (s == 0 && a == 2 ? zoffset : 0), sp.axis[a]->settings.source);
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
			if (n != settings.queue_end) {
				// If only one of them is set, set the other one as well to make the rounded corner work.
				if (!isnan(queue[settings.queue_start].data[a0 + a]) && isnan(queue[n].data[a0 + a])) {
					queue[n].data[a0 + a] = sp.axis[a]->settings.source + sp.axis[a]->settings.next_dist - (s == 0 && a == 2 ? zoffset : 0);
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
	// We are prepared and can start the segment. {{{
	bool action = false;
#ifdef DEBUG_MOVE
	debug("Move was prepared");
#endif
	float vq;
	if (n == settings.queue_end) { // {{{
		// There is no next segment; we should stop at the end.
		prepared = false;
#ifdef DEBUG_MOVE
		debug("Building final segment.");
#endif
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				sp.axis[a]->settings.dist = sp.axis[a]->settings.next_dist;
				if (sp.axis[a]->settings.dist != 0)
					action = true;
				sp.axis[a]->settings.next_dist = 0;
#ifdef DEBUG_MOVE
				debug("Last segment distance for motor %d is %f", a, sp.axis[a]->settings.dist);
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
				sp.axis[a]->settings.dist = sp.axis[a]->settings.next_dist;
				if (isnan(queue[n].data[a0 + a]))
					sp.axis[a]->settings.next_dist = 0;
				else
					sp.axis[a]->settings.next_dist = queue[n].data[a0 + a] + (s == 0 && a == 2 ? zoffset : 0) - (sp.axis[a]->settings.source + sp.axis[a]->settings.dist);
				if (sp.axis[a]->settings.next_dist != 0 || sp.axis[a]->settings.dist != 0)
					action = true;
#ifdef DEBUG_MOVE
				debug("Connecting distance for motor %d is %f, to %f = %f - (%f + %f)", a, sp.axis[a]->settings.dist, sp.axis[a]->settings.next_dist, queue[n].data[a0 + a], sp.axis[a]->settings.source, sp.axis[a]->settings.dist);
#endif
			}
			a0 += sp.num_axes;
		}
		vq = queue[n].f[0] * feedrate;
	}
	// }}}
	float v0 = queue[settings.queue_start].f[0] * feedrate;
	float vp = queue[settings.queue_start].f[1] * feedrate;
	//debug("mv %f %f %f %f", feedrate, v0, vp, vq);
	if (queue[settings.queue_start].cb)
		cbs_after_current_move += 1;
	if (settings.queue_end == settings.queue_start)
		send_host(CMD_CONTINUE, 0);
	probing = queue[settings.queue_start].probe;
	run_time = queue[settings.queue_start].time;
	run_dist = queue[settings.queue_start].dist;
	settings.queue_start = n;
	settings.queue_full = false;
	if (!action) {
#ifdef DEBUG_MOVE
		debug("Skipping zero-distance prepared move");
#endif
		num_cbs += cbs_after_current_move;
		cbs_after_current_move = 0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings.dist = NAN;
		}
		settings.fq = 0;
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
		if (isnan(sp.max_v) || isinf(sp.max_v) || sp.max_v <= 0)
			continue;
		// max_mm is the maximum speed in mm/s.
		float max_mm = probing ? space_types[sp.type].probe_speed(&sp) : sp.max_v;
		float dist = 0, distn = 0;
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			dist += sp.axis[a]->settings.dist * sp.axis[a]->settings.dist;
			distn += sp.axis[a]->settings.next_dist * sp.axis[a]->settings.next_dist;
		}
		dist = sqrt(dist);
		distn = sqrt(distn);
		float max = max_mm / dist;
		if (v0 > max)
			v0 = max;
		if (vp > max)
			vp = max;
		max = max_mm / distn;
		if (vq > max)
			vq = max;
	}
#ifdef DEBUG_MOVE
	debug("After limiting, v0 = %f /s, vp = %f /s and vq = %f /s", v0, vp, vq);
#endif
	// }}}
	// Already set up: f0, v0, vp, vq, mtr->dist, mtr->next_dist.
	// To do: start_time, t0, tp, fmain, fp, fq, mtr->main_dist
#ifdef DEBUG_MOVE
	debug("Preparation did f0 = %f", settings.f0);
#endif

	// Use maximum deviation to find fraction where to start rounded corner.
	float factor = vq / vp;
	done_factor = NAN;
	if (vq == 0) {
		settings.fp = 0;
		settings.fq = 0;
	}
	else {
		settings.fp = factor > 1 ? .5 / factor : .5;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			if (sp.num_axes < 2)
				continue;
			if (sp.max_deviation == 0) {
				settings.fp = 0;
				break;
			}
			float norma2 = 0, normb2 = 0, diff2 = 0;
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				float nd = sp.axis[a]->settings.next_dist * factor;
				norma2 += sp.axis[a]->settings.dist * sp.axis[a]->settings.dist;
				normb2 += nd * nd;
				float d = sp.axis[a]->settings.dist - sp.axis[a]->settings.next_dist;
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
			debug("Space %d fp %f dev %f", s, settings.fp, sp.max_deviation);
#endif
			if (new_fp < settings.fp)
				settings.fp = new_fp;
		}
		settings.fq = settings.fp * factor;
	}
	if (isnan(done_factor))
		done_factor = 1;
	// Set up t0, tp.
	settings.t0 = (1 - settings.fp) / (fabs(v0 + vp) / 2);
	settings.tp = settings.fp / (fabs(vp) / 2);
	// Set up f1, f2.
	settings.f1 = .5 * fabs(v0) * settings.t0;
	settings.f2 = 1 - settings.fp - settings.f1;

	// Finish. {{{
	bool anything = false;
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		//debug("space %d axes %d motors %d", s, sp.num_axes, sp.num_motors);
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			//debug("axis %d", a);
			sp.axis[a]->settings.main_dist = sp.axis[a]->settings.dist * (1 - settings.fp);
			// Fill target for filling endpos below.
			if ((sp.axis[a]->settings.dist > 0 && sp.axis[a]->settings.next_dist < 0) || (sp.axis[a]->settings.dist < 0 && sp.axis[a]->settings.next_dist > 0))
				sp.axis[a]->settings.target = sp.axis[a]->settings.source + sp.axis[a]->settings.dist;
			else
				sp.axis[a]->settings.target = sp.axis[a]->settings.source + sp.axis[a]->settings.dist + sp.axis[a]->settings.next_dist; // * settings.fq;
#ifdef DEBUG_MOVE
			debug("Axis %d %d dist %f main dist = %f, next dist = %f currentpos = %d current = %f", s, a, sp.axis[a]->settings.dist, sp.axis[a]->settings.main_dist, sp.axis[a]->settings.next_dist, sp.motor[a]->settings.current_pos, sp.axis[a]->settings.current);
#endif
		}
		bool ok = true;
		// Using NULL as target fills endpos.
		space_types[sp.type].xyz2motors(&sp, NULL, &ok);
		for (int m = 0; m < sp.num_motors; ++m) {
			float ep = sp.motor[m]->settings.endpos * sp.motor[m]->steps_per_unit;
			int iep = int(ep + (ep > 0 ? .49 : -.49));
			if (sp.motor[m]->settings.current_pos != iep) {
				anything = true;
				break;
			}
			//debug("any %d %d %d %d %d", s, m, anything, sp.motor[m]->settings.current_pos, iep);
		}
	}
	if (!anything) {
		num_cbs += cbs_after_current_move;
		cbs_after_current_move = 0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings.dist = NAN;
		}
		settings.fq = 0;
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
	debug("Segment has been set up: f0=%f fp=%f fq=%f v0=%f /s vp=%f /s vq=%f /s t0=%f s tp=%f s", settings.f0, settings.fp, settings.fq, v0, vp, vq, settings.t0, settings.tp);
#endif
	// Reset time.
	settings.hwtime = 0;
	settings.last_time = 0;
	settings.last_current_time = 0;
	settings.start_time = settings.last_time - uint32_t(settings.f0 / fabs(vp) * 1e6);
	if (stopped) {
#ifdef DEBUG_MOVE
		debug("starting new move");
#endif
		//debug("curf1 %d", current_fragment);
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings.source = sp.axis[a]->settings.current;
		}
#ifdef DEBUG_PATH
		fprintf(stderr, "\n");
#endif
	}
	//debug("last=%ld", long(settings.last_time));
	first_fragment = current_fragment;	// Do this every time, because otherwise the queue must be regenerated.	TODO: send partial fragment to make sure this hack actually works, or fix it properly.
	stopped = false;
	moving = true;
	//debug("start=%ld, last-start=%ld", long(settings.start_time), long(settings.last_time - settings.start_time));
	// }}}
	return num_cbs;
}

void abort_move(int pos) { // {{{
	aborting = true;
	//debug("abort pos %d", pos);
	//debug("abort; cf %d rf %d first %d moving %d discarding %d fragments, regenerating %d ticks", current_fragment, running_fragment, first_fragment, moving, FRAGMENTS_PER_BUFFER - free_fragments - 3, pos);
	//debug("try aborting move");
	//debug("abort copying from %d", prev_f);
	current_fragment = running_fragment;
	//debug("current abort -> %x", current_fragment);
	restore_settings();
#ifdef DEBUG_MOVE
	debug("move no longer prepared");
#endif
	//debug("free abort reset");
	current_fragment_pos = 0;
	//if (num_spaces > 0 && spaces[0].num_axes > 0)
	//	fcpdebug(0, 0, "starting hwpos %x", spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0]);
	moving = true;
	//debug("restoring position for fragment %d to position %d", current_fragment, pos);
	while (moving && current_fragment_pos < pos) {
		//debug("tick %d %d %d", current_fragment_pos, settings.hwtime, current_fragment);
		apply_tick();
		//if (num_spaces > 0 && spaces[0].num_axes > 0)
		//	fcpdebug(0, 0, "current hwpos %x time %d", spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0], settings.hwtime);
	}
	//if (num_spaces > 0 && spaces[0].num_axes > 0)
	//	fcpdebug(0, 0, "ending hwpos %x", spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0]);
	//debug("done restoring position");
	// Copy settings back to previous fragment.
	store_settings();
	moving = false;
	stopped = true;
	prepared = false;
	current_fragment_pos = 0;
	//if (num_spaces > 0 && spaces[0].num_axes > 0)
	//	fcpdebug(0, 0, "final hwpos %x", spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0]);
	//debug("curf3 %d", current_fragment);
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			//debug("setting axis %d source to %f", a, sp.axis[a]->settings.current);
			sp.axis[a]->settings.source = sp.axis[a]->settings.current;
			sp.axis[a]->settings.dist = NAN;
			sp.axis[a]->settings.next_dist = NAN;
		}
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->settings.last_v = 0;
			//debug("setting motor %d pos to %d", m, sp.motor[m]->settings.current_pos);
		}
	}
	//debug("aborted move");
	aborting = false;
} // }}}

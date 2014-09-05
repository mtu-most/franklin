// vim: set foldmethod=marker :
#include "firmware.h"

//#define DEBUG_MOVE

// Set up:
// start_time		micros() at start of move.
// t0			time to do main part.
// tp			time to do connection.
// mtr->dist		total distance of this segment (mm).
// mtr->next_dist		total distance of next segment (mm).
// mtr->main_dist		distance of main part (mm).
// v0, vp		start and end velocity for main part. (fraction/s)
// vq			end velocity of connector part. (fraction/s)

// Used from previous segment (if move_prepared == true): tp, vq.
void next_move () {
#ifdef HAVE_SPACES
	uint8_t a0;
	if (queue_start == queue_end && !queue_full)
		return;
#ifdef DEBUG_MOVE
	debug ("Next move; queue start = %d, end = %d", queue_start, queue_end);
	debug ("queue start.y = %f", F(queue[queue_start].data[1]));
#endif
	// Set everything up for running queue[queue_start].
	freeze_time = NAN;
	uint8_t n = (queue_start + 1) % QUEUE_LENGTH;

	// Make sure printer state is good. {{{
	// If the source is unknown, determine it from current_pos.
	//for (uint8_t a = 0; a < num_axes; ++a)
	//	debug("target %d %f", a, F(queue[queue_start].data[a]));
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
#ifdef DEBUG_MOVE
			debug("maybe resetting: %d %f", s, F(sp.motor[0]->current_pos));
#endif
			if (isnan(sp.axis[a]->source)) {
				space_types[sp.type].reset_pos(&sp);
				for (uint8_t a = 0; a < sp.num_axes; ++a)
					sp.axis[a]->current = sp.axis[a]->source;
				break;
			}
#ifdef DEBUG_MOVE
			else
				debug("non-nan: %d %d %f %f", s, a, F(sp.axis[a]->source), F(sp.motor[a]->current_pos));
#endif
		}
	}
	// }}}

	f0 = fq;
	// If no move is prepared, set next_dist from the queue; it will be used as dist below.
	if (!move_prepared) { // {{{
#ifdef DEBUG_MOVE
		debug ("No move prepared.");
		debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
		debug ("queue start.y = %f", F(queue[queue_start].data[1]));
#endif
		f0 = 0;
		uint8_t a0 = 0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			space_types[sp.type].check_position(&sp, &queue[queue_start].data[a0]);
			for (uint8_t m = 0; m < sp.num_motors; ++m) {
				sp.motor[m]->last_v = 0;
				sp.motor[m]->last_distance = 0;
			}
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if (isnan(queue[queue_start].data[a0 + a])) {
					sp.axis[a]->next_dist = 0;
					//debug("not using object %d", mtr);
				}
				else {
					sp.axis[a]->next_dist = queue[queue_start].data[a0 + a] - sp.axis[a]->source + sp.axis[a]->offset;
#ifdef DEBUG_MOVE
					debug("next dist of %d = %f (%f - %f + %f)", a0 + a, F(sp.axis[a]->next_dist), F(queue[queue_start].data[a0 + a]), F(sp.axis[a]->source), F(sp.axis[a]->offset));
					debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
					debug ("queue start.y = %f", F(queue[queue_start].data[1]));
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
					queue[n].data[a0 + a] = sp.axis[a]->source + sp.axis[a]->next_dist - sp.axis[a]->offset;
#ifdef DEBUG_MOVE
					debug("filling next %d with %f", a0 + a, F(queue[n].data[a0 + a]));
					debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
					debug ("queue start.y = %f", F(queue[queue_start].data[1]));
#endif
				}
				if (isnan(queue[queue_start].data[a]) && !isnan(queue[n].data[a])) {
					queue[queue_start].data[a0 + a] = sp.axis[a]->source - sp.axis[a]->offset;
#ifdef DEBUG_MOVE
					debug("filling %d with %f", a0 + a, F(queue[queue_start].data[a0 + a]));
					debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
					debug ("queue start.y = %f", F(queue[queue_start].data[1]));
#endif
				}
			}
			if ((!isnan(queue[queue_start].data[a0 + a]) && isnan(sp.axis[a]->source)) || (n != queue_end && !isnan(queue[n].data[a0 + a]) && isnan(sp.axis[a]->source))) {
				debug ("Motor positions are not known, so move cannot take place; aborting move and removing it from the queue: %f %f %f", F(queue[queue_start].data[a0 + a]), F(queue[n].data[a0 + a]), F(sp.axis[a]->source));
				// This possibly removes one move too many, but it shouldn't happen anyway.
				if (queue[queue_start].cb) {
					++num_movecbs;
					try_send_next ();
				}
				if (queue_end == queue_start) {
					continue_cb |= 1;
					try_send_next ();
				}
				queue_start = n;
				queue_full = false;
				abort_move ();
				return;
			}
		}
		a0 += sp.num_axes;
	}
	// }}}
	// We are prepared and can start the segment. {{{
	bool action = false;
#ifdef DEBUG_MOVE
	debug ("Move was prepared with tp = %f", F(tp));
	debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
	debug ("queue start.y = %f", F(queue[queue_start].data[1]));
#endif
	float vq;
	if (n == queue_end) { // {{{
		// There is no next segment; we should stop at the end.
#ifdef DEBUG_MOVE
		debug ("Building final segment.");
		debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
		debug ("queue start.y = %f", F(queue[queue_start].data[1]));
#endif
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				sp.axis[a]->dist = sp.axis[a]->next_dist;
				if (sp.axis[a]->dist != 0)
					action = true;
				sp.axis[a]->next_dist = 0;
#ifdef DEBUG_MOVE
				debug ("Last segment distance for motor %d is %f", a, F(sp.axis[a]->dist));
				debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
				debug ("queue start.y = %f", F(queue[queue_start].data[1]));
#endif
			}
		}
		vq = 0;
		move_prepared = false;
	}
	// }}}
	else { // {{{
		// There is a next segment; we should connect to it.
#ifdef DEBUG_MOVE
		debug ("Building a connecting segment.");
		debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
		debug ("queue start.y = %f", F(queue[queue_start].data[1]));
#endif
		uint8_t a0 = 0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			space_types[sp.type].check_position(&sp, &queue[n].data[a0]);
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				sp.axis[a]->dist = sp.axis[a]->next_dist;
				if (isnan(queue[n].data[a0 + a]))
					sp.axis[a]->next_dist = 0;
				else
					sp.axis[a]->next_dist = queue[n].data[a0 + a] + sp.axis[a]->offset - (sp.axis[a]->source + sp.axis[a]->dist);
				if (sp.axis[a]->next_dist != 0 || sp.axis[a]->dist != 0)
					action = true;
#ifdef DEBUG_MOVE
				debug ("Connecting distance for motor %d is %f, to %f = %f + %f - (%f + %f)", a, F(sp.axis[a]->dist), F(sp.axis[a]->next_dist), F(queue[n].data[a0 + a]), F(sp.axis[a]->offset), F(sp.axis[a]->source), F(sp.axis[a]->dist));
				debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
				debug ("queue start.y = %f", F(queue[queue_start].data[1]));
#endif
			}
			a0 += sp.num_axes;
		}
		vq = queue[n].f[0] * feedrate;
		move_prepared = true;
	}
	// }}}
	float v0 = queue[queue_start].f[0] * feedrate;
	float vp = queue[queue_start].f[1] * feedrate;
	current_move_has_cb = queue[queue_start].cb;
	if (queue_end == queue_start) {
		continue_cb |= 1;
		try_send_next ();
	}
	queue_start = n;
	queue_full = false;
	if (!action) {
#ifdef DEBUG_MOVE
		debug ("Skipping zero-distance prepared move");
		debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
#endif
		if (current_move_has_cb) {
			++num_movecbs;
			try_send_next ();
		}
		//debug("moving->false");
		moving = false;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->dist = NAN;
		}
		fq = 0;
		return next_move ();
	}
	// }}}

	// Currently set up:
	// f0: fraction of move already done by connection.
	// v0: this move's requested starting speed.
	// vp: this move's requested ending speed.
	// vq: next move's requested starting speed.
	// current_move_has_cb: if a cb should be fired after this segment is complete.
	// move_prepared: if this segment connects to a following segment.
	// mtr->dist: total distance of this segment (mm).
	// mtr->next_dist: total distance of next segment (mm).
#ifdef DEBUG_MOVE
	debug ("Set up: tp = %f s, v0 = %f /s, vp = %f /s, vq = %f /s", F(tp), F(v0), F(vp), F(vq));
	debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
#endif

	// Limit v0, vp, vq. {{{
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			if (!isnan(sp.axis[a]->max_v)) {
				float max;
				if (sp.axis[a]->dist != 0) {
					max = sp.axis[a]->max_v / abs(sp.axis[a]->dist);
					if (v0 > max) {
						v0 = max;
						//debug("limited v0 to %f for max v %f dist %f so %f", F(v0), F(sp.axis[a].max_v), F(sp.axis[a].dist), F(max));
					}
					if (vp > max)
						vp = max;
				}
				if (sp.axis[a]->next_dist != 0) {
					max = sp.axis[a]->max_v / abs(sp.axis[a]->next_dist);
					if (vq > max)
						vq = max;
				}
			}
		}
	}
#ifdef DEBUG_MOVE
	debug ("After limiting, v0 = %f /s, vp = %f /s and vq = %f /s", F(v0), F(vp), F(vq));
	debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
#endif
	// }}}
	// Already set up: f0, v0, vp, vq, mtr->dist, mtr->next_dist.
	// To do: start_time, t0, tp, fmain, fp, fq, mtr->main_dist
#ifdef DEBUG_MOVE
	debug ("Preparation did f0 = %f", F(f0));
	debug ("currentpos10 = %f", F(spaces[1].motor[0]->current_pos));
#endif

	// Use maximum deviation to find fraction where to start rounded corner.
	float factor = vq / vp;
	if (vq == 0) {
		fp = 0;
		fq = 0;
	}
	else {
		fp = factor > 1 ? .5 / factor : .5;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			if (sp.num_axes < 2)
				continue;
			if (sp.max_deviation == 0) {
				fp = 0;
				break;
			}
			float norma2 = 0, normb2 = 0, diff2 = 0;
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				float nd = sp.axis[a]->next_dist * factor;
				norma2 += sp.axis[a]->dist * sp.axis[a]->dist;
				normb2 += nd * nd;
				float d = sp.axis[a]->dist - sp.axis[a]->next_dist;
				diff2 += d * d;
			}
			// Calculate distances and ignore spaces which don't have two segments.
			float normb(sqrt(normb2));
			if (normb <= 0)
				continue;
			float norma(sqrt(norma2));
			if (norma <= 0)
				continue;
			float new_fp = sp.max_deviation / sqrt(normb / (norma + normb) * diff2);
#ifdef DEBUG_MOVE
			debug ("Space %d fp %f dev %f", s, F(fp), F(sp.max_deviation));
#endif
			if (new_fp < fp)
				fp = new_fp;
		}
		fq = fp * factor;
	}
	// Set up t0, tp.
	t0 = (1 - fp) / (abs(v0 + vp) / 2);
	tp = fp / (abs(vp) / 2);
	// Set up f1, f2.
	f1 = .5 * abs(v0) * t0;
	f2 = 1 - fp - f1;

	// Finish. {{{
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		sp.active = false;
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			if (sp.axis[a]->dist != 0 || sp.axis[a]->next_dist != 0)
				sp.active = true;
			sp.axis[a]->main_dist = sp.axis[a]->dist * (1 - fp);
#ifdef DEBUG_MOVE
			debug ("Axis %d %d dist %f main dist = %f, next dist = %f currentpos = %f", s, a, F(sp.axis[a]->dist), F(sp.axis[a]->main_dist), F(sp.axis[a]->next_dist), F(sp.motor[a]->current_pos));
#endif
		}
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
	debug ("Segment has been set up: f0=%f fp=%f fq=%f v0=%f /s vp=%f /s vq=%f /s t0=%f s tp=%f s", F(f0), F(fp), F(fq), F(v0), F(vp), F(vq), F(t0), F(tp));
#endif
	//debug("moving->true");
	moving = true;
	start_time = micros () - long(f0 / abs(vp) * 1e6);
	// }}}
#endif
}

void abort_move () { // {{{
#ifdef HAVE_SPACES
	//debug ("try aborting move");
	if (moving) {
		//debug ("aborting move");
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				//debug ("setting axis %d source to %f", a, F(axis[a].current));
				sp.axis[a]->source = sp.axis[a]->current;
				sp.axis[a]->dist = NAN;
				sp.axis[a]->next_dist = NAN;
			}
			for (uint8_t m = 0; m < sp.num_motors; ++m)
				sp.motor[m]->last_v = 0;
		}
		//debug("moving->false");
		moving = false;
		move_prepared = false;
#ifdef DEBUG_MOVE
		debug ("move no longer prepared");
#endif
		if (t0 == 0 && (queue_end != queue_start || queue_full)) {
			// This was probably the preparation for a move; remove the move itself from the queue.
			// If it wasn't, doing this doesn't really harm either.
			if (queue_end == queue_start) {
				continue_cb |= 1;
				try_send_next ();
			}
			if (queue[queue_start].cb) {
				//debug ("movecb 4");
				++num_movecbs;
				try_send_next ();
			}
			queue_start = (queue_start + 1) % QUEUE_LENGTH;
			queue_full = false;
		}
	}
#endif
} // }}}

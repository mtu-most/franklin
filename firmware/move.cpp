// vim: set foldmethod=marker :
#include "firmware.h"

//#define DEBUG_MOVE

#if MAXAXES > 0
void reset_pos ()	// {{{
{
#ifdef DEBUG_MOVE
	debug ("Recalculating current position");
#endif
	// Determine current location of extruder.
	if (printer_type == 0) { // {{{
		for (uint8_t a = 0; a < MAXAXES; ++a)
			axis[a].source = axis[a].current_pos == MAXLONG ? NAN : axis[a].current_pos / axis[a].motor.steps_per_mm;
	}
	// }}}
#if MAXAXES >= 3
	else if (printer_type == 1) { // {{{
		//debug ("new delta position");
		// All axes' current_pos must be valid and equal, in other words, that x=y=0.
		if (axis[0].current_pos == MAXLONG || axis[1].current_pos == MAXLONG || axis[2].current_pos == MAXLONG || axis[0].current_pos != axis[1].current_pos || axis[0].current_pos != axis[2].current_pos) {
#ifdef DEBUG_MOVE
			debug ("Refusing to set new delta position from current_pos = (%d, %d, %d)", int (axis[0].current_pos), int (axis[1].current_pos), int (axis[2].current_pos));
#endif
			axis[0].source = NAN;
			axis[1].source = NAN;
			axis[2].source = NAN;
		}
		else {
			axis[0].source = 0;
			axis[1].source = 0;
			axis[2].source = axis[0].current_pos / axis[0].motor.steps_per_mm;
		}
		for (uint8_t a = 3; a < MAXAXES; ++a)
			axis[a].source = axis[a].current_pos == MAXLONG ? NAN : axis[a].current_pos / axis[a].motor.steps_per_mm;
	}
	// }}}
#endif
	else
		debug ("Error: invalid printer type %d", printer_type);
	for (uint8_t a = 0; a < MAXAXES; ++a) {
		axis[a].current = axis[a].source;
#ifdef DEBUG_MOVE
		debug ("New position for axis %d = %f", a, F(axis[a].source));
#endif
	}
}
// }}}
#endif

#if MAXAXES > 0

#if MAXAXES >= 3
static bool check_delta (uint8_t a, float *target) {
	float dx = target[0] - axis[a].x;
	float dy = target[1] - axis[a].y;
	float r2 = dx * dx + dy * dy;
	if (r2 > axis[a].axis_max * axis[a].axis_max) {
		//debug ("not ok: %f %f %f %f %f %f %f", F(target[0]), F(target[1]), F(dx), F(dy), F(r2), F(l2), F(axis[a].delta_length));
		// target is too far away from axis.  Pull it towards axis so that it is on the edge.
		// target = axis + (target - axis) * (l - epsilon) / r.
		float factor = axis[a].axis_max / sqrt (r2);
		target[0] = axis[a].x + (target[0] - axis[a].x) * factor;
		target[1] = axis[a].y + (target[1] - axis[a].y) * factor;
		return false;
	}
	// Inner product shows if projection is inside or outside the printable region.
	float projection = -(dx * axis[a].x + dy * axis[a].y) / axis[a].delta_radius;
	if (projection < axis[a].axis_min) {
		//debug ("not ok: %f %f %f %f %f", F(inner), F(dx), F(dy), F(axis[a].x), F(axis[a].y));
		// target is on the wrong side of axis.  Pull it towards plane so it is on the edge.
		target[0] -= (axis[a].axis_min - projection - .001) / axis[a].delta_radius * axis[a].x;
		target[1] -= (axis[a].axis_min - projection - .001) / axis[a].delta_radius * axis[a].y;
		// Assume this was a small correction; that way, things will work even if numerical errors cause this to be called for the real move.
		dx = target[0] - axis[a].x;
		dy = target[1] - axis[a].y;
		r2 = dx * dx + dy * dy;
		return false;
	}
	return true;
}
#endif

static void apply_offsets (float *data) {
	// Apply offsets.
	for (uint8_t a = 0; a < num_axes; ++a) {
		data[AXIS0 + a] += axis[a].offset;
	}
#if MAXAXES >= 3
	if (printer_type == 1) {
		for (uint8_t timeout = 0; timeout < 2; ++timeout) {
			bool ok = true;
			for (uint8_t a = 0; a < num_axes; ++a)
				ok &= check_delta (a, &data[AXIS0]);
			if (ok)
				break;
		}
	}
#endif
}
#endif

// Set up:
// start_time		micros() at start of move.
// t0			time to do main part.
// tp			time to do connection.
// m->dist		total distance of this segment (mm).
// m->next_dist		total distance of next segment (mm).
// m->main_dist		distance of main part (mm).
// m->steps_done	(extruder only): steps of this segment that have been done in preparation.
// v0, vp		start and end velocity for main part. (fraction/s)
// vq			end velocity of connector part. (fraction/s)

// Used from previous segment (if move_prepared == true): tp, vq.
void next_move () {
#if MAXAXES > 0 || MAXEXTRUDERS > 0
	if (queue_start == queue_end && !queue_full)
		return;
#ifdef DEBUG_MOVE
	debug ("Next move; queue start = %d, end = %d", queue_start, queue_end);
#endif
	// Set everything up for running queue[queue_start].
	freeze_time = -1;

#if MAXAXES > 0
	// Make sure printer state is good. {{{
	// If the source is unknown, determine it from current_pos.
	for (uint8_t a = 0; a < num_axes; ++a) {
		if (isnan (axis[a].source)) {
			reset_pos ();
			break;
		}
	}
#endif

#if MAXAXES >= 3
	// For a delta, if any axis moves, all motors move; avoid trouble by setting none of them to nan.
	if (printer_type == 1 && (!isnan (queue[queue_start].data[AXIS0]) || !isnan (queue[queue_start].data[AXIS0 + 1]) || !isnan (queue[queue_start].data[AXIS0 + 2]))) {
		for (uint8_t a = 0; a < 3; ++a) {
			if (isnan (queue[queue_start].data[AXIS0 + a])) {
#ifdef DEBUG_MOVE
				debug ("Filling delta axis %d", a);
#endif
				// Offset will be added later; compensate for it.
				queue[queue_start].data[AXIS0 + a] = axis[a].source - axis[a].offset;
			}
		}
	}
#endif
#if MAXAXES > 0
	for (uint8_t a = 0; a < num_axes; ++a) {
		if (!isnan (queue[queue_start].data[AXIS0 + a]) && isnan (axis[a].source)) {
			debug ("Motor positions are not known, so move cannot take place; aborting move and removing it from the queue.");
			// This possible removes one move too many, but it shouldn't happen anyway.
			if (queue[queue_start].cb) {
				++num_movecbs;
				try_send_next ();
			}
			if (queue_end == queue_start) {
				continue_cb |= 1;
				try_send_next ();
			}
			queue_start = (queue_start + 1) % QUEUE_LENGTH;
			queue_full = false;
			abort_move ();
			return;
		}
	}
#endif
	// }}}

	f0 = vq * tp / 2e6;
	// If no move is prepared, do a fake segment with only the speed change at the end; don't pop anything off the queue.
	if (!move_prepared) { // {{{
#ifdef DEBUG_MOVE
		debug ("No move prepared.");
#endif
		f0 = 0;
		for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
			uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
			if (!motors[m])
				continue;
			if (isnan (queue[queue_start].data[mt + 2]))
				motors[m]->next_dist = 0;
			else {
#if MAXAXES > 0
				if (mt < num_axes)
					motors[m]->next_dist = queue[queue_start].data[mt + 2] - axis[mt].source;
				else
#endif
					motors[m]->next_dist = queue[queue_start].data[mt + 2];
				debug("next dist of motor %d = %f", m, F(motors[m]->next_dist));
			}
		}
	}
	// }}}
	// Otherwise, we are prepared and can start the segment. {{{
	bool action = false;
#ifdef DEBUG_MOVE
	debug ("Move was prepared with tp = %d", int (tp / 1000));
#endif
	uint8_t n = (queue_start + 1) % QUEUE_LENGTH;
	if (n == queue_end) { // {{{
		// There is no next segment; we should stop at the end.
#ifdef DEBUG_MOVE
		debug ("Building final segment.");
#endif
		for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
			uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
			if (!motors[m])
				continue;
			motors[m]->dist = motors[m]->next_dist;
			// For a delta, this multiplication isn't really allowed, but we're only looking at orders of magnitude, so it's ok.
			if (abs (motors[m]->dist) * motors[m]->steps_per_mm > .001)
				action = true;
			else
				motors[m]->dist = 0;
			motors[m]->next_dist = 0;
#ifdef DEBUG_MOVE
			debug ("Last segment distance for motor %d is %f", m, F(motors[m]->dist));
#endif
		}
		vq = 0;
		move_prepared = false;
	}
	// }}}
	else { // {{{
		// There is a next segment; we should connect to it.
#ifdef DEBUG_MOVE
		debug ("Building a connecting segment.");
#endif
		// Apply offsets to next segment.
#if MAXAXES > 0
		apply_offsets (queue[n].data);
#endif
		for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
			uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
			if (!motors[m])
				continue;
			motors[m]->dist = motors[m]->next_dist;
			if (isnan (queue[n].data[mt + 2]))
				motors[m]->next_dist = 0;
			else {
#if MAXAXES > 0
				if (mt < num_axes)
					motors[m]->next_dist = queue[n].data[mt + 2] - (axis[mt].source + motors[m]->dist);
				else
#endif
					motors[m]->next_dist = queue[n].data[mt + 2];
			}
			// For a delta, this multiplication isn't really allowed, but we're only looking at orders of magnitude, so it's ok.
			if (abs (motors[m]->next_dist) * motors[m]->steps_per_mm > .001 || abs (motors[m]->dist) * motors[m]->steps_per_mm > .001)
				action = true;
#ifdef DEBUG_MOVE
			debug ("Connecting distance for motor %d is %f, to %f", m, F(motors[m]->dist), F(motors[m]->next_dist));
#endif
		}
		vq = queue[n].data[F0] * feedrate;
		move_prepared = true;
	}
	// }}}
	v0 = queue[queue_start].data[F0] * feedrate;
	vp = queue[queue_start].data[F1] * feedrate;
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
#endif
		if (current_move_has_cb) {
			++num_movecbs;
			try_send_next ();
		}
		//debug("moving->false");
		moving = false;
		for (uint8_t m = 0; m < MAXOBJECT; ++m) {
			if (!motors[m])
				continue;
			motors[m]->dist = NAN;
		}
		return next_move ();
	}
	// }}}

	// Currently set up:
	// f0: fraction of move already done by connection.
	// v0: previous move's final speed.
	// vp: this move's requested ending speed.
	// vq: next move's requested starting speed.
	// current_move_has_cb: if a cb should be fired after this segment is complete.
	// move_prepared: if this segment connects to a following segment.
	// m->dist: total distance of this segment (mm).
	// m->next_dist: total distance of next segment (mm).
#ifdef DEBUG_MOVE
	debug ("Set up: tp = %d ms, v0 = %f /s, vp = %f /s, vq = %f /s", int (tp / 1000), F(v0), F(vp), F(vq));
#endif

	// Limit v0, vp, vq. {{{
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[m])
			continue;
		float a;
		if (abs (motors[m]->dist) > .001) {
			a = motors[m]->max_v / abs (motors[m]->dist);
			if (!isnan (a) && !isinf (a)) {
				if (v0 > a)
					v0 = a;
				if (vp > a)
					vp = a;
			}
		}
		if (abs (motors[m]->next_dist) > .001) {
			a = motors[m]->max_v / abs (motors[m]->next_dist);
			if (!isnan (a) && !isinf (a) && vq > a)
				vq = a;
		}
	}
#ifdef DEBUG_MOVE
	debug ("After limiting, v0 = %f /s, vp = %f /s and vq = %f /s", F(v0), F(vp), F(vq));
#endif
	// }}}
	// Already set up: f0, v0, vp, vq, m->dist, m->next_dist.
	// To do: start_time, t0, tp, m->main_dist, m->steps_done
#ifdef DEBUG_MOVE
	debug ("Preparation did f0 = %f", F(f0));
#endif

#if MAXAXES > 0
	// Use maximum deviation to find fraction where to start rounded corner.
	float mm_round;
	float norma2 = 0, normb2 = 0, inner = 0, summed2 = 0;
	for (uint8_t a = 0; a < num_axes; ++a) {
		inner += axis[a].motor.dist * axis[a].motor.next_dist;
		norma2 += axis[a].motor.dist * axis[a].motor.dist;
		normb2 += axis[a].motor.next_dist * axis[a].motor.next_dist;
		float s = axis[a].motor.dist + axis[a].motor.next_dist;
		summed2 += s * s;
	}
	if (norma2 < .01 || normb2 < .01) {
		// At least one of the segments is 0; no rounded corner.
		mm_round = 0;
	}
	else {
		float alpha = acos(inner / sqrt(norma2 * normb2));
		float mm = max_deviation / sin(alpha);
		float scale = sqrt(summed2) / 2;
		mm_round = mm / scale;
	}
	// Convert mm to fraction for both p and q.
	float fp = norma2 >= .01 ? mm_round / sqrt(norma2) : 0;
	float fq = normb2 >= .01 ? mm_round / sqrt(normb2) : 0;
	if (fp > .5) {
		fq *= .5 / fp;
		fp = .5;
	}
	if (fq > .5) {
		fp *= .5 / fq;
		fq = .5;
	}
#else
	float fp = 0;
	float fq = 0;
#endif

	// Compute t0 and tp.
	tp = long (fp * 1e6 / vp);
	t0 = long ((1. - f0 - fp) * 2e6 / (v0 + vp));

	// Finish. {{{
	start_time = micros ();
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[m])
			continue;
		if (motors[m]->dist != 0 || motors[m]->next_dist != 0) {
			motors[m]->last_time = start_time;
			SET (motors[m]->enable_pin);
			motors_busy |= 1 << m;
			/*if (mt < num_axes)
				debug ("Move motor %f from %f (really %f) over %f steps (f0=%f)", m, F(axis[mt].source), F(axis[mt].current), F(motors[m]->dist), F(f0));*/
		}
		if (isinf (vp) || isinf (v0))
			motors[m]->main_dist = 0;
		else
			motors[m]->main_dist = motors[m]->dist * (f0 + (v0 + vp) * t0 / 2e6);
#ifdef DEBUG_MOVE
		debug ("Motor %d dist %f main dist = %f, next dist = %f", m, F(motors[m]->dist), F(motors[m]->main_dist), F(motors[m]->next_dist));
#endif
#if MAXEXTRUDERS > 0
		if (mt >= num_axes)
			extruder[mt - num_axes].steps_done = f0 * motors[m]->dist * motors[m]->steps_per_mm;
#endif
	}
#ifdef DEBUG_MOVE
	debug ("Segment has been set up: f0=%f fp=%f fq=%f v0=%f /s vp=%f /s vq=%f /s t0=%d ms tp=%d ms", F(f0), F(fp), F(fq), F(v0), F(vp), F(vq), int (t0/1000), int (tp/1000));
#endif
	//debug("moving->true");
	moving = true;
	// }}}
#endif
}

void abort_move () { // {{{
#if MAXAXES > 0 || MAXEXTRUDERS > 0
	//debug ("try aborting move");
	if (moving) {
		//debug ("aborting move");
#if MAXAXES > 0
		for (uint8_t a = 0; a < MAXAXES; ++a) {
			//debug ("setting axis %d source to %f", a, F(axis[a].current));
			axis[a].source = axis[a].current;
		}
#endif
		for (uint8_t m = 0; m < MAXOBJECT; ++m) {
			if (!motors[m])
				continue;
			motors[m]->continuous_steps_per_s = 0;
			motors[m]->f = 0;
			motors[m]->dist = NAN;
			motors[m]->next_dist = NAN;
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

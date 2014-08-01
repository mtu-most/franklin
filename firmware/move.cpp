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
			axis[a].source = axis[a].current_pos;
	}
	// }}}
#if MAXAXES >= 3
	else if (printer_type == 1) { // {{{
		//debug ("new delta position");
		// All axes' current_pos must be valid and equal, in other words, x=y=0.
		if (axis[0].current_pos != axis[1].current_pos || axis[0].current_pos != axis[2].current_pos) {
#ifdef DEBUG_MOVE
			debug ("Refusing to set new delta position from current_pos = (%f, %f, %f)", F(axis[0].current_pos), F(axis[1].current_pos), F(axis[2].current_pos));
#endif
			axis[0].source = NAN;
			axis[1].source = NAN;
			axis[2].source = NAN;
		}
		else {
			axis[0].source = 0;
			axis[1].source = 0;
			axis[2].source = axis[0].current_pos;
		}
		for (uint8_t a = 3; a < MAXAXES; ++a)
			axis[a].source = axis[a].current_pos;
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
static bool check_delta (uint8_t a, float *target) {	// {{{
	float dx = target[0] - axis[a].x;
	float dy = target[1] - axis[a].y;
	float r2 = dx * dx + dy * dy;
	if (r2 > axis[a].axis_max * axis[a].axis_max) {
		debug ("not ok 1: %f %f %f %f %f %f %f", F(target[0]), F(target[1]), F(dx), F(dy), F(r2), F(axis[a].delta_length), F(axis[a].axis_max));
		// target is too far away from axis.  Pull it towards axis so that it is on the edge.
		// target = axis + (target - axis) * (l - epsilon) / r.
		float factor(axis[a].axis_max / sqrt(r2));
		target[0] = axis[a].x + (target[0] - axis[a].x) * factor;
		target[1] = axis[a].y + (target[1] - axis[a].y) * factor;
		return false;
	}
	// Inner product shows if projection is inside or outside the printable region.
	float projection = -(dx / axis[a].delta_radius * axis[a].x + dy / axis[a].delta_radius * axis[a].y);
	if (projection < axis[a].axis_min) {
		debug ("not ok 2: %f %f %f %f %f", F(projection), F(dx), F(dy), F(axis[a].x), F(axis[a].y));
		// target is on the wrong side of axis.  Pull it towards plane so it is on the edge.
		target[0] -= (axis[a].axis_min - projection - 1) / axis[a].delta_radius * axis[a].x;
		target[1] -= (axis[a].axis_min - projection - 1) / axis[a].delta_radius * axis[a].y;
		// Assume this was a small correction; that way, things will work even if numerical errors cause this to be called for the real move.
		return false;
	}
	debug("ok");
	return true;
}	// }}}
#endif

static void apply_offsets (float *data) {	// {{{
	// Apply offsets.
	for (uint8_t a = 0; a < num_axes; ++a) {
		data[a] += axis[a].offset;
	}
#if MAXAXES >= 3
	if (printer_type == 1) {
		if (isnan(data[0]) || isnan(data[1]) || isnan(data[2])) {
			data[0] = NAN;
			data[1] = NAN;
			data[2] = NAN;
		}
		else {
			for (uint8_t timeout = 0; timeout < 2; ++timeout) {
				bool ok = true;
				for (uint8_t a = 0; a < num_axes; ++a)
					ok &= check_delta (a, data);
				if (ok)
					break;
			}
		}
	}
#endif
}	// }}}
#endif

// Set up:
// start_time		micros() at start of move.
// t0			time to do main part.
// tp			time to do connection.
// mtr->dist		total distance of this segment (mm).
// mtr->next_dist		total distance of next segment (mm).
// mtr->main_dist		distance of main part (mm).
// mtr->distance_done	(extruder only): steps of this segment that have been done in preparation.
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
	freeze_time = NAN;

#if MAXAXES > 0
	// Make sure printer state is good. {{{
	// If the source is unknown, determine it from current_pos.
	for (uint8_t a = 0; a < num_axes; ++a) {
		if (isnan(axis[a].source)) {
			reset_pos ();
			break;
		}
	}
#endif

#if MAXAXES >= 3
	// For a delta, if any axis moves, all motors move; avoid trouble by setting none of them to nan.
	if (printer_type == 1 && (isnan(queue[queue_start].data[0]) || isnan(queue[queue_start].data[1]) || isnan(queue[queue_start].data[2]))) {
		for (uint8_t a = 0; a < 3; ++a) {
			if (isnan(queue[queue_start].data[a])) {
#ifdef DEBUG_MOVE
				debug ("Filling delta axis %d", a);
#endif
				// Offset will be added later; compensate for it.
				queue[queue_start].data[a] = axis[a].source - axis[a].offset;
			}
		}
	}
#endif
#if MAXAXES > 0
	for (uint8_t a = 0; a < num_axes; ++a) {
		debug("checking %f %f", F(queue[queue_start].data[a]), F(axis[a].source));
		if (!isnan(queue[queue_start].data[a]) && isnan(axis[a].source)) {
			debug ("Motor positions are not known, so move cannot take place; aborting move and removing it from the queue.");
			// This possibly removes one move too many, but it shouldn't happen anyway.
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

	f0 = fq;
	// If no move is prepared, set next_dist from the queue; it will be used as dist below.
	if (!move_prepared) { // {{{
#ifdef DEBUG_MOVE
		debug ("No move prepared.");
#endif
#if MAXAXES > 0
		apply_offsets (queue[queue_start].data);
#endif
		f0 = 0;
		for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
			uint8_t mtr = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
			if (!motors[mtr])
				continue;
			motors[mtr]->last_v = 0;
			if (isnan(queue[queue_start].data[mt]))
				motors[mtr]->next_dist = 0;
			else {
#if MAXAXES > 0
				if (mt < num_axes) {
					motors[mtr]->next_dist = queue[queue_start].data[mt] - axis[mt].source;
#ifdef DEBUG_MOVE
					debug("next dist of axis %d: %f - %f = %f", mt, F(queue[queue_start].data[mt]), F(axis[mt].source), F(motors[mtr]->next_dist));
#endif
				}
				else {
#endif
					motors[mtr]->next_dist = queue[queue_start].data[mt];
#ifdef DEBUG_MOVE
					debug("next dist of extruder %d = %f", mt - num_axes, F(motors[mtr]->next_dist));
#endif
				}
			}
		}
	}
	// }}}
	// Otherwise, we are prepared and can start the segment. {{{
	bool action = false;
#ifdef DEBUG_MOVE
	debug ("Move was prepared with tp = %f", F(tp));
#endif
	uint8_t n = (queue_start + 1) % QUEUE_LENGTH;
	float vq;
	if (n == queue_end) { // {{{
		// There is no next segment; we should stop at the end.
#ifdef DEBUG_MOVE
		debug ("Building final segment.");
#endif
		for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
			uint8_t mtr = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
			if (!motors[mtr])
				continue;
			motors[mtr]->dist = motors[mtr]->next_dist;
			if (motors[mtr]->dist != 0)
				action = true;
			motors[mtr]->next_dist = 0;
#ifdef DEBUG_MOVE
			debug ("Last segment distance for motor %d is %f", mtr, F(motors[mtr]->dist));
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
			uint8_t mtr = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
			if (!motors[mtr])
				continue;
			motors[mtr]->dist = motors[mtr]->next_dist;
			if (isnan(queue[n].data[mt]))
				motors[mtr]->next_dist = 0;
			else {
#if MAXAXES > 0
				if (mt < num_axes)
					motors[mtr]->next_dist = queue[n].data[mt] - (axis[mt].source + motors[mtr]->dist);
				else
#endif
					motors[mtr]->next_dist = queue[n].data[mt];
			}
			if (motors[mtr]->next_dist != 0 || motors[mtr]->dist != 0)
				action = true;
#ifdef DEBUG_MOVE
			debug ("Connecting distance for motor %d is %f, to %f", mtr, F(motors[mtr]->dist), F(motors[mtr]->next_dist));
#endif
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
#endif
		if (current_move_has_cb) {
			++num_movecbs;
			try_send_next ();
		}
		//debug("moving->false");
		moving = false;
		for (uint8_t mtr = 0; mtr < MAXOBJECT; ++mtr) {
			if (!motors[mtr])
				continue;
			motors[mtr]->dist = NAN;
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
#endif

	// Limit v0, vp, vq. {{{
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t mtr = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[mtr])
			continue;
		if (!isnan(motors[mtr]->max_v)) {
			float a;
			if (motors[mtr]->dist != 0) {
				a = motors[mtr]->max_v / motors[mtr]->dist;
				if (v0 > a) {
					v0 = a;
					//debug("limited v0 to %f for max v %f dist %f so %f", F(v0), F(motors[mtr]->max_v), F(motors[mtr]->dist), F(a));
				}
				if (vp > a)
					vp = a;
			}
			if (motors[mtr]->next_dist != 0) {
				a = motors[mtr]->max_v / motors[mtr]->next_dist;
				if (vq > a)
					vq = a;
			}
		}
	}
#ifdef DEBUG_MOVE
	debug ("After limiting, v0 = %f /s, vp = %f /s and vq = %f /s", F(v0), F(vp), F(vq));
#endif
	// }}}
	// Already set up: f0, v0, vp, vq, mtr->dist, mtr->next_dist.
	// To do: start_time, t0, tp, fmain, fp, fq, mtr->main_dist, mtr->distance_done
#ifdef DEBUG_MOVE
	debug ("Preparation did f0 = %f", F(f0));
#endif

	float fp;
	float fq;
#if MAXAXES > 0
	// Use maximum deviation to find fraction where to start rounded corner.
	float m_round;
	float norma2 = 0, normb2 = 0, inner = 0;
	for (uint8_t a = 0; a < num_axes; ++a) {
		inner += axis[a].motor.dist * axis[a].motor.next_dist;
		norma2 += axis[a].motor.dist * axis[a].motor.dist;
		normb2 += axis[a].motor.next_dist * axis[a].motor.next_dist;
	}
	float norma(sqrt(norma2));
	float normb(sqrt(normb2));
	if (norma <= 0 || normb <= 0) {
		// At least one of the segments is 0; no rounded corner.
		m_round = 0;
	}
	else {
		float summed2 = 0;
		for (uint8_t a = 0; a < num_axes; ++a) {
			float s = axis[a].motor.dist / norma + axis[a].motor.next_dist / normb;
			summed2 += s * s;
		}
		float scale(sqrt(summed2) / 2);
		m_round = max_deviation / scale;
	}
	// Convert m to fraction for both p and q.
	fp = norma > 0 ? m_round / norma : 0;
	fq = normb > 0 ? m_round / normb : 0;
	if (fp > .5) {
		fq *= .5 / fp;
		fp = .5;
	}
	if (fq > .5) {
		fp *= .5 / fq;
		fq = .5;
	}
#else
	fp = 0;
	fq = 0;
#endif
	// Set up t0, tp.
	t0 = (1 - fp) / (abs(v0 + vp) / 2);
	tp = fp / (abs(vp) / 2);
	// Set up f1, f2.
	f1 = .5 * abs(v0) * t0;
	f2 = 1 - fp - f1;

	// Finish. {{{
	start_time = micros ();
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t mtr = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[mtr])
			continue;
		if (motors[mtr]->dist != 0 || motors[mtr]->next_dist != 0) {
			motors[mtr]->last_time = start_time;
			SET (motors[mtr]->enable_pin);
			motors_busy |= 1 << mtr;
			/*if (mt < num_axes)
				debug ("Move motor %f from %f (really %f) over %f steps (f0=%f)", mtr, F(axis[mt].source), F(axis[mt].current), F(motors[mtr]->dist), F(f0));*/
		}
		motors[mtr]->main_dist = motors[mtr]->dist * (1 - fp);
#ifdef DEBUG_MOVE
		debug ("Motor %d dist %f main dist = %f, next dist = %f", mtr, F(motors[mtr]->dist), F(motors[mtr]->main_dist), F(motors[mtr]->next_dist));
#endif
#if MAXEXTRUDERS > 0
		if (mt >= num_axes)
			extruder[mt - num_axes].distance_done = motors[mtr]->dist * f0;
#endif
	}
#ifdef DEBUG_MOVE
	debug ("Segment has been set up: f0=%f fp=%f fq=%f v0=%f /s vp=%f /s vq=%f /s t0=%f μs tp=%f μs", F(f0), F(fp), F(fq), F(v0), F(vp), F(vq), F(t0), F(tp));
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
		for (uint8_t mtr = 0; mtr < MAXOBJECT; ++mtr) {
			if (!motors[mtr])
				continue;
			motors[mtr]->continuous_v = 0;
			motors[mtr]->last_v = 0;
			motors[mtr]->dist = NAN;
			motors[mtr]->next_dist = NAN;
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

// vim: set foldmethod=marker :
#include "firmware.h"

#define DEBUG_MOVE

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
		if (axis[0].current_pos == MAXLONG || axis[1].current_pos == MAXLONG || axis[2].current_pos == MAXLONG || axis[0].current_pos != axis[1].current_pos || axis[0].current_pos != axis[2].current_pos) {
#ifdef DEBUG_MOVE
			debug ("Refusing to set new delta position from current_pos = (%ld, %ld, %ld)", F(axis[0].current_pos), F(axis[1].current_pos), F(axis[2].current_pos));
#endif
			axis[0].source = MAXLONG;
			axis[1].source = MAXLONG;
			axis[2].source = MAXLONG;
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
		debug ("New position for axis %d = %ld", a, F(axis[a].source));
#endif
	}
}
// }}}
#endif

#if MAXAXES > 0

#if MAXAXES >= 3
static bool check_delta (uint8_t a, int32_t *target) {	// {{{
	int32_t dx = target[0] - axis[a].x;
	int32_t dy = target[1] - axis[a].y;
	int32_t r2 = m(dx) * m(dx) + m(dy) * m(dy);
	if (r2 > m(axis[a].axis_max) * m(axis[a].axis_max)) {
		debug ("not ok 1: %ld %ld %ld %ld %ld %ld %ld", F(target[0]), F(target[1]), F(dx), F(dy), F(r2), F(axis[a].delta_length), F(axis[a].axis_max));
		// target is too far away from axis.  Pull it towards axis so that it is on the edge.
		// target = axis + (target - axis) * (l - epsilon) / r.
		int32_t factor(m2(axis[a].axis_max / sqrt(r2)));
		target[0] = axis[a].x + m2((target[0] - axis[a].x) * factor);
		target[1] = axis[a].y + m2((target[1] - axis[a].y) * factor);
		return false;
	}
	// Inner product shows if projection is inside or outside the printable region.
	int32_t projection = -m(dx / k(axis[a].delta_radius) * axis[a].x + dy / k(axis[a].delta_radius) * axis[a].y);
	if (projection < axis[a].axis_min) {
		debug ("not ok 2: %ld %ld %ld %ld %ld", F(projection), F(dx), F(dy), F(axis[a].x), F(axis[a].y));
		// target is on the wrong side of axis.  Pull it towards plane so it is on the edge.
		target[0] -= k(axis[a].axis_min - projection - 1) / axis[a].delta_radius * m(axis[a].x);
		target[1] -= k(axis[a].axis_min - projection - 1) / axis[a].delta_radius * m(axis[a].y);
		// Assume this was a small correction; that way, things will work even if numerical errors cause this to be called for the real move.
		return false;
	}
	debug("ok");
	return true;
}	// }}}
#endif

static void apply_offsets (int32_t *data) {	// {{{
	// Apply offsets.
	for (uint8_t a = 0; a < num_axes; ++a) {
		data[a] = data[a] == MAXLONG ? MAXLONG : data[a] + axis[a].offset;
	}
#if MAXAXES >= 3
	if (printer_type == 1) {
		if (data[0] == MAXLONG || data[1] == MAXLONG || data[2] == MAXLONG) {
			data[0] = MAXLONG;
			data[1] = MAXLONG;
			data[2] = MAXLONG;
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
	freeze_time = -1;

#if MAXAXES > 0
	// Make sure printer state is good. {{{
	// If the source is unknown, determine it from current_pos.
	for (uint8_t a = 0; a < num_axes; ++a) {
		if (axis[a].source == MAXLONG) {
			reset_pos ();
			break;
		}
	}
#endif

#if MAXAXES >= 3
	// For a delta, if any axis moves, all motors move; avoid trouble by setting none of them to nan.
	if (printer_type == 1 && (queue[queue_start].data[0] == MAXLONG || queue[queue_start].data[1] == MAXLONG || queue[queue_start].data[2] == MAXLONG)) {
		for (uint8_t a = 0; a < 3; ++a) {
			if (queue[queue_start].data[a] == MAXLONG) {
#ifdef DEBUG_MOVE
				debug ("Filling delta axis %d", a);
#endif
				// Offset will be added later; compensate for it.
				queue[queue_start].data[a] = axis[a].source == MAXLONG ? MAXLONG : axis[a].source - axis[a].offset;
			}
		}
	}
#endif
#if MAXAXES > 0
	for (uint8_t a = 0; a < num_axes; ++a) {
		debug("checking %ld %ld", F(queue[queue_start].data[a]), F(axis[a].source));
		if (queue[queue_start].data[a] != MAXLONG && axis[a].source == MAXLONG) {
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
			if (queue[queue_start].data[mt] == MAXLONG)
				motors[mtr]->next_dist = 0;
			else {
#if MAXAXES > 0
				if (mt < num_axes) {
					motors[mtr]->next_dist = queue[queue_start].data[mt] - axis[mt].source;
#ifdef DEBUG_MOVE
					debug("next dist of axis %d: %ld - %ld = %ld", mt, F(queue[queue_start].data[mt]), F(axis[mt].source), F(motors[mtr]->next_dist));
#endif
				}
				else {
#endif
					motors[mtr]->next_dist = queue[queue_start].data[mt];
#ifdef DEBUG_MOVE
					debug("next dist of extruder %d = %ld", mt - num_axes, F(motors[mtr]->next_dist));
#endif
				}
			}
		}
	}
	// }}}
	// Otherwise, we are prepared and can start the segment. {{{
	bool action = false;
#ifdef DEBUG_MOVE
	debug ("Move was prepared with tp = %ld", F(tp));
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
			debug ("Last segment distance for motor %d is %ld", mtr, F(motors[mtr]->dist));
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
			if (queue[n].data[mt] == MAXLONG)
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
			debug ("Connecting distance for motor %d is %ld, to %ld", mtr, F(motors[mtr]->dist), F(motors[mtr]->next_dist));
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
			motors[mtr]->dist = MAXLONG;
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
	debug ("Set up: tp = %ld s, v0 = %f /s, vp = %f /s, vq = %f /s", F(tp), F(v0), F(vp), F(vq));
#endif

	// Limit v0, vp, vq. {{{
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t mtr = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[mtr])
			continue;
		if (motors[mtr]->max_v != MAXLONG) {
			float a;
			if (motors[mtr]->dist != 0) {
				a = motors[mtr]->max_v / motors[mtr]->dist;
				if (v0 > a)
					v0 = a;
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
	debug ("Preparation did f0 = %ld", F(f0));
#endif

	int32_t fp;
	int32_t fq;
#if MAXAXES > 0
	// Use maximum deviation to find fraction where to start rounded corner.
	int32_t um_round;
	int32_t norma2 = 0, normb2 = 0, inner = 0;
	for (uint8_t a = 0; a < num_axes; ++a) {
		inner += m2(axis[a].motor.dist) * m2(axis[a].motor.next_dist);
		norma2 += m2(axis[a].motor.dist) * m2(axis[a].motor.dist);
		normb2 += m2(axis[a].motor.next_dist) * m2(axis[a].motor.next_dist);
	}
	int32_t norma(sqrt(kf(norma2)));
	int32_t normb(sqrt(kf(normb2)));
	if (norma <= 0 || normb <= 0) {
		// At least one of the segments is 0; no rounded corner.
		um_round = 0;
	}
	else {
		int32_t summed2 = 0;
		for (uint8_t a = 0; a < num_axes; ++a) {
			int32_t s = m2(k(axis[a].motor.dist) / norma + k(axis[a].motor.next_dist) / normb);
			summed2 += s * s;
		}
		int32_t scale(sqrt(kf(summed2)) / 2);
		um_round = k(max_deviation) / scale;
	}
	// Convert mm to fraction for both p and q.  Initially use μ1 instead of n1 to avoid overflows.
	fp = norma >= k(1) ? k(um_round) / m(norma) : 0;
	fq = normb >= k(1) ? k(um_round) / m(normb) : 0;
	if (fp > M(1) / 2) {
		fq = k(fq) / 2 / fp * k(1);
		fp = M(1) / 2;
	}
	if (fq > M(1) / 2) {
		fp = k(fq) / 2 / fq * k(1);
		fq = M(1) / 2;
	}
	fp = k(fp);
	fq = k(fq);
#else
	fp = 0;
	fq = 0;
#endif
	// Set up t0, tp.
	t0 = m((G(1) - fp) / (abs(v0 + vp) / 2));
	tp = m(fp / (abs(vp) / 2));
	// Set up f1, f2.
	f1 = k(.5 * abs(v0) * t0);
	f2 = G(1) - fp - f1;

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
				debug ("Move motor %ld from %ld (really %ld) over %ld steps (f0=%f)", mtr, F(axis[mt].source), F(axis[mt].current), F(motors[mtr]->dist), F(f0));*/
		}
		motors[mtr]->main_dist = m(motors[mtr]->dist * u(G(1) - fp));
#ifdef DEBUG_MOVE
		debug ("Motor %d dist %ld main dist = %ld, next dist = %ld", mtr, F(motors[mtr]->dist), F(motors[mtr]->main_dist), F(motors[mtr]->next_dist));
#endif
#if MAXEXTRUDERS > 0
		if (mt >= num_axes)
			extruder[mt - num_axes].distance_done = motors[mtr]->dist * f0;
#endif
	}
#ifdef DEBUG_MOVE
	debug ("Segment has been set up: f0=%ld fp=%ld fq=%ld v0=%f /s vp=%f /s vq=%f /s t0=%ld μs tp=%ld μs", F(f0), F(fp), F(fq), F(v0), F(vp), F(vq), F(t0), F(tp));
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
			//debug ("setting axis %d source to %ld", a, F(axis[a].current));
			axis[a].source = axis[a].current;
		}
#endif
		for (uint8_t mtr = 0; mtr < MAXOBJECT; ++mtr) {
			if (!motors[mtr])
				continue;
			motors[mtr]->continuous_v = 0;
			motors[mtr]->f = 0;
			motors[mtr]->last_v = 0;
			motors[mtr]->dist = MAXLONG;
			motors[mtr]->next_dist = MAXLONG;
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

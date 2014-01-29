// vim: set foldmethod=marker :
#include "firmware.h"

//#define DEBUG_MOVE

// Set up:
// start_time		micros() at start of move.
// t0			time to do main part.
// tq			time to do connection.
// m->dist		total distance of this segment (mm).
// m->next_dist		total distance of next segment (mm).
// m->main_dist		distance of main part (mm).
// m->move_done	0
// v0, vp		start and end velocity for main part. (fraction/s)
// vq			end velocity of connector part. (fraction/s)
void next_move () {
	if (queue_start == queue_end)
		return;
#ifdef DEBUG_MOVE
	debug ("next move qstart %d qend %d", queue_start, queue_end);
#endif
	// Set everything up for running queue[queue_start].

	if (isnan (axis[0].source)) { // {{{
		// Determine current location of extruder.
		if (printer_type == 0) { // {{{
			for (uint8_t a = 0; a < MAXAXES; ++a)
				axis[a].source = axis[a].current_pos / axis[a].motor.steps_per_mm + axis[a].offset;
		}
		// }}}
		else if (printer_type == 1) { // {{{
			// This is very slow, but that's no problem, because it is only used after the position is lost
			// (when a limit switch is hit, when a motor is run, and when reset is received.)
			//
			// (z_i-z_p)^2=l_i^2-(x_i-x_p)^2-(y_i-y_p)^2 === Z_i
			// dZ_i/dx_p=2(x_i-x_p) and dZ_i/dy_p=2(y_i-y_p)
			debug ("new delta position");
			float xp = 0;
			float yp = 0;
			while (false && true) {
				for (uint8_t c = 0; c < 2; ++c) {
					for (uint8_t a = 0; a < 3; ++a) {
						//float dx = axis[a].x - xp;
						//float dy = axis[a].y - yp;
						float Z[3];
						for (uint8_t a2 = 0; a2 < 3; ++a2) {
							float dx2 = axis[a2].x - xp;
							float dy2 = axis[a2].y - yp;
							Z[a2] = sqrt (axis[a2].delta_length * axis[a2].delta_length - dx2 * dx2 - dy2 * dy2);
						}
						//float dZdxp = -dx / Z[a];
						//float dZdyp = -dy / Z[a];
						uint8_t b = (a + 1) % 3;
						uint8_t c = (a + 2) % 3;
						float error = Z[b] * axis[b].motor.steps_per_mm - axis[b].current_pos;
						error += Z[c] * axis[c].motor.steps_per_mm - axis[c].current_pos;
						error /= 2;
						error = Z[a] * axis[a].motor.steps_per_mm - axis[a].current_pos - error;
						// Positive error means Z is too large.
					}
				}
			}
			// TODO
			axis[0].source = axis[0].offset;
			axis[1].source = axis[1].offset;
			axis[2].source = axis[2].limit_max_pos / axis[2].motor.steps_per_mm + axis[2].offset;
		}
		// }}}
		else
			debug ("Error: invalid printer type %d", printer_type);
	}
	// }}}
	if (!move_prepared) { // {{{
		// We have to speed up first; don't pop anything off the queue yet, just set values to handle segment 0.
#ifdef DEBUG_MOVE
		debug ("no move prepared");
#endif
		bool action = false;
		for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
			uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
			if (!motors[m])
				continue;
			if (isnan (queue[queue_start].data[mt + 2])) {
				motors[m]->dist = NAN;
				motors[m]->next_dist = NAN;
				continue;
			}
			motors[m]->dist = 0;
			if (mt < num_axes)
				motors[m]->next_dist = queue[queue_start].data[mt + 2] - axis[mt].offset - axis[mt].source;
			else
				motors[m]->next_dist = queue[queue_start].data[mt + 2];
			if (motors[m]->next_dist != 0)
				action = true;
#ifdef DEBUG_MOVE
			debug ("next dist motor %d %f", m, &motors[m]->next_dist);
#endif
		}
		if (!action) {
			// Nothing to do; discard this segment and try again.
#ifdef DEBUG_MOVE
			debug ("skipping zero move");
#endif
			if (((queue_end + 1) & QUEUE_LENGTH_MASK) == queue_start)
				continue_cb |= 1;
			if (queue[queue_start].cb)
				++num_movecbs;
			queue_start = (queue_start + 1) & QUEUE_LENGTH_MASK;
			try_send_next ();
			moving = false;
			return next_move ();
		}
		tq = 0;	// Previous value; new value is computed below.
		v0 = 0;
		vp = 0;
		vq = queue[queue_start].data[F0] * feedrate;
		current_move_has_cb = false;
		move_prepared = true;
	}
	// }}}
	else { // We are prepared and can start at segment 1.  Set up everything. {{{
		bool action = false;
		v0 = vq;
#ifdef DEBUG_MOVE
		debug ("move prepared, v0 = %f tq = %d", &v0, int (tq / 1000));
#endif
		uint8_t n = (queue_start + 1) & QUEUE_LENGTH_MASK;
		if (n == queue_end) { // {{{
#ifdef DEBUG_MOVE
			debug ("last segment");
#endif
			// There is no next segment; we should stop at the end.
			for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
				uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
				if (!motors[m])
					continue;
				motors[m]->dist = motors[m]->next_dist;
				if (isnan (queue[queue_start].data[mt + 2]))
					motors[m]->next_dist = NAN;
				else
					motors[m]->next_dist = 0;
				if ((!isnan (motors[m]->next_dist) && motors[m]->next_dist != 0) || (!isnan (motors[m]->dist) && motors[m]->dist != 0))
					action = true;
#ifdef DEBUG_MOVE
				debug ("m %d dist %f nd %f", m, &motors[m]->dist, &motors[m]->next_dist);
#endif
			}
			vp = queue[queue_start].data[F1] * feedrate;
			vq = 0;
			move_prepared = false;
		}
		// }}}
		else {
#ifdef DEBUG_MOVE
			debug ("connection");
#endif
			// There is a next segment; we should connect to it.
			for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
				uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
				if (!motors[m])
					continue;
				motors[m]->dist = motors[m]->next_dist;
				if (isnan (queue[queue_start].data[mt + 2]))
					motors[m]->next_dist = NAN;
				else
					motors[m]->next_dist = mt < num_axes ? queue[n].data[mt + 2] - axis[mt].offset - (axis[mt].source + motors[m]->dist) : queue[n].data[mt + 2];
				if ((!isnan (motors[m]->next_dist) && motors[m]->next_dist != 0) || (!isnan (motors[m]->dist) && motors[m]->dist != 0))
					action = true;
#ifdef DEBUG_MOVE
				debug ("m2 %d dist %f nd %f", m, &motors[m]->dist, &motors[m]->next_dist);
#endif
			}
			vp = queue[queue_start].data[F1] * feedrate;
			vq = queue[n].data[F0] * feedrate;
			move_prepared = true;
		}
		current_move_has_cb = queue[queue_start].cb;
		if (((queue_end + 1) & QUEUE_LENGTH_MASK) == queue_start) {
#ifdef DEBUG_MOVE
			debug ("continue regular");
#endif
			continue_cb |= 1;
			try_send_next ();
		}
		queue_start = n;
		if (!action) {
#ifdef DEBUG_MOVE
			debug ("skipping zero prepared move");
#endif
			if (current_move_has_cb) {
				++num_movecbs;
				try_send_next ();
			}
			moving = false;
			return next_move ();
		}
	} // }}}
	// Limit vp and vq. {{{
#ifdef DEBUG_MOVE
	debug ("before limit vp %f vq %f", &vp, &vq);
#endif
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[m] || (isnan (motors[m]->dist) && isnan (motors[m]->next_dist)))
			continue;
		// TODO: limit motors, not axes, for delta.
		float max = motors[m]->dist > 0 ? motors[m]->max_v_pos : motors[m]->max_v_neg;
		if (!isnan (motors[m]->dist) && vp * abs (motors[m]->dist) > max)
			vp = max / abs (motors[m]->dist);
		if (!isnan (motors[m]->next_dist) && vq * abs (motors[m]->next_dist) > max)
			vq = max / abs (motors[m]->next_dist);
#ifdef DEBUG_MOVE
		debug ("%d vpq %f %f %f %f", m, &vp, &vq, &max, &motors[m]->next_dist);
#endif
	}
	if (isinf (vp))
		vp = 0;
	if (isinf (vq))
		vq = 0;
#ifdef DEBUG_MOVE
	debug ("after limit vp %f vq %f", &vp, &vq);
#endif
	// }}}
	// Already set up: v0, vp, vq, m->dist, m->next_dist.
	f0 = v0 * tq / 2e6;
#ifdef DEBUG_MOVE
	debug ("f0 %f tq %d v0 %f", &f0, int (tq / 1000), &v0);
#endif
	// Find shortest tq. {{{
	tq = 0;
	t0 = 0;
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[m] || (isnan (motors[m]->dist) && isnan (motors[m]->next_dist)))
			continue;
		RESET (motors[m]->enable_pin);
		// Find speeds in mm/s.
		float v = isnan (motors[m]->dist) ? 0 : vp * motors[m]->dist;
		float v0_mm = isnan (motors[m]->dist) ? 0 : v0 * motors[m]->dist;
		float next_v = isnan (motors[m]->next_dist) ? 0 : vq * motors[m]->next_dist;
		long min_t = long (abs (v - next_v) / motors[m]->max_a * 1e6);
#ifdef DEBUG_MOVE
		debug ("mint %d %f %f", int (min_t / 1000), &v, &next_v);
#endif
		if (tq < min_t)
			tq = min_t;
		min_t = long (abs (v - v0_mm) / motors[m]->max_a * 1e6);
		if (t0 < min_t)
			t0 = min_t;
	}
#ifdef DEBUG_MOVE
	debug ("tq %d t0 %d", int (tq / 1000), int (t0 / 1000));
#endif
	// }}}
	// Check if we're within the segment's limits. {{{
	float ftq = tq / 1e6;
	float fq = vq / 2 * ftq;
	float fp = vp / 2 * ftq;
	float fmain = (v0 + vp) * t0 / 2e6;
#ifdef DEBUG_MOVE
	debug ("ftq %f fp %f fq %f", &ftq, &fp, &fq);
#endif
	if (fq > .5 || fp > .5 || fmain + fp > 1 - f0) {
		// Change vp and vq so it's ok. TODO: check this.
		float factor = .5 / max (fq, fp);
		float mf = (1 - f0) / (fmain + fp);
		if (mf < factor)
			factor = mf;
		fp *= factor;
		fq *= factor;
		fmain *= factor;
		factor = sqrt (factor);
		vp *= factor;
		vq *= factor;
		tq *= factor;
#ifdef DEBUG_MOVE
		debug ("adjust %f f0 %f fp %f fq %f fmain %f v %f %f tq %d", &factor, &f0, &fp, &fq, &fmain, &vp, &vq, int (tq / 1000));
#endif
	}
	// }}}
	// Fill variables. {{{
	if (abs (v0 + vp) > 1e-10) {
		t0 = long ((1 - fp - f0) / (v0 + vp) * 2e6);
#ifdef DEBUG_MOVE
		debug ("t0 set %d fp %f f0 %f v0 %f vp %f", int (t0 / 1000), &fp, &f0, &v0, &vp);
#endif
	}
	else
		t0 = 0;
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[m])
			continue;
		if (isnan (motors[m]->dist)) {
			motors[m]->main_dist = 0;
			continue;
		}
		motors[m]->main_dist = motors[m]->dist * (f0 + (v0 + vp) * t0 / 2e6);
#ifdef DEBUG_MOVE
		debug ("main dist %d %f %f %f %d", m, &motors[m]->main_dist, &v0, &vp, int (t0 / 1000));
#endif
		if (mt < num_axes)
			axis[mt].current = axis[mt].source + axis[mt].motor.dist * f0;
		else
			extruder[mt - num_axes].steps_done = f0 * motors[m]->dist * motors[m]->steps_per_mm;
	}
#ifdef DEBUG_MOVE
	debug ("source 2 %f pos %d", &axis[0].source, int (axis[0].current_pos));
	debug ("general f0=%f fp=%f fq=%f v0=%f vp=%f vq=%f t0=%d tq=%d", &f0, &fp, &fq, &v0, &vp, &vq, int (t0/1000), int (tq/1000));
#endif
	motors_busy = true;
	if (tq == 0) {
		if (t0 == 0) {
			// This is a no-move segment; pop it off the queue to prevent a deadlock.
			if (((queue_end + 1) & QUEUE_LENGTH_MASK) == queue_start) {
#ifdef DEBUG_MOVE
				debug ("continue zero");
#endif
				continue_cb |= 1;
				try_send_next ();
			}
			queue_start = (queue_start + 1) & QUEUE_LENGTH_MASK;
		}
		move_prepared = false;
	}
	moving = true;
	start_time = micros ();
	// }}}
}

void abort_move () {
	if (moving) {
		if (current_move_has_cb)
			++num_movecbs;
		for (uint8_t a = 0; a < MAXAXES; ++a) {
			if (isnan (axis[a].motor.dist))
				continue;
			axis[a].source = axis[a].current;
			axis[a].motor.dist = NAN;
		}
		moving = false;
		move_prepared = false;
#ifdef DEBUG_MOVE
		debug ("move no longer prepared");
#endif
	}
	if (((queue_end + 1) & QUEUE_LENGTH_MASK) == queue_start)
		continue_cb |= 1;
	while (queue_start != queue_end) {
		if (queue[queue_start].cb)
			++num_movecbs;
		queue_start = (queue_start + 1) & QUEUE_LENGTH_MASK;
	}
	// Not always required, but this is a slow operation anyway and it doesn't harm if it's called needlessly.
	try_send_next ();
}

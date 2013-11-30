// vim: set foldmethod=marker foldmarker={,} :
#include "firmware.h"

void next_move () {
	if (queue_start == queue_end)
		return;
	//debug ("next %d %d", queue_start, queue_end);
	// Set everything up for running queue[queue_start].
	v[1] = queue[queue_start].data[F0] * feedrate;
	v[2] = queue[queue_start].data[F1] * feedrate;
	// For all motors except delta axes, set steps_total, steps_done.
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[m] || isnan (queue[queue_start].data[mt + 2]) || (printer_type == 1 && m < 2 + MAXAXES))
			continue;
		float target = queue[queue_start].data[mt + 2];
		int32_t diff;
		if (m >= 2 && m < 2 + MAXAXES)
			diff = int32_t (target * motors[m]->steps_per_mm + .5) - axis[m - 2].current_pos;
		else
			diff = int32_t (target * motors[m]->steps_per_mm + .5);
		if (diff != 0 && (motors[m]->audio_flags & (Motor::PLAYING | Motor::STATE)) == (Motor::PLAYING | Motor::STATE)) {
			diff -= 1;
			motors[m]->audio_flags &= ~Motor::STATE;
		}
		motors[m]->positive = diff > 0;
		motors[m]->steps_total = abs (diff);
		motors[m]->steps_done = 0;
		//debug ("motor %d steps %d", m, motors[m]->steps_total);
	}
	// For delta axes, set steps_total, steps_done.
	if (printer_type == 1) {
#define sin120 0.8660254037844386	// .5*sqrt(3)
#define cos120 -.5
		// Coordinates of axes (at angles 0, 120, 240; each with its own radius).
		axis[0].x = axis[0].delta_radius;
		axis[1].x = axis[1].delta_radius * cos120;
		axis[2].x = axis[2].delta_radius * cos120;
		axis[0].y = 0;
		axis[1].y = axis[1].delta_radius * sin120;
		axis[2].y = axis[2].delta_radius * -sin120;
		for (uint8_t ta = 0; ta < 3; ++ta)
			axis[ta].z = sqrt (axis[ta].delta_length * axis[ta].delta_length - axis[ta].delta_radius * axis[ta].delta_radius);
		// Set target coordinates.
		for (uint8_t c = 0; c < 3; ++c) {
			if (!isnan (queue[queue_start].data[AXIS0 + c]))
				delta_target[c] = queue[queue_start].data[AXIS0 + c];
			else
				delta_target[c] = delta_source[c];
		}
		// Limit delta_target to reachable coordinates.
		int32_t diff[3];
		bool ok = true;
		for (uint8_t ta = 0; ta < 3; ++ta)
			diff[ta] = delta_to_axis (ta, delta_target, &ok);
		if (!ok) {
			debug ("Warning: limiting delta target");
			for (uint8_t ta = 0; ta < 3; ++ta)
				diff[ta] = delta_to_axis (ta, delta_target, &ok);
		}
		// Find target motor positions.
		for (uint8_t ta = 0; ta < 3; ++ta) {
			axis[ta].motor.steps_total = abs (diff[ta]);
			axis[ta].motor.positive = diff[ta] > 0;
			axis[ta].motor.steps_done = 0;
			//debug ("delta %d steps %x %x", ta, int (axis[ta].motor.steps_total >> 16), int (axis[ta].motor.steps_total));
		}
	}
	// Set v[0], v[3], f[0], f[3].
	v[0] = 0;
	v[3] = 0;
	f[0] = 0;
	f[3] = 1;
	// Limit v[1] and v[2] to max values.
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[m] || isnan (queue[queue_start].data[mt + 2]) || motors[m]->steps_total == 0)
			continue;
		float max_v = motors[m]->positive ? motors[m]->max_v_pos : motors[m]->max_v_neg;
		float v1 = motors[m]->steps_total * v[1];
		float v2 = motors[m]->steps_total * v[2];
		if (v1 > max_v) {
			//float tmp = v[1];
			//debug ("limit %d v1 %d to %f %f %f", m, int (motors[m]->steps_total), &max_v, &tmp, &v1);
			v[1] = max_v / motors[m]->steps_total;
		}
		if (v2 > max_v) {
			//float tmp = v[2];
			//debug ("limit %d v2 %d to %f %f %f", m, int (motors[m]->steps_total), &max_v, &tmp, &v2);
			v[2] = max_v / motors[m]->steps_total;
		}
	}
	//debug ("v %f %f", &v[1], &v[2]);
	// Find (shortest possible) start and end times (now delta, later absolute).
	float t1f = 0, t2f = 0, t3f = 0;
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[m] || isnan (queue[queue_start].data[mt + 2]) || motors[m]->steps_total == 0)
			continue;
		float limit = abs ((v[1] - v[0]) * motors[m]->steps_total) / motors[m]->max_a;
		if (t1f < limit)
			t1f = limit;
		limit = abs ((v[3] - v[2]) * motors[m]->steps_total) / motors[m]->max_a;
		if (t3f < limit)
			t3f = limit;
	}
	//debug ("limits %f %f", &t1f, &t3f);
	// Set t2f and update t1f and t3f to be absolute time.
	// t2f - t1f is the time for the piece in the middle; it's the fraction to do divided by the average speed.
	// The fraction is all (1) minus what is done during t1 and t3 (which have average speeds of v[x]/2).
	// The average speed is halfway the start and end speed.
	t2f = t1f + (1. - (v[1] * t1f / 2 + v[2] * t3f / 2)) / ((v[1] + v[2]) / 2);
	t3f += t2f;
	//debug ("times %f %f %f", &t1f, &t2f, &t3f);
	if (t2f < t1f) {
		// Speeding up and slowing down takes more steps than we need to do in total.
		// Speed up as fast as we can, then slow down as fast as we can, doing the correct number of steps.
		float t1 = -INFINITY;
		float t2 = -INFINITY, t_error = 0;
		for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
			uint8_t m = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
			if (!motors[m] || isnan (queue[queue_start].data[mt + 2]) || motors[m]->steps_total == 0)
				continue;
			float new_a = motors[m]->max_a / motors[m]->steps_total;
			float new_t1 = sqrt (f[3] / new_a + (v[0] * v[0] + v[3] * v[3]) / (2 * new_a * new_a)) - v[0] / new_a;
			if (new_t1 > t1) {
				t1 = new_t1;
				t2 = t1 + (v[0] - v[3]) / new_a;
				a[0] = new_a;
				f[1] = v[0] * t1 + .5 * new_a * t1 * t1;
				t_error = 2 * f[3] / (v[0] + v[3]);
			}
		}
		if (t1 <= 0) {
			// This shouldn't happen: we entered the segment at high speed, and do not have enough steps to slow down within the restrictions.
			// Slow down faster than max_a; let's hope we aren't skipping steps.
			t1f = 0;
			t2f = 0;
			t3f = t_error;
			f[1] = f[0];
			v[1] = v[0];
			f[2] = f[1];
			v[2] = v[1];
			debug ("Warning: exceeding max acceleration");
		}
		else {
			t1f = t1;
			t2f = t1f;
			t3f = t2f + t2;
			v[1] = v[0] + a[0] * t1;
			f[2] = f[1];
			v[2] = v[1];
		}
	}
	// Set a[0], a[2], f[1], f[2]
	a[0] = (v[1] - v[0]) / t1f;
	a[2] = (v[3] - v[2]) / (t3f - t2f);
	f[1] = f[0] + (v[1] + v[0]) / 2 * t1f;
	f[2] = f[3] - (v[2] + v[3]) / 2 * (t3f - t2f);
	// Set a[1]
	a[1] = (v[2] - v[1]) / (t2f - t1f);
	// Go to next queue position.
	if (((queue_end + 1) & QUEUE_LENGTH_MASK) == queue_start)
		continue_cb = true;
	queue_start = (queue_start + 1) & QUEUE_LENGTH_MASK;
	if (continue_cb)
		try_send_next ();
	// Set up motors.
	for (uint8_t m = 0; m < MAXOBJECT; ++m) {
		if (!motors[m] || (motors[m]->steps_total == 0 && (printer_type == 0 || m >= 2 + 3)))
			continue;
		if (motors[m]->positive)
			SET (motors[m]->dir_pin);
		else
			RESET (motors[m]->dir_pin);
		RESET (motors[m]->enable_pin);
	}
	motors_busy = true;
	phase = 0;
	t[0] = micros ();
	t[1] = t[0] + (unsigned long)(t1f * 1e6);
	t[2] = t[0] + (unsigned long)(t2f * 1e6);
	t[3] = t[0] + (unsigned long)(t3f * 1e6);
	//debug ("xsteps %x %x t %d %d %d %d v %f %f %f %f a %f %f %f %f f %f %f %f %f", int (axis[0].motor.steps_total >> 16), int (axis[1].motor.steps_total), int (t[0]), int (t[1]), int (t[2]), int (t[3]), &v[0], &v[1], &v[2], &v[3], &a[0], &a[1], &a[2], &a[3], &f[0], &f[1], &f[2], &f[3]);
}

// vim: set filetype=cpp foldmethod=marker foldmarker={,} :
#define EXTERN	// This must be done in exactly one cc-file.
#include "firmware.h"

// Loop function handles all regular updates.
// I'd have liked to have a timer interrupt for these actions, but arduino doesn't allow it.

static uint8_t temp_counter = 1;
static uint8_t temp_current = 0;
static void handle_temps (unsigned long current_time, unsigned long longtime) {
	if (--temp_counter)
		return;
	temp_counter = 20;
	uint8_t i;
	for (i = 1; i <= MAXOBJECT; ++i) {
		uint8_t next = (temp_current + i) % MAXOBJECT;
		if (temps[next] && (!isnan (temps[next]->target) || !isnan (temps[next]->min_alarm) || !isnan (temps[next]->max_alarm)) && !temps[next]->power_pin.invalid () && (!temps[next]->thermistor_pin.invalid () || (temps[next]->target > 0 && isinf (temps[next]->target)))) {
			temp_current = next;
			break;
		}
	}
	// If there is no temperature handling to do; return.
	if (i > MAXOBJECT) {
		return;
	}
	float temp = NAN;
	if (!temps[temp_current]->thermistor_pin.invalid ()) {
		temp = temps[temp_current]->read ();
		// First of all, if an alarm should be triggered, do so.
		if ((!isnan (temps[temp_current]->min_alarm) && temps[temp_current]->min_alarm < temp) || (!isnan (temps[temp_current]->max_alarm) && temps[temp_current]->max_alarm > temp)) {
			temps[temp_current]->min_alarm = NAN;
			temps[temp_current]->max_alarm = NAN;
			which_tempcbs |= (1 << temp_current);
			try_send_next ();
		}
	}
	if (isinf (temps[temp_current]->target) && temps[temp_current]->target > 0) {
		if (!temps[temp_current]->is_on) {
			//debug ("switching on %d", temp_current);
			SET (temps[temp_current]->power_pin);
			temps[temp_current]->is_on = true;
			++temps_busy;
			last_active = longtime;
		}
	}
	if (temps[temp_current]->thermistor_pin.invalid ())
		return;
	if (isnan (temps[temp_current]->core_C) || isnan (temps[temp_current]->shell_C) || isnan (temps[temp_current]->transfer) || isnan (temps[temp_current]->radiation)) {
		// No valid settings; use simple on/off-regime based on current temperature only.
		if (temp < temps[temp_current]->target) {
			if (!temps[temp_current]->is_on) {
				//debug ("switching on %d", temp_current);
				SET (temps[temp_current]->power_pin);
				temps[temp_current]->is_on = true;
				++temps_busy;
				last_active = longtime;
			}
		}
		else {
			if (temps[temp_current]->is_on) {
				//debug ("switching off %d", temp_current);
				RESET (temps[temp_current]->power_pin);
				temps[temp_current]->is_on = false;
				--temps_busy;
				last_active = longtime;
			}
		}
		return;
	}
	unsigned long dt = current_time - temps[temp_current]->last_time;
	if (dt == 0)
		return;
	temps[temp_current]->last_time = current_time;
	float fdt = dt  * 1.0 / 1e6;
	// Heater and core/shell transfer.
	if (temps[temp_current]->is_on)
		temps[temp_current]->core_T += temps[temp_current]->power / temps[temp_current]->core_C * fdt / 2;
	float Q = temps[temp_current]->transfer * (temps[temp_current]->core_T - temps[temp_current]->shell_T) * fdt;
	temps[temp_current]->core_T -= Q / temps[temp_current]->core_C;
	temps[temp_current]->shell_T += Q / temps[temp_current]->shell_C;
	if (temps[temp_current]->is_on)
		temps[temp_current]->core_T += temps[temp_current]->power / temps[temp_current]->core_C * fdt / 2;
	// Set shell to measured value.
	temps[temp_current]->shell_T = temp;
	// Add energy if required.
	float E = temps[temp_current]->core_T * temps[temp_current]->core_C + temps[temp_current]->shell_T * temps[temp_current]->shell_C;
	float T = E / (temps[temp_current]->core_C + temps[temp_current]->shell_C);
	if (T < temps[temp_current]->target) {
		if (!temps[temp_current]->is_on) {
			SET (temps[temp_current]->power_pin);
			temps[temp_current]->is_on = true;
			++temps_busy;
		}
	}
	else {
		if (temps[temp_current]->is_on) {
			RESET (temps[temp_current]->power_pin);
			temps[temp_current]->is_on = false;
			--temps_busy;
		}
	}
}

static void done_motors () {
	// Mark motors as not moving.
	for (uint8_t m = 0; m < MAXOBJECT; ++m) {
		if (!motors[m] || isnan (motors[m]->dist))
			continue;
		motors[m]->dist = NAN;
	}
	moving = false;
	next_move ();
}

static bool do_steps (uint8_t m, int16_t num_steps) {
	if (num_steps == 0)
		return true;
	if (motors[m]->positive != (num_steps > 0)) {
		if (num_steps > 0)
			SET (motors[m]->dir_pin);
		else
			RESET (motors[m]->dir_pin);
		motors[m]->positive = (num_steps > 0);
		//debug ("reverse positive %d %d", motors[m]->positive, num_steps);
	}
	if (abs (num_steps) > 30) {	// Testing shows that up to 33 steps at once works (with 16x microstepping).
		debug ("many steps %d: %d", m, num_steps);
	}
	if (m >= 2 && m < MAXAXES + 2) {
		// No problem if limit switch is not hit.
		if (motors[m]->positive ? GET (axis[m - 2].limit_max_pin, false) : GET (axis[m - 2].limit_min_pin, false)) {
			// Hit endstop; abort current move and notify host.
			debug ("hit %d %d %d %d", int (m - 2), int (axis[m - 2].current_pos), int (motors[m]->positive), int (axis[m - 2].limit_max_pin.write ()));
			// Stop continuous move only for the motor that hits the switch.
			motors[m]->f = 0;
			motors[m]->continuous_steps_per_s = 0;
			limits_pos[m - 2] = axis[m - 2].current_pos;
			try_send_next ();
			if (moving && !isnan (motors[m]->dist)) {
				abort_move ();
				done_motors ();
			}
			return false;
		}
		axis[m - 2].current_pos += num_steps;
	}
        for (int16_t s = 0; s < abs (num_steps); ++s) {
		SET (motors[m]->step_pin);
		RESET (motors[m]->step_pin);
	}
	return true;
}

static void move_axes (float target[3]) {
	uint8_t a = 0;
	switch (printer_type) {
	case 1:
		// Delta movements are hardcoded to 3 axes.
		for (uint8_t a2 = 0; a2 < 3; ++a2) {
			bool ok = true;
			int32_t the_target = delta_to_axis (a2, target, &ok);
			if (!do_steps (2 + a2, the_target - axis[a2].current_pos)) {
				// The move has been aborted.
				return;
			}
			axis[a2].current = target[a2];
		}
		a = 3;
		// Fall through to handle the non-delta axes.
	case 0:
		for (; a < num_axes; ++a)
			if (!isnan (target[a])) {
				if (!do_steps (2 + a, target[a] * axis[a].motor.steps_per_mm - axis[a].current_pos))
					return;
				axis[a].current = target[a];
			}
		break;
	default:
		debug ("Bug: printer_type %d not handled by move in " __FILE__, printer_type);
		break;
	}
}

static void handle_motors (unsigned long current_time, unsigned long longtime) {
	if (pause_all)
		return;
	// Check for continuous moves.
	for (uint8_t m = 0; m < MAXOBJECT; ++m) {
		if (!motors[m] || !isnan (motors[m]->dist))
			continue;
		if (motors[m]->continuous_steps_per_s == 0 && motors[m]->f == 0)
			continue;
		last_active = longtime;
		float current_t = (current_time - motors[m]->continuous_last_time) / 1e6;
		motors[m]->continuous_last_time = current_time;
		if (motors[m]->continuous_steps_per_s != motors[m]->f) {
			// Getting up to speed, or slowing down.
			if (motors[m]->continuous_steps_per_s > motors[m]->f) {
				motors[m]->f += current_t * motors[m]->max_a * motors[m]->steps_per_mm;
				if (motors[m]->continuous_steps_per_s < motors[m]->f)
					motors[m]->f = motors[m]->continuous_steps_per_s;
			}
			else {
				motors[m]->f -= current_t * motors[m]->max_a * motors[m]->steps_per_mm;
				if (motors[m]->f < 0 || (motors[m]->f == 0 && motors[m]->continuous_steps_per_s < 0)) {
					motors[m]->f = -motors[m]->f;
					motors[m]->continuous_steps_per_s = -motors[m]->continuous_steps_per_s;
					if (motors[m]->positive) {
						motors[m]->positive = false;
						RESET (motors[m]->dir_pin);
					}
					else {
						motors[m]->positive = true;
						SET (motors[m]->dir_pin);
					}
					//debug ("new positive %d", motors[m]->positive);
					if (motors[m]->continuous_steps_per_s < motors[m]->f)
						motors[m]->f = motors[m]->continuous_steps_per_s;
				}
				else if (motors[m]->continuous_steps_per_s > motors[m]->f)
					motors[m]->f = motors[m]->continuous_steps_per_s;
			}
		}
		motors[m]->continuous_steps += motors[m]->f * current_t;
		int16_t steps (motors[m]->continuous_steps);
		motors[m]->continuous_steps -= steps;
		do_steps (m, steps * (motors[m]->positive ? 1 : -1));
		continue;
	}
	// Check for regular move.
	if (!moving)
		return;
	last_active = longtime;
	long t = current_time - start_time;
	float target[num_axes];
	while (t >= t0 + tq) {	// Finish this move and prepare next.
		for (uint8_t a = 0; a < num_axes; ++a) {
			if (isnan (axis[a].motor.dist)) {
				target[a] = NAN;
				continue;
			}
			float dist = isnan (axis[a].motor.dist) ? 0 : axis[a].motor.dist;
			float next_dist = isnan (axis[a].motor.next_dist) ? 0 : axis[a].motor.next_dist;
			target[a] = axis[a].source + axis[a].motor.main_dist + (dist * vp + next_dist * vq) * tq / 2e6;
		}
		move_axes (target);
		for (uint8_t e = 0; e < num_extruders; ++e) {
			if (isnan (extruder[e].motor.dist))
				continue;
			float dist = isnan (extruder[e].motor.dist) ? 0 : extruder[e].motor.dist;
			float next_dist = isnan (extruder[e].motor.next_dist) ? 0 : extruder[e].motor.next_dist;
			float mm = extruder[e].motor.main_dist + (dist * vp + next_dist * vq) * tq / 2e6;
			if (!do_steps (2 + MAXAXES + e, mm * extruder[e].motor.steps_per_mm - extruder[e].steps_done))
				return;
		}
		if (moving && current_move_has_cb) {
			++num_movecbs;
			try_send_next ();
		}
		for (uint8_t a = 0; a < num_axes; ++a)
			if (!isnan (axis[a].motor.dist)) {
				//debug ("before source %d %f %f", a, &axis[a].source, &axis[a].motor.dist);
				axis[a].source += axis[a].motor.dist;
				//debug ("after source %d %f %f %d", a, &axis[a].source, &axis[a].motor.dist, int (axis[a].current_pos));
			}
		done_motors ();
		if (!moving)
			return;
		t = micros () - start_time;
	}
	if (t < t0) {	// Main part.
		float dist_fraction = f0 + (v0 + (vp - v0) / 2 * (t * 1. / t0)) * t / 1e6;
		for (uint8_t a = 0; a < num_axes; ++a) {
			if (isnan (axis[a].motor.dist)) {
				target[a] = NAN;
				continue;
			}
			target[a] = axis[a].source + axis[a].motor.dist * dist_fraction;
		}
		for (uint8_t e = 0; e < num_extruders; ++e) {
			if (isnan (extruder[e].motor.dist))
				continue;
			int32_t steps = extruder[e].motor.dist * dist_fraction * extruder[e].motor.steps_per_mm;
			do_steps (2 + MAXAXES + e, steps - extruder[e].steps_done);
			extruder[e].steps_done = steps;
		}
	}
	else {	// Connector part.
		t -= t0;
		for (uint8_t a = 0; a < num_axes; ++a) {
			if (isnan (axis[a].motor.dist) && isnan (axis[a].motor.next_dist)) {
				target[a] = NAN;
				continue;
			}
			float dist = isnan (axis[a].motor.dist) ? 0 : axis[a].motor.dist;
			float next_dist = isnan (axis[a].motor.next_dist) ? 0 : axis[a].motor.next_dist;
			target[a] = axis[a].source + axis[a].motor.main_dist + (dist * vp + (next_dist * vq - dist * vp) / 2 * (t * 1. / tq)) * t / 1e6;
		}
		for (uint8_t e = 0; e < num_extruders; ++e) {
			if (isnan (extruder[e].motor.dist) && isnan (extruder[e].motor.next_dist))
				continue;
			float dist = isnan (extruder[e].motor.dist) ? 0 : extruder[e].motor.dist;
			float next_dist = isnan (extruder[e].motor.next_dist) ? 0 : extruder[e].motor.next_dist;
			int32_t steps = (extruder[e].motor.main_dist + (dist * vp + (next_dist * vq - dist * vp) / 2 * (t * 1. / tq)) * t / 1e6) * extruder[e].motor.steps_per_mm;
			do_steps (2 + MAXAXES + e, steps - extruder[e].steps_done);
			extruder[e].steps_done = steps;
		}
	}
	move_axes (target);
}

static void handle_audio (unsigned long current_time, unsigned long longtime) {
	if (audio_head != audio_tail) {
		last_active = longtime;
		int8_t audio_state = -1;
		int16_t bit = (current_time - audio_start) / audio_us_per_bit;
		int16_t byte = bit >> 3;
		while (byte >= AUDIO_FRAGMENT_SIZE) {
			if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
			{
				continue_cb |= 2;
				try_send_next ();
			}
			audio_head = (audio_head + 1) % AUDIO_FRAGMENTS;
			if (audio_tail == audio_head)
				break;
			byte -= AUDIO_FRAGMENT_SIZE;
			// us per fragment = us/bit*bit/fragment
			audio_start += audio_us_per_bit * 8 * AUDIO_FRAGMENT_SIZE;
		}
		if (audio_head != audio_tail)
			audio_state = (audio_buffer[audio_head][byte] >> (bit & 7)) & 1;
		(void)audio_state;
		// TODO: move motors.
	}
}

static void handle_led (unsigned long current_time) {
	unsigned timing = temps_busy > 0 ? 1000 / 80 : 1000 / 50;
	if (current_time - led_last < timing)
		return;
	led_last += timing;
	led_phase += 1;
	led_phase %= 50;
	// Timings read from https://en.wikipedia.org/wiki/File:Wiggers_Diagram.png (phonocardiogram).
	bool state = (led_phase <= 4 || (led_phase >= 14 && led_phase <= 17));
	if (state)
		SET (led_pin);
	else
		RESET (led_pin);
}

void loop () {
	serial ();
	unsigned long current_time = micros ();
	unsigned long longtime = millis ();
	handle_temps (current_time, longtime);	// Periodic temps stuff: temperature regulation.
	handle_motors (current_time, longtime);	// Movement.
	handle_led (longtime);	// heart beat.
	handle_audio (current_time, longtime);
	if (motors_busy && motor_limit > 0 && longtime - last_active > motor_limit) {
		for (uint8_t m = 0; m < MAXOBJECT; ++m) {
			if (!motors[m])
				continue;
			SET (motors[m]->enable_pin);
		}
		motors_busy = false;
	}
	if (temps_busy > 0 && temp_limit > 0 && longtime - last_active > temp_limit) {
		for (uint8_t current_t = 0; current_t < MAXOBJECT; ++current_t) {
			if (!temps[current_t])
				continue;
			RESET (temps[current_t]->power_pin);
			temps[current_t]->target = NAN;
			temps[current_t]->is_on = false;
			last_active = longtime;
		}
		temps_busy = 0;
	}
}

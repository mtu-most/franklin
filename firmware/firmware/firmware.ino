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
		if (temps[next] && (!isnan (temps[next]->target) || !isnan (temps[next]->min_alarm) || !isnan (temps[next]->max_alarm)) && temps[next]->power_pin < 255 && (temps[next]->thermistor_pin < 255 || (temps[next]->target > 0 && isinf (temps[next]->target)))) {
			temp_current = next;
			break;
		}
	}
	// If there is no temperature handling to do; return.
	if (i > MAXOBJECT) {
		return;
	}
	float temp = NAN;
	if (temps[temp_current]->thermistor_pin < 255) {
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
	if (temps[temp_current]->thermistor_pin >= 255)
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
		if (!motors[m] || motors[m]->steps_total == 0)
			continue;
		motors[m]->steps_total = 0;
	}
	if (printer_type == 1) {
		for (uint8_t a = 0; a < 3; ++a)
			delta_source[a] = delta_target[a];
	}
	phase = 3;
	next_move ();
}

static bool do_steps (uint8_t m, int16_t num_steps) {
	if (num_steps > 30) {	// Testing shows that up to 33 steps at once works (with 16x microstepping).
		debug ("many steps %d: %d", m, num_steps);
	}
	if (m >= 2 && m < MAXAXES + 2) {
		// No problem if limit switch is not hit.
		if (motors[m]->positive ? GET (axis[m - 2].limit_max_pin, false) : GET (axis[m - 2].limit_min_pin, false)) {
			// Hit endstop; abort current move and notify host.
			debug ("hit %d %d", int (m - 2), int (axis[m - 2].current_pos));
			// Stop continuous move only for the motor that hits the switch.
			motors[m]->f = 0;
			motors[m]->continuous_steps_per_s = 0;
			limits_pos[m - 2] = axis[m - 2].current_pos;
			try_send_next ();
			if (motors[m]->steps_total != 0) {
				if (queue[queue_start].cb)
					++num_movecbs;
				done_motors ();
			}
			delta_source[0] = NAN;
			return false;
		}
		axis[m - 2].current_pos += int32_t (num_steps) * (motors[m]->positive ? 1 : -1);
	}
        for (int16_t s = 0; s < num_steps; ++s) {
		SET (motors[m]->step_pin);
		RESET (motors[m]->step_pin);
	}
	motors[m]->steps_done += num_steps;
	return true;
}

static void reverse (uint8_t m) {
	bool positive = !motors[m]->positive;
	if (positive)
		SET (motors[m]->dir_pin);
	else
		RESET (motors[m]->dir_pin);
	motors[m]->positive = positive;
	motors[m]->steps_done = -motors[m]->steps_done;
	motors[m]->steps_total = -motors[m]->steps_total;
}

static void handle_motors (unsigned long current_time, unsigned long longtime) {
	if (pause_all)
		return;
	int8_t audio_state = -1;
	uint8_t bit = (current_time - audio_start) / audio_us_per_bit;
	uint8_t byte = bit >> 3;
	if (audio_head != audio_tail) {
		while (byte >= AUDIO_FRAGMENT_SIZE) {
			audio_head = (audio_head + 1) % AUDIO_FRAGMENTS;
			if (audio_tail == audio_head)
				break;
			byte -= AUDIO_FRAGMENT_SIZE;
			// us per fragment = us/bit*bit/fragment
			audio_start += audio_us_per_bit * 8 * AUDIO_FRAGMENT_SIZE;
		}
		if (audio_head != audio_tail)
			audio_state = (audio_buffer[audio_head][byte] >> (bit & 7)) & 1;
	}
	while (phase < 3 && current_time - t[phase] >= t[phase + 1] - t[phase]) {
		//debug ("step %d %d %d %d %d %d", int (t[0]), int (current_time - t[0]), int (t[1] - t[0]), int (t[2] - t[0]), int (t[3] - t[0]), int (phase));
		if (++phase >= 3) {
			for (uint8_t m = 0; m < MAXOBJECT; ++m) {
				if (!motors[m] || !motors[m]->steps_total)
				       continue;
				// The move has ended; do any remaining steps.
				if (motors[m]->steps_total < 0)
					reverse (m);
				if (!do_steps (m, motors[m]->steps_total - motors[m]->steps_done))
					return;
			}
			if (queue[queue_start].cb) {
				++num_movecbs;
				try_send_next ();
			}
			done_motors ();
			break;
		}
	}
	// Check for continuous moves.
	for (uint8_t m = 0; m < MAXOBJECT; ++m) {
		if (!motors[m] || motors[m]->steps_total != 0)
			continue;
		if (motors[m]->continuous_steps_per_s == 0 && motors[m]->f == 0) {
			// Non-moving motor: play audio, if we should.
			if (motors[m]->audio_flags & Motor::PLAYING && audio_state >= 0 && !!audio_state != !!(motors[m]->audio_flags & Motor::STATE)) {
				if (motors[m]->audio_flags & Motor::STATE)
					RESET (motors[m]->dir_pin);
				else
					SET (motors[m]->dir_pin);
				SET (motors[m]->step_pin);
				RESET (motors[m]->step_pin);
				motors[m]->audio_flags ^= Motor::STATE;
			}
			continue;
		}
		last_active = longtime;
		float current_t = (current_time - motors[m]->continuous_last_time) / 1e6;
		motors[m]->continuous_last_time = current_time;
		if (motors[m]->continuous_steps_per_s != motors[m]->f) {
			// Getting up to speed, or slowing down.
			if (motors[m]->continuous_steps_per_s > motors[m]->f) {
				motors[m]->f += current_t * motors[m]->max_a;
				if (motors[m]->continuous_steps_per_s < motors[m]->f)
					motors[m]->f = motors[m]->continuous_steps_per_s;
			}
			else {
				motors[m]->f -= current_t * motors[m]->max_a;
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
		do_steps (m, steps);
		continue;
	}
	if (phase < 3) {
		last_active = longtime;
		float current_t = (current_time - t[phase]) / 1e6;
		float fraction = f[phase] + v[phase] * current_t + .5 * a[phase] * current_t * current_t;
		if (fraction > f[phase + 1])
			fraction = f[phase + 1];
		for (uint8_t m = 0; m < MAXOBJECT; ++m) {
			// Handle all regularly moving motors here, except for axes on a delta printer which knows its location.
			if (!motors[m] || motors[m]->steps_total == 0 || (printer_type == 1 && m < 2 + MAXAXES && !isnan (delta_source[0])))
				continue;
			do_steps (m, motors[m]->steps_total * fraction - motors[m]->steps_done);
		}
		if (printer_type == 1 && !isnan (delta_source[0])) {
			float pos[3];
			for (uint8_t c = 0; c < 3; ++c)
				pos[c] = delta_source[c] + (delta_target[c] - delta_source[c]) * fraction;
			// Delta movements are hardcoded to 3 axes.
			for (uint8_t a = 0; a < 3; ++a) {
				bool ok = true;
				int32_t diff = delta_to_axis (a, pos, &ok);
				bool positive = diff > 0;
				if (positive != axis[a].motor.positive)
					reverse (a + 2);
				do_steps (2 + a, abs (diff));
			}
		}
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

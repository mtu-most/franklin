// vim: set filetype=cpp foldmethod=marker foldmarker={,} :
#define EXTERN	// This must be done in exactly one source file.
#include "firmware.h"

//#define movedebug(...) do {} while (0)
#define movedebug(...) debug(__VA_ARGS__)

// Loop function handles all regular updates.

#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
static void handle_temps(unsigned long current_time, unsigned long longtime) {
	if (adc_phase == 0)
		return;
	if (adc_phase == 1) {
		// Find the temp to measure next time.
		uint8_t i;
		for (i = 1; i <= MAXOBJECT; ++i) {
			uint8_t next = (temp_current + i) % MAXOBJECT;
			if (temps[next] &&
					((temps[next]->adctarget < MAXINT && temps[next]->adctarget >= 0)
					 || (temps[next]->adcmin_alarm >= 0 && temps[next]->adcmin_alarm < MAXINT)
					 || temps[next]->adcmax_alarm < MAXINT
#ifndef LOWMEM
					 || temps[next]->gpios
#endif
					 )
					&& temps[next]->thermistor_pin.valid()) {
				temp_current = next;
				break;
			}
		}
		// If there is no temperature handling to do; disable (and abort the current one as well; it is no longer needed).
		if (i > MAXOBJECT) {
			adc_phase = 0;
			return;
		}
		temps[temp_current]->setup_read();
	}
	int16_t temp = temps[temp_current]->get_value();
	if (temp < 0)	// Not done yet.
		return;
	//debug("temp for %d: %d", temp_current, temp);
	// Set the phase so next time another temp is measured.
	adc_phase = 1;
	// First of all, if an alarm should be triggered, do so.  Adc values are higher for lower temperatures.
	if ((temps[temp_current]->adcmin_alarm < MAXINT && temps[temp_current]->adcmin_alarm > temp) || temps[temp_current]->adcmax_alarm < temp) {
		temps[temp_current]->min_alarm = MAXLONG;
		temps[temp_current]->max_alarm = MAXLONG;
		temps[temp_current]->adcmin_alarm = MAXINT;
		temps[temp_current]->adcmax_alarm = MAXINT;
		which_tempcbs |= (1 << temp_current);
		try_send_next();
	}
#ifndef LOWMEM
	// And handle any linked gpios.
	for (Gpio *g = temps[temp_current]->gpios; g; g = g->next) {
		// adc values are lower for higher temperatures.
		if (temp < g->adcvalue)
			SET(g->pin);
		else
			RESET(g->pin);
	}
#endif
	// If we don't have model settings, simply use the target as a switch between on and off.
#ifndef LOWMEM
	// Don't use those values yet.
	if (true || temps[temp_current]->core_C <= 0 || temps[temp_current]->shell_C <= 0 || temps[temp_current]->transfer <= 0 || temps[temp_current]->radiation <= 0)
#else
	if (true)
#endif
	{
		// No valid settings; use simple on/off-regime based on current temperature only.  Note that adc values are lower for higher temperatures.
		if (temp > temps[temp_current]->adctarget) {
			if (!temps[temp_current]->is_on) {
				//debug("switching on %d", temp_current);
				SET(temps[temp_current]->power_pin);
				temps[temp_current]->is_on = true;
				temps[temp_current]->last_time = current_time;
				++temps_busy;
			}
			else
				temps[temp_current]->time_on += current_time - temps[temp_current]->last_time;
		}
		else {
			if (temps[temp_current]->is_on) {
				//debug("switching off %d", temp_current);
				RESET(temps[temp_current]->power_pin);
				temps[temp_current]->is_on = false;
				temps[temp_current]->time_on += current_time - temps[temp_current]->last_time;
				--temps_busy;
			}
		}
		return;
	}
#ifndef LOWMEM
	// TODO: Make this work and decide on units.
	// We have model settings.
	unsigned long dt = current_time - temps[temp_current]->last_time;
	if (dt == 0)
		return;
	temps[temp_current]->last_time = current_time;
	// Heater and core/shell transfer.
	if (temps[temp_current]->is_on)
		temps[temp_current]->core_T += temps[temp_current]->power / temps[temp_current]->core_C * dt;
	int32_t Q = temps[temp_current]->transfer * (temps[temp_current]->core_T - temps[temp_current]->shell_T) * dt;
	temps[temp_current]->core_T -= Q / temps[temp_current]->core_C;
	temps[temp_current]->shell_T += Q / temps[temp_current]->shell_C;
	if (temps[temp_current]->is_on)
		temps[temp_current]->core_T += temps[temp_current]->power / temps[temp_current]->core_C * dt / 2;
	// Set shell to measured value.
	temps[temp_current]->shell_T = temp;
	// Add energy if required.
	int32_t E = temps[temp_current]->core_T * temps[temp_current]->core_C + temps[temp_current]->shell_T * temps[temp_current]->shell_C;
	int32_t T = E / (temps[temp_current]->core_C + temps[temp_current]->shell_C);
	// Set the pin to correct value.
	if (T < temps[temp_current]->target) {
		if (!temps[temp_current]->is_on) {
			SET(temps[temp_current]->power_pin);
			temps[temp_current]->is_on = true;
			++temps_busy;
		}
		else
			temps[temp_current]->time_on += current_time - temps[temp_current]->last_time;
	}
	else {
		if (temps[temp_current]->is_on) {
			RESET(temps[temp_current]->power_pin);
			temps[temp_current]->is_on = false;
			temps[temp_current]->time_on += current_time - temps[temp_current]->last_time;
			--temps_busy;
		}
	}
#endif
}
#endif

#if MAXAXES > 0 || MAXEXTRUDERS > 0
static bool delayed;

static void done_motors() {
	//debug("motors done");
	// Mark motors as not moving.
	for (uint8_t m = 0; m < MAXOBJECT; ++m) {
		if (!motors[m])
			continue;
		motors[m]->dist = MAXLONG;
	}
	//debug("moving->false");
	moving = false;
	next_move();
}

static bool do_steps(uint8_t m, int32_t distance, unsigned long current_time) {
	if (distance == 0)
		return true;
	int32_t dt = current_time - motors[m]->last_time;
	float v = distance / uf(dt);
	int32_t old_distance = distance;
	(void)&old_distance;
	// Limit v.
	if (abs(v) > motors[m]->limit_v) {
		delayed = true;
		v = (distance < 0 ? -1 : 1) * motors[m]->limit_v;
		distance = u(v * dt);
		movedebug("d1 %d %ld %ld", m, F(distance), F(old_distance));
	}
	// Limit a.
	float limit_dv = motors[m]->limit_a * uf(dt);
	if (abs(motors[m]->last_v - v) > limit_dv) {
		delayed = true;
		float old_v = v;
		v = motors[m]->last_v + (v < motors[m]->last_v ? -1 : 1) * limit_dv;
		if (old_v != 0 && motors[m]->last_v != 0 && (v < 0) ^ (motors[m]->last_v < 0)) {
			// Set v to 0 and record this as an event, to prevent it from happening again next iteration.
			movedebug("< %d %f", m, F(old_distance));
			motors[m]->last_v = 0;
			motors[m]->last_time = current_time;
			return true;
		}
		distance = u(v * dt);
		movedebug("d2 %d %ld %ld", m, F(distance), F(old_distance));
	}
	int32_t old_pos;
#if MAXAXES > 0
	if (m >= 2 && m < MAXAXES + 2) {
		old_pos = axis[m - 2].current_pos;
	}
#else
	// Dummy clause to make the else below work in case there are no axes.
	if (false) {
	}
#endif
#if MAXEXTRUDERS > 0
	else {
		old_pos = extruder[m - 2 - MAXAXES].distance_done;
	}
#endif
	int32_t new_pos = old_pos == MAXLONG ? MAXLONG : old_pos + distance;
	int16_t steps = m(m2(distance) * m2(motors[m]->steps_per_m));
	movedebug("steps %ld %ld %d %ld %ld", F(old_pos), F(new_pos), steps, F(distance), F(current_time));
	if (steps == 0) {
		movedebug("");
		return true;
	}
	// Limit steps per iteration.
	if (abs(steps) > motors[m]->max_steps) {
		delayed = true;
		steps = (steps < 0 ? -1 : 1) * motors[m]->max_steps;
		distance = M(steps) / motors[m]->steps_per_m;
		movedebug("d3 %d %ld %ld", m, F(distance), F(old_distance));
		new_pos = old_pos == MAXLONG ? MAXLONG : old_pos + distance;
		v = distance / uf(dt);
	}
	movedebug("action %d old %ld new %ld step %d", m, F(old_pos), F(new_pos), steps);
	// Set positive and direction pin.
	if (motors[m]->positive != (steps > 0)) {
		if (distance > 0)
			SET(motors[m]->dir_pin);
		else
			RESET(motors[m]->dir_pin);
		motors[m]->positive = (steps > 0);
	}
#if MAXAXES > 0
	if (m >= 2 && m < MAXAXES + 2) {
		// Check limit switches.
		int32_t cp = axis[m - 2].current_pos;
		if (motors[m]->positive ? GET(axis[m - 2].limit_max_pin, false) || (cp != MAXLONG && cp + distance > axis[m - 2].motor_max) : GET(axis[m - 2].limit_min_pin, false) || (cp != MAXLONG && cp + distance < axis[m - 2].motor_min)) {
			// Hit endstop; abort current move and notify host.
			debug("hit obj %d positive %d dist %ld current_pos %ld min %ld max %ld oldpos %ld newpos %ld last_v %f v %f", int(m - 2), motors[m]->positive, F(distance), F(axis[m - 2].current_pos), F(axis[m - 2].motor_min), F(axis[m - 2].motor_max), F(old_pos), F(new_pos), F(motors[m]->last_v), F(v));
			// Stop continuous move only for the motor that hits the switch.
			motors[m]->f = 0;
			motors[m]->last_v = 0;
			motors[m]->continuous_v = 0;
			limits_pos[m - 2] = axis[m - 2].current_pos;
			if (current_move_has_cb) {
				//debug("movecb 3");
				++num_movecbs;
			}
			try_send_next();
			//debug("aborting for limit");
			abort_move();
			next_move();
			return false;
		}
		axis[m - 2].current_pos = new_pos;
	}
#else
	// Dummy clause to make the else below work in case there are no axes.
	if (false) {
	}
#endif
#if MAXEXTRUDERS > 0
	else {
		extruder[m - 2 - MAXAXES].distance_done = new_pos;
	}
#endif
	// Record this iteration.
	motors[m]->last_v = v;
	motors[m]->last_time = current_time;
	// Move.
        for (int16_t s = 0; s < abs(steps); ++s) {
		SET(motors[m]->step_pin);
		RESET(motors[m]->step_pin);
	}
	return true;
}

#if MAXAXES > 0
static bool move_axes(int32_t target[3], unsigned long current_time) {
	uint8_t a = 0;
	switch (printer_type) {
#if MAXAXES >= 3
	case 1:
	{
		// Delta movements are hardcoded to 3 axes.
		if (target[0] == MAXLONG || target[1] == MAXLONG || target[2] == MAXLONG) {
			// Fill up missing targets.
			for (uint8_t aa = 0; aa < 3; ++aa) {
				if (target[aa] == MAXLONG)
					target[aa] = axis[aa].current;
			}
		}
		int32_t the_target[3];
		bool ok = true;
		for (a = 0; a < 3; ++a) {
			the_target[a] = delta_to_axis(a, target, &ok);
		}
		if (!ok) {
			for (a = 0; a < 3; ++a)
				the_target[a] = delta_to_axis(a, target, &ok);
		}
		for (a = 0; a < 3; ++a) {
			if (!do_steps(2 + a, the_target[a] - axis[a].current_pos, current_time)) {
				// The move has been aborted.
				return false;
			}
		}
		if (!delayed) {
			for (a = 0; a < 3; ++a) {
				//debug("setting axis %d current to %f", a, F(target[a]));
				axis[a].current = target[a];
			}
		}
		else
			a = 3;
		// Fall through to handle the non-delta axes.
	}
#endif
	case 0:
		for (; a < num_axes; ++a)
			if (target[a] != MAXLONG) {
				movedebug("moveaxis %d %ld", a, F(target[a]));
				if (!do_steps(2 + a, target[a] - axis[a].current_pos, current_time))
					return false;
				if (!delayed)
					axis[a].current = target[a];
			}
		break;
	default:
		debug("Bug: printer_type %d not handled by move in " __FILE__, printer_type);
		break;
	}
	return true;
}
#endif

static void handle_motors(unsigned long current_time, unsigned long longtime) {
#if MAXAXES > 0
	for (uint8_t a = 0; a < MAXAXES; ++a) {	// Check sense pins.
		if (axis[a].current_pos != MAXLONG && GET(axis[a].sense_pin, false) ^ bool(axis[a].sense_state & 0x80)) {
			axis[a].sense_state ^= 0x80;
			axis[a].sense_state |= 1;
			axis[a].sense_pos = axis[a].current_pos;
			try_send_next();
		}
	}
#endif
	// Check for continuous moves.
	for (uint8_t m = 0; m < MAXOBJECT; ++m) {
		if (!motors[m] || motors[m]->dist != MAXLONG)
			continue;
		if (motors[m]->continuous_v == 0 && motors[m]->f == 0)
			continue;
		movedebug("Continuous %d", m);
		last_active = longtime;
		int32_t current_t = current_time - motors[m]->last_time;
		if (motors[m]->continuous_v != motors[m]->f) {
			// Getting up to speed, or slowing down.
			//debug("speed change from %ld to %ld (%ld %ld)", F(motors[m]->f), F(motors[m]->continuous_v), F(current_t), F(motors[m]->limit_a));
			if (motors[m]->continuous_v > motors[m]->f) {
				motors[m]->f += current_t * m(motors[m]->limit_a);
				if (motors[m]->continuous_v < motors[m]->f)
					motors[m]->f = motors[m]->continuous_v;
			}
			else {
				motors[m]->f -= current_t * m(motors[m]->limit_a);
				if (motors[m]->f < 0 || (motors[m]->f == 0 && motors[m]->continuous_v < 0)) {
					motors[m]->f = -motors[m]->f;
					motors[m]->continuous_v = -motors[m]->continuous_v;
					if (motors[m]->positive) {
						motors[m]->positive = false;
						RESET(motors[m]->dir_pin);
					}
					else {
						motors[m]->positive = true;
						SET(motors[m]->dir_pin);
					}
					//debug("new positive %d", motors[m]->positive);
					if (motors[m]->continuous_v < motors[m]->f)
						motors[m]->f = motors[m]->continuous_v;
				}
				else if (motors[m]->continuous_v > motors[m]->f)
					motors[m]->f = motors[m]->continuous_v;
			}
		}
		motors[m]->continuous_f += m(m(motors[m]->f) * current_t);
		int16_t steps(m(m2(motors[m]->continuous_f) * m2(motors[m]->steps_per_m)));
		//debug("speed %ld t %ld cf %ld step %d", F(motors[m]->f), F(current_t), F(motors[m]->continuous_f), steps);
		motors[m]->continuous_f -= M(steps) / motors[m]->steps_per_m;
		delayed = false;
		do_steps(m, M(steps) * (motors[m]->positive ? 1 : -1) / motors[m]->steps_per_m, current_time);
		if (delayed || steps)
			motors[m]->last_time = current_time;
		continue;
	}
	// Check for regular move.
	if (!moving)
		return;
	last_active = longtime;
	long t;
	delayed = false;
	if (freeze_time >= 0)
		t = freeze_time;
	else
		t = current_time - start_time;
#if MAXAXES > 0
	int32_t target[num_axes];
#endif
	delayed = false;
	if (t >= t0 + tp) {	// Finish this move and prepare next.
		movedebug("finishing %ld", F(t));
#if MAXAXES > 0
		for (uint8_t a = 0; a < num_axes; ++a) {
			if ((axis[a].motor.dist == MAXLONG || axis[a].motor.dist == 0) && (axis[a].motor.next_dist == MAXLONG || axis[a].motor.next_dist == 0)) {
				target[a] = MAXLONG;
				continue;
			}
			int32_t dist = axis[a].motor.dist == MAXLONG ? 0 : axis[a].motor.dist;
			int32_t next_dist = axis[a].motor.next_dist == MAXLONG ? 0 : axis[a].motor.next_dist;
			target[a] = axis[a].source + dist + m(m2(next_dist) * m2(u(fq)));
		}
		if (!move_axes(target, current_time))
			return;
#endif
#if MAXEXTRUDERS > 0
		for (uint8_t e = 0; e < num_extruders; ++e) {
			if (extruder[e].motor.dist == MAXLONG)
				continue;
			int32_t dist = extruder[e].motor.dist == MAXLONG ? 0 : extruder[e].motor.dist;
			int32_t next_dist = extruder[e].motor.next_dist == MAXLONG ? 0 : extruder[e].motor.next_dist;
			int32_t um = dist + m(m2(next_dist) * m2(u(fq)));
			if (!do_steps(2 + MAXAXES + e, um - extruder[e].distance_done, current_time))
				return;
		}
#endif
		if (delayed) // Not really done yet.
			return;
		if (moving && current_move_has_cb) {
			//debug("movecb 1");
			++num_movecbs;
			try_send_next();
		}
#if MAXAXES > 0
		for (uint8_t a = 0; a < num_axes; ++a)
			if (axis[a].motor.dist != MAXLONG) {
				//debug("before source %d %f %f", a, F(axis[a].source), F(axis[a].motor.dist));
				axis[a].source += axis[a].motor.dist;
				//debug("after source %d %f %f %f", a, F(axis[a].source), F(axis[a].motor.dist), F(axis[a].current_pos));
			}
#endif
		done_motors();
		return;
	}
	if (t < t0) {	// Main part.
		int32_t t_fraction = k2(t) / m2(t0);
		int32_t current_f = m2(m(u(f1) * (k(2) - t_fraction) + u(f2) * t_fraction) * t_fraction);
		movedebug("main t %ld t0 %ld tp %ld tfrac %ld f1 %ld f2 %ld cf %ld", F(t), F(t0), F(tp), F(t_fraction), F(f1), F(f2), F(current_f));
#if MAXAXES > 0
		for (uint8_t a = 0; a < num_axes; ++a) {
			if (axis[a].motor.dist == MAXLONG || axis[a].motor.dist == 0) {
				target[a] = MAXLONG;
				continue;
			}
			target[a] = axis[a].source + m(m2(axis[a].motor.dist) * current_f);
		}
#endif
#if MAXEXTRUDERS > 0
		for (uint8_t e = 0; e < num_extruders; ++e) {
			if (extruder[e].motor.dist == MAXLONG || extruder[e].motor.dist == 0)
				continue;
			int32_t dist = m(m2(extruder[e].motor.dist) * current_f);
			do_steps(2 + MAXAXES + e, dist - extruder[e].distance_done, current_time);
		}
#endif
	}
	else {	// Connector part.
		movedebug("connector %ld %ld %ld", F(t), F(t0), F(tp));
		long tc = t - t0;
#if MAXAXES > 0
		for (uint8_t a = 0; a < num_axes; ++a) {
			if ((axis[a].motor.dist == MAXLONG || axis[a].motor.dist == 0) && (axis[a].motor.next_dist == MAXLONG || axis[a].motor.next_dist == 0)) {
				target[a] = MAXLONG;
				continue;
			}
			int32_t dist = axis[a].motor.dist == MAXLONG ? 0 : axis[a].motor.dist;
			int32_t next_dist = axis[a].motor.next_dist == MAXLONG ? 0 : axis[a].motor.next_dist;
			int32_t t_fraction = k(tc) / m(tp);
			int32_t current_f2 = m2(m(m(u(fp) * m(M(2) - t_fraction)) * t_fraction));
			int32_t current_f3 = m2(m(m(u(fq) * m(t_fraction)) * t_fraction));
			target[a] = axis[a].source + axis[a].motor.main_dist + m2(dist) * current_f2 + m2(next_dist) * current_f3;
		}
#endif
#if MAXEXTRUDERS > 0
		for (uint8_t e = 0; e < num_extruders; ++e) {
			if (extruder[e].motor.dist == MAXLONG && extruder[e].motor.next_dist == MAXLONG)
				continue;
			int32_t dist = extruder[e].motor.dist == MAXLONG ? 0 : extruder[e].motor.dist;
			int32_t next_dist = extruder[e].motor.next_dist == MAXLONG ? 0 : extruder[e].motor.next_dist;
			int32_t t_fraction = k(tc) / m(tp);
			int32_t current_f2 = m2(m(m(u(fp) * m(M(2) - t_fraction)) * t_fraction));
			int32_t current_f3 = m2(m(m(u(fq) * m(t_fraction)) * t_fraction));
			int32_t distance = extruder[e].motor.main_dist + m2(dist) * current_f2 + m2(next_dist) * current_f3;
			do_steps(2 + MAXAXES + e, distance - extruder[e].distance_done, current_time);
		}
#endif
	}
#if MAXAXES > 0
	move_axes(target, current_time);
#endif
	if (delayed) {
		if (freeze_time < 0) {
			//debug("freeze normal");
			freeze_time = t;
		}
	}
	else if (freeze_time >= 0) {
		// freeze_time = current_time_0 - start_time_0
		// start_time_1 = current_time_1 - freeze_time
		// = current_time_1 - current_time_0 + start_time_0
		// => start_time += current_time_1 - current_time_0
		start_time = current_time - freeze_time;
		freeze_time = -1;
		//debug("unfreeze");
	}
}
#endif

#ifdef AUDIO
static void handle_audio(unsigned long current_time, unsigned long longtime) {
	if (audio_head != audio_tail) {
		last_active = longtime;
		int16_t bit = (current_time - audio_start) / audio_us_per_bit;
		int16_t byte = bit >> 3;
		while (byte >= AUDIO_FRAGMENT_SIZE) {
			//debug("next audio fragment");
			if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
			{
				//debug("continue audio");
				continue_cb |= 2;
				try_send_next();
			}
			audio_head = (audio_head + 1) % AUDIO_FRAGMENTS;
			if (audio_tail == audio_head) {
				//debug("audio done");
				return;
			}
			byte -= AUDIO_FRAGMENT_SIZE;
			// us per fragment = us/bit*bit/fragment
			audio_start += audio_us_per_bit * 8 * AUDIO_FRAGMENT_SIZE;
		}
		uint8_t old_state = audio_state;
		audio_state = (audio_buffer[audio_head][byte] >> (bit & 7)) & 1;
		if (audio_state != old_state) {
			for (uint8_t m = 0; m < MAXOBJECT; ++m) {
				if (!motors[m] || !(motors[m]->audio_flags & Motor::PLAYING))
					continue;
				if (audio_state) {
					if (!motors[m]->positive) {
						SET(motors[m]->dir_pin);
						motors[m]->positive = true;
					}
				}
				else {
					if (motors[m]->positive) {
						RESET(motors[m]->dir_pin);
						motors[m]->positive = false;
					}
				}
				SET(motors[m]->step_pin);
				RESET(motors[m]->step_pin);
			}
		}
	}
}
#endif

static void handle_led(unsigned long current_time) {
	unsigned timing = temps_busy > 0 ? 1000 / 100 : 1000 / 50;
	if (current_time - led_last < timing)
		return;
	led_last += timing;
	led_phase += 1;
	led_phase %= 50;
	// Timings read from https://en.wikipedia.org/wiki/File:Wiggers_Diagram.png (phonocardiogram).
	bool state = (led_phase <= 4 || (led_phase >= 14 && led_phase <= 17));
	if (state)
		SET(led_pin);
	else
		RESET(led_pin);
}

void loop() {
	serial();
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0 || MAXAXES > 0 || defined(AUDIO)
	unsigned long current_time = micros();
#endif
	unsigned long longtime = millis();
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
	handle_temps(current_time, longtime);	// Periodic temps stuff: temperature regulation.
#endif
#if MAXAXES > 0 || MAXEXTRUDERS > 0
	handle_motors(current_time, longtime);	// Movement.
#endif
	handle_led(longtime);	// heart beat.
#ifdef AUDIO
	handle_audio(current_time, longtime);
#endif
#if MAXAXES > 0 || MAXEXTRUDERS > 0
	if (motors_busy != 0 && motor_limit > 0 && longtime - last_active > motor_limit) {
		for (uint8_t m = 0; m < MAXOBJECT; ++m) {
			if (!motors[m])
				continue;
			RESET(motors[m]->enable_pin);
		}
#if MAXAXES > 0
		for (uint8_t a = 0; a < MAXAXES; ++a) {
			axis[a].current_pos = MAXLONG;
			axis[a].current = MAXLONG;
			axis[a].source = MAXLONG;
		}
#endif
		motors_busy = 0;
		which_autosleep |= 1;
	}
#endif
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
	if (temps_busy > 0 && temp_limit > 0 && longtime - last_active > temp_limit) {
		for (uint8_t current_t = 0; current_t < MAXOBJECT; ++current_t) {
			if (!temps[current_t])
				continue;
			RESET(temps[current_t]->power_pin);
			temps[current_t]->target = MAXLONG;
			temps[current_t]->adctarget = MAXINT;
			temps[current_t]->is_on = false;
			last_active = longtime;
		}
		temps_busy = 0;
		which_autosleep |= 2;
	}
#endif
	if (which_autosleep != 0)
		try_send_next();
}

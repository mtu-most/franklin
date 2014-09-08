// vim: set filetype=cpp foldmethod=marker foldmarker={,} :
#define EXTERN	// This must be done in exactly one source file.
#include "firmware.h"

#if 1
#define movedebug(...) do {} while (0)
#else
#define movedebug(...) debug(__VA_ARGS__)
#endif

//#define TIMING

// Loop function handles all regular updates.

#ifdef HAVE_TEMPS
static void handle_temps(unsigned long current_time, unsigned long longtime) {
	static int sleeper = 0;
	if (sleeper > 0 && requested_temp >= num_temps) {
		sleeper -= 1;
		return;
	}
	sleeper = 1000;
	if (adc_phase == 0)
		return;
	if (adc_phase == 1) {
		if (requested_temp < num_temps)
			temp_current = requested_temp;
		else {
			// Find the temp to measure next time.
			uint8_t i;
			for (i = 1; i <= num_temps; ++i) {
				uint8_t next = (temp_current + i) % num_temps;
				if (((temps[next].adctarget < MAXINT && temps[next].adctarget >= 0)
						 || (temps[next].adcmin_alarm >= 0 && temps[next].adcmin_alarm < MAXINT)
						 || temps[next].adcmax_alarm < MAXINT
#ifdef HAVE_GPIOS
						 || temps[next].gpios
#endif
						 )
						&& temps[next].thermistor_pin.valid()) {
					temp_current = next;
					break;
				}
			}
			// If there is no temperature handling to do; disable (and abort the current one as well; it is no longer needed).
			if (i > num_temps) {
				adc_phase = 0;
				return;
			}
		}
		adc_start(temps[temp_current].thermistor_pin.pin);
		adc_phase = 2;
		return;
	}
	int16_t temp = temps[temp_current].get_value();
	if (temp < 0) {	// Not done yet.
		return;
	}
	//debug("done temperature %d %d", temp_current, temp);
	if (requested_temp == temp_current) {
		//debug("replying temp");
		requested_temp = ~0;
		ReadFloat f;
		f.f = temps[temp_current].fromadc(temp);
		//debug("read temp %f", F(f.f));
		reply[0] = 2 + sizeof(float);
		reply[1] = CMD_TEMP;
		for (uint8_t b = 0; b < sizeof(float); ++b)
			reply[2 + b] = f.b[b];
		reply_ready = true;
		try_send_next();
	}
	//debug("temp for %d: %d", temp_current, temp);
	// Set the phase so next time another temp is measured.
	adc_phase = 1;
	// First of all, if an alarm should be triggered, do so.  Adc values are higher for lower temperatures.
	if ((temps[temp_current].adcmin_alarm < MAXINT && temps[temp_current].adcmin_alarm > temp) || temps[temp_current].adcmax_alarm < temp) {
		temps[temp_current].min_alarm = NAN;
		temps[temp_current].max_alarm = NAN;
		temps[temp_current].adcmin_alarm = MAXINT;
		temps[temp_current].adcmax_alarm = MAXINT;
		temps[temp_current].alarm = true;
		try_send_next();
	}
#ifdef HAVE_GPIOS
	// And handle any linked gpios.
	for (Gpio *g = temps[temp_current].gpios; g; g = g->next) {
		//debug("setting gpio for temp %d: %d %d", temp_current, temp, g->adcvalue);
		// adc values are lower for higher temperatures.
		if (temp < g->adcvalue)
			SET(g->pin);
		else
			RESET(g->pin);
	}
#endif
	// If we don't have model settings, simply use the target as a switch between on and off.
	/* Don't use those values yet.
	if (true || temps[temp_current].core_C <= 0 || temps[temp_current].shell_C <= 0 || temps[temp_current].transfer <= 0 || temps[temp_current].radiation <= 0)
	*/
	{
		// No valid settings; use simple on/off-regime based on current temperature only.  Note that adc values are lower for higher temperatures.
		if (temp > temps[temp_current].adctarget) {
			if (!temps[temp_current].is_on) {
				//debug("switching on %d", temp_current);
				SET(temps[temp_current].power_pin);
				temps[temp_current].is_on = true;
				temps[temp_current].last_time = current_time;
				++temps_busy;
			}
			else
				temps[temp_current].time_on += current_time - temps[temp_current].last_time;
		}
		else {
			if (temps[temp_current].is_on) {
				//debug("switching off %d", temp_current);
				RESET(temps[temp_current].power_pin);
				temps[temp_current].is_on = false;
				temps[temp_current].time_on += current_time - temps[temp_current].last_time;
				--temps_busy;
			}
		}
		return;
	}
	/*
	// TODO: Make this work and decide on units.
	// We have model settings.
	unsigned long dt = current_time - temps[temp_current].last_time;
	if (dt == 0)
		return;
	temps[temp_current].last_time = current_time;
	// Heater and core/shell transfer.
	if (temps[temp_current].is_on)
		temps[temp_current].core_T += temps[temp_current].power / temps[temp_current].core_C * dt;
	float Q = temps[temp_current].transfer * (temps[temp_current].core_T - temps[temp_current].shell_T) * dt;
	temps[temp_current].core_T -= Q / temps[temp_current].core_C;
	temps[temp_current].shell_T += Q / temps[temp_current].shell_C;
	if (temps[temp_current].is_on)
		temps[temp_current].core_T += temps[temp_current].power / temps[temp_current].core_C * dt / 2;
	// Set shell to measured value.
	temps[temp_current].shell_T = temp;
	// Add energy if required.
	float E = temps[temp_current].core_T * temps[temp_current].core_C + temps[temp_current].shell_T * temps[temp_current].shell_C;
	float T = E / (temps[temp_current].core_C + temps[temp_current].shell_C);
	// Set the pin to correct value.
	if (T < temps[temp_current].target) {
		if (!temps[temp_current].is_on) {
			SET(temps[temp_current].power_pin);
			temps[temp_current].is_on = true;
			++temps_busy;
		}
		else
			temps[temp_current].time_on += current_time - temps[temp_current].last_time;
	}
	else {
		if (temps[temp_current].is_on) {
			RESET(temps[temp_current].power_pin);
			temps[temp_current].is_on = false;
			temps[temp_current].time_on += current_time - temps[temp_current].last_time;
			--temps_busy;
		}
	}
	*/
}
#endif

#ifdef HAVE_SPACES
static bool delayed;

static void check_distance(Motor *mtr, float distance, float dt, float &factor) {
	if (isnan(distance) || distance == 0) {
		mtr->target_dist = 0;
		return;
	}
	mtr->target_dist = distance;
	mtr->target_v = distance / dt;
	float v = abs(distance / dt);
	int8_t s = (mtr->target_v < 0 ? -1 : 1);
	// When turning around, ignore limits (they shouldn't have been violated anyway).
	if (mtr->last_v * s < 0)
		mtr->last_v = 0;
	// Limit a-.
	float max_dist = (mtr->endpos - mtr->current_pos) * s;
	if (max_dist > 0 && v * v / 2 / mtr->limit_a > max_dist) {
		//debug("a- %f %f %f %f %d", F(mtr->endpos), F(mtr->limit_a), F(max_dist), F(mtr->current_pos), s);
		v = sqrt(max_dist * 2 * mtr->limit_a);
		distance = s * v * dt;
	}
	// Limit a+.
	float limit_dv = mtr->limit_a * dt;
	if (v - mtr->last_v * s > limit_dv) {
		//debug("a+ %f %f %f %d", F(mtr->target_v), F(limit_dv), F(mtr->last_v), s);
		distance = (limit_dv * s + mtr->last_v) * dt;
		v = abs(distance / dt);
	}
	// Limit v.
	if (v > mtr->limit_v) {
		//debug("v");
		distance = (s * mtr->limit_v) * dt;
		v = abs(distance / dt);
	}
	int16_t steps;
	if (!isnan(mtr->current_pos))
		steps = int32_t((mtr->current_pos + distance) * mtr->steps_per_m) - int32_t(mtr->current_pos * mtr->steps_per_m);
	else
		steps = distance * mtr->steps_per_m;
	// Limit steps per iteration.
	if (abs(steps) > mtr->max_steps) {
		//debug("s %d", steps);
		distance = s * (mtr->max_steps + .1) / mtr->steps_per_m;
	}
	float f = distance / mtr->target_dist;
	//debug("%f %f", F(mtr->target_dist), F(distance));
	if (f < factor)
		factor = f;
}

static void move_axes(Space *s, unsigned long current_time, float &factor) {
	float motors_target[s->num_motors];
	bool ok = true;
	space_types[s->type].xyz2motors(s, motors_target, &ok);
	// Try again if it didn't work; it should have moved target to a better location.
	if (!ok)
		space_types[s->type].xyz2motors(s, motors_target, &ok);
	movedebug("ok %d", ok);
	for (uint8_t m = 0; m < s->num_motors; ++m) {
		//movedebug("move %d %f %f %f", m, F(target[m]), F(motors_target[m]), F(s->motor[m]->current_pos));
		check_distance(s->motor[m], motors_target[m] - s->motor[m]->current_pos, (current_time - last_time) / 1e6, factor);
	}
}

static bool do_steps(float factor, unsigned long current_time) {
	if (factor == 0)
		return true;
	int16_t max_steps = 0;
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		if (!sp.active)
			continue;
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			float target = mtr.current_pos + mtr.target_dist * factor;
			mtr.steps = int32_t(target * mtr.steps_per_m) - int32_t(mtr.current_pos * mtr.steps_per_m);
			if (abs(mtr.steps) > max_steps)
				max_steps = abs(mtr.steps);
		}
	}
	//debug("%f %d", F(factor), max_steps);
	if (max_steps == 0)
		return true;
	if (factor < 1)
		start_time += (current_time - last_time) * (1 - factor);
	last_time = current_time;
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		if (!sp.active)
			continue;
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			// Check limit switches.
			float target = mtr.current_pos + mtr.target_dist * factor;
			if (mtr.steps == 0) {
				mtr.last_v = mtr.target_v * factor;
				mtr.current_pos = target;
				continue;
			}
			float cp = mtr.current_pos;
			if (mtr.steps > 0 ? GET(mtr.limit_max_pin, false) || cp + mtr.target_dist * factor > mtr.motor_max : GET(mtr.limit_min_pin, false) || cp + mtr.target_dist * factor < mtr.motor_min) {
				// Hit endstop; abort current move and notify host.
				debug("hit limit %d %d %d", s, m, mtr.target_dist > 0);
				mtr.last_v = 0;
				mtr.limits_pos = isnan(mtr.current_pos) ? INFINITY * (mtr.target_dist > 0 ? 1 : -1) : mtr.current_pos;
				if (moving && current_move_has_cb) {
					//debug("movecb 3");
					++num_movecbs;
				}
				try_send_next();
				//debug("aborting for limit");
				abort_move();
				next_move();
				return false;
			}
			mtr.last_v = mtr.target_v * factor;
			//debug("%f %f %d", F(mtr.current_pos), F(target), mtr.steps);
			mtr.current_pos = target;
			// Set direction pin.
			if (mtr.steps > 0)
				SET(mtr.dir_pin);
			else
				RESET(mtr.dir_pin);
			// Move.
			for (int16_t st = 0; st < abs(mtr.steps); ++st) {
				SET(mtr.step_pin);
				RESET(mtr.step_pin);
			}
		}
	}
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		if (!sp.active)
			continue;
		for (uint8_t a = 0; a < sp.num_axes; ++a)
			sp.axis[a]->current += (sp.axis[a]->target - sp.axis[a]->current) * factor;
	}
	return true;
}


static void handle_motors(unsigned long current_time, unsigned long longtime) {
	// Check sense pins.
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			if (!sp.motor[m]->sense_pin.valid())
				continue;
			if (!isnan(sp.motor[m]->current_pos) && GET(sp.motor[m]->sense_pin, false) ^ bool(sp.motor[m]->sense_state & 0x80)) {
				sp.motor[m]->sense_state ^= 0x80;
				sp.motor[m]->sense_state |= 1;
				sp.motor[m]->sense_pos = sp.motor[m]->current_pos;
				try_send_next();
			}
		}
	}
	// Check for move.
	if (!moving)
		return;
	last_active = longtime;
	float t = (current_time - start_time) / 1e6;
	delayed = false;
	float factor = 1;
	if (t >= t0 + tp) {	// Finish this move and prepare next.
		movedebug("finishing %f", F(t));
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			if (!sp.active)
				continue;
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if ((isnan(sp.axis[a]->dist) || sp.axis[a]->dist == 0) && (isnan(sp.axis[a]->next_dist) || sp.axis[a]->next_dist == 0)) {
					sp.axis[a]->target = NAN;
					continue;
				}
				sp.axis[a]->target = sp.axis[a]->source + sp.axis[a]->dist + sp.axis[a]->next_dist * fq;
			}
			move_axes(&sp, current_time, factor);
		}
		if (!do_steps(factor, current_time))
			return;
		if (factor < 1) // Not really done yet.
			return;
		if (moving && current_move_has_cb) {
			//debug("movecb 1");
			++num_movecbs;
			try_send_next();
		}
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				//debug("before source %d %f %f", a, F(axis[a].source), F(axis[a].motor.dist));
				sp.axis[a]->source += sp.axis[a]->dist;
				//debug("after source %d %f %f %f", a, F(axis[a].source), F(axis[a].motor.dist), F(axis[a].current_pos));
			}
			// Mark motors as not moving.
			for (uint8_t a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->dist = NAN;
		}
		//debug("motors done");
		//debug("moving->false");
		moving = false;
		next_move();
		return;
	}
	if (t < t0) {	// Main part.
		float t_fraction = t / t0;
		float current_f = (f1 * (2 - t_fraction) + f2 * t_fraction) * t_fraction;
		movedebug("main t %f t0 %f tp %f tfrac %f f1 %f f2 %f cf %f", F(t), F(t0), F(tp), F(t_fraction), F(f1), F(f2), F(current_f));
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			movedebug("try %d %d", s, sp.active);
			if (!sp.active)
				continue;
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if (isnan(sp.axis[a]->dist) || sp.axis[a]->dist == 0) {
					sp.axis[a]->target = NAN;
					continue;
				}
				sp.axis[a]->target = sp.axis[a]->source + sp.axis[a]->dist * current_f;
				//movedebug("do %d %d %f %f", s, a, F(sp.axis[a]->dist), F(target[a]));
			}
			move_axes(&sp, current_time, factor);
		}
	}
	else {	// Connector part.
		movedebug("connector %f %f %f", F(t), F(t0), F(tp));
		float tc = t - t0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			if (!sp.active)
				continue;
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if ((isnan(sp.axis[a]->dist) || sp.axis[a]->dist == 0) && (isnan(sp.axis[a]->next_dist) || sp.axis[a]->next_dist == 0)) {
					sp.axis[a]->target = NAN;
					continue;
				}
				float t_fraction = tc / tp;
				float current_f2 = fp * (2 - t_fraction) * t_fraction;
				float current_f3 = fq * t_fraction * t_fraction;
				sp.axis[a]->target = sp.axis[a]->source + sp.axis[a]->main_dist + sp.axis[a]->dist * current_f2 + sp.axis[a]->next_dist * current_f3;
			}
			move_axes(&sp, current_time, factor);
		}
	}
	do_steps(factor, current_time);
}
#endif

#ifdef HAVE_AUDIO
static void handle_audio(unsigned long current_time, unsigned long longtime) {
	if (audio_head != audio_tail) {
		last_active = longtime;
		int16_t bit = (current_time - audio_start) / audio_us_per_bit;
		int16_t audio_byte = bit >> 3;
		while (audio_byte >= AUDIO_FRAGMENT_SIZE) {
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
			audio_byte -= AUDIO_FRAGMENT_SIZE;
			// us per fragment = us/bit*bit/fragment
			audio_start += audio_us_per_bit * 8 * AUDIO_FRAGMENT_SIZE;
		}
		uint8_t old_state = audio_state;
		audio_state = (audio_buffer[audio_head][audio_byte] >> (bit & 7)) & 1;
		if (audio_state != old_state) {
			for (uint8_t s = 0; s < num_spaces; ++s) {
				Space &sp = spaces[s];
				for (uint8_t m = 0; m < sp.num_motors; ++m) {
					if (!(sp.motor[m]->audio_flags & Motor::PLAYING))
						continue;
					if (audio_state)
						SET(sp.motor[m]->dir_pin);
					else
						RESET(sp.motor[m]->dir_pin);
					SET(sp.motor[m]->step_pin);
					RESET(sp.motor[m]->step_pin);
				}
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
#ifdef TIMING
	unsigned long first_t = micros();
#endif
	serial();
#ifdef TIMING
	unsigned long serial_t = micros() - first_t;
#endif
#if defined(HAVE_TEMPS) || defined(HAVE_SPACES) || defined(HAVE_AUDIO)
	unsigned long current_time = micros();
#endif
	unsigned long longtime = millis();
#ifdef HAVE_TEMPS
	handle_temps(current_time, longtime);	// Periodic temps stuff: temperature regulation.
#endif
#ifdef TIMING
	unsigned long temp_t = micros() - current_time;
#endif
#ifdef HAVE_SPACES
	handle_motors(current_time, longtime);	// Movement.
#endif
#ifdef TIMING
	unsigned long motor_t = micros() - current_time;
#endif
	handle_led(longtime);	// heart beat.
#ifdef TIMING
	unsigned long led_t = micros() - current_time;
#endif
#ifdef HAVE_AUDIO
	handle_audio(current_time, longtime);
#endif
#ifdef TIMING
	unsigned long audio_t = micros() - current_time;
#endif
#ifdef HAVE_SPACES
	if (motors_busy && (longtime - last_active) / 1e3 > motor_limit) {
		debug("motor timeout");
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			for (uint8_t m = 0; m < sp.num_motors; ++m) {
				RESET(sp.motor[m]->enable_pin);
				sp.motor[m]->current_pos = NAN;
			}
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				sp.axis[a]->current = NAN;
				sp.axis[a]->source = NAN;
			}
		}
		motors_busy = false;
		which_autosleep |= 1;
	}
#endif
#ifdef HAVE_TEMPS
	if (temps_busy > 0 && (longtime - last_active) / 1e3 > temp_limit) {
		for (uint8_t current_t = 0; current_t < num_temps; ++current_t) {
			RESET(temps[current_t].power_pin);
			temps[current_t].target = NAN;
			temps[current_t].adctarget = MAXINT;
			temps[current_t].is_on = false;
			last_active = longtime;
		}
		temps_busy = 0;
		which_autosleep |= 2;
	}
#endif
	if (which_autosleep != 0)
		try_send_next();
#ifdef TIMING
	unsigned long end_t = micros() - current_time;
	end_t -= audio_t;
	audio_t -= led_t;
	led_t -= motor_t;
	motor_t -= temp_t;
	static int waiter = 0;
	if (waiter > 0)
		waiter -= 1;
	else {
		waiter = 977;
		debug("t: serial %ld temp %ld motor %ld led %ld audio %ld end %ld", F(serial_t), F(temp_t), F(motor_t), F(led_t), F(audio_t), F(end_t));
	}
#endif
}

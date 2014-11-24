// vim: set foldmethod=marker :

// Includes and debugging. {{{
#include "cdriver.h"

#if 0
#define movedebug(...) debug(__VA_ARGS__)
#else
#define movedebug(...) do {} while (0)
#endif

//#define TIMING
// }}}

static void handle_temps(uint32_t current_time, uint32_t longtime) { // {{{
	if (next_temp_time > 0)
		return;
	if (adc_phase == 0) {
		next_temp_time = ~0;
		return;
	}
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
						 || temps[next].following_gpios < num_gpios
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
	int32_t temp = temps[temp_current].get_value();
	if (temp < 0) {	// Not done yet.
		return;
	}
	next_temp_time = 100000;
	//debug("done temperature %d %d", temp_current, temp);
	if (requested_temp == temp_current) {
		//debug("replying temp");
		requested_temp = ~0;
		float result = temps[temp_current].fromadc(temp);
		send_host(CMD_TEMP, 0, 0, result);
	}
	//debug("temp for %d: %d", temp_current, temp);
	// Set the phase so next time another temp is measured.
	adc_phase = 1;
	// First of all, if an alarm should be triggered, do so.  Adc values are higher for lower temperatures.
	//debug("alarms: %d %d %d %d", temp_current, temps[temp_current].adcmin_alarm, temps[temp_current].adcmax_alarm, temp);
	if ((temps[temp_current].adcmin_alarm < MAXINT && temps[temp_current].adcmin_alarm >= temp) || temps[temp_current].adcmax_alarm <= temp) {
		temps[temp_current].min_alarm = NAN;
		temps[temp_current].max_alarm = NAN;
		temps[temp_current].adcmin_alarm = MAXINT;
		temps[temp_current].adcmax_alarm = MAXINT;
		send_host(CMD_TEMPCB, temp_current);
	}
	// And handle any linked gpios.
	for (uint8_t g = temps[temp_current].following_gpios; g < num_gpios; g = gpios[g].next) {
		//debug("setting gpio for temp %d: %d %d", temp_current, temp, g->adcvalue);
		// adc values are lower for higher temperatures.
		if (temp < gpios[g].adcvalue)
			SET(gpios[g].pin);
		else
			RESET(gpios[g].pin);
	}
	// If we don't have model settings, simply use the target as a switch between on and off.
	/* Don't use those values yet.
	if (true || temps[temp_current].core_C <= 0 || temps[temp_current].shell_C <= 0 || temps[temp_current].transfer <= 0 || temps[temp_current].radiation <= 0)
	*/
	{
		// No valid settings; use simple on/off-regime based on current temperature only.  Note that adc values are lower for higher temperatures.
		if (temp > temps[temp_current].adctarget) {
			if (!temps[temp_current].is_on) {
				//debug("switching on %d (%d > %d)", temp_current, temp, temps[temp_current].adctarget);
				SET(temps[temp_current].power_pin);
				temps[temp_current].is_on = true;
				temps[temp_current].last_temp_time = current_time;
				++temps_busy;
			}
			else
				temps[temp_current].time_on += current_time - temps[temp_current].last_temp_time;
		}
		else {
			if (temps[temp_current].is_on) {
				//debug("switching off %d (%d <= %d)", temp_current, temp, temps[temp_current].adctarget);
				RESET(temps[temp_current].power_pin);
				temps[temp_current].is_on = false;
				temps[temp_current].time_on += current_time - temps[temp_current].last_temp_time;
				--temps_busy;
			}
		}
		return;
	}
	/*
	// TODO: Make this work and decide on units.
	// We have model settings.
	uint32_t dt = current_time - temps[temp_current].last_temp_time;
	if (dt == 0)
		return;
	temps[temp_current].last_temp_time = current_time;
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
			temps[temp_current].time_on += current_time - temps[temp_current].last_temp_time;
	}
	else {
		if (temps[temp_current].is_on) {
			RESET(temps[temp_current].power_pin);
			temps[temp_current].is_on = false;
			temps[temp_current].time_on += current_time - temps[temp_current].last_temp_time;
			--temps_busy;
		}
	}
	*/
} // }}}

// Space things. {{{
static void check_distance(Motor *mtr, float distance, float dt, float &factor) { // {{{
	if (isnan(distance) || distance == 0) {
		mtr->target_dist = 0;
		mtr->last_v = 0;
		return;
	}
	//debug("cd %f %f", F(distance), F(dt));
	mtr->target_dist = distance;
	mtr->target_v = distance / dt;
	float v = fabs(mtr->target_v);
	int8_t s = (mtr->target_v < 0 ? -1 : 1);
	// When turning around, ignore limits (they shouldn't have been violated anyway).
	if (mtr->last_v * s < 0) {
		//debug("!");
		mtr->last_v = 0;
	}
	// Limit v.
	if (v > mtr->limit_v) {
		//debug("v %f %f", F(v), F(mtr->limit_v));
		distance = (s * mtr->limit_v) * dt;
		v = fabs(distance / dt);
	}
	//debug("cd2 %f %f", F(distance), F(dt));
	// Limit a+.
	float limit_dv = mtr->limit_a * dt;
	if (v - mtr->last_v * s > limit_dv) {
		//debug("a+ %f %f %f %d", F(mtr->target_v), F(limit_dv), F(mtr->last_v), s);
		distance = (limit_dv * s + mtr->last_v) * dt;
		v = fabs(distance / dt);
	}
	//debug("cd3 %f %f", F(distance), F(dt));
	// Limit a-.
	// Distance to travel until end of segment or connection.
	float max_dist = (mtr->endpos - mtr->current_pos / mtr->steps_per_m) * s;
	// Find distance traveled when slowing down at maximum a.
	// x = 1/2 at²
	// t = sqrt(2x/a)
	// v = at
	// v² = 2a²x/a = 2ax
	// x = v²/2a
	float limit_dist = v * v / 2 / mtr->limit_a;
	//debug("max %f limit %f v %f a %f", max_dist, limit_dist, v, mtr->limit_a);
	if (max_dist > 0 && limit_dist > max_dist) {
		//debug("a- %f %f %f %d %d %f", F(mtr->endpos), F(mtr->limit_a), F(max_dist), F(mtr->current_pos), s, dt);
		v = sqrt(max_dist * 2 * mtr->limit_a);
		distance = s * v * dt;
	}
	//debug("cd4 %f %f", F(distance), F(dt)); */
	float f = distance / mtr->target_dist;
	//movedebug("checked %f %f", F(mtr->target_dist), F(distance));
	if (f < factor)
		factor = f;
} // }}}

static void move_axes(Space *s, uint32_t current_time, float &factor) { // {{{
	float motors_target[s->num_motors];
	bool ok = true;
	space_types[s->type].xyz2motors(s, motors_target, &ok);
	// Try again if it didn't work; it should have moved target to a better location.
	if (!ok)
		space_types[s->type].xyz2motors(s, motors_target, &ok);
	//movedebug("ok %d", ok);
	for (uint8_t m = 0; m < s->num_motors; ++m) {
		//movedebug("move %d %f %f %f", m, F(target[m]), F(motors_target[m]), F(s->motor[m]->current_pos));
		check_distance(s->motor[m], motors_target[m] - s->motor[m]->current_pos / s->motor[m]->steps_per_m, (current_time - last_time) / 1e6, factor);
	}
} // }}}

static void do_steps(float &factor, uint32_t current_time) { // {{{
	//debug("steps");
	if (factor <= 0) {
		next_motor_time = 0;
		movedebug("end move");
		for (uint8_t s = 0; s < num_spaces; ++s)
			spaces[s].active = false;
		moving = false;
		arch_move();
		return;
	}
	//movedebug("do steps %f %d", F(factor), max_steps);
	next_motor_time = 0;
	bool have_steps = false;
	for (uint8_t s = 0; !have_steps && s < num_spaces; ++s) {
		Space &sp = spaces[s];
		if (!sp.active)
			continue;
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			float num = (mtr.current_pos / mtr.steps_per_m + mtr.target_dist * factor) * mtr.steps_per_m;
			if (mtr.current_pos != int(num + (num > 0 ? .49 : -.49))) {
				//debug("have steps %d %f %f", mtr.current_pos, mtr.target_dist, factor);
				have_steps = true;
				break;
			}
			//debug("no steps yet %d %d", s, m);
		}
	}
	uint32_t the_last_time = last_current_time;
	last_current_time = current_time;
	// If there are no steps to take, wait until there are.
	if (!have_steps)
		return;
	// Adjust start time if factor < 1.
	if (factor > 0 && factor < 1) {
		start_time += (current_time - the_last_time) * ((1 - factor) * .99);
		movedebug("correct: %f %d", F(factor), int(start_time));
	}
	else
		movedebug("no correct: %f %d", F(factor), int(start_time));
	last_time = current_time;
	// Move the motors.
	//debug("start move");
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		if (!sp.active)
			continue;
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			if (mtr.target_dist == 0) {
				mtr.last_v = 0;
				continue;
			}
			float target = mtr.current_pos / mtr.steps_per_m + mtr.target_dist * factor;
			//debug("ccp3 %d %d %d %f %f %f %f %f", m, stopping, mtr.current_pos, F(target), F(mtr.last_v), mtr.steps_per_m, mtr.target_dist, factor);
			mtr.current_pos = (target * mtr.steps_per_m + (target > 0 ? .49 : -.49));
			//debug("cp3 %d", mtr.current_pos);
			mtr.last_v = mtr.target_v * factor;
		}
	}
	arch_move();
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		if (!sp.active)
			continue;
		for (uint8_t a = 0; a < sp.num_axes; ++a)
			sp.axis[a]->current += (sp.axis[a]->target - sp.axis[a]->current) * factor;
	}
} // }}}

static void handle_motors(uint32_t current_time, uint32_t longtime) { // {{{
	//debug("handle");
	if (next_motor_time > 0)
		return;
	// Check for move.
	if (!moving || stopping) {
		next_motor_time = ~0;
		//debug("setting next motor time to ~0");
		return;
	}
	last_active = longtime;
	float factor = 1;
	float t = (current_time - start_time) / 1e6;
	//buffered_debug("f%f %f %f %ld %ld", F(t), F(t0), F(tp), F(long(current_time)), F(long(start_time)));
	if (t >= t0 + tp) {	// Finish this move and prepare next.
		movedebug("finishing %f %f %f %ld %ld", F(t), F(t0), F(tp), F(long(current_time)), F(long(start_time)));
		//buffered_debug("a");
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			if (!sp.active)
				continue;
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				//debug("before source %d %f %f", a, F(axis[a].source), F(axis[a].motor.dist));
				if (!isnan(sp.axis[a]->dist)) {
					sp.axis[a]->source += sp.axis[a]->dist;
					sp.axis[a]->dist = NAN;
					// Set this here, so it isn't set if dist was NaN to begin with.
					// Note that target is not set for future iterations, but it isn't changed.
					sp.axis[a]->target = sp.axis[a]->source;
				}
				//debug("after source %d %f %f %f %f", a, F(sp.axis[a]->source), F(sp.axis[a]->dist), F(sp.motor[a]->current_pos), F(factor));
			}
			move_axes(&sp, current_time, factor);
			//debug("f %f", F(factor));
		}
		//debug("f2 %f %ld %ld", F(factor), F(last_time), F(current_time));
		do_steps(factor, current_time);
		//debug("f3 %f", F(factor));
		// Start time may have changed; recalculate t.
		t = (current_time - start_time) / 1e6;
		if (t / (t0 + tp) >= done_factor) {
			//buffered_debug("b");
			moving = false;
			uint8_t had_cbs = cbs_after_current_move;
			cbs_after_current_move = 0;
			had_cbs += next_move();
			if (moving) {
				//buffered_debug("c");
				//debug("movecb 1");
				if (had_cbs > 0)
					send_host(CMD_MOVECB, had_cbs);
				return;
			}
			//buffered_debug("d");
			cbs_after_current_move += had_cbs;
			if (factor == 1) {
				//buffered_debug("e");
				moving = false;
				arch_move();
				//debug("movecb 1");
				if (cbs_after_current_move > 0) {
					send_host(CMD_MOVECB, cbs_after_current_move);
					cbs_after_current_move = 0;
				}
			}
			else {
				moving = true;
				next_motor_time = 0;
				//if (factor > 0)
				//	debug("not done %f", F(factor));
			}
		}
		return;
	}
	if (t < t0) {	// Main part.
		float t_fraction = t / t0;
		float current_f = (f1 * (2 - t_fraction) + f2 * t_fraction) * t_fraction;
		movedebug("main t %f t0 %f tp %f tfrac %f f1 %f f2 %f cf %f", F(t), F(t0), F(tp), F(t_fraction), F(f1), F(f2), F(current_f));
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			//movedebug("try %d %d", s, sp.active);
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
} // }}}
// }}}

// Memory handling. {{{
void _mem_alloc(uint32_t size, void **target) {
	*target = malloc(size);
	if (!*target) {
		debug("unable to allocate memory");
		reset();
	}
}

void _mem_retarget(void **target, void **newtarget) {
	*newtarget = *target;
	*target = NULL;
}

void _mem_free(void **target) {
	free(*target);
	*target = NULL;
}
// }}}

void reset() { // {{{
	// This shouldn't happen.  But if it does, die.
	// First disable all pins.
	debug("Reset requested; exiting.");
	led_pin.read(0);
	probe_pin.read(0);
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (unsigned m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->step_pin.read(0);
			sp.motor[m]->dir_pin.read(0);
			sp.motor[m]->enable_pin.read(0);
			sp.motor[m]->limit_min_pin.read(0);
			sp.motor[m]->limit_max_pin.read(0);
			sp.motor[m]->sense_pin.read(0);
		}
	}
	for (uint8_t t = 0; t < num_temps; ++t) {
		temps[t].power_pin.read(0);
		temps[t].thermistor_pin.read(0);
	}
	for (uint8_t g = 0; g < num_gpios; ++g)
		gpios[g].pin.read(0);
	exit(0);
} // }}}

int main(int argc, char **argv) { // {{{
	if (argc != 2) {
		debug("Franklin cdriver is not intended to be called directly.\n");
		exit(1);
	}
	setup(argv[1]);
	while(true) {
		// Handle any arch-specific periodic things.
		arch_run();
		// Timekeeping.
		uint32_t current_time;
		uint32_t longtime;
		get_current_times(&current_time, &longtime);
		uint32_t delta = current_time - last_current_time;
		//debug("delta: %ld", F(long(delta)));
		if (!serialdev[0]->available() && (!serialdev[1] || !serialdev[1]->available())) {
			uint32_t next_time = ~0;
			if (next_temp_time < next_time)
				next_time = next_temp_time;
			if (next_motor_time < next_time)
				next_time = next_motor_time;
			//debug("next time: %d", next_time);
			if (next_time > delta + 10000) {
				// Wait for next event; compensate for time already during this iteration (+10ms, to be sure); don't alter "infitity" flag.
				wait_for_event(~next_time ? next_time - delta - 10000: ~0, last_current_time);
				get_current_times(&current_time, &longtime);
				delta = current_time - last_current_time;
			}
		}
		//debug("event");
		last_current_time = current_time;
		// Update next_*_time.
		if (~next_motor_time)
			next_motor_time = next_motor_time > delta ? next_motor_time - delta : 0;
		if (~next_temp_time)
			next_temp_time = next_temp_time > delta ? next_temp_time - delta : 0;
		// Timeouts.  Do this before calling other things, because last_active may be updated and become larger than longtime.
		if (motors_busy && (longtime - last_active) / 1e3 > motor_limit) {
			debug("motor timeout %ld %ld %f", F(long(longtime)), F(long(last_active)), F(motor_limit));
			for (uint8_t s = 0; s < num_spaces; ++s) {
				Space &sp = spaces[s];
				for (uint8_t m = 0; m < sp.num_motors; ++m) {
					RESET(sp.motor[m]->enable_pin);
					sp.motor[m]->current_pos = 0;
				}
				for (uint8_t a = 0; a < sp.num_axes; ++a) {
					sp.axis[a]->current = NAN;
					sp.axis[a]->source = NAN;
				}
			}
			motors_busy = false;
			send_host(CMD_AUTOSLEEP, 0);
		}
		if (temps_busy > 0 && (longtime - last_active) / 1e3 > temp_limit) {
			for (uint8_t current_t = 0; current_t < num_temps; ++current_t) {
				RESET(temps[current_t].power_pin);
				temps[current_t].target = NAN;
				temps[current_t].adctarget = MAXINT;
				temps[current_t].is_on = false;
			}
			temps_busy = 0;
			send_host(CMD_AUTOSLEEP, 1);
		}
		// Handle all periodic things.
#ifdef TIMING
		uint32_t first_t = utime();
#endif
		if (!next_motor_time)
			handle_motors(current_time, longtime);	// Movement.
#ifdef TIMING
		uint32_t motor_t = utime() - current_time;
#endif
		if (!next_temp_time)
			handle_temps(current_time, longtime);	// Periodic temps stuff: temperature regulation.
#ifdef TIMING
		uint32_t temp_t = utime() - current_time;
#endif
		//debug("serial");
		serial(0);
		//debug("serial 0 done");
		if (serialdev[1])
			serial(1);
		//debug("serial 1 done");
#ifdef TIMING
		uint32_t serial_t = utime() - first_t;
		uint32_t end_t = utime() - current_time;
		end_t -= motor_t;
		motor_t -= temp_t;
		static int waiter = 0;
		if (waiter > 0)
			waiter -= 1;
		else {
			waiter = 977;
			debug("t: serial %ld temp %ld motor %ld end %ld", F(serial_t), F(temp_t), F(motor_t), F(end_t));
		}
#endif
	}
} // }}}

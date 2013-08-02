// vim: set filetype=cpp foldmethod=marker foldmarker={,} :
#include "firmware.hh"

// Loop function handles all regular updates.
// I'd have liked to have a timer interrupt for these actions, but arduino doesn't allow it.

float Temp::read () {
	if (thermistor_pin >= 255)
		return NAN;
	uint16_t adc = analogRead (thermistor_pin);
	// adc = adc0 * exp (-beta * (T - T0))
	// so
	//	adc/adc0 = exp (-beta * (T-T0))
	//	ln (adc/adc0) = -beta*(T-T0)
	//	T = -ln(adc/adc0)/beta+T0
	if (adc0 > 0)
		return -log (adc / adc0) / beta + T0;
	// adc0 <= 0 is used for debugging: return raw value.
	return adc;
}

void debug (char const *fmt, ...) {
	va_list ap;
	va_start (ap, fmt);
	Serial.write (CMD_DEBUG);
	for (char const *p = fmt; *p; ++p) {
		if (*p == '%') {
			++p;
			switch (*p) {
			case 0: {
				Serial.write ('%');
				--p;
				break;
			}
			case '%': {
				Serial.write ('%');
				break;
			}
			case 'd': {
				int arg = va_arg (ap, int);
				Serial.print (arg, DEC);
				break;
			}
			case 'x': {
				int arg = va_arg (ap, int);
				Serial.print (arg, HEX);
				break;
			}
			case 'f': {
				float arg = va_arg (ap, float);
				Serial.print (arg);
				break;
			}
			case 's': {
				char const *arg = va_arg (ap, char const *);
				Serial.print (arg);
				break;
			}
			case 'c': {
				char arg = va_arg (ap, char);
				Serial.write (arg);
				break;
			}
			default: {
				Serial.write ('%');
				Serial.write (*p);
				break;
			}
			}
		}
		else {
			Serial.write (*p);
		}
	}
	va_end (ap);
	Serial.write (0);
	Serial.flush ();
}

static float predict (Temp *temp, float energy, float added, float convection) {	// Predict the temperature after one step, given current and added energy
	float rt4 = roomtemperature * roomtemperature;
	rt4 *= rt4;
	// Radiation is supposed to go with T ** 4, so expect that.
	float temp4 = energy * energy;
	temp4 *= temp4;
	// Store "current" temperature.
	float current_temp = energy;
	// Radiation balance.
	energy -= temp4 * temp->radiation;
	energy += rt4 * temp->radiation;
	// Convection balance.
	energy -= current_temp * convection;
	energy += roomtemperature * convection;
	// Heater energy.
	energy += added;
	// Extruded material heat loss (assuming extrusion is constant during buffer time).
	energy -= temp->extra_loss * energy;
	return energy;
}

static uint8_t temp_counter = 1;

void loop () {
	serial ();
	unsigned long current_time = micros ();
	for (uint8_t t = 0; t < FLAG_EXTRUDER0 + num_extruders; ++t) {	// Periodic temps stuff: temperature regulation.
		if (--temp_counter)
			continue;
		temp_counter = 100;
		// Only spend time if it may be useful.
		if (!temps[t] || isnan (temps[t]->target) || temps[t]->power_pin >= 255 || temps[t]->thermistor_pin >= 255)
			continue;
		float temp = temps[t]->read ();
		// First of all, if an alarm should be triggered, do so.
		if (!isnan (temps[t]->min_alarm) && temps[t]->min_alarm < temp || !isnan (temps[t]->max_alarm) && temps[t]->max_alarm > temp) {
			temps[t]->min_alarm = NAN;
			temps[t]->max_alarm = NAN;
			which_tempcbs |= (1 << t);
			try_send_next ();
		}
		unsigned long dt = current_time - temps[t]->last_time;
		if (dt == 0)
			continue;
		unsigned long shift_dt = current_time - temps[t]->last_shift_time;
		temps[t]->last_time = current_time;
		if (temps[t]->is_on)
			temps[t]->buffer[0] += temps[t]->power * dt;
		while (shift_dt >= temps[t]->buffer_delay && temps[t]->buffer_delay != 0) {
			temps[t]->last_shift_time += temps[t]->buffer_delay * 1000000;
			shift_dt -= temps[t]->buffer_delay * 1000000;
			// Adjust convection: predicted - (last_temp - roomtemp) * convection = temp.
			float predicted = predict (temps[t], temps[t]->last_temp, temps[t]->buffer[3], 0);
			temps[t]->convection = (predicted - temp) / (temps[t]->last_temp - roomtemperature);
			temps[t]->last_temp = temp;
			for (uint8_t b = 3; b > 0; --b)
				temps[t]->buffer[b] = temps[t]->buffer[b - 1];
			temps[t]->buffer[0] = 0;
		}
		// Predict temperature when heat that would be added now would reach the sensor.
		float energy = temp;
		for (int8_t b = 3; b >= 0; --b)
			energy = predict (temps[t], energy, temps[t]->buffer[b], temps[t]->convection);
		if (energy < temps[t]->target) {
			SET (temps[t]->power_pin);
			temps[t]->is_on = true;
		}
		else {
			RESET (temps[t]->power_pin);
			temps[t]->is_on = false;
		}
	}
	for (uint8_t m = 0; m < FLAG_EXTRUDER0 + num_extruders; ++m) {	// Periodic motors stuff: movement.
		if (!motors[m])
		       continue;
		if (motors[m]->steps_done >= motors[m]->steps_total) {
			if (!motors[m]->continuous)
				continue;
			unsigned long t = current_time - motors[m]->start_time;
			if (t <= 0)
				continue;
			float f = motors[m]->f1;
			if (f > motors[m]->max_f)
				f = motors[m]->max_f;
			if (t * f >= 1000000.) {
				SET (motors[m]->step_pin);
				if (t * f < 2000000.)
					motors[m]->start_time += (unsigned long)(1000000. / f);
				else {
					// We're too slow to keep up; prevent catch-up attempts.
					motors[m]->start_time = current_time;
				}
				RESET (motors[m]->step_pin);
			}
			motors[m]->f = f;
			continue;
		}
		float t = (current_time - motors[m]->start_time) / 1000000.;
		motors[m]->f = 2 * motors[m]->a * t + motors[m]->f0;
		float now = motors[m]->a * t * t + motors[m]->f0 * t;
		unsigned steps = now * motors[m]->steps_total - motors[m]->steps_done;
	        for (unsigned s = 0; s < steps; ++s) {
			SET (motors[m]->step_pin);
			++motors[m]->steps_done;
			RESET (motors[m]->step_pin);
		}
		if (motors[m]->steps_done >= motors[m]->steps_total) {
			if (!--motors_busy) {
				if (queue[queue_start].cb) {
					++num_movecbs;
					try_send_next ();
				}
				next_move ();
			}
		}
	}
	for (uint8_t a = 0; a < 3; ++a) {	// Periodic axis stuff: endstop checking.
		if ((axis[a].motor.continuous || axis[a].motor.steps_total > axis[a].motor.steps_done) && (axis[a].motor.positive ? GET (axis[a].limit_max_pin, false) : GET (axis[a].limit_min_pin, false))) {
			debug ("hit %d", a);
			// Hit endstop; abort current move and notify host.
			axis[a].motor.continuous = false;	// Stop continuous move only for the motor that hits the switch.
			for (uint8_t m = 0; m < FLAG_EXTRUDER0 + num_extruders; ++m) {
				if (!motors[m])
					continue;
				motors[m]->steps_total = 0;
			}
			limits_hit |= (1 << a);
			if (queue[queue_start].cb)
				++num_movecbs;
			try_send_next ();
			next_move ();
		}
	}
	for (uint8_t e = 0; e < num_extruders; ++e) {	// Periodic extruder stuff: extrusion heat loss.
		float mm_per_s = extruder[e].motor.f / extruder[e].motor.steps_per_mm;
		float mm_per_shift = mm_per_s * extruder[e].temp.buffer_delay / 1000000;
		extruder[e].temp.extra_loss = mm_per_shift * extruder[e].capacity;
	}
}

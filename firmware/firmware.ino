// vim: set filetype=cpp foldmethod=marker foldmarker={,} :
#define EXTERN	// This must be done in exactly one cc-file.
#include "firmware.hh"

// Loop function handles all regular updates.
// I'd have liked to have a timer interrupt for these actions, but arduino doesn't allow it.

static float predict (Temp *temp, Extruder *extruder, float energy, float added, float convection) {	// Predict the temperature after one step, given current and added energy
	float rt4 = roomtemperature + 273.15;
	rt4 *= rt4;
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
	energy += (roomtemperature + 273.15) * convection;
	// Heater energy.
	energy += added;
	// Extruded material heat loss (assuming extrusion is constant during buffer time).
	if (extruder) {
		float mm_per_s = extruder->motor.f / extruder->motor.steps_per_mm;
		float mm_per_shift = mm_per_s * temp->buffer_delay / 1e6;
		energy -= mm_per_shift * extruder->capacity * energy;
	}
	return energy;
}

static uint8_t temp_counter = 1;
static uint8_t temp_current = 0;
static void handle_temps (unsigned long current_time) {
	if (--temp_counter)
		return;
	temp_counter = 20;
	// Only spend time if it may be useful.
	if (temps[temp_current] && (!isnan (temps[temp_current]->target) || !isnan (temps[temp_current]->min_alarm) || !isnan (temps[temp_current]->max_alarm)) && temps[temp_current]->power_pin < 255 && temps[temp_current]->thermistor_pin < 255) {
		float temp = temps[temp_current]->read () + 273.15;
		// First of all, if an alarm should be triggered, do so.
		if (!isnan (temps[temp_current]->min_alarm) && temps[temp_current]->min_alarm < temp || !isnan (temps[temp_current]->max_alarm) && temps[temp_current]->max_alarm > temp) {
			temps[temp_current]->min_alarm = NAN;
			temps[temp_current]->max_alarm = NAN;
			which_tempcbs |= (1 << temp_current);
			try_send_next ();
		}
		unsigned long dt = current_time - temps[temp_current]->last_time;
		if (dt == 0) {
			temp_current = (temp_current + 1) % (FLAG_EXTRUDER0 + num_extruders);
			return;
		}
		unsigned long shift_dt = current_time - temps[temp_current]->last_shift_time;
		temps[temp_current]->last_time = current_time;
		if (temps[temp_current]->is_on)
			temps[temp_current]->buffer[0] += temps[temp_current]->power * dt;
		while (shift_dt >= temps[temp_current]->buffer_delay && temps[temp_current]->buffer_delay != 0) {
			temps[temp_current]->last_shift_time += temps[temp_current]->buffer_delay * 1000000.;
			shift_dt -= temps[temp_current]->buffer_delay * 1000000.;
			// Adjust convection: predicted - (last_temp - roomtemp) * convection = temp.
			float predicted = predict (temps[temp_current], &extruder[temp_current], temps[temp_current]->last_temp, temps[temp_current]->buffer[3], 0);
			temps[temp_current]->convection = (predicted - temp) / (temps[temp_current]->last_temp - roomtemperature - 273.15);
			temps[temp_current]->last_temp = temp;
			for (uint8_t b = 3; b > 0; --b)
				temps[temp_current]->buffer[b] = temps[temp_current]->buffer[b - 1];
			temps[temp_current]->buffer[0] = 0;
		}
		// Predict temperature when heat that would be added now would reach the sensor.
		float energy = temp;
		for (int8_t b = 3; b >= 0; --b)
			energy = predict (temps[temp_current], &extruder[temp_current], energy, temps[temp_current]->buffer[b], temps[temp_current]->convection);
		if (energy < temps[temp_current]->target) {
			SET (temps[temp_current]->power_pin);
			temps[temp_current]->is_on = true;
		}
		else {
			RESET (temps[temp_current]->power_pin);
			temps[temp_current]->is_on = false;
		}
	}
	temp_current = (temp_current + 1) % (FLAG_EXTRUDER0 + num_extruders);
}

static void handle_motors (unsigned long current_time) {
	for (uint8_t m = 0; m < FLAG_EXTRUDER0 + num_extruders; ++m) {
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
			if (t * f >= 1e6) {
				SET (motors[m]->step_pin);
				if (t * f < 2e6)
					motors[m]->start_time += (unsigned long)(1e6 / f);
				else {
					// We're too slow to keep up; prevent catch-up attempts.
					motors[m]->start_time = current_time;
				}
				RESET (motors[m]->step_pin);
			}
			motors[m]->f = f;
			continue;
		}
		float t = (current_time - motors[m]->start_time) / 1e6;
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
}

static void handle_axis (unsigned long current_time) {
	for (uint8_t axis_current = 0; axis_current < 3; ++axis_current) {
		if ((!axis[axis_current].motor.continuous && axis[axis_current].motor.steps_total <= axis[axis_current].motor.steps_done) || (axis[axis_current].motor.positive ? !GET (axis[axis_current].limit_max_pin, false) : !GET (axis[axis_current].limit_min_pin, false)))
			continue;
		debug ("hit %d", axis_current);
		// Hit endstop; abort current move and notify host.
		axis[axis_current].motor.continuous = false;	// Stop continuous move only for the motor that hits the switch.
		for (uint8_t m = 0; m < FLAG_EXTRUDER0 + num_extruders; ++m) {
			if (!motors[m])
				continue;
			motors[m]->steps_total = 0;
		}
		limits_hit |= (1 << axis_current);
		if (queue[queue_start].cb)
			++num_movecbs;
		try_send_next ();
		next_move ();
	}
}

void loop () {
	serial ();
	unsigned long current_time = micros ();
	handle_temps (current_time);	// Periodic temps stuff: temperature regulation.
	handle_motors (current_time);	// Movement.
	handle_axis (current_time);	// limit switches.
}

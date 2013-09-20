#include <EEPROM.h>

// vim: set filetype=cpp foldmethod=marker foldmarker={,} :
#define EXTERN	// This must be done in exactly one cc-file.
#include "firmware.h"

// Loop function handles all regular updates.
// I'd have liked to have a timer interrupt for these actions, but arduino doesn't allow it.

static uint8_t temp_counter = 1;
static uint8_t temp_current = 0;
static void handle_temps (unsigned long current_time) {
	if (--temp_counter)
		return;
	temp_counter = 20;
	uint8_t i;
	for (i = 1; i <= 2 + MAXAXES + MAXEXTRUDERS + MAXTEMPS; ++i) {
		uint8_t next = (temp_current + i) % (2 + MAXAXES + MAXEXTRUDERS + MAXTEMPS);
		if (temps[next] && (!isnan (temps[next]->target) || !isnan (temps[next]->min_alarm) || !isnan (temps[next]->max_alarm)) && temps[next]->power_pin < 255 && (temps[next]->thermistor_pin < 255 || (temps[next]->target > 0 && isinf (temps[next]->target)))) {
			temp_current = next;
			break;
		}
	}
	// If there is no temperature handling to do; return.
	if (i > 2 + MAXAXES + MAXEXTRUDERS + MAXTEMPS) {
		return;
	}
	float temp = temps[temp_current]->read ();
	// First of all, if an alarm should be triggered, do so.
	if ((!isnan (temps[temp_current]->min_alarm) && temps[temp_current]->min_alarm < temp) || (!isnan (temps[temp_current]->max_alarm) && temps[temp_current]->max_alarm > temp)) {
		temps[temp_current]->min_alarm = NAN;
		temps[temp_current]->max_alarm = NAN;
		which_tempcbs |= (1 << temp_current);
		try_send_next ();
	}
	if (isnan (temps[temp_current]->core_C) || isnan (temps[temp_current]->shell_C) || isnan (temps[temp_current]->transfer) || isnan (temps[temp_current]->radiation)) {
		// No valid settings; use simple on/off-regime based on current temperature only.
		if (temp < temps[temp_current]->target) {
			if (!temps[temp_current]->is_on)
				debug ("switching on %d", temp_current);
			SET (temps[temp_current]->power_pin);
			temps[temp_current]->is_on = true;
		}
		else {
			if (temps[temp_current]->is_on)
				debug ("switching off %d", temp_current);
			RESET (temps[temp_current]->power_pin);
			temps[temp_current]->is_on = false;
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
		SET (temps[temp_current]->power_pin);
		temps[temp_current]->is_on = true;
	}
	else {
		RESET (temps[temp_current]->power_pin);
		temps[temp_current]->is_on = false;
	}
}

static void handle_motors (unsigned long current_time, unsigned long longtime) {
	for (uint8_t m = 0; m < MAXOBJECT; ++m) {
		if (!motors[m])
		       continue;
		if (motors[m]->steps_done >= motors[m]->steps_total) {
			if (!motors[m]->continuous)
				continue;
			last_active = longtime;
			float f = motors[m]->f;
			float max_f = (motors[m]->positive ? motors[m]->max_f_pos : motors[m]->max_f_neg);
			if (f > max_f) {
				f = max_f;
				motors[m]->f = f;
			}
			unsigned steps = int (current_time * f / 1e6) - int (last_time * f / 1e6);
			for (unsigned s = 0; s < steps; ++s) {
				SET (motors[m]->step_pin);
				RESET (motors[m]->step_pin);
				if (m >= 2 && m < num_axes + 2)
					axis[m - 2].current_pos += (motors[m]->positive ? 1 : -1);
			}
			continue;
		}
		last_active = longtime;
		float t = (current_time - start_time) / 1e6;
		motors[m]->f = 2 * motors[m]->a * t + f0;
		float now = motors[m]->a * t * t + f0 * t;
		unsigned steps = now * motors[m]->steps_total - motors[m]->steps_done;
	        for (unsigned s = 0; s < steps; ++s) {
			SET (motors[m]->step_pin);
			++motors[m]->steps_done;
			RESET (motors[m]->step_pin);
			if (m >= 2 && m < num_axes + 2) {
				axis[m - 2].current_pos += (motors[m]->positive ? 1 : -1);
			}
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
	last_time = current_time;
}

static void handle_axis (unsigned long current_time) {
	for (uint8_t axis_current = 0; axis_current < num_axes; ++axis_current) {
		// No check if motors are not moving.
		if (axis[axis_current].motor.steps_total <= axis[axis_current].motor.steps_done && !axis[axis_current].motor.continuous)
			continue;
		// No problem if limit switch is not hit.
		if (axis[axis_current].motor.positive ? !GET (axis[axis_current].limit_max_pin, false) : !GET (axis[axis_current].limit_min_pin, false))
			continue;
		bool need_cb = axis[axis_current].motor.steps_total > axis[axis_current].motor.steps_done && queue[queue_start].cb;
		debug ("hit %d %d %d", int (axis_current), axis[axis_current].motor.positive, axis[axis_current].limit_min_pin);
		// Hit endstop; abort current move and notify host.
		axis[axis_current].motor.continuous = false;	// Stop continuous move only for the motor that hits the switch.
		for (uint8_t m = 2 + MAXAXES; m < MAXOBJECT; ++m) {
			if (!motors[m])
				continue;
			if (motors[m]->steps_total > motors[m]->steps_done)
				--motors_busy;
			motors[m]->steps_total = 0;
		}
		limits_pos[axis_current] = axis[axis_current].current_pos;
		if (need_cb)
			++num_movecbs;
		try_send_next ();
		next_move ();
	}
}

void loop () {
	serial ();
	unsigned long current_time = micros ();
	unsigned long longtime = millis ();
	handle_temps (current_time);	// Periodic temps stuff: temperature regulation.
	handle_motors (current_time, longtime);	// Movement.
	handle_axis (current_time);	// limit switches.
	if (motor_limit > 0 && longtime - last_active > motor_limit) {
		for (uint8_t m = 0; m < MAXOBJECT; ++m) {
			if (!motors[m])
				continue;
			SET (motors[m]->enable_pin);
		}
	}
	if (temp_limit > 0 && longtime - last_active > temp_limit) {
		for (uint8_t t = 0; t < MAXOBJECT; ++t) {
			if (!temps[t])
				continue;
			RESET (temps[t]->power_pin);
			temps[t]->target = NAN;
		}
	}
}

// vim: set filetype=cpp :
#include "firmware.hh"

// Loop function handles all regular updates.
// I'd have liked to have a timer interrupt for these actions, but arduino doesn't allow it.

// Actions:
// Control heaters.
// Move motors.

float Temp::read ()
{
	if (thermistor_pin >= 255)
		return -1;
	uint16_t adc = analogRead (thermistor_pin);
	// adc = adc0 * exp (-beta * (T - T0))
	// so
	//	adc/adc0 = exp (-beta * (T-T0))
	//	ln (adc/adc0) = -beta*(T-T0)
	//	T = -ln(adc/adc0)/beta+T0
	return -log (adc / adc0) / beta + T0;
}

void loop ()
{
	serial ();
	unsigned long long current_time = millis ();
	for (uint8_t t = 0; t < FLAG_EXTRUDER0 + num_extruders; ++t)
	{
		// Only spend time if it may be useful.
		if (!temps[t] || temps[t]->target < 0 || temps[t]->power_pin == 255 || temps[t]->thermistor_pin == 255)
			continue;
		float temp = temps[t]->read ();
		// First of all, if an alarm should be triggered, do so.
		if (!isnan (temps[t]->min_alarm) && temps[t]->min_alarm < temp || !isnan (temps[t]->max_alarm) && temps[t]->max_alarm > temp)
		{
			temps[t]->min_alarm = NAN;
			temps[t]->max_alarm = NAN;
			which_tempcbs |= (1 << t);
			try_send_next ();
		}
		uint16_t dt = current_time - temps[t]->last_time;
		uint16_t shift_dt = current_time - temps[t]->last_shift_time;
		temps[t]->last_time = current_time;
		if (temps[t]->is_on)
			temps[t]->buffer[0] += temps[t]->power * dt;
		while (shift_dt >= temps[t]->buffer_delay)
		{
			temps[t]->last_shift_time += temps[t]->buffer_delay;
			shift_dt -= temps[t]->buffer_delay;
			for (uint8_t b = 3; b > 0; --b)
				temps[t]->buffer[b] = temps[t]->buffer[b - 1];
			temps[t]->buffer[0] = 0;
		}
		float energy = temp;
		for (int8_t b = 3; b >= 0; ++b)
		{
			// Radiation is supposed to go with T ** 4, so expect that.
			float temp2 = energy * energy;
			energy -= temps[t]->extra_loss * energy;
			energy -= temp2 * temp2 * temps[t]->radiation;
			energy += temps[t]->buffer[b];
		}
		if (energy < temps[t]->target)
		{
			SET (temps[t]->power_pin);
			temps[t]->is_on = true;
		}
		else
		{
			RESET (temps[t]->power_pin);
			temps[t]->is_on = false;
		}
	}
	for (uint8_t m = 0; m < FLAG_EXTRUDER0 + num_extruders; ++m)
	{
		if (!motors[m])
		       continue;
		if (motors[m]->steps_done >= motors[m]->steps_total)
		{
			if (!motors[m]->continuous)
				continue;
			float t = current_time - motors[m]->start_time;
			if (t * motors[m]->f1 >= 1)
			{
				SET (motors[m]->step_pin);
				motors[m]->start_time += 1 / motors[m]->f1;
				RESET (motors[m]->step_pin);
			}
			continue;
		}
		float t = current_time - motors[m]->start_time;
		float a = (motors[m]->f1 * motors[m]->f1 - motors[m]->f0 * motors[m]->f0) / 4;
		motors[m]->f = 2 * a * t + motors[m]->f0;
		float now = a * t * t + motors[m]->f0 * t;
		if (now * motors[m]->steps_total > motors[m]->steps_done)
		{
			SET (motors[m]->step_pin);
			++motors[m]->steps_done;
			if (motors[m]->steps_done >= motors[m]->steps_total)
			{
				if (!--motors_busy)
				{
					if (queue[queue_start].cb)
					{
						++num_movecbs;
						try_send_next ();
					}
					next_move ();
				}
			}
			RESET (motors[m]->step_pin);
		}
	}
	for (uint8_t a = 0; a < 3; ++a)
	{
		if (axis[a].motor.positive && !GET (axis[a].limit_max_pin, true) || !axis[a].motor.positive && !GET (axis[a].limit_min_pin, true))
		{
			// Hit endstop; abort current move and notify host.
			axis[a].motor.continuous = false;	// Stop continuous move only for the motor that hits the switch.
			for (uint8_t m = 0; m < FLAG_EXTRUDER0 + num_extruders; ++m)
			{
				if (!motors[m])
					continue;
				motors[m]->steps_total = 0;
			}
			limits_hit |= (1 << m);
			try_send_next ();
		}
	}
	for (uint8_t e = 0; e < num_extruders; ++e)
	{
		float mm_per_s = extruder[e].motor.f / extruder[e].motor.steps_per_mm;
		float mm_per_shift = mm_per_s * extruder[e].temp.buffer_delay / 1000;
		extruder[e].temp.extra_loss = mm_per_shift * extruder[e].capacity;
	}
}

// vim: set foldmethod=marker foldmarker={,} :
#include "firmware.hh"

void next_move () {
	if (queue_start == queue_end)
		return;
	// Set everything up for running queue[queue_start].
	uint8_t num = TEMP0;
	f0 = queue[queue_start].data[F0];
	f1 = queue[queue_start].data[F1];
	// Set up motors for moving.
	start_time = micros ();
	for (uint8_t a = 0; a < num_axes; ++a) {
		float target = queue[queue_start].data[AXIS0 + a];
		axis[a].motor.steps_done = 0;
		if (isnan (target)) {
			axis[a].motor.steps_total = 0;
			continue;
		}
		int32_t diff = int32_t (target * axis[a].motor.steps_per_mm) - axis[a].current_pos;
		axis[a].motor.steps_total = abs (diff);
		axis[a].motor.positive = diff > 0;
	}
	for (uint8_t e = 0; e < num_extruders; ++e) {
		float target = queue[queue_start].data[EXTRUDER0 + e];
		extruder[e].motor.steps_done = 0;
		if (isnan (target)) {
			extruder[e].motor.steps_total = 0;
			continue;
		}
		int32_t diff (queue[queue_start].data[EXTRUDER0 + e] * extruder[e].motor.steps_per_mm + .5);
		extruder[e].motor.steps_total = abs (diff);
		extruder[e].motor.positive = target > 0;
	}
	for (uint8_t m = 0; m < num; ++m) {
		if (!motors[m] || motors[m]->steps_total == 0)
			continue;
		if (motors[m]->positive)
			SET (motors[m]->dir_pin);
		else
			RESET (motors[m]->dir_pin);
		RESET (motors[m]->enable_pin);
		// Check if f0 and f1 are acceptable; adjust as needed.
		//motors[m]->max_f		steps/s
		//f0, f1			(fraction)/s
		//motors[m]->steps_total	steps
		float max_f = (motors[m]->positive ? motors[m]->max_f_pos : motors[m]->max_f_neg);
		if (f0 * motors[m]->steps_total > max_f)
			f0 = max_f / motors[m]->steps_total;
		if (f1 * motors[m]->steps_total > max_f)
			f1 = max_f / motors[m]->steps_total;
	}
	for (uint8_t m = 0; m < num; ++m) {	// Set flow rates to determined (limited) values.
		if (!motors[m] || motors[m]->steps_total == 0)
			continue;
		++motors_busy;
		motors[m]->f = f0;
		motors[m]->a = (f1 * f1 - f0 * f0) / 4;
	}
	queue_start = (queue_start + 1) & QUEUE_LENGTH_MASK;
}

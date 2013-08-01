#include "firmware.hh"

void next_move ()
{
	if (queue_start == queue_end)
		return;
	// Set everything up for running queue[queue_start].
	uint8_t num = FLAG_EXTRUDER0 + num_extruders;
	float f0 = queue[queue_start].data[FLAG_F0];
	float f1 = queue[queue_start].data[FLAG_F1];
	// Set up motors for moving.
	unsigned long long time = millis ();
	for (uint8_t m = 0; m < num; ++m)
	{
		float target = queue[queue_start].data[m];
		if (!motors[m])
			continue;
		if (isnan (target))
		{
			motors[m]->steps_total = 0;
			continue;
		}
		motors[m]->start_time = time;
		motors[m]->steps_total = uint16_t (abs (target) * motors[m]->steps_per_mm);
		motors[m]->steps_done = 0;
		motors[m]->positive = target > 0;
		if (motors[m]->positive)
			SET (motors[m]->dir_pin);
		else
			RESET (motors[m]->dir_pin);
		if (motors[m]->steps_total > 0)
			++motors_busy;
		// Check if f0 and f1 are acceptable; adjust as needed.
		//motors[m]->max_f		steps/ms
		//f0, f1			(fractie)/ms
		//motors[m]->steps_total	steps
		if (f0 * motors[m]->steps_total > motors[m]->max_f)
			f0 = motors[m]->max_f / motors[m]->steps_total;
		if (f1 * motors[m]->steps_total > motors[m]->max_f)
			f1 = motors[m]->max_f / motors[m]->steps_total;
	}
	for (uint8_t m = 0; m < num; ++m)
	{
		if (!motors[m] || motors[m]->steps_total == 0)
			continue;
		motors[m]->f0 = f0;
		motors[m]->f1 = f1;
		motors[m]->f = f0;
	}
	queue_start = (queue_start + 1) & QUEUE_LENGTH_MASK;
}


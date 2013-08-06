#include "firmware.hh"

void setup ()
{
	// Initialize volatile variables.
	Serial.begin (115200);
	command_end = 0;
	motors_busy = 0;
	queue_start = 0;
	queue_end = 0;
	num_movecbs = 0;
	continue_cb = false;
	which_tempcbs = 0;
	limits_hit = 0;
	pause_all = false;
	last_packet = NULL;
	out_busy = false;
	reply_ready = false;
	// Prepare asynchronous command buffers.
	limitcb_buffer[0] = 3;
	limitcb_buffer[1] = CMD_LIMIT;
	limitcb_buffer[2] = 0;
	movecb_buffer[0] = 3;
	movecb_buffer[1] = CMD_MOVECB;
	movecb_buffer[2] = 0;
	tempcb_buffer[0] = 3;
	tempcb_buffer[1] = CMD_TEMPCB;
	continue_buffer[0] = 3;
	continue_buffer[1] = CMD_CONTINUE;
	continue_buffer[2] = 0;
	for (uint8_t i = 0; i < MAXOBJECT; ++i)
	{
		if (i == 0)
		{
			motors[i] = NULL;
			temps[i] = NULL;
			objects[i] = &constants;
		}
		else if (i == 1)
		{
			motors[i] = NULL;
			temps[i] = NULL;
			objects[i] = &variables;
		}
		else if (i < 5)
		{
			motors[i] = &axis[i - 2].motor;
			temps[i] = NULL;
			objects[i] = &axis[i - 2];
		}
		else if (i == FLAG_BED)
		{
			motors[i] = NULL;
			temps[i] = &bed;
			objects[i] = &bed;
		}
		else
		{
			motors[i] = &extruder[i - FLAG_EXTRUDER0].motor;
			temps[i] = &extruder[i - FLAG_EXTRUDER0].temp;
			objects[i] = &extruder[i - FLAG_EXTRUDER0];
		}
	}
	for (uint8_t m = 0; m < MAXOBJECT; ++m)
	{
		if (!motors[m])
			continue;
		motors[m]->steps_total = 0;
		motors[m]->steps_done = 0;
		motors[m]->continuous = false;
	}
	unsigned long time = micros ();
	for (uint8_t t = 0; t < MAXOBJECT; ++t)
	{
		if (!temps[t])
			continue;
		temps[t]->last_time = time;
		temps[t]->last_shift_time = time;
		temps[t]->is_on = false;
		temps[t]->min_alarm = NAN;
		temps[t]->max_alarm = NAN;
		temps[t]->last_temp = NAN;
	}
	uint16_t address = 0;
	objects[0]->address = address;	// Not used, but initialized anyway.
	for (uint8_t o = 1; o < MAXOBJECT; ++o)
	{
		objects[o]->address = address;
		objects[o]->load (address, true);
	}
	Serial.write (CMD_INIT);
}

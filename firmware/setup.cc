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
	pause_all = false;
	last_packet = NULL;
	out_busy = false;
	reply_ready = false;
	f0 = 0;
	f1 = 0;
	start_time = micros ();
	// Prepare asynchronous command buffers.
	limitcb_buffer[0] = 7;
	limitcb_buffer[1] = CMD_LIMIT;
	limitcb_buffer[2] = 0;
	limitcb_buffer[3] = 0;
	limitcb_buffer[4] = 0;
	limitcb_buffer[5] = 0;
	limitcb_buffer[6] = 0;
	movecb_buffer[0] = 3;
	movecb_buffer[1] = CMD_MOVECB;
	movecb_buffer[2] = 0;
	tempcb_buffer[0] = 3;
	tempcb_buffer[1] = CMD_TEMPCB;
	continue_buffer[0] = 3;
	continue_buffer[1] = CMD_CONTINUE;
	continue_buffer[2] = 0;
	motors[F0] = NULL;
	temps[F0] = NULL;
	objects[F0] = &constants;
	motors[F1] = NULL;
	temps[F1] = NULL;
	objects[F1] = &variables;
	uint8_t i = 2;
	for (uint8_t a = 0; a < MAXAXES; ++a, ++i)
	{
		motors[i] = &axis[a].motor;
		temps[i] = NULL;
		objects[i] = &axis[a];
		limits_pos[a] = NAN;
	}
	for (uint8_t e = 0; e < MAXEXTRUDERS; ++e, ++i)
	{
		motors[i] = &extruder[e].motor;
		temps[i] = &extruder[e].temp;
		objects[i] = &extruder[e];
	}
	for (uint8_t t = 0; t < MAXAXES; ++t, ++i)
	{
		motors[i] = NULL;
		temps[i] = &temp[t];
		objects[i] = &temp[t];
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
		temps[t]->is_on = false;
		temps[t]->min_alarm = NAN;
		temps[t]->max_alarm = NAN;
	}
	uint16_t address = 0;
	objects[F0]->address = address;	// Not used, but initialized anyway.
	for (uint8_t o = 1; o < MAXOBJECT; ++o)
	{
		objects[o]->address = address;
		objects[o]->load (address, true);
	}
	if (address > EEsize)
		debug ("Warning: data doesn't fit in EEPROM; decrease MAXAXES, MAXEXTRUDERS, or MAXTEMPS and reflash the firmware!");
	Serial.write (CMD_INIT);
}

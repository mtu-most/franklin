#include "firmware.h"

void setup ()
{
	uint8_t mcusr = MCUSR;
	MCUSR = 0;
	watchdog_disable ();
	// Initialize volatile variables.
	Serial.begin (115200);
	initialized = false;
	adc_phase = 0;
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
	temp_current = 0;
#endif
	command_end = 0;
	motors_busy = 0;
	queue_start = 0;
	queue_end = 0;
	queue_full = false;
	num_movecbs = 0;
	continue_cb = 0;
	which_tempcbs = 0;
	ping = 0;
	last_packet = NULL;
	out_busy = false;
	reply_ready = false;
	led_pin.read (0x100);
	led_phase = 0;
	temps_busy = 0;
	led_last = millis ();
	last_active = millis ();
	v0 = 0;
	vp = 0;
	vq = 0;
	t0 = 0;
	tq = 0;
	f0 = 0;
	//debug("moving->false");
	moving = false;
	move_prepared = false;
	current_move_has_cb = false;
	which_autosleep = 0;
#ifdef AUDIO
	audio_head = 0;
	audio_tail = 0;
	audio_state = 0;
	audio_us_per_bit = 125; // 1000000 / 8000;
#endif
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		printerid[i] = 0;
#if MAXAXES > 0 || MAXEXTRUDERS > 0
	motors[F0] = NULL;
	motors[F1] = NULL;
#endif
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
	temps[F0] = NULL;
	temps[F1] = NULL;
#endif
	objects[F0] = &constants;
	objects[F1] = &variables;
	uint8_t i = 2;
	(void)&i;	// If pretty much nothing is available, this variable is not used.
#if MAXAXES > 0
	for (uint8_t a = 0; a < MAXAXES; ++a, ++i)
	{
		motors[i] = &axis[a].motor;
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
		temps[i] = NULL;
#endif
		objects[i] = &axis[a];
		limits_pos[a] = NAN;
		axis[a].current_pos = MAXLONG;
		axis[a].source = NAN;
		axis[a].current = NAN;
		axis[a].sense_state = 0;
		axis[a].sense_pos = MAXLONG;
		axis[a].limit_min_pin.read (0x100);
		axis[a].limit_max_pin.read (0x100);
		axis[a].sense_pin.read (0x100);
	}
#endif
#if MAXEXTRUDERS > 0
	for (uint8_t e = 0; e < MAXEXTRUDERS; ++e, ++i)
	{
		motors[i] = &extruder[e].motor;
		temps[i] = &extruder[e].temp;
		objects[i] = &extruder[e];
	}
#endif
#if MAXTEMPS > 0
	for (uint8_t t = 0; t < MAXTEMPS; ++t, ++i)
	{
#if MAXAXES > 0 || MAXEXTRUDERS > 0
		motors[i] = NULL;
#endif
		temps[i] = &temp[t];
		objects[i] = &temp[t];
	}
#endif
#if MAXGPIOS > 0
	for (uint8_t g = 0; g < MAXGPIOS; ++g, ++i)
	{
#if MAXAXES > 0 || MAXEXTRUDERS > 0
		motors[i] = NULL;
#endif
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
		temps[i] = NULL;
#endif
		objects[i] = &gpio[g];
		gpio[g].pin.read (0x100);
		gpio[g].state = 0;
#ifndef LOWMEM
		gpio[g].prev = NULL;
		gpio[g].next = NULL;
		gpio[g].master = 0;
#endif
	}
#endif
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
	unsigned long time = micros ();
#endif
	for (uint8_t o = 0; o < MAXOBJECT; ++o)
	{
#if MAXAXES > 0 || MAXEXTRUDERS > 0
		if (motors[o]) {
			motors[o]->dist = NAN;
			motors[o]->next_dist = NAN;
			motors[o]->f = 0;
			motors[o]->continuous_steps_per_s = 0;
#ifdef AUDIO
			motors[o]->audio_flags = 0;
#endif
			motors[o]->enable_pin.read (0x100);
			motors[o]->step_pin.read (0x100);
			motors[o]->dir_pin.read (0x100);
		}
#endif
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
		if (temps[o]) {
			temps[o]->last_time = time;
			temps[o]->is_on = false;
			temps[o]->min_alarm = NAN;
			temps[o]->max_alarm = NAN;
			temps[o]->target = NAN;
			temps[o]->last = NAN;
			temps[o]->power_pin.read (0x100);
			temps[o]->thermistor_pin.read (0x100);
#ifndef LOWMEM
			temps[o]->gpios = 0;
#endif
		}
#endif
	}
#if SERIAL_BUFFERSIZE > 0
	SETUP_SERIALS;
	for (uint8_t i = 0; i < NUMSERIALS; ++i)
		serialactive[i] = false;
	serial_out_busy = false;
	serialbuffer[0] = 0;
#endif
	int16_t address = 0;
	objects[F0]->address = address;	// Not used, but initialized anyway.
	for (uint8_t o = 1; o < MAXOBJECT; ++o)
	{
		objects[o]->address = address;
		objects[o]->load (address, true);
		// A packet is length, command, channel, data.
		int16_t len = 3 + address - objects[o]->address;
		if (len + (len + 2) / 3 >= COMMAND_SIZE)
			debug ("Warning: object %d is too large for the serial protocol!", o);
	}
	if (address - 1 > E2END)
		debug ("Warning: data doesn't fit in EEPROM; decrease MAXAXES, MAXEXTRUDERS, or MAXTEMPS and reflash the firmware!");
	Serial.write (CMD_ID);
	for (uint8_t i = 0; i < 8; ++i)
		Serial.write ('\0');
	debug ("Startup.  MCUSR: %x, Eeprom used: %d, available: %d", mcusr, address, E2END);
}

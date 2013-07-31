#define EXTERN	// This must be done in exactly one cc-file.
#include "firmware.hh"
#include <EEPROM.h>

static uint8_t read_8 (uint16_t &address, bool eeprom)
{
	if (eeprom)
		return EEPROM.read (address++);
	return command[address++];
}

static void write_8 (uint16_t &address, uint8_t data, bool eeprom)
{
	if (eeprom)
	{
		debug ("EEPROM[%x] = %x", address, data);
		EEPROM.write (address++, data);
		return;
	}
	reply[address++] = data;
}

static uint16_t read_16 (uint16_t &address, bool eeprom)
{
	uint8_t l = read_8 (address, eeprom);
	uint8_t h = read_8 (address, eeprom);
	return (uint16_t (h) << 8) | uint16_t (l);
}

static void write_16 (uint16_t &address, uint16_t data, bool eeprom)
{
	write_8 (address, data & 0xff, eeprom);
	write_8 (address, data >> 8, eeprom);
}

static float read_float (uint16_t &address, bool eeprom)
{
	ReadFloat ret;
	for (uint8_t t = 0; t < sizeof (float); ++t)
		ret.b[t] = read_8 (address, eeprom);
	return ret.f;
}

static void write_float (uint16_t &address, float data, bool eeprom)
{
	ReadFloat d;
	d.f = data;
	for (uint8_t t = 0; t < sizeof (float); ++t)
		write_8 (address, d.b[t], eeprom);
}

void Temp::load (uint16_t &addr, bool eeprom)
{
	beta = read_float (addr, eeprom);
	T0 = read_float (addr, eeprom);
	adc0 = read_float (addr, eeprom);
	radiation = read_float (addr, eeprom);
	power = read_float (addr, eeprom);
	buffer_delay = read_16 (addr, eeprom);
	power_pin = read_8 (addr, eeprom);
	thermistor_pin = read_8 (addr, eeprom);
}

void Temp::save (uint16_t &addr, bool eeprom)
{
	write_float (addr, beta, eeprom);
	write_float (addr, T0, eeprom);
	write_float (addr, adc0, eeprom);
	write_float (addr, radiation, eeprom);
	write_float (addr, power, eeprom);
	write_16 (addr, buffer_delay, eeprom);
	write_8 (addr, power_pin, eeprom);
	write_8 (addr, thermistor_pin, eeprom);
}

void Motor::load (uint16_t &addr, bool eeprom)
{
	step_pin = read_8 (addr, eeprom);
	dir_pin = read_8 (addr, eeprom);
	sleep_pin = read_8 (addr, eeprom);
	steps_per_mm = read_float (addr, eeprom);
}

void Motor::save (uint16_t &addr, bool eeprom)
{
	write_8 (addr, step_pin, eeprom);
	write_8 (addr, dir_pin, eeprom);
	write_8 (addr, sleep_pin, eeprom);
	write_float (addr, steps_per_mm, eeprom);
}

void Axis::load (uint16_t &addr, bool eeprom)
{
	motor.load (addr, eeprom);
	limit_min_pin = read_8 (addr, eeprom);
	limit_max_pin = read_8 (addr, eeprom);
}

void Axis::save (uint16_t &addr, bool eeprom)
{
	motor.save (addr, eeprom);
	write_8 (addr, limit_min_pin, eeprom);
	write_8 (addr, limit_max_pin, eeprom);
}

void Extruder::load (uint16_t &addr, bool eeprom)
{
	motor.load (addr, eeprom);
	temp.load (addr, eeprom);
	filament_heat = read_float (addr, eeprom);
	nozzle_size = read_float (addr, eeprom);
	filament_size = read_float (addr, eeprom);
}

void Extruder::save (uint16_t &addr, bool eeprom)
{
	motor.save (addr, eeprom);
	temp.save (addr, eeprom);
	write_float (addr, filament_heat, eeprom);
	write_float (addr, nozzle_size, eeprom);
	write_float (addr, filament_size, eeprom);
}

void bed_load (uint16_t &addr, bool eeprom)
{
	num_extruders = read_8 (addr, eeprom);
	debug ("addr: %x, num_extruders: %d", addr, num_extruders);
	// If num_extruders is an invalid value, the eeprom is probably not initialized; use 1 as default.
	if (num_extruders > MAXOBJECT - FLAG_EXTRUDER0)
		num_extruders = 1;
	bed.load (addr, eeprom);
}

void bed_save (uint16_t &addr, bool eeprom)
{
	write_8 (addr, num_extruders, eeprom);
	bed.save (addr, eeprom);
}

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
		if (i < 3)
		{
			motors[i] = &axis[i].motor;
			temps[i] = NULL;
			objects[i] = &axis[i];
		}
		else if (i == FLAG_BED)
		{
			motors[i] = NULL;
			temps[i] = &bed;
			objects[i] = &bed;
		}
		else if (i >= FLAG_EXTRUDER0)
		{
			motors[i] = &extruder[i - FLAG_EXTRUDER0].motor;
			temps[i] = &extruder[i - FLAG_EXTRUDER0].temp;
			objects[i] = &extruder[i - FLAG_EXTRUDER0];
		}
		else
		{
			motors[i] = NULL;
			temps[i] = NULL;
			objects[i] = NULL;
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
	for (uint8_t t = 0; t < MAXOBJECT; ++t)
	{
		if (!temps[t])
			continue;
		unsigned long long time = millis ();
		temps[t]->last_time = time;
		temps[t]->last_shift_time = time;
		temps[t]->is_on = false;
		temps[t]->extra_loss = 0;
		temps[t]->min_alarm = NAN;
		temps[t]->max_alarm = NAN;
	}
	uint16_t address = 0;
	for (uint8_t o = 0; o < MAXOBJECT; ++o)
	{
		if (!objects[o])
			continue;
		objects[o]->address = address;
		if (o == FLAG_BED)
			bed_load (address, true);
		else
			objects[o]->load (address, true);
	}
	Serial.write (CMD_INIT);
}

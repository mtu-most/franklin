#include "firmware.h"

void Motor::load (int16_t &addr, bool eeprom)
{
	bool e = GET (enable_pin, true);
	bool d = GET (dir_pin, false);
	bool s = GET (step_pin, false);
	step_pin.read (read_16 (addr, eeprom));
	dir_pin.read (read_16 (addr, eeprom));
	enable_pin.read (read_16 (addr, eeprom));
	steps_per_mm = read_float (addr, eeprom);
	max_v_neg = read_float (addr, eeprom);
	max_v_pos = read_float (addr, eeprom);
	max_a = read_float (addr, eeprom);
	SET_OUTPUT (step_pin);
	SET_OUTPUT (dir_pin);
	SET_OUTPUT (enable_pin);
	if (e)
		SET (enable_pin);
	else
		RESET (enable_pin);
	if (d)
		SET (dir_pin);
	else
		RESET (dir_pin);
	if (s)
		SET (step_pin);
	else
		RESET (step_pin);
}

void Motor::save (int16_t &addr, bool eeprom)
{
	write_16 (addr, step_pin.write (), eeprom);
	write_16 (addr, dir_pin.write (), eeprom);
	write_16 (addr, enable_pin.write (), eeprom);
	write_float (addr, steps_per_mm, eeprom);
	write_float (addr, max_v_neg, eeprom);
	write_float (addr, max_v_pos, eeprom);
	write_float (addr, max_a, eeprom);
}

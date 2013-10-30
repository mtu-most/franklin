#include "firmware.h"

void Axis::load (int16_t &addr, bool eeprom)
{
	motor.load (addr, eeprom);
	limit_min_pin = read_8 (addr, eeprom);
	limit_max_pin = read_8 (addr, eeprom);
	limit_min_pos = read_32 (addr, eeprom);
	limit_max_pos = read_32 (addr, eeprom);
	delta_length = read_float (addr, eeprom);
	delta_radius = read_float (addr, eeprom);
	SET_INPUT (limit_min_pin);
	SET_INPUT (limit_max_pin);
}

void Axis::save (int16_t &addr, bool eeprom)
{
	motor.save (addr, eeprom);
	write_8 (addr, limit_min_pin, eeprom);
	write_8 (addr, limit_max_pin, eeprom);
	write_32 (addr, limit_min_pos, eeprom);
	write_32 (addr, limit_max_pos, eeprom);
	write_float (addr, delta_length, eeprom);
	write_float (addr, delta_radius, eeprom);
}

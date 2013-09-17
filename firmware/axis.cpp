#include "firmware.h"

void Axis::load (uint16_t &addr, bool eeprom)
{
	motor.load (addr, eeprom);
	limit_min_pin = read_8 (addr, eeprom);
	limit_max_pin = read_8 (addr, eeprom);
	SET_INPUT (limit_min_pin);
	SET_INPUT (limit_max_pin);
}

void Axis::save (uint16_t &addr, bool eeprom)
{
	motor.save (addr, eeprom);
	write_8 (addr, limit_min_pin, eeprom);
	write_8 (addr, limit_max_pin, eeprom);
}

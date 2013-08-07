#include "firmware.hh"

void Constants::load (uint16_t &addr, bool eeprom)
{
	// This function must not be called.
}

void Constants::save (uint16_t &addr, bool eeprom)
{
	write_8 (addr, MAXOBJECT, eeprom);
}

void Variables::load (uint16_t &addr, bool eeprom)
{
	num_extruders = read_8 (addr, eeprom);
	roomtemperature = read_float (addr, eeprom) + 273.15;
	// If num_extruders is an invalid value, the eeprom is probably not initialized; use 1 as default.
	if (num_extruders > MAXOBJECT - FLAG_EXTRUDER0)
		num_extruders = 1;
}

void Variables::save (uint16_t &addr, bool eeprom)
{
	write_8 (addr, num_extruders, eeprom);
	write_float (addr, roomtemperature - 273.15, eeprom);
}

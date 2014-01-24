#include "firmware.h"

void Extruder::load (int16_t &addr, bool eeprom)
{
	motor.load (addr, eeprom);
	temp.load (addr, eeprom);
	filament_heat = read_float (addr, eeprom);
	nozzle_size = read_float (addr, eeprom);
	filament_size = read_float (addr, eeprom);
}

void Extruder::save (int16_t &addr, bool eeprom)
{
	motor.save (addr, eeprom);
	temp.save (addr, eeprom);
	write_float (addr, filament_heat, eeprom);
	write_float (addr, nozzle_size, eeprom);
	write_float (addr, filament_size, eeprom);
}

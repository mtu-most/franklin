#include "firmware.h"

#if MAXEXTRUDERS > 0
void Extruder::load(int16_t &addr, bool eeprom)
{
	motor.load(addr, eeprom);
	temp.load(addr, eeprom);
#ifndef LOWMEM
	filament_heat = read_32(addr, eeprom);
	nozzle_size = read_32(addr, eeprom);
	filament_size = read_32(addr, eeprom);
#else
	read_32(addr, eeprom);
	read_32(addr, eeprom);
	read_32(addr, eeprom);
#endif
}

void Extruder::save(int16_t &addr, bool eeprom)
{
	motor.save(addr, eeprom);
	temp.save(addr, eeprom);
#ifndef LOWMEM
	write_32(addr, filament_heat, eeprom);
	write_32(addr, nozzle_size, eeprom);
	write_32(addr, filament_size, eeprom);
#else
	write_32(addr, 0, eeprom);
	write_32(addr, 0, eeprom);
	write_32(addr, 0, eeprom);
#endif
}
#endif

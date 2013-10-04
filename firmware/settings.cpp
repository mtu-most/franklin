#include "firmware.h"

void Constants::load (uint16_t &addr, bool eeprom)
{
	// This function must not be called.
	debug ("Constants::load must not be called!");
}

void Constants::save (uint16_t &addr, bool eeprom)
{
	// This should never be called with eeprom true, but if it is, don't write to eeprom anyway.
	write_8 (addr, NAMELEN, false);
	write_8 (addr, MAXAXES, false);
	write_8 (addr, MAXEXTRUDERS, false);
	write_8 (addr, MAXTEMPS, false);
}

void Variables::load (uint16_t &addr, bool eeprom)
{
	for (uint8_t i = 0; i < NAMELEN; ++i)
		name[i] = read_8 (addr, eeprom);
	num_axes = read_8 (addr, eeprom);
	num_extruders = read_8 (addr, eeprom);
	num_temps = read_8 (addr, eeprom);
	printer_type = read_8 (addr, eeprom);
	led_pin = read_8 (addr, eeprom);
	room_T = read_float (addr, eeprom) + 273.15;
	motor_limit = read_32 (addr, eeprom);
	temp_limit = read_32 (addr, eeprom);
	// If settings are invalid values, the eeprom is probably not initialized; use defaults.
	if (num_axes > MAXAXES || num_extruders > MAXEXTRUDERS || num_temps > MAXTEMPS) {
		memset (name, 0, NAMELEN);
		num_axes = 3;
		num_extruders = 1;
		num_temps = 1;
		led_pin = 255;
		room_T = 20 + 273.15;
		motor_limit = 10 * 1000;
		temp_limit = (unsigned long)5 * 60 * 1000;
	}
	SET_OUTPUT (led_pin);
}

void Variables::save (uint16_t &addr, bool eeprom)
{
	for (uint8_t i = 0; i < NAMELEN; ++i)
		write_8 (addr, name[i], eeprom);
	write_8 (addr, num_axes, eeprom);
	write_8 (addr, num_extruders, eeprom);
	write_8 (addr, num_temps, eeprom);
	write_8 (addr, printer_type, eeprom);
	write_8 (addr, led_pin, eeprom);
	write_float (addr, room_T - 273.15, eeprom);
	write_32 (addr, motor_limit, eeprom);
	write_32 (addr, temp_limit, eeprom);
}
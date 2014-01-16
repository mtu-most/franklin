#include "firmware.h"

void Axis::load (int16_t &addr, bool eeprom)
{
	motor.load (addr, eeprom);
	limit_min_pin.read (read_16 (addr, eeprom));
	limit_max_pin.read (read_16 (addr, eeprom));
	limit_min_pos = read_32 (addr, eeprom);
	limit_max_pos = read_32 (addr, eeprom);
	delta_length = read_float (addr, eeprom);
	delta_radius = read_float (addr, eeprom);
	SET_INPUT (limit_min_pin);
	SET_INPUT (limit_max_pin);
#define sin120 0.8660254037844386	// .5*sqrt(3)
#define cos120 -.5
	// Coordinates of axes (at angles 0, 120, 240; each with its own radius).
	if (this == &axis[0]) {
		x = delta_radius;
		y = 0;
		z = sqrt (delta_length * delta_length - delta_radius * delta_radius);
	} else if (this == &axis[1]) {
		x = delta_radius * cos120;
		y = delta_radius * sin120;
		z = sqrt (delta_length * delta_length - delta_radius * delta_radius);
	} else if (this == &axis[2]) {
		x = delta_radius * cos120;
		y = delta_radius * -sin120;
		z = sqrt (delta_length * delta_length - delta_radius * delta_radius);
	}
}

void Axis::save (int16_t &addr, bool eeprom)
{
	motor.save (addr, eeprom);
	write_16 (addr, limit_min_pin.write (), eeprom);
	write_16 (addr, limit_max_pin.write (), eeprom);
	write_32 (addr, limit_min_pos, eeprom);
	write_32 (addr, limit_max_pos, eeprom);
	write_float (addr, delta_length, eeprom);
	write_float (addr, delta_radius, eeprom);
}

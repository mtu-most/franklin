#include "firmware.h"

#if MAXAXES > 0
void Axis::load(int16_t &addr, bool eeprom)
{
	motor.load(addr, eeprom);
	limit_min_pin.read(read_16(addr, eeprom));
	limit_max_pin.read(read_16(addr, eeprom));
	sense_pin.read(read_16(addr, eeprom));
	limit_pos = read_32(addr, eeprom);
	axis_min = read_32(addr, eeprom);
	axis_max = read_32(addr, eeprom);
	motor_min = read_32(addr, eeprom);
	motor_max = read_32(addr, eeprom);
	park = read_32(addr, eeprom);
	delta_length = read_32(addr, eeprom);
	delta_radius = read_32(addr, eeprom);
	offset = read_32(addr, eeprom);
	SET_INPUT(limit_min_pin);
	SET_INPUT(limit_max_pin);
	SET_INPUT(sense_pin);
	z = sqrt(delta_length * delta_length - delta_radius * delta_radius);
#ifndef LOWMEM
	compute_axes();
#endif
}

#ifndef LOWMEM
#define sin120 0.8660254037844386	// .5*sqrt(3)
#define cos120 -.5
void compute_axes()
{
#if MAXAXES >= 3
	// Coordinates of axes (at angles 0, 120, 240; each with its own radius).
	/* Originally u was on the x axis, but v on the y axis is more natural,
	   because the front should be as open as possible.
	   Putting v on the y axis makes sense for people who are confusing
	   uvw and xyz.  That's a very weak argument for this choice; my guess
	   is that others confuse them, and this is why other firmwares use
	   this convention.  It is used here because there is no "better" way,
	   so this firmware now behaves like the others.
	axis[0].x = axis[0].delta_radius;
	axis[0].y = 0;
	axis[1].x = axis[1].delta_radius * cos120;
	axis[1].y = axis[1].delta_radius * sin120;
	axis[2].x = axis[2].delta_radius * cos120;
	axis[2].y = axis[2].delta_radius * -sin120;
	*/
	float x[3], y[3];
	x[0] = axis[0].delta_radius * sin120;
	y[0] = axis[0].delta_radius * cos120;
	x[1] = 0;
	y[1] = axis[1].delta_radius;
	x[2] = axis[2].delta_radius * -sin120;
	y[2] = axis[2].delta_radius * cos120;
	for (uint8_t a = 0; a < 3; ++a) {
		axis[a].x = x[a] * cos(angle) - y[a] * sin(angle);
		axis[a].y = y[a] * cos(angle) + x[a] * sin(angle);
	}
#endif
}
#endif

void Axis::save(int16_t &addr, bool eeprom)
{
	motor.save(addr, eeprom);
	write_16(addr, limit_min_pin.write(), eeprom);
	write_16(addr, limit_max_pin.write(), eeprom);
	write_16(addr, sense_pin.write(), eeprom);
	write_32(addr, limit_pos, eeprom);
	write_32(addr, axis_min, eeprom);
	write_32(addr, axis_max, eeprom);
	write_32(addr, motor_min, eeprom);
	write_32(addr, motor_max, eeprom);
	write_32(addr, park, eeprom);
	write_32(addr, delta_length, eeprom);
	write_32(addr, delta_radius, eeprom);
	write_32(addr, offset, eeprom);
}
#endif

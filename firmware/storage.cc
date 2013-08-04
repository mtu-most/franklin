#include <EEPROM.h>
#include "firmware.hh"

uint8_t read_8 (uint16_t &address, bool eeprom)
{
	if (eeprom)
		return EEPROM.read (address++);
	return command[address++];
}

void write_8 (uint16_t &address, uint8_t data, bool eeprom)
{
	if (eeprom)
	{
		debug ("EEPROM[%x] = %x", address, data);
		EEPROM.write (address++, data);
		return;
	}
	reply[address++] = data;
}

uint16_t read_16 (uint16_t &address, bool eeprom)
{
	uint8_t l = read_8 (address, eeprom);
	uint8_t h = read_8 (address, eeprom);
	return (uint16_t (h) << 8) | uint16_t (l);
}

void write_16 (uint16_t &address, uint16_t data, bool eeprom)
{
	write_8 (address, data & 0xff, eeprom);
	write_8 (address, data >> 8, eeprom);
}

float read_float (uint16_t &address, bool eeprom)
{
	ReadFloat ret;
	for (uint8_t t = 0; t < sizeof (float); ++t)
		ret.b[t] = read_8 (address, eeprom);
	return ret.f;
}

void write_float (uint16_t &address, float data, bool eeprom)
{
	ReadFloat d;
	d.f = data;
	for (uint8_t t = 0; t < sizeof (float); ++t)
		write_8 (address, d.b[t], eeprom);
}

#include "firmware.h"

uint8_t read_8(int32_t &address, bool eeprom)
{
	if (eeprom)
		return read_eeprom(address++);
	return command[address++];
}

void write_8(int32_t &address, uint8_t data, bool eeprom)
{
	if (eeprom)
	{
		//debug("EEPROM[%x] = %x", address, data);
		write_eeprom(address++, data);
		watchdog_reset();
		return;
	}
	reply[address++] = data;
}

int16_t read_16(int32_t &address, bool eeprom)
{
	uint8_t l = read_8(address, eeprom);
	uint8_t h = read_8(address, eeprom);
	return ((uint16_t(h) & 0xff) << 8) | (uint16_t(l) & 0xff);
}

void write_16(int32_t &address, int16_t data, bool eeprom)
{
	write_8(address, data & 0xff, eeprom);
	write_8(address, (data >> 8) & 0xff, eeprom);
}

float read_float(int32_t &address, bool eeprom)
{
	ReadFloat ret;
	for (uint8_t t = 0; t < sizeof(float); ++t)
		ret.b[t] = read_8(address, eeprom);
	return ret.f;
}

void write_float(int32_t &address, float data, bool eeprom)
{
	ReadFloat d;
	d.f = data;
	for (uint8_t t = 0; t < sizeof(float); ++t)
		write_8(address, d.b[t], eeprom);
}

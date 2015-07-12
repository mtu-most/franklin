#include "cdriver.h"

uint8_t read_8(int32_t &address)
{
	return command[0][address++];
}

void write_8(int32_t &address, uint8_t data)
{
	datastore[address++] = data;
}

int16_t read_16(int32_t &address)
{
	uint8_t l = read_8(address);
	uint8_t h = read_8(address);
	return ((uint16_t(h) & 0xff) << 8) | (uint16_t(l) & 0xff);
}

void write_16(int32_t &address, int16_t data)
{
	write_8(address, data & 0xff);
	write_8(address, (data >> 8) & 0xff);
}

double read_float(int32_t &address)
{
	ReadFloat ret;
	for (uint8_t t = 0; t < sizeof(double); ++t)
		ret.b[t] = read_8(address);
	return ret.f;
}

void write_float(int32_t &address, double data)
{
	ReadFloat d;
	d.f = data;
	for (uint8_t t = 0; t < sizeof(double); ++t)
		write_8(address, d.b[t]);
}

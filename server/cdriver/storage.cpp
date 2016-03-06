/* storage.cpp - write and read elements into and from streams for Franklin
 * Copyright 2014 Michigan Technological University
 * Author: Bas Wijnen <wijnen@debian.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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

/* gpio.cpp - Gpio handling for Franklin
 * Copyright 2014-2016 Michigan Technological University
 * Copyright 2016 Bas Wijnen <wijnen@debian.org>
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

void Gpio::load(int32_t &addr)
{
	pin.read(read_16(addr));
	state = read_8(addr);
	reset = (state >> 2) & 0x3;
	state &= 0x3;
	// State:  0: off, 1: on, 2: pullup input, 3: disabled
	switch (state) {
	case 0:
		SET_OUTPUT(pin);
		RESET(pin);
		break;
	case 1:
		SET_OUTPUT(pin);
		SET(pin);
		break;
	case 2:
		SET_INPUT(pin);
		break;
	case 3:
		SET_INPUT_NOPULLUP(pin);
		break;
	}
#ifdef SERIAL
	arch_pin_set_reset(pin, reset);
#endif
	double duty = read_float(addr);
	if (pin.valid())
		arch_set_duty(pin, duty);
}

void Gpio::save(int32_t &addr)
{
	write_16(addr, pin.write());
	write_8(addr, state | (reset << 2));
	write_float(addr, pin.valid() ? arch_get_duty(pin) : 1);
}

void Gpio::init() {
	pin.init();
	state = 3;
	reset = 3;
}

void Gpio::free() {
	pin.read(0);
}

void Gpio::copy(Gpio &dst) {
	dst.pin.read(pin.write());
	dst.state = state;
}

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

void Gpio::load() {
	if (pin.valid() && pin.write() != shmem->ints[1]) {
		arch_set_duty(pin, 1);
		arch_set_pin_motor(pin, -1, -1, 1);
	}
	pin.read(shmem->ints[1]);
	state = shmem->ints[2];
	leader = shmem->ints[3];
	ticks = shmem->ints[4];
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
	duty = shmem->floats[0];
	if (pin.valid()) {
		arch_set_duty(pin, duty);
		arch_set_pin_motor(pin, leader & 0xf, leader >> 4, ticks);
	}
}

void Gpio::save() {
	shmem->ints[1] = pin.write();
	shmem->ints[2] = state | (reset << 2);
	shmem->ints[3] = leader;
	shmem->ints[4] = ticks;
	shmem->floats[0] = duty;
}

void Gpio::init() {
	pin.init();
	state = 3;
	reset = 3;
	leader = -1;
	ticks = 1;
	duty = 1;
	changed = false;
	value = false;
}

void Gpio::free() {
	pin.read(0);
}

void Gpio::copy(Gpio &dst) {
	dst.pin.read(pin.write());
	dst.state = state;
	dst.duty = duty;
	dst.leader = leader;
	dst.ticks = ticks;
}

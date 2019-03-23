/* globals.cpp - Global settings handling for Franklin
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

#if 0
#define ldebug debug
#else
#define ldebug(...) do {} while(0)
#endif

bool globals_load() {
	bool change_hw = false;
	int nt = shmem->ints[2];
	int ng = shmem->ints[3];
	// Free the old memory and initialize the new memory.
	ldebug("num temps %d->%d", num_temps, nt);
	if (nt != num_temps) {
		for (uint8_t t = nt; t < num_temps; ++t)
			temps[t].free();
		Temp *new_temps = new Temp[nt];
		for (uint8_t t = 0; t < min(nt, num_temps); ++t)
			temps[t].copy(new_temps[t]);
		for (uint8_t t = num_temps; t < nt; ++t)
			new_temps[t].init();
		delete[] temps;
		temps = new_temps;
		num_temps = nt;
	}
	if (ng != num_gpios) {
		for (uint8_t g = ng; g < num_gpios; ++g)
			gpios[g].free();
		Gpio *new_gpios = new Gpio[ng];
		for (uint8_t g = 0; g < min(ng, num_gpios); ++g)
			gpios[g].copy(new_gpios[g]);
		for (uint8_t g = num_gpios; g < ng; ++g)
			new_gpios[g].init();
		delete[] gpios;
		gpios = new_gpios;
		num_gpios = ng;
	}
	ldebug("new done");
	int p = led_pin.write();
	led_pin.read(shmem->ints[4]);
	if (p != led_pin.write())
		change_hw = true;
	p = stop_pin.write();
	stop_pin.read(shmem->ints[5]);
	if (p != stop_pin.write())
		change_hw = true;
	p = probe_pin.write();
	probe_pin.read(shmem->ints[6]);
	if (p != probe_pin.write())
		change_hw = true;
	p = spiss_pin.write();
	spiss_pin.read(shmem->ints[7]);
	if (p != spiss_pin.write())
		change_hw = true;
	int t = timeout;
	timeout = shmem->ints[8];
	if (t != timeout)
		change_hw = true;
	bed_id = shmem->ints[9];
	fan_id = shmem->ints[10];
	spindle_id = shmem->ints[11];
	feedrate = shmem->floats[0];
	if (std::isnan(feedrate) || std::isinf(feedrate) || feedrate <= 0)
		feedrate = 1;
	max_deviation = shmem->floats[1];
	max_v = shmem->floats[2];
	max_a = shmem->floats[3];
	current_extruder = shmem->ints[12];
	targetx = shmem->floats[4];
	targety = shmem->floats[5];
	targetangle = shmem->floats[6];
	zoffset = shmem->floats[7];
	bool store = shmem->ints[13];
	if (store && !store_adc) {
		store_adc = fopen("/tmp/franklin-adc-dump", "a");
	}
	else if (!store && store_adc) {
		fclose(store_adc);
		store_adc = NULL;
	}
	ldebug("all done");
	if (change_hw)
		arch_motors_change();
	return true;
}

void globals_save() {
	shmem->ints[0] = QUEUE_LENGTH;
	shmem->ints[1] = NUM_PINS;
	shmem->ints[2] = num_temps;
	shmem->ints[3] = num_gpios;
	shmem->ints[4] = led_pin.write();
	shmem->ints[5] = stop_pin.write();
	shmem->ints[6] = probe_pin.write();
	shmem->ints[7] = spiss_pin.write();
	shmem->ints[8] = timeout;
	shmem->ints[9] = bed_id;
	shmem->ints[10] = fan_id;
	shmem->ints[11] = spindle_id;
	shmem->ints[12] = current_extruder;
	shmem->ints[13] = store_adc != NULL;
	shmem->floats[0] = feedrate;
	shmem->floats[1] = max_deviation;
	shmem->floats[2] = max_v;
	shmem->floats[3] = max_a;
	shmem->floats[4] = targetx;
	shmem->floats[5] = targety;
	shmem->floats[6] = targetangle;
	shmem->floats[7] = zoffset;
}

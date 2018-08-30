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
	if (nt != num_temps) {
		ldebug("new temp");
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
	int ce = shmem->ints[12];
	targetx = shmem->floats[4];
	targety = shmem->floats[5];
	targetangle = shmem->floats[6];
	double zo = shmem->floats[7];
	/*if (motors_busy && (current_extruder != ce || zoffset != zo) && settings.queue_start == settings.queue_end && !settings.queue_full && !computing_move) {
		// FIXME: move to current position
		queue[settings.queue_end].probe = false;
		queue[settings.queue_end].cb = false;
		queue[settings.queue_end].f[0] = INFINITY;
		queue[settings.queue_end].f[1] = INFINITY;
		for (int i = 0; i < spaces[0].num_axes; ++i) {
			queue[settings.queue_end].data[i] = spaces[0].axis[i]->settings.current - (i == 2 ? zoffset : 0);
			for (int s = 0; s < NUM_SPACES; ++s)
				queue[settings.queue_end].data[i] = space_types[spaces[s].type].unchange0(&spaces[s], i, queue[settings.queue_end].data[i]);
		}
		for (int i = spaces[0].num_axes; i < 10; ++i) { // TODO: Make 10 a dynamic size.
			queue[settings.queue_end].data[i] = NAN;
		}
		settings.queue_end = (settings.queue_end + 1) % QUEUE_LENGTH;
		// This shouldn't happen and causes communication problems, but if you have a 1-item buffer it is correct.
		if (settings.queue_end == settings.queue_start)
			settings.queue_full = true;
		current_extruder = ce;
		zoffset = zo;
		next_move();
		buffer_refill();
	}
	else */ {
		current_extruder = ce;
		zoffset = zo;
	}
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

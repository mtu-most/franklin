/* globals.cpp - Global settings handling for Franklin
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

#if 0
#define ldebug debug
#else
#define ldebug(...) do {} while(0)
#endif

bool globals_load(int32_t &addr)
{
	bool change_hw = false;
	uint8_t nt = read_8(addr);
	uint8_t ng = read_8(addr);
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
	led_pin.read(read_16(addr));
	if (p != led_pin.write())
		change_hw = true;
	p = stop_pin.write();
	stop_pin.read(read_16(addr));
	if (p != stop_pin.write())
		change_hw = true;
	p = probe_pin.write();
	probe_pin.read(read_16(addr));
	if (p != probe_pin.write())
		change_hw = true;
	p = spiss_pin.write();
	spiss_pin.read(read_16(addr));
	if (p != spiss_pin.write())
		change_hw = true;
	int t = timeout;
	timeout = read_16(addr);
	if (t != timeout)
		change_hw = true;
	bed_id = read_16(addr);
	fan_id = read_16(addr);
	spindle_id = read_16(addr);
	feedrate = read_float(addr);
	if (isnan(feedrate) || isinf(feedrate) || feedrate <= 0)
		feedrate = 1;
	max_deviation = read_float(addr);
	max_v = read_float(addr);
	int ce = read_8(addr);
	targetx = read_float(addr);
	targety = read_float(addr);
	double zo = read_float(addr);
	if (motors_busy && (current_extruder != ce || zoffset != zo) && settings.queue_start == settings.queue_end && !settings.queue_full && !computing_move) {
		queue[settings.queue_end].probe = false;
		queue[settings.queue_end].cb = false;
		queue[settings.queue_end].f[0] = INFINITY;
		queue[settings.queue_end].f[1] = INFINITY;
		for (int i = 0; i < spaces[0].num_axes; ++i) {
			queue[settings.queue_end].data[i] = spaces[0].axis[i]->settings.current - (i == 2 ? zoffset : 0);
			for (int s = 0; s < NUM_SPACES; ++s)
				queue[settings.queue_end].data[i] = space_types[spaces[s].type].unchange0(&spaces[s], i, queue[settings.queue_end].data[i]);
		}
		for (int i = spaces[0].num_axes; i < QUEUE_LENGTH; ++i) {
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
	else {
		current_extruder = ce;
		zoffset = zo;
	}
	bool store = read_8(addr);
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

void globals_save(int32_t &addr)
{
	write_8(addr, QUEUE_LENGTH);
	write_8(addr, NUM_PINS);
	write_8(addr, num_temps);
	write_8(addr, num_gpios);
	write_16(addr, led_pin.write());
	write_16(addr, stop_pin.write());
	write_16(addr, probe_pin.write());
	write_16(addr, spiss_pin.write());
	write_16(addr, timeout);
	write_16(addr, bed_id);
	write_16(addr, fan_id);
	write_16(addr, spindle_id);
	write_float(addr, feedrate);
	write_float(addr, max_deviation);
	write_float(addr, max_v);
	write_8(addr, current_extruder);
	write_float(addr, targetx);
	write_float(addr, targety);
	write_float(addr, zoffset);
	write_8(addr, store_adc != NULL);
}

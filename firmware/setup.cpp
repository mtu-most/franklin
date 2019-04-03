/* setup.cpp - initialization for Franklin
 * vim: set foldmethod=marker :
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

#define EXTERN	// This must be done in exactly one source file.
#include "firmware.h"

void setup()
{
	serial_buffer_head = serial_buffer;
	serial_buffer_tail = serial_buffer;
	serial_overflow = false;
	debug_value = 0x1337;
	arch_setup_start();
	enabled_pins = NUM_DIGITAL_PINS;
	for (uint8_t p = 0; p < NUM_DIGITAL_PINS; ++p) {
		pin[p].duty = 255;
		// Reset state is unset, then unset the pin.
		pin[p].state = CTRL_UNSET << 2 | CTRL_RESET;
		UNSET(p);
	}
	pin_events = 0;
	notified_current_fragment = 0;
	current_fragment = notified_current_fragment;
	last_fragment = current_fragment;
	filling = 0;
	// Disable all adcs.
	for (uint8_t a = 0; a < NUM_ANALOG_INPUTS; ++a) {
		for (uint8_t i = 0; i < 2; ++i) {
			adc[a].linked[i] = ~0;
			adc[a].value[i] = 1 << 15;
			adc[a].is_on[i] = false;
		}
	}
	adc_phase = INACTIVE;
	adc_current = ~0;
	adc_next = ~0;
	// Set up communication state.
	command_end = 0;
	had_data = false;
	ping = 0;
	out_busy = 0;
	ff_in = 0;
	ff_out = 0;
	reply_ready = 0;
	adcreply_ready = 0;
	timeout = false;
	timeout_time = 0;
	// Set up homing state.
	homers = 0;
	home_step_time = 0;
	// Set up movement state.
	last_len = 0;
	stopping = -1;
	arch_set_speed(0);
	current_len = 0;
	active_motors = 0;
	move_phase = 0;
	full_phase = 1;
	// Set up led state.
	led_fast = 0;
	led_last = millis();
	led_phase = 0;
	led_pin = ~0;
	stop_pin = ~0;
	probe_pin = ~0;
	spiss_pin = ~0;
	// Do arch-specific things.  This fills machineid and uuid.
	arch_setup_end();
	// Inform host of reset.
	send_id(CMD_STARTUP);
}

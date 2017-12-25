/* timer.cpp - timing related parts for Franklin
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

#include "firmware.h"

void handle_motors() {
	cli();
	uint8_t state = step_state;
	uint8_t cf = current_fragment;
	uint8_t cs = current_sample;
	sei();
	if (state == STEP_STATE_STOP)
		return;
	last_active = seconds();
	// Check probe.
	bool probed;
	if (settings[cf].flags & Settings::PROBING && probe_pin < NUM_DIGITAL_PINS) {
		if (state == STEP_STATE_RUN || state == STEP_STATE_NEXT || state == STEP_STATE_WAIT) {
			// Probing, but state is set to run; fix that.
			cli();
			step_state = STEP_STATE_PROBE;
			// Update other variables which may have been changed by the ISR.
			state = step_state;
			cf = current_fragment;
			cs = current_sample;
			sei();
		}
		if (state == STEP_STATE_PROBE) {
			if (GET(probe_pin) ^ bool(pin_flags & 2)) {
				step_state = STEP_STATE_STOP;
				stopping = active_motors;
			}
			probed = true;
		}
		else {
			probed = false;
		}
	}
	else
		probed = true;	// If we didn't need to probe; don't block later on.
	if (stopping < 0) {
		if (stop_pin < NUM_DIGITAL_PINS && GET(stop_pin) ^ bool(pin_flags & 4)) {
			step_state = STEP_STATE_STOP;
			stopping = active_motors;
		}
		else {
			// Check sensors.
			for (uint8_t m = 0; m < active_motors; ++m) {
				if (!(motor[m].intflags & Motor::ACTIVE))
					continue;
				// Check limit switches.
				if (stopping < 0) {
					int16_t value = *reinterpret_cast <volatile int16_t *>(&buffer[cf][m][cs]);
					if (value == 0)
						continue;
					uint8_t limit_pin = value < 0 ? motor[m].limit_min_pin : motor[m].limit_max_pin;
					if (limit_pin < NUM_DIGITAL_PINS) {
						bool inverted = motor[m].flags & (value < 0 ? Motor::INVERT_LIMIT_MIN : Motor::INVERT_LIMIT_MAX);
						if (GET(limit_pin) ^ inverted) {
							step_state = STEP_STATE_STOP;
							//debug("hit %d pin %d pos %d state %d sample %d", m, limit_pin, int(motor[m].current_pos), state, current_sample);
							stopping = m;
							motor[m].flags |= Motor::LIMIT;
							break;
						}
					}
				}
			}
		}
	}
	if (stopping >= 0) {
		// Notify host.
		// Use actual current sample, not the one that was used for testing.
		limit_fragment_pos = current_sample;
		arch_set_speed(0);
		return;
	}
	if (homers > 0) {
		// Homing.
		if (state == STEP_STATE_PROBE) {
			probed = true;
			for (uint8_t m = 0; m < active_motors; ++m) {
				if (!(motor[m].intflags & Motor::ACTIVE))
					continue;
				// Get the "wrong" limit pin for the given direction.
				int16_t value = *reinterpret_cast <volatile int16_t *>(&buffer[cf][m][cs]);
				uint8_t limit_pin = (value < 0 ? motor[m].limit_max_pin : motor[m].limit_min_pin);
				bool inverted = motor[m].flags & (value < 0 ? Motor::INVERT_LIMIT_MAX : Motor::INVERT_LIMIT_MIN);
				if (limit_pin < NUM_DIGITAL_PINS && GET(limit_pin) ^ inverted) {
					// Limit pin still triggered; continue moving.
					continue;
				}
				// Limit pin no longer triggered.  Stop moving and possibly notify host.
				motor[m].intflags &= ~Motor::ACTIVE;
				if (!--homers) {
					arch_set_speed(0);
					return;
				}
			}
		}
		else
			probed = false;
	}
	if (state == STEP_STATE_PROBE && probed) {
		if (homers > 0)
			current_sample = 0;	// Use only the first sample for homing.
		uint8_t new_state = homers > 0 || (settings[cf].flags & Settings::PROBING) ? STEP_STATE_SINGLE : STEP_STATE_RUN;
		//debug("step_state non zero %d (fragment %d)", new_state, cf);
		step_state = new_state;
	}
	cli();
	if (step_state == STEP_STATE_NEXT || step_state == STEP_STATE_WAIT)
		step_state = STEP_STATE_RUN;
	sei();
}

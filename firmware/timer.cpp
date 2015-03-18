#include "firmware.h"

void handle_motors() {
	if (steps_prepared >= 2)
		return;
	if (underrun) {
		if (steps_prepared == 0)
			set_speed(0);
		return;
	}
	last_active = seconds();
	/*
	arch_cli();
	uint16_t debug_tmp = debug_value1;
	arch_sei();
	*/
	// Check sensors.
	for (uint8_t m = 0; m < active_motors; ++m) {
		Fragment &fragment = motor[m].buffer[current_fragment];
		if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
			continue;
		// Check sense pins.
		if (motor[m].sense_pin < NUM_DIGITAL_PINS) {
			if (GET(motor[m].sense_pin) ^ bool(motor[m].flags & Motor::SENSE_STATE)) {
				//debug("sense %d %x", m, motor[m].flags);
				motor[m].flags ^= Motor::SENSE_STATE;
				motor[m].flags |= (motor[m].flags & Motor::SENSE_STATE ? Motor::SENSE1 : Motor::SENSE0);
				arch_record_sense(motor[m].flags & Motor::SENSE_STATE);
			}
		}
		// Check probe.
		bool hit = false;
		if (settings[current_fragment].probing && probe_pin < NUM_DIGITAL_PINS && (GET(probe_pin) ^ bool(pin_flags & 2))) {
			// Hit probe.
			hit = true;
		}
		else {
		// Check limit switches.
			uint8_t limit_pin = (fragment.dir ? motor[m].limit_min_pin : motor[m].limit_max_pin);
			if (limit_pin < NUM_DIGITAL_PINS) {
				bool inverted = motor[m].flags & (fragment.dir ? Motor::INVERT_LIMIT_MIN : Motor::INVERT_LIMIT_MAX);
				if (GET(limit_pin) ^ inverted)
					hit = true;
			}
		}
		if (hit) {
			// Hit endstop.
			steps_prepared = 0;
			//debug("hit limit %d curpos %ld dir %d cf %d ncf %d lf %d cfp %d", m, F(motor[m].current_pos), fragment.dir, current_fragment, notified_current_fragment, last_fragment, current_fragment_pos);
			// Notify host.
			motor[m].flags |= Motor::LIMIT;
			limit_fragment_pos = current_fragment_pos;
			current_fragment_pos = 0;
			set_speed(0);
			filling = 0;
			last_fragment = current_fragment;
			notified_current_fragment = current_fragment;
			stopping = true;
			for (uint8_t mc = 0; mc < active_motors; ++mc)
				motor[mc].steps_current = 0;
			return;
		}
	}
	// Move motors.
	if (homers > 0) {
		// Homing.
		for (uint8_t m = 0; m < active_motors; ++m) {
			Fragment &fragment = motor[m].buffer[current_fragment];
			if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
				continue;
			// Get twe "wrong" limit pin for the given direction.
			uint8_t limit_pin = (fragment.dir ? motor[m].limit_max_pin : motor[m].limit_min_pin);
			bool inverted = motor[m].flags & (fragment.dir ? Motor::INVERT_LIMIT_MAX : Motor::INVERT_LIMIT_MIN);
			if (limit_pin >= NUM_DIGITAL_PINS || GET(limit_pin) ^ inverted) {
				// Limit pin still triggered; continue moving.
				continue;
			}
			// Limit pin no longer triggered.  Stop moving and possibly notify host.
			motor[m].dir = DIR_NONE;
			motor[m].next_dir = DIR_NONE;
			fragment.dir = DIR_NONE;
			if (!--homers)
				set_speed(0);
		}
	}
	else {
		// Regular move.  (Not homing.)
		// Move to next buffer.
		while (!underrun && current_fragment_pos >= settings[current_fragment].len) {
			current_fragment_pos -= settings[current_fragment].len;
			uint8_t new_current_fragment = (current_fragment + 1) % FRAGMENTS_PER_MOTOR;
			if (last_fragment == new_current_fragment) {
				// Underrun.
				//debug("underrun for %d", last_fragment);
				underrun = true;
				for (uint8_t m = 0; m < active_motors; ++m)
					motor[m].next_next_steps = 0;
			}
			else {
				for (uint8_t m = 0; m < active_motors; ++m) {
					Fragment &fragment = motor[m].buffer[new_current_fragment];
					motor[m].next_dir = fragment.dir;
				}
			}
			current_fragment = new_current_fragment;
		}
		if (!underrun) {
			for (uint8_t m = 0; m < active_motors; ++m) {
				Fragment &fragment = motor[m].buffer[current_fragment];
				if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
					continue;
				motor[m].next_next_steps = fragment.samples[current_fragment_pos];
			}
			current_fragment_pos += 1;
		}
	}
	if (steps_prepared == 0) {
		move_phase = 0;
		for (uint8_t m = 0; m < active_motors; ++m) {
			motor[m].steps_current = 0;
			Fragment &fragment = motor[m].buffer[current_fragment];
			if (homers == 0) {
				if (fragment.dir != DIR_NONE && fragment.dir != DIR_AUDIO) {
					motor[m].next_steps = motor[m].next_next_steps;
					motor[m].next_next_steps = 0;
				}
				motor[m].dir = motor[m].next_dir;
			}
		}
	}
	if (!underrun) {
		arch_cli();
		steps_prepared += 1;
		arch_sei();
	}
}

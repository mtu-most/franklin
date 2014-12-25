#include "firmware.h"

void do_steps() {
	static bool lock = false;
	// Only move if the move was prepared.
	if (lock || !steps_prepared)
		return;
	lock = true;
	// Enable interrupts as soon as possible, so the uart doesn't overrun.
	sei();
	move_phase += 1;
	for (uint8_t m = 0; m < active_motors; ++m) {
		if (motor[m].dir != DIR_POSITIVE && motor[m].dir != DIR_NEGATIVE)
			continue;
		uint8_t steps_target = uint32_t(motor[m].next_steps) * move_phase / full_phase - motor[m].steps_current;
		if (steps_target == 0)
			continue;
		if (motor[m].dir_pin < NUM_DIGITAL_PINS) {
			if (motor[m].dir == DIR_POSITIVE)
				SET(motor[m].dir_pin);
			else
				RESET(motor[m].dir_pin);
		}
		if (motor[m].step_pin < NUM_DIGITAL_PINS) {
			for (uint8_t i = 0; i < steps_target; ++i) {
				SET(motor[m].step_pin);
				RESET(motor[m].step_pin);
			}
		}
		motor[m].current_pos += (motor[m].dir == DIR_POSITIVE ? steps_target : -steps_target);
		motor[m].steps_current += steps_target;
	}
	if (move_phase >= full_phase) {
		move_phase = 0;
		for (uint8_t m = 0; m < active_motors; ++m) {
			motor[m].steps_current = 0;
			motor[m].next_steps = motor[m].next_next_steps;
		}
		steps_prepared -= 1;
	}
	lock = false;
}

void handle_motors() {
	// Check sensors.
	for (uint8_t m = 0; m < active_motors; ++m) {
		Fragment &fragment = buffer[motor[m].buffer][current_fragment];
		if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
			continue;
		// Check sense pins.
		if (motor[m].sense_pin < NUM_DIGITAL_PINS) {
			if (GET(motor[m].sense_pin) ^ bool(motor[m].flags & Motor::SENSE_STATE)) {
				//debug("sense %d %x", m, motor[m].flags);
				motor[m].flags ^= Motor::SENSE_STATE;
				motor[m].flags |= (motor[m].flags & Motor::SENSE_STATE ? Motor::SENSE1 : Motor::SENSE0);
				cli();
				for (int mi = 0; mi < active_motors; ++mi)
					motor[mi].sense_pos[(motor[m].flags & Motor::SENSE_STATE) ? 1 : 0] = motor[mi].current_pos;
				sei();
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
			debug("hit limit %d curpos %ld dir %d frag %d;%d;%d;%d", m, F(motor[m].current_pos), fragment.dir, current_fragment, notified_current_fragment, last_fragment, current_fragment_pos);
			// Notify host.
			motor[m].flags |= Motor::LIMIT;
			limit_fragment_pos = current_fragment_pos;
			current_fragment_pos = 0;
			set_speed(0);
			filling = 0;
			current_fragment = (last_fragment + 1) % FRAGMENTS_PER_BUFFER;
			notified_current_fragment = current_fragment;
			stopping = true;
			move_phase = 0;
			for (uint8_t mc = 0; mc < active_motors; ++mc)
				motor[mc].steps_current = 0;
			return;
		}
	}
	// Move motors.
	if (homers > 0) {
		// Homing.
		for (uint8_t m = 0; m < active_motors; ++m) {
			Fragment &fragment = buffer[motor[m].buffer][current_fragment];
			if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
				continue;
			// Get twe "wrong" limit pin for the given direction.
			uint8_t limit_pin = (fragment.dir ? motor[m].limit_max_pin : motor[m].limit_min_pin);
			bool inverted = motor[m].flags & (fragment.dir ? Motor::INVERT_LIMIT_MAX : Motor::INVERT_LIMIT_MIN);
			if (GET(limit_pin) ^ inverted) {
				// Limit pin still triggered; continue moving.
				continue;
			}
			// Limit pin no longer triggered.  Stop moving and possibly notify host.
			motor[m].dir = DIR_NONE;
			fragment.dir = DIR_NONE;
			if (!--homers)
				set_speed(0);
		}
	}
	else {
		// Regular move.  (Not homing.)
		// Move to next buffer.
		while (!stopped && current_fragment_pos >= settings[current_fragment].len) {
			current_fragment_pos -= settings[current_fragment].len;
			uint8_t new_current_fragment = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
			if (last_fragment == (filling > 0 ? new_current_fragment : current_fragment)) {
				// Underrun.
				set_speed(0);
				underrun = true;
			}
			else {
				for (uint8_t m = 0; m < active_motors; ++m) {
					Fragment &fragment = buffer[motor[m].buffer][new_current_fragment];
					motor[m].dir = fragment.dir;
				}
			}
			current_fragment = new_current_fragment;
			//debug("new fragment: %d", current_fragment);
		}
		if (!stopped) {
			for (uint8_t m = 0; m < active_motors; ++m) {
				Fragment &fragment = buffer[motor[m].buffer][current_fragment];
				if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
					continue;
				motor[m].next_next_steps = fragment.samples[current_fragment_pos];
			}
			current_fragment_pos += 1;
		}
	}
	if (!stopped)
		steps_prepared += 1;
}

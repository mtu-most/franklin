#include "firmware.h"

void do_steps() {
	static bool lock = false;
	// Only move if the move was prepared.
	if (lock || !steps_prepared) {
		arch_sei();
		return;
	}
	lock = true;
	// Enable interrupts as soon as possible, so the uart doesn't overrun.
	arch_sei();
	move_phase += 1;
	for (uint8_t m = 0; m < active_motors; ++m) {
		if (motor[m].dir != DIR_POSITIVE && motor[m].dir != DIR_NEGATIVE)
			continue;
		uint8_t steps_target = uint32_t(motor[m].next_steps) * move_phase / full_phase - motor[m].steps_current;
		//debug("stepping %d %d %d %d %d %d %d", m, steps_target, move_phase, full_phase, motor[m].steps_current, motor[m].next_steps, motor[m].next_next_steps);
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
		arch_cli();
		motor[m].current_pos += (motor[m].dir == DIR_POSITIVE ? steps_target : -steps_target);
		motor[m].steps_current += steps_target;
		//if (m == 0)
		//	debug_value += steps_target;
		arch_sei();
	}
	if (move_phase >= full_phase) {
		for (uint8_t m = 0; m < active_motors; ++m) {
			if (motor[m].dir != DIR_POSITIVE && motor[m].dir != DIR_NEGATIVE)
				continue;
			if (motor[m].steps_current != motor[m].next_steps) {
				debug("Problem %d: %d != %d (%d %d)", m, motor[m].steps_current, motor[m].next_steps, move_phase, full_phase);
			}
		}
		move_phase = 0;
		for (uint8_t m = 0; m < active_motors; ++m) {
			motor[m].steps_current = 0;
			if (steps_prepared == 2) {
				motor[m].next_steps = motor[m].next_next_steps;
				//debug_value1 += debug_value;
				//debug_value = 0;
				motor[m].dir = motor[m].next_dir;
			}
		}
		if (steps_prepared == 1 && underrun)
			set_speed(0);
		arch_cli();
		steps_prepared -= 1;
		arch_sei();
	}
	lock = false;
}

void handle_motors() {
	/*
	arch_cli();
	uint16_t debug_tmp = debug_value1;
	arch_sei();
	*/
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
			debug("hit limit %d curpos %ld dir %d cf %d ncf %d lf %d cfp %d", m, F(motor[m].current_pos), fragment.dir, current_fragment, notified_current_fragment, last_fragment, current_fragment_pos);
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
			uint8_t new_current_fragment = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
			if (last_fragment == new_current_fragment) {
				// Underrun.
				//debug("underrun for %d", last_fragment);
				underrun = true;
				for (uint8_t m = 0; m < active_motors; ++m)
					motor[m].next_next_steps = 0;
			}
			else {
				for (uint8_t m = 0; m < active_motors; ++m) {
					Fragment &fragment = buffer[motor[m].buffer][new_current_fragment];
					motor[m].next_dir = fragment.dir;
				}
			}
			/*
			debug_add(debug_tmp);
			arch_cli();
			debug_value1 -= debug_tmp;
			arch_sei();
			*/
			current_fragment = new_current_fragment;
		}
		if (!underrun) {
			for (uint8_t m = 0; m < active_motors; ++m) {
				Fragment &fragment = buffer[motor[m].buffer][current_fragment];
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
			Fragment &fragment = buffer[motor[m].buffer][current_fragment];
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

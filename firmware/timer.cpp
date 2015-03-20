#include "firmware.h"

void handle_motors() {
	if (step_state == 1) {
		set_speed(0);
		return;
	}
	last_active = seconds();
	// Check sensors.
	for (uint8_t m = 0; m < active_motors; ++m) {
		if (!(motor[m].flags & Motor::ACTIVE))
			continue;
		//debug("check %d", m);
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
			uint8_t limit_pin = buffer[current_fragment][m][current_sample] < 0 ? motor[m].limit_min_pin : motor[m].limit_max_pin;
			if (limit_pin < NUM_DIGITAL_PINS) {
				bool inverted = motor[m].flags & (buffer[current_fragment][m][current_sample] < 0 ? Motor::INVERT_LIMIT_MIN : Motor::INVERT_LIMIT_MAX);
				if (GET(limit_pin) ^ inverted) {
					debug("hit %d %d", step_state, buffer[current_fragment][m][current_sample]);
					hit = true;
				}
			}
		}
		if (hit) {
			// Hit endstop.
			step_state = 1;
			debug("hit limit %d curpos %ld cf %d ncf %d lf %d cfp %d", m, F(motor[m].current_pos), current_fragment, notified_current_fragment, last_fragment, current_sample);
			// Notify host.
			motor[m].flags |= Motor::LIMIT;
			limit_fragment_pos = current_sample;
			current_sample = 0;
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
	if (homers > 0) {
		// Homing.
		for (uint8_t m = 0; m < active_motors; ++m) {
			if (!(motor[m].flags & Motor::ACTIVE))
				continue;
			// Get twe "wrong" limit pin for the given direction.
			uint8_t limit_pin = (buffer[current_fragment][m][current_sample] < 0 ? motor[m].limit_max_pin : motor[m].limit_min_pin);
			bool inverted = motor[m].flags & (buffer[current_fragment][m][current_sample] < 0 ? Motor::INVERT_LIMIT_MAX : Motor::INVERT_LIMIT_MIN);
			if (limit_pin >= NUM_DIGITAL_PINS || GET(limit_pin) ^ inverted) {
				// Limit pin still triggered; continue moving.
				continue;
			}
			// Limit pin no longer triggered.  Stop moving and possibly notify host.
			motor[m].flags &= ~Motor::ACTIVE;
			if (!--homers)
				set_speed(0);
		}
	}
	if (step_state == 0) {
		if (homers > 0)
			current_sample = 0;	// Use only the first sample for homing.
		step_state = homers > 0 || settings[current_fragment].probing ? 2 : 3;
	}
}

#include "firmware.h"

void handle_motors() {
	if (step_state == 1)
		return;
	last_active = seconds();
	cli();
	uint8_t state = step_state;
	uint8_t cf = current_fragment;
	uint8_t cs = current_sample;
	sei();
	// Check probe.
	bool probed;
	if (settings[cf].flags & Settings::PROBING && probe_pin < NUM_DIGITAL_PINS) {
		if (state == 0) {
			if (GET(probe_pin) ^ bool(pin_flags & 2))
				stopping = active_motors;
			probed = true;
		}
		else
			probed = false;
	}
	else
		probed = true;	// If we didn't need to probe; don't block later on.
	if (stopping < 0) {
		if (stop_pin < NUM_DIGITAL_PINS && GET(stop_pin) ^ bool(pin_flags & 4)) {
			stopping = active_motors;
		}
		else {
			// Check sensors.
			for (uint8_t m = 0; m < active_motors; ++m) {
				if (!(motor[m].intflags & Motor::ACTIVE))
					continue;
				//debug("check %d", m);
				// Check sense pins.
				if (motor[m].sense_pin < NUM_DIGITAL_PINS) {
					if (GET(motor[m].sense_pin) ^ bool(motor[m].flags & Motor::SENSE_STATE)) {
						//debug("sense %d %x", m, motor[m].flags);
						motor[m].flags ^= Motor::SENSE_STATE;
						motor[m].flags |= (motor[m].flags & Motor::SENSE_STATE ? Motor::SENSE1 : Motor::SENSE0);
						uint8_t sense_state = motor[m].flags & Motor::SENSE_STATE ? 1 : 0;
						cli();
						for (int mi = 0; mi < active_motors; ++mi)
							motor[mi].sense_pos[sense_state] = motor[mi].current_pos;
						sei();
					}
				}
				// Check limit switches.
				if (stopping < 0) {
					int8_t value = buffer[cf][m][cs];
					if (value == 0)
						continue;
					uint8_t limit_pin = value < 0 ? motor[m].limit_min_pin : motor[m].limit_max_pin;
					if (limit_pin < NUM_DIGITAL_PINS) {
						bool inverted = motor[m].flags & (value < 0 ? Motor::INVERT_LIMIT_MIN : Motor::INVERT_LIMIT_MAX);
						if (GET(limit_pin) ^ inverted) {
							debug("hit %d pos %d state %d sample %d", m, motor[m].current_pos, state, buffer[cf][m][cs]);
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
		// Hit endstop or probe; disable timer interrupt.
		step_state = 1;
		//debug("hit limit %d curpos %ld cf %d ncf %d lf %d cfp %d", m, F(motor[m].current_pos), cf, notified_current_fragment, last_fragment, cs);
		// Notify host.
		limit_fragment_pos = cs;
		arch_set_speed(0);
		return;
	}
	if (homers > 0) {
		// Homing.
		if (state == 0) {
			probed = true;
			for (uint8_t m = 0; m < active_motors; ++m) {
				if (!(motor[m].intflags & Motor::ACTIVE))
					continue;
				// Get twe "wrong" limit pin for the given direction.
				uint8_t limit_pin = (buffer[cf][m][cs] < 0 ? motor[m].limit_max_pin : motor[m].limit_min_pin);
				bool inverted = motor[m].flags & (buffer[cf][m][cs] < 0 ? Motor::INVERT_LIMIT_MAX : Motor::INVERT_LIMIT_MIN);
				if (limit_pin >= NUM_DIGITAL_PINS || GET(limit_pin) ^ inverted) {
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
	if (state == 0 && probed) {
		if (homers > 0)
			current_sample = 0;	// Use only the first sample for homing.
		step_state = homers > 0 || (settings[cf].flags & Settings::PROBING) ? 2 : 3;
	}
}

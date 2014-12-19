#include "firmware.h"

static void do_steps(Dir dir, uint8_t m, uint8_t value) {
	if (dir != DIR_POSITIVE && dir != DIR_NEGATIVE) {
		debug("Invalid direction in do_steps");
		return;
	}
	if (motor[m].dir_pin < NUM_DIGITAL_PINS) {
		if (dir == DIR_POSITIVE)
			SET(motor[m].dir_pin);
		else
			RESET(motor[m].dir_pin);
	}
	if (motor[m].step_pin < NUM_DIGITAL_PINS) {
		for (uint8_t i = 0; i < value; ++i) {
			SET(motor[m].step_pin);
			RESET(motor[m].step_pin);
		}
	}
	motor[m].current_pos += (dir == DIR_POSITIVE ? value : -value);
}

// When to do the steps.
static uint16_t const lookup[] = {
	0x0000,
	0x0040,
	0x0204,
	0x0842,
	0x0889,
	0x1249,
	0x14a5,
	0x1555,
	0x5555,
	0x56b5,
	0x5b6d,
	0x5bbb,
	0x5ef7,
	0x5fbf,
	0x5fff,
	0x7fff
};

ISR(TIMER1_COMPA_vect) {
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
				for (int mi = 0; mi < active_motors; ++mi)
					motor[mi].sense_pos[(motor[m].flags & Motor::SENSE_STATE) ? 1 : 0] = motor[mi].current_pos;
			}
		}
		// Check limit switches.
		uint8_t limit_pin = (fragment.dir ? motor[m].limit_min_pin : motor[m].limit_max_pin);
		if (limit_pin < NUM_DIGITAL_PINS) {
			bool inverted = motor[m].flags & (fragment.dir ? Motor::INVERT_LIMIT_MIN : Motor::INVERT_LIMIT_MAX);
			if (GET(limit_pin) ^ inverted) {
				// Hit endstop.
				//debug("hit limit %d curpos %ld dir %d frag %d;%d;%d;%d", m, F(motor[m].current_pos), fragment.dir, current_fragment, notified_current_fragment, last_fragment, current_fragment_pos);
				// Notify host.
				motor[m].flags |= Motor::LIMIT;
				limit_fragment_pos = current_fragment_pos;
				current_fragment_pos = 0;
				set_speed(0);
				filling = 0;
				current_fragment = (last_fragment + 1) % FRAGMENTS_PER_BUFFER;
				notified_current_fragment = current_fragment;
				stopping = true;
				return;
			}
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
				do_steps(fragment.dir, m, 1);
				continue;
			}
			// Limit pin no longer triggered.  Stop moving and possibly notify host.
			fragment.dir = DIR_NONE;
			if (!--homers) {
				set_speed(0);
			}
		}
	}
	else {
		// Regular move.  (Not homing.)
		// Move to next buffer.
		while (!stopped && current_fragment_pos >= fragment_len[current_fragment]) {
			current_fragment_pos -= fragment_len[current_fragment];
			uint8_t new_current_fragment = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
			if (last_fragment == (filling > 0 ? new_current_fragment : current_fragment)) {
				// Underrun.
				set_speed(0);
				underrun = true;
			}
			current_fragment = new_current_fragment;
			//debug("new fragment: %d", current_fragment);
		}
		if (!stopped) {
			for (uint8_t m = 0; m < active_motors; ++m) {
				Fragment &fragment = buffer[motor[m].buffer][current_fragment];
				if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
					continue;
				uint8_t value = (fragment.samples[current_fragment_pos >> 1] >> (4 * (current_fragment_pos & 1))) & 0xf;
				do_steps(fragment.dir, m, value);
			}
			current_fragment_pos += 1;
		}
	}
}

void set_speed(uint16_t count) {
	stopped = (count == 0);
	if (stopped)
		TIMSK1 = 0;
	else {
		// Set TOP.
		OCR1AH = (count >> 7) & 0xff;
		OCR1AL = (count << 1) & 0xff;
		// Clear counter.
		TCNT1H = 0;
		TCNT1L = 0;
		// Clear and enable interrupt.
		TIFR1 = 1 << OCF1A;
		TIMSK1 = 1 << OCIE1A;
	}
}

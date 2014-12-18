// vim: set filetype=cpp foldmethod=marker foldmarker={,} :
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
		for (uint8_t s = 0; s < value; ++s) {
			SET(motor[m].step_pin);
			RESET(motor[m].step_pin);
		}
	}
	motor[m].current_pos += (dir == DIR_POSITIVE ? value : -value);
}

static void handle_motors(uint32_t current_time) {
	// Check sensors.
	for (uint8_t m = 0; m < active_motors; ++m) {
		Fragment &fragment = buffer[motor[m].buffer][current_fragment];
		if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
			continue;
		// Check sense pins.
		if (motor[m].sense_pin < NUM_DIGITAL_PINS) {
			if (GET(motor[m].sense_pin) ^ bool(motor[m].flags & Motor::SENSE_STATE)) {
				debug("sense %d %x", m, motor[m].flags);
				motor[m].flags ^= Motor::SENSE_STATE;
				motor[m].flags |= (motor[m].flags & Motor::SENSE_STATE ? Motor::SENSE1 : Motor::SENSE0);
				for (int mi = 0; mi < active_motors; ++mi)
					motor[mi].sense_pos[(motor[m].flags & Motor::SENSE_STATE) ? 1 : 0] = motor[mi].current_pos;
				try_send_next();
			}
		}
		// Check limit switches.
		uint8_t limit_pin = (fragment.dir ? motor[m].limit_min_pin : motor[m].limit_max_pin);
		if (limit_pin < NUM_DIGITAL_PINS) {
			bool inverted = motor[m].flags & (fragment.dir ? Motor::INVERT_LIMIT_MIN : Motor::INVERT_LIMIT_MAX);
			if (GET(limit_pin) ^ inverted) {
				// Hit endstop.
				debug("hit limit %d curpos %ld dir %d frag %d;%d;%d;%d", m, F(motor[m].current_pos), fragment.dir, current_fragment, notified_current_fragment, last_fragment, current_fragment_pos);
				// Notify host.
				motor[m].flags |= Motor::LIMIT;
				limit_fragment_pos = current_fragment_pos;
				current_fragment_pos = 0;
				stopped = true;
				filling = 0;
				current_fragment = (last_fragment + 1) % FRAGMENTS_PER_BUFFER;
				notified_current_fragment = current_fragment;
				stopping = true;
				try_send_next();
				return;
			}
		}
	}
	// Move motors.
	if (homers > 0) {
		// Homing.
		if (current_time - start_time < home_step_time)
			return;
		start_time += home_step_time;
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
				stopped = true;
				try_send_next();
			}
		}
	}
	else {
		// Regular move.  (Not homing.)
		bool want_send = false;
		int8_t fragment_diff = (current_time - start_time) / time_per_sample - current_fragment_pos;
		while (!stopped && fragment_diff > 0) {
			// Move to next buffer.
			while (!stopped && current_fragment_pos >= fragment_len[current_fragment]) {
				start_time += fragment_len[current_fragment] * time_per_sample;
				current_fragment_pos -= fragment_len[current_fragment];
				uint8_t new_current_fragment = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
				if (last_fragment == (filling > 0 ? new_current_fragment : current_fragment)) {
					// Underrun.
					stopped = true;
					underrun = true;
				}
				current_fragment = new_current_fragment;
				//debug("new fragment: %d", current_fragment);
				want_send = true;
			}
			if (stopped)
				break;
			for (uint8_t m = 0; m < active_motors; ++m) {
				Fragment &fragment = buffer[motor[m].buffer][current_fragment];
				if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
					continue;
				uint8_t value = (fragment.samples[current_fragment_pos >> 1] >> (4 * (current_fragment_pos & 1))) & 0xf;
				do_steps(fragment.dir, m, value);
			}
			current_fragment_pos += 1;
			fragment_diff -= 1;
		}
		if (want_send)
			try_send_next();
	}
}

static uint8_t next_adc(uint8_t old) {
	for (uint8_t a = 1; a <= NUM_ANALOG_INPUTS; ++a) {
		uint8_t n = (old + a) % NUM_ANALOG_INPUTS;
		if (adc[n].value[0] & 0x8000)
			// Invalid pin.
			continue;
		return n;
	}
	return ~0;
}

static void handle_adc(uint32_t current_time) {
	if (adc_phase == INACTIVE)
		return;
	if (!adc_ready(adc_current)) {
		//debug("adc %d not ready", adc_current);
		return;
	}
	uint16_t value = adc_get(adc_current);
	//debug("adc %d = %d", adc_current, value);
	// Send to host if it is waiting and buffer is free.
	if (adc_current == adc_next && !adcreply_ready) {
		adcreply[0] = CMD_ADC;
		adcreply[1] = adc_current;
		*reinterpret_cast <int16_t *>(&adcreply[2]) = value;
		adcreply_ready = 4;
		try_send_next();
		adc_next = next_adc(adc_next);
	}
	// Adjust heater and fan.
	for (uint8_t n = 0; n < 2; ++n) {
		if (adc[adc_current].linked[n] < NUM_DIGITAL_PINS) {
			if (((adc[adc_current].value[n] & 0x4000) != 0) ^ ((adc[adc_current].value[n] & 0x3fff) > value)) {
				RESET(adc[adc_current].linked[n]);
				if (n == 0)
					led_fast -= 1;
			}
			else {
				SET(adc[adc_current].linked[n]);
				if (n == 0)
					led_fast += 1;
			}
		}
	}
	adc_current = next_adc(adc_current);
	if (adc_current == uint8_t(~0))
		return;
	// Start new measurement.
	adc_ready(adc_current);
}

static void handle_led(uint32_t current_time) {
	uint32_t timing = led_fast ? 1000000 / 100 : 1000000 / 50;
	if (current_time - led_last < timing)
		return;
	while (current_time - led_last >= timing) {
		led_last += timing;
		led_phase += 1;
	}
	//debug("t %ld", F(next_led_time));
	led_phase %= 50;
	// Timings read from https://en.wikipedia.org/wiki/File:Wiggers_Diagram.png (phonocardiogram).
	bool state = (led_phase <= 4 || (led_phase >= 14 && led_phase <= 17));
	if (state)
		SET(led_pin);
	else
		RESET(led_pin);
}

void loop() {
	// Timekeeping.
	uint32_t current_time = utime();
	// Handle all periodic things.
	// LED
	if (led_pin < NUM_DIGITAL_PINS)
		handle_led(current_time);	// heart beat.
	// Motors
	if (!stopped)
		handle_motors(current_time);
	// ADC
	handle_adc(current_time);
	// Serial
	serial();
}

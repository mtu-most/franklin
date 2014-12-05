// vim: set filetype=cpp foldmethod=marker foldmarker={,} :
#include "firmware.h"

static void handle_motors(uint32_t current_time) {
	// Check sensors.
	for (uint8_t m = 0; m < NUM_MOTORS; ++m) {
		if (!(motor[m].flags & Motor::ACTIVE))
			continue;
		Fragment &fragment = buffer[motor[m].buffer][current_fragment];
		if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
			continue;
		// Check sense pins.
		if (motor[m].sense_pin < NUM_DIGITAL_PINS) {
			if (GET(motor[m].sense_pin) ^ bool(motor[m].flags & Motor::SENSE_STATE)) {
				motor[m].flags |= (motor[m].flags & Motor::SENSE_STATE ? Motor::SENSE1 : Motor::SENSE0);
				motor[m].flags ^= Motor::SENSE_STATE;
				motor[m].sense_pos[(motor[m].flags & Motor::SENSE_STATE) ? 1 : 0] = motor[m].current_pos;
				try_send_next();
			}
		}
		// Check limit switches.
		uint8_t limit_pin = (fragment.dir ? motor[m].limit_min_pin : motor[m].limit_max_pin);
		if (limit_pin < NUM_DIGITAL_PINS) {
			bool inverted = motor[m].flags & (fragment.dir ? Motor::INVERT_LIMIT_MIN : Motor::INVERT_LIMIT_MAX);
			if (GET(limit_pin) ^ inverted) {
				// Hit endstop.
				debug("hit limit %d curpos %ld dir %d frag %d", m, F(motor[m].current_pos), fragment.dir, current_fragment);
				// Notify host.
				limit_time = last_current_time - start_time;
				motor[m].flags |= Motor::LIMIT;
				limit_fragment = current_fragment;
				stopped = true;
				filling = 0;
				current_fragment = 0;
				last_fragment = FRAGMENTS_PER_BUFFER - 1;
				notified_current_fragment = current_fragment;
				stopping = true;
				try_send_next();
				return;
			}
		}
	}
	last_current_time = current_time;
	// Move motors.
	for (uint8_t m = 0; m < NUM_MOTORS; ++m) {
		if (!(motor[m].flags & Motor::ACTIVE))
			continue;
		Fragment &fragment = buffer[motor[m].buffer][current_fragment];
		if (fragment.dir == DIR_NONE || fragment.dir == DIR_AUDIO)
			continue;
		if (motor[m].dir_pin < NUM_DIGITAL_PINS) {
			if (fragment.dir)
				RESET(motor[m].dir_pin);
			else
				SET(motor[m].dir_pin);
		}
		uint16_t pos = (last_current_time - start_time) / fragment.us_per_sample;
		while (motor[m].pos < pos && pos < fragment.num_samples) {
			uint8_t value = (fragment.samples[motor[m].pos >> 1] >> (4 * (motor[m].pos & 1))) & 0xf;
			for (uint8_t s = 0; s < value; ++s) {
				SET(motor[m].step_pin);
				RESET(motor[m].step_pin);
			}
			motor[m].current_pos += (fragment.dir ? -1 : 1) * value;
			++motor[m].pos;
		}
	}
	// Move to next buffer.
	bool want_send = false;
	while (last_current_time - start_time >= fragment_time[current_fragment]) {
		start_time += fragment_time[current_fragment];
		for (uint8_t m = 0; m < NUM_MOTORS; ++m) {
			if (!(motor[m].flags & Motor::ACTIVE))
				continue;
			motor[m].pos = 0;
		}
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
	if (want_send)
		try_send_next();
}

static uint8_t next_adc(uint8_t old) {
	for (uint8_t a = 0; a <= NUM_ANALOG_INPUTS; ++a) {
		uint8_t n = (old + a) % NUM_ANALOG_INPUTS;
		if (adc[n].value[0] & 0x8000)
			// Invalid pin.
			continue;
		return n;
	}
	return ~0;
}

static void handle_adc(uint32_t current_time) {
	if (current_time - last_adc_time < ADC_INTERVAL)
		return;
	if (adc_phase == INACTIVE) {
		last_adc_time += ADC_INTERVAL;
		return;
	}
	if (!adc_ready(adc_current)) {
		//debug("adc %d not ready", adc_current);
		return;
	}
	last_adc_time += ADC_INTERVAL;
	uint16_t value = adc_get(adc_current);
	//debug("adc %d = %d", adc_current, value);
	if (adc_current == adc_next && !adcreply_ready) {
		adcreply[0] = CMD_ADC;
		adcreply[1] = adc_current;
		*reinterpret_cast <int16_t *>(&adcreply[2]) = value;
		adcreply_ready = 4;
		try_send_next();
		adc_next = next_adc(adc_next);
	}
	for (uint8_t n = 0; n < 2; ++n) {
		if (adc[adc_current].linked[n] < NUM_DIGITAL_PINS) {
			if ((adc[adc_current].value[n] & 0x4000) ^ ((adc[adc_current].value[n] & 0x3fff) > value))
				SET(adc[adc_current].linked[n]);
			else
				RESET(adc[adc_current].linked[n]);
		}
	}
	adc_current = next_adc(adc_current);
	if (adc_current != uint8_t(~0)) {
		// Start new measurement.
		adc_ready(adc_current);
		return;
	}
	adc_phase = INACTIVE;
	adc_current = ~0;
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

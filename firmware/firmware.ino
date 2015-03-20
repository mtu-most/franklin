// vim: set filetype=cpp foldmethod=marker foldmarker={,} :
#include "firmware.h"
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

static void handle_adc() {
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
		adc_next = next_adc(adc_next);
	}
	// Adjust heater and fan.
	for (uint8_t n = 0; n < 2; ++n) {
		if (adc[adc_current].linked[n] < NUM_DIGITAL_PINS) {
			if (((adc[adc_current].value[n] & 0x4000) != 0) ^ ((adc[adc_current].value[n] & 0x3fff) > value)) {
				RESET(adc[adc_current].linked[n]);
				if (n == 0 && adc[adc_current].is_on) {
					led_fast -= 1;
					adc[adc_current].is_on = false;
				}
			}
			else {
				SET(adc[adc_current].linked[n]);
				if (n == 0 && !adc[adc_current].is_on) {
					led_fast += 1;
					adc[adc_current].is_on = true;
				}
			}
		}
	}
	adc_current = next_adc(adc_current);
	if (adc_current == uint8_t(~0))
		return;
	// Start new measurement.
	adc_ready(adc_current);
}

static void handle_led() {
	uint16_t timing = 1000 / (50 * (led_fast + 1));
	uint16_t current_time = millis();
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
	if (state ^ bool(pin_flags & 1))
		SET(led_pin);
	else
		RESET(led_pin);
}

int main(void) {
	setup();
	while (true) {
		// Handle all periodic things.
		// LED
		if (led_pin < NUM_DIGITAL_PINS)
			handle_led();	// heart beat.
		handle_motors();
		// ADC
		handle_adc();
		handle_motors();
		// Serial
		serial();
		handle_motors();
		// Send serial data, if any.
		try_send_next();
		// Motor precompute steps.
		handle_motors();
		uint16_t dt = (seconds() - last_active + 0x10000) & 0xffff;
		if (enabled_pins > 0 && timeout_time > 0 && timeout_time <= dt) {
			// Disable LED.
			led_pin = ~0;
			// Disable adcs.
			for (uint8_t a = 0; a < NUM_ANALOG_INPUTS; ++a)
				adc[a].disable();
			// Disable pins.
			for (uint8_t p = 0; p < NUM_DIGITAL_PINS; ++p)
				pin[p].disable(p);
			debug("timeout %d %d %d", seconds(), dt, last_active);
			timeout = true;
		}
		//debug("!%x. %x,", debug_value, int(pin[0x30].avr_output));
	}
}

/* firmware.ino - main loop for Franklin
 * vim: set filetype=cpp foldmethod=marker foldmarker={,} :
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

#ifndef NO_ADC
static uint8_t next_adc(uint8_t old) {
	for (uint8_t a = 1; a <= ADC_LAST_PIN; ++a) {
		uint8_t n = old + a;
		while (n > ADC_LAST_PIN)
			n -= ADC_LAST_PIN;
		if (!(ADC_PIN_MASK & (1 << n)))
			continue;
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
	int16_t value = adc_get(adc_current);
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
#ifndef NO_TEMP_HOLD
	unsigned long now = millis();
	if (unsigned(now - adc[adc_current].last_change) >= adc[adc_current].hold_time)
#endif
	{
		for (uint8_t n = 0; n < 2; ++n) {
			bool invert = (adc[adc_current].value[n] & 0x4000) == 0;
			int16_t treshold = adc[adc_current].value[n] & 0x3fff;
			bool higher = value >= treshold;
			//debug("limits %d %d %x %x %x %x", adc_current, n, adc[adc_current].limit[n][0], adc[adc_current].limit[n][1], value, adc[adc_current].value[n]);
#ifndef NO_TEMP_LIMIT
			if (value < (adc[adc_current].limit[n][0] & 0x3fff)) {
				// Below lower limit: force heater to be ON or fan to be OFF. (Fan is inverted.)
				if (adc[adc_current].limit[n][0] & 0x4000) {
					//debug("limit l0 %d %d invert %d", adc_current, n, invert);
					higher = true;
				}
				else {
					//debug("limit l1 %d %d invert %d", adc_current, n, invert);
					higher = false;
				}
			}
			else if (value >= (adc[adc_current].limit[n][1] & 0x3fff)) {
				// Above upper limit: force heater to be OFF or fan to be ON. (Fan is inverted.)
				if (adc[adc_current].limit[n][1] & 0x4000) {
					//debug("limit h0 %d %d invert %d", adc_current, n, invert);
					higher = true;
				}
				else {
					//debug("limit h1 %d %d invert %d", adc_current, n, invert);
					higher = false;
				}
			}
#endif
			//debug("adc test diff %d hold %d current %d %d higher %d invert %d", int(now - adc[adc_current].last_change), adc[adc_current].hold_time, adc_current, n, higher, invert);
			if (invert ^ higher) {
				if (adc[adc_current].is_on[n]) {
					//debug("switch off %d %d value %x treshold %x higher %d invert %d", adc_current, n, value, treshold, higher, invert);
#ifndef NO_TEMP_HOLD
					adc[adc_current].last_change = now;
#endif
					if (Gpio::check_pin(adc[adc_current].linked[n]))
						RESET(adc[adc_current].linked[n]);
#ifndef NO_LED
					if (n == 0)
						led_fast -= 1;
#endif
					adc[adc_current].is_on[n] = false;
				}
				else {
					//debug("already off");
				}
			}
			else {
				//debug("adc set %d %d %d", n, value, adc[adc_current].value[n]);
				if (!adc[adc_current].is_on[n]) {
					//debug("switch on %d %d value %x treshold %x higher %d invert %d", adc_current, n, value, treshold, higher, invert);
					if (Gpio::check_pin(adc[adc_current].linked[n]))
						SET(adc[adc_current].linked[n]);
#ifndef NO_TEMP_HOLD
					adc[adc_current].last_change = now;
#endif
#ifndef NO_LED
					if (n == 0)
						led_fast += 1;
#endif
					adc[adc_current].is_on[n] = true;
				}
				else {
					//debug("already on");
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

void AdcData::disable() {
	if (value[0] & 0x8000)
		return;
	for (uint8_t i = 0; i < 2; ++i) {
		if (!Gpio::check_pin(linked[i]))
			continue;
		UNSET(linked[i]);
		linked[i] = ~0;
	}
}
#endif

#ifndef NO_LED
static void handle_led() {
	uint16_t timing = 1000 / (50 * (led_fast + 1));
	uint16_t current_time = millis();
	if (current_time - led_last < timing)
		return;
	while (current_time - led_last >= timing) {
		led_last += timing;
		led_phase += 1;
		while (led_phase >= 50)
			led_phase -= 50;
	}
	//debug("t %ld", F(next_led_time));
	// Timings read from https://en.wikipedia.org/wiki/File:Wiggers_Diagram.png (phonocardiogram).
	bool state = (led_phase <= 4 || (led_phase >= 14 && led_phase <= 17));
	if (state ^ bool(pin_flags & 1))
		SET(led_pin);
	else
		RESET(led_pin);
}
#endif

static void handle_pins() {
	for (uint8_t p = 0; p <= GPIO_LAST_PIN; ++p) {
		if (!Gpio::check_pin(p))
			continue;
#ifndef NO_PIN_MOTOR
		if (pin[p - GPIO_FIRST_PIN].motor < active_motors) {
			pin[p - GPIO_FIRST_PIN].duty = (motor[pin[p - GPIO_FIRST_PIN].motor].current_pos * pin[p - GPIO_FIRST_PIN].ticks) & 0x7fff;	// TODO: reset duty on home.
			if (CONTROL_CURRENT(pin[p - GPIO_FIRST_PIN].state) == CTRL_SET)
				SET(p);
		}
#endif
		if (!(pin[p - GPIO_FIRST_PIN].state & CTRL_NOTIFY))
			continue;
		bool new_state = GET(p);
		if (new_state == pin[p - GPIO_FIRST_PIN].value())
			continue;
		//debug("read pin %d, old %d new %d", p, new_state, pin[p - GPIO_FIRST_PIN].value());
		if (new_state)
			pin[p - GPIO_FIRST_PIN].state |= CTRL_VALUE;
		else
			pin[p - GPIO_FIRST_PIN].state &= ~CTRL_VALUE;
		if (!pin[p - GPIO_FIRST_PIN].event()) {
			pin[p - GPIO_FIRST_PIN].state |= CTRL_EVENT;
			pin_events += 1;
		}
	}
}

int main(void) {
	setup();
	while (true) {
		// Handle all periodic things.
		// LED
#ifndef NO_LED
		if (led_pin <= GPIO_LAST_PIN)
			handle_led();	// heartbeat.
#endif
		handle_motors();
		// ADC
#ifndef NO_ADC
		handle_adc();
#endif
		handle_motors();
		// Serial
		serial();
		handle_motors();
		// Send serial data, if any.
		try_send_next();
		handle_motors();
		// Update pin states.
		handle_pins();
		handle_motors();
		// Handle PWM of outputs.
#ifndef NO_PWM
		arch_outputs();
#endif
		handle_motors();
		// Timeout.
#ifndef NO_TIMEOUT
		uint16_t dt = seconds() - last_active;
		if (enabled_pins > 0 && step_state == STEP_STATE_STOP && timeout_time > 0 && timeout_time <= dt) {
			// Disable LED and probe.
			led_pin = ~0;
			probe_pin = ~0;
			// Disable motors.
			for (uint8_t m = 0; m < active_motors; ++m)
				motor[m].disable(m);
			active_motors = 0;
			// Disable adcs.
			for (uint8_t a = 0; a < ADC_NUM_PINS; ++a)
				adc[a].disable();
			// Disable pins.
			for (uint8_t p = 0; p <= GPIO_LAST_PIN; ++p) {
				if (Gpio::check_pin(p))
					pin[p - GPIO_FIRST_PIN].disable(p);
			}
			//debug("timeout");
			timeout = true;
		}
#endif
		arch_tick();
		//debug("!%x %x %x %x.", enabled_pins, timeout_time, dt, last_active);
#ifndef NO_DEBUG
		if (debug_value != debug_value1 || debug_value2 != debug_value3) {
			debug_value1 = debug_value;
			debug_value3 = debug_value2;
			debug("!%x.%x;", debug_value, debug_value2);
		}
#endif
	}
}

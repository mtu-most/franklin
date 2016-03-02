/* packet.cpp - command packet handling for Franklin
 * vim: set foldmethod=marker :
 * Copyright 2014 Michigan Technological University
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

//#define cmddebug debug
#define cmddebug(...) do {} while (0)
//#define cmddebug(...) do { debug_add(command(0)); } while (0)

static uint16_t read_16(int pos) {
	return command(pos) | (command(pos + 1) << 8);
}

static uint32_t read_32(int pos) {
	uint32_t ret = 0;
	for (uint8_t b = 0; b < 4; ++b)
		ret |= command(pos + b) << (8 * b);
	return ret;
}

void packet()
{
	last_active = seconds();
	switch (command(0))
	{
	case CMD_BEGIN:	// begin: request response
	{
		cmddebug("CMD_BEGIN");
		// A server is running; start the watchdog.
		arch_watchdog_enable();
		for (uint8_t i = 0; i < ID_SIZE; ++i)
			printerid[1 + i] = command(2 + i);
		// Because this is a new connection: reset active_motors and all ADC pins.
		active_motors = 0;
		for (uint8_t a = 0; a < NUM_ANALOG_INPUTS; ++a)
			adc[a].disable();
		filling = 0;
		arch_set_speed(0);
		homers = 0;
		home_step_time = 0;
		reply[0] = CMD_READY;
		reply[1] = 27;
		*reinterpret_cast <uint32_t *>(&reply[2]) = PROTOCOL_VERSION;
		reply[6] = NUM_DIGITAL_PINS;
		reply[7] = NUM_ANALOG_INPUTS;
		reply[8] = NUM_MOTORS;
		reply[9] = 1 << FRAGMENTS_PER_MOTOR_BITS;
		reply[10] = BYTES_PER_FRAGMENT;
		for (uint8_t i = 0; i < UUID_SIZE; ++i) {
			BUFFER_CHECK(reply, 11 + i);
			reply[11 + i] = uuid[i];
		}
		reply_ready = 11 + UUID_SIZE;
		write_ack();
		return;
	}
	case CMD_PING:
	{
		//cmddebug("CMD_PING");
		ping |= 1 << command(1);
		write_ack();
		return;
	}
	case CMD_SET_UUID: // reset controller; used before reprogramming flash.
	{
		cmddebug("CMD_SET_UUID");
		for (uint8_t i = 0; i < UUID_SIZE; ++i) {
			uuid[i] = command(1 + i);
			EEPROM.write(i, uuid[i]);
		}
		write_ack();
		return;
	}
	case CMD_SETUP:
	{
		cmddebug("CMD_SETUP");
		if (command(1) > NUM_MOTORS) {
			debug("num motors %d > available %d", command(1), NUM_MOTORS);
			write_stall();
			return;
		}
		// Reset newly (de)activated motors.
		for (uint8_t m = active_motors; m < command(1); ++m)
			motor[m].init(m);
		for (uint8_t m = command(1); m < active_motors; ++m)
			motor[m].disable(m);
		active_motors = command(1);
		time_per_sample = read_32(2);
		uint8_t p = led_pin;
		led_pin = command(6);
		if (p != led_pin) {
			if (p < NUM_DIGITAL_PINS)
				UNSET(p);
			if (led_pin < NUM_DIGITAL_PINS)
				SET_OUTPUT(led_pin);
		}
		p = stop_pin;
		stop_pin = command(7);
		if (p != stop_pin) {
			if (p < NUM_DIGITAL_PINS)
				UNSET(p);
			if (stop_pin < NUM_DIGITAL_PINS)
				SET_INPUT(stop_pin);
		}
		p = probe_pin;
		probe_pin = command(8);
		if (p != probe_pin) {
			if (p < NUM_DIGITAL_PINS)
				UNSET(p);
			if (probe_pin < NUM_DIGITAL_PINS)
				SET_INPUT(probe_pin);
		}
		pin_flags = command(9);
		timeout_time = read_16(10);
		if (command(13) < active_motors) {
			audio = 2;
			move_phase = 0;
			full_phase = 1;
			audio_motor = &motor[command(13)];
		}
		else {
			uint8_t fpb = 0;
			while (time_per_sample / TIME_PER_ISR >= uint16_t(1) << fpb)
				fpb += 1;
			fpb -= 1;
			cli();
			audio = 0;
			audio_motor = 0;
			full_phase_bits = fpb;
			full_phase = 1 << full_phase_bits;
			sei();
		}
		p = spiss_pin;
		spiss_pin = command(12);
		if (p != spiss_pin) {
			if ((p < NUM_DIGITAL_PINS) ^ (spiss_pin < NUM_DIGITAL_PINS)) {
				if (spiss_pin < NUM_DIGITAL_PINS)
					arch_spi_start();
				else
					arch_spi_stop();
			}
			if (p < NUM_DIGITAL_PINS)
				UNSET(p);
			if (spiss_pin < NUM_DIGITAL_PINS) {
				if (pin_flags & 8)
					SET(spiss_pin);
				else
					RESET(spiss_pin);
			}
		}
		write_ack();
		return;
	}
	case CMD_CONTROL:
	{
		cmddebug("CMD_CONTROL");
		uint8_t p = command(1);
		if (p >= NUM_DIGITAL_PINS) {
			debug("invalid pin in control: %d", p);
			write_stall();
			return;
		}
		uint8_t value = command(2);
		pin[p].duty = command(3);
		pin[p].set_state((pin[p].state & ~0xc) | (value & 0xc));
		switch (CONTROL_CURRENT(value)) {
		case CTRL_RESET:
			RESET(p);
			break;
		case CTRL_SET:
			SET(p);
			break;
		case CTRL_UNSET:
			UNSET(p);
			break;
		case CTRL_INPUT:
			SET_INPUT(p);
			break;
		}
		write_ack();
		return;
	}
	case CMD_MSETUP:
	{
		cmddebug("CMD_MSETUP");
		uint8_t m = command(1);
		if (m >= active_motors) {
			debug("MSETUP called for invalid motor %d", m);
			write_stall();
			return;
		}
		uint8_t step = motor[m].step_pin;
		uint8_t dir = motor[m].dir_pin;
		uint8_t limit_min = motor[m].limit_min_pin;
		uint8_t limit_max = motor[m].limit_max_pin;
		uint8_t flags = motor[m].flags;
		motor[m].step_pin = command(2);
		motor[m].dir_pin = command(3);
		motor[m].limit_min_pin = command(4);
		motor[m].limit_max_pin = command(5);
		motor[m].follow = command(6);
		uint8_t const intmask = Motor::INVERT_STEP;
		uint8_t const mask = Motor::INVERT_LIMIT_MIN | Motor::INVERT_LIMIT_MAX;
		cli();
		motor[m].intflags &= ~intmask;
		motor[m].intflags |= command(7) & intmask;
		motor[m].flags &= ~mask;
		motor[m].flags |= command(7) & mask;
		arch_msetup(m);
		sei();
		if (step != motor[m].step_pin || (flags & Motor::INVERT_STEP) != (motor[m].intflags & Motor::INVERT_STEP)) {
			//debug("new step for %d", m);
			if (step < NUM_DIGITAL_PINS)
				UNSET(step);
			if (motor[m].step_pin < NUM_DIGITAL_PINS) {
				if (motor[m].intflags & Motor::INVERT_STEP)
					SET(motor[m].step_pin);
				else
					RESET(motor[m].step_pin);
			}
		}
		if (dir != motor[m].dir_pin) {
			//debug("new dir for %d", m);
			if (dir < NUM_DIGITAL_PINS)
				UNSET(dir);
			if (motor[m].dir_pin < NUM_DIGITAL_PINS)
				RESET(motor[m].dir_pin);
		}
		if (limit_min != motor[m].limit_min_pin) {
			//debug("new limit_min for %d", m);
			if (limit_min < NUM_DIGITAL_PINS)
				UNSET(limit_min);
			if (motor[m].limit_min_pin < NUM_DIGITAL_PINS)
				SET_INPUT(motor[m].limit_min_pin);
		}
		if (limit_max != motor[m].limit_max_pin) {
			//debug("new limit_max for %d", m);
			if (limit_max < NUM_DIGITAL_PINS)
				UNSET(limit_max);
			if (motor[m].limit_max_pin < NUM_DIGITAL_PINS)
				SET_INPUT(motor[m].limit_max_pin);
		}
		write_ack();
		return;
	}
	case CMD_ASETUP:
	{
		cmddebug("CMD_ASETUP");
		uint8_t a = command(1);
		if (a >= NUM_ANALOG_INPUTS) {
			debug("ASETUP called for invalid adc %d", a);
			write_stall();
			return;
		}
		for (uint8_t i = 0; i < 2; ++i) {
			// If the old one was active, deactivate it.
			if (~adc[a].value[0] & 0x8000 && adc[a].linked[i] < NUM_DIGITAL_PINS) {
				UNSET(adc[a].linked[i]);
				if (i == 0 && adc[a].is_on) {
					adc[a].is_on = false;
					led_fast -= 1;
				}
			}
			adc[a].linked[i] = command(2 + i);
			adc[a].limit[i] = read_16(4 + 2 * i);
			adc[a].value[i] = read_16(8 + 2 * i);
			//debug("adc %d link %d pin %d value %x", a, i, adc[a].linked[i], adc[a].value[i]);
		}
		if (adc_phase == INACTIVE && ~adc[a].value[0] & 0x8000) {
			adc_phase = PREPARING;
			adc_current = a;
			adc_next = a;
			adc_ready(a);
		}
		write_ack();
		return;
	}
	case CMD_HOME:
	{
		cmddebug("CMD_HOME");
		if (step_state != STEP_STATE_STOP || stopping >= 0) {
			debug("HOME seen while moving");
			write_stall();
			return;
		}
		if (current_fragment != last_fragment) {
			debug("HOME seen with non-empty buffer");
			write_stall();
			return;
		}
		for (uint8_t m = 0; m < active_motors; ++m) {
			if (command(5 + m) == 1) {
				// Fill both sample 0 and 1, because the interrupt handler may change current_sample at any time.
				buffer[current_fragment][m][0] = 1;
				buffer[current_fragment][m][1] = 0;
				buffer[current_fragment][m][2] = 1;
				buffer[current_fragment][m][3] = 0;
				homers += 1;
				motor[m].intflags |= Motor::ACTIVE;
				motor[m].steps_current = 0;
			}
			else if (int8_t(command(5 + m)) == -1) {
				// Fill both sample 0 and 1, because the interrupt handler may change current_sample at any time.
				buffer[current_fragment][m][0] = 0xff;
				buffer[current_fragment][m][1] = 0xff;
				buffer[current_fragment][m][2] = 0xff;
				buffer[current_fragment][m][3] = 0xff;
				homers += 1;
				motor[m].intflags |= Motor::ACTIVE;
				motor[m].steps_current = 0;
			}
			else if (command(5 + m) == 0) {
				buffer[current_fragment][m][0] = 0x00;
				buffer[current_fragment][m][1] = 0x80;
				motor[m].intflags &= ~Motor::ACTIVE;
			}
			else {
				debug("invalid dir in HOME: %d", command(5 + m));
				homers = 0;
				write_stall();
				return;
			}
		}
		if (homers > 0) {
			home_step_time = read_32(1);
			// current_sample is always 0 while homing; set len to 2, so it doesn't go to the next fragment.
			settings[current_fragment].len = 4;
			current_len = settings[current_fragment].len;
			//debug("home no probe %d", current_fragment);
			settings[current_fragment].flags &= ~Settings::PROBING;
			BUFFER_CHECK(buffer, current_fragment);
			current_buffer = &buffer[current_fragment];
			current_sample = 0;
			//debug("step_state home 0");
			step_state = STEP_STATE_PROBE;
			arch_set_speed(home_step_time);
			write_ack();
		}
		else {
			debug("nothing to home");
			write_stall();
		}
		return;
	}
	case CMD_START_MOVE:
	case CMD_START_PROBE:
	{
		cmddebug("CMD_START_MOVE");
		last_len = command(1);	// Do this even when ignoring the command.
		if (stopping >= 0) {
			//debug("ignoring start move while stopping");
			write_ack();
			return;
		}
		if (filling > 0) {
			debug("START_MOVE seen while filling (%d)", filling);
			write_stall();
			return;
		}
		if (command(1) > BYTES_PER_FRAGMENT) {
			debug("Fragment size too large (%d > %d)", command(1), BYTES_PER_FRAGMENT);
			write_stall();
			return;
		}
		uint8_t next = (last_fragment + 1) & FRAGMENTS_PER_MOTOR_MASK;
		if (next == current_fragment) {
			debug("New buffer sent with full buffer.");
			write_stall();
			return;
		}
		settings[last_fragment].len = command(1);
		filling = command(2);
		for (uint8_t m = 0; m < active_motors; ++m) {
			buffer[last_fragment][m][0] = 0x00;	// Sentinel indicating no data is available for this motor.
			buffer[last_fragment][m][1] = 0x80;
		}
		if (command(0) == CMD_START_MOVE) {
			//debug("move no probe %d", last_fragment);
			settings[last_fragment].flags &= ~Settings::PROBING;
		}
		else {
			//debug("move probe %d", last_fragment);
			settings[last_fragment].flags |= Settings::PROBING;
		}
		if (filling == 0)
			last_fragment = next;
		//debug("new filling: %d %d", filling, last_fragment);
		write_ack();
		return;
	}
	case CMD_MOVE:
	case CMD_MOVE_SINGLE:
	{
		cmddebug("CMD_MOVE(_SINGLE)");
		uint8_t m = command(1);
		if (m >= NUM_MOTORS) {
			debug("invalid buffer %d to fill", m);
			write_stall();
			return;
		}
		if (stopping >= 0) {
			//debug("ignoring move while stopping");
			write_ack();
			return;
		}
		if (filling == 0) {
			debug("MOVE seen while not filling");
			write_stall();
			return;
		}
		if (buffer[last_fragment][m][0] != 0 || buffer[last_fragment][m][1] != int8_t(0x80)) {
			debug("duplicate buffer %d to fill", m);
			write_stall();
			return;
		}
		for (uint8_t b = 0; b < last_len; ++b)
			buffer[last_fragment][m][b] = static_cast<int8_t>(command(2 + b));
		if (command(0) != CMD_MOVE_SINGLE) {
			for (uint8_t f = 0; f < active_motors; ++f) {
				if (motor[f].follow == m) {
					for (uint8_t b = 0; b < last_len; ++b)
						buffer[last_fragment][f][b] = buffer[last_fragment][m][b];
				}
			}
		}
		filling -= 1;
		if (filling == 0) {
			//debug("filled %d; current %d notified %d", last_fragment, current_fragment, notified_current_fragment);
			last_fragment = (last_fragment + 1) & FRAGMENTS_PER_MOTOR_MASK;
		}
		write_ack();
		return;
	}
	case CMD_START:
	{
		cmddebug("CMD_START");
		if (stopping >= 0) {
			//debug("ignoring start while stopping");
			write_ack();
			return;
		}
		if (step_state != STEP_STATE_STOP) {
			debug("Received START while not stopped");
			write_stall();
			return;
		}
		if (current_fragment == last_fragment) {
			debug("Received START while buffer is empty");
			write_stall();
			return;
		}
		//debug("starting.  last %d; current %d notified %d", last_fragment, current_fragment, notified_current_fragment);
		BUFFER_CHECK(buffer, current_fragment);
		current_buffer = &buffer[current_fragment];
		for (uint8_t m = 0; m < active_motors; ++m) {
			if (buffer[current_fragment][m][0] != 0 || buffer[current_fragment][m][1] != int8_t(0x80)) {
				motor[m].intflags |= Motor::ACTIVE;
				motor[m].steps_current = 0;
			}
			else
				motor[m].intflags &= ~Motor::ACTIVE;
		}
		current_sample = 0;
		current_len = settings[current_fragment].len;
		//debug("step_state start 0");
		step_state = STEP_STATE_PROBE;
		arch_set_speed(time_per_sample);
		write_ack();
		return;
	}
	case CMD_ABORT:
	{
		cmddebug("CMD_ABORT");
		for (uint8_t p = 0; p < NUM_DIGITAL_PINS; ++p) {
			switch (CONTROL_RESET(pin[p].state)) {
			case CTRL_RESET:
				RESET(p);
				break;
			case CTRL_SET:
				SET(p);
				break;
			case CTRL_UNSET:
				UNSET(p);
				break;
			case CTRL_INPUT:
				SET_INPUT(p);
				break;
			}
		}
		// Fall through.
	}
	case CMD_STOP:
	{
		cmddebug("CMD_STOP");
		if (filling > 0) {
			debug("STOP seen while filling");
			write_stall();
			return;
		}
		arch_set_speed(0);
		filling = 0;
		homers = 0;
		home_step_time = 0;
		reply[0] = CMD_STOPPED;
		reply[1] = active_motors;
		reply[2] = current_sample;
		for (uint8_t m = 0; m < active_motors; ++m) {
			motor[m].intflags &= ~Motor::ACTIVE;
			motor[m].steps_current = 0;
			BUFFER_CHECK(reply, 2 + 4 * m);
			*reinterpret_cast <int32_t *>(&reply[3 + 4 * m]) = motor[m].current_pos;
			//debug("cp %d %ld", m, F(motor[m].current_pos));
		}
		reply_ready = 3 + 4 * active_motors;
		current_fragment = last_fragment;
		notified_current_fragment = current_fragment;
		//debug("stop new current %d", current_fragment);
		current_sample = 0;
		write_ack();
		return;
	}
	case CMD_DISCARD:
	{
		cmddebug("CMD_DISCARD");
		// TODO: work well with interrupts.
		filling = 0;
		if (command(1) >= ((last_fragment - current_fragment) & FRAGMENTS_PER_MOTOR_MASK)) {
			debug("discarding more than entire buffer");
			write_stall();
			return;
		}
		cli();
		last_fragment = (last_fragment - command(1)) & FRAGMENTS_PER_MOTOR_MASK;
		sei();
		write_ack();
		return;
	}
	case CMD_GETPIN:
	{
		cmddebug("CMD_GETPIN");
		if (command(1) >= NUM_DIGITAL_PINS) {
			debug("invalid pin %02x for GETPIN", uint8_t(command(1)));
			write_stall();
			return;
		}
		reply[0] = CMD_PIN;
		reply[1] = GET(command(1));
		reply_ready = 2;
		write_ack();
		return;
	}
	case CMD_SPI:
	{
		cmddebug("CMD_SPI");
		uint8_t bits = command(1);
		if (pin_flags & 8)
			RESET(spiss_pin);
		else
			SET(spiss_pin);
		for (uint8_t i = 0; i * 8 < bits; ++i) {
			uint8_t b = bits - i * 8;
			arch_spi_send(command(2 + i), b > 8 ? 8 : b);
		}
		if (pin_flags & 8)
			SET(spiss_pin);
		else
			RESET(spiss_pin);
		write_ack();
		return;
	}
	case CMD_PINNAME:
	{
		cmddebug("CMD_PINNAME");
		uint8_t pin_ = command(1);
		reply[0] = CMD_NAMED_PIN;
		reply[1] = arch_pin_name(reinterpret_cast <char *>(&reply[2]), pin_ < 128, pin_ & 0x7f);
		reply_ready = 2 + reply[1];
		write_ack();
		return;
	}
	default:
	{
		debug("Invalid command %x %x %x %x", uint8_t(command(0)), uint8_t(command(1)), uint8_t(command(2)), uint8_t(command(3)));
		write_stall();
		return;
	}
	}
}

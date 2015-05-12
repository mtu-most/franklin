#include "firmware.h"

//#define cmddebug debug
//#define cmddebug(...) do {} while (0)
#define cmddebug(...) do { debug_add(command(0)); } while (0)

void packet()
{
	last_active = seconds();
	switch (command(0))
	{
	case CMD_BEGIN:	// begin: request response
	{
		cmddebug("CMD_BEGIN");
		// A server is running; start the watchdog.
		watchdog_enable();
		write_ack();
		for (uint8_t i = 0; i < ID_SIZE / 3; ++i) {
			for (uint8_t k = 0; k < 3; ++k)
				printerid[k * (ID_SIZE / 3) + i] = command(2 + i);
		}
		// Because this is a new connection: reset active_motors and all ADC pins.
		active_motors = 0;
		for (uint8_t a = 0; a < NUM_ANALOG_INPUTS; ++a)
			adc[a].disable();
		filling = 0;
		set_speed(0);
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
		for (uint8_t i = 0; i < 16; ++i)
			reply[11 + i] = uuid[i];
		reply_ready = 27;
		return;
	}
	case CMD_PING:
	{
		//cmddebug("CMD_PING");
		ping |= 1 << command(1);
		write_ack();
		return;
	}
	case CMD_RESET: // reset controller; used before reprogramming flash.
	{
		cmddebug("CMD_RESET");
		uint32_t magic = *reinterpret_cast <volatile int32_t *>(&command(1));
		if (magic != RESET_MAGIC) {
			debug("invalid reset magic %lx", F(magic));
			write_stall();
			return;
		}
		write_ack();
		serial_flush();
		reset();
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
			motor[m].disable();
		for (uint8_t m = command(1); m < active_motors; ++m)
			motor[m].init();
		active_motors = command(1);
		time_per_sample = *reinterpret_cast <volatile int32_t *>(&command(2));
		if (led_pin < NUM_DIGITAL_PINS)
			UNSET(led_pin);
		if (probe_pin < NUM_DIGITAL_PINS)
			UNSET(probe_pin);
		led_pin = command(6);
		probe_pin = command(7);
		pin_flags = command(8);
		if (led_pin < NUM_DIGITAL_PINS)
			SET_OUTPUT(led_pin);
		if (probe_pin < NUM_DIGITAL_PINS)
			SET_INPUT(probe_pin);
		timeout_time = *reinterpret_cast <volatile uint16_t *>(&command(9));
		cli();
		full_phase_bits = 0;
		while (time_per_sample / TIME_PER_ISR >= uint16_t(1) << full_phase_bits)
			full_phase_bits += 1;
		sei();
		full_phase_bits -= 1;
		full_phase = 1 << full_phase_bits;
		write_ack();
		return;
	}
	case CMD_CONTROL:
	{
		cmddebug("CMD_CONTROL");
		uint8_t num = command(1);
		for (uint8_t i = 0; i < num; ++i) {
			if (command(2 + i + 1) >= NUM_DIGITAL_PINS) {
				debug("invalid pin in control: %d", command(2 + i + 1));
				continue;
			}
			pin[command(2 + i + 1)].set_state((pin[command(2 + i + 1)].state & ~0xc) | (command(2 + i) & 0xc));
			switch (CONTROL_CURRENT(command(2 + i))) {
			case CTRL_RESET:
				RESET(command(2 + i + 1));
				break;
			case CTRL_SET:
				SET(command(2 + i + 1));
				break;
			case CTRL_UNSET:
				UNSET(command(2 + i + 1));
				break;
			case CTRL_INPUT:
				SET_INPUT(command(2 + i + 1));
				break;
			}
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
		uint8_t sense = motor[m].sense_pin;
		uint8_t flags = motor[m].flags;
		motor[m].step_pin = command(2);
		motor[m].dir_pin = command(3);
		motor[m].limit_min_pin = command(4);
		motor[m].limit_max_pin = command(5);
		motor[m].sense_pin = command(6);
		uint8_t const mask = Motor::INVERT_LIMIT_MIN | Motor::INVERT_LIMIT_MAX | Motor::INVERT_STEP;
		motor[m].flags &= ~mask;
		motor[m].flags |= command(7) & mask;
		arch_msetup(m);
		if (step != motor[m].step_pin || (flags & Motor::INVERT_STEP) != (motor[m].flags & Motor::INVERT_STEP)) {
			if (step < NUM_DIGITAL_PINS)
				UNSET(step);
			if (motor[m].step_pin < NUM_DIGITAL_PINS) {
				if (motor[m].flags & Motor::INVERT_STEP)
					SET(motor[m].step_pin);
				else
					RESET(motor[m].step_pin);
			}
		}
		if (dir != motor[m].dir_pin) {
			if (dir < NUM_DIGITAL_PINS)
				UNSET(dir);
			if (motor[m].dir_pin < NUM_DIGITAL_PINS)
				RESET(motor[m].dir_pin);
		}
		if (limit_min != motor[m].limit_min_pin) {
			if (limit_min < NUM_DIGITAL_PINS)
				UNSET(limit_min);
			if (motor[m].limit_min_pin < NUM_DIGITAL_PINS)
				SET_INPUT(motor[m].limit_min_pin);
		}
		if (limit_max != motor[m].limit_max_pin) {
			if (limit_max < NUM_DIGITAL_PINS)
				UNSET(limit_max);
			if (motor[m].limit_max_pin < NUM_DIGITAL_PINS)
				SET_INPUT(motor[m].limit_max_pin);
		}
		if (sense != motor[m].sense_pin) {
			if (sense < NUM_DIGITAL_PINS)
				UNSET(sense);
			if (motor[m].sense_pin < NUM_DIGITAL_PINS)
				SET_INPUT(motor[m].sense_pin);
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
			adc[a].value[i] = *reinterpret_cast <volatile uint16_t *>(&command(4 + 2 * i));
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
		if (step_state != 1 || stopping >= 0) {
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
			if (command(5 + m) == 1 || int8_t(command(5 + m)) == -1) {
				// Fill both sample 0 and 1, because the interrupt handler may change current_sample at any time.
				buffer[current_fragment][m][0] = command(5 + m);
				buffer[current_fragment][m][1] = command(5 + m);
				homers += 1;
				motor[m].flags |= Motor::ACTIVE;
			}
			else if (command(5 + m) == 0) {
				buffer[current_fragment][m][0] = 0x80;
				motor[m].flags &= ~Motor::ACTIVE;
			}
			else {
				debug("invalid dir in HOME: %d", command(5 + m));
				homers = 0;
				write_stall();
				return;
			}
		}
		if (homers > 0) {
			home_step_time = *reinterpret_cast <volatile uint32_t *>(&command(1));
			// current_sample is always 0 while homing; set len to 2, so it doesn't go to the next fragment.
			settings[current_fragment].len = 2;
			current_len = settings[current_fragment].len;
			settings[current_fragment].probing = false;
			current_buffer = &buffer[current_fragment];
			current_sample = 0;
			step_state = 0;
			set_speed(home_step_time);
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
		if (stopping >= 0) {
			//debug("ignoring start move while stopping");
			debug_add(1);
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
		settings[last_fragment].len = command(1);
		last_len = command(1);
		filling = command(2);
		for (uint8_t m = 0; m < active_motors; ++m)
			buffer[last_fragment][m][0] = 0x80;	// Sentinel indicating no data is available for this motor.
		settings[last_fragment].probing = command(0) == CMD_START_PROBE;
		if (filling == 0)
			last_fragment = (last_fragment + 1) & FRAGMENTS_PER_MOTOR_MASK;
		//debug("new filling: %d %d", filling, last_fragment);
		write_ack();
		return;
	}
	case CMD_MOVE:
	case CMD_AUDIO:
	{
		cmddebug("CMD_MOVE");
		if (command(1) >= NUM_MOTORS) {
			debug("invalid buffer %d to fill", command(1));
			write_stall();
			return;
		}
		if (stopping >= 0) {
			//debug("ignoring move while stopping");
			debug_add(0);
			write_ack();
			return;
		}
		if (filling == 0) {
			debug("MOVE seen while not filling");
			write_stall();
			return;
		}
		if (buffer[last_fragment][command(1)][0] != int8_t(0x80)) {
			debug("duplicate buffer %d to fill", command(1));
			write_stall();
			return;
		}
		motor[command(1)].audio = command(0) == CMD_AUDIO;
		for (uint8_t b = 0; b < last_len; ++b)
			buffer[last_fragment][command(1)][b] = static_cast<int8_t>(command(2 + b));
		filling -= 1;
		if (filling == 0)
			last_fragment = (last_fragment + 1) & FRAGMENTS_PER_MOTOR_MASK;
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
		if (step_state != 1) {
			debug("Received START while not stopped");
			write_stall();
			return;
		}
		if (current_fragment == last_fragment) {
			debug("Received START while buffer is empty");
			write_stall();
			return;
		}
		current_buffer = &buffer[current_fragment];
		for (uint8_t m = 0; m < active_motors; ++m) {
			if (buffer[current_fragment][m][0] != int8_t(0x80)) {
				motor[m].flags |= Motor::ACTIVE;
				motor[m].steps_current = 0;
			}
			else
				motor[m].flags &= ~Motor::ACTIVE;
		}
		current_sample = 0;
		current_len = settings[current_fragment].len;
		step_state = 0;
		set_speed(time_per_sample);
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
		set_speed(0);
		homers = 0;
		home_step_time = 0;
		reply[0] = CMD_STOPPED;
		reply[1] = current_sample;
		for (uint8_t m = 0; m < active_motors; ++m) {
			motor[m].flags &= ~Motor::ACTIVE;
			motor[m].steps_current = 0;
			*reinterpret_cast <int32_t *>(&reply[2 + 4 * m]) = motor[m].current_pos;
			//debug("cp %d %ld", m, F(motor[m].current_pos));
		}
		reply_ready = 2 + 4 * active_motors;
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
		if (filling > 0) {
			debug("DISCARD seen while filling");
			write_stall();
			return;
		}
		if (command(1) >= ((last_fragment - current_fragment) & FRAGMENTS_PER_MOTOR_MASK)) {
			debug("discarding more than entire buffer");
			write_stall();
			return;
		}
		last_fragment = (last_fragment - command(1)) & FRAGMENTS_PER_MOTOR_MASK;
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
	default:
	{
		debug("Invalid command %x %x %x %x", uint8_t(command(0)), uint8_t(command(1)), uint8_t(command(2)), uint8_t(command(3)));
		write_stall();
		return;
	}
	}
}

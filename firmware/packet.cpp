#include "firmware.h"

//#define DEBUG_CMD

void packet()
{
	switch (command[0])
	{
	case CMD_BEGIN:	// begin: request response
	{
#ifdef DEBUG_CMD
		debug("CMD_BEGIN");
#endif
		// A server is running; start the watchdog.
		watchdog_enable();
		write_ack();
		reply[0] = CMD_READY;
		reply[1] = 12;
		*reinterpret_cast <uint32_t *>(&reply[2]) = 0;
		reply[6] = NUM_DIGITAL_PINS;
		reply[7] = NUM_ANALOG_INPUTS;
		reply[8] = NUM_MOTORS;
		reply[9] = NUM_BUFFERS;
		reply[10] = FRAGMENTS_PER_BUFFER;
		reply[11] = BYTES_PER_FRAGMENT;
		reply_ready = 12;
		try_send_next();
		return;
	}
	case CMD_PING:
	{
#ifdef DEBUG_CMD
		//debug("CMD_PING");
#endif
		write_ack();
		ping |= 1 << command[1];
		try_send_next();
		return;
	}
	case CMD_RESET: // reset controller; used before reprogramming flash.
	{
#ifdef DEBUG_CMD
		debug("CMD_RESET");
#endif
		uint32_t magic = *reinterpret_cast <int32_t *>(&command[1]);
		if (magic != RESET_MAGIC) {
			debug("invalid reset magic %lx", F(magic));
			write_stall();
			return;
		}
		write_ack();
		Serial.flush();
		reset();
	}
	case CMD_SETUP:
	{
#ifdef DEBUG_CMD
		debug("CMD_SETUP");
#endif
		led_pin = command[1];
		time_per_sample = *reinterpret_cast <int32_t *>(&command[2]);
		write_ack();
		return;
	}
	case CMD_CONTROL:
	{
#ifdef DEBUG_CMD
		debug("CMD_CONTROL");
#endif
		uint8_t num = command[1];
		for (uint8_t i = 0; i < num; ++i) {
			if (command[2 + i + 1] >= NUM_DIGITAL_PINS) {
				debug("invalid pin in control: %d", command[2 + i + 1]);
				continue;
			}
			pin[command[2 + i + 1]].state &= ~0xc;
			pin[command[2 + i + 1]].state |= command[2 + i] & 0xc;
			switch (CONTROL_CURRENT(command[2 + i])) {
			case CTRL_RESET:
				RESET(command[2 + i + 1]);
				break;
			case CTRL_SET:
				SET(command[2 + i + 1]);
				break;
			case CTRL_UNSET:
				UNSET(command[2 + i + 1]);
				break;
			case CTRL_INPUT:
				SET_INPUT(command[2 + i + 1]);
				break;
			}
		}
		write_ack();
		return;
	}
	case CMD_MSETUP:
	{
#ifdef DEBUG_CMD
		debug("CMD_MSETUP");
#endif
		uint8_t m = command[1];
		if (m >= NUM_MOTORS) {
			debug("MSETUP called for invalid motor %d", m);
			write_stall();
			return;
		}
		motor[m].step_pin = command[2];
		motor[m].dir_pin = command[3];
		motor[m].limit_min_pin = command[4];
		motor[m].limit_max_pin = command[5];
		motor[m].sense_pin = command[6];
		if (motor[m].flags & Motor::ACTIVE)
			active_motors -= 1;
		uint8_t const mask = Motor::INVERT_LIMIT_MIN | Motor::INVERT_LIMIT_MAX | Motor::INVERT_STEP | Motor::ACTIVE;
		motor[m].flags &= ~mask;
		motor[m].flags |= command[7] & mask;
		if (motor[m].flags & Motor::ACTIVE)
			active_motors += 1;
		if (motor[m].flags & Motor::INVERT_STEP)
			SET(motor[m].step_pin);
		else
			RESET(motor[m].step_pin);
		if (motor[m].flags & Motor::INVERT_STEP)
			SET(motor[m].dir_pin);
		else
			RESET(motor[m].dir_pin);
		SET_INPUT(motor[m].limit_min_pin);
		SET_INPUT(motor[m].limit_max_pin);
		SET_INPUT(motor[m].sense_pin);
		write_ack();
		return;
	}
	case CMD_ASETUP:
	{
#ifdef DEBUG_CMD
		debug("CMD_ASETUP");
#endif
		uint8_t a = command[1];
		if (a >= NUM_ANALOG_INPUTS) {
			debug("ASETUP called for invalid adc %d", a);
			write_stall();
			return;
		}
		for (uint8_t i = 0; i < 2; ++i) {
			adc[a].linked[i] = command[2 + i];
			adc[a].value[i] = *reinterpret_cast <uint16_t *>(&command[4 + 2 * i]);
		}
		if (adc_phase == INACTIVE && (adc[a].value[0] & 0x8000) == 0) {
			adc_phase = PREPARING;
			adc_current = a;
			adc_next = a;
			adc_ready(a);
		}
		write_ack();
		return;
	}
	case CMD_START_MOVE:
	{
#ifdef DEBUG_CMD
		debug("CMD_START_MOVE");
#endif
		if (stopping) {
			debug("ignoring start move while stopping");
			write_ack();
			return;
		}
		if (filling > 0)
			debug("START_MOVE seen while filling (%d)", filling);
		last_fragment = (last_fragment + 1) % FRAGMENTS_PER_BUFFER;
		fragment_len[last_fragment] = command[1];
		filling = command[2];
		for (uint8_t m = 0; m < NUM_MOTORS; ++m) {
			if (!(motor[m].flags & Motor::ACTIVE))
				continue;
			buffer[motor[m].buffer][last_fragment].dir = DIR_NONE;
		}
		//debug("new filling: %d %d", filling, last_fragment);
		write_ack();
		return;
	}
	case CMD_MOVE:
	{
#ifdef DEBUG_CMD
		debug("CMD_MOVE");
#endif
		if (command[1] >= NUM_BUFFERS) {
			debug("invalid buffer %d to fill", command[1]);
			write_stall();
			return;
		}
		if (stopping) {
			debug("ignoring move while stopping");
			write_ack();
			return;
		}
		if (filling == 0)
			debug("MOVE seen while not filling");
		else
			filling -= 1;
		Fragment &fragment = buffer[command[1]][last_fragment];
		fragment.dir = Dir(command[2]);
		for (uint8_t b = 0; b < (fragment_len[last_fragment] + 1) / 2; ++b) {
			fragment.samples[b] = command[3 + b];
		}
		write_ack();
		return;
	}
	case CMD_START:
	{
#ifdef DEBUG_CMD
		debug("CMD_START");
#endif
		if (stopping) {
			debug("ignoring start while stopping");
			write_ack();
			return;
		}
		start_time = utime();
		current_fragment_pos = 0;
		stopped = false;
		write_ack();
		return;
	}
	case CMD_STOP:
	{
#ifdef DEBUG_CMD
		debug("CMD_STOP");
#endif
		stopped = true;
		write_ack();
		reply[0] = CMD_STOPPED;
		reply[1] = current_fragment;
		reply[2] = current_fragment_pos;
		uint8_t mi = 0;
		for (uint8_t m = 0; m < NUM_MOTORS; ++m) {
			if (!(motor[m].flags & Motor::ACTIVE))
				continue;
			*reinterpret_cast <uint32_t *>(&reply[3 + 4 * mi]) = motor[m].current_pos;
			++mi;
		}
		reply_ready = 3 + 4 * mi;
		try_send_next();
		return;
	}
	case CMD_ABORT:
	{
#ifdef DEBUG_CMD
		debug("CMD_ABORT");
#endif
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
		stopped = true;
		write_ack();
		reply[0] = CMD_STOPPED;
		reply[1] = current_fragment;
		reply[2] = current_fragment_pos;
		uint8_t mi = 0;
		for (uint8_t m = 0; m < NUM_MOTORS; ++m) {
			if (!(motor[m].flags & Motor::ACTIVE)) {
				debug("skip abortpos %d", m);
				continue;
			}
			*reinterpret_cast <uint32_t *>(&reply[3 + 4 * mi]) = motor[m].current_pos;
			debug("abort pos %d %ld", m, motor[m].current_pos);
			++mi;
		}
		reply_ready = 3 + 4 * mi;
		try_send_next();
		return;
	}
	case CMD_GETPIN:
	{
#ifdef DEBUG_CMD
		debug("CMD_GETPIN");
#endif
		if (command[1] >= NUM_DIGITAL_PINS) {
			debug("invalid pin %02x for GETPIN", uint8_t(command[1]));
			write_stall();
			return;
		}
		write_ack();
		reply[0] = CMD_PIN;
		reply[1] = GET(command[1]);
		reply_ready = 2;
		try_send_next();
		return;
	}
	default:
	{
		debug("Invalid command %x %x %x %x", uint8_t(command[0]), uint8_t(command[1]), uint8_t(command[2]), uint8_t(command[3]));
		write_stall();
		return;
	}
	}
}

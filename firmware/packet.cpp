#include "firmware.h"

//#define DEBUG_CMD

static uint8_t get_which()
{
	return command[2] & 0x3f;
}

void packet()
{
	// command[0] is the length not including checksum bytes.
	// command[1] is the command.
	uint8_t which;
	switch (command[1])
	{
	case CMD_BEGIN:	// begin: request response
	{
#ifdef DEBUG_CMD
		debug("CMD_BEGIN");
#endif
		// A server is running; start the watchdog.
		watchdog_enable();
		for (uint8_t i = 0; i < ID_SIZE; ++i)
			printerid[i] = command[2 + i];
		write_ack();
		reply[0] = 2 + sizeof(uint32_t) + 2 + sizeof(uint16_t) + 1;
		reply[1] = CMD_START;
		*reinterpret_cast <uint32_t *>(&reply[2]) = 0;
		reply[6] = NUM_DIGITAL_PINS;
		reply[7] = NUM_ANALOG_INPUTS;
		*reinterpret_cast <uint16_t *>(&reply[8]) = E2END;
		reply[10] = ADCBITS;
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_PING:
	{
#ifdef DEBUG_CMD
		//debug("CMD_PING");
#endif
		write_ack();
		ping |= 1 << command[2];
		try_send_next();
		return;
	}
	case CMD_RESET: // reset controller; used before reprogramming flash.
	{
#ifdef DEBUG_CMD
		debug("CMD_RESET");
#endif
		write_ack();
		Serial.flush();
		reset();
	}
	case CMD_SETUP:
	{
#ifdef DEBUG_CMD
		debug("CMD_SETUP");
#endif
		uint8_t nm = command[2];
		Motor **mtrs;
		mem_alloc(nm * sizeof(Motor *), &mtrs, "motor array");
		if (mtrs) {
			uint8_t cm;
			for (cm = num_motors; cm < nm; ++cm) {
				mem_alloc(sizeof(Motor), &mtrs[cm], "single motor");
				if (!mtrs[cm])
					break;
			}
			if (cm >= nm) {
				for (cm = nm; cm < num_motors; ++cm) {
					motor[cm]->fini();
					mem_free(&motor[cm]);
				}
				for (cm = num_motors; cm < nm; ++cm)
					mtrs[cm]->init();
				for (cm = 0; cm < min(nm, num_motors); ++cm)
					mem_retarget(&motor[cm], &mtrs[cm]);
				mem_free(&motor);
				mem_retarget(&mtrs, &motor);
				num_motors = nm;
				led_pin.read(*reinterpret_cast <uint16_t *>(&command[3]));
				speed_error = command[5];
				SET_OUTPUT(led_pin);
				write_ack();
				return;
			}
			for (uint8_t cm2 = num_motors; cm2 < cm; ++cm2) {
				mtrs[cm2]->fini();
				mem_free(&mtrs[cm2]);
			}
			mem_free(&mtrs);
		}
		debug("Out of memory for setup %d motors", nm);
		write_stall();
		return;
	}
	case CMD_MSETUP:
	{
#ifdef DEBUG_CMD
		debug("CMD_MSETUP");
#endif
		which = get_which();
		if (which >= num_motors) {
			debug("MSETUP called for invalid motor %d", which);
			write_stall();
			return;
		}
		motor[which]->step_pin.read(*reinterpret_cast <uint16_t *>(&command[3]));
		//debug("m %d step pin %d %d", which, motor[which]->step_pin.pin, motor[which]->step_pin.flags);
		motor[which]->dir_pin.read(*reinterpret_cast <uint16_t *>(&command[5]));
		motor[which]->limit_min_pin.read(*reinterpret_cast <uint16_t *>(&command[7]));
		motor[which]->limit_max_pin.read(*reinterpret_cast <uint16_t *>(&command[9]));
		motor[which]->sense_pin.read(*reinterpret_cast <uint16_t *>(&command[11]));
		motor[which]->max_steps = command[13];
		motor[which]->max_v = *reinterpret_cast <float *>(&command[14]);
		motor[which]->a = *reinterpret_cast <float *>(&command[18]);
		//debug("limits %d %f %f", which, F(motor[which]->max_v), F(motor[which]->a));
		SET_OUTPUT(motor[which]->step_pin);
		SET_OUTPUT(motor[which]->dir_pin);
		SET_INPUT(motor[which]->limit_min_pin);
		SET_INPUT(motor[which]->limit_max_pin);
		SET_INPUT(motor[which]->sense_pin);
		write_ack();
		return;
	}
	case CMD_MOVE:
	{
#ifdef DEBUG_CMD
		debug("CMD_MOVE");
#endif
		if (stopping) {
			// Ignore all moves while stopping.
			write_ack();
			return;
		}
		uint8_t whichlen = ((num_motors - 1) >> 3) + 1;
		uint8_t current = 2 + whichlen;
		uint32_t now = utime();
		for (uint8_t m = 0; m < num_motors; ++m) {
			if (!(command[2 + (m >> 3)] & (1 << (m & 0x7)))) {
				if (motor[m]->target_v != 0) {
					motor[m]->start_pos += motor[m]->target_v * (now - start_time);
					motor[m]->target_v = 0;
				}
				continue;
			}
			motor[m]->target_v = *reinterpret_cast <float *>(&command[current]);
			motor[m]->end_pos = *reinterpret_cast <int32_t *>(&command[current + sizeof(float)]);
			motor[m]->start_pos = *reinterpret_cast <int32_t *>(&command[current + sizeof(int32_t) + sizeof(float)]);
			motor[m]->on_track = false;
			current += sizeof(int32_t) * 2 + sizeof(float);
			//debug("m %d cp %ld ep %ld sp %ld v %f", m, F(motor[m]->current_pos), F(motor[m]->end_pos), F(motor[m]->start_pos), F(motor[m]->v));
		}
		start_time = now;
		write_ack();
		return;
	}
	case CMD_ABORT:
	{
#ifdef DEBUG_CMD
		debug("CMD_ABORT");
#endif
		for (uint8_t m = 0; m < num_motors; ++m) {
			motor[m]->start_pos = motor[m]->current_pos;
			motor[m]->target_v = 0;
		}
		write_ack();
		return;
	}
	case CMD_SETPOS:	// Set current position
	{
#ifdef DEBUG_CMD
		debug("CMD_SETPOS");
#endif
		which = get_which();
		if (which >= num_motors) {
			debug("SETPOS called for invalid motor %d", which);
			write_stall();
			return;
		}
		if (motor[which]->target_v != 0) {
			debug("SETPOS called for moving motor %d (v=%f)", which, F(motor[which]->target_v));
			write_stall();
			return;
		}
		int32_t diff = *reinterpret_cast <int32_t *>(&command[3]) - motor[which]->start_pos;
		motor[which]->current_pos += diff;
		// Don't start moving.
		debug("set %d start %ld diff %ld", which, F(motor[which]->start_pos), F(diff));
		motor[which]->start_pos += diff;
		write_ack();
		return;
	}
	case CMD_ADDPOS:
	{
#ifdef DEBUG_CMD
		debug("CMD_ADDPOS");
#endif
		which = get_which();
		if (which >= num_motors) {
			debug("SETPOS called for invalid motor %d", which);
			write_stall();
			return;
		}
		int32_t diff = *reinterpret_cast <int32_t *>(&command[3]);
		motor[which]->current_pos += diff;
		debug("add %d start %ld diff %ld", which, F(motor[which]->start_pos), F(diff));
		motor[which]->start_pos += diff;
		write_ack();
		return;
	}
	case CMD_GETPOS:	// Get current position
	{
#ifdef DEBUG_CMD
		debug("CMD_GETPOS");
#endif
		which = get_which();
		if (which >= num_motors) {
			debug("GETPOS called for invalid motor %d", which);
			write_stall();
			return;
		}
		write_ack();
		reply[0] = 6;
		reply[1] = CMD_POS;
		*reinterpret_cast <int32_t *>(&reply[2]) = motor[which]->current_pos;
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_RESETPIN:
	{
#ifdef DEBUG_CMD
		debug("CMD_RESETPIN");
#endif
		if (command[2] >= NUM_DIGITAL_PINS) {
			debug("RESETPIN called for invalid pin %d", command[2]);
			write_stall();
			return;
		}
		RAW_SET_OUTPUT(command[2]);
		RAW_RESET(command[2]);
		write_ack();
		return;
	}
	case CMD_SETPIN:
	{
#ifdef DEBUG_CMD
		debug("CMD_SETPIN");
#endif
		if (command[2] >= NUM_DIGITAL_PINS) {
			debug("SETPIN called for invalid pin %d", command[2]);
			write_stall();
			return;
		}
		RAW_SET_OUTPUT(command[2]);
		RAW_SET(command[2]);
		write_ack();
		return;
	}
	case CMD_UNSETPIN:
	{
#ifdef DEBUG_CMD
		debug("CMD_UNSETPIN");
#endif
		if (command[2] >= NUM_DIGITAL_PINS) {
			debug("UNSETPIN called for invalid pin %d", command[2]);
			write_stall();
			return;
		}
		RAW_SET_INPUT_NOPULLUP(command[2]);
		write_ack();
		return;
	}
	case CMD_INPUTPIN:
	{
#ifdef DEBUG_CMD
		debug("CMD_INPUTPIN");
#endif
		if (command[2] >= NUM_DIGITAL_PINS) {
			debug("INPUTPIN called for invalid pin %d", command[2]);
			write_stall();
			return;
		}
		RAW_SET_INPUT(command[2]);
		write_ack();
		return;
	}
	case CMD_GETPIN:
	{
#ifdef DEBUG_CMD
		debug("CMD_GETPIN");
#endif
		if (command[2] >= NUM_DIGITAL_PINS) {
			debug("Reading invalid pin %d", command[2]);
			write_stall();
			return;
		}
		write_ack();
		reply[0] = 3;
		reply[1] = CMD_PIN;
		reply[2] = RAW_GET(command[2]);
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_GETADC:
	{
#ifdef DEBUG_CMD
		debug("CMD_GETADC");
#endif
		if (command[2] >= NUM_ANALOG_INPUTS) {
			debug("Reading invalid adc %d", command[2]);
			write_stall();
			return;
		}
		//debug("get adc %d", command[2]);
		adc_current = command[2];
		adc_start(command[2]);
		write_ack();
		return;
	}
#ifdef HAVE_AUDIO
	case CMD_AUDIO_SETUP:
	{
#ifdef DEBUG_CMD
		debug("CMD_AUDIO_SETUP");
#endif
		audio_us_per_sample = get_int16(2);
		uint8_t m0 = 0;
		for (uint8_t t = 0; t < num_spaces; ++t) {
			for (uint8_t m = 0; m < spaces[t].num_motors; ++m) {
				if (command[4 + ((m0 + m) >> 3)] & (1 << ((m0 + m) & 0x7))) {
					spaces[t].motor[m]->audio_flags |= Motor::PLAYING;
					SET(spaces[t].motor[m]->enable_pin);
					motors_busy = true;
				}
				else
					spaces[t].motor[m]->audio_flags &= ~Motor::PLAYING;
			}
			m0 += spaces[t].num_motors;
		}
		// Abort any currently playing sample.
		audio_head = 0;
		audio_tail = 0;
		write_ack();
		return;
	}
	case CMD_AUDIO_DATA:
	{
#ifdef DEBUG_CMD
		debug("CMD_AUDIO_DATA");
#endif
		initialized = true;
		last_active = millis();
		if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
		{
			debug("Audio buffer is full");
			write_stall();
			return;
		}
		for (uint8_t i = 0; i < AUDIO_FRAGMENT_SIZE; ++i)
			audio_buffer[audio_tail][i] = command[2 + i];
		if (audio_tail == audio_head)
			audio_start = utime();
		audio_tail = (audio_tail + 1) % AUDIO_FRAGMENTS;
		if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
			write_ackwait();
		else
			write_ack();
		next_audio_time = 0;
		return;
	}
#endif
	default:
	{
		debug("Invalid command %x %x %x %x", command[0], command[1], command[2], command[3]);
		write_stall();
		return;
	}
	}
}

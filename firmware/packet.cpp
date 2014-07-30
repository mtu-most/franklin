#include "firmware.h"

//#define DEBUG_CMD

static uint8_t get_which()
{
	return command[2] & 0x3f;
}

#if MAXTEMPS > 0 || MAXEXTRUDERS > 0 || MAXAXES > 0
static int32_t get_int32(uint8_t offset)
{
	int32_t ret = 0;
	for (uint8_t t = 0; t < 4; ++t)
		ret |= ((uint32_t)command[offset + t] & 0xff) << (8 * t);
	return ret;
}
#endif

#if defined(AUDIO) || !defined(LOWMEM)
static int16_t get_int16(uint8_t offset)
{
	return ((uint16_t)command[offset] & 0xff) | (uint16_t)command[offset + 1] << 8;
}
#endif

void packet()
{
	// command[0] is the length not including checksum bytes.
	// command[1] is the command.
	uint8_t which;
	int16_t addr;
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
			printerid[i] = command[6 + i];
		write_ack();
		reply[0] = 6;
		reply[1] = CMD_START;
		reply[2] = 0;
		reply[3] = 0;
		reply[4] = 0;
		reply[5] = 0;
		reply_ready = true;
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
#if MAXEXTRUDERS > 0 || MAXAXES > 0
	case CMD_GOTO:	// goto
	case CMD_GOTOCB:	// goto with callback
	{
#ifdef DEBUG_CMD
		debug("CMD_GOTO(CB)");
#endif
		last_active = millis();
		if (queue_full)
		{
			debug("Host ignores wait request");
			Serial.write(CMD_STALL);
			return;
		}
		uint8_t const num = 2 + num_axes + num_extruders;
		uint8_t const offset = 2 + ((num - 1) >> 3) + 1;	// Bytes from start of command where values are.
		uint8_t t = 0;
		for (uint8_t ch = 0; ch < num; ++ch)
		{
			if (command[2 + (ch >> 3)] & (1 << (ch & 0x7)))
			{
				ReadFloat f;
				for (uint8_t i = 0; i < sizeof(int32_t); ++i)
					f.b[i] = command[offset + i + t * sizeof(int32_t)];
#if MAXAXES > 0
				if (ch >= 2 && ch < 2 + num_axes && axis[ch - 2].current_pos == MAXLONG) {
					debug("Moving uninitialized axis %d", ch - 2);
					Serial.write(CMD_STALL);
					return;
				}
#endif
				if (ch < 2) {
					queue[queue_end].f[ch] = f.f;
					debug("goto %d %f", ch, F(f.f));
				}
				else {
					queue[queue_end].data[ch - 2] = f.i;
					debug("goto %d %ld", ch, F(f.i));
				}
				initialized = true;
				++t;
			}
			else {
				if (ch < 2)
					queue[queue_end].f[ch] = NAN;
				else
					queue[queue_end].data[ch - 2] = MAXLONG;
			}
		}
		if (!(command[2] & 0x1) || isnan(queue[queue_end].f[0]))
			queue[queue_end].f[0] = INFINITY;
		if (!(command[2] & 0x2) || isnan(queue[queue_end].f[1]))
			queue[queue_end].f[1] = queue[queue_end].f[0];
		// f0 and f1 must be valid.
		float f0 = queue[queue_end].f[0];
		float f1 = queue[queue_end].f[1];
		if (f0 < 0 || f1 < 0 || (f0 == 0 && f1 == 0))
		{
			debug("Invalid f0 or f1: %f %f", F(f0), F(f1));
			Serial.write(CMD_STALL);
			return;
		}
		queue[queue_end].cb = command[1] == CMD_GOTOCB;
		queue_end = (queue_end + 1) % QUEUE_LENGTH;
		if (queue_end == queue_start) {
			queue_full = true;
			write_ackwait();
		}
		else
			write_ack();
		if (!moving)
			next_move();
		//else
			//debug("waiting with move");
		break;
	}
	case CMD_RUN:	// run motor
	{
#ifdef DEBUG_CMD
		debug("CMD_RUN");
#endif
		last_active = millis();
		which = get_which();
		// Motor must exist, must not be doing a move.
		if (!motors[which])
		{
			debug("Running invalid motor %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		if (moving_motor(which))
		{
			debug("Running moving motor %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		ReadFloat f;
		for (uint8_t i = 0; i < sizeof(int32_t); ++i) {
			f.b[i] = command[3 + i];
		}
		if (!isnan(f.f) && f.f != 0)
			initialized = true;
		write_ack();
		//debug("running %d", which);
#if MAXAXES >= 3
		if (printer_type == 1 && which < 2 + 3) {
			for (uint8_t a = 0; a < 3; ++a) {
				axis[a].source = MAXLONG;
				axis[a].current = MAXLONG;
			}
		}
		else
#endif
#if MAXAXES > 0
		if (which < 2 + MAXAXES) {
			axis[which - 2].source = MAXLONG;
			axis[which - 2].current = MAXLONG;
		}
#endif
		if (isnan(f.f))
			f.f = 0;
		//debug("at speed %f", F(f.f));
		if (f.f > 0) {
			if (f.f > motors[which]->limit_v)
				f.f = motors[which]->limit_v;
		}
		else if (f.f < 0) {
			if (f.f < -motors[which]->limit_v)
				f.f = -motors[which]->limit_v;
		}
		//debug("I mean at speed %f", F(f.f));
		if (motors[which]->f != 0) {
			//debug("running changes speed from %f to %f", F(motors[which]->f), F(f.f));
		       	if ((motors[which]->positive && f.f < 0) || (!motors[which]->positive && f.f > 0))
				motors[which]->continuous_v = -abs(f.f);
			else
				motors[which]->continuous_v = abs(f.f);
		}
		else {
			//debug("new running speed %ld", F(f.i));
			if (f.i > 0) {
				SET(motors[which]->dir_pin);
				motors[which]->continuous_v = f.f;
				motors[which]->positive = true;
			}
			else {
				RESET(motors[which]->dir_pin);
				motors[which]->continuous_v = -f.f;
				motors[which]->positive = false;
			}
			//debug("initial positive %d", motors[which]->positive);
			motors[which]->last_time = micros();
			SET(motors[which]->enable_pin);
			motors[which]->continuous_f = 0;
			motors_busy |= 1 << which;
		}
		return;
	}
	case CMD_SLEEP:	// disable motor current
	{
#ifdef DEBUG_CMD
		debug("CMD_SLEEP");
#endif
		last_active = millis();
		which = get_which();
		if (!motors[which])
		{
			debug("Sleeping invalid motor %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		if (moving_motor(which))
		{
			debug("Sleeping moving axis %d", which - 2);
			Serial.write(CMD_STALL);
			return;
		}
		if (command[2] & 0x80) {
			RESET(motors[which]->enable_pin);
			motors_busy &= ~(1 << which);
#if MAXAXES >= 3
			if (printer_type == 1 && which < 2 + 3) {
				for (uint8_t a = 2; a < 2 + 3; ++a) {
					axis[a - 2].current_pos = MAXLONG;
					axis[a - 2].source = MAXLONG;
					axis[a - 2].current = MAXLONG;
				}
			}
			else
#endif
#if MAXAXES > 0
			if (which < 2 + MAXAXES) {
				axis[which - 2].current_pos = MAXLONG;
				axis[which - 2].source = MAXLONG;
				axis[which - 2].current = MAXLONG;
			}
#endif
		}
		else {
			SET(motors[which]->enable_pin);
			motors_busy |= 1 << which;
		}
		write_ack();
		return;
	}
#endif
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
	case CMD_SETTEMP:	// set target temperature and enable control
	{
#ifdef DEBUG_CMD
		debug("CMD_SETTEMP");
#endif
		last_active = millis();
		which = get_which();
		if (!temps[which])
		{
			debug("Setting invalid temp %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		temps[which]->target = get_int32(3);
		temps[which]->adctarget = temps[which]->toadc(temps[which]->target);
		debug("adc target %d from %d", temps[which]->adctarget, int16_t(temps[which]->target / 1024));
		if (temps[which]->adctarget >= MAXINT) {
			// loop() doesn't handle it anymore, so it isn't disabled there.
			//debug("Temp %d disabled", which);
			if (temps[which]->is_on) {
				RESET(temps[which]->power_pin);
				temps[which]->is_on = false;
				--temps_busy;
			}
		}
		else if (temps[which]->adctarget < 0) {
			// loop() doesn't handle it anymore, so it isn't enabled there.
			//debug("Temp %d enabled", which);
			if (!temps[which]->is_on) {
				SET(temps[which]->power_pin);
				temps[which]->is_on = true;
				++temps_busy;
			}
		}
		else {
			//debug("Temp %d set to %ld", which, F(target));
			initialized = true;
		}
		adc_phase = 1;
		write_ack();
		return;
	}
	case CMD_WAITTEMP:	// wait for a temperature sensor to reach a target range
	{
#ifdef DEBUG_CMD
		debug("CMD_WAITTEMP");
#endif
		initialized = true;
		which = get_which();
		if (!temps[which])
		{
			debug("Waiting for invalid temp %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		ReadFloat min, max;
		for (uint8_t i = 0; i < sizeof(int32_t); ++i)
		{
			min.b[i] = command[3 + i];
			max.b[i] = command[3 + i + sizeof(int32_t)];
		}
		temps[which]->min_alarm = min.i;
		temps[which]->max_alarm = max.i;
		temps[which]->adcmin_alarm = temps[which]->toadc(temps[which]->min_alarm);
		temps[which]->adcmax_alarm = temps[which]->toadc(temps[which]->max_alarm);
		write_ack();
		adc_phase = 1;
		return;
	}
	case CMD_READTEMP:	// read temperature
	{
#ifdef DEBUG_CMD
		debug("CMD_READTEMP");
#endif
		which = get_which();
		if (!temps[which])
		{
			debug("Reading invalid temp %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		write_ack();
		ReadFloat f;	// TODO: start a measurement and report when it's ready.
		f.i = temps[which]->fromadc(temps[which]->adclast);
		debug("read temp %ld", F(f.i));
		reply[0] = 2 + sizeof(int32_t);
		reply[1] = CMD_TEMP;
		for (uint8_t b = 0; b < sizeof(int32_t); ++b)
			reply[2 + b] = f.b[b];
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_READPOWER:	// read used power
	{
#ifdef DEBUG_CMD
		debug("CMD_READPOWER");
#endif
		which = get_which();
		if (!temps[which])
		{
			debug("Reading power of invalid temp %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		write_ack();
		reply[0] = 10;
		reply[1] = CMD_POWER;
		ReadFloat on, current;
		unsigned long t = micros();
		if (temps[which]->is_on) {
			// This causes an insignificant error in the model, but when using this you probably aren't using the model anyway, and besides you won't notice the error even if you do.
			temps[which]->time_on += t - temps[which]->last_time;
			temps[which]->last_time = t;
		}
		on.ui = temps[which]->time_on;
		current.ui = t;
		for (uint8_t b = 0; b < sizeof(unsigned long); ++b)
		{
			reply[2 + b] = on.b[b];
			reply[6 + b] = current.b[b];
		}
	}
#endif
#if MAXAXES > 0
	case CMD_SETPOS:	// Set current position
	{
#ifdef DEBUG_CMD
		debug("CMD_SETPOS");
#endif
		last_active = millis();
		which = get_which();
		if (which < 2 || which > 2 + MAXAXES)
		{
			debug("Setting position of non-axis component %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		if (!(motors_busy & 1 << which))
		{
			debug("Setting position of sleeping axis %d (busy %x)", which - 2, motors_busy);
			Serial.write(CMD_STALL);
			return;
		}
		if (moving_motor(which))
		{
			debug("Setting position of moving axis %d", which - 2);
			Serial.write(CMD_STALL);
			return;
		}
#if MAXAXES >= 3
		if (printer_type == 1 && which <= 2 + 3) {
			for (uint8_t a = 0; a < 3; ++a) {
				axis[a].source = MAXLONG;
				axis[a].current = MAXLONG;
			}
		}
		else
#endif
		{
			axis[which - 2].source = MAXLONG;
			axis[which - 2].current = MAXLONG;
		}
		write_ack();
		axis[which - 2].current_pos = get_int32(3);
		return;
	}
	case CMD_GETPOS:	// Get current position
	{
#ifdef DEBUG_CMD
		debug("CMD_GETPOS");
#endif
		which = get_which();
		if (which < 2 || which > 2 + MAXAXES)
		{
			debug("Getting position of invalid axis %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		write_ack();
		if (axis[which - 2].source == MAXLONG)
			reset_pos();
		ReadFloat pos, current;
		pos.i = axis[which - 2].current_pos;
		current.i = axis[which - 2].current - axis[which - 2].offset;
		reply[0] = 2 + 2 * sizeof(int32_t);
		reply[1] = CMD_POS;
		for (uint8_t b = 0; b < sizeof(int32_t); ++b)
			reply[2 + b] = pos.b[b];
		for (uint8_t b = 0; b < sizeof(int32_t); ++b)
			reply[2 + sizeof(int32_t) + b] = current.b[b];
		reply_ready = true;
		try_send_next();
		return;
	}
#endif
	case CMD_LOAD:	// reload settings from eeprom
	{
#ifdef DEBUG_CMD
		debug("CMD_LOAD");
#endif
		which = get_which();
		if (which < 1 || which >= MAXOBJECT)
		{
			debug("Loading invalid object %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		addr = objects[which]->address;
		objects[which]->load(addr, true);
		write_ack();
		return;
	}
	case CMD_SAVE:	// save settings to eeprom
	{
#ifdef DEBUG_CMD
		debug("CMD_SAVE");
#endif
		which = get_which();
		if (which < 1 || which >= MAXOBJECT)
		{
			debug("Saving invalid object %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		if (moving)
		{
			debug("Saving %d while moving would disrupt move", which);
			Serial.write(CMD_STALL);
			return;
		}
		addr = objects[which]->address;
		objects[which]->save(addr, true);
		write_ack();
		return;
	}
	case CMD_READ:	// reply settings to host
	{
#ifdef DEBUG_CMD
		debug("CMD_READ");
#endif
		which = get_which();
		if (which < 0 || which >= MAXOBJECT)
		{
			debug("Reading invalid object %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		addr = 2;
		objects[which]->save(addr, false);
		reply[0] = addr;
		reply[1] = CMD_DATA;
		write_ack();
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_WRITE:	// change settings from host
	{
		which = get_which();
#ifdef DEBUG_CMD
		debug("CMD_WRITE %d", which);
#endif
		if (which < 1 || which >= MAXOBJECT)
		{
			debug("Writing invalid object %d", which);
			Serial.write(CMD_STALL);
			return;
		}
		addr = 3;
		objects[which]->load(addr, false);
		write_ack();
		return;
	}
	case CMD_QUEUED:
	{
#ifdef DEBUG_CMD
		debug("CMD_QUEUED");
#endif
		last_active = millis();
		reply[0] = 3;
		reply[1] = CMD_QUEUE;
		reply[2] = queue_full ? QUEUE_LENGTH : (queue_end - queue_start + QUEUE_LENGTH) % QUEUE_LENGTH;
		if (command[2]) {
			queue_start = 0;
			queue_end = 0;
			queue_full = false;
			//debug("aborting at request");
			abort_move();
		}
		write_ack();
		reply_ready = true;
		try_send_next();
		return;
	}
	case CMD_PING:
	{
#ifdef DEBUG_CMD
		debug("CMD_PING");
#endif
		write_ack();
		ping |= 1 << command[2];
		try_send_next();
		return;
	}
#if MAXGPIOS > 0
	case CMD_READGPIO:
	{
#ifdef DEBUG_CMD
		debug("CMD_READGPIO");
#endif
		if (command[2] < 2 + MAXAXES + MAXEXTRUDERS + MAXTEMPS || command[2] >= 2 + MAXAXES + MAXEXTRUDERS + MAXTEMPS + MAXGPIOS)
		{
			debug("GETGPIO called for non-gpio channel %d", command[2]);
			Serial.write(CMD_STALL);
			return;
		}
		write_ack();
		reply[0] = 3;
		reply[1] = CMD_PIN;
		reply[2] = GET(gpio[command[2] - (2 + MAXAXES + MAXEXTRUDERS + MAXTEMPS)].pin, false) ? 1 : 0;
		reply_ready = true;
		try_send_next();
		return;
	}
#endif
#ifdef AUDIO
	case CMD_AUDIO_SETUP:
	{
#ifdef DEBUG_CMD
		debug("CMD_AUDIO_SETUP");
#endif
		last_active = millis();
		audio_us_per_bit = get_int16(2);
		for (uint8_t a = 0; a < num_axes; ++a)
		{
			if (command[4 + ((a + 2) >> 3)] & (1 << ((a + 2) & 0x7))) {
				axis[a].motor.audio_flags |= Motor::PLAYING;
				SET(axis[a].motor.enable_pin);
				motors_busy |= 1 << (a + 2);
			}
			else
				axis[a].motor.audio_flags &= ~Motor::PLAYING;
		}
		for (uint8_t e = 0; e < num_extruders; ++e)
		{
			if (command[4 + ((e + 2 + num_axes) >> 3)] & (1 << ((e + 2 + num_axes) & 0x7))) {
				extruder[e].motor.audio_flags |= Motor::PLAYING;
				SET(extruder[e].motor.enable_pin);
				motors_busy |= 1 << (2 + MAXAXES + e);
			}
			else
				extruder[e].motor.audio_flags &= ~Motor::PLAYING;
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
			Serial.write(CMD_STALL);
			return;
		}
		for (uint8_t i = 0; i < AUDIO_FRAGMENT_SIZE; ++i)
			audio_buffer[audio_tail][i] = command[2 + i];
		if (audio_tail == audio_head)
			audio_start = micros();
		audio_tail = (audio_tail + 1) % AUDIO_FRAGMENTS;
		if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
			write_ackwait();
		else
			write_ack();
		return;
	}
#endif
	default:
	{
		debug("Invalid command %x %x %x %x", command[0], command[1], command[2], command[3]);
		Serial.write(CMD_STALL);
		return;
	}
	}
}

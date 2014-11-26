// vim: set filetype=cpp foldmethod=marker foldmarker={,} :
#include "firmware.h"

static void handle_motors(uint32_t current_time) {
	for (uint8_t m = 0; m < num_motors; ++m) {
		// Check sense pins.
		if (motor[m]->sense_pin.valid()) { // Activate delayed setpos.
			if (GET(motor[m]->sense_pin, false) ^ bool(motor[m]->sense_state & 0x80)) {
				motor[m]->sense_state ^= 0x80;
				motor[m]->sense_state |= 1;
				motor[m]->sense_pos = motor[m]->current_pos;
				try_send_next();
			}
		}
		// If arrived, activate delayed setpos.
		if (motor[m]->v == 0 && motor[m]->start_pos == motor[m]->current_pos) {
			motor[m]->last_step_t = current_time;
			if (motor[m]->setpos != MAXLONG) {
				debug("setpos %d delayed %ld", m, F(motor[m]->setpos));
				motor[m]->current_pos = motor[m]->setpos;
				motor[m]->start_pos = motor[m]->setpos;
				motor[m]->setpos = MAXLONG;
				num_setpos -= 1;
				write_ack();
			}
			continue;
		}
		// target is where we should be without any limits applied.
		int32_t target = motor[m]->start_pos + motor[m]->target_v * (current_time - start_time);
		// dt is used for converting v and a.
		uint32_t dt = current_time - motor[m]->last_step_t;
		// Try to get on track if we're not.
		if (!motor[m]->on_track) {
			// Sign of the target v.
			int8_t s = motor[m]->target_v < 0 ? -1 : 1;
			// If moving in the target direction and the number of steps to reach the target is allowed, be on track.
			if ((target - motor[m]->current_pos) * s >= 0 && (target - motor[m]->current_pos) * s <= motor[m]->max_steps) {
				motor[m]->on_track = true;
				motor[m]->v = motor[m]->target_v;
				//debug("n %d %f", m, F(motor[m]->v));
			}
			else {
				// Cannot be on track yet; try to get there.
				// tt is the time it takes to change from v to target_v with maximum a.
				uint32_t tt = fabs(motor[m]->v - motor[m]->target_v) / motor[m]->a;
				// if target position after tt < actual position after this speed change, we need to correct negatively; otherwise positively.
				if (target + motor[m]->target_v * tt < motor[m]->current_pos + (motor[m]->target_v + motor[m]->v) / 2 * tt) {
					// Use -a.
					// When moving to a rest position, allow any move; otherwise, refuse to correct against requested direction; stand still instead.
					if (motor[m]->target_v == 0)
						motor[m]->v = motor[m]->v - motor[m]->a * dt;
					else
						motor[m]->v = max(0, fabs(motor[m]->v - motor[m]->a * dt)) * s;
					// Don't go faster than max_v.
					if (motor[m]->v < -motor[m]->max_v)
						motor[m]->v = -motor[m]->max_v;
				}
				else {
					// Use +a.
					// When moving to a rest position, allow any move; otherwise, refuse to correct against requested direction; stand still instead.
					if (motor[m]->target_v == 0)
						motor[m]->v = motor[m]->v + motor[m]->a * dt;
					else
						motor[m]->v = max(0, fabs(motor[m]->v + motor[m]->a * dt)) * s;
					// Don't go faster than max_v.
					if (motor[m]->v > motor[m]->max_v)
						motor[m]->v = motor[m]->max_v;
				}
				// Using this new v, determine the corrected target.
				target = motor[m]->current_pos + motor[m]->v * dt;
			}
		}
		int32_t steps = target - motor[m]->current_pos;
		// If this takes the position further than end_pos, stop there.
		if ((target >= motor[m]->end_pos && motor[m]->end_pos >= motor[m]->start_pos)
				|| (target <= motor[m]->end_pos && motor[m]->end_pos <= motor[m]->start_pos)) {
			//debug("finish %d %ld %ld %ld %ld %f %ld %ld", m, F(motor[m]->current_pos), F(target), F(motor[m]->end_pos), F(motor[m]->start_pos), F(motor[m]->target_v), F(current_time), F(start_time));
			steps = motor[m]->end_pos - motor[m]->current_pos;
			target = motor[m]->current_pos + steps;
		}
		// Limit steps per iteration.
		if (abs(steps) > motor[m]->max_steps) {
			steps = motor[m]->max_steps * (steps > 0 ? 1 : -1);
			target = motor[m]->current_pos + steps;
			motor[m]->on_track = false;	// No longer on track.
			motor[m]->v = (motor[m]->v < 0 ? -1 : 1) * motor[m]->max_steps * 1. / dt;
			//debug("nv %d %f %ld %ld", m, F(motor[m]->v), F(steps), F(dt));
		}
		if (steps == 0)
			continue;
		// Update time for future dt computations.
		motor[m]->last_step_t = current_time;
		// Check limit switches.
		if (steps > 0 ? GET(motor[m]->limit_max_pin, false) : GET(motor[m]->limit_min_pin, false)) {
			// Hit endstop; abort current move and notify host.
			int32_t ddt = current_time - start_time;
			debug("hit limit %d target %ld curpos %ld steps %ld targetv %f curv %f ontrack %d startpos %ld dt %ld", m, F(target), F(motor[m]->current_pos), F(steps), F(motor[m]->target_v), F(motor[m]->v), motor[m]->on_track, F(motor[m]->start_pos), F(ddt));
			motor[m]->switch_pos = motor[m]->current_pos;
			// Stop moving.
			for (uint8_t mm = 0; mm < num_motors; ++mm) {
				debug("stopping %d, sp->%ld", m, F(motor[mm]->current_pos));
				motor[mm]->start_pos = motor[mm]->current_pos;
				motor[mm]->target_v = 0;
				motor[mm]->v = 0;
			}
			stopping = 1;
			try_send_next();
			return;
		}
		// Record move.
		if (abs(motor[m]->current_pos - target) > motor[m]->max_steps)
			debug("woah %d %ld %ld", m, F(motor[m]->current_pos), F(target));
		motor[m]->current_pos = target;
		// Set direction pin.
		if (steps > 0)
			SET(motor[m]->dir_pin);
		else
			RESET(motor[m]->dir_pin);
		microdelay();
		// Move.
		for (int16_t st = 0; st < abs(steps); ++st) {
			SET(motor[m]->step_pin);
			RESET(motor[m]->step_pin);
		}
	}
}

static void handle_adc() {
	if (adc_current == uint8_t(~0))
		return;
	if (adcreply_ready) {
		debug("waiting for previous adcreply to complete?!");
	}
	if (!adc_ready(adc_current)) {
		//debug("adc %d not ready", adc_current);
		return;
	}
	adcreply[0] = 4;
	adcreply[1] = CMD_ADC;
	*reinterpret_cast <int16_t *>(&adcreply[2]) = adc_get(adc_current);
	adcreply_ready = true;
	adc_current = ~0;

	try_send_next();
}

#ifdef HAVE_AUDIO
static void handle_audio(uint32_t current_time, uint32_t longtime) {
	if (audio_head != audio_tail) {
		last_active = longtime;
		int32_t sample = (current_time - audio_start) / audio_us_per_sample;
		int32_t audio_byte = sample >> 3;
		while (audio_byte >= AUDIO_FRAGMENT_SIZE) {
			//debug("next audio fragment");
			if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
			{
				//debug("continue audio");
				continue_cb |= 2;
				try_send_next();
			}
			audio_head = (audio_head + 1) % AUDIO_FRAGMENTS;
			if (audio_tail == audio_head) {
				//debug("audio done");
				next_audio_time = ~0;
				return;
			}
			audio_byte -= AUDIO_FRAGMENT_SIZE;
			// us per fragment = us/sample*sample/fragment
			audio_start += audio_us_per_sample * 8 * AUDIO_FRAGMENT_SIZE;
		}
		uint8_t old_state = audio_state;
		audio_state = (audio_buffer[audio_head][audio_byte] >> (sample & 7)) & 1;
		if (audio_state != old_state) {
			for (uint8_t s = 0; s < num_spaces; ++s) {
				Space &sp = spaces[s];
				for (uint8_t m = 0; m < sp.num_motors; ++m) {
					if (!(sp.motor[m]->audio_flags & Motor::PLAYING))
						continue;
					if (audio_state > old_state)
						SET(sp.motor[m]->dir_pin);
					else
						RESET(sp.motor[m]->dir_pin);
					microdelay();
					SET(sp.motor[m]->step_pin);
					RESET(sp.motor[m]->step_pin);
				}
			}
		}
	}
}
#endif

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
	// Handle any arch-specific periodic things.
	arch_run();
	// Timekeeping.
	uint32_t current_time = utime();
	// Handle all periodic things.

#ifdef TIMING
	uint32_t first_t = utime();
#endif

	// LED
	handle_led(current_time);	// heart beat.
#ifdef TIMING
	uint32_t led_t = utime() - current_time;
#endif

	// Motors
	handle_motors(current_time);
#ifdef TIMING
	uint32_t motor_t = utime() - current_time;
#endif

	// ADC
	handle_adc();
#ifdef TIMING
	uint32_t adc_t = utime() - current_time;
#endif

	// Audio
#ifdef HAVE_AUDIO
	handle_audio(current_time);
#endif
#ifdef TIMING
	uint32_t audio_t = utime() - current_time;
#endif

	// Serial
	serial();
#ifdef TIMING
	uint32_t serial_t = utime() - first_t;
#endif

#ifdef TIMING
	uint32_t end_t = utime() - current_time;
	end_t -= serial_t;
	serial_t -= audio_t;
	audio_t -= adc_t;
	adc_t -= motor_t;
	motor_t -= led_t;
	static int waiter = 0;
	if (waiter > 0)
		waiter -= 1;
	else {
		waiter = 977;
		debug("t: serial %ld motor %ld led %ld audio %ld end %ld", F(serial_t), F(motor_t), F(led_t), F(audio_t), F(end_t));
	}
#endif
}

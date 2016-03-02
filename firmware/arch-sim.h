/* arch-sim.h - simulator specific handling for Franklin
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

// This is not an include quard.  This file is included twice; the first time the first part of the file is used, the second time the second part. {{{
#ifndef _ARCH_BBB_H
#define _ARCH_BBB_H

#include <stdint.h>
#include <stdlib.h>
#include <cstdio>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>

#define NUM_DIGITAL_PINS 15
#define NUM_ANALOG_INPUTS 7

#define cli() do {} while(0)
#define sei() do {} while(0)

#ifdef F
#undef F
#endif
#define F(x) (x)
#define L

#define ARCH_PIN_DATA
#define ARCH_MOTOR

#define TIME_PER_ISR 20
#ifdef FAST_ISR
#undef FAST_ISR
#endif

// Everything before this line is used at the start of firmware.h; everything after it at the end.
#else
// }}}

#ifndef NODEBUG
static inline void debug_add(int i) { (void)&i; }
static inline void debug_dump() { abort(); }
static inline void debug(char const *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	fprintf(stderr, "DBG: ");
	vfprintf(stderr, fmt, ap);
	fprintf(stderr, "\n");
	va_end(ap);
}
#endif

// Serial communication.  {{{
inline static void arch_serial_write(uint8_t c) {
	//debug("$ %02x", c & 0xff);
	::write(1, &c, 1);
}

inline static void arch_serial_flush() {
	std::fflush(stdout);
}
// }}}

// ADC. {{{
// Fake heater on pin 0, adc 0.
EXTERN int sim_temp;
static inline void adc_start(uint8_t adcpin) {
	(void)&adcpin;
}

static inline bool adc_ready(uint8_t pin_) {
	(void)&pin_;
	return true;
}

static inline int16_t adc_get(uint8_t pin_) {
	if (pin_ != 0)
		return 0;
	if (CONTROL_CURRENT(pin[0].state) == CTRL_RESET)
		sim_temp = min(sim_temp + 1, (1 << 10) - 1);
	else
		sim_temp = max(sim_temp - 1, 0);
	return sim_temp;
}
// }}}

// Watchdog and reset. {{{
static inline void arch_watchdog_enable() {
}

static inline void arch_watchdog_disable() {
}

static inline void arch_watchdog_reset() {
}

static inline void arch_reset() {
	abort();
}
// }}}

// Setup. {{{
static inline void arch_setup_start() {
	fcntl(0, F_SETFL, O_NONBLOCK);
}

EXTERN struct pollfd sim_pollfd;
static inline void arch_setup_end() {
	sim_pollfd.fd = 0;
	sim_pollfd.events = POLLIN | POLLPRI;
}

static inline void arch_msetup(uint8_t m) {
	(void)&m;
}

static inline void arch_set_speed(uint16_t count) {
	if (count == 0)
		step_state = STEP_STATE_STOP;
}

static inline void arch_tick() {
	while (true) {
		int c = fgetc(stdin);
		if (c == EOF) {
			if (errno == EWOULDBLOCK || errno == EAGAIN)
				break;
			debug("EOF on input; exiting.");
			abort();
		}
		if (serial_overflow)
			break;
		//debug("%%  %02x", c & 0xff);
		int n = (serial_buffer_head + 1) & SERIAL_MASK;
		if (n == serial_buffer_tail) {
			serial_overflow = true;
			break;
		}
		BUFFER_CHECK(serial_buffer, serial_buffer_head);
		serial_buffer[serial_buffer_head] = c;
		serial_buffer_head = n;
	}
	// Do moves.
	SLOW_ISR();
}
// }}}

// Timekeeping. {{{
EXTERN long long sim_t;
static inline uint16_t millis() {
	sim_t += 1;
	return sim_t;
}

static inline uint16_t seconds() {
	sim_t += 1;
	return sim_t / 1000;
}
// }}}

// Pin control. {{{
inline void SET_OUTPUT(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	if ((pin[pin_no].state & 0x3) == CTRL_SET || (pin[pin_no].state & 0x3) == CTRL_RESET)
		return;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_RESET);
}

inline void SET_INPUT(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_INPUT);
}

inline void UNSET(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_UNSET);
}

inline void SET(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	if ((pin[pin_no].state & 0x3) == CTRL_SET)
		return;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_SET);
}

inline void RESET(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	if ((pin[pin_no].state & 0x3) != CTRL_SET)
		return;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_RESET);
}

// Limit switches on 1, 2, 3.
inline bool GET(uint8_t pin_no) {
	if (pin_no < 1 || pin_no > 3)
		return false;
	return motor[pin_no - 1].current_pos > 0;
}
// }}}

#endif

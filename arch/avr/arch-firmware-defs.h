/* arch/avr/arch-firmware-defs.h - avr specific parts for Franklin; defines.
 * vim: set foldmethod=marker :
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

#ifndef _ARCH_AVR_DEFS_H
#define _ARCH_AVR_DEFS_H

#define INFO_ENABLE
#define NO_main

#include <avr-ll.hh>

// Defines and includes.
// Note: When changing this, also change max in cdriver/space.cpp
#ifdef FAST_ISR
// 100 is for a 16MHz crystal.
#define TIME_PER_ISR int(100 * 16e6 / F_CPU)
#ifndef STEPS_DELAY // {{{
#define STEPS_DELAY 5	// Extra delay for steps, in units of 6 clock pulses.
#endif // }}}
#ifndef ADD_DIR_DELAY // {{{
#if 1
#define ADD_DIR_DELAY
#else
#define ADD_DIR_DELAY \
		"\t"	"ldi 18, 40"	"\n" \
	"1:\t"		"dec 18"	"\n" \
		"\t"	"brne 1b"	"\n"
#endif
#endif // }}}
#else
#define TIME_PER_ISR 500
#endif
#define BAUD 115200

#define ARCH_PIN_DATA \
	bool avr_on; \
	int32_t avr_target;

#define ARCH_MOTOR \
	volatile uint8_t *step_port; \
	volatile uint8_t *dir_port; \
	volatile uint8_t step_bitmask, dir_bitmask;

static inline void arch_watchdog_reset() { // {{{
#ifdef WATCHDOG
	Wdt::reset();
#endif
} // }}}
static inline void arch_setup_end() { // {{{
} // }}}
static inline void arch_tick() { // {{{
} // }}}
void update_timer1_pwm();
void arch_claim_serial();
void arch_setup_start();
bool adc_ready(uint8_t pin_);
int16_t adc_get(uint8_t pin_);
void arch_outputs();

#endif

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
	volatile uint8_t *avr_mode; \
	volatile uint8_t *avr_output; \
	volatile uint8_t *avr_input; \
	uint8_t avr_bitmask; \
	bool avr_on; \
	int32_t avr_target;

#define ARCH_MOTOR \
	volatile uint16_t step_port, dir_port; \
	volatile uint8_t step_bitmask, dir_bitmask;

// Define things that pins_arduino.h needs from Arduino.h (which shouldn't be included). {{{
#define ARDUINO_MAIN
#define NOT_A_PIN 0
#define NOT_A_PORT 0
#define NOT_ON_TIMER 0xff
#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PK 11
#define PL 12
#define TIMER0 0
#define TIMER0A 0
#define TIMER0B 1
#define TIMER1A 2
#define TIMER1B 3
#define TIMER1C 4
#define TIMER2  5
#define TIMER2A 5
#define TIMER2B 6
#define TIMER3A 7
#define TIMER3B 8
#define TIMER3C 9
#define TIMER4A 10
#define TIMER4B 11
#define TIMER4C 12
#define TIMER4D NOT_ON_TIMER   // Not supported.
#define TIMER5A 13
#define TIMER5B 14
#define TIMER5C 15

#include <stdio.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

struct Timer_data {
	volatile uint8_t *mode, *oc;
	uint8_t mode_mask;
	uint8_t num;
	char part;
	Timer_data(volatile uint8_t *mode_, volatile uint8_t *oc_, uint8_t mode_mask_, uint8_t num_, char part_) : mode(mode_), oc(oc_), mode_mask(mode_mask_), num(num_), part(part_) {}
};
inline bool timer_is_16bit(int t) {
	return t != TIMER0 && t != TIMER2;
}
static const Timer_data timer_data[] = {
	Timer_data(&TCCR0A, &OCR0A, 2 << 6, 0, 'A'),
	Timer_data(&TCCR0A, &OCR0B, 2 << 4, 0, 'B'),

	Timer_data(&TCCR1A, &OCR1AL, 2 << 6, 1, 'A'),
	Timer_data(&TCCR1A, &OCR1BL, 2 << 4, 1, 'B'),
#ifdef OCR1C
	Timer_data(&TCCR1A, &OCR1CL, 2 << 2, 1, 'C'),
#else
	Timer_data(NULL, NULL, 0, 0, 'x'),
#endif

	Timer_data(&TCCR2A, &OCR2A, 2 << 6, 2, 'A'),
	Timer_data(&TCCR2A, &OCR2B, 2 << 4, 2, 'B'),

#ifdef TCCR3A
	Timer_data(&TCCR3A, &OCR3AL, 2 << 6, 3, 'A'),
	Timer_data(&TCCR3A, &OCR3BL, 2 << 4, 3, 'B'),
#ifdef OCR3C
	Timer_data(&TCCR3A, &OCR3CL, 2 << 2, 3, 'C'),
#else
	Timer_data(NULL, NULL, 0, 0, 'x'),
#endif
#endif
#ifdef TCCR4A
	Timer_data(&TCCR4A, &OCR4AL, 2 << 6, 4, 'A'),
	Timer_data(&TCCR4A, &OCR4BL, 2 << 4, 4, 'B'),
	Timer_data(&TCCR4A, &OCR4CL, 2 << 2, 4, 'C'),
#endif
#ifdef TCCR5A
	Timer_data(&TCCR5A, &OCR5AL, 2 << 6, 5, 'A'),
	Timer_data(&TCCR5A, &OCR5BL, 2 << 4, 5, 'B'),
	Timer_data(&TCCR5A, &OCR5CL, 2 << 2, 5, 'C'),
#endif
};

#ifdef F
#undef F
#endif
#define F(x) &(x)
#define L "l"

#include <pins_arduino.h>

static inline void arch_watchdog_reset() { // {{{
#ifdef WATCHDOG
	wdt_reset();
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

/* arch-avr.h - avr specific parts for Franklin
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

// This is not an include quard.  This file is included twice; the first time the first part of the file is used, the second time the second part.
#ifndef _ARCH_AVR_H
#define _ARCH_AVR_H

// Defines and includes.  {{{
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

//#define pindebug debug
#define pindebug(...) do {} while (0)

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
#define TIMER4D NOT_ON_TIMER	// Not supported.
#define TIMER5A 13
#define TIMER5B 14
#define TIMER5C 15
// }}}

#include <stdio.h>
#include <pins_arduino.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

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
// }}}

static inline void arch_disable_isr() { // {{{
	TIMSK1 = 0;
} // }}}

static inline void arch_enable_isr() { // {{{
	TIMSK1 = 1 << OCIE1A;
} // }}}

static inline void arch_set_speed(uint16_t us_per_sample);
static inline void arch_serial_write(uint8_t data);

// Everything before this line is used at the start of firmware.h; everything after it at the end.
#else

#ifdef __cplusplus
// Defined by arduino: NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS

// Serial communication. {{{
// Variables. {{{
#ifdef UDR3
#define NUM_SERIAL_PORTS 4
#else
#ifdef UDR2
#define NUM_SERIAL_PORTS 3
#else
#ifdef UDR1
#define NUM_SERIAL_PORTS 2
#else
#define NUM_SERIAL_PORTS 1
#endif
#endif
#endif
EXTERN volatile int8_t avr_last_serial;
EXTERN int8_t avr_which_serial;
extern volatile uint8_t *const avr_serial_ports[NUM_SERIAL_PORTS][6];
#ifdef DEFINE_VARIABLES
#define ONEPORT(p) {&UDR ## p, &UCSR ## p ## A, &UCSR ## p ## B, &UCSR ## p ## C, &UBRR ## p ## H, &UBRR ## p ## L}
volatile uint8_t *const avr_serial_ports[NUM_SERIAL_PORTS][6] = {
#ifdef UDR3
	ONEPORT(0), ONEPORT(1), ONEPORT(2), ONEPORT(3)
#else
#ifdef UDR2
	ONEPORT(0), ONEPORT(1), ONEPORT(2)
#else
#ifdef UDR1
	ONEPORT(0), ONEPORT(1)
#else
	ONEPORT(0)
#endif
#endif
#endif
};
#endif
// }}}

static inline void arch_serial_write(uint8_t data) { // {{{
	if (avr_which_serial < 0)
		return;
	while (~*avr_serial_ports[avr_which_serial][1] & (1 << UDRE0)) {}
	*avr_serial_ports[avr_which_serial][0] = data;
} // }}}

static inline void arch_serial_flush() { // {{{
	if (avr_which_serial < 0)
		return;
	while (~*avr_serial_ports[avr_which_serial][1] & (1 << TXC0)) {}
} // }}}

static inline void arch_claim_serial() { // {{{
	if (avr_which_serial == avr_last_serial)
		return;
	avr_which_serial = avr_last_serial;
	for (uint8_t i = 0; i < NUM_SERIAL_PORTS; ++i) {
		if (i == avr_which_serial)
			*avr_serial_ports[i][2] = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
		else
			*avr_serial_ports[i][2] = 0;
	}
} // }}}
#ifdef DEFINE_VARIABLES
// Serial input ISR. {{{
/* 16 MHz, 1Mbit: 16 cycles/bit; 160 cycles/byte. */
#define avr_serial_input(which, status, data) \
	asm( \
									/* 7	 7 (for vectoring the interrupt). */ \
		"\t"	"push 29"				"\n"	/* 2	 9 */ \
		"\t"	"in 29, __SREG__"			"\n"	/* 1	10 */ \
		"\t"	"push 29"				"\n"	/* 2	12 */ \
		"\t"	"lds 29, %[udr]"			"\n"	/* 2	14 */ \
		"\t"	"push 30"				"\n"	/* 2	16 */ \
		"\t"	"push 31"				"\n"	/* 2	18 */ \
		"\t"	"lds 30, %[ucsra]"			"\n"	/* 2	20 */ \
		"\t"	"andi 30, %[statusmask]"		"\n"	/* 1	21 */ \
		"\t"	"brne 5f"				"\n"	/* 1	22 */ \
		"\t"	"ldi 30, " #which			"\n"	/* 1	23 */ \
		"\t"	"sts %[avr_last_serial], 30"		"\n"	/* 2	25 */ \
		"\t"	"lds 30, %[serial_buffer_head]"		"\n"	/* 2	27 */ \
		"\t"	"lds 31, %[serial_buffer_head] + 1"	"\n"	/* 2	29 */ \
		"\t"	"st z, 29"				"\n"	/* 2	31 */ \
		"\t"	"inc 30"				"\n"	/* 1	32 */ \
		"\t"	"breq 1f"				"\n"	/* 1	33 */ \
		"\t"	"lds 29, %[serial_buffer_tail]"		"\n"	/* 2	35 */ \
		"\t"	"cp 30, 29"				"\n"	/* 1	36 */ \
		"\t"	"breq 2f"				"\n"	/* 1	37 */ \
	"6:\t"		/* No overflow. */			"\n"	/*   */ \
		"\t"	"sts %[serial_buffer_head], 30"		"\n"	/* 2	39 */ \
	"7:\t"		/* Finish. */				"\n"	/*   */ \
		"\t"	"pop 31"				"\n"	/* 2	41 */ \
		"\t"	"pop 30"				"\n"	/* 2	43 */ \
		"\t"	"pop 29"				"\n"	/* 2	45 */ \
		"\t"	"out __SREG__, 29"			"\n"	/* 1	46 */ \
		"\t"	"pop 29"				"\n"	/* 2	48 */ \
		"\t"	"reti"					"\n"	/* 4	52 */ \
	"1:\t"		/* Carry in in lo8(head). */		"\n" \
		"\t"	"inc 31"				"\n" \
		"\t"	"andi 31, %[serialmask]"		"\n" \
		"\t"	"ori 31, hi8(%[serial_buffer])"		"\n" \
		"\t"	"lds 29, %[serial_buffer_tail]"		"\n" \
		"\t"	"cp 30, 29"				"\n" \
		"\t"	"breq 3f"				"\n" \
	"4:\t"		/* No overflow. */			"\n" \
		"\t"	"sts %[serial_buffer_head] + 1, 31"	"\n" \
		"\t"	"rjmp 6b"				"\n" \
	"3:\t"		/* lo8(head) == lo8(tail). */		"\n" \
		"\t"	"lds 29, %[serial_buffer_tail] + 1"	"\n" \
		"\t"	"cp 31, 29"				"\n" \
		"\t"	"brne 4b"				"\n" \
	"5:\t"		/* Overflow. */				"\n" \
		"\t"	"ldi 29, 1"				"\n" \
		"\t"	"sts %[serial_overflow], 29"		"\n" \
		"\t"	"rjmp 7b"				"\n" \
	"2:\t"		/* lo8(head) == lo8(tail). */		"\n" \
		"\t"	"lds 29, %[serial_buffer_tail] + 1"	"\n" \
		"\t"	"cp 31, 29"				"\n" \
		"\t"	"brne 6b"				"\n" \
		"\t"	"rjmp 5b"				"\n" \
		:: \
			[serial_buffer] "" (&serial_buffer), \
			[serial_buffer_head] "" (&serial_buffer_head), \
			[serial_buffer_tail] "" (&serial_buffer_tail), \
			[serial_overflow] "" (&serial_overflow), \
			[avr_last_serial] "" (&avr_last_serial), \
			[ucsra] "" (_SFR_MEM_ADDR(status)), \
			[udr] "" (_SFR_MEM_ADDR(data)), \
			[statusmask] "M" ((1 << FE0) | (1 << DOR0)), \
			[serialmask] "M" (SERIAL_MASK >> 8), \
			[port] "M" (which) \
	)
// }}}

ISR(USART0_RX_vect, ISR_NAKED) { // {{{
	avr_serial_input(0, UCSR0A, UDR0);
} // }}}

#ifdef UDR1
ISR(USART1_RX_vect, ISR_NAKED) { // {{{
	avr_serial_input(1, UCSR1A, UDR1);
} // }}}
#endif

#ifdef UDR2
ISR(USART2_RX_vect, ISR_NAKED) { // {{{
	avr_serial_input(2, UCSR2A, UDR2);
} // }}}
#endif

#ifdef UDR3
ISR(USART3_RX_vect, ISR_NAKED) { // {{{
	avr_serial_input(3, UCSR3A, UDR3);
} // }}}
#endif
#endif
// }}}

// Debugging. {{{
#ifndef NO_DEBUG
#define AVR_DEBUG_BITS 5
EXTERN volatile int avr_debug[1 << AVR_DEBUG_BITS];
EXTERN volatile int avr_debug_ptr;
static inline void debug_add(int i) { // {{{
	avr_debug[avr_debug_ptr] = i;
	avr_debug_ptr = (avr_debug_ptr + 1) & ((1 << AVR_DEBUG_BITS) - 1);
} // }}}

static inline void debug_dump() { // {{{
	debug("Debug dump (most recent last):");
	for (int i = 0; i < 1 << AVR_DEBUG_BITS; ++i)
		debug("%x", avr_debug[(avr_debug_ptr + i) & ((1 << AVR_DEBUG_BITS) - 1)]);
	debug("dump done");
} // }}}

static inline void avr_print_num(int32_t num, int base) { // {{{
	uint32_t anum;
	if (num < 0) {
		arch_serial_write('-');
		anum = -num;
	}
	else
		anum = num;
	int digits = 1;
	uint32_t power = base;
	while (anum / power > 0) {
		digits += 1;
		power *= base;
	}
	power /= base;
	for (int d = 0; d < digits; ++d) {
		uint8_t c = anum / power;
		anum -= c * power;
		power /= base;
		arch_serial_write(c < 10 ? '0' + c : 'a' + c - 10);
	}
} // }}}

static inline void debug(char const *fmt, ...) { // {{{
#if DEBUG_BUFFER_LENGTH > 0
	buffered_debug_flush();
#endif
	va_list ap;
	va_start(ap, fmt);
	arch_serial_write(CMD_DEBUG);
	for (char const *p = fmt; *p; ++p) {
		if (*p == '%') {
			bool longvalue = false;
			while (true) {
				++p;
				switch (*p) {
				case 0: {
					arch_serial_write('%');
					--p;
					break;
				}
				case 'l': {
					longvalue = true;
					continue;
				}
				case '%': {
					arch_serial_write('%');
					break;
				}
				case 'd': {
					if (longvalue) {
						int32_t *arg = va_arg(ap, int32_t *);
						avr_print_num(*arg, 10);
					}
					else {
						int16_t arg = va_arg(ap, int16_t);
						avr_print_num(arg, 10);
					}
					break;
				}
				case 'x': {
					if (longvalue) {
						int32_t *arg = va_arg(ap, int32_t *);
						avr_print_num(*arg, 16);
					}
					else {
						int arg = va_arg(ap, int16_t);
						avr_print_num(arg, 16);
					}
					break;
				}
				case 's': {
					char const *arg = va_arg(ap, char const *);
					while (*arg)
						arch_serial_write(*arg++);
					break;
				}
				case 'c': {
					char arg = va_arg(ap, int);
					arch_serial_write(arg);
					break;
				}
				default: {
					arch_serial_write('%');
					arch_serial_write(*p);
					break;
				}
				}
				break;
			}
		}
		else {
			arch_serial_write(*p);
		}
	}
	va_end(ap);
	arch_serial_write(0);
} // }}}
#endif
// }}}

// ADC. {{{
EXTERN uint8_t avr_adc_last_pin;
#define AVR_ADCSRA_BASE	((1 << ADEN) | 7)	// Prescaler set to 128.

static inline void adc_start(uint8_t adcpin) { // {{{
	// Mostly copied from /usr/share/arduino/hardware/arduino/cores/arduino/wiring_analog.c.
#if defined(ADCSRB) && defined(MUX5)
	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
	uint8_t pin_ = adcpin;
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin_ >> 3) & 0x01) << MUX5);
#else
	uint8_t pin_ = adcpin;
#endif

	ADMUX = (pin_ & 0x7) | 0x40;
	// Start the conversion.
	ADCSRA = AVR_ADCSRA_BASE | (1 << ADSC);
	avr_adc_last_pin = adcpin;
} // }}}

static inline bool adc_ready(uint8_t pin_) { // {{{
	if (pin_ != avr_adc_last_pin) {
		adc_phase = PREPARING;
		adc_start(pin_);
		return false;
	}
	if (bit_is_set(ADCSRA, ADSC))
		return false;
	if (adc_phase != MEASURING) {
		adc_phase = MEASURING;
		ADCSRA = AVR_ADCSRA_BASE | (1 << ADSC);
		return false;
	}
	return true;
} // }}}

static inline int16_t adc_get(uint8_t pin_) { // {{{
	(void)&pin_;
	int16_t low = uint8_t(ADCL);
	int16_t high = uint8_t(ADCH);
	int16_t ret = (high << 8) | low;
	adc_phase = INACTIVE;
	return ret;
} // }}}
// }}}

// Watchdog and reset. {{{
static inline void arch_watchdog_enable() { // {{{
#ifdef WATCHDOG
	wdt_reset();
	wdt_enable(WDTO_4S);
#endif
} // }}}

static inline void arch_watchdog_disable() { // {{{
#ifdef WATCHDOG
	wdt_disable();
#endif
} // }}}

static inline void arch_watchdog_reset() { // {{{
#ifdef WATCHDOG
	wdt_reset();
#endif
} // }}}

static inline void arch_reset() { // {{{
	// Warning: this may not work with all bootloaders.
	// The problem is that the bootloader must disable or reset the watchdog timer.
	// If it doesn't, it will continue resetting the device.
	wdt_enable(WDTO_15MS);	// As short as possible.
	while(1) {}
} // }}}
// }}}

// Setup. {{{
EXTERN volatile uint32_t avr_time_h, avr_seconds_h, avr_seconds;
EXTERN int timer1_pins[3];
EXTERN int num_timer1_pins;
EXTERN uint16_t timer1_top;

static inline void arch_setup_start() { // {{{
	cli();
	for (uint8_t pin_no = 0; pin_no < NUM_DIGITAL_PINS; ++pin_no) {
		uint8_t port = pgm_read_word(digital_pin_to_port_PGM + pin_no);
		pin[pin_no].avr_mode = reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_mode_PGM + port));
		pin[pin_no].avr_output = reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_output_PGM + port));
		pin[pin_no].avr_input = reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_input_PGM + port));
		pin[pin_no].avr_bitmask = pgm_read_word(digital_pin_to_bit_mask_PGM + pin_no);
		pin[pin_no].avr_on = false;
		pin[pin_no].avr_target = 0;
		int timer = pgm_read_byte(digital_pin_to_timer_PGM + pin_no);
		if (timer != NOT_ON_TIMER && timer_data[timer].num == 1)
			timer1_pins[num_timer1_pins++] = pin_no;
	}
	avr_time_h = 0;
	avr_seconds_h = 0;
	avr_seconds = 0;
	arch_watchdog_disable();
	// Serial ports.
	avr_last_serial = -1;
	avr_which_serial = -1;
	int const ubrr = F_CPU / (8. * BAUD) - .5;
	for (uint8_t i = 0; i < NUM_SERIAL_PORTS; ++i) {
		*avr_serial_ports[i][1] = 1 << U2X0;	// ucsra
		*avr_serial_ports[i][2] = (1 << RXCIE0) | (1 << RXEN0);	// ucsrb
		*avr_serial_ports[i][3] = 6;	// ucsrc
		*avr_serial_ports[i][4] = ubrr >> 8;	// ubrrh
		*avr_serial_ports[i][5] = ubrr & 0xff;	// ubrrl 1:1M; 16:115k2
	}
	// Setup timer1 for microsecond counting.
	TCCR1A = 0x02;
	TCCR1B = 0x19;	// clock at F_CPU.
	ICR1H = 0;
	ICR1L = 0xff;
	timer1_top = 0xff;
	TIMSK1 = 0;	// Disable the interrupt while the motors are disabled.
	TIFR1 = 0xff;
	// Setup timer0 for millis().
	TCCR0A = 3;
	TCCR0B = 5;	// Clock/1024: 15.625 ticks/millisecond.
	TIFR0 = 1 << TOV0;
	TIMSK0 |= 1 << TOIE0;
	// Enable timer2 for pwm.
	TCCR2A = 3;
	TCCR2B = 1;
	// And the other timers that exist; use same period as timer0 for all of them, but use 15 bits.
#ifdef TCCR3A
	TCCR3A = 0x02;
	TCCR3B = 0x19;
	ICR3H = 0x7f;
	ICR3L = 0xff;
	TCNT3H = 0;
	TCNT3L = 0;
#endif
#ifdef TCCR4A
	TCCR4A = 0x02;
	TCCR4B = 0x19;
	ICR4H = 0x7f;
	ICR4L = 0xff;
	TCNT4H = 0;
	TCNT4L = 0;
#endif
#ifdef TCCR5A
	TCCR5A = 0x02;
	TCCR5B = 0x19;
	ICR5H = 0x7f;
	ICR5L = 0xff;
	TCNT5H = 0;
	TCNT5L = 0;
#endif
	// Setup ADC.
	ADCSRA = AVR_ADCSRA_BASE;
	// Enable interrupts.
	sei();
	// machineid will be filled by CMD_BEGIN.  Initialize it to 0.
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		machineid[1 + i] = 0;
	// Initialize uuid from EEPROM.
	for (uint8_t i = 0; i < UUID_SIZE; ++i)
		machineid[1 + ID_SIZE + i] = EEPROM.read(i);
} // }}}

static inline void arch_setup_end() { // {{{
} // }}}

static inline void arch_tick() { // {{{
} // }}}

static inline void arch_msetup(uint8_t m) { // {{{
	if (motor[m].step_pin < NUM_DIGITAL_PINS) {
		motor[m].step_port = int(pin[motor[m].step_pin].avr_output);
		motor[m].step_bitmask = pin[motor[m].step_pin].avr_bitmask;
	}
	else {
		motor[m].step_port = 0;	// This will access r0, which is harmless because bitmask is 0.
		motor[m].step_bitmask = 0;
	}
	if (motor[m].dir_pin < NUM_DIGITAL_PINS) {
		motor[m].dir_port = int(pin[motor[m].dir_pin].avr_output);
		motor[m].dir_bitmask = pin[motor[m].dir_pin].avr_bitmask;
	}
	else {
		motor[m].dir_port = 0;	// This will access r0, which is harmless because bitmask is 0.
		motor[m].dir_bitmask = 0;
	}
} // }}}

static inline void update_timer1_pwm() { // {{{
	for (int i = 0; i < num_timer1_pins; ++i) {
		// OC / top = duty / 0x7fff
		int tp = timer1_pins[i];
		uint32_t value = pin[tp].duty;
		if (value < 0x7fff) {
			int timer = pgm_read_byte(digital_pin_to_timer_PGM + tp);
			Timer_data const &p = timer_data[timer];
			value *= timer1_top;
			value >>= 15;
			p.oc[1] = (value >> 8) & 0xff;
			p.oc[0] = value & 0xff;
			//debug("set timer1 pin %d:%d to %d for %d with top %d (%x %x) ocr1b %x %x", i, tp, (int)value, pin[tp].duty, timer1_top, ICR1H, ICR1L, OCR1BH, OCR1BL);
		}
		//else
			//debug("full power for timer1 pin %d:%d", i, tp);
	}
} // }}}

static inline void arch_set_speed(uint16_t us_per_sample) { // {{{
	if (us_per_sample == 0) {
		TIMSK1 = 0;
		step_state = STEP_STATE_STOP;
	}
	else {
		uint32_t c = us_per_sample;
		c *= F_CPU / 1000000;
		timer1_top = c >> full_phase_bits;
		//debug("%x us, top=%x:%x, fpb=%d", us_per_sample, int((c >> 16) & 0xffff), int(c & 0xffff), full_phase_bits);
		// Set TOP.
		ICR1H = (timer1_top >> 8) & 0xff;
		ICR1L = timer1_top & 0xff;
		// Clear counter.
		TCNT1H = 0;
		TCNT1L = 0;
		update_timer1_pwm();
		// Clear and enable interrupt.
		TIFR1 = 1 << OCF1A;
		TIMSK1 = 1 << OCIE1A;
	}
} // }}}

static inline void arch_spi_start() { // {{{
	SET_OUTPUT(MOSI);
	RESET(SCK);
} // }}}

static inline void arch_spi_send(uint8_t data, uint8_t bits) { // {{{
	arch_spi_start();
	while (bits > 0) {
		if (data & 0x80)
			SET(MOSI);
		else
			RESET(MOSI);
		SET(SCK);
		RESET(SCK);
		data <<= 1;
		bits -= 1;
	}
} // }}}

static inline void arch_spi_stop() { // {{{
	UNSET(MOSI);
	UNSET(SCK);
} // }}}

static inline int8_t arch_pin_name(char *buffer_, bool digital, uint8_t pin_) { // {{{
	char const *extra;
	switch (pin_) {
		case MOSI:
			extra = "; MOSI";
			break;
		case MISO:
			extra = "; MISO";
			break;
		case SCK:
			extra = "; SCK";
			break;
		case SDA:
			extra = "; SDA";
			break;
		case SCL:
			extra = "; SCL";
			break;
		default:
			extra = "";
			break;
	}
	if (digital) {
		int port = 'A' - 1 + pgm_read_byte(digital_pin_to_port_PGM + pin_);
		int b = pgm_read_byte(digital_pin_to_bit_mask_PGM + pin_);
		int bit = b & 0xf0
			? b & 0xc0
				? b & 0x80 ? 7 : 6
				: b & 0x20 ? 5 : 4
			: b & 0x0c
				? b & 0x08 ? 3 : 2
				: b & 0x02 ? 1 : 0;
		if (pin_ >= A0)
			return sprintf(buffer_, "D%d/P%c%d (A%d%s)", pin_, port, bit, pin_ - A0, extra);
		else {
			int timer = pgm_read_byte(digital_pin_to_timer_PGM + pin_);
			if (timer == NOT_ON_TIMER) {
				if (extra[0] != '\0')
					return sprintf(buffer_, "D%d/P%c%d (%s)", pin_, port, bit, &extra[2]);
				else
					return sprintf(buffer_, "D%d/P%c%d", pin_, port, bit);
			}
			else
				return sprintf(buffer_, "D%d/P%c%d (PWM%d%c%s)", pin_, port, bit, timer_data[timer].num, timer_data[timer].part, extra);
		}
	}
	else {
		if (pin_ + A0 < NUM_DIGITAL_PINS)
			return sprintf(buffer_, "A%d (D%d)", pin_, pin_ + A0);
		else
			return sprintf(buffer_, "A%d", pin_);
	}
} // }}}

// }}}

#ifdef DEFINE_VARIABLES
#define offsetof(type, field) __builtin_offsetof(type, field)
#ifdef FAST_ISR
ISR(TIMER1_COMPA_vect, ISR_NAKED) { // {{{
	asm volatile (
		// Naked ISR, so all registers must be saved.
		"\t"	"push 16"			"\n"	// step_state
		"\t"	"in 16, __SREG__"		"\n"
		"\t"	"push 16"			"\n"
		// If step_state < 2: return.
		"\t"	"lds 16, %[step_state]"		"\n"
		"\t"	"cpi 16, %[state_non_move]"	"\n"
		"\t"	"brcc 1f"			"\n"
		"\t"	"rjmp isr_end16"		"\n"
	"2:\t"						"\n"
		"\t"	"rjmp isr_underrun"		"\n"
	"1:\t"						"\n"
		// Enable interrupts, so the uart doesn't overrun.
		"\t"	"push 27"			"\n"	// 0 or temporary value
		"\t"	"clr 27"			"\n"
		"\t"	"sts %[timsk], 27"		"\n"
		"\t"	"sei"				"\n"

		// Save all registers that are used.
		"\t"	"push 0"			"\n"	// temporary value
		"\t"	"push 1"			"\n"	// temporary value
		"\t"	"push 17"			"\n"	// move phase
		"\t"	"push 20"			"\n"	// motor flags
		"\t"	"push 21"			"\n"	// temporary value
		"\t"	"push 24"			"\n"	// temporary value
		"\t"	"push 25"			"\n"	// temporary value
		"\t"	"push 26"			"\n"	// temporary value
		"\t"	"push 28"			"\n"	// motor pointer L
		"\t"	"push 29"			"\n"	// motor pointer H
		"\t"	"push 30"			"\n"	// buffer pointer L
		"\t"	"push 31"			"\n"	// buffer pointer H
		// move_phase += 1;
		"\t"	"lds 17, %[move_phase]"		"\n"
		"\t"	"inc 17"			"\n"
		"\t"	"sts %[move_phase], 17"		"\n"
		// 16 is motor countdown; y(28,29) is motor pointer.
		"\t"	"lds 16, %[active_motors]"		"\n"
		"\t"	"ldi 28, lo8(%[motor])"		"\n"
		"\t"	"ldi 29, hi8(%[motor])"		"\n"
		// If active_motors is 0 and move is requested; trigger underrun immediately.
		"\t"	"tst 16"			"\n"
		"\t"	"breq 2b"			"\n"

	// Register usage: (outdated!)
	// 16: motor count down; note: current motor counts up, so this is not the current motor.
	// 17: move_phase + 1.
	// 20: flags.
	// 0: steps target.
	// 1,21 general purpose.
	// 24: sample value
	// x(26,27): pin pointer for varying pins.	x.h is 0 a lot (but not always).
	// y(28,29): current motor pointer.
	// z(30,31): buffer pointer (pointing at current_sample for current motor).
		// Set z to current buffer.
		"\t"	"lds 30, %[current_buffer]"	"\n"
		"\t"	"lds 31, %[current_buffer] + 1"	"\n"
		"\t"	"lds 1, %[current_sample]"	"\n"
		"\t"	"add 30, 1"			"\n"
		"\t"	"adc 31, 27"			"\n"
	"isr_action_loop:"				"\n"
		// If not active: continue	>20(flags) {{{
		"\t"	"ldd 20, y + %[flags]"		"\n"
		"\t"	"sbrs 20, %[activebit]"		"\n"
		"\t"	"rjmp isr_action_continue"	"\n"
		// }}}
		// If pattern: skip normal movement.
		"\t"	"sbrc 20, %[patternbit]"	"\n"
		"\t"	"rjmp isr_action_pattern"	"\n"

		// Load value in 24.
		"\t"	"ld 24, z"			"\n"
		// Load dir data. >x(26,27)(dir port) >0(bitmask) >21(current value) <20(flags) {{{
		"\t"	"ldd 26, y + %[dir_port]"	"\n"
		"\t"	"ldd 27, y + %[dir_port] + 1"	"\n"	// r27 can be non-zero from here.
		"\t"	"ldd 0, y + %[dir_bitmask]"	"\n"
		"\t"	"ld 21, x"			"\n"	// r21 is current port value.
		// }}}
		// positive ? set dir : reset dir (don't send yet) <0(dir bitmask) <21(current value) >21(new value) >24(abs numsteps) >t(sample sign) {{{
		"\t"	"tst 24"			"\n"	// Test sample sign.
		"\t"	"brmi 1f"			"\n"
		"\t"	"or 21, 0"			"\n"
		"\t"	"clt"				"\n"
		"\t"	"rjmp 2f"			"\n"
	"1:\t"		"com 0"				"\n"
		"\t"	"and 21, 0"			"\n"
		"\t"	"set"				"\n"
	"2:\t"						"\n"
		// Set direction (x is port register).
		"\t"	"st x, 21"			"\n"
		// }}}

		// Convert encoded sample into value r24 -> r24,25
		"\t"	"clr 25"			"\n"
		"\t"	"lsl 24"			"\n"
	"1:\t"						"\n"
		"\t"	"lsl 24"			"\n"
		"\t"	"brcc 1f"			"\n"
		"\t"	"inc 25"			"\n"
		"\t"	"rjmp 1b"			"\n"
	"1:\t"						"\n"
		"\t"	"lsr 25"			"\n"
		"\t"	"ror 24"			"\n"
		"\t"	"lsr 25"			"\n"
		"\t"	"ror 24"			"\n"

		/* Compute steps.  <24,25(abs numsteps) <17(move_phase) <y(28,29)(motor) >0,1(steps) {{{ */
		/* steps target = b.sample * move_phase / full_phase - m.steps_current; */
		"\t"	"clr 21"			"\n"
		"\t"	"mul 25, 17"			"\n"	// r0:r1 = r24*r17
		"\t"	"movw 26, 0"			"\n"
		"\t"	"mul 24, 17"			"\n"
		"\t"	"add 1, 26"			"\n"
		"\t"	"adc 27, 21"			"\n"	// r0,1,27 is the decoded number of steps to take at full phase.

		"\t"	"lds 25, %[full_phase_bits]"	"\n"
		"\t"	"tst 25"			"\n"
		"\t"	"rjmp 2f"			"\n"
	"1:\t"		"lsr 27"			"\n"	// r0,1,27 >>= full_phase_bits
		"\t"	"ror 1"				"\n"
		"\t"	"ror 0"				"\n"
		"\t"	"dec 25"			"\n"
	"2:\t"		"brne 1b"			"\n"
		// r0,1 = (r24 * r17) >> full_phase_bits	(r27 is always zero after the shifts)

		"\t"	"ldd 24, y + %[steps_current]"	"\n"
		"\t"	"ldd 25, y + %[steps_current] + 1"	"\n"
		"\t"	"std y + %[steps_current], 0"	"\n"	// motor[m].steps_current += steps_target;
		"\t"	"std y + %[steps_current] + 1, 1"	"\n"
		"\t"	"sub 0, 24"			"\n"
		"\t"	"sbc 1, 25"			"\n"

		/* If no steps: continue. */
		"\t"	"brne 1f"			"\n"
		"\t"	"clr 27"			"\n"
		"\t"	"rjmp isr_action_continue"	"\n"
	"1:\t"						"\n"
		// }}}

		//motor[m].current_pos += (motor[m].dir == DIR_POSITIVE ? steps_target : -steps_target);
		"\t"	"ldd 25, y + %[current_pos]"		"\n"
		"\t"	"brts 1f"				"\n"
		"\t"	"add 25, 0"				"\n"
		"\t"	"std y + %[current_pos], 25"		"\n"
		"\t"	"ldd 25, y + %[current_pos] + 1"	"\n"
		"\t"	"adc 25, 1"				"\n"
		"\t"	"std y + %[current_pos] + 1, 25"	"\n"
		"\t"	"brcc 2f"				"\n"
		"\t"	"ldd 25, y + %[current_pos] + 2"	"\n"
		"\t"	"inc 25"				"\n"
		"\t"	"std y + %[current_pos] + 2, 25"	"\n"
		"\t"	"brne 2f"				"\n"
		"\t"	"ldd 25, y + %[current_pos] + 3"	"\n"
		"\t"	"inc 25"				"\n"
		"\t"	"std y + %[current_pos] + 3, 25"	"\n"
		"\t"	"rjmp 2f"				"\n"
	"1:\t"		"sub 25, 0"				"\n"
		"\t"	"std y + %[current_pos], 25"		"\n"
		"\t"	"ldd 25, y + %[current_pos] + 1"	"\n"
		"\t"	"sbc 25, 1"				"\n"
		"\t"	"std y + %[current_pos] + 1, 25"	"\n"
		"\t"	"brcc 2f"				"\n"
		"\t"	"ldd 25, y + %[current_pos] + 2"	"\n"
		"\t"	"subi 25, 1"				"\n"
		"\t"	"std y + %[current_pos] + 2, 25"	"\n"
		"\t"	"brcc 2f"				"\n"
		"\t"	"ldd 25, y + %[current_pos] + 3"	"\n"
		"\t"	"subi 25, 1"				"\n"
		"\t"	"std y + %[current_pos] + 3, 25"	"\n"
	"2:\t"
		"\t"	"movw 24, 0"				"\n"

		/* Set up step set and reset values. >x(26,27)(port) >1(value for on) >0(value for off)*/
		"\t"	"ldd 26, y + %[step_port]"	"\n"
		"\t"	"ldd 27, y + %[step_port] + 1"	"\n"
		"\t"	"ldd 21, y + %[step_bitmask]"	"\n"
		"\t"	"ld 0, x"			"\n"
		"\t"	"mov 1, 0"			"\n"
		"\t"	"or 1, 21"			"\n"
		"\t"	"com 21"			"\n"
		"\t"	"and 0, 21"			"\n"
		"\t"	"sbrs 20, %[step_invert_bit]"	"\n"
		"\t"	"rjmp 1f"			"\n"
		/* Swap contents of 0 and 1. */
		"\t"	"eor 0, 1"			"\n"
		"\t"	"eor 1, 0"			"\n"
		"\t"	"eor 0, 1"			"\n"
	"1:\t"						"\n"
		ADD_DIR_DELAY
		// Send pulses. <24,25(abs numsteps) <x(26,27)(port) <1(value for on) <0(value for off) Delay of 1 μs is required according to a4988 datasheet.  At 16MHz, that's 16 clock cycles.
		"\t"	"sbiw 24, 0"			"\n"
	"2:\t"		"breq 3f"			"\n"	// 1	3
		"\t"	"nop"				"\n"	// 1	4
		"\t"	"ldi 21, 3 + %[delay]"		"\n"	// 1	5
		"\t"	"nop"				"\n"	// 1	6
	"1:\t"		"dec 21"			"\n"	// n	n+6
		"\t"	"brne 1b"			"\n"	// 2n-1	3n+5
		"\t"	"st x, 1"			"\n"	// 2	3n+7=16 => n=3
		"\t"	"ldi 21, 4 + %[delay]"		"\n"	// 1	1
	"1:\t"		"dec 21"			"\n"	// m	m+1
		"\t"	"brne 1b"			"\n"	// 2m-1	3m
		"\t"	"sbiw 24, 1"			"\n"	// 2	3m+2
		"\t"	"st x, 0"			"\n"	// 2	3m+4=16 => m=4
		"\t"	"rjmp 2b"			"\n"	// 2	2
	"3:\t"		"clr 27"			"\n"	// r27 is 0 again.
		ADD_DIR_DELAY
		"\t"	"rjmp isr_action_continue"	"\n"
		//*/
	"isr_action_pattern:"				"\n"
		// Step pin is set every ISR, but it changes max 8 times per sample.
		"\t"	"dec 17"			"\n"
		"\t"	"lds 25, %[full_phase_bits]"	"\n"
		// Shift right until only 3 bits are left.
	"1:\t"		"cpi 25, 3"			"\n"
		"\t"	"breq 1f"			"\n"
		"\t"	"lsr 17"			"\n"
		"\t"	"dec 25"			"\n"
		"\t"	"rjmp 1b"			"\n"
	"1:\t"						"\n"

		// Load value.
		"\t"	"ld 24, z"			"\n"
		// Shift bits.
		"\t"	"cp 17, 27"			"\n"
	"1:\t"		"breq 1f"			"\n"
		"\t"	"lsr 24"			"\n"
		"\t"	"dec 17"			"\n"
		"\t"	"rjmp 1b"			"\n"
	"1:\t"						"\n"
		// Set or clear the step pin based on bit 0 of r24.
		"\t"	"sbrc 20, %[step_invert_bit]"	"\n"
		"\t"	"com 24"			"\n"
		"\t"	"sbrs 20, %[current_step_bit]"	"\n"
		"\t"	"rjmp 1f"			"\n"
		"\t"	"sbrc 24, 0"			"\n"
		"\t"	"rjmp 1f"			"\n"
		"\t"	"ldi 26, %[current_dir]"	"\n"
		"\t"	"eor 20, 26"			"\n"
		"\t"	"ldd 26, y + %[dir_port]"	"\n"
		"\t"	"ldd 27, y + %[dir_port] + 1"	"\n"
		"\t"	"ldd 1, y + %[dir_bitmask]"	"\n"
		"\t"	"ld 25, x"			"\n"
		"\t"	"or 25, 1"			"\n"
		"\t"	"com 1"				"\n"
		"\t"	"sbrs 20, %[current_dir_bit]"	"\n"
		"\t"	"and 25, 1"			"\n"
		"\t"	"st x, 25"			"\n"
	"1:"						"\n"
		// Save current step flag.
		"\t"	"ldi 25, %[current_step]"	"\n"
		"\t"	"or 20, 25"			"\n"
		"\t"	"com 25"			"\n"
		"\t"	"sbrs 24, 0"			"\n"
		"\t"	"and 20, 25"			"\n"
		// Store updated flags.
		"\t"	"std y + %[flags], 20"		"\n"

		"\t"	"ldd 26, y + %[step_port]"	"\n"
		"\t"	"ldd 27, y + %[step_port] + 1"	"\n"
		"\t"	"ldd 1, y + %[step_bitmask]"	"\n"
		"\t"	"ld 25, x"			"\n"
		"\t"	"or 25, 1"			"\n"
		"\t"	"com 1"				"\n"
		"\t"	"sbrs 24, 0"			"\n"
		"\t"	"and 25, 1"			"\n"
		"\t"	"st x, 25"			"\n"

		// Restore variables.
		"\t"	"clr 27"			"\n"
		"\t"	"lds 17, %[move_phase]"		"\n"

		::
			[current_buffer] "" (&current_buffer),
			[full_phase_bits] "" (&full_phase_bits),
			[move_phase] "" (&move_phase),
			[step_state] "" (&step_state),
			[active_motors] "" (&active_motors),
			[current_sample] "" (&current_sample),
			[motor] "" (&motor),
			[current_pos] "I" (offsetof(Motor, current_pos)),
			[step_port] "I" (offsetof(Motor, step_port)),
			[step_bitmask] "I" (offsetof(Motor, step_bitmask)),
			[step_invert_bit] "M" (Motor::INVERT_STEP_BIT),
			[dir_port] "I" (offsetof(Motor, dir_port)),
			[dir_bitmask] "I" (offsetof(Motor, dir_bitmask)),
			[steps_current] "I" (offsetof(Motor, steps_current)),
			[flags] "I" (offsetof(Motor, intflags)),
			[activebit] "I" (Motor::ACTIVE_BIT),
			[patternbit] "I" (Motor::PATTERN_BIT),
			[current_step_bit] "M" (Motor::CURRENT_STEP_BIT),
			[current_step] "M" (Motor::CURRENT_STEP),
			[current_dir_bit] "M" (Motor::CURRENT_DIR_BIT),
			[current_dir] "M" (Motor::CURRENT_DIR),
			[fragment_size] "I" (BYTES_PER_FRAGMENT),
			[motor_size] "" (sizeof(Motor)),
			[timsk] "M" (_SFR_MEM_ADDR(TIMSK1)),
			[state_non_move] "M" (NUM_NON_MOVING_STATES),
			[delay] "M" (STEPS_DELAY),
			[debug] "" (&debug_value),
			[debug2] "" (&debug_value2)
		);

	asm volatile (
		// Increment Y and Z, dec counter 16 and loop.
	"isr_action_continue:"				"\n"
		"\t"	"dec 16"			"\n"
		"\t"	"breq 1f"			"\n"
		"\t"	"adiw 28, %[motor_size]"	"\n"
		"\t"	"adiw 30, %[fragment_size]"	"\n"
		"\t"	"rjmp isr_action_loop"		"\n"
	"1:\t"						"\n"
		// Check if this sample is completed (move_phase >= full_phase).
		"\t"	"lds 16, %[full_phase]"		"\n"
		"\t"	"cp 17, 16"			"\n"
		"\t"	"brcc 1f"			"\n"
		"\t"	"rjmp isr_end"			"\n"
	"1:\t"						"\n"
		// Next sample.
		// move_phase = 0;
		"\t"	"sts %[move_phase], 27"		"\n"
		// Decay step_state.
		"\t"	"lds 16, %[step_state]"		"\n"
		"\t"	"subi 16, %[state_decay]"	"\n"
		"\t"	"sts %[step_state], 16"		"\n"
	"1:\t"						"\n"
		// for all motors: steps_current = 0.
		"\t"	"lds 17, %[active_motors]"	"\n"
		"\t"	"ldi 28, lo8(%[motor])"		"\n"
		"\t"	"ldi 29, hi8(%[motor])"		"\n"
	"1:\t"		"std y + %[steps_current], 27"	"\n"
		"\t"	"std y + %[steps_current] + 1, 27"	"\n"
		"\t"	"adiw 28, %[motor_size]"	"\n"
		"\t"	"dec 17"			"\n"
		"\t"	"brne 1b"			"\n"
		// If current_sample + 1 < len: inc and return.
		"\t"	"lds 16, %[current_sample]"	"\n"
		"\t"	"inc 16"			"\n"
		"\t"	"lds 17, %[current_len]"	"\n"
		"\t"	"cp 16, 17"			"\n"
		"\t"	"brcc 1f"			"\n"
		"\t"	"sts %[current_sample], 16"	"\n"
		"\t"	"rjmp isr_end"			"\n"
	"1:\t"						"\n"
		// Go to next fragment.
		"\t"	"sts %[current_sample], 27"	"\n"
		"\t"	"lds 16, %[current_fragment]"	"\n"
		"\t"	"inc 16"			"\n"
		"\t"	"cpi 16, %[num_fragments]"	"\n"
		"\t"	"brcs 1f"			"\n"
		"\t"	"clr 16"			"\n"
		"\t"	"ldi 30, lo8(%[buffer])"	"\n"
		"\t"	"ldi 31, hi8(%[buffer])"	"\n"
		"\t"	"rjmp 2f"			"\n"
	"1:\t"		"lds 30, %[current_buffer]"	"\n"
		"\t"	"lds 31, %[current_buffer] + 1"	"\n"
		"\t"	"subi 30, -%[buffer_size] & 0xff"	"\n"
		"\t"	"sbci 31, (-%[buffer_size] >> 8) & 0xff"	"\n"
	"2:\t"		"sts %[current_buffer], 30"	"\n"
		"\t"	"sts %[current_buffer] + 1, 31"	"\n"
		"\t"	"sts %[current_fragment], 16"	"\n"
		/* Check for underrun. */
		"\t"	"lds 17, %[last_fragment]"		"\n"
		"\t"	"cp 16, 17"			"\n"
		"\t"	"breq isr_underrun"		"\n"
		/* There is a next fragment; set it up. */
		/* Activation of all motors. */
		"\t"	"lds 17, %[active_motors]"	"\n"
		"\t"	"ldi 28, lo8(%[motor])"		"\n"
		"\t"	"ldi 29, hi8(%[motor])"		"\n"
	"2:\t"		"ldd 24, y + %[flags]"		"\n"
		"\t"	"ori 24, %[active]"		"\n"
		"\t"	"ld 25, z"			"\n"
		"\t"	"cpi 25, 0x80"			"\n"
		"\t"	"brne 1f"			"\n"
		"\t"	"andi 24, ~%[active]"		"\n"
	"1:\t"		"std y + %[flags], 24"		"\n"
		"\t"	"adiw 28, %[motor_size]"	"\n"
		"\t"	"adiw 30, %[fragment_size]"	"\n"
		"\t"	"dec 17"			"\n"
		"\t"	"brne 2b"			"\n"
		/* Set current length. */
		"\t"	"ldi 28, lo8(%[settings])"		"\n"
		"\t"	"ldi 29, hi8(%[settings])"		"\n"
		"\t"	"tst 16"			"\n"
		"\t"	"breq 2f"			"\n"
	"1:\t"		"adiw 28, %[settings_size]"	"\n"
		"\t"	"dec 16"			"\n"
		"\t"	"brne 1b"			"\n"
	"2:\t"		"ldd 16, y + %[len]"		"\n"
		"\t"	"sts %[current_len], 16"	"\n"
		"\t"	"rjmp isr_end"			"\n"
		// Underrun.
	"isr_underrun:\t"				"\n"
		"\t"	"ldi 16, %[state_stop]"		"\n"
		"\t"	"sts %[step_state], 16"		"\n"
	"isr_end:"					"\n"
		"\t"	"pop 31"			"\n"
		"\t"	"pop 30"			"\n"
		"\t"	"pop 29"			"\n"
		"\t"	"pop 28"			"\n"

		"\t"	"pop 26"			"\n"
		"\t"	"pop 25"			"\n"
		"\t"	"pop 24"			"\n"
		"\t"	"pop 21"			"\n"
		"\t"	"pop 20"			"\n"
		"\t"	"pop 17"			"\n"
		"\t"	"pop 1"				"\n"
		"\t"	"pop 0"				"\n"
		"\t"	"cli"				"\n"
		"\t"	"lds 16, %[step_state]"		"\n"
		"\t"	"cpi 16, %[state_stop]"		"\n"
		"\t"	"breq 1f"			"\n"
		"\t"	"ldi 27, %[timskval]"		"\n"
		"\t"	"sts %[timsk], 27"		"\n"
	"1:"						"\n"
		"\t"	"pop 27"			"\n"
	"isr_end16:"					"\n"
		"\t"	"pop 16"			"\n"
		"\t"	"out __SREG__, 16"		"\n"
		"\t"	"pop 16"			"\n"
		"\t"	"reti"				"\n"

		::
			[current_buffer] "" (&current_buffer),
			[full_phase] "" (&full_phase),
			[move_phase] "" (&move_phase),
			[step_state] "" (&step_state),
			[active_motors] "" (&active_motors),
			[current_sample] "" (&current_sample),
			[current_len] "" (&current_len),
			[motor] "" (&motor),
			[current_fragment] "" (&current_fragment),
			[last_fragment] "" (&last_fragment),
			[buffer] "" (&buffer),
			[settings] "" (&settings),
			[steps_current] "I" (offsetof(Motor, steps_current)),
			[flags] "I" (offsetof(Motor, intflags)),
			[active] "M" (Motor::ACTIVE),
			[fragment_size] "I" (BYTES_PER_FRAGMENT),
			[num_fragments] "M" (1 << FRAGMENTS_PER_MOTOR_BITS),
			[buffer_size] "" (BYTES_PER_FRAGMENT * NUM_MOTORS),
			[motor_size] "" (sizeof(Motor)),
			[settings_size] "I" (sizeof(Settings)),
			[len] "I" (offsetof(Settings, len)),
			[timsk] "M" (_SFR_MEM_ADDR(TIMSK1)),
			[timskval] "M" (1 << OCIE1A),
			[state_stop] "M" (STEP_STATE_STOP),
			[state_decay] "M" (STATE_DECAY)
		);
}
#else
ISR(TIMER1_COMPA_vect) {
	SLOW_ISR();
}
// }}}
#endif

// Timekeeping. {{{
ISR(TIMER0_OVF_vect) { // {{{
	// At overflow of TIMER0, adjust:
	// avr_seconds_h: high byte for seconds.
	// avr_seconds: low byte for seconds. Used directly for counting seconds.
	// avr_time_h: high bits for millis(). Lower bits of this are always 0. Bitwise or'd with TCNT0 when used.

	// Timing:
	// 16e6 system ticks per second.
	// 16e6/1024=15625 timer ticks per second.
	// 15.625=125/8 timer ticks per ms.
	// 16e6/1024/256=1/0.016384 overflows per second
	// 16.384 ms/overflow

	// When there are avr_time_h >> 8 overflows,
	// there are (avr_time_h >> 8) / (1e6 / (1 << 14)) seconds
	// = (avr_time_h << 6) / 1e6
	// = avr_time_h / 15625

	// seconds = overflows * 0.016384
	// 	= avr_time_h / 256 * 0.016384
	//	= avr_time_h * 64 / 1000

	// Record the overflow.
	avr_time_h += 0x100;

	// Check if avr_time_h should overflow itself.
	// Also update seconds counter.
	uint32_t top = uint32_t(15625) << 11;
	while (avr_time_h > top) {
		avr_time_h -= top;
		avr_seconds += 1e3;
	}
} // }}}
#endif

static inline uint16_t millis() { // {{{
	cli();
	uint8_t l = TCNT0;
	uint32_t h = avr_time_h;
	if (TIFR0 & (1 << TOV0)) {
		l = TCNT0;
		h += 0x100;
	}
	sei();
	// There are 125/8 timer ticks/ms, so divide the timer ticks by 125/8 to get ms.
	return ((h | l) << 3) / 125;
} // }}}

static inline uint16_t seconds() { // {{{
	return avr_seconds + (avr_time_h >> 8) / 15625;
} // }}}
// }}}

// Pin control. {{{
inline void SET_OUTPUT(uint8_t pin_no) { // {{{
	if ((pin[pin_no].state & 0x3) == CTRL_SET || (pin[pin_no].state & 0x3) == CTRL_RESET)
		return;
	*pin[pin_no].avr_output &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_mode |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_RESET);
	pindebug("output pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline void SET_INPUT(uint8_t pin_no) { // {{{
	*pin[pin_no].avr_mode &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_output |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_INPUT | CTRL_NOTIFY);
	pindebug("input pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline void UNSET(uint8_t pin_no) { // {{{
	int timer = pgm_read_byte(digital_pin_to_timer_PGM + pin_no);
	if (timer != NOT_ON_TIMER) {
		Timer_data const *data = &timer_data[timer];
		*data->mode &= ~data->mode_mask;
		//debug("unset pin %d, OC%d%c mode to %x", pin_no, data->num, data->part, *data->mode);
	}
	*pin[pin_no].avr_mode &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_output &= ~pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_UNSET);
	pindebug("unset pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline void SET(uint8_t pin_no) { // {{{
	int timer = pgm_read_byte(digital_pin_to_timer_PGM + pin_no);
	if (timer != NOT_ON_TIMER) {
		Timer_data const *data = &timer_data[timer];
		if (pin[pin_no].duty < 0x7fff) {
			*data->mode |= data->mode_mask;
			//debug("set pin %d, OC%d%c mode to %x", pin_no, data->num, data->part, *data->mode);
			if (data->num == 1)
				update_timer1_pwm();
			else {
				if (timer_is_16bit(data->num)) {
					data->oc[1] = (pin[pin_no].duty >> 8) & 0xff;
					data->oc[0] = pin[pin_no].duty & 0xff;
				}
				else {
					*data->oc = (pin[pin_no].duty >> 7) & 0xff;
				}
			}
		}
		else {
			*data->mode &= ~data->mode_mask;
			//debug("set2 pin %d, OC%d%c mode to %x", pin_no, data->num, data->part, *data->mode);
		}
	}
	else if ((pin[pin_no].state & 0x3) == CTRL_SET)
		return;
	pin[pin_no].avr_on = true;
	pin[pin_no].avr_target = 0;
	*pin[pin_no].avr_output |= pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_mode |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_SET);
	pindebug("set pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline void RESET(uint8_t pin_no) { // {{{
	if ((pin[pin_no].state & 0x3) == CTRL_RESET)
		return;
	int timer = pgm_read_byte(digital_pin_to_timer_PGM + pin_no);
	if (timer != NOT_ON_TIMER) {
		Timer_data const *data = &timer_data[timer];
		*data->mode &= ~data->mode_mask;
		//debug("reset pin %d, OC%d%c mode to %x", pin_no, data->num, data->part, *data->mode);
	}
	*pin[pin_no].avr_output &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_mode |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_RESET);
	pindebug("reset pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline bool GET(uint8_t pin_no) { // {{{
	return *pin[pin_no].avr_input & pin[pin_no].avr_bitmask;
} // }}}

EXTERN uint8_t avr_outputs_last;
inline void arch_outputs() { // {{{
	uint8_t now = TCNT0;
	int16_t interval = (now - avr_outputs_last) & 0xff;
	if (interval <= 0)
		return;
	avr_outputs_last = now;
	for (uint8_t p = 0; p < NUM_DIGITAL_PINS; ++p) {
		if ((pin[p].state & 0x3) != CTRL_SET)
			continue;
		int timer = pgm_read_byte(digital_pin_to_timer_PGM + p);
		if (timer != NOT_ON_TIMER)
			continue;
		if (pin[p].avr_on)
			pin[p].avr_target -= uint32_t(interval) << 15;
		pin[p].avr_target += interval * (uint32_t(pin[p].duty) + 1);
		if (pin[p].avr_target < 0) {
			//if (pin[p].avr_on)
			//	debug("pin %d off target %x:%x duty %x interval %x", p, uint16_t(pin[p].avr_target >> 16), uint16_t(pin[p].avr_target), pin[p].duty, interval);
			*pin[p].avr_output &= ~pin[p].avr_bitmask;
			pin[p].avr_on = false;
		}
		else {
			//if (!pin[p].avr_on)
			//	debug("pin %d on target %x:%x duty %x interval %x", p, uint16_t(pin[p].avr_target >> 16), uint16_t(pin[p].avr_target), pin[p].duty, interval);
			*pin[p].avr_output |= pin[p].avr_bitmask;
			pin[p].avr_on = true;
		}
	}
	//debug("pin 4 state %x target %x:%x duty %x", pin[4].state, uint16_t(pin[4].avr_target >> 16), uint16_t(pin[4].avr_target), pin[4].duty);
} // }}}
// }}}
#endif
#endif

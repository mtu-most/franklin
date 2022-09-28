/* arch-firmware.cpp - avr specific parts for Franklin
 * vim: set foldmethod=marker :
 * Copyright 2014-2016 Michigan Technological University
 * Copyright 2016-2022 Bas Wijnen <wijnen@debian.org>
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

// Enable ADC pin info (for this file only).
#define INFO_ENABLE_ADC

#include "../../firmware/firmware.h"

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
struct SerialInfo {
	volatile uint8_t *udr;
	volatile uint8_t *ucsra;
	volatile uint8_t *ucsrb;
	volatile uint8_t *ucsrc;
	volatile uint8_t *ubrrh;
	volatile uint8_t *ubrrl;
	uint8_t tx_pin;
	uint8_t rx_pin;
};
#define ONEPORT(p) {udr: &UDR ## p, ucsra: &UCSR ## p ## A, ucsrb: &UCSR ## p ## B, ucsrc: &UCSR ## p ## C, ubrrh: &UBRR ## p ## H, ubrrl: &UBRR ## p ## L, tx_pin: TXD ## p ## _PIN, rx_pin: RXD ## p ## _PIN}
static SerialInfo const avr_serial_ports[NUM_SERIAL_PORTS] PROGMEM = {
#ifdef UDR0
	ONEPORT(0),
#endif
#ifdef UDR1
	ONEPORT(1),
#endif
#ifdef UDR2
	ONEPORT(2),
#endif
#ifdef UDR3
	ONEPORT(3),
#endif
};

static volatile uint8_t *avr_ucsra;
static volatile uint8_t *avr_udr;
// }}}

void arch_serial_write(uint8_t data) { // {{{
	if (avr_which_serial < 0)
		return;
	while (~*avr_ucsra & (1 << UDRE0)) {}
	*avr_udr = data;
} // }}}

static inline void arch_serial_flush() { // {{{
	if (avr_which_serial < 0)
		return;
	while (~*avr_ucsra & (1 << TXC0)) {}
} // }}}

void arch_claim_serial() { // {{{
	if (avr_which_serial == avr_last_serial)
		return;
	avr_which_serial = avr_last_serial;
	avr_ucsra = reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&avr_serial_ports[avr_which_serial].ucsra));
	avr_udr = reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&avr_serial_ports[avr_which_serial].udr));
	for (uint8_t i = 0; i < NUM_SERIAL_PORTS; ++i) {
		volatile uint8_t *ucsrb = reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&avr_serial_ports[i].ucsrb));
		if (i == avr_which_serial)
			*ucsrb = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
		else
			*ucsrb = 0;
	}
} // }}}

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
			[serialmask] "M" (SERIAL_MASK >> 8) \
	)
// }}}

#ifdef UDR0
ISR(USART0_RX_vect, ISR_NAKED) { // {{{
	avr_serial_input(0, UCSR0A, UDR0);
} // }}}
#endif

#ifdef UDR1
ISR(USART1_RX_vect, ISR_NAKED) { // {{{
	avr_serial_input(
// Only for atmega32u4: UDR0 does not exist, but UDR1 does. No other ports exist.
#ifdef UDR0
			1,
#else
			0,
#endif
			UCSR1A, UDR1);
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
// }}}

// Debugging. {{{
#ifndef NO_DEBUG
void debug_add(int i) { // {{{
	avr_debug[avr_debug_ptr] = i;
	avr_debug_ptr = (avr_debug_ptr + 1) & ((1 << AVR_DEBUG_BITS) - 1);
} // }}}

void debug_dump() { // {{{
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

void debug(char const *fmt, ...) { // {{{
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

#ifndef NO_ADC
// ADC. {{{
#define AVR_ADCSRA_BASE	((1 << ADEN) | 7)	// Prescaler set to 128.

void arch_adc_start(uint8_t adcpin) { // {{{
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

bool adc_ready(uint8_t pin_) { // {{{
	if (pin_ != avr_adc_last_pin) {
		adc_phase = PREPARING;
		arch_adc_start(pin_);
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

int16_t adc_get(uint8_t pin_) { // {{{
	(void)&pin_;
	int16_t low = uint8_t(ADCL);
	int16_t high = uint8_t(ADCH);
	int16_t ret = (high << 8) | low;
	adc_phase = INACTIVE;
	return ret;
} // }}}
// }}}
#endif

// EEPROM. {{{
uint8_t EEPROM_read(uint16_t addr) {
	uint8_t value;
	asm volatile (
		"\t"	"cli"				"\n"
		"1:\t"	"sbic %[eecr], %[eepe_bit]"	"\n"
		"\t"	"rjmp 1b"			"\n"
		"\t"	"out %[eearl], %[addrl]"	"\n"
		"\t"	"out %[eearh], %[addrh]"	"\n"
		"\t"	"sbi %[eecr], 0"		"\n"
		"\t"	"in %[data], %[eedr]"		"\n"
		"\t"	"sei"				"\n"
		:
		[data] "=r" (value)
		:
		[eecr] "M" (_SFR_IO_ADDR(EECR)),
		[eepe_bit] "" (EEPE),
		[addrl] "r" (addr & 0xff),
		[addrh] "r" (addr >> 8),
		[eearl] "M" (_SFR_IO_ADDR(EEARL)),
		[eearh] "M" (_SFR_IO_ADDR(EEARH)),
		[eedr] "M" (_SFR_IO_ADDR(EEDR))
	);
	return value;
}

void EEPROM_write(uint16_t addr, uint8_t value) {
	asm volatile (
		"\t"	"cli"				"\n"
		"1:\t"	"in __tmp_reg__, %[spmcsr]"	"\n"
		"\t"	"sbrc __tmp_reg__, %[spmen]"	"\n"
		"\t"	"rjmp 1b"			"\n"
		"1:\t"	"sbic %[eecr], %[eepe_bit]"	"\n"
		"\t"	"rjmp 1b"			"\n"
		"\t"	"out %[eearl], %[addrl]"	"\n"
		"\t"	"out %[eearh], %[addrh]"	"\n"
		"\t"	"out %[eedr], %[data]"		"\n"
		"\t"	"out %[eecr], %[eempe]"		"\n"
		"\t"	"out %[eecr], %[eepe]"		"\n"
		"\t"	"sei"				"\n"
		::
		[eecr] "M" (_SFR_IO_ADDR(EECR)),
		[eepe_bit] "" (EEPE),
		[spmcsr] "M" (_SFR_IO_ADDR(SPMCSR)),
		[spmen] "" (SPMEN),
		[addrl] "r" (addr & 0xff),
		[addrh] "r" (addr >> 8),
		[eearl] "M" (_SFR_IO_ADDR(EEARL)),
		[eearh] "M" (_SFR_IO_ADDR(EEARH)),
		[data] "r" (value),
		[eedr] "M" (_SFR_IO_ADDR(EEDR)),
		[eempe] "r" (1 << EEMPE),
		[eepe] "r" ((1 << EEMPE) | (1 << EEPE))
	);
}
// }}}

// Watchdog and reset. {{{
void arch_watchdog_enable() { // {{{
#ifdef WATCHDOG
	Wdt::reset();
	Wdt::enable(true, true, 8);
#endif
} // }}}

static inline void arch_watchdog_disable() { // {{{
#ifdef WATCHDOG
	Wdt::disable();
#endif
} // }}}

static inline void arch_reset() { // {{{
	// Warning: this may not work with all bootloaders.
	// The problem is that the bootloader must disable or reset the watchdog timer.
	// If it doesn't, it will continue resetting the device.
	Wdt::enable(false, true, 0);	// no interrupt, reset, As short as possible.
	while(1) {}
} // }}}
// }}}

// Setup. {{{

void arch_setup_start() { // {{{
	cli();
	for (uint8_t pin_no = GPIO_FIRST_PIN; pin_no < GPIO_LAST_PIN; ++pin_no) {
		if (!Gpio::check_pin(pin_no))
			continue;
		pin[pin_no - GPIO_FIRST_PIN].avr_on = false;
		pin[pin_no - GPIO_FIRST_PIN].avr_target = 0;
	}
	avr_time_h = 0;
	avr_seconds_h = 0;
	avr_seconds = 0;
	arch_watchdog_disable();
	// Serial ports.
	avr_last_serial = -1;
	avr_which_serial = -1;
	int const ubrr = int(F_CPU / (8. * BAUD) - .5);
	for (uint8_t i = 0; i < NUM_SERIAL_PORTS; ++i) {
		*reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&avr_serial_ports[i].ucsra)) = 1 << U2X0;
		*reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&avr_serial_ports[i].ucsrb)) = (1 << RXCIE0) | (1 << RXEN0);
		*reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&avr_serial_ports[i].ucsrc)) = 6;
		*reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&avr_serial_ports[i].ubrrh)) = ubrr >> 8;
		*reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&avr_serial_ports[i].ubrrl)) = ubrr & 0xff;	// 1:1M; 16:115k2
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
#ifndef NO_ADC
	ADCSRA = AVR_ADCSRA_BASE;
#endif
	// Enable interrupts.
	sei();
	// machineid will be filled by CMD_BEGIN.  Initialize it to 0.
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		machineid[1 + i] = 0;
	// Initialize uuid from EEPROM.
	for (uint8_t i = 0; i < UUID_SIZE; ++i)
		machineid[1 + ID_SIZE + i] = EEPROM_read(i);
} // }}}

void arch_msetup(uint8_t m) { // {{{
	if (motor[m].step_pin <= GPIO_LAST_PIN) {
		motor[m].step_port = &Gpio::PORT(motor[m].step_pin >> 3);
		motor[m].step_bitmask = _BV(motor[m].step_pin & 7);
	}
	else {
		motor[m].step_port = 0;	// This will access r0, which is harmless because bitmask is 0.
		motor[m].step_bitmask = 0;
	}
	if (motor[m].dir_pin <= GPIO_LAST_PIN) {
		motor[m].dir_port = &Gpio::PORT(motor[m].dir_pin >> 3);
		motor[m].dir_bitmask = _BV(motor[m].dir_pin & 7);
	}
	else {
		motor[m].dir_port = 0;	// This will access r0, which is harmless because bitmask is 0.
		motor[m].dir_bitmask = 0;
	}
	//debug("motor %x step: %x->%x:%x/%x, dir %x:%x/%x", m, motor[m].step_pin, motor[m].step_pin >> 3, motor[m].step_port, motor[m].step_bitmask, motor[m].dir_pin, motor[m].dir_port, motor[m].dir_bitmask);
} // }}}

uint8_t timer_pins[] = {
#ifdef OC1A_PIN
	OC1A_PIN,
#endif
#ifdef OC1B_PIN
	OC1B_PIN,
#endif
#ifdef OC1C_PIN
	OC1C_PIN,
#endif

#ifdef OC0A_PIN
	OC0A_PIN,
#endif
#ifdef OC0B_PIN
	OC0B_PIN,
#endif

#ifdef OC2A_PIN
	OC2A_PIN,
#endif
#ifdef OC2B_PIN
	OC2B_PIN,
#endif

#ifdef OC3A_PIN
	OC3A_PIN,
#endif
#ifdef OC3B_PIN
	OC3B_PIN,
#endif
#ifdef OC3C_PIN
	OC3C_PIN,
#endif

#ifdef OC4A_PIN
	OC4A_PIN,
#endif
#ifdef OC4B_PIN
	OC4B_PIN,
#endif
#ifdef OC4C_PIN
	OC4C_PIN,
#endif

#ifdef OC5A_PIN
	OC5A_PIN,
#endif
#ifdef OC5B_PIN
	OC5B_PIN,
#endif
#ifdef OC5C_PIN
	OC5C_PIN,
#endif
};

Timer_data const timer_data[] PROGMEM = {
// Timer1 pins must be first in this list.
	TIMER_DATA(&TCCR1A, &OCR1AL, 2 << 6, true, 1, 0)
#ifdef OC1B_PIN
	, TIMER_DATA(&TCCR1A, &OCR1BL, 2 << 4, true, 1, 1)
#endif
#ifdef OC1C_PIN
	, TIMER_DATA(&TCCR1A, &OCR1CL, 2 << 2, true, 1, 2)
#endif

#ifdef OC0A_PIN
	, TIMER_DATA(&TCCR0A, &OCR0A, 2 << 6, false, 0, 0)
#endif
#ifdef OC0B_PIN
	, TIMER_DATA(&TCCR0A, &OCR0B, 2 << 4, false, 0, 1)
#endif

#ifdef OC2A_PIN
	, TIMER_DATA(&TCCR2A, &OCR2A, 2 << 6, false, 2, 0)
#endif
#ifdef OC2B_PIN
	, TIMER_DATA(&TCCR2A, &OCR2B, 2 << 4, false, 2, 1)
#endif

#ifdef OC3A_PIN
	, TIMER_DATA(&TCCR3A, &OCR3AL, 2 << 6, true, 3, 0)
#endif
#ifdef OC3B_PIN
	, TIMER_DATA(&TCCR3A, &OCR3BL, 2 << 4, true, 3, 1)
#endif
#ifdef OC3C_PIN
	, TIMER_DATA(&TCCR3A, &OCR3CL, 2 << 2, true, 3, 2)
#endif

#ifdef OC4A_PIN
	, TIMER_DATA(&TCCR4A, &OCR4AL, 2 << 6, true, 4, 0)
#endif
#ifdef OC4B_PIN
	, TIMER_DATA(&TCCR4A, &OCR4BL, 2 << 4, true, 4, 1)
#endif
#ifdef OC4C_PIN
	, TIMER_DATA(&TCCR4A, &OCR4CL, 2 << 2, true, 4, 2)
#endif

#ifdef OC5A_PIN
	, TIMER_DATA(&TCCR5A, &OCR5AL, 2 << 6, true, 5, 0)
#endif
#ifdef OC5B_PIN
	, TIMER_DATA(&TCCR5A, &OCR5BL, 2 << 4, true, 5, 1)
#endif
#ifdef OC5C_PIN
	, TIMER_DATA(&TCCR5A, &OCR5CL, 2 << 2, true, 5, 2)
#endif
};

void update_timer1_pwm() { // {{{
	for (uint8_t i = 0;; ++i) {
		Timer_data const *data = &timer_data[i];
		if (pgm_read_byte(&data->num) != 1)
			break;
		// OC / top = duty / 0x7fff
		uint32_t value = pin[timer_pins[i] - GPIO_FIRST_PIN].duty;
		if (value < 0x7fff) {
			value *= timer1_top;
			value >>= 15;
			(reinterpret_cast <volatile uint16_t *> (pgm_read_ptr(&data->oc)))[1] = (value >> 8) & 0xff;
			(reinterpret_cast <volatile uint16_t *> (pgm_read_ptr(&data->oc)))[0] = value & 0xff;
			//debug("set timer1 pin %d:%d to %d for %d with top %d (%x %x) ocr1b %x %x", i, tp, (int)value, pin[tp - GPIO_FIRST_PIN].duty, timer1_top, ICR1H, ICR1L, OCR1BH, OCR1BL);
		}
		//else
			//debug("full power for timer1 pin %d:%d", i, tp);
	}
} // }}}

void arch_set_speed(uint16_t us_per_sample) { // {{{
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

#ifndef NO_SPI
void arch_spi_start() { // {{{
	SET_OUTPUT(MOSI_PIN);
	RESET(SCK_PIN);
} // }}}

void arch_spi_send(uint8_t data, uint8_t bits) { // {{{
	arch_spi_start();
	while (bits > 0) {
		if (data & 0x80)
			SET(MOSI_PIN);
		else
			RESET(MOSI_PIN);
		SET(SCK_PIN);
		RESET(SCK_PIN);
		data <<= 1;
		bits -= 1;
	}
} // }}}

void arch_spi_stop() { // {{{
	UNSET(MOSI_PIN);
	UNSET(SCK_PIN);
} // }}}
#endif

int8_t arch_pin_name(char *buffer_, bool digital, uint8_t pin_) { // {{{
	char *b = buffer_;
	if (digital) {

		// Skip pins that are in use.
		if (!Gpio::check_pin(pin_)
			|| (avr_which_serial >= 0 && (
				pin_ == pgm_read_byte(&avr_serial_ports[avr_which_serial].tx_pin)
				|| pin_ == pgm_read_byte(&avr_serial_ports[avr_which_serial].rx_pin)
			))
#if !defined(USE_RESET_PIN) && defined(RESET_PIN)
			|| pin_ == RESET_PIN
#endif
#if !defined(USE_XTAL_PINS) && defined(XTAL1_PIN)
			|| pin_ == XTAL1_PIN
#endif
#if !defined(USE_XTAL_PINS) && defined(XTAL2_PIN)
			|| pin_ == XTAL2_PIN
#endif
				) {
			buffer_[0] = '\0';
			return 0;
		}

		Timer_data const *data = get_timer(pin_);
		*b++ = 'P';
		*b++ = 'A' + (pin_ >> 3);
		*b++ = '0' + (pin_ & 7);
		if (data) {
			*b++ = '/';
			*b++ = 'O';
			*b++ = 'C';
			*b++ = '0' + pgm_read_byte(&data->num);
			*b++ = 'A' + pgm_read_byte(&data->part);
		}
	}
	else {
		uint8_t d_pin = Info::get_id(Info::first_adc_pin() + pin_);
		*b++ = 'A';
		if (pin_ >= 10) {
			*b++ = '1';
			pin_ -= 10;
		}
		*b++ = '0' + pin_;
		if (Gpio::check_pin(d_pin)) {
			*b++ = '/';
			*b++ = 'P';
			*b++ = 'A' + (d_pin >> 3);
			*b++ = '0' + (d_pin & 7);
		}
	}
	return b - buffer_;
} // }}}

// }}}

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
#ifndef NO_PATTERN
		"\t"	"sbrc 20, %[patternbit]"	"\n"
		"\t"	"rjmp isr_action_pattern"	"\n"
#endif

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
#ifndef NO_PATTERN
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
#endif
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
#ifndef NO_PATTERN
			[patternbit] "I" (Motor::PATTERN_BIT),
#endif
			[current_step_bit] "M" (Motor::CURRENT_STEP_BIT),
			[current_step] "M" (Motor::CURRENT_STEP),
			[current_dir_bit] "M" (Motor::CURRENT_DIR_BIT),
			[current_dir] "M" (Motor::CURRENT_DIR),
			[fragment_size] "I" (BYTES_PER_FRAGMENT),
			[motor_size] "" (sizeof(Motor)),
			[timsk] "M" (_SFR_MEM_ADDR(TIMSK1)),
			[state_non_move] "M" (NUM_NON_MOVING_STATES),
			[delay] "M" (STEPS_DELAY)
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
		avr_seconds += 1000;
	}
} // }}}
// }}}

// Pin control. {{{
void arch_outputs() { // {{{
	uint8_t now = TCNT0;
	int16_t interval = (now - avr_outputs_last) & 0xff;
	if (interval <= 0)
		return;
	avr_outputs_last = now;
	for (uint8_t p = GPIO_FIRST_PIN; p <= GPIO_LAST_PIN; ++p) {
		if ((pin[p - GPIO_FIRST_PIN].state & 0x3) != CTRL_SET)
			continue;
		if (get_timer(p))
			continue;
		if (pin[p - GPIO_FIRST_PIN].avr_on)
			pin[p - GPIO_FIRST_PIN].avr_target -= uint32_t(interval) << 15;
		pin[p - GPIO_FIRST_PIN].avr_target += interval * (uint32_t(pin[p - GPIO_FIRST_PIN].duty) + 1);
		if (pin[p - GPIO_FIRST_PIN].avr_target < 0) {
			//if (pin[p - GPIO_FIRST_PIN].avr_on)
			//	debug("pin %d off target %x:%x duty %x interval %x", p, uint16_t(pin[p - GPIO_FIRST_PIN].avr_target >> 16), uint16_t(pin[p - GPIO_FIRST_PIN].avr_target), pin[p - GPIO_FIRST_PIN].duty, interval);
			Gpio::PORT(p >> 3) &= ~_BV(p & 7);
			pin[p - GPIO_FIRST_PIN].avr_on = false;
		}
		else {
			//if (!pin[p - GPIO_FIRST_PIN].avr_on)
			//	debug("pin %d on target %x:%x duty %x interval %x", p, uint16_t(pin[p - GPIO_FIRST_PIN].avr_target >> 16), uint16_t(pin[p - GPIO_FIRST_PIN].avr_target), pin[p - GPIO_FIRST_PIN].duty, interval);
			Gpio::PORT(p >> 3) |= _BV(p & 7);
			pin[p - GPIO_FIRST_PIN].avr_on = true;
		}
	}
	//debug("pin 4 state %x target %x:%x duty %x", pin[4 - GPIO_FIRST_PIN].state, uint16_t(pin[4 - GPIO_FIRST_PIN].avr_target >> 16), uint16_t(pin[4 - GPIO_FIRST_PIN].avr_target), pin[4 - GPIO_FIRST_PIN].duty);
} // }}}
// }}}

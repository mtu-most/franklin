// vim: set foldmethod=marker :
// This is not an include quard.  This file is included twice; the first time the first part of the file is used, the second time the second part. {{{
#ifndef _ARCH_AVR_H
#define _ARCH_AVR_H

#define TIME_PER_ISR 20

#if 0
#define pindebug debug
#else
#define pindebug(...) do {} while (0)
#endif

// Define things that pins_arduino.h needs from Arduino.h (which shouldn't be included).
#define ARDUINO_MAIN
#define NOT_A_PIN 0
#define NOT_A_PORT 0
#define NOT_ON_TIMER 0
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
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER2  5
#define TIMER2A 6
#define TIMER2B 7
#define TIMER3A 8
#define TIMER3B 9
#define TIMER3C 10
#define TIMER4A 11
#define TIMER4B 12
#define TIMER4C 13
#define TIMER4D 14      
#define TIMER5A 15
#define TIMER5B 16
#define TIMER5C 17



#include <pins_arduino.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

#ifndef NO_DEBUG
static inline void debug(char const *fmt, ...);
#else
#define debug(...) do {} while (0)
#endif

#ifdef F
#undef F
#endif
#define F(x) &(x)

#define ARCH_PIN_DATA \
	volatile uint8_t *avr_mode; \
	volatile uint8_t *avr_output; \
	volatile uint8_t *avr_input; \
	uint8_t avr_bitmask;

inline void SET_OUTPUT(uint8_t pin_no);
inline void SET_INPUT(uint8_t pin_no);
inline void UNSET(uint8_t pin_no);
inline void SET(uint8_t pin_no);
inline void RESET(uint8_t pin_no);
inline bool GET(uint8_t pin_no);

#define ARCH_MOTOR \
	volatile uint16_t step_port, dir_port; \
	volatile uint8_t step_bitmask, dir_bitmask;

// Everything before this line is used at the start of firmware.h; everything after it at the end.
#else
EXTERN Pin_t pin[NUM_DIGITAL_PINS];
// }}}

#ifdef __cplusplus
// Defined by arduino: NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS

// Serial communication. {{{
static inline void debug_add(int i);
static inline void debug_dump();
#define SERIAL_MASK ((1 << SERIAL_SIZE_BITS) - 1)
#ifndef NO_SERIAL1
EXTERN volatile uint8_t which_serial;
#endif

static inline void serial_write(uint8_t data) {
#ifndef NO_SERIAL1
	if (which_serial != 1) {
#endif
		while (~UCSR0A & (1 << UDRE0)) {}
		UDR0 = data;
#ifndef NO_SERIAL1
	}
	else {
		while (~UCSR1A & (1 << UDRE1)) {}
		UDR1 = data;
	}
#endif
}

static inline void serial_flush() {
#ifndef NO_SERIAL1
	if (which_serial == 0)
#endif
		while (~UCSR0A & (1 << TXC0)) {}
#ifndef NO_SERIAL1
	else if (which_serial == 1)
		while (~UCSR1A & (1 << TXC1)) {}
#endif
}

static inline void arch_write_current_pos(uint8_t offset) {
	cli();
	for (uint8_t m = 0; m < active_motors; ++m)
		*reinterpret_cast <int32_t *>(&pending_packet[offset + 4 * m]) = motor[m].current_pos;
	sei();
}

static inline void arch_record_sense(bool state) {
	cli();
	for (int mi = 0; mi < active_motors; ++mi)
		motor[mi].sense_pos[state ? 1 : 0] = motor[mi].current_pos;
	sei();
}

#ifdef DEFINE_VARIABLES
static inline void handle_serial_input(
#ifndef NO_SERIAL1
		uint8_t which,
#endif
		uint8_t data, uint8_t status) {
#ifndef NO_SERIAL1
	if (which_serial != which) {
		if (status != 0 || data != CMD_ID)
			return;
		which_serial = which;
		// Disable other port so the pins can be used.
		if (which == 0)
			UCSR1B = 0;
		else
			UCSR0B = 0;
	}
#endif
	if (serial_overflow)
		return;
	uint16_t next = (serial_buffer_head + 1) & SERIAL_MASK;
	if (status != 0 || next == serial_buffer_tail) {
		debug_add(0xb00);
		debug_add(status);
		debug_add(serial_buffer_head);
		debug_add(serial_buffer_tail);
		serial_overflow = true;
		return;
	}
	serial_buffer[serial_buffer_head] = data;
	serial_buffer_head = next;
}

ISR(USART0_RX_vect) {
	uint8_t status = UCSR0A;
	handle_serial_input(
#ifndef NO_SERIAL1
			0,
#endif
			UDR0, status & ((1 << FE0) | (1 << DOR0)));
}

#ifndef NO_SERIAL1
ISR(USART1_RX_vect) {
	uint8_t status = UCSR1A;
	handle_serial_input(1, UDR1, status & ((1 << FE1) | (1 << DOR1)));
}
#endif
#endif
// }}}

// Debugging. {{{
#ifdef NO_DEBUG
static inline void debug_add(int i) { (void)&i; }
static inline void debug_dump() {}
#else
#define AVR_DEBUG_BITS 5
EXTERN volatile int avr_debug[1 << AVR_DEBUG_BITS];
EXTERN volatile int avr_debug_ptr;
static inline void debug_add(int i) {
	avr_debug[avr_debug_ptr] = i;
	avr_debug_ptr = (avr_debug_ptr + 1) & ((1 << AVR_DEBUG_BITS) - 1);
}

static inline void debug_dump() {
	debug("Debug dump (most recent last):");
	for (int i = 0; i < 1 << AVR_DEBUG_BITS; ++i)
		debug("%x", avr_debug[(avr_debug_ptr + i) & ((1 << AVR_DEBUG_BITS) - 1)]);
	debug("dump done");
}
static inline void print_num(int32_t num, int base) {
	uint32_t anum;
	if (num < 0) {
		serial_write('-');
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
		serial_write(c < 10 ? '0' + c : 'a' + c - 10);
	}
}

static inline void debug(char const *fmt, ...) {
#if DEBUG_BUFFER_LENGTH > 0
	buffered_debug_flush();
#endif
	va_list ap;
	va_start(ap, fmt);
	serial_write(CMD_DEBUG);
	for (char const *p = fmt; *p; ++p) {
		if (*p == '%') {
			bool longvalue = false;
			while (true) {
				++p;
				switch (*p) {
				case 0: {
					serial_write('%');
					--p;
					break;
				}
				case 'l': {
					longvalue = true;
					continue;
				}
				case '%': {
					serial_write('%');
					break;
				}
				case 'd': {
					if (longvalue) {
						int32_t *arg = va_arg(ap, int32_t *);
						print_num(*arg, 10);
					}
					else {
						int16_t arg = va_arg(ap, int16_t);
						print_num(arg, 10);
					}
					break;
				}
				case 'x': {
					if (longvalue) {
						int32_t *arg = va_arg(ap, int32_t *);
						print_num(*arg, 16);
					}
					else {
						int arg = va_arg(ap, int16_t);
						print_num(arg, 16);
					}
					break;
				}
				case 's': {
					char const *arg = va_arg(ap, char const *);
					while (*arg)
						serial_write(*arg++);
					break;
				}
				case 'c': {
					char arg = va_arg(ap, int);
					serial_write(arg);
					break;
				}
				default: {
					serial_write('%');
					serial_write(*p);
					break;
				}
				}
				break;
			}
		}
		else {
			serial_write(*p);
		}
	}
	va_end(ap);
	serial_write(0);
}
#endif
// }}}

// ADC. {{{
EXTERN uint8_t adc_last_pin;
#define ADCBITS 10
#define AVR_ADCSRA_BASE	((1 << ADEN) | 7)	// Prescaler set to 128.

#define fabs abs

static inline void adc_start(uint8_t adcpin) {
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
	adc_last_pin = adcpin;
}

static inline bool adc_ready(uint8_t pin_) {
	if (pin_ != adc_last_pin) {
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
}

static inline int16_t adc_get(uint8_t pin_) {
	(void)&pin_;
	int16_t low = uint8_t(ADCL);
	int16_t high = uint8_t(ADCH);
	int16_t ret = (high << 8) | low;
	adc_phase = INACTIVE;
	return ret;
}
// }}}

// Watchdog and reset. {{{
static inline void watchdog_enable() {
#ifdef WATCHDOG
	wdt_reset();
	wdt_enable(WDTO_1S);
#endif
}

static inline void watchdog_disable() {
#ifdef WATCHDOG
	wdt_disable();
#endif
}

static inline void watchdog_reset() {
#ifdef WATCHDOG
	wdt_reset();
#endif
}

static inline void reset() {
	wdt_enable(WDTO_15MS);	// As short as possible.
	while(1) {}
}
// }}}

// Setup. {{{
EXTERN volatile uint32_t avr_time_h, avr_seconds_h, avr_seconds;
EXTERN volatile bool lock;

static inline void arch_setup_start() {
	cli();
	lock = false;
	for (uint8_t pin_no = 0; pin_no < NUM_DIGITAL_PINS; ++pin_no) {
		uint8_t port = pgm_read_word(digital_pin_to_port_PGM + pin_no);
		pin[pin_no].avr_mode = reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_mode_PGM + port));
		pin[pin_no].avr_output = reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_output_PGM + port));
		pin[pin_no].avr_input = reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_input_PGM + port));
		pin[pin_no].avr_bitmask = pgm_read_word(digital_pin_to_bit_mask_PGM + pin_no);
	}
	avr_time_h = 0;
	avr_seconds_h = 0;
	avr_seconds = 0;
	watchdog_disable();
	// Serial ports.
	UCSR0A = 1 << U2X0;
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = 6;
	UBRR0H = 0;
	UBRR0L = 16;
#ifndef NO_SERIAL1
	UCSR1A = 1 << U2X1;
	UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);
	UCSR1C = 6;
	UBRR1H = 0;
	UBRR1L = 16;
	which_serial = -1;
#endif
	serial_overflow = false;
	serial_buffer_tail = 0;
	serial_buffer_head = 0;
	// Setup timer1 for microsecond counting.
	TCCR1A = 0;
	TCCR1B = 0x09;	// 16MHz clock.
	TIMSK1 = 0;	// Disable the interrupt while the motors are disabled.
	TIFR1 = 0xff;
	// Setup timer0 for millis().
	TCCR0A = 0;
	TCCR0B = 4;	// Clock/256: 62.5 ticks/millisecond.
	TIFR0 = 1 << TOV0;
	TIMSK0 |= 1 << TOIE0;
	// Setup ADC.
	ADCSRA = AVR_ADCSRA_BASE;
	// Enable interrupts.
	sei();
	// Initialize uuid from EEPROM.
	for (uint8_t i = 0; i < 16; ++i)
		uuid[i] = EEPROM.read(i);
	//// Make it a UUID (version 4).
	//printerid[7] &= 0x0f;
	//printerid[7] |= 0x40;
	//printerid[9] &= 0x3f;
	//printerid[9] |= 0x80;
	// printerid will be filled by CMD_BEGIN.  Initialize it to 0.
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		printerid[i] = 0;
}

static inline void arch_setup_end() {
	debug("Startup.");
}

static inline void arch_msetup(uint8_t m) {
	if (motor[m].step_pin < NUM_DIGITAL_PINS) {
		motor[m].step_port = int(pin[motor[m].step_pin].avr_output);
		motor[m].step_bitmask = pin[motor[m].step_pin].avr_bitmask;
	}
	else
		motor[m].step_bitmask = 0;
	if (motor[m].dir_pin < NUM_DIGITAL_PINS) {
		motor[m].dir_port = int(pin[motor[m].dir_pin].avr_output);
		motor[m].dir_bitmask = pin[motor[m].dir_pin].avr_bitmask;
	}
	else
		motor[m].dir_bitmask = 0;
}

static inline void set_speed(uint16_t count) {
	if (count == 0)
		step_state = 1;
	else {
		uint32_t c = count;
		c *= 16;
		c /= full_phase;
		// Set TOP.
		OCR1AH = (c >> 8) & 0xff;
		OCR1AL = c & 0xff;
		// Clear counter.
		TCNT1H = 0;
		TCNT1L = 0;
		// Clear and enable interrupt.
		TIFR1 = 1 << OCF1A;
		TIMSK1 |= 1 << OCIE1A;
	}
}
// }}}

#ifdef DEFINE_VARIABLES
#define offsetof(type, field) __builtin_offsetof(type, field) //(int(reinterpret_cast<volatile char *>(&reinterpret_cast<type *>(0)->field)))
ISR(TIMER1_COMPA_vect, ISR_NAKED) { // {{{
	asm volatile(
		// Naked ISR, so all registers must be saved.
		"\t"	"push 16"			"\n"
		"\t"	"in 16, __SREG__"		"\n"
		"\t"	"push 16"			"\n"
		// If lock or step_state < 2: return (increment lock to queue another ISR if locked).
		"\t"	"lds 16, lock"			"\n"
		"\t"	"inc 16"			"\n"
		"\t"	"sts lock, 16"			"\n"
		"\t"	"cpi 16, 1"			"\n"
		"\t"	"breq isr_restart"		"\n"
		"\t"	"rjmp isr_end_nolock"		"\n"
	"isr_restart:"					"\n"
		"\t"	"lds 16, step_state"		"\n"
		"\t"	"cpi 16, 2"			"\n"
		"\t"	"brcc 1f"			"\n"
		"\t"	"rjmp isr_end_lockonly"		"\n"
		// Enable interrupts, so the uart doesn't overrun.
	"1:\t"		"sei"				"\n"
		"\t"	"push 0"			"\n"
		"\t"	"push 1"			"\n"
		"\t"	"push 17"			"\n"
		"\t"	"push 18"			"\n"
		"\t"	"push 19"			"\n"
		"\t"	"push 20"			"\n"
		"\t"	"push 26"			"\n"
		"\t"	"push 27"			"\n"
		"\t"	"push 28"			"\n"
		"\t"	"push 29"			"\n"
		"\t"	"push 30"			"\n"
		"\t"	"push 31"			"\n"
		"\t"	"lds 16, active_motors"		"\n"
		// move_phase += 1;
		"\t"	"lds 17, move_phase"		"\n"
		"\t"	"inc 17"			"\n"
		"\t"	"sts move_phase, 17"		"\n"
		// Clear x.h
		"\t"	"clr 27"			"\n"
	// Register usage:
	// 16: motor countdown.
	// 17: move_phase.
	// 18: sample value (signed).
	// 19: sample value (abs).
	// 0: steps target.
	// 1, 20: general purpose.
	// x: pin pointer for varying pins.	x.h is 0 a lot (but not always).
	// y: motor pointer.
	// z: buffer pointer (pointing at current_sample).
		"\t"	"ldi 28, lo8(motor)"		"\n"
		"\t"	"ldi 29, hi8(motor)"		"\n"
		"\t"	"lds 30, current_buffer"	"\n"
		"\t"	"lds 31, current_buffer + 1"	"\n"
		"\t"	"lds 18, current_sample"	"\n"
		"\t"	"add 30, 18"			"\n"
		"\t"	"adc 31, 27"			"\n"
	"isr_action_loop:"				"\n"
		// If not active: continue
		"\t"	"ldd 20, y + %[flags]"		"\n"
		"\t"	"sbrs 20, %[activebit]"		"\n"
		"\t"	"rjmp isr_action_continue"	"\n"
		// Load value.
		"\t"	"ld 18, z"			"\n"
		"\t"	"mov 19, 18"			"\n"
		// positive ? set dir : reset dir
		"\t"	"ldd 26, y + %[dir_port]"	"\n"
		"\t"	"ldd 27, y + %[dir_port] + 1"	"\n"	// r27 can be non-zero from here.
		"\t"	"ldd 0, y + %[dir_bitmask]"	"\n"
		"\t"	"ld 1, x"			"\n"	// 1 is current port value.
		"\t"	"sbrc 20, %[audiobit]"		"\n"
		"\t"	"rjmp isr_audio"		"\n"
		"\t"	"tst 18"			"\n"	// Test sample sign.
		"\t"	"brmi 1f"			"\n"
		"\t"	"or 1, 0"			"\n"
		"\t"	"rjmp 2f"			"\n"
	"1:\t"		"com 0"				"\n"
		"\t"	"and 1, 0"			"\n"
		"\t"	"neg 19"			"\n"
	"2:\t"		"st x, 1"			"\n"

#define compute_steps(CONTINUE) \
		/* steps target = b.sample * move_phase / full_phase - m.steps_current; */ \
		"\t"	"mul 19, 17"			"\n" \
		"\t"	"lds 19, full_phase_bits"	"\n" \
		"\t"	"tst 19"			"\n" \
		"\t"	"rjmp 2f"			"\n" \
	"1:\t"		"lsr 1"				"\n" \
		"\t"	"ror 0"				"\n" \
		"\t"	"dec 19"			"\n" \
	"2:\t"		"brne 1b"			"\n" \
		"\t"	"ldd 19, y + %[steps_current]"	"\n" \
		"\t"	"std y + %[steps_current], 0"	"\n" /* motor[m].steps_current += steps_target; */ \
		"\t"	"sub 0, 19"			"\n" \
		/* If no steps: continue. */ \
		"\t"	"brne 1f"			"\n" \
		"\t"	"clr 27"			"\n"	/* We jump into the zone where r27 is expected to be 0. */ \
		"\t"	CONTINUE			"\n" \
	"1:\t"						"\n"

		compute_steps("rjmp isr_action_continue")

		//motor[m].current_pos += (motor[m].dir == DIR_POSITIVE ? steps_target : -steps_target);
		"\t"	"ldd 19, y + %[current_pos]"		"\n"
		"\t"	"tst 18"				"\n"
		"\t"	"brmi 1f"				"\n"
		"\t"	"add 19, 0"				"\n"
		"\t"	"std y + %[current_pos], 19"		"\n"
		"\t"	"brcc 2f"				"\n"
		"\t"	"ldd 19, y + %[current_pos] + 1"	"\n"
		"\t"	"inc 19"				"\n"
		"\t"	"std y + %[current_pos] + 1, 19"	"\n"
		"\t"	"brne 2f"				"\n"
		"\t"	"ldd 19, y + %[current_pos] + 2"	"\n"
		"\t"	"inc 19"				"\n"
		"\t"	"std y + %[current_pos] + 2, 19"	"\n"
		"\t"	"brne 2f"				"\n"
		"\t"	"ldd 19, y + %[current_pos] + 3"	"\n"
		"\t"	"inc 19"				"\n"
		"\t"	"std y + %[current_pos] + 3, 19"	"\n"
		"\t"	"rjmp 2f"				"\n"
	"1:\t"		"sub 19, 0"				"\n"
		"\t"	"std y + %[current_pos], 19"		"\n"
		"\t"	"brcc 2f"				"\n"
		"\t"	"ldd 19, y + %[current_pos] + 1"	"\n"
		"\t"	"subi 19, 1"				"\n"
		"\t"	"std y + %[current_pos] + 1, 19"	"\n"
		"\t"	"brcc 2f"				"\n"
		"\t"	"ldd 19, y + %[current_pos] + 2"	"\n"
		"\t"	"subi 19, 1"				"\n"
		"\t"	"std y + %[current_pos] + 2, 19"	"\n"
		"\t"	"brcc 2f"				"\n"
		"\t"	"ldd 19, y + %[current_pos] + 3"	"\n"
		"\t"	"subi 19, 1"				"\n"
		"\t"	"std y + %[current_pos] + 3, 19"	"\n"
	"2:\t"							"\n"

#define setup_step_masks \
		/* Set up step set and reset values. */ \
		"\t"	"ldd 26, y + %[step_port]"	"\n" \
		"\t"	"ldd 27, y + %[step_port] + 1"	"\n" \
		"\t"	"ldd 18, y + %[step_bitmask]"	"\n" \
		"\t"	"ld 19, x"			"\n" \
		"\t"	"mov 1, 19"			"\n" \
		"\t"	"or 19, 18"			"\n" \
		"\t"	"com 18"			"\n" \
		"\t"	"and 1, 18"			"\n" \
		"\t"	"sbrs 20, %[step_invert_bit]"	"\n" \
		"\t"	"breq 1f"			"\n" \
		/* Swap contents of 19 and 1. */ \
		"\t"	"eor 19, 1"			"\n" \
		"\t"	"eor 1, 19"			"\n" \
		"\t"	"eor 19, 1"			"\n"

		setup_step_masks

		// Send pulses.  Delay 1 μs is required according to a4988 datasheet.  At 16MHz, that's 16 clock cycles.
	"1:\t"		"tst 0"				"\n"
	"2:\t"		"breq 3f"			"\n"	// 1	3m+4
		"\t"	"nop"				"\n"	// 1	3m+5
		"\t"	"st x, 19"			"\n"	// 2	3m+7=16 => m=3
		"\t"	"ldi 18, 4"			"\n"	// 1	1
		"\t"	"nop"				"\n"	// 1	2
		"\t"	"nop"				"\n"	// 1	3
	"1:\t"		"dec 18"			"\n"	// n	n+3
		"\t"	"brne 1b"			"\n"	// 2n-1	3n+2
		"\t"	"st x, 1"			"\n"	// 2	3n+4=16 => n=4
		"\t"	"ldi 18, 3"			"\n"	// 1	1
	"1:\t"		"dec 18"			"\n"	// m	m+1
		"\t"	"brne 1b"			"\n"	// 2m-1	3m
		"\t"	"dec 0"				"\n"	// 1	3m+1
		"\t"	"rjmp 2b"			"\n"	// 2	3m+3
	"3:\t"		"clr 27"			"\n"	// r27 is 0 again.
		//*/
		// Increment Y and Z, dec counter 16 and loop.
	"isr_action_continue:"				"\n"
		"\t"	"dec 16"			"\n"
		"\t"	"breq 1f"			"\n"
		"\t"	"adiw 28, %[motor_size]"	"\n"
		"\t"	"adiw 30, %[fragment_size]"	"\n"
		"\t"	"rjmp isr_action_loop"		"\n"
	"1:\t"						"\n"
		// Check if this sample is completed (move_phase >= full_phase).
		"\t"	"lds 16, full_phase"		"\n"
		"\t"	"cp 17, 16"			"\n"
		"\t"	"brcc 1f"			"\n"
		"\t"	"rjmp isr_end"			"\n"
	"1:\t"						"\n"
		// Next sample.
		// move_phase = 0;
		"\t"	"sts move_phase, 27"		"\n"
		// If step_state == 2(single): step_state = 0(wait for sensor).
		"\t"	"lds 16, step_state"		"\n"
		"\t"	"cpi 16, 2"			"\n"
		"\t"	"brne 1f"			"\n"
		"\t"	"sts step_state, 27"		"\n"
	"1:\t"						"\n"
		// for all motors: steps_current = 0.
		"\t"	"lds 17, active_motors"		"\n"
		"\t"	"ldi 28, lo8(motor)"		"\n"
		"\t"	"ldi 29, hi8(motor)"		"\n"
	"1:\t"		"std y + %[steps_current], 27"	"\n"
		"\t"	"adiw 28, %[motor_size]"	"\n"
		"\t"	"dec 17"			"\n"
		"\t"	"brne 1b"			"\n"
		// If current_sample + 1 < len: inc and return.
		"\t"	"lds 16, current_sample"	"\n"
		"\t"	"inc 16"			"\n"
		"\t"	"lds 17, current_len"		"\n"
		"\t"	"cp 16, 17"			"\n"
		"\t"	"brcc 1f"			"\n"
		"\t"	"sts current_sample, 16"	"\n"
		"\t"	"rjmp isr_end"			"\n"
	"1:\t"						"\n"
		// Go to next fragment.
		"\t"	"sts current_sample, 27"	"\n"
		"\t"	"lds 16, current_fragment"	"\n"
		"\t"	"inc 16"			"\n"
		"\t"	"cpi 16, %[num_fragments]"	"\n"
		"\t"	"brcs 1f"			"\n"
		"\t"	"clr 16"			"\n"
		"\t"	"ldi 30, lo8(buffer)"		"\n"
		"\t"	"ldi 31, hi8(buffer)"		"\n"
		"\t"	"rjmp 2f"			"\n"
	"1:\t"		"lds 30, current_buffer"	"\n"
		"\t"	"lds 31, current_buffer + 1"	"\n"
		"\t"	"subi 30, -%[buffer_size] & 0xff"	"\n"
		"\t"	"sbci 31, (-%[buffer_size] >> 8) & 0xff"	"\n"
	"2:\t"		"sts current_buffer, 30"	"\n"
		"\t"	"sts current_buffer + 1, 31"	"\n"
		"\t"	"sts current_fragment, 16"	"\n"
		// Check for underrun.
		"\t"	"lds 17, last_fragment"		"\n"
		"\t"	"cp 16, 17"			"\n"
		"\t"	"breq 3f"			"\n"
		// There is a next fragment; set it up.
		// Activation of all motors.
		"\t"	"lds 17, active_motors"		"\n"
		"\t"	"ldi 28, lo8(motor)"		"\n"
		"\t"	"ldi 29, hi8(motor)"		"\n"
	"2:\t"		"ldd 18, y + %[flags]"		"\n"
		"\t"	"andi 18, ~%[active]"		"\n"
		"\t"	"ld 19, z"			"\n"
		"\t"	"cpi 19, 0x80"			"\n"
		"\t"	"breq 1f"			"\n"
		"\t"	"ori 18, %[active]"		"\n"
	"1:\t"		"std y + %[flags], 18"		"\n"
		"\t"	"adiw 28, %[motor_size]"	"\n"
		"\t"	"adiw 30, %[fragment_size]"	"\n"
		"\t"	"dec 17"			"\n"
		"\t"	"brne 2b"			"\n"
		// Set current length.
		"\t"	"ldi 28, lo8(settings)"		"\n"
		"\t"	"ldi 29, hi8(settings)"		"\n"
		"\t"	"tst 16"			"\n"
		"\t"	"breq 2f"			"\n"
	"1:\t"		"adiw 28, %[settings_size]"	"\n"
		"\t"	"dec 16"			"\n"
		"\t"	"brne 1b"			"\n"
	"2:\t"		"ldd 16, y + %[len]"		"\n"
		"\t"	"sts current_len, 16"		"\n"
		"\t"	"rjmp isr_end"			"\n"
		// Underrun.
	"3:\t"		"ldi 16, 1"			"\n"
		"\t"	"sts step_state, 16"		"\n"
	"isr_end:"	"\n"
		"\t"	"pop 31"			"\n"
		"\t"	"pop 30"			"\n"
		"\t"	"pop 29"			"\n"
		"\t"	"pop 28"			"\n"
		"\t"	"pop 27"			"\n"
		"\t"	"pop 26"			"\n"
		"\t"	"pop 20"			"\n"
		"\t"	"pop 19"			"\n"
		"\t"	"pop 18"			"\n"
		"\t"	"pop 17"			"\n"
		"\t"	"pop 1"				"\n"
		"\t"	"pop 0"				"\n"
		"\t"	"cli"				"\n"
	"isr_end_lockonly:"				"\n"
		"\t"	"lds 16, lock"			"\n"
		"\t"	"dec 16"			"\n"
		"\t"	"sts lock, 16"			"\n"
		"\t"	"breq isr_end_nolock"		"\n"
		"\t"	"rjmp isr_restart"		"\n"
	"isr_end_nolock:"				"\n"
		"\t"	"pop 16"			"\n"
		"\t"	"out __SREG__, 16"		"\n"
		"\t"	"pop 16"			"\n"
		"\t"	"reti"				"\n"
	// Audio handling.  Registers at entry:
		// 19: Sample value.
		// 20: Flags.
		// 0: Dir bitmask.
		// 1: Dir port current value.
		// 18, x: General purpose.
		// y: Motor.
	"isr_audio:"					"\n"
		"\t"	"push 16"			"\n"
		"\t"	"push 17"			"\n"
		"\t"	"push 30"			"\n"
		"\t"	"push 31"			"\n"
		// Prepare dir bitmasks (to stack).
		"\t"	"movw 30, 26"			"\n"
		"\t"	"mov 16, 1"			"\n"
		"\t"	"or 16, 0"			"\n"
		"\t"	"com 0"				"\n"
		"\t"	"and 1, 0"			"\n"
		"\t"	"mov 17, 1"			"\n"

		// Compute steps for this iteration.
		compute_steps("rjmp isr_audio_continue")	// Changes: 1, 19; result: 0

		// Prepare step bitmasks.
		setup_step_masks // Changes: 18.  Results: 1 (reset), 19 (set), x (port).

		// Do steps.
		// 0: counter.
		// 1: reset step.
		// 19: set step.
		// 17: reset dir.
		// 16: set dir.
		// 20: flags.
		// x: step.
		// z: dir.
		// 18: general purpose.
		"\t"	"tst 0"				"\n"
		"\t"	"rjmp isr_audio_end_loop"	"\n"
	"isr_audio_loop:"				"\n"
		// Set dir.
		"\t"	"mov 18, 16"			"\n"	// 1		3m+4
		"\t"	"sbrc 20, %[audiostatebit]"	"\n"	// 1/2		3m+5/6
		"\t"	"mov 18, 17"			"\n"	// 1/0		3m+6
		"\t"	"st z, 18"			"\n"	// 2		3m+8
		"\t"	"ldi 18, (1 << %[audiostatebit])"	"\n"	// 1	3m+9
		"\t"	"eor 20, 18"			"\n"	// 1		3m+10
		"\t"	"nop"				"\n"	// 1		3m+11
		
		// Step.
		"\t"	"st x, 19"			"\n"	// 2		3m+13=16 => m=1
		// Wait.
		"\t"	"ldi 18, 4"			"\n"	// 1		1
	"1:\t"		"dec 18"			"\n"	// n		n+1
		"\t"	"brne 1b"			"\n"	// 2n-1		3n
		"\t"	"nop"				"\n"	// 1		3m+1
		"\t"	"nop"				"\n"	// 1		3m+2
		"\t"	"st x, 1"			"\n"	// 2		3n+4=16	=> n=4
		// Wait.
		"\t"	"ldi 18, 1"			"\n"	// 1		1
	"1:\t"		"dec 18"			"\n"	// m		m+1
		"\t"	"brne 1b"			"\n"	// 2m-1		3m
		"\t"	"dec 0"				"\n"	// 1		3m+1
	"isr_audio_end_loop:"				"\n"
		"\t"	"breq isr_audio_loop"		"\n"	// 2		3m+3
		"\t"	"std y + %[flags], 20"		"\n"
	"isr_audio_continue:"				"\n"
		"\t"	"pop 31"			"\n"
		"\t"	"pop 30"			"\n"
		"\t"	"pop 17"			"\n"
		"\t"	"pop 16"			"\n"
		"\t"	"clr 27"			"\n"
		"\t"	"rjmp isr_action_continue"	"\n"
		::
			[current_pos] "I" (offsetof(Motor, current_pos)),
			[step_port] "I" (offsetof(Motor, step_port)),
			[step_bitmask] "I" (offsetof(Motor, step_bitmask)),
			[step_invert_bit] "M" (Motor::INVERT_STEP_BIT),
			[dir_port] "I" (offsetof(Motor, dir_port)),
			[dir_bitmask] "I" (offsetof(Motor, dir_bitmask)),
			[steps_current] "I" (offsetof(Motor, steps_current)),
			[flags] "I" (offsetof(Motor, intflags)),
			[activebit] "I" (Motor::ACTIVE_BIT),
			[active] "M" (Motor::ACTIVE),
			[audiobit] "I" (Motor::AUDIO_BIT),
			[audiostatebit] "I" (Motor::AUDIO_STATE_BIT),
			[fragment_size] "I" (BYTES_PER_FRAGMENT),
			[num_fragments] "M" (1 << FRAGMENTS_PER_MOTOR_BITS),
			[buffer_size] "" (BYTES_PER_FRAGMENT * NUM_MOTORS),
			[motor_size] "" (sizeof(Motor)),
			[settings_size] "I" (sizeof(Settings)),
			[len] "I" (offsetof(Settings, len))
		);
}
// }}}

// Timekeeping. {{{
ISR(TIMER0_OVF_vect) {
	uint32_t top = uint32_t(125) << 16;
	avr_time_h += 0x100;
	while (avr_time_h > top)
		avr_time_h -= top;
	uint32_t t = avr_time_h - avr_seconds_h + top;
	while (t >= top)
		t -= top;
	if (t >= 62500) {
		avr_seconds_h += 62500;
		while (avr_seconds_h >= top)
			avr_seconds_h -= top;
		avr_seconds += 1;
	}
}
#endif

static inline uint16_t millis() {
	cli();
	uint8_t l = TCNT0;
	uint32_t h = avr_time_h;
	if (TIFR0 & (1 << TOV0)) {
		l = TCNT0;
		h += 0x100;
	}
	sei();
	return ((h | l) << 1) / 125;
}

static inline uint16_t seconds() {
	return avr_seconds;
}
// }}}

// Pin control. {{{
inline void SET_OUTPUT(uint8_t pin_no) {
	if ((pin[pin_no].state & 0x3) == CTRL_SET || (pin[pin_no].state & 0x3) == CTRL_RESET)
		return;
	*pin[pin_no].avr_output &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_mode |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_RESET);
	pindebug("output pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
}

inline void SET_INPUT(uint8_t pin_no) {
	*pin[pin_no].avr_mode &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_output |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_INPUT | CTRL_NOTIFY);
	pindebug("input pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
}

inline void UNSET(uint8_t pin_no) {
	*pin[pin_no].avr_mode &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_output &= ~pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_UNSET);
	pindebug("unset pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
}

inline void SET(uint8_t pin_no) {
	if ((pin[pin_no].state & 0x3) == CTRL_SET)
		return;
	*pin[pin_no].avr_output |= pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_mode |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_SET);
	pindebug("set pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
}

inline void RESET(uint8_t pin_no) {
	if ((pin[pin_no].state & 0x3) == CTRL_RESET)
		return;
	*pin[pin_no].avr_output &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_mode |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_RESET);
	pindebug("reset pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
}

inline bool GET(uint8_t pin_no) {
	return *pin[pin_no].avr_input & pin[pin_no].avr_bitmask;
}
// }}}
#endif
#endif

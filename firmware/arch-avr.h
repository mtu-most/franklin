// vim: set foldmethod=marker :
// This is not an include quard.  This file is included twice; the first time the first part of the file is used, the second time the second part.
#ifndef _ARCH_AVR_H
#define _ARCH_AVR_H

// Defines and includes.  {{{
#ifdef FAST_ISR
#define TIME_PER_ISR 75
#else
#define TIME_PER_ISR 500
#endif

//#define pindebug debug
#define pindebug(...) do {} while (0)

// Define things that pins_arduino.h needs from Arduino.h (which shouldn't be included). {{{
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
// }}}

#include <pins_arduino.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

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
	int16_t avr_target;

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

static inline void arch_set_speed(uint16_t count);

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
volatile uint8_t *const avr_serial_ports[NUM_SERIAL_PORTS][6] = {
#ifdef UDR3
	{&UDR0, &UCSR0A, &UCSR0B, &UCSR0C, &UBRR0H, &UBRR0L},
	{&UDR1, &UCSR1A, &UCSR1B, &UCSR1C, &UBRR1H, &UBRR1L},
	{&UDR2, &UCSR2A, &UCSR2B, &UCSR2C, &UBRR2H, &UBRR2L},
	{&UDR3, &UCSR3A, &UCSR3B, &UCSR3C, &UBRR3H, &UBRR3L}
#else
#ifdef UDR2
	{&UDR0, &UCSR0A, &UCSR0B, &UCSR0C, &UBRR0H, &UBRR0L},
	{&UDR1, &UCSR1A, &UCSR1B, &UCSR1C, &UBRR1H, &UBRR1L},
	{&UDR2, &UCSR2A, &UCSR2B, &UCSR2C, &UBRR2H, &UBRR2L}
#else
#ifdef UDR1
	{&UDR0, &UCSR0A, &UCSR0B, &UCSR0C, &UBRR0H, &UBRR0L},
	{&UDR1, &UCSR1A, &UCSR1B, &UCSR1C, &UBRR1H, &UBRR1L}
#else
	{&UDR0, &UCSR0A, &UCSR0B, &UCSR0C, &UBRR0H, &UBRR0L}
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
		"\t"	"push 31"				"\n"	/* 2	 9 */ \
		"\t"	"in 31, __SREG__"			"\n"	/* 1	10 */ \
		"\t"	"push 31"				"\n"	/* 2	12 */ \
		"\t"	"push 30"				"\n"	/* 2	14 */ \
		"\t"	"push 29"				"\n"	/* 2	16 */ \
		"\t"	"lds 29, %[udr]"			"\n"	/* 2	18 */ \
		"\t"	"lds 30, %[ucsra]"			"\n"	/* 2	20 */ \
		"\t"	"andi 30, %[statusmask]"		"\n"	/* 1	21 */ \
		"\t"	"brne 5f"				"\n"	/* 1	22 */ \
		"\t"	"ldi 30, " #which			"\n"	/* 1	23 */ \
		"\t"	"sts avr_last_serial, 30"		"\n"	/* 2	25 */ \
		"\t"	"lds 30, serial_buffer_head"		"\n"	/* 2	27 */ \
		"\t"	"lds 31, serial_buffer_head + 1"	"\n"	/* 2	29 */ \
		"\t"	"st z, 29"				"\n"	/* 2	31 */ \
		"\t"	"inc 30"				"\n"	/* 1	32 */ \
		"\t"	"breq 1f"				"\n"	/* 1	33 */ \
		"\t"	"lds 29, serial_buffer_tail"		"\n"	/* 2	35 */ \
		"\t"	"cp 30, 29"				"\n"	/* 1	36 */ \
		"\t"	"breq 2f"				"\n"	/* 1	37 */ \
	"6:\t"		/* No overflow. */			"\n"	/*   */ \
		"\t"	"sts serial_buffer_head, 30"		"\n"	/* 2	39 */ \
	"7:\t"		/* Finish. */				"\n"	/*   */ \
		"\t"	"pop 29"				"\n"	/* 2	41 */ \
		"\t"	"pop 30"				"\n"	/* 2	43 */ \
		"\t"	"pop 31"				"\n"	/* 2	45 */ \
		"\t"	"out __SREG__, 31"			"\n"	/* 1	46 */ \
		"\t"	"pop 31"				"\n"	/* 2	48 */ \
		"\t"	"reti"					"\n"	/* 4	52 */ \
	"1:\t"		/* Carry in in lo8(head). */		"\n" \
		"\t"	"inc 31"				"\n" \
		"\t"	"andi 31, %[serialmask]"		"\n" \
		"\t"	"ori 31, hi8(serial_buffer)"		"\n" \
		"\t"	"lds 29, serial_buffer_tail"		"\n" \
		"\t"	"cp 30, 29"				"\n" \
		"\t"	"breq 3f"				"\n" \
	"4:\t"		/* No overflow. */			"\n" \
		"\t"	"sts serial_buffer_head + 1, 31"	"\n" \
		"\t"	"rjmp 6b"				"\n" \
	"3:\t"		/* lo8(head) == lo8(tail). */		"\n" \
		"\t"	"lds 29, serial_buffer_tail + 1"	"\n" \
		"\t"	"cp 31, 29"				"\n" \
		"\t"	"brne 4b"				"\n" \
	"5:\t"		/* Overflow. */				"\n" \
		"\t"	"ldi 29, 1"				"\n" \
		"\t"	"sts serial_overflow, 29"		"\n" \
		"\t"	"rjmp 7b"				"\n" \
	"2:\t"		/* lo8(head) == lo8(tail). */		"\n" \
		"\t"	"lds 29, serial_buffer_tail + 1"	"\n" \
		"\t"	"cp 31, 29"				"\n" \
		"\t"	"brne 6b"				"\n" \
		"\t"	"rjmp 5b"				"\n" \
		:: \
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
	}
	avr_time_h = 0;
	avr_seconds_h = 0;
	avr_seconds = 0;
	arch_watchdog_disable();
	// Serial ports.
	avr_last_serial = -1;
	avr_which_serial = -1;
	for (uint8_t i = 0; i < NUM_SERIAL_PORTS; ++i) {
		*avr_serial_ports[i][1] = 1 << U2X0;	// ucsra
		*avr_serial_ports[i][2] = (1 << RXCIE0) | (1 << RXEN0);	// ucsrb
		*avr_serial_ports[i][3] = 6;	// ucsrc
		*avr_serial_ports[i][4] = 0;	// ubrrh
		*avr_serial_ports[i][5] = 16;	// ubrrl 1:1M; 16:115k2
	}
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
	for (uint8_t i = 0; i < UUID_SIZE; ++i)
		uuid[i] = EEPROM.read(i);
	// printerid will be filled by CMD_BEGIN.  Initialize it to 0.
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		printerid[i] = 0;
} // }}}

static inline void arch_setup_end() { // {{{
	debug("Startup.");
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

static inline void arch_set_speed(uint16_t count) { // {{{
	if (count == 0) {
		TIMSK1 = 0;
		step_state = 1;
	}
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
		TIMSK1 = 1 << OCIE1A;
	}
} // }}}

static inline void arch_spi_start() { // {{{
	SET_OUTPUT(SS);
	if (!(SPCR & 0x40)) {
		SPCR = 0x53;
		SPSR = 0;
	}
} // }}}

static inline void arch_spi_send(uint8_t data) { // {{{
	arch_spi_start();
	SPDR = data;
	while (!(SPSR & 0x80)) {}
} // }}}

static inline void arch_spi_stop() { // {{{
	SPCR = 0x10;
} // }}}

// }}}

#ifdef DEFINE_VARIABLES
#define offsetof(type, field) __builtin_offsetof(type, field)
#ifdef FAST_ISR
ISR(TIMER1_COMPA_vect, ISR_NAKED) { // {{{
	asm volatile (
		// Naked ISR, so all registers must be saved.
		"\t"	"push 16"			"\n"
		"\t"	"in 16, __SREG__"		"\n"
		"\t"	"push 16"			"\n"
		// If step_state < 2: return.
		"\t"	"lds 16, step_state"		"\n"
		"\t"	"cpi 16, 2"			"\n"
		"\t"	"brcc 1f"			"\n"
		"\t"	"rjmp isr_end16"		"\n"
	"1:\t"						"\n"
		// Enable interrupts, so the uart doesn't overrun.
		"\t"	"push 27"			"\n"
		"\t"	"clr 27"			"\n"
		"\t"	"sts %[timsk], 27"		"\n"
		"\t"	"sei"				"\n"

		// If audio, everything is different.
		"\t"	"lds 16, audio"			"\n"
		"\t"	"tst 16"			"\n"
		"\t"	"breq 1f"			"\n"
		"\t"	"rjmp isr_audio"		"\n"
	"1:\t"						"\n"

		// Save all registers that are used.
		"\t"	"push 0"			"\n"
		"\t"	"push 1"			"\n"
		"\t"	"push 17"			"\n"
		"\t"	"push 18"			"\n"
		"\t"	"push 19"			"\n"
		"\t"	"push 20"			"\n"
		"\t"	"push 21"			"\n"
		"\t"	"push 26"			"\n"
		"\t"	"push 28"			"\n"
		"\t"	"push 29"			"\n"
		"\t"	"push 30"			"\n"
		"\t"	"push 31"			"\n"
		// move_phase += 1;
		"\t"	"lds 17, move_phase"		"\n"
		"\t"	"inc 17"			"\n"
		"\t"	"sts move_phase, 17"		"\n"
		// 16 is motor countdown; y is motor pointer.
		"\t"	"lds 16, active_motors"		"\n"
		"\t"	"ldi 28, lo8(motor)"		"\n"
		"\t"	"ldi 29, hi8(motor)"		"\n"
		// If active_motors is 0 and move is requested; trigger underrun immediately.
		"\t"	"tst 16"			"\n"
		"\t"	"brne 1f"			"\n"
		"\t"	"rjmp isr_underrun"		"\n"
	"1:\t"						"\n"

	// Register usage:
	// 16: motor countdown.
	// 17: move_phase.
	// 18: sample value (signed).
	// 19: sample value (abs).
	// 20: flags.
	// 0: steps target.
	// 1,21 general purpose.
	// x: pin pointer for varying pins.	x.h is 0 a lot (but not always).
	// y: motor pointer.
	// z: buffer pointer (pointing at current_sample).
		"\t"	"lds 30, current_buffer"	"\n"
		"\t"	"lds 31, current_buffer + 1"	"\n"
		"\t"	"lds 18, current_sample"	"\n"
		"\t"	"add 30, 18"			"\n"
		"\t"	"adc 31, 27"			"\n"
	"isr_action_loop:"				"\n"
		// If not active: continue	>20(flags) {{{
		"\t"	"ldd 20, y + %[flags]"		"\n"
		"\t"	"sbrs 20, %[activebit]"		"\n"
		"\t"	"rjmp isr_action_continue"	"\n"
		// }}}
		// Load value.
		"\t"	"ld 18, z"			"\n"
		"\t"	"mov 19, 18"			"\n"
		// Load dir data. >26+27(dir port) >0(bitmask) >21(current value) <20(flags) {{{
		"\t"	"ldd 26, y + %[dir_port]"	"\n"
		"\t"	"ldd 27, y + %[dir_port] + 1"	"\n"	// r27 can be non-zero from here.
		"\t"	"ldd 0, y + %[dir_bitmask]"	"\n"
		"\t"	"ld 21, x"			"\n"	// 1 is current port value.
		// }}}
		// positive ? set dir : reset dir (don't send yet) <18(numsteps) <0(dir bitmask) <21(current value) >21(new value) >19(abs numsteps) {{{
		"\t"	"tst 18"			"\n"	// Test sample sign.
		"\t"	"brmi 1f"			"\n"
		"\t"	"or 21, 0"			"\n"
		"\t"	"rjmp 2f"			"\n"
	"1:\t"		"com 0"				"\n"
		"\t"	"and 21, 0"			"\n"
		"\t"	"neg 19"			"\n"
	"2:\t"						"\n"
		// }}}

		/* Compute steps.  <19(abs numsteps) <17(move_phase) <y(motor) >0(steps) X1 {{{ */
		/* steps target = b.sample * move_phase / full_phase - m.steps_current; */
		"\t"	"mul 19, 17"			"\n"
		"\t"	"lds 19, full_phase_bits"	"\n"
		"\t"	"tst 19"			"\n"
		"\t"	"rjmp 2f"			"\n"
	"1:\t"		"lsr 1"				"\n"
		"\t"	"ror 0"				"\n"
		"\t"	"dec 19"			"\n"
	"2:\t"		"brne 1b"			"\n"
		"\t"	"ldd 19, y + %[steps_current]"	"\n"
		"\t"	"std y + %[steps_current], 0"	"\n" /* motor[m].steps_current += steps_target; */
		"\t"	"sub 0, 19"			"\n"
		/* If no steps: continue. */
		"\t"	"brne 1f"			"\n"
		"\t"	"clr 27"			"\n"	/* We jump into the zone where r27 is expected to be 0. */
		"\t"	"rjmp isr_action_continue"	"\n"
	"1:\t"						"\n"
		// }}}

		// Set direction.
		"\t"	"st x, 21"			"\n"

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
	"2:\t"

		/* Set up step set and reset values. */
		"\t"	"ldd 26, y + %[step_port]"	"\n"
		"\t"	"ldd 27, y + %[step_port] + 1"	"\n"
		"\t"	"ldd 18, y + %[step_bitmask]"	"\n"
		"\t"	"ld 19, x"			"\n"
		"\t"	"mov 1, 19"			"\n"
		"\t"	"or 19, 18"			"\n"
		"\t"	"com 18"			"\n"
		"\t"	"and 1, 18"			"\n"
		"\t"	"sbrs 20, %[step_invert_bit]"	"\n"
		"\t"	"rjmp 1f"			"\n"
		/* Swap contents of 19 and 1. */
		"\t"	"eor 19, 1"			"\n"
		"\t"	"eor 1, 19"			"\n"
		"\t"	"eor 19, 1"			"\n"
	"1:\t"						"\n"

		// Send pulses.  Delay of 1 μs is required according to a4988 datasheet.  At 16MHz, that's 16 clock cycles.
		"\t"	"tst 0"				"\n"
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
#define next_fragment(activation, underrun) \
		"\t"	"lds 16, current_fragment"	"\n" \
		"\t"	"inc 16"			"\n" \
		"\t"	"cpi 16, %[num_fragments]"	"\n" \
		"\t"	"brcs 1f"			"\n" \
		"\t"	"clr 16"			"\n" \
		"\t"	"ldi 30, lo8(buffer)"		"\n" \
		"\t"	"ldi 31, hi8(buffer)"		"\n" \
		"\t"	"rjmp 2f"			"\n" \
	"1:\t"		"lds 30, current_buffer"	"\n" \
		"\t"	"lds 31, current_buffer + 1"	"\n" \
		"\t"	"subi 30, -%[buffer_size] & 0xff"	"\n" \
		"\t"	"sbci 31, (-%[buffer_size] >> 8) & 0xff"	"\n" \
	"2:\t"		"sts current_buffer, 30"	"\n" \
		"\t"	"sts current_buffer + 1, 31"	"\n" \
		"\t"	"sts current_fragment, 16"	"\n" \
		/* Check for underrun. */ \
		"\t"	"lds 17, last_fragment"		"\n" \
		"\t"	"cp 16, 17"			"\n" \
		"\t"	"breq " underrun		"\n" \
		/* There is a next fragment; set it up. */ \
		activation \
		/* Set current length. */ \
		"\t"	"ldi 28, lo8(settings)"		"\n" \
		"\t"	"ldi 29, hi8(settings)"		"\n" \
		"\t"	"tst 16"			"\n" \
		"\t"	"breq 2f"			"\n" \
	"1:\t"		"adiw 28, %[settings_size]"	"\n" \
		"\t"	"dec 16"			"\n" \
		"\t"	"brne 1b"			"\n" \
	"2:\t"		"ldd 16, y + %[len]"		"\n" \
		"\t"	"sts current_len, 16"		"\n"

		next_fragment(
		/* Activation of all motors. */
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
		"\t"	"brne 2b"			"\n", "isr_underrun")

		"\t"	"rjmp isr_end"			"\n"
		// Underrun.
	"isr_underrun:\t"				"\n"
		"\t"	"ldi 16, 1"			"\n"
		"\t"	"sts step_state, 16"		"\n"
	"isr_end:"					"\n"
		"\t"	"pop 31"			"\n"
		"\t"	"pop 30"			"\n"
		"\t"	"pop 29"			"\n"
		"\t"	"pop 28"			"\n"
		"\t"	"pop 26"			"\n"
		"\t"	"pop 21"			"\n"
		"\t"	"pop 20"			"\n"
		"\t"	"pop 19"			"\n"
		"\t"	"pop 18"			"\n"
		"\t"	"pop 17"			"\n"
		"\t"	"pop 1"				"\n"
		"\t"	"pop 0"				"\n"
	"isr_end_audio:"				"\n"
		"\t"	"cli"				"\n"
		"\t"	"lds 16, step_state"		"\n"
		"\t"	"cpi 16, 1"			"\n"
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
		//"\t"	"rjmp isr_end_nonaked"		"\n"

	// Audio handling.  16 and 27 are pushed; 16 is audio.  At exit: rjmp isr_end_audio.
	"isr_audio:"					"\n"
		"\t"	"push 17"			"\n"
		"\t"	"push 28"			"\n"
		"\t"	"push 29"			"\n"
		"\t"	"push 30"			"\n"
		"\t"	"push 31"			"\n"

		"\t"	"lds 30, current_buffer"	"\n"
		"\t"	"lds 31, current_buffer + 1"	"\n"
		"\t"	"lds 17, move_phase"		"\n"
		"\t"	"tst 17"			"\n"
		"\t"	"breq 2f"			"\n"
	"1:"						"\n"
		"\t"	"adiw 30, %[fragment_size]"	"\n"
		"\t"	"dec 17"			"\n"
		"\t"	"brne 1b"			"\n"
	"2:"						"\n"
		"\t"	"lds 17, current_sample"	"\n"
		"\t"	"add 30, 17"			"\n"
		"\t"	"brcc 1f"			"\n"
		"\t"	"inc 31"			"\n"
	"1:"						"\n"
		"\t"	"lds 17, audio_bit"		"\n"
		"\t"	"ld 27, z"			"\n"
		"\t"	"and 27, 17"			"\n"
		"\t"	"breq 1f"			"\n"
		"\t"	"ldi 27, 1"			"\n"
	"1:\t"						"\n"
		"\t"	"eor 16, 27"			"\n"
		"\t"	"sbrs 16, 0"			"\n"
		"\t"	"rjmp isr_audio_end"		"\n"
		"\t"	"andi 16, ~1"			"\n"
		"\t"	"or 16, 27"			"\n"
		"\t"	"sts audio, 16"			"\n"

		// Set dir.
		"\t"	"ldi 28, lo8(audio_motor)"	"\n"
		"\t"	"ldi 29, hi8(audio_motor)"	"\n"
		"\t"	"ld 30, y"			"\n"
		"\t"	"ldd 31, y + 1"			"\n"
		"\t"	"ldd 16, z + %[dir_bitmask]"	"\n"
		"\t"	"ldd 28, z + %[dir_port]"	"\n"
		"\t"	"ldd 29, z + %[dir_port] + 1"	"\n"
		"\t"	"ld 17, y"			"\n"
		"\t"	"or 17, 16"			"\n"
		"\t"	"tst 27"			"\n"
		"\t"	"breq 1f"			"\n"
		"\t"	"com 16"			"\n"
		"\t"	"and 17, 16"			"\n"
	"1:\t"						"\n"
		"\t"	"st y, 17"			"\n"
		// Take step.
		"\t"	"ldd 16, z + %[step_bitmask]"	"\n"
		"\t"	"ldd 28, z + %[step_port]"	"\n"
		"\t"	"ldd 29, z + %[step_port] + 1"	"\n"
		"\t"	"ld 17, y"			"\n"
		"\t"	"or 17, 16"			"\n"
		"\t"	"com 16"			"\n"
		"\t"	"and 16, 17"			"\n"
		"\t"	"ldd 30, z + %[flags]"		"\n"
		"\t"	"sbrs 30, %[step_invert_bit]"	"\n"
		"\t"	"rjmp 1f"			"\n"
		// Step is inverted: swap 16 and 17.
		"\t"	"eor 16, 17"			"\n"
		"\t"	"eor 17, 16"			"\n"
		"\t"	"eor 16, 17"			"\n"
	"1:\t"						"\n"
		"\t"	"st y, 16"			"\n"	// 2	2
		"\t"	"nop"				"\n"	// 1	3
		"\t"	"nop"				"\n"	// 1	4
		// Wait 1 us (16 cycles).
		"\t"	"ldi 16, 4"			"\n"	// 1	5
	"1:\t"						"\n"
		"\t"	"dec 16"			"\n"	// n	5+n
		"\t"	"brne 1b"			"\n"	// 2n-1	4+3n == 16 => n = 4
		"\t"	"st y, 17"			"\n"

	"isr_audio_end:\t"				"\n"
		"\t"	"lds 16, audio_bit"		"\n"
		"\t"	"lsl 16"			"\n"
		"\t"	"brne 4f"			"\n"
		// Byte is done.
		"\t"	"lds 16, current_sample"	"\n"
		"\t"	"inc 16"			"\n"
		"\t"	"lds 17, current_len"		"\n"
		"\t"	"cp 16, 17"			"\n"
		"\t"	"brne 3f"			"\n"
		// Motor is done.
		"\t"	"lds 16, move_phase"		"\n"
		"\t"	"inc 16"			"\n"
		"\t"	"lds 17, active_motors"		"\n"
		"\t"	"cp 16, 17"			"\n"
		"\t"	"breq 1f"			"\n"
		// Increment current_buffer; continue.
		"\t"	"sts move_phase, 16"		"\n"
		"\t"	"rjmp 8f"			"\n"
	"1:\t"						"\n"
		// Fragment is done.
		"\t"	"ldi 17, 0"			"\n"
		"\t"	"sts move_phase, 17"		"\n"
		next_fragment("", "isr_audio_underrun")
	"8:\t"						"\n"
		"\t"	"ldi 16, 0"			"\n"
	"3:\t"						"\n"
		"\t"	"sts current_sample, 16"	"\n"
	"7:\t"						"\n"
		"\t"	"ldi 16, 1"			"\n"
	"4:\t"						"\n"
		"\t"	"sts audio_bit, 16"		"\n"
	"5:\t"						"\n"
		"\t"	"pop 31"			"\n"
		"\t"	"pop 30"			"\n"
		"\t"	"pop 29"			"\n"
		"\t"	"pop 28"			"\n"
		"\t"	"pop 17"			"\n"
		"\t"	"rjmp isr_end_audio"		"\n"
	"isr_audio_underrun:"				"\n"
		"\t"	"ldi 16, 1"			"\n"
		"\t"	"sts step_state, 16"		"\n"
		"\t"	"rjmp 5b"			"\n"

	"isr_end_nonaked:"				"\n"
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
			[fragment_size] "I" (BYTES_PER_FRAGMENT),
			[num_fragments] "M" (1 << FRAGMENTS_PER_MOTOR_BITS),
			[buffer_size] "" (BYTES_PER_FRAGMENT * NUM_MOTORS),
			[motor_size] "" (sizeof(Motor)),
			[settings_size] "I" (sizeof(Settings)),
			[len] "I" (offsetof(Settings, len)),
			[timsk] "M" (_SFR_MEM_ADDR(TIMSK1)),
			[timskval] "M" (1 << OCIE1A)
		);
}
#else
ISR(TIMER1_COMPA_vect) {
	SLOW_ISR();
}
#endif
// }}}

// Timekeeping. {{{
ISR(TIMER0_OVF_vect) { // {{{
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
	return ((h | l) << 1) / 125;
} // }}}

static inline uint16_t seconds() { // {{{
	return avr_seconds;
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
	*pin[pin_no].avr_mode &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_output &= ~pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_UNSET);
	pindebug("unset pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline void SET(uint8_t pin_no) { // {{{
	if ((pin[pin_no].state & 0x3) == CTRL_SET)
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
	uint16_t interval = now - avr_outputs_last;
	if (interval == 0)
		return;
	avr_outputs_last = now;
	for (uint8_t p = 0; p < NUM_DIGITAL_PINS; ++p) {
		if ((pin[p].state & 0x3) != CTRL_SET)
			continue;
		if (pin[p].avr_on)
			pin[p].avr_target -= interval;
		pin[p].avr_target += (interval * (int32_t(pin[p].duty) + 1)) >> 8;
		if (pin[p].avr_target < 0) {
			*pin[p].avr_output &= ~pin[p].avr_bitmask;
			pin[p].avr_on = false;
		}
		else {
			*pin[p].avr_output |= pin[p].avr_bitmask;
			pin[p].avr_on = true;
		}
	}
} // }}}
// }}}
#endif
#endif

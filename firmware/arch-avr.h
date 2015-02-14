// vim: set foldmethod=marker :
// This is not an include quard.  This file is included twice; the first time the first part of the file is used, the second time the second part. {{{
#ifndef _ARCH_AVR_H
#define _ARCH_AVR_H

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

#define arch_cli cli
#define arch_sei sei

inline void SET_OUTPUT(uint8_t pin_no);
inline void SET_INPUT(uint8_t pin_no);
inline void UNSET(uint8_t pin_no);
inline void SET(uint8_t pin_no);
inline void RESET(uint8_t pin_no);
inline bool GET(uint8_t pin_no);

// Everything before this line is used at the start of firmware.h; everything after it at the end.
#else
EXTERN Pin_t pin[NUM_DIGITAL_PINS];
// }}}

#ifdef __cplusplus
// Defined by arduino: NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS

// Serial communication. {{{
static inline void debug_add(int i);
static inline void debug_dump();
#define SERIAL_BUFFER_SIZE (COMMAND_BUFFER_SIZE + 10)
EXTERN volatile bool serial_overflow;
EXTERN volatile uint8_t which_serial;
EXTERN volatile uint16_t serial_buffer_head;
EXTERN volatile uint16_t serial_buffer_tail;
EXTERN volatile uint8_t serial_buffer[SERIAL_BUFFER_SIZE];

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

inline static void clear_overflow() {
	command_end = 0;
	serial_buffer_head = 0;
	serial_buffer_tail = 0;
	serial_overflow = false;
	debug("serial flushed after overflow");
	debug_dump();
	serial_write(CMD_NACK);
}

static inline uint16_t serial_available() {
	cli();
	uint16_t ret = (SERIAL_BUFFER_SIZE + serial_buffer_head - serial_buffer_tail) % SERIAL_BUFFER_SIZE;
	sei();
	return ret;
}

static inline uint8_t serial_read() {
	cli();
	uint8_t ret = serial_buffer[serial_buffer_tail];
	//debug("%x", ret);
	serial_buffer_tail = (serial_buffer_tail + 1) % SERIAL_BUFFER_SIZE;
	sei();
	return ret;
}

static inline void serial_flush() {
	if (which_serial == 0)
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
static inline void handle_serial_input(uint8_t which, uint8_t data, uint8_t status) {
	if (which_serial != which) {
		if (status != 0 || data != CMD_ID)
			return;
		which_serial = which;
#ifndef NO_SERIAL1
		// Disable other port so the pins can be used.
		if (which == 0)
			UCSR1B = 0;
		else
			UCSR0B = 0;
#endif
	}
	if (serial_overflow)
		return;
	uint16_t next = (serial_buffer_head + 1) % SERIAL_BUFFER_SIZE;
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
	handle_serial_input(0, UDR0, status & ((1 << FE0) | (1 << DOR0)));
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
#ifndef NO_DEBUG
#define AVR_NUM_DEBUG 20
EXTERN volatile int avr_debug[AVR_NUM_DEBUG];
EXTERN volatile int avr_debug_ptr;
static inline void debug_add(int i) {
	avr_debug[avr_debug_ptr] = i;
	avr_debug_ptr = (avr_debug_ptr + 1) % AVR_NUM_DEBUG;
}

static inline void debug_dump() {
	debug("Debug dump (most recent last):");
	for (int i = 0; i < AVR_NUM_DEBUG; ++i)
		debug("%x", avr_debug[(avr_debug_ptr + i) % AVR_NUM_DEBUG]);
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
	serial_write((uint8_t)0);
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
EXTERN uint8_t mcusr;
EXTERN volatile uint32_t avr_time_h, avr_seconds_h, avr_seconds;

static inline void arch_setup_start() {
	cli();
	mcusr = MCUSR;
	avr_time_h = 0;
	avr_seconds_h = 0;
	avr_seconds = 0;
	MCUSR = 0;
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
	TIMSK0 = 1 << TOIE0;
	// Setup ADC.
	ADCSRA = AVR_ADCSRA_BASE;
	// Enable interrupts.
	sei();
	// Initialize printer id from EEPROM.
	for (uint8_t i = 0; i < 16; ++i)
		printerid[i] = EEPROM.read(i);
	// Make it a UUID (version 4).
	printerid[7] &= 0x0f;
	printerid[7] |= 0x40;
	printerid[9] &= 0x3f;
	printerid[9] |= 0x80;
	// printerid[16:20] will be filled by CMD_BEGIN.  Initialize it to 0.
	for (uint8_t i = 0; i < 4; ++i)
		printerid[16 + i] = 0;
	// Fill magic.
	printerid[20] = 0xe1;
	printerid[21] = 0xd5;
	printerid[22] = 0xe6;
	printerid[23] = 0xcb;
}

static inline void arch_setup_end() {
	debug("Startup.  MCUSR: %x", mcusr);
}

static inline void set_speed(uint16_t count) {
	if (homers == 0) {
		for (uint8_t m = 0; m < active_motors; ++m) {
			motor[m].next_steps = 0;
			motor[m].next_next_steps = 0;
		}
	}
	stopped = (count == 0);
	underrun = stopped;
	if (!stopped) {
		// Set TOP.
		OCR1AH = (count >> 7) & 0xff;
		OCR1AL = (count << 1) & 0xff;
		// Clear counter.
		TCNT1H = 0;
		TCNT1L = 0;
		// Clear and enable interrupt.
		TIFR1 = 1 << OCF1A;
		TIMSK1 = 1 << OCIE1A;
	}
}
// }}}

// Timekeeping. {{{
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

#ifdef DEFINE_VARIABLES
ISR(TIMER0_OVF_vect) {
	uint32_t top = uint32_t(125) << 16;
	avr_time_h += 0x100;
	avr_time_h %= top;	// Wrap at the right number.
	if (((avr_time_h - avr_seconds_h + top) % top) >= 62500) {
		avr_seconds_h += 62500;
		avr_seconds_h %= top;	// Wrap at the right number.
		avr_seconds += 1;
	}
}

ISR(TIMER1_COMPA_vect) {
	do_steps();
}
#endif
// }}}

// Pin control. {{{
inline void SET_OUTPUT(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	if ((pin[pin_no].state & 0x3) == CTRL_SET || (pin[pin_no].state & 0x3) == CTRL_RESET)
		return;
	uint8_t bit = pgm_read_word(digital_pin_to_bit_mask_PGM + pin_no);
	uint8_t port = pgm_read_word(digital_pin_to_port_PGM + pin_no);
	*reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_output_PGM + port)) &= ~bit;
	*reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_mode_PGM + port)) |= bit;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_RESET);
}

inline void SET_INPUT(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	uint8_t bit = pgm_read_word(digital_pin_to_bit_mask_PGM + pin_no);
	uint8_t port = pgm_read_word(digital_pin_to_port_PGM + pin_no);
	*reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_mode_PGM + port)) &= ~bit;
	*reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_output_PGM + port)) |= bit;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_INPUT);
}

inline void UNSET(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	uint8_t bit = pgm_read_word(digital_pin_to_bit_mask_PGM + pin_no);
	uint8_t port = pgm_read_word(digital_pin_to_port_PGM + pin_no);
	*reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_mode_PGM + port)) &= ~bit;
	*reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_output_PGM + port)) &= ~bit;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_UNSET);
}

inline void SET(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	if ((pin[pin_no].state & 0x3) == CTRL_SET)
		return;
	uint8_t bit = pgm_read_word(digital_pin_to_bit_mask_PGM + pin_no);
	uint8_t port = pgm_read_word(digital_pin_to_port_PGM + pin_no);
	*reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_output_PGM + port)) |= bit;
	*reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_mode_PGM + port)) |= bit;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_SET);
}

inline void RESET(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	if ((pin[pin_no].state & 0x3) != CTRL_SET)
		return;
	uint8_t bit = pgm_read_word(digital_pin_to_bit_mask_PGM + pin_no);
	uint8_t port = pgm_read_word(digital_pin_to_port_PGM + pin_no);
	*reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_output_PGM + port)) &= ~bit;
	*reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_mode_PGM + port)) |= bit;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_RESET);
}

inline bool GET(uint8_t pin_no) {
	uint8_t bit = pgm_read_word(digital_pin_to_bit_mask_PGM + pin_no);
	uint8_t port = pgm_read_word(digital_pin_to_port_PGM + pin_no);
	return *reinterpret_cast <volatile uint8_t *>(pgm_read_word(port_to_input_PGM + port)) & bit;
}
// }}}

/* Memory handling (disabled) {{{
EXTERN char storage[DYNAMIC_MEM_SIZE];
EXTERN uint16_t mem_used;
struct Memrecord {
	uint16_t size;
	void **target;
	inline Memrecord *next() { return reinterpret_cast <Memrecord *>(&(reinterpret_cast <char *>(this))[sizeof(Memrecord) + size]); }
};

//#define DEBUG_ALLOC

#ifdef DEBUG_ALLOC
#define mem_alloc(s, t, d) do { debug("mem alloc " #d " at %x size %x used %x", unsigned(&storage) + mem_used, s, mem_used); _mem_alloc((s), reinterpret_cast <void **>(t)); } while (false)
#else
#define mem_alloc(s, t, d) _mem_alloc((s), reinterpret_cast <void **>(t))
#endif
static inline void _mem_alloc(uint32_t size, void **target) {
	if (size + sizeof(Memrecord) > sizeof(storage) - mem_used) {
		debug("Out of memory");
		*target = NULL;
		return;
	}
	Memrecord *record = reinterpret_cast <Memrecord *>(&storage[mem_used]);
	record->size = size;
	record->target = target;
	*record->target = &record[1];
	mem_used += sizeof(Memrecord) + size;
}

#define mem_retarget(t1, t2) _mem_retarget(reinterpret_cast <void **>(t1), reinterpret_cast <void **>(t2))
static inline void _mem_retarget(void **target, void **newtarget) {
	if (*target == NULL) {
#ifdef DEBUG_ALLOC
		debug("mem retarget NULL used %x", mem_used);
#endif
		*newtarget = NULL;
		return;
	}
	Memrecord *record = &reinterpret_cast <Memrecord *>(*target)[-1];
#ifdef DEBUG_ALLOC
	debug("mem retarget size %x used %x value %x record %x", record->size, mem_used, unsigned(*target), unsigned(record));
#endif
	*newtarget = *target;
	*target = NULL;
	record->target = newtarget;
}

static inline void _mem_dump() {
	uint16_t start = unsigned(&storage);
	debug("Memory dump.  Total size: %x, used %x, storage at %x", DYNAMIC_MEM_SIZE, mem_used, start);
	uint16_t addr = 0;
	while (addr < mem_used) {
		Memrecord *record = reinterpret_cast <Memrecord *>(&storage[addr]);
		if (unsigned(*record->target) - 4 == start + addr)
			debug("  Record at %x+%x=%x, size %x, pointer at %x", addr, start, start + addr, record->size, unsigned(record->target));
		else
			debug("  Record at %x+%x=%x, size %x, pointer at %x, pointing at %x+4", addr, start, start + addr, record->size, unsigned(record->target), unsigned(*record->target) - 4);
		addr += sizeof(Memrecord) + record->size;
	}
}

#define mem_free(t) _mem_free(reinterpret_cast <void **>(t))
static inline void _mem_free(void **target) {
	if (*target == NULL) {
#ifdef DEBUG_ALLOC
		debug("mem free NULL");
#endif
		return;
	}
	Memrecord *record = &reinterpret_cast <Memrecord *>(*target)[-1];
	*target = NULL;
	uint16_t oldsize = record->size;
	uint16_t start = reinterpret_cast <char *>(record) - storage + sizeof(Memrecord) + record->size;
#ifdef DEBUG_ALLOC
	debug("mem free size %x at %x, next %x, used %x", record->size, unsigned(record), start, mem_used);
	_mem_dump();
#endif
	Memrecord *current = reinterpret_cast <Memrecord *>(&storage[start]);
	while (start < mem_used) {
#ifdef DEBUG_ALLOC
		debug("moving %x for free from start %x", current->size, start);
#endif
		Memrecord *next = current->next();
		char *src = reinterpret_cast <char *>(current);
		char *dst = reinterpret_cast <char *>(record);
		uint16_t sz = current->size + sizeof(Memrecord);
		for (uint16_t i = 0; i < sz; ++i)
			dst[i] = src[i];
		*record->target = &record[1];
		// record is new location of moved part.
		// current is old location.
		// next is location of next part.
		for (Memrecord *m = reinterpret_cast <Memrecord *>(storage); unsigned(m) <= unsigned(record); m = m->next()) {
			//debug("check1 %x %x %x", unsigned(m->target), unsigned(current), unsigned(current) + record->size);
			if (unsigned(m->target) >= unsigned(current) && unsigned(m->target) < unsigned(current) + sizeof(Memrecord) + record->size) {
				//debug("hit!");
				m->target = reinterpret_cast <void **>(&reinterpret_cast <char *>(m->target)[-oldsize - sizeof(Memrecord)]);
			}
		}
		for (Memrecord *m = reinterpret_cast <Memrecord *>(next); unsigned(m) < unsigned(&storage[mem_used]); m = m->next()) {
			//debug("check2 %x %x %x", unsigned(m->target), unsigned(current), unsigned(current) + record->size);
			if (unsigned(m->target) >= unsigned(current) && unsigned(m->target) < unsigned(current) + sizeof(Memrecord) + record->size) {
				//debug("hit!");
				m->target = reinterpret_cast <void **>(&reinterpret_cast <char *>(m->target)[-oldsize - sizeof(Memrecord)]);
			}
		}
		record = reinterpret_cast <Memrecord *>(&reinterpret_cast <char *>(record)[sz]);
		current = next;
		start += sz;
	}
	mem_used -= sizeof(Memrecord) + oldsize;
#ifdef DEBUG_ALLOC
	_mem_dump();
#endif
}
// }}}*/
#endif
#endif

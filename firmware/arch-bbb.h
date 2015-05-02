// vim: set foldmethod=marker :
// This is not an include quard.  This file is included twice; the first time the first part of the file is used, the second time the second part. {{{
#ifndef _ARCH_BBB_H
#define _ARCH_BBB_H

#include <stdint.h>
#include <stdlib.h>

#define NUM_DIGITAL_PINS 5
#define NUM_ANALOG_INPUTS 7
#define ADCBITS 12
#define SERIAL_BUFFER_SIZE 10

#define cli() do {} while(0)
#define sei() do {} while(0)

#ifndef NO_DEBUG
static inline void debug(char const *fmt, ...);
#else
#define debug(...) do {} while (0)
#endif

#ifdef F
#undef F
#endif
#define F(x) (x)

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

inline static void debug_dump() {
}

inline static void debug(char const *fmt, ...) {
	(void)&fmt;
}

static inline void arch_write_current_pos(uint8_t offset) {
	for (uint8_t m = 0; m < active_motors; ++m)
		*reinterpret_cast <int32_t *>(&pending_packet[offset + 4 * m]) = motor[m].current_pos;
}

static inline void arch_record_sense(bool state) {
	for (int mi = 0; mi < active_motors; ++mi)
		motor[mi].sense_pos[state ? 1 : 0] = motor[mi].current_pos;
}

// Serial communication.  {{{
EXTERN volatile bool serial_overflow;
EXTERN volatile uint8_t which_serial;
EXTERN volatile uint16_t serial_buffer_head;
EXTERN volatile uint16_t serial_buffer_tail;
EXTERN volatile uint8_t serial_buffer[SERIAL_BUFFER_SIZE];

inline static void serial_write(uint8_t c) {
	(void)&c;
}

inline static void clear_overflow() {
	command_end = 0;
	debug("serial flushed after overflow");
	serial_write(CMD_NACK);
}

inline static bool serial_available() {
	return false;
}

inline static uint8_t serial_read() {
	return 0;
}

inline static void serial_flush() {
}
// }}}

// ADC. {{{
static inline void adc_start(uint8_t adcpin) {
	(void)&adcpin;
}

static inline bool adc_ready(uint8_t pin_) {
	(void)&pin_;
	return true;
}

static inline int16_t adc_get(uint8_t pin_) {
	(void)&pin_;
	return 0;
}
// }}}

// Watchdog and reset. {{{
static inline void watchdog_enable() {
}

static inline void watchdog_disable() {
}

static inline void watchdog_reset() {
}

static inline void reset() {
	abort();
}
// }}}

// Setup. {{{
static inline void arch_setup_start() {
	// TODO: call do_steps on interval.
}

static inline void arch_setup_end() {
}

static inline void set_speed(uint16_t count) {
	if (homers == 0) {
		for (uint8_t m = 0; m < active_motors; ++m) {
			motor[m].next_steps = 0;
			motor[m].next_next_steps = 0;
		}
	}
}
// }}}

// Timekeeping. {{{
static inline uint16_t millis() {
	return 0;
}

static inline uint16_t seconds() {
	return 0;
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

inline bool GET(uint8_t pin_no) {
	(void)&pin_no;
	return false;
}
// }}}

#endif

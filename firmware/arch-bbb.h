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

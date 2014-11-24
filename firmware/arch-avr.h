// This is not an include quard.  This file is included twice; the first time the first part of the file is used, the second time the second part.
#ifndef _ARCH_AVR_H
#define _ARCH_AVR_H
#include <Arduino.h>
#include <avr/wdt.h>
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

// Arduino is slow enough to not need explicit microsecond delays.
#define microdelay() do {} while(0)
// We don't care about using full cpu power on Arduino.
#define wait_for_event(x, t) do {} while(0)

#define RAW_SET_OUTPUT(pin_no) do { pinMode((pin_no), OUTPUT); } while (0)
#define RAW_SET_INPUT(pin_no) do { pinMode((pin_no), INPUT_PULLUP); } while (0)
#define RAW_SET_INPUT_NOPULLUP(pin_no) do { pinMode((pin_no), INPUT); } while (0)
#define RAW_SET(pin_no) do { digitalWrite((pin_no), HIGH); } while (0)
#define RAW_RESET(pin_no) do { digitalWrite((pin_no), LOW); } while (0)
#define RAW_GET(pin_no) (digitalRead(pin_no) == HIGH)

#define SET_OUTPUT(pin_no) do { if ((pin_no).valid()) { pinMode((pin_no).pin, OUTPUT); }} while (0)
#define SET_INPUT(pin_no) do { if ((pin_no).valid()) { pinMode((pin_no).pin, INPUT_PULLUP); }} while (0)
#define SET_INPUT_NOPULLUP(pin_no) do { if ((pin_no).valid()) { pinMode((pin_no).pin, INPUT); }} while (0)
#define SET(pin_no) do { if ((pin_no).valid()) { digitalWrite((pin_no).pin, (pin_no).inverted() ? LOW : HIGH); } } while (0)
#define RESET(pin_no) do { if ((pin_no).valid()) { digitalWrite((pin_no).pin, (pin_no).inverted() ? HIGH : LOW); } } while (0)
#define GET(pin_no, _default) ((pin_no).valid() ? digitalRead((pin_no).pin) == HIGH ? !(pin_no).inverted() : (pin_no).inverted() : _default)

// Everything before this line is used at the start of firmware.h; everything after it at the end.
#else

#ifdef __cplusplus
#ifdef USE_SERIAL1
#define Serial Serial1
#endif

// Defined by arduino: NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS

#ifndef NO_DEBUG
static inline void debug(char const *fmt, ...) {
#if DEBUG_BUFFER_LENGTH > 0
	buffered_debug_flush();
#endif
	va_list ap;
	va_start(ap, fmt);
	Serial.write(CMD_DEBUG);
	for (char const *p = fmt; *p; ++p) {
		if (*p == '%') {
			bool longvalue = false;
			while (true) {
				++p;
				switch (*p) {
				case 0: {
					Serial.write('%');
					--p;
					break;
				}
				case 'l': {
					longvalue = true;
					continue;
				}
				case '%': {
					Serial.write('%');
					break;
				}
				case 'd': {
					if (longvalue) {
						int32_t *arg = va_arg(ap, int32_t *);
						Serial.print(*arg, DEC);
					}
					else {
						int arg = va_arg(ap, int);
						Serial.print(arg, DEC);
					}
					break;
				}
				case 'x': {
					if (longvalue) {
						int32_t *arg = va_arg(ap, int32_t *);
						Serial.print(*arg, HEX);
					}
					else {
						int arg = va_arg(ap, int);
						Serial.print(arg, HEX);
					}
					break;
				}
				case 'f': {
					float *arg = va_arg(ap, float *);
					Serial.print(*arg, 5);
					break;
				}
				case 's': {
					char const *arg = va_arg(ap, char const *);
					Serial.print(arg);
					break;
				}
				case 'c': {
					char arg = va_arg(ap, int);
					Serial.write(arg);
					break;
				}
				default: {
					Serial.write('%');
					Serial.write(*p);
					break;
				}
				}
				break;
			}
		}
		else {
			Serial.write(*p);
		}
	}
	va_end(ap);
	Serial.write((uint8_t)0);
	Serial.flush();
}
#endif

EXTERN uint8_t adc_last_pin;
#define ADCBITS 10

#define fabs abs

static inline void adc_start(uint8_t adcpin) {
	// Mostly copied from /usr/share/arduino/hardware/arduino/cores/arduino/wiring_analog.c.
#if defined(__AVR_ATmega32U4__)
	uint8_t pin = analogPinToChannel(adcpin);
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#elif defined(ADCSRB) && defined(MUX5)
	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
	uint8_t pin = adcpin;
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#else
	uint8_t pin = adcpin;
#endif

#if defined(ADMUX)
	ADMUX = (DEFAULT << 6) | (pin & 0x7);
#endif
	// Start the conversion.
	ADCSRA |= 1 << ADSC;
	adc_last_pin = ~0;
}

static inline bool adc_ready(uint8_t pin) {
	if (bit_is_set(ADCSRA, ADSC))
		return false;
	if (pin != adc_last_pin) {
		adc_last_pin = pin;
		ADCSRA |= 1 << ADSC;
		return false;
	}
	return true;
}

static inline int32_t adc_get(uint8_t pin) {
	int32_t low = uint8_t(ADCL);
	int32_t high = uint8_t(ADCH);
	int32_t ret = (high << 8) | low;
	//debug("adc: %ld", F(ret));
	return ret;
}

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

EXTERN uint8_t mcusr;
EXTERN uint16_t mem_used;
EXTERN uint32_t time_h;

static inline void arch_setup_start() {
	mcusr = MCUSR;
	mem_used = 0;
	time_h = 0;
	MCUSR = 0;
	// Setup timer1 for microsecond counting.
	TCCR1A = 0;
	TCCR1B = 2;	// Clock/8, in other words with 16MHz clock, 2MHz counting; 2 counts/us.
	// Disable all outputs.
	for (uint8_t i = 0; i < NUM_DIGITAL_PINS; ++i)
		RAW_SET_INPUT_NOPULLUP(i);
	// Initialize printer id from EEPROM.
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		printerid[i] = EEPROM.read(i);
	// Make it a UUID (version 4).
	printerid[7] &= 0x0f;
	printerid[7] |= 0x40;
	printerid[9] &= 0x3f;
	printerid[9] |= 0x80;
}

static inline bool arch_run() {
	if (TIFR1 & (1 << TOV1)) {
		TIFR1 = 1 << TOV1;
		time_h += 1;
		return true;
	}
	return false;
}

static inline void arch_setup_end() {
	debug("Startup.  MCUSR: %x", mcusr);
}

static inline uint32_t utime() {
	uint32_t l = uint8_t(TCNT1L);
	uint32_t h = uint8_t(TCNT1H);
	// If a  carry happened just now, get new values.
	if (arch_run()) {
		l = uint8_t(TCNT1L);
		h = uint8_t(TCNT1H);
	}
	// Don't use 16,8,0, because we have 2 counts/us, not 1.
	return (time_h << 15) | (h << 7) | (l >> 1);
}

static inline void get_current_times(uint32_t *current_time, uint32_t *longtime) {
	*current_time = utime();
	*longtime = millis();
}

// Memory handling
EXTERN char storage[DYNAMIC_MEM_SIZE];
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
#endif
#endif

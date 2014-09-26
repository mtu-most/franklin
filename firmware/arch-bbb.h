// This is not an include quard.  This file is included twice; the first time the first part of the file is used, the second time the second part.
#ifndef _ARCH_AVR_H
#define _ARCH_AVR_H
#include <stdint.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
//#include <cstdio>
#include <sys/time.h>

#define SET_OUTPUT(pin_no) do { if ((pin_no).valid ()) {}} while (0)
#define SET_INPUT(pin_no) do { if ((pin_no).valid ()) {}} while (0)
#define SET_INPUT_NOPULLUP(pin_no) do { if ((pin_no).valid ()) {}} while (0)
#define SET(pin_no) do { if ((pin_no).valid ()) {} } while (0)
#define RESET(pin_no) do { if ((pin_no).valid ()) {} } while (0)
#define GET(pin_no, _default) ((pin_no).valid () ? false ? !(pin_no).inverted () : (pin_no).inverted () : _default)

#define A0 66
#define NUM_ANALOG_INPUTS 7
#define NUM_DIGITAL_PINS A0 + NUM_ANALOG_INPUTS	// Not true, but otherwise the last pins will be displayed as analog.  Now there are some unusable analog pins displayed.

#ifdef WATCHDOG
//#warning Watchdog is not supported on BeagleBone; ignoring request to use it.
#undef WATCHDOG
#endif

extern std::string _adc[NUM_ANALOG_INPUTS];

static inline void arch_setup_start() {
	for (int i = 0; i < A0; ++i) {
		std::ofstream f("/sys/class/gpio/export");
		f << i << '\n';
		f.close();
	}
	// TODO: find correct filename.
	std::ofstream f("/sys/devices/bone_capemgr.*/slots");
	f << "cape-bone-iio\n";
	f.close();
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i) {
		std::ostringstream s;
		// TODO: find out where these things are, and if it varies, implement a search.
		s << "/sys/devices/ocp.2/helper.14/AIN" << i;
		_adc[i] = s.str();
	}
}

static inline void arch_setup_end() {
}

static inline void adc_start(uint8_t pin) {
}

static inline bool adc_ready(uint8_t pin) {
	return true;
}

static inline int16_t adc_get(uint8_t pin) {
	std::ifstream f(_adc[pin].c_str());
	int16_t value;
	f >> value;
	return value;
}

static inline void reset() {
	// This shouldn't happen.  But if it does, die.
	exit(0);
}

static inline void watchdog_reset() {}
static inline void watchdog_enable() {}
static inline void watchdog_disable() {}

class FakeSerial {
public:
	void begin(int baud) {}
	void write(char c) { std::cout.put(c); }
	char read() { return std::cin.get(); }
	int readBytes (char *buffer, int len) { return 0; }
	void flush() {}
	int available() { return 0; }
};

#define debug(...) do { buffered_debug_flush(); fprintf(stderr, __VA_ARGS__); } while (0)
#define F(x) (x)

static inline unsigned long micros() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000000 + tv.tv_usec;
}
static inline unsigned long millis() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
#define delayMicroseconds usleep

#define min(a, b) (a < b ? a : b)
#define max(a, b) (a > b ? a : b)

static inline uint8_t read_eeprom(uint16_t address) {
	return 0;
}

static inline void write_eeprom(uint16_t address, uint8_t data) {
}

#define E2END 0xffffffff

#else

EXTERN FakeSerial Serial;
EXTERN std::string _adc[NUM_ANALOG_INPUTS];

// Memory handling
EXTERN uint16_t mem_used;
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
void _mem_alloc(uint16_t size, void **target);

#define mem_retarget(t1, t2) _mem_retarget(reinterpret_cast <void **>(t1), reinterpret_cast <void **>(t2))
void _mem_retarget(void **target, void **newtarget);

void _mem_dump();

#define mem_free(t) _mem_free(reinterpret_cast <void **>(t))
void _mem_free(void **target);

#endif

#include <stdint.h>
#include <stdlib.h>
#include <iostream>
#include <cstdio>

#define SET_OUTPUT(pin_no) do { if (!(pin_no).invalid ()) {}} while (0)
#define SET_INPUT(pin_no) do { if (!(pin_no).invalid ()) {}} while (0)
#define SET_INPUT_NOPULLUP(pin_no) do { if (!(pin_no).invalid ()) {}} while (0)
#define SET(pin_no) do { if (!(pin_no).invalid ()) {} } while (0)
#define RESET(pin_no) do { if (!(pin_no).invalid ()) {} } while (0)
#define GET(pin_no, _default) (!(pin_no).invalid () ? false ? !(pin_no).inverted () : (pin_no).inverted () : _default)

#undef SERIAL_BUFFERSIZE
#define SERIAL_BUFFERSIZE 0
#define NUM_DIGITAL_PINS 10
#define NUM_ANALOG_INPUTS 0

#ifdef WATCHDOG
#undef WATCHDOG
#endif

static inline void adc_start(uint8_t pin) {
}

static inline bool adc_ready(uint8_t pin) {
	return false;
}

static inline int16_t adc_get(uint8_t pin) {
	return 0;
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

static FakeSerial Serial;

#define debug(...) fprintf(stderr, __VA_ARGS__)
#define F(x) (x)

static inline long long micros() { return 0; }
static inline long long millis() { return 0; }

#define A0 16
#define NUM_ANALOG_INPUTS 0

#define min(a, b) (a < b ? a : b)
#define max(a, b) (a > b ? a : b)

static inline void arch_setup_start() {
}

static inline void arch_setup_end(int address) {
	(void)&address;
}

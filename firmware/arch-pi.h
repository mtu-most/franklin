#include <stdint.h>
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

static inline uint16_t adc_get(uint8_t pin) {
	return 0;
}

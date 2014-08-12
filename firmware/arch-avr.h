// This is not an include quard.  This file is included twice; the first time the first part of the file is used, the second time the second part.
#ifndef _ARCH_AVR_H
#define _ARCH_AVR_H
#include <Arduino.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#define SET_OUTPUT(pin_no) do { if ((pin_no).valid ()) { pinMode ((pin_no).pin, OUTPUT); }} while (0)
#define SET_INPUT(pin_no) do { if ((pin_no).valid ()) { pinMode ((pin_no).pin, INPUT_PULLUP); }} while (0)
#define SET_INPUT_NOPULLUP(pin_no) do { if ((pin_no).valid ()) { pinMode ((pin_no).pin, INPUT); }} while (0)
#define SET(pin_no) do { if ((pin_no).valid ()) { digitalWrite ((pin_no).pin, (pin_no).inverted () ? LOW : HIGH); } } while (0)
#define RESET(pin_no) do { if ((pin_no).valid ()) { digitalWrite ((pin_no).pin, (pin_no).inverted () ? HIGH : LOW); } } while (0)
#define GET(pin_no, _default) ((pin_no).valid () ? digitalRead ((pin_no).pin) == HIGH ? !(pin_no).inverted () : (pin_no).inverted () : _default)

#if SERIAL_BUFFERSIZE > 0
// This is really ugly, but it's how it's done in the arduino sources, and no other variables are defined.
#if defined(UBRR3H)
#define NUMSERIALS 4
#define SETUP_SERIALS do { serialport[0] = &Serial; serialport[1] = &Serial1; serialport[2] = &Serial2; serialport[3] = &Serial3; } while (0)
#elif defined(UBRR2H)
#define NUMSERIALS 3
#define SETUP_SERIALS do { serialport[0] = &Serial; serialport[1] = &Serial1; serialport[2] = &Serial2; } while (0)
#elif defined(UBRR1H)
#define NUMSERIALS 2
#define SETUP_SERIALS do { serialport[0] = &Serial; serialport[1] = &Serial1; } while (0)
#else
#define NUMSERIALS 1
#define SETUP_SERIALS do { serialport[0] = &Serial; } while (0)
#endif
#endif

// Everything before this line is used at the start of firmware.h; everything after it at the end.
#else

// Defined by arduino: NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS

EXTERN uint8_t adc_last_pin;

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

#ifdef __cplusplus
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

static inline int16_t adc_get(uint8_t pin) {
	int16_t low = uint8_t(ADCL);
	int16_t high = uint8_t(ADCH);
	return (high << 8) | low;
}

static inline void watchdog_enable() {
#ifdef WATCHDOG
	wdt_reset();
	wdt_enable(WDTO_120MS);
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

static inline void debug (char const *fmt, ...) {
	buffered_debug_flush();
	va_list ap;
	va_start (ap, fmt);
	Serial.write (CMD_DEBUG);
	for (char const *p = fmt; *p; ++p) {
		if (*p == '%') {
			bool longvalue = false;
			while (true) {
				++p;
				switch (*p) {
				case 0: {
					Serial.write ('%');
					--p;
					break;
				}
				case 'l': {
					longvalue = true;
					continue;
				}
				case '%': {
					Serial.write ('%');
					break;
				}
				case 'd': {
					if (longvalue) {
						int32_t *arg = va_arg (ap, int32_t *);
						Serial.print (*arg, DEC);
					}
					else {
						int arg = va_arg (ap, int);
						Serial.print (arg, DEC);
					}
					break;
				}
				case 'x': {
					if (longvalue) {
						int32_t *arg = va_arg (ap, int32_t *);
						Serial.print (*arg, HEX);
					}
					else {
						int arg = va_arg (ap, int);
						Serial.print (arg, HEX);
					}
					break;
				}
				case 'f': {
					float *arg = va_arg (ap, float *);
					Serial.print (*arg, 5);
					break;
				}
				case 's': {
					char const *arg = va_arg (ap, char const *);
					Serial.print (arg);
					break;
				}
				case 'c': {
					char arg = va_arg (ap, int);
					Serial.write (arg);
					break;
				}
				default: {
					Serial.write ('%');
					Serial.write (*p);
					break;
				}
				}
				break;
			}
		}
		else {
			Serial.write (*p);
		}
	}
	va_end (ap);
	Serial.write ((uint8_t)0);
	Serial.flush ();
}
#ifdef F
#undef F
#endif
#define F(x) &(x)

static uint8_t mcusr;

static inline void arch_setup_start() {
	mcusr = MCUSR;
	MCUSR = 0;
}

static inline void arch_setup_end() {
	debug ("Startup.  MCUSR: %x", mcusr);
}

static inline uint8_t read_eeprom(uint16_t address) {
	return EEPROM.read(address);
}

static inline void write_eeprom(uint16_t address, uint8_t data) {
	EEPROM.write(address, data);
}

#endif
#endif

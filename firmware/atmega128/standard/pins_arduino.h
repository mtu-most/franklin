#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

/* Analog pins:
   F0-F7	A0-7, D45-62
   Digital pins:
   A0-7		D16-23
   B0-7		D8-15
   C0-7		D24-31
   D0-7		D32-39
   E0-7		D0-7
   G0-4		D40-44
   Serial port 0:
   E0(Rx)+E1(Tx)
   */

#define NUM_DIGITAL_PINS            63
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS : -1)

#define digitalPinHasPWM(p)         (false)	// There are pwms, but I don't care about them.

static const uint8_t SS   = 8;	// B0
static const uint8_t MOSI = 10;	// B2
static const uint8_t MISO = 11;	// B3
static const uint8_t SCK  = 9;	// B1

static const uint8_t SDA = 33;	// D1
static const uint8_t SCL = 32;	// D0

static const uint8_t A0 = NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS + 0;
static const uint8_t A1 = NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS + 1;
static const uint8_t A2 = NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS + 2;
static const uint8_t A3 = NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS + 3;
static const uint8_t A4 = NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS + 4;
static const uint8_t A5 = NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS + 5;
static const uint8_t A6 = NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS + 6;
static const uint8_t A7 = NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS + 7;

#ifdef ARDUINO_MAIN

#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] =
{
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
	(uint16_t) &DDRG,
};

const uint16_t PROGMEM port_to_output_PGM[] =
{
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
	(uint16_t) &PORTG,
};

const uint16_t PROGMEM port_to_input_PGM[] =
{
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
	(uint16_t) &PING,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] =
{
	PE, PE, PE, PE, PE, PE, PE, PE, 
	PB, PB, PB, PB, PB, PB, PB, PB, 
	PA, PA, PA, PA, PA, PA, PA, PA, 
	PC, PC, PC, PC, PC, PC, PC, PC, 
	PD, PD, PD, PD, PD, PD, PD, PD, 
	PG, PG, PG, PG, PG,
	PF, PF, PF, PF, PF, PF, PF, PF, 
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
	_BV(0), _BV(1), _BV(2), _BV(3), _BV(4), _BV(5), _BV(6), _BV(7),
	_BV(0), _BV(1), _BV(2), _BV(3), _BV(4), _BV(5), _BV(6), _BV(7),
	_BV(0), _BV(1), _BV(2), _BV(3), _BV(4), _BV(5), _BV(6), _BV(7),
	_BV(0), _BV(1), _BV(2), _BV(3), _BV(4), _BV(5), _BV(6), _BV(7),
	_BV(0), _BV(1), _BV(2), _BV(3), _BV(4), _BV(5), _BV(6), _BV(7),
	_BV(0), _BV(1), _BV(2), _BV(3), _BV(4),
	_BV(0), _BV(1), _BV(2), _BV(3), _BV(4), _BV(5), _BV(6), _BV(7),
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
	NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, 
	NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, TIMER0A,      TIMER1A,      TIMER1B,      TIMER2A, 
	NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, 
	NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, 
	NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, 
	NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER,
	NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, NOT_ON_TIMER, 
};

#endif // ARDUINO_MAIN

#endif // Pins_Arduino_h
// vim:ai:cin:sts=2 sw=2 ft=cpp

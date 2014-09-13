#include "firmware.h"

#ifdef HAVE_GPIOS
void Gpio::load(uint8_t self, int16_t &addr, bool eeprom)
{
	pin.read(read_16(addr, eeprom));
	state = read_8(addr, eeprom);
#ifdef HAVE_TEMPS
	uint8_t oldmaster = master;
	master = read_8(addr, eeprom);
	value = read_float(addr, eeprom);
#else
	read_8(addr, eeprom);
	read_float(addr, eeprom);
#endif
	// State:  0: off, 1: on, 2: pullup input, 3: disabled
	switch (state) {
	case 0:
		SET_OUTPUT(pin);
		RESET(pin);
		break;
	case 1:
		SET_OUTPUT(pin);
		SET(pin);
		break;
	case 2:
		SET_INPUT(pin);
		break;
	case 3:
		SET_INPUT_NOPULLUP(pin);
		break;
	}
#ifdef HAVE_TEMPS
	if (master < num_temps) {
		adcvalue = temps[master].toadc(value);
		adc_phase = 1;
	}
	if (oldmaster != master)
	{
		// Disable old links.
		if (oldmaster < num_temps)
		{
			if (prev < num_gpios)
				gpios[prev].next = next;
			else
				temps[oldmaster].following_gpios = next;
			if (next < num_gpios)
				gpios[next].prev = prev;
		}
		// Set new links.
		if (master < num_temps)
		{
			prev = ~0;
			next = temps[master].following_gpios;
			temps[master].following_gpios = self;
			if (next < num_gpios)
				gpios[next].prev = self;
			// Also, set pin to output.
			SET_OUTPUT(pin);
		}
		else
		{
			prev = ~0;
			next = ~0;
		}
	}
#endif
}

void Gpio::save(int16_t &addr, bool eeprom)
{
	write_16(addr, pin.write(), eeprom);
	write_8(addr, state, eeprom);
#ifdef HAVE_TEMPS
	write_8(addr, master, eeprom);
	write_float(addr, value, eeprom);
#else
	write_8(addr, 0, eeprom);
	write_float(addr, NAN, eeprom);
#endif
}

int16_t Gpio::savesize0() {
	return 1 * 2 + 2 * 1 + sizeof(float) * 1;
}

void Gpio::init() {
	pin.flags = 0;
	pin.read(0);
	state = 0;
#ifdef HAVE_TEMPS
	value = NAN;
	master = ~0;
	prev = ~0;
	next = ~0;
#endif
}

void Gpio::free() {
#ifdef HAVE_TEMPS
	if (master < num_temps) {
		if (prev < num_gpios)
			gpios[prev].next = next;
		else
			temps[master].following_gpios = next;
		if (next < num_gpios)
			gpios[next].prev = prev;
	}
#endif
}

void Gpio::copy(Gpio &dst) {
	dst.pin.flags = 0;
	dst.pin.read(pin.write());
	dst.state = state;
#ifdef HAVE_TEMPS
	dst.value = value;
	dst.next = next;
	dst.prev = prev;
	dst.master = master;
#endif
}
#endif

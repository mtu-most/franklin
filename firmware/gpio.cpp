#include "firmware.h"

#ifdef HAVE_GPIOS
void Gpio::load(int16_t &addr, bool eeprom)
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
			if (prev)
				prev->next = next;
			else
				temps[oldmaster].gpios = next;
			if (next)
				next->prev = prev;
		}
		// Set new links.
		if (master < num_temps)
		{
			prev = NULL;
			next = temps[master].gpios;
			temps[master].gpios = this;
			if (next)
				next->prev = this;
			// Also, set pin to output.
			SET_OUTPUT(pin);
		}
		else
		{
			prev = NULL;
			next = NULL;
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

int16_t Gpio::size0() {
	return 1 * 2 + 2 * 1 + sizeof(float) * 1;
}

void Gpio::init() {
	pin.flags = 0;
	pin.read(0);
	state = 0;
	value = NAN;
	master = ~0;
	prev = NULL;
	next = NULL;
}

void Gpio::free() {
	if (master < num_temps) {
		if (prev)
			prev->next = next;
		else
			temps[master].gpios = next;
		if (next)
			next->prev = prev;
	}
}

void Gpio::copy(Gpio &dst) {
	dst.pin.flags = 0;
	dst.pin.read(pin.write());
	dst.state = state;
	dst.value = value;
	if (master < num_temps) {
		dst.next = next;
		if (next)
			next->prev = &dst;
		dst.prev = prev;
		if (prev)
			prev->next = &dst;
		else
			temps[master].gpios = &dst;
	}
	else
		dst.master = ~0;
}
#endif

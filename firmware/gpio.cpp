#include "firmware.h"

#ifdef HAVE_GPIOS
void Gpio::load (int16_t &addr, bool eeprom)
{
	pin.read (read_16 (addr, eeprom));
	state = read_8 (addr, eeprom);
#ifndef LOWMEM
	uint8_t oldmaster = master;
	master = read_8 (addr, eeprom);
	value = read_float (addr, eeprom);
#else
	read_8 (addr, eeprom);
	read_float (addr, eeprom);
#endif
	// State:  0: off, 1: on, 2: pullup input, 3: disabled
	switch (state) {
	case 0:
		SET_OUTPUT (pin);
		RESET (pin);
		break;
	case 1:
		SET_OUTPUT (pin);
		SET (pin);
		break;
	case 2:
		SET_INPUT (pin);
		break;
	case 3:
		SET_INPUT_NOPULLUP (pin);
		break;
	}
#ifndef LOWMEM
	if (master >= MAXOBJECT || !temps[master])
		master = 0;
	if (temps[master])
		adcvalue = temps[master]->toadc (value);
	if (oldmaster != master)
	{
		// Disable old links.
		if (oldmaster != 0)
		{
			if (prev)
				prev->next = next;
			else
				temps[oldmaster]->gpios = next;
			if (next)
				next->prev = prev;
		}
		// Set new links.
		if (master != 0)
		{
			prev = NULL;
			next = temps[master]->gpios;
			temps[master]->gpios = this;
			if (next)
				next->prev = this;
			// Also, set pin to output.
			SET_OUTPUT (pin);
		}
		else
		{
			prev = NULL;
			next = NULL;
		}
	}
#endif
}


void Gpio::save (int16_t &addr, bool eeprom)
{
	write_16 (addr, pin.write (), eeprom);
	write_8 (addr, state, eeprom);
#ifndef LOWMEM
	write_8 (addr, master, eeprom);
	write_float (addr, value, eeprom);
#else
	write_8 (addr, 0, eeprom);
	write_float (addr, NAN, eeprom);
#endif
}
#endif

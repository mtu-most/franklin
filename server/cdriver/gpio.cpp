#include "cdriver.h"

void Gpio::load(uint8_t self, int32_t &addr, bool eeprom)
{
	pin.read(read_16(addr, eeprom));
	state = read_8(addr, eeprom);
	uint8_t oldmaster = master;
	master = read_8(addr, eeprom);
	value = read_float(addr, eeprom);
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
	if (master < num_temps) {
		adcvalue = temps[master].toadc(value);
		adc_phase = 1;
		next_temp_time = 0;
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
}

void Gpio::save(int32_t &addr, bool eeprom)
{
	write_16(addr, pin.write(), eeprom);
	write_8(addr, state, eeprom);
	write_8(addr, master, eeprom);
	write_float(addr, value, eeprom);
}

int32_t Gpio::savesize0() {
	return 1 * 2 + 2 * 1 + sizeof(float) * 1;
}

void Gpio::init() {
	pin.init();
	state = 0;
	value = NAN;
	master = ~0;
	prev = ~0;
	next = ~0;
}

void Gpio::free() {
	pin.read(0);
	if (master < num_temps) {
		if (prev < num_gpios)
			gpios[prev].next = next;
		else
			temps[master].following_gpios = next;
		if (next < num_gpios)
			gpios[next].prev = prev;
	}
}

void Gpio::copy(Gpio &dst) {
	dst.pin.read(pin.write());
	dst.state = state;
	dst.value = value;
	dst.next = next;
	dst.prev = prev;
	dst.master = master;
}

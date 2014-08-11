#include "firmware.h"

bool globals_load(int16_t &addr, bool eeprom)
{
	uint8_t ns = read_8(addr, eeprom);
	uint8_t nt = read_8(addr, eeprom);
	uint8_t ng = read_8(addr, eeprom);
	uint8_t nl = read_8(addr, eeprom);
	// Check if there is enough memory for the new values.
	if (ns != num_spaces || nt != num_temps || ng != num_gpios || nl != namelength) {
		uint16_t size = nl + 1;
		size += globals_save(~0, 0, true);
		for (uint8_t s = 0; s < ns; ++s)
			size += Space::save((s < num_spaces ? s : ~0), 0, true);
		for (uint8_t t = 0; t < nt; ++t)
			size += Temp::save((s < num_temps ? s : ~0), 0, true);
		for (uint8_t g = 0; g < ng; ++g)
			size += Gpio::save((g < num_gpios ? g : ~0), 0, true);
		if (size > E2END) {
			debug("New settings make size %d, which is larger than %d: rejecting.", size, E2END);
			return false;
		}
		if (ns != num_spaces || nt != num_temps || ng != num_gpios) {
			// Free the old memory.
			for (uint8_t s = ns; s < num_spaces; ++s)
				delete spaces[s];
			for (uint8_t t = nt; t < num_temps; ++t)
				delete temps[t];
			for (uint8_t g = ng; g < num_gpios; ++g)
				delete gpios[g];
			// Initialize the new memory.
			Space *new_spaces = new Space *[ns];
			for (uint8_t s = 0; s < min(ns, num_spaces); ++s)
				new_spaces[s] = spaces[s];
			for (uint8_t s = num_spaces; s < ns; ++s)
				new_spaces[s] = new Space;
			delete[] spaces;
			spaces = new_spaces;

			Temp *new_temps = new Temp *[nt];
			for (uint8_t t = 0; t < min(nt, num_temps); ++t)
				new_temps[t] = temps[t];
			for (uint8_t t = num_temps; t < nt; ++t)
				new_temps[t] = new Temp;
			delete[] temps;
			temps = new_temps;

			Gpio *new_gpios = new Gpio *[ng];
			for (uint8_t g = 0; g < min(ns, num_gpios); ++g)
				new_gpios[g] = gpios[g];
			for (uint8_t g = num_gpios; g < ns; ++g)
				new_gpios[g] = new Gpio;
			delete[] gpios;
			gpios = new_gpios;

			delete[] name;
			name = new char[nl];
			for (uint8_t s = 0; s < num_spaces; ++s)
				spaces[s].init(name);
			for (uint8_t t = 0; t < num_temps; ++t)
				temps[t].init();
			for (uint8_t g = 0; g < num_gpios; ++g)
				gpios[g].init();
		}
	}
	namelen = nl;
	for (uint8_t n = 0; n < namelen; ++n)
		name[n] = read_8(addr, eeprom);
	max_deviation = read_float(addr, eeprom);
	led_pin.read(read_16(addr, eeprom));
	probe_pin.read(read_16(addr, eeprom));
#ifndef LOWMEM
	room_T = read_float(addr, eeprom);
#else
	read_float(addr, eeprom);	// Discard value.
#endif
	motor_limit = read_float(addr, eeprom);
	temp_limit = read_float(addr, eeprom);
	feedrate = read_float(addr, eeprom);
	if (isnan(feedrate) || isinf(feedrate) || feedrate <= 0)
		feedrate = 1;
	SET_OUTPUT(led_pin);
}

void globals_save(uint8_t which, int16_t &addr, bool eeprom)
{
	if (!eeprom) {
		write_8(which, addr, QUEUE_LENGTH, false);
#ifdef HAVE_AUDIO
		write_8(which, addr, AUDIO_FRAGMENTS, false);
		write_8(which, addr, AUDIO_FRAGMENT_SIZE, false);
#else
		write_8(which, addr, 0, false);
		write_8(which, addr, 0, false);
#endif
		write_8(which, addr, NUM_DIGITAL_PINS, false);
		write_8(which, addr, NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS, false);
	}
	write_8(which, addr, num_spaces, eeprom);
	write_8(which, addr, num_temps, eeprom);
	write_8(which, addr, num_gpios, eeprom);
	write_8(which, addr, namelen, eeprom);
	for (uint8_t i = 0; i < namelen; ++i)
		write_8(which, addr, name[i], eeprom);
	write_float(which, addr, max_deviation, eeprom);
	write_16(which, addr, led_pin.write(), eeprom);
	write_16(which, addr, probe_pin.write(), eeprom);
#ifndef LOWMEM
	write_float(which, addr, room_T, eeprom);
#else
	write_float(which, addr, NAN, eeprom);
#endif
	write_float(which, addr, motor_limit, eeprom);
	write_float(which, addr, temp_limit, eeprom);
	write_float(which, addr, feedrate, eeprom);
}

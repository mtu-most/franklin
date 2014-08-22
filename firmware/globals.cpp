#include "firmware.h"

bool globals_load(int16_t &addr, bool eeprom)
{
#ifdef HAVE_SPACES
	uint8_t ns = read_8(addr, eeprom);
#else
	read_8(addr, eeprom);
#endif
#ifdef HAVE_TEMPS
	uint8_t nt = read_8(addr, eeprom);
#else
	read_8(addr, eeprom);
#endif
#ifdef HAVE_GPIOS
	uint8_t ng = read_8(addr, eeprom);
#else
	read_8(addr, eeprom);
#endif
	uint8_t nl = read_8(addr, eeprom);
	// Check if there is enough memory for the new values.
	if (
#ifdef HAVE_SPACES
			ns != num_spaces ||
#endif
#ifdef HAVE_TEMPS
		       	nt != num_temps ||
#endif
#ifdef HAVE_GPIOS
		       	ng != num_gpios ||
#endif
		       	nl != namelen) {
		uint16_t size = nl + 1;
		size += globals_size();
#ifdef HAVE_SPACES
		for (uint8_t s = 0; s < ns; ++s)
			size += s < num_spaces ? spaces[s].size() : Space::size0();
#endif
#ifdef HAVE_TEMPS
		for (uint8_t t = 0; t < nt; ++t)
			size += t < num_temps ? temps[t].size() : Temp::size0();
#endif
#ifdef HAVE_GPIOS
		for (uint8_t g = 0; g < ng; ++g)
			size += g < num_gpios ? gpios[g].size() : Gpio::size0();
#endif
		if (size > E2END) {
			debug("New settings make size %d, which is larger than %d: rejecting.", size, E2END);
			return false;
		}
		else {
			debug("New settings make size %d out of %d", size, E2END);
		}
		if (nl != namelen) {
			delete[] name;
			namelen = nl;
			name = new char[nl];
		}
		// Free the old memory and initialize the new memory.
#ifdef HAVE_SPACES
		if (ns != num_spaces) {
			for (uint8_t s = ns; s < num_spaces; ++s)
				spaces[s].free();
			Space *new_spaces = new Space[ns];
			for (uint8_t s = 0; s < min(ns, num_spaces); ++s)
				spaces[s].copy(new_spaces[s]);
			for (uint8_t s = num_spaces; s < ns; ++s)
				new_spaces[s].init();
			delete[] spaces;
			num_spaces = ns;
			spaces = new_spaces;
		}
#endif
#ifdef HAVE_TEMPS
		if (nt != num_temps) {
			for (uint8_t t = nt; t < num_temps; ++t)
				temps[t].free();
			Temp *new_temps = new Temp[nt];
			for (uint8_t t = 0; t < min(nt, num_temps); ++t)
				temps[t].copy(new_temps[t]);
			for (uint8_t t = num_temps; t < nt; ++t)
				new_temps[t].init();
			delete[] temps;
			num_temps = nt;
			temps = new_temps;
		}
#endif
#ifdef HAVE_GPIOS
		if (ng != num_gpios) {
			for (uint8_t g = nt; g < num_gpios; ++g)
				gpios[g].free();
			Gpio *new_gpios = new Gpio[ng];
			for (uint8_t g = 0; g < min(ns, num_gpios); ++g)
				gpios[g].copy(new_gpios[g]);
			for (uint8_t g = num_gpios; g < ns; ++g)
				new_gpios[g].init();
			delete[] gpios;
			num_gpios = ng;
			gpios = new_gpios;
		}
#endif
	}
	for (uint8_t n = 0; n < namelen; ++n)
		name[n] = read_8(addr, eeprom);
	led_pin.read(read_16(addr, eeprom));
	probe_pin.read(read_16(addr, eeprom));
	probe_dist = read_float(addr, eeprom);
	motor_limit = read_float(addr, eeprom);
	temp_limit = read_float(addr, eeprom);
	feedrate = read_float(addr, eeprom);
	if (isnan(feedrate) || isinf(feedrate) || feedrate <= 0)
		feedrate = 1;
	SET_OUTPUT(led_pin);
	return true;
}

void globals_save(int16_t &addr, bool eeprom)
{
	if (!eeprom) {
		write_8(addr, QUEUE_LENGTH, false);
#ifdef HAVE_AUDIO
		write_8(addr, AUDIO_FRAGMENTS, false);
		write_8(addr, AUDIO_FRAGMENT_SIZE, false);
#else
		write_8(addr, 0, false);
		write_8(addr, 0, false);
#endif
		write_8(addr, NUM_DIGITAL_PINS, false);
		write_8(addr, NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS, false);
	}
#ifdef HAVE_SPACES
	write_8(addr, num_spaces, eeprom);
#else
	write_8(addr, 0, eeprom);
#endif
#ifdef HAVE_TEMPS
	write_8(addr, num_temps, eeprom);
#else
	write_8(addr, 0, eeprom);
#endif
#ifdef HAVE_GPIOS
	write_8(addr, num_gpios, eeprom);
#else
	write_8(addr, 0, eeprom);
#endif
	write_8(addr, namelen, eeprom);
	for (uint8_t i = 0; i < namelen; ++i)
		write_8(addr, name[i], eeprom);
	write_16(addr, led_pin.write(), eeprom);
	write_16(addr, probe_pin.write(), eeprom);
	write_float(addr, probe_dist, eeprom);
	write_float(addr, motor_limit, eeprom);
	write_float(addr, temp_limit, eeprom);
	write_float(addr, feedrate, eeprom);
}

int16_t globals_size() {
	return 1 * 4 + 2 * 2 + sizeof(float) * 5 + namelen;
}

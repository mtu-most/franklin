#include "cdriver.h"

#if 0
#define ldebug debug
#else
#define ldebug(...) do {} while(0)
#endif

bool globals_load(int32_t &addr, bool eeprom)
{
	uint8_t ns = read_8(addr, eeprom);
	uint8_t nt = read_8(addr, eeprom);
	uint8_t ng = read_8(addr, eeprom);
	uint8_t nl = read_8(addr, eeprom);
	// Check if there is enough memory for the new values.
	if (
			ns != num_spaces ||
		       	nt != num_temps ||
		       	ng != num_gpios ||
		       	nl != namelen) {
		ldebug("something changed %d %d %d %d %d %d %d %d", ns, num_spaces, nt, num_temps, ng, num_gpios, nl, namelen);
		uint32_t savesize = nl + 1;
		savesize += globals_savesize();
		ldebug("size");
		for (uint8_t s = 0; s < ns; ++s) {
			savesize += s < num_spaces ? spaces[s].savesize() : Space::savesize0();
		}
		ldebug("space size");
		for (uint8_t t = 0; t < nt; ++t) {
			savesize += t < num_temps ? temps[t].savesize() : Temp::savesize0();
		}
		ldebug("temps size");
		for (uint8_t g = 0; g < ng; ++g) {
			savesize += g < num_gpios ? gpios[g].savesize() : Gpio::savesize0();
		}
		ldebug("size done");
		if (savesize > E2END) {
			debug("New settings make savesize %d, which is larger than %d: rejecting.", savesize, E2END);
			return false;
		}
		else {
			ldebug("New settings make savesize %d out of %d.", savesize, E2END);
		}
		if (nl != namelen) {
			ldebug("new name");
			char *newname;
			mem_alloc(nl, &newname, "name");
			if (newname) {
				mem_free(&name);
				mem_retarget(&newname, &name);
				namelen = nl;
			}
		}
		// Free the old memory and initialize the new memory.
		if (ns != num_spaces) {
			ldebug("new space");
			for (uint8_t s = ns; s < num_spaces; ++s)
				spaces[s].free();
			Space *new_spaces;
			mem_alloc(sizeof(Space) * ns, &new_spaces, "spaces");
			if (new_spaces) {
				for (uint8_t s = 0; s < min(ns, num_spaces); ++s)
					spaces[s].copy(new_spaces[s]);
				for (uint8_t s = num_spaces; s < ns; ++s)
					new_spaces[s].init();
				mem_free (&spaces);
				num_spaces = ns;
				mem_retarget(&new_spaces, &spaces);
			}
		}
		ldebug("new done");
		if (nt != num_temps) {
			ldebug("new temp");
			for (uint8_t t = nt; t < num_temps; ++t)
				temps[t].free();
			Temp *new_temps;
			mem_alloc(sizeof(Temp) * nt, &new_temps, "temps");
			if (new_temps) {
				for (uint8_t t = 0; t < min(nt, num_temps); ++t)
					temps[t].copy(new_temps[t]);
				for (uint8_t t = num_temps; t < nt; ++t)
					new_temps[t].init();
				mem_free(&temps);
				num_temps = nt;
				mem_retarget(&new_temps, &temps);
			}
		}
		ldebug("new done");
		if (ng != num_gpios) {
			for (uint8_t g = ng; g < num_gpios; ++g)
				gpios[g].free();
			Gpio *new_gpios;
			mem_alloc(sizeof(Gpio) * ng, &new_gpios, "gpios");
			if (new_gpios) {
				for (uint8_t g = 0; g < min(ng, num_gpios); ++g)
					gpios[g].copy(new_gpios[g]);
				for (uint8_t g = num_gpios; g < ng; ++g)
					new_gpios[g].init();
				mem_free(&gpios);
				num_gpios = ng;
				mem_retarget(&new_gpios, &gpios);
			}
		}
		ldebug("new done");
	}
	// If allocation failed, namelen may not be large enough.  Read all the bytes anyway.
	for (uint8_t n = 0; n < nl; ++n) {
		char c = read_8(addr, eeprom);
		if (n < namelen)
			name[n] = c;
	}
	led_pin.read(read_16(addr, eeprom));
	probe_pin.read(read_16(addr, eeprom));
	probe_dist = read_float(addr, eeprom);
	probe_safe_dist = read_float(addr, eeprom);
	bed_id = read_8(addr, eeprom);
	motor_limit = read_float(addr, eeprom);
	temp_limit = read_float(addr, eeprom);
	feedrate = read_float(addr, eeprom);
	if (isnan(feedrate) || isinf(feedrate) || feedrate <= 0)
		feedrate = 1;
	ldebug("all done");
	arch_motors_change();
	return true;
}

void globals_save(int32_t &addr, bool eeprom)
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
	write_8(addr, num_spaces, eeprom);
	write_8(addr, num_temps, eeprom);
	write_8(addr, num_gpios, eeprom);
	write_8(addr, namelen, eeprom);
	for (uint8_t i = 0; i < namelen; ++i)
		write_8(addr, name[i], eeprom);
	write_16(addr, led_pin.write(), eeprom);
	write_16(addr, probe_pin.write(), eeprom);
	write_float(addr, probe_dist, eeprom);
	write_float(addr, probe_safe_dist, eeprom);
	write_8(addr, bed_id, eeprom);
	write_float(addr, motor_limit, eeprom);
	write_float(addr, temp_limit, eeprom);
	write_float(addr, feedrate, eeprom);
}

int32_t globals_savesize() {
	return 1 * 4 + 2 * 2 + sizeof(float) * 5 + namelen;
}

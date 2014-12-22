#include "cdriver.h"

#if 0
#define ldebug debug
#else
#define ldebug(...) do {} while(0)
#endif

bool globals_load(int32_t &addr)
{
	uint8_t ns = read_8(addr);
	uint8_t nt = read_8(addr);
	uint8_t ng = read_8(addr);
	uint8_t nl = read_8(addr);
	if (nl != namelen) {
		ldebug("new name");
		delete[] name;
		name = new char[nl];
		namelen = nl;
	}
	// Free the old memory and initialize the new memory.
	if (ns != num_spaces) {
		ldebug("new space");
		for (uint8_t s = ns; s < num_spaces; ++s)
			spaces[s].free();
		Space *new_spaces = new Space[ns];
		for (uint8_t s = 0; s < min(ns, num_spaces); ++s)
			spaces[s].copy(new_spaces[s]);
		for (uint8_t s = num_spaces; s < ns; ++s)
			new_spaces[s].init(s);
		delete[] spaces;
		spaces = new_spaces;
		num_spaces = ns;
	}
	if (nt != num_temps) {
		ldebug("new temp");
		for (uint8_t t = nt; t < num_temps; ++t)
			temps[t].free();
		Temp *new_temps = new Temp[nt];
		for (uint8_t t = 0; t < min(nt, num_temps); ++t)
			temps[t].copy(new_temps[t]);
		for (uint8_t t = num_temps; t < nt; ++t)
			new_temps[t].init();
		delete[] temps;
		temps = new_temps;
		num_temps = nt;
	}
	ldebug("new done");
	if (ng != num_gpios) {
		for (uint8_t g = ng; g < num_gpios; ++g)
			gpios[g].free();
		Gpio *new_gpios = new Gpio[ng];
		for (uint8_t g = 0; g < min(ng, num_gpios); ++g)
			gpios[g].copy(new_gpios[g]);
		for (uint8_t g = num_gpios; g < ng; ++g)
			new_gpios[g].init();
		delete[] gpios;
		gpios = new_gpios;
		num_gpios = ng;
	}
	ldebug("new done");
	for (uint8_t n = 0; n < nl; ++n)
		name[n] = read_8(addr);
	led_pin.read(read_16(addr));
	probe_pin.read(read_16(addr));
	probe_dist = read_float(addr);
	probe_safe_dist = read_float(addr);
	bed_id = read_8(addr);
	timeout = read_16(addr);
	feedrate = read_float(addr);
	if (isnan(feedrate) || isinf(feedrate) || feedrate <= 0)
		feedrate = 1;
	ldebug("all done");
	arch_motors_change();
	return true;
}

void globals_save(int32_t &addr)
{
	write_8(addr, QUEUE_LENGTH);
#ifdef HAVE_AUDIO
	write_8(addr, AUDIO_FRAGMENTS);
	write_8(addr, AUDIO_FRAGMENT_SIZE);
#else
	write_8(addr, 0);
	write_8(addr, 0);
#endif
	write_8(addr, NUM_DIGITAL_PINS);
	write_8(addr, NUM_ANALOG_INPUTS);
	write_8(addr, num_spaces);
	write_8(addr, num_temps);
	write_8(addr, num_gpios);
	write_8(addr, namelen);
	for (uint8_t i = 0; i < namelen; ++i)
		write_8(addr, name[i]);
	write_16(addr, led_pin.write());
	write_16(addr, probe_pin.write());
	write_float(addr, probe_dist);
	write_float(addr, probe_safe_dist);
	write_8(addr, bed_id);
	write_16(addr, timeout);
	write_float(addr, feedrate);
}

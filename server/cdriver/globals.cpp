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
	led_pin.read(read_16(addr));
	probe_pin.read(read_16(addr));
	timeout = read_16(addr);
	feedrate = read_float(addr);
	if (isnan(feedrate) || isinf(feedrate) || feedrate <= 0)
		feedrate = 1;
	int ce = read_8(addr);
	if (current_extruder != ce && num_spaces > 0 && queue_start == queue_end && !queue_full) {
		queue[queue_end].probe = false;
		queue[queue_end].cb = false;
		queue[queue_end].f[0] = INFINITY;
		queue[queue_end].f[1] = INFINITY;
		for (int i = 0; num_spaces > 0 && i < spaces[0].num_axes; ++i) {
			queue[queue_end].data[i] = spaces[0].axis[i]->settings[current_fragment].current;
			for (int s = 0; s < num_spaces; ++s)
				queue[queue_end].data[i] = space_types[spaces[s].type].unchange0(&spaces[s], i, queue[queue_end].data[i]);
		}
		for (int i = spaces[0].num_axes; i < QUEUE_LENGTH; ++i) {
			queue[queue_end].data[i] = NAN;
		}
		queue_end = (queue_end + 1) % QUEUE_LENGTH;
		// This shouldn't happen and causes communication problems, but if you have a 1-item buffer it is correct.
		if (queue_end == queue_start)
			queue_full = true;
		current_extruder = ce;
		next_move();
		buffer_refill();
	}
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
	write_16(addr, led_pin.write());
	write_16(addr, probe_pin.write());
	write_16(addr, timeout);
	write_float(addr, feedrate);
	write_8(addr, current_extruder);
}

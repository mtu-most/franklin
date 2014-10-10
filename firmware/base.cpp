#include "firmware.h"

#ifdef DEFINE_MAIN
#include "firmware.ino"
// Memory handling
void _mem_alloc(uint16_t size, void **target) {
	*target = malloc(size);
	if (!*target) {
		debug("unable to allocate memory");
		reset();
	}
}

void _mem_retarget(void **target, void **newtarget) {
	*newtarget = *target;
	*target = NULL;
}

void _mem_free(void **target) {
	free(*target);
	*target = NULL;
}

// Main function.
int main(int argc, char **argv) {
	setup();
	while(true)
		loop();
}

void reset() {
	// This shouldn't happen.  But if it does, die.
	// First disable all pins.
	debug("Reset requested; exiting.");
	led_pin.read(0);
	probe_pin.read(0);
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (unsigned m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->step_pin.read(0);
			sp.motor[m]->dir_pin.read(0);
			sp.motor[m]->enable_pin.read(0);
			sp.motor[m]->limit_min_pin.read(0);
			sp.motor[m]->limit_max_pin.read(0);
			sp.motor[m]->sense_pin.read(0);
		}
	}
	for (uint8_t t = 0; t < num_temps; ++t) {
		temps[t].power_pin.read(0);
		temps[t].thermistor_pin.read(0);
	}
	for (uint8_t g = 0; g < num_gpios; ++g)
		gpios[g].pin.read(0);
	exit(0);
}

#endif

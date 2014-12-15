// vim: set foldmethod=marker :

#include "cdriver.h"

void reset() { // {{{
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
		temps[t].power_pin[0].read(0);
		temps[t].power_pin[1].read(0);
		temps[t].thermistor_pin.read(0);
	}
	for (uint8_t g = 0; g < num_gpios; ++g)
		gpios[g].pin.read(0);
	exit(0);
} // }}}

void disconnect() { // {{{
	// Hardware has disconnected.  Notify host and wait for reconnect.
	arch_disconnect();
	send_host(CMD_DISCONNECT);
	while (!arch_connected()) {
		poll(&pollfds[0], 1, -1);
		serial(0);
	}
}
// }}}

int main(int argc, char **argv) { // {{{
	if (argc != 2) {
		debug("Franklin cdriver is not intended to be called directly.\n");
		exit(1);
	}
	setup(argv[1]);
	while(true) {
		for (int i = 0; i < 2; ++i)
			pollfds[i].revents = 0;
		poll(pollfds, arch_active() ? 2 : 1, -1);
		for (int i = 0; i < 2; ++i) {
			if (pollfds[i].revents)
				serial(i);
		}
	}
} // }}}

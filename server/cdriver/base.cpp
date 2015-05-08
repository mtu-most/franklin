// vim: set foldmethod=marker :

#include "cdriver.h"

#ifdef SERIAL
// Only for connections that can fail.
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
	while (arch_fds() == 0) {
		poll(&pollfds[1], 1, -1);
		serial(0);
	}
}
// }}}
#endif

// Time handling.  {{{
static void get_current_times(uint32_t *current_time, uint32_t *longtime) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	if (current_time)
		*current_time = tv.tv_sec * 1000000 + tv.tv_usec;
	if (longtime)
		*longtime = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	//fprintf(stderr, "current times: %d %d\n", *current_time, *longtime);
}

uint32_t utime() {
	uint32_t ret;
	get_current_times(&ret, NULL);
	return ret;
}

uint32_t millis() {
	uint32_t ret;
	get_current_times(NULL, &ret);
	return ret;
}
// }}}

int main(int argc, char **argv) { // {{{
	if (argc != 3) {
		debug("Franklin cdriver is not intended to be called directly (argc = %d).\n", argc);
		exit(1);
	}
	setup(argv[1], argv[2]);
	struct itimerspec zero;
	zero.it_interval.tv_sec = 0;
	zero.it_interval.tv_nsec = 0;
	zero.it_value.tv_sec = 0;
	zero.it_value.tv_nsec = 0;
	while (true) {
		int arch = arch_fds();
		for (int i = 0; i < 2 + arch; ++i)
			pollfds[i].revents = 0;
		poll(pollfds, arch + 2, -1);
		if (pollfds[0].revents) {
			timerfd_settime(pollfds[0].fd, 0, &zero, NULL);
			if (run_file_wait)
				run_file_wait -= 1;
			run_file_fill_queue();
		}
		for (int i = 0; i < 2; ++i) {
			if (pollfds[1 + i].revents)
				serial(i);
		}
	}
} // }}}

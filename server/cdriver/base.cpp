// vim: set foldmethod=marker :

#define EXTERN	// This must be done in exactly one source file.
#include "cdriver.h"

#ifdef SERIAL
// Only for connections that can fail.
void disconnect() { // {{{
	// Hardware has disconnected.  Notify host and wait for reconnect.
	arch_disconnect();
	send_host(CMD_DISCONNECT);
	while (arch_fds() == 0) {
		poll(&pollfds[1], 1, -1);
		if (pollfds[1].revents & (POLLHUP | POLLERR)) {
			debug("Hang up (or error) on command input");
			exit(0);
		}
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
	int delay = 0;
	while (true) {
		int arch = arch_fds();
		for (int i = 0; i < 2 + arch; ++i)
			pollfds[i].revents = 0;
		poll(host_block ? &pollfds[2] : pollfds, arch + (host_block ? 0 : 2), delay);
		if (pollfds[0].revents) {
			timerfd_settime(pollfds[0].fd, 0, &zero, NULL);
			debug("gcode wait done; stop waiting (was %d)", run_file_wait);
			if (run_file_wait)
				run_file_wait -= 1;
			run_file_fill_queue();
		}
		if (pollfds[1].revents)
			serial(0);
		delay = arch_tick();
	}
} // }}}

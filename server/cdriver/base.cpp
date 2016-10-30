/* base.cpp - main function for Franklin
 * vim: set foldmethod=marker :
 * Copyright 2014-2016 Michigan Technological University
 * Copyright 2016 Bas Wijnen <wijnen@debian.org>
 * Author: Bas Wijnen <wijnen@debian.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define EXTERN	// This must be done in exactly one source file.
#include "cdriver.h"

#ifdef SERIAL
// Only for connections that can fail.
void disconnect(bool notify) { // {{{
	// Hardware has disconnected.  Notify host and wait for reconnect.
	arch_disconnect();
	if (notify)
		send_host(CMD_DISCONNECT);
}
// }}}
#endif

// Time handling.  {{{
static void get_current_times(int32_t *current_time, int32_t *longtime) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	if (current_time)
		*current_time = tv.tv_sec * 1000000 + tv.tv_usec;
	if (longtime)
		*longtime = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	//fprintf(stderr, "current times: %d %d\n", *current_time, *longtime);
}

int32_t utime() {
	int32_t ret;
	get_current_times(&ret, NULL);
	return ret;
}

int32_t millis() {
	int32_t ret;
	get_current_times(NULL, &ret);
	return ret;
}
// }}}

int main(int argc, char **argv) { // {{{
	(void)&argc;
	(void)&argv;
	setup();
	struct itimerspec zero;
	zero.it_interval.tv_sec = 0;
	zero.it_interval.tv_nsec = 0;
	zero.it_value.tv_sec = 0;
	zero.it_value.tv_nsec = 0;
	int delay = 0;
	while (true) {
		for (int i = 0; i < 2 + arch_fds(); ++i)
			pollfds[i].revents = 0;
		while (true) {
			bool action = false;
			if (!host_block && serialdev[0]->available())
				action |= serial(0);
			if (arch_fds() && serialdev[1]->available())
				action |= serial(1);
			if (!action)
				break;
		}
		poll(host_block ? &pollfds[2] : pollfds, arch_fds() + (host_block ? 0 : 2), delay);
		if (pollfds[0].revents) {
			timerfd_settime(pollfds[0].fd, 0, &zero, NULL);
			//debug("gcode wait done; stop waiting (was %d)", run_file_wait);
			if (run_file_wait)
				run_file_wait -= 1;
			run_file_fill_queue();
		}
		if (pollfds[1].revents)
			serial(0);
		delay = arch_tick();
	}
} // }}}

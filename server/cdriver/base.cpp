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
#include <execinfo.h>

//#define cdebug debug
#define cdebug(...) do {} while (0)

#ifdef SERIAL
// Only for connections that can fail.
void disconnect(bool notify, char const *reason, ...) { // {{{
	// Hardware has disconnected.  Notify host and wait for reconnect.
	arch_disconnect();
	if (notify) {
		if (reason == NULL)
			shmem->interrupt_str[0] = '\0';
		else {
			va_list v;
			va_start(v, reason);
			vsnprintf(const_cast <char *>(shmem->interrupt_str), PATH_MAX, reason, v);
			va_end(v);
		}
		prepare_interrupt();
		send_to_parent(CMD_DISCONNECT);
	}
}
// }}}
#endif

void debug_backtrace() {
	void *array[10];
	size_t size;

	// get void*'s for all entries on the stack
	size = backtrace(array, 10);

	// print out all the frames to stderr
	debug("Stack backtrace:");
	char **frames = backtrace_symbols(array, size);
	if (frames == NULL) {
		debug("unable to print stack backtrace");
		return;
	}
	for (size_t i = 0; i < size; ++i) {
		debug("frame %ld: %s", long(i), frames[i]);
	}
	free(frames);
}

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

static void handle_request() { // {{{
	// There is exactly one command waiting, so don't try to read more.
	cdebug("command received");
	char cmd;
	while (true) {
		errno = 0;
		int ret = read(fromserver, &cmd, 1);
		if (ret == 1)
			break;
		if (errno == EINTR || errno == EAGAIN)
			continue;
		debug("cannot read from server: %s (%d, %d)", strerror(errno), errno, ret);
		exit(0);
		//abort();
	}
	cdebug("command was %x", cmd);
	request(cmd);
} // }}}

static void handle_interrupt_reply() { // {{{
	char cmd;
	while (true) {
		errno = 0;
		cdebug("reading interrupt reply");
		int ret = read(interrupt_reply, &cmd, 1);
		if (ret == 1)
			break;
		if (errno == EINTR || errno == EAGAIN)
			continue;
		debug("cannot read interrupt reply from server: %s", strerror(errno));
		exit(0);
		//abort();
	}
	if (!interrupt_pending)
		debug("received interrupt reply without pending interrupt");
	if (stopping == 2) {
		sending_fragment = 0;
		stopping = 1;
	}
	cdebug("received interrupt reply");
	interrupt_pending = false;
} // }}}

static void handle_pending_events() { // {{{
	if (interrupt_pending)
		return;
	if (num_file_done_events > 0) {
		prepare_interrupt();
		send_to_parent(CMD_FILE_DONE);
		num_file_done_events -=1;
		return;
	}
	cdebug("pending %d", cb_pending);
	if (cb_pending && !arch_running()) {
		cdebug("sending movecb");
		prepare_interrupt();
		cb_pending = false;
		send_to_parent(CMD_MOVECB);
		return;
	}
	if (stopping == 1) {
		stopping = 0;
	}
	buffer_refill();
} // }}}

int main(int argc, char **argv) { // {{{
	(void)&argc;
	memfd = atoi(argv[1]);
	shmem = reinterpret_cast <SharedMemory *>(mmap(NULL, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, memfd, 0));
	// to from int int_reply
	toserver = shmem->ints[0];
	fromserver = shmem->ints[1];
	interrupt = shmem->ints[2];
	interrupt_reply = shmem->ints[3];
	pollfds[1].fd = fromserver;
	pollfds[1].events = POLLIN | POLLPRI;
	pollfds[1].revents = 0;
	pollfds[2].fd = interrupt_reply;
	pollfds[2].events = POLLIN | POLLPRI;
	pollfds[2].revents = 0;
	setup();
	delayed_reply(); // Let server know we are ready.
	struct itimerspec zero;
	zero.it_interval.tv_sec = 0;
	zero.it_interval.tv_nsec = 0;
	zero.it_value.tv_sec = 0;
	zero.it_value.tv_nsec = 0;
	int delay = 0;
	while (true) {
		for (int i = 0; i < BASE_FDS + arch_fds(); ++i)
			pollfds[i].revents = 0;
		while (true) {
			bool action = false;
			if (arch_fds() && serialdev && serialdev->available())
				action |= serial(true);
			if (!interrupt_pending && pins_changed > 0) {
				for (int i = 0; i < num_gpios; ++i) {
					if (gpios[i].changed) {
						gpios[i].changed = false;
						pins_changed -= 1;
						prepare_interrupt();	// Does nothing, but good to always have it paired with send_to_parent.
						shmem->interrupt_ints[0] = i;
						shmem->interrupt_ints[1] = gpios[i].pin.inverted() ? !gpios[i].value : gpios[i].value;
						send_to_parent(CMD_PINCHANGE);
						action = true;
						break;
					}
				}
				if (!action) {
					debug("warning: pins_changed = %d, but no pins are marked changed", pins_changed);
					pins_changed = 0;
				}
			}
			if (!action)
				break;
		}
		//debug("polling with delay %d", delay);
		poll(pollfds, arch_fds() + BASE_FDS, delay);
		//debug("poll values in %d pri %d err %d hup %d nval %d out %d", POLLIN, POLLPRI, POLLERR, POLLHUP, POLLNVAL, POLLOUT);
		cdebug("poll return %d %d %d (pending %d)", pollfds[0].revents, pollfds[1].revents, pollfds[2].revents, interrupt_pending);
		if (pollfds[0].revents) {
			timerfd_settime(pollfds[0].fd, 0, &zero, NULL);
			if (run_file_wait > 0)
				run_file_wait -= 1;
		}
		if (pollfds[2].revents)
			handle_interrupt_reply();
		if (pollfds[1].revents)
			handle_request();
		handle_pending_events();
		delay = arch_tick();
	}
} // }}}

void send_to_parent(char cmd) { // {{{
	if (interrupt_pending) {
		debug("send_to_parent called without previous prepare_interrupt!");
		abort();
		prepare_interrupt();
	}
	if (stopping == 3 && cmd == CMD_LIMIT) {
		stopping = 2;
	}
	cdebug("sending interrupt 0x%x", cmd);
	if (write(interrupt, &cmd, 1) != 1) {
		debug("failed to write to parent");
		abort();
	}
	cdebug("interrupt sent: %02x", cmd);
	interrupt_pending = true;
} // }}}

void prepare_interrupt() { // {{{
	while (interrupt_pending) {
		// Ignore timeouts.
		pollfds[1].revents = 0;
		pollfds[2].revents = 0;
		poll(&pollfds[1], BASE_FDS - 1, -1);
		if (pollfds[2].revents)
			handle_interrupt_reply();
		if (pollfds[1].revents)
			handle_request();
	}
} // }}}

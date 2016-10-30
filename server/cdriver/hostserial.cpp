/* hostserial.cpp - serial port host handling for Franklin
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

#include "cdriver.h"

void HostSerial::begin() {
	pollfds[1].fd = 0;
	pollfds[1].events = POLLIN | POLLPRI;
	pollfds[1].revents = 0;
	start = 0;
	end = 0;
	fcntl(0, F_SETFL, O_NONBLOCK);
}

void HostSerial::write(char c) {
	//debug("Firmware write byte: %x", c);
	while (true) {
		errno = 0;
		int ret = ::write(1, &c, 1);
		if (ret == 1)
			break;
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			debug("write to host failed: %d %s", ret, strerror(errno));
			abort();
		}
	}
}

void HostSerial::refill() {
	start = 0;
	end = ::read(0, buffer, sizeof(buffer));
	//debug("refill %d bytes", end);
	if (end < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK)
			debug("read returned error: %s", strerror(errno));
		end = 0;
	}
	if (end == 0 && pollfds[1].revents) {
		debug("EOF detected on standard input; exiting.");
		exit(0);
	}
	pollfds[1].revents = 0;
}

int HostSerial::read() {
	if (start == end)
		refill();
	if (start == end) {
		debug("EOF on standard input; exiting.");
		exit(0);
	}
	int ret = buffer[start++];
	//debug("Cdriver read byte from host: %x", ret);
	return ret;
}

int HostSerial::available() {
	if (start == end)
		refill();
	//debug("available on host: %d - %d = %d", end, start, end - start);
	return end - start;
}

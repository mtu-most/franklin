#include "cdriver.h"

void HostSerial::begin(int baud) {
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
		abort();
	}
	pollfds[1].revents = 0;
}

int HostSerial::read() {
	if (start == end)
		refill();
	if (start == end) {
		debug("EOF on standard input; exiting.");
		abort();
	}
	int ret = buffer[start++];
	//debug("Firmware read byte: %x", ret);
	return ret;
}

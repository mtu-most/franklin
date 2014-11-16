// vim: set foldmethod=marker :
#ifndef _ARCH_AVR_H
// Includes and defines. {{{
//#define DEBUG_AVRCOMM

#define _ARCH_AVR_H
#include <stdint.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <sys/time.h>
#include <poll.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <sys/mman.h>

#define debug(...) do { buffered_debug_flush(); fprintf(stderr, "#"); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while (0)
#define F(x) (x)
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

// Not defines, because they can change value, but they are expected to be defines.
EXTERN uint8_t NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS, ADCBITS;
EXTERN uint16_t E2END;
// }}}


enum HWCommands { // {{{
	HWC_BEGIN,	// 00
	HWC_PING,	// 01
	HWC_RESET,	// 02
	HWC_SETUP,	// 03
	HWC_MSETUP,	// 04
	HWC_MOVE,	// 05
	HWC_ABORT,	// 06
	HWC_SETPOS,	// 07
	HWC_ADDPOS,	// 08
	HWC_GETPOS,	// 09
	HWC_RESETPIN,	// 0a
	HWC_SETPIN,	// 0b
	HWC_UNSETPIN,	// 0c
	HWC_INPUTPIN,	// 0d
	HWC_GETPIN,	// 0e
	HWC_GETADC,	// 0f
	HWC_AUDIO_SETUP,// 10
	HWC_AUDIO_DATA,	// 11

	HWC_START,	// 12
	HWC_PONG,	// 13
	HWC_POS,	// 14
	HWC_PIN,	// 15
	HWC_ADC,	// 16

	HWC_LIMIT,	// 17
	HWC_SENSE0,	// 18
	HWC_SENSE1	// 19
};
// }}}
struct AVRMotor { // {{{
	int space, motor;
}; // }}}
struct AVRSerial : public Serial_t { // {{{
	char buffer[256];
	int start, end, fd;
	inline void begin(char const *port, int baud);
	inline void write(char c);
	inline void refill();
	inline int read();
	int readBytes (char *target, int len) {
		for (int i = 0; i < len; ++i)
			*target++ = read();
		return len;
	}
	void flush() {}
	int available() {
		if (start == end)
			refill();
		return end - start;
	}
};
// }}}
struct FakeSerial : public Serial_t { // {{{
	char buffer[256];
	int start, end;
	inline void begin(int baud);
	inline void write(char c);
	inline void refill();
	inline int read();
	int readBytes (char *target, int len) {
		for (int i = 0; i < len; ++i)
			*target++ = read();
		return len;
	}
	void flush() {}
	int available() {
		if (start == end)
			refill();
		return end - start;
	}
};
// }}}

// Declarations of static variables; extern because this is a header file. {{{
EXTERN FakeSerial Serial;
EXTERN AVRSerial avr_serial;
EXTERN struct pollfd avr_pollfds[2];
EXTERN bool avr_wait_for_reply;
EXTERN uint8_t avr_num_motors;
EXTERN uint8_t avr_pong;
EXTERN char avr_buffer[256];
EXTERN int avr_limiter_space;
EXTERN int avr_adc;
EXTERN bool avr_wait_for_adc;
// }}}

// Timekeeping. {{{
static inline void get_current_times(uint32_t *current_time, uint32_t *longtime) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	if (current_time)
		*current_time = tv.tv_sec * 1000000 + tv.tv_usec;
	if (longtime)
		*longtime = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	//debug("current times: %d %d", *current_time, *longtime);
}

static inline uint32_t utime() {
	uint32_t ret;
	get_current_times(&ret, NULL);
	return ret;
}
static inline uint32_t millis() {
	uint32_t ret;
	get_current_times(NULL, &ret);
	return ret;
}
// }}}

static inline int32_t arch_getpos(uint8_t s, uint8_t m);
static inline void avr_write_ack();

// Serial port communication. {{{
static inline void avr_send() {
	//debug("avr_send");
	send_packet();
	uint32_t start = millis();
	int phase = 0;
	while (out_busy) {
		uint32_t now = millis();
		//debug("avr send");
		if (now - start > 1000) {
			debug("avr_send failed, packet start: %02x %02x %02x", avr_buffer[0], avr_buffer[1], avr_buffer[2]);
			break;
		}
		// TODO: use select.
		serial(1);
		//if (out_busy[1])
		//	debug("avr waiting for ack");
		if ((now - start) / 100 > phase) {
			debug("resending packet");
			send_packet();
			phase = (now - start) / 100;
		}
	}
}

static inline void avr_call1(uint8_t cmd, uint8_t arg) {
	avr_buffer[0] = 3;
	avr_buffer[1] = cmd;
	avr_buffer[2] = arg;
	prepare_packet(avr_buffer);
	avr_send();
}

static inline void avr_get_reply() {
	// Wait for reply.  It is stored in command[1].
	uint32_t now = utime();
	while (avr_wait_for_reply) {
		//debug("avr wait");
		// TODO: use select.
		serial(1);
		if (avr_wait_for_reply && utime() - now >= 1000000) {
			debug("no reply; resending");
			avr_send();
			now = utime();
			continue;
		}
	}
}

static inline void hwpacket() {
	// Handle data in command[1].
#if 0
	fprintf(stderr, "packet received:");
	for (uint8_t i = 1; i < command[1][0]; ++i)
		fprintf(stderr, " %02x", command[1][i]);
	fprintf(stderr, "\n");
#endif
	switch (command[1][1]) {
	case HWC_LIMIT:
	case HWC_SENSE0:
	case HWC_SENSE1:
	{
		uint8_t which = command[1][2];
		if (which >= avr_num_motors) {
			debug("cdriver: Invalid limit or sense for avr motor %d", which);
			write_stall();
			return;
		}
		int s = 0, m = 0;
		for (s = 0; s < num_spaces; ++s) {
			if (which < spaces[s].num_motors) {
				m = which;
				break;
			}
			which -= spaces[s].num_motors;
		}
		ReadFloat f;
		for (uint8_t i = 0; i < sizeof(int32_t); ++i)
			f.b[i] = command[1][3 + i];
		spaces[s].motor[m]->current_pos = f.i;
		if (command[1][1] == HWC_LIMIT) {
			avr_limiter_space = s;
			abort_move(false);
			int num_movecbs = cbs_after_current_move;
			cbs_after_current_move = 0;
			num_movecbs += next_move();
			send_host(CMD_LIMIT, s, m, f.i / spaces[s].motor[m]->steps_per_m, num_movecbs);
		}
		else {
			spaces[s].motor[m]->sense_pos = f.i;
			spaces[s].motor[m]->sense_state = 1;
			send_host(CMD_SENSE, s, m, 0, command[1][1] == HWC_SENSE1);
		}
		avr_write_ack();
		return;
	}
	case HWC_PONG:
		avr_pong = command[1][2];
		avr_write_ack();
		return;
	case HWC_ADC:
		if (!avr_wait_for_adc)
			debug("received unexpected adc reading");
		avr_wait_for_adc = false;
		avr_adc = (command[1][2] & 0xff) | ((command[1][3] & 0xff) << 8);
		avr_write_ack();
		return;
	default:
		if (!avr_wait_for_reply)
			debug("received unexpected reply!");
		avr_wait_for_reply = false;
		return;
	}
}

static inline void arch_ack() {
	if (avr_wait_for_reply)
		return;
	if (avr_limiter_space < 0)
		return;
	int space = avr_limiter_space;
	avr_limiter_space = -1;
	for (uint8_t m = 0; m < spaces[space].num_motors; ++m) {
		spaces[space].motor[m]->current_pos = arch_getpos(space, m);
	}
}

static inline void avr_write_ack() {
	write_ack();
	if (!out_busy)
		arch_ack();
}
// }}}

// Hardware interface {{{
EXTERN char avr_pins[256];

static inline void SET_INPUT(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin] == 2)
		return;
	avr_pins[_pin.pin] = 2;
	avr_call1(HWC_INPUTPIN, _pin.pin);
}

static inline void SET_INPUT_NOPULLUP(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin] == 3)
		return;
	avr_pins[_pin.pin] = 3;
	avr_call1(HWC_UNSETPIN, _pin.pin);
}

static inline void RESET(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin] == 0)
		return;
	avr_pins[_pin.pin] = 0;
	avr_call1(_pin.inverted() ? HWC_SETPIN : HWC_RESETPIN, _pin.pin);
}

static inline void SET(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin] == 1)
		return;
	avr_pins[_pin.pin] = 1;
	avr_call1(_pin.inverted() ? HWC_RESETPIN : HWC_SETPIN, _pin.pin);
}

static inline void SET_OUTPUT(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin] < 2)
		return;
	RESET(_pin);
}

static inline bool GET(Pin_t _pin, bool _default) {
	if (!_pin.valid())
		return _default;
	if (avr_wait_for_reply)
		debug("cdriver problem: avr_wait_for_reply already set!");
	avr_wait_for_reply = true;
	avr_call1(HWC_GETPIN, _pin.pin);
	avr_get_reply();
	avr_write_ack();
	return _pin.inverted() ^ command[1][2];
}
// }}}

// Setup hooks. {{{
static inline void arch_reset() {
	// Initialize connection.
	if (avr_pong == 2) {
		debug("reset ignored");
		return;
	}
	avr_serial.write(CMD_ACK0);
	avr_serial.write(CMD_ACK1);
	avr_call1(HWC_PING, 0);
	avr_call1(HWC_PING, 1);
	avr_call1(HWC_PING, 2);
	uint32_t start = millis();
	while (avr_pong != 2) {
		//debug("avr pongwait %d", avr_pong);
		uint32_t now = millis();
		if (now - start >= 1000) {
			debug("no pong seen; giving up.\n");
			reset();
		}
		serial(1);
	}
}

static inline void arch_setup_start(char const *port) {
	// Set up arch variables.
	avr_wait_for_reply = false;
	avr_wait_for_adc = false;
	avr_adc = -1;
	avr_num_motors = 0;
	avr_pong = ~0;
	avr_limiter_space = -1;
	avr_limiter_space = -1;
	for (int i = 0; i < 0x100; ++i)
		avr_pins[i] = 3;	// INPUT_NOPULLUP.
	// Set up serial port.
	avr_serial.begin(port, 115200);
	serialdev[1] = &avr_serial;
	arch_reset();
}

static inline void arch_setup_end() {
	// Get constants.
	avr_buffer[0] = 2 + ID_SIZE;
	avr_buffer[1] = HWC_BEGIN;
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		avr_buffer[2 + i] = printerid[i];
	if (avr_wait_for_reply)
		debug("avr_wait_for_reply already set in begin");
	avr_wait_for_reply = true;
	prepare_packet(avr_buffer);
	avr_send();
	avr_get_reply();
	avr_write_ack();
	ReadFloat f;
	for (uint8_t i = 0; i < sizeof(uint32_t); ++i)
		f.b[i] = command[1][2 + i];
	protocol_version = f.ui;
	NUM_DIGITAL_PINS = command[1][6];
	NUM_ANALOG_INPUTS = command[1][7];
	E2END = (command[1][8] & 0xff) | ((command[1][9] & 0xff) << 8);
	ADCBITS = command[1][10];
	avr_pong = -1;	// Choke on reset again.
}

static inline void arch_motor_change(uint8_t s, uint8_t sm) {
	uint8_t m = sm;
	for (uint8_t st = 0; st < s; ++st)
		m += spaces[st].num_motors;
	avr_buffer[0] = 22;
	avr_buffer[1] = HWC_MSETUP;
	avr_buffer[2] = m;
	Motor &mtr = *spaces[s].motor[sm];
	uint16_t p = mtr.step_pin.write();
	//debug("arch motor change %d %d %d %x", s, sm, m, p);
	avr_buffer[3] = p & 0xff;
	avr_buffer[4] = (p >> 8) & 0xff;
	p = mtr.dir_pin.write();
	avr_buffer[5] = p & 0xff;
	avr_buffer[6] = (p >> 8) & 0xff;
	p = mtr.limit_min_pin.write();
	avr_buffer[7] = p & 0xff;
	avr_buffer[8] = (p >> 8) & 0xff;
	p = mtr.limit_max_pin.write();
	avr_buffer[9] = p & 0xff;
	avr_buffer[10] = (p >> 8) & 0xff;
	p = mtr.sense_pin.write();
	avr_buffer[11] = p & 0xff;
	avr_buffer[12] = (p >> 8) & 0xff;
	avr_buffer[13] = mtr.max_steps;
	ReadFloat max_v, a;
	max_v.f = mtr.limit_v * mtr.steps_per_m / 1e6;
	a.f = 1.1 * mtr.limit_a * mtr.steps_per_m / 1e12;
	for (int i = 0; i < sizeof(float); ++i) {
		avr_buffer[14 + i] = max_v.b[i];
		avr_buffer[14 + i + sizeof(float)] = a.b[i];
	}
	prepare_packet(avr_buffer);
	avr_send();
}

static inline void arch_change(bool motors) {
	avr_num_motors = 0;
	for (uint8_t s = 0; s < num_spaces; ++s)
		avr_num_motors += spaces[s].num_motors;
	avr_buffer[0] = 2 + 1 + 2 + sizeof(int32_t);
	avr_buffer[1] = HWC_SETUP;
	avr_buffer[2] = avr_num_motors;
	uint16_t p = led_pin.write();
	avr_buffer[3] = p & 0xff;
	avr_buffer[4] = (p >> 8) & 0xff;
	avr_buffer[5] = 100; // speed_error.
	prepare_packet(avr_buffer);
	avr_send();
	if (motors) {
		for (uint8_t s = 0; s < num_spaces; ++s) {
			for (uint8_t m = 0; m < spaces[s].num_motors; ++m)
				arch_motor_change(s, m);
		}
	}
}

static inline void arch_motors_change() {
	arch_change(true);
}

static inline void arch_globals_change() {
	arch_change(false);
}
// }}}

// Running hooks. {{{
static inline void arch_run() {
}

static inline void wait_for_event(uint32_t micro, uint32_t current_time) {
	avr_pollfds[0].revents = 0;
	avr_pollfds[1].revents = 0;
	//debug("polling with micro %d", micro);
	poll(avr_pollfds, 2, micro == ~0 ? -1 : micro / 1000);
}

static inline int32_t arch_getpos(uint8_t s, uint8_t m) {
	float steps_per_m = spaces[s].motor[m]->steps_per_m;
	for (uint8_t st = 0; st < s; ++st)
		m += spaces[st].num_motors;
	avr_buffer[0] = 3;
	avr_buffer[1] = HWC_GETPOS;
	avr_buffer[2] = m;
	prepare_packet(avr_buffer);
	if (avr_wait_for_reply) {
		debug("avr_wait_for_reply already set in getpos");
		abort();
	}
	avr_wait_for_reply = true;
	avr_send();
	avr_get_reply();
	avr_write_ack();
	ReadFloat pos;
	for (uint8_t i = 0; i < sizeof(int32_t); ++i)
		pos.b[i] = command[1][2 + i];
	//debug("getpos %d %d", m, pos.i);
	return pos.i;
}

static inline void arch_setpos(uint8_t s, uint8_t m) {
	ReadFloat pos;
	pos.i = spaces[s].motor[m]->current_pos;
	for (uint8_t st = 0; st < s; ++st)
		m += spaces[st].num_motors;
	//debug("setpos %d %d", m, pos.i);
	avr_buffer[0] = 3 + sizeof(int32_t);
	avr_buffer[1] = HWC_SETPOS;
	avr_buffer[2] = m;
	for (uint8_t i = 0; i < sizeof(int32_t); ++i)
		avr_buffer[3 + i] = pos.b[i];
	prepare_packet(avr_buffer);
	avr_send();
}

static inline int32_t arch_addpos(uint8_t s, uint8_t m, int32_t diff) {
	ReadFloat pos;
	pos.i = diff;
	for (uint8_t st = 0; st < s; ++st)
		m += spaces[st].num_motors;
	//debug("setpos %d %d", m, pos.i);
	avr_buffer[0] = 3 + sizeof(int32_t);
	avr_buffer[1] = HWC_ADDPOS;
	avr_buffer[2] = m;
	for (uint8_t i = 0; i < sizeof(int32_t); ++i)
		avr_buffer[3 + i] = pos.b[i];
	prepare_packet(avr_buffer);
	avr_send();
}

static inline void arch_move() { // {{{
	uint8_t wlen = ((avr_num_motors - 1) >> 3) + 1;
	for (uint8_t i = 0; i < wlen; ++i)
		avr_buffer[2 + i] = 0;
	uint8_t m = 0, mi = 0;
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		if (!sp.active) {
			m += sp.num_motors;
			continue;
		}
		for (uint8_t sm = 0; sm < sp.num_motors; ++sm, ++m) {
			Motor &mtr = *sp.motor[sm];
			avr_buffer[2 + (m >> 3)] |= 1 << (m & 0x7);
			ReadFloat limit, current, speed;
			speed.f = moving ? mtr.last_v * mtr.steps_per_m / 1e6 : 0;
			limit.i = speed.f < 0 ? -MAXLONG : MAXLONG;
			current.i = mtr.current_pos;
			//debug("move %d %x %f %d %d", m, avr_buffer[2], speed.f, limit.i, current.i);
			for (uint8_t i = 0; i < 4; ++i) {
				avr_buffer[2 + wlen + mi * 12 + i] = speed.b[i];
				avr_buffer[2 + wlen + mi * 12 + 4 + i] = limit.b[i];
				avr_buffer[2 + wlen + mi * 12 + 8 + i] = current.b[i];
			}
			mi += 1;
		}
	}
	avr_buffer[0] = 2 + wlen + mi * 12;
	avr_buffer[1] = HWC_MOVE;
	prepare_packet(avr_buffer);
	avr_send();
}
// }}}
// }}}

// ADC hooks. {{{
static inline int adc_get(uint8_t _pin) {
	return avr_adc;
}

static inline void adc_start(uint8_t _pin) {
	//debug("getting adc");
	if (avr_wait_for_adc)
		debug("avr_wait_for_adc already set");
	avr_wait_for_adc = true;
	avr_adc = -1;
	avr_call1(HWC_GETADC, _pin);
}

static inline bool adc_ready(uint8_t _pin) {
	return avr_adc >= 0;
}
// }}}

// Debugging hooks. {{{
static inline void START_DEBUG() {
	fprintf(stderr, "cdriver debug from firmware: ");
}

static inline void DO_DEBUG(char c) {
	fprintf(stderr, "%c"
#ifdef DEBUG_AVRCOMM
				" "
#endif
				, c);
}

static inline void END_DEBUG() {
	fprintf(stderr, "\n");
}
// }}}

// Stubs. {{{
static inline void watchdog_reset() {}
static inline void watchdog_enable() {}
static inline void watchdog_disable() {}

static inline uint8_t read_eeprom(uint32_t address) {
	return 0;	// TODO
}

static inline void write_eeprom(uint32_t address, uint8_t data) {
	// TODO
}
// }}}

// Inline AVRSerial methods. {{{
void AVRSerial::begin(char const *port, int baud) {
	// Open serial port and prepare pollfd.
	fd = open(port, O_RDWR);
	avr_pollfds[1].fd = fd;
	avr_pollfds[1].events = POLLIN | POLLPRI;
	avr_pollfds[1].revents = 0;
	start = 0;
	end = 0;
	fcntl(fd, F_SETFL, O_NONBLOCK);
	// TODO: Set baud rate and control.
}

void AVRSerial::write(char c) {
#ifdef DEBUG_AVRCOMM
	debug("w\t%02x", c & 0xff);
#endif
	while (true) {
		errno = 0;
		int ret = ::write(fd, &c, 1);
		if (ret == 1)
			break;
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			debug("write to avr failed: %d %s", ret, strerror(errno));
			abort();
		}
	}
}

void AVRSerial::refill() {
	start = 0;
	end = ::read(fd, buffer, sizeof(buffer));
	//debug("refill %d bytes", end);
	if (end < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK)
			debug("read returned error: %s", strerror(errno));
		end = 0;
	}
	if (end == 0 && avr_pollfds[1].revents) {
		debug("EOF detected on serial port; exiting.");
		reset();
	}
	avr_pollfds[1].revents = 0;
}

int AVRSerial::read() {
	if (start == end)
		refill();
	if (start == end) {
		debug("eof on input; exiting.");
		reset();
	}
	int ret = buffer[start++];
#ifdef DEBUG_AVRCOMM
	debug("r %02x", ret & 0xff);
#endif
	return ret;
}
// }}}

// Inline FakeSerial methods. {{{
void FakeSerial::begin(int baud) {
	avr_pollfds[0].fd = 0;
	avr_pollfds[0].events = POLLIN | POLLPRI;
	avr_pollfds[0].revents = 0;
	start = 0;
	end = 0;
	fcntl(0, F_SETFL, O_NONBLOCK);
}

void FakeSerial::write(char c) {
	//debug("Firmware write byte: %x", c);
	while (true) {
		errno = 0;
		int ret = ::write(1, &c, 1);
		if (ret == 1)
			break;
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			debug("write to host failed: %d %s", ret, strerror(errno));
			exit(0);
		}
	}
}

void FakeSerial::refill() {
	start = 0;
	end = ::read(0, buffer, sizeof(buffer));
	//debug("refill %d bytes", end);
	if (end < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK)
			debug("read returned error: %s", strerror(errno));
		end = 0;
	}
	if (end == 0 && avr_pollfds[0].revents) {
		debug("EOF detected on standard input; exiting.");
		reset();
	}
	avr_pollfds[0].revents = 0;
}

int FakeSerial::read() {
	if (start == end)
		refill();
	if (start == end) {
		debug("eof on input; exiting.");
		reset();
	}
	int ret = buffer[start++];
	//debug("Firmware read byte: %x", ret);
	return ret;
}
// }}}

#endif

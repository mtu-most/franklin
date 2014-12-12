// vim: set foldmethod=marker :
#ifndef _ARCH_AVR_H
// Includes and defines. {{{
//#define DEBUG_AVRCOMM

#define _ARCH_AVR_H
#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <cstdlib>
#include <cstdio>
#include <sys/time.h>
#include <poll.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <sys/mman.h>

#define ADCBITS 10
#define debug(...) do { buffered_debug_flush(); fprintf(stderr, "#"); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while (0)
#define F(x) (x)
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

// Not defines, because they can change value.
EXTERN uint8_t NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS, NUM_MOTORS, NUM_BUFFERS, FRAGMENTS_PER_BUFFER, BYTES_PER_FRAGMENT;
// }}}

enum Control {
	CTRL_RESET,
	CTRL_SET,
	CTRL_UNSET,
	CTRL_INPUT
};

enum HWCommands { // {{{
	HWC_BEGIN = 0x40,
	HWC_PING,	// 41
	HWC_RESET,	// 42
	HWC_SETUP,	// 43
	HWC_CONTROL,	// 44
	HWC_MSETUP,	// 45
	HWC_ASETUP,	// 46
	HWC_HOME,	// 47
	HWC_START_MOVE,	// 48
	HWC_MOVE,	// 49
	HWC_START,	// 4a
	HWC_STOP,	// 4b
	HWC_ABORT,	// 4c
	HWC_DISCARD,	// 4d
	HWC_GETPIN,	// 4e

	HWC_READY = 0x60,
	HWC_PONG,	// 61
	HWC_HOMED,	// 62
	HWC_PIN,	// 63
	HWC_STOPPED,	// 64

	HWC_DONE,	// 65
	HWC_UNDERRUN,	// 66
	HWC_ADC,	// 67
	HWC_LIMIT,	// 68
	HWC_SENSE0,	// 69
	HWC_SENSE1	// 6a
};
// }}}

extern int avr_active_motors;
static inline int hwpacketsize(int len, int *available) {
	int const arch_packetsize[16] = { 0, 2, 0, 2, 0, 2, 2, 4, 0, 0, 0, -1, -1, -1, -1, -1 };
	switch (command[1][0] & ~0x10) {
	case HWC_READY:
		if (len >= 2)
			return command[1][1];
		if (*available == 0)
			return 2;	// The data is not available, so this will not trigger the packet to be parsed yet.
		command[1][1] = serialdev[1]->read();
		command_end[1] += 1;
		*available -= 1;
		return command[1][1];
	case HWC_HOMED:
		return 1 + 4 * avr_active_motors;
	case HWC_STOPPED:
		return 2 + 4 * avr_active_motors;
	case HWC_LIMIT:
		return 3 + 4 * avr_active_motors;
	case HWC_SENSE0:
	case HWC_SENSE1:
		return 2 + 4 * avr_active_motors;
	default:
		return arch_packetsize[command[1][0] & 0xf];
	}
}

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
struct HostSerial : public Serial_t { // {{{
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
EXTERN HostSerial host_serial;
EXTERN AVRSerial avr_serial;
EXTERN bool avr_wait_for_reply;
EXTERN uint8_t avr_pong;
EXTERN char avr_buffer[256];
EXTERN int avr_limiter_space;
EXTERN int avr_limiter_motor;
EXTERN int *avr_adc;
EXTERN bool avr_running;
EXTERN char *avr_pins;
EXTERN int32_t *avr_pos_offset;
EXTERN int avr_active_motors;
EXTERN int *avr_adc_id;
EXTERN uint8_t *avr_control_queue;
EXTERN bool *avr_in_control_queue;
EXTERN int avr_control_queue_length;
// }}}

// Time handling.  {{{
static inline void get_current_times(uint32_t *current_time, uint32_t *longtime) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	if (current_time)
		*current_time = tv.tv_sec * 1000000 + tv.tv_usec;
	if (longtime)
		*longtime = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	//fprintf(stderr, "current times: %d %d\n", *current_time, *longtime);
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

static inline void avr_write_ack();

// Serial port communication. {{{
static inline void avr_send();

static inline void try_send_control() {
	if (out_busy || avr_control_queue_length == 0)
		return;
	avr_buffer[0] = HWC_CONTROL;
	avr_buffer[1] = avr_control_queue_length;
	for (int i = 0; i < avr_control_queue_length; ++i) {
		avr_buffer[2 + i * 2] = avr_control_queue[i * 2];
		avr_buffer[3 + i * 2] = avr_control_queue[i * 2 + 1];
		avr_in_control_queue[avr_control_queue[i * 2 + 1]] = false;
	}
	prepare_packet(avr_buffer, 2 + avr_control_queue_length * 2);
	avr_control_queue_length = 0;
	avr_send();
}

static inline void avr_send() {
	//debug("avr_send");
	send_packet();
	for (int counter = 0; counter < 0x28 && out_busy; ++counter) {
		//debug("avr send");
		poll(&pollfds[1], 1, 100);
		serial(1);
		//if (out_busy[1])
		//	debug("avr waiting for ack");
		if (out_busy && (counter & 0xf) == 0xf) {
			debug("resending packet");
			send_packet();
		}
	}
	if (out_busy) {
		debug("avr_send failed, packet start: %02x %02x %02x", avr_buffer[0], avr_buffer[1], avr_buffer[2]);
		out_busy = false;
	}
	try_send_control();
}

static inline void avr_call1(uint8_t cmd, uint8_t arg) {
	avr_buffer[0] = cmd;
	avr_buffer[1] = arg;
	prepare_packet(avr_buffer, 2);
	avr_send();
}

static inline void avr_get_reply() {
	for (int counter = 0; avr_wait_for_reply && counter < 0x10; ++counter) {
		//debug("avr wait");
		pollfds[1].revents = 0;
		poll(&pollfds[1], 1, 100);
		serial(1);
		if (avr_wait_for_reply && (counter & 0x7) == 0x7) {
			debug("no reply; resending");
			avr_send();
		}
	}
}

static inline void avr_get_current_pos(int offset) {
	int mi = 0;
	for (int ts = 0; ts < num_spaces; mi += spaces[ts++].num_motors) {
		for (int tm = 0; tm < spaces[ts].num_motors; ++tm) {
			spaces[ts].motor[tm]->settings[current_fragment].current_pos = -avr_pos_offset[tm + mi];
			for (int i = 0; i < 4; ++i) {
				spaces[ts].motor[tm]->settings[current_fragment].current_pos += int(uint8_t(command[1][offset + 4 * (tm + mi) + i])) << (i * 8);
			}
			spaces[ts].motor[tm]->settings[current_fragment].hwcurrent_pos = spaces[ts].motor[tm]->settings[current_fragment].current_pos;
			debug("cp %d %d %d", ts, tm, spaces[ts].motor[tm]->settings[current_fragment].hwcurrent_pos);
		}
	}
}

static inline void arch_stop();

static inline void hwpacket(int len) {
	// Handle data in command[1].
#if 0
	fprintf(stderr, "packet received:");
	for (uint8_t i = 0; i < len; ++i)
		fprintf(stderr, " %02x", command[1][i]);
	fprintf(stderr, "\n");
#endif
	switch (command[1][0] & ~0x10) {
	case HWC_LIMIT:
	case HWC_SENSE0:
	case HWC_SENSE1:
	{
		uint8_t which = command[1][1];
		if (which >= NUM_MOTORS) {
			debug("cdriver: Invalid limit or sense for avr motor %d", which);
			write_stall();
			return;
		}
		avr_write_ack();
		int s = 0, m = 0;
		for (s = 0; s < num_spaces; ++s) {
			if (which < spaces[s].num_motors) {
				m = which;
				break;
			}
			which -= spaces[s].num_motors;
		}
		//debug("cp1 %d", spaces[s].motor[m]->current_pos);
		int offset, pos;
		bool limit = (command[1][0] & ~0x10) == HWC_LIMIT;
		if (limit) {
			//debug("limit!");
			offset = 3;
			pos = command[1][2];
		}
		else
			offset = 2;
		avr_get_current_pos(offset);
		if (limit) {
			abort_move(pos);
			send_host(CMD_LIMIT, s, m, spaces[s].motor[m]->settings[current_fragment].current_pos / spaces[s].motor[m]->steps_per_m, cbs_after_current_move);
			cbs_after_current_move = 0;
			avr_running = false;
			free_fragments = FRAGMENTS_PER_BUFFER;
		}
		else {
			spaces[s].motor[m]->sense_state = 1;
			send_host(CMD_SENSE, s, m, 0, (command[1][0] & ~0x10) == HWC_SENSE1);
		}
		return;
	}
	case HWC_PONG:
	{
		avr_pong = command[1][1];
		avr_write_ack();
		return;
	}
	case HWC_ADC:
	{
		int pin = command[1][1];
		if (pin < 0 || pin >= NUM_ANALOG_INPUTS) {
			debug("invalid adc %d received", pin);
			avr_write_ack();
			return;
		}
		avr_adc[pin] = (command[1][2] & 0xff) | ((command[1][3] & 0xff) << 8);
		avr_write_ack();
		if (avr_adc_id[pin] >= 0 && avr_adc_id[pin] < num_temps)
			handle_temp(avr_adc_id[pin], avr_adc[pin]);
		return;
	}
	case HWC_UNDERRUN:
	{
		debug("underrun");
		avr_running = false;
		// Fall through.
	}
	case HWC_DONE:
	{
		int cbs = 0;
		for (int i = 0; i < command[1][1]; ++i)
			cbs += settings[(current_fragment + free_fragments + i + (moving ? 1 : 0)) % FRAGMENTS_PER_BUFFER].cbs;
		if (cbs)
			send_host(CMD_MOVECB, cbs);
		free_fragments += command[1][1];
		avr_write_ack();
		if (!out_busy)
			buffer_refill();
		return;
	}
	default:
	{
		if (!avr_wait_for_reply)
			debug("received unexpected reply!");
		avr_wait_for_reply = false;
		return;
	}
	}
}

static inline void avr_write_ack() {
	write_ack();
}

static inline bool arch_active() {
	return true;
}
// }}}

// Hardware interface {{{
static inline void avr_setup_pin(int pin, int type) {
	if (avr_in_control_queue[pin])
	{
		for (int i = 0; i < avr_control_queue_length; ++i) {
			if (avr_control_queue[i * 2 + 1] != pin)
				continue;
			avr_control_queue[i * 2] = type | 0xc;
			return;
		}
	}
	avr_control_queue[avr_control_queue_length * 2] = type | 0xc;
	avr_control_queue[avr_control_queue_length * 2 + 1] = pin;
	avr_control_queue_length += 1;
	try_send_control();
}

static inline void SET_INPUT(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin] == 2)
		return;
	avr_pins[_pin.pin] = 2;
	avr_setup_pin(_pin.pin, CTRL_INPUT);
}

static inline void SET_INPUT_NOPULLUP(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin] == 3)
		return;
	avr_pins[_pin.pin] = 3;
	avr_setup_pin(_pin.pin, CTRL_UNSET);
}

static inline void RESET(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin] == 0)
		return;
	avr_pins[_pin.pin] = 0;
	avr_setup_pin(_pin.pin, _pin.inverted() ? CTRL_SET : CTRL_RESET);
}

static inline void SET(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin] == 1)
		return;
	avr_pins[_pin.pin] = 1;
	avr_setup_pin(_pin.pin, _pin.inverted() ? CTRL_RESET : CTRL_SET);
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
	for (int counter = 0; avr_pong != 2 && counter < 100; ++counter) {
		//debug("avr pongwait %d", avr_pong);
		pollfds[1].revents = 0;
		poll(&pollfds[1], 1, 10);
		serial(1);
	}
	if (avr_pong != 2) {
		debug("no pong seen; giving up.\n");
		reset();
	}
}

enum MotorFlags {
	LIMIT = 1,
	SENSE0 = 2,
	SENSE1 = 4,
	INVERT_LIMIT_MIN = 8,
	INVERT_LIMIT_MAX = 16,
	INVERT_STEP = 32,
	SENSE_STATE = 64,
	ACTIVE = 128
};

static inline void arch_motor_change(uint8_t s, uint8_t sm) {
	uint8_t m = sm;
	for (uint8_t st = 0; st < s; ++st)
		m += spaces[st].num_motors;
	avr_buffer[0] = HWC_MSETUP;
	avr_buffer[1] = m;
	Motor &mtr = *spaces[s].motor[sm];
	//debug("arch motor change %d %d %d %x", s, sm, m, p);
	avr_buffer[2] = (mtr.step_pin.valid() ? mtr.step_pin.pin : ~0);
	avr_buffer[3] = (mtr.dir_pin.valid() ? mtr.dir_pin.pin : ~0);
	if (mtr.dir_pin.inverted()) {
		avr_buffer[4] = (mtr.limit_max_pin.valid() ? mtr.limit_max_pin.pin : ~0);
		avr_buffer[5] = (mtr.limit_min_pin.valid() ? mtr.limit_min_pin.pin : ~0);
	}
	else {
		avr_buffer[4] = (mtr.limit_min_pin.valid() ? mtr.limit_min_pin.pin : ~0);
		avr_buffer[5] = (mtr.limit_max_pin.valid() ? mtr.limit_max_pin.pin : ~0);
	}
	avr_buffer[6] = (mtr.sense_pin.valid() ? mtr.sense_pin.pin : ~0);
	avr_buffer[7] = ACTIVE | (mtr.step_pin.inverted() ? INVERT_STEP : 0) | (mtr.limit_min_pin.inverted() ? INVERT_LIMIT_MIN : 0) | (mtr.limit_max_pin.inverted() ? INVERT_LIMIT_MAX : 0);
	prepare_packet(avr_buffer, 8);
	avr_send();
}

static inline void arch_change(bool motors) {
	avr_buffer[0] = HWC_SETUP;
	avr_buffer[1] = led_pin.valid() ? led_pin.pin : ~0;
	for (int i = 0; i < 4; ++i)
		avr_buffer[2 + i] = (hwtime_step >> (8 * i)) & 0xff;
	prepare_packet(avr_buffer, 6);
	avr_send();
	if (motors) {
		int num_motors = 0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			for (uint8_t m = 0; m < spaces[s].num_motors; ++m) {
				arch_motor_change(s, m);
				num_motors += 1;
			}
		}
		for (int m = num_motors; m < avr_active_motors; ++m) {
			avr_buffer[0] = HWC_MSETUP;
			avr_buffer[1] = m;
			//debug("arch motor change %d %d %d %x", s, sm, m, p);
			avr_buffer[2] = ~0;
			avr_buffer[3] = ~0;
			avr_buffer[4] = ~0;
			avr_buffer[5] = ~0;
			avr_buffer[6] = ~0;
			avr_buffer[7] = 0;
			prepare_packet(avr_buffer, 8);
			avr_send();
		}
		avr_active_motors = num_motors;
	}
}

static inline void arch_motors_change() {
	arch_change(true);
}

static inline void arch_globals_change() {
	arch_change(false);
}

static inline void arch_setup_start(char const *port) {
	// Set up arch variables.
	avr_wait_for_reply = false;
	avr_adc = NULL;
	avr_running = false;
	NUM_ANALOG_INPUTS = 0;
	avr_pong = ~0;
	avr_limiter_space = -1;
	avr_limiter_motor = 0;
	avr_active_motors = 0;
	// Set up serial port.
	avr_serial.begin(port, 115200);
	serialdev[1] = &avr_serial;
	arch_reset();
}

static inline void arch_setup_end() {
	// Get constants.
	avr_buffer[0] = HWC_BEGIN;
	if (avr_wait_for_reply)
		debug("avr_wait_for_reply already set in begin");
	avr_wait_for_reply = true;
	prepare_packet(avr_buffer, 1);
	avr_send();
	avr_get_reply();
	avr_write_ack();
	protocol_version = 0;
	for (uint8_t i = 0; i < sizeof(uint32_t); ++i)
		protocol_version |= int(uint8_t(command[1][2 + i])) << (i * 8);
	NUM_DIGITAL_PINS = command[1][6];
	NUM_ANALOG_INPUTS = command[1][7];
	NUM_MOTORS = command[1][8];
	NUM_BUFFERS = command[1][9];
	FRAGMENTS_PER_BUFFER = command[1][10];
	BYTES_PER_FRAGMENT = command[1][11];
	avr_control_queue = new uint8_t[NUM_DIGITAL_PINS * 2];
	avr_in_control_queue = new bool[NUM_DIGITAL_PINS];
	avr_control_queue_length = 0;
	avr_pong = -1;	// Choke on reset again.
	avr_pins = new char[NUM_DIGITAL_PINS];
	for (int i = 0; i < NUM_DIGITAL_PINS; ++i) {
		avr_pins[i] = 3;	// INPUT_NOPULLUP.
		avr_in_control_queue[i] = false;
	}
	avr_adc = new int[NUM_ANALOG_INPUTS];
	avr_adc_id = new int[NUM_ANALOG_INPUTS];
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i) {
		avr_adc[i] = ~0;
		avr_adc_id[i] = ~0;
	}
	avr_pos_offset = new int32_t[NUM_MOTORS];
	for (int m = 0; m < NUM_MOTORS; ++m)
		avr_pos_offset[m] = 0;
}

static inline void arch_setup_temp(int id, int thermistor_pin, bool active, int power_pin = ~0, bool invert = false, int adctemp = 0) {
	avr_adc_id[thermistor_pin] = id;
	avr_buffer[0] = HWC_ASETUP;
	avr_buffer[1] = thermistor_pin;
	avr_buffer[2] = power_pin;
	avr_buffer[3] = ~0;
	int32_t t;
	if (active)
		t = (min(0x3fff, max(0, adctemp))) | (invert ? 0x4000 : 0);
	else
		t = 0xffff;
	//debug("setup adc %d 0x%x", id, t);
	avr_buffer[4] = t & 0xff;
	avr_buffer[5] = (t >> 8) & 0xff;
	avr_buffer[6] = ~0;
	avr_buffer[7] = ~0;
	prepare_packet(avr_buffer, 8);
	avr_send();
}
// }}}

// Running hooks. {{{
static inline void arch_addpos(uint8_t s, uint8_t m, int diff) {
	for (uint8_t st = 0; st < s; ++st)
		m += spaces[st].num_motors;
	//debug("setpos %d %d", m, pos.i);
	avr_pos_offset[m] -= diff;
}

static inline void arch_send_fragment(int fragment) {
	if (stopping)
		return;
	avr_buffer[0] = HWC_START_MOVE;
	//debug("send fragment %d %d", settings[fragment].fragment_length, fragment);
	avr_buffer[1] = settings[fragment].fragment_length;
	avr_buffer[2] = settings[fragment].num_active_motors;
	prepare_packet(avr_buffer, 3);
	avr_send();
	int mi = 0;
	for (int s = 0; !stopping && s < num_spaces; mi += spaces[s++].num_motors) {
		for (uint8_t m = 0; !stopping && m < spaces[s].num_motors; ++m) {
			if (spaces[s].motor[m]->settings[fragment].dir == 0)
				continue;
			//debug("sending %d %d %d", s, m, spaces[s].motor[m]->dir[fragment]);
			avr_buffer[0] = HWC_MOVE;
			avr_buffer[1] = mi + m;
			avr_buffer[2] = ((spaces[s].motor[m]->settings[fragment].dir < 0) ^ (spaces[s].motor[m]->dir_pin.inverted()) ? 1 : 0);
			int bytes = (settings[fragment].fragment_length + 1) / 2;
			for (int i = 0; i < bytes; ++i)
				avr_buffer[3 + i] = spaces[s].motor[m]->settings[fragment].data[i];
			prepare_packet(avr_buffer, 3 + bytes);
			avr_send();
		}
	}
}

static inline void arch_start_move() {
	if (avr_running)
		return;
	avr_running = true;
	avr_buffer[0] = HWC_START;
	prepare_packet(avr_buffer, 1);
	avr_send();
}

static inline void arch_stop() {
	if (out_busy) {
		stop_pending = true;
		return;
	}
	avr_running = false;
	avr_buffer[0] = HWC_STOP;
	if (avr_wait_for_reply)
		debug("avr_wait_for_reply already set in stop");
	avr_wait_for_reply = true;
	prepare_packet(avr_buffer, 1);
	avr_send();
	avr_get_reply();
	avr_write_ack();
	avr_get_current_pos(2);
	//debug("aborting at request");
	abort_move(command[1][1]);
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

// Inline AVRSerial methods. {{{
void AVRSerial::begin(char const *port, int baud) {
	// Open serial port and prepare pollfd.
	fd = open(port, O_RDWR);
	pollfds[1].fd = fd;
	pollfds[1].events = POLLIN | POLLPRI;
	pollfds[1].revents = 0;
	start = 0;
	end = 0;
	fcntl(fd, F_SETFL, O_NONBLOCK);
#if 0
	// Set baud rate and control.
	tcflush(fd, TCIOFLUSH);
	struct termios opts;
	tcgetattr(fd, &opts);
	opts.c_iflag = IGNBRK;
	opts.c_oflag = 0;
	opts.c_cflag = CS8 | CREAD | CLOCAL;
	opts.c_lflag = 0;
	opts.c_cc[VMIN] = 1;
	opts.c_cc[VTIME] = 0;
	cfsetispeed(&opts, B115200);
	cfsetospeed(&opts, B115200);
	tcsetattr(fd, TCSANOW, &opts);
#endif
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
	if (end == 0 && pollfds[1].revents) {
		debug("EOF detected on serial port; exiting.");
		reset();
	}
	pollfds[1].revents = 0;
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

// Inline HostSerial methods. {{{
void HostSerial::begin(int baud) {
	pollfds[0].fd = 0;
	pollfds[0].events = POLLIN | POLLPRI;
	pollfds[0].revents = 0;
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
			exit(0);
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
	if (end == 0 && pollfds[0].revents) {
		debug("EOF detected on standard input; exiting.");
		reset();
	}
	pollfds[0].revents = 0;
}

int HostSerial::read() {
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

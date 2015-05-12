// vim: set foldmethod=marker :
#ifndef _ARCH_AVR_H
#define _ARCH_AVR_H

// Includes and defines. {{{
//#define DEBUG_AVRCOMM

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

// Enable all the parts for a serial connection (which can fail) to the printer.
#define SERIAL
#define ADCBITS 10

// Not defines, because they can change value.
EXTERN uint8_t NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS, NUM_MOTORS, FRAGMENTS_PER_BUFFER, BYTES_PER_FRAGMENT;
// }}}

enum Control { // {{{
	CTRL_RESET,
	CTRL_SET,
	CTRL_INPUT,
	CTRL_UNSET,
	CTRL_NOTIFY = 0x40
};
// }}}

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
	HWC_START_PROBE,// 49
	HWC_MOVE,	// 4a
	HWC_START,	// 4b
	HWC_STOP,	// 4c
	HWC_ABORT,	// 4d
	HWC_DISCARD,	// 4e
	HWC_GETPIN,	// 4f

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
	HWC_SENSE1,	// 6a
	HWC_TIMEOUT,	// 6b
	HWC_PINCHANGE	// 6c
};
// }}}

// Function declarations. {{{
static inline void avr_send();
static inline void arch_start_move(int extra);
static inline void arch_motors_change();
static inline void RESET(Pin_t _pin);
// }}}

extern int avr_active_motors;
static inline int hwpacketsize(int len, int *available) { // {{{
	int const arch_packetsize[16] = { 0, 2, 0, 2, 0, 3, 0, 4, 0, 0, 0, 1, 3, -1, -1, -1 };
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
	case HWC_UNDERRUN:
		return 3 + 4 * avr_active_motors;
	case HWC_SENSE0:
	case HWC_SENSE1:
		return 2 + 4 * avr_active_motors;
	default:
		return arch_packetsize[command[1][0] & 0xf];
	}
}
// }}}

struct AVRSerial : public Serial_t { // {{{
	char buffer[256];
	int start, end_, fd;
	inline void begin(char const *port, int baud);
	inline void end() { close(fd); }
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
		if (start == end_)
			refill();
		return end_ - start;
	}
};
// }}}
struct Avr_pin_t { // {{{
	char state;
	char reset;
};
// }}}

// Declarations of static variables; extern because this is a header file. {{{
EXTERN AVRSerial avr_serial;
EXTERN bool avr_wait_for_reply;
EXTERN uint8_t avr_pong;
EXTERN char avr_buffer[256];
EXTERN int avr_limiter_space;
EXTERN int avr_limiter_motor;
EXTERN int *avr_adc;
EXTERN bool avr_running;
EXTERN Avr_pin_t *avr_pins;
EXTERN int32_t *avr_pos_offset;
EXTERN int avr_active_motors;
EXTERN int *avr_adc_id;
EXTERN uint8_t *avr_control_queue;
EXTERN bool *avr_in_control_queue;
EXTERN int avr_control_queue_length;
EXTERN bool avr_connected;
EXTERN bool avr_homing;
EXTERN bool avr_filling;
// }}}

#define avr_write_ack(reason) do { \
	/*debug("ack: %s", reason); */\
	write_ack(); \
} while(0)

// Serial port communication. {{{
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
		poll(&pollfds[2], 1, 100);
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
	for (int counter = 0; avr_wait_for_reply && counter < 0x80; ++counter) {
		//debug("avr wait");
		pollfds[2].revents = 0;
		poll(&pollfds[2], 1, 0x40);
		serial(1);
	}
	if (avr_wait_for_reply) {
		debug("no reply!");
		return;
	}
	avr_write_ack("reply");
}

static inline void avr_get_current_pos(int offset, bool check) {
	int mi = 0;
	for (int ts = 0; ts < num_spaces; mi += spaces[ts++].num_motors) {
		for (int tm = 0; tm < spaces[ts].num_motors; ++tm) {
			int old = spaces[ts].motor[tm]->settings[current_fragment].hwcurrent_pos;
			cpdebug(ts, tm, "cpb offset %d raw %d hwpos %d", avr_pos_offset[tm + mi], spaces[ts].motor[tm]->settings[current_fragment].hwcurrent_pos, spaces[ts].motor[tm]->settings[current_fragment].hwcurrent_pos + avr_pos_offset[tm + mi]);
			spaces[ts].motor[tm]->settings[current_fragment].current_pos = 0;
			for (int i = 0; i < 4; ++i) {
				spaces[ts].motor[tm]->settings[current_fragment].current_pos += int(uint8_t(command[1][offset + 4 * (tm + mi) + i])) << (i * 8);
			}
			if (spaces[ts].motor[tm]->dir_pin.inverted())
				spaces[ts].motor[tm]->settings[current_fragment].current_pos *= -1;
			spaces[ts].motor[tm]->settings[current_fragment].current_pos -= avr_pos_offset[tm + mi];
			spaces[ts].motor[tm]->settings[current_fragment].hwcurrent_pos = spaces[ts].motor[tm]->settings[current_fragment].current_pos;
			cpdebug(ts, tm, "cpa offset %d raw %d hwpos %d", avr_pos_offset[tm + mi], spaces[ts].motor[tm]->settings[current_fragment].hwcurrent_pos, spaces[ts].motor[tm]->settings[current_fragment].hwcurrent_pos + avr_pos_offset[tm + mi]);
			cpdebug(ts, tm, "getpos diff %d", spaces[ts].motor[tm]->settings[current_fragment].hwcurrent_pos - old);
			if (check && old != spaces[ts].motor[tm]->settings[current_fragment].hwcurrent_pos)
				abort();
		}
	}
}

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
		avr_write_ack("limit/sense");
		float pos;
		int s, m;
		if (which >= avr_active_motors) {
			s = -1;
			m = -1;
			pos = NAN;
		}
		else {
			for (s = 0; s < num_spaces; ++s) {
				if (which < spaces[s].num_motors) {
					m = which;
					break;
				}
				which -= spaces[s].num_motors;
			}
			cpdebug(s, m, "limit/sense");
			pos = spaces[s].motor[m]->settings[current_fragment].current_pos / spaces[s].motor[m]->steps_per_unit;
		}
		if ((command[1][0] & ~0x10) == HWC_LIMIT) {
			//debug("limit %d", command[1][2]);
			avr_homing = false;
			abort_move(int8_t(command[1][2]));
			//int i = 0;
			//for (int is = 0; is < num_spaces; ++is)
			//	for (int im = 0; im < spaces[is].num_motors; ++im, ++i)
			//		fprintf(stderr, "\t%8d", spaces[is].motor[im]->settings[current_fragment].current_pos + avr_pos_offset[i]);
			//fprintf(stderr, "\n");
			avr_get_current_pos(3, false);
			//i = 0;
			//for (int is = 0; is < num_spaces; ++is)
			//	for (int im = 0; im < spaces[is].num_motors; ++im, ++i)
			//		fprintf(stderr, "\t%8d", spaces[is].motor[im]->settings[current_fragment].current_pos + avr_pos_offset[i]);
			//fprintf(stderr, "\n");
			sending_fragment = 0;
			stopping = 2;
			send_host(CMD_LIMIT, s, m, pos);
			cbs_after_current_move = 0;
			avr_running = false;
			//debug("free limit");
		}
		else {
			// Sense
			avr_get_current_pos(2, false);	// TODO: this must not mess up actual current pos.
			spaces[s].motor[m]->sense_state = 1;
			send_host(CMD_SENSE, s, m, 0, (command[1][0] & ~0x10) == HWC_SENSE1);
		}
		return;
	}
	case HWC_PONG:
	{
		avr_pong = command[1][1];
		avr_write_ack("pong");
		return;
	}
	case HWC_ADC:
	{
		int pin = command[1][1];
		if (pin < 0 || pin >= NUM_ANALOG_INPUTS) {
			if (avr_pong == -1)
				debug("invalid adc %d received", pin);
			avr_write_ack("invalid adc");
			return;
		}
		avr_adc[pin] = (command[1][2] & 0xff) | ((command[1][3] & 0xff) << 8);
		avr_write_ack("adc");
		if (avr_adc_id[pin] >= 0 && avr_adc_id[pin] < num_temps)
			handle_temp(avr_adc_id[pin], avr_adc[pin]);
		return;
	}
	case HWC_UNDERRUN:
	{
		if (!avr_running) {
			debug("unexpected underrun?");
			//abort();
		}
		avr_running = false;
		if (moving) {
			debug("underrun");
			if (stopped)
				arch_start_move(command[1][1]);
			// Buffer is too slow with refilling; this will fix itself.
		}
		else {
			// Only overwrite current position if the new value is correct.
			if (stopped && !sending_fragment && command[1][1] == 0) {
				avr_get_current_pos(3, true);
			}
		}
		// Fall through.
	}
	case HWC_DONE:
	{
		//debug("done: %d %d %d", command[1][1], command[1][2], sending_fragment);
		if (FRAGMENTS_PER_BUFFER == 0) {
			debug("Done received while fragments per buffer is zero");
			avr_write_ack("invalid done");
			return;
		}
		first_fragment = -1;
		/*if (command[1][1] + command[1][2] != FRAGMENTS_PER_BUFFER - (running_fragment - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER) {
			debug("free fragments out of sync: %d %d %d %d", command[1][1] + command[1][2], FRAGMENTS_PER_BUFFER - (running_fragment - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER, FRAGMENTS_PER_BUFFER, sending_fragment);
			abort();
		}*/
		int cbs = 0;
		for (int i = 0; i < command[1][1]; ++i) {
			int f = (running_fragment + i) % FRAGMENTS_PER_BUFFER;
			//debug("fragment %d: cbs=%d current=%d", f, settings[f].cbs, current_fragment);
			cbs += settings[f].cbs;
		}
		if (cbs)
			send_host(CMD_MOVECB, cbs);
		if ((running_fragment - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER < command[1][1]) {
			debug("Done count %d higher than busy fragments; clipping", command[1][1]);
			avr_write_ack("invalid done");
			abort();
		}
		else
			avr_write_ack("done");
		running_fragment = (running_fragment + command[1][1]) % FRAGMENTS_PER_BUFFER;
		if (current_fragment == running_fragment && (command[1][0] & ~0x10) == HWC_DONE) {
			debug("Done received, but should be underrun");
			abort();
		}
		if (!out_busy)
			buffer_refill();
		run_file_fill_queue();
		return;
	}
	case HWC_HOMED:
	{
		if (!avr_homing)
			abort();
		stopped = true;
		moving = false;
		avr_homing = false;
		avr_get_current_pos(1, false);
		//int i = 0;
		//for (int s = 0; s < num_spaces; ++s)
		//	for (int m = 0; m < spaces[s].num_motors; ++m, ++i)
		//		fprintf(stderr, "\t%8d", spaces[s].motor[m]->settings[current_fragment].current_pos + avr_pos_offset[i]);
		//fprintf(stderr, "\n");
		avr_write_ack("homed");
		send_host(CMD_HOMED);
		return;
	}
	case HWC_TIMEOUT:
	{
		avr_write_ack("timeout");
		for (int i = 0; i < NUM_DIGITAL_PINS; ++i)
			avr_pins[i].state = avr_pins[i].reset;
		for (int i = 0; i < num_gpios; ++i)
			gpios[i].state = gpios[i].reset;
		motors_busy = false;
		// Everything has shut down; reset pins to normal (but inactive).
		arch_motors_change();
		for (int s = 0; s < num_spaces; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m) {
				RESET(spaces[s].motor[m]->step_pin);
				RESET(spaces[s].motor[m]->dir_pin);
				RESET(spaces[s].motor[m]->enable_pin);
			}
		}
		for (int t = 0; t < num_temps; ++t)
			settemp(t, NAN);
		send_host(CMD_TIMEOUT);
		return;
	}
	case HWC_PINCHANGE:
	{
		avr_write_ack("pinchange");
		for (int i = 0; i < num_gpios; ++i) {
			if (gpios[i].pin.pin == command[1][1])
				send_host(CMD_PINCHANGE, i, gpios[i].pin.inverted() ? !command[1][2] : command[1][2]);
		}
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
// }}}

// Hardware interface {{{
static inline void avr_setup_pin(int pin, int type, int resettype, int extra) {
	if (avr_in_control_queue[pin])
	{
		for (int i = 0; i < avr_control_queue_length; ++i) {
			if (avr_control_queue[i * 2 + 1] != pin)
				continue;
			avr_control_queue[i * 2] = type | (resettype << 2) | extra;
			return;
		}
	}
	avr_control_queue[avr_control_queue_length * 2] = type | (resettype << 2) | extra;
	avr_control_queue[avr_control_queue_length * 2 + 1] = pin;
	avr_control_queue_length += 1;
	avr_in_control_queue[pin] = true;
	try_send_control();
}

static inline void SET_INPUT(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state == 2)
		return;
	avr_pins[_pin.pin].state = 2;
	avr_setup_pin(_pin.pin, CTRL_INPUT, avr_pins[_pin.pin].reset, CTRL_NOTIFY);
}

static inline void SET_INPUT_NOPULLUP(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state == 3)
		return;
	avr_pins[_pin.pin].state = 3;
	avr_setup_pin(_pin.pin, CTRL_UNSET, avr_pins[_pin.pin].reset, CTRL_NOTIFY);
}

static inline void RESET(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state == 0)
		return;
	avr_pins[_pin.pin].state = 0;
	if (_pin.inverted())
		avr_setup_pin(_pin.pin, CTRL_SET, avr_pins[_pin.pin].reset < 2 ? 1 - avr_pins[_pin.pin].reset : avr_pins[_pin.pin].reset, 0);
	else
		avr_setup_pin(_pin.pin, CTRL_RESET, avr_pins[_pin.pin].reset, 0);
}

static inline void SET(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state == 1)
		return;
	avr_pins[_pin.pin].state = 1;
	if (_pin.inverted())
		avr_setup_pin(_pin.pin, CTRL_RESET, avr_pins[_pin.pin].reset < 2 ? 1 - avr_pins[_pin.pin].reset : avr_pins[_pin.pin].reset, 0);
	else
		avr_setup_pin(_pin.pin, CTRL_SET, avr_pins[_pin.pin].reset, 0);
}

static inline void SET_OUTPUT(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state < 2)
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
	return _pin.inverted() ^ command[1][1];
}

static inline void arch_pin_set_reset(Pin_t _pin, char state) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].reset == state)
		return;
	avr_pins[_pin.pin].reset = state;
	int s, r;
	if (_pin.inverted()) {
		s = avr_pins[_pin.pin].state < 2 ? 1 - avr_pins[_pin.pin].state : avr_pins[_pin.pin].state;
		r = avr_pins[_pin.pin].reset < 2 ? 1 - avr_pins[_pin.pin].reset : avr_pins[_pin.pin].reset;
	}
	else {
		s = avr_pins[_pin.pin].state;
		r = avr_pins[_pin.pin].reset;
	}
	avr_setup_pin(_pin.pin, s, r, state < 2 ? 0 : CTRL_NOTIFY);
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
		pollfds[2].revents = 0;
		poll(&pollfds[2], 1, 10);
		serial(1);
	}
	if (avr_pong != 2) {
		debug("no pong seen; giving up.\n");
		abort();
	}
}

enum MotorFlags {
	LIMIT = 1,
	SENSE0 = 2,
	SENSE1 = 4,
	INVERT_LIMIT_MIN = 8,
	INVERT_LIMIT_MAX = 16,
	INVERT_STEP = 32,
	SENSE_STATE = 64
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
	bool mininvert, maxinvert;
	if (mtr.dir_pin.inverted()) {
		avr_buffer[4] = (mtr.limit_max_pin.valid() ? mtr.limit_max_pin.pin : ~0);
		avr_buffer[5] = (mtr.limit_min_pin.valid() ? mtr.limit_min_pin.pin : ~0);
		mininvert = mtr.limit_max_pin.inverted();
		maxinvert = mtr.limit_min_pin.inverted();
	}
	else {
		avr_buffer[4] = (mtr.limit_min_pin.valid() ? mtr.limit_min_pin.pin : ~0);
		avr_buffer[5] = (mtr.limit_max_pin.valid() ? mtr.limit_max_pin.pin : ~0);
		mininvert = mtr.limit_min_pin.inverted();
		maxinvert = mtr.limit_max_pin.inverted();
	}
	avr_buffer[6] = (mtr.sense_pin.valid() ? mtr.sense_pin.pin : ~0);
	avr_buffer[7] = (mtr.step_pin.inverted() ? INVERT_STEP : 0) | (mininvert ? INVERT_LIMIT_MIN : 0) | (maxinvert ? INVERT_LIMIT_MAX : 0);
	prepare_packet(avr_buffer, 8);
	avr_send();
}

static inline void arch_change(bool motors) {
	int old_active_motors = avr_active_motors;
	if (motors) {
		avr_active_motors = 0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			avr_active_motors += spaces[s].num_motors;
		}
	}
	avr_buffer[0] = HWC_SETUP;
	avr_buffer[1] = avr_active_motors;
	for (int i = 0; i < 4; ++i)
		avr_buffer[2 + i] = (hwtime_step >> (8 * i)) & 0xff;
	avr_buffer[6] = led_pin.valid() ? led_pin.pin : ~0;
	avr_buffer[7] = probe_pin.valid() ? probe_pin.pin : ~0;
	avr_buffer[8] = (led_pin.inverted() ? 1 : 0) | (probe_pin.inverted() ? 2 : 0);
	avr_buffer[9] = timeout & 0xff;
	avr_buffer[10] = (timeout >> 8) & 0xff;
	prepare_packet(avr_buffer, 11);
	avr_send();
	if (motors) {
		for (uint8_t s = 0; s < num_spaces; ++s) {
			for (uint8_t m = 0; m < spaces[s].num_motors; ++m) {
				arch_motor_change(s, m);
			}
		}
		for (int m = old_active_motors; m < avr_active_motors; ++m) {
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
	avr_running = true;	// Force arch_stop from setup to do something.
	avr_homing = false;
	avr_filling = false;
	NUM_ANALOG_INPUTS = 0;
	avr_pong = -2;
	avr_limiter_space = -1;
	avr_limiter_motor = 0;
	avr_active_motors = 0;
	// Set up serial port.
	avr_connected = true;
	avr_serial.begin(port, 115200);
	serialdev[1] = &avr_serial;
	arch_reset();
}

static inline void arch_setup_end(char const *run_id) {
	// Get constants.
	avr_buffer[0] = HWC_BEGIN;
	avr_buffer[1] = 10;
	for (int i = 0; i < 8; ++i)
		avr_buffer[2 + i] = run_id[i];
	if (avr_wait_for_reply)
		debug("avr_wait_for_reply already set in begin");
	avr_wait_for_reply = true;
	prepare_packet(avr_buffer, 10);
	avr_send();
	avr_get_reply();
	protocol_version = 0;
	for (uint8_t i = 0; i < sizeof(uint32_t); ++i)
		protocol_version |= int(uint8_t(command[1][2 + i])) << (i * 8);
	NUM_DIGITAL_PINS = command[1][6];
	NUM_ANALOG_INPUTS = command[1][7];
	NUM_MOTORS = command[1][8];
	FRAGMENTS_PER_BUFFER = command[1][9];
	BYTES_PER_FRAGMENT = command[1][10];
	//id[0][:8] + '-' + id[0][8:12] + '-' + id[0][12:16] + '-' + id[0][16:20] + '-' + id[0][20:32]
	for (int i = 0; i < 16; ++i)
		uuid[i] = command[1][11 + i];
	avr_control_queue = new uint8_t[NUM_DIGITAL_PINS * 2];
	avr_in_control_queue = new bool[NUM_DIGITAL_PINS];
	avr_control_queue_length = 0;
	avr_pong = -1;	// Choke on reset again.
	avr_pins = new Avr_pin_t[NUM_DIGITAL_PINS];
	for (int i = 0; i < NUM_DIGITAL_PINS; ++i) {
		avr_pins[i].reset = 3;	// INPUT_NOPULLUP.
		avr_pins[i].state = avr_pins[i].reset;
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

static inline void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin = ~0, bool heater_invert = false, int heater_adctemp = 0, int fan_pin = ~0, bool fan_invert = false, int fan_adctemp = 0) {
	avr_adc_id[thermistor_pin] = id;
	avr_buffer[0] = HWC_ASETUP;
	avr_buffer[1] = thermistor_pin;
	avr_buffer[2] = heater_pin;
	avr_buffer[3] = fan_pin;
	int32_t th, tf;
	if (active) {
		th = (min(0x3fff, max(0, heater_adctemp))) | (heater_invert ? 0x4000 : 0);
		tf = (min(0x3fff, max(0, fan_adctemp))) | (fan_invert ? 0x4000 : 0);
	}
	else {
		th = 0xffff;
		tf = 0xffff;
	}
	//debug("setup adc %d 0x%x 0x%x -> %x %x", id, heater_adctemp, fan_adctemp, th, tf);
	avr_buffer[4] = th & 0xff;
	avr_buffer[5] = (th >> 8) & 0xff;
	avr_buffer[6] = tf & 0xff;
	avr_buffer[7] = (tf >> 8) & 0xff;
	prepare_packet(avr_buffer, 8);
	avr_send();
}

static inline void arch_disconnect() {
	avr_connected = false;
	avr_serial.end();
}

static inline int arch_fds() {
	return avr_connected ? 1 : 0;
}

static inline void arch_reconnect(char *port) {
	avr_connected = true;
	avr_serial.begin(port, 115200);
	avr_serial.write(CMD_NACK);	// Just to be sure.
}
// }}}

// Running hooks. {{{
static inline void arch_addpos(int s, int m, int diff) {
	int mi = m;
	for (uint8_t st = 0; st < s; ++st)
		mi += spaces[st].num_motors;
	avr_pos_offset[mi] -= diff;
	cpdebug(s, m, "arch addpos %d %d %d %d", diff, avr_pos_offset[m], spaces[s].motor[m]->settings[current_fragment].hwcurrent_pos + avr_pos_offset[mi], spaces[s].motor[m]->settings[current_fragment].hwcurrent_pos);
}

static inline void arch_send_fragment(int fragment) {
	if (stopping)
		return;
	avr_buffer[0] = probing ? HWC_START_PROBE : HWC_START_MOVE;
	//debug("send fragment %d %d %d", settings[fragment].fragment_length, fragment, settings[fragment].num_active_motors);
	avr_buffer[1] = settings[fragment].fragment_length;
	avr_buffer[2] = settings[fragment].num_active_motors;
	sending_fragment = settings[fragment].num_active_motors + 1;
	prepare_packet(avr_buffer, 3);
	avr_send();
	int mi = 0;
	avr_filling = true;
	for (int s = 0; !stopping && s < num_spaces; mi += spaces[s++].num_motors) {
		for (uint8_t m = 0; !stopping && m < spaces[s].num_motors; ++m) {
			if (!spaces[s].motor[m]->settings[fragment].active)
				continue;
			cpdebug(s, m, "sending %d %d", fragment, settings[fragment].fragment_length);
			avr_buffer[0] = HWC_MOVE;
			avr_buffer[1] = mi + m;
			for (int i = 0; i < settings[fragment].fragment_length; ++i)
				avr_buffer[2 + i] = (spaces[s].motor[m]->dir_pin.inverted() ? -1 : 1) * spaces[s].motor[m]->settings[fragment].data[i];
			prepare_packet(avr_buffer, 2 + settings[fragment].fragment_length);
			avr_send();
		}
	}
	avr_filling = false;
}

static inline void arch_start_move(int extra) {
	if (out_busy) {
		start_pending = true;
		return;
	}
	if (avr_running || avr_filling || stopping || avr_homing || (running_fragment - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER <= extra + 2)
		return;
	//debug("start move %d %d", sending_fragment, extra);
	avr_running = true;
	avr_buffer[0] = HWC_START;
	prepare_packet(avr_buffer, 1);
	avr_send();
}

static inline bool arch_running() {
	return avr_running;
}

static inline void arch_stop() {
	avr_homing = false;
	if (out_busy) {
		stop_pending = true;
		return;
	}
	if (!avr_running) {
		current_fragment_pos = 0;
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
	abort_move(command[1][1]);
	avr_get_current_pos(2, false);
	current_fragment_pos = 0;
}

static inline void arch_home() {
	avr_homing = true;
	avr_buffer[0] = HWC_HOME;
	int speed = 10000;	// μs/step.
	for (int i = 0; i < 4; ++i)
		avr_buffer[1 + i] = (speed >> (8 * i)) & 0xff;
	int mi = 0;
	for (int s = 0; s < num_spaces; mi += spaces[s++].num_motors) {
		Space &sp = spaces[s];
		for (int m = 0; m < sp.num_motors; ++m) {
			if (abs(int8_t(command[0][2 + mi + m])) <= 1) {
				avr_buffer[5 + mi + m] = (sp.motor[m]->dir_pin.inverted() ? -1 : 1) * command[0][2 + mi + m];
			}
			else {
				debug("invalid code in home: %d", command[0][2 + m]);
				return;
			}
		}
	}
	prepare_packet(avr_buffer, 5 + avr_active_motors);
	avr_send();
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
	debug("opening %s", port);
	fd = open(port, O_RDWR);
	pollfds[2].fd = fd;
	pollfds[2].events = POLLIN | POLLPRI;
	pollfds[2].revents = 0;
	start = 0;
	end_ = 0;
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
			disconnect();
		}
	}
}

void AVRSerial::refill() {
	start = 0;
	end_ = ::read(fd, buffer, sizeof(buffer));
	//debug("refill %d bytes", end_);
	if (end_ < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK)
			debug("read returned error: %s", strerror(errno));
		end_ = 0;
	}
	if (end_ == 0 && pollfds[2].revents) {
		debug("EOF detected on serial port; waiting for reconnect.");
		disconnect();
	}
	pollfds[2].revents = 0;
}

int AVRSerial::read() {
	while (true) {
		if (start == end_)
			refill();
		if (start != end_)
			break;
		debug("eof on input; waiting for reconnect.");
		disconnect();
	}
	int ret = buffer[start++];
#ifdef DEBUG_AVRCOMM
	debug("r %02x", ret & 0xff);
#endif
	return ret;
}
// }}}

#endif

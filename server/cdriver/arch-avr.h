/* arch-avr.h - avr-specific parts for Franklin {{{
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
 * }}} */

// Do this the first time the file is included, the rest the second time.
#ifndef ADCBITS

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
#include <sys/types.h>
#include <sys/socket.h>

// Enable all the parts for a serial connection (which can fail) to the machine.
#define SERIAL
#define ADCBITS 10
#define DATA_TYPE int16_t
#define ARCH_MOTOR DATA_TYPE *avr_data;
#define ARCH_SPACE
#define ARCH_NEW_MOTOR(s, m, base) base[m]->avr_data = new DATA_TYPE[BYTES_PER_FRAGMENT / sizeof(DATA_TYPE)];
#define DATA_DELETE(s, m) delete[] (spaces[s].motor[m]->avr_data)
#define DATA_CLEAR() do { for (int s = 0; s < NUM_SPACES; ++s) for (int m = 0; m < spaces[s].num_motors; ++m) memset((spaces[s].motor[m]->avr_data), 0, BYTES_PER_FRAGMENT); } while (0)
#define DATA_SET(s, m, v) spaces[s].motor[m]->avr_data[current_fragment_pos] = v;
#define SAMPLES_PER_FRAGMENT (BYTES_PER_FRAGMENT / sizeof(DATA_TYPE))
#define ARCH_MAX_FDS 1	// Maximum number of fds for arch-specific purposes.

#else

// Not defines, because they can change value.
EXTERN uint8_t NUM_PINS, NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS, NUM_MOTORS, FRAGMENTS_PER_BUFFER, BYTES_PER_FRAGMENT;
EXTERN int avr_audio;
// }}}

enum Control { // {{{
	CTRL_RESET,
	CTRL_SET,
	CTRL_INPUT,
	CTRL_UNSET,
	CTRL_NOTIFY = 0x40
}; // }}}

enum HWCommands { // {{{
	HWC_BEGIN = 0x00,
	HWC_PING,	// 01
	HWC_SET_UUID,	// 02
	HWC_SETUP,	// 03
	HWC_CONTROL,	// 04
	HWC_MSETUP,	// 05
	HWC_ASETUP,	// 06
	HWC_HOME,	// 07
	HWC_START_MOVE,	// 08
	HWC_START_PROBE,// 09
	HWC_MOVE,	// 0a
	HWC_MOVE_SINGLE,// 0b
	HWC_START,	// 0c
	HWC_STOP,	// 0d
	HWC_ABORT,	// 0e
	HWC_DISCARD,	// 0f
	HWC_GETPIN,	// 10
	HWC_SPI,	// 11
	HWC_PINNAME,	// 12
};

enum HWResponses {
	HWC_READY = 0x10,
	HWC_PONG,	// 11
	HWC_HOMED,	// 12
	HWC_PIN,	// 13
	HWC_STOPPED,	// 14
	HWC_NAMED_PIN,	// 15

	HWC_DONE,	// 16
	HWC_UNDERRUN,	// 17
	HWC_ADC,	// 18
	HWC_LIMIT,	// 19
	HWC_TIMEOUT,	// 1a
	HWC_PINCHANGE,	// 1b
}; // }}}

// Function declarations. {{{
int hwpacketsize(int len, int *available);
void try_send_control();
void arch_had_ack();
void avr_send();
void avr_call1(uint8_t cmd, uint8_t arg);
void avr_get_current_pos(int offset, bool check);
bool hwpacket(int len);
void avr_setup_pin(int pin, int type, int resettype, int extra);
void SET_INPUT(Pin_t _pin);
void SET_INPUT_NOPULLUP(Pin_t _pin);
void RESET(Pin_t _pin);
void SET(Pin_t _pin);
void SET_OUTPUT(Pin_t _pin);
void avr_get_cb_wrap();
void GET(Pin_t _pin, bool _default, void(*cb)(bool));
void avr_send_pin(Pin_t _pin);
void arch_pin_set_reset(Pin_t _pin, char state);
double arch_get_duty(Pin_t _pin);
void arch_set_duty(Pin_t _pin, double duty);
void arch_reset();
void arch_motor_change(uint8_t s, uint8_t sm);
void arch_change(bool motors);
void arch_motors_change();
void arch_globals_change();
void arch_setup_start();
void arch_setup_end();
void arch_set_uuid();
void arch_connect(char const *run_id, char const *port);
void arch_send_pin_name(int pin);
void avr_connect2();
void arch_request_temp(int which);
void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin = ~0, bool heater_invert = false, int heater_adctemp = 0, int heater_limit_l = ~0, int heater_limit_h = ~0, int fan_pin = ~0, bool fan_invert = false, int fan_adctemp = 0, int fan_limit_l = ~0, int fan_limit_h = ~0, double hold_time = 0);
void arch_disconnect();
int arch_fds();
int arch_tick();
void arch_reconnect(char *port);
void arch_addpos(int s, int m, double diff);
void arch_stop(bool fake);
void avr_stop2();
bool arch_send_fragment();
void arch_start_move(int extra);
bool arch_running();
void arch_home();
off_t arch_send_audio(uint8_t *map, off_t pos, off_t max, int motor);
void arch_do_discard();
void arch_discard();
void arch_send_spi(int len, uint8_t *data);
void START_DEBUG();
void DO_DEBUG(char c);
void END_DEBUG();
// }}}

struct AVRSerial : public Serial_t { // {{{
	char buffer[256];
	int start, end_, fd;
	void begin(char const *port);
	void end() { close(fd); }
	void write(char c);
	void refill();
	int read();
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
}; // }}}
struct Avr_pin_t { // {{{
	char state;
	char reset;
	int duty;
}; // }}}

// Declarations of static variables; extern because this is a header file. {{{
EXTERN AVRSerial avr_serial;
EXTERN uint8_t avr_pong;
EXTERN char avr_buffer[256];
EXTERN int avr_limiter_space;
EXTERN int avr_limiter_motor;
EXTERN bool avr_running;
EXTERN Avr_pin_t *avr_pins;
EXTERN double *avr_pos_offset;
EXTERN int avr_active_motors;
EXTERN int *avr_adc_id;
EXTERN uint8_t *avr_control_queue;
EXTERN bool *avr_in_control_queue;
EXTERN int avr_control_queue_length;
EXTERN bool avr_homing;
EXTERN bool avr_filling;
EXTERN void (*avr_get_cb)(bool);
EXTERN bool avr_get_pin_invert;
EXTERN bool avr_stop_fake;
EXTERN void (*avr_cb)();
EXTERN int *avr_pin_name_len;
EXTERN char **avr_pin_name;
EXTERN bool avr_uuid_dirty;
// }}}

#define avr_write_ack(reason) do { \
	/*debug("ack %d: %s", ff_in, reason);*/ \
	write_ack(); \
} while(0)

#ifdef DEFINE_VARIABLES
// Serial port communication. {{{
int hwpacketsize(int len, int *available) { // {{{
	int const arch_packetsize[16] = { 0, 2, 0, 2, 0, 0, 3, 0, 4, 0, 1, 3, -1, -1, -1, -1 };
	if (arch_packetsize[command[1][0] & 0xf] > 0)
		return arch_packetsize[command[1][0] & 0xf];
	if (len < 2) {
		if (*available == 0)
			return 2;	// The data is not available, so this will not trigger the packet to be parsed yet.
		command[1][1] = serialdev[1]->read();
		command_end[1] += 1;
		*available -= 1;
	}
	switch (command[1][0] & 0x1f) {
	case HWC_READY:
		return command[1][1];
	case HWC_HOMED:
		return 2 + 4 * command[1][1];
	case HWC_STOPPED:
		return 3 + 4 * command[1][1];
	case HWC_NAMED_PIN:
		return 2 + command[1][1];
	case HWC_LIMIT:
	case HWC_UNDERRUN:
		return 4 + 4 * command[1][1];
	default:
		debug("ignoring invalid serial command %x", command[1][0]);
		return 1;	// Parse and fail immediately.
	}
} // }}}

void try_send_control() { // {{{
	if (!connected || preparing || out_busy >= 3 || avr_control_queue_length == 0)
		return;
	avr_control_queue_length -= 1;
	avr_buffer[0] = HWC_CONTROL;
	avr_buffer[1] = avr_control_queue[avr_control_queue_length * 3];
	avr_buffer[2] = avr_control_queue[avr_control_queue_length * 3 + 1];
	avr_buffer[3] = avr_control_queue[avr_control_queue_length * 3 + 2];
	avr_in_control_queue[avr_control_queue[avr_control_queue_length * 3]] = false;
	prepare_packet(avr_buffer, 4);
	avr_send();
} // }}}

void arch_had_ack() { // {{{
	if (out_busy == 0)
		try_send_control();
} // }}}

void avr_send() { // {{{
	//debug("avr_send");
	if (!connected) {
		debug("send called while not connected");
		abort();
	}
	while (out_busy >= 3) {
		//debug("avr send");
		poll(&pollfds[BASE_FDS], 1, -1);
		serial(1);
	}
	serial_cb[out_busy] = avr_cb;
	avr_cb = NULL;
	send_packet();
	if (out_busy < 3)
		try_send_control();
} // }}}

void avr_call1(uint8_t cmd, uint8_t arg) { // {{{
	avr_buffer[0] = cmd;
	avr_buffer[1] = arg;
	prepare_packet(avr_buffer, 2);
	avr_send();
} // }}}

double arch_round_pos(int s, int m, double pos) { // {{{
	if (s >= NUM_SPACES)
		return pos;
	int mi = 0;
	for (int ts = 0; ts < s; ++ts) mi += spaces[ts].num_motors;
	if (mi + m >= NUM_MOTORS)
		return pos;
	return round(pos + avr_pos_offset[mi + m]) - avr_pos_offset[mi + m];
} // }}}

void avr_get_current_pos(int offset, bool check) { // {{{
	int mi = 0;
	for (int ts = 0; ts < NUM_SPACES; mi += spaces[ts++].num_motors) {
		for (int tm = 0; tm < spaces[ts].num_motors; ++tm) {
			double old = spaces[ts].motor[tm]->settings.current_pos;
			cpdebug(ts, tm, "cpb offset %f raw %f hwpos %f", avr_pos_offset[tm + mi], spaces[ts].motor[tm]->settings.current_pos, spaces[ts].motor[tm]->settings.current_pos + avr_pos_offset[tm + mi]);
			double p = 0;
			for (int i = 0; i < 4; ++i) {
				p += int(uint8_t(command[1][offset + 4 * (tm + mi) + i])) << (i * 8);
			}
			if (spaces[ts].motor[tm]->dir_pin.inverted())
				p *= -1;
			p -= avr_pos_offset[tm + mi];
			cpdebug(ts, tm, "cpa offset %f raw %f hwpos %f", avr_pos_offset[tm + mi], p, p + avr_pos_offset[tm + mi]);
			cpdebug(ts, tm, "getpos offset %f diff %d", avr_pos_offset[tm + mi], arch_round_pos(ts, tm, p) - arch_round_pos(ts, tm, old));
			if (check) {
				if (arch_round_pos(ts, tm, old) != arch_round_pos(ts, tm, p)) {
					if (moving_to_current == 1)
						moving_to_current = 2;
					else {
						debug("WARNING: position for %d %d out of sync!  old = %f, new = %f offset = %f", ts, tm, old, p, avr_pos_offset[tm + mi]);
						//abort();
						spaces[ts].motor[tm]->settings.current_pos = p;
					}
				}
				else {
					//debug("Check: position for %d %d in sync, old = %f, new = %f offset = %f", ts, tm, old, p, avr_pos_offset[tm + mi]);
				}
			}
			else {
				// Motor positions were unknown; no check, just update position.
				if (arch_round_pos(ts, tm, old) != arch_round_pos(ts, tm, p)) {
					cpdebug(ts, tm, "update current pos from %f to %f", spaces[ts].motor[tm]->settings.current_pos, p);
					spaces[ts].motor[tm]->settings.current_pos = p;
				}
			}
		}
	}
} // }}}

bool hwpacket(int len) { // {{{
	(void)&len;
	// Handle data in command[1].
#if 0
	if (command[1][0] != HWC_ADC) {
		fprintf(stderr, "packet received:");
		for (uint8_t i = 0; i < len; ++i)
			fprintf(stderr, " %02x", command[1][i]);
		fprintf(stderr, "\n");
	}
#endif
	switch (command[1][0]) {
	case HWC_LIMIT: // {{{
	{
		uint8_t which = command[1][2];
		if (which > NUM_MOTORS) {
			if (initialized) {
				debug("cdriver: Invalid limit for avr motor %d", which);
				abort();
			}
			avr_write_ack("pre-limit");
			return false;
		}
		avr_write_ack("limit");
		avr_homing = false;
		abort_move(int8_t(command[1][3] / 2));
		avr_get_current_pos(4, false);
		if (spaces[0].num_axes > 0)
			cpdebug(0, 0, "ending hwpos %f", spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0]);
		double pos;
		int s, m = -1;
		if (which >= avr_active_motors) {
			s = -1;
			m = -1;
			pos = NAN;
		}
		else {
			for (s = 0; s < NUM_SPACES; ++s) {
				if (which < spaces[s].num_motors) {
					m = which;
					break;
				}
				which -= spaces[s].num_motors;
			}
			fcpdebug(s, m, "limit");
			pos = spaces[s].motor[m]->settings.current_pos / spaces[s].motor[m]->steps_per_unit;
		}
		//debug("cbs after current cleared %d for limit", cbs_after_current_move);
		cbs_after_current_move = 0;
		avr_running = false;
		stopping = 2;
		send_host(CMD_LIMIT, s, m, pos);
		//debug("limit done");
		return false;
	} // }}}
	case HWC_PONG: // {{{
	{
		avr_pong = command[1][1];
		avr_write_ack("pong");
		return false;
	} // }}}
	case HWC_ADC: // {{{
	{
		int pin = command[1][1];
		if (pin < 0 || pin >= NUM_ANALOG_INPUTS) {
			if (avr_pong == 255)
				debug("invalid adc %d received", pin);
			avr_write_ack("invalid adc");
			return false;
		}
		int adc = (command[1][2] & 0xff) | ((command[1][3] & 0xff) << 8);
		avr_write_ack("adc");
		if (pin < NUM_ANALOG_INPUTS && avr_adc_id[pin] >= 0 && avr_adc_id[pin] < num_temps)
			handle_temp(avr_adc_id[pin], adc);
		return false;
	} // }}}
	case HWC_UNDERRUN: // {{{
	{
		if (host_block) {
			// STOP was sent; ignore UNDERRUN.
			avr_write_ack("stopped underrun");
			//debug("underrun check2 %d %d %d", sending_fragment, current_fragment, running_fragment);
			return false;
		}
		if (!avr_running) {
			debug("unexpected underrun?");
			//abort();
		}
		avr_running = false;
		if (computing_move) {
			//debug("underrun %d %d %d", sending_fragment, current_fragment, running_fragment);
			if (!sending_fragment && (current_fragment - (running_fragment + command[1][2] + command[1][3]) + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER > 1)
				arch_start_move(command[1][2]);
			// Buffer is too slow with refilling; this will fix itself.
		}
		else {
			// Only overwrite current position if the new value is correct.
			//debug("underrun ok current=%d running=%d computing_move=%d sending=%d pending=%d finishing=%d transmitting=%d", current_fragment, running_fragment, computing_move, sending_fragment, command[1][3], run_file_finishing, transmitting_fragment);
			if (!sending_fragment && !transmitting_fragment) {
				if (command[1][3] == 0) {
					avr_get_current_pos(4, true);
					if (run_file_finishing) {
						send_host(CMD_FILE_DONE);
						abort_run_file();
					}
				}
			}
			//debug("underrun check %d %d %d", sending_fragment, current_fragment, running_fragment);
		}
	} // }}}
		// Fall through.
	case HWC_DONE: // {{{
	{
		if (host_block) {
			// STOP was sent; ignore DONE.
			//debug("done during stop");
			avr_write_ack("stopped done");
			return false;
		}
		int offset = command[1][0] == HWC_UNDERRUN ? 1 : 0;
		//debug("done: %d pending %d sending %d current %d running %d", command[1][offset + 1], command[1][offset + 2], sending_fragment, current_fragment, running_fragment);
		if (FRAGMENTS_PER_BUFFER == 0) {
			//debug("Done received while fragments per buffer is zero");
			avr_write_ack("invalid done");
			return false;
		}
		first_fragment = -1;
		int cbs = 0;
		//debug("done: %d pending %d sending %d preparing %d current %d running %d", command[1][offset + 1], command[1][offset + 2], sending_fragment, preparing, current_fragment, running_fragment);
		for (int i = 0; i < command[1][offset + 1]; ++i) {
			int f = (running_fragment + i) % FRAGMENTS_PER_BUFFER;
			//debug("fragment %d: cbs=%d current=%d", f, history[f].cbs, current_fragment);
			cbs += history[f].cbs;
			history[f].cbs = 0;
		}
		if (!avr_running) {
			cbs += cbs_after_current_move;
			cbs_after_current_move = 0;
		}
		//debug("cbs: %d after current %d computing %d", cbs, cbs_after_current_move, computing_move);
		if (cbs && !host_block)
			send_host(CMD_MOVECB, cbs);
		if ((current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER + 1 < command[1][offset + 1] + command[1][offset + 2]) {
			debug("Done count %d+%d higher than busy fragments %d+1; clipping", command[1][offset + 1], command[1][offset + 2], (current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER);
			avr_write_ack("invalid done");
			//abort();
		}
		else
			avr_write_ack("done");
		running_fragment = (running_fragment + command[1][offset + 1]) % FRAGMENTS_PER_BUFFER;
		//debug("running -> %x", running_fragment);
		if (current_fragment == running_fragment && command[1][0] == HWC_DONE) {
			debug("Done received, but should be underrun");
			//abort();
		}
		if (out_busy < 3)
			buffer_refill();
		//else
		//	debug("no refill");
		run_file_fill_queue();
		return false;
	} // }}}
	case HWC_HOMED: // {{{
	{
		if (!avr_homing) {
			if (initialized)
				abort();
			avr_write_ack("pre-homed");
			return false;
		}
		computing_move = false;
		avr_homing = false;
		avr_get_current_pos(2, false);
		avr_write_ack("homed");
		send_host(CMD_HOMED);
		return false;
	} // }}}
	case HWC_TIMEOUT: // {{{
	{
		avr_write_ack("timeout");
		for (int i = 0; i < NUM_DIGITAL_PINS; ++i)
			avr_pins[i].state = avr_pins[i].reset;
		for (int i = 0; i < num_gpios; ++i)
			gpios[i].state = gpios[i].reset;
		motors_busy = false;
		// Everything has shut down; reset pins to normal (but inactive).
		arch_motors_change();
		for (int s = 0; s < NUM_SPACES; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m) {
				RESET(spaces[s].motor[m]->step_pin);
				RESET(spaces[s].motor[m]->dir_pin);
				RESET(spaces[s].motor[m]->enable_pin);
				spaces[s].motor[m]->settings.current_pos = 0;
			}
		}
		for (int m = 0; m < NUM_MOTORS; ++m)
			avr_pos_offset[m] = 0;
		for (int t = 0; t < num_temps; ++t)
			settemp(t, NAN);
		send_host(CMD_TIMEOUT);
		return false;
	} // }}}
	case HWC_PINCHANGE: // {{{
	{
		avr_write_ack("pinchange");
		for (int i = 0; i < num_gpios; ++i) {
			if (gpios[i].pin.pin == command[1][1])
				send_host(CMD_PINCHANGE, i, gpios[i].pin.inverted() ? !command[1][2] : command[1][2]);
		}
		return false;
	} // }}}
	default: // {{{
	{
		if (expected_replies <= 0) {
			debug("Received unexpected reply %x", command[1][0]);
			avr_write_ack("unexpected reply");
		}
		else
			return true;
		return false;
	} // }}}
	}
} // }}}
// }}}

// Hardware interface {{{
void avr_setup_pin(int pin, int type, int resettype, int extra) { // {{{
	if (!connected)
		return;
	if (pin < 0 || pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin to set up");
		abort();
	}
	//debug("pin %d type %d reset %d extra %d", pin, type, resettype, extra);
	if (avr_in_control_queue[pin])
	{
		for (int i = 0; i < avr_control_queue_length; ++i) {
			if (avr_control_queue[i * 3] != pin)
				continue;
			avr_control_queue[i * 3 + 1] = type | (resettype << 2) | extra;
			avr_control_queue[i * 3 + 2] = avr_pins[pin].duty;
			return;
		}
	}
	avr_control_queue[avr_control_queue_length * 3] = pin;
	avr_control_queue[avr_control_queue_length * 3 + 1] = type | (resettype << 2) | extra;
	avr_control_queue[avr_control_queue_length * 3 + 2] = avr_pins[pin].duty;
	avr_control_queue_length += 1;
	avr_in_control_queue[pin] = true;
	try_send_control();
} // }}}

void SET_INPUT(Pin_t _pin) { // {{{
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state == 2)
		return;
	avr_pins[_pin.pin].state = 2;
	avr_setup_pin(_pin.pin, CTRL_INPUT, avr_pins[_pin.pin].reset, CTRL_NOTIFY);
} // }}}

void SET_INPUT_NOPULLUP(Pin_t _pin) { // {{{
	if (!_pin.valid() || _pin.pin >= NUM_DIGITAL_PINS)
		return;
	if (avr_pins[_pin.pin].state == 3)
		return;
	avr_pins[_pin.pin].state = 3;
	avr_setup_pin(_pin.pin, CTRL_UNSET, avr_pins[_pin.pin].reset, CTRL_NOTIFY);
} // }}}

void RESET(Pin_t _pin) { // {{{
	if (!_pin.valid())
		return;
	//debug("reset %d", _pin.pin);
	if (avr_pins[_pin.pin].state == 0)
		return;
	avr_pins[_pin.pin].state = 0;
	if (_pin.inverted())
		avr_setup_pin(_pin.pin, CTRL_SET, avr_pins[_pin.pin].reset < 2 ? 1 - avr_pins[_pin.pin].reset : avr_pins[_pin.pin].reset, 0);
	else
		avr_setup_pin(_pin.pin, CTRL_RESET, avr_pins[_pin.pin].reset, 0);
} // }}}

void SET(Pin_t _pin) { // {{{
	if (!_pin.valid())
		return;
	//debug("set %d", _pin.pin);
	if (avr_pins[_pin.pin].state == 1)
		return;
	avr_pins[_pin.pin].state = 1;
	if (_pin.inverted())
		avr_setup_pin(_pin.pin, CTRL_RESET, avr_pins[_pin.pin].reset < 2 ? 1 - avr_pins[_pin.pin].reset : avr_pins[_pin.pin].reset, 0);
	else
		avr_setup_pin(_pin.pin, CTRL_SET, avr_pins[_pin.pin].reset, 0);
} // }}}

void SET_OUTPUT(Pin_t _pin) { // {{{
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state < 2)
		return;
	RESET(_pin);
} // }}}

void avr_get_cb_wrap() { // {{{
	void (*cb)(bool) = avr_get_cb;
	avr_get_cb = NULL;
	bool arg = avr_get_pin_invert ^ command[1][1];
	avr_write_ack("get");
	cb(arg);
} // }}}

void GET(Pin_t _pin, bool _default, void(*cb)(bool)) { // {{{
	if (!connected || !_pin.valid())
		cb(_default);
	wait_for_reply[expected_replies++] = avr_get_cb_wrap;
	avr_get_cb = cb;
	avr_get_pin_invert = _pin.inverted();
	avr_call1(HWC_GETPIN, _pin.pin);
} // }}}

void avr_send_pin(Pin_t _pin) { // {{{
	int s, r;
	if (_pin.inverted()) {
		s = avr_pins[_pin.pin].state < 2 ? 1 - avr_pins[_pin.pin].state : avr_pins[_pin.pin].state;
		r = avr_pins[_pin.pin].reset < 2 ? 1 - avr_pins[_pin.pin].reset : avr_pins[_pin.pin].reset;
	}
	else {
		s = avr_pins[_pin.pin].state;
		r = avr_pins[_pin.pin].reset;
	}
	avr_setup_pin(_pin.pin, s, r, avr_pins[_pin.pin].reset != 2 ? 0 : CTRL_NOTIFY);
} // }}}

void arch_pin_set_reset(Pin_t _pin, char state) { // {{{
	if (!_pin.valid() || _pin.pin >= NUM_DIGITAL_PINS) {
		// Ignore requests to set reset state of invalid and analog pins.
		return;
	}
	if (avr_pins[_pin.pin].reset == state)
		return;
	avr_pins[_pin.pin].reset = state;
	avr_send_pin(_pin);
} // }}}

double arch_get_duty(Pin_t _pin) { // {{{
	if (!connected)
		return 1;
	if (_pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_get_duty: %d (max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return 1;
	}
	return (avr_pins[_pin.pin].duty + 1) / 256.;
} // }}}

void arch_set_duty(Pin_t _pin, double duty) { // {{{
	if (!connected)
		return;
	if (_pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_set_duty: %d (max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return;
	}
	int hwduty = round(duty * 256) - 1;
	if (hwduty < 0)
		hwduty = 0;
	if (hwduty > 255) {
		debug("invalid duty value %d; clipping to 255.", hwduty);
		hwduty = 255;
	}
	if (hwduty != avr_pins[_pin.pin].duty) {
		avr_pins[_pin.pin].duty = hwduty;
		avr_send_pin(_pin);
	}
} // }}}
// }}}

// Setup hooks. {{{
void arch_reset() { // {{{
	// Initialize connection.
	if (avr_pong == 7) {
		debug("reset ignored");
		return;
	}
	// Wait for reset to complete.
	sleep(2);
	avr_serial.write(CMD_ACK1);
	avr_serial.write(CMD_ACK2);
	avr_serial.write(CMD_ACK3);
	avr_serial.write(CMD_ACK0);
	avr_serial.write(CMD_ACK1);
	avr_serial.write(CMD_ACK2);
	avr_serial.write(CMD_ACK3);
	avr_serial.write(CMD_STALLACK);
	// Just in case the controller was reset: reclaim port by requesting ID.
	avr_serial.write(CMD_ID);
	avr_call1(HWC_PING, 0);
	avr_call1(HWC_PING, 1);
	avr_call1(HWC_PING, 2);
	avr_call1(HWC_PING, 3);
	avr_serial.write(CMD_ID);
	avr_call1(HWC_PING, 4);
	avr_call1(HWC_PING, 5);
	avr_call1(HWC_PING, 6);
	avr_call1(HWC_PING, 7);
	int32_t before = millis();
	while (avr_pong != 7 && millis() - before < 2000) {
		//debug("avr pongwait %d", avr_pong);
		pollfds[BASE_FDS].revents = 0;
		poll(&pollfds[BASE_FDS], 1, 1);
		serial(1);
	}
	if (avr_pong != 7) {
		debug("no pong seen; giving up.\n");
		abort();
	}
	arch_change(true);
	if (avr_uuid_dirty) {
		arch_set_uuid();
		avr_uuid_dirty = false;
	}
} // }}}

enum MotorFlags { // {{{
	LIMIT			= 0x01,
	INVERT_LIMIT_MIN	= 0x02,
	INVERT_LIMIT_MAX	= 0x04,
	INVERT_STEP		= 0x40
}; // }}}

void arch_motor_change(uint8_t s, uint8_t sm) { // {{{
	Motor &mtr = *spaces[s].motor[sm];
	if (!connected)
		return;
	uint8_t m = sm;
	for (uint8_t st = 0; st < s; ++st)
		m += spaces[st].num_motors;
	avr_buffer[0] = HWC_MSETUP;
	avr_buffer[1] = m;
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
	int fm = space_types[spaces[s].type].follow(&spaces[s], sm);
	if (fm >= 0) {
		int fs = fm >> 8;
		fm &= 0x7f;
		if (spaces[s].motor[sm]->dir_pin.inverted() ^ spaces[fs].motor[fm]->dir_pin.inverted())
			fm |= 0x80;
		for (int fi = 0; fi < fs; ++fi)
			fm += spaces[fi].num_motors;
		avr_buffer[6] = fm;
	}
	else
		avr_buffer[6] = 0xff;
	avr_buffer[7] = (mtr.step_pin.inverted() ? INVERT_STEP : 0) | (mininvert ? INVERT_LIMIT_MIN : 0) | (maxinvert ? INVERT_LIMIT_MAX : 0);
	prepare_packet(avr_buffer, 8);
	avr_send();
} // }}}

void arch_change(bool motors) { // {{{
	int old_active_motors = avr_active_motors;
	if (connected) {
		if (motors) {
			avr_active_motors = 0;
			for (uint8_t s = 0; s < NUM_SPACES; ++s) {
				avr_active_motors += spaces[s].num_motors;
			}
		}
		avr_buffer[0] = HWC_SETUP;
		if (avr_audio >= 0) {
			avr_buffer[1] = NUM_MOTORS;
			for (int i = 0; i < 4; ++i)
				avr_buffer[2 + i] = (audio_hwtime_step >> (8 * i)) & 0xff;
			avr_buffer[13] = avr_audio;
		}
		else {
			avr_buffer[1] = avr_active_motors;
			for (int i = 0; i < 4; ++i)
				avr_buffer[2 + i] = (hwtime_step >> (8 * i)) & 0xff;
			avr_buffer[13] = 0xff;
		}
		avr_buffer[6] = led_pin.valid() ? led_pin.pin : ~0;
		avr_buffer[7] = stop_pin.valid() ? stop_pin.pin : ~0;
		avr_buffer[8] = probe_pin.valid() ? probe_pin.pin : ~0;
		avr_buffer[9] = (led_pin.inverted() ? 1 : 0) | (probe_pin.inverted() ? 2 : 0) | (stop_pin.inverted() ? 4 : 0) | (spiss_pin.inverted() ? 8 : 0);
		avr_buffer[10] = timeout & 0xff;
		avr_buffer[11] = (timeout >> 8) & 0xff;
		avr_buffer[12] = spiss_pin.valid() ? spiss_pin.pin : ~0;
		prepare_packet(avr_buffer, 14);
		avr_send();
	}
	if (motors) {
		for (uint8_t s = 0; s < NUM_SPACES; ++s) {
			for (uint8_t m = 0; m < spaces[s].num_motors; ++m) {
				arch_motor_change(s, m);
			}
		}
		if (connected) {
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
} // }}}

void arch_motors_change() { // {{{
	if (preparing || out_busy >= 3) {
		change_pending = true;
		return;
	}
	change_pending = false;
	arch_change(true);
} // }}}

void arch_globals_change() { // {{{
	arch_change(false);
} // }}}

void arch_setup_start() { // {{{
	// Set up arch variables.
	avr_running = false;
	avr_homing = false;
	avr_filling = false;
	NUM_PINS = 0;
	NUM_ANALOG_INPUTS = 0;
	avr_pong = 254;
	avr_limiter_space = -1;
	avr_limiter_motor = 0;
	avr_active_motors = 0;
	avr_audio = -1;
	avr_uuid_dirty = false;
	// Set up serial port.
	connected = false;
	serialdev[1] = &avr_serial;
} // }}}

void arch_setup_end() {
	// Nothing to do.
}

void arch_set_uuid() { // {{{
	if (!connected) {
		avr_uuid_dirty = true;
		return;
	}
	avr_buffer[0] = HWC_SET_UUID;
	for (uint8_t i = 0; i < UUID_SIZE; ++i)
		avr_buffer[1 + i] = uuid[i];
	prepare_packet(avr_buffer, 1 + UUID_SIZE);
	avr_send();
} // }}}

static void avr_connect3();
static int avr_next_pin_name;

void arch_send_pin_name(int pin) { // {{{
	memcpy(datastore, avr_pin_name[pin], avr_pin_name_len[pin]);
	send_host(CMD_PINNAME, pin, 0, 0, 0, avr_pin_name_len[pin]);
} // }}}

static void avr_connect4() { // {{{
	while (out_busy >= 3) {
		poll(&pollfds[2], 1, -1);
		serial(1);
	}
	avr_pin_name_len[avr_next_pin_name] = command[1][1] + 1;
	avr_pin_name[avr_next_pin_name] = new char[command[1][1] + 1];
	memcpy(&avr_pin_name[avr_next_pin_name][1], &command[1][2], command[1][1]);
	if (avr_next_pin_name < NUM_DIGITAL_PINS)
		avr_pin_name[avr_next_pin_name][0] = 7;
	else
		avr_pin_name[avr_next_pin_name][0] = 8;
	avr_write_ack("pin name");
	arch_send_pin_name(avr_next_pin_name);
	avr_next_pin_name += 1;
	avr_connect3();
} // }}}

static void avr_connect3() { // {{{
	//debug("sending pin %d name", avr_next_pin_name);
	if (avr_next_pin_name >= NUM_PINS) {
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int m = 0; m < sp.num_motors; ++m) {
				delete[] sp.motor[m]->avr_data;
				sp.motor[m]->avr_data = new DATA_TYPE[BYTES_PER_FRAGMENT / sizeof(DATA_TYPE)];
			}
		}
		connect_end();
		return;
	}
	avr_buffer[0] = HWC_PINNAME;
	avr_buffer[1] = avr_next_pin_name < NUM_DIGITAL_PINS ? avr_next_pin_name : (avr_next_pin_name - NUM_DIGITAL_PINS) | 0x80;
	wait_for_reply[expected_replies++] = avr_connect4;
	prepare_packet(avr_buffer, 2);
	avr_send();
} // }}}

void avr_connect2() { // {{{
	protocol_version = 0;
	for (uint8_t i = 0; i < sizeof(uint32_t); ++i)
		protocol_version |= int(uint8_t(command[1][2 + i])) << (i * 8);
	NUM_DIGITAL_PINS = command[1][6];
	NUM_ANALOG_INPUTS = command[1][7];
	NUM_PINS = NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS;
	NUM_MOTORS = command[1][8];
	FRAGMENTS_PER_BUFFER = command[1][9];
	BYTES_PER_FRAGMENT = command[1][10];
	//id[0][:8] + '-' + id[0][8:12] + '-' + id[0][12:16] + '-' + id[0][16:20] + '-' + id[0][20:32]
	for (int i = 0; i < UUID_SIZE; ++i)
		uuid[i] = command[1][11 + i];
	avr_write_ack("setup");
	avr_control_queue = new uint8_t[NUM_DIGITAL_PINS * 3];
	avr_in_control_queue = new bool[NUM_DIGITAL_PINS];
	avr_control_queue_length = 0;
	avr_pong = 255;	// Choke on reset again.
	avr_pins = new Avr_pin_t[NUM_DIGITAL_PINS];
	for (int i = 0; i < NUM_DIGITAL_PINS; ++i) {
		avr_pins[i].reset = 3;	// INPUT_NOPULLUP.
		avr_pins[i].state = avr_pins[i].reset;
		avr_pins[i].duty = 255;
		avr_in_control_queue[i] = false;
	}
	avr_adc_id = new int[NUM_ANALOG_INPUTS];
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i)
		avr_adc_id[i] = ~0;
	avr_pos_offset = new double[NUM_MOTORS];
	for (int m = 0; m < NUM_MOTORS; ++m)
		avr_pos_offset[m] = 0;
	avr_next_pin_name = 0;
	avr_pin_name_len = new int[NUM_PINS];
	avr_pin_name = new char *[NUM_PINS];
	avr_connect3();
} // }}}

void arch_connect(char const *run_id, char const *port) { // {{{
	connected = true;
	avr_serial.begin(port);
	if (!connected)
		return;
	arch_reset();
	// Get constants.
	avr_buffer[0] = HWC_BEGIN;
	avr_buffer[1] = 10;
	for (int i = 0; i < ID_SIZE; ++i)
		avr_buffer[2 + i] = run_id[i];
	wait_for_reply[expected_replies++] = avr_connect2;
	prepare_packet(avr_buffer, 10);
	avr_send();
} // }}}

void arch_request_temp(int which) { // {{{
	if (connected && which >= 0 && which < num_temps && temps[which].thermistor_pin.pin >= NUM_DIGITAL_PINS && temps[which].thermistor_pin.pin < NUM_PINS) {
		requested_temp = which;
		return;
	}
	requested_temp = ~0;
	send_host(CMD_TEMP, 0, 0, NAN);
} // }}}

void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin, bool heater_invert, int heater_adctemp, int heater_limit_l, int heater_limit_h, int fan_pin, bool fan_invert, int fan_adctemp, int fan_limit_l, int fan_limit_h, double hold_time) { // {{{
	if (!connected)
		return;
	if (thermistor_pin < NUM_DIGITAL_PINS || thermistor_pin >= NUM_PINS) {
		debug("setup for invalid adc %d requested", thermistor_pin);
		return;
	}
	// Make sure the controls for the heater and fan have been sent, otherwise they override this.
	try_send_control();
	while (out_busy >= 3) {
		//debug("avr send");
		poll(&pollfds[BASE_FDS], 1, -1);
		serial(1);
		try_send_control();
	}
	thermistor_pin -= NUM_DIGITAL_PINS;
	avr_adc_id[thermistor_pin] = id;
	avr_buffer[0] = HWC_ASETUP;
	avr_buffer[1] = thermistor_pin;
	avr_buffer[2] = heater_pin;
	avr_buffer[3] = fan_pin;
	int32_t th, tf, lh[2], lf[2];
	if (active) {
		int hi = heater_invert ? 0x4000 : 0;
		int fi = fan_invert ? 0x4000 : 0;
		th = (min(0x3fff, max(0, heater_adctemp))) | hi;
		tf = (min(0x3fff, max(0, fan_adctemp))) | fi;
		lh[0] = (heater_limit_l & 0x3fff) | hi;
		lh[1] = (heater_limit_h & 0x3fff) | hi;
		lf[0] = (fan_limit_l & 0x3fff) | fi;
		lf[1] = (fan_limit_h & 0x3fff) | fi;
	}
	else {
		if (id == requested_temp) {
			send_host(CMD_TEMP, 0, 0, NAN);
			requested_temp = ~0;
		}
		th = 0xffff;
		tf = 0xffff;
		lh[0] = 0x3fff;
		lh[1] = 0x3fff;
		lf[0] = 0x3fff;
		lf[1] = 0x3fff;
	}
	//debug("setup adc %d 0x%x 0x%x -> %x %x", id, heater_adctemp, fan_adctemp, th, tf);
	avr_buffer[4] = lh[0] & 0xff;
	avr_buffer[5] = (lh[0] >> 8) & 0xff;
	avr_buffer[6] = lf[0] & 0xff;
	avr_buffer[7] = (lf[0] >> 8) & 0xff;
	avr_buffer[8] = lh[1] & 0xff;
	avr_buffer[9] = (lh[1] >> 8) & 0xff;
	avr_buffer[10] = lf[1] & 0xff;
	avr_buffer[11] = (lf[1] >> 8) & 0xff;
	avr_buffer[12] = th & 0xff;
	avr_buffer[13] = (th >> 8) & 0xff;
	avr_buffer[14] = tf & 0xff;
	avr_buffer[15] = (tf >> 8) & 0xff;
	uint16_t hold_time_ms = hold_time * 1000;
	avr_buffer[16] = hold_time_ms & 0xff;
	avr_buffer[17] = (hold_time_ms >> 8) & 0xff;
	prepare_packet(avr_buffer, 18);
	avr_send();
} // }}}

void arch_disconnect() { // {{{
	connected = false;
	avr_serial.end();
	if (requested_temp != uint8_t(~0)) {
		requested_temp = ~0;
		send_host(CMD_TEMP, 0, 0, NAN);
	}
} // }}}

int arch_fds() { // {{{
	return connected ? 1 : 0;
} // }}}

void arch_reconnect(char *port) { // {{{
	connected = true;
	avr_serial.begin(port);
	if (!connected)
		return;
	for (int i = 0; i < 4; ++i)
		avr_serial.write(cmd_nack[i]);	// Just to be sure.
} // }}}
// }}}

// Running hooks. {{{
int arch_tick() { // {{{
	if (connected) {
		serial(1);
		return 500;
	}
	return -1;
} // }}}

void arch_addpos(int s, int m, double diff) { // {{{
	if (s >= NUM_SPACES)
		return;
	int mi = 0;
	for (uint8_t st = 0; st < s; ++st)
		mi += spaces[st].num_motors;
	if (mi + m >= NUM_MOTORS)
		return;
	if (!isnan(diff))
		avr_pos_offset[mi + m] -= diff;
	else
		abort();
	//debug("addpos %d %d %f -> %f", s, m, diff, avr_pos_offset[mi]);
	cpdebug(s, m, "arch addpos diff %f offset %f raw %f pos %f", diff, avr_pos_offset[mi], spaces[s].motor[m]->settings.current_pos + avr_pos_offset[mi], spaces[s].motor[m]->settings.current_pos);
} // }}}

void arch_stop(bool fake) { // {{{
	if (!connected) {
		stop_pending = true;
		return;
	}
	//debug("blocking host");
	host_block = true;
	if (preparing || out_busy >= 3) {
		//debug("not yet stopping");
		stop_pending = true;
		return;
	}
	stop_pending = false;
	if (!avr_running && !avr_homing) {
		//debug("not running, so not stopping");
		current_fragment_pos = 0;
		computing_move = false;	// Not running, but preparations could have started.
		//debug("no longer blocking host");
		host_block = false;
		return;
	}
	avr_running = false;
	avr_homing = false;
	avr_buffer[0] = HWC_STOP;
	wait_for_reply[expected_replies++] = avr_stop2;
	avr_stop_fake = fake;
	prepare_packet(avr_buffer, 1);
	avr_send();
} // }}}

void avr_stop2() { // {{{
	if (!avr_stop_fake)
		abort_move(command[1][2] / 2);
	avr_get_current_pos(3, false);
	current_fragment = running_fragment;
	//debug("current_fragment = running_fragment; %d", current_fragment);
	current_fragment_pos = 0;
	num_active_motors = 0;
	//debug("no longer blocking host 2");
	host_block = false;
	avr_write_ack("stop");
	serial(0);	// Handle any data that was refused before.
} // }}}

static void avr_sent_fragment() { // {{{
	if (sending_fragment == 0) {
		debug("calling avr_sent_fragment with zero sending_fragment");
		return;
	}
	if (stopping)
		return;
	sending_fragment -= 1;
	if (sending_fragment == 0) {
	}
} // }}}

bool arch_send_fragment() { // {{{
	if (!connected || host_block || stopping || discard_pending || stop_pending) {
		//debug("not sending arch frag %d %d %d %d", host_block, stopping, discard_pending, stop_pending);
		return false;
	}
	while (out_busy >= 3) {
		poll(&pollfds[BASE_FDS], 1, -1);
		serial(1);
	}
	if (stop_pending || discard_pending)
		return false;
	avr_buffer[0] = settings.probing ? HWC_START_PROBE : HWC_START_MOVE;
	//debug("send fragment current-fragment-pos=%d current-fragment=%d active-moters=%d running=%d num-running=0x%x", current_fragment_pos, current_fragment, num_active_motors, running_fragment, (current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER);
	avr_buffer[1] = current_fragment_pos * 2;
	avr_buffer[2] = num_active_motors;
	sending_fragment = num_active_motors + 1;
	if (prepare_packet(avr_buffer, 3)) {
		transmitting_fragment = true;
		avr_cb = &avr_sent_fragment;
		avr_send();
		int mi = 0;
		avr_filling = true;
		int cfp = current_fragment_pos;
		for (int s = 0; !host_block && !stopping && !discard_pending && !stop_pending && s < NUM_SPACES; mi += spaces[s++].num_motors) {
			for (uint8_t m = 0; !host_block && !stopping && !discard_pending && !stop_pending && m < spaces[s].num_motors; ++m) {
				if (!spaces[s].motor[m]->active)
					continue;
				cpdebug(s, m, "sending %d %d", current_fragment, current_fragment_pos);
				//debug("sending %d %d cf %d cp 0x%x", s, m, current_fragment, current_fragment_pos);
				while (out_busy >= 3) {
					poll(&pollfds[BASE_FDS], 1, -1);
					serial(1);
				}
				if (stop_pending || discard_pending)
					break;
				avr_buffer[0] = settings.single ? HWC_MOVE_SINGLE : HWC_MOVE;
				avr_buffer[1] = mi + m;
				for (int i = 0; i < cfp; ++i) {
					int value = (spaces[s].motor[m]->dir_pin.inverted() ? -1 : 1) * spaces[s].motor[m]->avr_data[i];
					avr_buffer[2 + 2 * i] = value & 0xff;
					avr_buffer[2 + 2 * i + 1] = (value >> 8) & 0xff;
				}
				if (prepare_packet(avr_buffer, 2 + 2 * cfp)) {
					avr_cb = &avr_sent_fragment;
					avr_send();
				}
				else
					break;
			}
		}
		transmitting_fragment = false;
	}
	avr_filling = false;
	return !host_block && !stopping && !discard_pending && !stop_pending;
} // }}}

void arch_start_move(int extra) { // {{{
	if (host_block)
		return;
	if (!connected || preparing || sending_fragment || out_busy >= 3) {
		//debug("no start yet");
		start_pending = true;
		return;
	}
	if (avr_running || avr_filling || stopping || avr_homing) {
		//debug("not startable");
		return;
	}
	if ((running_fragment - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER <= extra + 2) {
		//debug("no buffer no start");
		return;
	}
	//debug("start move %d %d %d %d", current_fragment, running_fragment, sending_fragment, extra);
	while (out_busy >= 3) {
		poll(&pollfds[BASE_FDS], 1, -1);
		serial(1);
	}
	start_pending = false;
	avr_running = true;
	avr_buffer[0] = HWC_START;
	if (prepare_packet(avr_buffer, 1))
		avr_send();
} // }}}

bool arch_running() { // {{{
	return avr_running;
} // }}}

void arch_home() { // {{{
	if (!connected)
		return;
	avr_homing = true;
	while (out_busy >= 3) {
		poll(&pollfds[BASE_FDS], 1, -1);
		serial(1);
	}
	avr_buffer[0] = HWC_HOME;
	int speed = 10000;	// μs/step.
	for (int i = 0; i < 4; ++i)
		avr_buffer[1 + i] = (speed >> (8 * i)) & 0xff;
	int mi = 0;
	for (int s = 0; s < NUM_SPACES; mi += spaces[s++].num_motors) {
		Space &sp = spaces[s];
		for (int m = 0; m < sp.num_motors; ++m) {
			if (abs(int8_t(command[0][3 + mi + m])) <= 1) {
				avr_buffer[5 + mi + m] = (sp.motor[m]->dir_pin.inverted() ? -1 : 1) * command[0][3 + mi + m];
			}
			else {
				debug("invalid code in home: %d", command[0][3 + mi + m]);
				return;
			}
		}
	}
	if (prepare_packet(avr_buffer, 5 + avr_active_motors))
		avr_send();
} // }}}

void arch_stop_audio() { // {{{
	if (avr_audio < 0)
		return;
	debug("audio recover");
	arch_stop(false);
	avr_audio = -1;
	arch_globals_change();
} // }}}

off_t arch_send_audio(uint8_t *map, off_t pos, off_t max, int motor) { // {{{
	if (!connected)
		return max;
	if (avr_audio != motor) {
		arch_stop(false);
		avr_audio = motor;
		arch_globals_change();
	}
	int len = max - pos >= NUM_MOTORS * BYTES_PER_FRAGMENT ? BYTES_PER_FRAGMENT : (max - pos) / NUM_MOTORS;
	if (len <= 0)
		return max;
	while (out_busy >= 3) {
		poll(&pollfds[BASE_FDS], 1, -1);
		serial(1);
	}
	avr_buffer[0] = HWC_START_MOVE;
	avr_buffer[1] = len;
	avr_buffer[2] = NUM_MOTORS;
	sending_fragment = NUM_MOTORS + 1;
	if (!prepare_packet(avr_buffer, 3)) {
		debug("audio upload failed");
		return pos + NUM_MOTORS * len;
	}
	avr_cb = &avr_sent_fragment;
	avr_send();
	avr_filling = true;
	for (int m = 0; m < NUM_MOTORS; ++m) {
		while (out_busy >= 3) {
			poll(&pollfds[BASE_FDS], 1, -1);
			serial(1);
		}
		avr_buffer[0] = HWC_MOVE_SINGLE;
		avr_buffer[1] = m;
		for (int i = 0; i < len; ++i)
			avr_buffer[2 + i] = map[pos + m * len + i];
		if (!prepare_packet(avr_buffer, 2 + len)) {
			debug("audio data upload failed");
			break;
		}
		avr_cb = &avr_sent_fragment;
		avr_send();
	}
	avr_filling = false;
	return pos + NUM_MOTORS * len;
} // }}}

void arch_do_discard() { // {{{
	int cbs = 0;
	while (out_busy >= 3) {
		poll(&pollfds[BASE_FDS], 1, -1);
		serial(1);
	}
	if (!discard_pending)
		return;
	discard_pending = false;
	int fragments = (current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
	if (fragments <= 2)
		return;
	for (int i = 0; i < fragments - 2; ++i) {
		current_fragment = (current_fragment - 1 + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
		//debug("current_fragment = (current_fragment - 1 + FRAGMENTS_PER_BUFFER) %% FRAGMENTS_PER_BUFFER; %d", current_fragment);
		//debug("restoring %d %d", current_fragment, history[current_fragment].cbs);
		cbs += history[current_fragment].cbs;
	}
	restore_settings();
	history[(current_fragment - 1 + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER].cbs += cbs + cbs_after_current_move;
	//debug("cbs after current cleared after setting %d+%d in history", cbs, cbs_after_current_move);
	cbs_after_current_move = 0;
	avr_buffer[0] = HWC_DISCARD;
	avr_buffer[1] = fragments - 2;
	// We're in the middle of a move again, so make sure the computation is restarted.
	computing_move = true;
	if (prepare_packet(avr_buffer, 2))
		avr_send();
} // }}}

void arch_discard() { // {{{
	// Discard much of the buffer, so the upcoming change will be used almost immediately.
	if (!avr_running || stopping || avr_homing)
		return;
	discard_pending = true;
	if (connected && !avr_filling)
		arch_do_discard();
} // }}}

void arch_send_spi(int bits, uint8_t *data) { // {{{
	if (!connected)
		return;
	while (out_busy >= 3) {
		poll(&pollfds[BASE_FDS], 1, -1);
		serial(1);
	}
	avr_buffer[0] = HWC_SPI;
	avr_buffer[1] = bits;
	for (int i = 0; i * 8 < bits; ++i)
		avr_buffer[2 + i] = data[i];
	if (prepare_packet(avr_buffer, 2 + (bits + 7) / 8))
		avr_send();
} // }}}
// }}}

// Debugging hooks. {{{
void START_DEBUG() { // {{{
	fprintf(stderr, "cdriver debug from firmware: ");
} // }}}

void DO_DEBUG(char c) { // {{{
	fprintf(stderr, "%c"
#ifdef DEBUG_AVRCOMM
				" "
#endif
				, c);
} // }}}

void END_DEBUG() { // {{{
	fprintf(stderr, "\n");
} // }}}
// }}}

// AVRSerial methods. {{{
void AVRSerial::begin(char const *port) { // {{{
	// Open serial port and prepare pollfd.
	//debug("opening %s", port);
	if (port[0] == '!') {
		int pipes[2];
		socketpair(AF_LOCAL, SOCK_STREAM, 0, pipes);
		pid_t pid = fork();
		if (!pid) {
			// Child.
			close(pipes[0]);
			dup2(pipes[1], 0);
			dup2(pipes[1], 1);
			close(pipes[1]);
			execlp(&port[1], &port[1], NULL);
			abort();
		}
		// Parent.
		close(pipes[1]);
		fd = pipes[0];
	}
	else {
		fd = open(port, O_RDWR);
		if (fd < 0) {
			debug("failed to open port %s: %s", port, strerror(errno));
			disconnect(true);
			return;
		}
	}
	pollfds[BASE_FDS].fd = fd;
	pollfds[BASE_FDS].events = POLLIN | POLLPRI;
	pollfds[BASE_FDS].revents = 0;
	start = 0;
	end_ = 0;
	fcntl(fd, F_SETFL, O_NONBLOCK);
} // }}}

void AVRSerial::write(char c) { // {{{
#ifdef DEBUG_AVRCOMM
	debug("w\t%02x", c & 0xff);
#endif
	if (!connected) {
		debug("writing to serial while not connected");
		abort();
	}
	while (true) {
		errno = 0;
		int ret = ::write(fd, &c, 1);
		if (ret == 1)
			break;
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			debug("write to avr failed: %d %s", ret, strerror(errno));
			disconnect(true);
			return;	// This causes protocol errors during reconnect, but they will be handled.
		}
	}
} // }}}

void AVRSerial::refill() { // {{{
	start = 0;
	end_ = ::read(fd, buffer, sizeof(buffer));
	//debug("%s", strerror(errno));
	//debug("refill %d bytes", end_);
	if (end_ < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK)
			debug("read returned error: %s", strerror(errno));
		end_ = 0;
	}
	if (end_ == 0 && pollfds[BASE_FDS].revents) {
		debug("EOF detected on serial port; waiting for reconnect.");
		disconnect(true);
	}
	pollfds[BASE_FDS].revents = 0;
} // }}}

int AVRSerial::read() { // {{{
	while (true) {
		if (start == end_)
			refill();
		if (start != end_)
			break;
		debug("eof on input; waiting for reconnect.");
		disconnect(true);
	}
	int ret = buffer[start++];
#ifdef DEBUG_AVRCOMM
	debug("r %02x", ret & 0xff);
#endif
	return ret;
} // }}}
// }}}
#endif

#endif

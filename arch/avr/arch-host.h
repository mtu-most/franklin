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

#ifndef AVR_ARCH_H
#define AVR_ARCH_H

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
#include <cmath>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>

#define AVR_BUFFER_DATA_TYPE int8_t
struct avr_Data {
	AVR_BUFFER_DATA_TYPE *buffer;
	uint8_t motor;
};
// Enable all the parts for a serial connection (which can fail) to the machine.
#define SERIAL
#define ADCBITS 10
#define ARCH_MOTOR avr_Data avr_data;
#define ARCH_PATTERN avr_Data avr_data;
#define ARCH_SPACE
#define ARCH_NEW_MOTOR(s, m, base) base[m]->avr_data.buffer = new AVR_BUFFER_DATA_TYPE[BYTES_PER_FRAGMENT / sizeof(AVR_BUFFER_DATA_TYPE)];
#define DATA_DELETE(s, m) delete[] (spaces[s].motor[m]->avr_data.buffer)
#define DATA_CLEAR() do { \
		for (int s = 0; s < NUM_SPACES; ++s) \
			for (int m = 0; m < spaces[s].num_motors; ++m) \
				memset((spaces[s].motor[m]->avr_data.buffer), 0, BYTES_PER_FRAGMENT); \
		memset(pattern.avr_data.buffer, 0, BYTES_PER_FRAGMENT); \
	} while (0)
#define DATA_SET(s, m, v) spaces[s].motor[m]->avr_data.buffer[current_fragment_pos] = v;
#define PATTERN_SET(v) pattern.avr_data.buffer[current_fragment_pos] = v;
#define SAMPLES_PER_FRAGMENT (BYTES_PER_FRAGMENT / sizeof(AVR_BUFFER_DATA_TYPE))
#define ARCH_MAX_FDS 1	// Maximum number of fds for arch-specific purposes.

#endif

// Not defines, because they can change value.
EXTERN uint8_t NUM_PINS, NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS, NUM_MOTORS, FRAGMENTS_PER_BUFFER, BYTES_PER_FRAGMENT, TIME_PER_ISR;
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
	HWC_PATTERN,	// 0c
	HWC_START,	// 0d
	HWC_STOP,	// 0e
	HWC_ABORT,	// 0f
	HWC_DISCARD,	// 10
	HWC_GETPIN,	// 11
	HWC_SPI,	// 12
	HWC_PINNAME,	// 13
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
void arch_set_duty(Pin_t _pin, double duty);
void arch_set_pin_motor(Pin_t _pin, int s, int m, int ticks);
void arch_reset();
void arch_motor_change(uint8_t s, uint8_t sm);
void arch_pattern_change();
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
void arch_reconnect(const char *port);
void arch_addpos(int s, int m, double diff);
void arch_change_steps_per_unit(int s, int m, double factor);
void arch_invertpos(int s, int m);
void arch_stop(bool fake);
void avr_stop2();
bool arch_send_fragment();
void arch_start_move(int extra);
bool arch_running();
void arch_home();
void arch_discard();
void arch_send_spi(int len, const uint8_t *data);
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
	int motor;
	int ticks;
}; // }}}

// Declarations of static variables; extern because this is a header file. {{{
EXTERN AVRSerial avr_serial;
EXTERN uint8_t avr_pong;
EXTERN char avr_buffer[256];
EXTERN int avr_limiter_space;
EXTERN int avr_limiter_motor;
EXTERN bool avr_running;
EXTERN Avr_pin_t *avr_pins;
EXTERN double *avr_pos_offset;	// pos + offset = hwpos
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


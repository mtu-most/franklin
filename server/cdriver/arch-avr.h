// vim: set foldmethod=marker :
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

// Enable all the parts for a serial connection (which can fail) to the printer.
#define SERIAL
#define ADCBITS 10
#define DATA_TYPE char
#define ARCH_MOTOR DATA_TYPE *avr_data;
#define ARCH_SPACE
#define ARCH_NEW_MOTOR(s, m, base) base[m]->avr_data = new DATA_TYPE[BYTES_PER_FRAGMENT]
#define DATA_DELETE(s, m) delete[] (spaces[s].motor[m]->avr_data)
#define DATA_CLEAR(s, m) memset((spaces[s].motor[m]->avr_data), 0, BYTES_PER_FRAGMENT)
#define DATA_SET(s, m, v) spaces[s].motor[m]->avr_data[current_fragment_pos] = v;
#define SAMPLES_PER_FRAGMENT BYTES_PER_FRAGMENT

#else

// Not defines, because they can change value.
EXTERN uint8_t NUM_DIGITAL_PINS, NUM_ANALOG_INPUTS, NUM_MOTORS, FRAGMENTS_PER_BUFFER, BYTES_PER_FRAGMENT;
EXTERN int avr_audio;
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
	HWC_START,	// 0b
	HWC_STOP,	// 0c
	HWC_ABORT,	// 0d
	HWC_DISCARD,	// 0e
	HWC_GETPIN,	// 0f

	HWC_READY = 0x10,
	HWC_PONG,	// 11
	HWC_HOMED,	// 12
	HWC_PIN,	// 13
	HWC_STOPPED,	// 14

	HWC_DONE,	// 15
	HWC_UNDERRUN,	// 16
	HWC_ADC,	// 17
	HWC_LIMIT,	// 18
	HWC_SENSE0,	// 19
	HWC_SENSE1,	// 1a
	HWC_TIMEOUT,	// 1b
	HWC_PINCHANGE,	// 1c
};
// }}}

// Function declarations. {{{
int hwpacketsize(int len, int *available);
void try_send_control();
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
void arch_setup_start(char const *port);
void arch_set_uuid();
void arch_setup_end(char const *run_id);
void avr_setup_end2();
void arch_setup_end(char const *run_id);
void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin = ~0, bool heater_invert = false, int heater_adctemp = 0, int fan_pin = ~0, bool fan_invert = false, int fan_adctemp = 0);
void arch_disconnect();
int arch_fds();
int arch_tick();
void arch_reconnect(char *port);
void arch_addpos(int s, int m, int diff);
void arch_stop(bool fake);
void avr_stop2();
bool arch_send_fragment();
void arch_start_move(int extra);
bool arch_running();
void arch_home();
off_t arch_send_audio(uint8_t *map, off_t pos, off_t max, int motor);
void arch_do_discard();
void arch_discard();
void START_DEBUG();
void DO_DEBUG(char c);
void END_DEBUG();
// }}}

struct AVRSerial : public Serial_t { // {{{
	char buffer[256];
	int start, end_, fd;
	void begin(char const *port, int baud);
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
};
// }}}
struct Avr_pin_t { // {{{
	char state;
	char reset;
	int duty;
};
// }}}

// Declarations of static variables; extern because this is a header file. {{{
EXTERN AVRSerial avr_serial;
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
EXTERN void (*avr_get_cb)(bool);
EXTERN bool avr_get_pin_invert;
EXTERN bool avr_stop_fake;
// }}}

#define avr_write_ack(reason) do { \
	/*debug("ack: %s", reason); */\
	write_ack(); \
} while(0)

#ifdef DEFINE_VARIABLES
// Serial port communication. {{{
int hwpacketsize(int len, int *available) {
	int const arch_packetsize[16] = { 0, 2, 0, 2, 0, 3, 0, 4, 0, 0, 0, 1, 3, -1, -1, -1 };
	int num_motors = avr_audio >= 0 ? NUM_MOTORS : avr_active_motors;
	switch (command[1][0] & 0x1f) {
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
		return 1 + 4 * num_motors;
	case HWC_STOPPED:
		return 2 + 4 * num_motors;
	case HWC_LIMIT:
	case HWC_UNDERRUN:
		return 3 + 4 * num_motors;
	case HWC_SENSE0:
	case HWC_SENSE1:
		return 2 + 4 * num_motors;
	default:
		return arch_packetsize[command[1][0] & 0xf];
	}
}

void try_send_control() {
	if (out_busy >= 3 || avr_control_queue_length == 0)
		return;
	avr_control_queue_length -= 1;
	avr_buffer[0] = HWC_CONTROL;
	avr_buffer[1] = avr_control_queue[avr_control_queue_length * 3];
	avr_buffer[2] = avr_control_queue[avr_control_queue_length * 3 + 1];
	avr_buffer[3] = avr_control_queue[avr_control_queue_length * 3 + 2];
	avr_in_control_queue[avr_control_queue[avr_control_queue_length * 3]] = false;
	prepare_packet(avr_buffer, 4);
	avr_send();
}

void avr_send() {
	//debug("avr_send");
	while (out_busy >= 3) {
		//debug("avr send");
		poll(&pollfds[2], 1, -1);
		serial(1);
	}
	out_busy += 1;
	send_packet();
	if (out_busy < 3)
		try_send_control();
}

void avr_call1(uint8_t cmd, uint8_t arg) {
	avr_buffer[0] = cmd;
	avr_buffer[1] = arg;
	prepare_packet(avr_buffer, 2);
	avr_send();
}

void avr_get_current_pos(int offset, bool check) {
	int mi = 0;
	for (int ts = 0; ts < num_spaces; mi += spaces[ts++].num_motors) {
		for (int tm = 0; tm < spaces[ts].num_motors; ++tm) {
			int old = spaces[ts].motor[tm]->settings.current_pos;
			cpdebug(ts, tm, "cpb offset %d raw %d hwpos %d", avr_pos_offset[tm + mi], spaces[ts].motor[tm]->settings.current_pos, spaces[ts].motor[tm]->settings.current_pos + avr_pos_offset[tm + mi]);
			spaces[ts].motor[tm]->settings.current_pos = 0;
			for (int i = 0; i < 4; ++i) {
				spaces[ts].motor[tm]->settings.current_pos += int(uint8_t(command[1][offset + 4 * (tm + mi) + i])) << (i * 8);
			}
			if (spaces[ts].motor[tm]->dir_pin.inverted())
				spaces[ts].motor[tm]->settings.current_pos *= -1;
			spaces[ts].motor[tm]->settings.current_pos -= avr_pos_offset[tm + mi];
			cpdebug(ts, tm, "cpa offset %d raw %d hwpos %d", avr_pos_offset[tm + mi], spaces[ts].motor[tm]->settings.current_pos, spaces[ts].motor[tm]->settings.current_pos + avr_pos_offset[tm + mi]);
			cpdebug(ts, tm, "getpos offset %d diff %d", avr_pos_offset[tm + mi], spaces[ts].motor[tm]->settings.current_pos - old);
			if (check && old != spaces[ts].motor[tm]->settings.current_pos)
				//abort();
				debug("WARNING: out of sync!");
		}
	}
}

bool hwpacket(int len) {
	// Handle data in command[1].
#if 0
	fprintf(stderr, "packet received:");
	for (uint8_t i = 0; i < len; ++i)
		fprintf(stderr, " %02x", command[1][i]);
	fprintf(stderr, "\n");
#endif
	switch (command[1][0]) {
	case HWC_LIMIT:
	case HWC_SENSE0:
	case HWC_SENSE1:
	{
		uint8_t which = command[1][1];
		if (which >= NUM_MOTORS) {
			debug("cdriver: Invalid limit or sense for avr motor %d", which);
			abort();
		}
		avr_write_ack("limit/sense");
		double pos;
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
			pos = spaces[s].motor[m]->settings.current_pos / spaces[s].motor[m]->steps_per_unit;
		}
		if ((command[1][0]) == HWC_LIMIT) {
			debug("limit %d", command[1][2]);
			avr_homing = false;
			abort_move(int8_t(command[1][2]));
			//int i = 0;
			//for (int is = 0; is < num_spaces; ++is)
			//	for (int im = 0; im < spaces[is].num_motors; ++im, ++i)
			//		fprintf(stderr, "\t%8d", spaces[is].motor[im]->settings.current_pos + avr_pos_offset[i]);
			//fprintf(stderr, "\n");
			avr_get_current_pos(3, false);
			//i = 0;
			//for (int is = 0; is < num_spaces; ++is)
			//	for (int im = 0; im < spaces[is].num_motors; ++im, ++i)
			//		fprintf(stderr, "\t%8d", spaces[is].motor[im]->settings.current_pos + avr_pos_offset[i]);
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
			send_host(CMD_SENSE, s, m, 0, command[1][0] == HWC_SENSE1);
		}
		return false;
	}
	case HWC_PONG:
	{
		avr_pong = command[1][1];
		avr_write_ack("pong");
		return false;
	}
	case HWC_ADC:
	{
		int pin = command[1][1];
		if (pin < 0 || pin >= NUM_ANALOG_INPUTS) {
			if (avr_pong == -1)
				debug("invalid adc %d received", pin);
			avr_write_ack("invalid adc");
			return false;
		}
		avr_adc[pin] = (command[1][2] & 0xff) | ((command[1][3] & 0xff) << 8);
		avr_write_ack("adc");
		if (avr_adc_id[pin] >= 0 && avr_adc_id[pin] < num_temps)
			handle_temp(avr_adc_id[pin], avr_adc[pin]);
		return false;
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
			//debug("underrun ok stopped=%d sending=%d pending=%d finishing=%d", stopped, sending_fragment, command[1][2], run_file_finishing);
			if (stopped && !sending_fragment && command[1][2] == 0) {
				avr_get_current_pos(3, true);
				if (run_file_finishing) {
					send_host(CMD_FILE_DONE);
					abort_run_file();
				}
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
			return false;
		}
		first_fragment = -1;
		int cbs = 0;
		for (int i = 0; i < command[1][1]; ++i) {
			int f = (running_fragment + i) % FRAGMENTS_PER_BUFFER;
			//debug("fragment %d: cbs=%d current=%d", f, history[f].cbs, current_fragment);
			cbs += history[f].cbs;
		}
		if (cbs)
			send_host(CMD_MOVECB, cbs);
		if ((current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER + 1 < command[1][1] + command[1][2]) {
			//debug("Done count %d+%d higher than busy fragments %d+1; clipping", command[1][1], command[1][2], (current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER);
			avr_write_ack("invalid done");
			//abort();
		}
		else
			avr_write_ack("done");
		running_fragment = (running_fragment + command[1][1]) % FRAGMENTS_PER_BUFFER;
		//debug("running -> %x", running_fragment);
		if (current_fragment == running_fragment && command[1][0] == HWC_DONE) {
			debug("Done received, but should be underrun");
			//abort();
		}
		if (out_busy < 3)
			buffer_refill();
		run_file_fill_queue();
		return false;
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
		//		fprintf(stderr, "\t%8d", spaces[s].motor[m]->settings.current_pos + avr_pos_offset[i]);
		//fprintf(stderr, "\n");
		avr_write_ack("homed");
		send_host(CMD_HOMED);
		return false;
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
				spaces[s].motor[m]->settings.current_pos = 0;
			}
		}
		for (int m = 0; m < NUM_MOTORS; ++m)
			avr_pos_offset[m] = 0;
		for (int t = 0; t < num_temps; ++t)
			settemp(t, NAN);
		send_host(CMD_TIMEOUT);
		return false;
	}
	case HWC_PINCHANGE:
	{
		avr_write_ack("pinchange");
		for (int i = 0; i < num_gpios; ++i) {
			if (gpios[i].pin.pin == command[1][1])
				send_host(CMD_PINCHANGE, i, gpios[i].pin.inverted() ? !command[1][2] : command[1][2]);
		}
		return false;
	}
	default:
		if (expected_replies <= 0) {
			debug("Received unexpected reply %x", command[1][0]);
			avr_write_ack("unexpected reply");
		}
		else
			return true;
		return false;
	}
}
// }}}

// Hardware interface {{{
void avr_setup_pin(int pin, int type, int resettype, int extra) {
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
}

void SET_INPUT(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state == 2)
		return;
	avr_pins[_pin.pin].state = 2;
	avr_setup_pin(_pin.pin, CTRL_INPUT, avr_pins[_pin.pin].reset, CTRL_NOTIFY);
}

void SET_INPUT_NOPULLUP(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state == 3)
		return;
	avr_pins[_pin.pin].state = 3;
	avr_setup_pin(_pin.pin, CTRL_UNSET, avr_pins[_pin.pin].reset, CTRL_NOTIFY);
}

void RESET(Pin_t _pin) {
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

void SET(Pin_t _pin) {
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

void SET_OUTPUT(Pin_t _pin) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state < 2)
		return;
	RESET(_pin);
}

void avr_get_cb_wrap() {
	void (*cb)(bool) = avr_get_cb;
	avr_get_cb = NULL;
	bool arg = avr_get_pin_invert ^ command[1][1];
	avr_write_ack("get");
	cb(arg);
}

void GET(Pin_t _pin, bool _default, void(*cb)(bool)) {
	if (!_pin.valid())
		cb(_default);
	wait_for_reply[expected_replies++] = avr_get_cb_wrap;
	avr_get_cb = cb;
	avr_get_pin_invert = _pin.inverted();
	avr_call1(HWC_GETPIN, _pin.pin);
}

void avr_send_pin(Pin_t _pin) {
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
}

void arch_pin_set_reset(Pin_t _pin, char state) {
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].reset == state)
		return;
	avr_pins[_pin.pin].reset = state;
	avr_send_pin(_pin);
}

double arch_get_duty(Pin_t _pin) {
	if (_pin.pin < 0 || _pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_get_duty: %d (max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return 1;
	}
	return (avr_pins[_pin.pin].duty + 1) / 256.;
}

void arch_set_duty(Pin_t _pin, double duty) {
	if (_pin.pin < 0 || _pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_set_duty: %d (max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return;
	}
	avr_pins[_pin.pin].duty = int(duty * 256 + .5) - 1;
	if (avr_pins[_pin.pin].duty < 0)
		avr_pins[_pin.pin].duty = 0;
	if (avr_pins[_pin.pin].duty > 255) {
		debug("invalid duty value %d; clipping to 255.", avr_pins[_pin.pin].duty);
		avr_pins[_pin.pin].duty = 255;
	}
	avr_send_pin(_pin);
}
// }}}

// Setup hooks. {{{
void arch_reset() {
	// Initialize connection.
	if (avr_pong == 7) {
		debug("reset ignored");
		return;
	}
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
	uint32_t before = millis();
	while (avr_pong != 7 && millis() - before < 2000) {
		//debug("avr pongwait %d", avr_pong);
		pollfds[2].revents = 0;
		poll(&pollfds[2], 1, 1);
		serial(1);
	}
	if (avr_pong != 7) {
		debug("no pong seen; giving up.\n");
		abort();
	}
}

enum MotorFlags {
	LIMIT			= 0x01,
	INVERT_LIMIT_MIN	= 0x02,
	INVERT_LIMIT_MAX	= 0x04,
	SENSE0			= 0x08,
	SENSE1			= 0x10,
	SENSE_STATE		= 0x20,
	INVERT_STEP		= 0x40
};

void arch_motor_change(uint8_t s, uint8_t sm) {
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

void arch_change(bool motors) {
	int old_active_motors = avr_active_motors;
	if (motors) {
		avr_active_motors = 0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			avr_active_motors += spaces[s].num_motors;
		}
	}
	avr_buffer[0] = HWC_SETUP;
	if (avr_audio >= 0) {
		avr_buffer[1] = NUM_MOTORS;
		for (int i = 0; i < 4; ++i)
			avr_buffer[2 + i] = (audio_hwtime_step >> (8 * i)) & 0xff;
		avr_buffer[11] = avr_audio;
	}
	else {
		avr_buffer[1] = avr_active_motors;
		for (int i = 0; i < 4; ++i)
			avr_buffer[2 + i] = (hwtime_step >> (8 * i)) & 0xff;
		avr_buffer[11] = 0xff;
	}
	avr_buffer[6] = led_pin.valid() ? led_pin.pin : ~0;
	avr_buffer[7] = probe_pin.valid() ? probe_pin.pin : ~0;
	avr_buffer[8] = (led_pin.inverted() ? 1 : 0) | (probe_pin.inverted() ? 2 : 0);
	avr_buffer[9] = timeout & 0xff;
	avr_buffer[10] = (timeout >> 8) & 0xff;
	prepare_packet(avr_buffer, 12);
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

void arch_motors_change() {
	arch_change(true);
}

void arch_globals_change() {
	arch_change(false);
}

void arch_setup_start(char const *port) {
	// Set up arch variables.
	avr_adc = NULL;
	avr_running = true;	// Force arch_stop from setup to do something.
	avr_homing = false;
	avr_filling = false;
	NUM_ANALOG_INPUTS = 0;
	avr_pong = -2;
	avr_limiter_space = -1;
	avr_limiter_motor = 0;
	avr_active_motors = 0;
	avr_audio = -1;
	// Set up serial port.
	avr_connected = true;
	avr_serial.begin(port, 115200);
	serialdev[1] = &avr_serial;
	arch_reset();
}

void arch_set_uuid() {
	avr_buffer[0] = HWC_SET_UUID;
	for (uint8_t i = 0; i < UUID_SIZE; ++i)
		avr_buffer[1 + i] = uuid[i];
	prepare_packet(avr_buffer, 1 + UUID_SIZE);
	avr_send();
}

void avr_setup_end2() {
	protocol_version = 0;
	for (uint8_t i = 0; i < sizeof(uint32_t); ++i)
		protocol_version |= int(uint8_t(command[1][2 + i])) << (i * 8);
	NUM_DIGITAL_PINS = command[1][6];
	NUM_ANALOG_INPUTS = command[1][7];
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
	avr_pong = -1;	// Choke on reset again.
	avr_pins = new Avr_pin_t[NUM_DIGITAL_PINS];
	for (int i = 0; i < NUM_DIGITAL_PINS; ++i) {
		avr_pins[i].reset = 3;	// INPUT_NOPULLUP.
		avr_pins[i].state = avr_pins[i].reset;
		avr_pins[i].duty = 255;
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
	setup_end();
}

void arch_setup_end(char const *run_id) {
	// Get constants.
	avr_buffer[0] = HWC_BEGIN;
	avr_buffer[1] = 10;
	for (int i = 0; i < 8; ++i)
		avr_buffer[2 + i] = run_id[i];
	wait_for_reply[expected_replies++] = avr_setup_end2;
	prepare_packet(avr_buffer, 10);
	avr_send();
}

void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin, bool heater_invert, int heater_adctemp, int fan_pin, bool fan_invert, int fan_adctemp) {
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

void arch_disconnect() {
	avr_connected = false;
	avr_serial.end();
}

int arch_fds() {
	return avr_connected ? 1 : 0;
}

void arch_reconnect(char *port) {
	avr_connected = true;
	avr_serial.begin(port, 115200);
	for (int i = 0; i < 4; ++i)
		avr_serial.write(cmd_nack[i]);	// Just to be sure.
}
// }}}

// Running hooks. {{{
int arch_tick() {
	serial(1);
	return 500;
}

void arch_addpos(int s, int m, int diff) {
	int mi = m;
	for (uint8_t st = 0; st < s; ++st)
		mi += spaces[st].num_motors;
	avr_pos_offset[mi] -= diff;
	cpdebug(s, m, "arch addpos %d %d %d %d", diff, avr_pos_offset[m], spaces[s].motor[m]->settings.current_pos + avr_pos_offset[mi], spaces[s].motor[m]->settings.current_pos);
}

void arch_stop(bool fake) {
	avr_homing = false;
	if (out_busy >= 3) {
		stop_pending = true;
		return;
	}
	if (!avr_running) {
		current_fragment_pos = 0;
		return;
	}
	avr_running = false;
	avr_buffer[0] = HWC_STOP;
	wait_for_reply[expected_replies++] = avr_stop2;
	avr_stop_fake = fake;
	prepare_packet(avr_buffer, 1);
	avr_send();
}

void avr_stop2() {
	if (!avr_stop_fake)
		abort_move(command[1][1]);
	avr_get_current_pos(2, false);
	current_fragment_pos = 0;
	avr_write_ack("stop");
}

bool arch_send_fragment() {
	if (stopping || discard_pending || stop_pending)
		return false;
	if (avr_audio >= 0) {
		arch_stop(false);
		avr_audio = -1;
		arch_globals_change();
	}
	while (out_busy >= 3) {
		poll(&pollfds[2], 1, -1);
		serial(1);
	}
	if (stop_pending || discard_pending)
		return false;
	avr_buffer[0] = settings.probing ? HWC_START_PROBE : HWC_START_MOVE;
	//debug("send fragment %d %d %d", current_fragment_pos, fragment, settings.num_active_motors);
	avr_buffer[1] = current_fragment_pos;
	avr_buffer[2] = num_active_motors;
	sending_fragment = num_active_motors + 1;
	if (prepare_packet(avr_buffer, 3)) {
		avr_send();
		int mi = 0;
		avr_filling = true;
		for (int s = 0; !stopping && !discard_pending && s < num_spaces; mi += spaces[s++].num_motors) {
			for (uint8_t m = 0; !stopping && !discard_pending && m < spaces[s].num_motors; ++m) {
				if (!spaces[s].motor[m]->active)
					continue;
				cpdebug(s, m, "sending %d %d", fragment, current_fragment_pos);
				//debug("sending %d %d %d %x", s, m, current_fragment, current_fragment_pos);
				while (out_busy >= 3) {
					poll(&pollfds[2], 1, -1);
					serial(1);
				}
				if (stop_pending || discard_pending)
					break;
				avr_buffer[0] = HWC_MOVE;
				avr_buffer[1] = mi + m;
				for (int i = 0; i < current_fragment_pos; ++i)
					avr_buffer[2 + i] = (spaces[s].motor[m]->dir_pin.inverted() ? -1 : 1) * spaces[s].motor[m]->avr_data[i];
				if (prepare_packet(avr_buffer, 2 + current_fragment_pos))
					avr_send();
				else
					break;
			}
		}
	}
	avr_filling = false;
	return !stopping && !discard_pending && !stop_pending;
}

void arch_start_move(int extra) {
	if (out_busy >= 3) {
		start_pending = true;
		return;
	}
	if (avr_running || avr_filling || stopping || avr_homing || (running_fragment - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER <= extra + 2)
		return;
	//debug("start move %d %d", sending_fragment, extra);
	while (out_busy >= 3) {
		poll(&pollfds[2], 1, -1);
		serial(1);
	}
	avr_running = true;
	avr_buffer[0] = HWC_START;
	if (prepare_packet(avr_buffer, 1))
		avr_send();
}

bool arch_running() {
	return avr_running;
}

void arch_home() {
	avr_homing = true;
	while (out_busy >= 3) {
		poll(&pollfds[2], 1, -1);
		serial(1);
	}
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
	if (prepare_packet(avr_buffer, 5 + avr_active_motors))
		avr_send();
}

off_t arch_send_audio(uint8_t *map, off_t pos, off_t max, int motor) {
	if (avr_audio != motor) {
		arch_stop(false);
		avr_audio = motor;
		arch_globals_change();
	}
	int len = max - pos >= NUM_MOTORS * BYTES_PER_FRAGMENT ? BYTES_PER_FRAGMENT : (max - pos) / NUM_MOTORS;
	if (len <= 0)
		return max;
	while (out_busy >= 3) {
		poll(&pollfds[2], 1, -1);
		serial(1);
	}
	avr_buffer[0] = HWC_START_MOVE;
	avr_buffer[1] = len;
	avr_buffer[2] = NUM_MOTORS;
	sending_fragment = NUM_MOTORS + 1;
	if (prepare_packet(avr_buffer, 3))
		avr_send();
	avr_filling = true;
	for (int m = 0; m < NUM_MOTORS; ++m) {
		while (out_busy >= 3) {
			poll(&pollfds[2], 1, -1);
			serial(1);
		}
		avr_buffer[0] = HWC_MOVE;
		avr_buffer[1] = m;
		for (int i = 0; i < len; ++i)
			avr_buffer[2 + i] = map[pos + m * len + i];
		if (!prepare_packet(avr_buffer, 2 + len))
			break;
		avr_send();
	}
	avr_filling = false;
	return pos + NUM_MOTORS * len;
}

void arch_do_discard() {
	while (out_busy >= 3) {
		poll(&pollfds[2], 1, -1);
		serial(1);
	}
	avr_buffer[0] = HWC_DISCARD;
	avr_buffer[1] = discard_pending;
	discard_pending = 0;
	if (prepare_packet(avr_buffer, 2))
		avr_send();
}

void arch_discard() {
	// Discard much of the buffer, so the upcoming change will be used almost immediately.
	if (!avr_running || stopping || avr_homing)
		return;
	int fragments = (current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
	if (fragments <= 2)
		return;
	discard_pending = fragments - 2;
	current_fragment = (current_fragment - discard_pending + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
	//debug("current discard -> %x", current_fragment);
	restore_settings();
	if (!avr_filling)
		arch_do_discard();
}
// }}}

// Debugging hooks. {{{
void START_DEBUG() {
	fprintf(stderr, "cdriver debug from firmware: ");
}

void DO_DEBUG(char c) {
	fprintf(stderr, "%c"
#ifdef DEBUG_AVRCOMM
				" "
#endif
				, c);
}

void END_DEBUG() {
	fprintf(stderr, "\n");
}
// }}}

// AVRSerial methods. {{{
void AVRSerial::begin(char const *port, int baud) {
	// Open serial port and prepare pollfd.
	debug("opening %s", port);
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
	}
	pollfds[2].fd = fd;
	pollfds[2].events = POLLIN | POLLPRI;
	pollfds[2].revents = 0;
	start = 0;
	end_ = 0;
	fcntl(fd, F_SETFL, O_NONBLOCK);
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
	//debug("%s", strerror(errno));
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

#endif

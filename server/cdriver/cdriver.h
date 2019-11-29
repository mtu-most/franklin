/* cdriver.h - declarations for Franklin
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

#ifndef _CDRIVER_H
#define _CDRIVER_H

// Exactly one file defines EXTERN as empty, which leads to the data to be defined.
#ifndef EXTERN
#define EXTERN extern
#else
#define DEFINE_VARIABLES
#endif

#include "module.h"
#include "configuration.h"
#include <cstdio>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <poll.h>
#include <sys/types.h>
#include <sys/timerfd.h>
#include <string>

#define PROTOCOL_VERSION ((uint32_t)5)	// Required version response in BEGIN.
#define BASE_FDS 3

#define MAXLONG (int32_t((uint32_t(1) << 31) - 1))
#define MAXINT MAXLONG

#include ARCH_INCLUDE

#define debug(...) do { buffered_debug_flush(); fprintf(stderr, "#"); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); fflush(stderr); } while (0)

extern "C" {

struct Pin_t {
	int flags;
	int pin;
	inline bool valid();
	bool inverted() { return flags & 2; }
	uint16_t write() { return flags << 8 | pin; }
	void init() { flags = 0; pin = 0; }
	inline void read(uint16_t data);
};

union ReadFloat {
	double f;
	int32_t i;
	uint32_t ui;
	uint8_t b[sizeof(double)];
};

enum SingleByteHostCommands {
	OK = 0xb3,
	WAIT = 0xad
};

enum SingleByteCommands {	// See serial.cpp for computation of command values. {{{
	CMD_NACK0 = 0xf0,	// Incorrect packet; please resend.
	CMD_NACK1 = 0x91,	// Incorrect packet; please resend.
	CMD_NACK2 = 0xa2,	// Incorrect packet; please resend.
	CMD_NACK3 = 0xc3,	// Incorrect packet; please resend.
	CMD_ACK0 = 0xc4,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACK1 = 0xa5,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACK2 = 0x96,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACK3 = 0xf7,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_STALL0 = 0x88,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL1 = 0xe9,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL2 = 0xda,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL3 = 0xbb,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_ID = 0xbc,		// Request/reply machine ID code.
	CMD_DEBUG = 0xdd,	// Debug message; a nul-terminated message follows (no checksum; no resend).
	CMD_STARTUP = 0xee,	// Starting up.
	CMD_STALLACK = 0x8f	// Clear stall.
}; // }}}

extern SingleByteCommands cmd_ack[4];
extern SingleByteCommands cmd_nack[4];
extern SingleByteCommands cmd_stall[4];

// All temperatures are stored in Kelvin, but communicated in °C.
struct Temp {
	// See temp.c from definition of calibration constants.
	double R0, R1, logRc, beta, Tc;	// calibration values of thermistor.  [Ω, Ω, logΩ, K, K]
	/*
	// Temperature balance calibration.
	double power;			// added power while heater is on.  [W]
	double core_C;			// heat capacity of the core.  [J/K]
	double shell_C;		// heat capacity of the shell.  [J/K]
	double transfer;		// heat transfer between core and shell.  [W/K]
	double radiation;		// radiated power = radiation * (shell_T ** 4 - room_T ** 4) [W/K**4]
	double convection;		// convected power = convection * (shell_T - room_T) [W/K]
	*/
	// Pins.
	Pin_t power_pin[2];
	double fan_duty;
	Pin_t thermistor_pin;
	// Volatile variables.
	double target[2], limit[2][2];			// target and limit temperature; NAN to disable. [K]
	int32_t adctarget[2], adclimit[2][2];		// target and limit temperature in adc counts; -1 for disabled. [adccounts]
	int32_t adclast;		// last measured temperature. [adccounts]
	/*
	double core_T, shell_T;	// current temperatures. [K]
	*/
	uint8_t following_gpios;	// linked list of gpios monitoring this temp.
	double min_alarm;		// NAN, or the temperature at which to trigger the callback.  [K]
	double max_alarm;		// NAN, or the temperature at which to trigger the callback.  [K]
	int32_t adcmin_alarm;		// -1, or the temperature at which to trigger the callback.  [adccounts]
	int32_t adcmax_alarm;		// -1, or the temperature at which to trigger the callback.  [adccounts]
	// Internal variables.
	int32_t last_temp_time;		// last value of micros when this heater was handled.
	int32_t time_on;		// Time that the heater has been on since last reading.  [μs]
	bool is_on[2];			// If the heater is currently on.
	double hold_time;		// Minimum time to hold value after change.
	unsigned long last_change_time;	// millis() when value was last changed.
	double K;			// Thermistor constant; kept in memory for performance.
	int32_t last_value;		// last measured value.
	// Functions.
	double fromadc(int32_t adc);	// convert ADC to K.
	int32_t toadc(double T, int32_t default_);	// convert K to ADC.
	void load(int id);
	void save();
	void init();
	void free();
	void copy(Temp &dst);
};

// Variables defining the current move:
// double unitg[3]: vector from current position to "target" (not actually reached if h != 0,0,0)
// double unith[3]: vector perpendicular to g in the plane of the curve. (or 0,0,0 if Jh==0)
// double Jg, Jh, a0g, a0h, v0g, v0h, x0g, x0h: Jerk and initial a, v and x for g and h.
// int32_t hwtime: current time, where 0 is start of segment
// int32_t end_time: time when current segment is completed.
// int hwtime_step: time difference for each step.
struct History {
	double unitg[3], unith[3];
	double Jg, Jh, a0g, a0h, v0g, v0h, x0g, x0h;
	int32_t hwtime, end_time;
	int hwtime_step;
	double run_time;
	int gcode_line;
	int64_t current_restore;
	uint8_t pattern[PATTERN_MAX];
	int pattern_size;	// in bytes; each bit in the pattern is a pulse.
};

struct Resume {
	double x[3], v[3], a[3];
	History settings;
};

struct Motor_History {
	double current_pos;	// Current position of motor (in steps), and (cast to int) what the hardware currently thinks.
};

struct Axis_History {
	double source;	// Source position of current movement of axis, or current position if there is no movement.
	double endpos;
};

struct Axis {
	Axis_History *history;
	Axis_History settings;
	double park;		// Park position; not used by the firmware, but stored for use by the host.
	uint8_t park_order;
	double min_pos, max_pos;
	double current;
	double target;	// Target position for this tick.
	double last_target; // Used when retargeting to a position with NaN components.
	double resume_start, resume_end;	// Position to reset to when resuming.
	void *type_data;
};

struct Motor {
	Motor_History *history;
	Motor_History settings;
	Pin_t step_pin;
	Pin_t dir_pin;
	Pin_t enable_pin;
	double steps_per_unit;			// hardware calibration [steps/unit]; if negative, send PWM for DC motor, value is -seconds/unit.
	Pin_t limit_min_pin;
	Pin_t limit_max_pin;
	double home_pos;	// Position of motor (in μm) when the home switch is triggered.
	bool active;
	double limit_v, limit_a;		// maximum value for f [m/s], [m/s^2].
	uint8_t home_order;
	double target_pos;	// Internal variable for moving.
	double last_v;	// Internal variable for checking limits.
	void *type_data;
	ARCH_MOTOR
};

struct Pattern {
	Pin_t step_pin;
	Pin_t dir_pin;
	bool active;
	ARCH_MOTOR
};

struct Space;

struct SpaceType {
	// Only these two are required to be implement by each space type.
	void (*motors2xyz)(Space *s, const double *motors, double *xyz);
	void (*xyz2motors)(Space *s);

	// Check if position is valid and if not, move it to a valid value.
	void (*check_position)(Space *s, double *data);

	// Allocate data for space type.
	bool (*init_space)(Space *s);
	void (*init_axis)(Space *s, int a);
	void (*init_motor)(Space *s, int m);

	// Fill allocated data with values from shmem.
	void (*load_space)(Space *s);
	void (*load_axis)(Space *s, int a);
	void (*load_motor)(Space *s, int m);

	// Write current values back to shmem.
	void (*save_space)(Space *s);
	void (*save_axis)(Space *s, int a);
	void (*save_motor)(Space *s, int m);

	// Deallocate data for space type.
	void (*free_space)(Space *s);
	void (*free_axis)(Space *s, int a);
	void (*free_motor)(Space *s, int m);

	// Used by extruder to handle tool offset.
	double (*change0)(Space *s, int axis, double value);
	double (*unchange0)(Space *s, int axis, double value);

	// Safe speed for probing, should result in 1 step per iteration.
	double (*probe_speed)(Space *s);

	// Internal function for follower space.
	int (*follow)(Space *s, int axis);
};

struct Space {
	void *type_data;
	Motor **motor;
	Axis **axis;
	int id;
	int type;
	int num_axes, num_motors;
	void load_info();
	void load_axis(int a);
	void load_motor(int m);
	void save_info();
	void save_axis(int a);
	void save_motor(int m);
	void init(int space_id);
	bool setup_nums(int na, int nm);
	void cancel_update();
	ARCH_SPACE
};

void Cartesian_init(int num);
void Extruder_init(int num);
void Follower_init(int num);
void Polar_init(int num);
void Hbot_init(int num);

EXTERN SpaceType *space_types;
EXTERN int num_space_types;
EXTERN int current_extruder;

struct Gpio {
	Pin_t pin;
	uint8_t state, reset;
	int space, motor;
	double duty;
	bool changed, value;
	void setup(uint8_t new_state);
	void load();
	void save();
	void init();
	void free();
	void copy(Gpio &dst);
};

struct Serial_t {
	virtual void write(char c) = 0;
	virtual int read() = 0;
	virtual int readBytes (char *target, int len) = 0;
	virtual void flush() = 0;
	virtual int available() = 0;
};

#define COMMAND_SIZE 256
static int const FULL_COMMAND_SIZE = COMMAND_SIZE + (COMMAND_SIZE + 2) / 3;

// Globals
EXTERN double max_deviation;
EXTERN double max_v, max_a, max_J;
EXTERN uint8_t num_extruders;
EXTERN int num_temps;
EXTERN int num_gpios;
EXTERN uint32_t protocol_version;
EXTERN int num_subfragments_bits;
EXTERN Pin_t led_pin, stop_pin, probe_pin, spiss_pin;
EXTERN uint16_t timeout;
EXTERN int bed_id, fan_id, spindle_id;
//EXTERN double room_T;	//[°C]
EXTERN double feedrate;		// Multiplication factor for f values, used at start of move.
EXTERN double targetx, targety, targetangle, zoffset;	// Offset for axis 2 of space 0.
// Other variables.
EXTERN Serial_t *serialdev;
EXTERN unsigned char command[FULL_COMMAND_SIZE];
EXTERN int command_end;
EXTERN Space spaces[NUM_SPACES];
EXTERN Temp *temps;
EXTERN Gpio *gpios;
EXTERN Pattern pattern;
EXTERN FILE *store_adc;
EXTERN uint8_t temps_busy;
EXTERN MoveCommand queue[QUEUE_LENGTH];
EXTERN int default_hwtime_step, min_hwtime_step;
EXTERN uint8_t which_autosleep;		// which autosleep message to send (0: none, 1: motor, 2: temp, 3: both)
EXTERN uint8_t ping;			// bitmask of waiting ping replies.
EXTERN bool initialized;
EXTERN int queue_start, queue_end;
EXTERN bool queue_full;
EXTERN bool probing, single;
EXTERN bool motors_busy;
EXTERN int out_busy;
EXTERN int32_t out_time;
EXTERN char pending_packet[4][FULL_COMMAND_SIZE];
EXTERN int pending_len[4];
EXTERN void (*serial_cb[4])();
EXTERN int32_t last_active;
EXTERN int32_t last_micros;
EXTERN int16_t led_phase;
EXTERN Resume resume;
EXTERN bool pausing, resume_pending;
EXTERN History *history;
EXTERN History settings;
EXTERN double final_x[3], final_v[3], final_a[3];	// For checking that the segments fit.
EXTERN bool computing_move;	// True as long as steps are sent to firmware.
EXTERN bool aborting, preparing;
EXTERN int first_fragment;
EXTERN int stopping;		// From limit.
EXTERN int sending_fragment;
EXTERN bool transmitting_fragment;
EXTERN bool start_pending, stop_pending, change_pending;
EXTERN int discarding;
EXTERN bool discard_pending;
EXTERN double done_factor;
EXTERN uint8_t requested_temp;
EXTERN bool refilling;
EXTERN double factor;
EXTERN int current_fragment, running_fragment;
EXTERN unsigned current_fragment_pos;
EXTERN int num_active_motors;
EXTERN struct pollfd pollfds[BASE_FDS + ARCH_MAX_FDS];
EXTERN void (*wait_for_reply[4])();
EXTERN int expected_replies;
EXTERN int pins_changed;

// Event data and flags for pending interrupts.
EXTERN int num_file_done_events;
EXTERN bool continue_event;
EXTERN bool cb_pending;

#if DEBUG_BUFFER_LENGTH > 0
EXTERN char debug_buffer[DEBUG_BUFFER_LENGTH];
EXTERN int16_t debug_buffer_ptr;
// debug.cpp
void buffered_debug_flush();
void buffered_debug(char const *fmt, ...);
#else
#define buffered_debug debug
#define buffered_debug_flush() do {} while(0)
#endif
#define DEBUG_LIST_SIZE 30
EXTERN int debug_list[DEBUG_LIST_SIZE][4];
EXTERN int debug_list_next;
void debug_add(int a, int b = 0xfbfbfbfb, int c = 0xfbfbfbfb, int d = 0xfbfbfbfb);
void debug_dump();

// Force cpdebug if requested, to enable only specific lines without adding all the cp things in manually.
//#define fcpdebug(s, m, fmt, ...) do { if (s == 1 && m == 0) debug("CP curfragment %d curpos %f current %f " fmt, current_fragment, spaces[s].motor[m]->settings.current_pos, spaces[s].axis[m]->settings.current, ##__VA_ARGS__); } while (0)
#define fcpdebug(s, m, fmt, ...) do { debug("CP %d %d curfragment %d curpos %f current %f " fmt, s, m, current_fragment, spaces[s].motor[m]->settings.current_pos, spaces[s].axis[m]->current, ##__VA_ARGS__); } while (0)
//#define cpdebug fcpdebug
#define cpdebug(...) do {} while (0)

// packet.cpp
void request(int req);
bool compute_current_pos(double x[3], double v[3], double a[3], bool store);
int prepare_retarget(int q, int tool, double x[3], double v[3], double a[3], bool resuming = false);
void smooth_stop(int q, double x[3], double v[3]);
void do_resume();
int go_to(bool relative, MoveCommand const *move, bool cb, bool queue_only = false);
void settemp(int which, double target);
void waittemp(int which, double mintemp, double maxtemp);
void setpos(int which, int t, double f, bool reset);
void delayed_reply();
void send_to_parent(char cmd);
void prepare_interrupt();

// serial.cpp
bool serial(bool allow_pending);	// Handle commands from serial.
void serial_wait(int timeout = -1);
bool prepare_packet(char *the_packet, int len);
void send_packet();
void write_ack();
void write_nack();
EXTERN uint8_t ff_in;	// Index of next in-packet that is expected.
EXTERN uint8_t ff_out;	// Index of next out-packet that will be sent.

// move.cpp
void next_move(int32_t start_time);
void abort_move(int pos);
void discard_finals();

// run.cpp
struct ProbeFile {
	double targetx, targety, x0, y0, w, h, sina, cosa;
	unsigned long nx, ny;
	double angle;
	double sample[0];
} __attribute__((__packed__));
void run_file(char const *name, char const *probe_name, bool start, double sina, double cosa);
void abort_run_file();
void run_file_fill_queue(bool move_allowed = true);
void run_adjust_probe(double x, double y, double z);
double run_find_pos(const double pos[3]);
EXTERN int64_t run_file_current;
EXTERN std::string probe_file_name;
EXTERN off_t probe_file_size;
EXTERN ProbeFile *probe_file_map;
EXTERN std::string run_file_name;
EXTERN off_t run_file_size;
EXTERN Run_Record *run_file_map;
EXTERN int run_file_num_strings;
EXTERN off_t run_file_first_string;
EXTERN int run_file_num_records;
EXTERN int run_file_wait;
EXTERN struct itimerspec run_file_timer;
EXTERN double run_file_refx;
EXTERN double run_file_refy;
EXTERN double run_file_refz;
EXTERN double run_file_sina;
EXTERN double run_file_cosa;

// setup.cpp
void setup();
void connect_machine(char const *port, char const *run_id);
void connect_end();
void check_protocol();
Axis_History *setup_axis_history();
Motor_History *setup_motor_history();
EXTERN bool host_block;
EXTERN bool connected;

// temp.cpp
void handle_temp(int id, int temp);

// space.cpp
void buffer_refill();
void store_settings();
void restore_settings();
void reset_pos(Space *s);

// globals.cpp
bool globals_load();
void globals_save();

// base.cpp
void debug_backtrace();
void disconnect(bool notify);
int32_t utime();
int32_t millis();
EXTERN bool interrupt_pending;

#include ARCH_INCLUDE

// ===============
// Arch interface.
// ===============
// Defined or variables:
// NUM_PINS
// ADCBITS
// FRAGMENTS_PER_BUFFER
// BYTES_PER_FRAGMENT
void SET_INPUT(Pin_t _pin);
void SET_INPUT_NOPULLUP(Pin_t _pin);
void RESET(Pin_t _pin);
void SET(Pin_t _pin);
void SET_OUTPUT(Pin_t _pin);
void GET(Pin_t _pin, bool _default, void(*cb)(bool));
void arch_setup_start();
void arch_setup_end();
void arch_connect(char const *run_id, char const *port);
void arch_motors_change();
void arch_addpos(int s, int m, double diff);
void arch_invertpos(int s, int m);
void arch_stop(bool fake = false);
void arch_home();
bool arch_running();
double arch_round_pos(int s, int m, double pos);
//void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin = ~0, bool heater_invert = false, int heater_adctemp = 0, int heater_limit_l = ~0, int heater_limit_h = ~0, int fan_pin = ~0, bool fan_invert = false, int fan_adctemp = 0, int fan_limit_l = ~0, int fan_limit_h = ~0, double hold_time = 0);
void arch_start_move(int extra);
bool arch_send_fragment();

#ifdef SERIAL
int hwpacketsize(int len, int *available);
bool hwpacket(int len);
void arch_reconnect(const char *port);
void arch_disconnect();
int arch_fds();
// Serial_t derivative Serial;
//void arch_pin_set_reset(Pin_t pin_, int state);
void START_DEBUG();
void DO_DEBUG(char c);
void END_DEBUG();
#endif

bool Pin_t::valid() {
	return pin < NUM_PINS && flags & 1;
}

void Pin_t::read(uint16_t data) {
	int new_pin = data & 0xff;
	int new_flags = data >> 8;
	if (valid() && (new_pin != pin || new_flags != flags)) {
		SET_INPUT_NOPULLUP(*this);
#ifdef SERIAL
		// Reset is not recorded on connections that cannot fail.
		arch_pin_set_reset(*this, 3);
#endif
	}
	pin = new_pin;
	flags = new_flags;
	if (flags & ~3) {
		flags = 0;
		pin = 0;
	}
}
#endif

}

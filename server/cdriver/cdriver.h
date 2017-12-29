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

#include "configuration.h"
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/timerfd.h>
#include <string>

#define PROTOCOL_VERSION ((uint32_t)3)	// Required version response in BEGIN.
#define ID_SIZE 8
#define UUID_SIZE 16
#define BASE_FDS 2

#define MAXLONG (int32_t((uint32_t(1) << 31) - 1))
#define MAXINT MAXLONG

// Exactly one file defines EXTERN as empty, which leads to the data to be defined.
#ifndef EXTERN
#define EXTERN extern
#else
#define DEFINE_VARIABLES
#endif

#define NUM_SPACES 3

#include ARCH_INCLUDE

#define debug(...) do { buffered_debug_flush(); fprintf(stderr, "#"); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); fflush(stderr); } while (0)

static inline int min(int a, int b) {
	return a < b ? a : b;
}

static inline int max(int a, int b) {
	return a > b ? a : b;
}

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
	CMD_NACK0 = 0xf0,       // Incorrect packet; please resend.
	CMD_NACK1 = 0x91,       // Incorrect packet; please resend.
	CMD_NACK2 = 0xa2,       // Incorrect packet; please resend.
	CMD_NACK3 = 0xc3,       // Incorrect packet; please resend.
	CMD_ACK0 = 0xc4,        // Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACK1 = 0xa5,        // Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACK2 = 0x96,        // Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACK3 = 0xf7,        // Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_STALL0 = 0x88,      // Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL1 = 0xe9,      // Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL2 = 0xda,      // Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL3 = 0xbb,      // Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_ID = 0xbc,          // Request/reply machine ID code.
	CMD_DEBUG = 0xdd,       // Debug message; a nul-terminated message follows (no checksum; no resend).
	CMD_STARTUP = 0xee,     // Starting up.
	CMD_STALLACK = 0x8f     // Clear stall.
}; // }}}

extern const SingleByteCommands cmd_ack[4];
extern const SingleByteCommands cmd_nack[4];
extern const SingleByteCommands cmd_stall[4];

enum Command {
	// from host
	CMD_SET_UUID,	// 22 bytes: uuid.
	CMD_GET_UUID,	// 0.  Reply: UUID.
	CMD_LINE,	// 1-2 byte: which channels (depending on number of extruders); channel * 4 byte: values [fraction/s], [mm].  Reply (later): MOVECB.
	CMD_SINGLE,	// Same as CMD_LINE.
	CMD_PROBE,	// Same as CMD_LINE.
	CMD_PARSE_GCODE,// 2 byte: in filename length, in filename, out filename
	CMD_RUN_FILE,	// n byte: filename.
	CMD_SLEEP,	// 1 byte: which channel (b0-6); on/off (b7 = 1/0).
	CMD_SETTEMP,	// 1 byte: which channel; 4 bytes: target [°C].
	CMD_WAITTEMP,	// 1 byte: which channel; 4 bytes: lower limit; 4 bytes: upper limit [°C].  Reply (later): TEMPCB.  Disable with WAITTEMP (NAN, NAN).
	CMD_READTEMP,	// 1 byte: which channel.  Reply: TEMP. [°C]
	CMD_READPOWER,	// 1 byte: which channel.  Reply: POWER. [μs, μs]
	CMD_SETPOS,	// 1 byte: which channel; 4 bytes: pos.
	CMD_GETPOS,	// 1 byte: which channel.  Reply: POS. [steps, mm]
	CMD_READ_GLOBALS,
	CMD_WRITE_GLOBALS,
	CMD_READ_SPACE_INFO,	// 1 byte: which channel.  Reply: DATA.
	CMD_READ_SPACE_AXIS,	// 1 byte: which channel.  Reply: DATA.
	CMD_READ_SPACE_MOTOR,	// 1 byte: which channel; n bytes: data.
	CMD_WRITE_SPACE_INFO,	// 1 byte: which channel.  Reply: DATA.
	CMD_WRITE_SPACE_AXIS,	// 1 byte: which channel; n bytes: data.
	CMD_WRITE_SPACE_MOTOR,	// 1 byte: which channel; n bytes: data.
	CMD_READ_TEMP,	// 1 byte: which channel.  Reply: DATA.
	CMD_WRITE_TEMP,	// 1 byte: which channel; n bytes: data.
	CMD_READ_GPIO,	// 1 byte: which channel.  Reply: DATA.
	CMD_WRITE_GPIO,	// 1 byte: which channel; n bytes: data.
	CMD_QUEUED,	// 1 byte: 0: query queue length; 1: stop and query queue length.  Reply: QUEUE.
	CMD_READPIN,	// 1 byte: which channel. Reply: GPIO.
	CMD_HOME,	// 1 byte: homing space; n bytes: homing type (0=pos, 1=neg, 3=no)
	CMD_FORCE_DISCONNECT,	// 0
	CMD_CONNECT,	// 8 byte: run ID, n bytes: port name (0-terminated)
	CMD_RECONNECT,	// n bytes: port name (0-terminated)
	CMD_RESUME,
	CMD_GETTIME,
	CMD_SPI,
	CMD_ADJUSTPROBE,	// 3 doubles: probe position.
	CMD_TP_GETPOS,
	CMD_TP_SETPOS,	// 1 double: new toolpath position.
	CMD_TP_FINDPOS,	// 3 doubles: search position or NaN.
	CMD_MOTORS2XYZ,	// 1 byte: which space, n doubles: motor positions.  Reply: m times XYZ.
	// to host
		// responses to host requests; only one active at a time.
	CMD_UUID = 0x40,	// 16 byte uuid.
	CMD_TEMP,	// 4 byte: requested channel's temperature. [°C]
	CMD_POWER,	// 4 byte: requested channel's power time; 4 bytes: current time. [μs, μs]
	CMD_POS,	// 4 byte: pos [steps]; 4 byte: current [mm].
	CMD_DATA,	// n byte: requested data.
	CMD_PIN,	// 1 byte: 0 or 1: pin state.
	CMD_QUEUE,	// 1 byte: current number of records in queue.
	CMD_HOMED,	// 0
	CMD_TIME,
	CMD_TP_POS,	// double: current or found position in toolpath.
	CMD_XYZ,	// double: axis position.
	CMD_PARSED_GCODE,	// 0
		// asynchronous events.
	CMD_MOVECB,	// 1 byte: number of movecb events.
	CMD_TEMPCB,	// 1 byte: which channel.  Byte storage for which needs to be sent.
	CMD_CONTINUE,	// 1 byte: is_audio.  Bool flag if it needs to be sent.
	CMD_LIMIT,	// 1 byte: which channel.
	CMD_TIMEOUT,	// 0
	CMD_DISCONNECT,	// 0
	CMD_PINCHANGE,	// 1 byte: pin, 1 byte: current value.
		// Updates from RUN_FILE.
	CMD_UPDATE_TEMP,
	CMD_UPDATE_PIN,
	CMD_CONFIRM,
	CMD_FILE_DONE,
	CMD_PARKWAIT,
	CMD_CONNECTED,
		// Pin names; broadcast during setup.
	CMD_PINNAME,
};

enum RunType {
	RUN_SYSTEM,
	RUN_PRE_LINE,
	RUN_LINE,
	RUN_PRE_ARC,
	RUN_ARC,
	RUN_GPIO,
	RUN_SETTEMP,
	RUN_WAITTEMP,
	RUN_SETPOS,
	RUN_WAIT,
	RUN_CONFIRM,
	RUN_PARK,
};
void parse_gcode(std::string const &infilename, std::string const &outfilename);

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
	// Functions.
	int32_t get_value();		// Get thermistor reading, or -1 if it isn't available yet.
	double fromadc(int32_t adc);	// convert ADC to K.
	int32_t toadc(double T, int32_t default_);	// convert K to ADC.
	void load(int32_t &addr, int id);
	void save(int32_t &addr);
	void init();
	void free();
	void copy(Temp &dst);
};

struct History {
	double t0, tp;
	double f0, f1, f2, fp, fq, fmain;
	int32_t hwtime, start_time, last_time, last_current_time;
	int cbs;
	int queue_start, queue_end;
	bool queue_full;
	int run_file_current;
	bool probing, single;
	double run_time, run_dist;
};

struct Space_History {
	double dist[2];
	bool arc[2];
	double angle[2], helix[2];
	double offset[2][3];
	double radius[2][2];
	double e1[2][3];
	double e2[2][3];
	double normal[2][3];
};

struct Motor_History {
	double last_v;		// v during last iteration, for using limit_a [m/s].
	double target_v, target_dist;	// Internal values for moving.
	double current_pos;	// Current position of motor (in steps), and (cast to int) what the hardware currently thinks.
	double endpos;
};

struct Axis_History {
	double dist[2], main_dist;
	double source, current;	// Source position of current movement of axis (in μm), or current position if there is no movement.
	double target;
	double endpos[2];
};

struct Axis {
	Axis_History *history;
	Axis_History settings;
	double park;		// Park position; not used by the firmware, but stored for use by the host.
	uint8_t park_order;
	double min_pos, max_pos;
	void *type_data;
};

struct Motor {
	Motor_History *history;
	Motor_History settings;
	Pin_t step_pin;
	Pin_t dir_pin;
	Pin_t enable_pin;
	double steps_per_unit;			// hardware calibration [steps/unit].
	Pin_t limit_min_pin;
	Pin_t limit_max_pin;
	double home_pos;	// Position of motor (in μm) when the home switch is triggered.
	bool active;
	double limit_v, limit_a;		// maximum value for f [m/s], [m/s^2].
	uint8_t home_order;
	ARCH_MOTOR
};

struct Space;

struct SpaceType {
	void (*xyz2motors)(Space *s, double *motors);
	void (*check_position)(Space *s, double *data);
	void (*load)(Space *s, uint8_t old_type, int32_t &addr);
	void (*save)(Space *s, int32_t &addr);
	bool (*init)(Space *s);
	void (*free)(Space *s);
	void (*afree)(Space *s, int a);
	double (*change0)(Space *s, int axis, double value);
	double (*unchange0)(Space *s, int axis, double value);
	double (*probe_speed)(Space *s);
	int (*follow)(Space *s, int axis);
	void (*motors2xyz)(Space *s, double *motors, double *xyz);
};

struct Space {
	Space_History *history;
	Space_History settings;
	void *type_data;
	Motor **motor;
	Axis **axis;
	int id;
	int type;
	int num_axes, num_motors;
	void load_info(int32_t &addr);
	void load_axis(int a, int32_t &addr);
	void load_motor(int m, int32_t &addr);
	void save_info(int32_t &addr);
	void save_axis(int a, int32_t &addr);
	void save_motor(int m, int32_t &addr);
	void init(int space_id);
	bool setup_nums(int na, int nm);
	void cancel_update();
	ARCH_SPACE
};

#define DEFAULT_TYPE 0
#define EXTRUDER_TYPE 3
#define FOLLOWER_TYPE 4
void Cartesian_init(int num);
void Delta_init(int num);
void Polar_init(int num);
void Extruder_init(int num);
void Follower_init(int num);

#define setup_spacetypes() do { \
	Cartesian_init(0); \
	Delta_init(1); \
	Polar_init(2); \
	Extruder_init(3); \
	Follower_init(4); \
} while(0)
#define NUM_SPACE_TYPES 5
EXTERN SpaceType space_types[NUM_SPACE_TYPES];
EXTERN int current_extruder;

struct Gpio {
	Pin_t pin;
	uint8_t state, reset;
	void setup(uint8_t new_state);
	void load(int32_t &addr);
	void save(int32_t &addr);
	void init();
	void free();
	void copy(Gpio &dst);
};

struct MoveCommand {
	bool cb;
	bool probe, single;
	double f[2];
	double data[10];	// Value if given, NAN otherwise.  Variable size array. TODO
	double time, dist;
	bool arc;
	double center[3];
	double normal[3];
};

struct Serial_t {
	virtual void write(char c) = 0;
	virtual int read() = 0;
	virtual int readBytes (char *target, int len) = 0;
	virtual void flush() = 0;
	virtual int available() = 0;
};

struct HostSerial : public Serial_t {
	char buffer[256];
	int start, end;
	void begin();
	void write(char c);
	void refill();
	int read();
	int readBytes (char *target, int len) {
		for (int i = 0; i < len; ++i)
			*target++ = read();
		return len;
	}
	void flush() {}
	int available();
};
EXTERN HostSerial host_serial;

#define COMMAND_SIZE 256
#define FULL_SERIAL_COMMAND_SIZE (COMMAND_SIZE + (COMMAND_SIZE + 2) / 3)
#define HOST_COMMAND_SIZE 0x4000
static int const FULL_COMMAND_SIZE[2] = {HOST_COMMAND_SIZE, FULL_SERIAL_COMMAND_SIZE};

// Globals
EXTERN double max_deviation;
EXTERN double max_v;
EXTERN unsigned char uuid[UUID_SIZE];
EXTERN uint8_t num_extruders;
EXTERN uint8_t num_temps;
EXTERN uint8_t num_gpios;
EXTERN uint32_t protocol_version;
EXTERN uint8_t machine_type;		// 0: cartesian, 1: delta.
EXTERN Pin_t led_pin, stop_pin, probe_pin, spiss_pin;
EXTERN uint16_t timeout;
EXTERN int bed_id, fan_id, spindle_id;
//EXTERN double room_T;	//[°C]
EXTERN double feedrate;		// Multiplication factor for f values, used at start of move.
EXTERN double targetx, targety, targetangle, zoffset;	// Offset for axis 2 of space 0.
// Other variables.
EXTERN Serial_t *serialdev[2];
EXTERN unsigned char *command[2];
EXTERN int command_end[2];
EXTERN Space spaces[NUM_SPACES];
EXTERN Temp *temps;
EXTERN Gpio *gpios;
EXTERN FILE *store_adc;
EXTERN uint8_t temps_busy;
EXTERN MoveCommand queue[QUEUE_LENGTH];
EXTERN uint8_t continue_cb;		// is a continue event waiting to be sent out? (0: no, 1: move, 2: audio, 3: both)
EXTERN uint8_t which_autosleep;		// which autosleep message to send (0: none, 1: motor, 2: temp, 3: both)
EXTERN uint8_t ping;			// bitmask of waiting ping replies.
EXTERN bool initialized;
EXTERN int cbs_after_current_move;
EXTERN bool motors_busy;
EXTERN int out_busy;
EXTERN int32_t out_time;
EXTERN char pending_packet[4][FULL_SERIAL_COMMAND_SIZE];
EXTERN int pending_len[4];
EXTERN void (*serial_cb[4])();
EXTERN char datastore[HOST_COMMAND_SIZE];
EXTERN int32_t last_active;
EXTERN int32_t last_micros;
EXTERN int16_t led_phase;
EXTERN History *history;
EXTERN History settings;
EXTERN bool computing_move;	// True as long as steps are sent to firmware.
EXTERN bool aborting, prepared, preparing;
EXTERN int first_fragment;
EXTERN int stopping;		// From limit.
EXTERN int sending_fragment;
EXTERN bool transmitting_fragment;
EXTERN bool start_pending, stop_pending, change_pending, discarding;
EXTERN bool discard_pending;
EXTERN double done_factor;
EXTERN uint8_t requested_temp;
EXTERN bool refilling;
EXTERN int current_fragment, running_fragment;
EXTERN unsigned current_fragment_pos;
EXTERN int num_active_motors;
EXTERN int hwtime_step, audio_hwtime_step;
EXTERN struct pollfd pollfds[BASE_FDS + ARCH_MAX_FDS];
EXTERN void (*wait_for_reply[4])();
EXTERN int expected_replies;

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

// Force cpdebug if requested, to enable only specific lines without adding all the cp things in manually.
#define fcpdebug(s, m, fmt, ...) do { if (s == 1 && m == 0) debug("CP curfragment %d curpos %f current %f " fmt, current_fragment, spaces[s].motor[m]->settings.current_pos, spaces[s].axis[m]->settings.current, ##__VA_ARGS__); } while (0)
//#define fcpdebug(s, m, fmt, ...) do { debug("CP curfragment %d curpos %f current %f " fmt, current_fragment, spaces[s].motor[m]->settings.current_pos, spaces[s].axis[m]->settings.current, ##__VA_ARGS__); } while (0)
//#define cpdebug fcpdebug
#define cpdebug(...) do {} while (0)

// packet.cpp
void packet();	// A command packet has arrived; handle it.
void settemp(int which, double target);
void waittemp(int which, double mintemp, double maxtemp);
void setpos(int which, int t, double f);

// serial.cpp
bool serial(uint8_t which);	// Handle commands from serial.
bool prepare_packet(char *the_packet, int len);
void send_packet();
void write_ack();
void write_nack();
void send_host(char cmd, int s = 0, int m = 0, double f = 0, int e = 0, unsigned len = 0);
EXTERN uint8_t ff_in;	// Index of next in-packet that is expected.
EXTERN uint8_t ff_out;	// Index of next out-packet that will be sent.

// move.cpp
int next_move();
void abort_move(int pos);

// run.cpp
struct Run_Record {
	uint8_t type;
	int32_t tool;
	double X, Y, Z, E, F_start, F_end;
	double time, dist;
} __attribute__((__packed__));
struct ProbeFile {
	double targetx, targety, x0, y0, w, h, sina, cosa;
	unsigned long nx, ny;
	double angle;
	double sample[0];
} __attribute__((__packed__));
void run_file(int name_len, char const *name, int probe_name_len, char const *probe_name, bool start, double sina, double cosa, int audio);
void abort_run_file();
void run_file_fill_queue();
void run_adjust_probe(double x, double y, double z);
double run_find_pos(double pos[3]);
EXTERN char probe_file_name[256];
EXTERN off_t probe_file_size;
EXTERN ProbeFile *probe_file_map;
EXTERN char run_file_name[256];
EXTERN off_t run_file_size;
EXTERN Run_Record *run_file_map;
EXTERN int run_file_num_strings;
EXTERN off_t run_file_first_string;
EXTERN int run_file_num_records;
EXTERN int run_file_wait_temp;
EXTERN int run_file_wait;
EXTERN struct itimerspec run_file_timer;
EXTERN double run_file_refx;
EXTERN double run_file_refy;
EXTERN double run_file_refz;
EXTERN double run_file_sina;
EXTERN double run_file_cosa;
EXTERN bool run_file_finishing;
EXTERN int run_file_audio;

// setup.cpp
void setup();
void connect(char const *port, char const *run_id);
void connect_end();
Axis_History *setup_axis_history();
Motor_History *setup_motor_history();
EXTERN bool host_block;
EXTERN bool sent_names;
EXTERN bool connected;

// storage.cpp
uint8_t read_8(int32_t &address);
void write_8(int32_t &address, uint8_t data);
int16_t read_16(int32_t &address);
void write_16(int32_t &address, int16_t data);
double read_float(int32_t &address);
void write_float(int32_t &address, double data);

// temp.cpp
void handle_temp(int id, int temp);

// space.cpp
void buffer_refill();
void store_settings();
void restore_settings();
void apply_tick();
void send_fragment();
void move_to_current();
void reset_pos(Space *s);
EXTERN int moving_to_current;

// globals.cpp
bool globals_load(int32_t &address);
void globals_save(int32_t &address);

// base.cpp
void disconnect(bool notify);
int32_t utime();
int32_t millis();

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
void arch_stop(bool fake = false);
void arch_home();
bool arch_running();
double arch_round_pos(int s, int m, double pos);
void arch_stop_audio();
//void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin = ~0, bool heater_invert = false, int heater_adctemp = 0, int heater_limit_l = ~0, int heater_limit_h = ~0, int fan_pin = ~0, bool fan_invert = false, int fan_adctemp = 0, int fan_limit_l = ~0, int fan_limit_h = ~0, double hold_time = 0);
void arch_start_move(int extra);
bool arch_send_fragment();

#ifdef SERIAL
int hwpacketsize(int len, int *available);
bool hwpacket(int len);
void arch_reconnect(char *port);
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

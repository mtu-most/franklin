#ifndef _CDRIVER_H
#define _CDRIVER_H

#include "configuration.h"
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/timerfd.h>

#define PROTOCOL_VERSION ((uint32_t)0)	// Required version response in BEGIN.
#define ID_SIZE 24

#define MAXLONG (int32_t((uint32_t(1) << 31) - 1))
#define MAXINT MAXLONG

// Exactly one file defines EXTERN as empty, which leads to the data to be defined.
#ifndef EXTERN
#define EXTERN extern
#endif

#define debug(...) do { buffered_debug_flush(); fprintf(stderr, "#"); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while (0)

static inline int min(int a, int b) {
	return a < b ? a : b;
}

static inline int max(int a, int b) {
	return a > b ? a : b;
}

struct Pin_t {
	uint8_t flags;
	uint8_t pin;
	bool valid() { return flags & 1; }
	bool inverted() { return flags & 2; }
	uint16_t write() { return flags << 8 | pin; }
	void init() { flags = 0; pin = 0; }
	inline void read(uint16_t data);
};

union ReadFloat {
	float f;
	int32_t i;
	uint32_t ui;
	uint8_t b[sizeof(float)];
};

enum SingleByteHostCommands {
	OK = 0xb3,
	WAIT = 0xad
};

enum SingleByteCommands {	// See serial.cpp for computation of command values. {{{
	CMD_NACK = 0x80,	// Incorrect packet; please resend.
	CMD_ACK0 = 0xb3,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_STALL0 = 0x87,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL1 = 0x9e,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_ID = 0xaa,		// Request/reply printer ID code.
	CMD_ACK1 = 0xad,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_DEBUG = 0xb4,	// Debug message; a nul-terminated message follows (no checksum; no resend).
	CMD_STARTUP = 0x99
}; // }}}

enum Command {
	// from host
	CMD_RESET,	// 1 byte: 0.
	CMD_GOTO,	// 1-2 byte: which channels (depending on number of extruders); channel * 4 byte: values [fraction/s], [mm].  Reply (later): MOVECB.
	CMD_RUN_FILE,	// n byte: filename.
	CMD_PROBE,	// same.  Reply (later): LIMIT/MOVECB.
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
	CMD_RECONNECT,	// 1 byte: name length, n bytes: port name
	CMD_AUDIO_SETUP,	// 1-2 byte: which channels (like for goto); 2 byte: μs_per_sample.
	CMD_AUDIO_DATA,	// AUDIO_FRAGMENT_SIZE bytes: data.  Returns ACK or ACKWAIT.
	CMD_RESUME,
	// to host
		// responses to host requests; only one active at a time.
	CMD_TEMP = 0x40,	// 4 byte: requested channel's temperature. [°C]
	CMD_POWER,	// 4 byte: requested channel's power time; 4 bytes: current time. [μs, μs]
	CMD_POS,	// 4 byte: pos [steps]; 4 byte: current [mm].
	CMD_DATA,	// n byte: requested data.
	CMD_PIN,	// 1 byte: 0 or 1: pin state.
	CMD_QUEUE,	// 1 byte: current number of records in queue.
	CMD_HOMED,	// 0
		// asynchronous events.
	CMD_MOVECB,	// 1 byte: number of movecb events.
	CMD_TEMPCB,	// 1 byte: which channel.  Byte storage for which needs to be sent.
	CMD_CONTINUE,	// 1 byte: is_audio.  Bool flag if it needs to be sent.
	CMD_LIMIT,	// 1 byte: which channel.
	CMD_TIMEOUT,	// 0
	CMD_SENSE,	// 1 byte: which channel (b0-6); new state (b7); 4 byte: motor position at trigger.
	CMD_DISCONNECT,	// 0
		// Updates from RUN_FILE.
	CMD_UPDATE_TEMP,
	CMD_UPDATE_PIN,
	CMD_CONFIRM,
	CMD_FILE_DONE,
};

// All temperatures are stored in Kelvin, but communicated in °C.
struct Temp
{
	// See temp.c from definition of calibration constants.
	float R0, R1, logRc, beta, Tc;	// calibration values of thermistor.  [Ω, Ω, logΩ, K, K]
	/*
	// Temperature balance calibration.
	float power;			// added power while heater is on.  [W]
	float core_C;			// heat capacity of the core.  [J/K]
	float shell_C;		// heat capacity of the shell.  [J/K]
	float transfer;		// heat transfer between core and shell.  [W/K]
	float radiation;		// radiated power = radiation * (shell_T ** 4 - room_T ** 4) [W/K**4]
	float convection;		// convected power = convection * (shell_T - room_T) [W/K]
	*/
	// Pins.
	Pin_t power_pin[2];
	Pin_t thermistor_pin;
	// Volatile variables.
	float target[2];			// target temperature; NAN to disable. [K]
	int32_t adctarget[2];		// target temperature in adc counts; -1 for disabled. [adccounts]
	int32_t adclast;		// last measured temperature. [adccounts]
	/*
	float core_T, shell_T;	// current temperatures. [K]
	*/
	uint8_t following_gpios;	// linked list of gpios monitoring this temp.
	float min_alarm;		// NAN, or the temperature at which to trigger the callback.  [K]
	float max_alarm;		// NAN, or the temperature at which to trigger the callback.  [K]
	int32_t adcmin_alarm;		// -1, or the temperature at which to trigger the callback.  [adccounts]
	int32_t adcmax_alarm;		// -1, or the temperature at which to trigger the callback.  [adccounts]
	// Internal variables.
	uint32_t last_temp_time;	// last value of micros when this heater was handled.
	uint32_t time_on;		// Time that the heater has been on since last reading.  [μs]
	bool is_on[2];			// If the heater is currently on.
	float K;			// Thermistor constant; kept in memory for performance.
	// Functions.
	int32_t get_value();		// Get thermistor reading, or -1 if it isn't available yet.
	float fromadc(int32_t adc);	// convert ADC to K.
	int32_t toadc(float T, int32_t default_);	// convert K to ADC.
	void load(int32_t &addr, int id);
	void save(int32_t &addr);
	void init();
	void free();
	void copy(Temp &dst);
};

struct History
{
	float t0, tp;
	float f0, f1, f2, fp, fq, fmain;
	int fragment_length;
	int num_active_motors;
	uint32_t hwtime, start_time, last_time, last_current_time;
	int cbs;
	int queue_start, queue_end;
	bool queue_full;
	int run_file_current;
};

struct Motor_History
{
	int dir;
	char *data;
	float last_v;		// v during last iteration, for using limit_a [m/s].
	float target_v, target_dist;	// Internal values for moving.
	int32_t current_pos, hwcurrent_pos;	// Current position of motor (in steps), and what the hardware currently thinks.
	float endpos;
};

struct Axis_History
{
	float dist, next_dist, main_dist;
	float source, current;	// Source position of current movement of axis (in μm), or current position if there is no movement.
	float target;
};

struct Axis
{
	Axis_History *settings;
	float offset;		// Position where axis claims to be when it is at 0.
	float park;		// Park position; not used by the firmware, but stored for use by the host.
	uint8_t park_order;
	float max_v;
	float min_pos, max_pos;
	void *type_data;
};

struct Motor
{
	Motor_History *settings;
	Pin_t step_pin;
	Pin_t dir_pin;
	Pin_t enable_pin;
	float steps_per_unit;			// hardware calibration [steps/unit].
	uint8_t max_steps;			// maximum number of steps in one iteration.
	Pin_t limit_min_pin;
	Pin_t limit_max_pin;
	float home_pos;	// Position of motor (in μm) when the home switch is triggered.
	Pin_t sense_pin;
	uint8_t sense_state;
	float sense_pos;
	float limit_v, limit_a;		// maximum value for f [m/s], [m/s^2].
	uint8_t home_order;
#ifdef HAVE_AUDIO
	uint8_t audio_flags;
	enum Flags {
		PLAYING = 1,
		STATE = 2
	};
#endif
};

struct Space;

struct SpaceType
{
	void (*xyz2motors)(Space *s, float *motors, bool *ok);
	void (*reset_pos)(Space *s);
	void (*check_position)(Space *s, float *data);
	void (*load)(Space *s, uint8_t old_type, int32_t &addr);
	void (*save)(Space *s, int32_t &addr);
	bool (*init)(Space *s);
	void (*free)(Space *s);
	void (*afree)(Space *s, int a);
	float (*change0)(Space *s, int axis, float value);
	float (*unchange0)(Space *s, int axis, float value);
};

struct Space
{
	uint8_t type;
	uint8_t id;
	void *type_data;
	float max_deviation;
	uint8_t num_axes, num_motors;
	Motor **motor;
	Axis **axis;
	void load_info(int32_t &addr);
	void load_axis(uint8_t a, int32_t &addr);
	void load_motor(uint8_t m, int32_t &addr);
	void save_info(int32_t &addr);
	void save_axis(uint8_t a, int32_t &addr);
	void save_motor(uint8_t m, int32_t &addr);
	void init(uint8_t space_id);
	void free();
	void copy(Space &dst);
	bool setup_nums(uint8_t na, uint8_t nm);
	void cancel_update();
};

#define DEFAULT_TYPE 0
void Cartesian_init(int num);
void Delta_init(int num);
void Extruder_init(int num);

#define NUM_SPACE_TYPES 3
EXTERN SpaceType space_types[NUM_SPACE_TYPES];
#define setup_spacetypes() do { \
	Cartesian_init(0); \
	Delta_init(1); \
	Extruder_init(2); \
} while(0)
EXTERN int current_extruder;

struct Gpio
{
	Pin_t pin;
	uint8_t state, reset;
	void setup(uint8_t new_state);
	void load(uint8_t self, int32_t &addr);
	void save(int32_t &addr);
	void init();
	void free();
	void copy(Gpio &dst);
};

struct MoveCommand
{
	bool cb;
	bool probe;
	float f[2];
	float data[10];	// Value if given, NAN otherwise.  Variable size array. TODO
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
	void begin(int baud);
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
		if (start == end)
			refill();
		return end - start;
	}
};
EXTERN HostSerial host_serial;

#define COMMAND_SIZE 127
#define COMMAND_LEN_MASK 0x7f
#define FULL_COMMAND_SIZE (COMMAND_SIZE + (COMMAND_SIZE + 2) / 3)

// Globals
EXTERN uint8_t num_spaces;
EXTERN uint8_t num_extruders;
EXTERN uint8_t num_temps;
EXTERN uint8_t num_gpios;
EXTERN uint32_t protocol_version;
EXTERN uint8_t printer_type;		// 0: cartesian, 1: delta.
EXTERN Pin_t led_pin, probe_pin;
EXTERN uint16_t timeout;
//EXTERN float room_T;	//[°C]
EXTERN float feedrate;		// Multiplication factor for f values, used at start of move.
// Other variables.
EXTERN Serial_t *serialdev[2];
EXTERN unsigned char command[2][FULL_COMMAND_SIZE];
EXTERN uint8_t command_end[2];
EXTERN Space *spaces;
EXTERN Temp *temps;
EXTERN Gpio *gpios;
EXTERN uint8_t temps_busy;
EXTERN MoveCommand queue[QUEUE_LENGTH];
EXTERN bool probing;
EXTERN uint8_t continue_cb;		// is a continue event waiting to be sent out? (0: no, 1: move, 2: audio, 3: both)
EXTERN uint8_t which_autosleep;		// which autosleep message to send (0: none, 1: motor, 2: temp, 3: both)
EXTERN uint8_t ping;			// bitmask of waiting ping replies.
EXTERN bool initialized;
EXTERN int cbs_after_current_move;
EXTERN bool motors_busy;
EXTERN bool out_busy;
EXTERN uint32_t out_time;
EXTERN char pending_packet[FULL_COMMAND_SIZE];
EXTERN int pending_len;
EXTERN char datastore[FULL_COMMAND_SIZE];
EXTERN uint32_t last_active;
EXTERN int16_t led_phase;
EXTERN uint8_t temp_current;
EXTERN History *settings;
#ifdef HAVE_AUDIO
EXTERN uint8_t audio_buffer[AUDIO_FRAGMENTS][AUDIO_FRAGMENT_SIZE];
EXTERN uint8_t audio_head, audio_tail, audio_state;
EXTERN uint32_t audio_start;
EXTERN int16_t audio_us_per_sample;
#endif
EXTERN bool moving, aborting, stopped, prepared;
EXTERN int first_fragment;
EXTERN int stopping;		// From limit.
EXTERN int sending_fragment;	// To compute how many fragments are in use from free_fragments.
EXTERN bool start_pending, stop_pending;
EXTERN float done_factor;
EXTERN uint8_t requested_temp;
EXTERN bool refilling;
EXTERN int current_fragment;
EXTERN int current_fragment_pos;
EXTERN int hwtime_step;
EXTERN int free_fragments;
EXTERN struct pollfd pollfds[3];

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
//#define cpdebug debug
#define cpdebug(...) do {} while(0)

// packet.cpp
void packet();	// A command packet has arrived; handle it.
void settemp(int which, float target);
void waittemp(int which, float mintemp, float maxtemp);
void setpos(int which, int t, int f);

// serial.cpp
void serial(uint8_t which);	// Handle commands from serial.
void prepare_packet(char *the_packet, int len);
void send_packet();
void write_ack();
void write_stall();
void send_host(char cmd, int s = 0, int m = 0, float f = 0, int e = 0, int len = 0);

// move.cpp
uint8_t next_move();
void abort_move(int pos);

// run.cpp
struct Run_Record {
	uint8_t type;
	int32_t tool;
	float x, X, y, Y, z, Z, e, E, f, F;
} __attribute__((__packed__));
void run_file(int name_len, char const *name, float refx, float refy, float refz, float sina, float cosa);
void abort_run_file();
void run_file_fill_queue();
EXTERN char run_file_name[256];
EXTERN off_t run_file_size;
EXTERN Run_Record *run_file_map;
EXTERN int run_file_num_strings;
EXTERN off_t run_file_first_string;
EXTERN int run_file_num_records;
EXTERN int run_file_wait_temp;
EXTERN int run_file_wait;
EXTERN struct itimerspec run_file_timer;
EXTERN float run_file_refx;
EXTERN float run_file_refy;
EXTERN float run_file_refz;
EXTERN float run_file_sina;
EXTERN float run_file_cosa;

// setup.cpp
void setup(char const *port, char const *run_id);

// storage.cpp
uint8_t read_8(int32_t &address);
void write_8(int32_t &address, uint8_t data);
int16_t read_16(int32_t &address);
void write_16(int32_t &address, int16_t data);
float read_float(int32_t &address);
void write_float(int32_t &address, float data);

// temp.cpp
void handle_temp(int id, int temp);

// space.cpp
void reset_dirs(int fragment, bool allow_new);
void buffer_refill();
void copy_fragment_settings(int src, int dst);
void apply_tick();
void send_fragment();

// globals.cpp
bool globals_load(int32_t &address);
void globals_save(int32_t &address);

// base.cpp
void reset();
void disconnect();
uint32_t utime();
uint32_t millis();

#include ARCH_INCLUDE

// ===============
// Arch interface.
// ===============
// Defined or variables:
// NUM_ANALOG_INPUTS
// NUM_DIGITAL_PINS
// ADCBITS
// FRAGMENTS_PER_BUFFER
// BYTES_PER_FRAGMENT
static inline void SET_INPUT(Pin_t _pin);
static inline void SET_INPUT_NOPULLUP(Pin_t _pin);
static inline void RESET(Pin_t _pin);
static inline void SET(Pin_t _pin);
static inline void SET_OUTPUT(Pin_t _pin);
static inline bool GET(Pin_t _pin, bool _default);
static inline void arch_setup_start(char const *port);
static inline void arch_setup_end(char const *run_id);
static inline void arch_motors_change();
static inline void arch_addpos(int s, int m, int diff);
static inline void arch_stop();
static inline void arch_home();
static inline bool arch_running();
//static inline void arch_setup_temp(int which, int thermistor_pin, int active, int power_pin = -1, bool power_inverted = true, int power_target = 0, int fan_pin = -1, bool fan_inverted = false, int fan_target = 0);
static inline void arch_start_move(int extra);
static inline void arch_send_fragment(int fragment);

#ifdef SERIAL
static inline int hwpacketsize(int len, int *available);
static inline void hwpacket(int len);
static inline void arch_reconnect(char *port);
static inline void arch_disconnect();
static inline int arch_fds();
// Serial_t derivative Serial;
//static inline void arch_pin_set_reset(Pin_t pin_, int state);
static inline void START_DEBUG();
static inline void DO_DEBUG(char c);
static inline void END_DEBUG();
#endif


void Pin_t::read(uint16_t data) {
	int new_pin = data & 0xff;
	int new_flags = data >> 8;
	if (new_pin != pin || new_flags != flags) {
		SET_INPUT_NOPULLUP(*this);
#ifdef SERIAL
		// Reset is not recorded on connections that cannot fail.
		arch_pin_set_reset(*this, 3);
#endif
	}
	pin = new_pin;
	flags = new_flags;
	if (flags & ~3 || pin >= NUM_DIGITAL_PINS) {
		flags = 0;
		pin = 0;
	}
}
#endif

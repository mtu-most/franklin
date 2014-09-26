#ifndef _FIRMWARE_H
#define _FIRMWARE_H

#include "configuration.h"
#if !defined(HAVE_SPACES) && defined(HAVE_AUDIO)
#error Cannot have audio without spaces.
#endif
#include ARCH_INCLUDE
#include <math.h>
#include <stdarg.h>

#define ID_SIZE 8	// Number of bytes in printerid.

#define MAXLONG (int32_t((uint32_t(1) << 31) - 1))
#define MAXINT (int16_t((uint16_t(1) << 15) - 1))

// Exactly one file defines EXTERN as empty, which leads to the data to be defined.
#ifndef EXTERN
#define EXTERN extern
#endif

struct Pin_t {
	uint8_t flags;
	uint8_t pin;
	bool valid() { return flags & 1; }
	bool inverted() { return flags & 2; }
	uint16_t write() { return flags << 8 | pin; }
	void read(uint16_t data) {
		if ((data & 0xff) != pin)
			SET_INPUT_NOPULLUP(*this);
		pin = data & 0xff;
		flags = data >> 8;
		if (flags & ~3 || pin >= NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS) {
			flags = 0;
			pin = 0;
		}
	}
};

union ReadFloat {
	float f;
	uint32_t ui;
	uint8_t b[sizeof(float)];
};

enum SingleByteCommands {	// See serial.cpp for computation of command values.
// These bytes (except RESET) are sent in reply to a received packet only.
	CMD_NACK = 0x80,	// Incorrect packet; please resend.
	CMD_ACK0 = 0xb3,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACKWAIT0 = 0xb4,	// Packet properly received and accepted, but queue is full so no new GOTO commands are allowed until CONTINUE.  (Never sent from host.)
	CMD_STALL = 0x87,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_ACKWAIT1 = 0x99,	// Printer started and is ready for commands.
	CMD_ID = 0xaa,		// Request/reply printer ID code.
	CMD_ACK1 = 0xad,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_DEBUG = 0x9e	// Debug message; a nul-terminated message follows (no checksum; no resend).
};

enum Command {
	// from host
	CMD_BEGIN,	// 4 byte: 0 (preferred protocol version). Reply: START.
	CMD_PING,	// 1 byte: code.  Reply: PONG.
	CMD_RESET,	// 1 byte: 0.
	CMD_GOTO,	// 1-2 byte: which channels (depending on number of extruders); channel * 4 byte: values [fraction/s], [mm].
	CMD_GOTOCB,	// same.  Reply (later): MOVECB.
	CMD_SLEEP,	// 1 byte: which channel (b0-6); on/off (b7 = 1/0).
	CMD_SETTEMP,	// 1 byte: which channel; 4 bytes: target [°C].
	CMD_WAITTEMP,	// 1 byte: which channel; 4 bytes: lower limit; 4 bytes: upper limit [°C].  Reply (later): TEMPCB.  Disable with WAITTEMP (NAN, NAN).
	CMD_READTEMP,	// 1 byte: which channel.  Reply: TEMP. [°C]
	CMD_READPOWER,	// 1 byte: which channel.  Reply: POWER. [μs, μs]
	CMD_SETPOS,	// 1 byte: which channel; 4 bytes: pos.
	CMD_GETPOS,	// 1 byte: which channel.  Reply: POS. [steps, mm]
	CMD_LOAD,	// 1 byte: which channel.
	CMD_SAVE,	// 1 byte: which channel.
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
	CMD_AUDIO_SETUP,	// 1-2 byte: which channels (like for goto); 2 byte: μs_per_sample.
	CMD_AUDIO_DATA,	// AUDIO_FRAGMENT_SIZE bytes: data.  Returns ACK or ACKWAIT.
	// to host
		// responses to host requests; only one active at a time.
	CMD_START,	// 4 byte: 0 (protocol version).
	CMD_TEMP,	// 4 byte: requested channel's temperature. [°C]
	CMD_POWER,	// 4 byte: requested channel's power time; 4 bytes: current time. [μs, μs]
	CMD_POS,	// 4 byte: pos [steps]; 4 byte: current [mm].
	CMD_DATA,	// n byte: requested data.
	CMD_PONG,	// 1 byte: PING argument.
	CMD_PIN,	// 1 byte: 0 or 1: pin state.
	CMD_QUEUE,	// 1 byte: current number of records in queue.
		// asynchronous events.
	CMD_MOVECB,	// 1 byte: number of movecb events.
	CMD_TEMPCB,	// 1 byte: which channel.  Byte storage for which needs to be sent.
	CMD_CONTINUE,	// 1 byte: is_audio.  Bool flag if it needs to be sent.
	CMD_LIMIT,	// 1 byte: which channel.
	CMD_AUTOSLEEP,	// 1 byte: what: 1: motor; 2: temp; 3: both.
	CMD_SENSE,	// 1 byte: which channel (b0-6); new state (b7); 4 byte: motor position at trigger.
};

#ifdef HAVE_TEMPS
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
	Pin_t power_pin;
	Pin_t thermistor_pin;
	// Volatile variables.
	float target;			// target temperature; NAN to disable. [K]
	int16_t adctarget;		// target temperature in adc counts; -1 for disabled. [adccounts]
	int16_t adclast;		// last measured temperature. [adccounts]
	/*
	float core_T, shell_T;	// current temperatures. [K]
	*/
#ifdef HAVE_GPIOS
	uint8_t following_gpios;	// linked list of gpios monitoring this temp.
#endif
	float min_alarm;		// NAN, or the temperature at which to trigger the callback.  [K]
	float max_alarm;		// NAN, or the temperature at which to trigger the callback.  [K]
	int16_t adcmin_alarm;		// -1, or the temperature at which to trigger the callback.  [adccounts]
	int16_t adcmax_alarm;		// -1, or the temperature at which to trigger the callback.  [adccounts]
	bool alarm;
	// Internal variables.
	unsigned long last_time;	// last value of micros when this heater was handled.
	unsigned long time_on;		// Time that the heater has been on since last reading.  [μs]
	bool is_on;			// If the heater is currently on.
	float K;			// Thermistor constant; kept in memory for performance.
	// Functions.
	int16_t get_value();		// Get thermistor reading, or -1 if it isn't available yet.
	float fromadc(int16_t adc);	// convert ADC to K.
	int16_t toadc(float T);	// convert K to ADC.
	void load(int16_t &addr, bool eeprom);
	void save(int16_t &addr, bool eeprom);
	void init();
	void free();
	void copy(Temp &dst);
	static int16_t savesize0();
	int16_t savesize() { return savesize0(); }
};
#endif

#ifdef HAVE_SPACES
struct Axis
{
	float offset;		// Position where axis claims to be when it is at 0.
	float park;		// Park position; not used by the firmware, but stored for use by the host.
	uint8_t park_order;
	float source, current;	// Source position of current movement of axis (in μm), or current position if there is no movement.
	float max_v;
	float min, max;
	float dist, next_dist, main_dist;
	float target;
};

struct Motor
{
	Pin_t step_pin;
	Pin_t dir_pin;
	Pin_t enable_pin;
	float steps_per_m;			// hardware calibration [steps/m].
	uint8_t max_steps;			// maximum number of steps in one iteration.
	Pin_t limit_min_pin;
	Pin_t limit_max_pin;
	float home_pos;	// Position of motor (in μm) when the home switch is triggered.
	Pin_t sense_pin;
	uint8_t sense_state;
	float sense_pos;
	float limits_pos;	// position when limit switch was hit or nan
	float limit_v, limit_a;		// maximum value for f [m/s], [m/s^2].
	uint8_t home_order;
	float last_v;		// v during last iteration, for using limit_a [m/s].
	int16_t steps;
	float target_v, target_dist;	// Internal values for moving.
	float current_pos;	// Current position of motor (in μm).
	float endpos;
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
	void (*load)(Space *s, uint8_t old_type, int16_t &addr, bool eeprom);
	void (*save)(Space *s, int16_t &addr, bool eeprom);
	bool (*init)(Space *s);
	void (*free)(Space *s);
	int16_t (*savesize)(Space *s);
#ifdef HAVE_EXTRUDER
	bool (*change0)(Space *s);
#endif
};

struct Space
{
	uint8_t type;
	void *type_data;
	float max_deviation;
	uint8_t num_axes, num_motors;
	Motor **motor;
	Axis **axis;
	bool active;
	void load_info(int16_t &addr, bool eeprom);
	void load_axis(uint8_t a, int16_t &addr, bool eeprom);
	void load_motor(uint8_t m, int16_t &addr, bool eeprom);
	void save_info(int16_t &addr, bool eeprom);
	void save_axis(uint8_t a, int16_t &addr, bool eeprom);
	void save_motor(uint8_t m, int16_t &addr, bool eeprom);
	void init();
	void free();
	void copy(Space &dst);
	static int16_t savesize0();
	int16_t savesize();
	bool setup_nums(uint8_t na, uint8_t nm);
	void cancel_update();
	int16_t savesize_std();
};

// Type 0: Extruder.
#ifdef HAVE_EXTRUDER
void Extruder_init(uint8_t num);
#define EXTRUDER_INIT(num) Extruder_init(num);
#define HAVE_TYPE_EXTRUDER true
#else
#define EXTRUDER_INIT(num)
#define HAVE_TYPE_EXTRUDER false
#endif

// Type 1: Cartesian (always available).
#define DEFAULT_TYPE 1
void Cartesian_init(uint8_t num);
#define CARTESIAN_INIT(num) Cartesian_init(num);
#define HAVE_TYPE_CARTESIAN true

// Type 2: Delta.
#ifdef HAVE_DELTA
void Delta_init(uint8_t num);
#define DELTA_INIT(num) Delta_init(num);
#define HAVE_TYPE_DELTA true
#else
#define DELTA_INIT(num)
#define HAVE_TYPE_DELTA false
#endif

#define NUM_SPACE_TYPES 3
EXTERN bool have_type[NUM_SPACE_TYPES];
EXTERN SpaceType space_types[NUM_SPACE_TYPES];
#define setup_spacetypes() do { \
	EXTRUDER_INIT(0) \
	have_type[0] = HAVE_TYPE_EXTRUDER; \
	CARTESIAN_INIT(1) \
	have_type[1] = HAVE_TYPE_CARTESIAN; \
	DELTA_INIT(2) \
	have_type[2] = HAVE_TYPE_DELTA; \
} while(0)
#endif

#ifdef HAVE_GPIOS
struct Gpio
{
	Pin_t pin;
	uint8_t state;
#ifdef HAVE_TEMPS
	uint8_t master;
	float value;
	int16_t adcvalue;
	uint8_t prev, next;
#endif
	void setup(uint8_t new_state);
	void load(uint8_t self, int16_t &addr, bool eeprom);
	void save(int16_t &addr, bool eeprom);
	void init();
	void free();
	void copy(Gpio &dst);
	static int16_t savesize0();
	int16_t savesize() { return savesize0(); }
};
#endif

struct MoveCommand
{
	bool cb;
	float f[2];
	float data[10];	// Value if given, NAN otherwise.  Variable size array. TODO
};

#define COMMAND_SIZE 127
#define COMMAND_LEN_MASK 0x7f

// Globals
EXTERN char *name;
EXTERN uint8_t namelen;
#ifdef HAVE_SPACES
EXTERN uint8_t num_spaces;
#endif
#ifdef HAVE_EXTRUDERS
EXTERN uint8_t num_extruders;
#endif
#ifdef HAVE_TEMPS
EXTERN uint8_t num_temps, bed_id;
#endif
#ifdef HAVE_GPIOS
EXTERN uint8_t num_gpios;
#endif
EXTERN uint8_t printer_type;		// 0: cartesian, 1: delta.
EXTERN Pin_t led_pin, probe_pin;
EXTERN float probe_dist, probe_safe_dist;
//EXTERN float room_T;	//[°C]
EXTERN float feedrate;		// Multiplication factor for f values, used at start of move.
// Other variables.
EXTERN char printerid[ID_SIZE];
EXTERN unsigned char command[COMMAND_SIZE];
EXTERN uint8_t command_end;
EXTERN char reply[COMMAND_SIZE];
EXTERN char out_buffer[16];
#ifdef HAVE_SPACES
EXTERN Space *spaces;
#endif
#ifdef HAVE_TEMPS
EXTERN Temp *temps;
#endif
#ifdef HAVE_GPIOS
EXTERN Gpio *gpios;
#endif
EXTERN uint8_t temps_busy;
EXTERN MoveCommand queue[QUEUE_LENGTH];
EXTERN uint8_t queue_start, queue_end;
EXTERN bool queue_full;
EXTERN uint8_t num_movecbs;		// number of event notifications waiting to be sent out.
EXTERN uint8_t continue_cb;		// is a continue event waiting to be sent out? (0: no, 1: move, 2: audio, 3: both)
EXTERN uint8_t which_autosleep;		// which autosleep message to send (0: none, 1: motor, 2: temp, 3: both)
EXTERN uint8_t ping;			// bitmask of waiting ping replies.
EXTERN bool initialized;
EXTERN bool motors_busy;
EXTERN bool out_busy;
EXTERN unsigned long out_time;
EXTERN bool reply_ready;
EXTERN char *last_packet;
EXTERN unsigned long last_active, led_last;
EXTERN float motor_limit, temp_limit;
EXTERN int16_t led_phase;
EXTERN uint8_t adc_phase;
#ifdef HAVE_TEMPS
EXTERN uint8_t temp_current;
#endif
#ifdef HAVE_AUDIO
EXTERN uint8_t audio_buffer[AUDIO_FRAGMENTS][AUDIO_FRAGMENT_SIZE];
EXTERN uint8_t audio_head, audio_tail, audio_state;
EXTERN unsigned long audio_start;
EXTERN int16_t audio_us_per_sample;
#endif
EXTERN unsigned long start_time, last_time;
EXTERN float t0, tp;
EXTERN bool moving;
EXTERN float f0, f1, f2, fp, fq, fmain;
EXTERN bool move_prepared;
EXTERN uint8_t cbs_after_current_move;
EXTERN float done_factor;
EXTERN uint8_t requested_temp;

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

// packet.cpp
void packet();	// A command packet has arrived; handle it.

// serial.cpp
void serial();	// Handle commands from serial.
void send_packet(char *the_packet);
void try_send_next();
void write_ack();
void write_ackwait();

// move.cpp
uint8_t next_move();
void abort_move();

// setup.cpp
void setup();
void load_all();

// firmware.ino
void loop();	// Do stuff which needs doing: moving motors and adjusting heaters.

// storage.cpp
uint8_t read_8(int16_t &address, bool eeprom);
void write_8(int16_t &address, uint8_t data, bool eeprom);
int16_t read_16(int16_t &address, bool eeprom);
void write_16(int16_t &address, int16_t data, bool eeprom);
float read_float(int16_t &address, bool eeprom);
void write_float(int16_t &address, float data, bool eeprom);

// globals.cpp
bool globals_load(int16_t &address, bool eeprom);
void globals_save(int16_t &address, bool eeprom);
int16_t globals_savesize();

#include ARCH_INCLUDE

#endif

#ifndef _FIRMWARE_H

/*
Units:
x: μm		μ1
v: μm/s		1/s	-> float
a: μm/s²		-> float
t: μs	(but motor_limit, temp_limit: ms)
T: mK	(but β, Tc: K (float))
feedrate: 1		-> float
α: rad			-> float
R: Ω			-> float

# TODO: determine reasonable values.
C: mJ/K
J: mJ/Ks
I: mJ/K⁴
P: mW
*/

#define u(x) (int32_t(x) >> 20)
#define m(x) (int32_t(x) >> 10)
#define m2(x) (int32_t(x) >> 5)
#define mf(x) (float(x) / (1 << 10))
#define k(x) int32_t((x) * (1 << 10))
#define kf(x) (float(x) * (1 << 10))
#define kM(x) int32_t((x) * (1 << 15))
#define M(x) int32_t((x) * ((int32_t)1 << 20))

#include "configuration.h"
#include ARCH_INCLUDE
#define _FIRMWARE_H
#include <math.h>
#include <stdarg.h>

#ifdef LOWMEM
// Adjust all settings to low memory limits.
#undef NAMELEN
#define NAMELEN 8
#undef QUEUE_LENGTH
#define QUEUE_LENGTH 1
#undef MAXAXES
#define MAXAXES 2
#undef MAXEXTRUDERS
#define MAXEXTRUDERS 0
#undef MAXTEMPS
#define MAXTEMPS 0
#undef MAXGPIOS
#define MAXGPIOS 0
#ifdef AUDIO
#undef AUDIO
#endif
#ifdef WATCHDOG
#undef WATCHDOG
#endif
#endif // LOWMEM

#define ID_SIZE 8	// Number of bytes in printerid.
#define MAXOBJECT (2 + MAXAXES + MAXEXTRUDERS + MAXTEMPS + MAXGPIOS)		// Total number of supported objects.

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
		if ((flags & ~2) != 1 || pin >= NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS) {
			pin = 0;
			flags = 0;
		}
	}
};

union ReadFloat {
	float f;
	int32_t i;
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
	CMD_RUN,	// 1 byte: which channel (b0-6).  4 byte: speed [mm/s] (0 means off).
	CMD_SLEEP,	// 1 byte: which channel (b0-6); on/off (b7 = 1/0).
	CMD_SETTEMP,	// 1 byte: which channel; 4 bytes: target [°C].
	CMD_WAITTEMP,	// 1 byte: which channel; 4 bytes: lower limit; 4 bytes: upper limit [°C].  Reply (later): TEMPCB.  Disable with WAITTEMP (NAN, NAN).
	CMD_READTEMP,	// 1 byte: which channel.  Reply: TEMP. [°C]
	CMD_READPOWER,	// 1 byte: which channel.  Reply: POWER. [μs, μs]
	CMD_SETPOS,	// 1 byte: which channel; 4 bytes: pos.
	CMD_GETPOS,	// 1 byte: which channel.  Reply: POS. [steps, mm]
	CMD_LOAD,	// 1 byte: which channel.
	CMD_SAVE,	// 1 byte: which channel.
	CMD_READ,	// 1 byte: which channel.  Reply: DATA.
	CMD_WRITE,	// 1 byte: which channel; n bytes: data.
	CMD_QUEUED,	// 1 byte: 0: query queue length; 1: stop and query queue length.  Reply: QUEUE.
	CMD_READGPIO,	// 1 byte: which channel. Reply: GPIO.
	CMD_AUDIO_SETUP,	// 1-2 byte: which channels (like for goto); 2 byte: μs_per_bit.
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

struct Object
{
	int16_t address;
	virtual void load(int16_t &address, bool eeprom) = 0;
	virtual void save(int16_t &address, bool eeprom) = 0;
	virtual ~Object() {}
};

#if MAXGPIOS > 0
// Need this declaration to have the pointer in Temp.
struct Gpio;
#endif

#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
// All temperatures are stored in Kelvin, but communicated in °C.
struct Temp : public Object
{
	// See temp.c from definition of calibration constants.
	float R0, R1, logRc, beta, Tc;	// calibration values of thermistor.  [Ω, Ω, logΩ, K, K]
#ifndef LOWMEM
	// Temperature balance calibration.
	int32_t power;			// added power while heater is on.  [W]
	int32_t core_C;			// heat capacity of the core.  [J/K]
	int32_t shell_C;		// heat capacity of the shell.  [J/K]
	int32_t transfer;		// heat transfer between core and shell.  [W/K]
	int32_t radiation;		// radiated power = radiation * (shell_T ** 4 - room_T ** 4) [W/K**4]
	int32_t convection;		// convected power = convection * (shell_T - room_T) [W/K]
#endif
	// Pins.
	Pin_t power_pin;
	Pin_t thermistor_pin;
	// Volatile variables.
	int32_t target;			// target temperature; NAN to disable. [K]
	int16_t adctarget;		// target temperature in adc counts; -1 for disabled. [adccounts]
	int16_t adclast;		// last measured temperature. [adccounts]
#ifndef LOWMEM
	int32_t core_T, shell_T;	// current temperatures. [K]
#if MAXGPIOS > 0
	Gpio *gpios;			// linked list of gpios monitoring this temp.
#endif
#endif
	int32_t min_alarm;		// NAN, or the temperature at which to trigger the callback.  [K]
	int32_t max_alarm;		// NAN, or the temperature at which to trigger the callback.  [K]
	int16_t adcmin_alarm;		// -1, or the temperature at which to trigger the callback.  [adccounts]
	int16_t adcmax_alarm;		// -1, or the temperature at which to trigger the callback.  [adccounts]
	// Internal variables.
	unsigned long last_time;	// last value of micros when this heater was handled.
	unsigned long time_on;		// Time that the heater has been on since last reading.  [μs]
	bool is_on;			// If the heater is currently on.
	float K;			// Thermistor constant; kept in memory for performance.
	// Functions.
	void setup_read();		// Initialize ADC for reading the thermistor.
	int16_t get_value();		// Get thermistor reading, or -1 if it isn't available yet.
	int32_t fromadc(int16_t adc);	// convert ADC to K.
	int16_t toadc(int32_t T);	// convert K to ADC.
	virtual void load(int16_t &addr, bool eeprom);
	virtual void save(int16_t &addr, bool eeprom);
	virtual ~Temp() {}
};
#endif

#if MAXEXTRUDERS > 0 || MAXAXES > 0
struct Motor : public Object
{
	Pin_t step_pin;
	Pin_t dir_pin;
	Pin_t enable_pin;
	int32_t steps_per_m;			// hardware calibration [steps/m].
	float max_v, limit_v, limit_a;		// maximum value for f [μm/s], [mm/s^2]!.
	uint8_t max_steps;			// maximum number of steps in one iteration.
	float continuous_v;			// speed for continuous run.
	int32_t continuous_f;			// fractional continuous distance that has been done.
	float f;
	unsigned long last_time;		// micros value when last iteration was run.
	float last_v;				// v at last time, for using limit_a [mm/s].
	bool positive;				// direction of current movement.
	int32_t dist, next_dist, main_dist;
#ifdef AUDIO
	uint8_t audio_flags;
	enum Flags {
		PLAYING = 1,
		STATE = 2
	};
#endif
	virtual void load(int16_t &addr, bool eeprom);
	virtual void save(int16_t &addr, bool eeprom);
	virtual ~Motor() {}
};
#endif

struct Constants : public Object
{
	virtual void load(int16_t &addr, bool eeprom);	// NI
	virtual void save(int16_t &addr, bool eeprom);
	virtual ~Constants() {}
};

struct Variables : public Object
{
	virtual void load(int16_t &addr, bool eeprom);
	virtual void save(int16_t &addr, bool eeprom);
	virtual ~Variables() {}
};

#if MAXAXES > 0
struct Axis : public Object
{
	Motor motor;
	int32_t limit_pos;	// Position of motor (in μm) when the limit switch is triggered.
	int32_t delta_length, delta_radius;	// Calibration values for delta: length of the tie rod and the horizontal distance between the vertical position and the zero position.
	int32_t offset;		// Position where axis claims to be when it is at 0.
	int32_t park;		// Park position; not used by the firmware, but stored for use by the host.
	int32_t axis_min, axis_max;	// Limits for the movement of this axis.
	int32_t motor_min, motor_max;	// Limits for the movement of this motor.
	Pin_t limit_min_pin;
	Pin_t limit_max_pin;
	Pin_t sense_pin;
	uint8_t sense_state;
	int32_t sense_pos;
	int32_t current_pos;	// Current position of motor (in μm).
	int32_t source, current;	// Source position of current movement of axis (in μm), or current position if there is no movement.
	int32_t x, y, z;		// Position of tower on the base plane, and the carriage height at zero position; only used for delta printers.
	virtual void load(int16_t &addr, bool eeprom);
	virtual void save(int16_t &addr, bool eeprom);
	virtual ~Axis() {}
};
#endif

#if MAXEXTRUDERS > 0
struct Extruder : public Object
{
	Motor motor;
	Temp temp;		// temperature regulation.
#ifndef LOWMEM
	// TODO: Actually use this and decide on units.
	int32_t filament_heat;	// constant for how much heat is extracted per mm filament.
	int32_t nozzle_size;
	int32_t filament_size;
	int32_t capacity;	// heat capacity of filament in [energy]/mm/K
#endif
	int32_t distance_done;	// steps done during current move.
	virtual void load(int16_t &addr, bool eeprom);
	virtual void save(int16_t &addr, bool eeprom);
	virtual ~Extruder() {}
};
#endif

#if MAXGPIOS > 0
struct Gpio : public Object
{
	Pin_t pin;
	// bit 0: low/high; bit 1: in/out; bit 2: pullup enabled/disabled; bit 3: last read value (for change notification).
	uint8_t state;
#ifndef LOWMEM
	uint8_t master;
	int32_t value;
	int16_t adcvalue;
	Gpio *prev, *next;
#endif
	void setup(uint8_t new_state);
	virtual void load(int16_t &addr, bool eeprom);
	virtual void save(int16_t &addr, bool eeprom);
	virtual ~Gpio() {}
};
#endif

struct MoveCommand
{
	bool cb;
	float f[2];
	int32_t data[MAXAXES + MAXEXTRUDERS];	// Value if given, MAXLONG otherwise.
};

#define COMMAND_SIZE 127
#define COMMAND_LEN_MASK 0x7f
// Code 1 variables
EXTERN uint8_t name[NAMELEN];
EXTERN uint8_t num_extruders;
EXTERN uint8_t num_axes;
EXTERN uint8_t num_temps;
EXTERN uint8_t num_gpios;
EXTERN uint8_t printer_type;		// 0: cartesian, 1: delta.
EXTERN Pin_t led_pin, probe_pin;
EXTERN int32_t max_deviation;
#ifndef LOWMEM
EXTERN int32_t room_T;	//[°C]
#endif
EXTERN float feedrate;		// Multiplication factor for f values, used at start of move.
EXTERN int32_t angle;
// Other variables.
EXTERN char printerid[ID_SIZE];
EXTERN unsigned char command[COMMAND_SIZE];
EXTERN uint8_t command_end;
EXTERN char reply[COMMAND_SIZE];
EXTERN char out_buffer[10];
EXTERN Constants constants;
EXTERN Variables variables;
#if MAXAXES > 0
EXTERN Axis axis[MAXAXES];
#endif
#if MAXTEMPS > 0
EXTERN Temp temp[MAXTEMPS];
#endif
#if MAXEXTRUDERS > 0
EXTERN Extruder extruder[MAXEXTRUDERS];
#endif
#if MAXGPIOS > 0
EXTERN Gpio gpio[MAXGPIOS];
#endif
EXTERN uint8_t temps_busy;
#if MAXAXES > 0 || MAXEXTRUDERS > 0
EXTERN Motor *motors[MAXOBJECT];
#endif
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
EXTERN Temp *temps[MAXOBJECT];
#endif
EXTERN Object *objects[MAXOBJECT];
EXTERN MoveCommand queue[QUEUE_LENGTH];
EXTERN uint8_t queue_start, queue_end;
EXTERN bool queue_full;
EXTERN uint8_t num_movecbs;		// number of event notifications waiting to be sent out.
EXTERN uint8_t continue_cb;		// is a continue event waiting to be sent out? (0: no, 1: move, 2: audio, 3: both)
EXTERN uint32_t which_tempcbs;		// bitmask of waiting temp cbs.
#if MAXAXES > 0
EXTERN int32_t limits_pos[MAXAXES];	// position when limit switch was hit or nan
#endif
EXTERN uint8_t which_autosleep;		// which autosleep message to send (0: none, 1: motor, 2: temp, 3: both)
EXTERN uint8_t ping;			// bitmask of waiting ping replies.
EXTERN bool initialized;
EXTERN uint32_t motors_busy;		// bitmask of motors which are enabled.
EXTERN bool out_busy;
EXTERN unsigned long out_time;
EXTERN bool reply_ready;
EXTERN char *last_packet;
EXTERN unsigned long last_active, led_last, motor_limit, temp_limit;
EXTERN int16_t led_phase;
EXTERN uint8_t adc_phase;
#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
EXTERN uint8_t temp_current;
#endif
#ifdef AUDIO
EXTERN uint8_t audio_buffer[AUDIO_FRAGMENTS][AUDIO_FRAGMENT_SIZE];
EXTERN uint8_t audio_head, audio_tail, audio_state;
EXTERN unsigned long audio_start;
EXTERN int16_t audio_us_per_bit;
#endif
EXTERN unsigned long start_time;
EXTERN long freeze_time;
EXTERN long t0, tp;
EXTERN bool moving;
EXTERN float v0, vp, vq;
EXTERN float f0;
EXTERN bool move_prepared;
EXTERN bool current_move_has_cb;
EXTERN char debug_buffer[DEBUG_BUFFER_LENGTH];
EXTERN uint16_t debug_buffer_ptr;

// debug.cpp
void buffered_debug_flush();
void buffered_debug(char const *fmt, ...);

// packet.cpp
void packet();	// A command packet has arrived; handle it.

// serial.cpp
void serial();	// Handle commands from serial.
void send_packet(char *the_packet);
void try_send_next();
void write_ack();
void write_ackwait();

// move.cpp
void next_move();
void abort_move();
void reset_pos();

// setup.cpp
void setup();

// firmware.ino
void loop();	// Do stuff which needs doing: moving motors and adjusting heaters.

// storage.cpp
uint8_t read_8(int16_t &address, bool eeprom);
void write_8(int16_t &address, uint8_t data, bool eeprom);
int16_t read_16(int16_t &address, bool eeprom);
void write_16(int16_t &address, int16_t data, bool eeprom);
int32_t read_32(int16_t &address, bool eeprom);
void write_32(int16_t &address, int32_t data, bool eeprom);
float read_float(int16_t &address, bool eeprom);
void write_float(int16_t &address, float data, bool eeprom);

// axis.cpp
#ifndef LOWMEM
void compute_axes();
#endif

#if MAXAXES >= 3
static inline int32_t delta_to_axis(uint8_t a, int32_t *target, bool *ok) {
	int32_t dx = target[0] - axis[a].x;
	int32_t dy = target[1] - axis[a].y;
	int32_t dz = target[2] - axis[a].z;
	int32_t r2 = m(dx) * dx + m(dy) * dy;
	int32_t l2 = m(axis[a].delta_length) * axis[a].delta_length;
	int32_t dest = int32_t(sqrt(kf(l2 - r2))) + dz;
	//debug("dta dx %f dy %f dz %f z %f, r %f target %f", F(dx), F(dy), F(dz), F(axis[a].z), F(r), F(target));
	return dest;
}
#endif

#if MAXAXES > 0 || MAXEXTRUDERS > 0
static inline bool moving_motor(uint8_t which) {
	if (!moving)
		return false;
#if MAXAXES >= 3
	if (printer_type == 1 && which < 2 + 3) {
		for (uint8_t a = 2; a < 2 + 3; ++a)
			if (motors[a]->dist != 0 && motors[a]->dist != MAXLONG)
				return true;
		return false;
	}
#endif
	return motors[which]->dist != MAXLONG;
}
#endif

#include ARCH_INCLUDE

#endif

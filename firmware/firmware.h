#ifndef _DATA_H
#define _DATA_H

#include <Arduino.h>
#include <math.h>

#define NAMELEN 16		// Length of the printer name.
#define QUEUE_LENGTH_LOG2 4
#define QUEUE_LENGTH (1 << QUEUE_LENGTH_LOG2)	// Number of items which can be in the queue.
#define QUEUE_LENGTH_MASK ((1 << QUEUE_LENGTH_LOG2) - 1)	// Mask to use for circular queue.
#define MAXAXES 8		// Total number of supported axes (normally 3 are in use: x, y, z).
#define MAXEXTRUDERS 8		// Total number of supported extruders.
#define MAXTEMPS 8		// Total number of supported standalone temps (normally 1 is in use: bed).
#define MAXOBJECT (2 + MAXAXES + MAXEXTRUDERS + MAXTEMPS)		// Total number of supported objects.
#define AUDIO_FRAGMENTS 8
#define AUDIO_FRAGMENT_SIZE 32

#define MSGBUFSIZE 127	// Buffer size for messages; cannot be larger than 127.
#define MAXMSGLEN ((MSGBUFSIZE / 4 * 3) + ((MSGBUFSIZE % 4 == 0) ? 0 : (MSGBUFSIZE % 4) - 1))

#define F0 0
#define F1 1
#define AXIS0 2
#define EXTRUDER0 (AXIS0 + num_axes)
#define TEMP0 (EXTRUDER0 + num_extruders)

#define MAXLONG (int32_t ((~uint32_t (0)) >> 1))

// Exactly one file defines EXTERN as empty, which leads to the data to be defined.
#ifndef EXTERN
#define EXTERN extern
#endif

#define SET_OUTPUT(pin_no) do { if (!(pin_no).invalid ()) { pinMode ((pin_no).pin, OUTPUT); }} while (0)
#define SET_INPUT(pin_no) do { if (!(pin_no).invalid ()) { pinMode ((pin_no).pin, INPUT_PULLUP); }} while (0)
#define SET_INPUT_NOPULLUP(pin_no) do { if (!(pin_no).invalid ()) { pinMode ((pin_no).pin, INPUT); }} while (0)
#define SET(pin_no) do { if (!(pin_no).invalid ()) { digitalWrite ((pin_no).pin, (pin_no).inverted () ? LOW : HIGH); } } while (0)
#define RESET(pin_no) do { if (!(pin_no).invalid ()) { digitalWrite ((pin_no).pin, (pin_no).inverted () ? HIGH : LOW); } } while (0)
#define GET(pin_no, _default) (!(pin_no).invalid () ? digitalRead ((pin_no).pin) == HIGH ? !(pin_no).inverted () : (pin_no).inverted () : _default)

struct Pin_t {
	uint8_t flags;
	uint8_t pin;
	bool invalid () { return flags & 1; }
	bool inverted () { return flags & 2; }
	uint16_t write () { return flags << 8 | pin; }
	void read (uint16_t data) {
		if ((data & 0xff) != pin)
			SET_INPUT_NOPULLUP (*this);
		pin = data & 0xff;
		flags = data >> 8;
	}
};

union ReadFloat {
	float f;
	int32_t i;
	uint32_t u;
	uint8_t b[sizeof (float)];
};

enum SingleByteCommands {	// See serial.cpp for computation of command values.
// These bytes (except RESET) are sent in reply to a received packet only.
	CMD_ACK = 0x80,		// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_NACK = 0xe1,	// Incorrect checksum.
	CMD_ACKWAIT = 0xd2,	// Packet properly received and accepted, but queue is full so no new GOTO commands are allowed until CONTINUE.  (Never sent from host.)
	CMD_STALL = 0xb3,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_RESET = 0xf4,	// Emergency reset: clear queue, stop and sleep motors, temperatures off.  (Never sent to host.)  Typically sent 3 times with 5 ms pauses in between.
	CMD_INIT = 0x95,	// Printer started and is ready for commands.
	CMD_ACKRESET = 0xa6,	// Reset received.
	CMD_DEBUG = 0xc7	// Debug message; a nul-terminated message follows (no checksum; no resend).
};

enum Command {
	// from host
	CMD_BEGIN,	// 4 byte: 0 (preferred protocol version). Reply: START.
	CMD_GOTO,	// 1-2 byte: which channels (depending on number of extruders); channel * 4 byte: values [fraction/s], [mm].
	CMD_GOTOCB,	// same.  Reply (later): MOVECB.
	CMD_RUN,	// 1 byte: which channel (b0-6).  4 byte: speed [mm/s] (0 means off).
	CMD_SLEEP,	// 1 byte: which channel (b0-6); on/off (b7 = 1/0).
	CMD_SETTEMP,	// 1 byte: which channel; 4 bytes: target [degrees C].
	CMD_WAITTEMP,	// 1 byte: which channel; 4 bytes: lower limit; 4 bytes: upper limit [degrees C].  Reply (later): TEMPCB.  Disable with WAITTEMP (NAN, NAN).
	CMD_READTEMP,	// 1 byte: which channel.  Reply: TEMP. [degrees C]
	CMD_SETPOS,	// 1 byte: which channel; 4 bytes: pos.
	CMD_GETPOS,	// 1 byte: which channel.  Reply: POS. [steps, mm]
	CMD_LOAD,	// 1 byte: which channel.
	CMD_SAVE,	// 1 byte: which channel.
	CMD_READ,	// 1 byte: which channel.  Reply: DATA.
	CMD_WRITE,	// 1 byte: which channel; n bytes: data.
	CMD_PAUSE,	// 1 byte: 0: not pause; 1: pause.
	CMD_PING,	// 1 byte: code.  Reply: PONG.
	CMD_AUDIO_SETUP,	// 1-2 byte: which channels (like for goto); 2 byte: us_per_bit.
	CMD_AUDIO_DATA,	// AUDIO_FRAGMENT_SIZE bytes: data.  Returns ACK or ACKWAIT.
	// to host
		// responses to host requests; only one active at a time.
	CMD_START,	// 4 byte: 0 (protocol version).
	CMD_TEMP,	// 1 byte: requested channel; 4 byte: requested channel's temperature. [degrees C]
	CMD_POS,	// 4 byte: pos [steps]; 4 byte: current [mm].
	CMD_DATA,	// n byte: requested data.
	CMD_PONG,	// 1 byte: PING argument.
		// asynchronous events.
	CMD_MOVECB,	// 1 byte: number of movecb events.
	CMD_TEMPCB,	// 1 byte: which channel.  Byte storage for which needs to be sent.
	CMD_CONTINUE,	// 1 byte: is_audio.  Bool flag if it needs to be sent.
	CMD_LIMIT,	// 1 byte: which channel.
	CMD_MESSAGE,	// 4 byte: code; n byte: string: message with no defined meaning; code may be used for a protocol.
};

struct Object
{
	int16_t address;
	virtual void load (int16_t &address, bool eeprom) = 0;
	virtual void save (int16_t &address, bool eeprom) = 0;
	virtual ~Object () {}
};

// All temperatures are stored in Kelvin, but communicated in degrees C.
struct Temp : public Object
{
	// See temp.c from definition of calibration constants.
	float alpha;				// alpha value of thermistor.  [1]
	float beta;				// beta value of thermistor.  [degrees C]
	// Temperature balance calibration.
	float power;				// added power while heater is on.  [W]
	float core_C;				// heat capacity of the core.  [J/K]
	float shell_C;				// heat capacity of the shell.  [J/K]
	float transfer;				// heat transfer between core and shell.  [W/K]
	float radiation;			// radiated power = radiation * (shell_T ** 4 - room_T ** 4) [W/K**4]
	float convection;			// convected power = convection * (shell_T - room_T) [W/K]
	// Pins.
	Pin_t power_pin;
	Pin_t thermistor_pin;
	// Volatile variables.
	float target;				// target temperature; NAN to disable. [degrees C]
	float core_T, shell_T;			// current temperatures. [degrees C]
	float min_alarm;			// NAN, or the temperature at which to trigger the callback.  [degrees C]
	float max_alarm;			// NAN, or the temperature at which to trigger the callback.  [degrees C]
	// Internal variables.
	unsigned long last_time;		// last value of micros when this heater was handled.
	bool is_on;				// If the heater is currently on.
	// Functions.
	float read ();				// Read current temperature.
	virtual void load (int16_t &addr, bool eeprom);
	virtual void save (int16_t &addr, bool eeprom);
	virtual ~Temp () {}
};

struct Motor : public Object
{
	Pin_t step_pin;
	Pin_t dir_pin;
	Pin_t enable_pin;
	float steps_per_mm;			// hardware calibration [steps/mm].
	float max_v_neg, max_v_pos, max_a;	// maximum value for f in positive and negative direction [mm/s], [mm/s^2].
	float continuous_steps_per_s;		// steps per second for continuous run.
	float continuous_steps;			// fractional continuous steps that have been done.
	float f;
	unsigned long continuous_last_time;	// micros value when last continuous iteration was run.
	bool positive;				// direction of current movement.
	float dist, next_dist, main_dist;
	uint8_t audio_flags;
	enum Flags {
		PLAYING = 1,
		STATE = 2
	};
	virtual void load (int16_t &addr, bool eeprom);
	virtual void save (int16_t &addr, bool eeprom);
	virtual ~Motor () {}
};

struct Constants : public Object
{
	virtual void load (int16_t &addr, bool eeprom);	// NI
	virtual void save (int16_t &addr, bool eeprom);
	virtual ~Constants () {}
};

struct Variables : public Object
{
	virtual void load (int16_t &addr, bool eeprom);
	virtual void save (int16_t &addr, bool eeprom);
	virtual ~Variables () {}
};

struct Axis : public Object
{
	Motor motor;
	int32_t limit_min_pos;	// Position of motor (in steps) when the min limit switch is triggered.
	int32_t limit_max_pos;	// Position of motor (in steps) when the max limit switch is triggered.
	float delta_length, delta_radius;	// Calibration values for delta: length of the tie rod and the horizontal distance between the vertical position and the zero position.
	float offset;		// Position where axis claims to be when it is at 0.
	Pin_t limit_min_pin;
	Pin_t limit_max_pin;
	int32_t current_pos;	// Current position of motor (in steps).
	float source, current;	// Source position of current movement of axis (in mm), or current position if there is no movement.
	float x, y, z;		// Position of tower on the base plane, and the carriage height at zero position; only used for delta printers.
	virtual void load (int16_t &addr, bool eeprom);
	virtual void save (int16_t &addr, bool eeprom);
	virtual ~Axis () {}
};

struct Extruder : public Object
{
	Motor motor;
	Temp temp;		// temperature regulation.
	float filament_heat;	// constant for how much heat is extracted per mm filament.
	float nozzle_size;
	float filament_size;
	float capacity;		// heat capacity of filament in [energy]/mm/K
	int32_t steps_done;	// steps done during current move.
	virtual void load (int16_t &addr, bool eeprom);
	virtual void save (int16_t &addr, bool eeprom);
	virtual ~Extruder () {}
};

struct MoveCommand
{
	bool cb;
	float data[MAXOBJECT];	// Value if given, NaN otherwise.
};

#define COMMAND_SIZE_LOG2 7
#define COMMAND_SIZE (1 << COMMAND_SIZE_LOG2)
#define COMMAND_LEN_MASK (COMMAND_SIZE - 1)
// Code 1 variables
EXTERN uint8_t name[NAMELEN];
EXTERN uint8_t num_extruders;
EXTERN uint8_t num_axes;
EXTERN uint8_t num_temps;
EXTERN uint8_t printer_type;		// 0: cartesian, 1: delta.
EXTERN Pin_t led_pin;
EXTERN float room_T;	//[degrees C]
EXTERN float feedrate;	// Multiplication factor for f values, used at start of move.
// Other variables.
EXTERN unsigned char command[COMMAND_SIZE];
EXTERN uint8_t command_end;
EXTERN char reply[COMMAND_SIZE];
EXTERN char limitcb_buffer[10];
EXTERN char movecb_buffer[4];
EXTERN char tempcb_buffer[4];
EXTERN char continue_buffer[4];
EXTERN Constants constants;
EXTERN Variables variables;
EXTERN Axis axis[MAXAXES];
EXTERN Temp temp[MAXTEMPS];
EXTERN Extruder extruder[MAXEXTRUDERS];
EXTERN bool motors_busy;
EXTERN uint8_t temps_busy;
EXTERN Motor *motors[MAXOBJECT];
EXTERN Temp *temps[MAXOBJECT];
EXTERN Object *objects[MAXOBJECT];
EXTERN MoveCommand queue[QUEUE_LENGTH];
EXTERN uint8_t queue_start, queue_end;
EXTERN uint8_t num_movecbs;		// number of event notifications waiting to be sent out.
EXTERN uint8_t continue_cb;		// is a continue event waiting to be sent out? (0: no, 1: move, 2: audio, 3: both)
EXTERN uint32_t which_tempcbs;		// bitmask of waiting temp cbs.
EXTERN int32_t limits_pos[MAXAXES];	// position when limit switch was hit or MAXLONG;
EXTERN bool have_msg;
EXTERN char msg_buffer[MSGBUFSIZE];
EXTERN bool pause_all;
EXTERN unsigned long pause_time;
EXTERN bool out_busy;
EXTERN bool reply_ready;
EXTERN char *last_packet;
EXTERN unsigned long last_active, led_last, motor_limit, temp_limit;
EXTERN uint16_t led_phase;
EXTERN float mean_temp;
EXTERN uint8_t audio_buffer[AUDIO_FRAGMENTS][AUDIO_FRAGMENT_SIZE];
EXTERN uint8_t audio_head, audio_tail;
EXTERN unsigned long audio_start;
EXTERN unsigned long start_time;
EXTERN long t0, tq;
EXTERN bool moving;
EXTERN float v0, vp, vq, f0;
EXTERN uint16_t audio_us_per_bit;
EXTERN bool move_prepared;
EXTERN bool current_move_has_cb;

// debug.cpp
void debug (char const *fmt, ...);

// packet.cpp
void packet ();	// A command packet has arrived; handle it.

// serial.cpp
void serial ();	// Handle commands from serial.
void send_packet (char *the_packet);
void try_send_next ();

// move.cpp
void next_move ();
void abort_move ();

// setup.cpp
void setup ();

// firmware.ino
void loop ();	// Do stuff which needs doing: moving motors and adjusting heaters.

// storage.cpp
uint8_t read_8 (int16_t &address, bool eeprom);
void write_8 (int16_t &address, uint8_t data, bool eeprom);
int16_t read_16 (int16_t &address, bool eeprom);
void write_16 (int16_t &address, int16_t data, bool eeprom);
int32_t read_32 (int16_t &address, bool eeprom);
void write_32 (int16_t &address, int32_t data, bool eeprom);
float read_float (int16_t &address, bool eeprom);
void write_float (int16_t &address, float data, bool eeprom);

static inline int32_t delta_to_axis (uint8_t a, float *target, bool *ok) {
	float dx = target[0] - axis[a].x;
	float dy = target[1] - axis[a].y;
	float dz = target[2] - axis[a].z;
	float r2 = dx * dx + dy * dy;
	float l2 = axis[a].delta_length * axis[a].delta_length;
	if (r2 > l2 + 100 - 20 * axis[a].delta_length) {
		*ok = false;
		debug ("not ok: %f %f %f %f %f %f %f", &target[0], &target[1], &dx, &dy, &r2, &l2, &axis[a].delta_length);
		// target is too far away from axis.  Pull it towards axis so that it is on the edge.
		// target = axis + (target - axis) * (l - epsilon) / r.
		float factor = (axis[a].delta_length - 10.) / sqrt (r2);
		target[0] = axis[a].x + (target[0] - axis[a].x) * factor;
		target[1] = axis[a].y + (target[1] - axis[a].y) * factor;
		return 0;
	}
	float inner = dx * axis[a].x + dy * axis[a].y;
	if (inner > 0) {
		*ok = false;
		debug ("not ok: %f %f %f %f %f", &inner, &dx, &dy, &axis[a].x, &axis[a].y);
		// target is on the wrong side of axis.  Pull it towards plane so it is on the edge.
		// target = axis + (target - (target.axis-epsilon)/|axis|2*axis)
		float factor = .99 - (target[0] * axis[a].x + target[1] * axis[a].y) / (axis[a].x * axis[a].x + axis[a].y * axis[a].y);
		target[0] += factor * axis[a].x;
		target[1] += factor * axis[a].y;
		return 0;
	}
	float dest = sqrt (l2 - r2) + dz;
	//debug ("dta dx %f dy %f dz %f z %f, r %f target %f", &dx, &dy, &dz, &axis[a].z, &r, &target);
	return dest * axis[a].motor.steps_per_mm;
}

#endif

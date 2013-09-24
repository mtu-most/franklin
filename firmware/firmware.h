#ifndef _DATA_H
#define _DATA_H

#include <Arduino.h>
#include <math.h>

#define QUEUE_LENGTH_LOG2 4
#define QUEUE_LENGTH (1 << QUEUE_LENGTH_LOG2)	// Number of items which can be in the queue.
#define QUEUE_LENGTH_MASK ((1 << QUEUE_LENGTH_LOG2) - 1)	// Mask to use for circular queue.
#define MAXAXES 8		// Total number of supported axes (normally 3 are in use: x, y, z).
#define MAXEXTRUDERS 8		// Total number of supported extruders.
#define MAXTEMPS 8		// Total number of supported standalone temps (normally 1 is in use: bed).
#define MAXOBJECT (2 + MAXAXES + MAXEXTRUDERS + MAXTEMPS)		// Total number of supported objects.

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

#define SET_OUTPUT(pin) do { if ((pin) < 255) { pinMode ((pin), OUTPUT); }} while (0)
#define SET_INPUT(pin) do { if ((pin) < 255) { pinMode ((pin), INPUT); }} while (0)
#define SET_INPUT_NOPULLUP(pin) do { if ((pin) < 255) { pinMode ((pin), INPUT); }} while (0)
#define SET(pin) do { if ((pin) < 255) { digitalWrite ((pin), HIGH); } } while (0)
#define RESET(pin) do { if ((pin) < 255) { digitalWrite ((pin), LOW); } } while (0)
#define GET(pin, _default) ((pin) < 255 ? digitalRead (pin) : _default)

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
	CMD_GETPOS,	// 1 byte: which channel.  Reply: POS. [steps]
	CMD_LOAD,	// 1 byte: which channel.
	CMD_SAVE,	// 1 byte: which channel.
	CMD_READ,	// 1 byte: which channel.  Reply: DATA.
	CMD_WRITE,	// 1 byte: which channel; n bytes: data.
	CMD_PAUSE,	// 1 byte: 0: not pause; 1: pause.
	CMD_PING,	// 1 byte: code.  Reply: PONG.
	CMD_PLAY,	// 1 byte: motor 1, 1 byte: motor 2, 4 bytes: #samples. 
	// to host
		// responses to host requests; only one active at a time.
	CMD_START,	// 4 byte: 0 (protocol version).
	CMD_TEMP,	// 1 byte: requested channel; 4 byte: requested channel's temperature. [degrees C]
	CMD_POS,	// 4 byte: pos. [steps]
	CMD_DATA,	// n byte: requested data.
	CMD_PONG,	// 1 byte: PING argument.
		// asynchronous events.
	CMD_MOVECB,	// 1 byte: number of movecb events.
	CMD_TEMPCB,	// 1 byte: which channel.  Byte storage for which needs to be sent.
	CMD_CONTINUE,	// 1 byte: 0 (because commands must not have 0 or 2 byte arguments).  Bool flag if it needs to be sent.
	CMD_LIMIT,	// 1 byte: which channel.
	CMD_MESSAGE,	// 4 byte: code; n byte: string: message with no defined meaning; code may be used for a protocol.
};

struct Object
{
	uint16_t address;
	virtual void load (uint16_t &address, bool eeprom) = 0;
	virtual void save (uint16_t &address, bool eeprom) = 0;
	virtual ~Object () {}
};

// All temperatures are in Kelvin.  Negative temperature target means heater is off.
// Energy unit is chosen such that the heat capacity of the extruder is 1, so the energy is equal to the temperature.
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
	uint8_t power_pin;
	uint8_t thermistor_pin;
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
	virtual void load (uint16_t &addr, bool eeprom);
	virtual void save (uint16_t &addr, bool eeprom);
	virtual ~Temp () {}
};

struct Motor : public Object
{
	uint8_t step_pin;
	uint8_t dir_pin;
	uint8_t enable_pin;
	float steps_per_mm;			// hardware calibration [steps/mm].
	float max_f_neg, max_f_pos;		// maximum value for f in positive and negative direction [steps/s].
	float a;				// acceleration of current move [s**-2]
	uint16_t steps_total;			// total number of steps for current move [steps].
	uint16_t steps_done;			// finished number of steps for current move [steps].
	bool positive;				// direction of current movement.
	bool continuous;			// whether the motor should move without scheduled end point.
	float f;				// current flowrate [steps/s]; used to communicate from extruder motor to extruder temp.
	virtual void load (uint16_t &addr, bool eeprom);
	virtual void save (uint16_t &addr, bool eeprom);
	virtual ~Motor () {}
};

struct Constants : public Object
{
	virtual void load (uint16_t &addr, bool eeprom);	// NI
	virtual void save (uint16_t &addr, bool eeprom);
	virtual ~Constants () {}
};

struct Variables : public Object
{
	virtual void load (uint16_t &addr, bool eeprom);
	virtual void save (uint16_t &addr, bool eeprom);
	virtual ~Variables () {}
};

struct Axis : public Object
{
	Motor motor;
	uint8_t limit_min_pin;
	uint8_t limit_max_pin;
	int32_t current_pos;
	virtual void load (uint16_t &addr, bool eeprom);
	virtual void save (uint16_t &addr, bool eeprom);
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
	virtual void load (uint16_t &addr, bool eeprom);
	virtual void save (uint16_t &addr, bool eeprom);
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
EXTERN uint8_t num_extruders;
EXTERN uint8_t num_axes;
EXTERN uint8_t num_temps;
EXTERN uint8_t led_pin;
EXTERN float room_T;	//[degrees C]
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
EXTERN uint8_t motors_busy;
EXTERN unsigned long start_time;	// start time of current move [us]
EXTERN unsigned long last_time;		// last time continuous move was handled [us]
EXTERN float f0, f1;			// start and end f for current move [s**-1]
EXTERN Motor *motors[MAXOBJECT];
EXTERN Temp *temps[MAXOBJECT];
EXTERN Object *objects[MAXOBJECT];
EXTERN MoveCommand queue[QUEUE_LENGTH];
EXTERN uint8_t queue_start, queue_end;
EXTERN uint8_t num_movecbs;		// number of event notifications waiting to be sent out.
EXTERN bool continue_cb;		// is a continue event waiting to be sent out?
EXTERN uint32_t which_tempcbs;		// bitmask of waiting temp cbs.
EXTERN int32_t limits_pos[MAXAXES];	// position when limit switch was hit or MAXLONG;
EXTERN bool have_msg;
EXTERN char msg_buffer[MSGBUFSIZE];
EXTERN bool pause_all;
EXTERN bool out_busy;
EXTERN bool reply_ready;
EXTERN char *last_packet;
EXTERN unsigned long last_active, motor_limit, temp_limit;

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

// setup.cpp
void setup ();

// firmware.ino
void loop ();	// Do stuff which needs doing: moving motors and adjusting heaters.

// storage.cpp
uint8_t read_8 (uint16_t &address, bool eeprom);
void write_8 (uint16_t &address, uint8_t data, bool eeprom);
uint16_t read_16 (uint16_t &address, bool eeprom);
void write_16 (uint16_t &address, uint16_t data, bool eeprom);
uint32_t read_32 (uint16_t &address, bool eeprom);
void write_32 (uint16_t &address, uint32_t data, bool eeprom);
float read_float (uint16_t &address, bool eeprom);
void write_float (uint16_t &address, float data, bool eeprom);

// play.cpp
void play (uint8_t which, uint32_t num_samples);

#endif

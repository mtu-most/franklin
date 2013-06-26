#ifndef _DATA_H
#define _DATA_H

#include <Arduino.h>
#include <math.h>

#define QUEUE_LENGTH 100	// Number of items which can be in the queue.
#define MAXOBJECT 16		// Total number of objects.  From Extruder 0 on, these are all extruders.

#define FLAG_X 0
#define FLAG_Y 1
#define FLAG_Z 2
#define FLAG_F0 3
#define FLAG_F1 4
#define FLAG_BED 5
#define FLAG_EXTRUDER0 6
// Maximum number of extruders supported by this board.  This is not the number
// of extruders available; that is a normal variable defined below (and
// changable in eeprom).
#define MAX_EXTRUDER MAXOBJECT - FLAG_EXTRUDER0

// Exactly one file defines EXTERN as empty, which leads to the data to be defined.
#ifndef EXTERN
#define EXTERN extern
#endif

#define SET(pin) do { if ((pin) < 255) digitalWrite ((pin), HIGH); } while (0)
#define RESET(pin) do { if ((pin) < 255) digitalWrite ((pin), LOW); } while (0)
#define GET(pin, _default) ((pin) < 255 ? digitalRead (pin) : _default)

union ReadFloat {
	float f;
	uint8_t b[sizeof (float)];
};

enum SingleByteCommands {	// See serial.cc for computation of command values.
// These bytes (except RESET) are sent in reply to a received packet only.
	ACK = 0x80,		// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	NACK = 0xe1,		// Incorrect checksum.
	ACKWAIT = 0xd2,		// Packet properly received and accepted, but queue is full so no new GOTO commands are allowed until CONTINUE.  (Never sent from host.)
	STALL = 0xb3,		// Packet properly received, but not accepted; don't resend packet unmodified.
	RESET = 0xf4,		// Emergency reset: clear queue, stop and sleep motors, temperatures off.  (Never sent to host.)  Typically sent 3 times with 5 ms pauses in between.
	// The following codes have proper checksum correction, but aren't used.  If new commands are added, they should use these codes.
	UNUSED1 = 0x95,		// Unused code.
	UNUSED2 = 0xa6,		// Unused code.
	UNUSED3 = 0xc7		// Unused code.
};

enum Command {
	// from host
	BEGIN,		// 4 byte: 0 (preferred protocol version). Reply: START.
	GOTO,		// 1-2 byte: which channels (depending on number of extruders); channel * 4 byte: values.
	GOTOCB,		// same.  Reply (later): MOVECB.
	RUN,		// 1 byte: which channel (b0-6); on/off (b7 = 1/0).  4 byte: speed.
	SLEEP,		// 1 byte: which channel (b0-6); on/off (b7 = 1/0).
	SETTEMP,	// 1 byte: which channel; 4 bytes: target.
	WAITTEMP,	// 1-2 byte: which channels; 8 bytes per channel: lower limit, upper limit.  Any of those channels will trigger the reply.  Reply (later): TEMPCB.
	READTEMP,	// 1 byte: which channel.  Reply: TEMP.
	LOAD,		// 1 byte: which channel.
	SAVE,		// 1 byte: which channel.
	READ,		// 1 byte: which channel.  Reply: DATA.
	WRITE,		// 1 byte: which channel; n bytes: data.
	PAUSE,		// 1 byte: 0: not pause; 1: pause.
	PING,		// 1 byte: code.  Reply: PONG.
	// to host
	START,		// 4 byte: 0 (protocol version).
	TEMP,		// 1 byte: requested channel; 4 byte: requested channel's temperature.
	DATA,		// 1 byte: requested channel; n byte: requested data.
	MOVECB,		// 1 byte: 0 (because commands must not have 0 or 2 byte arguments).
	TEMPCB,		// 1 byte: which channel.
	CONTINUE,	// 1 byte: 0 (because commands must not have 0 or 2 byte arguments).
	PONG,		// 1 byte: PING argument.
};

struct Object
{
	uint16_t address;
	virtual void load (uint16_t &address, bool eeprom) = 0;
	virtual void save (uint16_t &address, bool eeprom) = 0;
};

// All temperatures are in Kelvin.  Negative temperature target means heater is off.
// Energy unit is chosen such that the heat capacity of the extruder is 1, so the energy is equal to the temperature.
struct Temp : public Object
{
	Temp *next;
	float beta;				// beta value of thermistor; adc = adc0 * exp (-beta * (T - T0))
	float T0;				// temperature of calibration point.
	float adc0;				// adc reading of calibration point.
	float target;				// target temperature.
	float radiation;			// factor for how much this thing radiates E = T ** 4 * radiation for each buffer shift.
	float power;				// energy that is added per ms if heater is on.
	float buffer[4];			// history of added energy.
	uint16_t buffer_delay;			// time for the buffer to shift one slot, in ms.
	uint8_t power_pin;
	uint8_t thermistor_pin;
	unsigned long long last_time;		// Counter to keep track of how much action should be taken.
	unsigned long long last_shift_time;	// Counter to keep track of how much action should be taken.
	bool is_on;				// If the heater is currently on.
	float extra_loss;			// extra lost energy per buffer shift; used to compensate for extrusion loss.
	float read ();				// Read current temperature.
	virtual void load (uint16_t &addr, bool eeprom);
	virtual void save (uint16_t &addr, bool eeprom);
};

struct Motor : public Object
{
	Motor *next;
	uint8_t step_pin;
	uint8_t dir_pin;
	uint8_t sleep_pin;
	float steps_per_mm;
	unsigned long long start_time, end_time;	// ms.
	float f1, f0;	// steps per ms.
	uint16_t steps_total;
	uint16_t steps_done;
	bool positive;	// direction of current movement.
	bool continuous;	// whether the motor should move without scheduled end point.
	float f;	// current flowrate (steps/ms); used to communicate from extruder motor to extruder temp.
	virtual void load (uint16_t &addr, bool eeprom);
	virtual void save (uint16_t &addr, bool eeprom);
};

struct Axis : public Object
{
	Motor motor;
	uint8_t limit_min_pin;
	uint8_t limit_max_pin;
	virtual void load (uint16_t &addr, bool eeprom);
	virtual void save (uint16_t &addr, bool eeprom);
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
};

struct MoveCommand
{
	bool cb;
	float data[MAXOBJECT];	// Value if given, NaN otherwise.
};

#define COMMAND_SIZE_LOG2 7
#define COMMAND_SIZE (1 << COMMAND_SIZE_LOG2)
#define COMMAND_LEN_MASK (COMMAND_SIZE - 1)
EXTERN char command[COMMAND_SIZE];
EXTERN uint8_t command_end;
EXTERN char outcommand[COMMAND_SIZE];
EXTERN Temp bed;
EXTERN uint16_t bed_address;
EXTERN Axis axis[3];
EXTERN uint8_t num_extruders;
EXTERN Extruder extruder[MAX_EXTRUDER];
EXTERN uint8_t motors_busy;
EXTERN Motor *motors[MAXOBJECT];
EXTERN Temp *temps[MAXOBJECT];
EXTERN Object *objects[MAXOBJECT];
EXTERN MoveCommand queue[QUEUE_LENGTH];
EXTERN uint8_t queue_start, queue_end;
EXTERN uint8_t num_cbs;	// number of event notifications waiting to be sent out.
EXTERN bool pause_all;

uint16_t bed_load (bool eeprom);
void bed_save (bool eeprom);
void serial ();	// Handle commands from serial.
void send_notification ();
void packet ();	// A command packet has arrived; handle it.
void send_packet ();
void next_move ();

void setup ();
void loop ();	// Do stuff which needs doing: moving motors and adjusting heaters.

#endif

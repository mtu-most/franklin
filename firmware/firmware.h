#ifndef _FIRMWARE_H
#define _FIRMWARE_H

#include <math.h>
#include <stdarg.h>
#include ARCH_INCLUDE

#define ID_SIZE 8	// Number of bytes in printerid.
#define PROTOCOL_VERSION 0

#define MAXLONG (int32_t((uint32_t(1) << 31) - 1))
#define MAXINT ((1 << ADCBITS) + 1)

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
	void init() { flags = 0; pin = 0; }
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

enum SingleByteCommands {	// See serial.cpp for computation of command values.
// These bytes (except RESET) are sent in reply to a received packet only.
	CMD_NACK = 0x80,	// Incorrect packet; please resend.
	CMD_ACK0 = 0xb3,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACKWAIT0 = 0xb4,	// Packet properly received and accepted, but queue is full so no new GOTO commands are allowed until CONTINUE.  (Never sent from host.)
	CMD_STALL0 = 0x87,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL1 = 0x9e,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_ACKWAIT1 = 0x99,	// Printer started and is ready for commands.
	CMD_ID = 0xaa,		// Request/reply printer ID code.
	CMD_ACK1 = 0xad,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_DEBUG = 0x81	// Debug message; a nul-terminated message follows (no checksum; no resend).
};

enum Command {
	// from host
	CMD_BEGIN,	// 8:id
	CMD_PING,	// 1:code
	CMD_RESET,	// 4:magic
	CMD_SETUP,	// 1:num_motors, 2:led_pin, 4:speedfactor
	CMD_MSETUP,	// 1:motor, 2:step_pin, 2:dir_pin, 2:limit_min_pin, 2:limit_max_pin, 2:sense_pin, 1:max_steps

	CMD_MOVE,	// ?:which, (4:speed, 4:limit, 4:current)*

	CMD_SETPOS,	// 1:which, 4:pos
	CMD_ADDPOS,	// 1:which, 4:pos
	CMD_GETPOS,	// 1:which
	CMD_RESETPIN,	// 1:dpin
	CMD_SETPIN,	// 1:dpin
	CMD_UNSETPIN,	// 1:dpin
	CMD_INPUTPIN,	// 1:dpin
	CMD_GETPIN,	// 1:dpin
	CMD_GETADC,	// 1:apin

	CMD_AUDIO_SETUP,	// ?:which, 4:samples/s
	CMD_AUDIO_DATA,		// X:data

	// to host
		// responses to host requests; only one active at a time.
	CMD_START,	// 4:version, 1:num_dpins, 1:num_apins
	CMD_PONG,	// 1:code
	CMD_POS,	// 4:pos
	CMD_PIN,	// 1:state
	CMD_ADC,	// 2:adc reading

		// asynchronous events.
	CMD_LIMIT,	// 1:which
	CMD_SENSE0,	// 1:which, 4:pos
	CMD_SENSE1,	// 1:which, 4:pos
};

struct Motor
{
	Pin_t step_pin;
	Pin_t dir_pin;
	uint8_t max_steps;	// maximum number of steps in one iteration.
	float a;		// steps/μs²
	float max_v;		// steps/μs
	Pin_t limit_min_pin;
	Pin_t limit_max_pin;
	Pin_t sense_pin;
	uint8_t sense_state;
	int32_t sense_pos, switch_pos, current_pos, start_pos, end_pos;
	float v, target_v;	// steps/μs
	uint32_t last_step_t;	// time in μs
	bool on_track;
#ifdef HAVE_AUDIO
	uint8_t audio_flags;
	enum Flags {
		PLAYING = 1,
		STATE = 2
	};
#endif
	inline void init();
	void fini() {
		step_pin.read(0);
		dir_pin.read(0);
		limit_min_pin.read(0);
		limit_max_pin.read(0);
		sense_pin.read(0);
	}
};

#define COMMAND_SIZE 127
#define COMMAND_LEN_MASK 0x7f
#define FULL_COMMAND_SIZE (COMMAND_SIZE + (COMMAND_SIZE + 2) / 3)

EXTERN char printerid[ID_SIZE];
EXTERN unsigned char command[FULL_COMMAND_SIZE];
EXTERN uint8_t command_end;
EXTERN char reply[FULL_COMMAND_SIZE], adcreply[6];
EXTERN char out_buffer[16];
EXTERN uint8_t ping;			// bitmask of waiting ping replies.
EXTERN bool out_busy;
EXTERN uint32_t out_time;
EXTERN bool reply_ready, adcreply_ready;
EXTERN char pending_packet[FULL_COMMAND_SIZE];
EXTERN uint8_t adc_phase;
EXTERN uint8_t num_motors;
EXTERN Motor **motor;
EXTERN uint8_t stopping;
EXTERN bool led_fast;
EXTERN uint32_t led_last, led_phase, start_time;
EXTERN Pin_t led_pin;
EXTERN int8_t speed_error;	// Acceptable error in v to correct position.
EXTERN uint8_t adc_current;
#ifdef HAVE_AUDIO
EXTERN uint8_t audio_buffer[AUDIO_FRAGMENTS][AUDIO_FRAGMENT_SIZE];
EXTERN uint8_t audio_head, audio_tail, audio_state;
EXTERN uint32_t audio_start;
EXTERN int16_t audio_us_per_sample;
EXTERN bool continue_cb;
#endif

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
void send_packet();
void try_send_next();
void write_ack();
void write_ackwait();
void write_stall();

// setup.cpp
void setup();

// firmware.ino
void loop();	// Do stuff which needs doing: moving motors and adjusting heaters.

#include ARCH_INCLUDE

void Motor::init() {
	step_pin.init();
	dir_pin.init();
	max_steps = 0;
	limit_min_pin.init();
	limit_max_pin.init();
	sense_pin.init();
	sense_state = 0;
	sense_pos = MAXLONG;
	switch_pos = MAXLONG;
	current_pos = 0;
	start_pos = 0;
	end_pos = MAXLONG;
	v = 0;
	target_v = 0;
	last_step_t = utime();
#ifdef HAVE_AUDIO
	audio_flags = 0;
#endif
}
#endif

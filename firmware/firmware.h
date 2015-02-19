#ifndef _FIRMWARE_H
#define _FIRMWARE_H

#include <stdarg.h>
#include ARCH_INCLUDE

#define ID_SIZE 24	// Number of bytes in printerid; it's a UUID of 16 bytes, plus 4 bytes run_id and 4 magic bytes.
#define PROTOCOL_VERSION 0

#define ADC_INTERVAL 1000	// Delay 1 ms between ADC measurements.

#ifndef NUM_MOTORS
#error "NUM_MOTORS must be defined in the Makefile"
#endif
#ifndef NUM_BUFFERS
#error "NUM_BUFFERS must be defined in the Makefile"
#endif
#ifndef FRAGMENTS_PER_BUFFER
#error "FRAGMENTS_PER_BUFFER must be defined in the Makefile"
#endif
#ifndef BYTES_PER_FRAGMENT
#error "BYTES_PER_FRAGMENT must be defined in the Makefile"
#endif

#define MAXLONG (int32_t((uint32_t(1) << 31) - 1))
#define MAXINT ((1 << ADCBITS) + 1)

#define RESET_MAGIC 0xDEADBEEF

// Exactly one file defines EXTERN as empty, which leads to the data to be defined.
#ifndef EXTERN
#define EXTERN extern
#else
#define DEFINE_VARIABLES
#endif

#define MAX_REPLY_LEN (2 + 4 * NUM_MOTORS)
#define REPLY_BUFFER_SIZE (MAX_REPLY_LEN + (MAX_REPLY_LEN + 2) / 3)

//#define MAX_COMMAND_LEN1 (5 + NUM_MOTORS)
//#define MAX_COMMAND_LEN2 (3 + BYTES_PER_FRAGMENT)
#define MAX_COMMAND_LEN 256 // (MAX_COMMAND_LEN1 > MAX_COMMAND_LEN2 ? MAX_COMMAND_LEN1 : MAX_COMMAND_LEN2) FIXME
#define COMMAND_BUFFER_SIZE (MAX_COMMAND_LEN + (MAX_COMMAND_LEN + 2) / 3)

template <typename _A> _A min(_A a, _A b) { return a < b ? a : b; }
template <typename _A> _A max(_A a, _A b) { return a > b ? a : b; }
template <typename _A> _A abs(_A a) { return a > 0 ? a : -a; }

EXTERN volatile uint16_t debug_value, debug_value1;
EXTERN uint8_t printerid[ID_SIZE];
EXTERN uint8_t command[COMMAND_BUFFER_SIZE];
EXTERN uint16_t command_end;
EXTERN bool had_data;
EXTERN uint8_t reply[REPLY_BUFFER_SIZE], adcreply[6];
EXTERN uint8_t ping;			// bitmask of waiting ping replies.
EXTERN bool out_busy;
EXTERN uint16_t out_time;
EXTERN uint8_t reply_ready, adcreply_ready;
EXTERN bool timeout;
EXTERN uint8_t pending_packet[REPLY_BUFFER_SIZE > 6 ? REPLY_BUFFER_SIZE : 6];
EXTERN uint16_t pending_len;
EXTERN volatile uint8_t move_phase, full_phase;
EXTERN uint8_t filling;
EXTERN uint8_t led_fast;
EXTERN uint16_t led_last, led_phase, time_per_sample;
EXTERN uint8_t led_pin, probe_pin, pin_flags;
EXTERN uint16_t timeout_time, last_active;
EXTERN uint8_t enabled_pins;

enum SingleByteCommands {	// See serial.cpp for computation of command values.
// These bytes (except RESET) are sent in reply to a received packet only.
	CMD_NACK = 0x80,	// Incorrect packet; please resend.
	CMD_ACK0 = 0xb3,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_STALL0 = 0x87,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL1 = 0x9e,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_ID = 0xaa,		// Request/reply printer ID code.
	CMD_ACK1 = 0xad,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_DEBUG = 0xb4,	// Debug message; a nul-terminated message follows (no checksum; no resend).
	CMD_STARTUP = 0x99	// Starting up.
};

enum Control {
	// Control is 0x0000rrcc, with R=read request, r=reset value, c=current value.
	CTRL_RESET,
	CTRL_SET,
	CTRL_UNSET,
	CTRL_INPUT
};
#define CONTROL_RESET(x) (((x) >> 2) & 0x3)
#define CONTROL_CURRENT(x) ((x) & 0x3)

struct Pin_t {
	uint8_t state;
	void disable(uint8_t pin) {
		if (CONTROL_RESET(state) == CONTROL_CURRENT(state))
			return;
		switch (CONTROL_RESET(state)) {
		case CTRL_RESET:
			RESET(pin);
			break;
		case CTRL_SET:
			SET(pin);
			break;
		case CTRL_UNSET:
			UNSET(pin);
			break;
		case CTRL_INPUT:
			SET_INPUT(pin);
			break;
		}
	}
	void set_state(uint8_t new_state) {
		if (CONTROL_RESET(state) != CONTROL_CURRENT(state))
			enabled_pins -= 1;
		state = new_state;
		if (CONTROL_RESET(state) != CONTROL_CURRENT(state))
			enabled_pins += 1;
	}
};
extern Pin_t pin[NUM_DIGITAL_PINS];

enum Command {
	// from host
	CMD_BEGIN = 0x40,	// 0
	CMD_PING,	// 1:code
	CMD_RESET,	// 4:magic	reset the mcu.
	CMD_SETUP,	// 1:active_motors, 4:us/sample, 1:led_pin, 1:probe_pin 1:pin_flags 2:timeout
	CMD_CONTROL,	// 1:num_commands, {1: command, 1: arg}
	CMD_MSETUP,	// 1:motor, 1:step_pin, 1:dir_pin, 1:limit_min_pin, 1:limit_max_pin, 1:sense_pin, 1:flags
	CMD_ASETUP,	// 1:adc, 2:linked_pins, 4:values	(including flags)
	CMD_HOME,	// 4:us/step, {1:dir}*

	CMD_START_MOVE,	// 1:num_samples, 1:num_moving_motors
	CMD_START_PROBE,// 1:num_samples, 1:num_moving_motors
	CMD_MOVE,	// 1:which, 1:dir, *:samples	// dir: 0:positive, 1:negative, 2:audio
	CMD_START,	// 0 start moving.
	CMD_STOP,	// 0 stop moving.
	CMD_ABORT,	// 0 stop moving and set all pins to their reset state.
	CMD_DISCARD,	// 1:num_buffers
	CMD_GETPIN,	// 1:pin

	// to host
		// responses to host requests; only one active at a time.
	CMD_READY = 0x60,	// 1:packetlen, 4:version, 1:num_dpins, 1:num_adc, 1:num_motors, 1:num_buffers, 1:fragments/buffer, 1:bytes/fragment
	CMD_PONG,	// 1:code
	CMD_HOMED,	// {4:motor_pos}*
	CMD_PIN,	// 1:state
	CMD_STOPPED,	// 1:fragment_pos, {4:motor_pos}*

		// asynchronous events.
	CMD_DONE,	// 1:num
	CMD_UNDERRUN,	// 1:num
	CMD_ADC,	// 1:which, 2:adc reading
	CMD_LIMIT,	// 1:which, 1:pos, {4:motor_pos}*
	CMD_SENSE0,	// 1:which, {4:motor_pos}*
	CMD_SENSE1,	// 1:which, {4:motor_pos}*
	CMD_TIMEOUT	// 0
};

static inline uint16_t minpacketlen() {
	switch (command[0] & ~0x10) {
	case CMD_BEGIN:
		return 2;
	case CMD_PING:
		return 2;
	case CMD_RESET:
		return 5;
	case CMD_SETUP:
		return 11;
	case CMD_CONTROL:
		return 2;
	case CMD_MSETUP:
		return 8;
	case CMD_ASETUP:
		return 8;
	case CMD_HOME:
		return 5;
	case CMD_START_MOVE:
		return 3;
	case CMD_START_PROBE:
		return 3;
	case CMD_MOVE:
		return 3;
	case CMD_START:
		return 1;
	case CMD_STOP:
		return 1;
	case CMD_ABORT:
		return 1;
	case CMD_DISCARD:
		return 2;
	case CMD_GETPIN:
		return 2;
	default:
		debug("invalid command passed to minpacketlen: %x", command[0]);
		return 1;
	};
}

enum Dir {
	DIR_POSITIVE,
	DIR_NEGATIVE,
	DIR_AUDIO,
	DIR_NONE
};

struct Motor
{
	volatile int32_t current_pos;
	volatile uint8_t step_pin;
	volatile uint8_t dir_pin;
	volatile uint8_t next_steps, next_next_steps;
	volatile uint8_t steps_current;
	volatile Dir dir, next_dir;
	int32_t sense_pos[2];
	uint8_t limit_min_pin;
	uint8_t limit_max_pin;
	uint8_t sense_pin;
	uint8_t buffer;
	uint8_t flags;
	enum Flags {
		LIMIT = 1,
		SENSE0 = 2,
		SENSE1 = 4,
		INVERT_LIMIT_MIN = 8,
		INVERT_LIMIT_MAX = 16,
		INVERT_STEP = 32,
		SENSE_STATE = 64
	};
	void init(uint8_t b) {
		buffer = b;
		current_pos = 0;
		sense_pos[0] = 0xBEEFBEEF;
		sense_pos[1] = 0xFACEFACE;
		flags = 0;
		next_steps = 0;
		next_next_steps = 0;
		steps_current = 0;
		dir = DIR_NONE;
		next_dir = DIR_NONE;
	}
	void disable() {
		current_pos = 0;
		dir = DIR_NONE;
		next_dir = DIR_NONE;
		next_steps = 0;
		next_next_steps = 0;
		steps_current = 0;
	}
};

EXTERN Motor motor[NUM_MOTORS];
EXTERN volatile uint8_t active_motors;
EXTERN volatile uint8_t steps_prepared;	// Number of steps waiting to be sent (0, 1 or 2).
EXTERN volatile bool stopped;	// True if motors are not moving.
EXTERN volatile bool underrun;	// True if next fragment was not present.
EXTERN bool stopping;	// True if LIMIT has been sent to host, but not yet acknowledged.
EXTERN uint32_t home_step_time;
EXTERN uint8_t homers;

struct Fragment {
	Dir dir;
	uint16_t num_samples;
	uint8_t samples[BYTES_PER_FRAGMENT];
};

struct Settings {
	bool probing;
	uint8_t len;
};

typedef Fragment Buffer[FRAGMENTS_PER_BUFFER];

EXTERN Buffer buffer[NUM_BUFFERS];
EXTERN Settings settings[FRAGMENTS_PER_BUFFER];
EXTERN uint8_t notified_current_fragment;
EXTERN uint8_t current_fragment;	// Fragment that is currently active, or if none, the one that will next be active.
EXTERN uint8_t last_fragment;	// Fragment that is currently being filled.
EXTERN uint8_t limit_fragment_pos;
EXTERN uint16_t current_fragment_pos;
EXTERN uint8_t current_len;	// copy of settings[current_fragment].len, for when current_fragment changes during a fill.

struct Adc {
	uint8_t linked[2];
	uint16_t value[2];	// bit 15 in [0] set => invalid; bit 14 set => linked inverted.
	bool is_on;
	void disable() {
		if (value[0] & 0x8000)
			return;
		for (uint8_t i = 0; i < 2; ++i) {
			if (linked[i] >= NUM_DIGITAL_PINS)
				continue;
			if (value[i] & 0x4000)
				SET(linked[i]);
			else
				RESET(linked[i]);
			linked[i] = ~0;
		}
	}
};

enum AdcPhase {
	INACTIVE,	// No ADCs need to be measured.
	PREPARING,	// Measuring an Adc the first time.
	MEASURING	// Measuring an Adc the second time.
};

EXTERN Adc adc[NUM_ANALOG_INPUTS];
EXTERN uint8_t adc_current, adc_next;
EXTERN AdcPhase adc_phase;

// timer.cpp
void do_steps();
void handle_motors();

// packet.cpp
void packet();	// A command packet has arrived; handle it.

// serial.cpp
void serial();	// Handle commands from serial.
void send_packet();
void try_send_next();
void write_ack();
void write_stall();

// setup.cpp
void setup();

// firmware.ino
void loop();	// Do stuff which needs doing: moving motors and adjusting heaters.

#include ARCH_INCLUDE

#endif

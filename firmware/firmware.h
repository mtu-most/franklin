#ifndef _FIRMWARE_H
#define _FIRMWARE_H

#include <stdarg.h>
#include ARCH_INCLUDE

#define ID_SIZE 24	// Number of bytes in printerid; 8 bytes, repeated 3 times.
#define PROTOCOL_VERSION 0

#define ADC_INTERVAL 1000	// Delay 1 ms between ADC measurements.

#ifndef NUM_MOTORS
#error "NUM_MOTORS must be defined in the Makefile"
#endif
#ifndef FRAGMENTS_PER_MOTOR_BITS
#error "FRAGMENTS_PER_MOTOR_BITS must be defined in the Makefile"
#endif
#ifndef BYTES_PER_FRAGMENT
#error "BYTES_PER_FRAGMENT must be defined in the Makefile"
#endif
#ifndef SERIAL_SIZE_BITS
#error "SERIAL_SIZE_BITS must be defined in the Makefile"
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

// BEGIN reply is 27 bytes, which is the longest command that doesn't depend on NUM_MOTORS.
#define MAX_REPLY_LEN ((2 + 4 * NUM_MOTORS) > 27 ? (2 + 4 * NUM_MOTORS) : 27)
#define REPLY_BUFFER_SIZE (MAX_REPLY_LEN + (MAX_REPLY_LEN + 2) / 3)

#define SERIAL_MASK ((1 << SERIAL_SIZE_BITS) - 1)
#define FRAGMENTS_PER_MOTOR_MASK ((1 << FRAGMENTS_PER_MOTOR_BITS) - 1)

template <typename _A> _A min(_A a, _A b) { return a < b ? a : b; }
template <typename _A> _A max(_A a, _A b) { return a > b ? a : b; }
template <typename _A> _A abs(_A a) { return a > 0 ? a : -a; }

EXTERN volatile uint16_t debug_value, debug_value1;
EXTERN uint8_t printerid[ID_SIZE];
EXTERN uint8_t uuid[16];
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
EXTERN volatile uint16_t move_phase, full_phase, full_phase_bits;
EXTERN uint8_t filling;
EXTERN uint8_t led_fast;
EXTERN uint16_t led_last, led_phase, time_per_sample;
EXTERN uint8_t led_pin, probe_pin, pin_flags;
EXTERN uint16_t timeout_time, last_active;
EXTERN uint8_t enabled_pins;

EXTERN volatile bool serial_overflow;
EXTERN volatile uint16_t serial_buffer_head;
EXTERN volatile uint16_t serial_buffer_tail;
EXTERN volatile uint8_t serial_buffer[1 << SERIAL_SIZE_BITS];

enum SingleByteCommands {	// See serial.cpp for computation of command values.
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
	CTRL_INPUT,
	CTRL_UNSET,
	CTRL_VALUE = 0x10,
	CTRL_EVENT = 0x20,
	CTRL_NOTIFY = 0x40
};
#define CONTROL_CURRENT(x) ((x) & 0x3)
#define CONTROL_RESET(x) (((x) >> 2) & 0x3)
#define CONTROL_VALUE(x) (bool((x) & CTRL_VALUE))
#define CONTROL_EVENT(x) (bool((x) & CTRL_EVENT))
EXTERN uint8_t pin_events;

struct Pin_t {
	uint8_t state;
	bool value() {
		return CONTROL_VALUE(state);
	}
	bool event() {
		return CONTROL_EVENT(state);
	}
	void clear_event() {
		if (event()) {
			pin_events -= 1;
			state &= ~CTRL_EVENT;
		}
	}
	void disable(uint8_t pin) {
		clear_event();
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
		clear_event();
		state = new_state;
		if (CONTROL_RESET(state) != CONTROL_CURRENT(state))
			enabled_pins += 1;
		if (state & CTRL_NOTIFY) {
			state |= CTRL_EVENT;
			pin_events += 1;
		}
	}
	ARCH_PIN_DATA
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
	CMD_MOVE,	// 1:which, *:samples
	CMD_START,	// 0 start moving.
	CMD_STOP,	// 0 stop moving.
	CMD_ABORT,	// 0 stop moving and set all pins to their reset state.
	CMD_DISCARD,	// 1:num_fragments
	CMD_GETPIN,	// 1:pin

	// to host
		// responses to host requests; only one active at a time.
	CMD_READY = 0x60,	// 1:packetlen, 4:version, 1:num_dpins, 1:num_adc, 1:num_motors, 1:fragments/motor, 1:bytes/fragment
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
	CMD_TIMEOUT,	// 0
	CMD_PINCHANGE,	// 1:which, 1: state

	// from host, but not "normal".
	CMD_AUDIO = 0xc0
};

static inline volatile uint8_t &command(uint16_t pos) {
	return serial_buffer[(serial_buffer_tail + pos) & SERIAL_MASK];
}

static inline uint16_t minpacketlen() {
	switch (command(0) & ~0x10) {
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
		debug("invalid command passed to minpacketlen: %x", command(0));
		return 1;
	};
}

struct Motor
{
	volatile int32_t current_pos;
	uint8_t step_pin;
	uint8_t dir_pin;
	volatile uint8_t steps_current;
	int32_t sense_pos[2];
	uint8_t limit_min_pin;
	uint8_t limit_max_pin;
	uint8_t sense_pin;
	volatile uint8_t flags;
	volatile bool audio;
	ARCH_MOTOR
	enum Flags {
		LIMIT			= 0x01,
		SENSE0			= 0x02,
		SENSE1			= 0x04,
		INVERT_LIMIT_MIN	= 0x08,
		INVERT_LIMIT_MAX	= 0x10,
		INVERT_STEP		= 0x20,
		SENSE_STATE		= 0x40,
		ACTIVE_BIT = 7,
		ACTIVE			= 1 << ACTIVE_BIT
	};
	void init() {
		current_pos = 0;
		sense_pos[0] = 0xBEEFBEEF;
		sense_pos[1] = 0xFACEFACE;
		flags = 0;
		steps_current = 0;
		step_bitmask = 0;
		dir_bitmask = 0;
		audio = false;
		step_pin = ~0;
		dir_pin = ~0;
		limit_min_pin = ~0;
		limit_max_pin = ~0;
		sense_pin = ~0;
	}
	void disable() {
		current_pos = 0;
		steps_current = 0;
		audio = false;
		if (step_pin < NUM_DIGITAL_PINS)
			UNSET(step_pin);
		if (dir_pin < NUM_DIGITAL_PINS)
			UNSET(dir_pin);
		if (limit_min_pin < NUM_DIGITAL_PINS)
			UNSET(limit_min_pin);
		if (limit_max_pin < NUM_DIGITAL_PINS)
			UNSET(limit_max_pin);
		if (sense_pin < NUM_DIGITAL_PINS)
			UNSET(sense_pin);
	}
};

EXTERN volatile int8_t buffer[1 << FRAGMENTS_PER_MOTOR_BITS][NUM_MOTORS][BYTES_PER_FRAGMENT];
EXTERN Motor motor[NUM_MOTORS];
EXTERN volatile uint8_t active_motors;
EXTERN int stopping;	// number of switch which has been hit, or active_motors for a probe hit and -1 for none.
EXTERN uint32_t home_step_time;
EXTERN uint8_t homers;

struct Settings {
	bool probing;
	uint8_t len;
};

EXTERN Settings settings[1 << FRAGMENTS_PER_MOTOR_BITS];
EXTERN uint8_t notified_current_fragment;
EXTERN volatile uint8_t current_fragment;	// Fragment that is currently active, or if none, the one that will next be active.
EXTERN volatile uint8_t current_sample;		// The sample in the current fragment that is active.
EXTERN volatile uint8_t current_len;		// Copy of settings[current_fragment].len, for easy access from asm.
EXTERN volatile uint8_t step_state;		// 0: disabled; 1: Waiting for limit switch check; 2: Waiting for step; 3: free running.
EXTERN volatile int8_t (*volatile current_buffer)[NUM_MOTORS][BYTES_PER_FRAGMENT];
EXTERN volatile uint8_t last_fragment;	// Fragment that is currently being filled.
EXTERN uint8_t limit_fragment_pos;
EXTERN uint8_t last_len;	// copy of settings[last_fragment].len, for when current_fragment changes during a fill.

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

/* firmware.h - declarations for Franklin
 * vim: set foldmethod=marker :
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

#ifndef _FIRMWARE_H
#define _FIRMWARE_H

#define FAST_ISR

#include <stdarg.h>
#include ARCH_INCLUDE

#define ID_SIZE 8	// Number of bytes in machineid; 8.
#define UUID_SIZE 16	// Number of bytes in uuid; 16.
#define PROTOCOL_VERSION 4

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

#define MAXLONG (int32_t(uint32_t(~0) >> 1))
#define MAXINT (int(unsigned(~0) >> 1))

// Exactly one file defines EXTERN as empty, which leads to the data to be defined.
#ifndef EXTERN
#define EXTERN extern
#else
#define DEFINE_VARIABLES
#endif

#define BUFFER_CHECK(buffer, index) do { \
	int _i = (index); \
	if (_i < 0 || _i >= int(sizeof(buffer) / sizeof((buffer)[0]))) \
		debug("Failed buffer access for " #buffer "[" #index "]"); \
} while (0)

// BEGIN reply is the longest command that doesn't depend on NUM_MOTORS.
#define MAX_REPLY_LEN ((4 + 4 * NUM_MOTORS) > 11 + UUID_SIZE ? (4 + 4 * NUM_MOTORS) : 11 + UUID_SIZE)
#define REPLY_BUFFER_SIZE (MAX_REPLY_LEN + (MAX_REPLY_LEN + 2) / 3)

#define SERIAL_BUFFER_SIZE (1 << SERIAL_SIZE_BITS)
#define SERIAL_MASK (SERIAL_BUFFER_SIZE - 1)
#define FRAGMENTS_PER_MOTOR_MASK ((1 << FRAGMENTS_PER_MOTOR_BITS) - 1)

#ifndef NO_DEBUG
static inline void debug(char const *fmt, ...);
static inline void debug_add(int i);
static inline void debug_dump();
#else
static inline void debug(char const *fmt, ...) { (void)&fmt; }
static inline void debug_add(int i) { (void)&i; }
static inline void debug_dump() {}
#endif

static inline void arch_msetup(uint8_t m);

template <typename _A, typename _B> _A min(_A a, _B b) { return a < b ? a : b; }
template <typename _A, typename _B> _A max(_A a, _B b) { return a > b ? a : b; }
template <typename _A> _A abs(_A a) { return a > 0 ? a : -a; }
#define fabs abs

// Volatile variables which are used by interrupt handlers.
EXTERN volatile uint16_t debug_value, debug_value1;
EXTERN volatile uint8_t move_phase, full_phase, full_phase_bits;
EXTERN volatile bool serial_overflow;
EXTERN volatile uint8_t *serial_buffer_head;
EXTERN volatile uint8_t *serial_buffer_tail;
EXTERN volatile uint8_t serial_buffer[SERIAL_BUFFER_SIZE] __attribute__ ((aligned (SERIAL_BUFFER_SIZE)));
EXTERN volatile uint8_t buffer[1 << FRAGMENTS_PER_MOTOR_BITS][NUM_MOTORS][BYTES_PER_FRAGMENT];
EXTERN volatile uint8_t active_motors;
EXTERN volatile uint8_t current_fragment;	// Fragment that is currently active, or if none, the one that will next be active.
EXTERN volatile uint8_t current_sample;		// The sample in the current fragment that is active.
EXTERN volatile uint8_t current_len;		// Copy of settings[current_fragment].len, for easy access from asm.
EXTERN volatile uint8_t step_state;		// 0: disabled; 1: Waiting for limit switch check; 2: Waiting for step; 3: free running.
EXTERN volatile uint8_t (*volatile current_buffer)[NUM_MOTORS][BYTES_PER_FRAGMENT];
EXTERN volatile uint8_t last_fragment;	// Fragment that is currently being filled.

EXTERN uint8_t machineid[1 + ID_SIZE + UUID_SIZE + (1 + ID_SIZE + UUID_SIZE + 2) / 3];
EXTERN int16_t command_end;
EXTERN bool had_data;
EXTERN uint8_t reply[MAX_REPLY_LEN], adcreply[6];
EXTERN uint8_t ping;			// bitmask of waiting ping replies.
EXTERN uint8_t out_busy;
EXTERN uint8_t reply_ready, adcreply_ready;
EXTERN bool timeout;
EXTERN uint8_t ff_in;
EXTERN uint8_t ff_out;
EXTERN uint8_t pending_packet[4][REPLY_BUFFER_SIZE];
EXTERN int16_t pending_len[4];
EXTERN uint8_t filling;
EXTERN uint8_t led_fast;
EXTERN uint16_t led_last, led_phase, time_per_sample;
EXTERN uint8_t led_pin, stop_pin, probe_pin, pin_flags;
EXTERN uint8_t spiss_pin;
EXTERN uint16_t timeout_time, last_active;
EXTERN uint8_t enabled_pins;

enum SingleByteCommands {	// See serial.cpp for computation of command values.
	CMD_NACK0 = 0xf0,	// Incorrect packet; please resend.
	CMD_NACK1 = 0x91,	// Incorrect packet; please resend.
	CMD_NACK2 = 0xa2,	// Incorrect packet; please resend.
	CMD_NACK3 = 0xc3,	// Incorrect packet; please resend.
	CMD_ACK0 = 0xc4,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACK1 = 0xa5,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACK2 = 0x96,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_ACK3 = 0xf7,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_STALL0 = 0x88,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL1 = 0xe9,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL2 = 0xda,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL3 = 0xbb,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_ID = 0xbc,		// Request/reply machine ID code.
	CMD_DEBUG = 0xdd,	// Debug message; a nul-terminated message follows (no checksum; no resend).
	CMD_STARTUP = 0xee,	// Starting up.
	CMD_STALLACK = 0x8f	// Clear stall.
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

inline void SET_OUTPUT(uint8_t pin_no);
inline void SET_INPUT(uint8_t pin_no);
inline void UNSET(uint8_t pin_no);
inline void SET(uint8_t pin_no);
inline void RESET(uint8_t pin_no);
inline bool GET(uint8_t pin_no);

struct Pin_t {
	uint8_t state;
	uint8_t duty;
	uint8_t num_temps;
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
		//uint8_t old_enabled_pins = enabled_pins;
		//uint8_t old_state = state;
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
		//debug("new enabled: %d %d %d", old_enabled_pins, enabled_pins, old_state, new_state);
	}
	ARCH_PIN_DATA
};
EXTERN Pin_t pin[NUM_DIGITAL_PINS];

enum Command {
	// from host
	CMD_BEGIN = 0x00,	// 0
	CMD_PING,	// 1:code
	CMD_SET_UUID,	// 16: UUID
	CMD_SETUP,	// 1:active_motors, 4:us/sample, 1:led_pin, 1:stop_pin 1:probe_pin 1:pin_flags 2:timeout
	CMD_CONTROL,	// 1:num_commands, {1: command, 1: arg}
	CMD_MSETUP,	// 1:motor, 1:step_pin, 1:dir_pin, 1:limit_min_pin, 1:limit_max_pin, 1:follow, 1:flags
	CMD_ASETUP,	// 1:adc, 2:linked_pins, 4:limits, 4:values	(including flags)
	CMD_HOME,	// 4:us/step, {1:dir}*

	CMD_START_MOVE,	// 1:num_samples, 1:num_moving_motors
	CMD_START_PROBE,// 1:num_samples, 1:num_moving_motors
	CMD_MOVE,	// 1:which, *:samples
	CMD_MOVE_SINGLE,// 1:which, *:samples
	CMD_PWM,	// 1:which, *:samples
	CMD_START,	// 0 start moving.
	CMD_STOP,	// 0 stop moving.
	CMD_ABORT,	// 0 stop moving and set all pins to their reset state.
	CMD_DISCARD,	// 1:num_fragments
	CMD_GETPIN,	// 1:pin
	CMD_SPI,	// 1:size, size: data.
	CMD_PINNAME,	// 1:pin (0-127: digital, 128-255: analog)
};

enum RCommand {
	// to host
		// responses to host requests; only one active at a time.
	CMD_READY = 0x10,	// 1:packetlen, 4:version, 1:num_dpins, 1:num_adc, 1:num_motors, 1:fragments/motor, 1:bytes/fragment
	CMD_PONG,	// 1:code
	CMD_HOMED,	// {4:motor_pos}*
	CMD_PIN,	// 1:state
	CMD_STOPPED,	// 1:fragment_pos, {4:motor_pos}*
	CMD_NAMED_PIN,	// 1:length, n:name

		// asynchronous events.
	CMD_DONE,	// 1:num
	CMD_UNDERRUN,	// 1:num
	CMD_ADC,	// 1:which, 2:adc reading
	CMD_LIMIT,	// 1:which, 1:pos, {4:motor_pos}*
	CMD_TIMEOUT,	// 0
	CMD_PINCHANGE,	// 1:which, 1: state
};

static inline uint8_t command(int16_t pos) {
	//debug("cmd %x = %x (%x + %x & %x)", (serial_buffer_tail + pos) & SERIAL_MASK, serial_buffer[(serial_buffer_tail + pos) & SERIAL_MASK], serial_buffer_tail, pos, SERIAL_MASK);
	return *(volatile uint8_t *)(((uint16_t(serial_buffer_tail) + pos) & SERIAL_MASK) | uint16_t(serial_buffer));
}

static inline int16_t minpacketlen() {
	switch (command(0) & 0x1f) {
	case CMD_BEGIN:
		return 2;
	case CMD_PING:
		return 2;
	case CMD_SET_UUID:
		return 1 + UUID_SIZE;
	case CMD_SETUP:
		return 14;
	case CMD_CONTROL:
		return 4;
	case CMD_MSETUP:
		return 8;
	case CMD_ASETUP:
		return 18;
	case CMD_HOME:
		return 5;
	case CMD_START_MOVE:
		return 3;
	case CMD_START_PROBE:
		return 3;
	case CMD_MOVE:
		return 3;
	case CMD_MOVE_SINGLE:
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
	case CMD_SPI:
		return 2;
	case CMD_PINNAME:
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
	uint8_t limit_min_pin;
	uint8_t limit_max_pin;
	uint8_t follow;
	volatile uint8_t intflags;	// Flags that are used by the interrupt handler.
	uint8_t flags;	// Flags that are not used by the interrupt handler.
	ARCH_MOTOR
	enum IntFlags {
		ACTIVE_BIT = 0,
		ACTIVE			= 1 << ACTIVE_BIT,
		PWM_BIT = 2,
		PWM			= 1 << PWM_BIT,
		INVERT_STEP_BIT = 6,
		INVERT_STEP		= 1 << INVERT_STEP_BIT
	};
	enum Flags {
		LIMIT			= 0x01,
		INVERT_LIMIT_MIN	= 0x02,
		INVERT_LIMIT_MAX	= 0x04,
	};
	void init(uint8_t m) {
		//debug("init motor %d", m);
		current_pos = 0;
		intflags = 0;
		flags = 0;
		steps_current = 0;
		step_pin = ~0;
		dir_pin = ~0;
		limit_min_pin = ~0;
		limit_max_pin = ~0;
		follow = ~0;
		arch_msetup(m);
	}
	void disable(uint8_t m) {
		//debug("disable motor %d", m);
		intflags = 0;
		flags = 0;
		current_pos = 0;
		steps_current = 0;
		if (step_pin < NUM_DIGITAL_PINS)
			UNSET(step_pin);
		if (dir_pin < NUM_DIGITAL_PINS)
			UNSET(dir_pin);
		if (limit_min_pin < NUM_DIGITAL_PINS)
			UNSET(limit_min_pin);
		if (limit_max_pin < NUM_DIGITAL_PINS)
			UNSET(limit_max_pin);
		step_pin = ~0;
		dir_pin = ~0;
		limit_min_pin = ~0;
		limit_max_pin = ~0;
		follow = ~0;
		arch_msetup(m);
	}
};

EXTERN Motor motor[NUM_MOTORS];
EXTERN int stopping;	// number of switch which has been hit, or active_motors for a probe hit and -1 for none.
EXTERN uint32_t home_step_time;
EXTERN uint8_t homers;

struct Settings {
	uint8_t flags;
	uint8_t len;
	enum {
		PROBING = 1
	};
};

// Step states.  The non-moving states must be first.
#define STATE_DECAY 2
enum StepState {
	STEP_STATE_STOP,	// Not running.
	STEP_STATE_WAIT,	// Waiting for a probe before continuing as RUN
	STEP_STATE_PROBE,	// Waiting for a probe before continuing as SINGLE
	STEP_STATE_NEXT = STATE_DECAY + STEP_STATE_WAIT,	// Running a sample, then move to PROBE; reset to RUN when limits are checked.
	STEP_STATE_SINGLE = STATE_DECAY + STEP_STATE_PROBE,	// Running a single step, then fall back to PROBE.
	STEP_STATE_RUN = STATE_DECAY + STEP_STATE_NEXT,		// Running a sample, then move to NEXT.
};
#define NUM_NON_MOVING_STATES 3
EXTERN Settings settings[1 << FRAGMENTS_PER_MOTOR_BITS];
EXTERN uint8_t notified_current_fragment;

EXTERN uint8_t limit_fragment_pos;
EXTERN uint8_t last_len;	// copy of settings[last_fragment].len, for when current_fragment changes during a fill.

struct Adc {
	uint8_t linked[2];
	int16_t value[2];	// bit 15 in [0] set => invalid; bit 14 set => linked inverted.
	int16_t limit[2][2];
	bool is_on[2];
	uint16_t hold_time;
	unsigned long last_change;
	void disable() {
		if (value[0] & 0x8000)
			return;
		for (uint8_t i = 0; i < 2; ++i) {
			if (linked[i] >= NUM_DIGITAL_PINS)
				continue;
			UNSET(linked[i]);
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
void send_id(uint8_t cmd);

// setup.cpp
void setup();

// firmware.ino
void loop();	// Do stuff which needs doing: moving motors and adjusting heaters.

static inline void write_current_pos(uint8_t offset) {
	cli();
	for (uint8_t m = 0; m < active_motors; ++m) {
		BUFFER_CHECK(pending_packet[ff_out], offset + 4 * m);
		BUFFER_CHECK(motor, m);
		*reinterpret_cast <int32_t *>(&pending_packet[ff_out][offset + 4 * m]) = motor[m].current_pos;
	}
	sei();
}

static inline void SLOW_ISR() {
#ifndef FAST_ISR
	if (step_state < NUM_NON_MOVING_STATES)
		return;
	arch_disable_isr();
	sei();
	move_phase += 1;
	for (uint8_t m = 0; m < active_motors; ++m) {
		if (~motor[m].intflags & Motor::ACTIVE)
			continue;
		int16_t sample = *reinterpret_cast <volatile int16_t *> (&(*current_buffer)[m][current_sample]);
		if (sample == 0)
			continue;
		int8_t target = ((abs(sample) * move_phase) >> full_phase_bits) - motor[m].steps_current;
		//debug("sample %d %d %d %d %d %d %d", m, sample, abs(sample), move_phase, full_phase, target, motor[m].steps_current);
		if (target == 0)
			continue;
		// Set dir.
		if (sample < 0) {
			RESET(motor[m].dir_pin);
			//debug("reset dir %d", m);
		}
		else {
			SET(motor[m].dir_pin);
			//debug("set dir %d", m);
		}
		for (uint8_t i = 0; i < target; ++i) {
			if (motor[m].intflags & Motor::INVERT_STEP) {
				RESET(motor[m].step_pin);
				SET(motor[m].step_pin);
			}
			else {
				SET(motor[m].step_pin);
				RESET(motor[m].step_pin);
			}
			//debug("pulse %d", m);
			motor[m].steps_current += 1;
			motor[m].current_pos += sample > 0 ? 1 : -1;
		}
	}
	//debug("iteration frag %d sample %d = %d current %d pos %d", current_fragment, current_sample, (*current_buffer)[0][current_sample], motor[0].steps_current, motor[0].current_pos);
	if (move_phase >= full_phase) {
		move_phase = 0;
		step_state -= STATE_DECAY;
		for (uint8_t m = 0; m < active_motors; ++m)
			motor[m].steps_current = 0;
		current_sample += 2;
		if (current_sample >= current_len) {
			current_sample = 0;
			current_fragment = (current_fragment + 1) & ((1 << FRAGMENTS_PER_MOTOR_BITS) - 1);
			BUFFER_CHECK(buffer, current_fragment);
			current_buffer = &buffer[current_fragment];
			if (current_fragment != last_fragment) {
				for (uint8_t m = 0; m < active_motors; ++m) {
					//debug("active %d %d", m, (*current_buffer)[m][0]);
					BUFFER_CHECK(motor, m);
					if ((*current_buffer)[m][0] != 0 || (*current_buffer)[m][1] != uint8_t(0x80))
						motor[m].intflags |= Motor::ACTIVE;
					else
						motor[m].intflags &= ~Motor::ACTIVE;
				}
				BUFFER_CHECK(settings, current_fragment);
				current_len = settings[current_fragment].len;
			}
			else {
				// Underrun.
				arch_set_speed(0);
				//debug("underrun");
			}
		}
	}
	cli();
	if (step_state != STEP_STATE_STOP)
		arch_enable_isr();
#endif
}

#include ARCH_INCLUDE

#endif

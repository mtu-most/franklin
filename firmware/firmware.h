#ifndef _FIRMWARE_H
#define _FIRMWARE_H

#include <stdarg.h>
#include ARCH_INCLUDE

#define ID_SIZE 16	// Number of bytes in printerid; it's a UUID which is always 16 bytes.
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
#endif

#define COMMAND_SIZE 256
#define FULL_COMMAND_SIZE (COMMAND_SIZE + (COMMAND_SIZE + 2) / 3)

EXTERN char printerid[ID_SIZE];
EXTERN unsigned char command[FULL_COMMAND_SIZE];
EXTERN uint16_t command_end;
EXTERN char reply[FULL_COMMAND_SIZE], adcreply[6];
EXTERN uint8_t ping;			// bitmask of waiting ping replies.
EXTERN bool out_busy;
EXTERN uint32_t out_time;
EXTERN uint8_t reply_ready, adcreply_ready;
EXTERN char pending_packet[FULL_COMMAND_SIZE];
EXTERN uint16_t pending_len;
EXTERN bool stopped, underrun;
EXTERN uint8_t filling;
EXTERN bool led_fast;
EXTERN uint32_t led_last, led_phase, start_time, time_per_sample;
EXTERN uint8_t led_pin;

enum SingleByteCommands {	// See serial.cpp for computation of command values.
// These bytes (except RESET) are sent in reply to a received packet only.
	CMD_NACK = 0x80,	// Incorrect packet; please resend.
	CMD_ACK0 = 0xb3,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_STALL0 = 0x87,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_STALL1 = 0x9e,	// Packet properly received, but not accepted; don't resend packet unmodified.
	CMD_ID = 0xaa,		// Request/reply printer ID code.
	CMD_ACK1 = 0xad,	// Packet properly received and accepted; ready for next command.  Reply follows if it should.
	CMD_DEBUG = 0xb4,	// Debug message; a nul-terminated message follows (no checksum; no resend).
	UNUSED = 0x99
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

enum Command {
	// from host
	CMD_BEGIN = 0x40,	// 0
	CMD_PING,	// 1:code
	CMD_RESET,	// 4:magic	reset the mcu.
	CMD_SETUP,	// 1:led_pin, 4:us/sample
	CMD_CONTROL,	// 1:num_commands, {1: command, 1: arg}
	CMD_MSETUP,	// 1:motor, 1:step_pin, 1:dir_pin, 1:limit_min_pin, 1:limit_max_pin, 1:sense_pin, 1:flags
	CMD_ASETUP,	// 1:adc, 2:linked_pins, 4:values	(including flags)
	CMD_HOME,	// 4:us/step, {1:dir}*

	CMD_START_MOVE,	// 1:num_samples, 1:num_moving_motors
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
	CMD_HOMED,	// 1:code, {4:motor_pos}*
	CMD_PIN,	// 1:state
	CMD_STOPPED,	// 1:fragment, 1:fragment_pos, {4:motor_pos}*

		// asynchronous events.
	CMD_DONE,	// 1:num
	CMD_UNDERRUN,	// 1:num
	CMD_ADC,	// 1:which, 2:adc reading
	CMD_LIMIT,	// 1:which, 1:pos, {4:motor_pos}*
	CMD_SENSE0,	// 1:which, {4:motor_pos}*
	CMD_SENSE1	// 1:which, {4:motor_pos}*
};

static inline uint8_t minpacketlen() {
	switch (command[0] & ~0x10) {
	case CMD_BEGIN:
		return 1;
	case CMD_PING:
		return 2;
	case CMD_RESET:
		return 5;
	case CMD_SETUP:
		return 6;
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
		debug("invalid command passed to minpacketlen: %x", int(uint8_t(command[0])));
		return 1;
	};
}

struct Pin_t {
	uint8_t state;
};
EXTERN Pin_t pin[NUM_DIGITAL_PINS];

inline void SET_OUTPUT(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	if ((pin[pin_no].state & 0x3) == CTRL_SET || (pin[pin_no].state & 0x3) == CTRL_RESET)
		return;
	digitalWrite(pin_no, LOW);
	pinMode(pin_no, OUTPUT);
	pin[pin_no].state &= ~0x3;
	pin[pin_no].state |= CTRL_RESET;
}
inline void SET_INPUT(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	pinMode(pin_no, INPUT_PULLUP);
	pin[pin_no].state &= ~0x3;
	pin[pin_no].state |= CTRL_INPUT;
}
inline void UNSET(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	pinMode(pin_no, INPUT);
	pin[pin_no].state &= ~0x3;
	pin[pin_no].state |= CTRL_UNSET;
}
inline void SET(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	if ((pin[pin_no].state & 0x3) == CTRL_SET)
		return;
	SET_OUTPUT(pin_no);
	digitalWrite(pin_no, HIGH);
	pin[pin_no].state &= ~0x3;
	pin[pin_no].state |= CTRL_SET;
}
inline void RESET(uint8_t pin_no) {
	if (pin_no >= NUM_DIGITAL_PINS)
		return;
	if ((pin[pin_no].state & 0x3) != CTRL_SET)
		return;
	SET_OUTPUT(pin_no);
	digitalWrite(pin_no, LOW);
	pin[pin_no].state &= ~0x3;
	pin[pin_no].state |= CTRL_RESET;
}
inline bool GET(uint8_t pin_no) {
	return digitalRead(pin_no) == HIGH;
}

struct Motor
{
	int32_t current_pos, sense_pos[2];
	uint8_t step_pin;
	uint8_t dir_pin;
	uint8_t limit_min_pin;
	uint8_t limit_max_pin;
	uint8_t sense_pin;
	uint8_t buffer;
	uint8_t flags;
	uint16_t pos;
	enum Flags {
		LIMIT = 1,
		SENSE0 = 2,
		SENSE1 = 4,
		INVERT_LIMIT_MIN = 8,
		INVERT_LIMIT_MAX = 16,
		INVERT_STEP = 32,
		SENSE_STATE = 64,
		ACTIVE = 128
	};
	void init(uint8_t b) {
		buffer = b;
		current_pos = 0;
		sense_pos[0] = 0xBEEFBEEF;
		sense_pos[1] = 0xFACEFACE;
		flags = 0;
		pos = 0;
	}
};

EXTERN Motor motor[NUM_MOTORS];
EXTERN uint8_t active_motors;
EXTERN bool stopping;
EXTERN uint32_t home_step_time;
EXTERN uint8_t homers;

enum Dir {
	DIR_POSITIVE,
	DIR_NEGATIVE,
	DIR_AUDIO,
	DIR_NONE
};

struct Fragment {
	Dir dir;
	uint16_t num_samples;
	uint8_t samples[BYTES_PER_FRAGMENT];
};

typedef Fragment Buffer[FRAGMENTS_PER_BUFFER];

EXTERN Buffer buffer[NUM_BUFFERS];
EXTERN uint8_t fragment_len[FRAGMENTS_PER_BUFFER];
EXTERN uint8_t notified_current_fragment;
EXTERN uint8_t current_fragment;	// Fragment that is currently active, or if none, the one that will next be active.
EXTERN uint8_t last_fragment;	// Fragment that is currently being filled, or has last been filled.
EXTERN uint8_t limit_fragment;
EXTERN uint8_t limit_fragment_pos;
EXTERN uint8_t current_fragment_pos;

struct Adc {
	uint8_t linked[2];
	uint16_t value[2];	// bit 15 in [0] set => invalid; bit 14 set => linked inverted.
};

enum AdcPhase {
	INACTIVE,	// No ADCs need to be measured.
	PREPARING,	// Measuring an Adc the first time.
	MEASURING	// Measuring an Adc the second time.
};

EXTERN Adc adc[NUM_ANALOG_INPUTS];
EXTERN uint8_t adc_current, adc_next;
EXTERN AdcPhase adc_phase;

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

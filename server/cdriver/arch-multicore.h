/* arch-multicore.h - Multicore specific parts for Franklin {{{
 * vim: set foldmethod=marker :
 * Copyright 2014-2016 Michigan Technological University
 * Copyright 2016-2017 Bas Wijnen <wijnen@debian.org>
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
// }}}

#ifndef ADCBITS

// Includes. {{{
#include <stdint.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <sys/types.h>
#include <dirent.h>
#include <poll.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <cmath>
#include <sys/mman.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
// }}}

// Defines. {{{
#ifdef PINE64
#define NUM_GPIO_PINS 36
#define NUM_ANALOG_INPUTS 0
#else
#ifdef ORANGEPIZERO
#define NUM_GPIO_PINS 15
#define NUM_ANALOG_INPUTS 1
#endif
#endif
#define NUM_DIGITAL_PINS NUM_GPIO_PINS
#define NUM_PINS (NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS)
#define ADCBITS 12
#define FRAGMENTS_PER_BUFFER 8
#define SAMPLES_PER_FRAGMENT 256	// This value is used implicitly as the overflow value of uint8_t.

#define ARCH_MOTOR int mc_homer;
#define ARCH_SPACE

#define DATA_CLEAR() memset((void *)(mc_shared->buffer[mc_shared->next_fragment]), 0, sizeof(mc_shared->buffer[0]))
#define ARCH_NEW_MOTOR(s, m, base) do {} while (0)
#define DATA_DELETE(s, m) do {} while (0)
#define ARCH_MAX_FDS 0
// }}}

#else

struct MCTemp { // {{{
	int id;
	bool active;
	int heater_pin, fan_pin;
	bool heater_inverted, fan_inverted;
	int heater_adctemp, fan_adctemp;
	int heater_limit_l, heater_limit_h, fan_limit_l, fan_limit_h;
	double hold_time;
}; // }}}

enum MCPort { A, B, C, D, E, F, G, H, NUM_PORTS };
struct MCShared { // {{{
	volatile uint64_t base, dirs;
	volatile uint64_t buffer[FRAGMENTS_PER_BUFFER][SAMPLES_PER_FRAGMENT][2], homers[2];
	// These must be bytes, to be sure that read and write are atomic even with races between different cores.
	volatile uint8_t current_sample, current_fragment, next_fragment, state;
	volatile uint8_t mode[NUM_GPIO_PINS];	// Current mode for all pins.
	volatile uint32_t config[NUM_PORTS][4];
	volatile uint32_t data[NUM_PORTS];
	volatile uint32_t pull[NUM_PORTS][2];
	volatile int num_homers;
}; // }}}
EXTERN MCShared *mc_shared;

// Function declarations. {{{
void SET_OUTPUT(Pin_t _pin);
void SET_INPUT(Pin_t _pin);
void SET_INPUT_NOPULLUP(Pin_t _pin);
void SET(Pin_t _pin);
void RESET(Pin_t _pin);
void GET(Pin_t _pin, bool _default, void(*cb)(bool));
void arch_setup_start();
void arch_connect(char const *run_id, char const *port);
void arch_request_temp(int which);
void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin = ~0, bool heater_invert = false, int heater_adctemp = 0, int heater_limit_l = ~0, int heater_limit_h = ~0, int fan_pin = ~0, bool fan_invert = false, int fan_adctemp = 0, int fan_limit_l = ~0, int fan_limit_h = ~0, double hold_time = 0);
void arch_send_pin_name(int pin);
void arch_motors_change();
void arch_addpos(int s, int m, double diff);
void arch_invertpos(int s, int m);
void arch_stop(bool fake);
void arch_home();
bool arch_running();
void arch_start_move(int extra);
bool arch_send_fragment();
int arch_fds();
int arch_tick();
void arch_set_duty(Pin_t pin, double duty);
double arch_get_duty(Pin_t pin);
void arch_discard();
void arch_send_spi(int bits, const uint8_t *data);
void DATA_SET(int s, int m, int value);
// }}}

#ifdef DEFINE_VARIABLES
// Variables. {{{
static int mc_active_temp;
static MCTemp mc_temp[NUM_ANALOG_INPUTS];
enum State { GPIO_INPUT = 0, GPIO_OUTPUT = 1, GPIO_OFF = 7, };
enum Pull { NOPULL, PULLUP, PULLDOWN };
struct state_demux {
	int state;
	int pull;
	int value;
	state_demux(int s, int p, int v) : state(s), pull(p), value(v) {}
};
static state_demux const states[5] = {
	state_demux(GPIO_OUTPUT, NOPULL, 0),
	state_demux(GPIO_OUTPUT, NOPULL, 1),
	state_demux(GPIO_INPUT, PULLUP, 0),
	state_demux(GPIO_INPUT, NOPULL, 0),
	state_demux(GPIO_OFF, NOPULL, 0)
};

static volatile uint32_t *mc_piomem;
static struct timespec mc_pwm_time;
static double mc_duty[NUM_DIGITAL_PINS];
static bool mc_pwm_on[NUM_DIGITAL_PINS];
static double mc_pwm_target[NUM_DIGITAL_PINS];
bool mc_pin_state[NUM_DIGITAL_PINS];
int adc_fd;
// }}}

// Pin setting. {{{
// Set the direction of a single pin.
static void gpio_setdir(int port, int pin, int state) { // {{{
	//debug("setdir %x %x %x", port, pin, state);
	int unit = pin / 8;
	pin %= 8;
	uint32_t addr = (0x24 * port + 4 * unit) >> 2;
	uint32_t value = mc_shared->config[port][unit] & ~(7 << (pin * 4));
	value |= state << (pin * 4);
	mc_piomem[addr] = value;
	mc_shared->config[port][unit] = value;
} // }}}

// Set the pull direction of a pin.
static void gpio_setpull(int port, int pin, int pullstate) { // {{{
	//debug("setpull %x %x %x", port, pin, pullstate);
	int unit = pin / 16;
	pin %= 16;
	uint32_t addr = (0x1c + 0x24 * port + 4 * unit) >> 2;
	uint32_t value = mc_shared->pull[port][unit] & ~(3 << (pin * 2));
	value |= pullstate << (pin * 2);
	mc_piomem[addr] = value;
	mc_shared->pull[port][unit] = value;
} // }}}

// Set the value of an output pin.
static void gpio_setpin(int port, int pin, bool is_on) { // {{{
	//debug("setpin %x %x %x", port, pin, is_on);
	uint32_t addr = (0x10 + 0x24 * port) >> 2;
	uint32_t value = mc_shared->data[port] & ~(1 << pin);
	value |= is_on << pin;
	mc_piomem[addr] = value;
	mc_shared->data[port] = value;
} // }}}

// Get the value of an input pin.
static bool gpio_getpin(int port, int pin) { // {{{
	return (mc_piomem[(0x10 + 0x24 * port) >> 2] >> pin) & 1;
} // }}}

// Documentation on pin definitions. {{{
//int num_pins = 32 * NUM_PORTS;
//#define NONE "\x00N/A"
//#define PIN(x) "\x07" x
//#define NOPIN(x) "\x00" x
// Pin types:
// 1: realtime output
// 2: regular output
// 4: digital input
// 8: analog input
/*char const *pin_names[32 * NUM_PORTS] = {
	NOPIN("PB0"), NOPIN("PB1"), PIN("PB2 (E-27)"), PIN("PB3 (E-11)"), PIN("PB4 (E-12)"), PIN("PB5 (E-13)"), PIN("PB6 (E-15)"), NOPIN("PB7"),
	PIN("PB8 (E-29)"), PIN("PB9 (E-30)"), NONE, NONE, NONE, NONE, NONE, NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,

	PIN("PC0 (Pi-19)"), PIN("PC1 (Pi-21)"), PIN("PC2 (Pi-23)"), PIN("PC3 (Pi-24)"), PIN("PC4 (Pi-32)"), PIN("PC5 (Pi-33)"), PIN("PC6 (Pi-36)"), PIN("PC7 (Pi-11)"),
	PIN("PC8 (Pi-12)"), PIN("PC9 (Pi-35)"), PIN("PC10 (Pi-38)"), PIN("PC11 (Pi-40)"), PIN("PC12 (Pi-15)"), PIN("PC13 (Pi-16)"), PIN("PC14 (Pi-18)"), PIN("PC15 (Pi-22)"),
	PIN("PC16 (Pi-37)"), NONE, NONE, NONE, NONE, NONE, NONE, NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,

	PIN("PD0 (E-24)"), PIN("PD1 (E-23)"), PIN("PD2 (E-19)"), PIN("PD3 (E-21)"), NOPIN("PD4"), PIN("PD5 (E-22)"), PIN("PD6 (E-26)"), PIN("PD7 (E-28)"),
	NOPIN("PD8"), NOPIN("PD9"), NOPIN("PD10"), NOPIN("PD11"), NOPIN("PD12"), NOPIN("PD13"), NOPIN("PD14"), NOPIN("PD15"),
	NOPIN("PD16"), NOPIN("PD17"), NOPIN("PD18"), NOPIN("PD19"), NOPIN("PD20"), NOPIN("PD21"), NOPIN("PD22"), NOPIN("PD23"),
	NOPIN("PD24"), NONE, NONE, NONE, NONE, NONE, NONE, NONE,

	NOPIN("PE0"), NOPIN("PE1"), NOPIN("PE2"), NOPIN("PE3"), NOPIN("PE4"), NOPIN("PE5"), NOPIN("PE6"), NOPIN("PE7"),
	NOPIN("PE8"), NOPIN("PE9"), NOPIN("PE10"), NOPIN("PE11"), NOPIN("PE12"), NOPIN("PE13"), NOPIN("PE14"), NOPIN("PE15"),
	NOPIN("PE16"), NOPIN("PE17"), NONE, NONE, NONE, NONE, NONE, NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,

	NOPIN("PF0"), NOPIN("PF1"), NOPIN("PF2"), NOPIN("PF3"), NOPIN("PF4"), NOPIN("PF5"), NOPIN("PF6"), NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,

	NOPIN("PG0"), NOPIN("PG1"), NOPIN("PG2"), NOPIN("PG3"), NOPIN("PG4"), NOPIN("PG5"), NOPIN("PG6"), NOPIN("PG7"),
	NOPIN("PG8"), NOPIN("PG9"), NOPIN("PG10"), NOPIN("PG11"), NOPIN("PG12"), NOPIN("PG13"), NONE, NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,

	NOPIN("PH0"), NOPIN("PH1"), NOPIN("PH2"), NOPIN("PH3"), NOPIN("PH4"), PIN("PH5 (Pi-29)"), PIN("PH6 (Pi-31)"), PIN("PH7 (Pi-26)"),
	PIN("PH8 (E-10)"), PIN("PH9 (Pi-13)"), PIN("PH10"), PIN("PH11"), NONE, NONE, NONE, NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
	NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE
}; */
// Number of PINs: 36
// Number of NOPINs: 67 }}}
struct MCPin {
	int port;
	int pin;
	char const *name;
};

#ifdef PINE64
static MCPin mc_pins[] = { // {{{
	{ H, 8, "\x07" "E-10 (H8)" },
	{ B, 3, "\x07" "E-11 (B3)" },
	{ B, 4, "\x07" "E-12 (B4)" },
	{ B, 5, "\x07" "E-13 (B5)" },
	{ B, 6, "\x07" "E-15 (B6)" },
	{ D, 2, "\x07" "E-19 (D2)" },
	{ D, 3, "\x07" "E-21 (D3)" },
	{ D, 5, "\x07" "E-22 (D5)" },
	{ D, 1, "\x07" "E-23 (D1)" },
	{ D, 0, "\x07" "E-24 (D0)" },
	{ D, 6, "\x07" "E-26 (D6)" },
	{ B, 2, "\x07" "E-27 (B2)" },
	{ D, 7, "\x07" "E-28 (D7)" },
	{ B, 8, "\x07" "E-29 (B8)" },
	{ B, 9, "\x07" "E-30 (B9)" },
	{ C, 7, "\x07" "Pi-11 (C7)" },
	{ C, 8, "\x07" "Pi-12 (C8)" },
	{ H, 9, "\x07" "Pi-13 (H9)" },
	{ C, 12, "\x07" "Pi-15 (C12)" },
	{ C, 13, "\x07" "Pi-16 (C13)" },
	{ C, 14, "\x07" "Pi-18 (C14)" },
	{ C, 0, "\x07" "Pi-19 (C0)" },
	{ C, 1, "\x07" "Pi-21 (C1)" },
	{ C, 15, "\x07" "Pi-22 (C15)" },
	{ C, 2, "\x07" "Pi-23 (C2)" },
	{ C, 3, "\x07" "Pi-24 (C3)" },
	{ H, 7, "\x07" "Pi-26 (H7)" },
	{ H, 5, "\x07" "Pi-29 (H5)" },
	{ H, 6, "\x07" "Pi-31 (H6)" },
	{ C, 4, "\x07" "Pi-32 (C4)" },
	{ C, 5, "\x07" "Pi-33 (C5)" },
	{ C, 9, "\x07" "Pi-35 (C9)" },
	{ C, 6, "\x07" "Pi-36 (C6)" },
	{ C, 16, "\x07" "Pi-37 (C16)" },
	{ C, 10, "\x07" "Pi-38 (C10)" },
	{ C, 11, "\x07" "Pi-40 (C11)" },
}; // }}}
#else
#ifdef ORANGEPIZERO
static MCPin mc_pins[] = { // {{{
	{ A, 6, "\x07" "7 (A6)" },
	{ G, 6, "\x07" "8 (G6)" },
	{ G, 7, "\x07" "10 (G7)" },
	{ A, 1, "\x07" "11 (A1)" },
	{ A, 7, "\x07" "12 (A7)" },
	{ A, 0, "\x07" "13 (A0)" },
	{ A, 3, "\x07" "15 (A3)" },
	{ A, 19, "\x07" "16 (A19)" },
	{ A, 18, "\x07" "18 (A18)" },
	{ A, 15, "\x07" "19 (A15)" },
	{ A, 16, "\x07" "21 (A16)" },
	{ A, 2, "\x07" "22 (A2)" },
	{ A, 14, "\x07" "23 (A14)" },
	{ A, 13, "\x07" "24 (A13)" },
	{ A, 10, "\x07" "26 (A10)" },
}; // }}}
#else
#error "Unknown board"
#endif
#endif

void SET_OUTPUT(Pin_t _pin) { // {{{
	if (_pin.valid()) {
		debug("set output %d", _pin.pin);
		mc_pwm_target[_pin.pin] = 0;
		if (mc_shared->mode[_pin.pin] < 2)
			return;
		gpio_setdir(mc_pins[_pin.pin].port, mc_pins[_pin.pin].pin, GPIO_OUTPUT);
		gpio_setpull(mc_pins[_pin.pin].port, mc_pins[_pin.pin].pin, NOPULL);
		gpio_setpin(mc_pins[_pin.pin].port, mc_pins[_pin.pin].pin, _pin.inverted() ? true : false);
		mc_shared->mode[_pin.pin] = _pin.inverted() ? 1 : 0;
	}
} // }}}

void SET_INPUT(Pin_t _pin) { // {{{
	if (_pin.valid()) {
		//debug("set input %d", _pin.pin);
		if (mc_shared->mode[_pin.pin] == 2)
			return;
		gpio_setpull(mc_pins[_pin.pin].port, mc_pins[_pin.pin].pin, PULLUP);
		gpio_setdir(mc_pins[_pin.pin].port, mc_pins[_pin.pin].pin, GPIO_INPUT);
		// Set state to wrong value, so it will send an update event.
		mc_pin_state[_pin.pin] = !gpio_getpin(mc_pins[_pin.pin].port, mc_pins[_pin.pin].pin);
		mc_shared->mode[_pin.pin] = 2;
	}
} // }}}

void SET_INPUT_NOPULLUP(Pin_t _pin) { // {{{
	if (_pin.valid()) {
		//debug("set nopull %d", _pin.pin);
		if (mc_shared->mode[_pin.pin] == 3)
			return;
		gpio_setpull(mc_pins[_pin.pin].port, mc_pins[_pin.pin].pin, NOPULL);
		gpio_setdir(mc_pins[_pin.pin].port, mc_pins[_pin.pin].pin, GPIO_INPUT);
		// Set state to wrong value, so it will send an update event.
		mc_pin_state[_pin.pin] = !gpio_getpin(mc_pins[_pin.pin].port, mc_pins[_pin.pin].pin);
		mc_shared->mode[_pin.pin] = 3;
	}
} // }}}

#define RAWSET(_p) do { gpio_setpin(mc_pins[_p].port, mc_pins[_p].pin, true); mc_shared->mode[_p] = 1; } while (0)
#define RAWRESET(_p) do { gpio_setpin(mc_pins[_p].port, mc_pins[_p].pin, false); mc_shared->mode[_p] = 0; } while (0)
#define RAWGET(_p) (gpio_getpin(mc_pins[_p].port, mc_pins[_p].pin))
void SET(Pin_t _pin) { // {{{
	SET_OUTPUT(_pin);
	if (_pin.valid()) {
		if (_pin.inverted())
			RAWRESET(_pin.pin);
		else
			RAWSET(_pin.pin);
	}
} // }}}

void RESET(Pin_t _pin) { // {{{
	SET_OUTPUT(_pin);
	if (_pin.valid()) {
		if (_pin.inverted())
			RAWSET(_pin.pin);
		else
			RAWRESET(_pin.pin);
	}
} // }}}

void GET(Pin_t _pin, bool _default, void(*cb)(bool)) { // {{{
	if (_pin.valid() && _pin.pin < NUM_GPIO_PINS) {
		if (RAWGET(_pin.pin))
			cb(!_pin.inverted());
		else
			cb(_pin.inverted());
	}
	else
		cb(_default);
} // }}}
// }}}

// Setup helpers. {{{
static void mc_set_pins(uint64_t mask) { // {{{
	for (int i = 0; i < NUM_GPIO_PINS; ++i) {
		if (mc_shared->mode[i] >= 2) {
			// Refuse to change a pin that isn't set to output.
			continue;
		}
		uint8_t target = (mask >> i) & 1;
		if (target == mc_shared->mode[i]) {
			// Skip pins that are already in the correct state.
			continue;
		}
		gpio_setpin(mc_pins[i].port, mc_pins[i].pin, target);
		mc_shared->mode[i] = target;
	}
} // }}}

// This function is called in a separate thread, and moved to an isolated cpu core.  It handles the realtime operations.
static void mc_realtime() { // {{{
	uint64_t period = (hwtime_step * 1000) / 4;
	int fd = timerfd_create(CLOCK_MONOTONIC, 0);
	struct itimerspec it;
	time_t sec = period / 1000000000;
	long nsec = period - sec * 1000000000;
	it.it_value.tv_sec = sec;
	it.it_value.tv_nsec = nsec;
	it.it_interval.tv_sec = sec;
	it.it_interval.tv_nsec = nsec;
	timerfd_settime(fd, 0, &it, NULL);
	uint64_t num_exp;
	bool ignore_expiry = false;
	int home_delay = 0;
	while (true) {
		while (read(fd, &num_exp, 8) != 8) {}
		if (num_exp != 1 && !ignore_expiry) {
			debug("clock expired: %ld", (long)num_exp);
			ignore_expiry = true;
		}
		else
			ignore_expiry = false;
		if (home_delay > 0) {
			home_delay -= 1;
			continue;
		}
		// state = 0: waiting for limit check; main can set to 1, 2, or 3.
		// state = 1: not running; main can set to 0.
		// state = 2: Doing single step; rt can set to 0 or 1; main can set to 4 (and expect rt to set it to 0 or 1).
		// state = 3: Free running; main can set to 4.
		// state = 4: main requested stop; rt must set to 1.
		if (mc_shared->state == 4) {
			//debug("Request to stop.");
			mc_shared->state = 1;
			continue;
		}
		if (mc_shared->state < 2)
			continue;
		// Do a step.
		//debug("Step base %lx dirs %lx buffers %lx %lx sample %x", mc_shared->base, mc_shared->dirs, mc_shared->buffer[mc_shared->current_fragment][mc_shared->current_sample][0], mc_shared->buffer[mc_shared->current_fragment][mc_shared->current_sample][1], mc_shared->current_sample);
		mc_set_pins(mc_shared->base);
		volatile uint64_t *data;
		if (mc_shared->num_homers > 0) {
			home_delay = 1000;
			data = mc_shared->homers;
		}
		else
			data = mc_shared->buffer[mc_shared->current_fragment][mc_shared->current_sample];
		mc_set_pins(mc_shared->base | data[0]);
		while (read(fd, &num_exp, 8) != 8) {}
		mc_set_pins(mc_shared->base);
		mc_set_pins(mc_shared->base | mc_shared->dirs);
		while (read(fd, &num_exp, 8) != 8) {}
		mc_set_pins(mc_shared->base | mc_shared->dirs | data[1]);
		while (read(fd, &num_exp, 8) != 8) {}
		mc_set_pins(mc_shared->base | mc_shared->dirs);
		mc_set_pins(mc_shared->base);
		// Increment pointer position.
		mc_shared->current_sample += 1;
		if (mc_shared->current_sample == 0) {
			//mc_shared->current_sample = 0;
			mc_shared->current_fragment = (mc_shared->current_fragment + 1) % FRAGMENTS_PER_BUFFER;
			if (mc_shared->current_fragment == mc_shared->next_fragment) {
				mc_shared->state = 1;
				continue;
			}
		}
		// Update state.
		if (mc_shared->state == 2)
			mc_shared->state = 0;
	}
} // }}}

void arch_setup_start() { // {{{
	// Override hwtime_step.
	hwtime_step = 800;
	// Claim that firmware has correct version.
	protocol_version = PROTOCOL_VERSION;
	// Prepare gpios.
	int fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "unable to open /dev/mem: %s\n", strerror(errno));
		abort();
	}
	volatile uint32_t *gpiomem;
	gpiomem = reinterpret_cast <volatile uint32_t *>(mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x01C20000));
	if (gpiomem == MAP_FAILED) {
		fprintf(stderr, "mmap on /dev/mem failed: %s\n", strerror(errno));
		abort();
	}
	mc_piomem = gpiomem + (0x800 >> 2);
	// Prepare i2c handle for analog input.
	adc_fd = open("/dev/i2c-0", O_RDWR);
	if (adc_fd < 0) {
		fprintf(stderr, "unable to open /dev/i2c-0: %s\n", strerror(errno));
		abort();
	}
	if (ioctl(adc_fd, I2C_SLAVE, 0x4d) < 0) {
		fprintf(stderr, "unable to claim i2c device 0x4d: %s\n", strerror(errno));
		abort();
	}
	// Send pin names.
	for (int i = 0; i < NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS; ++i)
		arch_send_pin_name(i);
	clock_gettime(CLOCK_MONOTONIC, &mc_pwm_time);
	// Allocate shared memory.
	mc_shared = reinterpret_cast <MCShared *> (mmap(NULL, sizeof(MCShared), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));
	mc_shared->base = 0;
	mc_shared->dirs = 0;
	mc_shared->current_sample = 0;
	mc_shared->current_fragment = 0;
	mc_shared->next_fragment = 0;
	mc_shared->state = 1;
	mc_shared->num_homers = 0;
	for (int i = 0; i < NUM_DIGITAL_PINS; ++i)
		mc_shared->mode[i] = 4;
	for (int port = 0; port < NUM_PORTS; ++port) {
		for (int unit = 0; unit < 4; ++unit)
			mc_shared->config[port][unit] = mc_piomem[(0x24 * port + 4 * unit) >> 2];
		mc_shared->data[port] = mc_piomem[(0x10 + 0x24 * port) >> 2];
		for (int unit = 0; unit < 2; ++unit)
			mc_shared->pull[port][unit] = mc_piomem[(0x1c + 0x24 * port + 4 * unit) >> 2];
	}
	pid_t pid = fork();
	if (pid == -1) {
		debug("Fork failed; cannot continue: %s", strerror(errno));
		abort();
	}
	if (pid == 0) {
		// Child.
		// Move process to second processor core.
		// This core should have been isolated using isolcpus=1 on the kernel commandline.
		cpu_set_t *mask = CPU_ALLOC(2);
		size_t size = CPU_ALLOC_SIZE(2);
		CPU_ZERO_S(size, mask);
		CPU_SET(1, mask);
		sched_setaffinity(0, size, mask);
		CPU_FREE(mask);
		// Run the realtime handling function.  This does not return.
		mc_realtime();
	}
	connected = true;
} // }}}

void arch_setup_end() { // {{{
	connect_end();
} // }}}

static void mc_next_adc() { // {{{
#if NUM_ANALOG_INPUTS > 0
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i) {
		int n = (mc_active_temp + 1 + i) % NUM_ANALOG_INPUTS;
		if (mc_temp[n].active) {
			mc_active_temp = n;
			return;
		}
	}
#endif
	mc_active_temp = -1;
} // }}}

void arch_request_temp(int which) { // {{{
	if (which >= 0 && which < num_temps && temps[which].thermistor_pin.pin >= NUM_DIGITAL_PINS && temps[which].thermistor_pin.pin < NUM_PINS) {
		requested_temp = which;
		return;
	}
	shmem->floats[0] = NAN;
	delayed_reply();
	requested_temp = ~0;
} // }}}

void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin, bool heater_invert, int heater_adctemp, int heater_limit_l, int heater_limit_h, int fan_pin, bool fan_invert, int fan_adctemp, int fan_limit_l, int fan_limit_h, double hold_time) { // {{{
	if (thermistor_pin < NUM_DIGITAL_PINS || thermistor_pin >= NUM_PINS) {
		debug("setup for invalid adc %d requested", thermistor_pin);
		return;
	}
	thermistor_pin -= NUM_DIGITAL_PINS;
	mc_temp[thermistor_pin].active = active;
	mc_temp[thermistor_pin].id = id;
	mc_temp[thermistor_pin].heater_pin = heater_pin;
	mc_temp[thermistor_pin].heater_inverted = heater_invert;
	mc_temp[thermistor_pin].heater_adctemp = heater_adctemp;
	mc_temp[thermistor_pin].heater_limit_l = heater_limit_l;
	mc_temp[thermistor_pin].heater_limit_h = heater_limit_h;
	mc_temp[thermistor_pin].fan_pin = fan_pin;
	mc_temp[thermistor_pin].fan_inverted = fan_invert;
	mc_temp[thermistor_pin].fan_adctemp = fan_adctemp;
	mc_temp[thermistor_pin].fan_limit_l = fan_limit_l;
	mc_temp[thermistor_pin].fan_limit_h = fan_limit_h;
	mc_temp[thermistor_pin].hold_time = hold_time;
	if (mc_active_temp < 0)
		mc_next_adc();
	// TODO: use hold_time.
} // }}}

// Pin bit capabilities:
// Bit 0: step+dir
// Bit 1: other output
// Bit 2: digital input
// Bit 3: analog input
void arch_send_pin_name(int pin) { // {{{
	char const *name;
	if (pin < NUM_DIGITAL_PINS)
		name = mc_pins[pin].name;
	else
		name = "\x08" "i²c-0:0x4d";
	int len = strlen(name);
	prepare_interrupt();
	strcpy(shmem->interrupt_str, name, avr_pin_name_len[pin]);
	shmem->interrupt_ints[0] = pin;
	shmem->interrupt_ints[1] = len;
	send_to_parent(CMD_PINNAME);
} // }}}
// }}}

// Runtime helpers. {{{

// realtime core has per motor:
// buffer with steps: writable by main until active, then readable by rt until discarded.
// current fragment in buffer: writable by rt, readable by main.
// current sample in buffer: writable by rt, readable by main.
// state:
//
// state = 0: waiting for limit check; main can set to 1, 2, or 3.
// state = 1: not running; main can set to 0.
// state = 2: Doing single step; rt can set to 0 or 1; main can set to 4 (and expect rt to set it to 0 or 1).
// state = 3: Free running; main can set to 4.
// state = 4: main requested stop; rt must set to 1.
int arch_tick() { // {{{
	// This is called when the timeout expires.
	//debug("running fragment %d", running_fragment);
	// Fill buffer for realtime thread.
	int cf = mc_shared->current_fragment;
	if (cf != running_fragment) {
		//debug("cf=%d, runn=%d", cf, running_fragment);
		int cbs = 0;
		while (cf != running_fragment) {
			cbs += history[running_fragment].cbs;
			history[running_fragment].cbs = 0;
			running_fragment = (running_fragment + 1) % FRAGMENTS_PER_BUFFER;
		}
		if (cbs)
			num_movecbs += cbs;
		buffer_refill();
		run_file_fill_queue();
		if (!computing_move && run_file_finishing) {
			abort_run_file();
			num_file_done_events += 1;
		}
	}
	// Handle temps and check temp limits.
	if (mc_active_temp >= 0) {
		// New temperature ready to read.
		int a = mc_active_temp;
		mc_next_adc();
		if (a >= 0 && mc_temp[a].active) {
			char result[2];
			int t;
			if (read(adc_fd, result, 2) != 2) {
				debug("error reading adc: %s", strerror(errno));
				t = 0xffff;
			}
			else
				t = ((result[0] & 0xff) << 8) | (result[1] & 0xff);
			if (mc_temp[a].heater_pin >= 0) {
				if ((mc_temp[a].heater_adctemp < t) ^ mc_temp[a].heater_inverted)
					RAWSET(mc_temp[a].heater_pin);
				else
					RAWRESET(mc_temp[a].heater_pin);
			}
			if (mc_temp[a].fan_pin >= 0) {
				if ((mc_temp[a].fan_adctemp < t) ^ mc_temp[a].fan_inverted)
					RAWSET(mc_temp[a].fan_pin);
				else
					RAWRESET(mc_temp[a].fan_pin);
			}
			handle_temp(mc_temp[a].id, t);
		}
	}
	else
		mc_next_adc();
	// Pwm.
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	double diff = now.tv_sec - mc_pwm_time.tv_sec + (now.tv_nsec - mc_pwm_time.tv_nsec) / 1e9;
	for (int pin = 0; pin < NUM_DIGITAL_PINS; ++pin) {
		if (mc_shared->mode[pin] != 1 || mc_duty[pin] == 1)
			continue;
		if (mc_pwm_on[pin])
			mc_pwm_target[pin] -= diff;
		mc_pwm_target[pin] += diff * mc_duty[pin];
		if (mc_pwm_target[pin] < 0) {
			// Disable pin.
			gpio_setpin(mc_pins[pin].port, mc_pins[pin].pin, false);
			mc_pwm_on[pin] = false;
		}
		else {
			// Enable pin.
			gpio_setpin(mc_pins[pin].port, mc_pins[pin].pin, true);
			mc_pwm_on[pin] = true;
		}
		//debug("pwm pin %d target %f on %d", pin, mc_pwm_target[pin], mc_pwm_on[pin]);
	}
	mc_pwm_time.tv_sec = now.tv_sec;
	mc_pwm_time.tv_nsec = now.tv_nsec;
	// Check limit switches.
	int state = mc_shared->state;
	//debug("state: %d %d %d", state, mc_shared->current_fragment, mc_shared->current_sample);
	if (state != 2 && state != 1) {
		// Check probe.
		if (settings.probing && probe_pin.valid()) {
			if (RAWGET(probe_pin.pin) ^ probe_pin.inverted()) {
				// Probe hit.
				abort_move(0);
				sending_fragment = 0;
				stopping = 2;
				prepare_interrupt();
				shmem->interrupt_ints[0] = -1;
				shmem->interrupt_ints[1] = -1;
				shmem->interrupt_float = NAN;
				send_to_parent(CMD_LIMIT);
				//debug("cbs after current cleared %d for probe", cbs_after_current_move);
				cbs_after_current_move = 0;
			}
		}
		// Avoid race condition by reading cf twice (first was done at start of function).
		int cs;
		while (true) {
			cs = mc_shared->current_sample;
			int cf2 = mc_shared->current_fragment;
			if (cf == cf2)
				break;
			cf = cf2;
		}
		bool homing = false;
		for (int s = 0; s < NUM_SPACES; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m) {
				if (!spaces[s].motor[m]->step_pin.valid() || (!spaces[s].motor[m]->limit_max_pin.valid() && !spaces[s].motor[m]->limit_min_pin.valid())) {
					debug("no limit check for motor %d %d, info %d %d %d", s, m, spaces[s].motor[m]->step_pin.valid(), spaces[s].motor[m]->limit_max_pin.valid(), spaces[s].motor[m]->limit_min_pin.valid());
					continue;
				}
				int pin = spaces[s].motor[m]->step_pin.pin;
				if (mc_shared->num_homers == 0) {
					bool negative;
					if (mc_shared->buffer[cf][cs][0] & (1 << pin))
						negative = false;
					else if (mc_shared->buffer[cf][cs][1] & (1 << pin))
						negative = true;
					else
						continue;
					negative ^= spaces[s].motor[m]->dir_pin.inverted();
					Pin_t *p = negative ? &spaces[s].motor[m]->limit_max_pin : &spaces[s].motor[m]->limit_min_pin;
					if (!p->valid()) {
						debug("no limit check for motor %d %d, pin %d", s, m, p->pin);
						continue;
					}
					debug("limit check for motor %d %d, pin %d", s, m, p->pin);
					if (RAWGET(p->pin) ^ p->inverted()) {
						debug("limit hit.");
						// Limit hit.
						abort_move(0);
						sending_fragment = 0;
						stopping = 2;
						prepare_interrupt();
						shmem->interrupt_ints[0] = s;
						shmem->interrupt_ints[1] = m;
						shmem->interrupt_float = spaces[s].motor[m]->settings.current_pos;
						send_to_parent(CMD_LIMIT);
						//debug("cbs after current cleared %d after sending limit", cbs_after_current_move);
						cbs_after_current_move = 0;
					}
				}
				// Check switches during homing.
				if (spaces[s].motor[m]->mc_homer != 0) {
					homing = true;
					// Check "wrong" limit switch.
					bool negative = spaces[s].motor[m]->mc_homer < 0;
					negative ^= spaces[s].motor[m]->dir_pin.inverted();
					Pin_t *p = negative ? &spaces[s].motor[m]->limit_max_pin : &spaces[s].motor[m]->limit_min_pin;
					if (!p->valid() || !spaces[s].motor[m]->step_pin.valid())
						continue;
					if (!(RAWGET(p->pin) ^ p->inverted())) {
						debug("home limit lost");
						// Limit no longer hit.
						spaces[s].motor[m]->mc_homer = 0;
						mc_shared->homers[negative] &= ~(1 << spaces[s].motor[m]->step_pin.pin);
						mc_shared->num_homers -= 1;
						if (mc_shared->num_homers == 0) {
							// Done homing.
							mc_shared->state = 1;
							homing = false;
							prepare_interrupt();
							send_to_parent(CMD_HOMED);
						}
					}
				}
			}
		}
		if (state == 0) {
			state = (settings.probing || homing) ? 2 : 3;
			mc_shared->state = state;
		}
	}
	// Pin state monitoring.
	for (int i = 0; i < NUM_GPIO_PINS; ++i) {
		if (mc_shared->mode[i] < 2 || mc_shared->mode[i] > 3)
			continue;
		bool pin_state = RAWGET(i);
		if (mc_pin_state[i] == pin_state)
			continue;
		debug("interrupt on pin %d", i);
		for (int g = 0; g < num_gpios; ++g) {
			if (gpios[g].pin.valid() && gpios[g].pin.pin == i) {
				prepare_interrupt();
				shmem->interrupt_ints[0] = g;
				shmem->interrupt_ints[1] = pin_state ^ gpios[g].pin.inverted();
				send_to_parent(CMD_PINCHANGE, g, pin_state ^ gpios[g].pin.inverted());
			}
		}
		mc_pin_state[i] = pin_state;
	}
	// TODO: LED.
	// TODO: Timeout.
	return 10;
} // }}}

void arch_motors_change() { // {{{
	// Configure hardware for updated settings.
	// number of active motors.
	// hwtime_step
	// led pin
	// probe pin
	// timeout
	// motor pins
	mc_shared->base = 0;
	mc_shared->dirs = 0;
	for (int s = 0; s < NUM_SPACES; ++s) {
		for (int m = 0; m < spaces[s].num_motors; ++m) {
			Pin_t *p = &spaces[s].motor[m]->dir_pin;
			if (p->valid()) {
				if (p->inverted())
					mc_shared->base |= 1 << p->pin;
				mc_shared->dirs |= 1 << p->pin;
			}
			p = &spaces[s].motor[m]->step_pin;
			if (p->valid() && p->inverted()) {
				mc_shared->base |= 1 << p->pin;
			}
		}
	}
} // }}}

void arch_addpos(int s, int m, double diff) { // {{{
	(void)&s;
	(void)&m;
	(void)&diff;
	// Nothing to do.
} // }}}

void arch_invertpos(int s, int m) { // {{{
	(void)&s;
	(void)&m;
	// Nothing to do.
} // }}}

void arch_stop(bool fake) { // {{{
	(void)&fake;
	// Stop moving, update current_pos.
	int state = mc_shared->state;
	switch (state) {
	case 1:
		return;
	case 0:
		break;
	case 2:
	case 3:
		mc_shared->state = 4;
		// Wait for realtime thread to ack.
		while (mc_shared->state == 4) {}
		break;
	}
	mc_shared->state = 1;
	if (mc_shared->num_homers > 0) {
		for (int s = 0; s < NUM_SPACES; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m)
				spaces[s].motor[m]->mc_homer = 0;
		}
		mc_shared->num_homers = 0;
		mc_shared->homers[0] = 0;
		mc_shared->homers[1] = 0;
	}
	// Update current_pos.
	abort_move(mc_shared->current_sample);
	current_fragment_pos = 0;
} // }}}

void arch_home() { // {{{
	// Start homing.
	int mi = 0;
	for (int s = 0; s < NUM_SPACES; ++s) {
		for (int m = 0; m < spaces[s].num_motors; ++m) {
			switch (command[0][3 + mi + m]) {
			case 0x00:
				continue;
			case 0x01:
				mc_shared->num_homers += 1;
				spaces[s].motor[m]->mc_homer = 1;
				mc_shared->homers[1] |= 1 << spaces[s].motor[m]->step_pin.pin;
				continue;
			case 0xff:
				mc_shared->num_homers += 1;
				spaces[s].motor[m]->mc_homer = -1;
				mc_shared->homers[0] |= 1 << spaces[s].motor[m]->step_pin.pin;
				continue;
			default:
				debug("Invalid home state: %d for motor %d %d", command[0][3 + mi + m], s, m);
				abort();
			}
		}
		mi += spaces[s].num_motors;
	}
	if (mc_shared->num_homers == 0) {
		debug("no homers");
		prepare_interrupt();
		send_to_parent(CMD_HOMED);
	}
} // }}}

bool arch_running() { // {{{
	// True if an underrun will follow (at some point).
	return mc_shared->state != 1;
} // }}}

void arch_start_move(int extra) { // {{{
	(void)&extra;
	// Start moving with sent buffers.
	int state = mc_shared->state;
	if (state != 1) {
		//debug("info: arch_start_move called with non-1 state %d", state);
		return;
	}
	mc_shared->state = 0;
} // }}}

bool arch_send_fragment() { // {{{
	if (stopping)
		return false;
	mc_shared->next_fragment = (mc_shared->next_fragment + 1) % FRAGMENTS_PER_BUFFER;
	for (int i = 0; i < SAMPLES_PER_FRAGMENT; ++i) {
		mc_shared->buffer[mc_shared->next_fragment][i][0] = 0;
		mc_shared->buffer[mc_shared->next_fragment][i][1] = 0;
	}
	return true;
} // }}}

int arch_fds() { // {{{
	return 0;
} // }}}

double arch_get_duty(Pin_t _pin) { // {{{
	if (_pin.pin < 0 || _pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_get_duty: %d (max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return 1;
	}
	return mc_duty[_pin.pin];
} // }}}

void arch_set_duty(Pin_t _pin, double duty) { // {{{
	if (_pin.pin < 0 || _pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_set_duty: %d (max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return;
	}
	mc_duty[_pin.pin] = duty;
} // }}}

void arch_discard() { // {{{
	int fragments = (current_fragment - mc_shared->current_fragment) % FRAGMENTS_PER_BUFFER;
	if (fragments <= 2)
		return;
	current_fragment = (current_fragment - (fragments - 2)) % FRAGMENTS_PER_BUFFER;
	//debug("current_fragment = (current_fragment - (fragments - 2)) % FRAGMENTS_PER_BUFFER; %d", current_fragment);
	mc_shared->next_fragment = current_fragment;
	restore_settings();
} // }}}

void arch_send_spi(int bits, const uint8_t *data) { // {{{
	debug("send spi request ignored");
	(void)&bits;
	(void)&data;
} // }}}

static void mc_set_shared(int which, int s, int m) { // {{{
	int pin = spaces[s].motor[m]->step_pin.pin;
	if (spaces[s].motor[m]->step_pin.valid() && pin >= 0) {
		mc_shared->buffer[current_fragment][current_fragment_pos][which] |= 1 << pin;
	}
} // }}}

void DATA_SET(int s, int m, int value) { // {{{
	if (value) {
		if (value < -1 || value > 1) {
			debug("invalid sample %d for %d %d", value, s, m);
			//abort();
		}
		// The invert flag of the dir pin is used even if the pin is invalid (which it should never be).
		if ((value < 0) ^ spaces[s].motor[m]->dir_pin.inverted())
			mc_set_shared(0, s, m);
		else
			mc_set_shared(1, s, m);
	}
} // }}}

double arch_round_pos(int space, int motor, double src) { // {{{
	(void)&space;
	(void)&motor;
	return round(src);
} // }}}
// }}}
#endif

#endif

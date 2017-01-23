/* arch-bbb.cpp - bbb specific parts for Franklin {{{
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
#include <math.h>
#include <sys/mman.h>
#ifndef FAKE
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#endif
// }}}

// Defines. {{{
#define PRU 0
#define PRU_DATARAM PRUSS0_PRU0_DATARAM
#if !defined(PRU) || (PRU != 0 && PRU != 1)
#error PRU must be defined as 0 or 1.
#endif
#define NUM_ANALOG_INPUTS 7
#define NUM_GPIO_PINS (4 * 32)
#define NUM_DIGITAL_PINS (NUM_GPIO_PINS + 16)
#define NUM_PINS (NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS)
#define ADCBITS 12
#define FRAGMENTS_PER_BUFFER 8
#define SAMPLES_PER_FRAGMENT 256
#define BBB_PRU_FRAGMENT_MASK (FRAGMENTS_PER_BUFFER - 1)

#define ARCH_MOTOR int bbb_id;
#define ARCH_SPACE int bbb_id, bbb_m0;

#define DATA_CLEAR(s, m) do {} while (0)
#define ARCH_NEW_MOTOR(s, m, base) do {} while (0)
#define DATA_DELETE(s, m) do {} while (0)

#define ARCH_MAX_FDS NUM_GPIO_PINS	// Maximum number of fds for arch-specific purposes.
// }}}

#else

struct bbb_Gpio { // {{{
	uint32_t revision;		// 0
	uint32_t reserved0[3];
	uint32_t sysconfig;		// 10
	uint32_t reserved1[3];
	uint32_t eoi;			// 20
	uint32_t irqstatus_raw_0;	// 24
	uint32_t irqstatus_raw_1;	// 28
	uint32_t irqstatus_0;		// 2c
	uint32_t irqstatus_1;		// 30
	uint32_t irqstatus_set_0;	// 34
	uint32_t irqstatus_set_1;	// 38
	uint32_t irqstatus_clr_0;	// 3c
	uint32_t irqstatus_clr_1;	// 40
	uint32_t irqwaken_0;		// 44
	uint32_t irqwaken_1;		// 48
	uint32_t reserved2[50];
	uint32_t sysstatus;		// 114
	uint32_t reserved3[6];
	uint32_t ctrl;			// 130
	uint32_t oe;			// 134
	uint32_t datain;		// 138
	uint32_t dataout;		// 13c
	uint32_t leveldetect0;		// 140
	uint32_t leveldetect1;		// 144
	uint32_t risingdetect;		// 148
	uint32_t fallingdetect;		// 14c
	uint32_t debounceenable;	// 150
	uint32_t debouncingtime;	// 154
	uint32_t reserved4[14];
	uint32_t cleardataout;		// 190
	uint32_t setdataout;		// 194
};
// }}}

struct bbb_Temp { // {{{
	int id;
	FILE *file;	// For reading the ADC.
	bool active;
	int heater_pin, fan_pin;
	bool heater_inverted, fan_inverted;
	int heater_adctemp, fan_adctemp;
	int heater_limit_l, heater_limit_h, fan_limit_l, fan_limit_h;
	double hold_time;
}; // }}}

struct bbb_Pru { // {{{
	volatile uint16_t base, dirs;
	// These must be bytes, because read and write must be atomic.
	volatile uint8_t current_sample, current_fragment, next_fragment, state;
	volatile uint16_t buffer[FRAGMENTS_PER_BUFFER][SAMPLES_PER_FRAGMENT][2];
} __attribute__ ((packed)); // }}}

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
void arch_send_spi(int bits, uint8_t *data);
off_t arch_send_audio(uint8_t *data, off_t sample, off_t num_records, int motor);
void DATA_SET(int s, int m, int value);
// }}}

#ifdef DEFINE_VARIABLES
// Variables. {{{
#if PRU == 0
static int bbb_pru_pad[16] = { 110, 111, 112, 113, 114, 115, 116, 117, 90, 91, 92, 93, 94, 95, 44, 45 };
int bbb_pru_mode[16] = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6 };
#else
static int bbb_pru_pad[16] = { 70, 71, 72, 73, 74, 75, 76, 77, 86, 87, 88, 89, 62, 63, 42, 43 };
int bbb_pru_mode[16] = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 5, 5, 5, 5 };
#endif

static char const *bbb_apin_name[8] = {"P9_39", "P9_40", "P9_37", "P9_38", "P9_33", "P9_36", "P9_35", "1.65V"};
#ifndef FAKE
static volatile bbb_Gpio *bbb_gpio[4];
static int bbb_devmem;
#endif
enum BBB_State { MUX_DISABLED, MUX_INPUT, MUX_OUTPUT, MUX_PRU };
static int bbb_active_temp;
static bbb_Temp bbb_temp[NUM_ANALOG_INPUTS];
static int bbb_gpio_state[NUM_GPIO_PINS];
static bbb_Pru *bbb_pru;
#define USABLE(x) (x)
#define HDMI(x) (x)
#define FLASH(x) ""
#define INTERNAL ""
#define UNUSABLE ""
static char const *bbb_muxname[NUM_GPIO_PINS] = { // {{{
	UNUSABLE, UNUSABLE, USABLE("P9_22"), USABLE("P9_21"), USABLE("P9_18"), USABLE("P9_17"), INTERNAL, USABLE("P9_42"),
	HDMI("P8_35"), HDMI("P8_33"), HDMI("P8_31"), HDMI("P8_32"), UNUSABLE, UNUSABLE, USABLE("P9_26"), USABLE("P9_24"),
	UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, USABLE("P9_41"), UNUSABLE, USABLE("P8_19"), USABLE("P8_13"),
	UNUSABLE, UNUSABLE, USABLE("P8_14"), USABLE("P8_17"), UNUSABLE, UNUSABLE, USABLE("P9_11"), USABLE("P9_13"),

	FLASH("P8_25"), FLASH("P8_24"), FLASH("P8_5"), FLASH("P8_6"), FLASH("P8_23"), FLASH("P8_22"), FLASH("P8_3"), FLASH("P8_4"),
	UNUSABLE, UNUSABLE, INTERNAL, INTERNAL, USABLE("P8_12"), USABLE("P8_11"), USABLE("P8_16"), USABLE("P8_15"),
	USABLE("P9_15"), USABLE("P9_23"), USABLE("P9_14"), USABLE("P9_16"), UNUSABLE, INTERNAL, INTERNAL, INTERNAL,
	INTERNAL, UNUSABLE, UNUSABLE, INTERNAL, USABLE("P9_12"), USABLE("P8_26"), FLASH("P8_21"), FLASH("P8_20"),

	UNUSABLE, USABLE("P8_18"), USABLE("P8_07"), USABLE("P8_08"), USABLE("P8_10"), USABLE("P8_09"), HDMI("P8_45"), HDMI("P8_46"),
	HDMI("P8_43"), HDMI("P8_44"), HDMI("P8_41"), HDMI("P8_42"), HDMI("P8_39"), HDMI("P8_40"), HDMI("P8_37"), HDMI("P8_38"),
	HDMI("P8_36"), HDMI("P8_34"), UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, HDMI("P8_27"), HDMI("P8_29"),
	HDMI("P8_28"), HDMI("P8_30"), INTERNAL, INTERNAL, INTERNAL, INTERNAL, INTERNAL, INTERNAL,

	UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE,
	UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, HDMI("P9_31"), HDMI("P9_29"),
	USABLE("P9_30"), HDMI("P9_28"), USABLE("P9_42"), USABLE("P9_27"), USABLE("P9_41"), HDMI("P9_25"), UNUSABLE, UNUSABLE,
	UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE, UNUSABLE
}; // }}}
#ifndef FAKE
static volatile uint32_t *bbb_gpio_pad[4];
int bbb_hack_pipe[2];
#endif
// }}}

// Pin setting. {{{
static void bbb_setmux(Pin_t _pin, BBB_State mode) { // {{{
	int pin = _pin.pin;
	int prupin = 0; // Is only used when initialized, but the compiler doesn't know.
	if (pin >= NUM_GPIO_PINS) {
		prupin = pin - NUM_GPIO_PINS;
		pin = bbb_pru_pad[prupin];
	}
	if (bbb_gpio_state[pin] == mode)
		return;
	if (bbb_muxname[pin][0] == '\0') {
		if (mode != MUX_DISABLED)
			debug("trying to set up unusable gpio %d", pin);
		return;
	}
	int modes[4] = { 0x1f, 0x37, 0x1f, bbb_pru_mode[prupin] };
#ifndef FAKE
	if (write(bbb_hack_pipe[1], &modes[mode], 4) != 4)
		debug("warning: short write to modechange pipe hack");
	if (read(bbb_hack_pipe[0], (char *)bbb_gpio_pad[pin], 4) != 4)
		debug("warning: short read from modechange pipe hack");
#else
	debug("set mode %x %x", pin, modes[mode]);
#endif
}; // }}}

static void set_interrupt(Pin_t _pin, bool enabled) { // {{{
	if (pollfds[BASE_FDS + _pin.pin].fd == -1)
		return;
	if (pollfds[BASE_FDS + _pin.pin].fd < 0) {
		if (!enabled)
			return;
		pollfds[BASE_FDS + _pin.pin].fd += NUM_GPIO_PINS + 1;
	}
	else {
		if (enabled)
			return;
		pollfds[BASE_FDS + _pin.pin].fd -= NUM_GPIO_PINS + 1;
	}
	std::ostringstream s;
	s << "/sys/class/gpio/gpio" << _pin.pin << "/edge";
	std::string filename = s.str();
	std::ofstream f(filename.c_str());
	f << (enabled ? "both" : "none") << std::endl;
	f.close();
	debug("interrupt %d set to %d; %d", _pin.pin, pollfds[BASE_FDS + _pin.pin].fd, enabled);
} // }}}

void SET_OUTPUT(Pin_t _pin) { // {{{
	if (_pin.valid()) {
		if (_pin.pin < NUM_GPIO_PINS) {
			set_interrupt(_pin, false);
#ifdef FAKE
			debug("gpio set output pin %d", _pin.pin);
#else
			bbb_gpio[_pin.pin >> 5]->oe &= ~(1 << (_pin.pin & 0x1f));
#endif
			bbb_setmux(_pin, MUX_OUTPUT);
		}
		else
			bbb_setmux(_pin, MUX_PRU);
	}
} // }}}

void SET_INPUT(Pin_t _pin) { // {{{
	if (_pin.valid()) {
		if (_pin.pin < NUM_GPIO_PINS) {
#ifdef FAKE
			debug("gpio set input pin %d", _pin.pin);
#else
			bbb_gpio[_pin.pin >> 5]->oe |= 1 << (_pin.pin & 0x1f);
#endif
			set_interrupt(_pin, true);
		}
		else
			debug("warning: trying to set pru pin to input");
		bbb_setmux(_pin, MUX_INPUT);
	}
} // }}}

void SET_INPUT_NOPULLUP(Pin_t _pin) { // {{{
	if (_pin.valid()) {
		if (_pin.pin < NUM_GPIO_PINS) {
#ifdef FAKE
			debug("gpio set input pin %d", _pin.pin);
#else
			bbb_gpio[_pin.pin >> 5]->oe |= 1 << (_pin.pin & 0x1f);
#endif
			set_interrupt(_pin, true);
		}
		bbb_setmux(_pin, MUX_DISABLED);
	}
} // }}}

#ifdef FAKE
#define RAWSET(_p) debug("Set pin %d", _p)
#define RAWRESET(_p) debug("Unset pin %d", _p)
#define RAWGET(_p) false
#else
#define RAWSET(_p) bbb_gpio[(_p) >> 5]->setdataout = 1 << ((_p) & 0x1f)
#define RAWRESET(_p) bbb_gpio[(_p) >> 5]->cleardataout = 1 << ((_p) & 0x1f)
#define RAWGET(_p) (bool(bbb_gpio[(_p) >> 5]->datain & (1 << ((_p) & 0x1f))))
#endif
void SET(Pin_t _pin) { // {{{
	SET_OUTPUT(_pin);
	if (_pin.valid() && _pin.pin < NUM_GPIO_PINS) {
		if (_pin.inverted())
			RAWRESET(_pin.pin);
		else
			RAWSET(_pin.pin);
	}
} // }}}

void RESET(Pin_t _pin) { // {{{
	SET_OUTPUT(_pin);
	if (_pin.valid() && _pin.pin < NUM_GPIO_PINS) {
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
void arch_setup_start() { // {{{
#ifndef FAKE
	// Prepare for pinmux hack.
	if (pipe(bbb_hack_pipe)) {
		debug("unable to create pipe for pinmux hack: %s", strerror(errno));
		abort();
	}
	// Prepare system for using ADC and pruss.
#define SLOTS "/sys/devices/platform/bone_capemgr/slots"
	std::ofstream f(SLOTS);
	f << "BB-ADC\n";
	f.close();
	f.open(SLOTS);
	f << "uio-pruss-enable\n";
	f.close();
#endif
	// Set up analog inputs.
#ifdef FAKE
	std::string base("");
#else
	std::string base("/sys/devices/platform/ocp/44e0d000.tscadc/TI-am335x-adc/iio:device0/");
#endif
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i) {
		char num[2] = "0";
		num[0] += i;
		std::string name = base + "in_voltage" + num + "_raw";
		bbb_temp[i].file = fopen(name.c_str(), "r");
		if (!bbb_temp[i].file)
			debug("unable to open analog input %d: %s", i, strerror(errno));
		bbb_temp[i].active = false;
	}
	bbb_active_temp = -1;
	// Prepare gpios.
#ifndef FAKE
	unsigned gpio_base[4] = { 0x44e07000, 0x4804c000, 0x481ac000, 0x481ae000 };
	unsigned pad_control_base = 0x44e10000;
	int pad_offset[32 * 4] = {
		0x148, 0x14c, 0x150, 0x154, 0x158, 0x15c, 0x160, 0x164, 0xd0, 0xd4, 0xd8, 0xdc, 0x178, 0x17c, 0x180, 0x184,
		0x11c, 0x120, 0x21c, 0x1b0, 0x1b4, 0x124, 0x20, 0x24, -1, -1, 0x28, 0x2c, 0x128, 0x144, 0x70, 0x74,

		0x0, 0x4, 0x8, 0xc, 0x10, 0x14, 0x18, 0x1c, 0x168, 0x16c, 0x170, 0x174, 0x30, 0x34, 0x38, 0x3c,
		0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c, 0x60, 0x64, 0x68, 0x6c, 0x78, 0x7c, 0x80, 0x84,

		0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c, 0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc, 0xc0, 0xc4,
		0xc8, 0xcc, 0x134, 0x138, 0x13c, 0x140, 0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc, 0x100, 0x104,

		0x108, 0x10c, 0x110, 0x114, 0x118, 0x188, 0x18c, 0x1e4, 0x1e8, 0x12c, 0x130, -1, -1, 0x234, 0x190, 0x194,
		0x198, 0x19c, 0x1a0, 0x1a4, 0x1a8, 0x1ac, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
	};
	bbb_devmem = open("/dev/mem", O_RDWR);
	for (int i = 0; i < 4; ++i)
		bbb_gpio[i] = (volatile bbb_Gpio *)mmap(0, 0x2000, PROT_READ | PROT_WRITE, MAP_SHARED, bbb_devmem, gpio_base[i]);
	volatile uint32_t *bbb_padmap = (volatile uint32_t *)mmap(0, 0x2000, PROT_READ | PROT_WRITE, MAP_SHARED, bbb_devmem, pad_control_base);
	for (int i = 0; i < 32 * 4; ++i)
		bbb_gpio_pad[i] = pad_offset[i] >= 0 ? &bbb_padmap[(0x800 + pad_offset[i]) / 4] : NULL;
	// Run pru program.
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	debug("pru init %d", prussdrv_init());
	int ret = prussdrv_open(PRU_EVTOUT_0);
	if (ret) {
		debug("unable to open pru system");
		abort();
	}
	debug("init intc %d", prussdrv_pruintc_init(&pruss_intc_initdata));
	debug("pru mmap %d", prussdrv_map_prumem(PRU_DATARAM, (void **)&bbb_pru));
#else
	bbb_pru = new bbb_Pru;
#endif
	bbb_pru->base = 0;
	bbb_pru->dirs = 0;
	bbb_pru->current_fragment = 0;
	//debug("bbb_pru->current_fragment = 0; %d", current_fragment);
	bbb_pru->current_sample = 0;
	bbb_pru->next_fragment = 0;
	bbb_pru->state = 1;
#ifndef FAKE
	debug("pru exec %d", prussdrv_exec_program(PRU, "/usr/lib/franklin/bb/bbb_pru.bin"));
#endif
	// Override hwtime_step.
	hwtime_step = 40;
	// Claim that firmware has correct version.
	protocol_version = PROTOCOL_VERSION;
	for (int i = 0; i < NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS; ++i) {
		arch_send_pin_name(i);
#ifdef FAKE
		pollfds[BASE_FDS + i].fd = -1;
#else
		if (i < NUM_GPIO_PINS) {
			if (bbb_muxname[i][0] == '\0') {
				pollfds[BASE_FDS + i].fd = -1;
				continue;
			}
			std::ofstream e("/sys/class/gpio/export");
			e << i << std::endl;
			e.close();
			std::ostringstream fs;
			fs << "/sys/class/gpio/gpio" << i << "/value";
			std::string filename(fs.str());
			pollfds[BASE_FDS + i].fd = open(filename.c_str(), O_RDONLY | O_NONBLOCK);
			pollfds[BASE_FDS + i].events = POLLPRI;
			if (pollfds[BASE_FDS + i].fd < 0) {
				debug("error opening interrupt file %s: %s", filename.c_str(), strerror(errno));
				abort();
			}
			pollfds[BASE_FDS + i].fd -= NUM_GPIO_PINS + 1;
		}
#endif
	}
} // }}}

void arch_setup_end() { // {{{
	connect_end();
} // }}}

static void bbb_next_adc() { // {{{
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i) {
		int n = (bbb_active_temp + 1 + i) % NUM_ANALOG_INPUTS;
		if (bbb_temp[n].active) {
			bbb_active_temp = n;
			return;
		}
	}
	bbb_active_temp = -1;
} // }}}

void arch_request_temp(int which) { // {{{
	if (which >= 0 && which < num_temps && temps[which].thermistor_pin.pin >= NUM_DIGITAL_PINS && temps[which].thermistor_pin.pin < NUM_PINS) {
		requested_temp = which;
		return;
	}
	requested_temp = ~0;
} // }}}

void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin, bool heater_invert, int heater_adctemp, int heater_limit_l, int heater_limit_h, int fan_pin, bool fan_invert, int fan_adctemp, int fan_limit_l, int fan_limit_h, double hold_time) { // {{{
	if (thermistor_pin < NUM_DIGITAL_PINS || thermistor_pin >= NUM_PINS) {
		debug("setup for invalid adc %d requested", thermistor_pin);
		return;
	}
	thermistor_pin -= NUM_DIGITAL_PINS;
	bbb_temp[thermistor_pin].active = active;
	bbb_temp[thermistor_pin].id = id;
	bbb_temp[thermistor_pin].heater_pin = heater_pin;
	bbb_temp[thermistor_pin].heater_inverted = heater_invert;
	bbb_temp[thermistor_pin].heater_adctemp = heater_adctemp;
	bbb_temp[thermistor_pin].heater_limit_l = heater_limit_l;
	bbb_temp[thermistor_pin].heater_limit_h = heater_limit_h;
	bbb_temp[thermistor_pin].fan_pin = fan_pin;
	bbb_temp[thermistor_pin].fan_inverted = fan_invert;
	bbb_temp[thermistor_pin].fan_adctemp = fan_adctemp;
	bbb_temp[thermistor_pin].fan_limit_l = fan_limit_l;
	bbb_temp[thermistor_pin].fan_limit_h = fan_limit_h;
	bbb_temp[thermistor_pin].hold_time = hold_time;
	if (bbb_active_temp < 0)
		bbb_next_adc();
	// TODO: use hold_time.
} // }}}

// Pin bit capabilities:
// Bit 0: step+dir
// Bit 1: other output
// Bit 2: digital input
// Bit 3: analog input
void arch_send_pin_name(int pin) { // {{{
	int len;
	if (pin < NUM_GPIO_PINS) {
		if (bbb_muxname[pin][0] == '\0')
			len = sprintf(datastore, "%cN/A", 0);
		else
			len = sprintf(datastore, "%c%s (%d/%d-%d)", 6, bbb_muxname[pin], pin, pin / 32, pin % 32);
	}
	else if (pin < NUM_DIGITAL_PINS) {
		char const *s = bbb_muxname[bbb_pru_pad[pin - NUM_GPIO_PINS]];
		if (s[0] == '\0')
			len = sprintf(datastore, "%cN/A", 0);
		else
			len = sprintf(datastore, "%cPRU %d (%s)", 3, pin - NUM_GPIO_PINS, s);
	}
	else {
		len = sprintf(datastore, "%c%s (A%d)", bbb_temp[pin - NUM_DIGITAL_PINS].file ? 8 : 0, bbb_apin_name[pin - NUM_DIGITAL_PINS], pin - NUM_DIGITAL_PINS);
	}
	send_host(CMD_PINNAME, pin, 0, 0, 0, len);
} // }}}
// }}}

// Runtime helpers. {{{

// PRU has per motor:
// buffer with steps: writable by cpu until active, then readable by pru until discarded.
// current fragment in buffer: writable by pru, readable by cpu.
// current sample in buffer: writable by pru, readable by cpu.
// state.
//
// state: 0: waiting for limit check; cpu can set to 1, 2, or 3.
// state: 1: not running; cpu can set to 0.
// state: 2: Doing single step; pru can set to 0; cpu can set to 4 (and expect pru to set it to 0 or 1).
// state: 3: Free running; cpu can set to 4.
// state: 4: cpu requested stop; pru must set to 1.
int arch_tick() { // {{{
	// This is called when the timeout expires, but also when an interrupt is detected on an input pin.
	//debug("running fragment %d", running_fragment);
	// Fill buffer for pru.
	int cf = bbb_pru->current_fragment;
	if (cf != running_fragment) {
		debug("cf=%d, runn=%d", cf, running_fragment);
		int cbs = 0;
		while (cf != running_fragment) {
			cbs += history[running_fragment].cbs;
			history[running_fragment].cbs = 0;
			running_fragment = (running_fragment + 1) % FRAGMENTS_PER_BUFFER;
		}
		if (cbs)
			send_host(CMD_MOVECB, cbs);
		buffer_refill();
		run_file_fill_queue();
		if (!computing_move && run_file_finishing) {
			send_host(CMD_FILE_DONE);
			abort_run_file();
		}
	}
	// Handle temps and check temp limits.
	if (bbb_active_temp >= 0) {
		// New temperature ready to read.
		int a = bbb_active_temp;
		bbb_next_adc();
		char data[6];	// 12 bit adc: maximum 4 digits, plus newline and NUL.
		fseek(bbb_temp[a].file, SEEK_SET, 0);
		int num = fread(data, 1, sizeof(data), bbb_temp[a].file);
		if (num <= 0) {
			if (errno == EAGAIN)
				a = -1;
			else {
				debug("Error reading from adc %d: %s.", a, strerror(errno));
				data[0] = '0';
				data[1] = '\0';
			}
		}
		if (a >= 0 && bbb_temp[a].active) {
			int t = atoi(data);
			if (bbb_temp[a].heater_pin >= 0) {
				if ((bbb_temp[a].heater_adctemp < t) ^ bbb_temp[a].heater_inverted)
					RAWSET(bbb_temp[a].heater_pin);
				else
					RAWRESET(bbb_temp[a].heater_pin);
			}
			if (bbb_temp[a].fan_pin >= 0) {
				if ((bbb_temp[a].fan_adctemp < t) ^ bbb_temp[a].fan_inverted)
					RAWSET(bbb_temp[a].fan_pin);
				else
					RAWRESET(bbb_temp[a].fan_pin);
			}
			handle_temp(bbb_temp[a].id, t);
		}
	}
	else
		bbb_next_adc();
	// TODO: Pwm.
	// Check limit switches.
	int state = bbb_pru->state;
	//debug("pru state: %d %d %d", state, bbb_pru->current_fragment, bbb_pru->current_sample);
	if (state != 2 && state != 1) {
		// Check probe.
		if (settings.probing && probe_pin.valid()) {
			if (RAWGET(probe_pin.pin) ^ probe_pin.inverted()) {
				// Probe hit.
				sending_fragment = 0;
				stopping = 2;
				send_host(CMD_LIMIT, -1, -1, NAN);
				//debug("cbs after current cleared %d for probe", cbs_after_current_move);
				cbs_after_current_move = 0;
			}
		}
		int m0 = 0;
		// Avoid race condition by reading cf twice (first was done at start of function).
		int cs;
		while (true) {
			cs = bbb_pru->current_sample;
			int cf2 = bbb_pru->current_fragment;
			if (cf == cf2)
				break;
			cf = cf2;
		}
		for (int s = 0; s < NUM_SPACES; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m) {
				if (!spaces[s].motor[m]->active || !spaces[s].motor[m]->step_pin.valid() || spaces[s].motor[m]->step_pin.pin < NUM_GPIO_PINS)
					continue;
				int pin = spaces[s].motor[m]->step_pin.pin - NUM_GPIO_PINS;
				bool negative = bool(bbb_pru->buffer[cf][cs][0] & (1 << pin)) ^ spaces[s].motor[m]->dir_pin.inverted();
				Pin_t *p = negative ? &spaces[s].motor[m]->limit_max_pin : &spaces[s].motor[m]->limit_min_pin;
				if (!p->valid())
					continue;
				if (RAWGET(p->pin) ^ p->inverted()) {
					// Limit hit.
					sending_fragment = 0;
					stopping = 2;
					send_host(CMD_LIMIT, s, m, spaces[s].motor[m]->settings.current_pos / spaces[s].motor[m]->steps_per_unit);
					//debug("cbs after current cleared %d after sending limit", cbs_after_current_move);
					cbs_after_current_move = 0;
				}
			}
			m0 += spaces[s].num_motors;
		}
		// TODO: homing.
		if (state == 0) {
			state = settings.probing ? 2 : 3; // TODO: homing.
			bbb_pru->state = state;
		}
	}
	// Pin state monitoring.
	for (int i = 0; i < NUM_GPIO_PINS; ++i) {
		if (pollfds[BASE_FDS + i].revents & POLLPRI) {
			debug("interrupt on pin %d", i);
			lseek(pollfds[BASE_FDS + i].fd, 0, SEEK_SET);
			char buffer[10];
			read(pollfds[BASE_FDS + i].fd, buffer, sizeof(buffer));
			for (int g = 0; g < num_gpios; ++g) {
				if (gpios[g].pin.valid() && gpios[g].pin.pin == i)
					send_host(CMD_PINCHANGE, g, RAWGET(i) ^ gpios[g].pin.inverted());
			}
		}
	}
	// TODO: LED.
	// TODO: Timeout.
	return state == 3 ? 100 : state == 2 ? 10 : 200;
} // }}}

void arch_motors_change() { // {{{
	bbb_pru->base = 0;
	bbb_pru->dirs = 0;
	for (int s = 0; s < NUM_SPACES; ++s) {
		for (int m = 0; m < spaces[s].num_motors; ++m) {
			Pin_t *p = &spaces[s].motor[m]->dir_pin;
			if (p->valid() && p->pin >= NUM_GPIO_PINS) {
				int pin = p->pin - NUM_GPIO_PINS;
				if (p->inverted())
					bbb_pru->base |= 1 << pin;
				bbb_pru->dirs |= 1 << pin;
			}
			p = &spaces[s].motor[m]->step_pin;
			if (p->valid() && p->pin >= NUM_GPIO_PINS && p->inverted()) {
				int pin = p->pin - NUM_GPIO_PINS;
				bbb_pru->base |= 1 << pin;
			}
		}
	}
	// Configure hardware for updated settings.
	// number of active motors.
	// hwtime_step
	// led pin
	// probe pin
	// timeout
	// motor pins
} // }}}

void arch_addpos(int s, int m, double diff) { // {{{
	(void)&s;
	(void)&m;
	(void)&diff;
	// Nothing to do.
} // }}}

void arch_stop(bool fake) { // {{{
	(void)&fake;
	// Stop moving, update current_pos.
	int state = bbb_pru->state;
	switch (state) {
	case 1:
		return;
	case 0:
		break;
	case 2:
	case 3:
		bbb_pru->state = 4;
		// Wait for pru to ack.
#ifndef FAKE
		while (bbb_pru->state == 4) {}
#endif
		break;
	}
	bbb_pru->state = 1;
	// Update current_pos.
	abort_move(bbb_pru->current_sample);
	current_fragment_pos = 0;
} // }}}

void arch_home() { // TODO
	// Start homing.
}

bool arch_running() { // {{{
	// True if an underrun will follow.
	return bbb_pru->state != 1;
} // }}}

void arch_start_move(int extra) { // {{{
	(void)&extra;
	// Start moving with sent buffers.
	int state = bbb_pru->state;
	if (state != 1) {
		//debug("info: arch_start_move called with non-1 state %d", state);
		return;
	}
	bbb_pru->state = 0;
} // }}}

bool arch_send_fragment() { // {{{
	if (stopping)
		return false;
	bbb_pru->next_fragment = (bbb_pru->next_fragment + 1) & BBB_PRU_FRAGMENT_MASK;
	return true;
} // }}}

int arch_fds() { // {{{
	return ARCH_MAX_FDS;
} // }}}

double arch_get_duty(Pin_t _pin) { // TODO
	if (_pin.pin < 0 || _pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_get_duty: %d (max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return 1;
	}
	return 1;
}

void arch_set_duty(Pin_t _pin, double duty) { // TODO
	if (_pin.pin < 0 || _pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_set_duty: %d (max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return;
	}
	(void)&duty;
}

void arch_discard() { // {{{
	int fragments = (current_fragment - bbb_pru->current_fragment) & BBB_PRU_FRAGMENT_MASK;
	if (fragments <= 2)
		return;
	current_fragment = (current_fragment - (fragments - 2)) & BBB_PRU_FRAGMENT_MASK;
	//debug("current_fragment = (current_fragment - (fragments - 2)) & BBB_PRU_FRAGMENT_MASK; %d", current_fragment);
	bbb_pru->next_fragment = current_fragment;
	restore_settings();
} // }}}

void arch_send_spi(int bits, uint8_t *data) { // {{{
	debug("send spi request ignored");
	(void)&bits;
	(void)&data;
} // }}}

off_t arch_send_audio(uint8_t *data, off_t sample, off_t num_records, int motor) { // {{{
	// TODO.
	(void)&data;
	(void)&sample;
	(void)&motor;
	return num_records;
} // }}}

void arch_stop_audio() { // {{{
	// TODO.
} // }}}

static void bbb_set_pru(int which, int s, int m) { // {{{
	int pin = spaces[s].motor[m]->step_pin.pin - NUM_GPIO_PINS;
	if (spaces[s].motor[m]->step_pin.valid() && pin >= 0) {
		bbb_pru->buffer[current_fragment][current_fragment_pos][which] |= 1 << pin;
	}
} // }}}

void DATA_SET(int s, int m, int value) { // {{{
	if (value) {
		if (value < -1 || value > 1) {
			debug("invalid sample %d for %d %d", value, s, m);
			abort();
		}
		// The invert flag of the dir pin is used even if the pin is invalid (which it should never be).
		if ((value < 0) ^ spaces[s].motor[m]->dir_pin.inverted())
			bbb_set_pru(0, s, m);
		else
			bbb_set_pru(1, s, m);
	}
} // }}}

double arch_round_pos(int space, int motor, double src) { // {{{
	(void)&space;
	(void)&motor;
	return src;
} // }}}
// }}}
#endif

#endif

// vim: set foldmethod=marker :
#ifndef _ARCH_BBB_H
#define _ARCH_BBB_H

// Includes. {{{
#include <stdint.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <sys/time.h>
#include <poll.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <sys/mman.h>
// }}}

// Defines. {{{
#define NUM_ANALOG_INPUTS 7
#define NUM_DIGITAL_PINS (4 * 32)
#define ADCBITS 12
#define FRAGMENTS_PER_BUFFER 32
#define BYTES_PER_FRAGMENT 4096
// }}}

struct bbb_Gpio { // {{{
	unsigned revision;		// 0
	unsigned reserved0[3];
	unsigned sysconfig;		// 10
	unsigned reserved1[3];
	unsigned eoi;			// 20
	unsigned irqstatus_raw_0;	// 24
	unsigned irqstatus_raw_1;	// 28
	unsigned irqstatus_0;		// 2c
	unsigned irqstatus_1;		// 30
	unsigned irqstatus_set_0;	// 34
	unsigned irqstatus_set_1;	// 38
	unsigned irqstatus_clr_0;	// 3c
	unsigned irqstatus_clr_1;	// 40
	unsigned irqwaken_0;		// 44
	unsigned irqwaken_1;		// 48
	unsigned reserved2[50];
	unsigned sysstatus;		// 114
	unsigned reserved3[6];
	unsigned ctrl;			// 130
	unsigned oe;			// 134
	unsigned datain;		// 138
	unsigned dataout;		// 13c
	unsigned leveldetect0;		// 140
	unsigned leveldetect1;		// 144
	unsigned risingdetect;		// 148
	unsigned fallingdetect;		// 14c
	unsigned debounceenable;	// 150
	unsigned debouncingtime;	// 154
	unsigned reserved4[14];
	unsigned cleardataout;		// 190
	unsigned setdataout;		// 194
};
// }}}

struct bbb_Temp { // {{{
	int id;
	int fd;	// For reading the ADC.
	bool active;
	int power_pin, fan_pin;
	bool power_inverted, fan_inverted;
	int power_target, fan_target;
}; // }}}

// Function declarations. {{{
void SET_OUTPUT(Pin_t _pin);
void SET_INPUT(Pin_t _pin);
void SET_INPUT_NOPULLUP(Pin_t _pin);
void SET(Pin_t _pin);
void RESET(Pin_t _pin);
void GET(Pin_t _pin, bool _default, void(*cb)(bool));
void arch_setup_start(char const *port);
void arch_setup_end(char const *run_id);
void arch_setup_temp(int which, int thermistor_pin, int active, int power_pin = -1, bool power_inverted = true, int power_target = 0, int fan_pin = -1, bool fan_inverted = false, int fan_target = 0);
void arch_motors_change();
void arch_addpos(int s, int m, int diff);
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
off_t arch_send_audio(uint8_t *data, off_t sample, off_t num_records, int motor);
// }}}

// Variables. {{{
EXTERN volatile bbb_Gpio *bbb_gpio[4];
EXTERN int bbb_devmem;
EXTERN int bbb_active_temp;
EXTERN bbb_Temp bbb_temp[NUM_ANALOG_INPUTS];
// }}}

#ifdef DEFINE_VARIABLES
// Pin setting. {{{
void SET_OUTPUT(Pin_t _pin) {
	if (_pin.valid()) {
		bbb_gpio[_pin.pin >> 5]->oe &= ~(1 << (_pin.pin & 0x1f));
	}
}

void SET_INPUT(Pin_t _pin) {
	if (_pin.valid()) {
		bbb_gpio[_pin.pin >> 5]->oe |= 1 << (_pin.pin & 0x1f);
	}
}

void SET_INPUT_NOPULLUP(Pin_t _pin) {
	if (_pin.valid()) {
		bbb_gpio[_pin.pin >> 5]->oe |= 1 << (_pin.pin & 0x1f);
	}
}

void SET(Pin_t _pin) {
	if (_pin.valid()) {
		if (_pin.inverted())
			bbb_gpio[_pin.pin >> 5]->cleardataout = 1 << (_pin.pin & 0x1f);
		else
			bbb_gpio[_pin.pin >> 5]->setdataout = 1 << (_pin.pin & 0x1f);
	}
}

void RESET(Pin_t _pin) {
	if (_pin.valid()) {
		if (_pin.inverted())
			bbb_gpio[_pin.pin >> 5]->setdataout = 1 << (_pin.pin & 0x1f);
		else
			bbb_gpio[_pin.pin >> 5]->cleardataout = 1 << (_pin.pin & 0x1f);
	}
}

void GET(Pin_t _pin, bool _default, void(*cb)(bool)) {
	if (_pin.valid()) {
		if (bbb_gpio[_pin.pin >> 5]->datain & (1 << (_pin.pin & 0x1f)))
			cb(!_pin.inverted());
		else
			cb(_pin.inverted());
	}
	else
		cb(_default);
}
// }}}

// Setup helpers. {{{ TODO
void arch_setup_start(char const *port) {
	// TODO: find out if this thing varies, implement a search if it does.
	FILE *f = fopen("/sys/devices/bone_capemgr.9/slots", "w");
	fprintf(f, "cape-bone-iio\n");
	fclose(f);
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i) {
		// TODO: find out if this varies, implement a search if so.
		char *filename;
		asprintf(&filename, "/sys/devices/ocp.3/helper.15/AIN%d", i);
		bbb_temp[i].fd = open(filename, O_RDONLY);
		free(filename);
		bbb_temp[i].active = false;
	}
	for (int i = 0; i < NUM_DIGITAL_PINS; ++i) {
		f = std::fopen("/sys/class/gpio/export", "w");
		fprintf(f, "%d\n", i);
		fclose(f);
	}
	bbb_devmem = open("/dev/mem", O_RDWR);
	unsigned gpio_base[4] = { 0x44E07000, 0x4804C000, 0x481AC000, 0x481AE000 };
	for (int i = 0; i < 4; ++i)
		bbb_gpio[i] = (volatile bbb_Gpio *)mmap(0, 0x2000, PROT_READ | PROT_WRITE, MAP_SHARED, bbb_devmem, gpio_base[i]);
	bbb_active_temp = -1;
}

void arch_setup_end(char const *run_id) {
}

static void bbb_next_adc() {
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i) {
		int n = (bbb_active_temp + 1 + i) % NUM_ANALOG_INPUTS;
		if (bbb_temp[n].active) {
			bbb_active_temp = n;
			return;
		}
	}
	bbb_active_temp = -1;
}

void arch_setup_temp(int which, int thermistor_pin, int active, int power_pin, bool power_inverted, int power_target, int fan_pin, bool fan_inverted, int fan_target) {
	bbb_temp[thermistor_pin].active = active;
	bbb_temp[thermistor_pin].id = which;
	bbb_temp[thermistor_pin].power_pin = power_pin;
	bbb_temp[thermistor_pin].power_inverted = power_inverted;
	bbb_temp[thermistor_pin].power_target = power_target;
	bbb_temp[thermistor_pin].fan_pin = fan_pin;
	bbb_temp[thermistor_pin].fan_inverted = fan_inverted;
	bbb_temp[thermistor_pin].fan_target = fan_target;
	if (bbb_active_temp < 0)
		bbb_next_adc();
}
// }}}

// Runtime helpers. {{{

// PRU has per motor:
// buffer with steps: writable by cpu until active, then readable by pru until discarded.
// current fragment in buffer: writable by pru, readable by cpu.
// current sample in buffer: writable by pru, readable by cpu.
// current moving direction (-1/0/1): writable by pru, readable by cpu.
//
// PRU globals:
// state: 0: waiting for limit check; writable by cpu.
// state: 1: not running; writable by cpu.
// state: 2: Doing single step; pru can write to 0, cpu can write to 1. XXX
// state: 3: Free running; cpu can set to 1.
int arch_tick() {
	// Handle temps and check limit switches.
	if (bbb_active_temp >= 0) {
		int a = bbb_active_temp;
		// New temperature ready to read.
		char data[6];	// 12 bit adc: maximum 4 digits, plus newline and NUL.
		int num = read(bbb_temp[a].fd, data, sizeof(data));
		bbb_next_adc();
		if (num <= 0)
			debug("Error reading from adc.");
		else
			handle_temp(bbb_temp[a].id, atoi(data));
	}
	// Limit switches. TODO
	return 10;
}

void arch_motors_change() { // TODO
	// Configure hardware for updated settings.
	// number of active motors.
	// hwtime_step
	// led pin
	// probe pin
	// timeout
	// motor pins
}

void arch_addpos(int s, int m, int diff) { // TODO
	// hwcurrent_pos has been modified; this function must update arch internals to match.
}

void arch_stop(bool fake) { // TODO
	// Stop moving, update current_pos.
}

void arch_home() { // TODO
	// Start homing.
}

bool arch_running() { // TODO
	// True if an underrun will follow.
	return false;
}

void arch_start_move(int extra) { // TODO
	// Start moving with sent buffers.
}

bool arch_send_fragment() {
	// Send a fragment to the PRU. TODO
}

int arch_fds() {
	return 0;
}

double arch_get_duty(Pin_t _pin) { // TODO
	if (_pin.pin < 0 || _pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_get_duty: %d(max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return 1;
	}
	return 1;
}

void arch_set_duty(Pin_t _pin, double duty) { // TODO
	if (_pin.pin < 0 || _pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_set_duty: %d(max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return;
	}
	// TODO.
}

void arch_discard() {
	// TODO.
}

off_t arch_send_audio(uint8_t *data, off_t sample, off_t num_records, int motor) {
	// TODO.
}
// }}}
#endif

#endif

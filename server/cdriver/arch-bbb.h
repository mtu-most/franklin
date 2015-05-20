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

// Variables. {{{
EXTERN volatile bbb_Gpio *bbb_gpio[4];
EXTERN char *bbb_adc[NUM_ANALOG_INPUTS];
EXTERN int bbb_devmem;
// }}}

// Pin setting. {{{
static inline void SET_OUTPUT(Pin_t _pin) {
	if (_pin.valid()) {
		bbb_gpio[_pin.pin >> 5]->oe &= ~(1 << (_pin.pin & 0x1f));
	}
}

static inline void SET_INPUT(Pin_t _pin) {
	if (_pin.valid()) {
		bbb_gpio[_pin.pin >> 5]->oe |= 1 << (_pin.pin & 0x1f);
	}
}

static inline void SET_INPUT_NOPULLUP(Pin_t _pin) {
	if (_pin.valid()) {
		bbb_gpio[_pin.pin >> 5]->oe |= 1 << (_pin.pin & 0x1f);
	}
}

static inline void SET(Pin_t _pin) {
	if (_pin.valid()) {
		if (_pin.inverted())
			bbb_gpio[_pin.pin >> 5]->cleardataout = 1 << (_pin.pin & 0x1f);
		else
			bbb_gpio[_pin.pin >> 5]->setdataout = 1 << (_pin.pin & 0x1f);
	}
}

static inline void RESET(Pin_t _pin) {
	if (_pin.valid ()) {
		if (_pin.inverted())
			bbb_gpio[_pin.pin >> 5]->setdataout = 1 << (_pin.pin & 0x1f);
		else
			bbb_gpio[_pin.pin >> 5]->cleardataout = 1 << (_pin.pin & 0x1f);
	}
}

static inline bool GET(Pin_t _pin, bool _default) {
	if (_pin.valid ()) {
		if (bbb_gpio[_pin.pin >> 5]->datain & (1 << (_pin.pin & 0x1f)))
			return !_pin.inverted ();
		else
			return _pin.inverted ();
	}
	else
		return _default;
}
// }}}

// Setup helpers. {{{ TODO
static inline void arch_setup_start(char const *port) {
	// TODO: find out if this thing varies, implement a search if it does.
	FILE *f = fopen("/sys/devices/bone_capemgr.9/slots", "w");
	fprintf(f, "cape-bone-iio\n");
	fclose(f);
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i) {
		// TODO: find out if this varies, implement a search if so.
		asprintf(&bbb_adc[i], "/sys/devices/ocp.3/helper.15/AIN%d", i);
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
}

static inline void arch_setup_end(char const *run_id) {
}

static inline void arch_setup_temp(int which, int thermistor_pin, int active, int power_pin = -1, bool power_inverted = true, int power_target = 0, int fan_pin = -1, bool fan_inverted = false, int fan_target = 0) {
	// TODO
}
// }}}

// Runtime helpers. {{{ TODO
static inline void arch_motors_change() {
	// Configure hardware for updated settings.
	// number of active motors.
	// hwtime_step
	// led pin
	// probe pin
	// timeout
	// motor pins
}

static inline void arch_addpos(int s, int m, int diff) {
	// hwcurrent_pos has been modified; this function must update arch internals to match.
}

static inline void arch_stop() {
	// Stop moving, update current_pos.
}

static inline void arch_home() {
	// Start homing.
}

static inline bool arch_running() {
	// True if an underrun will follow.
	return false;
}

static inline void arch_start_move(int extra) {
	// Start moving with sent buffers.
}

static inline void arch_send_fragment(int fragment) {
	// Send a fragment to the PRU.
}

static inline int arch_fds() {
	return 0;
}
// }}}

#endif

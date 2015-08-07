// vim: set foldmethod=marker :
#ifndef ADCBITS

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
#define NUM_GPIO_PINS (4 * 32)
#define NUM_DIGITAL_PINS (NUM_GPIO_PINS + 16)
#define ADCBITS 12
#define FRAGMENTS_PER_BUFFER 8
#define SAMPLES_PER_FRAGMENT 256
#define BBB_PRU_FRAGMENT_MASK (FRAGMENTS_PER_BUFFER - 1)

#define ARCH_MOTOR int bbb_id;
#define ARCH_SPACE int bbb_id, bbb_m0;

#define DATA_CLEAR(s, m) do {} while (0)
#define ARCH_NEW_MOTOR(s, m, base) do {} while (0)
#define DATA_DELETE(s, m) do {} while (0)
// }}}

#else

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

struct bbb_Pru { // {{{
	volatile uint16_t buffer[FRAGMENTS_PER_BUFFER][SAMPLES_PER_FRAGMENT][2];
	volatile uint16_t neg_base, pos_base;
	// These must be bytes, because read and write must be atomic.
	volatile uint8_t current_sample, current_fragment, next_fragment, state;
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
void DATA_SET(int s, int m, int value);
// }}}

// Variables. {{{
EXTERN volatile bbb_Gpio *bbb_gpio[4];
EXTERN int bbb_devmem;
EXTERN int bbb_active_temp;
EXTERN bbb_Temp bbb_temp[NUM_ANALOG_INPUTS];
EXTERN bbb_Pru *bbb_pru;
// }}}

#ifdef DEFINE_VARIABLES
// Pin setting. {{{
void SET_OUTPUT(Pin_t _pin) {
	if (_pin.valid() && _pin.pin < NUM_GPIO_PINS) {
		bbb_gpio[_pin.pin >> 5]->oe &= ~(1 << (_pin.pin & 0x1f));
	}
}

void SET_INPUT(Pin_t _pin) {
	if (_pin.valid() && _pin.pin < NUM_GPIO_PINS) {
		bbb_gpio[_pin.pin >> 5]->oe |= 1 << (_pin.pin & 0x1f);
	}
}

void SET_INPUT_NOPULLUP(Pin_t _pin) {
	if (_pin.valid() && _pin.pin < NUM_GPIO_PINS) {
		bbb_gpio[_pin.pin >> 5]->oe |= 1 << (_pin.pin & 0x1f);
	}
}

#define RAWSET(_p) bbb_gpio[(_p) >> 5]->setdataout = 1 << ((_p) & 0x1f)
#define RAWRESET(_p) bbb_gpio[(_p) >> 5]->cleardataout = 1 << ((_p) & 0x1f)
void SET(Pin_t _pin) {
	if (_pin.valid() && _pin.pin < NUM_GPIO_PINS) {
		if (_pin.inverted())
			RAWRESET(_pin.pin);
		else
			RAWSET(_pin.pin);
	}
}

void RESET(Pin_t _pin) {
	if (_pin.valid() && _pin.pin < NUM_GPIO_PINS) {
		if (_pin.inverted())
			RAWSET(_pin.pin);
		else
			RAWRESET(_pin.pin);
	}
}

#define RAWGET(_p) (bool(bbb_gpio[(_p) >> 5]->datain & (1 << ((_p) & 0x1f))))
void GET(Pin_t _pin, bool _default, void(*cb)(bool)) {
	if (_pin.valid() && _pin.pin < NUM_GPIO_PINS) {
		if (RAWGET(_pin.pin))
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
	fprintf(f, "athena-0\n");
	fclose(f);
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i) {
		// TODO: find out if this varies, implement a search if so.
		char *filename;
		asprintf(&filename, "/sys/devices/ocp.3/helper.15/AIN%d", i);
		bbb_temp[i].fd = open(filename, O_RDONLY);
		free(filename);
		bbb_temp[i].active = false;
	}
	for (int i = 0; i < NUM_GPIO_PINS; ++i) {
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
	setup_end();
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
// state: 0: waiting for limit check; cpu can set to 1, 2, or 3.
// state: 1: not running; cpu can set to 0.
// state: 2: Doing single step; pru can set to 0; cpu can set to 4 (and expect pru to set it to 0 or 1).
// state: 3: Free running; cpu can set to 4.
// state: 4: cpu requested stop; pru must set to 1.
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
		else if (bbb_temp[a].active) {
			int t = atoi(data);
			if (bbb_temp[a].power_pin >= 0) {
				if ((bbb_temp[a].power_target < t) ^ bbb_temp[a].power_inverted)
					RAWSET(bbb_temp[a].power_pin);
				else
					RAWRESET(bbb_temp[a].power_pin);
			}
			if (bbb_temp[a].fan_pin >= 0) {
				if ((bbb_temp[a].fan_target < t) ^ bbb_temp[a].fan_inverted)
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
	int state = bbb_pru->state;
	if (state != 2 && state != 1) {
		// Check probe.
		if (settings.probing && probe_pin.valid()) {
			if (RAWGET(probe_pin.pin) ^ probe_pin.inverted()) {
				// Probe hit.
				sending_fragment = 0;
				stopping = 2;
				send_host(CMD_LIMIT, -1, -1, NAN);
				cbs_after_current_move = 0;
			}
		}
		int m0 = 0;
		// Avoid race condition by reading cf twice.
		int cf = bbb_pru->current_fragment;
		int cs;
		while (true) {
			cs = bbb_pru->current_sample;
			int cf2 = bbb_pru->current_fragment;
			if (cf == cf2)
				break;
			cf = cf2;
		}
		for (int s = 0; s < num_spaces; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m) {
				if (!spaces[s].motor[m]->active || !spaces[s].motor[m]->step_pin.valid() || spaces[s].motor[m]->step_pin.pin < NUM_GPIO_PINS)
					continue;
				int pin = spaces[s].motor[m]->step_pin.pin - NUM_GPIO_PINS;
				bool negative = bool((bbb_pru->buffer[cf][cs][0] ^ bbb_pru->neg_base) & (1 << pin)) ^ spaces[s].motor[m]->dir_pin.inverted();
				Pin_t *p = negative ? &spaces[s].motor[m]->limit_max_pin : &spaces[s].motor[m]->limit_min_pin;
				if (!p->valid())
					continue;
				if (RAWGET(p->pin) ^ p->inverted()) {
					// Limit hit.
					sending_fragment = 0;
					stopping = 2;
					send_host(CMD_LIMIT, s, m, spaces[s].motor[m]->settings.current_pos / spaces[s].motor[m]->steps_per_unit);
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
		// TODO: LED.
		// TODO: Timeout.
	}
	return state == 3 ? 100 : state == 2 ? 10 : 200;
}

void arch_motors_change() { // TODO
	for (int s = 0; s < num_spaces; ++s) {
		for (int m = 0; m < spaces[s].num_motors; ++m) {
			Pin_t *p = &spaces[s].motor[m]->dir_pin;
			if (p->valid() && p->pin >= NUM_GPIO_PINS) {
				int pin = p->pin - NUM_GPIO_PINS;
				if (p->inverted())
					bbb_pru->pos_base |= 1 << pin;
				else
					bbb_pru->neg_base |= 1 << pin;
			}
			p = &spaces[s].motor[m]->step_pin;
			if (p->valid() && p->pin >= NUM_GPIO_PINS && p->inverted()) {
				int pin = p->pin - NUM_GPIO_PINS;
				bbb_pru->pos_base |= 1 << pin;
				bbb_pru->neg_base |= 1 << pin;
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
}

void arch_addpos(int s, int m, int diff) {
	// Nothing to do.
}

void arch_stop(bool fake) {
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
		while (bbb_pru->state == 4) {}
		break;
	}
	bbb_pru->state = 1;
	// Update current_pos.
	abort_move(bbb_pru->current_sample);
	current_fragment_pos = 0;
}

void arch_home() { // TODO
	// Start homing.
}

bool arch_running() {
	// True if an underrun will follow.
	return bbb_pru->state != 1;
}

void arch_start_move(int extra) {
	// Start moving with sent buffers.
	int state = bbb_pru->state;
	if (state != 1)
		debug("arch_start_move called with non-1 state %d", state);
	bbb_pru->state = 0;
}

bool arch_send_fragment() {
	bbb_pru->next_fragment = (bbb_pru->next_fragment + 1) & BBB_PRU_FRAGMENT_MASK;
}

int arch_fds() {
	return 0;
}

double arch_get_duty(Pin_t _pin) { // TODO
	if (_pin.pin < 0 || _pin.pin >= NUM_GPIO_PINS) {
		debug("invalid pin for arch_get_duty: %d(max %d)", _pin.pin, NUM_GPIO_PINS);
		return 1;
	}
	return 1;
}

void arch_set_duty(Pin_t _pin, double duty) { // TODO
	if (_pin.pin < 0 || _pin.pin >= NUM_GPIO_PINS) {
		debug("invalid pin for arch_set_duty: %d(max %d)", _pin.pin, NUM_GPIO_PINS);
		return;
	}
	// TODO.
}

void arch_discard() {
	int fragments = (current_fragment - bbb_pru->current_fragment) & BBB_PRU_FRAGMENT_MASK;
	if (fragments <= 2)
		return;
	current_fragment = (current_fragment - (fragments - 2)) & BBB_PRU_FRAGMENT_MASK;
	bbb_pru->next_fragment = current_fragment;
	restore_settings();
}

off_t arch_send_audio(uint8_t *data, off_t sample, off_t num_records, int motor) {
	// TODO.
}

static void bbb_set_pru(int which, int s, int m) {
	int pin = spaces[s].motor[m]->step_pin.pin - NUM_GPIO_PINS;
	if (spaces[s].motor[m]->step_pin.valid() && pin >= 0) {
		if (spaces[s].motor[m]->step_pin.inverted())
			bbb_pru->buffer[current_fragment][current_fragment_pos][which] &= ~(1 << pin);
		else
			bbb_pru->buffer[current_fragment][current_fragment_pos][which] |= 1 << pin;
	}
}

void DATA_SET(int s, int m, int value) {
	int id = spaces[s].motor[m]->bbb_id;
	if (value) {
		if (value < -1 || value > 1) {
			debug("invalid sample %d for %d %d", value, s, m);
			abort();
		}
		// The invert flag of the dir pin is used even if the pin is invalid.
		if ((value < 0) ^ spaces[s].motor[m]->dir_pin.inverted())
			bbb_set_pru(0, s, m);
		else
			bbb_set_pru(1, s, m);
	}
}
// }}}
#endif

#endif

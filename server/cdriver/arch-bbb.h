// This is not an include quard.  This file is included twice; the first time the first part of the file is used, the second time the second part.
#ifndef _ARCH_BBB_H
#define _ARCH_BBB_H
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

#define SET_OUTPUT(pin_no) do { \
	if ((pin_no).valid()) { \
		int _pin = (pin_no).pin; \
		bbb_gpio[_pin >> 5]->oe &= ~(1 << (_pin & 0x1f)); \
	} \
} while (0)

#define SET_INPUT(pin_no) do { \
	if ((pin_no).valid()) { \
		int _pin = (pin_no).pin; \
		bbb_gpio[_pin >> 5]->oe |= 1 << (_pin & 0x1f); \
	} \
} while (0)

#define SET_INPUT_NOPULLUP(pin_no) do { \
	if ((pin_no).valid()) { \
		int _pin = (pin_no).pin; \
		bbb_gpio[_pin >> 5]->oe |= 1 << (_pin & 0x1f); \
	} \
} while (0)

#define SET(pin_no) do { \
	if ((pin_no).valid()) {\
		int _pin = (pin_no).pin; \
		if ((pin_no).inverted()) \
			bbb_gpio[_pin >> 5]->cleardataout = 1 << (_pin & 0x1f); \
		else \
			bbb_gpio[_pin >> 5]->setdataout = 1 << (_pin & 0x1f); \
	} \
} while (0)

#define RESET(pin_no) do { \
	if ((pin_no).valid ()) { \
		int _pin = (pin_no).pin; \
		if ((pin_no).inverted()) \
			bbb_gpio[_pin >> 5]->setdataout = 1 << (_pin & 0x1f); \
		else \
			bbb_gpio[_pin >> 5]->cleardataout = 1 << (_pin & 0x1f); \
	} \
} while (0)

#define GET(pin_no, _default) ((pin_no).valid () ? bbb_gpio[(pin_no).pin >> 5]->datain & (1 << ((pin_no).pin & 0x1f)) ? !(pin_no).inverted () : (pin_no).inverted () : _default)

#define A0 (4 * 32)
#define NUM_ANALOG_INPUTS 7
#define NUM_DIGITAL_PINS A0 + NUM_ANALOG_INPUTS	// Not true, but otherwise the last pins will be displayed as analog.  Now there are some unusable analog pins displayed.
#define ADCBITS 12

#ifdef WATCHDOG
//#warning Watchdog is not supported on BeagleBone; ignoring request to use it.
#undef WATCHDOG
#endif

#define debug(...) do { buffered_debug_flush(); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while (0)
#define F(x) (x)

struct bbb_Gpio {
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

extern volatile bbb_Gpio *bbb_gpio[4];
extern char *bbb_adc[NUM_ANALOG_INPUTS];
extern int bbb_devmem;
extern struct pollfd bbb_pollfd;

static inline void arch_setup_start() {
	// TODO: find out if this thing varies, implement a search if it does.
	FILE *f = fopen("/sys/devices/bone_capemgr.9/slots", "w");
	fprintf(f, "cape-bone-iio\n");
	fclose(f);
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i) {
		// TODO: find out if this varies, implement a search if so.
		asprintf(&bbb_adc[i], "/sys/devices/ocp.3/helper.15/AIN%d", i);
	}
	for (int i = 0; i < A0; ++i) {
		f = std::fopen("/sys/class/gpio/export", "w");
		fprintf(f, "%d\n", i);
		fclose(f);
	}
	bbb_devmem = open("/dev/mem", O_RDWR);
	unsigned gpio_base[4] = { 0x44E07000, 0x4804C000, 0x481AC000, 0x481AE000 };
	for (int i = 0; i < 4; ++i)
		bbb_gpio[i] = (volatile bbb_Gpio *)mmap(0, 0x2000, PROT_READ | PROT_WRITE, MAP_SHARED, bbb_devmem, gpio_base[i]);
	bbb_pollfd.fd = 0; // standard input.
	bbb_pollfd.events = POLLIN | POLLPRI;
	bbb_pollfd.revents = 0;
}

static inline void arch_setup_end() {
}

static inline void arch_run() {
}

void reset();

class FakeSerial {
	struct pollfd mypollfd;
	char buffer[256];
	int start, end;
public:
	void begin(int baud) {
		mypollfd.fd = 0;
		mypollfd.events = POLLIN | POLLPRI;
		start = 0;
		end = 0;
		fcntl(0, F_SETFL, O_NONBLOCK);
	}
	void write(char c) {
		//fprintf(stderr, "Firmware write byte: %x\n", c);
		::write(1, &c, 1);
	}
	void refill() {
		start = 0;
		end = ::read(0, buffer, sizeof(buffer));
		//fprintf(stderr, "refill %d bytes\n", end);
		if (end < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK)
				fprintf(stderr, "read returned error: %s\n", strerror(errno));
			end = 0;
		}
		if (end == 0 && bbb_pollfd.revents) {
			fprintf(stderr, "EOF detected on standard input; exiting.\n");
			reset();
		}
		bbb_pollfd.revents = 0;
	}
	int read() {
		if (start == end)
			refill();
		if (start == end) {
			fprintf(stderr, "eof on input; exiting.\n");
			reset();
		}
		int ret = buffer[start++];
		//fprintf(stderr, "Firmware read byte: %x\n", ret);
		return ret;
	}
	int readBytes (char *target, int len) {
		for (int i = 0; i < len; ++i)
			*target++ = read();
		return len;
	}
	void flush() {}
	int available() {
		if (start == end)
			refill();
		return end - start;
	}
};

static inline void get_current_times(uint32_t *current_time, uint32_t *longtime) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	if (current_time)
		*current_time = tv.tv_sec * 1000000 + tv.tv_usec;
	if (longtime)
		*longtime = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	//fprintf(stderr, "current times: %d %d\n", *current_time, *longtime);
}

static inline uint32_t utime() {
	uint32_t ret;
	get_current_times(&ret, NULL);
	return ret;
}
static inline uint32_t millis() {
	uint32_t ret;
	get_current_times(NULL, &ret);
	return ret;
}
//#define microdelay() usleep(1)
#define microdelay() do {} while(0)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

static inline uint8_t read_eeprom(uint32_t address) {
	return 0;
}

static inline void write_eeprom(uint32_t address, uint8_t data) {
}

static inline void wait_for_event(uint32_t micro, uint32_t current_time) {
	bbb_pollfd.revents = 0;
	fprintf(stderr, "polling with micro %d\n", micro);
	poll(&bbb_pollfd, 1, micro == ~0 ? -1 : micro / 1000);
}

#else

EXTERN FakeSerial Serial;
EXTERN volatile bbb_Gpio *bbb_gpio[4];
EXTERN char *bbb_adc[NUM_ANALOG_INPUTS];
EXTERN int bbb_devmem;
EXTERN struct pollfd bbb_pollfd;

/* TODO: merge this with arch_run.
#ifdef HAVE_AUDIO
static void handle_audio(uint32_t current_time, uint32_t longtime) {
	if (audio_head != audio_tail) {
		last_active = longtime;
		int32_t sample = (current_time - audio_start) / audio_us_per_sample;
		int32_t audio_byte = sample >> 3;
		while (audio_byte >= AUDIO_FRAGMENT_SIZE) {
			debug("next audio fragment");
			if ((audio_tail + 1) % AUDIO_FRAGMENTS == audio_head)
			{
				//debug("continue audio");
				continue_cb |= 2;
				try_send_next();
			}
			audio_head = (audio_head + 1) % AUDIO_FRAGMENTS;
			if (audio_tail == audio_head) {
				//debug("audio done");
				return;
			}
			audio_byte -= AUDIO_FRAGMENT_SIZE;
			// us per fragment = us/sample*sample/fragment
			audio_start += audio_us_per_sample * 8 * AUDIO_FRAGMENT_SIZE;
		}
		uint8_t old_state = audio_state;
		audio_state = (audio_buffer[audio_head][audio_byte] >> (sample & 7)) & 1;
		if (audio_state != old_state) {
			for (uint8_t s = 0; s < num_spaces; ++s) {
				Space &sp = spaces[s];
				for (uint8_t m = 0; m < sp.num_motors; ++m) {
					if (!(sp.motor[m]->audio_flags & Motor::PLAYING))
						continue;
					if (audio_state > old_state)
						SET(sp.motor[m]->dir_pin);
					else
						RESET(sp.motor[m]->dir_pin);
					SET(sp.motor[m]->step_pin);
					RESET(sp.motor[m]->step_pin);
				}
			}
		}
	}
}
#endif
*/
#endif

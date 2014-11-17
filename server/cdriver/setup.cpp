#define EXTERN	// This must be done in exactly one source file.
#include "cdriver.h"

void setup(char const *port)
{
	serialdev[0] = &real_serial;
	real_serial.begin(115200);
	serialdev[1] = NULL;
	arch_setup_start(port);
	watchdog_disable();
	setup_spacetypes();
	// Initialize volatile variables.
	initialized = false;
#if DEBUG_BUFFER_LENGTH > 0
	debug_buffer_ptr = 0;
#endif
	debug("Starting");
	adc_phase = 0;
	temp_current = 0;
	name = NULL;
	command_end[0] = 0;
	command_end[1] = 0;
	motors_busy = false;
	queue_start = 0;
	queue_end = 0;
	queue_full = false;
	continue_cb = 0;
	ping = 0;
	pending_packet[0] = 0;
	out_busy = false;
	led_pin.init();
	probe_pin.init();
	probe_dist = INFINITY;
	probe_safe_dist = INFINITY;
	led_phase = 0;
	temps_busy = 0;
	requested_temp = ~0;
	last_active = millis();
	t0 = 0;
	f0 = 0;
	//debug("moving->false");
	moving = false;
	stopping = false;
	move_prepared = false;
	cbs_after_current_move = 0;
	which_autosleep = 0;
#ifdef HAVE_AUDIO
	audio_head = 0;
	audio_tail = 0;
	audio_state = 0;
	audio_us_per_sample = 125; // 1000000 / 8000;
#endif
	last_current_time = utime();
	num_spaces = 0;
	spaces = NULL;
	next_motor_time = ~0;
	num_temps = 0;
	temps = NULL;
	next_temp_time = ~0;
	num_gpios = 0;
	gpios = NULL;
#ifdef HAVE_AUDIO
	next_audio_time = ~0;
#endif
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		printerid[i] = 0;
	arch_setup_end();
	load_all();
}

void load_all() {
	//debug("loading all");
	int32_t addr = 0;
	globals_load(addr, true);
	for (uint8_t t = 0; t < num_spaces; ++t) {
		//debug("space %d", t);
		spaces[t].load_info(addr, true);
		for (uint8_t a = 0; a < spaces[t].num_axes; ++a)
			spaces[t].load_axis(a, addr, true);
		for (uint8_t m = 0; m < spaces[t].num_motors; ++m)
			spaces[t].load_motor(m, addr, true);
	}
	for (uint8_t t = 0; t < num_temps; ++t) {
		//debug("temp %d", t);
		temps[t].load(addr, true);
	}
	for (uint8_t t = 0; t < num_gpios; ++t) {
		//debug("gpio %d", t);
		gpios[t].load(t, addr, true);
	}
}

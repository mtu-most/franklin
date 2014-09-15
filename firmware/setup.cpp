#include "firmware.h"

void setup()
{
	arch_setup_start();
	watchdog_disable();
#ifdef HAVE_SPACES
	setup_spacetypes();
#endif
	// Initialize volatile variables.
	Serial.begin(115200);
	initialized = false;
#if DEBUG_BUFFER_LENGTH > 0
	debug_buffer_ptr = 0;
#endif
	debug("Starting");
	adc_phase = 0;
#ifdef HAVE_TEMPS
	temp_current = 0;
#endif
	name = NULL;
	command_end = 0;
	motors_busy = false;
	queue_start = 0;
	queue_end = 0;
	queue_full = false;
	num_movecbs = 0;
	continue_cb = 0;
	ping = 0;
	last_packet = NULL;
	out_busy = false;
	reply_ready = false;
	led_pin.read(0);
	probe_pin.read(0);
	probe_dist = INFINITY;
	probe_safe_dist = INFINITY;
	led_phase = 0;
	temps_busy = 0;
	requested_temp = ~0;
	led_last = millis();
	last_active = millis();
	t0 = 0;
	f0 = 0;
	//debug("moving->false");
	moving = false;
	move_prepared = false;
	cbs_after_current_move = 0;
	which_autosleep = 0;
#ifdef HAVE_AUDIO
	audio_head = 0;
	audio_tail = 0;
	audio_state = 0;
	audio_us_per_sample = 125; // 1000000 / 8000;
#endif
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		printerid[i] = 0;
#ifdef HAVE_SPACES
	num_spaces = 0;
	spaces = NULL;
#endif
#ifdef HAVE_TEMPS
	num_temps = 0;
	temps = NULL;
#endif
#ifdef HAVE_GPIOS
	num_gpios = 0;
	gpios = NULL;
#endif
	debug("running arch end");
	Serial.write(CMD_ID);
	for (uint8_t i = 0; i < 8; ++i)
		Serial.write(uint8_t(0));
	arch_setup_end();
	load_all();
}

void load_all() {
	//debug("loading all");
	int16_t addr = 0;
	globals_load(addr, true);
#ifdef HAVE_SPACES
	for (uint8_t t = 0; t < num_spaces; ++t) {
		//debug("space %d", t);
		spaces[t].load_info(addr, true);
		for (uint8_t a = 0; a < spaces[t].num_axes; ++a)
			spaces[t].load_axis(a, addr, true);
		for (uint8_t m = 0; m < spaces[t].num_motors; ++m)
			spaces[t].load_motor(m, addr, true);
	}
#endif
#ifdef HAVE_TEMPS
	for (uint8_t t = 0; t < num_temps; ++t) {
		//debug("temp %d", t);
		temps[t].load(addr, true);
	}
#endif
#ifdef HAVE_GPIOS
	for (uint8_t t = 0; t < num_gpios; ++t) {
		//debug("gpio %d", t);
		gpios[t].load(t, addr, true);
	}
#endif
}

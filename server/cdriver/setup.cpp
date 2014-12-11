#define EXTERN	// This must be done in exactly one source file.
#include "cdriver.h"

void setup(char const *port)
{
	serialdev[0] = &host_serial;
	host_serial.begin(115200);
	serialdev[1] = NULL;
	arch_setup_start(port);
	setup_spacetypes();
	// Initialize volatile variables.
	initialized = false;
#if DEBUG_BUFFER_LENGTH > 0
	debug_buffer_ptr = 0;
#endif
	debug("Starting");
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
	pending_len = 0;
	out_busy = false;
	led_pin.init();
	probe_pin.init();
	probe_dist = INFINITY;
	probe_safe_dist = INFINITY;
	led_phase = 0;
	temps_busy = 0;
	requested_temp = ~0;
	last_active = millis();
	free_fragments = 0;
	refilling = false;
	current_fragment = 0;
	current_fragment_pos = 0;
	hwtime_step = 1000;	// TODO: make this dynamic.
	//debug("moving->false");
	moving = false;
	stopping = 0;
	stop_pending = false;
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
	num_temps = 0;
	temps = NULL;
	num_gpios = 0;
	gpios = NULL;
	arch_setup_end();
	if (protocol_version < PROTOCOL_VERSION) {
		debug("Printer has older Franklin version than host; please flash newer firmware.");
		exit(1);
	}
	else if (protocol_version > PROTOCOL_VERSION) {
		debug("Printer has newer Franklin version than host; please upgrade your host software.");
		exit(1);
	}
	// Now set things up that need information from the firmware.
	settings = new History[FRAGMENTS_PER_BUFFER];
	settings[current_fragment].t0 = 0;
	settings[current_fragment].f0 = 0;
	settings[current_fragment].num_active_motors = 0;
	settings[current_fragment].hwtime = 0;
	free_fragments = FRAGMENTS_PER_BUFFER;
}

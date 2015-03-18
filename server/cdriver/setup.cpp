#define EXTERN	// This must be done in exactly one source file.
#include "cdriver.h"

void setup(char const *port, char const *run_id)
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
	pollfds[0].fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
	pollfds[0].events = POLLIN | POLLPRI;
	pollfds[0].revents = 0;
	temp_current = 0;
	command_end[0] = 0;
	command_end[1] = 0;
	motors_busy = false;
	current_extruder = 0;
	continue_cb = 0;
	ping = 0;
	pending_len = 0;
	out_busy = false;
	led_pin.init();
	probe_pin.init();
	led_phase = 0;
	temps_busy = 0;
	requested_temp = ~0;
	last_active = millis();
	refilling = false;
	running_fragment = 0;
	current_fragment = running_fragment;
	current_fragment_pos = 0;
	hwtime_step = 4000;	// TODO: make this dynamic.
	moving = false;
	aborting = false;
	stopped = true;
	prepared = false;
	stopping = 0;
	sending_fragment = 0;
	start_pending = false;
	stop_pending = false;
	cbs_after_current_move = 0;
	which_autosleep = 0;
	timeout = 0;
#ifdef HAVE_AUDIO
	audio_head = 0;
	audio_tail = 0;
	audio_state = 0;
	audio_us_per_sample = 125; // 1000000 / 8000;
#endif
	num_spaces = 0;
	spaces = NULL;
	num_temps = 0;
	temps = NULL;
	num_gpios = 0;
	gpios = NULL;
	arch_setup_end(run_id);
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
	for (int i = 0; i < 2; ++i) {
		int f = (current_fragment - i + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
		settings[f].t0 = 0;
		settings[f].f0 = 0;
		settings[f].num_active_motors = 0;
		settings[f].hwtime = 0;
		settings[f].last_current_time = 0;
		settings[f].cbs = 0;
		settings[f].tp = 0;
		settings[f].f1 = 1;
		settings[f].f2 = 0;
		settings[f].fp = 0;
		settings[f].fq = 0;
		settings[f].fmain = 1;
		settings[f].fragment_length = 0;
		settings[f].start_time = 0;
		settings[f].last_time = 0;
		settings[f].queue_start = 0;
		settings[f].queue_end = 0;
		settings[f].queue_full = false;
	}
	// Update current position.
	first_fragment = current_fragment;
	arch_stop();
}

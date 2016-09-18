/* setup.cpp - data initialization for Franklin
 * Copyright 2014 Michigan Technological University
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

#include "cdriver.h"

static unsigned char host_command[HOST_COMMAND_SIZE];
#ifdef SERIAL
static unsigned char serial_command[FULL_SERIAL_COMMAND_SIZE];
#endif

void setup()
{
	command[0] = host_command;
#ifdef SERIAL
	command[1] = serial_command;
#endif
	preparing = false;
	host_block = false;
	sent_names = false;
	last_active = millis();
	last_micros = utime();
	serialdev[0] = &host_serial;
	host_serial.begin();
	serialdev[1] = NULL;
	command_end[1] = 0;
	arch_setup_start();
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
	command_end[0] = 0;
	motors_busy = false;
	current_extruder = 0;
	continue_cb = 0;
	ping = 0;
	for (int i = 0; i < 4; ++i) {
		pending_len[i] = 0;
		wait_for_reply[i] = NULL;
		serial_cb[i] = NULL;
	}
	out_busy = 0;
	led_pin.init();
	stop_pin.init();
	probe_pin.init();
	led_phase = 0;
	temps_busy = 0;
	store_adc = NULL;
	requested_temp = ~0;
	refilling = false;
	running_fragment = 0;
	current_fragment = running_fragment;
	current_fragment_pos = 0;
	num_active_motors = 0;
	hwtime_step = 10000; // Note: When changing this, also change max in cdriver/space.cpp
	audio_hwtime_step = 1;	// This is set by audio file.
	feedrate = 1;
	max_deviation = 0;
	max_v = INFINITY;
	targetx = 0;
	targety = 0;
	zoffset = 0;
	aborting = false;
	computing_move = false;
	moving_to_current = 0;
	prepared = false;
	stopping = 0;
	sending_fragment = 0;
	transmitting_fragment = false;
	start_pending = false;
	stop_pending = false;
	discard_pending = false;
	change_pending = false;
	discarding = false;
	cbs_after_current_move = 0;
	which_autosleep = 0;
	timeout = 0;
	bed_id = 255;
	fan_id = 255;
	spindle_id = 255;
	run_file_map = NULL;
	run_file_finishing = false;
	expected_replies = 0;
	num_temps = 0;
	temps = NULL;
	num_gpios = 0;
	gpios = NULL;
	for (int s = 0; s < NUM_SPACES; ++s)
		spaces[s].init(s);
}

void connect_end() {
	if (protocol_version < PROTOCOL_VERSION) {
		debug("Printer has older Franklin version %d than host which has %d; please flash newer firmware.", protocol_version, PROTOCOL_VERSION);
		exit(1);
	}
	else if (protocol_version > PROTOCOL_VERSION) {
		debug("Printer has newer Franklin version %d than host which has %d; please upgrade your host software.", protocol_version, PROTOCOL_VERSION);
		exit(1);
	}
	// Now set things up that need information from the firmware.
	history = new History[FRAGMENTS_PER_BUFFER];
	for (int i = 0; i < 2; ++i) {
		int f = (current_fragment - i + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
		history[f].t0 = 0;
		history[f].f0 = 0;
		history[f].hwtime = 0;
		history[f].last_current_time = 0;
		history[f].cbs = 0;
		history[f].tp = 0;
		history[f].f1 = 1;
		history[f].f2 = 0;
		history[f].fp = 0;
		history[f].fq = 0;
		history[f].fmain = 1;
		history[f].start_time = 0;
		history[f].last_time = 0;
		history[f].queue_start = 0;
		history[f].queue_end = 0;
		history[f].queue_full = false;
	}
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		sp.history = new Space_History[FRAGMENTS_PER_BUFFER];
		for (int a = 0; a < sp.num_axes; ++a) {
			delete[] sp.axis[a]->history;
			sp.axis[a]->history = new Axis_History[FRAGMENTS_PER_BUFFER];
			for (int f = 0; f < FRAGMENTS_PER_BUFFER; ++f) {
				sp.axis[a]->history[f].dist[0] = NAN;
				sp.axis[a]->history[f].dist[1] = NAN;
				sp.axis[a]->history[f].main_dist = NAN;
				sp.axis[a]->history[f].target = NAN;
				sp.axis[a]->history[f].source = NAN;
				sp.axis[a]->history[f].current = NAN;
			}
		}
		for (int m = 0; m < sp.num_motors; ++m) {
			delete[] sp.motor[m]->history;
			sp.motor[m]->history = new Motor_History[FRAGMENTS_PER_BUFFER];
			for (int f = 0; f < FRAGMENTS_PER_BUFFER; ++f) {
				sp.motor[m]->history[f].last_v = 0;
				sp.motor[m]->history[f].current_pos = 0;
				sp.motor[m]->history[f].last_v = 0;
				sp.motor[m]->history[f].target_v = NAN;
				sp.motor[m]->history[f].target_dist = NAN;
				sp.motor[m]->history[f].endpos = NAN;
			}
		}
	}
	// Update current position.
	first_fragment = current_fragment;
	//debug("not blocking host");
	host_block = false;
	arch_stop(true);
	// Update pin names at next globals update.
	sent_names = false;
	send_host(CMD_CONNECTED);
}

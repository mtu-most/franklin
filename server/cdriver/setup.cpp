/* setup.cpp - data initialization for Franklin
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

#include "cdriver.h"

void setup()
{
	connected = false;
	preparing = false;
	last_active = millis();
	last_micros = utime();
	serialdev = NULL;
	command_end = 0;
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
	motors_busy = false;
	current_extruder = 0;
	ping = 0;
	for (int i = 0; i < 4; ++i) {
		pending_len[i] = 0;
		wait_for_reply[i] = NULL;
		serial_cb[i] = NULL;
	}
	out_busy = 0;
	num_file_done_events = 0;
	continue_event = false;
	num_movecbs = 0;
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
	queue_start = 0;
	queue_end = 0;
	queue_full = false;
	//debug("current_fragment = running_fragment; %d %p", current_fragment, &current_fragment);
	current_fragment_pos = 0;
	num_active_motors = 0;
	default_hwtime_step = 4000;
	min_hwtime_step = 3000;
	settings.hwtime_step = default_hwtime_step;
	feedrate = 1;
	max_deviation = 0;
	max_v = 100;
	max_a = 10000;
	max_J = 10000;
	targetx = 0;
	targety = 0;
	zoffset = 0;
	aborting = false;
	computing_move = false;
	stopping = 0;
	sending_fragment = 0;
	transmitting_fragment = false;
	start_pending = false;
	stop_pending = false;
	discard_pending = false;
	change_pending = false;
	discarding = false;
	cbs_after_current_move = 0;
	interrupt_pending = false;
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
	pattern.step_pin.read(0);
	pattern.dir_pin.read(0);
	pattern.active = false;
	for (int s = 0; s < NUM_SPACES; ++s)
		spaces[s].init(s);
	arch_setup_end();
}

void check_protocol() {
	if (protocol_version < PROTOCOL_VERSION) {
		debug("Machine has older Franklin version %d than host which has %d; please flash newer firmware.", protocol_version, PROTOCOL_VERSION);
		exit(1);
	}
	else if (protocol_version > PROTOCOL_VERSION) {
		debug("Machine has newer Franklin version %d than host which has %d; please upgrade your host software.", protocol_version, PROTOCOL_VERSION);
		exit(1);
	}
}

void connect_end() {
	// Set things up that need information from the firmware.
	num_subfragments_bits = int(std::log2(settings.hwtime_step / TIME_PER_ISR));
	delete[] history;
	history = new History[FRAGMENTS_PER_BUFFER];
	for (int i = 0; i < 2; ++i) {
		int f = (current_fragment - i + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER;
		history[f].hwtime = 0;
		history[f].cbs = 0;
		history[f].run_time = 0;
	}
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		sp.history = new Space_History[FRAGMENTS_PER_BUFFER];
		for (int a = 0; a < sp.num_axes; ++a) {
			delete[] sp.axis[a]->history;
			sp.axis[a]->history = setup_axis_history();
		}
		for (int m = 0; m < sp.num_motors; ++m) {
			delete[] sp.motor[m]->history;
			sp.motor[m]->history = setup_motor_history();
		}
	}
	// Restore all temps to their current values.
	for (int t = 0; t < num_temps; ++t) {
		settemp(t, temps[t].target[0]);
		if (temps[t].power_pin[1].valid())
			arch_set_duty(temps[t].power_pin[1], temps[t].fan_duty);
	}
	// Set all gpio duty cycle values.
	for (int g = 0; g < num_gpios; ++g) {
		if (gpios[g].pin.valid())
			arch_set_duty(gpios[g].pin, gpios[g].duty);
	}
	// Update current position.
	first_fragment = current_fragment;
	arch_stop(true);
	// Update pin names at next globals update.
	if (connected) {
		prepare_interrupt();
		send_to_parent(CMD_CONNECTED);
	}
}

Axis_History *setup_axis_history() {
	Axis_History *ret = new Axis_History[FRAGMENTS_PER_BUFFER];
	for (int f = 0; f < FRAGMENTS_PER_BUFFER; ++f) {
		ret[f].target = NAN;
		ret[f].source = NAN;
		ret[f].current = NAN;
	}
	return ret;
}

Motor_History *setup_motor_history() {
	Motor_History *ret = new Motor_History[FRAGMENTS_PER_BUFFER];
	for (int f = 0; f < FRAGMENTS_PER_BUFFER; ++f) {
		ret[f].current_pos = 0;
		ret[f].last_v = 0;
		ret[f].target_v = NAN;
		ret[f].target_pos = NAN;
	}
	return ret;
}

/* space.cpp - Implementations of Space internals for Franklin {{{
 * vim: foldmethod=marker :
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
 * }}} */

#include "cdriver.h"

//#define DEBUG_PATH

#if 0
#define loaddebug debug
#else
#define loaddebug(...) do {} while(0)
#endif

#if 0
#define movedebug(...) debug(__VA_ARGS__)
#else
#define movedebug(...) do {} while (0)
#endif

// Setup. {{{
bool Space::setup_nums(int na, int nm) { // {{{
	if (na == num_axes && nm == num_motors)
		return true;
	loaddebug("new space %d %d %d %d", na, nm, num_axes, num_motors);
	int old_nm = num_motors;
	int old_na = num_axes;
	num_motors = nm;
	num_axes = na;
	if (na != old_na) {
		Axis **new_axes = new Axis *[na];
		loaddebug("new axes: %d", na);
		for (int a = 0; a < min(old_na, na); ++a)
			new_axes[a] = axis[a];
		for (int a = old_na; a < na; ++a) {
			new_axes[a] = new Axis;
			new_axes[a]->park = NAN;
			new_axes[a]->park_order = 0;
			new_axes[a]->min_pos = -INFINITY;
			new_axes[a]->max_pos = INFINITY;
			new_axes[a]->type_data = NULL;
			new_axes[a]->settings.dist[0] = NAN;
			new_axes[a]->settings.dist[1] = NAN;
			new_axes[a]->settings.main_dist = NAN;
			new_axes[a]->settings.target = NAN;
			new_axes[a]->settings.source = NAN;
			new_axes[a]->settings.current = NAN;
			new_axes[a]->history = setup_axis_history();
		}
		for (int a = na; a < old_na; ++a) {
			space_types[type].afree(this, a);
			delete[] axis[a]->history;
			delete axis[a];
		}
		delete[] axis;
		axis = new_axes;
	}
	if (nm != old_nm) {
		Motor **new_motors = new Motor *[nm];
		loaddebug("new motors: %d", nm);
		for (int m = 0; m < min(old_nm, nm); ++m)
			new_motors[m] = motor[m];
		for (int m = old_nm; m < nm; ++m) {
			new_motors[m] = new Motor;
			new_motors[m]->step_pin.init();
			new_motors[m]->dir_pin.init();
			new_motors[m]->enable_pin.init();
			new_motors[m]->limit_min_pin.init();
			new_motors[m]->limit_max_pin.init();
			new_motors[m]->steps_per_unit = 100;
			new_motors[m]->limit_v = INFINITY;
			new_motors[m]->limit_a = INFINITY;
			new_motors[m]->home_pos = NAN;
			new_motors[m]->home_order = 0;
			new_motors[m]->limit_v = INFINITY;
			new_motors[m]->limit_a = INFINITY;
			new_motors[m]->active = false;
			new_motors[m]->settings.last_v = 0;
			new_motors[m]->settings.current_pos = 0;
			new_motors[m]->settings.target_v = NAN;
			new_motors[m]->settings.target_pos = NAN;
			new_motors[m]->history = setup_motor_history();
			ARCH_NEW_MOTOR(id, m, new_motors);
		}
		for (int m = nm; m < old_nm; ++m) {
			DATA_DELETE(id, m);
			delete[] motor[m]->history;
			delete motor[m];
		}
		delete[] motor;
		motor = new_motors;
		arch_motors_change();
	}
	return true;
} // }}}

void Space::load_info() { // {{{
	loaddebug("loading space %d", id);
	int t = type;
	if (t < 0 || t >= NUM_SPACE_TYPES)
		t = DEFAULT_TYPE;
	type = shmem->ints[1];
	if (type < 0 || type >= NUM_SPACE_TYPES || (id == 1 && type != EXTRUDER_TYPE) || (id == 2 && type != FOLLOWER_TYPE)) {
		debug("request for type %d ignored", type);
		type = t;
		return;	// The rest of the info is not meant for this type, so ignore it.
	}
	if (t != type) {
		loaddebug("setting type to %d", type);
		space_types[t].free(this);
		if (!space_types[type].init(this)) {
			type = DEFAULT_TYPE;
			reset_pos(this);
			for (int a = 0; a < num_axes; ++a)
				axis[a]->settings.current = axis[a]->settings.source;
			return;	// The rest of the info is not meant for DEFAULT_TYPE, so ignore it.
		}
	}
	space_types[type].load(this);
	reset_pos(this);
	loaddebug("done loading space");
} // }}}

void reset_pos(Space *s) { // {{{
	double motors[s->num_motors];
	double xyz[s->num_axes];
	for (int m = 0; m < s->num_motors; ++m)
		motors[m] = s->motor[m]->settings.current_pos / s->motor[m]->steps_per_unit;
	space_types[s->type].motors2xyz(s, motors, xyz);
	for (int a = 0; a < s->num_axes; ++a) {
		s->axis[a]->settings.current = xyz[a];
		if (!computing_move)
			s->axis[a]->settings.source = xyz[a];
	}
} // }}}

void Space::load_axis(int a) { // {{{
	loaddebug("loading axis %d", a);
	axis[a]->park_order = shmem->ints[2];
	axis[a]->park = shmem->floats[0];
	axis[a]->min_pos = shmem->floats[1];
	axis[a]->max_pos = shmem->floats[2];
} // }}}

void Space::load_motor(int m) { // {{{
	loaddebug("loading motor %d", m);
	uint16_t enable = motor[m]->enable_pin.write();
	double old_home_pos = motor[m]->home_pos;
	double old_steps_per_unit = motor[m]->steps_per_unit;
	motor[m]->step_pin.read(shmem->ints[2]);
	motor[m]->dir_pin.read(shmem->ints[3]);
	motor[m]->enable_pin.read(shmem->ints[4]);
	motor[m]->limit_min_pin.read(shmem->ints[5]);
	motor[m]->limit_max_pin.read(shmem->ints[6]);
	motor[m]->home_order = shmem->ints[7];
	motor[m]->steps_per_unit = shmem->floats[0];
	if (std::isnan(motor[m]->steps_per_unit)) {
		debug("Trying to set NaN steps per unit for motor %d %d", id, m);
		motor[m]->steps_per_unit = old_steps_per_unit;
	}
	motor[m]->home_pos = shmem->floats[1];
	motor[m]->limit_v = shmem->floats[2];
	motor[m]->limit_a = shmem->floats[3];
	arch_motors_change();
	SET_OUTPUT(motor[m]->enable_pin);
	if (enable != motor[m]->enable_pin.write()) {
		if (motors_busy)
			SET(motor[m]->enable_pin);
		else {
			RESET(motor[m]->enable_pin);
		}
	}
	RESET(motor[m]->step_pin);
	RESET(motor[m]->dir_pin);
	SET_INPUT(motor[m]->limit_min_pin);
	SET_INPUT(motor[m]->limit_max_pin);
	if (!std::isnan(motor[m]->home_pos)) {
		// Axes with a limit switch.
		if (motors_busy && (old_home_pos != motor[m]->home_pos || old_steps_per_unit != motor[m]->steps_per_unit) && !std::isnan(old_home_pos)) {
			double ohp = old_home_pos * old_steps_per_unit;
			double hp = motor[m]->home_pos * motor[m]->steps_per_unit;
			double diff = hp - ohp;
			motor[m]->settings.current_pos += diff;
			//debug("load motor %d %d new home %f add %f", id, m, motor[m]->home_pos, diff);
			arch_addpos(id, m, diff);
		}
	}
	else {
		// Axes without a limit switch, including extruders.
		if (motors_busy && old_steps_per_unit != motor[m]->steps_per_unit) {
			debug("load motor %d %d new steps no home", id, m);
			double oldpos = motor[m]->settings.current_pos;
			double pos = oldpos / old_steps_per_unit;
			motor[m]->settings.current_pos = pos * motor[m]->steps_per_unit;
			arch_addpos(id, m, motor[m]->settings.current_pos - oldpos);
			// Adjust current_pos in all history.
			for (int h = 0; h < FRAGMENTS_PER_BUFFER; ++h) {
				oldpos = motor[m]->history[h].current_pos;
				pos = oldpos / old_steps_per_unit;
				motor[m]->history[h].current_pos = pos * motor[m]->steps_per_unit;
			}
		}
	}
	reset_pos(this);
} // }}}

void Space::save_info() { // {{{
	shmem->ints[1] = type;
	shmem->ints[2] = num_axes;
	shmem->ints[3] = num_motors;
	space_types[type].save(this);
} // }}}

void Space::save_axis(int a) { // {{{
	shmem->ints[2] = axis[a]->park_order;
	shmem->floats[0] = axis[a]->park;
	shmem->floats[1] = axis[a]->min_pos;
	shmem->floats[2] = axis[a]->max_pos;
} // }}}

void Space::save_motor(int m) { // {{{
	shmem->ints[2] = motor[m]->step_pin.write();
	shmem->ints[3] = motor[m]->dir_pin.write();
	shmem->ints[4] = motor[m]->enable_pin.write();
	shmem->ints[5] = motor[m]->limit_min_pin.write();
	shmem->ints[6] = motor[m]->limit_max_pin.write();
	shmem->ints[7] = motor[m]->home_order;
	shmem->floats[0] = motor[m]->steps_per_unit;
	shmem->floats[1] = motor[m]->home_pos;
	shmem->floats[2] = motor[m]->limit_v;
	shmem->floats[3] = motor[m]->limit_a;
} // }}}

void Space::init(int space_id) { // {{{
	type = space_id == 0 ? DEFAULT_TYPE : space_id == 1 ? EXTRUDER_TYPE : FOLLOWER_TYPE;
	id = space_id;
	type_data = NULL;
	num_axes = 0;
	num_motors = 0;
	motor = NULL;
	axis = NULL;
	history = NULL;
	space_types[type].init(this);
} // }}}

void Space::cancel_update() { // {{{
	// setup_nums failed; restore system to a usable state.
	type = DEFAULT_TYPE;
	int n = min(num_axes, num_motors);
	if (!setup_nums(n, n)) {
		debug("Failed to free memory; removing all motors and axes to make sure it works");
		if (!setup_nums(0, 0))
			debug("You're in trouble; this shouldn't be possible");
	}
} // }}}
// }}}

// Movement handling. {{{
static void send_fragment() { // {{{
	if (host_block) {
		current_fragment_pos = 0;
		return;
	}
	if (current_fragment_pos <= 0 || stopping || sending_fragment) {
		//debug("no send fragment %d %d %d", current_fragment_pos, stopping, sending_fragment);
		return;
	}
	if (num_active_motors == 0) {
		if (current_fragment_pos < 2) {
			// TODO: find out why this is attempted and avoid it.
			debug("not sending short fragment for 0 motors; %d %d", current_fragment, running_fragment);
			if (history[current_fragment].cbs) {
				if (settings.queue_start == settings.queue_end && !settings.queue_full) {
					// Send cbs immediately.
					if (!host_block) {
						num_movecbs += history[current_fragment].cbs;
						//debug("adding %d cbs in send_fragment", history[current_fragment].cbs);
						history[current_fragment].cbs = 0;
					}
				}
			}
			current_fragment_pos = 0;
			return;
		}
		else
			debug("sending fragment for 0 motors at position %d", current_fragment_pos);
		//abort();
	}
	//debug("sending %d prevcbs %d", current_fragment, history[(current_fragment + FRAGMENTS_PER_BUFFER - 1) % FRAGMENTS_PER_BUFFER].cbs);
	if (arch_send_fragment()) {
		current_fragment = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
		//debug("current_fragment = (current_fragment + 1) %% FRAGMENTS_PER_BUFFER; %d", current_fragment);
		//debug("current send -> %x", current_fragment);
		store_settings();
		if ((current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER >= MIN_BUFFER_FILL && !stopping) {
			arch_start_move(0);
		}
	}
} // }}}

/*static void check_distance(int sp, int mt, Motor *mtr, double distance, double dt, double &factor) { // {{{
	if (dt == 0) {
		factor = 0;
		return;
	}
	if (std::isnan(distance) || distance == 0) {
		//mtr->last_v = 0;
		return;
	}
	double orig_distance = distance;
	//debug("cd %f %f", distance, dt);
	mtr->settings.target_v = distance / dt;
	double v = fabs(mtr->settings.target_v);
	int s = (mtr->settings.target_v < 0 ? -1 : 1);
	// When turning around, ignore limits (they shouldn't have been violated anyway).
	if (mtr->settings.last_v * s < 0) {
		//debug("!");
		mtr->settings.last_v = 0;
	}
	// Limit v.
	if (v > mtr->limit_v) {
		//debug("v %f limit %f", v, mtr->limit_v);
		distance = (s * mtr->limit_v) * dt;
		v = fabs(distance / dt);
	}
	//debug("cd2 %f %f", distance, dt);
	// Limit a+.
	double limit_dv = mtr->limit_a * dt;
	if (v - mtr->settings.last_v * s > limit_dv) {
		//debug("a+ target v %f limit dv %f last v %f s %d", mtr->settings.target_v, limit_dv, mtr->settings.last_v, s);
		distance = (limit_dv * s + mtr->settings.last_v) * dt;
		v = fabs(distance / dt);
	}
	//debug("cd4 %f %f", distance, dt); * /
	int steps = arch_round_pos(sp, mt, mtr->settings.current_pos + distance * mtr->steps_per_unit) - round(mtr->settings.current_pos);
	int targetsteps = steps;
	//cpdebug(s, m, "cf %d value %d", current_fragment, value);
	if (settings.probing && steps)
		steps = s;
	else {
		// Maximum depends on number of subfragments.  Be conservative and use 0x7e per subfragment; assume 128 subfragments (largest power of 2 smaller than 10000/75)
		// TODO: 10000 and 75 should follow the actual values for step_time in cdriver and TIME_PER_ISR in firmware.
		int max = 0x7e << 7;
		if (abs(steps) > max) {
			debug("overflow %d from cp %f dist %f steps/mm %f dt %f s %d max %d", steps, mtr->settings.current_pos, distance, mtr->steps_per_unit, dt, s, max);
			steps = max * s;
		}
	}
	if (abs(steps) < abs(targetsteps)) {
		distance = (arch_round_pos(sp, mt, mtr->settings.current_pos) + steps + s * .5 - mtr->settings.current_pos) / mtr->steps_per_unit;
		v = fabs(distance / dt);
	}
	//debug("=============");
	double f = distance / orig_distance;
	//debug("checked %f %f %f %f", mtr->settings.target_dist, distance, factor, f);
	if (f < factor)
		factor = f;
} */// }}}

static double move_axes(Space *s) { // {{{
	bool ok = true;
	space_types[s->type].xyz2motors(s);
	// Try again if it didn't work; it should have moved target to a better location.
	if (!ok) {
		space_types[s->type].xyz2motors(s);
		movedebug("retried move");
	}
	return 1;
	/*
	//movedebug("ok %d", ok);
	double factor = 1;
	for (int m = 0; m < s->num_motors; ++m) {
		//if (s->id == 0 && m == 0)
			//debug("check move %d %d target %f current %f", s->id, m, s->motor[m]->settings.target_pos, s->motor[m]->settings.current_pos / s->motor[m]->steps_per_unit);
		double distance = s->motor[m]->settings.target_pos - s->motor[m]->settings.current_pos / s->motor[m]->steps_per_unit;
		check_distance(s->id, m, s->motor[m], distance, (settings.hwtime - settings.last_time) / 1e6, factor);
	}
	return factor; // */
} // }}}

static void do_steps() { // {{{
	// Do the steps to arrive at the correct position. Update axis and motor current positions.
	//debug("steps");
	// Set new current position.
	for (int s = 0; s < NUM_SPACES; ++s) {
		if (!settings.single && s == 2)
			continue;
		Space &sp = spaces[s];
		for (int a = 0; a < sp.num_axes; ++a) {
			if (!std::isnan(sp.axis[a]->settings.target)) {
				sp.axis[a]->settings.current = sp.axis[a]->settings.target;
			}
		}
	}
	// Move the motors.
	//debug("start move");
	for (int s = 0; s < NUM_SPACES; ++s) {
		if (!settings.single && s == 2)
			continue;
		Space &sp = spaces[s];
		for (int m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			double target = mtr.settings.target_pos;
			if (std::isnan(target))
				continue;
			cpdebug(s, m, "ccp3 stopping %d target %f lastv %f spm %f frag %d", stopping, target, mtr.settings.last_v, mtr.steps_per_unit, current_fragment);
			double new_cp = target * mtr.steps_per_unit;
			double rounded_cp = arch_round_pos(s, m, mtr.settings.current_pos);
			double rounded_new_cp = arch_round_pos(s, m, new_cp);
			if (rounded_cp != rounded_new_cp) {
				if (!mtr.active) {
					mtr.active = true;
					//debug("activating motor %d %d", s, m);
					num_active_motors += 1;
				}
				int diff = round(rounded_new_cp - rounded_cp);
				movedebug("sending %d %d steps %d", s, m, diff);
				DATA_SET(s, m, diff);
			}
			//debug("new cp: %d %d %f %d", s, m, new_cp, current_fragment_pos);
			if (!settings.single) {
				for (int mm = 0; mm < spaces[2].num_motors; ++mm) {
					int fm = space_types[spaces[2].type].follow(&spaces[2], mm);
					if (fm < 0)
						continue;
					int fs = fm >> 8;
					fm &= 0x7f;
					if (fs != s || fm != m || (fs == 2 && fm >= mm))
						continue;
					//debug("follow %d %d %d %d %d %f %f", s, m, fs, fm, mm, new_cp, mtr.settings.current_pos);
					spaces[2].motor[mm]->settings.current_pos += new_cp - mtr.settings.current_pos;
				}
			}
			mtr.settings.current_pos = new_cp;
			cpdebug(s, m, "cp three %f", target);
			mtr.settings.last_v = mtr.settings.target_v;
		}
	}
	current_fragment_pos += 1;
	if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
		//debug("fragment full %d %d %d", computing_move, current_fragment_pos, BYTES_PER_FRAGMENT);
		send_fragment();
	}
} // }}}

static double set_targets(double factor) { // {{{
	if (spaces[0].num_axes > 0) {
		double factor2 = 2 * factor - 1;
		double alpha = factor2 * settings.alpha_max;
		double denominator = sin(settings.alpha_max);
		double a = (denominator < 1e-10 ? factor2 : sin(alpha) / denominator);
		double cmax = cos(settings.alpha_max);
		double b = (cos(alpha) - cmax) / (1 - cmax);
		if (isnan(b))
			b = 1 - std::abs(factor2);	// Doesn't really matter; B == {0, 0, 0}.
		for (int i = 0; i < 3; ++i) {
			if (i < spaces[0].num_axes) {
				spaces[0].axis[i]->settings.target = settings.P[i] + a * settings.A[i] + b * settings.B[i];
				//debug("target %f P %f a %f A %f B %f amax %f factor2 %f", spaces[0].axis[i]->settings.target, settings.P[i],a, settings.A[i], settings.B[i], settings.alpha_max, factor2);
			}
		}
	}
	double min_f = 1;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		for (int a = (s == 0 ? 3 : 0); a < sp.num_axes; ++a) {
			auto ax = sp.axis[a];
			ax->settings.target = ax->settings.source + factor * (ax->settings.target - ax->settings.source);
		}
		double f = move_axes(&sp);
		if (min_f > f)
			min_f = f;
	}
	return min_f;
} // }}}

static void handle_motors() { // {{{
	// Check for move.
	if (!computing_move) {
		movedebug("handle motors not moving");
		return;
	}
	movedebug("handling %d %d", computing_move, cbs_after_current_move);
	while ((running_fragment - 1 - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER > (FRAGMENTS_PER_BUFFER > 4 ? 4 : FRAGMENTS_PER_BUFFER - 2)) {
		double t = settings.hwtime / 1e6;
		double target_factor;
		if (settings.hwtime >= settings.end_time)
			target_factor = 1;
		else if (settings.hwtime <= 0)
			target_factor = 0;
		else
			target_factor = ((settings.v1 - settings.v0) / (settings.end_time / 1e6) * t * t / 2 + settings.v0 * t) / settings.dist;
		movedebug("target factor: %f", target_factor);
		if (!std::isnan(target_factor)) { // Go straight to the next move if the distance was 0.
			double f = set_targets(target_factor);
			if (f < 1) {
				target_factor = settings.factor + f * (target_factor - settings.factor);
				// TODO: Adjust time.
				//settings.start_time += (settings.hwtime - the_last_time) * ((1 - factor) * .99);
			}
			else if (target_factor > 1) {
				target_factor = 1;
			}
			//debug("target factor %f time 0 -> %d -> %d v %f -> %f", target_factor, settings.hwtime, settings.end_time, settings.v0, settings.v1);
			set_targets(target_factor);
			settings.factor = target_factor;
			if (settings.factor < 1) {
				do_steps();
				settings.last_time = settings.hwtime;
				return;
			}
		}
		movedebug("next segment");
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int a = 0; a < sp.num_axes; ++a) {
				auto ax = sp.axis[a];
				ax->settings.source = ax->settings.target;
			}
		}
		// start new move; adjust time.
		num_movecbs = cbs_after_current_move;
		//debug("adding %d cbs because move is completed", cbs_after_current_move);
		cbs_after_current_move = next_move(settings.end_time);
		movedebug("next move prepared");
		if (!computing_move) {
			// There is no next move.
			continue_event = true;
			settings.last_time = settings.hwtime;
			do_steps();
			return;
		}
		movedebug("try again");
	}
} // }}}

static void apply_tick() { // {{{
	//debug("tick");
	settings.hwtime += hwtime_step;
	if (current_fragment_pos < SAMPLES_PER_FRAGMENT)
		handle_motors();
	//if (spaces[0].num_axes >= 2)
		//debug("move z %d %d %f %f %f", current_fragment, current_fragment_pos, spaces[0].axis[2]->settings.current, spaces[0].motor[0]->settings.current_pos, spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0]);
} // }}}

void store_settings() { // {{{
	current_fragment_pos = 0;
	num_active_motors = 0;
	if (FRAGMENTS_PER_BUFFER == 0)
		return;
	for (int i = 0; i < 3; ++i) {
		history[current_fragment].P[i] = settings.P[i];
		history[current_fragment].A[i] = settings.A[i];
		history[current_fragment].B[i] = settings.B[i];
	}
	history[current_fragment].v0 = settings.v0;
	history[current_fragment].v1 = settings.v1;
	history[current_fragment].dist = settings.dist;
	history[current_fragment].alpha_max = settings.alpha_max;
	history[current_fragment].hwtime = settings.hwtime;
	history[current_fragment].end_time = settings.end_time;
	history[current_fragment].last_time = settings.last_time;
	history[current_fragment].cbs = 0;
	history[current_fragment].queue_start = settings.queue_start;
	history[current_fragment].queue_end = settings.queue_end;
	history[current_fragment].queue_full = settings.queue_full;
	history[current_fragment].run_file_current = settings.run_file_current;
	history[current_fragment].probing = settings.probing;
	history[current_fragment].single = settings.single;
	history[current_fragment].run_time = settings.run_time;
	history[current_fragment].run_dist = settings.run_dist;
	history[current_fragment].factor = settings.factor;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		sp.history[current_fragment].dist[0] = sp.settings.dist[0];
		sp.history[current_fragment].dist[1] = sp.settings.dist[1];
		for (int i = 0; i < 2; ++i) {
			sp.history[current_fragment].arc[i] = sp.settings.arc[i];
			sp.history[current_fragment].angle[i] = sp.settings.angle[i];
			sp.history[current_fragment].helix[i] = sp.settings.helix[i];
			for (int t = 0; t < 2; ++t)
				sp.history[current_fragment].radius[i][t] = sp.settings.radius[i][t];
			for (int t = 0; t < 3; ++t) {
				sp.history[current_fragment].offset[i][t] = sp.settings.offset[i][t];
				sp.history[current_fragment].e1[i][t] = sp.settings.e1[i][t];
				sp.history[current_fragment].e2[i][t] = sp.settings.e2[i][t];
				sp.history[current_fragment].normal[i][t] = sp.settings.normal[i][t];
			}
		}
		for (int m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->active = false;
			sp.motor[m]->history[current_fragment].last_v = sp.motor[m]->settings.last_v;
			sp.motor[m]->history[current_fragment].target_v = sp.motor[m]->settings.target_v;
			sp.motor[m]->history[current_fragment].target_pos = sp.motor[m]->settings.target_pos;
			sp.motor[m]->history[current_fragment].current_pos = sp.motor[m]->settings.current_pos;
			cpdebug(s, m, "store");
		}
		for (int a = 0; a < sp.num_axes; ++a) {
			sp.axis[a]->history[current_fragment].dist[0] = sp.axis[a]->settings.dist[0];
			sp.axis[a]->history[current_fragment].dist[1] = sp.axis[a]->settings.dist[1];
			sp.axis[a]->history[current_fragment].main_dist = sp.axis[a]->settings.main_dist;
			sp.axis[a]->history[current_fragment].target = sp.axis[a]->settings.target;
			sp.axis[a]->history[current_fragment].source = sp.axis[a]->settings.source;
			sp.axis[a]->history[current_fragment].current = sp.axis[a]->settings.current;
			sp.axis[a]->history[current_fragment].endpos[0] = sp.axis[a]->settings.endpos[0];
			sp.axis[a]->history[current_fragment].endpos[1] = sp.axis[a]->settings.endpos[1];
		}
	}
	DATA_CLEAR();
} // }}}

void restore_settings() { // {{{
	current_fragment_pos = 0;
	num_active_motors = 0;
	if (FRAGMENTS_PER_BUFFER == 0)
		return;
	for (int i = 0; i < 3; ++i) {
		settings.P[i] = history[current_fragment].P[i];
		settings.A[i] = history[current_fragment].A[i];
		settings.B[i] = history[current_fragment].B[i];
	}
	settings.v0 = history[current_fragment].v0;
	settings.v1 = history[current_fragment].v1;
	settings.dist = history[current_fragment].dist;
	settings.alpha_max = history[current_fragment].alpha_max;
	settings.hwtime = history[current_fragment].hwtime;
	settings.end_time = history[current_fragment].end_time;
	settings.last_time = history[current_fragment].last_time;
	history[current_fragment].cbs = 0;
	settings.queue_start = history[current_fragment].queue_start;
	settings.queue_end = history[current_fragment].queue_end;
	settings.queue_full = history[current_fragment].queue_full;
	settings.run_file_current = history[current_fragment].run_file_current;
	settings.probing = history[current_fragment].probing;
	settings.single = history[current_fragment].single;
	settings.run_time = history[current_fragment].run_time;
	settings.run_dist = history[current_fragment].run_dist;
	settings.factor = history[current_fragment].factor;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		sp.settings.dist[0] = sp.history[current_fragment].dist[0];
		sp.settings.dist[1] = sp.history[current_fragment].dist[1];
		for (int i = 0; i < 2; ++i) {
			sp.settings.arc[i] = sp.history[current_fragment].arc[i];
			sp.settings.angle[i] = sp.history[current_fragment].angle[i];
			sp.settings.helix[i] = sp.history[current_fragment].helix[i];
			for (int t = 0; t < 2; ++t)
				sp.settings.radius[i][t] = sp.history[current_fragment].radius[i][t];
			for (int t = 0; t < 3; ++t) {
				sp.settings.offset[i][t] = sp.history[current_fragment].offset[i][t];
				sp.settings.e1[i][t] = sp.history[current_fragment].e1[i][t];
				sp.settings.e2[i][t] = sp.history[current_fragment].e2[i][t];
				sp.settings.normal[i][t] = sp.history[current_fragment].normal[i][t];
			}
		}
		for (int m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->active = false;
			sp.motor[m]->settings.last_v = sp.motor[m]->history[current_fragment].last_v;
			sp.motor[m]->settings.target_v = sp.motor[m]->history[current_fragment].target_v;
			sp.motor[m]->settings.target_pos = sp.motor[m]->history[current_fragment].target_pos;
			sp.motor[m]->settings.current_pos = sp.motor[m]->history[current_fragment].current_pos;
			cpdebug(s, m, "restore");
		}
		for (int a = 0; a < sp.num_axes; ++a) {
			sp.axis[a]->settings.dist[0] = sp.axis[a]->history[current_fragment].dist[0];
			sp.axis[a]->settings.dist[1] = sp.axis[a]->history[current_fragment].dist[1];
			sp.axis[a]->settings.main_dist = sp.axis[a]->history[current_fragment].main_dist;
			sp.axis[a]->settings.target = sp.axis[a]->history[current_fragment].target;
			sp.axis[a]->settings.source = sp.axis[a]->history[current_fragment].source;
			sp.axis[a]->settings.current = sp.axis[a]->history[current_fragment].current;
			sp.axis[a]->settings.endpos[0] = sp.axis[a]->history[current_fragment].endpos[0];
			sp.axis[a]->settings.endpos[1] = sp.axis[a]->history[current_fragment].endpos[1];
		}
	}
	DATA_CLEAR();
} // }}}

void buffer_refill() { // {{{
	//debug("refill");
	if (preparing || FRAGMENTS_PER_BUFFER == 0) {
		//debug("no refill because prepare");
		return;
	}
	if (!computing_move || refilling || stopping || discard_pending || discarding) {
		//debug("refill block %d %d %d %d %d", computing_move, refilling, stopping, discard_pending, discarding);
		return;
	}
	refilling = true;
	// send_fragment in the previous refill may have failed; try it again.
	if (current_fragment_pos > 0)
		send_fragment();
	//debug("refill start %d %d %d", running_fragment, current_fragment, sending_fragment);
	// Keep one free fragment, because we want to be able to rewind and use the buffer before the one currently active.
	while (computing_move && !stopping && !discard_pending && !discarding && (running_fragment - 1 - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER > (FRAGMENTS_PER_BUFFER > 4 ? 4 : FRAGMENTS_PER_BUFFER - 2) && !sending_fragment) {
		//debug("refill %d %d %f", current_fragment, current_fragment_pos, spaces[0].motor[0]->settings.current_pos);
		// fill fragment until full.
		apply_tick();
		//debug("refill2 %d %f", current_fragment, spaces[0].motor[0]->settings.current_pos);
		if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
			//debug("fragment full %d %d %d", computing_move, current_fragment_pos, BYTES_PER_FRAGMENT);
			send_fragment();
		}
		// Check for commands from host; in case of many short buffers, this loop may not end in a reasonable time.
		//serial(0);
	}
	if (stopping || discard_pending) {
		//debug("aborting refill for stopping");
		refilling = false;
		return;
	}
	if (!computing_move && current_fragment_pos > 0) {
		//debug("finalize");
		send_fragment();
	}
	refilling = false;
	arch_start_move(0);
} // }}}

void abort_move(int pos) { // {{{
	aborting = true;
	//debug("abort pos %d", pos);
	//debug("abort; cf %d rf %d first %d computing_move %d fragments, regenerating %d ticks", current_fragment, running_fragment, first_fragment, computing_move, pos);
	//debug("try aborting move");
	current_fragment = running_fragment;
	//debug("current_fragment = running_fragment; %d", current_fragment);
	//debug("current abort -> %x", current_fragment);
	while (pos < 0) {
		if (current_fragment == first_fragment) {
			pos = 0;
		}
		else {
			current_fragment = (current_fragment + FRAGMENTS_PER_BUFFER - 1) % FRAGMENTS_PER_BUFFER;
			//debug("current_fragment = (current_fragment + FRAGMENTS_PER_BUFFER - 1) %% FRAGMENTS_PER_BUFFER; %d", current_fragment);
			pos += SAMPLES_PER_FRAGMENT;
			running_fragment = current_fragment;
		}
	}
	restore_settings();
#ifdef DEBUG_MOVE
	debug("move no longer prepared");
#endif
	//debug("free abort reset");
	current_fragment_pos = 0;
	computing_move = true;
	while (computing_move && current_fragment_pos < unsigned(pos)) {
		//debug("abort reconstruct %d %d", current_fragment_pos, pos);
		apply_tick();
	}
	if (spaces[0].num_axes > 0)
		cpdebug(0, 0, "ending hwpos %f", arch_round_pos(0, 0, spaces[0].motor[0]->settings.current_pos) + avr_pos_offset[0]);
	// Copy settings back to previous fragment.
	store_settings();
	computing_move = false;
	prepared = false;
	current_fragment_pos = 0;
	for (int s = 0; s < NUM_SPACES; ++s) {
		Space &sp = spaces[s];
		sp.settings.dist[0] = 0;
		sp.settings.dist[1] = 0;
		for (int a = 0; a < sp.num_axes; ++a) {
			//debug("setting axis %d source to %f", a, sp.axis[a]->settings.current);
			if (!std::isnan(sp.axis[a]->settings.current))
				sp.axis[a]->settings.source = sp.axis[a]->settings.current;
			sp.axis[a]->settings.dist[0] = NAN;
			sp.axis[a]->settings.dist[1] = NAN;
		}
		for (int m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->settings.last_v = 0;
			//debug("setting motor %d pos to %f", m, sp.motor[m]->settings.current_pos);
		}
	}
	//debug("aborted move");
	aborting = false;
} // }}}
// }}}

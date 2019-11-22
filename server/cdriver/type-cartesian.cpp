/* type-cartesian.cpp - Cartesian, extruder and follower geometry handling for Franklin
 * vim: set foldmethod=marker :
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

// Cartesian functions. {{{
static void xyz2motors(Space *s) { // {{{
	for (uint8_t a = 0; a < s->num_axes; ++a) {
		s->motor[a]->target_pos = s->axis[a]->target;
	}
} // }}}

static void motors2xyz(Space *s, const double *motors, double *xyz) { // {{{
	for (uint8_t a = 0; a < s->num_axes; ++a)
		xyz[a] = motors[a];
} // }}}

static void check_position(Space *s, double *data) { // {{{
	(void)&s;
	(void)&data;
} // }}}

static void load_space(Space *s) { // {{{
	shmem->ints[3] = shmem->ints[2];
	if (!s->setup_nums(shmem->ints[2], shmem->ints[3])) {
		debug("Failed to set up cartesian axes");
		s->cancel_update();
	}
} // }}}

static void load_axis(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
} // }}}

static void load_motor(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static void save(Space *s) { // {{{
	(void)&s;
	// Record 0 items for modules that don't implement this.
	shmem->ints[100] = 0;
	shmem->ints[101] = 0;
} // }}}

static void asave(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
	// Record 0 items for modules that don't implement this.
	shmem->ints[100] = 0;
	shmem->ints[101] = 0;
} // }}}

static void msave(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
	// Record 0 items for modules that don't implement this.
	shmem->ints[100] = 0;
	shmem->ints[101] = 0;
} // }}}

static bool init(Space *s) { // {{{
	(void)&s;
	return true;
} // }}}

static void init_axis(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
} // }}}

static void init_motor(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static void free_space(Space *s) { // {{{
	(void)&s;
} // }}}

static void free_axis(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
} // }}}

static void free_motor(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static double change0(Space *s, int axis, double value) { // {{{
	(void)&s;
	(void)&axis;
	return value;
} // }}}

static double unchange0(Space *s, int axis, double value) { // {{{
	(void)&s;
	(void)&axis;
	return value;
} // }}}

static double probe_speed(Space *s) { // {{{
	if (s->num_motors >= 3)
		return 1e6 / settings.hwtime_step / s->motor[2]->steps_per_unit;
	return INFINITY;
} // }}}

static int follow(Space *s, int axis) { // {{{
	(void)&s;
	(void)&axis;
	return -1;
} // }}}

void Cartesian_init(int num) { // {{{
	space_types[num].xyz2motors = xyz2motors;
	space_types[num].motors2xyz = motors2xyz;
	space_types[num].check_position = check_position;
	space_types[num].init_space = init;
	space_types[num].init_axis = init_axis;
	space_types[num].init_motor = init_motor;
	space_types[num].load_space = load_space;
	space_types[num].load_axis = load_axis;
	space_types[num].load_motor = load_motor;
	space_types[num].save_space = save;
	space_types[num].save_axis = asave;
	space_types[num].save_motor = msave;
	space_types[num].free_space = free_space;
	space_types[num].free_axis = free_axis;
	space_types[num].free_motor = free_motor;
	space_types[num].change0 = change0;
	space_types[num].unchange0 = unchange0;
	space_types[num].probe_speed = probe_speed;
	space_types[num].follow = follow;
} // }}}
// }}}

// Extruder functions. {{{
struct ExtruderAxisData { // {{{
	double offset[3];
}; // }}}

#define EADATA(s, a) (*reinterpret_cast <ExtruderAxisData *>(s->axis[a]->type_data))

static void eload(Space *s) { // {{{
	shmem->ints[3] = shmem->ints[2];
	if (!s->setup_nums(shmem->ints[2], shmem->ints[3])) {
		debug("Failed to set up extruder axes");
		uint8_t n = min(s->num_axes, s->num_motors);
		if (!s->setup_nums(n, n)) {
			debug("Trouble!  Failed to abort.  Cancelling.");
			s->cancel_update();
		}
	}
} // }}}

static void eaload(Space *s, int a) { // {{{
	for (int o = 0; o < 3; ++o)
		EADATA(s, a).offset[o] = shmem->floats[100 + o];
} // }}}

static void emload(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static void esave(Space *s) { // {{{
	(void)&s;
} // }}}

static void easave(Space *s, int a) { // {{{
	for (int o = 0; o < 3; ++o)
		shmem->floats[100 + o] = EADATA(s, a).offset[o];
} // }}}

static void emsave(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static bool einit(Space *s) { // {{{
	(void)&s;
	return true;
} // }}}

static void eainit(Space *s, int a) { // {{{
	s->axis[a]->type_data = new ExtruderAxisData;
	for (int i = 0; i < 3; ++i) {
		EADATA(s, a).offset[i] = 0;
	}
} // }}}

static void eminit(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static void efree(Space *s) { // {{{
	(void)&s;
} // }}}

static void eafree(Space *s, int a) { // {{{
	delete reinterpret_cast <ExtruderAxisData *>(s->axis[a]->type_data);
} // }}}

static void emfree(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static double echange0(Space *s, int axis, double value) { // {{{
	if (current_extruder >= s->num_axes || axis >= 3)
		return value;
	return value + EADATA(s, current_extruder).offset[axis];
} // }}}

static double eunchange0(Space *s, int axis, double value) { // {{{
	if (current_extruder >= s->num_axes || axis >= 3)
		return value;
	return value - EADATA(s, current_extruder).offset[axis];
} // }}}

void Extruder_init(int num) { // {{{
	space_types[num].xyz2motors = xyz2motors;
	space_types[num].check_position = check_position;
	space_types[num].init_space = einit;
	space_types[num].init_axis = eainit;
	space_types[num].init_motor = eminit;
	space_types[num].load_space = eload;
	space_types[num].load_axis = eaload;
	space_types[num].load_motor = emload;
	space_types[num].save_space = esave;
	space_types[num].save_axis = easave;
	space_types[num].save_motor = emsave;
	space_types[num].free_space = efree;
	space_types[num].free_axis = eafree;
	space_types[num].free_motor = emfree;
	space_types[num].change0 = echange0;
	space_types[num].unchange0 = eunchange0;
	space_types[num].probe_speed = probe_speed;
	space_types[num].follow = follow;
	space_types[num].motors2xyz = motors2xyz;
} // }}}
// }}}

// Follower functions. {{{
struct FollowerData { // {{{
	int num_axes;
}; // }}}

struct FollowerAxisData { // {{{
	int space, motor;
}; // }}}

#define FDATA(s) (*reinterpret_cast <FollowerData *>(s->type_data))
#define FADATA(s, a) (*reinterpret_cast <FollowerAxisData *>(s->axis[a]->type_data))

static void fload(Space *s) { // {{{
	shmem->ints[3] = shmem->ints[2];
	if (!s->setup_nums(shmem->ints[2], shmem->ints[3])) {
		debug("Failed to set up follower axes");
		uint8_t n = min(s->num_axes, s->num_motors);
		if (!s->setup_nums(n, n)) {
			debug("Trouble!  Failed to abort.  Cancelling.");
			s->cancel_update();
		}
	}
	for (int a = FDATA(s).num_axes; a < s->num_axes; ++a) {
		s->axis[a]->type_data = new FollowerAxisData;
		FADATA(s, a).space = -1;
		FADATA(s, a).motor = -1;
	}
	FDATA(s).num_axes = s->num_axes;
	for (int a = 0; a < s->num_axes; ++a) {
		FADATA(s, a).space = shmem->ints[4 + 2 * a];
		FADATA(s, a).motor = shmem->ints[4 + 2 * a + 1];
	}
	arch_motors_change();
} // }}}

static void faload(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
} // }}}

static void fmload(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static void fsave(Space *s) { // {{{
	for (int a = 0; a < s->num_axes; ++a) {
		shmem->ints[4 + 2 * a] = FADATA(s, a).space;
		shmem->ints[4 + 2 * a + 1] = FADATA(s, a).motor;
	}
} // }}}

static void fasave(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
} // }}}

static void fmsave(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static bool finit(Space *s) { // {{{
	s->type_data = new FollowerData;
	FDATA(s).num_axes = 0;
	return true;
} // }}}

static void fainit(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
} // }}}

static void fminit(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static void ffree(Space *s) { // {{{
	delete reinterpret_cast <FollowerData *>(s->type_data);
} // }}}

static void fafree(Space *s, int a) { // {{{
	delete reinterpret_cast <FollowerAxisData *>(s->axis[a]->type_data);
} // }}}

static void fmfree(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

static int ffollow(Space *s, int axis) { // {{{
	if (axis < 0 || axis >= FDATA(s).num_axes)
		return -1;
	int fs = FADATA(s, axis).space;
	if (fs >= 0 && fs < NUM_SPACES) {
		int fm = FADATA(s, axis).motor;
		if (fm >= 0 && fm < spaces[fs].num_motors)
			return (fs << 8) | fm;
	}
	return -1;
} // }}}

void Follower_init(int num) { // {{{
	space_types[num].xyz2motors = xyz2motors;
	space_types[num].check_position = check_position;
	space_types[num].init_space = finit;
	space_types[num].init_axis = fainit;
	space_types[num].init_motor = fminit;
	space_types[num].load_space = fload;
	space_types[num].load_axis = faload;
	space_types[num].load_motor = fmload;
	space_types[num].save_space = fsave;
	space_types[num].save_axis = fasave;
	space_types[num].save_motor = fmsave;
	space_types[num].free_space = ffree;
	space_types[num].free_axis = fafree;
	space_types[num].free_motor = fmfree;
	space_types[num].change0 = change0;
	space_types[num].unchange0 = unchange0;
	space_types[num].probe_speed = probe_speed;
	space_types[num].follow = ffollow;
	space_types[num].motors2xyz = motors2xyz;
} // }}}
// }}}

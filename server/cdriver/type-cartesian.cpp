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

static void load(Space *s) { // {{{
	shmem->ints[3] = shmem->ints[2];
	if (!s->setup_nums(shmem->ints[2], shmem->ints[3])) {
		debug("Failed to set up cartesian axes");
		s->cancel_update();
	}
} // }}}

static void save(Space *s) { // {{{
	(void)&s;
} // }}}

static bool init(Space *s) { // {{{
	(void)&s;
	return true;
} // }}}

static void free(Space *s) { // {{{
	(void)&s;
} // }}}

static void afree(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
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
	space_types[num].check_position = check_position;
	space_types[num].load = load;
	space_types[num].save = save;
	space_types[num].init = init;
	space_types[num].free = free;
	space_types[num].afree = afree;
	space_types[num].change0 = change0;
	space_types[num].unchange0 = unchange0;
	space_types[num].probe_speed = probe_speed;
	space_types[num].follow = follow;
	space_types[num].motors2xyz = motors2xyz;
} // }}}
// }}}

// Extruder functions. {{{
struct ExtruderData { // {{{
	int num_axes;
}; // }}}

struct ExtruderAxisData { // {{{
	double offset[3];
}; // }}}

#define EDATA(s) (*reinterpret_cast <ExtruderData *>(s->type_data))
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
	for (int a = EDATA(s).num_axes; a < s->num_axes; ++a) {
		s->axis[a]->type_data = new ExtruderAxisData;
		for (int i = 0; i < 3; ++i) {
			EADATA(s, a).offset[i] = 0;
		}
	}
	EDATA(s).num_axes = s->num_axes;
	for (int a = 0; a < s->num_axes; ++a) {
		for (int o = 0; o < 3; ++o) {
			EADATA(s, a).offset[o] = shmem->floats[3 * a + o];
			//debug("load offset %d %d %d = %f", s->id, a, o, EADATA(s, a).offset[o]);
		}
	}
} // }}}

static void esave(Space *s) { // {{{
	for (int a = 0; a < s->num_axes; ++a) {
		for (int o = 0; o < 3; ++o)
			shmem->floats[3 * a + o] = EADATA(s, a).offset[o];
	}
} // }}}

static bool einit(Space *s) { // {{{
	s->type_data = new ExtruderData;
	EDATA(s).num_axes = 0;
	return true;
} // }}}

static void efree(Space *s) { // {{{
	delete reinterpret_cast <ExtruderData *>(s->type_data);
} // }}}

static void eafree(Space *s, int a) { // {{{
	delete reinterpret_cast <ExtruderAxisData *>(s->axis[a]->type_data);
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
	space_types[num].load = eload;
	space_types[num].save = esave;
	space_types[num].init = einit;
	space_types[num].free = efree;
	space_types[num].afree = eafree;
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

static void fsave(Space *s) { // {{{
	for (int a = 0; a < s->num_axes; ++a) {
		shmem->ints[4 + 2 * a] = FADATA(s, a).space;
		shmem->ints[4 + 2 * a + 1] = FADATA(s, a).motor;
	}
} // }}}

static bool finit(Space *s) { // {{{
	s->type_data = new FollowerData;
	FDATA(s).num_axes = 0;
	return true;
} // }}}

static void ffree(Space *s) { // {{{
	delete reinterpret_cast <FollowerData *>(s->type_data);
} // }}}

static void fafree(Space *s, int a) { // {{{
	delete reinterpret_cast <FollowerAxisData *>(s->axis[a]->type_data);
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
	space_types[num].load = fload;
	space_types[num].save = fsave;
	space_types[num].init = finit;
	space_types[num].free = ffree;
	space_types[num].afree = fafree;
	space_types[num].change0 = change0;
	space_types[num].unchange0 = unchange0;
	space_types[num].probe_speed = probe_speed;
	space_types[num].follow = ffollow;
	space_types[num].motors2xyz = motors2xyz;
} // }}}
// }}}

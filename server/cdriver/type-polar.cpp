/* type-polar.cpp - Polar geometry handling for Franklin
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

struct Polar_private {
	double max_r;
};

#define PRIVATE(s) (*reinterpret_cast <Polar_private *>(s->type_data))

static void xyz2motors(Space *s) {
	if (std::isnan(s->axis[0]->target) || std::isnan(s->axis[1]->target)) {
		// Fill up missing targets.
		for (uint8_t aa = 0; aa < 2; ++aa) {
			if (std::isnan(s->axis[aa]->target))
				s->axis[aa]->target = s->axis[aa]->current;
		}
	}
	double x = s->axis[0]->target;
	double y = s->axis[1]->target;
	double z = s->axis[2]->target;
	double r = sqrt(x * x + y * y);
	double theta = atan2(y, x);
	while (theta - s->motor[1]->settings.current_pos > 2 * M_PI)
		theta -= 2 * M_PI;
	while (theta - s->motor[1]->settings.current_pos < -2 * M_PI)
		theta += 2 * M_PI;
	s->motor[0]->target_pos = r;
	s->motor[1]->target_pos = theta;
	s->motor[2]->target_pos = z;
}

static void motors2xyz(Space *s, const double motors[3], double xyz[3]) {
	(void)&s;
	xyz[0] = motors[0] * cos(motors[1]);
	xyz[1] = motors[0] * sin(motors[1]);
	xyz[2] = motors[2];
}

static void check_position(Space *s, double *data) {
	double r = sqrt(data[0] * data[0] + data[1] * data[1]);
	if (r <= PRIVATE(s).max_r)
		return;
	data[0] *= PRIVATE(s).max_r / r;
	data[1] *= PRIVATE(s).max_r / r;
}

static void load(Space *s) {
	if (!s->setup_nums(3, 3)) {
		debug("Failed to set up polar axes");
		s->cancel_update();
		return;
	}
	PRIVATE(s).max_r = shmem->floats[0];
}

static void save(Space *s) {
	shmem->floats[0] = PRIVATE(s).max_r;
}

static bool init(Space *s) {
	s->type_data = new Polar_private;
	if (!s->type_data)
		return false;
	return true;
}

static void free(Space *s) {
	delete reinterpret_cast <Polar_private *>(s->type_data);
}

static void afree(Space *s, int a) {
	(void)&s;
	(void)&a;
}

static double change0(Space *s, int axis, double value) {
	(void)&s;
	(void)&axis;
	return value;
}

static double unchange0(Space *s, int axis, double value) {
	(void)&s;
	(void)&axis;
	return value;
}

static double probe_speed(Space *s) {
	return 1e6 / settings.hwtime_step / s->motor[2]->steps_per_unit;
}

static int follow(Space *s, int axis) {
	(void)&s;
	(void)&axis;
	return -1;
}

void Polar_init(int num) {
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
}

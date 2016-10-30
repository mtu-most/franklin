/* type-delta.cpp - Delta geometry handling for Franklin
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

struct Apex {
	double axis_min, axis_max;	// Limits for the movement of this axis.
	double rodlength, radius;	// Length of the tie rod and the horizontal distance between the vertical position and the zero position.
	double x, y, z;		// Position of tower on the base plane, and the carriage height at zero position.
};

struct Delta_private {
	Apex apex[3];
	double angle;			// Adjust the front of the printer.
};

#define PRIVATE(s) (*reinterpret_cast <Delta_private *>(s->type_data))
#define APEX(s, a) (PRIVATE(s).apex[a])

static bool check_delta(Space *s, uint8_t a, double *target) {	// {{{
	double dx = target[0] - APEX(s, a).x;
	double dy = target[1] - APEX(s, a).y;
	double r2 = dx * dx + dy * dy;
	double amax = APEX(s, a).axis_max < APEX(s, a).rodlength ? APEX(s, a).axis_max : APEX(s, a).rodlength;
	if (r2 > amax * amax) {
		debug ("not ok 1: %f %f %f %f %f %f %f", target[0], target[1], dx, dy, r2, APEX(s, a).rodlength, APEX(s, a).axis_max);
		// target is too far away from axis.  Pull it towards axis so that it is on the edge.
		// target = axis + (target - axis) * (l - epsilon) / r.
		double factor(amax / sqrt(r2));
		target[0] = APEX(s, a).x + (target[0] - APEX(s, a).x) * factor;
		target[1] = APEX(s, a).y + (target[1] - APEX(s, a).y) * factor;
		return false;
	}
	// Inner product shows if projection is inside or outside the printable region.
	double projection = -(dx / APEX(s, a).radius * APEX(s, a).x + dy / APEX(s, a).radius * APEX(s, a).y);
	double amin = APEX(s, a).axis_min < -APEX(s, a).rodlength ? -APEX(s, a).rodlength : APEX(s, a).axis_min;
	if (projection < amin) {
		debug ("not ok 2: %f %f %f %f %f", projection, dx, dy, APEX(s, a).x, APEX(s, a).y);
		// target is on the wrong side of axis.  Pull it towards plane so it is on the edge.
		target[0] -= ((amin - projection) / APEX(s, a).radius - .001) * APEX(s, a).x;
		target[1] -= ((amin - projection) / APEX(s, a).radius - .001) * APEX(s, a).y;
		// Assume this was a small correction; that way, things will work even if numerical errors cause this to be called for the real move.
		return false;
	}
	//debug("ok %d %d %f", s->id, a, projection);
	return true;
}	// }}}

static inline double delta_to_axis(Space *s, uint8_t a) {
	double dx = s->axis[0]->settings.target - APEX(s, a).x;
	double dy = s->axis[1]->settings.target - APEX(s, a).y;
	double dz = s->axis[2]->settings.target - APEX(s, a).z;
	double r2 = dx * dx + dy * dy;
	double l2 = APEX(s, a).rodlength * APEX(s, a).rodlength;
	double dest = sqrt(l2 - r2) + dz;
	//debug("dta dx %f dy %f dz %f z %f, r %f target %f", dx, dy, dz, APEX(s, a).z, r, target);
	return dest;
}

static void xyz2motors(Space *s, double *motors) {
	if (isnan(s->axis[0]->settings.target) || isnan(s->axis[1]->settings.target) || isnan(s->axis[2]->settings.target)) {
		// Fill up missing targets.
		for (uint8_t aa = 0; aa < 3; ++aa) {
			if (isnan(s->axis[aa]->settings.target))
				s->axis[aa]->settings.target = s->axis[aa]->settings.current;
		}
	}
	for (uint8_t a = 0; a < 3; ++a) {
		if (motors)
			motors[a] = delta_to_axis(s, a);
		else
			s->motor[a]->settings.endpos = delta_to_axis(s, a);
	}
}

static void reset_pos (Space *s) {
	// All axes' current_pos must be valid and equal, in other words, x=y=0.
	double p[3];
	// Epsilon is average step length.
	double epsilon = 3 / (s->motor[0]->steps_per_unit + s->motor[1]->steps_per_unit + s->motor[2]->steps_per_unit);
	for (uint8_t i = 0; i < 3; ++i)
		p[i] = s->motor[i]->settings.current_pos / s->motor[i]->steps_per_unit;
	if (fabs(p[0] - p[1]) > epsilon || fabs(p[0] - p[2]) > epsilon) {
		//debug("resetpos fails");
		s->axis[0]->settings.source = NAN;
		s->axis[1]->settings.source = NAN;
		s->axis[2]->settings.source = NAN;
	}
	else {
		//debug("resetpos %f", p[0]);
		s->axis[0]->settings.source = 0;
		s->axis[1]->settings.source = 0;
		s->axis[2]->settings.source = (p[0] + p[1] + p[2]) / 3;
	}
}

static void check_position(Space *s, double *data) {
	if (isnan(data[0]) || isnan(data[1])) {
		if (!isnan(data[0]))
			data[1] = s->axis[1]->settings.source;
		else if (!isnan(data[1]))
			data[0] = s->axis[0]->settings.source;
		else {
			// Cannot check; assume it's ok.
			return;
		}
	}
	for (uint8_t counter = 0; counter < 2; ++counter) {
		bool ok = true;
		for (uint8_t a = 0; a < s->num_axes; ++a)
			ok &= check_delta(s, a, data);
		if (ok)
			break;
	}
}

static void load(Space *s, uint8_t old_type, int32_t &addr) {
	(void)&old_type;
	if (!s->setup_nums(3, 3)) {
		debug("Failed to set up delta axes");
		s->cancel_update();
		return;
	}
	for (uint8_t a = 0; a < 3; ++a) {
		APEX(s, a).axis_min = read_float(addr);
		APEX(s, a).axis_max = read_float(addr);
		APEX(s, a).rodlength = read_float(addr);
		APEX(s, a).radius = read_float(addr);
	}
	PRIVATE(s).angle = read_float(addr);
	if (isinf(PRIVATE(s).angle) || isnan(PRIVATE(s).angle))
		PRIVATE(s).angle = 0;
#define sin210 -.5
#define cos210 -0.8660254037844386	// .5*sqrt(3)
#define sin330 -.5
#define cos330 0.8660254037844386	// .5*sqrt(3)
#define sin90 1
	// Coordinates of axes (at angles 210, 330, 90; each with its own radius).
	double x[3], y[3];
	x[0] = APEX(s, 0).radius * cos210;
	y[0] = APEX(s, 0).radius * sin210;
	x[1] = APEX(s, 1).radius * cos330;
	y[1] = APEX(s, 1).radius * sin330;
	x[2] = 0;
	y[2] = APEX(s, 2).radius * sin90;
	for (uint8_t a = 0; a < 3; ++a) {
		APEX(s, a).x = x[a] * cos(PRIVATE(s).angle) - y[a] * sin(PRIVATE(s).angle);
		APEX(s, a).y = y[a] * cos(PRIVATE(s).angle) + x[a] * sin(PRIVATE(s).angle);
		APEX(s, a).z = sqrt(APEX(s, a).rodlength * APEX(s, a).rodlength - APEX(s, a).radius * APEX(s, a).radius);
	}
}

static void save(Space *s, int32_t &addr) {
	for (uint8_t a = 0; a < 3; ++a) {
		write_float(addr, APEX(s, a).axis_min);
		write_float(addr, APEX(s, a).axis_max);
		write_float(addr, APEX(s, a).rodlength);
		write_float(addr, APEX(s, a).radius);
	}
	write_float(addr, PRIVATE(s).angle);
}

static bool init(Space *s) {
	s->type_data = new Delta_private;
	if (!s->type_data)
		return false;
	return true;
}

static void free(Space *s) {
	delete reinterpret_cast <Delta_private *>(s->type_data);
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
	double max_spu = 0;
	for (int i = 0; i < s->num_motors; ++i)
		if (max_spu < s->motor[i]->steps_per_unit)
			max_spu = s->motor[i]->steps_per_unit;
	return 1e6 / hwtime_step / max_spu;
}

static int follow(Space *s, int axis) {
	(void)&s;
	(void)&axis;
	return -1;
}

void Delta_init(int num) {
	space_types[num].xyz2motors = xyz2motors;
	space_types[num].reset_pos = reset_pos;
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
}

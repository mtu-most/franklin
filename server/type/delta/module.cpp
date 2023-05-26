/* delta.cpp - Delta geometry handling for Franklin
 * Copyright 2014-2016 Michigan Technological University
 * Copyright 2016-2019 Bas Wijnen <wijnen@debian.org>
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

#include <franklin-module.h>

struct SpaceData {
	double angle;			// Adjust the front of the machine.
	SpaceData() : angle(0) {}
};

struct MotorData {
	double axis_min, axis_max;	// Limits for the movement of this axis.
	double rodlength, radius;	// Length of the tie rod and the horizontal distance between the vertical position and the zero position.
	double x, y, z;		// Position of tower on the base plane, and the carriage height at zero position.
	MotorData() : axis_min(0), axis_max(INFINITY), rodlength(250), radius(126), x(0), y(0), z(0) {}
};

UseSpace(SpaceData);
UseMotor(MotorData);

static bool check_delta(Space *s, uint8_t m, double *target) {	// {{{
	// Check whether position is allowed by provided limits.
	// Adjust target if it isn't.
	double dx = target[0] - myMotor(s, m).x;
	double dy = target[1] - myMotor(s, m).y;
	double r2 = dx * dx + dy * dy;
	double amax = myMotor(s, m).axis_max < myMotor(s, m).rodlength ? myMotor(s, m).axis_max : myMotor(s, m).rodlength;
	if (r2 > amax * amax) {
		debug ("not ok 1: %f %f %f %f %f %f %f", target[0], target[1], dx, dy, r2, myMotor(s, m).rodlength, myMotor(s, m).axis_max);
		// target is too far away from axis.  Pull it towards axis so that it is on the edge.
		// target = axis + (target - axis) * (l - epsilon) / r.
		double factor(amax / sqrt(r2));
		target[0] = myMotor(s, m).x + (target[0] - myMotor(s, m).x) * factor;
		target[1] = myMotor(s, m).y + (target[1] - myMotor(s, m).y) * factor;
		return false;
	}
	// Inner product shows if projection is inside or outside the printable region.
	double projection = -(dx / myMotor(s, m).radius * myMotor(s, m).x + dy / myMotor(s, m).radius * myMotor(s, m).y);
	double amin = myMotor(s, m).axis_min < -myMotor(s, m).rodlength ? -myMotor(s, m).rodlength : myMotor(s, m).axis_min;
	if (projection < amin) {
		debug ("not ok 2: %f %f %f %f %f", projection, dx, dy, myMotor(s, m).x, myMotor(s, m).y);
		// target is on the wrong side of axis.  Pull it towards plane so it is on the edge.
		target[0] -= ((amin - projection) / myMotor(s, m).radius - .001) * myMotor(s, m).x;
		target[1] -= ((amin - projection) / myMotor(s, m).radius - .001) * myMotor(s, m).y;
		// Assume this was a small correction; that way, things will work even if numerical errors cause this to be called for the real move.
		return false;
	}
	//debug("ok %d %d %f", s->id, a, projection);
	return true;
}	// }}}

static inline double delta_to_axis(Space *s, uint8_t m) {
	double dx = s->axis[0]->target - myMotor(s, m).x;
	double dy = s->axis[1]->target - myMotor(s, m).y;
	double dz = s->axis[2]->target - myMotor(s, m).z;
	double r2 = dx * dx + dy * dy;
	double l2 = myMotor(s, m).rodlength * myMotor(s, m).rodlength;
	double dest = sqrt(l2 - r2) + dz;
	//debug("dta dx %f dy %f dz %f z %f, r %f target %f", dx, dy, dz, myMotor(s, m).z, r, target);
	return dest;
}

static double inner(double a[3], double b[3]) {
	double ret = 0;
	for (int i = 0; i < 3; ++i)
		ret += a[i] * b[i];
	return ret;
}

static double length(double a[3]) {
	return sqrt(inner(a, a));
}

static void norm(double a[3]) {
	double len = length(a);
	for (int i = 0; i < 3; ++i)
		a[i] /= len;
}

static void cross(double a[3], double b[3], double ret[3]) {
	for (int i = 0; i < 3; ++i)
		ret[i] = a[(i + 1) % 3] * b[(i + 2) % 3] - a[(i + 2) % 3] * b[(i + 1) % 3];
}

static void intersect_spheres(double A[3], double B[3], double len_AC, double len_BC, double D[3], double n[3], double *r) {
	double AB[3];
	for (int i = 0; i < 3; ++i)
		AB[i] = B[i] - A[i];
	double len_AB = length(AB);
	double len_AD = (len_AC * len_AC - len_BC * len_BC + len_AB * len_AB) / (2 * len_AB);
	*r = sqrt(len_AC * len_AC - len_AD * len_AD);
	for (int i = 0; i < 3; ++i) {
		n[i] = AB[i] / len_AB;
		D[i] = A[i] + n[i] * len_AD;
	}
}

void xyz2motors(Space *s) {
	for (uint8_t m = 0; m < 3; ++m)
		s->motor[m]->target_pos = delta_to_axis(s, m);
}

void check_position(Space *s, double *data) {
	if (std::isnan(data[0]) || std::isnan(data[1])) {
		if (!std::isnan(data[0]))
			data[1] = s->axis[1]->settings.source;
		else if (!std::isnan(data[1]))
			data[0] = s->axis[0]->settings.source;
		else {
			// Cannot check; assume it's ok.
			return;
		}
	}
	for (uint8_t counter = 0; counter < 2; ++counter) {
		bool ok = true;
		for (uint8_t m = 0; m < s->num_motors; ++m)
			ok &= check_delta(s, m, data);
		if (ok)
			break;
	}
}

static void update(Space *s) {
	if (s->motor[2]->type_data == NULL)
		return;
#define sin210 -.5
#define cos210 -0.8660254037844386	// .5*sqrt(3)
#define sin330 -.5
#define cos330 0.8660254037844386	// .5*sqrt(3)
#define sin90 1
	// Coordinates of axes (at angles 210, 330, 90; each with its own radius).
	double x[3], y[3];
	x[0] = myMotor(s, 0).radius * cos210;
	y[0] = myMotor(s, 0).radius * sin210;
	x[1] = myMotor(s, 1).radius * cos330;
	y[1] = myMotor(s, 1).radius * sin330;
	x[2] = 0;
	y[2] = myMotor(s, 2).radius * sin90;
	double angle = mySpace(s).angle * M_PI / 180;
	for (uint8_t m = 0; m < 3; ++m) {
		myMotor(s, m).x = x[m] * cos(angle) - y[m] * sin(angle);
		myMotor(s, m).y = y[m] * cos(angle) + x[m] * sin(angle);
		myMotor(s, m).z = sqrt(myMotor(s, m).rodlength * myMotor(s, m).rodlength - myMotor(s, m).radius * myMotor(s, m).radius);
	}
}

void load_space(Space *s) {
	mySpace(s).angle = load_float();
	if (std::isinf(mySpace(s).angle) || std::isnan(mySpace(s).angle))
		mySpace(s).angle = 0;
	update(s);
}

void load_motor(Space *s, int m) {
	myMotor(s, m).axis_min = load_float();
	myMotor(s, m).axis_max = load_float();
	myMotor(s, m).rodlength = load_float();
	myMotor(s, m).radius = load_float();
	update(s);
}

void save_space(Space *s) {
	save_float(mySpace(s).angle);
}

void save_motor(Space *s, int m) {
	save_float(myMotor(s, m).axis_min);
	save_float(myMotor(s, m).axis_max);
	save_float(myMotor(s, m).rodlength);
	save_float(myMotor(s, m).radius);
}

void motors2xyz(Space *s, const double motors[3], double xyz[3]) {
	// Find intersecting circle of spheres 0 and 1.
	double pos[3][3];	// Position of carriages: centers of spheres.
	for (int i = 0; i < 3; ++i) {
		pos[i][0] = myMotor(s, i).x;
		pos[i][1] = myMotor(s, i).y;
		pos[i][2] = myMotor(s, i).z + motors[i];
	}
	double P_uv[3];
	double n_uv[3];
	double r_uv;
	intersect_spheres(pos[0], pos[1], myMotor(s, 0).rodlength, myMotor(s, 1).rodlength, P_uv, n_uv, &r_uv);
	double P_uvw[3];
	double n_uvw[3];
	double r_uvw;
	intersect_spheres(P_uv, pos[2], r_uv, myMotor(s, 2).rodlength, P_uvw, n_uvw, &r_uvw);
	// Compute direction of L.
	double dir_L[3];
	cross(n_uv, n_uvw, dir_L);
	norm(dir_L);
	// Compute direction of M and M.
	double dir_M[3];
	cross(dir_L, n_uvw, dir_M);
	norm(dir_M);
	double PuvPuvw[3];
	for (int i = 0; i < 3; ++i)
		PuvPuvw[i] = P_uvw[i] - P_uv[i];
	double len_PP = std::sqrt(inner(PuvPuvw, PuvPuvw));
	// Compute alpha.
	double tan_alpha = std::tan(std::asin(inner(n_uv, PuvPuvw) / len_PP));
	// Compute P_M.
	double PuvwP_M[3];
	double P_M[3];
	for (int i = 0; i < 3; ++i) {
		PuvwP_M[i] = dir_M[i] * len_PP * tan_alpha;
		P_M[i] = P_uvw[i] + PuvwP_M[i];
	}
	double offset_P_M_squared = inner(PuvwP_M, PuvwP_M);
	double P_MR = std::sqrt(r_uvw * r_uvw - offset_P_M_squared);
	for (int i = 0; i < 3; ++i)
		xyz[i] = P_M[i] - dir_L[i] * P_MR;
	//debug("motors2xyz: Puv (%f,%f,%f) Puvw (%f,%f,%f) ruv %f ruvw %f lenPP %f dir_M (%f,%f,%f) PM (%f,%f,%f) ret (%f,%f,%f)", P_uv[0], P_uv[1], P_uv[2], P_uvw[0], P_uvw[1], P_uvw[2], r_uv, r_uvw, len_PP, dir_M[0], dir_M[1], dir_M[2], P_M[0], P_M[1], P_M[2], xyz[0], xyz[1], xyz[2]);
}

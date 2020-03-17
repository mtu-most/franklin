/* polar.cpp - Polar geometry handling for Franklin
 * Copyright 2019 Bas Wijnen <wijnen@debian.org>
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
	double max_r;
};

UseSpace(SpaceData);

void xyz2motors(Space *s) {
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

void motors2xyz(Space *s, const double motors[3], double xyz[3]) {
	(void)&s;
	xyz[0] = motors[0] * cos(motors[1]);
	xyz[1] = motors[0] * sin(motors[1]);
	xyz[2] = motors[2];
}

void check_position(Space *s, double *data) {
	double r = sqrt(data[0] * data[0] + data[1] * data[1]);
	if (r <= mySpace(s).max_r)
		return;
	double factor = mySpace(s).max_r / r;
	data[0] *= factor;
	data[1] *= factor;
}

void load_space(Space *s) {
	mySpace(s).max_r = load_float();
}

void save_space(Space *s) {
	save_float(mySpace(s).max_r);
}

/* hbot.cpp - H-Bot geometry handling for Franklin
 * Copyright 2014-2016 Michigan Technological University
 * Copyright 2016-2019 Bas Wijnen <wijnen@debian.org>
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

// x = (u + v) / 2
// y = (u - v) / 2

// u = x + y
// v = x - y

#include <franklin-module.h>

void xyz2motors(Space *s) {
	s->motor[0]->target_pos = s->axis[0]->target + s->axis[1]->target;
	s->motor[1]->target_pos = s->axis[0]->target - s->axis[1]->target;
}

void motors2xyz(Space *s, const double *motors, double *xyz) {
	(void)(&s);
	xyz[0] = (motors[0] + motors[1]) / 2;
	xyz[1] = (motors[0] - motors[1]) / 2;
}

void load(Space *s) {
	if (!s->setup_nums(3, 3)) {
		debug("Failed to set up H-bot axes");
		s->cancel_update();
	}
}

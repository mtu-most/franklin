/* extruder.cpp - Extruder geometry handling for Franklin
 * Copyright 2019-2020 Bas Wijnen <wijnen@debian.org>
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

struct AxisData {
	double offset[3];
};

UseAxis(AxisData);

void load_axis(Space *s, int a) {
	for (int o = 0; o < 3; ++o)
		myAxis(s, a).offset[o] = load_float();
}

void save_axis(Space *s, int a) {
	for (int o = 0; o < 3; ++o)
		save_float(myAxis(s, a).offset[o]);
}

void xyz2motors(Space *s) {
	for (int m = 0; m < s->num_motors; ++m)
		s->motor[m]->target_pos = s->axis[m]->target;
}

void motors2xyz(Space *s, const double motors[3], double xyz[3]) {
	for (int m = 0; m < s->num_motors; ++m)
		xyz[m] = motors[m];
}

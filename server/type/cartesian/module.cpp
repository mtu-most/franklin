/* type-cartesian.cpp - Cartesian and extruder geometry handling for Franklin
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

#include <franklin-module.h>

void xyz2motors(Space *s) { // {{{
	(void)&s;
} // }}}

void motors2xyz(Space *s, const double *motors, double *xyz) { // {{{
	(void)&s;
	(void)&motors;
	(void)&xyz;
} // }}}

void check_position(Space *s, double *data) { // {{{
	(void)&s;
	(void)&data;
} // }}}

void load_space(Space *s) { // {{{
	(void)&s;
} // }}}

void load_axis(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
} // }}}

void load_motor(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

void save_space(Space *s) { // {{{
	(void)&s;
} // }}}

void save_axis(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
} // }}}

void save_motor(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

bool init_space(Space *s) { // {{{
	(void)&s;
	return true;
} // }}}

void init_axis(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
} // }}}

void init_motor(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

void free_space(Space *s) { // {{{
	(void)&s;
} // }}}

void free_axis(Space *s, int a) { // {{{
	(void)&s;
	(void)&a;
} // }}}

void free_motor(Space *s, int m) { // {{{
	(void)&s;
	(void)&m;
} // }}}

double probe_speed(Space *s) { // {{{
	if (s->num_motors >= 3)
		return 1e6 / settings.hwtime_step / s->motor[2]->steps_per_unit;
	return INFINITY;
} // }}}

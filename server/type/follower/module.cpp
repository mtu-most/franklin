/* follower.cpp - Follower handling for Franklin
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

UseMotor(FollowerMotorData);

void load_motor(Space *s, int m) {
	myMotor(s, m).space = load_int();
	myMotor(s, m).motor = load_int();
}

void save_motor(Space *s, int m) {
	save_int(myMotor(s, m).space);
	save_int(myMotor(s, m).motor);
}

void xyz2motors(Space *s) {
	(void)&s;
	debug("xyz2motors should not be called for follower space");
	abort();
}

void motors2xyz(Space *s, const double motors[3], double xyz[3]) {
	(void)&s;
	(void)&motors;
	(void)&xyz;
	debug("motors2xyz should not be called for follower space");
	abort();
}

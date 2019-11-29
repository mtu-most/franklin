/* drawbot.cpp - Drawbot geometry handling for Franklin
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

struct MotorData {
	double x, y;
};

UseMotor(MotorData);

void load_space(Space *s) {
	if (!s->setup_nums(3, 3)) {
		debug("Failed to set up drawbot axes");
		s->cancel_update();
		return;
	}
}

void load_motor(Space *s, int m) {
	myMotor(s, m).x = shmem->floats[100];
	myMotor(s, m).y = shmem->floats[101];
}

void save_motor(Space *s, int m) {
	save_count(0, 2);
	shmem->floats[100] = myMotor(s, m).x;
	shmem->floats[101] = myMotor(s, m).y;
}

void xyz2motors(Space *s) {
	for (int m = 0; m < 2; ++m) {
		double dx = s->axis[0]->target - myMotor(s, m).x;
		double dy = s->axis[1]->target - myMotor(s, m).y;
		s->motor[m]->target_pos = sqrt(dx * dx + dy * dy);
	}
	for (int m = 2; m < s->num_motors; ++m)
		s->motor[m]->target_pos = s->axis[m]->target;
}

void motors2xyz(Space *s, const double motors[3], double xyz[3]) {
	double uv[2];
	uv[0] = myMotor(s, 1).x - myMotor(s, 0).x;
	uv[1] = myMotor(s, 1).y - myMotor(s, 0).y;
	double down[2], l;
	l = sqrt(uv[0] * uv[0] + uv[1] * uv[1]);
	for (int i = 0; i < 2; ++i)
		uv[i] /= l;
	down[0] = uv[1];
	down[1] = -uv[0];

	// Use symbols from delta documentation.
	double AC = motors[0];
	double BC = motors[1];
	double AB = l;

	double AD = (AC * AC - BC * BC) / (2 * AB) + AB / 2;
	double CD = sqrt(AC * AC - AD * AD);

	xyz[0] = myMotor(s, 0).x + AD * uv[0] + CD * down[0];
	xyz[1] = myMotor(s, 0).y + AD * uv[1] + CD * down[1];
	xyz[2] = motors[2];
}

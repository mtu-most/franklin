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

#include <cdriver.h>

struct Apex {
	double x, y;
};

#define APEX(s, m) (*reinterpret_cast <Apex *>(s->motor[m]->type_data))

extern "C" {
void load(Space *s) {
	if (!s->setup_nums(2, 2)) {
		debug("Failed to set up drawbot axes");
		s->cancel_update();
		return;
	}
	for (uint8_t m = 0; m < 2; ++m)
		s->motor[m]->type_data = new Apex;
}

void mload(Space *s, int m) {
	APEX(s, m).x = shmem->floats[100];
	APEX(s, m).y = shmem->floats[101];
}

void msave(Space *s, int m) {
	shmem->ints[100] = 0;
	shmem->ints[101] = 2;
	shmem->floats[100] = APEX(s, m).x;
	shmem->floats[101] = APEX(s, m).y;
}

void motor_free(Space *s, int m) {
	delete reinterpret_cast <Apex *>(s->motor[m]->type_data);
	s->motor[m]->type_data = NULL;
}


void xyz2motors(Space *s) {
	for (int m = 0; m < 2; ++m) {
		double dx = s->axis[0]->target - APEX(s, m).x;
		double dy = s->axis[1]->target - APEX(s, m).y;
		s->motor[m]->target_pos = sqrt(dx * dx + dy * dy);
	}
	for (int m = 2; m < s->num_motors; ++m)
		s->motor[m]->target_pos = s->axis[m]->target;
}

void motors2xyz(Space *s, const double motors[3], double xyz[3]) {
	double uv[2];
	uv[0] = APEX(s, 1).x - APEX(s, 0).x;
	uv[1] = APEX(s, 1).y - APEX(s, 0).y;
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

	xyz[0] = APEX(s, 0).x + AD * uv[0] + CD * down[0];
	xyz[1] = APEX(s, 0).y + AD * uv[1] + CD * down[1];
	xyz[2] = motors[2];
}
}

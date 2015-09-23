#include "cdriver.h"

struct Polar_private {
	double max_r;
};

#define PRIVATE(s) (*reinterpret_cast <Polar_private *>(s->type_data))

static void xyz2motors(Space *s, double *motors, bool *ok) {
	if (isnan(s->axis[0]->settings.target) || isnan(s->axis[1]->settings.target)) {
		// Fill up missing targets.
		for (uint8_t aa = 0; aa < 2; ++aa) {
			if (isnan(s->axis[aa]->settings.target))
				s->axis[aa]->settings.target = s->axis[aa]->settings.current;
		}
	}
	double x = s->axis[0]->settings.target;
	double y = s->axis[1]->settings.target;
	double z = s->axis[2]->settings.target;
	double r = sqrt(x * x + y * y);
	double theta = atan2(y, x);
	while (theta - s->motor[1]->settings.current_pos / s->motor[1]->steps_per_unit > 2 * M_PI)
		theta -= 2 * M_PI;
	while (theta - s->motor[1]->settings.current_pos / s->motor[1]->steps_per_unit < -2 * M_PI)
		theta += 2 * M_PI;
	if (motors) {
		motors[0] = r;
		motors[1] = theta;
		motors[2] = z;
	}
	else {
		s->motor[0]->settings.endpos = r;
		s->motor[0]->settings.endpos = theta;
		s->motor[0]->settings.endpos = z;
	}
}

static void reset_pos (Space *s) {
	double r = s->motor[0]->settings.current_pos / s->motor[0]->steps_per_unit;
	double theta = s->motor[1]->settings.current_pos / s->motor[1]->steps_per_unit;
	double z = s->motor[2]->settings.current_pos / s->motor[2]->steps_per_unit;
	s->axis[0]->settings.source = r * cos(theta);
	s->axis[1]->settings.source = r * sin(theta);
	s->axis[2]->settings.source = z;
}

static void check_position(Space *s, double *data) {
	double r = sqrt(data[0] * data[0] + data[1] * data[1]);
	if (r <= PRIVATE(s).max_r)
		return;
	data[0] *= PRIVATE(s).max_r / r;
	data[1] *= PRIVATE(s).max_r / r;
}

static void load(Space *s, uint8_t old_type, int32_t &addr) {
	if (!s->setup_nums(3, 3)) {
		debug("Failed to set up polar axes");
		s->cancel_update();
		return;
	}
	PRIVATE(s).max_r = read_float(addr);
}

static void save(Space *s, int32_t &addr) {
	write_float(addr, PRIVATE(s).max_r);
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
}

static double change0(Space *s, int axis, double value) {
	return value;
}

static double unchange0(Space *s, int axis, double value) {
	return value;
}

static double probe_speed(Space *s) {
	return 1e6 / hwtime_step / s->motor[2]->steps_per_unit;
}

void Polar_init(int num) {
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
}

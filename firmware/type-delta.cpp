#include <firmware.h>

#ifdef HAVE_SPACES
#ifdef HAVE_DELTA
struct Apex {
	float axis_min, axis_max;	// Limits for the movement of this axis.
	float length, radius;	// Length of the tie rod and the horizontal distance between the vertical position and the zero position.
	float x, y, z;		// Position of tower on the base plane, and the carriage height at zero position.
};

struct Delta_private {
	Apex apex[3];
};

#define APEX(s, a) (reinterpret_cast <Delta_private *>(s->private_data).apex[a])

static bool check_delta(Space *s, uint8_t a, float *target) {	// {{{
	float dx = target[0] - APEX(s, a).x;
	float dy = target[1] - APEX(s, a).y;
	float r2 = dx * dx + dy * dy;
	if (r2 > APEX(s, a).axis_max * APEX(s, a).axis_max) {
		//debug ("not ok 1: %f %f %f %f %f %f %f", F(target[0]), F(target[1]), F(dx), F(dy), F(r2), F(axis[a].length), F(axis[a].axis_max));
		// target is too far away from axis.  Pull it towards axis so that it is on the edge.
		// target = axis + (target - axis) * (l - epsilon) / r.
		float factor(APEX(s, a).axis_max / sqrt(r2));
		target[0] = APEX(s, a).x + (target[0] - APEX(s, a).x) * factor;
		target[1] = APEX(s, a).y + (target[1] - APEX(s, a).y) * factor;
		return false;
	}
	// Inner product shows if projection is inside or outside the printable region.
	float projection = -(dx / APEX(s, a).radius * APEX(s, a).x + dy / APEX(s, a).radius * APEX(s, a).y);
	if (projection < APEX(s, a).axis_min) {
		//debug ("not ok 2: %f %f %f %f %f", F(projection), F(dx), F(dy), F(APEX(s, a).x), F(APEX(s, a).y));
		// target is on the wrong side of axis.  Pull it towards plane so it is on the edge.
		target[0] -= (APEX(s, a).axis_min - projection - 1) / APEX(s, a).radius * APEX(s, a).x;
		target[1] -= (APEX(s, a).axis_min - projection - 1) / APEX(s, a).radius * APEX(s, a).y;
		// Assume this was a small correction; that way, things will work even if numerical errors cause this to be called for the real move.
		return false;
	}
	//debug("ok");
	return true;
}	// }}}

static inline float delta_to_axis(Space *s, uint8_t a, float *target, bool *ok) {
	float dx = target[0] - APEX(s, a).x;
	float dy = target[1] - APEX(s, a).y;
	float dz = target[2] - APEX(s, a).z;
	float r2 = dx * dx + dy * dy;
	float l2 = APEX(s, a).length * APEX(s, a).length;
	float dest = sqrt(l2 - r2) + dz;
	//debug("dta dx %f dy %f dz %f z %f, r %f target %f", F(dx), F(dy), F(dz), F(APEX(s, a).z), F(r), F(target));
	return dest;
}

static void xyz2motors(Space *s, float *xyz, float *motors, bool *ok) {
	if (isnan(xyz[0]) || isnan(xyz[1]) || isnan(xyz[2])) {
		// Fill up missing targets.
		for (uint8_t aa = 0; aa < 3; ++aa) {
			if (isnan(xyz[aa]))
				xyz[aa] = s->axis[aa].current;
		}
	}
	for (uint8_t a = 0; a < 3; ++a)
		motors[a] = delta_to_axis(a, xyz, ok);
}

static void reset_pos (Space *s) {
	// All axes' current_pos must be valid and equal, in other words, x=y=0.
	if (s->motor[0].current_pos != s->motor[1].current_pos || s->motor[0].current_pos != s->motor[2].current_pos) {
		//debug("resetpos fails");
		s->axis[0].source = NAN;
		s->axis[1].source = NAN;
		s->axis[2].source = NAN;
	}
	else {
		//debug("resetpos %f", F(APEX(s, a).current_pos));
		s->axis[0].source = 0;
		s->axis[1].source = 0;
		s->axis[2].source = s->motor[0].current_pos;
	}
}

static void check_position(Space *s, float *data) {
	if (isnan(data[0]) || isnan(data[1])) {
		// Cannot check; assume it's ok.
		return;
	}
	for (uint8_t timeout = 0; timeout < 2; ++timeout) {
		bool ok = true;
		for (uint8_t a = 0; a < s->num_axes; ++a)
			ok &= check_delta(s, a, data);
		if (ok)
			break;
	}
}

static void load(Space *s, int16_t &addr, bool eeprom) {
	for (uint8_t a = 0; a < 3; ++a) {
		APEX(s, a).axis_min = read_float(addr, eeprom);
		APEX(s, a).axis_max = read_float(addr, eeprom);
		APEX(s, a).length = read_float(addr, eeprom);
		APEX(s, a).radius = read_float(addr, eeprom);
		if (APEX(s, a).axis_max > APEX(s, a).length || APEX(s, a).axis_max < 0)
			APEX(s, a).axis_max = APEX(s, a).length;
		if (APEX(s, a).axis_min > APEX(s, a).APEX(s, a).axis_max)
			APEX(s, a).axis_min = 0;
	}
	s->setup_nums(3, 3);
}

static void save(Space *s, int16_t &addr, bool eeprom) {
	for (uint8_t a = 0; a < 3; ++a) {
		write_float(addr, APEX(s, a).axis_min, eeprom);
		write_float(addr, APEX(s, a).axis_max, eeprom);
		write_float(addr, APEX(s, a).length, eeprom);
		write_float(addr, APEX(s, a).radius, eeprom);
	}
}

static void init(Space *s) {
	s->private_data = new Delta_private;
}

static void free(Space *s) {
	delete s->private_data;
}

static uint16_t size(Space *s) {
	return sizeof(float) * 4 + s->size_std();
}

void Delta_init(uint8_t num) {
	space_types[num].xyz2motors = xyz2motors;
	space_types[num].reset_pos = reset_pos;
	space_types[num].check_position = check_position;
	space_types[num].load = load;
	space_types[num].save = save;
	space_types[num].init = init;
	space_types[num].free = free;
	space_types[num].size = size;
}
#endif
#endif

#if 0
#ifndef LOWMEM
#define sin120 0.8660254037844386	// .5*sqrt(3)
#define cos120 -.5
void compute_axes()
{

	axis_min = read_float(addr, eeprom);
	axis_max = read_float(addr, eeprom);
	delta_length = read_float(addr, eeprom);
	delta_radius = read_float(addr, eeprom);
	z = sqrt(delta_length * delta_length - delta_radius * delta_radius);
		write_float(addr, axis_min, eeprom);
		write_float(addr, axis_max, eeprom);
		write_float(addr, delta_length, eeprom);
		write_float(addr, delta_radius, eeprom);
#ifndef LOWMEM
	compute_axes();
#endif
}
#ifdef HAVE_DELTA
	// Coordinates of axes (at angles 0, 120, 240; each with its own radius).
	/* Originally u was on the x axis, but v on the y axis is more natural,
	   because the front should be as open as possible.
	   Putting v on the y axis makes sense for people who are confusing
	   uvw and xyz.  That's a very weak argument for this choice; my guess
	   is that others confuse them, and this is why other firmwares use
	   this convention.  It is used here because there is no "better" way,
	   so this firmware now behaves like the others.
	axis[0].x = axis[0].delta_radius;
	axis[0].y = 0;
	axis[1].x = axis[1].delta_radius * cos120;
	axis[1].y = axis[1].delta_radius * sin120;
	axis[2].x = axis[2].delta_radius * cos120;
	axis[2].y = axis[2].delta_radius * -sin120;
	*/
	float x[3], y[3];
	x[0] = axis[0].delta_radius * sin120;
	y[0] = axis[0].delta_radius * cos120;
	x[1] = 0;
	y[1] = axis[1].delta_radius;
	x[2] = axis[2].delta_radius * -sin120;
	y[2] = axis[2].delta_radius * cos120;
	for (uint8_t a = 0; a < 3; ++a) {
		axis[a].x = x[a] * cos(angle) - y[a] * sin(angle);
		axis[a].y = y[a] * cos(angle) + x[a] * sin(angle);
	}
#endif
}
#endif
#endif

#include "firmware.h"

#ifdef HAVE_SPACES
#ifdef HAVE_DELTA
struct Apex {
	float axis_min, axis_max;	// Limits for the movement of this axis.
	float rodlength, radius;	// Length of the tie rod and the horizontal distance between the vertical position and the zero position.
	float x, y, z;		// Position of tower on the base plane, and the carriage height at zero position.
};

struct Delta_private {
	Apex apex[3];
	float angle;			// Adjust the front of the printer.
};

#define PRIVATE(s) (*reinterpret_cast <Delta_private *>(s->type_data))
#define APEX(s, a) (PRIVATE(s).apex[a])

static bool check_delta(Space *s, uint8_t a, float *target) {	// {{{
	float dx = target[0] - APEX(s, a).x;
	float dy = target[1] - APEX(s, a).y;
	float r2 = dx * dx + dy * dy;
	if (r2 > APEX(s, a).axis_max * APEX(s, a).axis_max) {
		debug ("not ok 1: %f %f %f %f %f %f %f", F(target[0]), F(target[1]), F(dx), F(dy), F(r2), F(APEX(s, a).rodlength), F(APEX(s, a).axis_max));
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
		debug ("not ok 2: %f %f %f %f %f", F(projection), F(dx), F(dy), F(APEX(s, a).x), F(APEX(s, a).y));
		// target is on the wrong side of axis.  Pull it towards plane so it is on the edge.
		target[0] -= ((APEX(s, a).axis_min - projection) / APEX(s, a).radius - .001) * APEX(s, a).x;
		target[1] -= ((APEX(s, a).axis_min - projection) / APEX(s, a).radius - .001) * APEX(s, a).y;
		// Assume this was a small correction; that way, things will work even if numerical errors cause this to be called for the real move.
		return false;
	}
	//debug("ok");
	return true;
}	// }}}

static inline float delta_to_axis(Space *s, uint8_t a, bool *ok) {
	float dx = s->axis[0]->target - APEX(s, a).x;
	float dy = s->axis[1]->target - APEX(s, a).y;
	float dz = s->axis[2]->target - APEX(s, a).z;
	float r2 = dx * dx + dy * dy;
	float l2 = APEX(s, a).rodlength * APEX(s, a).rodlength;
	float dest = sqrt(l2 - r2) + dz;
	//debug("dta dx %f dy %f dz %f z %f, r %f target %f", F(dx), F(dy), F(dz), F(APEX(s, a).z), F(r), F(target));
	return dest;
}

static void xyz2motors(Space *s, float *motors, bool *ok) {
	if (isnan(s->axis[0]->target) || isnan(s->axis[1]->target) || isnan(s->axis[2]->target)) {
		// Fill up missing targets.
		for (uint8_t aa = 0; aa < 3; ++aa) {
			if (isnan(s->axis[aa]->target))
				s->axis[aa]->target = s->axis[aa]->current;
		}
	}
	for (uint8_t a = 0; a < 3; ++a) {
		if (motors)
			motors[a] = delta_to_axis(s, a, ok);
		else
			s->motor[a]->endpos = delta_to_axis(s, a, ok);
	}
}

static void reset_pos (Space *s) {
	// All axes' current_pos must be valid and equal, in other words, x=y=0.
	if (s->motor[0]->current_pos != s->motor[1]->current_pos || s->motor[0]->current_pos != s->motor[2]->current_pos) {
		//debug("resetpos fails");
		s->axis[0]->source = NAN;
		s->axis[1]->source = NAN;
		s->axis[2]->source = NAN;
	}
	else {
		//debug("resetpos %f", F(APEX(s, a).current_pos));
		s->axis[0]->source = 0;
		s->axis[1]->source = 0;
		s->axis[2]->source = s->motor[0]->current_pos;
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

static void load(Space *s, uint8_t old_type, int32_t &addr, bool eeprom) {
	if (!s->setup_nums(3, 3)) {
		debug("Failed to set up delta axes");
		s->cancel_update();
		return;
	}
	for (uint8_t a = 0; a < 3; ++a) {
		APEX(s, a).axis_min = read_float(addr, eeprom);
		APEX(s, a).axis_max = read_float(addr, eeprom);
		APEX(s, a).rodlength = read_float(addr, eeprom);
		APEX(s, a).radius = read_float(addr, eeprom);
		if (APEX(s, a).axis_max > APEX(s, a).rodlength || APEX(s, a).axis_max < 0)
			APEX(s, a).axis_max = APEX(s, a).rodlength;
		if (APEX(s, a).axis_min > APEX(s, a).axis_max)
			APEX(s, a).axis_min = 0;
	}
	PRIVATE(s).angle = read_float(addr, eeprom);
	if (isinf(PRIVATE(s).angle) || isnan(PRIVATE(s).angle))
		PRIVATE(s).angle = 0;
#define sin210 -.5
#define cos210 -0.8660254037844386	// .5*sqrt(3)
#define sin330 -.5
#define cos330 0.8660254037844386	// .5*sqrt(3)
#define sin90 1
	// Coordinates of axes (at angles 210, 330, 90; each with its own radius).
	float x[3], y[3];
	x[0] = APEX(s, 0).radius * cos210;
	y[0] = APEX(s, 0).radius * sin210;
	x[1] = APEX(s, 1).radius * cos330;
	y[1] = APEX(s, 1).radius * sin330;
	x[2] = 0;
	y[2] = APEX(s, 2).radius * sin90;
	for (uint8_t a = 0; a < 3; ++a) {
		APEX(s, a).x = x[a] * cos(PRIVATE(s).angle) - y[a] * sin(PRIVATE(s).angle);
		APEX(s, a).y = y[a] * cos(PRIVATE(s).angle) + x[a] * sin(PRIVATE(s).angle);
		APEX(s, a).z = sqrt(APEX(s, a).rodlength * APEX(s, a).rodlength - APEX(s, a).radius * APEX(s, a).radius);
	}
}

static void save(Space *s, int32_t &addr, bool eeprom) {
	for (uint8_t a = 0; a < 3; ++a) {
		write_float(addr, APEX(s, a).axis_min, eeprom);
		write_float(addr, APEX(s, a).axis_max, eeprom);
		write_float(addr, APEX(s, a).rodlength, eeprom);
		write_float(addr, APEX(s, a).radius, eeprom);
	}
	write_float(addr, PRIVATE(s).angle, eeprom);
}

static bool init(Space *s) {
	mem_alloc(sizeof(Delta_private), &s->type_data, "delta");
	if (!s->type_data)
		return false;
	return true;
}

static void free(Space *s) {
	mem_free(&s->type_data);
}

static int32_t savesize(Space *s) {
	return sizeof(float) * 4 + s->savesize_std();
}

static bool change0(Space *s) {
	return true;
}

void Delta_init(uint8_t num) {
	space_types[num].xyz2motors = xyz2motors;
	space_types[num].reset_pos = reset_pos;
	space_types[num].check_position = check_position;
	space_types[num].load = load;
	space_types[num].save = save;
	space_types[num].init = init;
	space_types[num].free = free;
	space_types[num].savesize = savesize;
#ifdef HAVE_EXTRUDER
	space_types[num].change0 = change0;
#endif
}
#endif
#endif

#include <firmware.h>

#if MAXAXES >= 3
struct Delta : public PrinterType {
	virtual void xyz2motors(float *xyz, float *motors, bool *ok);
	virtual void reset_pos();
	virtual void check_position(float *data);
	virtual void enable_motors();
	virtual void invalidate_axis(uint8_t a);
};

static bool check_delta (uint8_t a, float *target) {	// {{{
	float dx = target[0] - axis[a].x;
	float dy = target[1] - axis[a].y;
	float r2 = dx * dx + dy * dy;
	if (r2 > axis[a].axis_max * axis[a].axis_max) {
		//debug ("not ok 1: %f %f %f %f %f %f %f", F(target[0]), F(target[1]), F(dx), F(dy), F(r2), F(axis[a].delta_length), F(axis[a].axis_max));
		// target is too far away from axis.  Pull it towards axis so that it is on the edge.
		// target = axis + (target - axis) * (l - epsilon) / r.
		float factor(axis[a].axis_max / sqrt(r2));
		target[0] = axis[a].x + (target[0] - axis[a].x) * factor;
		target[1] = axis[a].y + (target[1] - axis[a].y) * factor;
		return false;
	}
	// Inner product shows if projection is inside or outside the printable region.
	float projection = -(dx / axis[a].delta_radius * axis[a].x + dy / axis[a].delta_radius * axis[a].y);
	if (projection < axis[a].axis_min) {
		//debug ("not ok 2: %f %f %f %f %f", F(projection), F(dx), F(dy), F(axis[a].x), F(axis[a].y));
		// target is on the wrong side of axis.  Pull it towards plane so it is on the edge.
		target[0] -= (axis[a].axis_min - projection - 1) / axis[a].delta_radius * axis[a].x;
		target[1] -= (axis[a].axis_min - projection - 1) / axis[a].delta_radius * axis[a].y;
		// Assume this was a small correction; that way, things will work even if numerical errors cause this to be called for the real move.
		return false;
	}
	//debug("ok");
	return true;
}	// }}}

static inline float delta_to_axis(uint8_t a, float *target, bool *ok) {
	float dx = target[0] - axis[a].x;
	float dy = target[1] - axis[a].y;
	float dz = target[2] - axis[a].z;
	float r2 = dx * dx + dy * dy;
	float l2 = axis[a].delta_length * axis[a].delta_length;
	float dest = sqrt(l2 - r2) + dz;
	//debug("dta dx %f dy %f dz %f z %f, r %f target %f", F(dx), F(dy), F(dz), F(axis[a].z), F(r), F(target));
	return dest;
}

void Delta::xyz2motors(float *xyz, float *motors, bool *ok) {
	// Delta movements are hardcoded to 3 axes.
	if (isnan(xyz[0]) || isnan(xyz[1]) || isnan(xyz[2])) {
		// Fill up missing targets.
		for (uint8_t aa = 0; aa < 3; ++aa) {
			if (isnan(xyz[aa]))
				xyz[aa] = axis[aa].current;
		}
	}
	uint8_t a;
	for (a = 0; a < 3; ++a)
		motors[a] = delta_to_axis(a, xyz, ok);
	for (; a < num_axes; ++a)
		motors[a] = xyz[a];
}

void Delta::reset_pos () {
	// All axes' current_pos must be valid and equal, in other words, x=y=0.
	if (axis[0].current_pos != axis[1].current_pos || axis[0].current_pos != axis[2].current_pos) {
		//debug("resetpos fails");
		axis[0].source = NAN;
		axis[1].source = NAN;
		axis[2].source = NAN;
	}
	else {
		//debug("resetpos %f", F(axis[0].current_pos));
		axis[0].source = 0;
		axis[1].source = 0;
		axis[2].source = axis[0].current_pos;
	}
	for (uint8_t a = 3; a < MAXAXES; ++a)
		axis[a].source = axis[a].current_pos;
	for (uint8_t a = 0; a < MAXAXES; ++a)
		axis[a].current = axis[a].source;
}

void Delta::check_position(float *data) {
	if (isnan(data[0]) || isnan(data[1])) {
		// Cannot check; assume it's ok.
		return;
	}
	for (uint8_t timeout = 0; timeout < 2; ++timeout) {
		bool ok = true;
		for (uint8_t a = 0; a < num_axes; ++a)
			ok &= check_delta (a, data);
		if (ok)
			break;
	}
}

static void enable_motor(uint8_t mtr) {
	motors[mtr]->last_time = start_time;
	SET (motors[mtr]->enable_pin);
	motors_busy |= 1 << mtr;
	/*if (mt < num_axes)
		debug ("Move motor %f from %f (really %f) over %f steps (f0=%f)", mtr, F(axis[mt].source), F(axis[mt].current), F(motors[mtr]->dist), F(f0));*/
}

void Delta::enable_motors() {
	for (uint8_t a = 0; a < 3; ++a) {
		if (!isnan(axis[a].motor.dist) || !isnan(axis[a].motor.next_dist)) {
			for (uint8_t aa = 0; aa < 3; ++aa)
				enable_motor(aa);
			break;
		}
	}
	for (uint8_t mt = 3; mt < num_axes + num_extruders; ++mt) {
		uint8_t mtr = mt < num_axes ? mt + 2 : mt + 2 + MAXAXES - num_axes;
		if (!motors[mtr])
			continue;
		if (!isnan(motors[mtr]->dist) || !isnan(motors[mtr]->next_dist))
			enable_motor(mtr);
	}
}

void Delta::invalidate_axis(uint8_t a) {
	if (a < 3) {
		for (uint8_t aa = 0; aa < 3; ++aa) {
			axis[aa].source = NAN;
			axis[aa].current = NAN;
		}
	}
	else {
		axis[a].source = NAN;
		axis[a].current = NAN;
	}
}

static Delta obj;
PrinterType *Type_Delta = &obj;
#else
PrinterType *Type_Delta = NULL;
#endif

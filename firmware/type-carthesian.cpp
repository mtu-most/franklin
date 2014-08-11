#include <firmware.h>

struct Cartesian : public PrinterType {
	virtual void xyz2motors(float *xyz, float *motors, bool *ok);
	virtual void reset_pos();
	virtual void check_position(float *data);
	virtual void enable_motors();
	virtual void invalidate_axis(uint8_t a);
};

void Cartesian::xyz2motors(float *xyz, float *motors, bool *ok) {
	for (uint8_t a = 0; a < num_axes; ++a)
		motors[a] = xyz[a];
}

void Cartesian::reset_pos () {
#ifdef HAVE_MOTORS
	for (uint8_t a = 0; a < num_axes; ++a) {
		axis[a].source = axis[a].current_pos;
		axis[a].current = axis[a].source;
	}
#endif
}

void Cartesian::check_position(float *data) {
}

void Cartesian::enable_motors() {
#ifdef HAVE_MOTORS
	for (uint8_t mt = 0; mt < num_axes + num_extruders; ++mt) {
		uint8_t mtr = mt < num_axes ? mt + 2 : mt + 2;
		if (!motors[mtr])
			continue;
		if (!isnan(motors[mtr]->dist) || !isnan(motors[mtr]->next_dist)) {
			motors[mtr]->last_time = start_time;
			SET (motors[mtr]->enable_pin);
			motors_busy |= 1 << mtr;
			/*if (mt < num_axes)
				debug ("Move motor %f from %f (really %f) over %f steps (f0=%f)", mtr, F(axis[mt].source), F(axis[mt].current), F(motors[mtr]->dist), F(f0));*/
		}
	}
#endif
}

void Cartesian::invalidate_axis(uint8_t a) {
#ifdef HAVE_MOTORS
	axis[a].source = NAN;
	axis[a].current = NAN;
#endif
}

static Cartesian obj;
PrinterType *Type_Cartesian = &obj;

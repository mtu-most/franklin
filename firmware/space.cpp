#include "firmware.h"

#ifdef HAVE_SPACES
static void setup_nums(Space *s, uint8_t na, uint8_t nm) {
	Axis **new_axes = new Axis *[na];
	for (uint8_t a = 0; a < min(s->num_axes, na); ++a)
		new_axes[a] = s->axis[a];
	for (uint8_t a = s->num_axes; a < na; ++a) {
		new_axes[a] = new Axis();
		s->axis[a]->source = NAN;
		s->axis[a]->current = NAN;
		s->axis[a]->dist = NAN;
		s->axis[a]->next_dist = NAN;
		s->axis[a]->main_dist = NAN;
	}
	delete[] s->axis;
	s->axis = new_axes;
	s->num_axes = na;
	Motor **new_motors = new Motor *[nm];
	for (uint8_t m = 0; m < min(s->num_motors, nm); ++m)
		new_motors[m] = s->motor[m];
	for (uint8_t m = s->num_motors; m < nm; ++m) {
		new_motors[m] = new Motor();
		s->motor[m]->sense_state = 0;
		s->motor[m]->sense_pos = NAN;
		s->motor[m]->motor_min = -INFINITY;
		s->motor[m]->motor_max = INFINITY;
		s->motor[m]->limits_pos = NAN;
		s->motor[m]->limit_v = INFINITY;
		s->motor[m]->limit_a = INFINITY;
		s->motor[m]->last_time = micros();
		s->motor[m]->last_v = 0;
		s->motor[m]->last_distance = 0;
		s->motor[m]->positive = false;
		s->motor[m]->current_pos = NAN;
#ifdef HAVE_AUDIO
		s->motor[m]->audio_flags = 0;
#endif
	}
	delete[] s->motor;
	s->motor = new_motors;
	s->num_motors = nm;
}

// This function is called by the space types to load motor and axis data to set num_axes and num_motors.
void Space::set_nums(uint8_t na, uint8_t nm, int16_t &addr, bool eeprom) {
	setup_nums(this, na, nm);
	// Set up components.
	for (uint8_t a = 0; a < num_axes; ++a) {
		axis[a]->offset = read_float(addr, eeprom);
		axis[a]->park = read_float(addr, eeprom);
		axis[a]->max_v = read_float(addr, eeprom);
	}
	for (uint8_t m = 0; m < num_motors; ++m) {
		motor[m]->step_pin.read(read_16(addr, eeprom));
		motor[m]->dir_pin.read(read_16(addr, eeprom));
		motor[m]->enable_pin.read(read_16(addr, eeprom));
		motor[m]->limit_min_pin.read(read_16(addr, eeprom));
		motor[m]->limit_max_pin.read(read_16(addr, eeprom));
		motor[m]->sense_pin.read(read_16(addr, eeprom));
		motor[m]->steps_per_m = read_float(addr, eeprom);
		motor[m]->max_steps = read_8(addr, eeprom);
		motor[m]->home_pos = read_float(addr, eeprom);
		motor[m]->motor_min = read_float(addr, eeprom);
		motor[m]->motor_max = read_float(addr, eeprom);
		motor[m]->limit_v = read_float(addr, eeprom);
		motor[m]->limit_a = read_float(addr, eeprom);
		SET_OUTPUT(motor[m]->step_pin);
		SET_OUTPUT(motor[m]->dir_pin);
		SET_OUTPUT(motor[m]->enable_pin);
		SET_INPUT(motor[m]->limit_min_pin);
		SET_INPUT(motor[m]->limit_max_pin);
		SET_INPUT(motor[m]->sense_pin);
	}
}

// This is called by the type's save function to save all the standard parts.
void Space::save_std(int16_t &addr, bool eeprom) {
	for (uint8_t a = 0; a < num_axes; ++a) {
		write_float(addr, axis[a]->park, eeprom);
		write_float(addr, axis[a]->offset, eeprom);
		write_float(addr, axis[a]->max_v, eeprom);
	}
	for (uint8_t m = 0; m < num_motors; ++m) {
		write_16(addr, motor[m]->step_pin.write(), eeprom);
		write_16(addr, motor[m]->dir_pin.write(), eeprom);
		write_16(addr, motor[m]->enable_pin.write(), eeprom);
		write_16(addr, motor[m]->limit_min_pin.write(), eeprom);
		write_16(addr, motor[m]->limit_max_pin.write(), eeprom);
		write_16(addr, motor[m]->sense_pin.write(), eeprom);
		write_float(addr, motor[m]->steps_per_m, eeprom);
		write_float(addr, motor[m]->max_steps, eeprom);
		write_float(addr, motor[m]->home_pos, eeprom);
		write_float(addr, motor[m]->motor_min, eeprom);
		write_float(addr, motor[m]->motor_max, eeprom);
		write_float(addr, motor[m]->limit_v, eeprom);
		write_float(addr, motor[m]->limit_a, eeprom);
	}
}

int16_t Space::size_std() {
	return 2 * 3 * num_motors + sizeof(float) * (3 * num_axes + 3 * num_motors);
}

void Space::load(int16_t &addr, bool eeprom)
{
	uint8_t t = type;
	type = read_8(addr, eeprom);
	if (t != type) {
		space_types[t].free(this);
		space_types[type].init(this);
	}
	space_types[type].load(this, addr, eeprom);
}

void Space::save(int16_t &addr, bool eeprom)
{
	write_8(addr, type, eeprom);
	space_types[type].save(this, addr, eeprom);
}

int16_t Space::size() {
	return 1 * 1 + space_types[type].size(this);
}

int16_t Space::size0() {
	return 2;	// Cartesian type, 0 axes.
}

void Space::init() {
	type = 0;
	type_data = NULL;
	num_axes = 0;
	num_motors = 0;
	motor = NULL;
	axis = NULL;
}

void Space::free() {
	space_types[type].free(this);
	for (uint8_t a = 0; a < num_axes; ++a)
		delete axis[a];
	for (uint8_t m = 0; m < num_motors; ++m)
		delete motor[m];
	delete motor;
	delete axis;
}

void Space::copy(Space &dst) {
	dst.type = type;
	dst.type_data = NULL;
	dst.num_axes = 0;
	dst.num_motors = 0;
	dst.motor = NULL;
	dst.axis = NULL;
	setup_nums(&dst, num_axes, num_motors);
	for (uint8_t a = 0; a < num_axes; ++a) {
		dst.axis[a]->offset = axis[a]->offset;
		dst.axis[a]->park = axis[a]->park;
		dst.axis[a]->max_v = axis[a]->max_v;
		dst.axis[a]->source = axis[a]->source;
		dst.axis[a]->current = axis[a]->current;
		dst.axis[a]->dist = axis[a]->dist;
		dst.axis[a]->next_dist = axis[a]->next_dist;
		dst.axis[a]->main_dist = axis[a]->main_dist;
	}
	for (uint8_t m = 0; m < num_motors; ++m) {
		dst.motor[m]->step_pin.flags = 0;
		dst.motor[m]->step_pin.read(motor[m]->dir_pin.write());
		dst.motor[m]->dir_pin.flags = 0;
		dst.motor[m]->dir_pin.read(motor[m]->dir_pin.write());
		dst.motor[m]->enable_pin.flags = 0;
		dst.motor[m]->enable_pin.read(motor[m]->enable_pin.write());
		dst.motor[m]->limit_min_pin.flags = 0;
		dst.motor[m]->limit_min_pin.read(motor[m]->limit_min_pin.write());
		dst.motor[m]->limit_max_pin.flags = 0;
		dst.motor[m]->limit_max_pin.read(motor[m]->limit_max_pin.write());
		dst.motor[m]->sense_pin.flags = 0;
		dst.motor[m]->sense_pin.read(motor[m]->sense_pin.write());
		dst.motor[m]->steps_per_m = motor[m]->steps_per_m;
		dst.motor[m]->max_steps = motor[m]->max_steps;
		dst.motor[m]->home_pos = motor[m]->home_pos;
		dst.motor[m]->motor_min = motor[m]->motor_min;
		dst.motor[m]->motor_max = motor[m]->motor_max;
		dst.motor[m]->sense_state = motor[m]->sense_state;
		dst.motor[m]->sense_pos = motor[m]->sense_pos;
		dst.motor[m]->limits_pos = motor[m]->limits_pos;
		dst.motor[m]->limit_v = motor[m]->limit_v;
		dst.motor[m]->limit_a = motor[m]->limit_a;
		dst.motor[m]->last_time = motor[m]->last_time;
		dst.motor[m]->last_v = motor[m]->last_v;
		dst.motor[m]->last_distance = motor[m]->last_distance;
		dst.motor[m]->positive = motor[m]->positive;
		dst.motor[m]->current_pos = motor[m]->current_pos;
#ifdef HAVE_AUDIO
		dst.motor[m]->audio_flags = 0;
#endif
	}
	space_types[type].copy(this, &dst);
}
#endif

#include "firmware.h"

#if 0
#define loaddebug debug
#else
#define loaddebug(...) do {} while(0)
#endif

#ifdef HAVE_SPACES
bool Space::setup_nums(uint8_t na, uint8_t nm) {
	if (na == num_axes && nm == num_motors)
		return true;
	loaddebug("new space %d %d %d %d", na, nm, num_axes, num_motors);
	int16_t memsz = namelen + 1;
	int16_t savesz = namelen + 1;
	memsz += globals_memsize();
	savesz += globals_savesize();
#ifdef HAVE_TEMPS
	for (uint8_t t = 0; t < num_temps; ++t) {
		memsz += t < num_temps ? temps[t].memsize() : Temp::memsize0();
		savesz += t < num_temps ? temps[t].savesize() : Temp::savesize0();
	}
#endif
#ifdef HAVE_GPIOS
	for (uint8_t g = 0; g < num_gpios; ++g) {
		memsz += g < num_gpios ? gpios[g].memsize() : Gpio::memsize0();
		savesz += g < num_gpios ? gpios[g].savesize() : Gpio::savesize0();
	}
#endif
	for (uint8_t s = 0; s < num_spaces; ++s) {
		memsz += s < num_spaces ? spaces[s].memsize() : Space::memsize0();
		savesz += s < num_spaces ? spaces[s].savesize() : Space::savesize0();
	}
	memsz -= memsize();
	savesz -= savesize();
	uint8_t old_nm = num_motors;
	uint8_t old_na = num_axes;
	num_motors = nm;
	num_axes = na;
	memsz += memsize();
	savesz += savesize();
	if (memsz > (XRAMEND - RAMSTART) / 2) {
		debug("New space settings make memsize %d, which is larger than half of %d: rejecting.", memsz, XRAMEND - RAMSTART);
		num_motors = old_nm;
		num_axes = old_na;
		return false;
	}
	if (savesz > E2END) {
		debug("New space settings make size %d, which is larger than %d: rejecting.", savesz, E2END);
		num_motors = old_nm;
		num_axes = old_na;
		return false;
	}
	if (na != old_na) {
		Axis **new_axes = new Axis *[na];
		loaddebug("new axes: %d %d %x", na, new_axes);
		for (uint8_t a = 0; a < min(old_na, na); ++a)
			new_axes[a] = axis[a];
		for (uint8_t a = old_na; a < na; ++a) {
			new_axes[a] = new Axis();
			loaddebug("new axis %x", new_axes[a]);
			new_axes[a]->source = NAN;
			new_axes[a]->current = NAN;
			new_axes[a]->dist = NAN;
			new_axes[a]->next_dist = NAN;
			new_axes[a]->main_dist = NAN;
		}
		for (uint8_t a = na; a < old_na; ++a)
			delete axis[a];
		delete[] axis;
		axis = new_axes;
	}
	if (nm != old_nm) {
		Motor **new_motors = new Motor *[nm];
		loaddebug("new motors: %d %x", nm, new_motors);
		for (uint8_t m = 0; m < min(old_nm, nm); ++m)
			new_motors[m] = motor[m];
		for (uint8_t m = old_nm; m < nm; ++m) {
			new_motors[m] = new Motor();
			loaddebug("new motor %x", new_motors[m]);
			new_motors[m]->sense_state = 0;
			new_motors[m]->sense_pos = NAN;
			new_motors[m]->motor_min = -INFINITY;
			new_motors[m]->motor_max = INFINITY;
			new_motors[m]->limits_pos = NAN;
			new_motors[m]->limit_v = INFINITY;
			new_motors[m]->limit_a = INFINITY;
			new_motors[m]->home_order = 0;
			new_motors[m]->last_time = micros();
			new_motors[m]->last_v = 0;
			new_motors[m]->last_distance = 0;
			new_motors[m]->current_pos = NAN;
#ifdef HAVE_AUDIO
			new_motors[m]->audio_flags = 0;
#endif
		}
		for (uint8_t m = nm; m < old_nm; ++m)
			delete motor[m];
		delete[] motor;
		motor = new_motors;
	}
	return true;
}

static void move_to_current(Space *s) {
	if (moving || !motors_busy)
		return;
	s->active = true;
	f0 = 0;
	fmain = 1;
	fp = 0;
	fq = 0;
	t0 = 0;
	tp = 0;
	current_move_has_cb = 0;
	moving = true;
	start_time = micros();
	for (uint8_t i = 0; i < 3; ++i) {
		s->axis[i]->dist = 0;
		s->axis[i]->next_dist = 0;
		s->axis[i]->main_dist = 0;
		s->motor[i]->last_v = 0;
		s->motor[i]->last_time = start_time;
		s->motor[i]->last_distance = 0;
	}
}

void Space::load_info(int16_t &addr, bool eeprom)
{
	loaddebug("loading space");
	uint8_t t = type;
	if (t >= NUM_SPACE_TYPES || !have_type[t])
		t = DEFAULT_TYPE;
	type = read_8(addr, eeprom);
	if (type >= NUM_SPACE_TYPES || !have_type[type]) {
		debug("request for type %d ignored", type);
		type = t;
	}
	float oldpos[num_motors], xyz[num_axes];
	bool ok;
	if (t != type) {
		//debug("setting type to %d", type);
		space_types[t].free(this);
		space_types[type].init(this);
		ok = false;
	}
	else {
		if (moving || !motors_busy)
			ok = false;
		else {
			ok = true;
			for (uint8_t i = 0; i < num_motors; ++i) {
				if (isnan(motor[i]->current_pos))
					ok = false;
			}
			if (ok) {
				for (uint8_t i = 0; i < num_axes; ++i)
					xyz[i] = axis[i]->current;
				ok = true;
				space_types[type].xyz2motors(this, xyz, oldpos, &ok);
			}
		}
	}
	max_deviation = read_float(addr, eeprom);
	space_types[type].load(this, t, addr, eeprom);
	if (ok) {
		float newpos[num_motors];
		space_types[type].xyz2motors(this, xyz, newpos, &ok);
		uint8_t i;
		for (i = 0; i < num_motors; ++i) {
			if (oldpos[i] != newpos[i]) {
				move_to_current(this);
				break;
			}
		}
	}
}

void Space::load_axis(uint8_t a, int16_t &addr, bool eeprom)
{
	loaddebug("loading axis %d", a);
	float old_offset = axis[a]->offset;
	axis[a]->offset = read_float(addr, eeprom);
	axis[a]->park = read_float(addr, eeprom);
	axis[a]->max_v = read_float(addr, eeprom);
	if (axis[a]->offset != old_offset)
		move_to_current(this);
}

void Space::load_motor(uint8_t m, int16_t &addr, bool eeprom)
{
	loaddebug("loading motor %d %x", m, motor[m]);
	uint16_t enable = motor[m]->enable_pin.write();
	float old_home_pos = motor[m]->home_pos;
	float old_steps_per_m = motor[m]->steps_per_m;
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
	motor[m]->home_order = read_8(addr, eeprom);
	SET_OUTPUT(motor[m]->step_pin);
	SET_OUTPUT(motor[m]->dir_pin);
	SET_OUTPUT(motor[m]->enable_pin);
	if (enable != motor[m]->enable_pin.write())
		RESET(motor[m]->enable_pin);
	RESET(motor[m]->step_pin);
	SET_INPUT(motor[m]->limit_min_pin);
	SET_INPUT(motor[m]->limit_max_pin);
	SET_INPUT(motor[m]->sense_pin);
	if (old_steps_per_m != motor[m]->steps_per_m) {
		motor[m]->current_pos *= motor[m]->steps_per_m / old_steps_per_m;
		// This makes sure that the new position matches the target again, so no need to move there.
	}
	if (old_home_pos != motor[m]->home_pos) {
		motor[m]->current_pos += motor[m]->home_pos - old_home_pos;
		move_to_current(this);
	}
}

void Space::save_info(int16_t &addr, bool eeprom)
{
	//debug("saving info %d %f %d %d", type, F(max_deviation), num_axes, num_motors);
	write_8(addr, type, eeprom);
	write_float(addr, max_deviation, eeprom);
	space_types[type].save(this, addr, eeprom);
}

void Space::save_axis(uint8_t a, int16_t &addr, bool eeprom) {
	write_float(addr, axis[a]->offset, eeprom);
	write_float(addr, axis[a]->park, eeprom);
	write_float(addr, axis[a]->max_v, eeprom);
}

void Space::save_motor(uint8_t m, int16_t &addr, bool eeprom) {
	write_16(addr, motor[m]->step_pin.write(), eeprom);
	write_16(addr, motor[m]->dir_pin.write(), eeprom);
	write_16(addr, motor[m]->enable_pin.write(), eeprom);
	write_16(addr, motor[m]->limit_min_pin.write(), eeprom);
	write_16(addr, motor[m]->limit_max_pin.write(), eeprom);
	write_16(addr, motor[m]->sense_pin.write(), eeprom);
	write_float(addr, motor[m]->steps_per_m, eeprom);
	write_8(addr, motor[m]->max_steps, eeprom);
	write_float(addr, motor[m]->home_pos, eeprom);
	write_float(addr, motor[m]->motor_min, eeprom);
	write_float(addr, motor[m]->motor_max, eeprom);
	write_float(addr, motor[m]->limit_v, eeprom);
	write_float(addr, motor[m]->limit_a, eeprom);
	write_8(addr, motor[m]->home_order, eeprom);
}

int16_t Space::memsize_std() {
	return num_axes * (sizeof(Axis) + sizeof(Axis *)) + num_motors * (sizeof(Motor) + sizeof(Motor *));
}

int16_t Space::savesize_std() {
	return (2 * 6 + 4 * 6 + 1 * 2) * num_motors + sizeof(float) * (1 + 3 * num_axes + 3 * num_motors);
}

int16_t Space::memsize() {
	return 1 * 1 + space_types[type].memsize(this);
}

int16_t Space::savesize() {
	return 1 * 1 + space_types[type].savesize(this);
}

int16_t Space::memsize0() {
	return 2;	// Cartesian type, 0 axes.
}

int16_t Space::savesize0() {
	return 2;	// Cartesian type, 0 axes.
}

void Space::init() {
	type = DEFAULT_TYPE;
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
	//debug("copy space");
	dst.type = type;
	dst.type_data = type_data;
	dst.num_axes = num_axes;
	dst.num_motors = num_motors;
	dst.motor = motor;
	dst.axis = axis;
}
#endif

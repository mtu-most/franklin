#include "cdriver.h"

#if 0
#define loaddebug debug
#else
#define loaddebug(...) do {} while(0)
#endif

bool Space::setup_nums(uint8_t na, uint8_t nm) {
	if (na == num_axes && nm == num_motors)
		return true;
	loaddebug("new space %d %d %d %d", na, nm, num_axes, num_motors);
	int32_t savesz = namelen + 1;
	savesz += globals_savesize();
	for (uint8_t t = 0; t < num_temps; ++t) {
		savesz += t < num_temps ? temps[t].savesize() : Temp::savesize0();
	}
	for (uint8_t g = 0; g < num_gpios; ++g) {
		savesz += g < num_gpios ? gpios[g].savesize() : Gpio::savesize0();
	}
	for (uint8_t s = 0; s < num_spaces; ++s) {
		savesz += s < num_spaces ? spaces[s].savesize() : Space::savesize0();
	}
	savesz -= savesize();
	uint8_t old_nm = num_motors;
	uint8_t old_na = num_axes;
	num_motors = nm;
	num_axes = na;
	savesz += savesize();
	if (savesz > E2END) {
		debug("New space settings make size %d, which is larger than %d: rejecting.", savesz, E2END);
		num_motors = old_nm;
		num_axes = old_na;
		return false;
	}
	if (na != old_na) {
		Axis **new_axes;
		mem_alloc(sizeof(Axis *) * na, &new_axes, "axes");
		if (!new_axes) {
			num_motors = old_nm;
			num_axes = old_na;
			return false;
		}
		loaddebug("new axes: %d %d %x", na, new_axes);
		for (uint8_t a = 0; a < min(old_na, na); ++a)
			mem_retarget(&axis[a], &new_axes[a]);
		for (uint8_t a = old_na; a < na; ++a) {
			mem_alloc(sizeof(Axis), &new_axes[a], "axis");
			if (!new_axes[a]) {
				num_motors = old_nm;
				num_axes = a;
				return false;
			}
			loaddebug("new axis %x", new_axes[a]);
			new_axes[a]->source = NAN;
			new_axes[a]->current = NAN;
			new_axes[a]->dist = NAN;
			new_axes[a]->next_dist = NAN;
			new_axes[a]->main_dist = NAN;
			new_axes[a]->offset = 0;
			new_axes[a]->park = NAN;
			new_axes[a]->park_order = 0;
			new_axes[a]->max_v = INFINITY;
			new_axes[a]->min = -INFINITY;
			new_axes[a]->max = INFINITY;
			new_axes[a]->target = NAN;
		}
		for (uint8_t a = na; a < old_na; ++a)
			mem_free(&axis[a]);
		mem_free(&axis);
		mem_retarget(&new_axes, &axis);
	}
	if (nm != old_nm) {
		Motor **new_motors;
		mem_alloc(sizeof(Motor *) * nm, &new_motors, "motors");
		if (!new_motors) {
			num_motors = old_nm;
			return false;
		}
		loaddebug("new motors: %d %x", nm, new_motors);
		for (uint8_t m = 0; m < min(old_nm, nm); ++m)
			mem_retarget(&motor[m], &new_motors[m]);
		for (uint8_t m = old_nm; m < nm; ++m) {
			mem_alloc(sizeof(Motor), &new_motors[m], "motor");
			if (!new_motors[m]) {
				num_motors = m;
				return false;
			}
			loaddebug("new motor %x", new_motors[m]);
			new_motors[m]->step_pin.init();
			new_motors[m]->dir_pin.init();
			new_motors[m]->enable_pin.init();
			new_motors[m]->limit_min_pin.init();
			new_motors[m]->limit_max_pin.init();
			new_motors[m]->sense_pin.init();
			new_motors[m]->sense_state = 0;
			new_motors[m]->sense_pos = NAN;
			new_motors[m]->steps_per_m = NAN;
			new_motors[m]->max_steps = 1;
			new_motors[m]->limit_v = INFINITY;
			new_motors[m]->limit_a = INFINITY;
			new_motors[m]->home_pos = NAN;
			new_motors[m]->home_order = 0;
			new_motors[m]->last_v = 0;
			new_motors[m]->current_pos = 0;
			new_motors[m]->limit_v = INFINITY;
			new_motors[m]->limit_a = INFINITY;
#ifdef HAVE_AUDIO
			new_motors[m]->audio_flags = 0;
#endif
			new_motors[m]->last_v = NAN;
			new_motors[m]->target_v = NAN;
			new_motors[m]->target_dist = NAN;
			new_motors[m]->endpos = NAN;
		}
		for (uint8_t m = nm; m < old_nm; ++m)
			mem_free(&motor[m]);
		mem_free(&motor);
		mem_retarget(&new_motors, &motor);
		arch_motors_change();
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
	cbs_after_current_move = 0;
	moving = true;
	next_motor_time = 0;
	start_time = utime();
	last_time = start_time;
	last_current_time = start_time;
	//debug("move to current");
	for (uint8_t i = 0; i < min(s->num_axes, s->num_motors); ++i) {
		s->axis[i]->dist = 0;
		s->axis[i]->next_dist = 0;
		s->axis[i]->main_dist = 0;
		s->motor[i]->last_v = 0;
	}
}

void Space::load_info(int32_t &addr, bool eeprom)
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
	float oldpos[num_motors];
	bool ok;
	if (t != type) {
		//debug("setting type to %d", type);
		space_types[t].free(this);
		if (!space_types[type].init(this)) {
			type = DEFAULT_TYPE;
			return;	// The rest of the package is not meant for DEFAULT_TYPE, so ignore it.
		}
		ok = false;
	}
	else {
		if (moving || !motors_busy)
			ok = false;
		else {
			for (uint8_t i = 0; i < num_axes; ++i)
				axis[i]->target = axis[i]->current;
			ok = true;
			space_types[type].xyz2motors(this, oldpos, &ok);
		}
	}
	max_deviation = read_float(addr, eeprom);
	space_types[type].load(this, t, addr, eeprom);
	if (ok) {
		float newpos[num_motors];
		space_types[type].xyz2motors(this, newpos, &ok);
		uint8_t i;
		for (i = 0; i < num_motors; ++i) {
			if (oldpos[i] != newpos[i]) {
				move_to_current(this);
				break;
			}
		}
	}
	loaddebug("done loading space");
}

void Space::load_axis(uint8_t a, int32_t &addr, bool eeprom)
{
	loaddebug("loading axis %d", a);
	float old_offset = axis[a]->offset;
	axis[a]->offset = read_float(addr, eeprom);
	axis[a]->park = read_float(addr, eeprom);
	axis[a]->park_order = read_8(addr, eeprom);
	axis[a]->max_v = read_float(addr, eeprom);
	axis[a]->min = read_float(addr, eeprom);
	axis[a]->max = read_float(addr, eeprom);
	if (axis[a]->offset != old_offset) {
		axis[a]->current += axis[a]->offset - old_offset;
		axis[a]->source += axis[a]->offset - old_offset;
		move_to_current(this);
	}
}

void Space::load_motor(uint8_t m, int32_t &addr, bool eeprom)
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
	motor[m]->limit_v = read_float(addr, eeprom);
	motor[m]->limit_a = read_float(addr, eeprom);
	motor[m]->home_order = read_8(addr, eeprom);
	arch_motors_change();
	SET_OUTPUT(motor[m]->enable_pin);
	if (enable != motor[m]->enable_pin.write()) {
		if (motors_busy)
			SET(motor[m]->enable_pin);
		else {
			RESET(motor[m]->enable_pin);
		}
	}
	RESET(motor[m]->step_pin);
	SET_INPUT(motor[m]->limit_min_pin);
	SET_INPUT(motor[m]->limit_max_pin);
	SET_INPUT(motor[m]->sense_pin);
	bool must_move = false;
	if (!isnan(motor[m]->home_pos)) {
		// Axes with a limit switch.
		if (old_steps_per_m != motor[m]->steps_per_m) {
			float diff = motor[m]->current_pos / old_steps_per_m - motor[m]->home_pos;
			float f = motor[m]->home_pos + diff;
			motor[m]->current_pos = f * motor[m]->steps_per_m + (f > 0 ? .49 : -.49);
			//debug("cp5 %d", motor[m]->current_pos);
			arch_setpos(id, m);
			must_move = true;
		}
		if (motors_busy && old_home_pos != motor[m]->home_pos) {
			float f = motor[m]->home_pos - old_home_pos;
			int32_t diff = f * motor[m]->steps_per_m + (f > 0 ? .49 : -.49);
			motor[m]->current_pos += diff;
			//debug("cp6 %d", motor[m]->current_pos);
			arch_addpos(id, m, diff);
			must_move = true;
		}
	}
	else {
		// Axes without a limit switch: extruders.
		if (motors_busy && old_steps_per_m != motor[m]->steps_per_m) {
			int32_t cp = motor[m]->current_pos;
			float pos = motor[m]->current_pos / old_steps_per_m;
			motor[m]->current_pos = pos * motor[m]->steps_per_m;
			arch_addpos(id, m, motor[m]->current_pos - cp);
		}
	}
	if (must_move)
		move_to_current(this);
}

void Space::save_info(int32_t &addr, bool eeprom)
{
	//debug("saving info %d %f %d %d", type, F(max_deviation), num_axes, num_motors);
	write_8(addr, type, eeprom);
	write_float(addr, max_deviation, eeprom);
	space_types[type].save(this, addr, eeprom);
}

void Space::save_axis(uint8_t a, int32_t &addr, bool eeprom) {
	write_float(addr, axis[a]->offset, eeprom);
	write_float(addr, axis[a]->park, eeprom);
	write_8(addr, axis[a]->park_order, eeprom);
	write_float(addr, axis[a]->max_v, eeprom);
	write_float(addr, axis[a]->min, eeprom);
	write_float(addr, axis[a]->max, eeprom);
}

void Space::save_motor(uint8_t m, int32_t &addr, bool eeprom) {
	write_16(addr, motor[m]->step_pin.write(), eeprom);
	write_16(addr, motor[m]->dir_pin.write(), eeprom);
	write_16(addr, motor[m]->enable_pin.write(), eeprom);
	write_16(addr, motor[m]->limit_min_pin.write(), eeprom);
	write_16(addr, motor[m]->limit_max_pin.write(), eeprom);
	write_16(addr, motor[m]->sense_pin.write(), eeprom);
	write_float(addr, motor[m]->steps_per_m, eeprom);
	write_8(addr, motor[m]->max_steps, eeprom);
	write_float(addr, motor[m]->home_pos, eeprom);
	write_float(addr, motor[m]->limit_v, eeprom);
	write_float(addr, motor[m]->limit_a, eeprom);
	write_8(addr, motor[m]->home_order, eeprom);
}

int32_t Space::savesize_std() {
	return (2 * 6 + 4 * 6 + 1 * 2) * num_motors + sizeof(float) * (1 + 3 * num_axes + 3 * num_motors);
}

int32_t Space::savesize() {
	return 1 * 1 + space_types[type].savesize(this);
}

int32_t Space::savesize0() {
	return 2;	// Cartesian type, 0 axes.
}

void Space::init(uint8_t space_id) {
	type = DEFAULT_TYPE;
	id = space_id;
	max_deviation = 0;
	type_data = NULL;
	num_axes = 0;
	num_motors = 0;
	motor = NULL;
	axis = NULL;
}

void Space::free() {
	space_types[type].free(this);
	for (uint8_t a = 0; a < num_axes; ++a)
		mem_free(&axis[a]);
	mem_free(&axis);
	for (uint8_t m = 0; m < num_motors; ++m) {
		motor[m]->step_pin.read(0);
		motor[m]->dir_pin.read(0);
		motor[m]->enable_pin.read(0);
		motor[m]->limit_min_pin.read(0);
		motor[m]->limit_max_pin.read(0);
		motor[m]->sense_pin.read(0);
		mem_free(&motor[m]);
	}
	mem_free(&motor);
}

void Space::copy(Space &dst) {
	//debug("copy space");
	dst.type = type;
	mem_retarget(&type_data, &dst.type_data);
	dst.num_axes = num_axes;
	dst.num_motors = num_motors;
	mem_retarget(&axis, &dst.axis);
	mem_retarget(&motor, &dst.motor);
}

void Space::cancel_update() {
	// setup_nums failed; restore system to a usable state.
	type = DEFAULT_TYPE;
	uint8_t n = min(num_axes, num_motors);
	if (!setup_nums(n, n)) {
		debug("Failed to free memory; erasing name and removing all motors and axes to make sure it works");
		mem_free(&name);
		namelen = 0;
		if (!setup_nums(0, 0))
			debug("You're in trouble; this shouldn't be possible");
	}
}

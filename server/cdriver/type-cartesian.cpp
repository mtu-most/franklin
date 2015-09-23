#include "cdriver.h"

static void xyz2motors(Space *s, double *motors, bool *ok) {
	for (uint8_t a = 0; a < s->num_axes; ++a) {
		if (motors)
			motors[a] = s->axis[a]->settings.target;
		else
			s->motor[a]->settings.endpos = s->axis[a]->settings.target;
	}
}

static void reset_pos(Space *s) {
	// If positions are unknown, pretend that they are 0.
	// This is mostly useful for extruders.
	for (uint8_t a = 0; a < s->num_axes; ++a) {
		s->axis[a]->settings.source = s->motor[a]->settings.current_pos / s->motor[a]->steps_per_unit;
		//debug("set pos for %d to %f", a, s->axis[a]->settings.source);
	}
}

static void check_position(Space *s, double *data) {
}

static void load(Space *s, uint8_t old_type, int32_t &addr) {
	uint8_t num = read_8(addr);
	if (!s->setup_nums(num, num)) {
		debug("Failed to set up cartesian axes");
		s->cancel_update();
	}
}

static void save(Space *s, int32_t &addr) {
	write_8(addr, s->num_axes);
}

static bool init(Space *s) {
	return true;
}

static void free(Space *s) {
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

void Cartesian_init(int num) {
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

struct ExtruderData {
	int num_axes;
};

struct ExtruderAxisData {
	double offset[3];
};

#define EDATA(s) (*reinterpret_cast <ExtruderData *>(s->type_data))
#define EADATA(s, a) (*reinterpret_cast <ExtruderAxisData *>(s->axis[a]->type_data))

static void eload(Space *s, uint8_t old_type, int32_t &addr) {
	uint8_t num = read_8(addr);
	if (!s->setup_nums(num, num)) {
		debug("Failed to set up extruder axes");
		uint8_t n = min(s->num_axes, s->num_motors);
		if (!s->setup_nums(n, n)) {
			debug("Trouble!  Failed to abort.  Cancelling.");
			s->cancel_update();
		}
	}
	for (int a = EDATA(s).num_axes; a < s->num_axes; ++a) {
		s->axis[a]->type_data = new ExtruderAxisData;
		for (int i = 0; i < 3; ++i)
			EADATA(s, a).offset[i] = 0;
	}
	EDATA(s).num_axes = s->num_axes;
	bool move = false;
	if (motors_busy && stopped && settings.queue_start == settings.queue_end && !settings.queue_full) {
		move = true;
		queue[settings.queue_end].probe = false;
		queue[settings.queue_end].cb = false;
		queue[settings.queue_end].f[0] = INFINITY;
		queue[settings.queue_end].f[1] = INFINITY;
		for (int i = 0; i < spaces[0].num_axes; ++i) {
			queue[settings.queue_end].data[i] = spaces[0].axis[i]->settings.current;
			for (int ss = 0; ss < 2; ++ss)
				queue[settings.queue_end].data[i] = space_types[spaces[ss].type].unchange0(&spaces[ss], i, queue[settings.queue_end].data[i]);
			if (i == 2)
				queue[settings.queue_end].data[i] -= zoffset;
		}
		for (int i = spaces[0].num_axes; i < QUEUE_LENGTH; ++i) {
			queue[settings.queue_end].data[i] = NAN;
		}
		cpdebug(0, 0, "eload end");
		settings.queue_end = (settings.queue_end + 1) % QUEUE_LENGTH;
		// This shouldn't happen and causes communication problems, but if you have a 1-item buffer it is correct.
		if (settings.queue_end == settings.queue_start)
			settings.queue_full = true;
	}
	for (int a = 0; a < s->num_axes; ++a) {
		for (int o = 0; o < 3; ++o)
			EADATA(s, a).offset[o] = read_float(addr);
	}
	if (move) {
		next_move();
		buffer_refill();
	}
}

static void esave(Space *s, int32_t &addr) {
	write_8(addr, s->num_axes);
	for (int a = 0; a < s->num_axes; ++a) {
		for (int o = 0; o < 3; ++o)
			write_float(addr, EADATA(s, a).offset[o]);
	}
}

static bool einit(Space *s) {
	s->type_data = new ExtruderData;
	EDATA(s).num_axes = 0;
	return true;
}

static void efree(Space *s) {
	delete reinterpret_cast <ExtruderData *>(s->type_data);
}

static void eafree(Space *s, int a) {
	delete reinterpret_cast <ExtruderAxisData *>(s->axis[a]->type_data);
}

static double echange0(Space *s, int axis, double value) {
	if (current_extruder >= s->num_axes || axis >= 3)
		return value;
	return value + EADATA(s, current_extruder).offset[axis];
}

static double eunchange0(Space *s, int axis, double value) {
	if (current_extruder >= s->num_axes || axis >= 3)
		return value;
	return value - EADATA(s, current_extruder).offset[axis];
}

void Extruder_init(int num) {
	space_types[num].xyz2motors = xyz2motors;
	space_types[num].reset_pos = reset_pos;
	space_types[num].check_position = check_position;
	space_types[num].load = eload;
	space_types[num].save = esave;
	space_types[num].init = einit;
	space_types[num].free = efree;
	space_types[num].afree = eafree;
	space_types[num].change0 = echange0;
	space_types[num].unchange0 = eunchange0;
	space_types[num].probe_speed = probe_speed;
}

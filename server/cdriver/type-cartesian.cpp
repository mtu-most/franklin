#include "cdriver.h"

static void xyz2motors(Space *s, float *motors, bool *ok) {
	for (uint8_t a = 0; a < s->num_axes; ++a) {
		if (motors)
			motors[a] = s->axis[a]->settings[current_fragment].target;
		else
			s->motor[a]->settings[current_fragment].endpos = s->axis[a]->settings[current_fragment].target;
	}
}

static void reset_pos(Space *s) {
	// If positions are unknown, pretend that they are 0.
	// This is mostly useful for extruders.
	for (uint8_t a = 0; a < s->num_axes; ++a) {
		s->axis[a]->source = s->motor[a]->settings[current_fragment].current_pos / s->motor[a]->steps_per_m;
		//debug("set pos for %d to %f", a, F(s->axis[a]->source));
	}
}

static void check_position(Space *s, float *data) {
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

static void change0(Space *s, int qpos) {
}

void Cartesian_init(uint8_t num) {
	space_types[num].xyz2motors = xyz2motors;
	space_types[num].reset_pos = reset_pos;
	space_types[num].check_position = check_position;
	space_types[num].load = load;
	space_types[num].save = save;
	space_types[num].init = init;
	space_types[num].free = free;
	space_types[num].afree = afree;
	space_types[num].change0 = change0;
}

struct ExtruderData {
	int current;
};

struct ExtruderAxisData {
	float offset[3];
};

#define EDATA(s) (*reinterpret_cast <ExtruderData *>(s->type_data))
#define EADATA(s, a) (*reinterpret_cast <ExtruderAxisData *>(s->axis[a]->type_data))

static void eload(Space *s, uint8_t old_type, int32_t &addr) {
	uint8_t num = read_8(addr);
	if (!s->setup_nums(num, num)) {
		debug("Failed to set up cartesian axes");
		uint8_t n = min(s->num_axes, s->num_motors);
		if (!s->setup_nums(n, n)) {
			debug("Trouble!  Failed to abort.  Cancelling.");
			s->cancel_update();
		}
	}
	for (int a = 0; a < s->num_axes; ++a) {
		s->axis[a]->type_data = new ExtruderAxisData;
		for (int o = 0; o < 3; ++o)
			EADATA(s, a).offset[o] = read_float(addr);
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
	EDATA(s).current = 0;
	return true;
}

static void efree(Space *s) {
	delete reinterpret_cast <ExtruderData *>(s->type_data);
}

static void eafree(Space *s, int a) {
	delete reinterpret_cast <ExtruderAxisData *>(s->axis[a]->type_data);
}

static void echange0(Space *s, int qpos) {
	int o = 0;
	for (int ss = 0; &spaces[ss] != s; ++ss)
		o += spaces[ss].num_axes;
	for (int c = 0; c < s->num_axes; ++c) {
		if (!isnan(queue[qpos].data[o + c])) {
			EDATA(s).current = c;
			break;
		}
	}
	if (EDATA(s).current >= s->num_axes)
		return;
	for (int a = 0; a < min(3, spaces[0].num_axes); ++a)
		queue[qpos].data[a] -= EADATA(s, EDATA(s).current).offset[a];
	return;
}

void Extruder_init(uint8_t num) {
	space_types[num].xyz2motors = xyz2motors;
	space_types[num].reset_pos = reset_pos;
	space_types[num].check_position = check_position;
	space_types[num].load = eload;
	space_types[num].save = esave;
	space_types[num].init = einit;
	space_types[num].free = efree;
	space_types[num].afree = eafree;
	space_types[num].change0 = echange0;
}

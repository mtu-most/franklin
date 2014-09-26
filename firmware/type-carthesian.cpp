#include "firmware.h"

#ifdef HAVE_SPACES
static void xyz2motors(Space *s, float *motors, bool *ok) {
	for (uint8_t a = 0; a < s->num_axes; ++a) {
		if (motors)
			motors[a] = s->axis[a]->target;
		else
			s->motor[a]->endpos = s->axis[a]->target;
	}
}

static void reset_pos (Space *s) {
	// If positions are unknown, pretend that they are 0.
	// This is mostly useful for extruders.
	for (uint8_t a = 0; a < s->num_axes; ++a) {
		if (isnan(s->motor[a]->current_pos)) {
			s->axis[a]->source = 0;
			s->motor[a]->current_pos = 0;
		}
		else
			s->axis[a]->source = s->motor[a]->current_pos;
		//debug("set pos for %d to %f", a, F(s->axis[a]->source));
	}
}

static void check_position(Space *s, float *data) {
}

static void load(Space *s, uint8_t old_type, int16_t &addr, bool eeprom) {
	uint8_t num = read_8(addr, eeprom);
	if (!s->setup_nums(num, num)) {
		debug("Failed to set up cartesian axes");
		s->cancel_update();
	}
}

static void save(Space *s, int16_t &addr, bool eeprom) {
	write_8(addr, s->num_axes, eeprom);
}

static bool init(Space *s) {
	return true;
}

static void free(Space *s) {
}

static int16_t savesize(Space *s) {
	return 1 * 1 + s->savesize_std();
}

static bool change0(Space *s) {
	return true;
}

void Cartesian_init(uint8_t num) {
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

#ifdef HAVE_EXTRUDER
struct ExtruderData {
	float dx, dy, dz;
};

#define EDATA(s) (*reinterpret_cast <ExtruderData *>(s->type_data))

static void eload(Space *s, uint8_t old_type, int16_t &addr, bool eeprom) {
	EDATA(s).dx = read_float(addr, eeprom);
	EDATA(s).dy = read_float(addr, eeprom);
	EDATA(s).dz = read_float(addr, eeprom);
	uint8_t num = read_8(addr, eeprom);
	if (!s->setup_nums(num, num)) {
		debug("Failed to set up cartesian axes");
		uint8_t n = min(s->num_axes, s->num_motors);
		if (!s->setup_nums(n, n)) {
			debug("Trouble!  Failed to abort.  Cancelling.");
			s->cancel_update();
		}
	}
}

static void esave(Space *s, int16_t &addr, bool eeprom) {
	write_float(addr, EDATA(s).dx, eeprom);
	write_float(addr, EDATA(s).dy, eeprom);
	write_float(addr, EDATA(s).dz, eeprom);
	write_8(addr, s->num_axes, eeprom);
}

static bool einit(Space *s) {
	mem_alloc(sizeof(ExtruderData), &s->type_data, "extruder");
	if (!s->type_data)
		return false;
	return true;
}

static void efree(Space *s) {
	mem_free(&s->type_data);
}

static int16_t esavesize(Space *s) {
	return 1 * 1 + 4 * 3 + s->savesize_std();
}

static bool echange0(Space *s) {
	return true;	// TODO
}

void Extruder_init(uint8_t num) {
	space_types[num].xyz2motors = xyz2motors;
	space_types[num].reset_pos = reset_pos;
	space_types[num].check_position = check_position;
	space_types[num].load = eload;
	space_types[num].save = esave;
	space_types[num].init = einit;
	space_types[num].free = efree;
	space_types[num].savesize = esavesize;
	space_types[num].change0 = echange0;
}
#endif
#endif

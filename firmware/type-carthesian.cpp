#include <firmware.h>

#ifdef HAVE_SPACES
static void xyz2motors(Space *s, float *xyz, float *motors, bool *ok) {
	for (uint8_t a = 0; a < s->num_axes; ++a)
		motors[a] = xyz[a];
}

static void reset_pos (Space *s) {
	for (uint8_t a = 0; a < s->num_axes; ++a)
		s->axis[a]->source = s->motor[a]->current_pos;
}

static void check_position(Space *s, float *data) {
}

static void load(Space *s, int16_t &addr, bool eeprom) {
	debug("loading cartesian from %d", addr);
	uint8_t num = read_8(addr, eeprom);
	s->setup_nums(num, num);
}

static void save(Space *s, int16_t &addr, bool eeprom) {
	write_8(addr, s->num_axes, eeprom);
}

static void init(Space *s) {
}

static void free(Space *s) {
}

static int16_t size(Space *s) {
	return 1 * 1 + s->size_std();
}

void Cartesian_init(uint8_t num) {
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

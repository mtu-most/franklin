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
	uint8_t old_nm = num_motors;
	uint8_t old_na = num_axes;
	num_motors = nm;
	num_axes = na;
	if (na != old_na) {
		Axis **new_axes = new Axis *[na];
		loaddebug("new axes: %d %d %x", na, new_axes);
		for (uint8_t a = 0; a < min(old_na, na); ++a)
			new_axes[a] = axis[a];
		for (uint8_t a = old_na; a < na; ++a) {
			new_axes[a] = new Axis;
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
			new_motors[m] = new Motor;
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
			new_motors[m]->hwcurrent_pos = 0;
			new_motors[m]->limit_v = INFINITY;
			new_motors[m]->limit_a = INFINITY;
#ifdef HAVE_AUDIO
			new_motors[m]->audio_flags = 0;
#endif
			new_motors[m]->last_v = NAN;
			new_motors[m]->target_v = NAN;
			new_motors[m]->target_dist = NAN;
			new_motors[m]->endpos = NAN;
			new_motors[m]->dir = new int[FRAGMENTS_PER_BUFFER];
			for (int f = 0; f < FRAGMENTS_PER_BUFFER; ++f)
				new_motors[m]->dir[f] = 0;
		}
		for (uint8_t m = nm; m < old_nm; ++m) {
			delete[] motor[m]->dir;
			delete motor[m];
		}
		delete[] motor;
		motor = new_motors;
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
	hwtime = 0;
	hwstart_time = 0;
	start_time = 0;
	last_time = 0;
	last_current_time = 0;
	//debug("move to current");
	for (uint8_t i = 0; i < min(s->num_axes, s->num_motors); ++i) {
		s->axis[i]->dist = 0;
		s->axis[i]->next_dist = 0;
		s->axis[i]->main_dist = 0;
		s->motor[i]->last_v = 0;
	}
}

void Space::load_info(int32_t &addr)
{
	loaddebug("loading space");
	uint8_t t = type;
	if (t >= NUM_SPACE_TYPES || !have_type[t])
		t = DEFAULT_TYPE;
	type = read_8(addr);
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
	max_deviation = read_float(addr);
	space_types[type].load(this, t, addr);
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

void Space::load_axis(uint8_t a, int32_t &addr)
{
	loaddebug("loading axis %d", a);
	float old_offset = axis[a]->offset;
	axis[a]->offset = read_float(addr);
	axis[a]->park = read_float(addr);
	axis[a]->park_order = read_8(addr);
	axis[a]->max_v = read_float(addr);
	axis[a]->min = read_float(addr);
	axis[a]->max = read_float(addr);
	if (axis[a]->offset != old_offset) {
		axis[a]->current += axis[a]->offset - old_offset;
		axis[a]->source += axis[a]->offset - old_offset;
		move_to_current(this);
	}
}

void Space::load_motor(uint8_t m, int32_t &addr)
{
	loaddebug("loading motor %d %x", m, motor[m]);
	uint16_t enable = motor[m]->enable_pin.write();
	float old_home_pos = motor[m]->home_pos;
	float old_steps_per_m = motor[m]->steps_per_m;
	motor[m]->step_pin.read(read_16(addr));
	motor[m]->dir_pin.read(read_16(addr));
	motor[m]->enable_pin.read(read_16(addr));
	motor[m]->limit_min_pin.read(read_16(addr));
	motor[m]->limit_max_pin.read(read_16(addr));
	motor[m]->sense_pin.read(read_16(addr));
	motor[m]->steps_per_m = read_float(addr);
	motor[m]->max_steps = read_8(addr);
	motor[m]->home_pos = read_float(addr);
	motor[m]->limit_v = read_float(addr);
	motor[m]->limit_a = read_float(addr);
	motor[m]->home_order = read_8(addr);
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
			uint32_t cpdiff = f * motor[m]->steps_per_m + (f > 0 ? .49 : -.49) - motor[m]->current_pos;
			motor[m]->current_pos += cpdiff;
			motor[m]->hwcurrent_pos += cpdiff;
			//debug("cp5 %d", motor[m]->current_pos);
			arch_addpos(id, m, cpdiff);
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
			motor[m]->hwcurrent_pos += motor[m]->current_pos - cp;
			arch_addpos(id, m, motor[m]->current_pos - cp);
		}
	}
	if (must_move)
		move_to_current(this);
}

void Space::save_info(int32_t &addr)
{
	//debug("saving info %d %f %d %d", type, F(max_deviation), num_axes, num_motors);
	write_8(addr, type);
	write_float(addr, max_deviation);
	space_types[type].save(this, addr);
}

void Space::save_axis(uint8_t a, int32_t &addr) {
	write_float(addr, axis[a]->offset);
	write_float(addr, axis[a]->park);
	write_8(addr, axis[a]->park_order);
	write_float(addr, axis[a]->max_v);
	write_float(addr, axis[a]->min);
	write_float(addr, axis[a]->max);
}

void Space::save_motor(uint8_t m, int32_t &addr) {
	write_16(addr, motor[m]->step_pin.write());
	write_16(addr, motor[m]->dir_pin.write());
	write_16(addr, motor[m]->enable_pin.write());
	write_16(addr, motor[m]->limit_min_pin.write());
	write_16(addr, motor[m]->limit_max_pin.write());
	write_16(addr, motor[m]->sense_pin.write());
	write_float(addr, motor[m]->steps_per_m);
	write_8(addr, motor[m]->max_steps);
	write_float(addr, motor[m]->home_pos);
	write_float(addr, motor[m]->limit_v);
	write_float(addr, motor[m]->limit_a);
	write_8(addr, motor[m]->home_order);
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
		delete axis[a];
	delete[] axis;
	for (uint8_t m = 0; m < num_motors; ++m) {
		motor[m]->step_pin.read(0);
		motor[m]->dir_pin.read(0);
		motor[m]->enable_pin.read(0);
		motor[m]->limit_min_pin.read(0);
		motor[m]->limit_max_pin.read(0);
		motor[m]->sense_pin.read(0);
		delete motor[m];
	}
	delete[] motor;
}

void Space::copy(Space &dst) {
	//debug("copy space");
	dst.type = type;
	dst.type_data = type_data;
	dst.num_axes = num_axes;
	dst.num_motors = num_motors;
	dst.axis = axis;
	dst.motor = motor;
}

void Space::cancel_update() {
	// setup_nums failed; restore system to a usable state.
	type = DEFAULT_TYPE;
	uint8_t n = min(num_axes, num_motors);
	if (!setup_nums(n, n)) {
		debug("Failed to free memory; erasing name and removing all motors and axes to make sure it works");
		delete[] name;
		namelen = 0;
		if (!setup_nums(0, 0))
			debug("You're in trouble; this shouldn't be possible");
	}
}

#if 0
#define movedebug(...) debug(__VA_ARGS__)
#else
#define movedebug(...) do {} while (0)
#endif

// Space things. {{{
static void check_distance(Motor *mtr, float distance, float dt, float &factor) { // {{{
	if (isnan(distance) || distance == 0) {
		mtr->target_dist = 0;
		//mtr->last_v = 0;
		return;
	}
	//debug("cd %f %f", F(distance), F(dt));
	mtr->target_dist = distance;
	mtr->target_v = distance / dt;
	float v = fabs(mtr->target_v);
	int8_t s = (mtr->target_v < 0 ? -1 : 1);
	// When turning around, ignore limits (they shouldn't have been violated anyway).
	if (mtr->last_v * s < 0) {
		//debug("!");
		mtr->last_v = 0;
	}
	// Limit v.
	if (v > mtr->limit_v) {
		//debug("v %f %f", F(v), F(mtr->limit_v));
		distance = (s * mtr->limit_v) * dt;
		v = fabs(distance / dt);
	}
	//debug("cd2 %f %f", F(distance), F(dt));
	// Limit a+.
	float limit_dv = mtr->limit_a * dt;
	if (v - mtr->last_v * s > limit_dv) {
		//debug("a+ %f %f %f %d", F(mtr->target_v), F(limit_dv), F(mtr->last_v), s);
		distance = (limit_dv * s + mtr->last_v) * dt;
		v = fabs(distance / dt);
	}
	//debug("cd3 %f %f", F(distance), F(dt));
	// Limit a-.
	// Distance to travel until end of segment or connection.
	float max_dist = (mtr->endpos - mtr->current_pos / mtr->steps_per_m) * s;
	// Find distance traveled when slowing down at maximum a.
	// x = 1/2 at²
	// t = sqrt(2x/a)
	// v = at
	// v² = 2a²x/a = 2ax
	// x = v²/2a
	float limit_dist = v * v / 2 / mtr->limit_a;
	//debug("max %f limit %f v %f a %f", max_dist, limit_dist, v, mtr->limit_a);
	if (max_dist > 0 && limit_dist > max_dist) {
		//debug("a- %f %f %f %d %d %f", F(mtr->endpos), F(mtr->limit_a), F(max_dist), F(mtr->current_pos), s, dt);
		v = sqrt(max_dist * 2 * mtr->limit_a);
		distance = s * v * dt;
	}
	//debug("cd4 %f %f", F(distance), F(dt)); */
	float f = distance / mtr->target_dist;
	//movedebug("checked %f %f", F(mtr->target_dist), F(distance));
	if (f < factor)
		factor = f;
} // }}}

static void move_axes(Space *s, uint32_t current_time, float &factor) { // {{{
	float motors_target[s->num_motors];
	bool ok = true;
	space_types[s->type].xyz2motors(s, motors_target, &ok);
	// Try again if it didn't work; it should have moved target to a better location.
	if (!ok)
		space_types[s->type].xyz2motors(s, motors_target, &ok);
	//movedebug("ok %d", ok);
	for (uint8_t m = 0; m < s->num_motors; ++m) {
		//movedebug("move %d %f %f %f", m, F(target[m]), F(motors_target[m]), F(s->motor[m]->current_pos));
		check_distance(s->motor[m], motors_target[m] - s->motor[m]->current_pos / s->motor[m]->steps_per_m, (current_time - last_time) / 1e6, factor);
	}
} // }}}

static void do_steps(float &factor, uint32_t current_time) { // {{{
	//debug("steps");
	if (factor <= 0) {
		movedebug("end move");
		for (uint8_t s = 0; s < num_spaces; ++s)
			spaces[s].active = false;
		moving = false;
		return;
	}
	//movedebug("do steps %f %d", F(factor), max_steps);
	bool have_steps = false;
	for (uint8_t s = 0; !have_steps && s < num_spaces; ++s) {
		Space &sp = spaces[s];
		if (!sp.active)
			continue;
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			float num = (mtr.current_pos / mtr.steps_per_m + mtr.target_dist * factor) * mtr.steps_per_m;
			if (mtr.current_pos != int(num + (num > 0 ? .49 : -.49))) {
				//debug("have steps %d %f %f", mtr.current_pos, mtr.target_dist, factor);
				have_steps = true;
				break;
			}
			//debug("no steps yet %d %d", s, m);
		}
	}
	uint32_t the_last_time = last_current_time;
	last_current_time = current_time;
	// If there are no steps to take, wait until there are.
	if (!have_steps)
		return;
	// Adjust start time if factor < 1.
	if (factor > 0 && factor < 1) {
		start_time += (current_time - the_last_time) * ((1 - factor) * .99);
		movedebug("correct: %f %d", F(factor), int(start_time));
	}
	else
		movedebug("no correct: %f %d", F(factor), int(start_time));
	last_time = current_time;
	// Move the motors.
	//debug("start move");
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		if (!sp.active)
			continue;
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			if (mtr.target_dist == 0) {
				//if (mtr.target_v == 0)
					//mtr.last_v = 0;
				continue;
			}
			float target = mtr.current_pos / mtr.steps_per_m + mtr.target_dist * factor;
			//debug("ccp3 %d %d %d %f %f %f %f %f", m, stopping, mtr.current_pos, F(target), F(mtr.last_v), mtr.steps_per_m, mtr.target_dist, factor);
			mtr.current_pos = (target * mtr.steps_per_m + (target > 0 ? .49 : -.49));
			//debug("cp3 %d", mtr.current_pos);
			mtr.last_v = mtr.target_v * factor;
		}
	}
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		if (!sp.active)
			continue;
		for (uint8_t a = 0; a < sp.num_axes; ++a)
			sp.axis[a]->current += (sp.axis[a]->target - sp.axis[a]->current) * factor;
	}
	return;
} // }}}

static void handle_motors(unsigned long long current_time) { // {{{
	// Check for move.
	if (!moving)
		return;
	last_active = current_time;
	float factor = 1;
	float t = (current_time - start_time) / 1e6;
	//buffered_debug("f%f %f %f %ld %ld", F(t), F(t0), F(tp), F(long(current_time)), F(long(start_time)));
	if (t >= t0 + tp) {	// Finish this move and prepare next.
		movedebug("finishing %f %f %f %ld %ld", F(t), F(t0), F(tp), F(long(current_time)), F(long(start_time)));
		//buffered_debug("a");
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			if (!sp.active)
				continue;
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				//debug("before source %d %f %f", a, F(axis[a].source), F(axis[a].motor.dist));
				if (!isnan(sp.axis[a]->dist)) {
					sp.axis[a]->source += sp.axis[a]->dist;
					sp.axis[a]->dist = NAN;
					// Set this here, so it isn't set if dist was NaN to begin with.
					// Note that target is not set for future iterations, but it isn't changed.
					sp.axis[a]->target = sp.axis[a]->source;
				}
				//debug("after source %d %f %f %f %f", a, F(sp.axis[a]->source), F(sp.axis[a]->dist), F(sp.motor[a]->current_pos), F(factor));
			}
			move_axes(&sp, current_time, factor);
			//debug("f %f", F(factor));
		}
		//debug("f2 %f %ld %ld", F(factor), F(last_time), F(current_time));
		do_steps(factor, current_time);
		//debug("f3 %f", F(factor));
		// Start time may have changed; recalculate t.
		t = (current_time - start_time) / 1e6;
		if (t / (t0 + tp) >= done_factor) {
			//buffered_debug("b");
			moving = false;
			uint8_t had_cbs = cbs_after_current_move;
			cbs_after_current_move = 0;
			had_cbs += next_move();
			if (moving) {
				//buffered_debug("c");
				//debug("movecb 1");
				if (had_cbs > 0)
					send_host(CMD_MOVECB, had_cbs);
				buffer_refill();
				return;
			}
			//buffered_debug("d");
			cbs_after_current_move += had_cbs;
			if (factor == 1) {
				//buffered_debug("e");
				moving = false;
				for (uint8_t s = 0; s < num_spaces; ++s) {
					Space &sp = spaces[s];
					if (!sp.active)
						continue;
					for (uint8_t m = 0; m < sp.num_motors; ++m)
						sp.motor[m]->last_v = 0;
				}
				//debug("movecb 1");
				if (cbs_after_current_move > 0) {
					send_host(CMD_MOVECB, cbs_after_current_move);
					cbs_after_current_move = 0;
				}
			}
			else {
				moving = true;
				//if (factor > 0)
				//	debug("not done %f", F(factor));
			}
		}
		return;
	}
	if (t < t0) {	// Main part.
		float t_fraction = t / t0;
		float current_f = (f1 * (2 - t_fraction) + f2 * t_fraction) * t_fraction;
		movedebug("main t %f t0 %f tp %f tfrac %f f1 %f f2 %f cf %f", F(t), F(t0), F(tp), F(t_fraction), F(f1), F(f2), F(current_f));
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			//movedebug("try %d %d", s, sp.active);
			if (!sp.active)
				continue;
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if (isnan(sp.axis[a]->dist) || sp.axis[a]->dist == 0) {
					sp.axis[a]->target = NAN;
					continue;
				}
				sp.axis[a]->target = sp.axis[a]->source + sp.axis[a]->dist * current_f;
				//movedebug("do %d %d %f %f", s, a, F(sp.axis[a]->dist), F(target[a]));
			}
			move_axes(&sp, current_time, factor);
		}
	}
	else {	// Connector part.
		movedebug("connector %f %f %f", F(t), F(t0), F(tp));
		float tc = t - t0;
		for (uint8_t s = 0; s < num_spaces; ++s) {
			Space &sp = spaces[s];
			if (!sp.active)
				continue;
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if ((isnan(sp.axis[a]->dist) || sp.axis[a]->dist == 0) && (isnan(sp.axis[a]->next_dist) || sp.axis[a]->next_dist == 0)) {
					sp.axis[a]->target = NAN;
					continue;
				}
				float t_fraction = tc / tp;
				float current_f2 = fp * (2 - t_fraction) * t_fraction;
				float current_f3 = fq * t_fraction * t_fraction;
				sp.axis[a]->target = sp.axis[a]->source + sp.axis[a]->main_dist + sp.axis[a]->dist * current_f2 + sp.axis[a]->next_dist * current_f3;
			}
			move_axes(&sp, current_time, factor);
		}
	}
	do_steps(factor, current_time);
} // }}}

void reset_dirs(int fragment) {
	//debug("resetting %d", fragment);
	num_active_motors[fragment] = 0;
	for (uint8_t s = 0; s < num_spaces; ++s) {
		Space &sp = spaces[s];
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			if (moving && sp.active && mtr.current_pos != mtr.hwcurrent_pos) {
				//debug("preactive %d %d %d %d %d", s, m, fragment, mtr.current_pos, mtr.hwcurrent_pos);
				num_active_motors[fragment] += 1;
				mtr.dir[fragment] = mtr.current_pos < mtr.hwcurrent_pos ? -1 : 1;
			}
			else {
				//debug("inactive %d %d %d", s, m, fragment);
				mtr.dir[fragment] = 0;
			}
		}
	}
}

static void send_fragment() {
	//debug("sending %d", current_fragment);
	fragment_len[current_fragment] = hwtime - hwstart_time;
	free_fragments -= 1;
	arch_send_fragment(current_fragment);
	if (stopping)
		return;
	// Prepare new fragment.
	hwstart_time = hwtime;
	current_fragment = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
	current_fragment_pos = 0;
	reset_dirs(current_fragment);
	arch_clear_fragment();
}

void buffer_refill() {
	if (refilling || stopping)
		return;
	refilling = true;
	while (moving && free_fragments > 0) {
		// fill fragment until full or dirchange.
		for (uint8_t s = 0; current_fragment_pos > 0 && s < num_spaces && !stopping; ++s) {
			Space &sp = spaces[s];
			if (!sp.active)
				continue;
			for (uint8_t m = 0; m < sp.num_motors && !stopping; ++m) {
				Motor &mtr = *sp.motor[m];
				if (mtr.current_pos == mtr.hwcurrent_pos)
					continue;
				if (mtr.dir[current_fragment] == 0) {
					debug("active %d %d %d %d", s, m, F(mtr.current_pos), F(mtr.hwcurrent_pos));
					mtr.dir[current_fragment] = (mtr.current_pos < mtr.hwcurrent_pos ? -1 : 1);
					num_active_motors[current_fragment] += 1;
					continue;
				}
				if ((mtr.dir[current_fragment] < 0 && mtr.current_pos > mtr.hwcurrent_pos) || (mtr.dir[current_fragment] > 0 && mtr.current_pos < mtr.hwcurrent_pos)) {
					//debug("dir change %d %d", s, m);
					send_fragment();
					if (free_fragments <= 0 && !stopping) {
						refilling = false;
						arch_start_move();
						return;
					}
					break;
				}
				//debug("no change %d %d %d %d", s, m, mtr.dir[current_fragment], current_fragment);
			}
		}
		if (stopping) {
			refilling = false;
			return;
		}
		// All directions are correct, or the fragment is sent and the new fragment is empty (and therefore all directions are correct).
		int mi = 0;
		for (uint8_t s = 0; (s == 0 || current_fragment_pos > 0) && s < num_spaces; mi += spaces[s].num_motors, ++s) {
			Space &sp = spaces[s];
			if (!sp.active)
				continue;
			for (uint8_t m = 0; m < sp.num_motors; ++m) {
				Motor &mtr = *sp.motor[m];
				int value = (mtr.current_pos - mtr.hwcurrent_pos) * mtr.dir[current_fragment];
				if (value > 15)
					value = 15;
				arch_set_value(mi + m, value);
				mtr.hwcurrent_pos += mtr.dir[current_fragment] * value;
			}
		}
		current_fragment_pos += 1;
		hwtime += hwtime_step;
		handle_motors(hwtime);
		if ((!moving && current_fragment_pos > 0) || current_fragment_pos >= BYTES_PER_FRAGMENT * 2) {
			//debug("fragment full %d %d %d", moving, current_fragment_pos, BYTES_PER_FRAGMENT * 2);
			send_fragment();
		}
	}
	refilling = false;
	if (stopping)
		return;
	//debug("free %d/%d", free_fragments, FRAGMENTS_PER_BUFFER);
	if (free_fragments < FRAGMENTS_PER_BUFFER)
		arch_start_move();
}
// }}}

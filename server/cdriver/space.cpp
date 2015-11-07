// vim: foldmethod=marker :
#include "cdriver.h"

#if 0
#define loaddebug debug
#else
#define loaddebug(...) do {} while(0)
#endif

#if 0
#define movedebug(...) debug(__VA_ARGS__)
#else
#define movedebug(...) do {} while (0)
#endif

// Setup. {{{
bool Space::setup_nums(uint8_t na, uint8_t nm) { // {{{
	if (na == num_axes && nm == num_motors)
		return true;
	loaddebug("new space %d %d %d %d", na, nm, num_axes, num_motors);
	uint8_t old_nm = num_motors;
	uint8_t old_na = num_axes;
	num_motors = nm;
	num_axes = na;
	if (na != old_na) {
		Axis **new_axes = new Axis *[na];
		loaddebug("new axes: %d", na);
		for (uint8_t a = 0; a < min(old_na, na); ++a)
			new_axes[a] = axis[a];
		for (uint8_t a = old_na; a < na; ++a) {
			new_axes[a] = new Axis;
			new_axes[a]->park = NAN;
			new_axes[a]->park_order = 0;
			new_axes[a]->min_pos = -INFINITY;
			new_axes[a]->max_pos = INFINITY;
			new_axes[a]->type_data = NULL;
			new_axes[a]->history = new Axis_History[FRAGMENTS_PER_BUFFER];
			new_axes[a]->settings.dist[0] = NAN;
			new_axes[a]->settings.dist[1] = NAN;
			new_axes[a]->settings.main_dist = NAN;
			new_axes[a]->settings.target = NAN;
			new_axes[a]->settings.source = NAN;
			new_axes[a]->settings.current = NAN;
			for (int f = 0; f < FRAGMENTS_PER_BUFFER; ++f) {
				new_axes[a]->history[f].dist[0] = NAN;
				new_axes[a]->history[f].dist[1] = NAN;
				new_axes[a]->history[f].main_dist = NAN;
				new_axes[a]->history[f].target = NAN;
				new_axes[a]->history[f].source = NAN;
				new_axes[a]->history[f].current = NAN;
			}
		}
		for (uint8_t a = na; a < old_na; ++a) {
			space_types[type].afree(this, a);
			delete[] axis[a]->history;
			delete axis[a];
		}
		delete[] axis;
		axis = new_axes;
	}
	if (nm != old_nm) {
		Motor **new_motors = new Motor *[nm];
		loaddebug("new motors: %d", nm);
		for (uint8_t m = 0; m < min(old_nm, nm); ++m)
			new_motors[m] = motor[m];
		for (uint8_t m = old_nm; m < nm; ++m) {
			new_motors[m] = new Motor;
			new_motors[m]->step_pin.init();
			new_motors[m]->dir_pin.init();
			new_motors[m]->enable_pin.init();
			new_motors[m]->limit_min_pin.init();
			new_motors[m]->limit_max_pin.init();
			new_motors[m]->sense_pin.init();
			new_motors[m]->sense_state = 0;
			new_motors[m]->sense_pos = NAN;
			new_motors[m]->steps_per_unit = NAN;
			new_motors[m]->max_steps = 1;
			new_motors[m]->limit_v = INFINITY;
			new_motors[m]->limit_a = INFINITY;
			new_motors[m]->home_pos = NAN;
			new_motors[m]->home_order = 0;
			new_motors[m]->limit_v = INFINITY;
			new_motors[m]->limit_a = INFINITY;
			new_motors[m]->active = false;
			new_motors[m]->history = new Motor_History[FRAGMENTS_PER_BUFFER];
			new_motors[m]->settings.last_v = 0;
			new_motors[m]->settings.current_pos = 0;
			new_motors[m]->settings.last_v = NAN;
			new_motors[m]->settings.target_v = NAN;
			new_motors[m]->settings.target_dist = NAN;
			new_motors[m]->settings.endpos = NAN;
			for (int f = 0; f < FRAGMENTS_PER_BUFFER; ++f) {
				new_motors[m]->history[f].last_v = 0;
				new_motors[m]->history[f].current_pos = 0;
				new_motors[m]->history[f].last_v = NAN;
				new_motors[m]->history[f].target_v = NAN;
				new_motors[m]->history[f].target_dist = NAN;
				new_motors[m]->history[f].endpos = NAN;
			}
			ARCH_NEW_MOTOR(id, m, new_motors);
		}
		for (uint8_t m = nm; m < old_nm; ++m) {
			DATA_DELETE(id, m);
			delete[] motor[m]->history;
			delete motor[m];
		}
		delete[] motor;
		motor = new_motors;
		arch_motors_change();
	}
	return true;
} // }}}

void move_to_current() { // {{{
	if (computing_move || !motors_busy) {
		if (moving_to_current == 0)
			moving_to_current = 1;
		return;
	}
	moving_to_current = 0;
	//debug("move to current");
	settings.f0 = 0;
	settings.fmain = 1;
	settings.fp = 0;
	settings.fq = 0;
	settings.t0 = 0;
	settings.tp = 0;
	cbs_after_current_move = 0;
	current_fragment_pos = 0;
	first_fragment = current_fragment;
	computing_move = true;
	settings.cbs = 0;
	settings.hwtime = 0;
	settings.start_time = 0;
	settings.last_time = 0;
	settings.last_current_time = 0;
	for (uint8_t s = 0; s < 2; ++s) {
		Space &sp = spaces[s];
		sp.settings.dist[0] = 0;
		sp.settings.dist[1] = 0;
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			cpdebug(s, a, "using current %f", sp.axis[a]->settings.current);
			sp.axis[a]->settings.source = sp.axis[a]->settings.current;
			sp.axis[a]->settings.target = sp.axis[a]->settings.current;
			sp.axis[a]->settings.dist[0] = 0;
			sp.axis[a]->settings.dist[1] = 0;
			sp.axis[a]->settings.main_dist = 0;
		}
		for (uint8_t m = 0; m < sp.num_motors; ++m)
			sp.motor[m]->settings.last_v = 0;
	}
#ifdef DEBUG_PATH
	fprintf(stderr, "\n");
#endif
	buffer_refill();
} // }}}

void Space::load_info(int32_t &addr) { // {{{
	loaddebug("loading space %d", id);
	uint8_t t = type;
	if (t >= NUM_SPACE_TYPES)
		t = DEFAULT_TYPE;
	type = read_8(addr);
	if (type >= NUM_SPACE_TYPES || (id == 1 && type != EXTRUDER_TYPE)) {
		debug("request for type %d ignored", type);
		type = t;
	}
	bool ok;
	if (t != type) {
		loaddebug("setting type to %d", type);
		space_types[t].free(this);
		if (!space_types[type].init(this)) {
			type = DEFAULT_TYPE;
			space_types[type].reset_pos(this);
			for (uint8_t a = 0; a < num_axes; ++a)
				axis[a]->settings.current = axis[a]->settings.source;
			return;	// The rest of the package is not meant for DEFAULT_TYPE, so ignore it.
		}
		ok = false;
	}
	else {
		if (computing_move || !motors_busy)
			ok = false;
		else
			ok = true;
	}
	space_types[type].load(this, t, addr);
	if (t != type) {
		space_types[type].reset_pos(this);
		loaddebug("resetting current for load space %d", id);
		for (uint8_t a = 0; a < num_axes; ++a)
			axis[a]->settings.current = axis[a]->settings.source;
	}
	if (ok)
		move_to_current();
	loaddebug("done loading space");
} // }}}

void Space::load_axis(uint8_t a, int32_t &addr) { // {{{
	loaddebug("loading axis %d", a);
	axis[a]->park = read_float(addr);
	axis[a]->park_order = read_8(addr);
	axis[a]->min_pos = read_float(addr);
	axis[a]->max_pos = read_float(addr);
} // }}}

void Space::load_motor(uint8_t m, int32_t &addr) { // {{{
	loaddebug("loading motor %d", m);
	uint16_t enable = motor[m]->enable_pin.write();
	double old_home_pos = motor[m]->home_pos;
	double old_steps_per_unit = motor[m]->steps_per_unit;
	motor[m]->step_pin.read(read_16(addr));
	motor[m]->dir_pin.read(read_16(addr));
	motor[m]->enable_pin.read(read_16(addr));
	motor[m]->limit_min_pin.read(read_16(addr));
	motor[m]->limit_max_pin.read(read_16(addr));
	motor[m]->sense_pin.read(read_16(addr));
	motor[m]->steps_per_unit = read_float(addr);
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
		if (motors_busy && (old_home_pos != motor[m]->home_pos || old_steps_per_unit != motor[m]->steps_per_unit) && !isnan(old_home_pos)) {
			int32_t hp = motor[m]->home_pos * motor[m]->steps_per_unit + (motor[m]->home_pos > 0 ? .49 : -.49);
			int32_t ohp = old_home_pos * old_steps_per_unit + (old_home_pos > 0 ? .49 : -.49);
			int32_t diff = hp - ohp;
			motor[m]->settings.current_pos += diff;
			cpdebug(id, m, "load motor new home add %d", diff);
			arch_addpos(id, m, diff);
			must_move = true;
		}
	}
	else {
		// Axes without a limit switch, including extruders.
		if (motors_busy && old_steps_per_unit != motor[m]->steps_per_unit) {
			int32_t cp = motor[m]->settings.current_pos;
			double pos = cp / old_steps_per_unit;
			cpdebug(id, m, "load motor new steps no home");
			motor[m]->settings.current_pos = pos * motor[m]->steps_per_unit;
			int diff = motor[m]->settings.current_pos - cp;
			arch_addpos(id, m, diff);
		}
	}
	if (must_move)
		move_to_current();
} // }}}

void Space::save_info(int32_t &addr) { // {{{
	write_8(addr, type);
	space_types[type].save(this, addr);
} // }}}

void Space::save_axis(uint8_t a, int32_t &addr) { // {{{
	write_float(addr, axis[a]->park);
	write_8(addr, axis[a]->park_order);
	write_float(addr, axis[a]->min_pos);
	write_float(addr, axis[a]->max_pos);
} // }}}

void Space::save_motor(uint8_t m, int32_t &addr) { // {{{
	write_16(addr, motor[m]->step_pin.write());
	write_16(addr, motor[m]->dir_pin.write());
	write_16(addr, motor[m]->enable_pin.write());
	write_16(addr, motor[m]->limit_min_pin.write());
	write_16(addr, motor[m]->limit_max_pin.write());
	write_16(addr, motor[m]->sense_pin.write());
	write_float(addr, motor[m]->steps_per_unit);
	write_8(addr, motor[m]->max_steps);
	write_float(addr, motor[m]->home_pos);
	write_float(addr, motor[m]->limit_v);
	write_float(addr, motor[m]->limit_a);
	write_8(addr, motor[m]->home_order);
} // }}}

void Space::init(uint8_t space_id) { // {{{
	type = space_id == 0 ? DEFAULT_TYPE : EXTRUDER_TYPE;
	id = space_id;
	type_data = NULL;
	num_axes = 0;
	num_motors = 0;
	motor = NULL;
	axis = NULL;
	history = new Space_History[FRAGMENTS_PER_BUFFER];
	space_types[type].init(this);
} // }}}

void Space::cancel_update() { // {{{
	// setup_nums failed; restore system to a usable state.
	type = DEFAULT_TYPE;
	uint8_t n = min(num_axes, num_motors);
	if (!setup_nums(n, n)) {
		debug("Failed to free memory; removing all motors and axes to make sure it works");
		if (!setup_nums(0, 0))
			debug("You're in trouble; this shouldn't be possible");
	}
} // }}}
// }}}

// Movement handling. {{{
static void check_distance(Motor *mtr, double distance, double dt, double &factor) { // {{{
	if (dt == 0) {
		factor = 0;
		return;
	}
	if (isnan(distance) || distance == 0) {
		mtr->settings.target_dist = 0;
		//mtr->last_v = 0;
		return;
	}
	//debug("cd %f %f", distance, dt);
	mtr->settings.target_dist = distance;
	mtr->settings.target_v = distance / dt;
	double v = fabs(mtr->settings.target_v);
	int8_t s = (mtr->settings.target_v < 0 ? -1 : 1);
	// When turning around, ignore limits (they shouldn't have been violated anyway).
	if (mtr->settings.last_v * s < 0) {
		movedebug("!");
		mtr->settings.last_v = 0;
	}
	// Limit v.
	if (v > mtr->limit_v) {
		movedebug("v %f %f", v, mtr->limit_v);
		distance = (s * mtr->limit_v) * dt;
		v = fabs(distance / dt);
	}
	//debug("cd2 %f %f", distance, dt);
	// Limit a+.
	double limit_dv = mtr->limit_a * dt;
	if (v - mtr->settings.last_v * s > limit_dv) {
		movedebug("a+ %f %f %f %d", mtr->settings.target_v, limit_dv, mtr->settings.last_v, s);
		distance = (limit_dv * s + mtr->settings.last_v) * dt;
		v = fabs(distance / dt);
	}
	//debug("cd3 %f %f", distance, dt);
	// Limit a-.
	// Distance to travel until end of segment or connection.
	double max_dist = (mtr->settings.endpos - mtr->settings.current_pos / mtr->steps_per_unit) * s;
	// Find distance traveled when slowing down at maximum a.
	// x = 1/2 at²
	// t = sqrt(2x/a)
	// v = at
	// v² = 2a²x/a = 2ax
	// x = v²/2a
	double limit_dist = v * v / 2 / mtr->limit_a;
	//debug("max %f limit %f v %f a %f", max_dist, limit_dist, v, mtr->limit_a);
	/*if (max_dist > 0 && limit_dist > max_dist) {
		movedebug("a- %f %f %f %d %d %f", mtr->settings.endpos, mtr->limit_a, max_dist, mtr->settings.current_pos, s, dt);
		v = sqrt(max_dist * 2 * mtr->limit_a);
		distance = s * v * dt;
	}*/
	//debug("cd4 %f %f", distance, dt); */
	int steps = distance * mtr->steps_per_unit;
	//cpdebug(s, m, "cp %d hwcp %d cf %d value %d", mtr.settings.current_pos, mtr.settings.current_pos, current_fragment, value);
	if (settings.probing && steps)
		steps = s;
	else {
		if (abs(steps) > 8 * mtr->max_steps)	// TODO: fix 8.
			steps = s * 8 * mtr->max_steps;
		if (abs(steps) > 0x7f) {
			debug("overflow %d", steps);
			steps = 0x7f * s;
		}
	}
	if (steps != int(distance * mtr->steps_per_unit)) {
		distance = steps / mtr->steps_per_unit;
		v = fabs(distance / dt);
	}
	//debug("move pos %d %d cf %d time %d cp %d hwcp %d diff %d value %d", s, m, current_fragment, settings.hwtime, sp.motor[m]->settings.current_pos + avr_pos_offset[m], sp.motor[m]->settings.current_pos + avr_pos_offset[m], sp.motor[m]->settings.current_pos - sp.motor[m]->settings.current_pos, value);
	//debug("=============");
	double f = distance / mtr->settings.target_dist;
	movedebug("checked %f %f", mtr->settings.target_dist, distance);
	if (f < factor)
		factor = f;
} // }}}

static void move_axes(Space *s, uint32_t current_time, double &factor) { // {{{
#ifdef DEBUG_PATH
	fprintf(stderr, "%d\t%d", current_time, s->id);
	for (int a = 0; a < s->num_axes; ++a) {
		if (isnan(s->axis[a]->settings.target))
			fprintf(stderr, "\t%f", s->axis[a]->settings.source);
		else
			fprintf(stderr, "\t%f", s->axis[a]->settings.target);
	}
	fprintf(stderr, "\n");
#endif
	double motors_target[s->num_motors];
	bool ok = true;
	space_types[s->type].xyz2motors(s, motors_target, &ok);
	// Try again if it didn't work; it should have moved target to a better location.
	if (!ok) {
		space_types[s->type].xyz2motors(s, motors_target, &ok);
		movedebug("retried move");
	}
	//movedebug("ok %d", ok);
	for (uint8_t m = 0; m < s->num_motors; ++m) {
		movedebug("move %d %f %f", m, motors_target[m], s->motor[m]->settings.current_pos / s->motor[m]->steps_per_unit);
		double distance = motors_target[m] - s->motor[m]->settings.current_pos / s->motor[m]->steps_per_unit;
		check_distance(s->motor[m], distance, (current_time - settings.last_time) / 1e6, factor);
	}
} // }}}

static bool do_steps(double &factor, uint32_t current_time) { // {{{
	//debug("steps");
	if (factor <= 0) {
		movedebug("end move");
		return false;
	}
	for (uint8_t s = 0; s < 2; ++s) {
		Space &sp = spaces[s];
		for (uint8_t a = 0; a < sp.num_axes; ++a) {
			if (!isnan(sp.axis[a]->settings.target))
				sp.axis[a]->settings.current += (sp.axis[a]->settings.target - sp.axis[a]->settings.current) * factor;
		}
	}
	if (factor < 1) {
		// Recalculate steps; ignore resulting factor.
		double dummy_factor = 1;
		for (uint8_t s = 0; s < 2; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if (!isnan(sp.axis[a]->settings.target))
					sp.axis[a]->settings.target = sp.axis[a]->settings.current;
			}
			move_axes(&sp, settings.last_time, dummy_factor);
		}
	}
	//debug("do steps %f", factor);
	uint32_t the_last_time = settings.last_current_time;
	settings.last_current_time = current_time;
	// Adjust start time if factor < 1.
	bool have_steps = false;
	if (factor < 1) {
		for (uint8_t s = 0; !have_steps && s < 2; ++s) {
			Space &sp = spaces[s];
			for (uint8_t m = 0; m < sp.num_motors; ++m) {
				Motor &mtr = *sp.motor[m];
				double num = (mtr.settings.current_pos / mtr.steps_per_unit + mtr.settings.target_dist * factor) * mtr.steps_per_unit;
				if (mtr.settings.current_pos != int(num + (num > 0 ? .49 : -.49))) {
					//debug("have steps %d %f %f", mtr.settings.current_pos, mtr.settings.target_dist, factor);
					have_steps = true;
					break;
				}
				//debug("no steps yet %d %d", s, m);
			}
		}
		// If there are no steps to take, wait until there are.
		//static int streak = 0;
		if (!have_steps) {
			//movedebug("no steps");
		//	if (streak++ > 50)
		//		abort();
			return false;
		}
		//streak = 0;
		settings.start_time += (current_time - the_last_time) * ((1 - factor) * .99);
		movedebug("correct: %f %d", factor, int(settings.start_time));
	}
	else
		movedebug("no correct: %f %d", factor, int(settings.start_time));
	settings.last_time = current_time;
	// Move the motors.
	//debug("start move");
	for (uint8_t s = 0; s < 2; ++s) {
		Space &sp = spaces[s];
		for (uint8_t m = 0; m < sp.num_motors; ++m) {
			Motor &mtr = *sp.motor[m];
			if (isnan(mtr.settings.target_dist) || mtr.settings.target_dist == 0) {
				//if (mtr.target_v == 0)
					//mtr.last_v = 0;
				continue;
			}
			double target = mtr.settings.current_pos / mtr.steps_per_unit + mtr.settings.target_dist * factor;
			cpdebug(s, m, "ccp3 stopping %d target %f lastv %f spm %f tdist %f factor %f frag %d", stopping, target, mtr.settings.last_v, mtr.steps_per_unit, mtr.settings.target_dist, factor, current_fragment);
			//if (fabs(mtr.settings.target_dist * factor) > .01)	// XXX: This shouldn't ever happen on my printer, but shouldn't be a limitation.
				//abort();
			int new_cp = target * mtr.steps_per_unit + (target > 0 ? .49 : -.49);
			if (mtr.settings.current_pos != new_cp) {
				have_steps = true;
				if (!mtr.active) {
					mtr.active = true;
					num_active_motors += 1;
				}
				DATA_SET(s, m, new_cp - mtr.settings.current_pos);
			}
			mtr.settings.current_pos = new_cp;
			//cpdebug(s, m, "cp three %f", target);
			mtr.settings.last_v = mtr.settings.target_v * factor;
		}
	}
	current_fragment_pos += 1;
	return have_steps;
} // }}}

void make_target(Space &sp, double f, bool next) { // {{{
	int a = 0;
	if (sp.settings.arc[next]) {
		// Convert time-fraction into angle-fraction.
		f = (2 * sp.settings.radius[next][0] * f) / (sp.settings.radius[next][0] + sp.settings.radius[next][1] - (sp.settings.radius[next][1] - sp.settings.radius[next][0]) * f);
		double angle = sp.settings.angle[next] * f;
		double radius = sp.settings.radius[next][0] + (sp.settings.radius[next][1] - sp.settings.radius[next][0]) * f;
		double helix = sp.settings.helix[next] * f;
		double cosa = cos(angle);
		double sina = sin(angle);
		//debug("e1 %f %f %f e2 %f %f %f", sp.settings.e1[next][0], sp.settings.e1[next][1], sp.settings.e1[next][2], sp.settings.e2[next][0], sp.settings.e2[next][1], sp.settings.e2[next][2]);
		for (int i = 0; i < min(3, sp.num_axes); ++i)
			sp.axis[i]->settings.target += radius * cosa * sp.settings.e1[next][i] + radius * sina * sp.settings.e2[next][i] - sp.settings.offset[next][i] + helix * sp.settings.normal[next][i];
		a = 3;
		// Fall through.
	}
	for (; a < sp.num_axes; ++a) {
		sp.axis[a]->settings.target += sp.axis[a]->settings.dist[next] * f;
		movedebug("do %d %d %f %f", sp.id, a, sp.axis[a]->settings.dist[0], sp.axis[a]->settings.target);
	}
} // }}}

static void handle_motors(unsigned long long current_time) { // {{{
	// Check for move.
	if (!computing_move) {
		movedebug("handle motors not moving");
		return;
	}
	movedebug("handling %d", computing_move);
	double factor = 1;
	double t = (current_time - settings.start_time) / 1e6;
	if (t >= settings.t0 + settings.tp) {	// Finish this move and prepare next. {{{
		movedebug("finishing %f %f %f %ld %ld", t, settings.t0, settings.tp, long(current_time), long(settings.start_time));
		for (uint8_t s = 0; s < 2; ++s) {
			Space &sp = spaces[s];
			bool new_move = false;
			if (!isnan(sp.settings.dist[0])) {
				for (uint8_t a = 0; a < sp.num_axes; ++a) {
					if (!isnan(sp.axis[a]->settings.dist[0])) {
						sp.axis[a]->settings.source += sp.axis[a]->settings.dist[0];
						sp.axis[a]->settings.dist[0] = NAN;
						//debug("new source %d %f", a, sp.axis[a]->settings.source);
					}
				}
				sp.settings.dist[0] = NAN;
			}
			for (uint8_t a = 0; a < sp.num_axes; ++a)
				sp.axis[a]->settings.target = sp.axis[a]->settings.source;
			move_axes(&sp, current_time, factor);
			//debug("f %f", factor);
		}
		//debug("f2 %f %ld %ld", factor, settings.last_time, current_time);
		bool did_steps = do_steps(factor, current_time);
		//debug("f3 %f", factor);
		// Start time may have changed; recalculate t.
		t = (current_time - settings.start_time) / 1e6;
		if (t / (settings.t0 + settings.tp) >= done_factor) {
			uint8_t had_cbs = cbs_after_current_move;
			cbs_after_current_move = 0;
			run_file_fill_queue();
			if (settings.queue_start != settings.queue_end || settings.queue_full) {
				had_cbs += next_move();
				if (!aborting && had_cbs > 0) {
					//debug("adding %d cbs to fragment %d", had_cbs, current_fragment);
					history[current_fragment].cbs += had_cbs;
				}
				return;
			}
			cbs_after_current_move += had_cbs;
			if (factor == 1) {
				//debug("queue done");
				if (!did_steps) {
					//debug("done move");
					computing_move = false;
				}
				for (uint8_t s = 0; s < 2; ++s) {
					Space &sp = spaces[s];
					for (uint8_t m = 0; m < sp.num_motors; ++m)
						sp.motor[m]->settings.last_v = 0;
				}
				if (cbs_after_current_move > 0) {
					if (!aborting) {
						//debug("adding %d cbs to final fragment %d", cbs_after_current_move, current_fragment);
						history[current_fragment].cbs += cbs_after_current_move;
					}
					cbs_after_current_move = 0;
				}
			}
		}
		return;
	} // }}}
	if (t < settings.t0) {	// Main part. {{{
		double t_fraction = t / settings.t0;
		double current_f = (settings.f1 * (2 - t_fraction) + settings.f2 * t_fraction) * t_fraction;
		movedebug("main t %f t0 %f tp %f tfrac %f f1 %f f2 %f cf %f", t, settings.t0, settings.tp, t_fraction, settings.f1, settings.f2, current_f);
		for (int s = 0; s < 2; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if (isnan(sp.axis[a]->settings.dist[0])) {
					sp.axis[a]->settings.target = NAN;
					continue;
				}
				sp.axis[a]->settings.target = sp.axis[a]->settings.source;
			}
			make_target(sp, current_f, false);
			move_axes(&sp, current_time, factor);
		}
	} // }}}
	else {	// Connector part. {{{
		movedebug("connector %f %f %f", t, settings.t0, settings.tp);
		double tc = t - settings.t0;
		double t_fraction = tc / settings.tp;
		double current_f2 = settings.fp * (2 - t_fraction) * t_fraction;
		double current_f3 = settings.fq * t_fraction * t_fraction;
		for (uint8_t s = 0; s < 2; ++s) {
			Space &sp = spaces[s];
			for (uint8_t a = 0; a < sp.num_axes; ++a) {
				if (isnan(sp.axis[a]->settings.dist[0]) && isnan(sp.axis[a]->settings.dist[1])) {
					sp.axis[a]->settings.target = NAN;
					continue;
				}
				sp.axis[a]->settings.target = sp.axis[a]->settings.source;
			}
			make_target(sp, (1 - settings.fp) + current_f2, false);
			make_target(sp, current_f3, true);
			move_axes(&sp, current_time, factor);
		}
	} // }}}
	do_steps(factor, current_time);
} // }}}

void store_settings() { // {{{
	current_fragment_pos = 0;
	num_active_motors = 0;
	history[current_fragment].t0 = settings.t0;
	history[current_fragment].tp = settings.tp;
	history[current_fragment].f0 = settings.f0;
	history[current_fragment].f1 = settings.f1;
	history[current_fragment].f2 = settings.f2;
	history[current_fragment].fp = settings.fp;
	history[current_fragment].fq = settings.fq;
	history[current_fragment].fmain = settings.fmain;
	history[current_fragment].cbs = 0;
	history[current_fragment].hwtime = settings.hwtime;
	history[current_fragment].start_time = settings.start_time;
	history[current_fragment].last_time = settings.last_time;
	history[current_fragment].last_current_time = settings.last_current_time;
	history[current_fragment].queue_start = settings.queue_start;
	history[current_fragment].queue_end = settings.queue_end;
	history[current_fragment].queue_full = settings.queue_full;
	history[current_fragment].run_file_current = settings.run_file_current;
	history[current_fragment].run_time = settings.run_time;
	history[current_fragment].run_dist = settings.run_dist;
	for (int s = 0; s < 2; ++s) {
		Space &sp = spaces[s];
		sp.history[current_fragment].dist[0] = sp.settings.dist[0];
		sp.history[current_fragment].dist[1] = sp.settings.dist[1];
		for (int i = 0; i < 2; ++i) {
			sp.history[current_fragment].arc[i] = sp.settings.arc[i];
			sp.history[current_fragment].angle[i] = sp.settings.angle[i];
			sp.history[current_fragment].helix[i] = sp.settings.helix[i];
			for (int t = 0; t < 2; ++t)
				sp.history[current_fragment].radius[i][t] = sp.settings.radius[i][t];
			for (int t = 0; t < 3; ++t) {
				sp.history[current_fragment].offset[i][t] = sp.settings.offset[i][t];
				sp.history[current_fragment].e1[i][t] = sp.settings.e1[i][t];
				sp.history[current_fragment].e2[i][t] = sp.settings.e2[i][t];
				sp.history[current_fragment].normal[i][t] = sp.settings.normal[i][t];
			}
		}
		for (int m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->active = false;
			DATA_CLEAR(s, m);
			sp.motor[m]->history[current_fragment].last_v = sp.motor[m]->settings.last_v;
			sp.motor[m]->history[current_fragment].target_v = sp.motor[m]->settings.target_v;
			sp.motor[m]->history[current_fragment].target_dist = sp.motor[m]->settings.target_dist;
			sp.motor[m]->history[current_fragment].current_pos = sp.motor[m]->settings.current_pos;
			sp.motor[m]->history[current_fragment].endpos = sp.motor[m]->settings.endpos;
			//fcpdebug(s, m, "store");
		}
		for (int a = 0; a < sp.num_axes; ++a) {
			sp.axis[a]->history[current_fragment].dist[0] = sp.axis[a]->settings.dist[0];
			sp.axis[a]->history[current_fragment].dist[1] = sp.axis[a]->settings.dist[1];
			sp.axis[a]->history[current_fragment].main_dist = sp.axis[a]->settings.main_dist;
			sp.axis[a]->history[current_fragment].target = sp.axis[a]->settings.target;
			sp.axis[a]->history[current_fragment].source = sp.axis[a]->settings.source;
			sp.axis[a]->history[current_fragment].current = sp.axis[a]->settings.current;
			sp.axis[a]->history[current_fragment].endpos[0] = sp.axis[a]->settings.endpos[0];
			sp.axis[a]->history[current_fragment].endpos[1] = sp.axis[a]->settings.endpos[1];
		}
	}
} // }}}

void restore_settings() { // {{{
	current_fragment_pos = 0;
	num_active_motors = 0;
	settings.t0 = history[current_fragment].t0;
	settings.tp = history[current_fragment].tp;
	settings.f0 = history[current_fragment].f0;
	settings.f1 = history[current_fragment].f1;
	settings.f2 = history[current_fragment].f2;
	settings.fp = history[current_fragment].fp;
	settings.fq = history[current_fragment].fq;
	settings.fmain = history[current_fragment].fmain;
	history[current_fragment].cbs = 0;
	settings.hwtime = history[current_fragment].hwtime;
	settings.start_time = history[current_fragment].start_time;
	settings.last_time = history[current_fragment].last_time;
	settings.last_current_time = history[current_fragment].last_current_time;
	settings.queue_start = history[current_fragment].queue_start;
	settings.queue_end = history[current_fragment].queue_end;
	settings.queue_full = history[current_fragment].queue_full;
	settings.run_file_current = history[current_fragment].run_file_current;
	settings.run_time = history[current_fragment].run_time;
	settings.run_dist = history[current_fragment].run_dist;
	for (int s = 0; s < 2; ++s) {
		Space &sp = spaces[s];
		sp.settings.dist[0] = sp.history[current_fragment].dist[0];
		sp.settings.dist[1] = sp.history[current_fragment].dist[1];
		for (int i = 0; i < 2; ++i) {
			sp.settings.arc[i] = sp.history[current_fragment].arc[i];
			sp.settings.angle[i] = sp.history[current_fragment].angle[i];
			sp.settings.helix[i] = sp.history[current_fragment].helix[i];
			for (int t = 0; t < 2; ++t)
				sp.settings.radius[i][t] = sp.history[current_fragment].radius[i][t];
			for (int t = 0; t < 3; ++t) {
				sp.settings.offset[i][t] = sp.history[current_fragment].offset[i][t];
				sp.settings.e1[i][t] = sp.history[current_fragment].e1[i][t];
				sp.settings.e2[i][t] = sp.history[current_fragment].e2[i][t];
				sp.settings.normal[i][t] = sp.history[current_fragment].normal[i][t];
			}
		}
		for (int m = 0; m < sp.num_motors; ++m) {
			sp.motor[m]->active = false;
			DATA_CLEAR(s, m);
			sp.motor[m]->settings.last_v = sp.motor[m]->history[current_fragment].last_v;
			sp.motor[m]->settings.target_v = sp.motor[m]->history[current_fragment].target_v;
			sp.motor[m]->settings.target_dist = sp.motor[m]->history[current_fragment].target_dist;
			sp.motor[m]->settings.current_pos = sp.motor[m]->history[current_fragment].current_pos;
			sp.motor[m]->settings.endpos = sp.motor[m]->history[current_fragment].endpos;
			//fcpdebug(s, m, "restore");
		}
		for (int a = 0; a < sp.num_axes; ++a) {
			sp.axis[a]->settings.dist[0] = sp.axis[a]->history[current_fragment].dist[0];
			sp.axis[a]->settings.dist[1] = sp.axis[a]->history[current_fragment].dist[1];
			sp.axis[a]->settings.main_dist = sp.axis[a]->history[current_fragment].main_dist;
			sp.axis[a]->settings.target = sp.axis[a]->history[current_fragment].target;
			sp.axis[a]->settings.source = sp.axis[a]->history[current_fragment].source;
			sp.axis[a]->settings.current = sp.axis[a]->history[current_fragment].current;
			sp.axis[a]->settings.endpos[0] = sp.axis[a]->history[current_fragment].endpos[0];
			sp.axis[a]->settings.endpos[1] = sp.axis[a]->history[current_fragment].endpos[1];
		}
	}
} // }}}

void send_fragment() { // {{{
	if (host_block) {
		current_fragment_pos = 0;
		return;
	}
	if (current_fragment_pos <= 0 || stopping || sending_fragment) {
		debug("no send fragment %d %d %d", current_fragment_pos, stopping, sending_fragment);
		return;
	}
	if (num_active_motors == 0) {
		if (current_fragment_pos < 2) {
			// TODO: find out why this is attempted and avoid it.
			debug("not sending short fragment for 0 motors; %d %d", current_fragment, running_fragment);
			if (history[current_fragment].cbs) {
				if (settings.queue_start == settings.queue_end && !settings.queue_full) {
					// Send cbs immediately.
					arch_send_movecbs(history[current_fragment].cbs);
					history[current_fragment].cbs = 0;
				}
			}
			current_fragment_pos = 0;
			return;
		}
		else
			debug("sending fragment for 0 motors at position %d", current_fragment_pos);
		//abort();
	}
	//debug("sending %d prevcbs %d", current_fragment, settings[(current_fragment - 1) % FRAGMENTS_PER_BUFFER].cbs);
	if (arch_send_fragment()) {
		current_fragment = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
		//debug("current send -> %x", current_fragment);
		store_settings();
		if ((current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER >= MIN_BUFFER_FILL && !stopping)
			arch_start_move(0);
	}
} // }}}

void apply_tick() { // {{{
	settings.hwtime += hwtime_step;
	handle_motors(settings.hwtime);
	//if (spaces[0].num_axes >= 2)
		//debug("move z %d %d %f %d %d", current_fragment, current_fragment_pos, spaces[0].axis[2]->settings.current, spaces[0].motor[0]->settings.current_pos, spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0]);
} // }}}

void buffer_refill() { // {{{
	if (moving_to_current == 2)
		move_to_current();
	if (!computing_move || refilling || stopping || discard_pending || discarding) {
		//debug("refill block %d %d %d", moving, refilling, stopping);
		return;
	}
	refilling = true;
	// send_fragment in the previous refill may have failed; try it again.
	if (current_fragment_pos > 0)
		send_fragment();
	//debug("refill start %d %d %d", running_fragment, current_fragment, sending_fragment);
	// Keep one free fragment, because we want to be able to rewind and use the buffer before the one currently active.
	while (computing_move && !stopping && !discard_pending && !discarding && (running_fragment - 1 - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER > 4 && !sending_fragment) {
		//debug("refill %d %d %d", current_fragment, current_fragment_pos, spaces[0].motor[0]->settings.current_pos);
		// fill fragment until full.
		apply_tick();
		//debug("refill2 %d %d", current_fragment, spaces[0].motor[0]->settings.current_pos);
		if (current_fragment_pos >= SAMPLES_PER_FRAGMENT) {
			//debug("fragment full %d %d %d", computing_move, current_fragment_pos, BYTES_PER_FRAGMENT);
			send_fragment();
		}
		// Check for commands from host; in case of many short buffers, this loop may not end in a reasonable time.
		//serial(0);
	}
	if (stopping || discard_pending) {
		//debug("aborting refill for stopping");
		refilling = false;
		return;
	}
	if (!computing_move && current_fragment_pos > 0) {
		//debug("finalize");
		send_fragment();
	}
	refilling = false;
	arch_start_move(0);
} // }}}
// }}}

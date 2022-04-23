/* run.cpp - parsed gcode execution handling for Franklin
 * Copyright 2014-2016 Michigan Technological University
 * Copyright 2016 Bas Wijnen <wijnen@debian.org>
 * Author: Bas Wijnen <wijnen@debian.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cdriver.h"
#include <sys/stat.h>
#include <sys/mman.h>

#if 0
#define rundebug debug
#else
#define rundebug(...) do {} while(0)
#endif

static int read_num(off_t offset) {
	int ret = 0;
	uint8_t const *map = reinterpret_cast<uint8_t const *>(run_file_map);
	for (int i = 0; i < 4; ++i)
		ret |= (map[offset + i]) << (8 * i);
	return ret;
}

struct String {
	int len;
	off_t start;
};

static String *strings;

static int pattern_size;
static uint8_t current_pattern[PATTERN_MAX];
static double pending_abc[3], pending_abc_h[3];

bool run_file(char const *name, bool start, double sina, double cosa) {
	rundebug("run file %d %f %f", start, sina, cosa);
	abort_run_file();
	if (name[0] == '\0')
		return false;
	run_file_name = name;
	settings.run_time = 0;
	settings.run_file_current = 0;
	int fd = open(run_file_name.c_str(), O_RDONLY);
	if (fd < 0) {
		debug("Failed to open run file '%s': %s", run_file_name.c_str(), strerror(errno));
		return false;
	}
	struct stat stat;
	if (fstat(fd, &stat) < 0) {
		debug("Failed to stat run file '%s': %s", run_file_name.c_str(), strerror(errno));
		close(fd);
		return false;
	}
	run_file_size = stat.st_size;
	run_file_map = reinterpret_cast<Run_Record *>(mmap(NULL, run_file_size, PROT_READ, MAP_SHARED, fd, 0));
	close(fd);
	delayed_reply();
	// File format:
	// records
	// strings
	// int32_t stringlengths[]
	// int32_t numstrings
	// double bbox[6]
	// double time
	run_file_num_strings = read_num(run_file_size - sizeof(double) * 7 - sizeof(int32_t));
	strings = new String[run_file_num_strings];
	off_t pos = run_file_size - sizeof(double) * 7 - sizeof(int32_t) - sizeof(int32_t) * run_file_num_strings;
	off_t current = 0;
	for (int i = 0; i < run_file_num_strings; ++i) {
		strings[i].start = current;
		strings[i].len = read_num(pos + sizeof(int32_t) * i);
		current += strings[i].len;
	}
	run_file_first_string = pos - current;
	run_file_num_records = run_file_first_string / sizeof(Run_Record);
	run_file_wait = start ? 0 : 1;
	run_file_timer.it_interval.tv_sec = 0;
	run_file_timer.it_interval.tv_nsec = 0;
	pausing = false;
	parkwaiting = false;
	resume_pending = false;
	run_file_sina = sina;
	run_file_cosa = cosa;
	run_file_next_command(settings.hwtime);
	return true;
}

void abort_run_file() {
	if (!run_file_map)
		return;
	munmap(run_file_map, run_file_size);
	run_file_map = NULL;
	delete[] strings;
	strings = NULL;
}

void run_file_next_command(int32_t start_time) {
	static bool lock = false;
	if (pausing || parkwaiting || lock || resume_pending) {
		next_move(start_time);
		return;
	}
	lock = true;
	rundebug("run queue, current = %" LONGFMT "/%" LONGFMT " wait = %d q = %d %d", settings.run_file_current, run_file_num_records, run_file_wait, settings.queue_end, settings.queue_start);
	double lastpos[6] = {0, 0, 0, 0, 0, 0};
	bool moving = false;
	while (!pausing	&& !parkwaiting // We are running.
			&& run_file_map	// There is a file to run.
			&& settings.queue_end == settings.queue_start	// The queue is empty
			&& settings.run_file_current < run_file_num_records	// There are records to send.
			&& !run_file_wait) {	// We are not waiting for something else (delay, temperature or confirm).
		Run_Record &r = run_file_map[settings.run_file_current];
		int t = r.type;
		if (!(t == RUN_POLY3PLUS || t == RUN_POLY3MINUS || t == RUN_POLY2 || t == RUN_ARC || t == RUN_ABC || t == RUN_PATTERN) && (arch_running() || computing_move || settings.queue_end != settings.queue_start || moving || sending_fragment || transmitting_fragment))
			break;
		rundebug("running %" LONGFMT ": %d %d running %d moving %d", settings.run_file_current, r.type, r.tool, arch_running(), moving);
		switch (r.type) {
			case RUN_SYSTEM:
			{
				char const *cmd = strndupa(&reinterpret_cast<char const *>(run_file_map)[run_file_first_string + strings[r.tool].start], strings[r.tool].len);
				debug("Running system command: %ld %d %s", strings[r.tool].start, strings[r.tool].len, cmd);
				int ret = system(cmd);
				debug("Done running system command, return = %d", ret);
				break;
			}
			case RUN_ABC:
			{
				for (int i = 0; i < 3; ++i) {
					pending_abc[i] = r.X[i];
					pending_abc_h[i] = r.h[i];
				}
				break;
			}
			case RUN_POLY3PLUS:
			case RUN_POLY3MINUS:
			case RUN_POLY2:
			{
				settings.queue_start = 0;
				settings.queue_end = 0;
				queue[settings.queue_end].reverse = r.type == RUN_POLY3MINUS;
				queue[settings.queue_end].single = false;
				queue[settings.queue_end].probe = false;
				queue[settings.queue_end].a0 = (r.type == RUN_POLY2 ? r.Jg : 0);
				queue[settings.queue_end].v0 = r.v0;
				double x = r.X[0] * run_file_cosa - r.X[1] * run_file_sina;
				double y = r.X[1] * run_file_cosa + r.X[0] * run_file_sina;
				double z = r.X[2];
				//debug("line %d: %f %f %f", settings.run_file_current, x, y, z);
				queue[settings.queue_end].target[0] = x;
				queue[settings.queue_end].target[1] = y;
				queue[settings.queue_end].target[2] = z;
				double distg = 0, disth = 0;
				for (int i = 0; i < 3; ++i) {
					double d = queue[settings.queue_end].target[i] - lastpos[i];
					if (!std::isnan(d))
						distg += d * d;
					d = pending_abc[i] - lastpos[3 + i];
					if (!std::isnan(d))
						distg += d * d;
					if (!std::isnan(r.h[i]))
						disth += r.h[i] * r.h[i];
					if (!std::isnan(pending_abc_h[i]))
						disth += pending_abc_h[i] * pending_abc_h[i];
					queue[settings.queue_end].target[3 + i] = pending_abc[i];
					pending_abc[i] = NAN;
					pending_abc_h[i] = NAN;
				}
				distg = std::sqrt(distg);
				disth = std::sqrt(disth);
				for (int i = 0; i < 6; ++i) {
					queue[settings.queue_end].unitg[i] = distg < 1e-10 ? 0 : (queue[settings.queue_end].target[i] - lastpos[i]) / distg;
					queue[settings.queue_end].unith[i] = disth < 1e-10 ? 0 : (i < 3 ? r.h[i]: pending_abc_h[i - 3]) / disth;
				}
				queue[settings.queue_end].Jg = (r.type == RUN_POLY2 ? 0 : r.Jg);
				queue[settings.queue_end].Jh = disth;
				queue[settings.queue_end].tf = r.tf;
				queue[settings.queue_end].e = r.E;
				queue[settings.queue_end].tool = r.tool;
				queue[settings.queue_end].time = r.time;
				queue[settings.queue_end].cb = false;
				queue[settings.queue_end].pattern_size = pattern_size;
				queue[settings.queue_end].gcode_line = r.gcode_line;
				if (pattern_size > 0)
					memcpy(queue[settings.queue_end].pattern, current_pattern, pattern_size);
				pattern_size = 0;
				for (int i = 0; i < 6; ++i)
					lastpos[i] = queue[settings.queue_end].target[i];
				settings.queue_end += 1;
				moving = true;
				break;
			}
			case RUN_ARC:
			{
				// TODO.
				debug("G2/G3 are currently not supported");
				abort();
				moving = true;
				break;
			}
			case RUN_GOTO:
			{
				// Goto commands need to wait for an underrun.
				for (int i = 0; i < 3; ++i) {
					if (!std::isnan(r.X[i]))
						lastpos[i] = r.X[i];
					if (!std::isnan(pending_abc[i]))
						lastpos[3 + i] = pending_abc[i];
				}
				MoveCommand move;
				move.cb = false;
				move.probe = false;
				move.single = false;
				move.pattern_size = 0;
				move.v0 = r.v0;
				move.tool = r.tool;
				move.target[0] = r.X[0] * run_file_cosa - r.X[1] * run_file_sina;
				move.target[1] = r.X[1] * run_file_cosa + r.X[0] * run_file_sina;
				move.target[2] = r.X[2];
				for (int i = 0; i < 3; ++i) {
					move.target[3 + i] = pending_abc[i];
					pending_abc[i] = NAN;
				}
				move.e = r.E;
				move.time = r.time;
				move.gcode_line = r.gcode_line;
				rundebug("run goto %f,%f,%f tool %d E %f v %f", r.X[0], r.X[1], r.X[2], r.tool, r.E, r.v0);
				settings.queue_end = go_to(false, &move, true);
				moving = true;
				break;
			}
			case RUN_PATTERN:
			{
				pattern_size = max(0, min(r.tool, PATTERN_MAX));
				memcpy(current_pattern, &r.X[0], pattern_size);
				break;
			}
			case RUN_GPIO:
			{
				int tool = r.tool;
				if (tool == -2)
					tool = fan_id != 255 ? fan_id : -1;
				else if (tool == -3)
					tool = spindle_id != 255 ? spindle_id : -1;
				if (tool < 0 || tool >= num_gpios) {
					if (tool != -1)
						debug("cannot set invalid gpio %d", tool);
					break;
				}
				if (r.X[0]) {
					gpios[tool].state = 1;
					gpios[tool].duty = r.X[0];
					arch_set_duty(gpios[tool].pin, r.X[0]);
					SET(gpios[tool].pin);
				}
				else {
					gpios[tool].state = 0;
					RESET(gpios[tool].pin);
				}
				prepare_interrupt();
				shmem->interrupt_ints[0] = tool;
				shmem->interrupt_ints[1] = gpios[tool].state;
				send_to_parent(CMD_UPDATE_PIN);
				break;
			}
			case RUN_SETTEMP:
			{
				int tool = r.tool;
				if (tool == -1)
					tool = bed_id != 255 ? bed_id : -1;
				rundebug("settemp %d %f", tool, r.X[0]);
				settemp(tool, r.X[0]);
				prepare_interrupt();
				shmem->interrupt_ints[0] = tool;
				shmem->interrupt_floats[0] = r.X[0];
				send_to_parent(CMD_UPDATE_TEMP);
				break;
			}
			case RUN_WAITTEMP:
			{
				int tool = r.tool;
				if (tool == -2)
					tool = bed_id != 255 ? bed_id : -1;
				if (tool == -3) {
					for (int i = 0; i < num_temps; ++i) {
						if (temps[i].min_alarm >= 0 || temps[i].max_alarm < MAXINT) {
							run_file_wait += 1;
							waittemp(i, temps[i].min_alarm, temps[i].max_alarm);
						}
					}
					moving = true;
					break;
				}
				if (tool < 0 || tool >= num_temps) {
					if (tool != -1)
						debug("cannot wait for invalid temp %d", tool);
					break;
				}
				else
					rundebug("waittemp %d", tool);
				if (temps[tool].adctarget[0] >= 0 && temps[tool].adctarget[0] < MAXINT) {
					rundebug("waiting");
					run_file_wait += 1;
					waittemp(tool, temps[tool].target[0], temps[tool].max_alarm);
				}
				else
					rundebug("not waiting");
				moving = true;
				break;
			}
			case RUN_SETPOS:
				if (r.tool >= spaces[1].num_axes) {
					debug("Not setting position of invalid extruder %d", r.tool);
					break;
				}
				setpos(1, r.tool, r.E, true);
				break;
			case RUN_WAIT:
				if (r.X[0] > 0) {
					run_file_timer.it_value.tv_sec = r.X[0];
					run_file_timer.it_value.tv_nsec = (r.X[0] - run_file_timer.it_value.tv_sec) * 1e9;
					run_file_wait += 1;
					timerfd_settime(pollfds[0].fd, 0, &run_file_timer, NULL);
					moving = true;
				}
				break;
			case RUN_CONFIRM:
			{
				int len = min(strings[r.tool].len, PATH_MAX);
				memcpy(const_cast<char *>(shmem->interrupt_str), &reinterpret_cast<char const *>(run_file_map)[run_file_first_string + strings[r.tool].start], len);
				run_file_wait += 1;
				prepare_interrupt();
				shmem->interrupt_ints[0] = r.X[0] ? 1 : 0;
				shmem->interrupt_ints[1] = len;
				send_to_parent(CMD_CONFIRM);
				moving = true;
				break;
			}
			case RUN_PARK:
				run_file_wait += 1;
				parkwaiting = true;
				prepare_interrupt();
				send_to_parent(CMD_PARKWAIT);
				moving = true;
				break;
			default:
				debug("Invalid record type %d in %s", r.type, run_file_name.c_str());
				break;
		}
		settings.run_file_current += 1;
	}
	rundebug("run queue done");
	if (run_file_map && settings.run_file_current >= run_file_num_records && !run_file_wait && settings.queue_start == settings.queue_end) {
		// Done.
		//debug("done running file");
		if (!computing_move && !sending_fragment && !transmitting_fragment && !arch_running()) {
			abort_run_file();
			num_file_done_events += 1;
		}
	}
	next_move(start_time);
	lock = false;
}

double run_find_pos(const double pos[3]) {
	// Find position in toolpath that is closest to requested position.
	if (!run_file_map)
		return NAN;
	double dist = INFINITY;
	double current[3] = {NAN, NAN, NAN};
	double center[3];
	double normal[3];
	double record = NAN;
	// These are not used yet.
	(void)&center;
	(void)&normal;
	for (int i = 0; i < run_file_num_records; ++i) {
		switch (run_file_map[i].type) {
			case RUN_SYSTEM:
			case RUN_GPIO:
			case RUN_SETTEMP:
			case RUN_WAITTEMP:
			case RUN_SETPOS:
			case RUN_WAIT:
			case RUN_CONFIRM:
			case RUN_PARK:
				continue;
			case RUN_POLY3PLUS:
			case RUN_POLY3MINUS:
				double target[3] = {run_file_map[i].X[0], run_file_map[i].X[1], run_file_map[i].X[2]};
				int k;
				double pt = 0, tt = 0;
				for (k = 0; k < 3; ++k) {
					if (std::isnan(pos[k]))
						continue;
					if (std::isnan(target[k])) {
						target[k] = current[k];
						if (std::isnan(target[k]))
							break;
					}
					// ((pos-O).(target-O))/((target-O).(target-O))*(target-O) = projection-O
					pt += (pos[k] - current[k]) * (target[k] - current[k]);
					tt += (target[k] - current[k]) * (target[k] - current[k]);
				}
				if (k < 3) {
					// Target position was NaN, pos was not.
					continue;
				}
				double fraction = pt / tt;
				if (fraction < 0)
					fraction = 0;
				if (fraction > 1)
					fraction = 1;
				double d = 0;
				for (k = 0; k < 3; ++k) {
					if (std::isnan(pos[k]))
						continue;
					double dd = pos[k] - ((target[k] - current[k]) * fraction + current[k]);
					d += dd * dd;
				}
				if (d < dist) {
					dist = d;
					record = i + fraction;
				}
				for (k = 0; k < 3; ++k) {
					if (!std::isnan(target[k]))
						current[k] = target[k];
				}
		}
	}
	return record;
}

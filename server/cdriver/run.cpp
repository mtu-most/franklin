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

static Run_Record run_preline;

static double probe_adjust;

void run_file(int name_len, char const *name, int probe_name_len, char const *probename, bool start, double sina, double cosa, int audio) {
	rundebug("run file %d %f %f", start, sina, cosa);
	abort_run_file();
	if (name_len == 0)
		return;
	strncpy(run_file_name, name, name_len);
	run_file_name[name_len] = '\0';
	strncpy(probe_file_name, probename, probe_name_len);
	probe_file_name[probe_name_len] = '\0';
	settings.run_time = 0;
	settings.run_dist = 0;
	settings.run_file_current = 0;
	int probe_fd = -1;
	if (probe_name_len > 0) {
		probe_fd = open(probe_file_name, O_RDONLY);
		if (probe_fd < 0) {
			debug("Failed to open probe file '%s': %s", probe_file_name, strerror(errno));
			return;
		}
		struct stat stat;
		if (fstat(probe_fd, &stat) < 0) {
			debug("Failed to stat probe file '%s': %s", probe_file_name, strerror(errno));
			close(probe_fd);
			return;
		}
		probe_file_size = stat.st_size;
		if (probe_file_size < 0 || unsigned(probe_file_size) < sizeof(ProbeFile)) {
			debug("Probe file too short");
			close(probe_fd);
			return;
		}
	}
	int fd = open(run_file_name, O_RDONLY);
	if (fd < 0) {
		debug("Failed to open run file '%s': %s", run_file_name, strerror(errno));
		if (probe_name_len > 0)
			close(probe_fd);
		return;
	}
	struct stat stat;
	if (fstat(fd, &stat) < 0) {
		debug("Failed to stat run file '%s': %s", run_file_name, strerror(errno));
		close(fd);
		if (probe_name_len > 0)
			close(probe_fd);
		return;
	}
	run_file_size = stat.st_size;
	run_file_map = reinterpret_cast<Run_Record *>(mmap(NULL, run_file_size, PROT_READ, MAP_SHARED, fd, 0));
	close(fd);
	if (probe_name_len > 0) {
		probe_file_map = reinterpret_cast<ProbeFile *>(mmap(NULL, probe_file_size, PROT_READ, MAP_SHARED, probe_fd, 0));
		close(probe_fd);
		if (((probe_file_map->nx + 1) * (probe_file_map->ny + 1)) * sizeof(double) + sizeof(ProbeFile) != unsigned(probe_file_size)) {
			debug("Invalid probe file size %ld != %ld", probe_file_size, ((probe_file_map->nx + 1) * (probe_file_map->ny + 1)) * sizeof(double) + sizeof(ProbeFile));
			munmap(probe_file_map, probe_file_size);
			munmap(run_file_map, run_file_size);
			probe_file_map = NULL;
			run_file_map = NULL;
			return;
		}
	}
	else
		probe_file_map = NULL;
	if (audio < 0) {
		// File format:
		// records
		// strings
		// int32_t stringlengths[]
		// int32_t numstrings
		// double bbox[8]
		run_file_num_strings = read_num(run_file_size - sizeof(double) * 8 - sizeof(int32_t));
		debug("num strings: %d", run_file_num_strings);
		strings = reinterpret_cast<String *>(malloc(run_file_num_strings * sizeof(String)));
		off_t pos = run_file_size - sizeof(double) * 8 - sizeof(int32_t) - sizeof(int32_t) * run_file_num_strings;
		off_t current = 0;
		for (int i = 0; i < run_file_num_strings; ++i) {
			strings[i].start = current;
			strings[i].len = read_num(pos + sizeof(int32_t) * i);
			current += strings[i].len;
		}
		run_file_first_string = pos - current;
		run_file_num_records = run_file_first_string / sizeof(Run_Record);
	}
	else {
		audio_hwtime_step = 1000000. / *reinterpret_cast <double *>(run_file_map);
		run_file_num_records = run_file_size - sizeof(double);
	}
	run_file_wait_temp = 0;
	run_file_wait = start ? 0 : 1;
	run_file_timer.it_interval.tv_sec = 0;
	run_file_timer.it_interval.tv_nsec = 0;
	run_file_refx = targetx;
	run_file_refy = targety;
	probe_adjust = 0;
	//debug("run target %f %f", targetx, targety);
	run_file_sina = sina;
	run_file_cosa = cosa;
	run_file_audio = audio;
	run_preline.X = NAN;
	run_preline.Y = NAN;
	run_preline.Z = NAN;
	run_preline.E = NAN;
	run_file_fill_queue();
}

void abort_run_file() {
	run_file_finishing = false;
	if (!run_file_map)
		return;
	munmap(run_file_map, run_file_size);
	run_file_map = NULL;
	if (probe_file_map) {
		munmap(probe_file_map, probe_file_size);
		probe_file_map = NULL;
	}
	free(strings);
	strings = NULL;
	arch_stop_audio();
}

static double handle_probe(double ox, double oy, double z) {
	ProbeFile *&p = probe_file_map;
	if (!p)
		return z + probe_adjust;
	if (isnan(ox) || isnan(oy) || isnan(z))
		return NAN;
	ox -= p->targetx;
	oy -= p->targety;
	double x = ox * p->cosa + oy * p->sina;
	double y = oy * p->cosa - ox * p->sina;
	x -= p->x0;
	y -= p->y0;
	int ix, iy;
	if (p->w == 0 || p->nx == 0) {
		x = 0;
		ix = 0;
	}
	else {
		x /= p->w / p->nx;
		if (x < 0)
			x = 0;
		ix = floor(x);
		if (x >= p->nx) {
			x = p->nx;
			ix = floor(x) - 1;
		}
	}
	if (p->h == 0 || p->ny == 0) {
		y = 0;
		iy = 0;
	}
	else {
		y /= p->h / p->ny;
		if (y < 0)
			y = 0;
		iy = floor(y);
		if (y >= p->ny) {
			y = p->ny;
			iy = floor(y) - 1;
		}
	}
	double fx = x - ix;
	double fy = y - iy;
	double l = p->sample[iy * (p->nx + 1) + ix] * (1 - fy) + p->sample[(iy + 1) * (p->nx + 1) + ix] * fy;
	double r = p->sample[iy * (p->nx + 1) + (ix + 1)] * (1 - fy) + p->sample[(iy + 1) * (p->nx + 1) + (ix + 1)] * fy;
	return z + l * (1 - fx) + r * fx + probe_adjust;
}

void run_file_fill_queue() {
	static bool lock = false;
	if (lock)
		return;
	lock = true;
	rundebug("run queue, current = %d/%d wait = %d tempwait = %d q = %d %d %d finish = %d", settings.run_file_current, run_file_num_records, run_file_wait, run_file_wait_temp, settings.queue_end, settings.queue_start, settings.queue_full, run_file_finishing);
	if (run_file_audio >= 0) {
		while (true) {
			if (!run_file_map || run_file_wait || run_file_finishing)
				break;
			if (settings.run_file_current >= run_file_num_records) {
				run_file_finishing = true;
				//debug("done running audio");
				break;
			}
			int16_t next = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
			if (next == running_fragment)
				break;
			settings.run_file_current = arch_send_audio(&reinterpret_cast <uint8_t *>(run_file_map)[sizeof(double)], settings.run_file_current, run_file_num_records, run_file_audio);
			current_fragment = next;
			//debug("current_fragment = next; %d", current_fragment);
			store_settings();
			if ((current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER >= MIN_BUFFER_FILL && !stopping)
				arch_start_move(0);
		}
		lock = false;
		return;
	}
	int cbs = 0;
	bool must_move = true;
	while (must_move) {
		must_move = false;
		while (run_file_map	// There is a file to run.
				&& (settings.queue_end - settings.queue_start + QUEUE_LENGTH) % QUEUE_LENGTH < 4	// There is space in the queue.
				&& !settings.queue_full	// Really, there is space in the queue.
				&& settings.run_file_current < run_file_num_records	// There are records to send.
				&& !run_file_wait_temp	// We are not waiting for a temp alarm.
				&& !run_file_wait	// We are not waiting for something else (pause or confirm).
				&& !run_file_finishing) {	// We are not waiting for underflow (should be impossible anyway, if there are commands in the queue).
			int t = run_file_map[settings.run_file_current].type;
			if (t != RUN_LINE && t != RUN_PRE_LINE && t != RUN_PRE_ARC && t != RUN_ARC && (arch_running() || settings.queue_end != settings.queue_start || computing_move || sending_fragment || transmitting_fragment))
				break;
			Run_Record &r = run_file_map[settings.run_file_current];
			rundebug("running %d: %d %d", settings.run_file_current, r.type, r.tool);
			switch (r.type) {
				case RUN_SYSTEM:
				{
					char const *cmd = strndupa(&reinterpret_cast<char const *>(run_file_map)[run_file_first_string + strings[r.tool].start], strings[r.tool].len);
					debug("Running system command: %ld %d %s", strings[r.tool].start, strings[r.tool].len, cmd);
					int ret = system(cmd);
					debug("Done running system command, return = %d", ret);
					break;
				}
				case RUN_PRE_ARC:
				{
					double x = r.X * run_file_cosa - r.Y * run_file_sina + run_file_refx;
					double y = r.Y * run_file_cosa + r.X * run_file_sina + run_file_refy;
					double z = r.Z;
					//debug("line %f %f %f", x, y, z);
					queue[settings.queue_end].center[0] = x;
					queue[settings.queue_end].center[1] = y;
					queue[settings.queue_end].center[2] = handle_probe(x, y, z);
					queue[settings.queue_end].normal[0] = r.E;
					queue[settings.queue_end].normal[1] = r.F_start;
					queue[settings.queue_end].normal[2] = r.F_end;
					break;
				}
				case RUN_PRE_LINE:
				{
					run_preline.X = r.X;
					run_preline.Y = r.Y;
					run_preline.Z = r.Z;
					run_preline.E = r.E;
					run_preline.tool = r.tool;
					break;
				}
				case RUN_LINE:
				case RUN_ARC:
				{
					queue[settings.queue_end].single = false;
					queue[settings.queue_end].probe = false;
					queue[settings.queue_end].arc = r.type == RUN_ARC;
					queue[settings.queue_end].f[0] = r.F_start;
					queue[settings.queue_end].f[1] = r.F_end;
					double x = r.X * run_file_cosa - r.Y * run_file_sina + run_file_refx;
					double y = r.Y * run_file_cosa + r.X * run_file_sina + run_file_refy;
					double z = r.Z;
					//debug("line/arc %d: %f %f %f", settings.run_file_current, x, y, z);
					int num0 = spaces[0].num_axes;
					if (num0 > 0) {
						queue[settings.queue_end].data[0] = x;
						if (num0 > 1) {
							queue[settings.queue_end].data[1] = y;
							if (num0 > 2) {
								queue[settings.queue_end].data[2] = handle_probe(x, y, z);
								if (num0 > 3) {
									queue[settings.queue_end].data[3] = run_preline.X;
									if (num0 > 4) {
										queue[settings.queue_end].data[4] = run_preline.Y;
										if (num0 > 5) {
											queue[settings.queue_end].data[5] = run_preline.Z;
										}
									}
									run_preline.X = NAN;
									run_preline.Y = NAN;
									run_preline.Z = NAN;
								}
							}
						}
					}
					for (int i = 6; i < num0; ++i)
						queue[settings.queue_end].data[i] = NAN;
					for (int i = 0; i < spaces[1].num_axes; ++i) {
						queue[settings.queue_end].data[num0 + i] = (i == r.tool ? r.E : i == run_preline.tool ? run_preline.E : NAN);
						//debug("queue %d + %d = %f", num0, i, queue[settings.queue_end].data[num0 + i]);
					}
					run_preline.E = NAN;
					num0 += spaces[1].num_axes;
					for (int s = 2; s < NUM_SPACES; ++s) {
						for (int i = 0; i < spaces[s].num_axes; ++i)
							queue[settings.queue_end].data[num0 + i] = NAN;
						num0 += spaces[s].num_axes;
					}
					queue[settings.queue_end].time = r.time;
					queue[settings.queue_end].dist = r.dist;
					queue[settings.queue_end].cb = false;
					settings.queue_end = (settings.queue_end + 1) % QUEUE_LENGTH;
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
					if (r.X) {
						gpios[tool].state = 1;
						SET(gpios[tool].pin);
					}
					else {
						gpios[tool].state = 0;
						RESET(gpios[tool].pin);
					}
					send_host(CMD_UPDATE_PIN, tool, gpios[tool].state);
					break;
				}
				case RUN_SETTEMP:
				{
					int tool = r.tool;
					if (tool == -1)
						tool = bed_id != 255 ? bed_id : -1;
					rundebug("settemp %d %f", tool, r.X);
					settemp(tool, r.X);
					send_host(CMD_UPDATE_TEMP, tool, 0, r.X);
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
								run_file_wait_temp += 1;
								waittemp(i, temps[i].min_alarm, temps[i].max_alarm);
							}
						}
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
						run_file_wait_temp += 1;
						waittemp(tool, temps[tool].target[0], temps[tool].max_alarm);
					}
					else
						rundebug("not waiting");
					break;
				}
				case RUN_SETPOS:
					if (r.tool >= spaces[1].num_axes) {
						debug("Not setting position of invalid extruder %d", r.tool);
						break;
					}
					setpos(1, r.tool, r.X);
					break;
				case RUN_WAIT:
					if (r.X > 0) {
						run_file_timer.it_value.tv_sec = r.X;
						run_file_timer.it_value.tv_nsec = (r.X - run_file_timer.it_value.tv_sec) * 1e9;
						run_file_wait += 1;
						timerfd_settime(pollfds[0].fd, 0, &run_file_timer, NULL);
					}
					break;
				case RUN_CONFIRM:
				{
					int len = min(strings[r.tool].len, 250);
					memcpy(datastore, &reinterpret_cast<char const *>(run_file_map)[run_file_first_string + strings[r.tool].start], len);
					run_file_wait += 1;
					send_host(CMD_CONFIRM, r.X ? 1 : 0, 0, 0, 0, len);
					break;
				}
				case RUN_PARK:
					run_file_wait += 1;
					send_host(CMD_PARKWAIT);
					break;
				default:
					debug("Invalid record type %d in %s", r.type, run_file_name);
					break;
			}
			settings.run_file_current += 1;
			if (!computing_move && (settings.queue_start != settings.queue_end || settings.queue_full))
				must_move = true;
		}
		if (must_move) {
			while (!computing_move && (settings.queue_start != settings.queue_end || settings.queue_full))
				cbs += next_move();
		}
	}
	if (cbs > 0)
		send_host(CMD_MOVECB, cbs);
	buffer_refill();
	rundebug("run queue done");
	if (run_file_map && settings.run_file_current >= run_file_num_records && !run_file_wait_temp && !run_file_wait && !run_file_finishing) {
		// Done.
		//debug("done running file");
		if (!computing_move && !sending_fragment && !arch_running()) {
			send_host(CMD_FILE_DONE);
			abort_run_file();
		}
		else
			run_file_finishing = true;
	}
	lock = false;
	return;
}

void run_adjust_probe(double x, double y, double z) {
	probe_adjust = 0;
	double probe_z = handle_probe(x, y, 0);
	probe_adjust = z - probe_z;
}

double run_find_pos(double pos[3]) {
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
			case RUN_PRE_LINE:
			case RUN_GPIO:
			case RUN_SETTEMP:
			case RUN_WAITTEMP:
			case RUN_SETPOS:
			case RUN_WAIT:
			case RUN_CONFIRM:
			case RUN_PARK:
				continue;
			case RUN_PRE_ARC:
				center[0] = run_file_map[i].X;
				center[1] = run_file_map[i].Y;
				center[2] = run_file_map[i].Z;
				normal[0] = run_file_map[i].E;
				normal[1] = run_file_map[i].F_start;
				normal[2] = run_file_map[i].F_end;
				continue;
			case RUN_ARC:
				// TODO: handle arc; workaround: fall through, handle as line.
			case RUN_LINE:
				double target[3] = {run_file_map[i].X, run_file_map[i].Y, run_file_map[i].Z};
				int k;
				double pt = 0, tt = 0;
				for (k = 0; k < 3; ++k) {
					if (isnan(pos[k]))
						continue;
					if (isnan(target[k])) {
						target[k] = current[k];
						if (isnan(target[k]))
							break;
					}
					/* ((pos-O).(target-O))/((target-O).(target-O))*(target-O) = projection-O
					   */
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
					if (isnan(pos[k]))
						continue;
					double dd = pos[k] - ((target[k] - current[k]) * fraction + current[k]);
					d += dd * dd;
				}
				if (d < dist) {
					dist = d;
					record = i + fraction;
				}
				for (k = 0; k < 3; ++k) {
					if (!isnan(target[k]))
						current[k] = target[k];
				}
		}
	}
	return record;
}

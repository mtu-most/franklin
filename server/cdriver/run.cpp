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

void run_file(int name_len, char const *name, int probe_name_len, char const *probename, double refx, double refy, double refz, double sina, double cosa, int audio) {
	//debug("run file %f %f %f %f %f", refx, refy, refz, sina, cosa);
	abort_run_file();
	if (name_len == 0)
		return;
	strncpy(run_file_name, name, name_len);
	run_file_name[name_len] = '\0';
	strncpy(probe_file_name, probename, probe_name_len);
	probe_file_name[probe_name_len] = '\0';
	run_time = 0;
	run_dist = 0;
	settings.run_file_current = 0;
	int probe_fd;
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
		if (probe_file_size < sizeof(ProbeFile)) {
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
		if ((probe_file_map->nx * probe_file_map->ny) * sizeof(double) + sizeof(ProbeFile) != probe_file_size) {
			debug("Invalid probe file size");
			munmap(probe_file_map, probe_file_size);
			munmap(run_file_map, run_file_size);
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
	else
		run_file_num_records = run_file_size;
	run_file_wait_temp = 0;
	run_file_wait = 0;
	run_file_timer.it_interval.tv_sec = 0;
	run_file_timer.it_interval.tv_nsec = 0;
	run_file_refx = refx;
	run_file_refy = refy;
	run_file_refz = refz;
	run_file_sina = sina;
	run_file_cosa = cosa;
	run_file_audio = audio;
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
}

enum {
	RUN_SYSTEM,
	RUN_GOTO,
	RUN_GPIO,
	RUN_SETTEMP,
	RUN_WAITTEMP,
	RUN_SETPOS,
	RUN_WAIT,
	RUN_CONFIRM,
};

static double handle_probe(double x, double y, double z) {
	return z;
}

void run_file_fill_queue() {
	static bool lock = false;
	if (lock)
		return;
	lock = true;
	rundebug("run queue, wait = %d tempwait = %d q = %d %d", run_file_wait, run_file_wait_temp, settings.queue_end, settings.queue_start);
	if (run_file_audio >= 0) {
		while (true) {
			if (!run_file_map || run_file_wait || run_file_finishing)
				break;
			if (settings.run_file_current >= run_file_num_records) {
				run_file_finishing = true;
				debug("done running audio");
				break;
			}
			int16_t next = (current_fragment + 1) % FRAGMENTS_PER_BUFFER;
			if (next == running_fragment)
				break;
			settings.run_file_current = arch_send_audio(reinterpret_cast <uint8_t *>(run_file_map), settings.run_file_current, run_file_num_records, run_file_audio);
			current_fragment = next;
			store_settings();
			if ((current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER >= MIN_BUFFER_FILL && !stopping)
				arch_start_move(0);
		}
		lock = false;
		return;
	}
	while (run_file_map	// There is a file to run.
		       	&& (settings.queue_end - settings.queue_start + QUEUE_LENGTH) % QUEUE_LENGTH < 2	// There is space in the queue.
			&& !settings.queue_full	// Really, there is space in the queue.
			&& (run_file_map[settings.run_file_current].type == RUN_GOTO || !arch_running())	// The command is GOTO (buffered), or the buffer is empty.
			&& settings.run_file_current < run_file_num_records	// There are records to send.
			&& !run_file_wait_temp	// We are not waiting for a temp alarm.
			&& !run_file_wait	// We are not waiting for something else (pause or confirm).
			&& !run_file_finishing) {	// We are not waiting for underflow (should be impossible anyway, if there are commands in the queue).
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
			case RUN_GOTO:
				queue[settings.queue_end].probe = false;
				queue[settings.queue_end].f[0] = r.f;
				queue[settings.queue_end].f[1] = r.F;
				if (num_spaces > 0) {
					double x = r.X * run_file_cosa - r.Y * run_file_sina + run_file_refx;
					double y = r.Y * run_file_cosa + r.X * run_file_sina + run_file_refy;
					double z = r.Z + run_file_refz;
					//debug("goto %f %f %f", x, y, z);
					int num0 = spaces[0].num_axes;
					if (num0 > 0) {
						queue[settings.queue_end].data[0] = x;
						if (num0 > 1) {
							queue[settings.queue_end].data[1] = y;
							if (num0 > 2)
								queue[settings.queue_end].data[2] = handle_probe(x, y, z);
						}
					}
					for (int i = 3; i < num0; ++i)
						queue[settings.queue_end].data[i] = NAN;
					if (num_spaces > 1) {
						for (int i = 0; i < spaces[1].num_axes; ++i)
							queue[settings.queue_end].data[num0 + i] = (i == r.tool ? r.E : NAN);
						num0 += spaces[1].num_axes;
						for (int s = 2; s < num_spaces; ++s) {
							for (int i = 0; i < spaces[s].num_axes; ++i)
								queue[settings.queue_end].data[num0] = NAN;
							num0 += spaces[s].num_axes;
						}
					}
				}
				queue[settings.queue_end].time = r.time;
				queue[settings.queue_end].dist = r.dist;
				queue[settings.queue_end].cb = false;
				settings.queue_end = (settings.queue_end + 1) % QUEUE_LENGTH;
				if (stopped)
					next_move();
				else
					rundebug("no");
				buffer_refill();
				break;
			case RUN_GPIO:
				//if (r.tool == -1)
				//	r.tool = fan_id;
				//else if (r.tool == -2)
				//	r.tool = spindle_id;
				if (r.tool < 0 || r.tool >= num_gpios) {
					debug("cannot set invalid gpio %d", r.tool);
					break;
				}
				if (r.x) {
					gpios[r.tool].state = 1;
					SET(gpios[r.tool].pin);
				}
				else {
					gpios[r.tool].state = 0;
					RESET(gpios[r.tool].pin);
				}
				send_host(CMD_UPDATE_PIN, r.tool, gpios[r.tool].state);
				break;
			case RUN_SETTEMP:
				//if (r.tool == -1)
				//	r.tool = bed_id;
				rundebug("settemp %d %f", r.tool, r.x);
				settemp(r.tool, r.x);
				send_host(CMD_UPDATE_TEMP, r.tool, 0, r.x);
				break;
			case RUN_WAITTEMP:
				//if (r.tool == -1)
				//	r.tool = bed_id;
				if (r.tool == -2) {
					for (int i = 0; i < num_temps; ++i) {
						if (temps[i].min_alarm >= 0 || temps[i].max_alarm < MAXINT) {
							run_file_wait_temp += 1;
							waittemp(i, temps[i].min_alarm, temps[i].max_alarm);
						}
					}
					break;
				}
				if (r.tool < 0 || r.tool >= num_temps) {
					debug("cannot wait for invalid temp %d", r.tool);
					break;
				}
				else
					rundebug("waittemp %d", r.tool);
				if (temps[r.tool].adctarget[0] >= 0 && temps[r.tool].adctarget[0] < MAXINT) {
					rundebug("waiting");
					run_file_wait_temp += 1;
					waittemp(r.tool, temps[r.tool].target[0], temps[r.tool].max_alarm);
				}
				else
					rundebug("not waiting");
				break;
			case RUN_SETPOS:
				if (num_spaces < 2 || r.tool >= spaces[1].num_axes) {
					debug("Not setting position of invalid extruder %d", r.tool);
					break;
				}
				setpos(1, r.tool, r.x);
				break;
			case RUN_WAIT:
				if (r.x > 0) {
					run_file_timer.it_value.tv_sec = r.x;
					run_file_timer.it_value.tv_nsec = (r.x - run_file_timer.it_value.tv_sec) * 1e9;
					run_file_wait += 1;
					timerfd_settime(pollfds[0].fd, 0, &run_file_timer, NULL);
				}
				break;
			case RUN_CONFIRM:
			{
				int len = min(strings[r.tool].len, 250);
				memcpy(datastore, &reinterpret_cast<char const *>(run_file_map)[run_file_first_string + strings[r.tool].start], len);
				run_file_wait += 1;
				send_host(CMD_CONFIRM, 0, 0, 0, 0, len);
				break;
			}
			default:
				debug("Invalid record type %d in %s", r.type, run_file_name);
				break;
		}
		settings.run_file_current += 1;
	}
	rundebug("run queue done");
	if (run_file_map && settings.run_file_current >= run_file_num_records && !run_file_wait_temp && !run_file_wait && !run_file_finishing) {
		// Done.
		debug("done running file");
		run_file_finishing = true;
	}
	lock = false;
	return;
}

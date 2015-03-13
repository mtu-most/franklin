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

void run_file(int name_len, char const *name, float refx, float refy, float refz, float sina, float cosa) {
	//debug("run file %f %f %f %f %f", refx, refy, refz, sina, cosa);
	abort_run_file();
	strncpy(run_file_name, name, name_len);
	run_file_name[name_len] = '\0';
	settings[current_fragment].run_file_current = 0;
	int fd = open(run_file_name, O_RDONLY);
	if (fd < 0) {
		debug("Failed to open run file %s: %s", run_file_name, strerror(errno));
		return;
	}
	struct stat stat;
	if (fstat(fd, &stat) < 0) {
		debug("Failed to stat run file %s: %s", run_file_name, strerror(errno));
		close(fd);
		return;
	}
	run_file_size = stat.st_size;
	run_file_map = reinterpret_cast<Run_Record *>(mmap(NULL, run_file_size, PROT_READ, MAP_SHARED, fd, 0));
	close(fd);
	run_file_num_strings = read_num(run_file_size - 4 * 6 - 4);
	strings = reinterpret_cast<String *>(malloc(run_file_num_strings * sizeof(String)));
	off_t pos = run_file_size - 4 * (6 + 1 + run_file_num_strings);
	off_t current = 0;
	for (int i = 0; i < run_file_num_strings; ++i) {
		strings[i].start = current;
		strings[i].len = read_num(pos + 4 * i);
		current += strings[i].len;
	}
	run_file_first_string = pos - current;
	run_file_num_records = run_file_first_string / sizeof(Run_Record);
	run_file_wait_temp = 0;
	run_file_wait = 0;
	run_file_timer.it_interval.tv_sec = 0;
	run_file_timer.it_interval.tv_nsec = 0;
	run_file_refx = refx;
	run_file_refy = refy;
	run_file_refz = refz;
	run_file_sina = sina;
	run_file_cosa = cosa;
	run_file_fill_queue();
}

void abort_run_file() {
	if (!run_file_map)
		return;
	munmap(run_file_map, run_file_size);
	run_file_map = NULL;
	free(strings);
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

void run_file_fill_queue() {
	static bool lock = false;
	if (lock)
		return;
	lock = true;
	rundebug("run queue, wait = %d tempwait = %d q = %d %d", run_file_wait, run_file_wait_temp, settings[current_fragment].queue_end, settings[current_fragment].queue_start);
	while (run_file_map && (settings[current_fragment].queue_end - settings[current_fragment].queue_start + QUEUE_LENGTH) % QUEUE_LENGTH < 2 && (run_file_map[settings[current_fragment].run_file_current].type == RUN_GOTO || !arch_running()) && !settings[current_fragment].queue_full && settings[current_fragment].run_file_current < run_file_num_records && !run_file_wait_temp && !run_file_wait) {
		Run_Record &r = run_file_map[settings[current_fragment].run_file_current];
		rundebug("running %d: %d %d", settings[current_fragment].run_file_current, r.type, r.tool);
		switch (r.type) {
			case RUN_SYSTEM:
			{
				char const *cmd = strndupa(&reinterpret_cast<char const *>(run_file_map)[run_file_first_string + strings[r.tool].start], strings[r.tool].len);
				debug("Running system command: %ld %d %s", strings[r.tool].start, strings[r.tool].len, cmd);
				system(cmd);
				break;
			}
			case RUN_GOTO:
				queue[settings[current_fragment].queue_end].probe = false;
				queue[settings[current_fragment].queue_end].f[0] = r.f;
				queue[settings[current_fragment].queue_end].f[1] = r.F;
				if (num_spaces > 0) {
					float x = r.X * run_file_cosa - r.Y * run_file_sina + run_file_refx;
					float y = r.Y * run_file_cosa + r.X * run_file_sina + run_file_refy;
					float z = r.Z + run_file_refz;
					//debug("goto %f %f %f", x, y, z);
					int num0 = spaces[0].num_axes;
					if (num0 > 0)
						queue[settings[current_fragment].queue_end].data[0] = x;
					if (num0 > 1)
						queue[settings[current_fragment].queue_end].data[1] = y;
					if (num0 > 2)
						queue[settings[current_fragment].queue_end].data[2] = z;
					for (int i = 3; i < num0; ++i)
						queue[settings[current_fragment].queue_end].data[i] = NAN;
					if (num_spaces > 1) {
						for (int i = 0; i < spaces[1].num_axes; ++i)
							queue[settings[current_fragment].queue_end].data[num0 + i] = (i == r.tool ? r.E : NAN);
						num0 += spaces[1].num_axes;
						for (int s = 2; s < num_spaces; ++s) {
							for (int i = 0; i < spaces[s].num_axes; ++i)
								queue[settings[current_fragment].queue_end].data[num0] = NAN;
							num0 += spaces[s].num_axes;
						}
					}
				}
				queue[settings[current_fragment].queue_end].cb = false;
				settings[current_fragment].queue_end = (settings[current_fragment].queue_end + 1) % QUEUE_LENGTH;
				if (stopped) {
					next_move();
					buffer_refill();
				}
				else
					rundebug("no");
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
		settings[current_fragment].run_file_current += 1;
	}
	rundebug("run queue done");
	if (run_file_map && !arch_running() && settings[current_fragment].run_file_current >= run_file_num_records && !run_file_wait_temp && !run_file_wait) {
		// Done.
		debug("done running file");
		send_host(CMD_FILE_DONE);
		abort_run_file();
	}
	lock = false;
	return;
}

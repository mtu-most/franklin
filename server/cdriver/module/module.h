/* module.h - declarations for Franklin
 * Copyright 2018 Bas Wijnen <wijnen@debian.org>
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

#ifndef _MODULE_H
#define _MODULE_H

#include <cmath>
#include <cstdio>
#include <stdint.h>
#include <cinttypes>

// EXTERN is defined in exactly one file; the variables are defined in that file.
#ifndef EXTERN
#define EXTERN extern
#endif

#define ID_SIZE 8
#define UUID_SIZE 16
#define NUM_SPACES 3

#define LONGFMT PRId64

#define PATTERN_MAX int(9 * sizeof(double))

// Extruder and follower type data.
struct ExtruderAxisData {
	double offset[6];
	ExtruderAxisData() : offset{0,0,0,0,0,0} {}
};
struct FollowerMotorData {
	int space, motor;
	FollowerMotorData() : space(-1), motor(0) {}
};

struct MoveCommand {
	bool cb;
	int probe, single, reverse;
	double a0, v0;
	int tool;	// Negative value means follower ~tool.
	double target[6];
	double unitg[6];
	double unith[6];
	double Jg, Jh;
	double tf;
	double e;
	double time;
	int64_t gcode_line;
	int64_t current_restore;
	int pattern_size;	// in bytes; each bit is a pulse.
	uint8_t pattern[PATTERN_MAX];
};

struct Run_Record {
	uint8_t type;
	int32_t tool;
	double X[3];
	double h[3];
	double Jg;
	double tf;
	double v0;
	double E;
	double time;
	int64_t gcode_line;
} __attribute__((__packed__));

enum Command {
	// from host
	CMD_SET_UUID,		// 00	22 bytes: uuid.
	CMD_GET_UUID,		// 01	0.  Reply: UUID.
	CMD_MOVE,		// 02	1-2 byte: which channels (depending on number of extruders); channel * 4 byte: values [fraction/s], [mm].  Reply (later): MOVECB.
	CMD_PARSE_GCODE,	// 03	2 byte: in filename length, in filename, out filename
	CMD_RUN,		// 04	n byte: filename.
	CMD_SLEEP,		// 05	1 byte: which channel (b0-6); on/off (b7 = 1/0).
	CMD_SETTEMP,		// 06	1 byte: which channel; 4 bytes: target [°C].
	CMD_WAITTEMP,		// 07	1 byte: which channel; 4 bytes: lower limit; 4 bytes: upper limit [°C].  Reply (later): TEMPCB.  Disable with WAITTEMP (NAN, NAN).
	CMD_TEMP_VALUE,		// 08	1 byte: which channel.  Reply: TEMP. [°C]
	CMD_POWER_VALUE,	// 09	1 byte: which channel.  Reply: POWER. [μs, μs]
	CMD_SETPOS,		// 0a	1 byte: which channel; 4 bytes: pos.
	CMD_GETPOS,		// 0b	1 byte: which channel.  Reply: POS. [steps, mm]
	CMD_READ_GLOBALS,	// 0c
	CMD_WRITE_GLOBALS,	// 0d
	CMD_READ_SPACE_INFO,	// 0e	1 byte: which channel.  Reply: DATA.
	CMD_READ_SPACE_AXIS,	// 0f	1 byte: which channel.  Reply: DATA.
	CMD_READ_SPACE_MOTOR,	// 10	1 byte: which channel; n bytes: data.
	CMD_WRITE_SPACE_INFO,	// 11	1 byte: which channel.  Reply: DATA.
	CMD_WRITE_SPACE_AXIS,	// 12	1 byte: which channel; n bytes: data.
	CMD_WRITE_SPACE_MOTOR,	// 13	1 byte: which channel; n bytes: data.
	CMD_READ_TEMP,		// 14	1 byte: which channel.  Reply: DATA.
	CMD_WRITE_TEMP,		// 15	1 byte: which channel; n bytes: data.
	CMD_READ_GPIO,		// 16	1 byte: which channel.  Reply: DATA.
	CMD_WRITE_GPIO,		// 17	1 byte: which channel; n bytes: data.
	CMD_QUEUED,		// 18	1 byte: 0: query queue length; 1: stop and query queue length.  Reply: QUEUE.
	CMD_PIN_VALUE,		// 19	1 byte: which channel. Reply: GPIO.
	CMD_HOME,		// 1a	1 byte: homing space; n bytes: homing type (0=pos, 1=neg, 3=no)
	CMD_FORCE_DISCONNECT,	// 1b	0
	CMD_CONNECT,		// 1c	8 byte: run ID, n bytes: port name (0-terminated)
	CMD_RECONNECT,		// 1d	n bytes: port name (0-terminated)
	CMD_PAUSE,		// 1e
	CMD_RESUME,		// 1f
	CMD_UNPAUSE,		// 20
	CMD_GET_TIME,		// 21
	CMD_SPI,		// 22
	CMD_ADJUST_PROBE,	// 23	3 doubles: probe position.
	CMD_TP_GETPOS,		// 24
	CMD_TP_SETPOS,		// 25	1 double: new toolpath position.
	CMD_TP_FINDPOS,		// 26	3 doubles: search position or NaN.
	CMD_MOTORS2XYZ,		// 27	1 byte: which space, n doubles: motor positions.  Reply: m times XYZ.
};

enum InterruptCommand {
	CMD_LIMIT,		// 00	1 byte: which channel.
	CMD_FILE_DONE,		// 01
	CMD_MOVECB,		// 02	1 byte: number of movecb events.
	CMD_HOMED,		// 03	0
	CMD_TIMEOUT,		// 04	0
	CMD_PINCHANGE,		// 05	1 byte: pin, 1 byte: current value.
	CMD_PINNAME,		// 06
	CMD_DISCONNECT,		// 07	0
	CMD_UPDATE_PIN,		// 08
	CMD_UPDATE_TEMP,	// 09
	CMD_CONFIRM,		// 0a
	CMD_PARKWAIT,		// 0b
	CMD_CONNECTED,		// 0c
	CMD_TEMPCB,		// 0d	1 byte: which channel.  Byte storage for which needs to be sent.
	CMD_MESSAGE,		// 0e
};

enum RunType {
	RUN_SYSTEM,		// 00
	RUN_POLY3PLUS,		// 01
	RUN_POLY3MINUS,		// 02
	RUN_POLY2,		// 03
	RUN_ARC,		// 04
	RUN_ABC,		// 05
	RUN_GOTO,		// 06
	RUN_GPIO,		// 07
	RUN_SETTEMP,		// 08
	RUN_WAITTEMP,		// 09
	RUN_SETPOS,		// 0a
	RUN_WAIT,		// 0b
	RUN_CONFIRM,		// 0c
	RUN_PARK,		// 0d
	RUN_PATTERN,		// 0e
};

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif
struct SharedMemory {
	volatile unsigned char uuid[UUID_SIZE];
	volatile char strs[5][PATH_MAX + 1];
	volatile int ints[500];
	volatile double floats[500];
	volatile MoveCommand move;
	volatile int interrupt_ints[2];
	volatile float interrupt_float;
	volatile char interrupt_str[PATH_MAX + 1];
};

extern "C" {
	EXTERN SharedMemory *shmem;
	EXTERN int memfd, fromserver, toserver, interrupt, interrupt_reply;
}

template <typename T> inline T min(T a, T b) {
	return a < b ? a : b;
}
template <> inline double min <double>(double a, double b) {
	return a < b || std::isnan(b) ? a : b;
}

template <typename T> inline T max(T a, T b) {
	return a > b ? a : b;
}
template <> inline double max <double>(double a, double b) {
	return a > b || std::isnan(b) ? a : b;
}

static inline double compute_max_v(double x, double v, double max_J, double max_a) {
	// Compute maximum vf that can be reached starting from v with J on length x.

	// Time for maximum ramp.
	double t_ramp_max = max_a / max_J;
	// Compute total x with only two ramps.
	double a_top_max = max_J * t_ramp_max;
	double v_top_max = max_J / 2 * t_ramp_max * t_ramp_max;
	// J/6*t^3 cancels against -J/6*t^3
	double x_ramp_max = a_top_max / 2 * t_ramp_max * t_ramp_max + v_top_max * t_ramp_max + 2 * v * t_ramp_max;
	double ret;
	if (x_ramp_max <= x) {
		// Ramp fits on segment, compute size of middle part.
		double t = t_ramp_max;
		double t2 = t * t;
		double t3 = t2 * t;
		double a = max_a / 2;
		double b = max_J * t2 / 2 + max_a * t + v;
		double c = max_a * t2 / 2 + max_J * t3 / 2 + 2 * v * t - x;
		double t_const_a = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
		ret = max_a * (t_const_a + t);
		//fprintf(stderr, "with max a\n");
	}
	else {
		// Ramp does not fit on segment.
		double b = x / max_J;
		double c = 2 * v / max_J;
		double k = std::pow(std::sqrt(81 * b * b + 12 * c * c * c) + 9 * b, 1. / 3);
		double t_ramp = (std::pow(2, 1. / 3) * k * k - 2 * std::pow(3, 1. / 3) * c) / (std::pow(6, 2. / 3) * k);
		ret = max_J * t_ramp * t_ramp;
		//fprintf(stderr, "without max a\n");
	}
	//fprintf(stderr, "compute_max_v: x=%f, v=%f, J=%f, a=%f, ret=%f->%f\n", x, v, max_J, max_a, ret, v + ret);
	// Check the result.
	double max_dv = max_J / 2 * t_ramp_max * t_ramp_max;
	bool have_max_a = ret > 2 * max_dv;
	double v1, v2, t_const_a, t_ramp, a_top;
	if (have_max_a) {
		a_top = max_a;
		v1 = v + max_dv;
		v2 = v + ret - max_dv;
		t_ramp = t_ramp_max;
		double dv_const_a = ret - 2 * max_dv;
		t_const_a = dv_const_a / max_a;
	}
	else {
		v1 = v + ret / 2;
		v2 = v1;
		t_ramp = std::sqrt(2 * (v1 - v) / max_J);
		a_top = max_J * t_ramp;
		t_const_a = 0;
	}
	double s = v * t_ramp + a_top / 2 * t_const_a * t_const_a + v1 * t_const_a + a_top / 2 * t_ramp * t_ramp + v2 * t_ramp;
	if ((s - x) * (s - x) > 1e-5)
		fprintf(stderr, "Warning: max v %f + %f does not fit in requested distance %f != %f with max a,J=%f, %f (%s)\n", v, ret, x, s, max_a, max_J, have_max_a ? "+" : "-");
	// Return 5% less so it still fits on segment with possible rounding errors.
	double result = v + ret * .95;
	return std::isnan(result) ? 0 : result;
}

#ifdef MODULE

#include <string>

#define debug(...) do { fprintf(stderr, "$"); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); fflush(stderr); } while (0)

extern "C" {
	// Globals
	EXTERN double max_deviation;
	EXTERN double max_v, max_a, max_J;
	EXTERN int num_extruders;
	EXTERN ExtruderAxisData *extruder_data;

	void parse_error(void *errors, char const *format, ...);
	void parse_gcode(std::string const &infilename, std::string const &outfilename, void *errors);
}

#endif

#endif

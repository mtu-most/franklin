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
#include <stdint.h>

// EXTERN is defined in exactly one file; the variables are defined in that file.
#ifndef EXTERN
#define EXTERN extern
#endif

#define ID_SIZE 8
#define UUID_SIZE 16
#define NUM_SPACES 3

#define TYPE_CARTESIAN 0
#define TYPE_EXTRUDER 1
#define TYPE_FOLLOWER 2
#define TYPE_DELTA 3
#define TYPE_POLAR 4
#define TYPE_HBOT 5
#define NUM_SPACE_TYPES 6

#define PATTERN_MAX int(9 * sizeof(double))

struct MoveCommand {
	bool cb;
	int probe, single;
	double v0;
	int tool;	// Negative value means follower ~tool.
	double X[6]; 	// Value if given, NAN otherwise. (x, y, z, a, b, c)
	double h[3];
	double Jg;	// If Jg < 0, this is a reverse curve.
	double tf;
	double e;
	double time, dist;
	int pattern_size;	// in bytes; each bit is a pulse.
	uint8_t pattern[PATTERN_MAX];
};

struct Run_Record {
	uint8_t type;
	int32_t tool;
	double g[3];
	double h[3];
	double Jg;	// If Jg < 0, this is a reverse curve.
	double tf;
	double v0;
	double E;
	double time, dist;
	double r;
} __attribute__((__packed__));

enum Command {
	// from host
	CMD_SET_UUID,	// 22 bytes: uuid.
	CMD_GET_UUID,	// 0.  Reply: UUID.
	CMD_MOVE,	// 1-2 byte: which channels (depending on number of extruders); channel * 4 byte: values [fraction/s], [mm].  Reply (later): MOVECB.
	CMD_PARSE_GCODE,// 2 byte: in filename length, in filename, out filename
	CMD_RUN,	// n byte: filename.
	CMD_SLEEP,	// 1 byte: which channel (b0-6); on/off (b7 = 1/0).
	CMD_SETTEMP,	// 1 byte: which channel; 4 bytes: target [°C].
	CMD_WAITTEMP,	// 1 byte: which channel; 4 bytes: lower limit; 4 bytes: upper limit [°C].  Reply (later): TEMPCB.  Disable with WAITTEMP (NAN, NAN).
	CMD_TEMP_VALUE,	// 1 byte: which channel.  Reply: TEMP. [°C]
	CMD_POWER_VALUE,	// 1 byte: which channel.  Reply: POWER. [μs, μs]
	CMD_SETPOS,	// 1 byte: which channel; 4 bytes: pos.
	CMD_GETPOS,	// 1 byte: which channel.  Reply: POS. [steps, mm]
	CMD_READ_GLOBALS,
	CMD_WRITE_GLOBALS,
	CMD_READ_SPACE_INFO,	// 1 byte: which channel.  Reply: DATA.
	CMD_READ_SPACE_AXIS,	// 1 byte: which channel.  Reply: DATA.
	CMD_READ_SPACE_MOTOR,	// 1 byte: which channel; n bytes: data.
	CMD_WRITE_SPACE_INFO,	// 1 byte: which channel.  Reply: DATA.
	CMD_WRITE_SPACE_AXIS,	// 1 byte: which channel; n bytes: data.
	CMD_WRITE_SPACE_MOTOR,	// 1 byte: which channel; n bytes: data.
	CMD_READ_TEMP,	// 1 byte: which channel.  Reply: DATA.
	CMD_WRITE_TEMP,	// 1 byte: which channel; n bytes: data.
	CMD_READ_GPIO,	// 1 byte: which channel.  Reply: DATA.
	CMD_WRITE_GPIO,	// 1 byte: which channel; n bytes: data.
	CMD_QUEUED,	// 1 byte: 0: query queue length; 1: stop and query queue length.  Reply: QUEUE.
	CMD_PIN_VALUE,	// 1 byte: which channel. Reply: GPIO.
	CMD_HOME,	// 1 byte: homing space; n bytes: homing type (0=pos, 1=neg, 3=no)
	CMD_FORCE_DISCONNECT,	// 0
	CMD_CONNECT,	// 8 byte: run ID, n bytes: port name (0-terminated)
	CMD_RECONNECT,	// n bytes: port name (0-terminated)
	CMD_RESUME,
	CMD_GET_TIME,
	CMD_SPI,
	CMD_ADJUST_PROBE,	// 3 doubles: probe position.
	CMD_TP_GETPOS,
	CMD_TP_SETPOS,	// 1 double: new toolpath position.
	CMD_TP_FINDPOS,	// 3 doubles: search position or NaN.
	CMD_MOTORS2XYZ,	// 1 byte: which space, n doubles: motor positions.  Reply: m times XYZ.
};

enum InterruptCommand {
	CMD_LIMIT,	// 1 byte: which channel.
	CMD_FILE_DONE,
	CMD_MOVECB,	// 1 byte: number of movecb events.
	CMD_HOMED,	// 0
	CMD_TIMEOUT,	// 0
	CMD_PINCHANGE,	// 1 byte: pin, 1 byte: current value.
	CMD_PINNAME,
	CMD_DISCONNECT,	// 0
	CMD_UPDATE_PIN,
	CMD_UPDATE_TEMP,
	CMD_CONFIRM,
	CMD_PARKWAIT,
	CMD_CONNECTED,
	CMD_TEMPCB,	// 1 byte: which channel.  Byte storage for which needs to be sent.
	CMD_CONTINUE,
};

enum RunType {
	RUN_SYSTEM,
	RUN_PRE_LINE,
	RUN_LINE,
	RUN_GOTO,
	RUN_GPIO,
	RUN_SETTEMP,
	RUN_WAITTEMP,
	RUN_SETPOS,
	RUN_WAIT,
	RUN_CONFIRM,
	RUN_PARK,
	RUN_PATTERN,
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

#ifdef MODULE

#include <string>

#define debug(...) do { fprintf(stderr, "$"); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); fflush(stderr); } while (0)

extern "C" {
	// Globals
	EXTERN double max_deviation;
	EXTERN double max_v, max_a;

	void parse_gcode(std::string const &infilename, std::string const &outfilename);
}

#endif

#endif

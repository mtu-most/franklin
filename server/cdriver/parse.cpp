/* parse.cpp - Parsing G-Code for Franklin.
 * Copyright 2017-2018 Bas Wijnen <wijnen@debian.org> {{{
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
 * }}} */

// Includes. {{{
#define EXTERN
#include "module.h"
#include <fstream>
#include <vector>
#include <list>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
// }}}

//#define pdebug(format, ...) debug("%4d " format, P1->gcode_line, ##__VA_ARGS__)
#define pdebug(...)

static double const C0 = 273.15; // 0 degrees celsius in kelvin.

// The class below only to hold its variables in a convenient way.  The constructor does all the work; once the object is constructed, it should be discarded.
// The parse_gcode function defined at the end is the only public part of this file and it does just that.
struct Parser { // {{{
	struct Chunk { // {{{
		char type;
		int code;
		double num;
	}; // }}}
	struct Record { // {{{
		bool arc;
		int tool;
		double x, y, z, a, b, c, f, e;
		double e0, a0, b0, c0;
		double center[3];
		double normal[3];
		double time;
		std::string pattern;
		Record(int gcode_line, bool arc_, int tl, double x_, double y_, double z_, double a_, double b_, double c_, double f_, double e_, double cx = NAN, double cy = NAN, double cz = NAN, double nx = NAN, double ny = NAN, double nz = NAN) : arc(arc_), tool(tl), x(x_), y(y_), z(z_), a(a_), b(b_), c(c_), f(f_), e(e_), gcode_line(gcode_line) {
			center[0] = cx;
			center[1] = cy;
			center[2] = cz;
			normal[0] = nx;
			normal[1] = ny;
			normal[2] = nz;
		}
		// Helper variables for computing the move.
		double s[3];	// Vector to this point.
		double length;	// Half of length of segment to this point.
		double unit[3];	// Unit vector along s.
		double from[3];	// Position halfway between previous point and here.
		double n[3];	// Vector normal to this and next segment.
		double nnAL[3], nnLK[3];	// Normal to segment and normal, for AL and LK.
		double theta;	// Angle between this and next segment.
		double x0;	// length of half curve.
		double v1;	// speed at corner.
		double dev;	// actual maximum deviation from line.
		double tf;	// time for curve.
		double Jg, Jh;	// Jerk along and perpendicular to segment.
		int gcode_line;
	}; // }}}
	// Variables. {{{
	std::ifstream infile;
	std::ofstream outfile;
	char type;
	int code;
	double num;
	char modetype;
	int modecode;
	double modenum;
	std::string message;
	std::vector <double> bbox;
	std::vector <std::string> strings;
	std::list <unsigned> stringmap;
	double unit;
	bool rel;
	bool erel;
	double spindle_speed;
	std::vector <double> pos;
	std::vector <double> epos;
	int current_tool;
	bool extruding;
	double current_f[2];
	bool tool_changed;
	std::list <Record> pending;
	int lineno;
	std::string line;
	unsigned linepos;
	double arc_normal[3];
	double last_time;
	std::list <Chunk> command;
	double max_dev;
	std::string pattern_data;
	// }}}
	// Member functions. {{{
	Parser(std::string const &infilename, std::string const &outfilename);
	void get_full_line();
	bool handle_command(bool handle_pattern);
	bool get_chunk(Chunk &ret, std::string &comment);
	int add_string(std::string const &str);
	void read_space();
	int read_int(double *power = NULL, int *sign = NULL);
	double read_fraction();
	void handle_coordinate(double value, int index, bool *controlled, bool rel);
	void flush_pending();
	void add_record(int gcode_line, RunType cmd, int tool = 0, double x = NAN, double y = NAN, double z = NAN, double hx = 0, double hy = 0, double hz = 0, double Jg = 0, double tf = NAN, double v0 = NAN, double e = NAN);
	void reset_pending_pos();
	// }}}
}; // }}}

void Parser::reset_pending_pos() { // {{{
	pending.pop_front();
	pending.push_front(Record(lineno, false, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], INFINITY, unsigned(current_tool) < epos.size() ? epos[current_tool] : NAN));
} // }}}

bool Parser::get_chunk(Chunk &ret, std::string &comment) { // {{{
	// Clear type so a premature return will also have an empty value.
	ret.type = 0;
	read_space();
	if (linepos >= line.size())
		return false;
	char t = line[linepos++];
	read_space();
	if (t == ';') {
		comment = line.substr(linepos);
		linepos = line.size();
		return false;
	}
	while (t == '(') {
		auto p = line.find(')', linepos);
		comment = line.substr(linepos, p - linepos);
		linepos = p + 1;
		read_space();
		t = line[linepos++];
		if (linepos >= line.size())
			return false;
	}
	read_space();
	ret.type = std::toupper(t);
	int sign;
	ret.code = read_int(NULL, &sign);
	ret.num = ret.code;
	// RepRap had made G-Code even worse by defining M117...
	if (ret.type == 'M' && ret.code == 117) {
		read_space();
		ret.type = 0;
		comment = "MSG," + line.substr(linepos);
		linepos = line.size();
		return false;
	}
	if (linepos >= line.size() || line[linepos] != '.')
		return true;
	linepos += 1;
	ret.num += sign * read_fraction();
	return true;
} // }}}

void Parser::handle_coordinate(double value, int index, bool *controlled, bool rel) { // {{{
	if (std::isnan(value))
		return;
	if (std::isnan(pos[index])) {
		*controlled = false;
		rel = false;
	}
	pos[index] = (rel ? pos[index] : 0) + value * unit;
} // }}}

bool Parser::handle_command(bool handle_pattern) { // {{{
	// M6 is "tool change"; record that it happened and turn it into G28: park. {{{
	if (type == 'M' && code == 6) {
		type = 'G';
		code = 28;
		tool_changed = true;
	} // }}}
	if (type == 'G') { // {{{
		switch (code) {
		case 1:
		case 81:
			modetype = type;
			modecode = code;
			modenum = num;
			// fall through.
		case 0:
			{
				double X = NAN, Y = NAN, Z = NAN, A = NAN, B = NAN, C = NAN, E = NAN, F = NAN, R = NAN;
				for (auto arg: command) {
					if (arg.type == 'X')
						X = arg.num;
					else if (arg.type == 'Y')
						Y = arg.num;
					else if (arg.type == 'Z')
						Z = arg.num;
					else if (arg.type == 'A')
						A = arg.num;
					else if (arg.type == 'B')
						B = arg.num;
					else if (arg.type == 'C')
						C = arg.num;
					else if (arg.type == 'E')
						E = arg.num;
					else if (arg.type == 'F')
						F = arg.num;
					else if (arg.type == 'R')
						R = arg.num;
					else
						debug("%d:ignoring invalid parameter %c: %s", lineno, arg.type, line.c_str());
				}
				if (!std::isnan(F)) {
					current_f[code == 0 ? 0 : 1] = F * unit / 60;
					//if (!(current_f[code == 0 ? 0 : 1] > 0))
						//debug("new f%d: %f", code == 0 ? 0 : 1, current_f[code == 0 ? 0 : 1]);
				}
				auto oldpos = pos;
				double estep, r;
				if (code != 81) {
					r = NAN;
					if (!std::isnan(E)) {
						if (erel)
							estep = E * unit;
						else
							estep = E * unit - epos[current_tool];
						epos[current_tool] += estep;
					}
					else
						estep = 0;
				}
				else {
					estep = 0;
					if (!std::isnan(R)) {
						if (rel)
							r = (std::isnan(pos[2]) ? 0 : pos[2]) + R * unit;
						else
							r = R * unit;
					}
					else
						r = (std::isnan(pos[2]) ? 0 : pos[2]);
				}
				if ((estep != 0) ^ extruding) {
					flush_pending();
					extruding = (estep != 0);
				}
				bool controlled = true;
				handle_coordinate(X, 0, &controlled, rel);
				handle_coordinate(Y, 1, &controlled, rel);
				handle_coordinate(Z, 2, &controlled, rel);
				handle_coordinate(A, 3, &controlled, rel);
				handle_coordinate(B, 4, &controlled, rel);
				handle_coordinate(C, 5, &controlled, rel);
				if (!controlled || (std::isnan(X) && std::isnan(Y) && std::isnan(Z))) {
					if (handle_pattern)
						debug("Warning: not handling pattern because position is unknown");
					flush_pending();
					add_record(lineno, RUN_GOTO, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], NAN, NAN, current_f[code == 0 ? 0 : 1], epos[current_tool]);
					reset_pending_pos();
					break;
				}
				if (code != 81) {
					pending.push_back(Record(lineno, false, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], current_f[code == 0 ? 0 : 1], epos[current_tool]));
					if (handle_pattern && code == 1)
						pending.back().pattern = pattern_data;
					pattern_data.clear();
				}
				else {
					if (std::isnan(oldpos[2]))
						oldpos[2] = r;
					flush_pending();
					// Only support OLD_Z (G90) retract mode; don't support repeats(L).
					// goto x, y
					pending.push_back(Record(lineno, false, current_tool, pos[0], pos[1], oldpos[2], NAN, NAN, NAN, INFINITY, NAN));
					// goto r
					pending.push_back(Record(lineno, false, current_tool, pos[0], pos[1], r, NAN, NAN, NAN, INFINITY, NAN));
					// goto z
					if (pos[2] != r) {
						pending.push_back(Record(lineno, false, current_tool, pos[0], pos[1], pos[2], NAN, NAN, NAN, current_f[1], NAN));
					}
					// go back to old z
					pending.push_back(Record(lineno, false, current_tool, pos[0], pos[1], oldpos[2], NAN, NAN, NAN, INFINITY, NAN));
					// update pos[2].
					pos[2] = oldpos[2];
				}
			}
			break;
		case 2:
		case 3:
			// TODO: This is broken, so it is disabled. It should be fixed and enabled.
			debug("Arc commands (G2/G3) are disabled at the moment");
			break;
			{
				modetype = type;
				modecode = code;
				modenum = num;
				double X = NAN, Y = NAN, Z = NAN, A = NAN, B = NAN, C = NAN, E = NAN, F = NAN, I = NAN, J = NAN, K = NAN;
				for (auto arg: command) {
					if (arg.type == 'X')
						X = arg.num;
					else if (arg.type == 'Y')
						Y = arg.num;
					else if (arg.type == 'Z')
						Z = arg.num;
					else if (arg.type == 'A')
						A = arg.num;
					else if (arg.type == 'B')
						B = arg.num;
					else if (arg.type == 'C')
						C = arg.num;
					else if (arg.type == 'E')
						E = arg.num;
					else if (arg.type == 'F')
						F = arg.num;
					else if (arg.type == 'I')
						I = arg.num;
					else if (arg.type == 'J')
						J = arg.num;
					else if (arg.type == 'K')
						K = arg.num;
					else
						debug("%d:ignoring invalid arc parameter %c: %s", lineno, arg.type, line.c_str());
				}
				if (!std::isnan(F)) {
					current_f[1] = F * unit / 60;
					//debug("new f1: %f", current_f[code]);
				}
				double estep;
				estep = std::isnan(E) ? 0 : erel ? E * unit - epos[current_tool] : E * unit;
				epos[current_tool] += estep;
				std::vector <double> center;
				center.push_back(0);
				center.push_back(0);
				center.push_back(0);
				bool controlled = true;
				handle_coordinate(X, 0, &controlled, rel);
				handle_coordinate(Y, 1, &controlled, rel);
				handle_coordinate(Z, 2, &controlled, rel);
				handle_coordinate(A, 3, &controlled, rel);
				handle_coordinate(B, 4, &controlled, rel);
				handle_coordinate(C, 5, &controlled, rel);
				center[0] = pos[0] + (std::isnan(I) ? 0 : I * unit);
				center[1] = pos[1] + (std::isnan(J) ? 0 : J * unit);
				center[2] = pos[2] + (std::isnan(K) ? 0 : K * unit);
				int s = code == 2 ? -1 : 1;
				pending.push_back(Record(lineno, true, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], -current_f[1], epos[current_tool], center[0], center[1], center[2], s * arc_normal[0], s * arc_normal[1], s * arc_normal[2]));
			}
			break;
		case 4:
			{
				double t = 0;
				for (auto arg: command) {
					if (arg.type == 'S')
						t = arg.num;
					else if (arg.type == 'P')
						t = arg.num / 1000;
				}
				flush_pending();
				add_record(lineno, RUN_WAIT, 0, t);
				last_time += t;
			}
			break;
		case 17:
			arc_normal[0] = 0;
			arc_normal[1] = 0;
			arc_normal[2] = 1;
			break;
		case 18:
			arc_normal[0] = 0;
			arc_normal[1] = 1;
			arc_normal[2] = 0;
			break;
		case 19:
			arc_normal[0] = 1;
			arc_normal[1] = 0;
			arc_normal[2] = 0;
			break;
		case 20:
			unit = 25.4;
			break;
		case 21:
			unit = 1;
			break;
		case 28:
			// Park.
			epos[current_tool] = 0;
			flush_pending();
			add_record(lineno, RUN_PARK);
			for (unsigned i = 0; i < pos.size(); ++i)
				pos[i] = NAN;
			reset_pending_pos();
			break;
		case 64:
		{
			flush_pending();
			bool specified = false;
			for (auto arg: command) {
				if (arg.type != 'P') {
					debug("ignoring %c argument for G64", arg.type);
					continue;
				}
				max_dev = arg.num;
				specified = true;
			}
			if (!specified)
				max_dev = max_deviation;
			break;
		}
		case 90:
			rel = false;
			erel = false;
			break;
		case 91:
			rel = true;
			erel = true;
			break;
		case 92:
			// Set position.  Only supported for extruders.
			for (auto arg: command) {
				if (arg.type != 'E') {
					debug("%d:Not setting position for unsupported type %c", lineno, arg.type);
					continue;
				}
				epos[current_tool] = arg.num;
				flush_pending();
				add_record(lineno, RUN_SETPOS, current_tool, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, arg.num);
				reset_pending_pos();
			}
			break;
		case 94:
			// Feedrate is in units per minute.  Nothing else is supported.
			break;
		default:
			// Command was not handled: complain.
			debug("%d:Invalid command %c.", type, code);
			break;
		}
	} // }}}
	else if (type == 'M') { // {{{
		int e = current_tool;
		double s = NAN;
		for (auto arg: command) {
			if (arg.type == 'E')
				e = arg.code;
			if (arg.type == 'S')
				s = arg.num;
		}
		switch (code) {
		case 0:
			// Request confirmation.
			flush_pending();
			add_record(lineno, RUN_CONFIRM, add_string(message), tool_changed ? 1 : 0);
			tool_changed = false;
			break;
		case 2:
			return false;
		case 3:
			// Spindle on, clockwise.
			flush_pending();
			add_record(lineno, RUN_GPIO, -3, std::isnan(s) ? spindle_speed : s);
			break;
		case 4:
			// Spindle on, counterclockwise.
			flush_pending();
			add_record(lineno, RUN_GPIO, -3, std::isnan(s) ? spindle_speed : s);
			break;
		case 5:
			// Spindle off.
			flush_pending();
			add_record(lineno, RUN_GPIO, -3, 0);
			break;
		case 9:
			// Coolant off.  Ignore.
			break;
		case 42:
			flush_pending();
			{
				// Initialize these to avoid warning; they always get new values if used.
				int p = 0;
				bool have_p = false;
				for (auto arg: command) {
					if (arg.type == 'P') {
						p = arg.code;
						have_p = true;
					}
				}
				if (!have_p || std::isnan(s)) {
					debug("%d:M42 needs both P and S arguments.", lineno);
					break;
				}
				add_record(lineno, RUN_GPIO, p, s);
			}
			break;
		case 82:
			erel = false;
			break;
		case 83:
			erel = true;
			break;
		case 84:
			// Stop idle hold on motors; ignore this command, G-Code should not control this.
			break;
		case 140:
			e = -1;
			// fall through.
		case 104:
			// Set extruder temperature.
			flush_pending();
			{
				double t = 0;
				bool have_t = false;
				for (auto arg: command) {
					if (arg.type == 'T') {
						t = arg.num;
						have_t = true;
					}
				}
				if (std::isnan(s)) {
					debug("%d:M104 needs S argument.", lineno);
					break;
				}
				add_record(lineno, RUN_SETTEMP, have_t ? t : e, s + C0);
			}
			break;
		case 106:
			// Fan on.
			flush_pending();
			add_record(lineno, RUN_GPIO, -2, std::isnan(s) ? 1 : s);
			break;
		case 107:
			// Fan off.
			flush_pending();
			add_record(lineno, RUN_GPIO, -2, 0);
			break;
		case 190:
			e = -1;
			// fall through.
		case 109:
			// Set extruder or bed temperature and wait for it to heat up.
			flush_pending();
			{
				double t = 0;
				bool have_t = false;
				for (auto arg: command) {
					if (arg.type == 'T') {
						t = arg.num;
						have_t = true;
					}
				}
				if (!have_t)
					t = e;
				if (!std::isnan(s))
					add_record(lineno, RUN_SETTEMP, t, s + C0);
				add_record(lineno, RUN_WAITTEMP, t);
			}
			break;
		case 116:
			// Wait for all temperatures to reach their target.
			flush_pending();
			add_record(lineno, RUN_WAITTEMP, -2);
			break;
		default:
			// Command was not handled: complain.
			debug("%d:Invalid command %c.", type, code);
			break;
		}
	} // }}}
	else if (type == 'D') { // {{{
		// TODO: Find out why this was in the other code; no D-commands are handled.
	} // }}}
	else if (type == 'T') { // {{{
		// New tool.
		while (code >= (signed)epos.size())
			epos.push_back(0);
		current_tool = code;
		// Make full stop between tools.
		flush_pending();
		// Apply new offset.
		pending.push_back(Record(lineno, false, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], INFINITY, epos[current_tool]));
	} // }}}
	else {
		// Command was not handled: complain.
		debug("%d:Invalid command %c.", type, code);
	}
	command.clear();
	return true;
} // }}}

int Parser::add_string(std::string const &str) { // {{{
	if (str.size() == 0)
		return 0;
	auto found = std::find(strings.begin(), strings.end(), str);
	if (found != strings.end())
		return found - strings.begin();
	strings.push_back(str);
	stringmap.push_back(str.size());
	return strings.size() - 1;
} // }}}

void Parser::read_space() { // {{{
	while (true) {
		if (linepos >= line.size())
			break;
		if (line[linepos] == ' ' || line[linepos] == '\t' || line[linepos] == '\r') {
			linepos += 1;
			continue;
		}
		break;
	}
} // }}}

int Parser::read_int(double *power, int *sign) { // {{{
	if (linepos >= line.size()) {
		debug("%d:int requested at end of line", lineno);
		return 0;
	}
	int s = 1;
	switch (line[linepos]) {
	case '-':
		s = -1;
		// Fall through.
	case '+':
		linepos += 1;
		if (linepos >= line.size()) {
			debug("%d:int requested at end of line", lineno);
			return 0;
		}
		break;
	default:
		break;
	}
	if (sign)
		*sign = s;
	read_space();
	int ret = 0;
	while (true) {
		if (linepos >= line.size())
			return s * ret;
		char c = line[linepos];
		if (c < '0' || c > '9')
			return s * ret;
		linepos += 1;
		int digit = c - '0';
		ret *= 10;
		ret += digit;
		if (power)
			*power *= 10;
	}
} // }}}

double Parser::read_fraction() { // {{{
	double power = 1;
	double n = read_int(&power);
	return n / power;
} // }}}

void Parser::get_full_line() { // {{{
	char buffer[200];
	line.clear();
	while (infile) {
		infile.getline(buffer, sizeof(buffer));
		line += buffer;
		if (infile.fail())
			continue;
		break;
	}
} // }}}

void decode_base64(std::string const &comment, int inpos, uint8_t *data, int outpos) { // {{{
	int value = 0;
	for (int i = 0; i < 4; ++i) {
		int digit;
		char c = comment[inpos + i];
		if (c >= 'A' && c <= 'Z')
			digit = c - 'A';
		else if (c >= 'a' && c <= 'z')
			digit = 26 + c - 'a';
		else if (c >= '0' && c <= '9')
			digit = 52 + c - '0';
		else if (c == '+')
			digit = 62;
		else if (c == '/')
			digit = 63;
		else
			digit = 0;
		value |= digit << (6 * (3 - i));
	}
	for (int i = 0; i < 3; ++i)
		data[outpos + i] = value >> (8 * (2 - i));
} // }}}

Parser::Parser(std::string const &infilename, std::string const &outfilename) // {{{
		: infile(infilename.c_str()), outfile(outfilename.c_str(), std::ios::binary) {
	strings.push_back("");
	modetype = 0;
	unit = 1;
	rel = false;
	erel = false;
	current_tool = 0;
	extruding = false;
	tool_changed = false;
	last_time = 0;
	arc_normal[0] = 0;
	arc_normal[1] = 0;
	arc_normal[2] = 1;
	epos.push_back(0);
	current_f[0] = INFINITY;
	current_f[1] = INFINITY;
	// Pending initial state contains the current position.
	pending.push_front(Record(lineno, false, 0, NAN, NAN, NAN, NAN, NAN, NAN, INFINITY, NAN));
	max_dev = max_deviation;
	for (int i = 0; i < 6; ++i) {
		pos.push_back(NAN);
		bbox.push_back(NAN);
	}
	lineno = 0;
	while (infile) {
		get_full_line();
		lineno += 1;
		linepos = 0;
		std::list <Chunk> chunks;
		Chunk chunk;
		std::string comment;
		while (true) {
			if (!get_chunk(chunk, comment))
				break;
			chunks.push_back(chunk);
		}
		if (comment.substr(0, 4) == "MSG,")
			message = comment.substr(4);
		if (comment.substr(0, 7) == "SYSTEM:") {
			int s = add_string(comment.substr(7));
			flush_pending();
			add_record(lineno, RUN_SYSTEM, s);
		}
		if (comment.substr(0, 4) == "PATTERN:") {
			// Decode base64 code for pattern.
			uint8_t data[2 * PATTERN_MAX];
			for (int i = 0; 4 * i + 3 < int(comment.size()) - 4 && 3 * i + 2 < 2 * PATTERN_MAX; i += 1) {
				// input = comment[4 + 4 * i:4 + 4 * (i + 1)]
				// output = data[3 * i:3 * (i + 1)]
				decode_base64(comment, 4 + 4 * i, data, 3 * i);
			}
			int size = ((comment.size() - 4) / 4) * 3;
			if (comment[comment.size() - 1] == '=') {
				if (comment[comment.size() - 2] == '=')
					size -= 2;
				else
					size -= 1;
			}
			pattern_data = std::string(reinterpret_cast <char *>(data), size);
			//debug("read pattern (%d=%d): %s", size, pattern_data.size(), pattern_data.c_str());
		}
		else
			pattern_data.clear();
		if (chunks.size() == 0)
			continue;
		if (chunks.front().type == 'N') {
			lineno = chunks.front().code;
			chunks.pop_front();
		}
		if (chunks.front().type == 'S') {
			// Spindle speed.
			spindle_speed = chunks.front().num;
			chunks.pop_front();
		}
		if (chunks.back().type == '*') {
			// Ignore checksums; the code isn't sent over an unreliable line and the checksum method is horrible anyway so if you need it, use something better.
			chunks.pop_back();
		}
		type = 0;
		bool stop = false;
		if (chunks.front().type == 'T') {
			type = 'T';
			code = chunks.front().code;
			num = chunks.front().num;
			chunks.pop_front();
		}
		while (chunks.size() > 0) {
			if (chunks.front().type == 'G' || chunks.front().type == 'M' || chunks.front().type == 'D') {
				if (type != 0) {
					if (!handle_command(false)) {
						stop = true;
						break;
					}
					type = 0;
				}
				type = chunks.front().type;
				code = chunks.front().code;
				num = chunks.front().num;
				chunks.pop_front();
				continue;
			}
			if (type == 0) {
				if (modetype == 0) {
					debug("%d:G-Code must have only G, M, T, S, or D-commands until first mode-command: %s", lineno, line.c_str());
					break;
				}
				type = modetype;
				code = modecode;
				num = modenum;
			}
			command.push_back(chunks.front());
			chunks.pop_front();
		}
		if (stop)
			break;
		if (type != 0) {
			if (!pattern_data.empty() && (type != 'G' || num != 1))
				debug("Warning: ignoring pattern comment because command is not G1");
			if (!handle_command(!pattern_data.empty()))
				break;
		}
	}
	// Write final data.
	flush_pending();
	// Strings.
	for (auto str: strings)
		outfile << str;
	// String lengths.
	for (auto str: stringmap) {
		uint32_t bin = str;
		outfile.write(reinterpret_cast <char *>(&bin), sizeof(bin));
	}
	// Number of strings.
	uint32_t strnum = strings.size() - 1;
	outfile.write(reinterpret_cast <char *>(&strnum), sizeof(strnum));
	// Bbox.
	for (double b: bbox)
		outfile.write(reinterpret_cast <char *>(&b), sizeof(b));
	// Time
	outfile.write(reinterpret_cast <char *>(&last_time), sizeof(last_time));
} // }}}

void Parser::flush_pending() { // {{{
	if (pending.size() <= 1)
		return;
	pending.push_back(pending.back());
	(--pending.end())->f = 0;
	pending.begin()->s[0] = 0;
	pending.begin()->s[1] = 0;
	pending.begin()->s[2] = 0;
	pending.begin()->e0 = 0;
	pending.begin()->a0 = 0;
	pending.begin()->b0 = 0;
	pending.begin()->c0 = 0;
	pending.begin()->length = 0;
	pending.begin()->unit[0] = 0;
	pending.begin()->unit[1] = 0;
	pending.begin()->unit[2] = 0;
	pending.begin()->f = 0;
	pending.begin()->v1 = 0;
	pending.push_front(pending.front());
	for (auto P0 = pending.begin(), P1 = ++pending.begin(); P1 != pending.end(); ++P0, ++P1) {
		P1->f = min(max_v, P1->f);
		P1->e0 = P0->e;
		P1->a0 = P0->a;
		P1->b0 = P0->b;
		P1->c0 = P0->c;
		P1->s[0] = P1->x - P0->x;
		P1->s[1] = P1->y - P0->y;
		P1->s[2] = P1->z - P0->z;
		P1->from[0] = P0->x + P1->s[0] * .5;
		P1->from[1] = P0->y + P1->s[1] * .5;
		P1->from[2] = P0->z + P1->s[2] * .5;
		P1->length = 0;
		for (int i = 0; i < 3; ++i) {
			if (std::isnan(P1->s[i]))
				P1->s[i] = 0; // If P0->x or P1->x is NaN, treat the length as zero.
			P1->length += P1->s[i] * P1->s[i];
		}
		P1->length = sqrt(P1->length);
		P0->n[0] = (P1->s[1] * P0->s[2]) - (P1->s[2] * P0->s[1]);
		P0->n[1] = (P1->s[2] * P0->s[0]) - (P1->s[0] * P0->s[2]);
		P0->n[2] = (P1->s[0] * P0->s[1]) - (P1->s[1] * P0->s[0]);
		P0->nnAL[0] = (P0->n[2] * P0->s[1]) - (P0->n[1] * P0->s[2]);
		P0->nnAL[1] = (P0->n[0] * P0->s[2]) - (P0->n[2] * P0->s[0]);
		P0->nnAL[2] = (P0->n[1] * P0->s[0]) - (P0->n[0] * P0->s[1]);
		P0->nnLK[0] = (P0->n[2] * P1->s[1]) - (P0->n[1] * P1->s[2]);
		P0->nnLK[1] = (P0->n[0] * P1->s[2]) - (P0->n[2] * P1->s[0]);
		P0->nnLK[2] = (P0->n[1] * P1->s[0]) - (P0->n[0] * P1->s[1]);
		double nnAL_length = sqrt(P0->nnAL[0] * P0->nnAL[0] + P0->nnAL[1] * P0->nnAL[1] + P0->nnAL[2] * P0->nnAL[2]);
		double nnLK_length = sqrt(P0->nnLK[0] * P0->nnLK[0] + P0->nnLK[1] * P0->nnLK[1] + P0->nnLK[2] * P0->nnLK[2]);
		for (int i = 0; i < 3; ++i) {
			P1->unit[i] = P1->length == 0 ? 0 : P1->s[i] / P1->length;
			if (std::isnan(P1->unit[i]))
				P1->unit[i] = 0;
			if (nnAL_length > 0)
				P0->nnAL[i] /= nnAL_length;
			else
				P0->nnAL[i] = 0;
			if (nnLK_length > 0)
				P0->nnLK[i] /= nnLK_length;
			else
				P0->nnLK[i] = 0;
		}
		P0->theta = acos(-P0->unit[0] * P1->unit[0] + -P0->unit[1] * P1->unit[1] + -P0->unit[2] * P1->unit[2]);
		if (P0->length < 1e-10 || P1->length < 1e-10 || std::isnan(P0->theta))
			P0->theta = 0;
		P1->length /= 2;
	}
	(--pending.end())->theta = 0;
	(--pending.end())->n[0] = 0;
	(--pending.end())->n[1] = 0;
	(--pending.end())->n[2] = 0;
	(--pending.end())->nnAL[0] = 0;
	(--pending.end())->nnAL[1] = 0;
	(--pending.end())->nnAL[2] = 0;
	(--pending.end())->nnLK[0] = 0;
	(--pending.end())->nnLK[1] = 0;
	(--pending.end())->nnLK[2] = 0;
	auto P1 = ++pending.begin();
	auto P2 = ++ ++pending.begin();
	while (P2 != pending.end()) {
		double s = -std::tan(P1->theta / 2);
		P1->dev = std::min(max_dev, std::min(P1->length, P2->length) / (3 * -(s + 1 / s) * std::sin(P1->theta / 2)));
		P1->x0 = 3 * P1->dev * (s + 1 / s) * std::sin(P1->theta / 2);
		if (std::isnan(s) || std::isnan(P1->dev) || std::isnan(P1->x0)) {
			s = 0;
			P1->dev = 0;
			P1->x0 = 0;
		}
		pdebug("s %f dev %f x0 %f", s, P1->dev, P1->x0);
		double sq = std::sqrt(s * s + 1);
		double max_v_J = std::pow(-max_J * P1->x0 * P1->x0 * sq / 2, 1. / 3);
		double max_v_a = std::sqrt(max_a * P1->x0 * sq / 2);
		P1->v1 = min(min(min(max_v_J, max_v_a), P1->f), P2->f);
		P1->tf = -P1->x0 / P1->v1;
		if (std::isnan(P1->tf))
			P1->tf = 0;
		P1->Jh = -2 * P1->v1 * P1->v1 * P1->v1 / ((s + 1 / s) * P1->x0 * P1->x0);
		if (std::isnan(P1->Jh))
			P1->Jh = 0;
		P1->Jg = P1->Jh / s;

		// Check if v0/v2 should be lowered.
		double v0 = compute_max_v(P1->length + P1->x0, P1->v1, max_J, max_a);
		double v2 = compute_max_v(P2->length + P1->x0, P1->v1, max_J, max_a);
		if (v2 < P2->f)
			P2->f = v2;
		if (v0 < P1->f) {
			P1->f = v0;
			--P1;
			--P2;
			continue;
		}
		//debug("next");
		++P1;
		++P2;
	}
	// Turn Records into Run_Records and write them out.  Ignore fake first and last record.
	while (pending.size() > 2) {
		//auto P0 = pending.begin(); TODO: use P0 and P1 instead of P1 and P2.
		P1 = ++pending.begin();
		P2 = ++ ++pending.begin();
		if (P1->arc) {
			abort();
			// TODO: support arcs.
			// TODO: Update time.
			// TODO: Update bbox.
		}
		else {
			// Not an arc.
			// TODO: support abc again.

			pdebug("Writing out segment from (%f,%f,%f) via (%f,%f,%f) to (%f,%f,%f)", P1->from[0], P1->from[1], P1->from[2], P1->x, P1->y, P1->z, P2->from[0], P2->from[1], P2->from[2]);

			double max_ramp_t = max_a / max_J;
			double max_ramp_dv = max_J / 2 * max_ramp_t * max_ramp_t;
			if (P1->f > 0) {
				double de = (P1->e - P1->e0) / 2;
				double total_dv = P1->f - P1->v1;
				// Compute s and t for all parts.
				bool have_max_a = total_dv > max_ramp_dv * 2;
				double s_const_v, s_ramp_start, s_const_a, s_ramp_end, s_curve;
				double t_const_v, t_ramp, t_const_a, t_curve;
				s_curve = -P1->x0;
				t_curve = P1->tf;
				if (have_max_a) {
					t_const_a = (total_dv - max_ramp_dv * 2) / max_a;
					s_const_a = max_a / 2 * t_const_a * t_const_a;
					t_ramp = max_ramp_t;
				}
				else {
					t_const_a = 0;
					s_const_a = 0;
					t_ramp = std::sqrt(total_dv / max_J);
				}
				double a_top = max_J * t_ramp;
				double t_ramp2 = t_ramp * t_ramp;
				double t_ramp3 = t_ramp2 * t_ramp;
				double dv_ramp = max_J / 2 * t_ramp2;
				s_ramp_start = P1->f * t_ramp - max_J / 6 * t_ramp3;
				s_ramp_end = (P1->v1 + dv_ramp) * t_ramp - a_top / 2 * t_ramp2 + max_J / 6 * t_ramp3;
				s_const_v = P1->length - s_curve - s_ramp_start - s_ramp_end - s_const_a;
				t_const_v = s_const_v / P1->f;

				pdebug("part 1 s,t: cv %f,%f rs %f,%f ca %f,%f re %f,%f curve %f,%f", s_const_v, t_const_v, s_ramp_start, t_ramp, s_const_a, t_const_a, s_ramp_end, t_ramp, s_curve, t_curve);
				if (s_const_v < -1e-2)
					abort();

				// constant v
				if (s_const_v > 1e-10) {
					double X[3];
					for (int i = 0; i < 3; ++i)
						X[i] = P1->from[i] + P1->unit[i] * s_const_v;
					add_record(P1->gcode_line, RUN_POLY3PLUS, P1->tool, X[0], X[1], X[2], 0, 0, 0, 0, t_const_v, P1->f, P1->e0 + de * s_const_v / P1->length);
					pdebug("1.const v to (%f,%f,%f) v=%f", X[0], X[1], X[2], P1->f);
				}
				if (P1->v1 != P1->f) {
					// start slowdown
					double X[3];
					for (int i = 0; i < 3; ++i)
						X[i] = P1->from[i] + P1->unit[i] * (s_const_v + s_ramp_start);
					add_record(P1->gcode_line, RUN_POLY3PLUS, P1->tool, X[0], X[1], X[2], 0, 0, 0, -max_J, t_ramp, P1->f, P1->e0 + de * (s_const_v + s_ramp_start) / P1->length);
					pdebug("1.start to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P1->f);
					// constant a slowdown
					if (have_max_a) {
						for (int i = 0; i < 3; ++i)
							X[i] = P1->from[i] + P1->unit[i] * (s_const_v + s_ramp_start + s_const_a);
						add_record(P1->gcode_line, RUN_POLY2, P1->tool, X[0], X[1], X[2], 0, 0, 0, -a_top, t_const_a, P1->f - dv_ramp, P1->e0 + de * (s_const_v + s_ramp_start + s_const_a) / P1->length);
						pdebug("1.const a to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P1->f - dv_ramp);
					}
					// stop slowdown
					for (int i = 0; i < 3; ++i)
						X[i] = P1->from[i] + P1->unit[i] * (P1->length - s_curve);
					add_record(P1->gcode_line, RUN_POLY3MINUS, P1->tool, X[0], X[1], X[2], 0, 0, 0, max_J, t_ramp, P1->v1, P1->e0 + de * (P1->length - s_curve) / P1->length);
					pdebug("1.stop- to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P1->v1);
				}
				// first half curve
				if (s_curve > 1e-10) {
					double h[3];
					for (int i = 0; i < 3; ++i)
						h[i] = P1->nnAL[i] * P1->Jh;
					add_record(P1->gcode_line, RUN_POLY3PLUS, P1->tool, P1->x, P1->y, P1->z, h[0], h[1], h[2], P1->Jg, t_curve, P1->v1, P1->e);
					pdebug("1.curve to (%f,%f,%f) v0=%f h = (%f,%f,%f)", P1->x, P1->y, P1->z, P1->v1, h[0], h[1], h[2]);
				}
			}

			if (P2->f > 0) {
				double de = P2->e - P2->e0;
				double total_dv = P2->f - P1->v1;
				// Compute s and t for all parts.
				bool have_max_a = total_dv > max_ramp_dv * 2;
				double s_const_v, s_ramp_start, s_const_a, s_ramp_end, s_curve;
				double t_const_v, t_ramp, t_const_a, t_curve;
				s_curve = -P1->x0;
				t_curve = P1->tf;
				if (have_max_a) {
					t_const_a = (total_dv - max_ramp_dv * 2) / max_a;
					s_const_a = max_a / 2 * t_const_a * t_const_a;
					t_ramp = max_ramp_t;
				}
				else {
					t_const_a = 0;
					s_const_a = 0;
					t_ramp = std::sqrt(total_dv / max_J);
				}
				double a_top = max_J * t_ramp;
				double t_ramp2 = t_ramp * t_ramp;
				double t_ramp3 = t_ramp2 * t_ramp;
				double dv_ramp = max_J / 2 * t_ramp2;
				s_ramp_start = P1->v1 * t_ramp + max_J / 6 * t_ramp3;
				s_ramp_end = (P2->f - dv_ramp) * t_ramp + a_top / 2 * t_ramp2 - max_J / 6 * t_ramp3;
				s_const_v = P2->length - s_curve - s_ramp_start - s_ramp_end - s_const_a;
				t_const_v = s_const_v / P2->f;

				pdebug("part 2 s,t: curve %f,%f rs %f,%f ca %f,%f re %f,%f cv %f,%f", s_curve, t_curve, s_ramp_start, t_ramp, s_const_a, t_const_a, s_ramp_end, t_ramp, s_const_v, t_const_v);
				if (s_const_v < -1e-2)
					abort();

				// second half curve
				if (s_curve > 1e-10) {
					double g[3], h[3];
					for (int i = 0; i < 3; ++i) {
						g[i] = P2->unit[i] * s_curve;
						h[i] = P1->nnLK[i] * P1->Jh;
					}
					add_record(P2->gcode_line, RUN_POLY3MINUS, P2->tool, P1->x + g[0], P1->y + g[1], P1->z + g[2], h[0], h[1], h[2], P1->Jg, t_curve, P1->v1, P2->e0 + de * s_curve / P2->length);
					pdebug("2.curve- to (%f,%f,%f) v=%f h (%f,%f,%f) nnLK.x=%f Jh=%f", P1->x + g[0], P1->y + g[1], P1->z + g[2], P1->v1, h[0], h[1], h[2], P1->nnLK[0], P1->Jh);
				}
				if (P1->v1 != P2->f) {
					// start speedup
					double X[3];
					for (int i = 0; i < 3; ++i)
						X[i] = P2->from[i] - P2->unit[i] * (s_const_v + s_ramp_end + s_const_a);
					add_record(P2->gcode_line, RUN_POLY3PLUS, P2->tool, X[0], X[1], X[2], 0, 0, 0, max_J, t_ramp, P1->v1, P2->e0 + de * (s_curve + s_ramp_start) / P2->length);
					pdebug("2.start to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P1->v1);
					// constant a speedup
					if (have_max_a) {
						for (int i = 0; i < 3; ++i)
							X[i] = P2->from[i] - P2->unit[i] * (s_const_v + s_ramp_end);
						add_record(P2->gcode_line, RUN_POLY2, P2->tool, X[0], X[1], X[2], 0, 0, 0, a_top, t_const_a, P1->v1 + dv_ramp, P2->e0 + de * (s_curve + s_ramp_start + s_const_a) / P2->length);
						pdebug("2.mid to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P1->v1 + dv_ramp);
					}
					// stop speedup
					for (int i = 0; i < 3; ++i)
						X[i] = P2->from[i] - P2->unit[i] * s_const_v;
					add_record(P2->gcode_line, RUN_POLY3MINUS, P2->tool, X[0], X[1], X[2], 0, 0, 0, -max_J, t_ramp, P2->f, P2->e - de * s_const_v / P2->length);
					pdebug("2.stop- to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P2->f);
				}
				// constant v
				if (s_const_v > 1e-10) {
					add_record(P2->gcode_line, RUN_POLY3PLUS, P1->tool, P2->from[0], P2->from[1], P2->from[2], 0, 0, 0, 0, t_const_v, P2->f, P2->e);
					pdebug("2.const v to (%f,%f,%f) v0=%f", P2->from[0], P2->from[1], P2->from[2], P2->f);
				}
			}

			// Update bbox.
			// Use "not larger" instead of "smaller" so NaNs are replaced.
			if (!std::isnan(P1->x)) {
				if (!(P1->x > bbox[0]))
					bbox[0] = P1->x;
				if (!(P1->x < bbox[1]))
					bbox[1] = P1->x;
			}
			if (!std::isnan(P1->y)) {
				if (!(P1->y > bbox[2]))
					bbox[2] = P1->y;
				if (!(P1->y < bbox[3]))
					bbox[3] = P1->y;
			}
			if (!std::isnan(P1->z)) {
				if (!(P1->z > bbox[4]))
					bbox[4] = P1->z;
				if (!(P1->z < bbox[5]))
					bbox[5] = P1->z;
			}
		}
		pending.pop_front();
	}
} // }}}

void Parser::add_record(int gcode_line, RunType cmd, int tool, double x, double y, double z, double hx, double hy, double hz, double Jg, double tf, double v0, double e) { // {{{
	Run_Record r;
	r.type = cmd;
	r.tool = tool;
	r.X[0] = x;
	r.X[1] = y;
	r.X[2] = z;
	r.h[0] = hx;
	r.h[1] = hy;
	r.h[2] = hz;
	r.Jg = Jg;
	r.tf = tf;
	r.v0 = v0;
	r.E = e;
	r.gcode_line = gcode_line;
	r.time = last_time;
	if (!std::isnan(tf))
		last_time += tf;
	outfile.write(reinterpret_cast <char *>(&r), sizeof(r));
} // }}}

void parse_gcode(std::string const &infilename, std::string const &outfilename) { // {{{
	// Create an instance of the class.  The constructor does all the work.
	Parser p(infilename, outfilename);
	// No other actions needed.
} // }}}
// vim: set foldmethod=marker :

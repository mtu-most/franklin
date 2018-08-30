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
// }}}

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
		double time, dist;
		Record(bool arc_, int tl, double x_, double y_, double z_, double a_, double b_, double c_, double f_, double e_, double cx = NAN, double cy = NAN, double cz = NAN, double nx = NAN, double ny = NAN, double nz = NAN) : arc(arc_), tool(tl), x(x_), y(y_), z(z_), a(a_), b(b_), c(c_), f(f_), e(e_) {
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
		double n[3];	// Vector normal to this and next segment.
		double nnAL[3], nnLK[3];	// Normal to segment and normal, for AL and LK.
		double theta;	// Angle between this and next segment.
		double v1;	// speed at corner.
		double k, C;
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
	std::vector <double> pos;
	std::vector <double> epos;
	int current_tool;
	double current_f[2];
	bool tool_changed;
	std::list <Record> pending;
	int lineno;
	std::string line;
	unsigned linepos;
	double arc_normal[3];
	double last_time, last_dist;
	std::list <Chunk> command;
	double max_dev;
	// }}}
	// Member functions. {{{
	Parser(std::string const &infilename, std::string const &outfilename);
	void get_full_line();
	bool handle_command();
	bool get_chunk(Chunk &ret, std::string &comment);
	int add_string(std::string const &str);
	void read_space();
	int read_int(double *power = NULL, int *sign = NULL);
	double read_fraction();
	void flush_pending();
	void add_curve(Record *P, double *start, double *end, double *B, double v0, double v1, double f, double r);
	void add_record(RunType cmd, int tool = 0, double x = NAN, double y = NAN, double z = NAN, double Bx = 0, double By = 0, double Bz = 0, double e = NAN, double v0 = INFINITY, double v1 = INFINITY, double radius = INFINITY);
	// }}}
}; // }}}

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
		comment = line.substr(linepos, p - 1);
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

bool Parser::handle_command() { // {{{
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
					if (!(current_f[code == 0 ? 0 : 1] > 0))
						debug("new f%d: %f", code == 0 ? 0 : 1, current_f[code == 0 ? 0 : 1]);
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
				if (!std::isnan(X)) pos[0] = (rel && !std::isnan(pos[0]) ? pos[0] : 0) + X * unit;
				if (!std::isnan(Y)) pos[1] = (rel && !std::isnan(pos[1]) ? pos[1] : 0) + Y * unit;
				if (!std::isnan(Z)) pos[2] = (rel && !std::isnan(pos[2]) ? pos[2] : 0) + Z * unit;
				if (!std::isnan(A)) pos[3] = (rel && !std::isnan(pos[3]) ? pos[3] : 0) + A * unit;
				if (!std::isnan(B)) pos[4] = (rel && !std::isnan(pos[4]) ? pos[4] : 0) + B * unit;
				if (!std::isnan(C)) pos[5] = (rel && !std::isnan(pos[5]) ? pos[5] : 0) + C * unit;
				if (code != 81) {
					double dist = 0;
					for (int i = 0; i < 3; ++i)
						dist += (pos[i] - oldpos[i]) * (pos[i] - oldpos[i]);
					dist = sqrt(dist);
					if (std::isnan(dist))
						dist = 0;
					pending.push_back(Record(false, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], dist > 0 ? current_f[code == 0 ? 0 : 1] / dist : INFINITY, epos[current_tool]));
				}
				else {
					if (std::isnan(oldpos[2]))
						oldpos[2] = r;
					flush_pending();
					// Only support OLD_Z (G90) retract mode; don't support repeats(L).
					// goto x, y
					pending.push_back(Record(false, current_tool, pos[0], pos[1], oldpos[2], NAN, NAN, NAN, INFINITY, NAN));
					double dist = sqrt((pos[0] - oldpos[0]) * (pos[0] - oldpos[0]) + (pos[1] - oldpos[1]) * (pos[1] - oldpos[1]));
					if (!std::isnan(dist))
						last_dist += dist;
					// goto r
					pending.push_back(Record(false, current_tool, pos[0], pos[1], r, NAN, NAN, NAN, INFINITY, NAN));
					dist = abs(r - oldpos[2]);
					if (!std::isnan(dist))
						last_dist += dist;
					// goto z
					if (pos[2] != r) {
						dist = abs(pos[2] - r);
						double f = dist > 0 ? current_f[1] / dist : INFINITY;
						pending.push_back(Record(false, current_tool, pos[0], pos[1], pos[2], NAN, NAN, NAN, f, NAN));
						if (std::isinf(f)) {
							if (!std::isnan(dist))
								last_dist += dist;
						}
						else {
							if (!std::isnan(f))
								last_time += 1 / f;
						}
					}
					// go back to old z
					pending.push_back(Record(false, current_tool, pos[0], pos[1], oldpos[2], NAN, NAN, NAN, INFINITY, NAN));
					dist = abs(r - oldpos[2]);
					if (!std::isnan(dist))
						last_dist += dist;
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
				if (!std::isnan(X)) pos[0] = (rel ? pos[0] : 0) + X * unit;
				if (!std::isnan(Y)) pos[1] = (rel ? pos[1] : 0) + Y * unit;
				if (!std::isnan(Z)) pos[2] = (rel ? pos[2] : 0) + Z * unit;
				if (!std::isnan(A)) pos[3] = (rel ? pos[3] : 0) + A * unit;
				if (!std::isnan(B)) pos[4] = (rel ? pos[4] : 0) + B * unit;
				if (!std::isnan(C)) pos[5] = (rel ? pos[5] : 0) + C * unit;
				center[0] = pos[0] + (std::isnan(I) ? 0 : I * unit);
				center[1] = pos[1] + (std::isnan(J) ? 0 : J * unit);
				center[2] = pos[2] + (std::isnan(K) ? 0 : K * unit);
				int s = code == 2 ? -1 : 1;
				pending.push_back(Record(true, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], -current_f[1], epos[current_tool], center[0], center[1], center[2], s * arc_normal[0], s * arc_normal[1], s * arc_normal[2]));
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
				add_record(RUN_WAIT, 0, t);
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
			add_record(RUN_PARK);
			for (unsigned i = 0; i < pos.size(); ++i)
				pos[i] = NAN;
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
				add_record(RUN_SETPOS, current_tool, arg.num);
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
		for (auto arg: command) {
			if (arg.type == 'E') {
				e = arg.code;
				break;
			}
		}
		switch (code) {
		case 0:
			// Request confirmation.
			flush_pending();
			add_record(RUN_CONFIRM, add_string(message), tool_changed ? 1 : 0);
			tool_changed = false;
			break;
		case 2:
			return false;
		case 3:
			// Spindle on, clockwise.
			flush_pending();
			add_record(RUN_GPIO, -3, 1);
			break;
		case 4:
			// Spindle on, counterclockwise.
			flush_pending();
			add_record(RUN_GPIO, -3, 1);
			break;
		case 5:
			// Spindle off.
			flush_pending();
			add_record(RUN_GPIO, -3, 0);
			break;
		case 9:
			// Coolant off.  Ignore.
			break;
		case 42:
			flush_pending();
			{
				// Initialize these to avoid warning; they always get new values if used.
				int p = 0;
				double s = NAN;
				bool have_p = false, have_s = false;
				for (auto arg: command) {
					if (arg.type == 'P') {
						p = arg.code;
						have_p = true;
					}
					if (arg.type == 'S') {
						s = arg.num;
						have_s = true;
					}
				}
				if (!have_p || !have_s) {
					debug("%d:M42 needs both P and S arguments.", lineno);
					break;
				}
				add_record(RUN_GPIO, p, s);
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
				double s = 0, t = 0;
				bool have_s = false, have_t = false;
				for (auto arg: command) {
					if (arg.type == 'S') {
						s = arg.num;
						have_s = true;
					}
					if (arg.type == 'T') {
						t = arg.num;
						have_t = true;
					}
				}
				if (!have_s) {
					debug("%d:M104 needs S argument.", lineno);
					break;
				}
				add_record(RUN_SETTEMP, have_t ? t : e, s + C0);
			}
			break;
		case 106:
			// Fan on.
			flush_pending();
			add_record(RUN_GPIO, -2, 1);
			break;
		case 107:
			// Fan off.
			flush_pending();
			add_record(RUN_GPIO, -2, 0);
			break;
		case 190:
			e = -1;
			// fall through.
		case 109:
			// Set extruder or bed temperature and wait for it to heat up.
			flush_pending();
			{
				double s = 0, t = 0;
				bool have_s = false, have_t = false;
				for (auto arg: command) {
					if (arg.type == 'S') {
						s = arg.num;
						have_s = true;
					}
					if (arg.type == 'T') {
						t = arg.num;
						have_t = true;
					}
				}
				if (!have_t)
					t = e;
				if (have_s)
					add_record(RUN_SETTEMP, t, s + C0);
				add_record(RUN_WAITTEMP, t);
			}
			break;
		case 116:
			// Wait for all temperatures to reach their target.
			flush_pending();
			add_record(RUN_WAITTEMP, -2);
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
		// Move to current position for extruder offset correction.
		flush_pending();
		pending.push_back(Record(false, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], INFINITY, epos[current_tool]));
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
	if (line[linepos] == '-') {
		s = -1;
		linepos += 1;
		if (linepos >= line.size()) {
			debug("%d:int requested at end of line", lineno);
			return 0;
		}
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

Parser::Parser(std::string const &infilename, std::string const &outfilename) // {{{
		: infile(infilename.c_str()), outfile(outfilename.c_str(), std::ios::binary) {
	strings.push_back("");
	modetype = 0;
	unit = 1;
	rel = false;
	erel = false;
	current_tool = 0;
	tool_changed = false;
	last_time = 0;
	last_dist = 0;
	arc_normal[0] = 0;
	arc_normal[1] = 0;
	arc_normal[2] = 1;
	epos.push_back(0);
	current_f[0] = INFINITY;
	current_f[1] = INFINITY;
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
			add_record(RUN_SYSTEM, s);
		}
		if (chunks.size() == 0)
			continue;
		if (chunks.front().type == 'N') {
			lineno = chunks.front().code;
			chunks.pop_front();
		}
		if (chunks.front().type == 'S') {
			// Spindle speed. Ignore.
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
					if (!handle_command()) {
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
		if (type != 0)
			if (!handle_command())
				break;
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
	// Time+dist
	outfile.write(reinterpret_cast <char *>(&last_time), sizeof(last_time));
	outfile.write(reinterpret_cast <char *>(&last_dist), sizeof(last_dist));
} // }}}

void Parser::flush_pending() { // {{{
	if (pending.size() == 0)
		return;
	pending.push_front(pending.front());
	pending.push_back(pending.back());
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
	int n = 0;
	for (auto P0 = pending.begin(), P1 = ++pending.begin(); P1 != pending.end(); ++P0, ++P1) {
		P1->f = min(max_v, P1->f);
		P1->e0 = P0->e;
		P1->a0 = P0->a;
		P1->b0 = P0->b;
		P1->c0 = P0->c;
		P1->s[0] = P1->x - P0->x;
		P1->s[1] = P1->y - P0->y;
		P1->s[2] = P1->z - P0->z;
		P1->length = sqrt(P1->s[0] * P1->s[0] + P1->s[1] * P1->s[1] + P1->s[2] * P1->s[2]);
		P0->n[0] = (P1->s[1] * P0->s[2]) - (P1->s[2] * P0->s[1]);
		P0->n[1] = (P1->s[2] * P0->s[0]) - (P1->s[0] * P0->s[2]);
		P0->n[2] = (P1->s[0] * P0->s[1]) - (P1->s[1] * P0->s[0]);
		P0->nnAL[0] = (P0->n[1] * P0->s[2]) - (P0->n[2] * P0->s[1]);
		P0->nnAL[1] = (P0->n[2] * P0->s[0]) - (P0->n[0] * P0->s[2]);
		P0->nnAL[2] = (P0->n[0] * P0->s[1]) - (P0->n[1] * P0->s[0]);
		P0->nnLK[0] = (P0->n[1] * P1->s[2]) - (P0->n[2] * P1->s[1]);
		P0->nnLK[1] = (P0->n[2] * P1->s[0]) - (P0->n[0] * P1->s[2]);
		P0->nnLK[2] = (P0->n[0] * P1->s[1]) - (P0->n[1] * P1->s[0]);
		double nnAL_length = sqrt(P0->nnAL[0] * P0->nnAL[0] + P0->nnAL[1] * P0->nnAL[1] + P0->nnAL[2] * P0->nnAL[2]);
		double nnAK_length = sqrt(P0->nnLK[0] * P0->nnLK[0] + P0->nnLK[1] * P0->nnLK[1] + P0->nnLK[2] * P0->nnLK[2]);
		for (int i = 0; i < 3; ++i) {
			P1->unit[i] = P1->length == 0 ? 0 : P1->s[i] / P1->length;
			if (nnAL_length > 0)
				P0->nnAL[i] /= nnAL_length;
			else
				P0->nnAL[i] = 0;
			if (nnAK_length > 0)
				P0->nnLK[i] /= nnAK_length;
			else
				P0->nnLK[i] = 0;
		}
		P0->theta = acos(-P0->unit[0] * P1->unit[0] + -P0->unit[1] * P1->unit[1] + -P0->unit[2] * P1->unit[2]);
		if (P0->length < 1e-10 || P1->length < 1e-10 || std::isnan(P0->theta))
			P0->theta = 0;
		P1->length /= 2;
		P0->k = (1 - std::sin(P0->theta / 2)) / (1 + std::sin(P0->theta / 2));
		P0->C = 2 * sqrt(P0->k) + 2 * sqrt(P0->k - P0->k * P0->k / 4);
		n += 1;
	}
	(--pending.end())->theta = 0;
	(--pending.end())->k = 1;
	(--pending.end())->C = 1 + sqrt(3) / 2;
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
	n = 1;
	while (P2 != pending.end()) {
		if (std::isnan(P1->C)) {
			debug("skipping segment (length %f), because C is NaN", P1->length);
			n += 1;
			++P1;
			++P2;
			continue;
		}
		double v1A1 = sqrt((2 * max_a * P1->length + P1->f * P1->f) / (2 * P1->C + 1));
		double v1K1 = sqrt((2 * max_a * P2->length + P2->f * P2->f) / (2 * P1->C + 1));
		double v1A0 = sqrt((2 * max_a * P1->length - P1->f * P1->f) / (2 * P1->C - 1));
		double v1K0 = sqrt((2 * max_a * P2->length - P2->f * P2->f) / (2 * P1->C - 1));
		if (P1->C > 0.5) {
			if (std::isnan(v1K0)) {
				P2->f = sqrt(2 * max_a * P2->length) * (1 - 1e-10);
				debug("sharp same %f", P1->f);
				continue;
			}
			if (std::isnan(v1A0)) {
				debug("sharp prev");
				P1->f = sqrt(2 * max_a * P1->length) * (1 - 1e-10);
				n -= 1;
				--P1;
				--P2;
				continue;
			}
			P1->v1 = min(min(min(max(P1->f, P2->f), max_a * max_dev / P1->k), v1A1 > P1->f ? v1A1 : v1A0), v1K1 > P2->f ? v1K1 : v1K0);
		}
		else {
			if (v1K1 < P2->f) {
				debug("dull same");
				P2->f = sqrt(max_a * P2->length / P1->C) * (1 - 1e-10);
				continue;
			}
			if (v1A1 < P1->f) {
				P1->f = sqrt(max_a * P1->length / P1->C) * (1 - 1e-10);
				debug("dull prev %f", v1A1);
				n -= 1;
				--P1;
				--P2;
				continue;
			}
			double vmax = min(min(min(max(P1->f, P2->f), max_a * max_dev / P1->k), v1A1), v1K1);
			if (vmax < v1K0) {
				debug("dull same 2");
				P2->f = sqrt(2 * max_a * P2->length - (2 * P1->C - 1) * vmax * vmax) * (1 - 1e-10);
				continue;
			}
			if (vmax < v1A0) {
				debug("dull prev 2");
				P1->f = sqrt(2 * max_a * P1->length - (2 * P1->C - 1) * vmax * vmax) * (1 - 1e-10);
				n -= 1;
				--P1;
				--P2;
				continue;
			}
			P1->v1 = vmax;
		}
		debug("next");
		n += 1;
		++P1;
		++P2;
	}
	// Turn Records into Run_Records and write them out.  Ignore fake first and last record.
	Record P0 = pending.front();
	pending.pop_front();
	while (!pending.empty()) {
		P1 = pending.begin();
		if (P1->arc) {
			// TODO: support arcs.
			// TODO: Update time+dist.
			// TODO: Update bbox.
		}
		else {
			// Not an arc.
			// TODO: support abc again.
			//if (((!std::isnan(old_p.a) || !std::isnan(p.a)) && old_p.a != p.a) || ((!std::isnan(old_p.b) || !std::isnan(p.b)) && old_p.b != p.b) || ((!std::isnan(old_p.c) || !std::isnan(p.c)) && old_p.c != p.c))
			//	add_record(RUN_PRE_LINE, p.tool, p.a, p.b, p.c);
			double r = P0.v1 * P0.v1 / max_a;
			double d = r * P0.k;
			double ANL = P0.length;
			double CNL = P0.C * r;
			double AC = ANL - CNL;
			double BC = abs(P0.v1 * P0.v1 - P0.f * P0.f) / (2 * max_a * max_a);
			debug("AC %f BC %f ANL %f CNL %f v1 %f r %f f %f max a %f", AC, BC, ANL, CNL, P0.v1, r, P0.f, max_a);
			double AB = AC - BC;
			double CD = acos((r - d / 2) / r) * r;
			double DEF = (M_PI / 2 - P0.theta / 2) * r + CD;
			double AEF = AC + CD + DEF;

			double LK = P1->length;
			double LI = P0.C / max_a * P0.v1 * P0.v1;
			double IK = LK - LI;
			double IJ = abs(P0.v1 * P0.v1 - P1->f * P1->f) / (2 * max_a * max_a);
			double JK = IK - IJ;
			double FGH = DEF;
			double FGK = FGH + CD + IK;

			double A[3], B[3], C[3], D[3], E[3], F[3], G[3], H[3], I[3], J[3], K[3], M[3], N[3], N2[3], DE[3], DF[3], FH[3], GH[3], M_DE[3], M_DF[3], M_FH[3], M_GH[3];
			double X0[3] = { P0.x - P0.s[0], P0.y - P0.s[1], P0.z - P0.s[2] };
			double X1[3] = { P0.x, P0.y, P0.z };
			double X2[3] = { P1->x, P1->y, P1->z };
			double l_MDE = 0, l_MDF = 0, l_MFH = 0, l_MGH = 0;
			for (int i = 0; i < 3; ++i) {
				A[i] = (X0[i] + X1[i]) / 2;
				B[i] = A[i] + P0.unit[i] * AB;
				C[i] = A[i] + P0.unit[i] * AC;
				N[i] = X1[i] - P0.unit[i] * std::cos(P0.theta / 2) * (r + d);
				M[i] = N[i] - P0.nnAL[i] * (r - d);
				E[i] = M[i] + P0.nnAL[i] * r;
				D[i] = (C[i] + E[i]) / 2;
				F[i] = M[i] + (r + d < 1e-10 ? 0 : (X1[i] - M[i]) * r / (r + d));

				K[i] = (X1[i] + X2[i]) / 2;
				J[i] = K[i] - P1->unit[i] * JK;
				I[i] = K[i] - P1->unit[i] * IK;
				N2[i] = X1[i] + P1->unit[i] * std::cos(P0.theta / 2) * (r + d);
				G[i] = N2[i] + P0.nnLK[i] * d;
				H[i] = (G[i] + I[i]) / 2;

				// Use DE and GE, not CD and HI, because they are easier to compute.
				// The resulting B-vector is the same length and opposite direction of what is needed.
				DE[i] = (D[i] + E[i]) / 2;
				DF[i] = (D[i] + F[i]) / 2;
				FH[i] = (F[i] + H[i]) / 2;
				GH[i] = (G[i] + H[i]) / 2;

				M_DE[i] = DE[i] - M[i];
				M_DF[i] = DF[i] - M[i];
				M_FH[i] = FH[i] - M[i];
				M_GH[i] = GH[i] - M[i];
				l_MDE += M_DE[i] * M_DE[i];
				l_MDF += M_DF[i] * M_DF[i];
				l_MFH += M_FH[i] * M_FH[i];
				l_MGH += M_GH[i] * M_GH[i];
			}
			double Ba_[3], Ba[3], Bb[3], Bb_[3];
			for (int i = 0; i < 3; ++i) {
				M_DE[i] /= sqrt(l_MDE);
				M_DF[i] /= sqrt(l_MDF);
				M_FH[i] /= sqrt(l_MFH);
				M_GH[i] /= sqrt(l_MGH);
				Ba_[i] = -(M[i] + r * M_DE[i] - DE[i]);
				Ba[i] = M[i] + r * M_DF[i] - DF[i];
				Bb[i] = M[i] + r * M_FH[i] - FH[i];
				Bb_[i] = -(M[i] + r * M_GH[i] - GH[i]);
			}
			debug("v %f %f %f", P0.f, P0.v1, P1->f);
			debug("C %f AB %f k %f theta %f", P0.C, AB, P0.k, P0.theta * 180 / M_PI);
			debug("nnAL %f %f %f", P0.nnAL[0], P0.nnAL[1], P0.nnAL[2]);
			debug("nnLK %f %f %f", P0.nnLK[0], P0.nnLK[1], P0.nnLK[2]);
			debug("unit0 %f %f %f", P0.unit[0], P0.unit[1], P0.unit[2]);
			debug("unit1 %f %f %f", P1->unit[0], P1->unit[1], P1->unit[2]);
			debug("A %f %f %f", A[0], A[1], A[2]);
			debug("B %f %f %f", B[0], B[1], B[2]);
			debug("C %f %f %f", C[0], C[1], C[2]);
			debug("D %f %f %f", D[0], D[1], D[2]);
			debug("E %f %f %f", E[0], E[1], E[2]);
			debug("F %f %f %f", F[0], F[1], F[2]);
			debug("M %f %f %f", M[0], M[1], M[2]);
			add_curve(&P0, A, B, NULL, P0.f, P0.f, AB / AEF, INFINITY);
			add_curve(&P0, B, C, NULL, P0.f, P0.v1, AC / AEF, INFINITY);
			add_curve(&P0, C, D, Ba_, P0.v1, P0.v1, (AC + CD) / AEF, -r);
			add_curve(&P0, D, F, Ba, P0.v1, P0.v1, 1, r);
			add_curve(&*P1, F, H, Bb, P0.v1, P0.v1, FGH / FGK, r);
			add_curve(&*P1, H, I, Bb_, P0.v1, P0.v1, (FGH + CD) / FGK, -r);
			add_curve(&*P1, I, J, NULL, P0.v1, P1->f, (FGH + CD + IJ) / FGK, INFINITY);
			add_curve(&*P1, J, K, NULL, P1->f, P1->f, .5, INFINITY);
			// Update time+dist.
			double dist = 0;
			double dists[3] = {P1->x - P0.x, P1->y - P0.y, P1->z - P0.z};
			for (int i = 0; i < 3; ++i) {
				double dst = dists[i] * dists[i];
				if (!std::isnan(dst))
					dist += dst;
			}
			dist = sqrt(dist);
			if (dist > 0) {
				if (std::isinf(P1->f)) {
					if (!std::isnan(dist))
						last_dist += dist;
				}
				else {
					double t = 1 / P1->f;
					if (std::isinf(t))
						debug("%d:wtf dist %f x %f y %f f %f", lineno, dist, P1->x, P1->y, P1->f);
					if (!std::isnan(t))
						last_time += t;
				}
			}
			// Update bbox.
			// Use "not larger" instead of "smaller" so NaNs are replaced.
			if (!(P1->x > bbox[0]))
				bbox[0] = P1->x;
			if (!(P1->x < bbox[1]))
				bbox[1] = P1->x;
			if (!(P1->y > bbox[2]))
				bbox[2] = P1->y;
			if (!(P1->y < bbox[3]))
				bbox[3] = P1->y;
			if (!(P1->z > bbox[4]))
				bbox[4] = P1->z;
			if (!(P1->z < bbox[5]))
				bbox[5] = P1->z;
		}
		P0 = *P1;
		pending.pop_front();
	}
} // }}}

void Parser::add_curve(Record *P, double *start, double *end, double *B, double v0, double v1, double f, double r) { // {{{
	if (f < 1e-10)
		return;
	/* TODO: Support abc
	if (!std::isnan(P->a) || !std::isnan(P->b) || !std::isnan(P->c))
		add_record(RUN_PRELINE, P->tool, P->a, P->b, P->c, 0, 0, 0, 0, vi, vf);
	*/
	double dist = 0;
	for (int i = 0; i < 3; ++i)
		dist += (start[i] - end[i]) * (start[i] - end[i]);
	if (dist < 1e-10)
		return;
	add_record(RUN_LINE, P->tool, end[0], end[1], end[2], B ? B[0] : 0, B ? B[1] : 0, B ? B[2] : 0, P->e0 + f * (P->e - P->e0), v0, v1, (P->n[2] < 0 ? -1 : 1) * r);
} // }}}

void Parser::add_record(RunType cmd, int tool, double x, double y, double z, double Bx, double By, double Bz, double e, double v0, double v1, double radius) { // {{{
	Run_Record r;
	r.type = cmd;
	r.tool = tool;
	r.X = x;
	r.Y = y;
	r.Z = z;
	r.Bx = Bx;
	r.By = By;
	r.Bz = Bz;
	r.E = e;
	r.v0 = v0;
	r.v1 = v1;
	r.time = last_time;
	r.dist = last_dist;
	r.r = radius;
	outfile.write(reinterpret_cast <char *>(&r), sizeof(r));
} // }}}

void parse_gcode(std::string const &infilename, std::string const &outfilename) {
	// Create an instance of the class.  The constructor does all the work.
	Parser p(infilename, outfilename);
	// No other actions needed.
}
// vim: set foldmethod=marker :

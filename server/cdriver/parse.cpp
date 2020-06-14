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
#include <map>
#include <list>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
// }}}

//#define pdebug(format, ...) debug("%4d " format, P1->gcode_line, ##__VA_ARGS__)
#ifndef pdebug
#define pdebug(...)
#endif

//#define flushdebug debug
#ifndef flushdebug
#define flushdebug(...)
#endif

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
	void *errors;
	char type;
	int code;
	double num;
	char modetype;
	int modecode;
	double modenum;
	std::string message;
	double bbox[6];
	std::vector <std::string> strings;
	std::list <unsigned> stringmap;
	double unit;
	bool rel;
	bool erel;
	double spindle_speed;
	double pos[6];
	std::map <int, double> epos;
	int max_epos;
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
	Parser(std::string const &infilename, std::string const &outfilename, void *errors);
	void get_full_line();
	bool handle_command(bool handle_pattern);
	bool get_chunk(Chunk &ret, std::string &comment);
	int add_string(std::string const &str);
	void read_space();
	int read_int(double *power = NULL, int *sign = NULL);
	double read_fraction();
	void handle_coordinate(double value, int index, bool *controlled, bool rel);
	void flush_pending(bool finish = true);
	void add_record(int gcode_line, RunType cmd, int tool = 0, double x = NAN, double y = NAN, double z = NAN, double hx = 0, double hy = 0, double hz = 0, double Jg = 0, double tf = NAN, double v0 = NAN, double e = NAN);
	void reset_pending_pos();
	void initialize_pending_front();
	// }}}
}; // }}}

void Parser::reset_pending_pos() { // {{{
	auto &f = pending.front();
	f.gcode_line = lineno;
	f.arc = false;
	f.tool = current_tool;
	f.x = pos[0];
	f.y = pos[1];
	f.z = pos[2];
	f.a = pos[3];
	f.b = pos[4];
	f.c = pos[5];
	f.e = epos.at(current_tool);
	initialize_pending_front();
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
	if (rel)
		pos[index] += value * unit;
	else {
		pos[index] = extruder_data[current_tool].offset[index] + value * unit;
	}
} // }}}

static double inner(double a[3], double b[3]) { // {{{
	double ret = 0;
	for (int i = 0; i < 3; ++i)
		ret += a[i] * b[i];
	return ret;
} // }}}

static void cross(double ret[3], double a[3], double b[3]) { // {{{
	for (int i = 0; i < 3; ++i) {
		int i1 = (i + 1) % 3;
		int i2 = (i + 2) % 3;
		ret[i] = a[i1] * b[i2] - a[i2] * b[i1];
	}
} // }}}

bool Parser::handle_command(bool handle_pattern) { // {{{
	// M6 is "tool change"; record that it happened and turn it into G28: park. {{{
	if (type == 'M' && code == 6) {
		type = 'G';
		code = 28;
		tool_changed = true;
	} // }}}
	//debug("handling %c%d", type, code);
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
						parse_error(errors, "%d: ignoring invalid parameter %c: %s", lineno, arg.type, line.c_str());
				}
				if (!std::isnan(F)) {
					current_f[code == 0 ? 0 : 1] = F * unit / 60;
					//if (!(current_f[code == 0 ? 0 : 1] > 0))
						//debug("new f%d: %f", code == 0 ? 0 : 1, current_f[code == 0 ? 0 : 1]);
				}
				double oldpos[6] = {pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]};
				double estep, r;
				if (code != 81) {
					r = NAN;
					if (!std::isnan(E)) {
						if (erel) {
							estep = E * unit;
							epos.at(current_tool) += estep;
						}
						else {
							estep = E * unit - epos.at(current_tool);
							epos.at(current_tool) = E * unit;
						}
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
					flushdebug("flushing because E start/stop in line");
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
						parse_error(errors, "Warning: not handling pattern because position is unknown", lineno);
					flushdebug("flushing because position is unknown or non-position move");
					flush_pending();
					add_record(lineno, RUN_GOTO, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], NAN, NAN, current_f[code == 0 ? 0 : 1], epos.at(current_tool));
					reset_pending_pos();
					break;
				}
				if (code != 81) {
					pending.push_back(Record(lineno, false, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], current_f[code == 0 ? 0 : 1], epos.at(current_tool)));
					// If length is enough, flush partial queue.
					// Compute length for coming to a stop from v=J/2*t**2
					double t_stop = std::sqrt(2 * current_f[code == 0 ? 0 : 1] / max_J);
					double s_stop = max_J / 6 * t_stop * t_stop * t_stop;
					double dist = 0;
					for (int i = 0; i < 3; ++i) {
						if (!std::isnan(pos[i]) && !std::isnan(oldpos[i]))
							dist += (pos[i] - oldpos[i]) * (pos[i] - oldpos[i]);
					}
					dist = std::sqrt(dist);
					if (dist == 0) {
						flushdebug("flushing because dist == 0");
						flush_pending();
					}
					else if (s_stop < dist / 20) {
						flushdebug("partial flush because dist is long");
						flush_pending(false);
					}
					if (handle_pattern && code == 1)
						pending.back().pattern = pattern_data;
					pattern_data.clear();
				}
				else {
					if (std::isnan(oldpos[2]))
						oldpos[2] = r;
					flushdebug("flushing for G81");
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
			{
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
					else if (arg.type == 'I')
						I = arg.num;
					else if (arg.type == 'J')
						J = arg.num;
					else if (arg.type == 'L')
						K = arg.num;
					else if (arg.type == 'E')
						E = arg.num;
					else if (arg.type == 'F')
						F = arg.num;
					else
						parse_error(errors, "%d: ignoring invalid arc parameter %c: %s", lineno, arg.type, line.c_str());
				}
				if (!std::isnan(F)) {
					current_f[1] = F * unit / 60;
					//if (!(current_f[code == 0 ? 0 : 1] > 0))
						//debug("new f%d: %f", code == 0 ? 0 : 1, current_f[code == 0 ? 0 : 1]);
				}
				double oldpos[6] = {pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]};
				double estep;
				if (!std::isnan(E)) {
					if (erel) {
						estep = E * unit;
						epos.at(current_tool) += estep;
					}
					else {
						estep = E * unit - epos.at(current_tool);
						epos.at(current_tool) = E * unit;
					}
				}
				else
					estep = 0;
				if ((estep != 0) ^ extruding) {
					flushdebug("flushing because E start/stop in arc");
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
				double center[3];
				if (std::isnan(I))
					I = 0;
				center[0] = oldpos[0] + I;
				if (std::isnan(J))
					J = 0;
				center[1] = oldpos[1] + J;
				if (std::isnan(K))
					K = 0;
				center[2] = oldpos[2] + K;
				double arm[2][3];
				for (int i = 0; i < 3; ++i) {
					arm[0][i] = oldpos[i] - center[i];
					arm[1][i] = pos[i] - center[i];
				}
				double cross_arm[3];
				cross(cross_arm, arm[0], arm[1]);
				double sign = inner(cross_arm, arc_normal);
				double len_arm[2], z_arm[2], r_arm[2];
				for (int a = 0; a < 2; ++a) {
					len_arm[a] = std::sqrt(inner(arm[a], arm[a]));
					z_arm[a] = inner(arc_normal, arm[a]);
					r_arm[a] = std::sqrt(len_arm[a] * len_arm[a] - z_arm[a] * z_arm[a]);
				}
				double angle = std::acos(inner(arm[0], arm[1]) / (len_arm[0] * len_arm[1]));
				if (sign < 0)
					angle *= -1;
				if (arm[0][0] == arm[1][0] && arm[0][1] == arm[1][1] && arm[0][2] == arm[1][2])
					angle = (code == 2 ? -1 : 1) * 2 * M_PI;
				else {
					if (angle < 0 && code == 3)
						angle += 2 * M_PI;
					if (angle > 0 && code == 2)
						angle -= 2 * M_PI;
				}
				double start[3];
				for (int i = 0; i < 3; ++i)
					start[i] = arm[0][i] / len_arm[0];
				int num = fabs(angle) * (r_arm[0] + r_arm[1]) / 2 + 1;
				for (int t = 0; t < num; ++t) {
					double factor = (t + 1.) / num;
					double z = z_arm[0] + factor * (z_arm[1] - z_arm[0]);
					double r = r_arm[0] + factor * (r_arm[1] - r_arm[0]);
					double ang = factor * angle;
					// Rotate start[] around arc_normal[] over ang.
					double c = std::cos(ang);
					double s = std::sin(ang);
					double target_arm[3];
					target_arm[0] = (c + arc_normal[0] * arc_normal[0] * (1 - c)) * start[0] +
						(arc_normal[0] * arc_normal[1] * (1 - c) - arc_normal[2] * s) * start[1] +
						(arc_normal[0] * arc_normal[2] * (1 - c) + arc_normal[1] * s) * start[2];
					target_arm[1] = (arc_normal[1] * arc_normal[0] * (1 - c) + arc_normal[2] * s) * start[0] +
						(c + arc_normal[1] * arc_normal[1] * (1 - c)) * start[1] +
						(arc_normal[1] * arc_normal[2] * (1 - c) - arc_normal[0] * s) * start[2];
					target_arm[2] = (arc_normal[2] * arc_normal[0] * (1 - c) - arc_normal[1] * s) * start[0] +
						(arc_normal[2] * arc_normal[1] * (1 - c) + arc_normal[0] * s) * start[1] +
						(c + arc_normal[2] * arc_normal[2] * (1 - c)) * start[2];
					double target[3];
					for (int i = 0; i < 3; ++i)
						target[i] = center[i] + z * arc_normal[i] + r * target_arm[i];
					//debug("arc part %f,%f,%f", target[0], target[1], target[2]);
					pending.push_back(Record(lineno, false, current_tool, target[0], target[1], target[2], NAN, NAN, NAN, current_f[1], NAN));
				}
			}
			// TODO: This is broken, so it is disabled. It should be fixed and enabled.
			//parse_error(errors, "Arc commands (G2/G3) are disabled at the moment", lineno);
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
						parse_error(errors, "%d: ignoring invalid arc parameter %c: %s", lineno, arg.type, line.c_str());
				}
				if (!std::isnan(F)) {
					current_f[1] = F * unit / 60;
					//debug("new f1: %f", current_f[code]);
				}
				double estep;
				estep = std::isnan(E) ? 0 : erel ? E * unit - epos.at(current_tool) : E * unit;
				epos.at(current_tool) += estep;
				double center[3];
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
				pending.push_back(Record(lineno, true, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], -current_f[1], epos.at(current_tool), center[0], center[1], center[2], s * arc_normal[0], s * arc_normal[1], s * arc_normal[2]));
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
				flushdebug("flushing for G4");
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
			epos.at(current_tool) = 0;
			flushdebug("flushing for G28");
			flush_pending();
			add_record(lineno, RUN_PARK);
			for (unsigned i = 0; i < 6; ++i)
				pos[i] = NAN;
			reset_pending_pos();
			break;
		case 64:
		{
			flushdebug("flushing for G64 (max dev)");
			flush_pending();
			bool specified = false;
			for (auto arg: command) {
				if (arg.type != 'P') {
					parse_error(errors, "%d: ignoring %c argument for G64", lineno, arg.type);
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
					parse_error(errors, "%d: Not setting position for unsupported type %c", lineno, arg.type);
					continue;
				}
				auto &cepos = epos.at(current_tool);
				if (cepos != arg.num) {
					epos.at(current_tool) = arg.num;
					flushdebug("flushing for G92 (set pos)");
					flush_pending();
					add_record(lineno, RUN_SETPOS, current_tool, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, arg.num);
					reset_pending_pos();
				}
			}
			break;
		case 94:
			// Feedrate is in units per minute.  Nothing else is supported.
			break;
		default:
			// Command was not handled: complain.
			parse_error(errors, "%d: Invalid command %c%d.", lineno, type, code);
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
			flushdebug("flushing for M0");
			flush_pending();
			add_record(lineno, RUN_CONFIRM, add_string(message), tool_changed ? 1 : 0);
			tool_changed = false;
			break;
		case 2:
			return false;
		case 3:
			// Spindle on, clockwise.
			flushdebug("flushing for M2/M3");
			flush_pending();
			add_record(lineno, RUN_GPIO, -3, std::isnan(s) ? spindle_speed : s);
			break;
		case 4:
			// Spindle on, counterclockwise.
			flushdebug("flushing for M4");
			flush_pending();
			add_record(lineno, RUN_GPIO, -3, std::isnan(s) ? spindle_speed : s);
			break;
		case 5:
			// Spindle off.
			flushdebug("flushing for M5");
			flush_pending();
			add_record(lineno, RUN_GPIO, -3, 0);
			break;
		case 9:
			// Coolant off.  Ignore.
			break;
		case 42:
			flushdebug("flushing for M42");
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
					parse_error(errors, "M42 needs both P and S arguments.", lineno);
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
			flushdebug("flushing for M104");
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
					parse_error(errors, "M104 needs S argument.", lineno);
					break;
				}
				add_record(lineno, RUN_SETTEMP, have_t ? t : e, s + C0);
			}
			break;
		case 106:
			// Fan on.
			flushdebug("flushing for M106");
			flush_pending();
			add_record(lineno, RUN_GPIO, -2, std::isnan(s) ? 1 : s);
			break;
		case 107:
			// Fan off.
			flushdebug("flushing for M107");
			flush_pending();
			add_record(lineno, RUN_GPIO, -2, 0);
			break;
		case 190:
			e = -1;
			// fall through.
		case 109:
			// Set extruder or bed temperature and wait for it to heat up.
			flushdebug("flushing for M109");
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
			flushdebug("flushing for M116");
			flush_pending();
			add_record(lineno, RUN_WAITTEMP, -2);
			break;
		default:
			// Command was not handled: complain.
			parse_error(errors, "%d: Invalid command %c%d.", lineno, type, code);
			break;
		}
	} // }}}
	else if (type == 'D') { // {{{
		// TODO: Find out why this was in the other code; no D-commands are handled.
	} // }}}
	else if (type == 'T') { // {{{
		// New tool.
		if (epos.find(code) == epos.end()) {
			epos.insert(std::pair <int, double>(code, 0));
			if (code >= max_epos)
				max_epos = code + 1;
		}
		current_tool = code;
		// Make full stop between tools.
		flushdebug("flushing for tool change");
		flush_pending();
		// Apply new offset.
		pending.push_back(Record(lineno, false, current_tool, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], INFINITY, epos.at(current_tool)));
	} // }}}
	else {
		// Command was not handled: complain.
		parse_error(errors, "%d: Invalid command %c%d.", lineno, type, code);
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
		parse_error(errors, "int requested at end of line", lineno);
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
			parse_error(errors, "int requested at end of line", lineno);
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

void Parser::initialize_pending_front() { // {{{
	auto f = pending.begin();
	f->s[0] = 0;
	f->s[1] = 0;
	f->s[2] = 0;
	f->e0 = 0;
	f->a0 = 0;
	f->b0 = 0;
	f->c0 = 0;
	f->length = 0;
	f->unit[0] = 0;
	f->unit[1] = 0;
	f->unit[2] = 0;
	f->f = 0;
	f->v1 = 0;
} // }}}

Parser::Parser(std::string const &infilename, std::string const &outfilename, void *errors) // {{{
		: infile(infilename.c_str()), outfile(outfilename.c_str(), std::ios::binary), errors(errors) {
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
	epos.insert(std::pair <int, double>(0, 0));
	current_f[0] = INFINITY;
	current_f[1] = INFINITY;
	// Pending initial state contains the current position.
	pending.push_front(Record(lineno, false, 0, NAN, NAN, NAN, NAN, NAN, NAN, INFINITY, NAN));
	initialize_pending_front();
	max_dev = max_deviation;
	for (int i = 0; i < 6; ++i) {
		pos[i] = NAN;
		bbox[i] = NAN;
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
			flushdebug("flushing for system");
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
					parse_error(errors, "%d: G-Code must have only G, M, T, S, or D-commands until first mode-command: %s", lineno, line.c_str());
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
				parse_error(errors, "Warning: ignoring pattern comment because command is not G1", lineno);
			if (!handle_command(!pattern_data.empty()))
				break;
		}
	}
	// Write final data.
	flushdebug("flushing for EOF");
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
	for (int i = 0; i < 6; ++i)
		outfile.write(reinterpret_cast <char *>(&bbox[i]), sizeof(double));
	// Time
	outfile.write(reinterpret_cast <char *>(&last_time), sizeof(last_time));
} // }}}

void Parser::flush_pending(bool finish) { // {{{
	flushdebug("flushing size %d finish %d", pending.size(), finish);
	if (finish) {
		pending.push_back(pending.back());
		auto &f = pending.back();
		f.f = 0;
		f.theta = 0;
		for (int i = 0; i < 3; ++i) {
			f.n[i] = 0;
			f.nnAL[i] = 0;
			f.nnLK[i] = 0;
		}
	}
	else if (pending.size() <= 1)
		return;
	// Compute static information about path. {{{
	for (auto P0 = pending.begin(), P1 = ++pending.begin(); P1 != pending.end(); ++P0, ++P1) {
		P0->f = min(max_v, P0->f);
		P1->f = min(max_v, P1->f);
		P1->e0 = (P0->e + P1->e) / 2;
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
	} // }}}
	// Compute dynamic information about path, back tracking when needed. {{{
	auto P0 = pending.begin();
	auto P1 = ++pending.begin();
	while (P1 != pending.end()) {
		double s = -std::tan(P0->theta / 2);
		P0->dev = std::min(max_dev, std::min(P0->length, P1->length) / (3 * -(s + 1 / s) * std::sin(P0->theta / 2)));
		P0->x0 = 3 * P0->dev * (s + 1 / s) * std::sin(P0->theta / 2);
		if (std::isnan(s) || std::isnan(P0->dev) || std::isnan(P0->x0)) {
			s = 0;
			P0->dev = 0;
			P0->x0 = 0;
		}
		pdebug("s %f dev %f x0 %f", s, P0->dev, P0->x0);
		double sq = std::sqrt(s * s + 1);
		double max_v_J = std::pow(max_J * P0->x0 * P0->x0 * sq / 2, 1. / 3);
		double max_v_a = std::sqrt(max_a * -P0->x0 * sq / 2);
		if (fabs(s) < 1e-10) {
			pdebug("setting v1 = 0 because s < epsilon");
			P0->v1 = 0;
		}
		else
			P0->v1 = min(min(min(max_v_J, max_v_a), P0->f), P1->f);
		//debug("setting v1 for %f,%f,%f to %f using x0=%f sq=%f f0=%f f1=%f max_J=%f max_a=%f", P0->x, P0->y, P0->z, P0->v1, P0->x0, sq, P0->f, P1->f, max_J, max_a);
		P0->tf = -P0->x0 / P0->v1;
		if (std::isnan(P0->tf))
			P0->tf = 0;
		P0->Jh = -2 * P0->v1 * P0->v1 * P0->v1 / ((s + 1 / s) * P0->x0 * P0->x0);
		if (std::isnan(P0->Jh))
			P0->Jh = 0;
		P0->Jg = P0->Jh / s;

		// Check if v0/v2 should be lowered.
		double v0 = compute_max_v(P0->length + P0->x0, P0->v1, max_J, max_a);
		double v2 = compute_max_v(P1->length + P0->x0, P0->v1, max_J, max_a);
		pdebug("check v2 f %f l %f x0 %f v1 %f v2 %f", P1->f, P1->length, P0->x0, P0->v1, v2);
		if (v2 < P1->f) {
			pdebug("limit v2 from %f to %f", P1->f, v2);
			P1->f = v2;
		}
		if (v0 < P0->f) {
			if (P0 != pending.begin()) {
				pdebug("back tracking set v0 from %f to %f", P0->f, v0);
				P0->f = v0;
				--P0;
				--P1;
				continue;
			}
			debug("Error: need to go back beyond start; ignoring.");
		}
		//debug("next");
		++P0;
		++P1;
	} // }}}
	// Turn Records into Run_Records and write them out.  Don't write last record. {{{
	while (pending.size() > 1) {
		P0 = pending.begin();
		P1 = ++pending.begin();
		if (P0->arc) {
			debug("arcs are currently not supported");
			abort();
			// TODO: support arcs.
			// TODO: Update time.
			// TODO: Update bbox.
		}
		else {
			// Not an arc.
			// TODO: support abc again.

			//debug("Writing out segment from (%f,%f,%f)@%f via (%f,%f,%f)@%f to (%f,%f,%f)@%f", P0->from[0], P0->from[1], P0->from[2], P0->f, P0->x, P0->y, P0->z, P0->v1, P1->from[0], P1->from[1], P1->from[2], P1->f);

			double max_ramp_t = max_a / max_J;
			double max_ramp_dv = max_J / 2 * max_ramp_t * max_ramp_t;
			if (P0->f > 0 || P0->v1 > 0) {
				double de = P0->e - P0->e0;
				double total_dv = P0->f - P0->v1;
				// Compute s and t for all parts.
				bool have_max_a = total_dv > max_ramp_dv * 2;
				double s_const_v, s_ramp_start, s_const_a, s_ramp_end, s_curve;
				double t_const_v, t_ramp, t_const_a, t_curve;
				s_curve = -P0->x0;
				t_curve = P0->tf;
				if (have_max_a) {
					t_const_a = (total_dv - max_ramp_dv * 2) / max_a;
					s_const_a = -max_a / 2 * t_const_a * t_const_a + (P0->f - max_ramp_dv) * t_const_a;
					t_ramp = max_ramp_t;
				}
				else {
					t_const_a = 0;
					s_const_a = 0;
					t_ramp = std::sqrt(total_dv / max_J);
				}
				double a_top = -max_J * t_ramp;
				double t_ramp2 = t_ramp * t_ramp;
				double t_ramp3 = t_ramp2 * t_ramp;
				double dv_ramp = max_J / 2 * t_ramp2;
				s_ramp_start = P0->f * t_ramp - max_J / 6 * t_ramp3;
				s_ramp_end = (P0->v1 + dv_ramp) * t_ramp + a_top / 2 * t_ramp2 + max_J / 6 * t_ramp3;
				s_const_v = P0->length - s_curve - s_ramp_start - s_ramp_end - s_const_a;
				t_const_v = s_const_v / P0->f;

				pdebug("part 1 s,t: cv %f,%f rs %f,%f ca %f,%f re %f,%f curve %f,%f", s_const_v, t_const_v, s_ramp_start, t_ramp, s_const_a, t_const_a, s_ramp_end, t_ramp, s_curve, t_curve);
				if (s_const_v < -1e-2) {
					parse_error(errors, "%d: Error: const v part is negative: %f", P0->gcode_line, s_const_v);
					//abort();
				}

				// constant v
				if (s_const_v > 1e-10) {
					double X[3];
					for (int i = 0; i < 3; ++i)
						X[i] = P0->from[i] + P0->unit[i] * s_const_v;
					add_record(P0->gcode_line, RUN_POLY3PLUS, P0->tool, X[0], X[1], X[2], 0, 0, 0, 0, t_const_v, P0->f, P0->e0 + de * s_const_v / P0->length);
					pdebug("1.const v to (%f,%f,%f) v=%f", X[0], X[1], X[2], P0->f);
				}
				if (P0->v1 != P0->f) {
					// start slowdown
					double X[3];
					for (int i = 0; i < 3; ++i)
						X[i] = P0->from[i] + P0->unit[i] * (s_const_v + s_ramp_start);
					add_record(P0->gcode_line, RUN_POLY3PLUS, P0->tool, X[0], X[1], X[2], 0, 0, 0, -max_J, t_ramp, P0->f, P0->e0 + de * (s_const_v + s_ramp_start) / P0->length);
					pdebug("1.start to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P0->f);
					// constant a slowdown
					if (have_max_a) {
						for (int i = 0; i < 3; ++i)
							X[i] = P0->from[i] + P0->unit[i] * (s_const_v + s_ramp_start + s_const_a);
						add_record(P0->gcode_line, RUN_POLY2, P0->tool, X[0], X[1], X[2], 0, 0, 0, a_top, t_const_a, P0->f - dv_ramp, P0->e0 + de * (s_const_v + s_ramp_start + s_const_a) / P0->length);
						pdebug("1.const a to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P0->f - dv_ramp);
					}
					// stop slowdown
					for (int i = 0; i < 3; ++i)
						X[i] = P0->from[i] + P0->unit[i] * (P0->length - s_curve);
					add_record(P0->gcode_line, RUN_POLY3MINUS, P0->tool, X[0], X[1], X[2], 0, 0, 0, max_J, t_ramp, P0->v1, P0->e0 + de * (P0->length - s_curve) / P0->length);
					pdebug("1.stop- to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P0->v1);
				}
				// first half curve
				if (s_curve > 1e-10) {
					double h[3];
					for (int i = 0; i < 3; ++i)
						h[i] = P0->nnAL[i] * P0->Jh;
					add_record(P0->gcode_line, RUN_POLY3PLUS, P0->tool, P0->x, P0->y, P0->z, h[0], h[1], h[2], P0->Jg, t_curve, P0->v1, P0->e);
					pdebug("1.curve to (%f,%f,%f) v0=%f h = (%f,%f,%f)", P0->x, P0->y, P0->z, P0->v1, h[0], h[1], h[2]);
				}
			}

			if (P1->f > 0 || P0->v1 > 0) {
				double de = P1->e0 - P0->e;
				double total_dv = P1->f - P0->v1;
				// Compute s and t for all parts.
				bool have_max_a = total_dv > max_ramp_dv * 2;
				double s_const_v, s_ramp_start, s_const_a, s_ramp_end, s_curve;
				double t_const_v, t_ramp, t_const_a, t_curve;
				s_curve = -P0->x0;
				t_curve = P0->tf;
				if (have_max_a) {
					t_const_a = (total_dv - max_ramp_dv * 2) / max_a;
					s_const_a = max_a / 2 * t_const_a * t_const_a + (P0->v1 + max_ramp_dv) * t_const_a;
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
				s_ramp_start = P0->v1 * t_ramp + max_J / 6 * t_ramp3;
				s_ramp_end = (P1->f - dv_ramp) * t_ramp + a_top / 2 * t_ramp2 - max_J / 6 * t_ramp3;
				s_const_v = P1->length - s_curve - s_ramp_start - s_ramp_end - s_const_a;
				t_const_v = s_const_v / P1->f;

				pdebug("part 2 s,t: curve %f,%f rs %f,%f ca %f,%f re %f,%f cv %f,%f", s_curve, t_curve, s_ramp_start, t_ramp, s_const_a, t_const_a, s_ramp_end, t_ramp, s_const_v, t_const_v);
				if (s_const_v < -1e-2) {
					parse_error(errors, "%d: Error: const v2 is negative: %f, len %f %f", P0->gcode_line, s_const_v, P0->length, P1->length);
					pdebug("part 2 s,t: curve %f,%f rs %f,%f ca %f,%f re %f,%f cv %f,%f", s_curve, t_curve, s_ramp_start, t_ramp, s_const_a, t_const_a, s_ramp_end, t_ramp, s_const_v, t_const_v);
					//abort();
				}

				// second half curve
				if (s_curve > 1e-10) {
					double g[3], h[3];
					for (int i = 0; i < 3; ++i) {
						g[i] = P1->unit[i] * s_curve;
						h[i] = P0->nnLK[i] * P0->Jh;
					}
					add_record(P1->gcode_line, RUN_POLY3MINUS, P1->tool, P0->x + g[0], P0->y + g[1], P0->z + g[2], h[0], h[1], h[2], P0->Jg, t_curve, P0->v1, P0->e + de * s_curve / P1->length);
					pdebug("2.curve- to (%f,%f,%f) v=%f h (%f,%f,%f) nnLK.x=%f Jh=%f", P0->x + g[0], P0->y + g[1], P0->z + g[2], P0->v1, h[0], h[1], h[2], P0->nnLK[0], P0->Jh);
				}
				if (P0->v1 != P1->f) {
					// start speedup
					double X[3];
					for (int i = 0; i < 3; ++i)
						X[i] = P1->from[i] - P1->unit[i] * (s_const_v + s_ramp_end + s_const_a);
					add_record(P1->gcode_line, RUN_POLY3PLUS, P1->tool, X[0], X[1], X[2], 0, 0, 0, max_J, t_ramp, P0->v1, P0->e + de * (s_curve + s_ramp_start) / P1->length);
					pdebug("2.start to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P0->v1);
					// constant a speedup
					if (have_max_a) {
						for (int i = 0; i < 3; ++i)
							X[i] = P1->from[i] - P1->unit[i] * (s_const_v + s_ramp_end);
						add_record(P1->gcode_line, RUN_POLY2, P1->tool, X[0], X[1], X[2], 0, 0, 0, a_top, t_const_a, P0->v1 + dv_ramp, P0->e + de * (s_curve + s_ramp_start + s_const_a) / P1->length);
						pdebug("2.mid to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P0->v1 + dv_ramp);
					}
					// stop speedup
					for (int i = 0; i < 3; ++i)
						X[i] = P1->from[i] - P1->unit[i] * s_const_v;
					add_record(P1->gcode_line, RUN_POLY3MINUS, P1->tool, X[0], X[1], X[2], 0, 0, 0, -max_J, t_ramp, P1->f, P1->e0 - de * s_const_v / P1->length);
					pdebug("2.stop- to (%f,%f,%f) v0=%f", X[0], X[1], X[2], P1->f);
				}
				// constant v
				if (s_const_v > 1e-10) {
					add_record(P1->gcode_line, RUN_POLY3PLUS, P0->tool, P1->from[0], P1->from[1], P1->from[2], 0, 0, 0, 0, t_const_v, P1->f, P1->e0);
					pdebug("2.const v to (%f,%f,%f) v0=%f", P1->from[0], P1->from[1], P1->from[2], P1->f);
				}
			}

			// Update bbox.
			// Use "not larger" instead of "smaller" so NaNs are replaced.
			if (!std::isnan(P0->x)) {
				if (!(P0->x > bbox[0]))
					bbox[0] = P0->x;
				if (!(P0->x < bbox[1]))
					bbox[1] = P0->x;
			}
			if (!std::isnan(P0->y)) {
				if (!(P0->y > bbox[2]))
					bbox[2] = P0->y;
				if (!(P0->y < bbox[3]))
					bbox[3] = P0->y;
			}
			if (!std::isnan(P0->z)) {
				if (!(P0->z > bbox[4]))
					bbox[4] = P0->z;
				if (!(P0->z < bbox[5]))
					bbox[5] = P0->z;
			}
		}
		pending.pop_front();
	} // }}}
	if (finish)
		initialize_pending_front();
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

void parse_gcode(std::string const &infilename, std::string const &outfilename, void *errors) { // {{{
	// Create an instance of the class.  The constructor does all the work.
	Parser p(infilename, outfilename, errors);
	// No other actions needed.
} // }}}
// vim: set foldmethod=marker :

/* parse.cpp - Parsing G-Code for Franklin.
 * Copyright 2017 Bas Wijnen <wijnen@debian.org> {{{
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
#include "cdriver.h"
#include <fstream>
#include <vector>
#include <list>
#include <algorithm>
#include <cctype>
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
		double x0, y0, z0, x, y, z, a0, b0, c0, a, b, c, f0, f1, e0, e;
		double center[3];
		double normal[3];
		double time, dist;
		Record(bool arc_, int tl, double x0_, double x_, double y0_, double y_, double z0_, double z_, double a0_, double a_, double b0_, double b_, double c0_, double c_, double f0_, double f1_, double e0_, double e_, double cx = NAN, double cy = NAN, double cz = NAN, double nx = NAN, double ny = NAN, double nz = NAN) : arc(arc_), tool(tl), x0(x0_), y0(y0_), z0(z0_), x(x_), y(y_), z(z_), a0(a0_), b0(b0_), c0(c0_), a(a_), b(b_), c(c_), f0(f0_), f1(f1_), e0(e0_), e(e_) {
			center[0] = cx;
			center[1] = cy;
			center[2] = cz;
			normal[0] = nx;
			normal[1] = ny;
			normal[2] = nz;
		}
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
	unsigned strings_length;
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
	void add_record(RunType cmd, int tool = 0, double x = NAN, double y = NAN, double z = NAN, double e = NAN, double f0 = INFINITY, double f1 = INFINITY);
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
				double old_f = current_f[code == 0 ? 0 : 1];
				if (!isnan(F)) {
					current_f[code == 0 ? 0 : 1] = F * unit / 60;
					//if (!(current_f[code == 0 ? 0 : 1] > 0))
					//	debug("new f%d: %f", code == 0 ? 0 : 1, current_f[code == 0 ? 0 : 1]);
				}
				// Slicers don't support acceleration, so set old speed to new speed.
				old_f = current_f[code == 0 ? 0 : 1];
				auto oldpos = pos;
				auto oldepos = epos;
				double estep, r;
				if (code != 81) {
					r = NAN;
					if (!isnan(E)) {
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
					if (!isnan(R)) {
						if (rel)
							r = pos[2] + R * unit;
						else
							r = R * unit;
					}
					else
						r = pos[2];
				}
				if (!isnan(X)) pos[0] = (rel ? pos[0] : 0) + X * unit;
				if (!isnan(Y)) pos[1] = (rel ? pos[1] : 0) + Y * unit;
				if (!isnan(Z)) pos[2] = (rel ? pos[2] : 0) + Z * unit;
				if (!isnan(A)) pos[3] = (rel ? pos[3] : 0) + A * unit;
				if (!isnan(B)) pos[4] = (rel ? pos[4] : 0) + B * unit;
				if (!isnan(C)) pos[5] = (rel ? pos[5] : 0) + C * unit;
				if (code != 81) {
					double dist = 0;
					for (int i = 0; i < 3; ++i)
						dist += (pos[i] - oldpos[i]) * (pos[i] - oldpos[i]);
					dist = sqrt(dist);
					if (isnan(dist))
						dist = 0;
					pending.push_back(Record(false, current_tool, oldpos[0], pos[0], oldpos[1], pos[1], oldpos[2], pos[2], oldpos[3], pos[3], oldpos[4], pos[4], oldpos[5], pos[5], dist > 0 ? old_f / dist : INFINITY, dist > 0 ? current_f[code == 0 ? 0 : 1] / dist : INFINITY, oldepos[current_tool], epos[current_tool]));
				}
				else {
					if (isnan(oldpos[2]))
						oldpos[2] = r;
					flush_pending();
					// Only support OLD_Z (G90) retract mode; don't support repeats(L).
					// goto x, y
					add_record(RUN_LINE, current_tool, pos[0], pos[1], oldpos[2]);
					double dist = sqrt((pos[0] - oldpos[0]) * (pos[0] - oldpos[0]) + (pos[1] - oldpos[1]) * (pos[1] - oldpos[1]));
					if (!isnan(dist))
						last_dist += dist;
					// goto r
					add_record(RUN_LINE, current_tool, pos[0], pos[1], r);
					dist = abs(r - oldpos[2]);
					if (!isnan(dist))
						last_dist += dist;
					// goto z
					if (pos[2] != r) {
						dist = abs(pos[2] - r);
						double f = dist > 0 ? current_f[1] / dist : INFINITY;
						add_record(RUN_LINE, current_tool, pos[0], pos[1], pos[2], NAN, f, f);
						if (isinf(f)) {
							if (!isnan(dist))
								last_dist += dist;
						}
						else {
							if (!isnan(f))
								last_time += 1 / f;
						}
					}
					// go back to old z
					add_record(RUN_LINE, current_tool, pos[0], pos[1], oldpos[2]);
					dist = abs(r - oldpos[2]);
					if (!isnan(dist))
						last_dist += dist;
					// update pos[2].
					pos[2] = oldpos[2];
				}
			}
			break;
		case 2:
		case 3:
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
				double old_f = current_f[1];
				if (!isnan(F)) {
					current_f[1] = F * unit / 60;
					//debug("new f1: %f", current_f[code]);
				}
				// Slicers don't support acceleration, so set old speed to new speed.
				old_f = current_f[1];
				auto oldpos = pos;
				auto oldepos = epos;
				double estep;
				estep = isnan(E) ? 0 : erel ? E * unit - epos[current_tool] : E * unit;
				epos[current_tool] += estep;
				std::vector <double> center;
				center.push_back(0);
				center.push_back(0);
				center.push_back(0);
				if (!isnan(X)) pos[0] = (rel ? pos[0] : 0) + X * unit;
				if (!isnan(Y)) pos[1] = (rel ? pos[1] : 0) + Y * unit;
				if (!isnan(Z)) pos[2] = (rel ? pos[2] : 0) + Z * unit;
				if (!isnan(A)) pos[3] = (rel ? pos[3] : 0) + A * unit;
				if (!isnan(B)) pos[4] = (rel ? pos[4] : 0) + B * unit;
				if (!isnan(C)) pos[5] = (rel ? pos[5] : 0) + C * unit;
				center[0] = pos[0] + (isnan(I) ? 0 : I * unit);
				center[1] = pos[1] + (isnan(J) ? 0 : J * unit);
				center[2] = pos[2] + (isnan(K) ? 0 : K * unit);
				int s = code == 2 ? -1 : 1;
				pending.push_back(Record(true, current_tool, oldpos[0], pos[0], oldpos[1], pos[1], oldpos[2], pos[2], oldpos[3], pos[3], oldpos[4], pos[4], oldpos[5], pos[5], -old_f, -current_f[1], oldepos[current_tool], epos[current_tool], center[0], center[1], center[2], s * arc_normal[0], s * arc_normal[1], s * arc_normal[2]));
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
		pending.push_back(Record(false, current_tool, pos[0], pos[0], pos[1], pos[1], pos[2], pos[2], pos[3], pos[3], pos[4], pos[4], pos[5], pos[5], INFINITY, INFINITY, epos[current_tool], epos[current_tool]));
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
	stringmap.push_back(strings_length);
	strings_length += str.size();
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
	strings_length = 0;
	arc_normal[0] = 0;
	arc_normal[1] = 0;
	arc_normal[2] = 1;
	epos.push_back(0);
	current_f[0] = INFINITY;
	current_f[1] = INFINITY;
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
	bool first = true;
	for (auto str: stringmap) {
		if (!first) {
			uint32_t bin = str;
			outfile.write(reinterpret_cast <char *>(&bin), sizeof(bin));
		}
		else
			first = false;
	}
	// Number of strings.
	uint32_t strnum = strings.size() - 1;
	outfile.write(reinterpret_cast <char *>(&strnum), sizeof(strnum));
	// Bbox.
	for (double b: bbox)
		outfile.write(reinterpret_cast <char *>(&b), sizeof(b));
	outfile.write(reinterpret_cast <char *>(&last_time), sizeof(last_time));
	outfile.write(reinterpret_cast <char *>(&last_dist), sizeof(last_dist));
	// Time+dist
} // }}}

void Parser::flush_pending() { // {{{
	// Turn Records into Run_Records and write them out.
	while (pending.size() > 0) {
		// TODO: Limit moves.
		// TODO: Look ahead to prevent overly limiting moves.
		Record &p = pending.front();
		if (p.arc) {
			add_record(RUN_PRE_ARC, p.tool, p.center[0], p.center[1], p.center[2], p.normal[0], p.normal[1], p.normal[2]);
			add_record(RUN_ARC, p.tool, p.x, p.y, p.z, p.e, p.f0, p.f1);
			// TODO: Update time+dist.
			// TODO: Update bbox.
		}
		else {
			// Not an arc.
			if (((!isnan(p.a0) || !isnan(p.a)) && p.a0 != p.a) || ((!isnan(p.b0) || !isnan(p.b)) && p.b0 != p.b) || ((!isnan(p.c0) || !isnan(p.c)) && p.c0 != p.c))
				add_record(RUN_PRE_LINE, p.tool, p.a, p.b, p.c);
			add_record(RUN_LINE, p.tool, p.x, p.y, p.z, p.e, p.f0, p.f1);
			// Update time+dist.
			double dist = 0;
			double dists[3] = {p.x - p.x0, p.y - p.y0, p.z - p.z0};
			for (int i = 0; i < 3; ++i) {
				double d = dists[i] * dists[i];
				if (!isnan(d))
					dist += d;
			}
			dist = sqrt(dist);
			if (dist > 0) {
				if (isinf(p.f0) || isinf(p.f1)) {
					if (!isnan(dist))
						last_dist += dist;
				}
				else {
					double t = 2 / (p.f0 + p.f1);
					if (isinf(t))
						debug("%d:wtf dist %f x %f y %f f0 %f f1 %f", lineno, dist, p.x, p.y, p.f0, p.f1);
					if (!isnan(t))
						last_time += t;
				}
			}
			// Update bbox.
			// Use "not larger" instead of "smaller" so NaNs are replaced.
			if (!(p.x > bbox[0]))
				bbox[0] = p.x;
			if (!(p.x < bbox[1]))
				bbox[1] = p.x;
			if (!(p.y > bbox[2]))
				bbox[2] = p.y;
			if (!(p.y < bbox[3]))
				bbox[3] = p.y;
			if (!(p.z > bbox[4]))
				bbox[4] = p.z;
			if (!(p.z < bbox[5]))
				bbox[5] = p.z;
		}
		pending.pop_front();
	}
} // }}}

void Parser::add_record(RunType cmd, int tool, double x, double y, double z, double e, double f0, double f1) { // {{{
	Run_Record r;
	r.type = cmd;
	r.tool = tool;
	r.X = x;
	r.Y = y;
	r.Z = z;
	r.E = e;
	r.F_start = f0;
	r.F_end = f1;
	r.time = last_time;
	r.dist = last_dist;
	outfile.write(reinterpret_cast <char *>(&r), sizeof(r));
} // }}}

void parse_gcode(std::string const &infilename, std::string const &outfilename) {
	// Create an instance of the class.  The constructor does all the work.
	Parser p(infilename, outfilename);
	// No other actions needed.
}
// vim: set foldmethod=marker :

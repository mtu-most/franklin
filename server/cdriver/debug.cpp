/* debug.cpp - Debug function implementations for Franklin
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

#if DEBUG_BUFFER_LENGTH > 0
void buffered_debug_flush() {
	if (debug_buffer_ptr == 0)
		return;
	debug_buffer[debug_buffer_ptr] = '\0';
	debug_buffer_ptr = 0;
	debug("%s", debug_buffer);
}

static void add_char(char c) {
	if  (debug_buffer_ptr >= DEBUG_BUFFER_LENGTH - 1)
		buffered_debug_flush();
	debug_buffer[debug_buffer_ptr++] = c;
}

static void add_num(int32_t num, int8_t base, int8_t decimal = 0) {
	if (num < 0) {
		add_char('-');
		num = -num;
	}
	int16_t digits = 0;
	int32_t n = num;
	int32_t power = 1;
	while (n > 0 || digits < decimal + 1) {
		n /= base;
		power *= base;
		digits += 1;
	}
	while (digits > 0) {
		if (digits-- == decimal)
			add_char('.');
		power /= base;
		uint8_t c = num / power;
		num -= uint32_t(c) * power;
		if (c < 10)
			add_char('0' + c);
		else
			add_char('a' + c - 10);
	}
}

void buffered_debug(char const *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	for (char const *p = fmt; *p; ++p) {
		if (*p == '%') {
			bool longvalue = false;
			while (true) {
				++p;
				switch (*p) {
				case 0: {
					add_char('%');
					--p;
					break;
				}
				case 'l': {
					longvalue = true;
					continue;
				}
				case '%': {
					add_char('%');
					break;
				}
				case 'd': {
					if (longvalue) {
						int32_t *arg = va_arg(ap, int32_t *);
						add_num(*arg, 10);
					}
					else {
						int arg = va_arg(ap, int);
						add_num(arg, 10);
					}
					break;
				}
				case 'x': {
					if (longvalue) {
						int32_t *arg = va_arg(ap, int32_t *);
						add_num(*arg, 16);
					}
					else {
						int arg = va_arg(ap, int);
						add_num(arg, 16);
					}
					break;
				}
				case 'f': {
					float *arg = va_arg(ap, float *);
					add_num(*arg * (longvalue ? 1e6 : 1e3), 10, longvalue ? 6 : 3);
					break;
				}
				case 's': {
					char const *arg = va_arg(ap, char const *);
					while (*arg)
						add_char(*arg);
					break;
				}
				case 'c': {
					char arg = va_arg(ap, int);
					add_char(arg);
					break;
				}
				default: {
					add_char('%');
					add_char(*p);
					break;
				}
				}
				break;
			}
		}
		else {
			add_char(*p);
		}
	}
	va_end(ap);
	add_char(';');
}
#endif

void debug_add(int a, int b, int c, int d) {
	debug_list[debug_list_next][0] = a;
	debug_list[debug_list_next][1] = b;
	debug_list[debug_list_next][2] = c;
	debug_list[debug_list_next][3] = d;
	debug_list_next = (debug_list_next + 1) % DEBUG_LIST_SIZE;
}

void debug_dump() {
	fprintf(stderr, "Debug dump:\n");
	for (int i = 0; i < DEBUG_LIST_SIZE; ++i) {
		int n = (debug_list_next + i) % DEBUG_LIST_SIZE;
		fprintf(stderr, "\t%02d", n);
		for (int m = 0; m < 4; ++m) {
			if (m > 0 && debug_list[n][m] == int(0xfbfbfbfb))
				continue;
			fprintf(stderr, "\t%08x", debug_list[n][m]);
		}
		fprintf(stderr, "\n");
	}
}

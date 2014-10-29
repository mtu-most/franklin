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

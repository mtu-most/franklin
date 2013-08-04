#include "firmware.hh"

void debug (char const *fmt, ...) {
	va_list ap;
	va_start (ap, fmt);
	Serial.write (CMD_DEBUG);
	for (char const *p = fmt; *p; ++p) {
		if (*p == '%') {
			++p;
			switch (*p) {
			case 0: {
				Serial.write ('%');
				--p;
				break;
			}
			case '%': {
				Serial.write ('%');
				break;
			}
			case 'd': {
				int arg = va_arg (ap, int);
				Serial.print (arg, DEC);
				break;
			}
			case 'x': {
				int arg = va_arg (ap, int);
				Serial.print (arg, HEX);
				break;
			}
			case 'f': {
				float arg = va_arg (ap, float);
				Serial.print (arg);
				break;
			}
			case 's': {
				char const *arg = va_arg (ap, char const *);
				Serial.print (arg);
				break;
			}
			case 'c': {
				char arg = va_arg (ap, char);
				Serial.write (arg);
				break;
			}
			default: {
				Serial.write ('%');
				Serial.write (*p);
				break;
			}
			}
		}
		else {
			Serial.write (*p);
		}
	}
	va_end (ap);
	Serial.write (0);
	Serial.flush ();
}

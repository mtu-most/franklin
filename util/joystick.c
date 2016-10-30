/* joystick.c - passing joystick constants to python for Franklin
 * vim: set foldmethod=marker :
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

#include <linux/joystick.h>
#include <stdio.h>

int main() {
	printf("import struct\n");
	printf("event = struct.Struct('=IhBB')\n");
	printf("corr = struct.Struct('=ihH')\n");

	printf("version = 0x%x\n", (unsigned)JS_VERSION);

	printf("event_button = 0x%x\n", (unsigned)JS_EVENT_BUTTON);
	printf("event_axis = 0x%x\n", (unsigned)JS_EVENT_AXIS);
	printf("event_init = 0x%x\n", (unsigned)JS_EVENT_INIT);

	printf("gversion = 0x%x\n", (unsigned)JSIOCGVERSION);
	printf("gaxes = 0x%x\n", (unsigned)JSIOCGAXES);
	printf("gbuttons = 0x%x\n", (unsigned)JSIOCGBUTTONS);
	printf("gname = 0x%x\n", (unsigned)(JSIOCGNAME(4096)));

	printf("scorr = 0x%x\n", (unsigned)JSIOCSCORR);
	printf("gcorr = 0x%x\n", (unsigned)JSIOCGCORR);

	printf("saxmap = 0x%x\n", (unsigned)JSIOCSAXMAP);
	printf("gaxmap = 0x%x\n", (unsigned)JSIOCGAXMAP);

	printf("sbtnmap = 0x%x\n", (unsigned)JSIOCSBTNMAP);
	printf("gbtnmap = 0x%x\n", (unsigned)JSIOCGBTNMAP);

	printf("corr_none = 0x%x\n", (unsigned)JS_CORR_NONE);
	printf("corr_broken = 0x%x\n", (unsigned)JS_CORR_BROKEN);

	return 0;
}

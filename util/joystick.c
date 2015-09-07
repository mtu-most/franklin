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

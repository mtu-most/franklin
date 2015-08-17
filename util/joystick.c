#include <linux/joystick.h>
#include <stdio.h>

int main() {
	printf("import struct\n");
	printf("event = struct.Struct('=IhBB')\n");
	printf("corr = struct.Struct('=ihH')\n");

	printf("version = 0x%x\n", JS_VERSION);

	printf("event_button = 0x%x\n", JS_EVENT_BUTTON);
	printf("event_axis = 0x%x\n", JS_EVENT_AXIS);
	printf("event_init = 0x%x\n", JS_EVENT_INIT);

	printf("gversion = 0x%lx\n", JSIOCGVERSION);
	printf("gaxes = 0x%lx\n", JSIOCGAXES);
	printf("gbuttons = 0x%lx\n", JSIOCGBUTTONS);
	printf("gname = 0x%x\n", JSIOCGNAME(4096));

	printf("scorr = 0x%lx\n", JSIOCSCORR);
	printf("gcorr = 0x%lx\n", JSIOCGCORR);

	printf("saxmap = 0x%lx\n", JSIOCSAXMAP);
	printf("gaxmap = 0x%lx\n", JSIOCGAXMAP);

	printf("sbtnmap = 0x%lx\n", JSIOCSBTNMAP);
	printf("gbtnmap = 0x%lx\n", JSIOCGBTNMAP);

	printf("corr_none = 0x%x\n", JS_CORR_NONE);
	printf("corr_broken = 0x%x\n", JS_CORR_BROKEN);

	return 0;
}

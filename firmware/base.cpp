#include "firmware.h"

#ifdef DEFINE_MAIN
#include "firmware.ino"
// Memory handling
void _mem_alloc(uint16_t size, void **target) {
	*target = malloc(size);
	if (!*target) {
		debug("unable to allocate memory");
		exit(0);
	}
}

void _mem_retarget(void **target, void **newtarget) {
	*newtarget = *target;
	*target = NULL;
}

void _mem_free(void **target) {
	free(*target);
	*target = NULL;
}

// Main function.
int main(int argc, char **argv) {
	setup();
	while(true)
		loop();
}

#endif

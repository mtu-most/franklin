#define EXTERN	// This must be done in exactly one source file.
#include "firmware.h"

void setup()
{
	arch_setup_start();
	watchdog_disable();
	// Initialize volatile variables.
	Serial.begin(115200);
#if DEBUG_BUFFER_LENGTH > 0
	debug_buffer_ptr = 0;
#endif
	debug("Starting");
	command_end = 0;
	ping = 0;
	pending_packet[0] = 0;
	out_busy = false;
	reply_ready = false;
	adcreply_ready = false;
	start_time = utime();
	led_pin.init();
	led_phase = 0;
	led_fast = false;
	led_last = start_time;
	num_motors = 0;
	motor = NULL;
	stopping = false;
	adc_current = ~0;
#ifdef HAVE_AUDIO
	audio_head = 0;
	audio_tail = 0;
	audio_state = 0;
	audio_us_per_sample = 125; // 1000000 / 8000;
	continue_cb = false;
#endif
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		printerid[i] = 0;
	Serial.write(CMD_ID);
	for (uint8_t i = 0; i < 8; ++i)
		Serial.write(printerid[i]);
	arch_setup_end();
}

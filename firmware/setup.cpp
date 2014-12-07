#define EXTERN	// This must be done in exactly one source file.
#include "firmware.h"

void setup()
{
	arch_setup_start();
	watchdog_disable();
	uint32_t current_time = utime();
	Serial.begin(115200);
	for (uint8_t p = 0; p < NUM_DIGITAL_PINS; ++p) {
		pin[p].state = CTRL_UNSET << 2 | CTRL_RESET;
		UNSET(p);
	}
	for (uint8_t m = 0; m < NUM_MOTORS; ++m)
		motor[m].init(m);
	for (uint8_t b = 0; b < NUM_BUFFERS; ++b) {
		for (uint8_t f = 0; f < FRAGMENTS_PER_BUFFER; ++f)
			buffer[b][f].num_samples = 0;
	}
	notified_current_fragment = 0;
	current_fragment = notified_current_fragment;
	last_fragment = FRAGMENTS_PER_BUFFER - 1;
	filling = 0;
	for (uint8_t a = 0; a < NUM_ANALOG_INPUTS; ++a) {
		for (uint8_t i = 0; i < 2; ++i) {
			adc[a].linked[i] = ~0;
			adc[a].value[i] = 1 << 15;
		}
	}
	adc_phase = INACTIVE;
	adc_current = ~0;
	adc_next = ~0;
	command_end = 0;
	ping = 0;
	out_busy = false;
	reply_ready = 0;
	adcreply_ready = 0;
	stopped = true;
	stopping = false;
	underrun = false;
	led_fast = false;
	led_last = current_time;
	led_phase = 0;
	led_pin = ~0;
	active_motors = 0;
	
	arch_setup_end();	// This fills printerid.
	Serial.write(CMD_ID);
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		Serial.write(printerid[i]);
}

#define EXTERN	// This must be done in exactly one source file.
#include "firmware.h"

void setup()
{
	arch_setup_start();
	watchdog_disable();
	Serial.begin(115200);
	for (uint8_t p = 0; p < NUM_DIGITAL_PINS; ++p) {
		// Reset state is unset, then unset the pin.
		pin[p].state = CTRL_UNSET << 2 | CTRL_RESET;
		UNSET(p);
	}
	// Initialize motors.
	for (uint8_t m = 0; m < NUM_MOTORS; ++m)
		motor[m].init(m);
	// Clear all buffers.
	for (uint8_t b = 0; b < NUM_BUFFERS; ++b) {
		for (uint8_t f = 0; f < FRAGMENTS_PER_BUFFER; ++f)
			buffer[b][f].num_samples = 0;
	}
	notified_current_fragment = 0;
	current_fragment = notified_current_fragment;
	last_fragment = FRAGMENTS_PER_BUFFER - 1;
	filling = 0;
	// Disable all adcs.
	for (uint8_t a = 0; a < NUM_ANALOG_INPUTS; ++a) {
		for (uint8_t i = 0; i < 2; ++i) {
			adc[a].linked[i] = ~0;
			adc[a].value[i] = 1 << 15;
			adc[a].is_on = false;
		}
	}
	adc_phase = INACTIVE;
	adc_current = ~0;
	adc_next = ~0;
	temps_disabled = true;
	// Set up communication state.
	command_end = 0;
	had_data = false;
	ping = 0;
	out_busy = false;
	reply_ready = 0;
	adcreply_ready = 0;
	// Set up homing state.
	homers = 0;
	home_step_time = 0;
	// Set up movement state.
	stopped = true;
	stopping = false;
	underrun = false;
	active_motors = 0;
	// Set up led state.
	led_fast = 0;
	led_last = millis();
	led_phase = 0;
	led_pin = ~0;
	// Do arch-specific things.  This fills printerid.
	arch_setup_end();
	// Inform host of reset.
	Serial.write(CMD_STARTUP);
	for (uint8_t i = 0; i < ID_SIZE; ++i)
		Serial.write(printerid[i]);
	Serial.write(CMD_STARTUP);
}

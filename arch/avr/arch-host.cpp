/* arch-avr.cpp - avr-specific parts for Franklin {{{
 * vim: set foldmethod=marker :
 * Copyright 2014-2016 Michigan Technological University
 * Copyright 2016-2022 Bas Wijnen <wijnen@debian.org>
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
 * }}} */

#include "cdriver.h"

// Serial port communication. {{{
int hwpacketsize(int len, int *available) { // {{{
	int const arch_packetsize[16] = { 0, 2, 0, 2, 0, 0, 3, 0, 4, 0, 1, 3, -1, -1, -1, -1 };
	if (arch_packetsize[command[0] & 0xf] > 0)
		return arch_packetsize[command[0] & 0xf];
	if (len < 2) {
		if (*available == 0)
			return 2;	// The data is not available, so this will not trigger the packet to be parsed yet.
		command[1] = serialdev->read();
		command_end += 1;
		*available -= 1;
	}
	switch (command[0] & 0x1f) {
	case HWC_READY:
		return command[1];
	case HWC_HOMED:
		return 2 + 4 * command[1];
	case HWC_STOPPED:
		return 3 + 4 * command[1];
	case HWC_NAMED_PIN:
		return 2 + command[1];
	case HWC_LIMIT:
	case HWC_UNDERRUN:
		return 4 + 4 * command[1];
	default:
		debug("ignoring invalid serial command %x", command[0]);
		return 1;	// Parse and fail immediately.
	}
} // }}}

#define CONTROL_SIZE 6
void try_send_control() { // {{{
	if (!connected || preparing || out_busy >= 3 || avr_control_queue_length == 0)
		return;
	avr_control_queue_length -= 1;
	avr_buffer[0] = HWC_CONTROL;
	avr_buffer[1] = avr_control_queue[avr_control_queue_length * CONTROL_SIZE];
	avr_buffer[2] = avr_control_queue[avr_control_queue_length * CONTROL_SIZE + 1];
	avr_buffer[3] = avr_control_queue[avr_control_queue_length * CONTROL_SIZE + 2];
	avr_buffer[4] = avr_control_queue[avr_control_queue_length * CONTROL_SIZE + 3];
	avr_buffer[5] = avr_control_queue[avr_control_queue_length * CONTROL_SIZE + 4];
	avr_buffer[6] = avr_control_queue[avr_control_queue_length * CONTROL_SIZE + 5];
	avr_in_control_queue[avr_control_queue[avr_control_queue_length * CONTROL_SIZE]] = false;
	prepare_packet(avr_buffer, CONTROL_SIZE + 1);
	avr_send();
} // }}}

void arch_had_ack() { // {{{
	if (out_busy == 0)
		try_send_control();
} // }}}

void avr_send() { // {{{
	//debug("avr_send");
	if (!connected) {
		debug("send called while not connected");
		abort();
	}
	while (out_busy >= 3)
		serial_wait();
	serial_cb[out_busy] = avr_cb;
	avr_cb = NULL;
	send_packet();
	if (out_busy < 3)
		try_send_control();
} // }}}

void avr_call1(uint8_t cmd, uint8_t arg) { // {{{
	avr_buffer[0] = cmd;
	avr_buffer[1] = arg;
	prepare_packet(avr_buffer, 2);
	avr_send();
} // }}}

double arch_round_pos(int s, int m, double pos) { // {{{
	if (s >= NUM_SPACES)
		return pos;
	int mi = 0;
	for (int ts = 0; ts < s; ++ts) mi += spaces[ts].num_motors;
	if (mi + m >= NUM_MOTORS)
		return pos;
	return round((pos + avr_pos_offset[mi + m]) * spaces[s].motor[m]->steps_per_unit) / spaces[s].motor[m]->steps_per_unit - avr_pos_offset[mi + m];
} // }}}

int arch_pos2hw(int s, int m, double pos) {
	int mi = 0;
	for (int ts = 0; ts < s; ++ts)
		mi += spaces[ts].num_motors;
	return (pos + avr_pos_offset[m + mi]) * spaces[s].motor[m]->steps_per_unit;
}

double arch_hw2pos(int s, int m, int hw) {
	int mi = 0;
	for (int ts = 0; ts < s; ++ts)
		mi += spaces[ts].num_motors;
	return hw / spaces[s].motor[m]->steps_per_unit - avr_pos_offset[m + mi];
}


void avr_get_current_pos(int offset, bool check) { // {{{
	static int have_error = 0;
	int mi = 0;
	for (int ts = 0; ts < NUM_SPACES; mi += spaces[ts++].num_motors) {
		for (int tm = 0; tm < spaces[ts].num_motors; ++tm) {
			int old_hw = spaces[ts].motor[tm]->settings.hw_pos;
			int p = 0;
			for (int i = 0; i < 4; ++i) {
				p += int(uint8_t(command[offset + 4 * (tm + mi) + i])) << (i * 8);
			}
			if (spaces[ts].motor[tm]->dir_pin.inverted())
				p *= -1;
			if (check) {
				if (old_hw != p) {
					if (ts != 2) {	// followers are expected to go out of sync all the time.
						debug("WARNING: position for %d %d out of sync!  old = %d, new = %d (difference = %d), offset = %f", ts, tm, old_hw, p, p - old_hw, avr_pos_offset[tm + mi]);
						have_error += 1;
					}
					spaces[ts].motor[tm]->settings.current_pos = p / spaces[ts].motor[tm]->steps_per_unit - avr_pos_offset[tm + mi];
					spaces[ts].motor[tm]->settings.hw_pos = p;
				}
				//else {
					//debug("Check: position for %d %d in sync, old = %f, new = %f offset = %f", ts, tm, old, p, avr_pos_offset[tm + mi]);
				//}
			}
			else {
				// Motor positions were unknown; no check, just update position.
				if (old_hw != p) {
					double new_pos = p / spaces[ts].motor[tm]->steps_per_unit - avr_pos_offset[tm + mi];
					cpdebug(ts, tm, "update current pos from %f to %f", spaces[ts].motor[tm]->settings.current_pos, new_pos);
					spaces[ts].motor[tm]->settings.current_pos = new_pos;
					spaces[ts].motor[tm]->settings.hw_pos = p;
				}
			}
		}
	}
	if (have_error > 0) {
		abort();
	}
} // }}}

bool hwpacket(int len) { // {{{
	(void)&len;
	// Handle data in command.
#if 0
	if (command[0] != HWC_ADC) {
		fprintf(stderr, "packet received:");
		for (uint8_t i = 0; i < len; ++i)
			fprintf(stderr, " %02x", command[i]);
		fprintf(stderr, "\n");
	}
#endif
	// Only used for underrun and done; initialized for done, overwritten by underrun. {{{
	bool sent_ack = false;
	int offset = 0;
	int done_count = command[1];
	int remaining_count = command[2];
	// }}}
	switch (command[0]) {
	case HWC_LIMIT: // {{{
	{
		uint8_t which = command[2];
		if (which > NUM_MOTORS) {
			if (initialized) {
				debug("cdriver: Invalid limit for avr motor %d", which);
				abort();
			}
			avr_write_ack("pre-limit");
			return false;
		}
		avr_write_ack("limit");
		avr_homing = false;
		abort_move(int8_t(command[3]));
		avr_get_current_pos(4, false);
		if (spaces[0].num_axes > 0)
			cpdebug(0, 0, "ending hwpos %f", spaces[0].motor[0]->settings.current_pos + avr_pos_offset[0]);
		double pos;
		int s, m = -1;
		if (which >= avr_active_motors) {
			s = -1;
			m = -1;
			pos = NAN;
		}
		else {
			for (s = 0; s < NUM_SPACES; ++s) {
				if (which < spaces[s].num_motors) {
					m = which;
					break;
				}
				which -= spaces[s].num_motors;
			}
			cpdebug(s, m, "limit");
			pos = spaces[s].motor[m]->settings.current_pos;
		}
		//debug("reset pending for limit %d %d", s, m);
		cb_pending = false;
		avr_running = false;
		settings.queue_start = 0;
		settings.queue_end = 0;
		stopping = 3;
		computing_move = false;
		current_fragment_pos = 0;
		current_fragment = 0;
		running_fragment = 0;
		prepare_interrupt();
		shmem->interrupt_ints[0] = s;
		shmem->interrupt_ints[1] = m;
		shmem->interrupt_float = pos;
		send_to_parent(CMD_LIMIT);
		//debug("limit done");
		return false;
	} // }}}
	case HWC_PONG: // {{{
	{
		avr_pong = command[1];
		avr_write_ack("pong");
		return false;
	} // }}}
	case HWC_ADC: // {{{
	{
		int pin = command[1];
		if (pin < 0 || pin >= NUM_ANALOG_INPUTS) {
			if (avr_pong == 255)
				debug("invalid adc %d received", pin);
			avr_write_ack("invalid adc");
			return false;
		}
		int adc = (command[2] & 0xff) | ((command[3] & 0xff) << 8);
		avr_write_ack("adc");
		if (pin < NUM_ANALOG_INPUTS && avr_adc_id[pin] >= 0 && avr_adc_id[pin] < num_temps)
			handle_temp(avr_adc_id[pin], adc);
		return false;
	} // }}}
	case HWC_UNDERRUN: // {{{
	{
		offset = 1;
		done_count = command[2];
		remaining_count = command[3];
		if (host_block) {
			// STOP was sent; ignore UNDERRUN.
			avr_write_ack("stopped underrun");
			//debug("underrun check2 %d %d %d", sending_fragment, current_fragment, running_fragment);
			return false;
		}
		if (!avr_running) {
			debug("unexpected underrun?");
			abort();
		}
		avr_running = false;
		if (computing_move) {
			debug("slowness underrun %d %d %d", sending_fragment, current_fragment, running_fragment);
			//abort();
			avr_write_ack("slowness underrun");
			if (!sending_fragment && discarding == 0 && (current_fragment - (running_fragment + done_count + remaining_count) + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER > 1)
				arch_start_move(done_count);
			// Buffer is too slow with refilling; this will fix itself.
		}
		else {
			cb_pending = true;
			// Only overwrite current position if the new value is correct.
			//debug("underrun ok current=%d running=%d computing_move=%d sending=%d pending=%d transmitting=%d", current_fragment, running_fragment, computing_move, sending_fragment, remaining_count, transmitting_fragment);
			if (!sending_fragment && !transmitting_fragment && discarding == 0 && current_fragment_pos == 0) {
				if (remaining_count == 0) {
					avr_get_current_pos(4, true);
					avr_write_ack("expected underrun");
					sent_ack = true;
					// An expected underrun during a job is the end of a goto operation and the next command should be sent.
					run_file_next_command(settings.hwtime);
					buffer_refill();
					// FIXME: this can trigger a repeat request, which is handled as a new request because write_ack() is not called yet.
				}
			}
			//debug("underrun check %d %d %d", sending_fragment, current_fragment, running_fragment);
		}
	} // }}}
		// Fall through.
	case HWC_DONE: // {{{
	{
		if (host_block) {
			// STOP was sent; ignore DONE.
			//debug("done during stop");
			if (!sent_ack)
				avr_write_ack("stopped done");
			return false;
		}
		//debug("done: %d pending %d sending %d current %d running %d", done_count, remaining_count, sending_fragment, current_fragment, running_fragment);
		if (FRAGMENTS_PER_BUFFER == 0) {
			//debug("Done received while fragments per buffer is zero");
			if (!sent_ack)
				avr_write_ack("invalid done");
			return false;
		}
		first_fragment = -1;
		if ((current_fragment + discarding - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER + 1 < done_count + remaining_count) {
			debug("Done count %d+%d higher than busy fragments %d+%d+1; clipping", done_count, remaining_count, (current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER, discarding);
			if (!sent_ack)
				avr_write_ack("invalid done");
			abort();
		}
		else if (!sent_ack)
			avr_write_ack("done");
		//debug("fragment done for gcode line %" LONGFMT, history[running_fragment].gcode_line);
		running_fragment = (running_fragment + done_count) % FRAGMENTS_PER_BUFFER;
		//debug("running -> %x", running_fragment);
		if ((current_fragment + discarding + (transmitting_fragment ? 1 : 0)) % FRAGMENTS_PER_BUFFER == running_fragment && offset == 0) {
			debug("Done received, but should be underrun (current: %d discarding: %d running: %d sending: %d transmitting %d)", current_fragment, discarding, running_fragment, sending_fragment, transmitting_fragment);
			abort();
		}
		if (out_busy < 3)
			buffer_refill();
		//else
		//	debug("no refill");
		return false;
	} // }}}
	case HWC_HOMED: // {{{
	{
		if (!avr_homing) {
			if (initialized) {
				debug("Unexpected home reply received");
				abort();
			}
			avr_write_ack("pre-homed");
			return false;
		}
		computing_move = false;
		avr_homing = false;
		avr_get_current_pos(2, false);
		// Firmware resets motor positions to 0 after homing.
		int m0 = 0;
		for (int s = 0; s < NUM_SPACES; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m) {
				avr_pos_offset[m0 + m] = -spaces[s].motor[m]->settings.current_pos;
				spaces[s].motor[m]->settings.hw_pos = 0;
			}
			m0 += spaces[s].num_motors;
		}
		avr_write_ack("homed");
		prepare_interrupt();
		send_to_parent(CMD_HOMED);
		return false;
	} // }}}
	case HWC_TIMEOUT: // {{{
	{
		avr_write_ack("timeout");
		for (int i = 0; i < NUM_DIGITAL_PINS; ++i)
			avr_pins[i].state = avr_pins[i].reset;
		for (int i = 0; i < num_gpios; ++i)
			gpios[i].state = gpios[i].reset;
		motors_busy = false;
		// Everything has shut down; reset pins to normal (but inactive).
		arch_motors_change();
		for (int s = 0; s < NUM_SPACES; ++s) {
			for (int m = 0; m < spaces[s].num_motors; ++m) {
				RESET(spaces[s].motor[m]->step_pin);
				RESET(spaces[s].motor[m]->dir_pin);
				RESET(spaces[s].motor[m]->enable_pin);
				spaces[s].motor[m]->settings.current_pos = 0;
				spaces[s].motor[m]->settings.hw_pos = 0;
			}
		}
		for (int m = 0; m < NUM_MOTORS; ++m)
			avr_pos_offset[m] = 0;
		for (int t = 0; t < num_temps; ++t)
			settemp(t, NAN);
		prepare_interrupt();
		send_to_parent(CMD_TIMEOUT);
		return false;
	} // }}}
	case HWC_PINCHANGE: // {{{
	{
		avr_write_ack("pinchange");
		for (int i = 0; i < num_gpios; ++i) {
			if (gpios[i].pin.pin == command[1]) {
				gpios[i].value = command[2];
				if (gpios[i].changed)
					continue;
				gpios[i].changed = true;
				pins_changed += 1;
			}
		}
		return false;
	} // }}}
	default: // {{{
	{
		if (expected_replies <= 0) {
			debug("Received unexpected reply %x", command[0]);
			avr_write_ack("unexpected reply");
		}
		else
			return true;
		return false;
	} // }}}
	}
} // }}}
// }}}

// Hardware interface {{{
void avr_setup_pin(int pin, int type, int resettype, int extra) { // {{{
	if (!connected)
		return;
	if (pin < 0 || pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin to set up");
		abort();
	}
	//debug("pin %d type %d reset %d extra %d", pin, type, resettype, extra);
	if (avr_in_control_queue[pin])
	{
		for (int i = 0; i < avr_control_queue_length; ++i) {
			if (avr_control_queue[i * CONTROL_SIZE] != pin)
				continue;
			avr_control_queue[i * CONTROL_SIZE + 1] = type | (resettype << 2) | extra;
			avr_control_queue[i * CONTROL_SIZE + 2] = avr_pins[pin].duty & 0xff;
			avr_control_queue[i * CONTROL_SIZE + 3] = (avr_pins[pin].duty >> 8) & 0xff;
			avr_control_queue[i * CONTROL_SIZE + 4] = avr_pins[pin].motor;
			avr_control_queue[i * CONTROL_SIZE + 5] = avr_pins[pin].ticks;
			return;
		}
	}
	avr_control_queue[avr_control_queue_length * CONTROL_SIZE] = pin;
	avr_control_queue[avr_control_queue_length * CONTROL_SIZE + 1] = type | (resettype << 2) | extra;
	avr_control_queue[avr_control_queue_length * CONTROL_SIZE + 2] = avr_pins[pin].duty & 0xff;
	avr_control_queue[avr_control_queue_length * CONTROL_SIZE + 3] = (avr_pins[pin].duty >> 8) & 0xff;
	avr_control_queue[avr_control_queue_length * CONTROL_SIZE + 4] = avr_pins[pin].motor;
	avr_control_queue[avr_control_queue_length * CONTROL_SIZE + 5] = avr_pins[pin].ticks;
	avr_control_queue_length += 1;
	avr_in_control_queue[pin] = true;
	try_send_control();
} // }}}

void SET_INPUT(Pin_t _pin) { // {{{
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state == 2)
		return;
	avr_pins[_pin.pin].state = 2;
	avr_setup_pin(_pin.pin, CTRL_INPUT, avr_pins[_pin.pin].reset, CTRL_NOTIFY);
} // }}}

void SET_INPUT_NOPULLUP(Pin_t _pin) { // {{{
	if (!_pin.valid() || _pin.pin >= NUM_DIGITAL_PINS)
		return;
	if (avr_pins[_pin.pin].state == 3)
		return;
	avr_pins[_pin.pin].state = 3;
	avr_setup_pin(_pin.pin, CTRL_UNSET, avr_pins[_pin.pin].reset, CTRL_NOTIFY);
} // }}}

void RESET(Pin_t _pin) { // {{{
	if (!_pin.valid())
		return;
	//debug("reset %d", _pin.pin);
	if (avr_pins[_pin.pin].state == 0)
		return;
	//debug("resetting pin %d inverted: %d", _pin.pin, _pin.inverted());
	avr_pins[_pin.pin].state = 0;
	if (_pin.inverted())
		avr_setup_pin(_pin.pin, CTRL_SET, avr_pins[_pin.pin].reset < 2 ? 1 - avr_pins[_pin.pin].reset : avr_pins[_pin.pin].reset, 0);
	else
		avr_setup_pin(_pin.pin, CTRL_RESET, avr_pins[_pin.pin].reset, 0);
} // }}}

void SET(Pin_t _pin) { // {{{
	if (!_pin.valid())
		return;
	//debug("set %d", _pin.pin);
	if (avr_pins[_pin.pin].state == 1)
		return;
	avr_pins[_pin.pin].state = 1;
	//debug("setting pin %d", _pin.pin);
	if (_pin.inverted())
		avr_setup_pin(_pin.pin, CTRL_RESET, avr_pins[_pin.pin].reset < 2 ? 1 - avr_pins[_pin.pin].reset : avr_pins[_pin.pin].reset, 0);
	else
		avr_setup_pin(_pin.pin, CTRL_SET, avr_pins[_pin.pin].reset, 0);
} // }}}

void SET_OUTPUT(Pin_t _pin) { // {{{
	if (!_pin.valid())
		return;
	if (avr_pins[_pin.pin].state < 2)
		return;
	RESET(_pin);
} // }}}

void avr_get_cb_wrap() { // {{{
	void (*cb)(bool) = avr_get_cb;
	avr_get_cb = NULL;
	bool arg = avr_get_pin_invert ^ command[1];
	avr_write_ack("get");
	cb(arg);
} // }}}

void GET(Pin_t _pin, bool _default, void(*cb)(bool)) { // {{{
	if (!connected || !_pin.valid())
		cb(_default);
	wait_for_reply[expected_replies++] = avr_get_cb_wrap;
	avr_get_cb = cb;
	avr_get_pin_invert = _pin.inverted();
	avr_call1(HWC_GETPIN, _pin.pin);
} // }}}

void avr_send_pin(Pin_t _pin) { // {{{
	int s, r;
	if (_pin.inverted()) {
		s = avr_pins[_pin.pin].state < 2 ? 1 - avr_pins[_pin.pin].state : avr_pins[_pin.pin].state;
		r = avr_pins[_pin.pin].reset < 2 ? 1 - avr_pins[_pin.pin].reset : avr_pins[_pin.pin].reset;
	}
	else {
		s = avr_pins[_pin.pin].state;
		r = avr_pins[_pin.pin].reset;
	}
	avr_setup_pin(_pin.pin, s, r, avr_pins[_pin.pin].reset != 2 ? 0 : CTRL_NOTIFY);
} // }}}

void arch_pin_set_reset(Pin_t _pin, char state) { // {{{
	if (!_pin.valid() || _pin.pin >= NUM_DIGITAL_PINS) {
		// Ignore requests to set reset state of invalid and analog pins.
		return;
	}
	if (avr_pins[_pin.pin].reset == state)
		return;
	avr_pins[_pin.pin].reset = state;
	avr_send_pin(_pin);
} // }}}

void arch_set_duty(Pin_t _pin, double duty) { // {{{
	if (!connected)
		return;
	if (_pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_set_duty: %d (max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return;
	}
	int hwduty = round(duty * 0x8000) - 1;
	if (hwduty < 0)
		hwduty = 0;
	if (hwduty > 0x7fff) {
		debug("invalid duty value %d; clipping to 0x7fff.", hwduty);
		hwduty = 0x7fff;
	}
	if (hwduty != avr_pins[_pin.pin].duty) {
		//debug("updating duty for %d to %d", _pin.pin, hwduty);
		avr_pins[_pin.pin].duty = hwduty;
		avr_send_pin(_pin);
	}
} // }}}

void arch_set_pin_motor(Pin_t _pin, int s, int m, int ticks) { // {{{
	if (!connected)
		return;
	if (_pin.pin >= NUM_DIGITAL_PINS) {
		debug("invalid pin for arch_set_pin_motor: %d (max %d)", _pin.pin, NUM_DIGITAL_PINS);
		return;
	}
	if (s < 0 || s >= NUM_SPACES || m < 0 || m >= spaces[s].num_motors)
		m = 255;
	else {
		for (int i = 0; i < s; ++i)
			m += spaces[s].num_motors;
	}
	if (m != avr_pins[_pin.pin].motor || ticks != avr_pins[_pin.pin].ticks) {
		avr_pins[_pin.pin].motor = m;
		avr_pins[_pin.pin].ticks = ticks;
		avr_send_pin(_pin);
	}
	//debug("set motor for pin %d to %d", _pin.pin, m);
} // }}}
// }}}

// Setup hooks. {{{
void arch_reset() { // {{{
	// Initialize connection.
	if (avr_pong == 7) {
		debug("reset ignored");
		return;
	}
	// Wait for reset to complete.
	sleep(2);
	last_micros = utime();
	avr_serial.write(CMD_ACK1);
	avr_serial.write(CMD_ACK2);
	avr_serial.write(CMD_ACK3);
	avr_serial.write(CMD_ACK0);
	avr_serial.write(CMD_ACK1);
	avr_serial.write(CMD_ACK2);
	avr_serial.write(CMD_ACK3);
	avr_serial.write(CMD_STALLACK);
	// Just in case the controller was reset: reclaim port by requesting ID.
	avr_serial.write(CMD_ID);
	avr_call1(HWC_PING, 0);
	avr_call1(HWC_PING, 1);
	avr_call1(HWC_PING, 2);
	avr_call1(HWC_PING, 3);
	avr_serial.write(CMD_ID);
	avr_call1(HWC_PING, 4);
	avr_call1(HWC_PING, 5);
	avr_call1(HWC_PING, 6);
	avr_call1(HWC_PING, 7);
	int32_t before = millis();
	while (avr_pong != 7 && millis() - before < 2000)
		serial_wait(100);
	if (avr_pong != 7) {
		debug("no pong seen; giving up.\n");
		abort();
	}
	if (avr_uuid_dirty) {
		avr_uuid_dirty = false;
		arch_set_uuid();
	}
} // }}}

enum MotorFlags { // {{{
	// active
	INVERT_STEP		= 0x02,
	PATTERN			= 0x04,
	// current dir
	// current step
	// limit
	INVERT_LIMIT_MIN	= 0x40,
	INVERT_LIMIT_MAX	= 0x80,
}; // }}}

void arch_motor_change(uint8_t s, uint8_t sm) { // {{{
	Motor &mtr = *spaces[s].motor[sm];
	if (!connected)
		return;
	uint8_t m = sm;
	for (uint8_t st = 0; st < s; ++st)
		m += spaces[st].num_motors;
	// Motor setup packet:
	// 0: MSETUP
	// 1: motor id
	// 2: step pin
	// 3: dir pin
	// 4: min limit pin
	// 5: max limit pin
	// 6: follow id
	// 7: flags
	avr_buffer[0] = HWC_MSETUP;
	avr_buffer[1] = m;
	//debug("arch motor change %d %d %d %x", s, sm, m, p);
	avr_buffer[2] = (mtr.step_pin.valid() ? mtr.step_pin.pin : ~0);
	avr_buffer[3] = (mtr.dir_pin.valid() ? mtr.dir_pin.pin : ~0);
	bool mininvert, maxinvert;
	if (mtr.dir_pin.inverted()) {
		avr_buffer[4] = (mtr.limit_max_pin.valid() ? mtr.limit_max_pin.pin : ~0);
		avr_buffer[5] = (mtr.limit_min_pin.valid() ? mtr.limit_min_pin.pin : ~0);
		mininvert = mtr.limit_max_pin.inverted();
		maxinvert = mtr.limit_min_pin.inverted();
	}
	else {
		avr_buffer[4] = (mtr.limit_min_pin.valid() ? mtr.limit_min_pin.pin : ~0);
		avr_buffer[5] = (mtr.limit_max_pin.valid() ? mtr.limit_max_pin.pin : ~0);
		mininvert = mtr.limit_min_pin.inverted();
		maxinvert = mtr.limit_max_pin.inverted();
	}
	if (s != 2)
		avr_buffer[6] = 0xff;
	else {
		FollowerMotorData *data = reinterpret_cast <FollowerMotorData *>(mtr.type_data);
		if (data->space < 0 || data->space >= 1 || data->motor < 0 || data->motor >= spaces[data->space].num_motors)
			avr_buffer[6] = 0xff;
		else {
			int fm = data->motor;
			for (int i = 0; i < data->space; ++i)
				fm += spaces[i].num_motors;
			if (mtr.dir_pin.inverted() ^ spaces[data->space].motor[data->motor]->dir_pin.inverted())
				fm |= 0x80;
			avr_buffer[6] = fm;
		}
	}
	// Flags is a bitmask of:
	// 1: step pin is inverted.
	// 2: motor is pattern channel.
	// 6: min limit is inverted.
	// 7: max limit is inverted.
	// Other bits (0, 3, 4, 5) are for internal use by the firmware and are ignored.
	avr_buffer[7] = (mtr.step_pin.inverted() ? INVERT_STEP : 0) | (mininvert ? INVERT_LIMIT_MIN : 0) | (maxinvert ? INVERT_LIMIT_MAX : 0);
	prepare_packet(avr_buffer, 8);
	avr_send();
} // }}}

void arch_pattern_change() { // {{{
	if (!connected)
		return;
	uint8_t m = 0;
	for (uint8_t st = 0; st < NUM_SPACES; ++st)
		m += spaces[st].num_motors;
	// Motor setup packet:
	// 0: MSETUP
	// 1: motor id
	// 2: step pin
	// 3: dir pin
	// 4: min limit pin
	// 5: max limit pin
	// 6: follow id
	// 7: flags
	avr_buffer[0] = HWC_MSETUP;
	avr_buffer[1] = m;
	//debug("arch motor change %d %d %d %x", s, sm, m, p);
	avr_buffer[2] = (pattern.step_pin.valid() ? pattern.step_pin.pin : ~0);
	avr_buffer[3] = (pattern.dir_pin.valid() ? pattern.dir_pin.pin : ~0);
	avr_buffer[4] = ~0;
	avr_buffer[5] = ~0;
	avr_buffer[6] = 0xff;
	// Flags is a bitmask of:
	// 1: step pin is inverted.
	// 2: motor is pattern channel.
	// 6: min limit is inverted.
	// 7: max limit is inverted.
	// Other bits (0, 3, 4, 5) are for internal use by the firmware and are ignored.
	avr_buffer[7] = PATTERN | (pattern.step_pin.inverted() ? INVERT_STEP : 0);
	prepare_packet(avr_buffer, 8);
	avr_send();
} // }}}

void arch_change(bool motors) { // {{{
	int old_active_motors = avr_active_motors;
	bool pattern_valid = false;
	if (connected) {
		pattern_valid = pattern.step_pin.valid() || pattern.dir_pin.valid();
		avr_active_motors = pattern_valid ? 1 : 0;
		for (uint8_t s = 0; s < NUM_SPACES; ++s) {
			avr_active_motors += spaces[s].num_motors;
		}
		avr_buffer[0] = HWC_SETUP;
		avr_buffer[1] = avr_active_motors;
		for (int i = 0; i < 4; ++i)
			avr_buffer[2 + i] = (settings.hwtime_step >> (8 * i)) & 0xff;
		avr_buffer[6] = led_pin.valid() ? led_pin.pin : ~0;
		avr_buffer[7] = stop_pin.valid() ? stop_pin.pin : ~0;
		avr_buffer[8] = probe_pin.valid() ? probe_pin.pin : ~0;
		avr_buffer[9] = (led_pin.inverted() ? 1 : 0) | (probe_pin.inverted() ? 2 : 0) | (stop_pin.inverted() ? 4 : 0) | (spiss_pin.inverted() ? 8 : 0);
		avr_buffer[10] = timeout & 0xff;
		avr_buffer[11] = (timeout >> 8) & 0xff;
		avr_buffer[12] = spiss_pin.valid() ? spiss_pin.pin : ~0;
		prepare_packet(avr_buffer, 13);
		avr_send();
	}
	if (motors) {
		for (uint8_t s = 0; s < NUM_SPACES; ++s) {
			for (uint8_t m = 0; m < spaces[s].num_motors; ++m) {
				arch_motor_change(s, m);
			}
		}
		if (pattern_valid)
			arch_pattern_change();
		if (connected) {
			for (int m = old_active_motors; m < avr_active_motors; ++m) {
				avr_buffer[0] = HWC_MSETUP;
				avr_buffer[1] = m;
				//debug("arch motor change %d %d %d %x", s, sm, m, p);
				avr_buffer[2] = ~0;
				avr_buffer[3] = ~0;
				avr_buffer[4] = ~0;
				avr_buffer[5] = ~0;
				avr_buffer[6] = ~0;
				avr_buffer[7] = 0;
				prepare_packet(avr_buffer, 8);
				avr_send();
			}
		}
	}
} // }}}

void arch_motors_change() { // {{{
	if (preparing || out_busy >= 3) {
		change_pending = true;
		return;
	}
	change_pending = false;
	arch_change(true);
} // }}}

void arch_globals_change() { // {{{
	arch_change(false);
} // }}}

void arch_setup_start() { // {{{
	// Set up arch variables.
	avr_running = false;
	avr_homing = false;
	avr_filling = false;
	NUM_PINS = 0;
	NUM_ANALOG_INPUTS = 0;
	avr_pong = 254;
	avr_limiter_space = -1;
	avr_limiter_motor = 0;
	avr_active_motors = 0;
	avr_uuid_dirty = false;
	// Set up serial port.
	connected = false;
	serialdev = &avr_serial;
} // }}}

void arch_setup_end() {
	// Nothing to do.
}

void arch_set_uuid() { // {{{
	if (!connected) {
		avr_uuid_dirty = true;
		return;
	}
	avr_buffer[0] = HWC_SET_UUID;
	for (uint8_t i = 0; i < UUID_SIZE; ++i)
		avr_buffer[1 + i] = shmem->uuid[i];
	prepare_packet(avr_buffer, 1 + UUID_SIZE);
	avr_send();
} // }}}

static void avr_connect3();
static int avr_next_pin_name;

void arch_send_pin_name(int pin) { // {{{
	prepare_interrupt();
	memcpy(const_cast <char *>(shmem->interrupt_str), avr_pin_name[pin], avr_pin_name_len[pin] + 1);
	shmem->interrupt_ints[0] = pin;
	shmem->interrupt_ints[1] = avr_pin_name_len[pin];
	send_to_parent(CMD_PINNAME);
} // }}}

static void avr_connect4() { // {{{
	while (out_busy >= 3)
		serial_wait();
	avr_pin_name_len[avr_next_pin_name] = command[1];
	avr_pin_name[avr_next_pin_name] = new char[command[1] + 1];
	memcpy(&avr_pin_name[avr_next_pin_name][1], &command[2], command[1]);
	if (avr_next_pin_name < NUM_DIGITAL_PINS)
		avr_pin_name[avr_next_pin_name][0] = 7;
	else
		avr_pin_name[avr_next_pin_name][0] = 8;
	avr_write_ack("pin name");
	arch_send_pin_name(avr_next_pin_name);
	avr_next_pin_name += 1;
	avr_connect3();
} // }}}

static void avr_connect3() { // {{{
	//debug("sending pin %d name", avr_next_pin_name);
	if (avr_next_pin_name >= NUM_PINS) {
		for (int s = 0; s < NUM_SPACES; ++s) {
			Space &sp = spaces[s];
			for (int m = 0; m < sp.num_motors; ++m) {
				DATA_DELETE(s, m);
				ARCH_NEW_MOTOR(s, m, sp.motor);
			}
		}
		delete[] pattern.avr_data.buffer;
		pattern.avr_data.buffer = new AVR_BUFFER_DATA_TYPE[BYTES_PER_FRAGMENT / sizeof(AVR_BUFFER_DATA_TYPE)];
		connect_end();
		arch_change(true);
		return;
	}
	avr_buffer[0] = HWC_PINNAME;
	avr_buffer[1] = avr_next_pin_name < NUM_DIGITAL_PINS ? avr_next_pin_name : (avr_next_pin_name - NUM_DIGITAL_PINS) | 0x80;
	wait_for_reply[expected_replies++] = avr_connect4;
	prepare_packet(avr_buffer, 2);
	avr_send();
} // }}}

void avr_connect2() { // {{{
	protocol_version = 0;
	for (uint8_t i = 0; i < sizeof(uint32_t); ++i)
		protocol_version |= int(uint8_t(command[2 + i])) << (i * 8);
	check_protocol();
	if (!connected)
		return;
	NUM_DIGITAL_PINS = command[6];
	NUM_ANALOG_INPUTS = command[7];
	NUM_PINS = NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS;
	NUM_MOTORS = command[8];
	FRAGMENTS_PER_BUFFER = command[9];
	BYTES_PER_FRAGMENT = command[10];
	TIME_PER_ISR = uint8_t(command[11]) | uint8_t(command[12]) << 8;
	//id[0][:8] + '-' + id[0][8:12] + '-' + id[0][12:16] + '-' + id[0][16:20] + '-' + id[0][20:32]
	for (int i = 0; i < UUID_SIZE; ++i)
		shmem->uuid[i] = command[11 + i];
	avr_write_ack("setup");
	avr_control_queue = new uint8_t[NUM_DIGITAL_PINS * CONTROL_SIZE];
	avr_in_control_queue = new bool[NUM_DIGITAL_PINS];
	avr_control_queue_length = 0;
	avr_pong = 255;	// Choke on reset again.
	avr_pins = new Avr_pin_t[NUM_DIGITAL_PINS];
	for (int i = 0; i < NUM_DIGITAL_PINS; ++i) {
		avr_pins[i].reset = 3;	// INPUT_NOPULLUP.
		avr_pins[i].state = avr_pins[i].reset;
		avr_pins[i].duty = 0x7fff;
		avr_pins[i].motor = 0xff;
		avr_in_control_queue[i] = false;
	}
	avr_adc_id = new int[NUM_ANALOG_INPUTS];
	for (int i = 0; i < NUM_ANALOG_INPUTS; ++i)
		avr_adc_id[i] = ~0;
	avr_pos_offset = new double[NUM_MOTORS];
	for (int m = 0; m < NUM_MOTORS; ++m)
		avr_pos_offset[m] = 0;
	avr_next_pin_name = 0;
	avr_pin_name_len = new int[NUM_PINS];
	avr_pin_name = new char *[NUM_PINS];
	avr_connect3();
} // }}}

void arch_connect(char const *run_id, char const *port) { // {{{
	connected = true;
	avr_serial.begin(port);
	if (!connected)
		return;
	arch_reset();
	// Get constants.
	avr_buffer[0] = HWC_BEGIN;
	// Send packet size. Required by older protocols.
	avr_buffer[1] = 10;
	for (int i = 0; i < ID_SIZE; ++i)
		avr_buffer[2 + i] = run_id[i];
	wait_for_reply[expected_replies++] = avr_connect2;
	prepare_packet(avr_buffer, 10);
	avr_send();
} // }}}

void arch_request_temp(int which) { // {{{
	if (connected && which >= 0 && which < num_temps && temps[which].thermistor_pin.pin >= NUM_DIGITAL_PINS && temps[which].thermistor_pin.pin < NUM_PINS) {
		if (interrupt_pending) {
			requested_temp = ~0;
			shmem->floats[0] = temps[which].fromadc(temps[which].last_value);
			delayed_reply();
		}
		else
			requested_temp = which;
	}
	else {
		requested_temp = ~0;
		shmem->floats[0] = NAN;
		delayed_reply();
	}
} // }}}

void arch_setup_temp(int id, int thermistor_pin, bool active, int heater_pin, bool heater_invert, int heater_adctemp, int heater_limit_l, int heater_limit_h, int fan_pin, bool fan_invert, int fan_adctemp, int fan_limit_l, int fan_limit_h, double hold_time) { // {{{
	if (!connected)
		return;
	if (thermistor_pin < NUM_DIGITAL_PINS || thermistor_pin >= NUM_PINS) {
		debug("setup for invalid adc %d requested", thermistor_pin);
		return;
	}
	// Make sure the controls for the heater and fan have been sent, otherwise they override this.
	try_send_control();
	while (out_busy >= 3 || avr_control_queue_length > 0) {
		serial_wait();
		try_send_control();
	}
	thermistor_pin -= NUM_DIGITAL_PINS;
	avr_adc_id[thermistor_pin] = id;
	avr_buffer[0] = HWC_ASETUP;
	avr_buffer[1] = thermistor_pin;
	avr_buffer[2] = heater_pin;
	avr_buffer[3] = fan_pin;
	int32_t th, tf, lh[2], lf[2];
	if (active) {
		int hi = heater_invert ? 0x4000 : 0;
		int fi = fan_invert ? 0x4000 : 0;
		th = (min(0x3fff, max(0, heater_adctemp))) | hi;
		tf = (min(0x3fff, max(0, fan_adctemp))) | fi;
		lh[0] = (heater_limit_l & 0x3fff) | hi;
		lh[1] = (heater_limit_h & 0x3fff) | hi;
		lf[0] = (fan_limit_l & 0x3fff) | fi;
		lf[1] = (fan_limit_h & 0x3fff) | fi;
	}
	else {
		if (id == requested_temp) {
			shmem->floats[0] = NAN;
			delayed_reply();
			requested_temp = ~0;
		}
		th = 0xffff;
		tf = 0xffff;
		lh[0] = 0x3fff;
		lh[1] = 0x3fff;
		lf[0] = 0x3fff;
		lf[1] = 0x3fff;
	}
	//debug("setup adc %d 0x%x 0x%x -> %x %x", id, heater_adctemp, fan_adctemp, th, tf);
	avr_buffer[4] = lh[0] & 0xff;
	avr_buffer[5] = (lh[0] >> 8) & 0xff;
	avr_buffer[6] = lf[0] & 0xff;
	avr_buffer[7] = (lf[0] >> 8) & 0xff;
	avr_buffer[8] = lh[1] & 0xff;
	avr_buffer[9] = (lh[1] >> 8) & 0xff;
	avr_buffer[10] = lf[1] & 0xff;
	avr_buffer[11] = (lf[1] >> 8) & 0xff;
	avr_buffer[12] = th & 0xff;
	avr_buffer[13] = (th >> 8) & 0xff;
	avr_buffer[14] = tf & 0xff;
	avr_buffer[15] = (tf >> 8) & 0xff;
	uint16_t hold_time_ms = hold_time * 1000;
	avr_buffer[16] = hold_time_ms & 0xff;
	avr_buffer[17] = (hold_time_ms >> 8) & 0xff;
	prepare_packet(avr_buffer, 18);
	avr_send();
} // }}}

void arch_disconnect() { // {{{
	connected = false;
	avr_serial.end();
	if (requested_temp != uint8_t(~0)) {
		shmem->floats[0] = NAN;
		delayed_reply();
		requested_temp = ~0;
	}
} // }}}

int arch_fds() { // {{{
	return connected ? 1 : 0;
} // }}}

void arch_reconnect(const char *port) { // {{{
	connected = true;
	avr_serial.begin(port);
	if (!connected)
		return;
	for (int i = 0; i < 4; ++i)
		avr_serial.write(cmd_nack[i]);	// Just to be sure.
} // }}}
// }}}

// Running hooks. {{{
int arch_tick() { // {{{
	if (connected) {
		serial(true);
		return 500;
	}
	return -1;
} // }}}

void arch_addpos(int s, int m, double diff) { // {{{
	if (s >= NUM_SPACES)
		return;
	int mi = 0;
	for (uint8_t st = 0; st < s; ++st)
		mi += spaces[st].num_motors;
	if (mi + m >= NUM_MOTORS)
		return;
	if (!std::isnan(diff))
		avr_pos_offset[mi + m] -= diff;
	else {
		debug("Error: addpos called with NaN argument");
		abort();
	}
	cpdebug(s, m, "arch addpos diff %f offset %f pos %f", diff, avr_pos_offset[mi], spaces[s].motor[m]->settings.current_pos);
} // }}}

void arch_change_steps_per_unit(int s, int m, double factor) { // {{{
	if (s >= NUM_SPACES)
		return;
	int mi = 0;
	for (uint8_t st = 0; st < s; ++st)
		mi += spaces[st].num_motors;
	if (mi + m >= NUM_MOTORS)
		return;
	if (!std::isnan(factor))
		avr_pos_offset[mi + m] *= factor;
	else {
		debug("Error: change_steps_per_unit called with NaN argument");
		abort();
	}
	cpdebug(s, m, "arch change_steps_per_unit factor %f offset %f pos %f", factor, avr_pos_offset[mi], spaces[s].motor[m]->settings.current_pos);
} // }}}

void arch_invertpos(int s, int m) { // {{{
	if (s >= NUM_SPACES)
		return;
	int mi = 0;
	for (uint8_t st = 0; st < s; ++st)
		mi += spaces[st].num_motors;
	if (mi + m >= NUM_MOTORS)
		return;
	spaces[s].motor[m]->settings.hw_pos *= -1;
	if (std::isnan(spaces[s].motor[m]->settings.current_pos))
		return;
	avr_pos_offset[mi + m] = -2 * spaces[s].motor[m]->settings.current_pos - avr_pos_offset[mi + m];
} // }}}

void arch_stop(bool fake) { // {{{
	if (!connected) {
		stop_pending = true;
		return;
	}
	//debug("blocking host");
	host_block = true;
	if (preparing || out_busy >= 3) {
		//debug("not yet stopping");
		stop_pending = true;
		return;
	}
	stop_pending = false;
	if (!avr_running && !avr_homing) {
		//debug("not running, so not stopping");
		current_fragment_pos = 0;
		computing_move = false;	// Not running, but preparations could have started.
		//debug("no longer blocking host");
		host_block = false;
		return;
	}
	avr_running = false;
	avr_homing = false;
	avr_buffer[0] = HWC_STOP;
	wait_for_reply[expected_replies++] = avr_stop2;
	avr_stop_fake = fake;
	prepare_packet(avr_buffer, 1);
	avr_send();
} // }}}

void avr_stop2() { // {{{
	if (!avr_stop_fake)
		abort_move(command[2] / 2);
	avr_get_current_pos(3, false);
	current_fragment = running_fragment;
	//debug("current_fragment = running_fragment; %d", current_fragment);
	current_fragment_pos = 0;
	num_active_motors = 0;
	//debug("no longer blocking host 2");
	host_block = false;
	avr_write_ack("stop");
} // }}}

static void avr_sent_fragment() { // {{{
	if (sending_fragment == 0) {
		//debug("calling avr_sent_fragment with zero sending_fragment");
		return;
	}
	if (stopping)
		return;
	sending_fragment -= 1;
} // }}}

bool arch_send_fragment() { // {{{
	if (!connected || host_block || stopping || discarding != 0 || stop_pending) {
		//debug("not sending arch frag block %d stop %d discard %d stop pending %d", host_block, stopping, discarding, stop_pending);
		return false;
	}
	while (out_busy >= 3)
		serial_wait();
	if (!connected || host_block || stopping || discarding != 0 || stop_pending)
		return false;
	avr_buffer[0] = probing ? HWC_START_PROBE : HWC_START_MOVE;
	//debug("send fragment current-fragment-pos=%d current-fragment=%d active-moters=%d running=%d num-running=0x%x", current_fragment_pos, current_fragment, num_active_motors, running_fragment, (current_fragment - running_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER);
	avr_buffer[1] = current_fragment_pos;
	avr_buffer[2] = num_active_motors;
	sending_fragment = num_active_motors + 1;
	if (prepare_packet(avr_buffer, 3)) {
		transmitting_fragment = true;
		avr_cb = &avr_sent_fragment;
		avr_send();
		int mi = 0;
		avr_filling = true;
		int cfp = current_fragment_pos;
		for (int s = 0; connected && !host_block && !stopping && discarding == 0 && !stop_pending && s < NUM_SPACES; mi += spaces[s++].num_motors) {
			for (uint8_t m = 0; !host_block && !stopping && discarding == 0 && !stop_pending && m < spaces[s].num_motors; ++m) {
				if (!spaces[s].motor[m]->active)
					continue;
				cpdebug(s, m, "sending %d %d", current_fragment, current_fragment_pos);
				//debug("sending %d %d cf %d cp 0x%x", s, m, current_fragment, current_fragment_pos);
				while (out_busy >= 3)
					serial_wait();
				if (stop_pending || discarding != 0)
					break;
				avr_buffer[0] = single ? HWC_MOVE_SINGLE : HWC_MOVE;
				avr_buffer[1] = mi + m;
				for (int i = 0; i < cfp; ++i) {
					int value = spaces[s].motor[m]->avr_data.buffer[i];
					avr_buffer[2 + i] = value;
				}
				if (prepare_packet(avr_buffer, 2 + cfp)) {
					avr_cb = &avr_sent_fragment;
					avr_send();
				}
				else
					break;
			}
		}
		if (sending_fragment > 0 && !host_block && !stopping && discarding == 0 && !stop_pending && pattern.active) {
			while (out_busy >= 3)
				serial_wait();
			if (!stop_pending && !stopping && discarding != 0) {
				avr_buffer[0] = single ? HWC_MOVE_SINGLE : HWC_MOVE;
				avr_buffer[1] = mi;
				for (int i = 0; i < cfp; ++i)
					avr_buffer[2 + i] = pattern.avr_data.buffer[i];
				if (prepare_packet(avr_buffer, 2 + cfp)) {
					avr_cb = &avr_sent_fragment;
					avr_send();
				}
			}
		}
		while (sending_fragment > 0 && !host_block && !stopping && discarding == 0 && !stop_pending)
			serial_wait();
		transmitting_fragment = false;
	}
	sending_fragment = 0;
	avr_filling = false;
	return !host_block && !stopping && discarding == 0 && !stop_pending;
} // }}}

void arch_start_move(int extra) { // {{{
	if (host_block)
		return;
	if (!connected || preparing || sending_fragment || out_busy >= 3) {
		//debug("no start yet");
		start_pending = true;
		return;
	}
	if (avr_running || avr_filling || stopping || avr_homing) {
		//debug("not startable");
		return;
	}
	if ((running_fragment - current_fragment + FRAGMENTS_PER_BUFFER) % FRAGMENTS_PER_BUFFER <= extra + 2) {
		//debug("no buffer no start");
		return;
	}
	//debug("start move %d %d %d %d", current_fragment, running_fragment, sending_fragment, extra);
	while (out_busy >= 3)
		serial_wait();
	start_pending = false;
	avr_running = true;
	avr_buffer[0] = HWC_START;
	if (prepare_packet(avr_buffer, 1))
		avr_send();
} // }}}

bool arch_running() { // {{{
	return avr_running;
} // }}}

void arch_home() { // {{{
	if (!connected)
		return;
	avr_homing = true;
	while (out_busy >= 3)
		serial_wait();
	avr_buffer[0] = HWC_HOME;
	int speed = 10000;	// μs/step.
	for (int i = 0; i < 4; ++i)
		avr_buffer[1 + i] = (speed >> (8 * i)) & 0xff;
	for (int m = 0; m < avr_active_motors; ++m) {
		if (m >= min(shmem->ints[0], spaces[0].num_motors))
			avr_buffer[5 + m] = 0;
		else if (abs(int8_t(shmem->ints[m + 1])) <= 1) {
			avr_buffer[5 + m] = (spaces[0].motor[m]->dir_pin.inverted() ? -1 : 1) * shmem->ints[m + 1];
		}
		else {
			debug("invalid code in home %d: %d", m, shmem->ints[m + 1]);
			return;
		}
	}
	if (prepare_packet(avr_buffer, 5 + avr_active_motors))
		avr_send();
} // }}}

static void avr_discard_done() { // {{{
	discarding = 0;
	//debug("discard done");
} // }}}

void arch_discard() { // {{{
	// Send a pending discard instruction to the firmware.
	if (!connected || avr_filling) {
		// Not now; it will be done later.
		return;
	}
	discard_pending = false;
	if (discarding == 0)
		return;
	while (out_busy >= 3)
		serial_wait();
	//debug("discard send");
	avr_buffer[0] = HWC_DISCARD;
	avr_buffer[1] = discarding;
	if (prepare_packet(avr_buffer, 2)) {
		avr_cb = &avr_discard_done;
		avr_send();
	}
	else {
		debug("prepare packet failed for discard");
		discarding = 0;
	}
	double motors[spaces[0].num_motors];
	double xyz[spaces[0].num_axes];
	for (int m = 0; m < spaces[0].num_motors; ++m)
		motors[m] = spaces[0].motor[m]->settings.current_pos;
	spaces[0].motors2xyz(motors, xyz);
	for (int a = 0; a < spaces[0].num_axes; ++a) {
		spaces[0].axis[a]->current = xyz[a];
		//debug("discard position %d: %f", a, xyz[a]);
	}
	settings.adjust = 0;
} // }}}

void arch_send_spi(int bits, const uint8_t *data) { // {{{
	if (!connected)
		return;
	while (out_busy >= 3)
		serial_wait();
	avr_buffer[0] = HWC_SPI;
	avr_buffer[1] = bits;
	for (int i = 0; i * 8 < bits; ++i)
		avr_buffer[2 + i] = data[i];
	if (prepare_packet(avr_buffer, 2 + (bits + 7) / 8))
		avr_send();
} // }}}
// }}}

// Debugging hooks. {{{
void START_DEBUG() { // {{{
	fprintf(stderr, "cdriver debug from firmware: ");
} // }}}

void DO_DEBUG(char c) { // {{{
	fprintf(stderr, "%c"
#ifdef DEBUG_AVRCOMM
				" "
#endif
				, c);
} // }}}

void END_DEBUG() { // {{{
	fprintf(stderr, "\n");
} // }}}
// }}}

// AVRSerial methods. {{{
void AVRSerial::begin(char const *port) { // {{{
	// Open serial port and prepare pollfd.
	//debug("opening %s", port);
	if (port[0] == '!') {
		int pipes[2];
		socketpair(AF_LOCAL, SOCK_STREAM, 0, pipes);
		pid_t pid = fork();
		if (!pid) {
			// Child.
			close(pipes[0]);
			dup2(pipes[1], 0);
			dup2(pipes[1], 1);
			close(pipes[1]);
			execlp(&port[1], &port[1], NULL);
			debug("execlp for child failed");
			abort();
		}
		// Parent.
		close(pipes[1]);
		fd = pipes[0];
	}
	else {
		fd = open(port, O_RDWR);
		if (fd < 0) {
			disconnect(true, "failed to open port %s: %s", port, strerror(errno));
			return;
		}
	}
	pollfds[BASE_FDS].fd = fd;
	pollfds[BASE_FDS].events = POLLIN | POLLPRI;
	pollfds[BASE_FDS].revents = 0;
	start = 0;
	end_ = 0;
	fcntl(fd, F_SETFL, O_NONBLOCK);
} // }}}

void AVRSerial::write(char c) { // {{{
#ifdef DEBUG_AVRCOMM
	debug("w\t%02x", c & 0xff);
#endif
	if (!connected) {
		debug("writing to serial while not connected");
		abort();
	}
	while (true) {
		errno = 0;
		int ret = ::write(fd, &c, 1);
		if (ret == 1)
			break;
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			disconnect(true, "write to avr failed: %d %s", ret, strerror(errno));
			return;	// This causes protocol errors during reconnect, but they will be handled.
		}
	}
} // }}}

void AVRSerial::refill() { // {{{
	start = 0;
	end_ = ::read(fd, buffer, sizeof(buffer));
	//debug("%s", strerror(errno));
	//debug("refill %d bytes", end_);
	if (end_ < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK)
			debug("read returned error: %s", strerror(errno));
		end_ = 0;
	}
	if (end_ == 0 && pollfds[BASE_FDS].revents) {
		disconnect(true, "EOF detected on serial port; waiting for reconnect.");
	}
	pollfds[BASE_FDS].revents = 0;
} // }}}

int AVRSerial::read() { // {{{
	while (true) {
		if (start == end_)
			refill();
		if (start != end_)
			break;
		disconnect(true, "eof on input; waiting for reconnect.");
	}
	int ret = buffer[start++];
#ifdef DEBUG_AVRCOMM
	debug("r %02x", ret & 0xff);
#endif
	return ret;
} // }}}
// }}}

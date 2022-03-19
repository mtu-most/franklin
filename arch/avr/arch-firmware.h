/* arch/avr/arch-firmware.h - avr specific parts for Franklin
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

#ifndef _ARCH_AVR_H
#define _ARCH_AVR_H
//#define pindebug debug
#define pindebug(...) do {} while (0)

EXTERN volatile uint32_t avr_time_h, avr_seconds_h, avr_seconds;
EXTERN volatile int8_t avr_last_serial;
EXTERN int8_t avr_which_serial;
EXTERN uint8_t avr_adc_last_pin;
EXTERN int timer1_pins[3];
EXTERN int num_timer1_pins;
EXTERN uint16_t timer1_top;
EXTERN uint8_t avr_outputs_last;
#ifndef NO_DEBUG
#define AVR_DEBUG_BITS 5
EXTERN volatile int avr_debug[1 << AVR_DEBUG_BITS];
EXTERN volatile int avr_debug_ptr;
#endif

static inline void arch_disable_isr() { // {{{
	TIMSK1 = 0;
} // }}}

static inline void arch_enable_isr() { // {{{
	TIMSK1 = 1 << OCIE1A;
} // }}}

void arch_set_speed(uint16_t us_per_sample);
void arch_serial_write(uint8_t data);
void arch_watchdog_enable();
uint8_t EEPROM_read(uint16_t addr);
void EEPROM_write(uint16_t addr, uint8_t value);
void arch_spi_start();
void arch_spi_send(uint8_t data, uint8_t bits);
void arch_spi_stop();
void arch_adc_start(uint8_t adcpin);
int8_t arch_pin_name(char *buffer_, bool digital, uint8_t pin_);

static inline uint16_t millis() { // {{{
	cli();
	uint8_t l = TCNT0;
	uint32_t h = avr_time_h;
	if (TIFR0 & (1 << TOV0)) {
		l = TCNT0;
		h += 0x100;
	}
	sei();
	// There are 125/8 timer ticks/ms, so divide the timer ticks by 125/8 to get ms.
	return ((h | l) << 3) / 125;
} // }}}

static inline uint16_t seconds() { // {{{
	return avr_seconds + (avr_time_h >> 8) / 15625;
} // }}}

inline void SET_OUTPUT(uint8_t pin_no) { // {{{
	if ((pin[pin_no].state & 0x3) == CTRL_SET || (pin[pin_no].state & 0x3) == CTRL_RESET)
		return;
	*pin[pin_no].avr_output &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_mode |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_RESET);
	pindebug("output pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline void SET_INPUT(uint8_t pin_no) { // {{{
	*pin[pin_no].avr_mode &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_output |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_INPUT | CTRL_NOTIFY);
	pindebug("input pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline void UNSET(uint8_t pin_no) { // {{{
	int timer = pgm_read_byte(digital_pin_to_timer_PGM + pin_no);
	if (timer != NOT_ON_TIMER) {
		Timer_data const *data = &timer_data[timer];
		*data->mode &= ~data->mode_mask;
		//debug("unset pin %d, OC%d%c mode to %x", pin_no, data->num, data->part, *data->mode);
	}
	*pin[pin_no].avr_mode &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_output &= ~pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_UNSET);
	pindebug("unset pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline void SET(uint8_t pin_no) { // {{{
	int timer = pgm_read_byte(digital_pin_to_timer_PGM + pin_no);
	if (timer != NOT_ON_TIMER) {
		Timer_data const *data = &timer_data[timer];
		if (pin[pin_no].duty < 0x7fff) {
			*data->mode |= data->mode_mask;
			//debug("set pin %d, OC%d%c mode to %x", pin_no, data->num, data->part, *data->mode);
			if (data->num == 1)
				update_timer1_pwm();
			else {
				if (timer_is_16bit(data->num)) {
					data->oc[1] = (pin[pin_no].duty >> 8) & 0xff;
					data->oc[0] = pin[pin_no].duty & 0xff;
				}
				else {
					*data->oc = (pin[pin_no].duty >> 7) & 0xff;
				}
			}
		}
		else {
			*data->mode &= ~data->mode_mask;
			//debug("set2 pin %d, OC%d%c mode to %x", pin_no, data->num, data->part, *data->mode);
		}
	}
	else if ((pin[pin_no].state & 0x3) == CTRL_SET)
		return;
	pin[pin_no].avr_on = true;
	pin[pin_no].avr_target = 0;
	*pin[pin_no].avr_output |= pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_mode |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_SET);
	pindebug("set pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline void RESET(uint8_t pin_no) { // {{{
	if ((pin[pin_no].state & 0x3) == CTRL_RESET)
		return;
	int timer = pgm_read_byte(digital_pin_to_timer_PGM + pin_no);
	if (timer != NOT_ON_TIMER) {
		Timer_data const *data = &timer_data[timer];
		*data->mode &= ~data->mode_mask;
		//debug("reset pin %d, OC%d%c mode to %x", pin_no, data->num, data->part, *data->mode);
	}
	*pin[pin_no].avr_output &= ~pin[pin_no].avr_bitmask;
	*pin[pin_no].avr_mode |= pin[pin_no].avr_bitmask;
	pin[pin_no].set_state((pin[pin_no].state & ~0x3) | CTRL_RESET);
	pindebug("reset pin %d %x %x %x", pin_no, int(pin[pin_no].avr_output), int(pin[pin_no].avr_mode), pin[pin_no].avr_bitmask);
} // }}}

inline bool GET(uint8_t pin_no) { // {{{
	return *pin[pin_no].avr_input & pin[pin_no].avr_bitmask;
} // }}}

#endif

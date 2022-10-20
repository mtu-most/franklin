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
#ifndef NO_SPI
void arch_spi_start();
void arch_spi_send(uint8_t data, uint8_t bits);
void arch_spi_stop();
#endif
void arch_adc_start(uint8_t adcpin);
int8_t arch_pin_name(char *buffer_, bool digital, uint8_t pin_);

#ifndef NO_PWM
struct Timer_data {
	volatile uint8_t *tccra, *oc;
	uint8_t tccra_mask;
	bool is_16bit;
	uint8_t num;
	uint8_t part;
};
#define TIMER_DATA(tccra_, oc_, tccra_mask_, is_16bit_, num_, part_) Timer_data { tccra: tccra_, oc: oc_, tccra_mask: tccra_mask_, is_16bit: is_16bit_, num: num_, part:part_ }
extern uint8_t timer_pins[];
extern Timer_data const timer_data[] PROGMEM;
uint8_t const num_timer_pins = 0
#ifdef OC1A_PIN
	+1
#endif
#ifdef OC1B_PIN
	+1
#endif
#ifdef OC1C_PIN
	+1
#endif

#ifdef OC0A_PIN
	+1
#endif
#ifdef OC0B_PIN
	+1
#endif

#ifdef OC2A_PIN
	+1
#endif
#ifdef OC2B_PIN
	+1
#endif

#ifdef OC3A_PIN
	+1
#endif
#ifdef OC3B_PIN
	+1
#endif
#ifdef OC3C_PIN
	+1
#endif

#ifdef OC4A_PIN
	+1
#endif
#ifdef OC4B_PIN
	+1
#endif
#ifdef OC4C_PIN
	+1
#endif

#ifdef OC5A_PIN
	+1
#endif
#ifdef OC5B_PIN
	+1
#endif
#ifdef OC5C_PIN
	+1
#endif
	;

static Timer_data const *get_timer(uint8_t pin_no) {
	for (uint8_t i = 0; i < num_timer_pins; ++i) {
		if (timer_pins[i] == pin_no)
			return &timer_data[i];
	}
	return nullptr;
}
#endif

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
	if ((pin[pin_no - GPIO_FIRST_PIN].state & 0x3) == CTRL_SET || (pin[pin_no - GPIO_FIRST_PIN].state & 0x3) == CTRL_RESET)
		return;
	Gpio::PORT(pin_no >> 3) &= ~_BV(pin_no & 7);
	Gpio::DDR(pin_no >> 3) |= _BV(pin_no & 7);
	pin[pin_no - GPIO_FIRST_PIN].set_state((pin[pin_no - GPIO_FIRST_PIN].state & ~0x3) | CTRL_RESET);
} // }}}

inline void SET_INPUT(uint8_t pin_no) { // {{{
	Gpio::DDR(pin_no >> 3) &= ~_BV(pin_no & 7);
	Gpio::PORT(pin_no >> 3) |= _BV(pin_no & 7);
	pin[pin_no - GPIO_FIRST_PIN].set_state((pin[pin_no - GPIO_FIRST_PIN].state & ~0x3) | CTRL_INPUT | CTRL_NOTIFY);
} // }}}

inline void UNSET(uint8_t pin_no) { // {{{
#ifndef NO_PWM
	Timer_data const *data = get_timer(pin_no);
	if (data != nullptr) {
		*reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&data->tccra)) &= ~pgm_read_byte(&data->tccra_mask);
	}
#endif
	Gpio::DDR(pin_no >> 3) &= ~_BV(pin_no & 7);
	Gpio::PORT(pin_no >> 3) &= ~_BV(pin_no & 7);
	pin[pin_no - GPIO_FIRST_PIN].set_state((pin[pin_no - GPIO_FIRST_PIN].state & ~0x3) | CTRL_UNSET);
} // }}}

inline static bool is_oc1_pin(uint8_t pin_no) { // {{{
	switch (pin_no) {
#ifdef OC1A_PIN
	case OC1A_PIN:
#endif
#ifdef OC1B_PIN
	case OC1B_PIN:
#endif
#ifdef OC1C_PIN
	case OC1C_PIN:
#endif
		return true;
	default:
		return false;
	}
} // }}}

inline void SET(uint8_t pin_no) { // {{{
#ifndef NO_PWM
	Timer_data const *data = get_timer(pin_no);
	if (data != nullptr) {
		if (pin[pin_no - GPIO_FIRST_PIN].duty < 0x7fff) {
			*reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&data->tccra)) |= pgm_read_byte(&data->tccra_mask);
			if (is_oc1_pin(pin_no))
				update_timer1_pwm();
			else {
				if (pgm_read_byte(&data->is_16bit)) {
					reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&data->oc))[1] = (pin[pin_no - GPIO_FIRST_PIN].duty >> 8) & 0xff;
					reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&data->oc))[0] = pin[pin_no - GPIO_FIRST_PIN].duty & 0xff;
				}
				else {
					*reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&data->oc)) = (pin[pin_no - GPIO_FIRST_PIN].duty >> 7) & 0xff;
				}
			}
		}
		else {
			volatile uint8_t *reg = reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&data->tccra));
			uint8_t mask = pgm_read_byte(&data->tccra_mask);
			*reg &= ~mask;
		}
	}
	else
#endif
		if ((pin[pin_no - GPIO_FIRST_PIN].state & 0x3) == CTRL_SET)
			return;
#ifndef NO_PWM
	pin[pin_no - GPIO_FIRST_PIN].avr_on = true;
	pin[pin_no - GPIO_FIRST_PIN].avr_target = 0;
#endif
	Gpio::PORT(pin_no >> 3) |= _BV(pin_no & 7);
	Gpio::DDR(pin_no >> 3) |= _BV(pin_no & 7);
	pin[pin_no - GPIO_FIRST_PIN].set_state((pin[pin_no - GPIO_FIRST_PIN].state & ~0x3) | CTRL_SET);
} // }}}

inline void RESET(uint8_t pin_no) { // {{{
	if ((pin[pin_no - GPIO_FIRST_PIN].state & 0x3) == CTRL_RESET)
		return;
#ifndef NO_PWM
	Timer_data const *data = get_timer(pin_no);
	if (data != nullptr) {
		volatile uint8_t *reg = reinterpret_cast <volatile uint8_t *> (pgm_read_ptr(&data->tccra));
		uint8_t mask = pgm_read_byte(&data->tccra_mask);
		*reg &= ~mask;
	}
#endif
	Gpio::PORT(pin_no >> 3) &= ~_BV(pin_no & 7);
	Gpio::DDR(pin_no >> 3) |= _BV(pin_no & 7);
	pin[pin_no - GPIO_FIRST_PIN].set_state((pin[pin_no - GPIO_FIRST_PIN].state & ~0x3) | CTRL_RESET);
} // }}}

inline bool GET(uint8_t pin_no) { // {{{
	return Gpio::PIN(pin_no >> 3) & _BV(pin_no & 7);
} // }}}

#endif

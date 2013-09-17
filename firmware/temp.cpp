#include "firmware.h"

void Temp::load (uint16_t &addr, bool eeprom)
{
	alpha = read_float (addr, eeprom);
	beta = read_float (addr, eeprom);
	core_C = read_float (addr, eeprom);
	shell_C = read_float (addr, eeprom);
	transfer = read_float (addr, eeprom);
	radiation = read_float (addr, eeprom);
	power = read_float (addr, eeprom);
	power_pin = read_8 (addr, eeprom);
	thermistor_pin = read_8 (addr, eeprom);
	SET_OUTPUT (power_pin);
}

void Temp::save (uint16_t &addr, bool eeprom)
{
	write_float (addr, alpha, eeprom);
	write_float (addr, beta, eeprom);
	write_float (addr, core_C, eeprom);
	write_float (addr, shell_C, eeprom);
	write_float (addr, transfer, eeprom);
	write_float (addr, radiation, eeprom);
	write_float (addr, power, eeprom);
	write_8 (addr, power_pin, eeprom);
	write_8 (addr, thermistor_pin, eeprom);
}

float Temp::read () {
	if (thermistor_pin >= 255)
		return NAN;
	uint16_t adc = analogRead (thermistor_pin);
	// Compute R from adc.
	// adc = maxadc * R / (R + Rs)
	// adc/maxadc = 1 - Rs / (R + Rs)
	// 1 - adc/maxadc = Rs / (R + Rs)
	// 1 / (1 - adc/maxadc) = (R + Rs) / Rs = R / Rs + 1
	// R / Rs = 1 / (1 - adc/maxadc) - 1
	// R = Rs * (1 / (1 - maxadc/adc) - 1)
	// R = Rs * ((maxadc/adc) / (1 - maxadc/adc))
	// R = Rs / ((1 - maxadc/adc) / (maxadc/adc))
	// R = Rs / (1 / (maxadc/adc) - 1)
	// Compute T from R.
	// R = k * exp (beta / T)
	// T = beta / (ln (R / k))
	// T = beta / (ln ((Rs / (1 / (maxadc/adc) - 1)) / k))
	// T = beta / (ln (Rs / k) - ln (maxadc/adc - 1))
	// beta := ln (R0/R1) / (1/T0 - 1/T1) (using calibrated R0, R1, T0, T1)
	// k := Rc * exp (-beta / Tc) (using calibrated Rc, Tc)
	// alpha := ln (Rs / k) (using series resistance Rs from hardware)
	if (!isnan (alpha))
		return beta / (alpha - log (1024. / adc - 1));
	// alpha == NaN is used for calibration: return raw value.
	return adc;
}

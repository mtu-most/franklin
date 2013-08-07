#include "firmware.hh"

void Temp::load (uint16_t &addr, bool eeprom)
{
	alpha = read_float (addr, eeprom);
	beta = read_float (addr, eeprom);
	R0 = read_float (addr, eeprom);
	radiation = read_float (addr, eeprom);
	power = read_float (addr, eeprom);
	buffer_delay = read_16 (addr, eeprom);
	power_pin = read_8 (addr, eeprom);
	thermistor_pin = read_8 (addr, eeprom);
}

void Temp::save (uint16_t &addr, bool eeprom)
{
	write_float (addr, alpha, eeprom);
	write_float (addr, beta, eeprom);
	write_float (addr, R0, eeprom);
	write_float (addr, radiation, eeprom);
	write_float (addr, power, eeprom);
	write_16 (addr, buffer_delay, eeprom);
	write_8 (addr, power_pin, eeprom);
	write_8 (addr, thermistor_pin, eeprom);
}

float Temp::read () {
	if (thermistor_pin >= 255)
		return NAN;
	uint16_t adc = analogRead (thermistor_pin);
	// R = k * exp (-beta / T)
	// adc = maxadc * R / (R + R0)
	// alpha = beta * ln (k / R0)
	if (!isnan (alpha))
		return alpha + beta * log (1 - 1024. / adc);
	// alpha == NaN is used for debugging: return raw value.
	return adc;
}

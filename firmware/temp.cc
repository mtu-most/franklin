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
	// R = alpha * exp (-beta * T)
	// adc = maxadc * R / (R + R0) = maxadc * (1 - R0/(R + R0))
	// so
	//	1 - adc/maxadc = R0/(R0 + alpha * exp (-beta * T))
	//	R0 + alpha * exp (-beta * T) = R0/(1 - adc/maxadc)
	//	alpha * exp (-beta * T) = R0 * (maxadc/(maxadc - adc) - 1)
	//	alpha * exp (-beta * T) = R0 * (maxadc + adc)/(maxadc - adc)
	//	exp (-beta * T) = R0 * (maxadc + adc)/(alpha * (maxadc - adc))
	//	-beta * T = ln (R0 * (maxadc + adc)/(alpha * (maxadc - adc)))
	//	T = -ln (R0 * (maxadc + adc)/(alpha * (maxadc - adc))) / beta
	if (alpha > 0)
		return -log (R0 * (1023 + adc) / (alpha * (1023 - adc))) / beta;
	// alpha <= 0 is used for debugging: return raw value.
	return adc;
}

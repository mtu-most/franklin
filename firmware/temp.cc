#include "firmware.hh"

void Temp::load (uint16_t &addr, bool eeprom)
{
	beta = read_float (addr, eeprom);
	T0 = read_float (addr, eeprom);
	adc0 = read_float (addr, eeprom);
	radiation = read_float (addr, eeprom);
	power = read_float (addr, eeprom);
	buffer_delay = read_16 (addr, eeprom);
	power_pin = read_8 (addr, eeprom);
	thermistor_pin = read_8 (addr, eeprom);
}

void Temp::save (uint16_t &addr, bool eeprom)
{
	write_float (addr, beta, eeprom);
	write_float (addr, T0, eeprom);
	write_float (addr, adc0, eeprom);
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
	// adc = adc0 * exp (-beta * (T - T0))
	// so
	//	adc/adc0 = exp (-beta * (T-T0))
	//	ln (adc/adc0) = -beta*(T-T0)
	//	T = -ln(adc/adc0)/beta+T0
	if (adc0 > 0)
		return -log (adc / adc0) / beta + T0;
	// adc0 <= 0 is used for debugging: return raw value.
	return adc;
}

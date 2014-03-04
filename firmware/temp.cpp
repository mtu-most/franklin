#include "firmware.h"

void Temp::load (int16_t &addr, bool eeprom)
{
	R0 = read_float (addr, eeprom);
	R1 = read_float (addr, eeprom);
	Rc = read_float (addr, eeprom);
	Tc = read_float (addr, eeprom) + 273.15;
	minus_beta = -read_float (addr, eeprom);
#ifndef LOWMEM
	core_C = read_float (addr, eeprom);
	shell_C = read_float (addr, eeprom);
	transfer = read_float (addr, eeprom);
	radiation = read_float (addr, eeprom);
	power = read_float (addr, eeprom);
#else
	read_float (addr, eeprom);
	read_float (addr, eeprom);
	read_float (addr, eeprom);
	read_float (addr, eeprom);
	read_float (addr, eeprom);
#endif
	power_pin.read (read_16 (addr, eeprom));
	thermistor_pin.read (read_16 (addr, eeprom));
	SET_OUTPUT (power_pin);
}

void Temp::save (int16_t &addr, bool eeprom)
{
	write_float (addr, R0, eeprom);
	write_float (addr, R1, eeprom);
	write_float (addr, Rc, eeprom);
	write_float (addr, Tc - 273.15, eeprom);
	write_float (addr, -minus_beta, eeprom);
#ifndef LOWMEM
	write_float (addr, core_C, eeprom);
	write_float (addr, shell_C, eeprom);
	write_float (addr, transfer, eeprom);
	write_float (addr, radiation, eeprom);
	write_float (addr, power, eeprom);
#else
	write_float (addr, NAN, eeprom);
	write_float (addr, NAN, eeprom);
	write_float (addr, NAN, eeprom);
	write_float (addr, NAN, eeprom);
	write_float (addr, NAN, eeprom);
#endif
	write_16 (addr, power_pin.write (), eeprom);
	write_16 (addr, thermistor_pin.write (), eeprom);
}

float Temp::read () {
	if (thermistor_pin.invalid ())
		return NAN;
	int16_t adc = analogRead (thermistor_pin.pin);
	// Symbols: A[ms]: adc value, R[01s]: resistor value, V[m01s]: voltage
	// with m: maximum value, 0: series resistor, 1: parallel resistor, s: sensed value (thermistor)
	// Compute Rs from As:
	// As/Am = Vs/Vm = [1/(1/R1+1/R0)]/[R0+1/(1/R1+1/R0)]
	// Am/As = 1+R0/Rs+0/R1
	// => 1/Rs = R0*Am/As-1/R0-1/R1
	// Compute T from R (:= Rs).
	// R = k * exp (beta / T)
	// T = beta/log(R/k)=-beta/log(k*Am/R0/As-k/R0-k/R1)
	if (!isnan (minus_beta)) {
		float k = Rc * exp (minus_beta / Tc);
		return minus_beta / log (k * (1024 / R0 / adc - 1 / R0 - 1 / R1));
	}
	// beta == NaN is used for calibration: return raw value.
	// This is only used for reading, so correct for conversion.
	return adc + 273.15;
}

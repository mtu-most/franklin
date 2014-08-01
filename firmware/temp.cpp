#include "firmware.h"

#if MAXTEMPS > 0 || MAXEXTRUDERS > 0
void Temp::load (int16_t &addr, bool eeprom)
{
	R0 = read_float (addr, eeprom);
	R1 = read_float (addr, eeprom);
	logRc = read_float (addr, eeprom);
	Tc = read_float (addr, eeprom);
	beta = read_float (addr, eeprom);
	K = exp (logRc - beta / Tc);
	//debug("K %f R0 %f R1 %f logRc %f Tc %f beta %f", F(K), F(R0), F(R1), F(logRc), F(Tc), F(beta));
#ifndef LOWMEM
#if MAXGPIOS > 0
	for (Gpio *gpio = gpios; gpio; gpio = gpio->next)
		gpio->adcvalue = toadc (gpio->value);
#endif
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
	write_float (addr, logRc, eeprom);
	write_float (addr, Tc, eeprom);
	write_float (addr, beta, eeprom);
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

void Temp::setup_read () {
	//debug("alt adc: %d", analogRead(thermistor_pin.pin));
	adc_start(thermistor_pin.pin);
	adc_phase = 2;
}

int16_t Temp::get_value () {
	if (!adc_ready(thermistor_pin.pin))
		return -1;
	adclast = adc_get(thermistor_pin.pin);
	//debug("adc: %d", adclast);
	return adclast;
}

float Temp::fromadc (int16_t adc) {
	if (adc < 0)
		return INFINITY;
	if (adc >= MAXINT)
		return NAN;
	//debug("adc: %d", adc);
	// Symbols: A[ms]: adc value, R[01s]: resistor value, V[m01s]: voltage
	// with m: maximum value, 0: series resistor, 1: parallel resistor, s: sensed value (thermistor)
	// Compute Rs from As:
	// As/Am = Vs/Vm = [1/(1/R1+1/R0)]/[R0+1/(1/R1+1/R0)]
	// Am/As = 1+R0/Rs+R0/R1
	// => 1/Rs = R0*Am/As-1/R0-1/R1
	// Compute T from R (:= Rs).
	// R = K * exp (beta / T)
	// T = beta/log(R/K)=-beta/log(K*Am/R0/As-K/R0-K/R1)
	if (isnan(beta)) {
		// beta == NAN is used for calibration: return raw value as K.
		return adc;
	}
	//debug("K: %f adc: %d beta: %f", F(K), adc, F(beta));
	return -beta / log(K * (1024 / R0 / adc - 1 / R0 - 1 / R1));
}

int16_t Temp::toadc (float T) {
	if (isnan(T) || T <= 0)
		return MAXINT;
	if (isinf(T) && T > 0)
		return -1;
	float Rs = K * exp (beta * 1. / T);
	//debug("K %f Rs %f R0 %f logRc %f Tc %f beta %f", F(K), F(Rs), F(R0), F(logRc), F(Tc), F(beta));
	return ((1 << 10) - 1) * Rs / (Rs + R0);
}
#endif

#include "firmware.h"

#ifdef HAVE_TEMPS
void Temp::load(int16_t &addr, bool eeprom)
{
	R0 = read_float(addr, eeprom);
	R1 = read_float(addr, eeprom);
	logRc = read_float(addr, eeprom);
	Tc = read_float(addr, eeprom);
	beta = read_float(addr, eeprom);
	K = exp(logRc - beta / Tc);
	//debug("K %f R0 %f R1 %f logRc %f Tc %f beta %f", F(K), F(R0), F(R1), F(logRc), F(Tc), F(beta));
#ifdef HAVE_GPIOS
	for (uint8_t gpio = following_gpios; gpio < num_gpios; gpio = gpios[gpio].next)
		gpios[gpio].adcvalue = toadc(gpios[gpio].value);
	if (following_gpios < num_gpios)
		adc_phase = 1;
#endif
	/*
	core_C = read_float(addr, eeprom);
	shell_C = read_float(addr, eeprom);
	transfer = read_float(addr, eeprom);
	radiation = read_float(addr, eeprom);
	power = read_float(addr, eeprom);
	*/
	power_pin.read(read_16(addr, eeprom));
	thermistor_pin.read(read_16(addr, eeprom));
	SET_OUTPUT(power_pin);
	if (is_on)
		SET(power_pin);
	else
		RESET(power_pin);
}

void Temp::save(int16_t &addr, bool eeprom)
{
	write_float(addr, R0, eeprom);
	write_float(addr, R1, eeprom);
	write_float(addr, logRc, eeprom);
	write_float(addr, Tc, eeprom);
	write_float(addr, beta, eeprom);
	/*
	write_float(addr, core_C, eeprom);
	write_float(addr, shell_C, eeprom);
	write_float(addr, transfer, eeprom);
	write_float(addr, radiation, eeprom);
	write_float(addr, power, eeprom);
	*/
	write_16(addr, power_pin.write(), eeprom);
	write_16(addr, thermistor_pin.write(), eeprom);
}

int16_t Temp::savesize0() {
	return 2 * 2 + sizeof(float) * 10;
}

int16_t Temp::get_value() {
	if (!thermistor_pin.valid())
		return MAXINT;
	if (!adc_ready(thermistor_pin.pin))
		return -1;
	adclast = adc_get(thermistor_pin.pin);
	//debug("adc: %d", adclast);
	return adclast;
}

float Temp::fromadc(int16_t adc) {
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
	// R = K * exp(beta / T)
	// T = beta/log(R/K)=-beta/log(K*Am/R0/As-K/R0-K/R1)
	if (isnan(beta)) {
		// beta == NAN is used for calibration: return raw value as K.
		return adc * R0 + R1;
	}
	//debug("K: %f adc: %d beta: %f", F(K), adc, F(beta));
	return -beta / log(K * (1024 / R0 / adc - 1 / R0 - 1 / R1));
}

int16_t Temp::toadc(float T) {
	if (isnan(T) || T <= 0)
		return MAXINT;
	if (isinf(T) && T > 0)
		return -1;
	if (isnan(beta)) {
		return T - R1 / R0;
	}
	float Rs = K * exp(beta * 1. / T);
	//debug("K %f Rs %f R0 %f logRc %f Tc %f beta %f", F(K), F(Rs), F(R0), F(logRc), F(Tc), F(beta));
	return ((1 << 10) - 1) * Rs / (Rs + R0);
}

void Temp::init() {
	R0 = NAN;
	R1 = INFINITY;
	logRc = NAN;
	Tc = 20 + 273.15;
	beta = NAN;
	/*
	core_C = NAN;
	shell_C = NAN;
	transfer = NAN;
	radiation = NAN;
	power = NAN;
	*/
	power_pin.flags = 0;
	power_pin.read(0);
	thermistor_pin.flags = 0;
	thermistor_pin.read(0);
	min_alarm = NAN;
	max_alarm = NAN;
	adcmin_alarm = -1;
	adcmax_alarm = MAXINT;
	alarm = false;
	target = NAN;
	adctarget = MAXINT;
#ifdef HAVE_GPIOS
	following_gpios = ~0;
#endif
	last_time = millis();
	time_on = 0;
	is_on = false;
	K = NAN;
}

void Temp::free() {
}

void Temp::copy(Temp &dst) {
	dst.R0 = R0;
	dst.R1 = R1;
	dst.logRc = logRc;
	dst.Tc = Tc;
	dst.beta = beta;
	/*
	dst.core_C = core_C;
	dst.shell_C = shell_C;
	dst.transfer = transfer;
	dst.radiation = radiation;
	dst.power = power;
	*/
	dst.power_pin.flags = 0;
	dst.power_pin.read(power_pin.write());
	dst.thermistor_pin.flags = 0;
	dst.thermistor_pin.read(thermistor_pin.write());
	dst.min_alarm = min_alarm;
	dst.max_alarm = max_alarm;
	dst.adcmin_alarm = adcmin_alarm;
	dst.adcmax_alarm = adcmax_alarm;
	dst.alarm = alarm;
	dst.target = target;
	dst.adctarget = adctarget;
#ifdef HAVE_GPIOS
	dst.following_gpios = following_gpios;
#endif
	dst.last_time = last_time;
	dst.time_on = time_on;
	dst.is_on = is_on;
	dst.K = K;
}
#endif

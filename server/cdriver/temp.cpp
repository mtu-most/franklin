/* temp.cpp - temp handling for Franklin
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

#include "cdriver.h"

void Temp::load(int id) {
	R0 = shmem->floats[0];
	R1 = shmem->floats[1];
	logRc = shmem->floats[2];
	Tc = shmem->floats[3];
	beta = shmem->floats[4];
	K = exp(logRc - beta / Tc);
	//debug("K %f R0 %f R1 %f logRc %f Tc %f beta %f", K, R0, R1, logRc, Tc, beta);
	/*
	core_C = shmem->floats[0];
	shell_C = shmem->floats[0];
	transfer = shmem->floats[0];
	radiation = shmem->floats[0];
	power = shmem->floats[0];
	*/
	if (power_pin[1].valid() && power_pin[1].write() != shmem->ints[2])
		arch_set_duty(power_pin[1], 1);
	power_pin[0].read(shmem->ints[1]);
	power_pin[1].read(shmem->ints[2]);
	int old_pin = thermistor_pin.write();
	int old_pin_pin = thermistor_pin.pin;
	bool old_valid = thermistor_pin.valid();
	thermistor_pin.read(shmem->ints[3]);
	target[1] = shmem->floats[5];
	adctarget[1] = toadc(target[1], MAXINT);
	fan_duty = shmem->floats[6];
	if (power_pin[1].valid())
		arch_set_duty(power_pin[1], fan_duty);
	for (int i = 0; i < 2; ++i) {
		limit[i][0] = shmem->floats[7 + 2 * i];
		limit[i][1] = shmem->floats[8 + 2 * i];
		int adc_i = std::isnan(beta) && R0 >= 0 ? 0 : 1;
		adclimit[i][adc_i] = toadc(limit[i][0], adc_i * MAXINT);
		adclimit[i][1 - adc_i] = toadc(limit[i][1], (1 - adc_i) * MAXINT);
		SET_OUTPUT(power_pin[i]);
	}
	last_change_time = millis();
	hold_time = shmem->floats[11];
	if (old_pin != thermistor_pin.write() && old_valid)
		arch_setup_temp(~0, old_pin_pin, false);
	if (thermistor_pin.valid()) {
		int llh = adclimit[0][0];
		int lhh = adclimit[0][1];
		int llf = adclimit[1][0];
		int lhf = adclimit[1][1];
		//debug("limits: %x %x %x %x", llh, lhh, llf, lhf);
		arch_setup_temp(id, thermistor_pin.pin, true, power_pin[0].valid() ? power_pin[0].pin : ~0, power_pin[0].inverted(), adctarget[0], llh, lhh, power_pin[1].valid() ? power_pin[1].pin : ~0, power_pin[1].inverted(), adctarget[1], llf, lhf, hold_time);
	}
}

void Temp::save() {
	shmem->ints[1] = power_pin[0].write();
	shmem->ints[2] = power_pin[1].write();
	shmem->ints[3] = thermistor_pin.write();
	shmem->floats[0] = R0;
	shmem->floats[1] = R1;
	shmem->floats[2] = logRc;
	shmem->floats[3] = Tc;
	shmem->floats[4] = beta;
	shmem->floats[5] = target[1];
	shmem->floats[6] = fan_duty;
	shmem->floats[7] = limit[0][0];
	shmem->floats[8] = limit[0][1];
	shmem->floats[9] = limit[1][0];
	shmem->floats[10] = limit[1][1];
	shmem->floats[11] = hold_time;
	/*
	shmem->floats[0] = core_C;
	shmem->floats[0] = shell_C;
	shmem->floats[0] = transfer;
	shmem->floats[0] = radiation;
	shmem->floats[0] = power;
	*/
}

double Temp::fromadc(int32_t adc) {
	if (adc >= MAXINT)
		return NAN;
	if (std::isnan(beta)) {
		// beta == NAN is used for calibration: return raw value as K.
		return adc * (R0 / 1000.) + (R1 / 1000.);
	}
	if (adc <= 0)
		return INFINITY;
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
	//debug("K: %f adc: %d beta: %f", K, adc, beta);
	return -beta / log(K * ((1 << ADCBITS) / R0 / adc - 1 / R0 - 1 / R1));
}

int32_t Temp::toadc(double T, int32_t default_) {
	if (std::isnan(T))
		return default_;
	if (std::isnan(beta))
		return (T - (R1 / 1000.)) / (R0 / 1000.);
	if (T < 0)
		return default_;
	if (std::isinf(T) && T > 0)
		return -1;
	double Rs = K * exp(beta * 1. / T);
	//debug("K %f Rs %f R0 %f logRc %f Tc %f beta %f", K, Rs, R0, logRc, Tc, beta);
	return ((1 << ADCBITS) - 1) * Rs / (Rs + R0);
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
	thermistor_pin.init();
	min_alarm = NAN;
	max_alarm = NAN;
	adcmin_alarm = -1;
	adcmax_alarm = MAXINT;
	fan_duty = 1;
	for (int i = 0; i < 2; ++i) {
		power_pin[i].init();
		target[i] = NAN;
		limit[i][0] = NAN;
		limit[i][1] = NAN;
		adctarget[i] = MAXINT;
		adclimit[i][0] = MAXINT;
		adclimit[i][1] = 0;
		is_on[i] = false;
	}
	following_gpios = ~0;
	last_temp_time = utime();
	time_on = 0;
	K = NAN;
	last_value = -1;
	hold_time = 0;
}

void Temp::free() {
	if (thermistor_pin.valid())
		arch_setup_temp(~0, thermistor_pin.pin, false);
	power_pin[0].read(0);
	power_pin[1].read(0);
	thermistor_pin.read(0);
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
	for (int i = 0; i < 2; ++i) {
		dst.power_pin[i].read(power_pin[i].write());
		dst.target[i] = target[i];
		dst.limit[i][0] = limit[i][0];
		dst.limit[i][1] = limit[i][1];
		dst.adctarget[i] = adctarget[i];
		dst.adclimit[i][0] = adclimit[i][0];
		dst.adclimit[i][1] = adclimit[i][1];
		dst.is_on[i] = is_on[i];
	}
	dst.thermistor_pin.read(thermistor_pin.write());
	dst.min_alarm = min_alarm;
	dst.max_alarm = max_alarm;
	dst.adcmin_alarm = adcmin_alarm;
	dst.adcmax_alarm = adcmax_alarm;
	dst.following_gpios = following_gpios;
	dst.last_temp_time = last_temp_time;
	dst.time_on = time_on;
	dst.K = K;
	dst.last_value = last_value;
}

void handle_temp(int id, int temp) { // {{{
	temps[id].last_value = temp;
	if (store_adc)
		fprintf(store_adc, "%d %d %f %d\n", millis(), id, temps[id].fromadc(temp), temp);
	if (requested_temp < num_temps && temps[requested_temp].thermistor_pin.pin == temps[id].thermistor_pin.pin) {
		//debug("replying temp");
		shmem->floats[0] = temps[requested_temp].fromadc(temp);
		requested_temp = ~0;
		delayed_reply();
	}
	//debug("temp for %d: %d", id, temp);
	// If an alarm should be triggered, do so.  Adc values are higher for lower temperatures.
	//debug("alarms: %d %d %d %d", id, temps[id].adcmin_alarm, temps[id].adcmax_alarm, temp);
	if (temps[id].adcmin_alarm >= temp || temps[id].adcmax_alarm <= temp) {
		//debug("alarm: %d %d %d %d", id, temps[id].adcmin_alarm, temps[id].adcmax_alarm, temp);
		temps[id].min_alarm = NAN;
		temps[id].max_alarm = NAN;
		temps[id].adcmin_alarm = -1;
		temps[id].adcmax_alarm = MAXINT;
		if (run_file_wait_temp) {
			run_file_wait_temp -= 1;
			run_file_fill_queue();
			buffer_refill();
		}
		else {
			prepare_interrupt();
			shmem->interrupt_ints[0] = id;
			send_to_parent(CMD_TEMPCB);
		}
	}
	/*
	// TODO: Make this work and decide on units.
	// We have model settings.
	int32_t dt = current_time - temps[id].last_temp_time;
	if (dt == 0)
		return;
	temps[id].last_temp_time = current_time;
	// Heater and core/shell transfer.
	if (temps[id].is_on)
		temps[id].core_T += temps[id].power / temps[id].core_C * dt;
	double Q = temps[id].transfer * (temps[id].core_T - temps[id].shell_T) * dt;
	temps[id].core_T -= Q / temps[id].core_C;
	temps[id].shell_T += Q / temps[id].shell_C;
	if (temps[id].is_on)
		temps[id].core_T += temps[id].power / temps[id].core_C * dt / 2;
	// Set shell to measured value.
	temps[id].shell_T = temp;
	// Add energy if required.
	double E = temps[id].core_T * temps[id].core_C + temps[id].shell_T * temps[id].shell_C;
	double T = E / (temps[id].core_C + temps[id].shell_C);
	// Set the pin to correct value.
	if (T < temps[id].target) {
		if (!temps[id].is_on) {
			SET(temps[id].power_pin);
			temps[id].is_on = true;
			++temps_busy;
		}
		else
			temps[id].time_on += current_time - temps[id].last_temp_time;
	}
	else {
		if (temps[id].is_on) {
			RESET(temps[id].power_pin);
			temps[id].is_on = false;
			temps[id].time_on += current_time - temps[id].last_temp_time;
			--temps_busy;
		}
	}
	*/
} // }}}

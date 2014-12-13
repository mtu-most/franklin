#include "cdriver.h"

void Gpio::load(uint8_t self, int32_t &addr)
{
	pin.read(read_16(addr));
	state = read_8(addr);
	// State:  0: off, 1: on, 2: pullup input, 3: disabled
	switch (state) {
	case 0:
		SET_OUTPUT(pin);
		RESET(pin);
		break;
	case 1:
		SET_OUTPUT(pin);
		SET(pin);
		break;
	case 2:
		SET_INPUT(pin);
		break;
	case 3:
		SET_INPUT_NOPULLUP(pin);
		break;
	}
}

void Gpio::save(int32_t &addr)
{
	write_16(addr, pin.write());
	write_8(addr, state);
}

int32_t Gpio::savesize0() {
	return 3;
}

void Gpio::init() {
	pin.init();
	state = 0;
}

void Gpio::free() {
	pin.read(0);
}

void Gpio::copy(Gpio &dst) {
	dst.pin.read(pin.write());
	dst.state = state;
}

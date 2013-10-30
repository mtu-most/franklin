#include "firmware.h"

#define FSIZE 30
#define Hz 8000

static unsigned long wait (unsigned long last)
{
	// Wait for the right time to set the next sample.
	while (true)
	{
		unsigned long now = micros ();
		if (now - last > 1000000 / Hz)
			return last + 1000000 / Hz;
	}
}

static int8_t play_sample (int8_t sample, int8_t mpos, uint32_t axes, uint32_t extruders)
{
	// Play the sample.
	if (mpos == sample)
		return mpos;
	int8_t step = 0;
	if (mpos < sample)
		step = 1;
	else
		step = -1;
	for (uint8_t a = 0; a < num_axes; ++a)
	{
		if (mpos < sample)
			SET (axis[a].motor.dir_pin);
		else
			RESET (axis[a].motor.dir_pin);
	}
	for (uint8_t e = 0; e < num_extruders; ++e)
	{
		if (mpos < sample)
			SET (extruder[e].motor.dir_pin);
		else
			RESET (extruder[e].motor.dir_pin);
	}
	while (mpos != sample)
	{
		for (uint8_t a = 0; a < num_axes; ++a)
		{
			if (axes & (1 << a))
			{
				SET (axis[a].motor.step_pin);
				RESET (axis[a].motor.step_pin);
			}
		}
		for (uint8_t e = 0; e < num_extruders; ++e)
		{
			if (extruders & (1 << e))
			{
				SET (extruder[e].motor.step_pin);
				RESET (extruder[e].motor.step_pin);
			}
		}
		mpos += step;
	}
	return mpos;
}

static void parse_data (uint8_t *src, int8_t *dst)
{
	dst[0] = src[0] & 0x7;
	dst[1] = (src[0] >> 3) & 0x7;
	dst[2] = (src[0] >> 6 | src[1] << 2) & 0x7;
	dst[3] = (src[1] >> 1) & 0x7;
	dst[4] = (src[1] >> 4) & 0x7;
	dst[5] = (src[1] >> 7 | src[2] << 1) & 0x7;
	dst[6] = (src[2] >> 2) & 0x7;
	dst[7] = (src[2] >> 5) & 0x7;
}

void play (int32_t size, uint32_t axes, uint32_t extruders)
{
	int32_t num_fragments = size / FSIZE;
	size = num_fragments * FSIZE;	// Round it down.
	int32_t next_fragment = 0;
	int32_t pos = 0;
	int8_t mpos = 0;
	// Immediately request the first two fragments.
	Serial.write (CMD_ACKWAIT);
	++next_fragment;
	Serial.write (CMD_ACKWAIT);
	++next_fragment;
	for (uint8_t a = 0; a < num_axes; ++a)
	{
		if (axes & (1 << a))
			RESET (axis[a].motor.enable_pin);
	}
	for (uint8_t e = 0; e < num_extruders; ++e)
	{
		if (extruders & (1 << e))
			RESET (extruder[e].motor.enable_pin);
	}
	unsigned long last_sample = micros ();
	while (pos < size)
	{
		// Decode the sample.
		uint8_t buffer[3];
		for (uint8_t t = 0; t < 3; ++t) {
			if (!Serial.available ()) {
				unsigned long long waitstart = millis ();
				while (!Serial.available ()) {
					if (millis () - waitstart > 100)
						break;
				}
				if (!Serial.available ()) {
					debug ("wtf? %d %d", int (pos), int (size));
					Serial.write (CMD_ACKWAIT);
					t = 0 - 1;
					continue;
				}
			}
			buffer[t] = Serial.read ();
		}
		int8_t target[8];
		parse_data (buffer, target);
		// Play the sample.
		for (uint8_t t = 0; t < 8; ++t)
		{
			last_sample = wait (last_sample);
			mpos = play_sample (target[t], mpos, axes, extruders);
		}
		pos += 3;
		if (pos / FSIZE != (pos - 3) / FSIZE && pos + FSIZE < size)
			Serial.write (CMD_ACKWAIT);
	}
	// Reset motor to its original position.
	mpos = play_sample (0, mpos, axes, extruders);
	last_active = millis ();
	Serial.write (CMD_ACK);
}

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

static uint8_t play_sample (Motor *m, uint8_t sample, uint8_t mpos)
{
	// Play the sample.
	if (mpos == sample)
		return mpos;
	if (mpos < sample)
	{
		SET (m->dir_pin);
		while (mpos < sample)
		{
			SET (m->step_pin);
			RESET (m->step_pin);
			mpos += 1;
		}
		return mpos;
	}
	RESET (m->dir_pin);
	while (mpos > sample)
	{
		SET (m->step_pin);
		RESET (m->step_pin);
		mpos -= 1;
	}
	return mpos;
}

static void parse_data (uint8_t *src, uint8_t *dst)
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

void play (uint8_t which, uint32_t num_samples)
{
	Motor *m = motors[which];
	uint8_t fragment[FSIZE];
	uint8_t fpos = 0;
	while (Serial.available () < FSIZE) {}
	for (uint8_t t = 0; t < FSIZE; ++t)
		fragment[t] = Serial.read ();
	uint8_t mpos = 0;
	RESET (m->enable_pin);
	unsigned long last_sample = micros ();
	while (num_samples-- > 0)
	{
		// Decode the sample.
		uint8_t target[8];
		parse_data (&fragment[fpos], target);
		fpos += 3;
		// Play the sample.
		for (uint8_t t = 0; t < 8; ++t)
		{
			last_sample = wait (last_sample);
			mpos = play_sample (m, target[t], mpos);
		}
		// Increment the counter, and start next fragment if at end.
		if (fpos >= FSIZE)
		{
			// If a new fragment is available, load it.
			if (Serial.available () < FSIZE)
			{
				// If at end and no next fragment present, signal buffer underflow and exit.
				Serial.write (CMD_NACK);
				debug ("available: %d", Serial.available ());
				break;
			}
			for (uint8_t t = 0; t < FSIZE; ++t)
				fragment[t] = Serial.read ();
			Serial.write (CMD_INIT);
			fpos = 0;
		}
	}
	// Reset motor to its original position.
	mpos = play_sample (m, 0, mpos);
	last_active = millis ();
}

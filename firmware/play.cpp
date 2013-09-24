#include "firmware.h"

#define FSIZE 32
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

void play (uint8_t which, uint32_t num_samples)
{
	Motor *m = motors[which];
	uint8_t fragment[2][FSIZE];
	uint8_t current = 0, fpos = 0;
	while (Serial.available () < FSIZE) {}
	for (uint8_t t = 0; t < FSIZE; ++t)
		fragment[0][t] = Serial.read ();
	uint8_t mpos = 0;
	bool have_other = false;
	RESET (m->enable_pin);
	unsigned long last_sample = micros ();
	while (num_samples-- > 0)
	{
		// If a new fragment is available, load it.
		if (!have_other && Serial.available () >= FSIZE)
		{
			for (uint8_t t = 0; t < FSIZE; ++t)
				fragment[1 - current][t] = Serial.read ();
			have_other = true;
		}
		// Decode the sample.
		uint8_t target[2] = {uint8_t (fragment[current][fpos] & 0x7), uint8_t ((fragment[current][fpos] >> 4) & 0x7)};
		last_sample = wait (last_sample);
		mpos = play_sample (m, target[0], mpos);
		last_sample = wait (last_sample);
		mpos = play_sample (m, target[1], mpos);
		// Increment the counter, and start next fragment if at end.
		if (++fpos == FSIZE)
		{
			// If at end and no next fragment present, signal buffer underflow and exit.
			if (!have_other)
			{
				Serial.write (CMD_NACK);
				debug ("available: %d", Serial.available ());
				break;
			}
			Serial.write (CMD_INIT);
			fpos = 0;
			current = 1 - current;
			have_other = false;
		}
	}
	// Reset motor to its original position.
	mpos = play_sample (m, 0, mpos);
	// Switch the motor off.
	SET (m->enable_pin);
}

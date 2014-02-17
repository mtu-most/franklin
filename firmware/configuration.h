// ========================== Compile time settings. ==========================
// Note that changing any of these will change the meaning of the bytes in
// EEPROM, so all your old settings are lost when you upload firmware with
// different settings for these values.

// Maximum length of the printer name in bytes.
#define NAMELEN 64

// Maximum number of move commands in the queue.
#define QUEUE_LENGTH 64	

// Maximum number of axes, extruders and temps.  You can use less, not more.
// It is a good idea to have a setting here that you will never reach.  The sum
// may not be higher than 125.  You may want a relatively high value for
// MAXTEMPS, because they can be used as GPIO.
#define MAXAXES 8
#define MAXEXTRUDERS 8
#define MAXTEMPS 8

// Audio settings.  The audio buffer is split in fragments.  New data is sent
// in complete fragment sizes.  Because handshaking may cost time, more  than 2
// fragments is a good idea.
// The data takes 1 bit per sample, and the fragment size is in bytes, so the
// number of samples in a fragment is 8 times AUDIO_FRAGMENT_SIZE.
#define AUDIO_FRAGMENTS 8
#define AUDIO_FRAGMENT_SIZE 32

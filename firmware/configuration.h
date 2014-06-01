// ========================== Compile time settings. ==========================
// Note that changing any of these will change the meaning of the bytes in
// EEPROM, so all your old settings are lost when you upload firmware with
// different settings for these values.

// When setting this, Things are stripped down so much that they fit into an
// Arduino Uno.  This severely limits the functionality.
// All other settings from this file are redefined in firmware.h if this is
// defined.
//#define LOWMEM

// Maximum length of the printer name in bytes.
#define NAMELEN 32

// Maximum number of move commands in the queue.
#define QUEUE_LENGTH 64

// Maximum number of axes, extruders, temps and gpios.  You can use less than
// what you define here, not more.  It is a good idea to have a setting here
// that you will never reach.  The sum may not be higher than 125.
#define MAXAXES 8
#define MAXEXTRUDERS 8
#define MAXTEMPS 8
#define MAXGPIOS 8

// If AUDIO is not defined, nothing related to audio will be included.
#define AUDIO
// Audio settings.  The audio buffer is split in fragments.  New data is sent
// in complete fragment sizes.  Because handshaking may cost time, more  than 2
// fragments is a good idea.
// The data takes 1 bit per sample, and the fragment size is in bytes, so the
// number of samples in a fragment is 8 times AUDIO_FRAGMENT_SIZE.
#ifdef AUDIO
#define AUDIO_FRAGMENTS 8
#define AUDIO_FRAGMENT_SIZE 32
#endif

// Watchdog.  If enabled, the device will automatically reset when it doesn't
// work properly.  However, it may also trigger when too much time is spent
// outputting debugging info.
// To enable, uncomment; to disable, comment it out.
//#define WATCHDOG

// ========================== Compile time settings. ==========================
// Note that changing any of these will change the meaning of the bytes in
// EEPROM, so all your old settings are lost when you upload firmware with
// different settings for these values.

// Maximum number of move commands in the queue.
#define QUEUE_LENGTH 10

#define HAVE_SPACES
#define HAVE_TEMPS
#define HAVE_GPIOS

// If HAVE_AUDIO is not defined, nothing related to audio will be included.
#define HAVE_AUDIO
// Audio settings.  The audio buffer is split in fragments.  New data is sent
// in complete fragment sizes.  Because handshaking may cost time, more  than 2
// fragments is a good idea.
// The data takes 1 bit per sample, and the fragment size is in bytes, so the
// number of samples in a fragment is 8 times AUDIO_FRAGMENT_SIZE.
#ifdef HAVE_AUDIO
#define AUDIO_FRAGMENTS 8
#define AUDIO_FRAGMENT_SIZE 32
#endif

// Watchdog.  If enabled, the device will automatically reset when it doesn't
// work properly.  However, it may also trigger when too much time is spent
// outputting debugging info.
// To enable, uncomment; to disable, comment it out.
//#define WATCHDOG

#define DEBUG_BUFFER_LENGTH 1000

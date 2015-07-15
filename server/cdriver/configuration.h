// ========================== Compile time settings. ==========================
// Note that changing any of these will change the meaning of the bytes in
// EEPROM, so all your old settings are lost when you upload firmware with
// different settings for these values.

// Uncomment to disable all debugging output.
//#define NO_DEBUG

// Maximum number of move commands in the queue.
#define QUEUE_LENGTH 200

// Number of buffers to fill before sending START_MOVE.  Lower number makes it
// start faster, but may cause buffer underruns.
#define MIN_BUFFER_FILL 1

// Watchdog.  If enabled, the device will automatically reset when it doesn't
// work properly.  However, it may also trigger when too much time is spent
// outputting debugging info.
// To enable, uncomment; to disable, comment it out.
#define WATCHDOG

#define DEBUG_BUFFER_LENGTH 0

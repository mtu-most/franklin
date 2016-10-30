/* configuration.h - Compile time configuration for Franklin
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

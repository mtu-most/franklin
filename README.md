# MOST RepRap host software
This is software for controlling a RepRap 3-D printer, created by the Michigan Technological University Open Sustainable Technology Lab (MOST).

Please see the wiki on github for up to date information.

## Introduction
It works slightly different from other solutions.  Others have firmware which
receives G-Code over the serial connection.  Any host program can then be used
to send that gcode.

This firmware, on the other hand, uses a custom protocol for communicating.  It
can only be used with the host-side software from the same release (the
protocol is not stable).  This host-side software is a server, which provides a
scriptable interface to Python programs, and a web interface for manual
control.  Both interfaces allow uploading of G-Code, so any slicer can be used
to generate it.

## What does it support?

The firmware supports:

 * Reliability due to the better communication protocol.
 * Carthesian and delta printers.
 * It's tested on Ramps and Melzi, and should work on any Arduino-based electronics.
 * Limits on speed and acceleration which are enforced by the firmware: to go somewhere as fast as possible, just set the speed to infinity.
 * Near-infinite printer elements (axes, extruders, regulated temperatures, and gpio pins).

The host supports:

 * When the connection is temporarily lost, the host automatically detects the printer again and continues as if nothing happened (even in the middle of a print).
 * Near-infinite flexibility due to the scripting interface.
 * Run the server on the computer connected to the printer (which can be a headless raspberry pi) and print from anywhere on the network.
 * Changing printing speed and filament flow while printing.
 * Changing Z offset while printing (especially useful during the first layer, if the switch isn't calibrated correctly.
 * Set everything up in the web interface; no need to flash the firmware to change settings.
 * For new printers, flashing the firmware can be done from the host interface.
 * Translation and rotation of the print using the nozzle as a reference point ("the lower left corner should be where the nozzle is now").

## What doesn't it support?

 * Printing from SD (planned).
 * Other printer types (should be easy to implement).

## Printer farms

This code is specifically intended to support running several printers from
one server.  This is possible by starting multiple server processes, but that
is ugly.  The server is designed in a way that it allows multiple printers to
be controlled at once.  The benefit of this would be that they can be
controlled over on the same web site (no need to have one tab open per
printer, although that is always possible anyway), or over the same scripting
connection (this is less interesting; it's not hard for a script to open
several connections).

The server runs as a system service, which detects printers (with the firmware
installed) being connected to and disconnected from the system.  To use a new
printer, it is then a simple matter of plugging it in and it's ready for use.
Or if it is a new printer, the port appears in the interface and the firmware
can be uploaded.

## Installation

For installation requirements, please refer to debian/control (the line that
starts with Build-Depends).

If you're running Debian, you should get the Debian package and install it.
It can be built by running ''debuild -uc -us'' in this directory.

If you're not running Debian, you should be.  It makes your life better.

Seriously though, I tried to keep the code as portable as possible.  It should
work on any system except Windows[1].  The automatic device detection relies on
udev, so that will only work on GNU/Linux.  It can be used without it, only the
automatic reconnect feature will not work.

1. I use a separate process for each printer and they communicate over standard
input and standard output.  I need to use select on standard input, and Windows
doesn't support that.  The only way to make it work on Windows would be to use
threads.  I've had such bad experience with that, that I will never use them
again.  However, if you run this host on a Debian server, Windows should have
no trouble connecting to it and using it.

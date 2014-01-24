# MOST RepRap host software
This is software for controlling a RepRap 3-D printer.

## Introduction
It works slightly different from other solutions.  Others have firmware which
receives G-Code over the serial connection.  Any host program can then be used
to send that gcode.

This firmware, on the other hand, uses a custom protocol for communicating.
It should only be used with the host-side software from the same release.
This host-side software is a server, which provides a scriptable interface to
Python programs, and a web interface where some simple operations can be
performed (move to a position, home, etc) and regular G-Code can be uploaded.

Because it does accept G-Code, any slicer can be used to generate it.

## What does it support?

 * Near-infinite flexibility due to the scripting interface.
 * Reliability due to the better communication protocol.
 * Run the server on the computer connected to the printer (which can even be a headless raspberry pi) and print from anywhere on the network.
 * Carthesian and delta printers.
 * It's tested on Ramps and Melzi, but should work on any Arduino-based electronics.
 * Limits on speed and acceleration which will not be violated.
 * Near-infinite axes, extruders, and temperature controllers.

## What doesn't it support?

 * Printing from SD (planned).
 * Other printer types (should be easy to implement).
 * Multiple printers (see below).

If there are other things you are missing, please send me a feature request.

## Printer farms

This code is specifically intended to support running several printers from
one server.  This is possible by starting multiple server processes, but that
is ugly.  The server is designed in a way that it allows multiple printers to
be controlled at once.  The benefit of this would be that they can be
controlled over on the same web site (no need to have one tab open per
printer, although that is always possible anyway), or over the same scripting
connection (this is less interesting; it's not hard for a script to open
several connections).

The plan is to have the server running as a system service, which will detect
printers (with the firmware installed) that are connected to and disconnected
from the system.  To use a new printer, it is then a simple matter of plugging
it in and it's ready for use.  Or if it's a very new printer, the firmware
first needs to be uploaded.

## Installation

For installation requirements, please refer to debian/control (the line that
starts with Build-Depends).

If you're running Debian, you should get or the Debian package and install it.
It can be built by running ''debuild -uc -us'' in this directory.

If you're not running Debian, you should be.  It makes your life better.

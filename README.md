# Franklin RepRap driver
This is software for controlling a RepRap 3-D printer, created by the Michigan Technological University Open Sustainable Technology Lab (MOST).

Please see the wiki on github for up to date information.

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

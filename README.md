# Franklin RepRap driver
This is software for controlling a RepRap 3-D printer, created by the Michigan
Technological University Open Sustainable Technology Lab (MOST).

Please see the wiki on github for up to date information.

## Installation

Installation requirements are listed in debian/control (the line that starts
with Build-Depends).  Some of them are not in Debian, so you need to build
them:

 * [python-fhs](https://github.com/wijnen/python-fhs)
 * [python-network](https://github.com/wijnen/python-network)
 * [python-websocketd](https://github.com/wijnen/python-websocketd)

For information on how to build my packages from source, please refer to [this
page](https://people.debian.org/~wijnen/mypackages.html).

I tried to keep the code as portable as possible.  It should work on any system
except Windows[1].  The automatic device detection relies on udev, so that will
only work on GNU/Linux.  It can be used without it, only the automatic
reconnect feature will not work.

[1] I use a separate process for each printer and they communicate over
standard input and standard output.  I need to use select on standard input,
and Windows doesn't support that.  The only way to make it work on Windows
would be to use threads.  I've had such bad experience with that, that I will
never use them again.  However, if you run this host on a Debian server,
Windows should have no trouble connecting to it and using it.

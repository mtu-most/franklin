# MOST RepRap host software
This is software for controlling a RepRap 3-D printer.  Use it if you don't
like the other options.

## Installation
Install the following Debian packages (or their versions for your OS):
 * python 2.7
 * python-serial
 * python-gtk2
 * python-avahi
 * apache2

Set up the apache userdir module: run "a2enmod userdir" and add in /etc/apache2/mods-available/userdir.conf:
        AllowOverride All
        Options MultiViews Indexes SymLinksIfOwnerMatch IncludesNoExec ExecCGI
        <Files *.cgi>
            sethandler cgi-script
        </Files>
Then restart apache.

Put this directory in `~/public_html`.  Then run `reprap-server`.  It should connect to the printer and home it.  If it doesn't, you may need to pass an arguments for which serial port to use: `reprap-server --serial /dev/ttySomething`.

When the server is running, you should be able to contact it at
http://localhost/most-reprap

# The problem that this code tries to solve
Controlling a 3D printer should be easy.  The firmware handles the hardware,
and the slicer handles the conversion from STL to GCODE.  The interface only
needs to pass the GCODE to the firmware, and allow some simple direct
manipulations such as setting the temperature.

But there are extra features which are useful.  Repetier Host allows on the fly
changes to the print speed and the amount of filament that is used, for
example.

Repetier Server allows sending GCODE with a web browser for printing.

But Repetier Host is written in .Net, and it regularly breaks my entire desktop
(it somehow manages to turn "focus to pointer" off and there is no way I can
switch it on again, other than rebooting the machine).  That is bad enough for
me to want something else.  Repetier Server doesn't allow direct control over
the printer.  So you either have a "normal" interface, or you can attach the
printer to a network, but not both.  And Repetier makes a giant mess of your
filesystem by installing itself in places where it shouldn't be going.  I think
all other printer interfaces do that, too.

# The solution has three parts
So my solution consists of several parts: a server (reprap-server), a cgi
script (index.cgi), and a web page (reprap.html, reprap.css and reprap.js).

## The server
This is the part that talks to the firmware.  It listens to the network, from
which it accepts any GCODE which it will pass on.  It also has a few
convenience methods for all the common operations.

The server fills the hole that the firmware should really be filling: it
remembers the state of the printer and allows clients to request it.  So for
example, the server can be asked ''what is the current location of the
extruder?'' and the answer will be in a standard and understandable format
(x,y,z).  The GCODE definition on the RepRap wiki has some rules about what the
firmware should do, but they are not strict enough to write an interface on,
and in any case they are completely ignored by firmware writers.  So the
interface must do this job.  The server does it, so clients don't have to worry
about a thing.

The server also has all the required workarounds for properly working with
multiple extruders (for example, sleep() will disable not only the current
extruder motor, but all of them).

The network interface that the server exports is only usable from Python; it
uses a protocol which is very convenient for the client: they simply have an
object where they call member functions, which return a value.

## The cgi script
One such Python program is the CGI script that comes with the server.  It
should be installed in a place where it can be reached by people who should
have access to the printer.  Use http authentication if you need to restrict
access.

The script doesn't do much; it provides a web site, and passes requests on to
the server.  It is set up such that the requests can be made using AJAX.

## The website
The website has a few buttons and a printer area where you can click to move
the extruder.  For each extruder, it has a temperature setting and a flowrate
setting.  It has a feedrate setting and bed temperature control.

It regularly requests the current temperatures from the server.  This has an unfortunate side-effect: at least in firefox, during this request, keyboard events are ignored.  So typing a new temperature in a box is annoying, because the numbers often aren't coming through.

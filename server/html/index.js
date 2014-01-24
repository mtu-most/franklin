var widget;
var rpc;

function init ()
{
	widget = document.getElementById ('printfile');
	rpc = Rpc (null, setup, function () { alert ('connection to server lost'); })
}

function setup ()
{
	setInterval (function () { rpc.call ('status', [], {}, set_status); }, 5000);
	// Get initial status right away.
	rpc.call ('status', [], {}, set_status);
}

function set_status (data) {
	settext (document.getElementById ('btemp_read'), data[0]);
	settext (document.getElementById ('etemp_read'), data[1]);
}

function settext (element, text)
{
	while (element.firstChild)
		element.removeChild (element.firstChild);
	element.appendChild (document.createTextNode (text));
}

function print () {
	if (widget.files.length < 1)
		return;
	var reader = new FileReader ();
	reader.onloadend = function (e) {
		if (e.target.readyState == FileReader.DONE)
			rpc.event ('gcode', [e.target.result], {});
	}
	reader.readAsBinaryString (widget.files[0]);
}

function etemp ()
{
	rpc.event ('settemp_extruder', [0, Number (document.getElementById ('etemp').value)], {});
}

function btemp ()
{
	rpc.event ('settemp_temp', [0, Number (document.getElementById ('btemp').value)], {});
}

function stop ()
{
	rpc.event ('pause', [true], {});
}

function resume ()
{
	rpc.event ('pause', [false], {});
}

function home ()
{
	rpc.event ('home_all', [], {});
}

function homez ()
{
	rpc.event ('home_z', [], {});
}

function sleep ()
{
	rpc.event ('sleep_all', [], {});
}

function feed ()
{
	rpc.event ('set_feedrate', [Number (document.getElementById ('feed').value)], {});
}

function setoffset ()
{
	rpc.event ('axis_set_offset', [0, Number (document.getElementById ('xoff').value)], {});
	rpc.event ('axis_set_offset', [1, Number (document.getElementById ('yoff').value)], {});
	rpc.event ('axis_set_offset', [2, Number (document.getElementById ('zoff').value)], {});
}

function gotopos ()
{
	rpc.event ('goto', [[Number (document.getElementById ('xpos').value), Number (document.getElementById ('ypos').value), Number (document.getElementById ('zpos').value)]], {});
}

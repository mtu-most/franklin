// vim: set foldmethod=indent :

var canvas, position, table;
var printer_pos, self_pos;
var temp, temptarget, flowfactor;
var bedtemp, bedtarget;
var speed;
var printfile, printsd;

function debug (text)
{
	var t = document.createTextNode (text);
	var p = document.createElement ('p');
	p.appendChild (t);
	document.getElementById ('debug').appendChild (p);
}

function dump (obj)
{
	var s = '';
	var i;
	for (i in obj)
		s += i + ': ' + obj[i] + '\n';
	return s;
}

function set_text (element, text)
{
	element.replaceChild (document.createTextNode (text), element.firstChild);
}

function send (request, check_return = true)
{
	r = new XMLHttpRequest ();
	if (request.constructor == String)
	{
		r.open ('GET', '?request=' + request, false);
		r.send ();
	}
	else
	{
		r.open ('POST', '', false);
		r.send (request);
	}
	if (check_return && r.response.trim () != 'ok')
		alert ('Error response on command "' + request + '": ' + r.response);
	else
		return r.response;
}

function line (p0, p1)
{
	canvas.moveTo (p0[0], p0[1]);
	canvas.lineTo (p1[0], p1[1]);
}

function toPixel (pos)
{
	return [50 + pos[0] * 2.5 + pos[1], 590 - pos[1] - pos[2] * 3];
}

function move (e)
{
	var x = e.layerX - canvas.canvas.offsetLeft;
	var y = e.layerY - canvas.canvas.offsetTop;
	if (e.shiftKey)
	{
		// x = 50 + X * 2.5 + Y; y = 590 - Y - Z * 3
		// Z = (590 - Y - y) / 3
		self_pos[2] = (590 - y - self_pos[1]) / 3;
	}
	else
	{
		// x = 50 + X * 2.5 + Y; y = 590 - Y - Z * 3
		// Y = 590 - y - Z * 3
		self_pos[1] = 590 - y - self_pos[2] * 3;
	}
	// X = (x - 50 - Y) / 2.5; altijd, met mogelijk nieuwe Y.
	self_pos[0] = (x - 50 - self_pos[1]) / 2.5;
	if (self_pos[0] < 0)
		self_pos[0] = 0;
	if (self_pos[1] < 0)
		self_pos[1] = 0;
	if (self_pos[2] < 0)
		self_pos[2] = 0;
	if (self_pos[0] > 200)
		self_pos[0] = 200;
	if (self_pos[1] > 200)
		self_pos[1] = 200;
	if (self_pos[2] > 100)
		self_pos[2] = 100;
	update ();
}

function clik (e)
{
	send ('goto&x=' + self_pos[0] + '&y=' + self_pos[1] + '&z=' + self_pos[2]);
}

function marker (pos)
{
	var base = toPixel ([pos[0], pos[1], 0]);
	var dot = toPixel (pos);
	line ([base[0] - 10, base[1] + 10], [base[0] + 10, base[1] - 10]);
	line ([base[0] - 20, base[1]], [base[0] + 20, base[1]]);
	line (base, dot);
	canvas.arc (dot[0], dot[1], 5, Math.PI / 2, 2 * Math.PI + Math.PI / 2, false);
	line (toPixel ([0, 0, pos[2]]), toPixel ([200, 0, pos[2]]));
	line (toPixel ([200, 0, pos[2]]), toPixel ([200, 200, pos[2]]));
	line (toPixel ([200, 200, pos[2]]), toPixel ([0, 200, pos[2]]));
	line (toPixel ([0, 200, pos[2]]), toPixel ([0, 0, pos[2]]));
}

function box ()
{
	a = toPixel ([0, 0, 0]);
	b = toPixel ([200, 0, 0]);
	c = toPixel ([0, 200, 0]);
	d = toPixel ([200, 200, 0]);
	e = toPixel ([0, 0, 100]);
	f = toPixel ([200, 0, 100]);
	g = toPixel ([0, 200, 100]);
	h = toPixel ([200, 200, 100]);
	line (a, b);
	line (a, c);
	line (b, d);
	line (c, d);
	line (a, e);
	line (b, f);
	line (c, g);
	line (d, h);
	line (e, f);
	line (e, g);
	line (f, h);
	line (g, h);
}

function update ()
{
	canvas.beginPath ();
	canvas.fillRect (0, 0, 800, 600);
	box ();
	marker (printer_pos);
	marker (self_pos);
	canvas.stroke ();
}

function refresh ()
{
	try
	{
		var config = JSON.parse (send ("config", false));
		var state = JSON.parse (send ("state", false));
		speed.value = config.feedfactor;
		bedtarget.value = state.bed_target;
		set_text (bedtemp, state.bed_current + '°C');
		for (var e = 0; e < extruders; ++e)
		{
			flowfactor[e].value = config.flowfactor[e];
			temptarget[e].value = state.temperature_target[e];
			set_text (temp[e], state.temperature_current[e] + '°C');
		}
		printer_pos = state.position;
		set_text (position, 'X:' + state.position[0] + ' Y:' + state.position[1] + ' Z:' + state.position[2]);
	}
	catch (e)
	{
		alert ('error refreshing: ' + e);
	}
	update ();
}

function init ()
{
	canvas = document.getElementById ('view').getContext ('2d');
	canvas.fillStyle = '#ff8';
	printer_pos = [0, 0, 0];
	self_pos = [0, 0, 0];
	bedtemp = document.getElementById ('bedtemp');
	bedtarget = document.getElementById ('bedtarget');
	speed = document.getElementById ('speed');
	position = document.getElementById ('position');
	table = document.getElementById ('table');
	printfile = document.getElementById ('printfile');
	printsd = document.getElementById ('printsd');
	var info = JSON.parse (send ("config", false));
	extruders = info.extruders;
	temp = [];
	temptarget = [];
	flowfactor = [];
	function texttd (text)
	{
		var ret = document.createElement ('td');
		ret.appendChild (document.createTextNode (text));
		return ret;
	}
	function input (e, request, arg)
	{
		var ret = document.createElement ('input');
		ret.type = 'text';
		ret.onchange = function () { send (request + '&' + arg + '=' + this.value + '&extruder=' + e); }
		return ret;
	}
	function extrudebox (e, request, label, dir)
	{
		var ret = document.createElement ('td');
		var input = document.createElement ('input');
		ret.appendChild (input);
		ret.appendChild (document.createTextNode (label));
		input.type = 'checkbox';
		input.onchange = function ()
		{
			if (!input.checked)
			{
				send ('extrude&dir=0&extruder=' + e);
			}
			else
			{
				input.other.firstChild.checked = false;
				send ('extrude&dir=' + dir + '&extruder=' + e);
			}
		}
		return ret;
	}
	for (var e = 0; e < extruders; ++e)
	{
		var tr = document.createElement ('tr');
		table.appendChild (tr);
		tr.appendChild (texttd ('Extruder ' + e + ':'));
		temp.push (texttd ('-°C'));
		tr.appendChild (temp[e]);
		temptarget.push (input (e, 'temperature', 'temperature'));
		tr.appendChild (temptarget[e]);
		var extrude = extrudebox (e, 'extrude', 'Extrude', 1);
		var retract = extrudebox (e, 'retract', 'Retract', -1);
		extrude.firstChild.other = retract;
		retract.firstChild.other = extrude;
		tr.appendChild (extrude);
		tr.appendChild (retract);
		tr.appendChild (texttd ('Flow factor:'));
		flowfactor.push (input (e, 'flowrate', 'factor'));
		tr.appendChild (flowfactor[e]);
	}
	refresh ();
	update ();
	//setInterval (timed_update, 2000);
}

function set_bedtarget ()
{
	send ('bed&temperature=' + bedtarget.value);
}

function update_feedrate ()
{
	send ('feedfactor&factor=' + feedrate_input.value);
}

function printfile_cb ()
{
	var fd = new FormData ();
	fd.append ('request', 'print');
	fd.append ('file', printfile.files[0]);
	send (fd);
}

function printsd_cb ()
{
}

function set_speed ()
{
	send ('feedfactor&factor=' + speed.value);
}

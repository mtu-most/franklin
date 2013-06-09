// vim: set foldmethod=indent :

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

function send (request)
{
	r = new XMLHttpRequest ();
	r.open ('GET', '?request=' + request, false);
	r.send ();
	return r.response;
}

var canvas;
var printer_pos, self_pos;
var temp_current, temp_target;
var current_bed, target_bed, bed_input;
var feedrate_input, feedrate_label;

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
	var x = e.layerX;
	var y = e.layerY;
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

function timed_update ()
{
	var info = JSON.parse (send ("config"));
	set_text (feedrate_label, info.feedfactor);
	var info = JSON.parse (send ("state"));
	target_bed.text = info.bed_target;
	current_bed.text = info.bed_current;
	set_text (target_bed, info.bed_target);
	set_text (current_bed, info.bed_current);
	for (var e = 0; e < extruders; ++e)
	{
		set_text (temp_target[e], info.temperature_target[e]);
		set_text (temp_current[e], info.temperature_current[e]);
	}
	printer_pos = info.position;
	set_text (x_label, info.position[0]);
	set_text (y_label, info.position[1]);
	set_text (z_label, info.position[2]);
	update ();
}

function init ()
{
	canvas = document.getElementById ('view').getContext ('2d');
	canvas.fillStyle = '#fff';
	printer_pos = [0, 0, 0];
	self_pos = [0, 0, 0];
	current_bed = document.getElementById ('current_bed');
	target_bed = document.getElementById ('target_bed');
	bed_input = document.getElementById ('bed');
	feedrate_label = document.getElementById ('current_feedrate');
	feedrate_input = document.getElementById ('feedrate');
	var info = JSON.parse (send ("config"));
	extruders = info.extruders;
	temp_current = [];
	temp_target = [];
	for (var e = 0; e < extruders; ++e)
	{
		var div = document.createElement ('div');
		div['class'] = 'temp';
		document.getElementById ('body').appendChild (div);
		div.appendChild (document.createTextNode ('E' + e));
		var temp_input = document.createElement ('input');
		temp_input.type = 'text';
		div.appendChild (temp_input);
		var element = document.createElement ('button');
		element['class'] = 'temp';
		element.type = 'button';
		element.onclick = function () { send ('temperature&temperature=' + this.input.value + '&extruder=' + this.e); }
		element.input = temp_input;
		element.e = e;
		element.appendChild (document.createTextNode ('Update'));
		div.appendChild (element);
		temp_current.push (document.createElement ('span'));
		temp_current[e].appendChild (document.createTextNode ('-'));
		div.appendChild (temp_current[e]);
		div.appendChild (document.createTextNode ('/'));
		temp_target.push (document.createElement ('span'));
		temp_target[e].appendChild (document.createTextNode ('-'));
		div.appendChild (temp_target[e]);
	}
	timed_update ();
	update ();
	setInterval (timed_update, 2000);
}

function setbed ()
{
	send ('bed&temperature=' + bed_input.value);
}

function update_feedrate ()
{
	send ('feedfactor&factor=' + feedrate_input.value);
}

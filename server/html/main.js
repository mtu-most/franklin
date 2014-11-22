// vim: set foldmethod=marker :
// Variables.  {{{
var get_pointer_pos_xy = function() { return [NaN, NaN]; }
var get_pointer_pos_z = function() { return [NaN, NaN]; }
var ports;
var labels_element, printers_element;
var selected_port, selected_printer;
var script_cbs;
var multiples;
var type2plural = {space: 'spaces', temp: 'temps', gpio: 'gpios', axis: 'axes', motor: 'motors'};
var space_types = ['Extruder', 'Cartesian', 'Delta'];
var hidetypes = [];
// }}}

// General supporting functions. {{{
function get_elements(list) { // {{{
	var ret = [];
	for (var i = 0; i < list.length; ++i) {
		if (!(list[i] instanceof Element))
			continue;
		ret.push(list[i]);
	}
	return ret;
} // }}}

function init() { // {{{
	script_cbs = new Object;
	ports = new Object;
	multiples = new Object;
	labels_element = document.getElementById('labels');
	printers_element = document.getElementById('printers');
	selected_port = null;
	selected_printer = null;
	setup();
	register_update('confirm', do_confirm);
	register_update('queue', do_queue);
	register_update('new_port', new_port);
	register_update('new_printer', new_printer);
	register_update('new_script', new_script);
	register_update('new_data', new_script_data);
	register_update('blocked', blocked);
	register_update('message', new_message);
	register_update('del_script', del_script);
	register_update('del_printer', del_printer);
	register_update('del_port', del_port);
	register_update('globals_update', update_globals);
	register_update('space_update', update_space);
	register_update('temp_update', update_temp);
	register_update('gpio_update', update_gpio);
	if (document.location.search.length > 0 && document.location.search[0] == '?') {
		var things = document.location.search.split('&');
		for (var t = 0; t < things.length; ++t) {
			var items = things[t].substring(1).split('=');
			if (items[0] == 'setup' && items[1] != '0')
				document.getElementById('container').RemoveClass('nosetup');
		}
	}
	var reading_temps = false;
	setInterval(function() {
		if (reading_temps || !selected_printer)
			return;
		reading_temps = true;
		var read = function(printer, num) {
			if (num >= printer.temps.length) {
				reading_temps = false;
				// Update temperature graph.
				var canvas = get_element(printer, [null, 'tempgraph']);
				var c = canvas.getContext('2d');
				c.save();
				c.clearRect(0, 0, canvas.width, canvas.width);
				c.translate(0, canvas.height);
				c.scale(1, -canvas.height / 250);
				for (var t = 0; t < printer.temps.length; ++t) {
					printer.temps[t].history.push(printer.temps[t].temp);
					if (printer.temps[t].history.length > canvas.width)
						printer.temps[t].history.splice(0, printer.temps[t].history.length - canvas.width);
					var data = printer.temps[t].history;
					c.beginPath();
					c.moveTo(canvas.width - data.length, data[0]);
					for (var i = 1; i < data.length; ++i)
						c.lineTo(canvas.width - data.length + i, data[i]);
					c.strokeStyle = '#f00';
					c.lineWidth = 4;
					c.stroke();
				}
				c.restore();
				return;
			}
			printer.call('readtemp', [num], {}, function(t) {
				if (num < printer.temps.length) {
					printer.temps[num].temp = t;
					var e = get_element(printer, [['temp', num], 'temp']);
					e.ClearAll();
					e.AddText(t.toFixed(1));
				}
				read(printer, num + 1);
			});
		};
		read(selected_printer, 0);
	}, 400);
} // }}}

function make_id(printer, id, extra) { // {{{
	// [null, 'num_spaces']
	// [['space', 1], 'num_axes']
	// [['axis', [0, 1]], 'offset']
	// [['motor', [0, 1]], 'delta_radius']
	var ret = printer ? printer.port.replace(/[^a-zA-Z0-9_]/g, '') : '';
	if (id[0] !== null) {
		if (typeof id[0][0] == 'string')
			ret += '_' + id[0][0];
		else {
			ret += '_' + id[0][0][0];
			ret += '_' + id[0][0][1];
		}
		if (typeof id[0][1] == 'number')
			ret += '_' + String(id[0][1]);
		else if (id[0][1] !== null) {
			ret += '_' + String(id[0][1][0]);
			ret += '_' + String(id[0][1][1]);
		}
	}
	ret += '_' + id[1];
	if (extra !== null && extra !== undefined)
		ret += '_' + String(extra);
	return ret;
} // }}}

function set_value(printer, id, value, reply, arg) { // {{{
	if (id.length == 2 || id[2] === undefined) {
		var obj = {};
		obj[id[1]] = value;
		if (id[0] === null) {
			// [null, 'num_spaces']
			printer.call('set_globals', [], obj, reply);
		}
		else {
			if (id[0][1] === null) {
				// [['space', null], 'num_axes']
				for (var n = 0; n < printer['num_' + type2plural[id[0][0]]]; ++n)
					printer.call('set_' + id[0][0], [n], obj, reply);
			}
			else if (typeof id[0][1] != 'number' && id[0][1][1] == null) {
				// [['axis', [0, null]], 'offset']
				// [['motor', [0, null]], 'delta_radius']
				if (id[0][0] != 'motor' || id[1].substr(0, 6) != 'delta_') {
					for (var n = 0; n < printer.spaces[id[0][1][0]]['num_' + type2plural[id[0][0]]]; ++n)
						printer.call('set_' + id[0][0], [[id[0][1][0], n]], obj, reply);
				}
				else {
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(6)] = value;
					var o = {};
					for (var n = 0; n < printer.spaces[id[0][1][0]].num_motors; ++n)
						o[n] = obj;
					printer.call('set_space', [id[0][1][0]], {delta: o}, reply);
				}
			}
			else if (id[0][0] != 'motor' || id[1].substr(0, 6) != 'delta_') {
				// [['space', 1], 'num_axes']
				// [['axis', [0, 1]], 'offset']
				printer.call('set_' + id[0][0], [id[0][1]], obj, reply);
			}
			else {
				// [['motor', [0, 1]], 'delta_radius']
				obj = {};	// obj was wrong in this case.
				obj[id[1].substr(6)] = value;
				var o = {};
				o[id[0][1][1]] = obj;
				printer.call('set_space', [id[0][1][0]], {delta: o}, reply);
			}
		}
	}
	else {
		var val;
		if (arg !== undefined)
			val = [value, arg];
		else
			val = [value];
		if (id[0] === null) {
			if (printer)
				printer.call(id[2], val, {}, reply);
			else
				rpc.call(id[2], val, {}, reply);
		}
		else if (id[0][1] === null) {
			// [['space', null], 'num_axes']
			for (var n = 0; n < printer['num_' + type2plural[id[0][0]]]; ++n)
				printer.call(id[2], [n, value], {}, reply);
		}
		else
			printer.call(id[2], [id[0][1], value], {}, reply);
	}
} // }}}

function get_value(printer, id) { // {{{
	if (id[0] === null)
		return printer[id[1]];
	if (id[0][0] == 'axis') {
		return printer.spaces[id[0][1][0]].axis[id[0][1][1]][id[1]];
	}
	else if (id[0][0] == 'motor') {
		return printer.spaces[id[0][1][0]].motor[id[0][1][1]][id[1]];
	}
	else {
		return printer[type2plural[id[0][0]]][id[0][1]][id[1]];
	}
} // }}}

function get_element(printer, id, extra) { // {{{
	return document.getElementById(make_id(printer, id, extra));
} // }}}

function select_printer(port) { // {{{
	if (port === undefined)
		port = selected_port;
	else
		selected_port = port;
	selected_printer = printers[port];
	for (var p in ports) {
		if (typeof ports[p] != 'object')
			continue;
		if (p == port) {
			ports[p][0].AddClass('active');
			ports[p][1].RemoveClass('hidden');
			if (ports[p][2])
				ports[p][2].RemoveClass('hidden');
		}
		else {
			ports[p][0].RemoveClass('active');
			ports[p][1].AddClass('hidden');
			if (ports[p][2])
				ports[p][2].AddClass('hidden');
		}
	}
} // }}}

function upload_buttons(port, buttons) { // {{{
	var ret = document.createElement('ul');
	for (var b = 0; b < buttons.length; ++b) {
		var button = ret.AddElement('li').AddElement('button');
		button.type = 'button';
		button.target = buttons[b][0];
		button.onclick = function() {
			rpc.call('upload', [port, this.target], {}, function(ret) { alert('upload done: ' + ret);});
		};
		button.AddText(buttons[b][1]);
	}
	return [ret];
}
// }}}

function update_temps(printer, t) { // {{{
	if (t === null) {
		for (var i = 0; i < printer.temptargets.length; ++i) {
			var f = function(i) {
				printer.call('readtemp', [i], {}, function(value) {
					printer.temptargets[i].ClearAll();
					printer.temptargets[i].AddText(value.toFixed(1));
				});
			};
			f(i);
		}
	}
	else {
		printer.call('readtemp', [t], {}, function(value) {
			printer.temptargets[t].ClearAll();
			printer.temptargets[t].AddText(value.toFixed(1));
		});
	}
}
// }}}

function floatkey(event, element) { // {{{
	var amount;
	var set = false;
	if (event.keyCode == 13) { // Enter
		var v = element.value;
		if (v[0] == '+')
			amount = Number(v.substr(1));
		else {
			amount = Number(v);
			set = true;
		}
	}
	else if (event.keyCode == 33) { // Page Up
		amount = 10;
	}
	else if (event.keyCode == 34) { // Page Down
		amount = -10;
	}
	else if (event.keyCode == 38) { // Up
		amount = 1;
	}
	else if (event.keyCode == 40) { // Down
		amount = -1;
	}
	else
		return;
	event.preventDefault();
	if (event.shiftKey)
		amount /= 10;
	if (element.obj[0] !== null && element.obj[0][1] === null) {
		for (var n = 0; n < element.printer['num_' + type2plural[element.obj[0][0]]]; ++n) {
			var obj = [[element.obj[0][0], n], element.obj[1], element.obj[2]];
			var value;
			if (set)
				value = amount;
			else if (element.obj.length == 2)
				value = get_value(element.printer, obj) / element.factor + amount;
			else
				continue;
			set_value(element.printer, obj, element.factor * value);
		}
		return;
	}
	if (element.obj[0] !== null && typeof element.obj[0][1] != 'number' && element.obj[0][1][1] === null) {
		for (var n = 0; n < element.printer.spaces[element.obj[0][1][0]]['num_' + type2plural[element.obj[0][0]]]; ++n) {
			var obj = [[element.obj[0][0], [element.obj[0][1][0], n]], element.obj[1]];
			var value;
			if (set)
				value = amount;
			else if (element.obj.length == 2)
				value = get_value(element.printer, obj) / element.factor + amount;
			else
				continue;
			set_value(element.printer, obj, element.factor * value);
		}
		return;
	}
	var value;
	if (set)
		value = amount;
	else if (element.obj.length == 2)
		value = get_value(element.printer, element.obj) / element.factor + amount;
	else
		return;
	if (element.set !== undefined)
		element.set(element.factor * value);
	else
		set_value(element.printer, element.obj, element.factor * value);
}
// }}}
// }}}

// Queue functions.  {{{
function queue_deselect() { // {{{
	var e = get_element(selected_printer, [null, 'queue']);
	for (var i = 1; i < e.options.length; ++i) {
		e.options[i].selected = false;
	}
}
// }}}

function queue_up() { // {{{
	var e = get_element(selected_printer, [null, 'queue']);
	for (var i = 1; i < e.options.length; ++i) {
		if (e.options[i].selected) {
			var cur = e.options[i];
			var prev = e.options[i - 1];
			e.insertBefore(e.removeChild(cur), prev);
		}
	}
}
// }}}

function queue_down() { // {{{
	var e = get_element(selected_printer, [null, 'queue']);
	for (var i = e.options.length - 2; i >= 0; --i) {
		if (e.options[i].selected) {
			var cur = e.options[i];
			var next = e.options[i + 1];
			e.insertBefore(e.removeChild(next), cur);
		}
	}
}
// }}}

function queue_del() { // {{{
	var e = get_element(selected_printer, [null, 'queue']);
	var rm = [];
	for (var i = 0; i < e.options.length; ++i) {
		if (e.options[i].selected)
			rm.push(e.options[i]);
	}
	for (var i = 0; i < rm.length; ++i)
		rpc.call('queue_remove', [rm[i].value], {});
}
// }}}

function get_queue(printer) { // {{{
	if (printer === undefined)
		printer = selected_printer;
	var toprint = [];
	var e = get_element(printer, [null, 'queue']);
	for (var i = 0; i < e.options.length; ++i) {
		if (e.options[i].selected)
			toprint.push(e.options[i].value);
	}
	return toprint;
}
// }}}

function queue_print(printer) { // {{{
	var angle = isNaN(printer.targetangle) ? 0 : printer.targetangle;
	var sina = Math.sin(angle);
	var cosa = Math.cos(angle);
	var action = function(a) {
		printer.call(a, [get_queue(), [printer.targetx, printer.targety, printer.targetz], angle * 180 / Math.PI], {});
	};
	if (get_element(printer, [null, 'probebox']).checked)
		action('queue_probe');
	else
		action('queue_print');
}
// }}}
// }}}

// Non-update events. {{{
function new_port() { // {{{
	ports[port] = [Label(), NoPrinter(), null, ''];
	ports[port][0].AddClass('setup');
	labels_element.Add(ports[port][0]);
	printers_element.Add(ports[port][1]);
} // }}}

function new_printer() { // {{{
	printer.targetangle = 0;
	printer.targetx = 0;
	printer.targety = 0;
	printer.targetz = 0;
	printer.temptargets = [];
	multiples[port] = {space: [], axis: [], motor: [], temp: [], gpio: []};
	ports[port][2] = Printer();
	ports[port][1] = printers_element.Add(ports[port][2]);
	ports[port][0].RemoveClass('setup');
	ports[port][1].RemoveClass('notconnected');
	ports[port][1].AddClass('connected');
	update_float(printer, [null, 'targetx']);
	update_float(printer, [null, 'targety']);
	update_float(printer, [null, 'targetz']);
	select_printer(port);
} // }}}

function new_script(name) { // {{{
	var data = scripts[name][1];
	var div = document.getElementById('scripts').AddElement('div');
	var new_data = function() {};
	eval(scripts[name][0]);
	var button = div.AddElement('button');
	button.type = 'button';
	button.AddText('Remove');
	button.name = name;
	button.addEventListener('click', function() { rpc.call('del_script', [button.name], {}); }, false);
	script_cbs[name] = [div, new_data];
} // }}}

function new_script_data(name) { // {{{
	script_cbs[name][1] ();
} // }}}

function blocked(reason) { // {{{
	var e = ports[port][2] !== null ? get_element(printer, [null, 'block1']) : get_element({'port': port}, [null, 'block2']);
	if (reason) {
		e.ClearAll();
		e.AddText(reason);
		e.RemoveClass('hidden');
	}
	else
		e.AddClass('hidden');
} // }}}

function new_message(msg) { // {{{
	var e = ports[port][2] !== null ? get_element(printer, [null, 'message1']) : get_element({'port': port}, [null, 'message2']);
	e.ClearAll();
	e.AddText(msg);
} // }}}

function del_script(name) { // {{{
	document.getElementById('scripts').removeChild(script_cbs[name][0]);
	delete script_cbs[name];
} // }}}

function del_printer() { // {{{
	ports[port][1].RemoveClass('connected');
	ports[port][1].AddClass('notconnected');
	ports[port][0].AddClass('setup');
	printers_element.removeChild(ports[port][2]);
	ports[port][2] = null;
	delete multiples[port];
	ports[port][3] = '';
	if (selected_port == port)
		selected_printer = null;
} // }}}

function del_port() { // {{{
	labels_element.removeChild(ports[port][0]);
	printers_element.removeChild(ports[port][1]);
	if (selected_port == port)
		selected_port = null;
} // }}}

function do_confirm(id, message) { // {{{
	var the_printer = printer;
	the_printer.call('confirm', [id, confirm(message)], {});
} // }}}

function do_queue() { // {{{
	var e = get_element(selected_printer, [null, 'queue']);
	var q = [];
	var must_deselect = selected_printer.queue.length > e.options.length;
	for (var i = 0; i < selected_printer.queue.length; ++i)
		q.push(selected_printer.queue[i][0]);
	var rm = [];
	for (var item = 0; item < e.options.length; ++item) {
		var i;
		for (i = 0; i < q.length; ++i) {
			if (q[i] == e.options[item].value) {
				if (must_deselect)
					e.options[item].selected = false;
				q.splice(i, 1);
				i = -1;
				break;
			}
		}
		if (i < 0) {
			continue;
		}
		rm.push(item);
	}
	for (var item = 0; item < rm.length; ++rm)
		e.removeChild(e.options[rm[item]]);
	for (var i = 0; i < q.length; ++i) {
		var option = e.AddElement('option');
		option.AddText(q[i]);
		option.value = q[i];
		option.selected = true;
	}
	start_move();
} // }}}
// }}}

// Update events(from server). {{{
function update_globals() { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	if (ports[port][3] != printer.name) {
		labels_element.removeChild(ports[port][0]);
		ports[port][0] = Label();
		labels_element.Add(ports[port][0]);
		ports[port][3] = printer.name;
		select_printer();
	}
	get_element(printer, [null, 'export']).href = printer.name + '.ini?port=' + encodeURIComponent(port);
	update_float(printer, [null, 'num_spaces']);
	update_float(printer, [null, 'num_temps']);
	update_float(printer, [null, 'num_gpios']);
	update_pin([null, 'led_pin']);
	update_pin([null, 'probe_pin']);
	update_float(printer, [null, 'probe_dist']);
	update_float(printer, [null, 'probe_safe_dist']);
	update_float(printer, [null, 'bed_id']);
	update_float(printer, [null, 'motor_limit']);
	update_float(printer, [null, 'temp_limit']);
	update_float(printer, [null, 'feedrate']);
	update_float(printer, [null, 'zoffset']);
	var stat = get_value(printer, [null, 'status']);
	var c = document.getElementById('container');
	c.RemoveClass('idle printing paused');
	if (stat === null)
		c.AddClass('idle');
	else if (stat)
		c.AddClass('printing');
	else
		c.AddClass('paused');
	var t = multiples[port].space;
	var showing = [], hiding = [];
	for (var s = 0; s < t.length; ++s) {
		while (t[s][1].length > printer.num_spaces) {
			// Remove axes and motors.
			var a = multiples[port].axis.pop();
			for (var as = 0; as < a.length; ++as) {
				while (a[as][1].length > 0) {
					var n = a[as][1].pop();
					for (var i = 0; i < n.length; ++i)
						a[as][0].removeChild(n[i]);
				}
			}
			var m = multiples[port].motor.pop();
			for (var ms = 0; ms < m.length; ++ms) {
				while (m[ms][1].length > 0) {
					var n = m[ms][1].pop();
					for (var i = 0; i < n.length; ++i)
						m[ms][0].removeChild(n[i]);
				}
			}
			var n = t[s][1].pop();
			for (var i = 0; i < n.length; ++i)
				t[s][0].removeChild(n[i]);
		}
		while (t[s][1].length < printer.num_spaces) {
			multiples[port].axis.push([]);
			multiples[port].motor.push([]);
			var n = t[s][3](t[s][1].length);
			t[s][1].push(n);
			for (var i = 0; i < n.length; ++i) {
				t[s][0].insertBefore(n[i], t[s][2]);
			}
		}
		if (!(t[s][2] instanceof Comment)) {
			if (printer.num_spaces >= 2)
				t[s][2].RemoveClass('hidden');
			else
				t[s][2].AddClass('hidden');
		}
		if (printer.num_spaces > 0)
			showing.push(t[s][0]);
		else
			hiding.push(t[s][0]);
	}
	t = multiples[port].temp;
	for (var s = 0; s < t.length; ++s) {
		while (t[s][1].length > printer.num_temps) {
			var n = t[s][1].pop();
			for (var i = 0; i < n.length; ++i)
				t[s][0].removeChild(n[i]);
		}
		while (t[s][1].length < printer.num_temps) {
			var n = t[s][3](t[s][1].length);
			t[s][1].push(n);
			for (var i = 0; i < n.length; ++i)
				t[s][0].insertBefore(n[i], t[s][2]);
		}
		if (!(t[s][2] instanceof Comment)) {
			if (printer.num_temps >= 2)
				t[s][2].RemoveClass('hidden');
			else
				t[s][2].AddClass('hidden');
		}
		if (printer.num_temps > 0)
			showing.push(t[s][0]);
		else
			hiding.push(t[s][0]);
	}
	t = multiples[port].gpio;
	for (var s = 0; s < t.length; ++s) {
		while (t[s][1].length > printer.num_gpios) {
			var n = t[s][1].pop();
			for (var i = 0; i < n.length; ++i)
				t[s][0].removeChild(n[i]);
		}
		while (t[s][1].length < printer.num_gpios) {
			var n = t[s][3](t[s][1].length);
			t[s][1].push(n);
			for (var i = 0; i < n.length; ++i) {
				var src;
				if (n[i] instanceof Array)
					src = n[i];
				else
					src = [n[i]];
				for (var j = 0; j < src.length; ++j) {
					t[s][0].insertBefore(src[j], t[s][2]);
				}
			}
		}
		if (!(t[s][2] instanceof Comment)) {
			if (printer.num_gpios >= 2)
				t[s][2].RemoveClass('hidden');
			else
				t[s][2].AddClass('hidden');
		}
		if (printer.num_gpios > 0)
			showing.push(t[s][0]);
		else
			hiding.push(t[s][0]);
	}
	// Update table visibility.
	for (var h = 0; h < hiding.length; ++h)
		hiding[h].AddClass('hidden');
	for (var s = 0; s < showing.length; ++s)
		showing[s].RemoveClass('hidden');
	update_canvas_and_spans();
} // }}}

function update_space(index) { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	var e = get_element(printer, [['space', index], 'type']);
	e.ClearAll();
	e.AddText(space_types[printer.spaces[index].type]);
	update_float(printer, [['space', index], 'max_deviation']);
	update_float(printer, [['space', index], 'num_axes']);
	var t = multiples[port].axis[index];
	for (var s = 0; s < t.length; ++s) {
		while (t[s][1].length > printer.spaces[index].axis.length) {
			var n = t[s][1].pop();
			for (var i = 0; i < n.length; ++i)
				t[s][0].removeChild(n[i]);
		}
		while (t[s][1].length < printer.spaces[index].axis.length) {
			var n = t[s][3](t[s][1].length);
			t[s][1].push(n);
			for (var i = 0; i < n.length; ++i)
				t[s][0].insertBefore(n[i], t[s][2]);
		}
		if (!(t[s][2] instanceof Comment)) {
			if (printer.spaces[index].num_axes >= 2)
				t[s][2].RemoveClass('hidden');
			else
				t[s][2].AddClass('hidden');
		}
	}
	var t = multiples[port].motor[index];
	for (var s = 0; s < t.length; ++s) {
		while (t[s][1].length > printer.spaces[index].motor.length) {
			var n = t[s][1].pop();
			for (var i = 0; i < n.length; ++i)
				t[s][0].removeChild(n[i]);
		}
		while (t[s][1].length < printer.spaces[index].motor.length) {
			var n = t[s][3](t[s][1].length);
			t[s][1].push(n);
			for (var i = 0; i < n.length; ++i)
				t[s][0].insertBefore(n[i], t[s][2]);
		}
		if (!(t[s][2] instanceof Comment)) {
			if (printer.spaces[index].num_motors >= 2)
				t[s][2].RemoveClass('hidden');
			else
				t[s][2].AddClass('hidden');
		}
	}
	for (var a = 0; a < printer.spaces[index].num_axes; ++a) {
		update_float(printer, [['axis', [index, a]], 'park']);
		update_float(printer, [['axis', [index, a]], 'park_order']);
		update_float(printer, [['axis', [index, a]], 'max_v']);
		update_float(printer, [['axis', [index, a]], 'min']);
		update_float(printer, [['axis', [index, a]], 'max']);
		if (index == 1)
			update_float(printer, [['axis', [index, a]], 'multiplier']);
	}
	for (var m = 0; m < printer.spaces[index].num_motors; ++m) {
		update_pin([['motor', [index, m]], 'step_pin']);
		update_pin([['motor', [index, m]], 'dir_pin']);
		update_pin([['motor', [index, m]], 'enable_pin']);
		update_pin([['motor', [index, m]], 'limit_min_pin']);
		update_pin([['motor', [index, m]], 'limit_max_pin']);
		update_pin([['motor', [index, m]], 'sense_pin']);
		update_float(printer, [['motor', [index, m]], 'steps_per_m']);
		update_float(printer, [['motor', [index, m]], 'max_steps']);
		update_float(printer, [['motor', [index, m]], 'home_pos']);
		update_float(printer, [['motor', [index, m]], 'limit_v']);
		update_float(printer, [['motor', [index, m]], 'limit_a']);
		update_float(printer, [['motor', [index, m]], 'home_order']);
	}
	if (printer.spaces[index].type == TYPE_DELTA) {
		for (var d = 0; d < 3; ++d) {
			update_float(printer, [['motor', [index, d]], 'delta_axis_min']);
			update_float(printer, [['motor', [index, d]], 'delta_axis_max']);
			update_float(printer, [['motor', [index, d]], 'delta_rodlength']);
			update_float(printer, [['motor', [index, d]], 'delta_radius']);
		}
		update_float(printer, [['space', index], 'delta_angle']);
	}
	var showing = [], hiding = [], newhidetypes = [];
	for (var h = 0; h < hidetypes.length; ++h) {
		if (hidetypes[h][2].parentNode === null)
			continue;
		newhidetypes.push(hidetypes[h]);
		if (printer.spaces[hidetypes[h][1]].type != hidetypes[h][0]) {
			hidetypes[h][2].AddClass('hidden');
			hiding.push(hidetypes[h][2].parentNode);
		}
		else {
			hidetypes[h][2].RemoveClass('hidden');
			showing.push(hidetypes[h][2].parentNode);
		}
	}
	hidetypes = newhidetypes;
	for (var h = 0; h < hiding.length; ++h)
		hiding[h].AddClass('hidden');
	for (var s = 0; s < showing.length; ++s)
		showing[s].RemoveClass('hidden');
	update_canvas_and_spans();
} // }}}

function update_temp(index) { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	update_pin([['temp', index], 'power_pin']);
	update_pin([['temp', index], 'thermistor_pin']);
	update_float(printer, [['temp', index], 'R0']);
	update_float(printer, [['temp', index], 'R1']);
	update_float(printer, [['temp', index], 'Rc']);
	update_float(printer, [['temp', index], 'Tc']);
	update_float(printer, [['temp', index], 'beta']);
	//update_float(printer, [['temp', index], 'core_C']);
	//update_float(printer, [['temp', index], 'shell_C']);
	//update_float(printer, [['temp', index], 'transfer']);
	//update_float(printer, [['temp', index], 'radiation']);
	//update_float(printer, [['temp', index], 'power']);
	update_float(printer, [['temp', index], 'value', 'settemp']);
} // }}}

function update_gpio(index) { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	update_pin([['gpio', index], 'pin']);
	if (printer.gpios[index].state == 1)
		get_element(printer, [['gpio', index], 'state']).checked = true;
	else
		get_element(printer, [['gpio', index], 'state']).checked = false;
	var master = update_temprange([['gpio', index], 'master']);
	update_float(printer, [['gpio', index], 'value']);
	//if (master != 0xff) {
		//get_element(printer, [['gpio', index], 'state'], 0).AddClass('empty');
	//}
	//else {
		//get_element(printer, [['gpio', index], 'state'], 0).RemoveClass('empty');
	//}
} // }}}
// }}}

// Update helpers. {{{
function update_choice(id) { // {{{
	var value = get_value(printer, id);
	dbg(id);
	var list = choices[make_id(printer, id)][1];
	var container = get_element(printer, [null, 'container']);
	for (var i = 0; i < list.length; ++i) {
		if (list[i].containerclass && i != value)
			container.RemoveClass(list[i].containerclass);
	}
	if (value < list.length) {
		list[value].checked = true;
		if (list[value].containerclass)
			container.AddClass(list[value].containerclass);
	}
	else {
		for (var i = 0; i < list.length; ++i)
			list[i].checked = false;
	}
} // }}}

function update_toggle(id) { // {{{
	var v = get_value(printer, id);
	var e = get_element(printer, id);
	e.ClearAll();
	e.AddText(e.value[Number(v)]);
} // }}}

function update_pin(id) { // {{{
	get_element(printer, id).selectedIndex = get_value(printer, id) & 0xff;
	get_element(printer, id, 'valid').checked = Boolean(get_value(printer, id) & 0x100);
	get_element(printer, id, 'inverted').checked = Boolean(get_value(printer, id) & 0x200);
} // }}}

function update_float(printer, id) { // {{{
	var e = get_element(printer, id);
	e.ClearAll();
	e.AddText((get_value(printer, id) / e.factor).toFixed(e.digits));
} // }}}

function update_floats(id) { // {{{
	var value = get_value(printer, id);
	for (var i = 0; i < value.length; ++i) {
		var e = get_element(printer, id.concat([i]));
		e.value = String(value[i]);
	}
} // }}}

function update_temprange(id) { // {{{
	var value = get_value(printer, id);
	get_element(printer, id).selectedIndex = value != 255 ? 1 + value : 0;
	return value;
} // }}}
// }}}

// Builders. {{{
function pinrange(only_analog) { // {{{
	var ret = [];
	var pin = 0;
	if (!only_analog) {
		for (var i = 0; i < printer.num_digital_pins; ++i) {
			var node = Create('option').AddText('D' + String(i));
			node.value = String(pin);
			ret.push(node);
			pin += 1;
		}
	}
	for (var i = 0; i < printer.num_pins - printer.num_digital_pins; ++i) {
		var node = Create('option').AddText('A' + String(i));
		node.value = String(pin);
		ret.push(node);
		pin += 1;
	}
	return ret;
} // }}}

function temprange() { // {{{
	var node = Create('option');
	node.AddText('None');
	node.value = String(0xff);
	var ret = [node];
	for (var i = 0; i < printer.num_temps; ++i) {
		node = Create('option');
		node.AddText(temp_name(printer, i));
		node.value = String(i);
		ret.push(node);
	}
	return ret;
} // }}}

function create_space_type_select() { // {{{
	var ret = document.createElement('select');
	for (var o = 0; o < space_types.length; ++o)
		ret.AddElement('option').AddText(space_types[o]);
	return ret;
} // }}}

var choices = new Object;

function Choice(obj, options, classes, containerclasses) { // {{{
	var ret = [];
	var id = make_id(printer, obj);
	choices[id] = [obj, []];
	for (var i = 0; i < options.length; ++i) {
		var e = Create('input');
		e.type = 'radio';
		if (classes) {
			if (typeof classes == 'string')
				e.AddClass(classes);
			else
				e.AddClass(classes[i]);
		}
		choices[id][1].push(e);
		e.name = id;
		e.id = make_id(printer, obj, String(i));
		if (containerclasses)
			e.containerclass = containerclasses[i];
		e.value = String(i);
		e.printer = printer;
		e.addEventListener('click', function() {
		       	set_value(this.printer, choices[this.name][0], Number(this.value));
		}, false);
		var l = Create('label');
		if (classes) {
			if (typeof classes == 'string')
				l.AddClass(classes);
			else
				l.AddClass(classes[i]);
		}
		l.htmlFor = e.id;
		l.AddText(options[i]);
		ret.push(e, l);
	}
	return ret;
} // }}}

function space_name(index) { // {{{
	if (index === null)
		return 'All Spaces';
	return 'Space ' + String(index);
} // }}}

function axis_name(index1, index2) { // {{{
	if (index2 === null)
		return 'All axes ' + String(index1);
	return 'Axis ' + String(index1) + ';' + String(index2);
} // }}}

function motor_name(index1, index2) { // {{{
	if (index2 === null)
		return 'All motors ' + String(index1);
	return 'Motor ' + String(index1) + ';' + String(index2);
} // }}}

function delta_name(index1, index2) { // {{{
	if (index1 === null)
		return 'All Apexes';
	return 'Apex ' + String(index1) + ';' + String(index2);
} // }}}

function temp_name(printer, index) { // {{{
	if (index === null)
		return 'All Temps';
	if (index == printer.bed_id)
		return 'Bed';
	if (printer.spaces.length > 1 && index < printer.spaces[1].num_axes)
		return 'Extruder ' + String(index);
	return 'Temp ' + String(index);
} // }}}

function gpio_name(index) { // {{{
	if (index === null)
		return 'All Gpios';
	return index == 0 ? 'Fan' : 'Gpio ' + String(index);
} // }}}

function make_pin_title(title, content) { // {{{
	var t = document.createElement('tr');
	t.AddElement('th').AddText(title);
	return [t].concat(content);
} // }}}

Object.prototype.AddMultiple = function(type, template, all, arg) { // {{{
	var one = function(t, template, i, arg) {
		var ret = [];
		var part = template(i, arg, t);
		if (!(part instanceof Array))
			part = [part];
		for (var p = 0; p < part.length; ++p) {
			if (part[p] instanceof Comment)
				continue;
			if (part[p] instanceof Text) {
				if (!/^\s*$/.exec(part[p].data))
					alert('Non-empty text node in result of multiple: ' + part[p].data);
				continue;
			}
			if (i === null || arg === null)
				part[p].AddClass('all hidden');
			ret.push(part[p]);
		}
		if (ret.length < 1)
			ret.push(document.createComment(''));
		return ret;
	};
	var t = this;
	var last;
	if (all !== false)
		if (type != 'axis' && type != 'motor')
			last = one(t, template, null);
		else
			last = one(t, template, arg, null);
	else
		last = [document.createComment('')];
	for (var i = 0; i < last.length; ++i)
		t.appendChild(last[i]);
	if (type != 'axis' && type != 'motor')
		multiples[port][type].push([t, [], last[0], function(i) { return one(t, template, i); }]);
	else {
		if (multiples[port][type][arg] !== undefined)
			multiples[port][type][arg].push([t, [], last[0], function(i) { return one(t, template, arg, i); }]);
	}
	return t;
}; // }}}

function make_table() { // {{{
	var t = document.createElement('table');
	for (var r = 0; r < arguments.length; ++r) {
		if (arguments[r] instanceof Element) {
			t.Add(arguments[r]);
		}
		else {
			if (arguments[r][0] == 'axis' || arguments[r][0] == 'motor')
				multiples[port].space.push([t, [], t.Add(arguments[r][2]), arguments[r][1]]);
			else
				multiples[port][arguments[r][0]].push([t, [], t.Add(arguments[r][2]), arguments[r][1]]);
		}
	}
	t.AddMultipleTitles = function(titles, classes, mouseovers) {
		this.titles = this.AddElement('tr');
		for (var cell = 0; cell < titles.length; ++cell) {
			var th = this.titles.AddElement('th', classes[cell]);
			th.AddText(titles[cell]);
			if (mouseovers && mouseovers[cell])
				th.title = mouseovers[cell];
		}
		return this;
	};
	return t;
} // }}}

function make_tablerow(title, cells, classes, id, onlytype, index) { // {{{
	var ret = document.createElement('tr');
	if (id)
		ret.id = make_id(printer, id);
	ret.AddElement('th', classes[0]).AddText(title);
	for (var cell = 0; cell < cells.length; ++cell) {
		var current_cell;
		if (!classes[1] || classes[1] == 'string')
			current_cell = ret.AddElement('td', classes[1]);
		else
			current_cell = ret.AddElement('td', classes[1][cell]);
		if (id)
			current_cell.id = make_id(printer, id, cell);
		if (cells[cell] instanceof Element) {
			current_cell.Add(cells[cell]);
		}
		else {
			for (var e = 0; e < cells[cell].length; ++e)
				current_cell.Add(cells[cell][e]);
		}
	}
	if (onlytype !== undefined) {
		hidetypes.push([onlytype, index, ret]);
	}
	return ret;
} // }}}
// }}}

// Set helpers(to server). {{{
function set_pin(printer, id) { // {{{
	var value = get_element(printer, id).selectedIndex;
	var valid = get_element(printer, id, 'valid').checked;
	var inverted = get_element(printer, id, 'inverted').checked;
	set_value(printer, id, value + 0x100 * valid + 0x200 * inverted);
} // }}}

function set_file(printer, id) { // {{{
	var element = get_element(printer, id);
	if (element.files.length < 1) {
		alert('please select a file');
		return;
	}
	var reader = new FileReader();
	reader.name = element.files[0].name;
	reader.onloadend = function(e) {
		function errors(err) {
			if (err.length == 0)
				return;
			alert('Errors in lines: ' + err.join(', '));
		}
		if (this.readyState != FileReader.DONE)
			return;
		set_value(printer, id, this.result, errors, this.name);
	};
	reader.readAsBinaryString(element.files[0]);
} // }}}

function set_gpio_master(printer, index) { // {{{
	var id = [['gpio', index], 'master'];
	var e = get_element(printer, id);
	set_value(printer, id, Number(e.childNodes[e.selectedIndex].value));
} // }}}
// }}}

// Canvas functions. {{{
function move_axis(axes, amounts, pos) { // {{{
	if (pos === undefined)
		pos = [];
	if (pos.length < axes.length) {
		selected_printer.call('get_axis_pos', [0, axes[pos.length]], {}, function(result) {
			pos.push(result + amounts[pos.length]);
			move_axis(axes, amounts, pos);
		});
		return;
	}
	var target = new Object;
	for (var a = 0; a < axes.length; ++a)
		target[axes[a]] = pos[a];
	selected_printer.call('goto', [[target]], {'cb': true}, function() {
		selected_printer.call('wait_for_cb', [], {}, function() {
			update_canvas_and_spans();
		});
	});
}
// }}}

function set_reference(x, y, ctrl) { // {{{
	selected_printer.call('get_axis_pos', [0, 0], {}, function(current_x) {
		selected_printer.call('get_axis_pos', [0, 1], {}, function(current_y) {
			if (ctrl) {
				selected_printer.reference = [x, y];
				update_canvas_and_spans();
			}
			else {
				var dx = Math.cos(selected_printer.targetangle) * (x - selected_printer.reference[0]) - Math.sin(selected_printer.targetangle) * (y - selected_printer.reference[1]);
				var dy = Math.cos(selected_printer.targetangle) * (y - selected_printer.reference[1]) + Math.sin(selected_printer.targetangle) * (x - selected_printer.reference[0]);
				selected_printer.reference = [x, y];
				selected_printer.call('goto', [[[current_x + dx, current_y + dy]]], {'cb': true}, function() {
					selected_printer.call('wait_for_cb', [], {}, function() {
						update_canvas_and_spans();
					});
				});
			}
		});
	});
}
// }}}

function update_canvas_and_spans(space, axis) { // {{{
	var printer = selected_printer;
	// When called as a cb, space may not be undefined.
	if (axis === undefined) {
		space = 0;
		axis = 0;
	}
	while (space < printer.spaces.length && (axis >= printer.spaces[space].axis.length || space == 0 && axis >= 3)) {
		space += 1;
		axis = 0;
	}
	if (space < printer.spaces.length && space < 2) {
		printer.call('get_axis_pos', [space, axis], {}, function(x) {
			//dbg('update ' + space + ',' + axis + ':' + x);
			printer.spaces[space].axis[axis].current = x;
			update_float(printer, [['axis', [space, axis]], 'current']);
			update_canvas_and_spans(space, axis + 1);
		});
		return;
	}
	update_float(printer, [null, 'targetx']);
	update_float(printer, [null, 'targety']);
	update_float(printer, [null, 'targetz']);
	update_float(printer, [null, 'targetangle']);
	redraw_canvas();
}
// }}}

function redraw_canvas() { // {{{
	if (!selected_printer || selected_printer.spaces.length < 1 || selected_printer.spaces[0].axis.length < 3)
		return;
	var canvas = document.getElementById(make_id(selected_printer, [null, 'xymap']));
	var zcanvas = document.getElementById(make_id(selected_printer, [null, 'zmap']));
	var c = canvas.getContext('2d');
	var zc = zcanvas.getContext('2d');
	var box = document.getElementById(make_id(selected_printer, [null, 'map']));
	var extra_height = box.clientHeight - canvas.clientHeight;
	var printerwidth;
	var printerheight;
	var outline;
	switch (selected_printer.spaces[0].type) {
	case TYPE_CARTESIAN:
		var xaxis = selected_printer.spaces[0].axis[0];
		var yaxis = selected_printer.spaces[0].axis[1];
		printerwidth = 2 * Math.max(xaxis.max, -xaxis.min) + .010;
		printerheight = 2 * Math.max(yaxis.max, -yaxis.min) + .010;
		outline = function(c) {
			c.beginPath();
			// Why does this not work?
			//c.rect(xaxis.axis_min, yaxis.axis_min, xaxis.axis_max - xaxis.axis_min, yaxis.axis_max - y.axis_min);
			c.moveTo(xaxis.min, yaxis.min);
			c.lineTo(xaxis.min, yaxis.max);
			c.lineTo(xaxis.max, yaxis.max);
			c.lineTo(xaxis.max, yaxis.min);
			c.lineTo(xaxis.min, yaxis.min);
			c.stroke();
		};
		break;
	case TYPE_DELTA:
		var names = ['X', 'Y', 'Z'];
		var radius = [];
		var length = [];
		for (var a = 0; a < 3; ++a) {
			radius.push(selected_printer.spaces[0].motor[a].delta_radius);
			length.push(selected_printer.spaces[0].motor[a].delta_rodlength);
		}
		//var origin = [[radius[0], 0], [radius[1] * -.5, radius[1] * .8660254037844387], [radius[2] * -.5, radius[2] * -.8660254037844387]];
		//var dx = [0, -.8660254037844387, .8660254037844387];
		//var dy = [1, -.5, -.5];
		var origin = [[radius[0] * -.8660254037844387, radius[0] * -.5], [radius[1] * .8660254037844387, radius[1] * -.5], [0, radius[2]]];
		var dx = [.5, .5, -1];
		var dy = [-.8660254037844387, .8660254037844387, 0];
		var intersects = [];
		var intersect = function(x0, y0, r, x1, y1, dx1, dy1, positive) {
			// Find intersection of circle(x-x0)^2+(y-y0)^2==r^2 and line l=(x1,y1)+t(dx1,dy1); use positive of negative solution for t.
			// Return coordinate.
			var s = positive ? 1 : -1;
			var k = (dx1 * (x1 - x0) + dy1 * (y1 - y0)) / (dx1 * dx1 + dy1 * dy1);
			var t = s * Math.sqrt(r * r - (x1 - x0) * (x1 - x0) - (y1 - y0) * (y1 - y0) + k * k) - k;
			return [x1 + t * dx1, y1 + t * dy1];
		};
		var angles = [[null, null], [null, null], [null, null]];
		var maxx = 0, maxy = 0;
		for (var a = 0; a < 3; ++a) {
			intersects.push([]);
			for (var aa = 0; aa < 2; ++aa) {
				var A = (a + aa + 1) % 3;
				var point = intersect(origin[A][0], origin[A][1], length[A], origin[a][0], origin[a][1], dx[a], dy[a], aa);
				intersects[a].push(point);
				if (Math.abs(point[0]) > maxx)
					maxx = Math.abs(point[0]);
				if (Math.abs(point[1]) > maxy)
					maxy = Math.abs(point[1]);
				angles[A][aa] = Math.atan2(point[1] - origin[A][1], point[0] - origin[A][0]);
			}
		}
		outline = function(c) {
			c.save();
			c.rotate(-selected_printer.spaces[0].delta_angle);
			c.beginPath();
			c.moveTo(intersects[0][0][0], intersects[0][0][1]);
			for (var a = 0; a < 3; ++a) {
				var A = (a + 1) % 3;
				var B = (a + 2) % 3;
				c.lineTo(intersects[a][1][0], intersects[a][1][1]);
				c.arc(origin[B][0], origin[B][1], length[B], angles[B][1], angles[B][0], false);
			}
			c.closePath();
			c.stroke();
			for (var a = 0; a < 3; ++a) {
				var w = c.measureText(names[a]).width / 1000;
				c.beginPath();
				c.save();
				c.translate(origin[a][0] + dy[a] * .015 - w / 2, origin[a][1] - dx[a] * .015);
				c.rotate(selected_printer.spaces[0].delta_angle);
				c.scale(.001, -.001);
				c.fillText(names[a], 0, 0);
				c.restore();
			}
			c.restore();
		};
		var extra = c.measureText(names[0]).width / 1000 + .02;
		printerwidth = 2 * (maxx + extra);
		printerheight = 2 * (maxy + extra);
		break;
	}
	//var factor = Math.sqrt(printerwidth * printerwidth + printerheight * printerheight);
	var factor = Math.max(printerwidth, printerheight);
	canvas.style.height = canvas.clientWidth + 'px';
	zcanvas.style.height = canvas.clientWidth + 'px';
	canvas.width = canvas.clientWidth;
	canvas.height = canvas.clientWidth;
	var zratio = .15 / .85;
	zcanvas.width = canvas.clientWidth * zratio;
	zcanvas.height = canvas.clientWidth;

	var b = selected_printer.bbox;
	var true_pos = [selected_printer.spaces[0].axis[0].current, selected_printer.spaces[0].axis[1].current];

	c.save();
	// Clear canvas.
	c.clearRect(0, 0, canvas.width, canvas.width);

	c.translate(canvas.width / 2, canvas.width / 2);
	c.scale(canvas.width / factor, -canvas.width / factor);
	c.lineWidth = 1.5 * factor / canvas.width;

	get_pointer_pos_xy = function(printer, e) {
		var rect = canvas.getBoundingClientRect();
		var x = e.clientX - rect.left - canvas.width / 2;
		var y = e.clientY - rect.top - canvas.width / 2;
		x /= canvas.width / factor;
		y /= -canvas.width / factor;
		return [x, y];
	}

	// Draw outline.
	c.strokeStyle = '#888';
	c.fillStyle = '#888';
	outline(c);
	// Draw center.
	c.beginPath();
	c.moveTo(.001, 0);
	c.arc(0, 0, .001, 0, 2 * Math.PI);
	c.fillStyle = '#888';
	c.fill();

	// Draw current location.
	c.beginPath();
	c.fillStyle = '#44f';
	c.moveTo(true_pos[0] + .003, true_pos[1]);
	c.arc(true_pos[0], true_pos[1], .003, 0, 2 * Math.PI);
	c.fill();

	c.save();
	c.translate(selected_printer.targetx, selected_printer.targety);
	c.rotate(selected_printer.targetangle);

	c.beginPath();
	if (b[0][0] != b[0][1] && b[1][0] != b[1][1]) {
		// Draw print bounding box.
		c.rect(b[0][0], b[1][0], b[0][1] - b[0][0], b[1][1] - b[1][0]);

		// Draw tick marks.
		c.moveTo((b[0][1] + b[0][0]) / 2, b[1][0]);
		c.lineTo((b[0][1] + b[0][0]) / 2, b[1][0] + .005);

		c.moveTo((b[0][1] + b[0][0]) / 2, b[1][1]);
		c.lineTo((b[0][1] + b[0][0]) / 2, b[1][1] - .005);

		c.moveTo(b[0][0], (b[1][1] + b[1][0]) / 2);
		c.lineTo(b[0][0] + .005, (b[1][1] + b[1][0]) / 2);

		c.moveTo(b[0][1], (b[1][1] + b[1][0]) / 2);
		c.lineTo(b[0][1] - .005, (b[1][1] + b[1][0]) / 2);

		// Draw central cross.
		c.moveTo((b[0][1] + b[0][0]) / 2 - .005, (b[1][1] + b[1][0]) / 2);
		c.lineTo((b[0][1] + b[0][0]) / 2 + .005, (b[1][1] + b[1][0]) / 2);
		c.moveTo((b[0][1] + b[0][0]) / 2, (b[1][1] + b[1][0]) / 2 + .005);
		c.lineTo((b[0][1] + b[0][0]) / 2, (b[1][1] + b[1][0]) / 2 - .005);
	}

	// Draw zero.
	c.moveTo(.003, 0);
	c.arc(0, 0, .003, 0, 2 * Math.PI);

	// Update it on screen.
	c.strokeStyle = '#000';
	c.stroke();

	c.restore();
	c.restore();

	// Z graph.
	zc.clearRect(0, 0, zcanvas.width, zcanvas.height);
	var zaxis = selected_printer.spaces[0].axis[2];
	var d = (zaxis.max - zaxis.min) * .03;
	var zfactor = zcanvas.height / (zaxis.max - zaxis.min) * .9;
	zc.translate(zcanvas.width * .8, zcanvas.height * .05);
	zc.scale(zfactor, -zfactor);
	zc.translate(0, -zaxis.max);

	get_pointer_pos_z = function(printer, e) {
		var rect = zcanvas.getBoundingClientRect();
		var z = e.clientY - rect.top + zcanvas.height * .05;
		z /= -zfactor;
		z += zaxis.max;
		return z;
	}

	// Draw current position.
	zc.beginPath();
	zc.moveTo(0, selected_printer.spaces[0].axis[2].current);
	zc.lineTo(-d, selected_printer.spaces[0].axis[2].current - d);
	zc.lineTo(-d, selected_printer.spaces[0].axis[2].current + d);
	zc.closePath();
	zc.fillStyle = '#44f';
	zc.fill();

	// Draw Axes.
	zc.beginPath();
	zc.moveTo(0, zaxis.min);
	zc.lineTo(0, zaxis.max);
	zc.moveTo(0, 0);
	zc.lineTo(-d * 2, 0);
	zc.strokeStyle = '#888';
	zc.lineWidth = 1.2 / zfactor;
	zc.stroke();

	// Draw target position.
	zc.beginPath();
	zc.moveTo(0, selected_printer.targetz);
	zc.lineTo(-d, selected_printer.targetz - d);
	zc.lineTo(-d, selected_printer.targetz + d);
	zc.closePath();
	zc.strokeStyle = '#000';
	zc.lineWidth = 1.5 / zfactor;
	zc.stroke();
}
// }}}

function key_move(key, shift, ctrl) { // {{{
	var value = Number(document.getElementById('move_amount').value) / 1000;
	if (shift)
		value /= 10;
	var spanx = document.getElementById('move_span_0');
	var spany = document.getElementById('move_span_1');
	var spanz = document.getElementById('move_span_2');
	// 10: enter
	if (key == 13) {
		update_canvas_and_spans();
	}
	// 27: escape
	else if (key == 27) {
		switch_show(false, 'mover');
		return false;
	}
	// 33: pgup
	else if (key == 33)
		move_axis([2], [value]);
	// 34: pgdn
	else if (key == 34)
		move_axis([2], [-value]);
	// 37: left
	else if (key == 37)
		move_axis([0], [-value]);
	// 38: up
	else if (key == 38)
		move_axis([1], [value]);
	// 39: right
	else if (key == 39)
		move_axis([0], [value]);
	// 40: down
	else if (key == 40)
		move_axis([1], [-value]);
	/* numeric 0: reference point is 0,0.
	else if (key == 96)
		set_reference(0, 0, ctrl);
	// numeric 1: reference point is bottom left.
	else if (key == 97)
		set_reference(selected_printer.bbox[0][0], selected_printer.bbox[1][0], ctrl);
	// numeric 2: reference point is bottom center.
	else if (key == 98)
		set_reference((selected_printer.bbox[0][0] + selected_printer.bbox[0][1]) / 2, selected_printer.bbox[1][0], ctrl);
	// numeric 3: reference point is bottom right.
	else if (key == 99)
		set_reference(selected_printer.bbox[0][1], selected_printer.bbox[1][0], ctrl);
	// numeric 4: reference point is center left.
	else if (key == 100)
		set_reference(selected_printer.bbox[0][0], (selected_printer.bbox[1][0] + selected_printer.bbox[1][1]) / 2, ctrl);
	// numeric 5: reference point is center.
	else if (key == 101)
		set_reference((selected_printer.bbox[0][0] + selected_printer.bbox[0][1]) / 2, (selected_printer.bbox[1][0] + selected_printer.bbox[1][1]) / 2, ctrl);
	// numeric 6: reference point is center right.
	else if (key == 102)
		set_reference(selected_printer.bbox[0][1], (selected_printer.bbox[1][0] + selected_printer.bbox[1][1]) / 2, ctrl);
	// numeric 7: reference point is top left.
	else if (key == 103)
		set_reference(selected_printer.bbox[0][0], selected_printer.bbox[1][1], ctrl);
	// numeric 8: reference point is top center.
	else if (key == 104)
		set_reference((selected_printer.bbox[0][0] + selected_printer.bbox[0][1]) / 2, selected_printer.bbox[1][1], ctrl);
	// numeric 9: reference point is top right.
	else if (key == 105)
		set_reference(selected_printer.bbox[0][1], selected_printer.bbox[1][1], ctrl); */
	else {
		//dbg('other key:' + key);
		return true;
	}
	return false;
} // }}}

function start_move() { // {{{
	// Update bbox.
	var q = get_element(selected_printer, [null, 'queue']);
	selected_printer.bbox = [[0, 0], [0, 0]];
	for (var e = 0; e < q.options.length; ++e) {
		if (!q.options[e].selected)
			continue;
		var name = q.options[e].value;
		var item;
		for (item = 0; item < selected_printer.queue.length; ++item)
			if (selected_printer.queue[item][0] == name)
				break;
		if (item >= selected_printer.queue.length)
			continue;
		if (selected_printer.queue[item][1][0][0] < selected_printer.bbox[0][0])
			selected_printer.bbox[0][0] = selected_printer.queue[item][1][0][0];
		if (selected_printer.queue[item][1][0][1] > selected_printer.bbox[0][1])
			selected_printer.bbox[0][1] = selected_printer.queue[item][1][0][1];
		if (selected_printer.queue[item][1][1][0] < selected_printer.bbox[1][0])
			selected_printer.bbox[1][0] = selected_printer.queue[item][1][1][0];
		if (selected_printer.queue[item][1][1][1] > selected_printer.bbox[1][1])
			selected_printer.bbox[1][1] = selected_printer.queue[item][1][1][1];
	}
	update_canvas_and_spans();
} // }}}

function reset_position() { // {{{
	selected_printer.targetangle = 0;
	update_canvas_and_spans();
} // }}}

var drag = [[NaN, NaN], [NaN, NaN], [NaN, NaN], false];

function xydown(e) {
	var pos = get_pointer_pos_xy(selected_printer, e);
	drag[0][0] = pos[0];
	drag[1][0] = pos[1];
	selected_printer.call('get_axis_pos', [0, 0], {}, function(x) {
		selected_printer.call('get_axis_pos', [0, 1], {}, function(y) {
			drag[0][1] = x;
			drag[1][1] = y;
		});
	});
}

function xymove(e) {
	if (drag[3])
		return;
	if (!(e.buttons & 1)) {
		drag[0][1] = NaN;
		drag[1][1] = NaN;
		return;
	}
	var pos = get_pointer_pos_xy(selected_printer, e);
	var dx = pos[0] - drag[0][0];
	var dy = pos[1] - drag[1][0];
	if (isNaN(drag[0][1]) || isNaN(drag[1][1]))
		return;
	drag[3] = true;
	selected_printer.call('goto', [[[drag[0][1] + dx, drag[1][1] + dy]]], {}, function() { drag[3] = false; update_canvas_and_spans(); });
}

function zdown(e) {
	drag[2][0] = get_pointer_pos_z(selected_printer, e);
	selected_printer.call('get_axis_pos', [0, 2], {}, function(z) { drag[2][1] = z; });
}

function zmove(e) {
	if (drag[3])
		return;
	if (!(e.buttons & 1)) {
		drag[2][1] = NaN;
		return;
	}
	var dz = get_pointer_pos_z(selected_printer, e) - drag[2][0];
	if (isNaN(drag[2][1]))
		return;
	drag[3] = true;
	selected_printer.call('goto', [[{2: drag[2][1] + dz}]], {}, function() { drag[3] = false; update_canvas_and_spans(); });
}

// }}}

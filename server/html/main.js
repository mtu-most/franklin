/* main.js - clientside functions for Franklin
 * vim: set foldmethod=marker :
 * Copyright 2014 Michigan Technological University
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

// Variables.  {{{
var get_pointer_pos_xy = function() { return [NaN, NaN]; };
var get_pointer_pos_z = function() { return [NaN, NaN]; };
var ports;
var labels_element, printers_element;
var selected_port, selected_printer;
var script_cbs;
var type2plural = {space: 'spaces', temp: 'temps', gpio: 'gpios', axis: 'axes', motor: 'motors'};
var space_types = ['Cartesian', 'Delta', 'Polar', 'Extruder', 'Follower'];
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
	labels_element = document.getElementById('labels');
	printers_element = document.getElementById('printers');
	selected_port = null;
	selected_printer = null;
	setup();
	register_update('confirm', do_confirm);
	register_update('queue', do_queue);
	register_update('audioqueue', do_audio_queue);
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
	var setupbox = document.getElementById('setupbox');
	if (document.location.search.length > 0 && document.location.search[0] == '?') {
		var things = document.location.search.split('&');
		for (var t = 0; t < things.length; ++t) {
			var items = things[t].substring(1).split('=');
			if (items[0] == 'setup') {
				if (items[1] != '0')
					setupbox.checked = true;
				else
					setupbox.checked = false;
			}
		}
	}
	var container = document.getElementById('container');
	if (setupbox.checked)
		container.RemoveClass('nosetup');
	else
		container.AddClass('nosetup');
	var reading_temps = false;
	window.AddEvent('keypress', keypress);
	setInterval(function() {
		if (reading_temps || !selected_printer || selected_printer.disabling)
			return;
		reading_temps = true;
		var read = function(printer, num) {
			if (num >= printer.temps.length) {
				reading_temps = false;
				// Update temperature graph.
				var canvas = get_element(printer, [null, 'tempgraph']);
				var c = canvas.getContext('2d');
				canvas.height = canvas.clientHeight;
				canvas.width = canvas.clientWidth;
				var scale = canvas.height / (printer.temp_scale_max - printer.temp_scale_min);
				c.clearRect(0, 0, canvas.width, canvas.height);
				c.save();
				c.translate(0, canvas.height);
				c.scale(scale, -scale);
				c.translate(0, -printer.temp_scale_min);
				// Draw grid.
				c.beginPath();
				var step = 15;
				for (var t = step; t < 120; t += step) {
					var x = (t / (2 * 60) * canvas.width) / scale;
					c.moveTo(x, printer.temp_scale_min);
					c.lineTo(x, printer.temp_scale_max);
				}
				step = Math.pow(10, Math.floor(Math.log(printer.temp_scale_max - printer.temp_scale_min) / Math.log(10)));
				for (var y = step * Math.floor(printer.temp_scale_min / step); y < printer.temp_scale_max; y += step) {
					c.moveTo(0, y);
					c.lineTo(canvas.width / scale, y);
				}
				c.save();
				c.lineWidth = 1 / scale;
				var makedash = function(array) {
					if (c.setLineDash !== undefined) {
						for (var i = 0; i < array.length; ++i)
							array[i] /= scale;
						c.setLineDash(array);
					}
				};
				makedash([1, 4]);
				c.strokeStyle = '#444';
				c.stroke();
				c.restore();
				// Draw grid scale.
				c.save();
				c.scale(1 / scale, -1 / scale);
				for (var y = step * Math.floor(printer.temp_scale_min / step); y < printer.temp_scale_max; y += step) {
					c.moveTo(0, y);
					var text = y.toFixed(0);
					c.fillText(text, (step / 50) * scale, -(y + step / 50) * scale);
				}
				c.restore();
				// Draw data.
				var time = new Date();
				printer.temphistory.push(time);
				var cutoff = time - 2 * 60 * 1000;
				while (printer.temphistory[0] < cutoff) {
					printer.temphistory.shift();
					for (var t = 0; t < printer.temps.length; ++t)
						printer.temps[t].history.shift();
				}
				var x = function(t) {
					return ((t - cutoff) / (2 * 60 * 1000) * canvas.width) / scale;
				};
				var y = function(d) {
					if (isNaN(d))
						return printer.temp_scale_min - 2 / scale;
					if (!isFinite(d))
						return printer.temp_scale_max + 2 / scale;
					return d;
				};
				for (var t = 0; t < printer.temps.length; ++t) {
					// Draw measured data.
					var value = printer.temps[t].temp;
					printer.temps[t].history.push([value, printer.temps[t].value]);
					var data = printer.temps[t].history;
					c.beginPath();
					value = data[0][0];
					if (isNaN(printer.temps[t].beta)) {
						value *= printer.temps[t].Rc / 100000;
						value += printer.temps[t].Tc;
					}
					c.moveTo(x(printer.temphistory[0]), y(value));
					for (var i = 1; i < data.length; ++i) {
						value = data[i][0];
						if (isNaN(printer.temps[t].beta)) {
							value *= printer.temps[t].Rc / 100000;
							value += printer.temps[t].Tc;
						}
						c.lineTo(x(printer.temphistory[i]), y(value));
					}
					c.strokeStyle = ['#f00', '#00f', '#0f0', '#ff0', '#000'][t < 5 ? t : 4];
					c.lineWidth = 1 / scale;
					c.stroke();
					// Draw temp targets.
					c.moveTo(x(printer.temphistory[0]), y(data[0][1]));
					for (var i = 1; i < data.length; ++i)
						c.lineTo(x(printer.temphistory[i]), y(data[i][1]));
					c.save();
					makedash([4, 2]);
					c.stroke();
					c.restore();
					// Draw values of temp targets.
					var old = null;
					c.fillStyle = c.strokeStyle;
					c.save();
					c.scale(1 / scale, -1 / scale);
					for (var i = 1; i < data.length; ++i) {
						if (data[i][1] != old && (!isNaN(data[i][1]) || !isNaN(old))) {
							old = data[i][1];
							var pos;
							if (isNaN(data[i][1]) || data[i][1] < printer.temp_scale_min)
								pos = printer.temp_scale_min;
							else if (data[i][1] / scale >= printer.temp_scale_max / scale - 12)
								pos = (printer.temp_scale_max / scale - 5) * scale;
							else
								pos = data[i][1];
							var text = data[i][1].toFixed(0);
							c.fillText(text, (x(printer.temphistory[i])) * scale, -(y(pos) + step / 50) * scale);
						}
					}
					c.restore();
				}
				c.restore();
				// Update everything else.
				update_canvas_and_spans(printer);
				return;
			}
			printer.call('readtemp', [num], {}, function(t) {
				if (num < printer.temps.length) {
					printer.temps[num].temp = t;
					var e = get_element(printer, [['temp', num], 'temp']);
					// Only update if printer still exists.
					if (e)
						e.ClearAll().AddText(t.toFixed(1));
				}
				read(printer, num + 1);
			});
		};
		read(selected_printer, 0);
	}, 400);
} // }}}

function make_id(printer, id, extra) { // {{{
	// [null, 'num_temps']
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
			// [null, 'num_temps']
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
				if ((id[0][0] != 'motor' || id[1].substr(0, 6) != 'delta_') && (id[0][0] != 'axis' || id[1].substr(0, 9) != 'extruder_') && (id[0][0] != 'motor' || id[1].substr(0, 9) != 'follower_')) {
					for (var n = 0; n < printer.spaces[id[0][1][0]]['num_' + type2plural[id[0][0]]]; ++n)
						printer.call('set_' + id[0][0], [[id[0][1][0], n]], obj, reply);
				}
				else if (id[0][0] == 'motor') {
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(6)] = value;
					var o = {};
					for (var n = 0; n < printer.spaces[id[0][1][0]].num_motors; ++n)
						o[n] = obj;
					printer.call('set_space', [id[0][1][0]], {delta: o}, reply);
				}
				else if (id[1][0] == 'e') {
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(9)] = value;
					var o = {};
					for (var n = 0; n < printer.spaces[id[0][1][0]].num_axes; ++n)
						o[n] = obj;
					printer.call('set_space', [id[0][1][0]], {extruder: o}, reply);
				}
				else {
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(9)] = value;
					var o = {};
					for (var n = 0; n < printer.spaces[id[0][1][0]].num_axes; ++n)
						o[n] = obj;
					printer.call('set_motor', [id[0][1][0]], {follower: o}, reply);
				}
			}
			else if ((id[0][0] != 'motor' || id[1].substr(0, 6) != 'delta_') && (id[0][0] != 'axis' || id[1].substr(0, 9) != 'extruder_') && (id[0][0] != 'motor' || id[1].substr(0, 9) != 'follower_')) {
				// [['space', 1], 'num_axes']
				// [['axis', [0, 1]], 'offset']
				printer.call('set_' + id[0][0], [id[0][1]], obj, reply);
			}
			else if (id[0][0] == 'motor') {
				if (id[1][0] != 'f') {
					// [['motor', [0, 1]], 'delta_radius']
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(6)] = value;
					var o = {};
					o[id[0][1][1]] = obj;
					printer.call('set_space', [id[0][1][0]], {delta: o}, reply);
				}
				else {
					// [['motor', [0, 1]], 'delta_radius']
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(9)] = value;
					var o = {};
					o[id[0][1][1]] = obj;
					printer.call('set_space', [id[0][1][0]], {follower: o}, reply);
				}
			}
			else {
				// [['motor', [0, 1]], 'delta_radius']
				obj = {};	// obj was wrong in this case.
				obj[id[1].substr(9)] = value;
				var o = {};
				o[id[0][1][1]] = obj;
				printer.call('set_space', [id[0][1][0]], {extruder: o}, reply);
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
	if (selected_printer !== null && selected_printer !== undefined) {
		update_state(get_value(printer, [null, 'status']));
	}
	else {
		update_state(null);
	}
} // }}}

function upload_buttons(port, ul, buttons) { // {{{
	for (var b = 0; b < buttons.length; ++b) {
		var li = ul.AddElement('li');
		// Add the upload button.
		var button = li.AddElement('button');
		button.type = 'button';
		button.target = buttons[b][0];
		button.onclick = function() {
			rpc.call('upload', [port, this.target], {}, function(ret) { alert('upload done: ' + ret);});
		};
		button.AddText(buttons[b][1]);
	}
}
// }}}

function floatkey(event, element) { // {{{
	if (event.ctrlKey || event.altKey)
		return;
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

function add_name(type, index1, index2) { // {{{
	while (printer.names[type].length <= index1) {
		printer.names[type].push([]);
		printer.name_values[type].push([]);
	}
	while (printer.names[type][index1].length <= index2) {
		printer.names[type][index1].push([]);
		printer.name_values[type][index1].push('');
	}
	var ret = Create('span', 'name').AddText(printer.name_values[type][index1][index2]);
	printer.names[type][index1][index2].push(ret);
	return ret;
} // }}}

function set_name(printer, type, index1, index2, name) { // {{{
	printer.name_values[type][index1][index2] = name;
	for (var i = 0; i < printer.names[type][index1][index2].length; ++i)
		printer.names[type][index1][index2][i].ClearAll().AddText(name);
} // }}}

function toggle_setup() { // {{{
	var e = document.getElementById('setupbox');
	var c = document.getElementById('container');
	if (e.checked)
		c.RemoveClass('nosetup');
	else
		c.AddClass('nosetup');
} // }}}

function exit() { // {{{
	if (confirm('Do you really want to shut down?'))
		rpc.call('exit');
} // }}}
// }}}

// Queue functions.  {{{
function queue_deselect(printer) { // {{{
	var e = get_element(printer, [null, 'queue']);
	for (var i = 1; i < e.options.length; ++i) {
		e.options[i].selected = false;
	}
}
// }}}

function queue_up(printer) { // {{{
	var e = get_element(printer, [null, 'queue']);
	for (var i = 1; i < e.options.length; ++i) {
		if (e.options[i].selected) {
			var cur = e.options[i];
			var prev = e.options[i - 1];
			e.insertBefore(e.removeChild(cur), prev);
		}
	}
}
// }}}

function queue_down(printer) { // {{{
	var e = get_element(printer, [null, 'queue']);
	for (var i = e.options.length - 2; i >= 0; --i) {
		if (e.options[i].selected) {
			var cur = e.options[i];
			var next = e.options[i + 1];
			e.insertBefore(e.removeChild(next), cur);
		}
	}
}
// }}}

function queue_del(printer) { // {{{
	var e = get_element(printer, [null, 'queue']);
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
		printer.call(a, [get_queue(printer), angle * 180 / Math.PI], {});
	};
	if (get_element(printer, [null, 'probebox']).checked)
		action('queue_probe');
	else
		action('queue_print');
}
// }}}

function audio_play(printer) { // {{{
	var e = get_element(printer, [null, 'audio']);
	var o = e.options[e.selectedIndex];
	if (o === undefined)
		return;
	printer.call('audio_play', [o.value]);
}

// }}}
function audio_del(printer) { // {{{
	var e = get_element(printer, [null, 'audio']);
	var o = e.options[e.selectedIndex];
	if (o === undefined)
		return;
	printer.call('audio_del', [o.value]);
}
// }}}
// }}}

// Non-update events. {{{
function new_port() { // {{{
	var p = port;
	var nop = NoPrinter();
	ports[p] = [Label(null), nop, null, null];
	ports[p][0].AddClass('setup');
	labels_element.Add(ports[p][0]);
	printers_element.Add(ports[p][1]);
	rpc.call('upload_options', [p], {}, function(options) {
		upload_buttons(p, nop.options, options);
	});
} // }}}

function new_printer() { // {{{
	printer.names = {space: [], axis: [], motor: [], temp: [], gpio: [], unit: []};
	printer.name_values = {space: [], axis: [], motor: [], temp: [], gpio: [], unit: []};
	printer.temphistory = [];
	printer.targetangle = 0;
	printer.nextangle = null;
	printer.disabling = false;
	printer.temptargets = [];
	printer.hidetypes = [];
	printer.tables = [];
	printer.idgroups = {bed: [], fan: [], spindle: []};
	printer.multiples = {space: [], temp: [], gpio: [], axis: [], motor: []};
	ports[port][2] = Printer();
	printers_element.Add(ports[port][2]);
	ports[port][0].RemoveClass('setup');
	ports[port][1].RemoveClass('notconnected');
	ports[port][1].AddClass('connected');
	ports[port][2].RemoveClass('notconnected');
	ports[port][2].AddClass('connected');
	update_globals();
	for (var i = 0; i < printer.spaces.length; ++i)
		update_space(i);
	for (var i = 0; i < printer.num_temps; ++i)
		update_temp(i);
	for (var i = 0; i < printer.num_gpios; ++i)
		update_gpio(i);
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
	if (ports[port] === undefined)
		return;
	ports[port][1].RemoveClass('connected');
	ports[port][1].AddClass('notconnected');
	ports[port][0].AddClass('setup');
	printers_element.removeChild(ports[port][2]);
	ports[port][2] = null;
	ports[port][3] = null;
	labels_element.removeChild(ports[port][0]);
	ports[port][0] = Label(null);
	labels_element.Add(ports[port][0]);
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
	update_canvas_and_spans(printer);
	var e = get_element(printer, [null, 'confirm']);
	e.ClearAll();
	if (id === null) {
		e.AddClass('hidden');
		return;
	}
	e.RemoveClass('hidden');
	// Cancel button.
	var button = e.AddElement('button', 'confirmabort').AddText('Abort').AddEvent('click', function() {
		var e = get_element(this.printer, [null, 'confirm']);
		e.ClearAll();
		e.AddClass('hidden');
		this.printer.call('confirm', [this.confirm_id, false], {});
	});
	button.type = 'button';
	button.printer = printer;
	button.confirm_id = id;
	// Message
	e.AddText(message);
	// Ok button.
	button = e.AddElement('button', 'confirmok').AddText('Ok').AddEvent('click', function() {
		var e = get_element(this.printer, [null, 'confirm']);
		e.ClearAll();
		e.AddClass('hidden');
		this.printer.call('confirm', [this.confirm_id, true], {});
	});
	button.type = 'button';
	button.printer = printer;
	button.confirm_id = id;
} // }}}

function do_queue() { // {{{
	var e = get_element(printer, [null, 'queue']);
	var q = [];
	var must_deselect = printer.queue.length > e.options.length;
	var no_select = e.options.length == 0;
	for (var i = 0; i < printer.queue.length; ++i)
		q.push(printer.queue[i][0]);
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
		rm.push(e.options[item]);
	}
	for (var item = 0; item < rm.length; ++rm)
		e.removeChild(rm[item]);
	for (var i = 0; i < q.length; ++i) {
		var option = e.AddElement('option');
		option.AddText(q[i]);
		option.value = q[i];
		if (i == 0 || !no_select)
			option.selected = true;
	}
	start_move(printer);
} // }}}

function do_audio_queue() { // {{{
	var e = get_element(printer, [null, 'audio']);
	var q = [];
	for (var i = 0; i < printer.audioqueue.length; ++i)
		q.push(printer.audioqueue[i]);
	var rm = [];
	for (var item = 0; item < e.options.length; ++item) {
		var i;
		for (i = 0; i < q.length; ++i) {
			if (q[i] == e.options[item].value) {
				q.splice(i, 1);
				i = -1;
				break;
			}
		}
		if (i < 0) {
			continue;
		}
		rm.push(e.options[item]);
	}
	for (var item = 0; item < rm.length; ++rm)
		e.removeChild(rm[item]);
	for (var i = 0; i < q.length; ++i) {
		var option = e.AddElement('option');
		option.AddText(q[i]);
		option.value = q[i];
		option.selected = true;
	}
	start_move(printer);
} // }}}
// }}}

// Update events(from server). {{{
function update_globals() { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	if (ports[port][3] != printer.uuid) {
		labels_element.removeChild(ports[port][0]);
		ports[port][0] = Label(printer);
		labels_element.Add(ports[port][0]);
		ports[port][3] = printer.uuid;
		select_printer();
	}
	get_element(printer, [null, 'export']).href = encodeURIComponent(printer.profile) + '.ini?port=' + encodeURIComponent(port);
	update_float(printer, [null, 'num_temps']);
	update_float(printer, [null, 'num_gpios']);
	update_pin([null, 'led_pin']);
	update_pin([null, 'stop_pin']);
	update_pin([null, 'probe_pin']);
	update_pin([null, 'spiss_pin']);
	update_float(printer, [null, 'probe_dist']);
	update_float(printer, [null, 'probe_safe_dist']);
	update_float(printer, [null, 'timeout']);
	update_float(printer, [null, 'feedrate']);
	update_float(printer, [null, 'max_deviation']);
	update_float(printer, [null, 'max_v']);
	update_float(printer, [null, 'targetx']);
	update_float(printer, [null, 'targety']);
	update_float(printer, [null, 'zoffset']);
	update_checkbox(printer, [null, 'store_adc']);
	update_checkbox(printer, [null, 'park_after_print']);
	update_checkbox(printer, [null, 'sleep_after_print']);
	update_checkbox(printer, [null, 'cool_after_print']);
	update_str(printer, [null, 'spi_setup']);
	update_float(printer, [null, 'temp_scale_min']);
	update_float(printer, [null, 'temp_scale_max']);
	set_name(printer, 'unit', 0, 0, printer.unit_name);
	// IDs.
	var ids = ['bed', 'fan', 'spindle'];
	for (var i = 0; i < ids.length; ++i) {
		var group = printer.idgroups[ids[i]];
		for (var e = 0; e < group.length; ++e) {
			var obj = group[e];
			obj.checked = obj.obj[0][1] == printer[ids[i] + '_id'];
		}
	}
	update_state(get_value(printer, [null, 'status']));
	var m = printer.multiples;
	for (var t = 0; t < m.space.length; ++t) {
		// Add table rows.
		while (m.space[t].tr.length < printer.spaces.length) { // TODO: set num spaces at initialization.
			var n = m.space[t].create(m.space[t].tr.length);
			m.space[t].tr.push(n);
			m.space[t].table.insertBefore(n, m.space[t].after);
		}
		if (!(m.space[t].after instanceof Comment)) {
			m.space[t].after.RemoveClass('hidden');
		}
	}
	for (var t = 0; t < m.axis.length; ++t) {
		while (m.axis[t].space.length < printer.spaces.length) { // TODO: set num spaces at initialization.
			var n;
			if (m.axis[t].all)
				n = m.axis[t].create(m.axis[t].space.length, null);
			else
				n = document.createComment('');
			m.axis[t].table.insertBefore(n, m.axis[t].after);
			m.axis[t].space.push({tr: [], after: n});
		}
	}
	for (var t = 0; t < m.motor.length; ++t) {
		while (m.motor[t].space.length < printer.spaces.length) { // TODO: set num spaces at initialization.
			var n;
			if (m.motor[t].all)
				n = m.motor[t].create(m.motor[t].space.length, null);
			else
				n = document.createComment('');
			m.motor[t].table.insertBefore(n, m.motor[t].after);
			m.motor[t].space.push({tr: [], after: n});
		}
	}
	for (var t = 0; t < m.temp.length; ++t) {
		while (m.temp[t].tr.length > printer.num_temps)
			m.temp[t].table.removeChild(m.temp[t].tr.pop());
		while (m.temp[t].tr.length < printer.num_temps) {
			var n = m.temp[t].create(m.temp[t].tr.length);
			m.temp[t].tr.push(n);
			m.temp[t].table.insertBefore(n, m.temp[t].after);
		}
		if (!(m.temp[t].after instanceof Comment)) {
			if (printer.num_temps >= 2)
				m.temp[t].after.RemoveClass('hidden');
			else
				m.temp[t].after.AddClass('hidden');
		}
	}
	for (var t = 0; t < m.gpio.length; ++t) {
		while (m.gpio[t].tr.length > printer.num_gpios)
			m.gpio[t].table.removeChild(m.gpio[t].tr.pop());
		while (m.gpio[t].tr.length < printer.num_gpios) {
			var n = m.gpio[t].create(m.gpio[t].tr.length);
			m.gpio[t].tr.push(n);
			m.gpio[t].table.insertBefore(n, m.gpio[t].after);
		}
		if (!(m.gpio[t].after instanceof Comment)) {
			if (printer.num_gpios >= 2)
				m.gpio[t].after.RemoveClass('hidden');
			else
				m.gpio[t].after.AddClass('hidden');
		}
	}
	update_profiles(printer);
	update_table_visibility();
	update_canvas_and_spans(printer);
} // }}}

function update_space(index) { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	set_name(printer, 'space', index, 0, printer.spaces[index].name);
	if (index == 0) {
		var e = get_element(printer, [['space', 0], 'type']);
		e.ClearAll();
		e.AddText(space_types[printer.spaces[index].type]);
	}
	update_float(printer, [['space', index], 'num_axes']);
	var m = printer.multiples;
	for (var t = 0; t < m.axis.length; ++t) {
		while (m.axis[t].space[index].tr.length > printer.spaces[index].axis.length)
			m.axis[t].table.removeChild(m.axis[t].space[index].tr.pop());
		while (m.axis[t].space[index].tr.length < printer.spaces[index].axis.length) {
			var tr = m.axis[t].create(index, m.axis[t].space[index].tr.length);
			m.axis[t].space[index].tr.push(tr);
			m.axis[t].table.insertBefore(tr, m.axis[t].space[index].after);
		}
		if (!(m.axis[t].space[index].after instanceof Comment)) {
			if (printer.spaces[index].num_axes >= 2)
				m.axis[t].space[index].after.RemoveClass('hidden');
			else
				m.axis[t].space[index].after.AddClass('hidden');
		}
	}
	for (var t = 0; t < m.motor.length; ++t) {
		while (m.motor[t].space[index].tr.length > printer.spaces[index].motor.length)
			m.motor[t].table.removeChild(m.motor[t].space[index].tr.pop());
		while (m.motor[t].space[index].tr.length < printer.spaces[index].motor.length) {
			var tr = m.motor[t].create(index, m.motor[t].space[index].tr.length);
			m.motor[t].space[index].tr.push(tr);
			m.motor[t].table.insertBefore(tr, m.motor[t].space[index].after);
		}
		if (!(m.motor[t].space[index].after instanceof Comment)) {
			if (printer.spaces[index].num_axes >= 2)
				m.motor[t].space[index].after.RemoveClass('hidden');
			else
				m.motor[t].space[index].after.AddClass('hidden');
		}
	}
	for (var a = 0; a < printer.spaces[index].num_axes; ++a) {
		set_name(printer, 'axis', index, a, printer.spaces[index].axis[a].name);
		if (index == 0) {
			update_float(printer, [['axis', [index, a]], 'park']);
			update_float(printer, [['axis', [index, a]], 'park_order']);
			update_float(printer, [['axis', [index, a]], 'min']);
			update_float(printer, [['axis', [index, a]], 'max']);
		}
		if (index == 1)
			update_float(printer, [['axis', [index, a]], 'multiplier']);
	}
	for (var m = 0; m < printer.spaces[index].num_motors; ++m) {
		set_name(printer, 'motor', index, m, printer.spaces[index].motor[m].name);
		update_pin([['motor', [index, m]], 'step_pin']);
		update_pin([['motor', [index, m]], 'dir_pin']);
		update_pin([['motor', [index, m]], 'enable_pin']);
		update_pin([['motor', [index, m]], 'limit_min_pin']);
		update_pin([['motor', [index, m]], 'limit_max_pin']);
		update_float(printer, [['motor', [index, m]], 'steps_per_unit']);
		if (index != 1)
			update_float(printer, [['motor', [index, m]], 'home_pos']);
		update_float(printer, [['motor', [index, m]], 'limit_v']);
		update_float(printer, [['motor', [index, m]], 'limit_a']);
		if (index != 1)
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
	if (printer.spaces[index].type == TYPE_POLAR) {
		update_float(printer, [['space', index], 'polar_max_r']);
	}
	if (printer.spaces[index].type == TYPE_EXTRUDER) {
		for (var d = 0; d < printer.spaces[index].axis.length; ++d) {
			update_float(printer, [['axis', [index, d]], 'extruder_dx']);
			update_float(printer, [['axis', [index, d]], 'extruder_dy']);
			update_float(printer, [['axis', [index, d]], 'extruder_dz']);
		}
	}
	if (printer.spaces[index].type == TYPE_FOLLOWER) {
		for (var d = 0; d < printer.spaces[index].motor.length; ++d) {
			update_float(printer, [['motor', [index, d]], 'follower_space']);
			update_float(printer, [['motor', [index, d]], 'follower_motor']);
		}
	}
	var newhidetypes = [];
	function hide_check(type, onlytype) {
		if (typeof onlytype == 'number')
			return type != onlytype;
		for (var i = 0; i < onlytype.length; ++i) {
			if (onlytype[i] == type)
				return false;
		}
		return true;
	}
	for (var h = 0; h < printer.hidetypes.length; ++h) {
		var ht = printer.hidetypes[h];
		if (!ht.tr.parentNode || !ht.tr.parentNode.parentNode) {
			continue;
		}
		newhidetypes.push(ht);
		if (hide_check(printer.spaces[ht.index].type, ht.only)) {
			ht.tr.AddClass('hidden');
		}
		else {
			ht.tr.RemoveClass('hidden');
		}
	}
	printer.hidetypes = newhidetypes;
	update_table_visibility();
	update_canvas_and_spans(printer);
} // }}}

function update_temp(index) { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	set_name(printer, 'temp', index, 0, printer.temps[index].name);
	update_pin([['temp', index], 'heater_pin']);
	update_pin([['temp', index], 'fan_pin']);
	update_pin([['temp', index], 'thermistor_pin']);
	update_float(printer, [['temp', index], 'fan_duty']);
	update_float(printer, [['temp', index], 'heater_limit']);
	update_float(printer, [['temp', index], 'fan_limit']);
	update_float(printer, [['temp', index], 'fan_temp']);
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
	set_name(printer, 'gpio', index, 0, printer.gpios[index].name);
	update_pin([['gpio', index], 'pin']);
	if (printer.gpios[index].reset < 2)
		get_element(printer, [['gpio', index], 'statespan']).RemoveClass('input');
	else
		get_element(printer, [['gpio', index], 'statespan']).AddClass('input');
	get_element(printer, [['gpio', index], 'state']).checked = printer.gpios[index].value;
	get_element(printer, [['gpio', index], 'reset']).selectedIndex = printer.gpios[index].reset;
	update_float(printer, [['gpio', index], 'duty']);
} // }}}
// }}}

// Update helpers. {{{
function update_table_visibility() { // {{{
	for (var t = 0; t < printer.tables.length; ++t) {
		var show = false;
		// Start at 1: skip title row.
		for (var c = 1; c < printer.tables[t].children.length; ++c) {
			if (!printer.tables[t].children[c].HaveClass('hidden')) {
				show = true;
				break;
			}
		}
		if (show) {
			printer.tables[t].RemoveClass('hidden');
		}
		else {
			printer.tables[t].AddClass('hidden');
		}
	}
} // }}}

function update_choice(id) { // {{{
	var value = get_value(printer, id);
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
	var e = get_element(printer, id);
	e.selectedIndex = get_value(printer, id) & 0xff;
	get_element(printer, id, 'valid').checked = Boolean(get_value(printer, id) & 0x100);
	if (!e.analog)
		get_element(printer, id, 'inverted').checked = Boolean(get_value(printer, id) & 0x200);
} // }}}

function update_float(printer, id) { // {{{
	var e = get_element(printer, id);
	e.ClearAll();
	e.AddText((get_value(printer, id) / e.factor).toFixed(e.digits));
} // }}}

function update_checkbox(printer, id) { // {{{
	var e = get_element(printer, id);
	e.checked = get_value(printer, id);
} // }}}

function update_str(printer, id) { // {{{
	var e = get_element(printer, id);
	e.ClearAll().AddText(get_value(printer, id));
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

function update_profiles(prt) { // {{{
	prt.call('list_profiles', [], {}, function(profiles) {
		var selector = get_element(prt, [null, 'profiles']);
		selector.ClearAll();
		for (var i = 0; i < profiles.length; ++i) {
			selector.AddElement('option').AddText(profiles[i]).value = profiles[i];
			if (prt.profile == profiles[i])
				selector.selectedIndex = i;
		}
	});
} // }}}

function update_state(state) { // {{{
	var c = document.getElementById('container');
	var pre;
	c.RemoveClass('idle printing paused');
	if (state === null) {
		c.AddClass('idle');
		pre = '';
	}
	else if (state) {
		c.AddClass('printing');
		pre = '# ';
	}
	else {
		c.AddClass('paused');
		pre = '+ ';
	}
	var doctitle;
	if (selected_printer !== null && selected_printer !== undefined)
		doctitle = selected_printer.profile;
	else
		doctitle = '[' + port + ']';
	document.title = pre + doctitle + ' - Franklin';
} // }}}
// }}}

// Builders. {{{
function pinrange(analog) { // {{{
	var ret = [];
	var pin = 0;
	if (analog) {
		for (var i = 0; i < printer.analog_pin_names.length; ++i) {
			var node = Create('option').AddText(printer.analog_pin_names[i]);
			node.value = String(pin);
			ret.push(node);
			pin += 1;
		}
	}
	else {
		for (var i = 0; i < printer.digital_pin_names.length; ++i) {
			var node = Create('option').AddText(printer.digital_pin_names[i]);
			node.value = String(pin);
			ret.push(node);
			pin += 1;
		}
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
		node.Add(temp_name(printer, i));
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
		var l = Create('label');
		var e = l.AddElement('input');
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
		if (classes) {
			if (typeof classes == 'string')
				l.AddClass(classes);
			else
				l.AddClass(classes[i]);
		}
		l.AddText(options[i]);
		ret.push(l);
	}
	return ret;
} // }}}

function UnitTitle(title, post, pre) { // {{{
	var ret = Create('span').AddText(title + ' (' + (pre ? pre : ''));
	ret.Add(add_name('unit', 0, 0));
	return ret.AddText((post ? post : '') + ')');
} // }}}

function space_name(index) { // {{{
	if (index === null)
		return 'all spaces';
	return add_name('space', index, 0);
} // }}}

function axis_name(index1, index2) { // {{{
	if (index2 === null)
		return ['all ', space_name(index1), ' axes'];
	return add_name('axis', index1, index2);
} // }}}

function motor_name(index1, index2) { // {{{
	if (index2 === null)
		return ['all ', space_name(index1), ' motors'];
	return add_name('motor', index1, index2);
} // }}}

function delta_name(index1, index2) { // {{{
	if (index1 === null)
		return 'all apexes';
	return motor_name(index1, index2);
} // }}}

function temp_name(printer, index) { // {{{
	if (index === null)
		return 'all temps';
	return add_name('temp', index, 0);
} // }}}

function gpio_name(index) { // {{{
	if (index === null)
		return 'all gpios';
	return add_name('gpio', index, 0);
} // }}}

function make_pin_title(title, content) { // {{{
	var container = document.createElement('tbody');
	var t = container.AddElement('tr');
	if (typeof title == 'string')
		t.AddElement('th').AddText(title);
	else
		t.AddElement('th').Add(title);
	for (var i = 0; i < content.length; ++i)
		container.Add(content[i]);
	return container;
} // }}}

Object.defineProperty(Object.prototype, 'AddMultiple', { // {{{
	enumerable: false,
	configurarable: true,
	writable: true,
	value: function(type, template, all, forbidden) {
		var one = function(t, template, i, arg) {
			var ret = template(i, arg, t);
			if (ret !== null && (i === null || arg === null))
				ret.AddClass('all hidden');
			return ret;
		};
		var me = this;
		var last;
		if (all !== false && type != 'axis' && type != 'motor')
			last = one(me, template, null);
		else
			last = document.createComment('');
		if (type != 'axis' && type != 'motor') {
			me.appendChild(last);
			printer.multiples[type].push({table: me, tr: [], after: last, create: function(i) { return one(me, template, i) || document.createComment(''); }});
		}
		else {
			me.appendChild(last);
			printer.multiples[type].push({table: me, space: [], after: last, all: all != false, create: function(i, arg) { return (i != forbidden && one(me, template, i, arg)) || document.createComment(''); }});
		}
		return me;
	}
}); // }}}

function make_table() { // {{{
	var t = document.createElement('table');
	printer.tables.push(t);
	t.AddMultipleTitles = function(titles, classes, mouseovers) {
		this.titles = this.AddElement('tr');
		for (var cell = 0; cell < titles.length; ++cell) {
			var th;
			if (classes[cell])
				th = this.titles.AddElement('th', classes[cell]);
			else
				th = this.titles.AddElement('th');
			th.Add(titles[cell]);
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
	ret.AddElement('th', classes[0]).Add(title);
	for (var cell = 0; cell < cells.length; ++cell) {
		var current_cell;
		if (!classes[1])
			current_cell = ret.AddElement('td');
		else if (classes[1] == 'string')
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
	if (onlytype !== undefined && index != null) {
		printer.hidetypes.push({only: onlytype, index: index, tr: ret});
	}
	return ret;
} // }}}
// }}}

// Set helpers (to server). {{{
function set_pin(printer, id) { // {{{
	var e = get_element(printer, id);
	var valid = get_element(printer, id, 'valid').checked;
	var inverted;
	if (e.analog)
		inverted = false;
	else
		inverted = get_element(printer, id, 'inverted').checked;
	set_value(printer, id, e.selectedIndex + 0x100 * valid + 0x200 * inverted);
} // }}}

function set_file(printer, id, action) { // {{{
	var element = get_element(printer, id);
	if (element.files.length < 1) {
		alert('please select a file');
		return;
	}
	var post = new XMLHttpRequest();
	var fd = new FormData();
	fd.append('port', printer.port);
	fd.append('action', action);
	fd.append('file', element.files[0]);
	post.open('POST', String(document.location), true);
	post.AddEvent('readystatechange', function() {
		if (this.readyState != this.DONE)
			return;
		if (!this.responseText)
			return;
		alert('Errors: ' + this.responseText);
	});
	post.send(fd);
} // }}}
// }}}

// Canvas functions. {{{
function fill(s) { // {{{
	s = s.toFixed(0);
	while (s.length < 2)
		s = '0' + s;
	return s;
}
// }}}

function display_time(t) { // {{{
	if (isNaN(t))
		return '-';
	var s = t;
	var m = Math.floor(s / 60);
	var h = Math.floor(m / 60);
	var d = Math.floor(h / 24);
	s -= m * 60;
	m -= h * 60;
	h -= d * 24;
	if (d != 0)
		return d.toFixed(0) + 'd ' + fill(h) + ':' + fill(m) + ':' + fill(s);
	if (h != 0)
		return h.toFixed(0) + ':' + fill(m) + ':' + fill(s);
	if (m != 0)
		return m.toFixed(0) + ':' + fill(s);
	return s.toFixed(0) + 's';
}
// }}}

function update_canvas_and_spans(p, space) { // {{{
	if (space === undefined) {
		space = 0;
	}
	if (space < 2) {	// Ignore follower positions.
		p.call('get_axis_pos', [space], {}, function(x) {
			//dbg('update ' + space + ',' + axis + ':' + x);
			for (var i = 0; i < x.length; ++i) {
				p.spaces[space].axis[i].current = x[i];
				update_float(p, [['axis', [space, i]], 'current']);
			}
			update_canvas_and_spans(p, space + 1);
		});
		return;
	}
	p.call('get_print_state', [], {}, function(state) {
		var e = get_element(p, [null, 'printstate']).ClearAll();
		if (isNaN(state[1]))
			e.AddText('State: ' + state[0]);
		else
			e.AddText('State: ' + state[0] + ' - Time: ' + display_time(state[1]) + '/' + display_time(state[2]) + ' - Remaining: ' + display_time(state[2] - state[1]));
		update_float(p, [null, 'targetangle']);
		redraw_canvas(p);
	});
}
// }}}

function redraw_canvas(printer) { // {{{
	if (printer.spaces.length < 1 || printer.spaces[0].axis.length < 1)
		return;
	var canvas = document.getElementById(make_id(printer, [null, 'xymap']));
	if (printer.spaces[0].axis.length >= 2) { // Draw XY {{{
		var c = canvas.getContext('2d');
		var box = document.getElementById(make_id(printer, [null, 'map']));
		var extra_height = box.clientHeight - canvas.clientHeight;
		var printerwidth;
		var printerheight;
		var outline, center;
		switch (printer.spaces[0].type) {
		case TYPE_CARTESIAN:
			var xaxis = printer.spaces[0].axis[0];
			var yaxis = printer.spaces[0].axis[1];
			printerwidth = xaxis.max - xaxis.min + .010;
			printerheight = yaxis.max - yaxis.min + .010;
			center = [(xaxis.min + xaxis.max) / 2, (yaxis.min + yaxis.max) / 2];
			outline = function(printer, c) {
				// Rectangle is always drawn; nothing else to do here.
			};
			break;
		case TYPE_DELTA:
			var radius = [];
			var length = [];
			for (var a = 0; a < 3; ++a) {
				radius.push(printer.spaces[0].motor[a].delta_radius);
				length.push(printer.spaces[0].motor[a].delta_rodlength);
			}
			//var origin = [[radius[0], 0], [radius[1] * -.5, radius[1] * .8660254037844387], [radius[2] * -.5, radius[2] * -.8660254037844387]];
			//var dx = [0, -.8660254037844387, .8660254037844387];
			//var dy = [1, -.5, -.5];
			var origin = [[radius[0] * -.8660254037844387, radius[0] * -.5], [radius[1] * .8660254037844387, radius[1] * -.5], [0, radius[2]]];
			center = [0, 0];
			var dx = [.5, .5, -1];
			var dy = [-.8660254037844387, .8660254037844387, 0];
			var intersects = [];
			var intersect = function(x0, y0, r, x1, y1, dx1, dy1, positive) {
				// Find intersection of circle(x-x0)^2+(y-y0)^2==r^2 and line l=(x1,y1)+t(dx1,dy1); use positive or negative solution for t.
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
			outline = function(printer, c) {
				c.save();
				c.rotate(printer.spaces[0].delta_angle);
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
					var name = printer.spaces[0].motor[a].name;
					var w = c.measureText(name).width;
					c.beginPath();
					c.save();
					c.translate(origin[a][0] + dy[a] * 10 - w / 2, origin[a][1] - dx[a] * 10);
					c.rotate(-printer.spaces[0].delta_angle);
					c.scale(1, -1);
					c.fillText(name, 0, 0);
					c.restore();
				}
				c.restore();
			};
			var extra = c.measureText(printer.spaces[0].motor[0].name).width + .02;
			printerwidth = 2 * (maxx + extra);
			printerheight = 2 * (maxy + extra);
			break;
		case TYPE_POLAR:
			printerwidth = 2 * printer.spaces[0].polar_max_r + 2;
			printerheight = 2 * printer.spaces[0].polar_max_r + 2;
			center = [0, 0];
			outline = function(printer, c) {
				c.beginPath();
				c.arc(0, 0, printer.spaces[0].polar_max_r, 0, 2 * Math.PI, false);
				c.stroke();
			};
			break;
		}
		//var factor = Math.sqrt(printerwidth * printerwidth + printerheight * printerheight);
		var factor = Math.max(printerwidth, printerheight);
		canvas.style.height = canvas.clientWidth + 'px';
		canvas.width = canvas.clientWidth;
		canvas.height = canvas.clientWidth;

		var b = printer.bbox;
		var true_pos = [printer.spaces[0].axis[0].current, printer.spaces[0].axis[1].current];

		c.save();
		// Clear canvas.
		c.clearRect(0, 0, canvas.width, canvas.width);

		c.translate(canvas.width / 2, canvas.width / 2);
		c.scale(canvas.width / factor, -canvas.width / factor);
		c.lineWidth = 1.5 * factor / canvas.width;
		c.translate(-center[0], -center[1]);

		get_pointer_pos_xy = function(printer, e) {
			var rect = canvas.getBoundingClientRect();
			var x = e.clientX - rect.left - canvas.width / 2;
			var y = e.clientY - rect.top - canvas.width / 2;
			x /= canvas.width / factor;
			y /= -canvas.width / factor;
			return [x, y];
		};

		// Draw outline.
		c.strokeStyle = '#888';
		c.fillStyle = '#888';
		outline(printer, c);
		// Draw limits.
		c.beginPath();
		var a = printer.spaces[0].axis;
		c.moveTo(a[0].min, a[1].min);
		c.lineTo(a[0].max, a[1].min);
		c.lineTo(a[0].max, a[1].max);
		c.lineTo(a[0].min, a[1].max);
		c.closePath();
		c.stroke();
		// Draw center.
		c.beginPath();
		c.moveTo(1, 0);
		c.arc(0, 0, 1, 0, 2 * Math.PI);
		c.fillStyle = '#888';
		c.fill();

		// Draw current location.
		c.beginPath();
		c.fillStyle = '#44f';
		c.moveTo(true_pos[0] + 3, true_pos[1]);
		c.arc(true_pos[0], true_pos[1], 3, 0, 2 * Math.PI);
		c.fill();

		c.save();
		c.translate(printer.targetx, printer.targety);
		c.rotate(printer.targetangle);

		c.beginPath();
		if (b[0] != b[1] && b[2] != b[3]) {
			// Draw print bounding box.
			c.rect(b[0], b[2], b[1] - b[0], b[3] - b[2]);

			// Draw tick marks.
			c.moveTo((b[1] + b[0]) / 2, b[2]);
			c.lineTo((b[1] + b[0]) / 2, b[2] + 5);

			c.moveTo((b[1] + b[0]) / 2, b[3]);
			c.lineTo((b[1] + b[0]) / 2, b[3] - 5);

			c.moveTo(b[0], (b[3] + b[2]) / 2);
			c.lineTo(b[0] + 5, (b[3] + b[2]) / 2);

			c.moveTo(b[1], (b[3] + b[2]) / 2);
			c.lineTo(b[1] - 5, (b[3] + b[2]) / 2);

			// Draw central cross.
			c.moveTo((b[1] + b[0]) / 2 - 5, (b[3] + b[2]) / 2);
			c.lineTo((b[1] + b[0]) / 2 + 5, (b[3] + b[2]) / 2);
			c.moveTo((b[1] + b[0]) / 2, (b[3] + b[2]) / 2 + 5);
			c.lineTo((b[1] + b[0]) / 2, (b[3] + b[2]) / 2 - 5);
		}

		// Draw zero.
		c.moveTo(3, 0);
		c.arc(0, 0, 3, 0, 2 * Math.PI);

		// Update it on screen.
		c.strokeStyle = '#000';
		c.stroke();

		c.restore();
		c.restore();
	} // }}}

	if (printer.spaces[0].axis.length != 2) { // Draw Z {{{
		var zaxis, zoffset;
		if (printer.spaces[0].axis.length >= 3) {
			zaxis = printer.spaces[0].axis[2];
			zoffset = printer.zoffset;
		}
		else {
			zaxis = printer.spaces[0].axis[0];
			zoffset = 0;
		}
		var zcanvas = document.getElementById(make_id(printer, [null, 'zmap']));
		var zc = zcanvas.getContext('2d');
		zcanvas.style.height = canvas.clientWidth + 'px';
		var zratio = .15 / .85;
		zcanvas.width = canvas.clientWidth * zratio;
		zcanvas.height = canvas.clientWidth;
		// Z graph.
		zc.clearRect(0, 0, zcanvas.width, zcanvas.height);
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
		};

		// Draw current position.
		zc.beginPath();
		zc.moveTo(0, zaxis.current + zoffset);
		zc.lineTo(-d, zaxis.current - d + zoffset);
		zc.lineTo(-d, zaxis.current + d + zoffset);
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
	} // }}}
}
// }}}

function start_move(printer) { // {{{
	// Update bbox.
	var q = get_element(printer, [null, 'queue']);
	printer.bbox = [null, null, null, null];
	for (var e = 0; e < q.options.length; ++e) {
		if (!q.options[e].selected)
			continue;
		var name = q.options[e].value;
		var item;
		for (item = 0; item < printer.queue.length; ++item)
			if (printer.queue[item][0] == name)
				break;
		if (item >= printer.queue.length)
			continue;
		if (printer.bbox[0] == null || printer.queue[item][1][0] < printer.bbox[0])
			printer.bbox[0] = printer.queue[item][1][0];
		if (printer.bbox[1] == null || printer.queue[item][1][1] > printer.bbox[1])
			printer.bbox[1] = printer.queue[item][1][1];
		if (printer.bbox[2] == null || printer.queue[item][1][2] < printer.bbox[2])
			printer.bbox[2] = printer.queue[item][1][2];
		if (printer.bbox[3] == null || printer.queue[item][1][3] > printer.bbox[3])
			printer.bbox[3] = printer.queue[item][1][3];
	}
	update_canvas_and_spans(printer);
} // }}}

function reset_position(printer) { // {{{
	printer.targetangle = 0;
	update_canvas_and_spans(printer);
} // }}}

var drag = [[NaN, NaN], [NaN, NaN], [NaN, NaN], 0, false];

function xydown(printer, e) { // {{{
	var pos = get_pointer_pos_xy(printer, e);
	drag[0][0] = pos[0];
	drag[1][0] = pos[1];
	drag[4] = false;
	if (e.button == 0) {
		printer.call('get_axis_pos', [0, 0], {}, function(x) {
			printer.call('get_axis_pos', [0, 1], {}, function(y) {
				drag[0][1] = x;
				drag[1][1] = y;
			});
		});
	}
	else {
		drag[0][1] = printer.targetx;
		drag[1][1] = printer.targety;
	}
	return false;
}
// }}}

function xyup(printer, e) { // {{{
	var pos = get_pointer_pos_xy(printer, e);
	if (drag[4])
		return false;
	if (!(e.buttons & 5))
		return false;
	if (e.buttons & 1) {
		printer.call('line_cb', [[[pos[0], pos[1]]]], {}, function() { update_canvas_and_spans(printer); });
	}
	else if (e.buttons & 4) {
		drag[3] += 1;
		printer.call('set_globals', [], {'targetx': pos[0], 'targety': pos[1]}, function() { drag[3] -= 1; });
	}
	return false;
}
// }}}

function xymove(printer, e) { // {{{
	drag[4] = true;
	if (drag[3])
		return false;
	if (!(e.buttons & 5)) {
		drag[0][1] = NaN;
		drag[1][1] = NaN;
		return false;
	}
	var pos = get_pointer_pos_xy(printer, e);
	var dx = pos[0] - drag[0][0];
	var dy = pos[1] - drag[1][0];
	if (isNaN(drag[0][1]) || isNaN(drag[1][1]))
		return false;
	if (e.buttons & 1) {
		drag[3] += 1;
		printer.call('line_cb', [[[drag[0][1] + dx, drag[1][1] + dy]]], {}, function() { drag[3] -= 1; update_canvas_and_spans(printer); });
	}
	else if (e.buttons & 4) {
		drag[3] += 1;
		printer.call('line_cb', [[[dx, dy]]], {relative: true}, function() {
			printer.call('move_target', [dx, dy], {}, function() {
				drag[0][0] += dx;
				drag[1][0] += dy;
				drag[3] -= 1;
			});
		});
	}
	return false;
}
// }}}

function zdown(printer, e) { // {{{
	drag[4] = false;
	drag[2][0] = get_pointer_pos_z(printer, e);
	if (e.button == 0)
		printer.call('get_axis_pos', [0, 2], {}, function(z) { drag[2][1] = z; });
	return false;
}
// }}}

function zup(printer, e) { // {{{
	var pos = get_pointer_pos_z(printer, e);
	if (drag[4])
		return false;
	if (!(e.buttons & 5))
		return false;
	if (e.buttons & 1) {
		printer.call('line_cb', [[{2: pos}]], {}, function() { update_canvas_and_spans(printer); });
	}
	return false;
}
// }}}

function zmove(printer, e) { // {{{
	drag[4] = true;
	if (drag[3])
		return false;
	if (!(e.buttons & 5)) {
		drag[2][1] = NaN;
		return false;
	}
	var dz = get_pointer_pos_z(printer, e) - drag[2][0];
	if (isNaN(drag[2][1]))
		return false;
	if (e.buttons & 1) {
		drag[3] = true;
		printer.call('line_cb', [[{2: drag[2][1] + dz}]], {}, function() { drag[3] = false; update_canvas_and_spans(printer); });
	}
	return false;
}
// }}}

function keypress(event) { // {{{
	var p = selected_printer;
	if (!p)
		return;
	if (event.keyCode >= 37 && event.keyCode <= 40 && (event.ctrlKey || event.altKey)) {
		var amount = 10;	// TODO: make this a setting.
		if (event.shiftKey)
			amount /= 10;
		var target = [[-amount, 0], [0, amount], [amount, 0], [0, -amount]][event.keyCode - 37];
		p.call('line_cb', [[target]], {relative: true}, function() {
			if (event.altKey) {
				p.call('set_globals', [], {'targetx': p.targetx + target[0], 'targety': p.targety + target[1]});
			}
			else
				update_canvas_and_spans(p);
		});
		event.preventDefault();
	}
	else if (event.charCode >= 48 && event.charCode <= 57 && event.ctrlKey) {
		var minx = p.bbox[0];
		var maxx = p.bbox[1];
		var miny = p.bbox[2];
		var maxy = p.bbox[3];
		var pos = [[0, 0],
			[minx, miny], [(minx + maxx) / 2, miny], [maxx, miny],
			[minx, (miny + maxy) / 2], [(minx + maxx) / 2, (miny + maxy) / 2], [maxx, (miny + maxy) / 2],
			[minx, maxy], [(minx + maxx) / 2, maxy], [maxx, maxy]][event.charCode - 48];
		var posx = Math.cos(p.targetangle) * pos[0] - Math.sin(p.targetangle) * pos[1];
		var posy = Math.cos(p.targetangle) * pos[1] + Math.sin(p.targetangle) * pos[0];
		posx += p.targetx;
		posy += p.targety;
		p.call('line_cb', [[[posx, posy]]], {}, function() { update_canvas_and_spans(p); });
		event.preventDefault();
	}
} // }}}

function update_angle(printer, value) { // {{{
	if (printer.nextangle !== null) {
		printer.nextangle = value;
		return;
	}
	printer.call('get_axis_pos', [0], {}, function(pos) {
		pos[0] -= printer.targetx;
		pos[1] -= printer.targety;
		var posx = Math.cos(printer.targetangle) * pos[0] + Math.sin(printer.targetangle) * pos[1];
		var posy = Math.cos(printer.targetangle) * pos[1] - Math.sin(printer.targetangle) * pos[0];
		pos[0] = Math.cos(value) * posx - Math.sin(value) * posy + printer.targetx;
		pos[1] = Math.cos(value) * posy + Math.sin(value) * posx + printer.targety;
		printer.call('line_cb', [[[pos[0], pos[1]]]], {}, function() {
			printer.targetangle = value;
			if (printer.nextangle !== null) {
				var a = printer.nextangle;
				printer.nextangle = null;
				return update_angle(printer, a);
			}
			else
				update_canvas_and_spans(printer);
		});
	});
}
// }}}
// }}}

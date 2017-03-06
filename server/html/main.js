/* main.js - clientside functions for Franklin {{{
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
 * }}} */

// Variables.  {{{
var get_pointer_pos_xy = function() { return [NaN, NaN]; };
var get_pointer_pos_z = function() { return [NaN, NaN]; };
var upload_options = {};
var labels_element, machines_element;
var selected_machine;
var type2plural = {space: 'spaces', temp: 'temps', gpio: 'gpios', axis: 'axes', motor: 'motors'};
var space_types = ['Cartesian', 'Delta', 'Polar', 'Extruder', 'Follower'];
var new_tab;
// }}}

// General supporting functions. {{{
AddEvent('load', function() { // {{{
	labels_element = document.getElementById('labels');
	machines_element = document.getElementById('machines');
	new_tab = document.getElementById('new_label');
	selected_machine = null;
	setup();
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
		if (reading_temps || !selected_machine || machines[selected_machine].ui.disabling)
			return;
		reading_temps = true;
		var read = function(ui, num) {
			if (!selected_machine || !machines[selected_machine] || machines[selected_machine].ui.disabling)
				return;
			if (ui === null)
				ui = machines[selected_machine].ui;
			if (ui !== machines[selected_machine].ui)
				return;
			if (num < ui.machine.temps.length) {
				ui.machine.call('readtemp', [num], {}, function(t) {
					if (num < ui.machine.temps.length) {
						ui.machine.temps[num].temp = t;
						var e = get_element(ui, [['temp', num], 'temp']);
						// Only update if machine still exists.
						if (e)
							e.ClearAll().AddText(isNaN(t) ? t : t.toFixed(1));
					}
					read(ui, num + 1);
				});
				return;
			}
			reading_temps = false;
			// Update temperature graph.
			var canvas = get_element(ui, [null, 'tempgraph']);
			if (!canvas) {
				reading_temps = false;
				return;
			}
			var c = canvas.getContext('2d');
			canvas.height = canvas.clientHeight;
			canvas.width = canvas.clientWidth;
			var scale = canvas.height / (ui.machine.temp_scale_max - ui.machine.temp_scale_min);
			c.clearRect(0, 0, canvas.width, canvas.height);
			c.save(); // Transform coordinates to proper units. {{{
			c.translate(0, canvas.height);
			c.scale(scale, -scale);
			c.translate(0, -ui.machine.temp_scale_min);
			// Draw grid.
			c.beginPath();
			var step = 15;
			for (var t = step; t < 120; t += step) {
				var x = (t / (2 * 60) * canvas.width) / scale;
				c.moveTo(x, ui.machine.temp_scale_min);
				c.lineTo(x, ui.machine.temp_scale_max);
			}
			step = Math.pow(10, Math.floor(Math.log(ui.machine.temp_scale_max - ui.machine.temp_scale_min) / Math.log(10)));
			for (var y = step * Math.floor(ui.machine.temp_scale_min / step); y < ui.machine.temp_scale_max; y += step) {
				c.moveTo(0, y);
				c.lineTo(canvas.width / scale, y);
			}
			c.save(); // Set linewidth. {{{
			c.lineWidth = 1 / scale;
			var makedash = function(array) {
				// If browser doesn't support this, use solid lines.
				if (c.setLineDash !== undefined) {
					for (var i = 0; i < array.length; ++i)
						array[i] /= scale;
					c.setLineDash(array);
				}
			};
			makedash([1, 4]);
			c.strokeStyle = '#444';
			c.stroke();
			c.restore(); // }}}
			// Draw grid scale.
			c.save(); // Scale units for the grid. {{{
			c.scale(1 / scale, -1 / scale);
			for (var y = step * Math.floor(ui.machine.temp_scale_min / step); y < ui.machine.temp_scale_max; y += step) {
				c.moveTo(0, y);
				var text = y.toFixed(0);
				c.fillText(text, (step / 50) * scale, -(y + step / 50) * scale);
			}
			c.restore(); // }}}
			// Draw data.
			var time = new Date();
			ui.temphistory.push(time);
			for (var t = 0; t < ui.machine.temps.length; ++t)
				ui.machine.temps[t].history.push([ui.machine.temps[t].temp, ui.machine.temps[t].value]);
			var cutoff = time - 2 * 60 * 1000;
			while (ui.temphistory.length > 1 && ui.temphistory[0] < cutoff)
				ui.temphistory.shift();
			for (var t = 0; t < ui.machine.temps.length; ++t) {
				while (ui.machine.temps[t].history.length > ui.temphistory.length)
					ui.machine.temps[t].history.shift();
			}
			var x = function(t) {
				return ((t - cutoff) / (2 * 60 * 1000) * canvas.width) / scale;
			};
			var y = function(d) {
				if (isNaN(d))
					return ui.machine.temp_scale_min - 2 / scale;
				if (!isFinite(d))
					return ui.machine.temp_scale_max + 2 / scale;
				return d;
			};
			for (var t = 0; t < ui.machine.temps.length; ++t) {
				// Draw measured data.
				var value;
				var data = ui.machine.temps[t].history;
				c.beginPath();
				value = data[0][0];
				if (isNaN(ui.machine.temps[t].beta)) {
					value *= ui.machine.temps[t].Rc / 100000;
					value += ui.machine.temps[t].Tc;
				}
				c.moveTo(x(ui.temphistory[0]), y(value));
				for (var i = 1; i < data.length; ++i) {
					value = data[i][0];
					if (isNaN(ui.machine.temps[t].beta)) {
						value *= ui.machine.temps[t].Rc / 100000;
						value += ui.machine.temps[t].Tc;
					}
					c.lineTo(x(ui.temphistory[i]), y(value));
				}
				c.strokeStyle = ['#f00', '#00f', '#0f0', '#ff0', '#000'][t < 5 ? t : 4];
				c.lineWidth = 1 / scale;
				c.stroke();
				// Draw temp targets.
				c.moveTo(x(ui.temphistory[0]), y(data[0][1]));
				for (var i = 1; i < data.length; ++i)
					c.lineTo(x(ui.temphistory[i]), y(data[i][1]));
				c.save(); // Use dached lines. {{{
				makedash([4, 2]);
				c.stroke();
				c.restore(); // }}}
				// Draw values of temp targets.
				var old = null;
				c.fillStyle = c.strokeStyle;
				c.save(); // Scale the target data. {{{
				c.scale(1 / scale, -1 / scale);
				for (var i = 1; i < data.length; ++i) {
					if (data[i][1] != old && (!isNaN(data[i][1]) || !isNaN(old))) {
						old = data[i][1];
						var pos;
						if (isNaN(data[i][1]) || data[i][1] < ui.machine.temp_scale_min)
							pos = ui.machine.temp_scale_min;
						else if (data[i][1] / scale >= ui.machine.temp_scale_max / scale - 12)
							pos = (ui.machine.temp_scale_max / scale - 5) * scale;
						else
							pos = data[i][1];
						var text = data[i][1].toFixed(0);
						c.fillText(text, (x(ui.temphistory[i])) * scale, -(y(pos) + step / 50) * scale);
					}
				}
				c.restore(); // }}}
			}
			c.restore(); // }}}
			// Update everything else.
			update_canvas_and_spans(ui);
		};
		read(null, 0);
	}, 400);
}); // }}}

function make_id(ui, id, extra) { // {{{
	// [null, 'num_temps']
	// [['space', 1], 'num_axes']
	// [['axis', [0, 1]], 'offset']
	// [['motor', [0, 1]], 'delta_radius']
	//console.log(ui);
	//console.error(id);
	if (!(ui instanceof Element))
		console.error(ui);
	var ret = ui ? ui.machine.uuid.replace(/[^a-zA-Z0-9_]/g, '') : '';
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

function set_value(ui, id, value, reply, arg) { // {{{
	if (id.length == 2 || id[2] === undefined) {
		var obj = {};
		obj[id[1]] = value;
		if (id[0] === null) {
			// [null, 'num_temps']
			ui.machine.call('set_globals', [], obj, reply);
		}
		else {
			if (id[0][1] === null) {
				// [['space', null], 'num_axes']
				for (var n = 0; n < ui.machine['num_' + type2plural[id[0][0]]]; ++n)
					ui.machine.call('set_' + id[0][0], [n], obj, reply);
			}
			else if (typeof id[0][1] != 'number' && id[0][1][1] == null) {
				// [['axis', [0, null]], 'offset']
				// [['motor', [0, null]], 'delta_radius']
				if ((id[0][0] != 'motor' || id[1].substr(0, 6) != 'delta_') && (id[0][0] != 'axis' || id[1].substr(0, 9) != 'extruder_') && (id[0][0] != 'motor' || id[1].substr(0, 9) != 'follower_')) {
					for (var n = 0; n < ui.machine.spaces[id[0][1][0]]['num_' + type2plural[id[0][0]]]; ++n)
						ui.machine.call('set_' + id[0][0], [[id[0][1][0], n]], obj, reply);
				}
				else if (id[0][0] == 'motor') {
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(6)] = value;
					var o = {};
					for (var n = 0; n < ui.machine.spaces[id[0][1][0]].num_motors; ++n)
						o[n] = obj;
					ui.machine.call('set_space', [id[0][1][0]], {delta: o}, reply);
				}
				else if (id[1][0] == 'e') {
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(9)] = value;
					var o = {};
					for (var n = 0; n < ui.machine.spaces[id[0][1][0]].num_axes; ++n)
						o[n] = obj;
					ui.machine.call('set_space', [id[0][1][0]], {extruder: o}, reply);
				}
				else {
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(9)] = value;
					var o = {};
					for (var n = 0; n < ui.machine.spaces[id[0][1][0]].num_axes; ++n)
						o[n] = obj;
					ui.machine.call('set_motor', [id[0][1][0]], {follower: o}, reply);
				}
			}
			else if ((id[0][0] != 'motor' || id[1].substr(0, 6) != 'delta_') && (id[0][0] != 'axis' || id[1].substr(0, 9) != 'extruder_') && (id[0][0] != 'motor' || id[1].substr(0, 9) != 'follower_')) {
				// [['space', 1], 'num_axes']
				// [['axis', [0, 1]], 'offset']
				ui.machine.call('set_' + id[0][0], [id[0][1]], obj, reply);
			}
			else if (id[0][0] == 'motor') {
				if (id[1][0] != 'f') {
					// [['motor', [0, 1]], 'delta_radius']
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(6)] = value;
					var o = {};
					o[id[0][1][1]] = obj;
					ui.machine.call('set_space', [id[0][1][0]], {delta: o}, reply);
				}
				else {
					// [['motor', [0, 1]], 'delta_radius']
					obj = {};	// obj was wrong in this case.
					obj[id[1].substr(9)] = value;
					var o = {};
					o[id[0][1][1]] = obj;
					ui.machine.call('set_space', [id[0][1][0]], {follower: o}, reply);
				}
			}
			else {
				// [['motor', [0, 1]], 'delta_radius']
				obj = {};	// obj was wrong in this case.
				obj[id[1].substr(9)] = value;
				var o = {};
				o[id[0][1][1]] = obj;
				ui.machine.call('set_space', [id[0][1][0]], {extruder: o}, reply);
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
			if (ui)
				ui.machine.call(id[2], val, {}, reply);
			else
				rpc.call(id[2], val, {}, reply);
		}
		else if (id[0][1] === null) {
			// [['space', null], 'num_axes']
			for (var n = 0; n < ui.machine['num_' + type2plural[id[0][0]]]; ++n)
				ui.machine.call(id[2], [n, value], {}, reply);
		}
		else
			ui.machine.call(id[2], [id[0][1], value], {}, reply);
	}
} // }}}

function get_value(ui, id) { // {{{
	if (id[0] === null)
		return ui.machine[id[1]];
	if (id[0][0] == 'axis') {
		return ui.machine.spaces[id[0][1][0]].axis[id[0][1][1]][id[1]];
	}
	else if (id[0][0] == 'motor') {
		return ui.machine.spaces[id[0][1][0]].motor[id[0][1][1]][id[1]];
	}
	else {
		return ui.machine[type2plural[id[0][0]]][id[0][1]][id[1]];
	}
} // }}}

function get_element(ui, id, extra) { // {{{
	return document.getElementById(make_id(ui, id, extra));
} // }}}

function select_machine(ui) { // {{{
	selected_machine = ui ? ui.machine.uuid : null;
	for (var p in machines) {
		if (typeof machines[p] != 'object')
			continue;
		if (p == selected_machine) {
			machines[p].label.AddClass('active');
			machines[p].ui.RemoveClass('hidden');
		}
		else {
			machines[p].label.RemoveClass('active');
			machines[p].ui.AddClass('hidden');
		}
	}
	if (selected_machine !== null && selected_machine !== undefined) {
		new_tab.RemoveClass('active');
		update_state(ui, get_value(ui, [null, 'status']));
	}
	else {
		new_tab.AddClass('active');
		update_state(ui, null);
	}
} // }}}

function upload_buttons(port, ul, buttons) { // {{{
	for (var b = 0; b < buttons.length; ++b) {
		var li = ul.AddElement('li');
		// Add the upload button.
		var button = li.AddElement('button', 'upload');
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
		if (v == '')
			return;
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
		// Update a series of elements.
		for (var n = 0; n < element.ui.machine['num_' + type2plural[element.obj[0][0]]]; ++n) {
			var obj = [[element.obj[0][0], n], element.obj[1], element.obj[2]];
			var value;
			if (set)
				value = amount;
			else if (element.obj.length == 2)
				value = get_value(element.ui, obj) / element.factor + amount;
			else {
				console.info('skip', element.obj);
				continue;
			}
			set_value(element.ui, obj, element.factor * value);
		}
		return;
	}
	if (element.obj[0] !== null && typeof element.obj[0][1] != 'number' && element.obj[0][1][1] === null) {
		// Update a series of axes or motors.
		for (var n = 0; n < element.ui.machine.spaces[element.obj[0][1][0]]['num_' + type2plural[element.obj[0][0]]]; ++n) {
			var obj = [[element.obj[0][0], [element.obj[0][1][0], n]], element.obj[1]];
			var value;
			if (set)
				value = amount;
			else if (element.obj.length == 2)
				value = get_value(element.ui, obj) / element.factor + amount;
			else
				continue;
			set_value(element.ui, obj, element.factor * value);
		}
		return;
	}
	var value;
	if (set)
		value = amount;
	else if (element.obj.length == 2)
		value = get_value(element.ui, element.obj) / element.factor + amount;
	else
		return;
	if (element.set !== undefined)
		element.set(element.factor * value);
	else
		set_value(element.ui, element.obj, element.factor * value);
}
// }}}

function add_name(ui, type, index1, index2) { // {{{
	while (ui.names[type].length <= index1) {
		ui.names[type].push([]);
		ui.name_values[type].push([]);
	}
	while (ui.names[type][index1].length <= index2) {
		ui.names[type][index1].push([]);
		ui.name_values[type][index1].push('');
	}
	var ret = Create('span', 'name').AddText(ui.name_values[type][index1][index2]);
	ui.names[type][index1][index2].push(ret);
	return ret;
} // }}}

function set_name(ui, type, index1, index2, name) { // {{{
	ui.name_values[type][index1][index2] = name;
	for (var i = 0; i < ui.names[type][index1][index2].length; ++i)
		ui.names[type][index1][index2][i].ClearAll().AddText(name);
} // }}}

function toggle_setup() { // {{{
	var e = document.getElementById('setupbox');
	var c = document.getElementById('container');
	if (e.checked)
		c.RemoveClass('nosetup');
	else
		c.AddClass('nosetup');
} // }}}

function update_firmwares(ports, firmwares) { // {{{
	var port = ports.options[ports.selectedIndex].value;
	firmwares.ClearAll();
	for (var o = 0; o < all_firmwares[port].length; ++o)
		firmwares.AddElement('option').AddText(all_firmwares[port][o][1]).value = all_firmwares[port][o][0];
} // }}}

function detect(ports) { // {{{
	var port = ports.options[ports.selectedIndex].value;
	rpc.call('detect', [port], {}, null);
} // }}}

function upload(ports, firmwares) { // {{{
	var port = ports.options[ports.selectedIndex].value;
	var firmware = firmwares.options[firmwares.selectedIndex].value;
	rpc.call('upload', [port, firmware], {}, function(ret) { alert('upload done: ' + ret);});
} // }}}
// }}}

// Queue functions.  {{{
function queue_deselect(ui) { // {{{
	var e = get_element(ui, [null, 'queue']);
	for (var i = 1; i < e.options.length; ++i) {
		e.options[i].selected = false;
	}
}
// }}}

function queue_up(ui) { // {{{
	var e = get_element(ui, [null, 'queue']);
	for (var i = 1; i < e.options.length; ++i) {
		if (e.options[i].selected) {
			var cur = e.options[i];
			var prev = e.options[i - 1];
			e.insertBefore(e.removeChild(cur), prev);
		}
	}
}
// }}}

function queue_down(ui) { // {{{
	var e = get_element(ui, [null, 'queue']);
	for (var i = e.options.length - 2; i >= 0; --i) {
		if (e.options[i].selected) {
			var cur = e.options[i];
			var next = e.options[i + 1];
			e.insertBefore(e.removeChild(next), cur);
		}
	}
}
// }}}

function queue_del(ui) { // {{{
	var e = get_element(ui, [null, 'queue']);
	var rm = [];
	for (var i = 0; i < e.options.length; ++i) {
		if (e.options[i].selected)
			rm.push(e.options[i]);
	}
	for (var i = 0; i < rm.length; ++i)
		rpc.call('queue_remove', [rm[i].value], {});
}
// }}}

function get_queue(ui) { // {{{
	var toprint = [];
	var e = get_element(ui, [null, 'queue']);
	for (var i = 0; i < e.options.length; ++i) {
		if (e.options[i].selected)
			toprint.push(e.options[i].value);
	}
	return toprint;
}
// }}}

function queue_run(ui) { // {{{
	var action;
	if (get_element(ui, [null, 'probebox']).checked) {
		action = 'queue_probe';
		kwargs = {};	// Probe jobs always pause after probing.
	}
	else {
		action = 'queue_run';
		kwargs = {paused: get_element(ui, [null, 'start_paused']).checked};
	}
	ui.machine.call(action, [get_queue(ui)], kwargs);
}
// }}}

function audio_play(ui) { // {{{
	var e = get_element(ui, [null, 'audio']);
	var o = e.options[e.selectedIndex];
	if (o === undefined)
		return;
	ui.call('audio_play', [o.value]);
}

// }}}
function audio_del(ui) { // {{{
	var e = get_element(ui, [null, 'audio']);
	var o = e.options[e.selectedIndex];
	if (o === undefined)
		return;
	ui.call('audio_del', [o.value]);
}
// }}}
// }}}

// Non-update events. {{{
function connect(ui, connected) { // {{{
	// TODO
} // }}}

function autodetect() { // {{{
} // }}}

function new_port(ui, port) { // {{{
	for (var p in machines) {
		var ports = get_element(machines[p].ui, [null, 'ports']);
		ports.ClearAll();
		var new_port = ports.AddElement('option').AddText(port);
		new_port.value = port;
	}
} // }}}

function disable_buttons(state) { // {{{
	var buttons = document.getElementsByClassName('upload');
	for (var b = 0; b < buttons.length; ++b) {
		buttons[b].disabled = state;
	}
} // }}}

function port_state(ui, port, state) { // {{{
	// 0: No ui.
	// 1: Detecting.
	// 2: Running.
	// 3: Flashing.
	// TODO
	return;
	/*
	for (var t = 0; t < 2; ++t) {	// Set class for Label and NoMachine.
		switch (state) {
		case 0:
			ports[port][t].AddClass('nodetect');
			ports[port][t].AddClass('noflash');
			disable_buttons(false);
			break;
		case 1:
			ports[port][t].RemoveClass('nodetect');
			ports[port][t].AddClass('noflash');
			disable_buttons(false);
			break;
		case 2:
			ports[port][t].AddClass('nodetect');
			ports[port][t].AddClass('noflash');
			disable_buttons(false);
			break;
		case 3:
			ports[port][t].AddClass('nodetect');
			ports[port][t].RemoveClass('noflash');
			disable_buttons(true);
			break;
		default:
			alert('Invalid ui state ' + state);
		}
	}*/
} // }}}

function new_machine(uuid) { // {{{
	machines[uuid].label = Label(machines[uuid]);
	machines[uuid].ui = Machine(machines[uuid]);
	var p = machines[uuid].ui;
	labels_element.insertBefore(machines[uuid].label, new_tab);
	machines_element.Add(p);
	machines[uuid].label.RemoveClass('connected');
	machines[uuid].ui.RemoveClass('connected');
	machines[uuid].label.AddClass('notconnected');
	machines[uuid].ui.AddClass('notconnected');
	globals_update(uuid);
	var ports_button = get_element(machines[uuid].ui, [null, 'ports']);
	ports_button.ClearAll();
	for (var port = 0; port < all_ports.length; ++port) {
		var new_port = ports_button.AddElement('option').AddText(all_ports[port]);
		new_port.value = all_ports[port];
		for (var o = 0; o < all_firmwares[new_port.value].length; ++o) {
			machines[uuid].ui.firmwares.AddElement('option').AddText(all_firmwares[new_port.value][o][1]).value = all_firmwares[new_port.value][o][0];
		}
	}
	if (selected_machine === null)
		select_machine(p);
} // }}}

function blocked(uuid, reason) { // {{{
	console.info(uuid);
	var e = get_element(machines[uuid].ui, [null, 'block1']);
	if (reason) {
		e.ClearAll();
		e.AddText(reason);
		e.RemoveClass('hidden');
	}
	else
		e.AddClass('hidden');
} // }}}

function message(uuid, msg) { // {{{
	var e = get_element(machines[uuid].ui, [null, 'message1']);
	e.ClearAll();
	e.AddText(msg);
} // }}}

function del_machine(uuid) { // {{{
	labels_element.removeChild(machines[uuid].label);
	machines_element.removeChild(machines[uuid].ui);
	delete machines[uuid];
} // }}}

function del_port(uuid, port) { // {{{
	for (var p in machines) {
		var ports = get_element(machines[p].ui, [null, 'ports']);
		for (var o = 0; o < ports.options.length; ++o) {
			if (ports.options[o].value == port)
				ports.removeChild(ports.options[o]);
		}
	}
} // }}}

function ask_confirmation(uuid, id, message) { // {{{
	update_canvas_and_spans(machines[uuid].ui);
	var e = get_element(machines[uuid].ui, [null, 'confirm']);
	e.ClearAll();
	if (id === null) {
		e.AddClass('hidden');
		return;
	}
	e.RemoveClass('hidden');
	// Cancel button.
	var button = e.AddElement('button', 'confirmabort').AddText('Abort').AddEvent('click', function() {
		var e = get_element(this.ui, [null, 'confirm']);
		e.ClearAll();
		e.AddClass('hidden');
		this.ui.machine.call('confirm', [this.confirm_id, false], {});
	});
	button.type = 'button';
	button.ui = machines[uuid].ui;
	button.confirm_id = id;
	// Message
	e.AddText(message);
	// Ok button.
	button = e.AddElement('button', 'confirmok').AddText('Ok').AddEvent('click', function() {
		var e = get_element(this.ui, [null, 'confirm']);
		e.ClearAll();
		e.AddClass('hidden');
		this.ui.machine.call('confirm', [this.confirm_id, true], {});
	});
	button.type = 'button';
	button.ui = machines[uuid].ui;
	button.confirm_id = id;
} // }}}

function queue(uuid) { // {{{
	var e = get_element(machines[uuid].ui, [null, 'queue']);
	var q = [];
	var must_deselect = machines[uuid].queue.length > e.options.length;
	var no_select = e.options.length == 0;
	for (var i = 0; i < machines[uuid].queue.length; ++i)
		q.push(machines[uuid].queue[i]);
	var rm = [];
	for (var item = 0; item < e.options.length; ++item) {
		var i;
		for (i = 0; i < q.length; ++i) {
			if (q[i][0] == e.options[item].value) {
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
		option.AddText(q[i][0] + ' - ' + display_time(q[i][1][6] + q[i][1][7] / machines[uuid].max_v));
		option.value = q[i][0];
		if (i == 0 || !no_select)
			option.selected = true;
	}
	start_move(machines[uuid].ui);
} // }}}

function audioqueue(uuid) { // {{{
	var e = get_element(machines[uuid].ui, [null, 'audio']);
	var q = [];
	for (var i = 0; i < machines[uuid].audioqueue.length; ++i)
		q.push(machines[uuid].audioqueue[i]);
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
	start_move(machines[uuid].ui);
} // }}}
// }}}

// Update events(from server). {{{
function globals_update(uuid) { // {{{
	var p = machines[uuid].ui;
	p.uuid.ClearAll().AddText(uuid);
	var container = get_element(p, [null, 'container']);
	if (!container)
		return;
	if (machines[uuid].connected) {
		machines[uuid].label.AddClass('isconnected');
		machines[uuid].ui.AddClass('isconnected');
		machines[uuid].label.RemoveClass('isnotconnected');
		machines[uuid].ui.RemoveClass('isnotconnected');
	} else {
		machines[uuid].label.RemoveClass('isconnected');
		machines[uuid].ui.RemoveClass('isconnected');
		machines[uuid].label.AddClass('isnotconnected');
		machines[uuid].ui.AddClass('isnotconnected');
	}
	get_element(p, [null, 'export']).href = encodeURIComponent(machines[uuid].name + '-' + machines[uuid].profile) + '.ini?machine=' + encodeURIComponent(uuid);
	machines[uuid].label.span.ClearAll().AddText(machines[uuid].name);
	update_str(p, [null, 'name']);
	update_float(p, [null, 'num_temps']);
	update_float(p, [null, 'num_gpios']);
	update_pin(p, [null, 'led_pin']);
	update_pin(p, [null, 'stop_pin']);
	update_pin(p, [null, 'probe_pin']);
	update_pin(p, [null, 'spiss_pin']);
	update_float(p, [null, 'probe_dist']);
	update_float(p, [null, 'probe_safe_dist']);
	update_float(p, [null, 'timeout']);
	update_float(p, [null, 'feedrate']);
	update_float(p, [null, 'max_deviation']);
	update_float(p, [null, 'max_v']);
	update_float(p, [null, 'targetx']);
	update_float(p, [null, 'targety']);
	update_float(p, [null, 'targetangle']);
	update_float(p, [null, 'zoffset']);
	update_checkbox(p, [null, 'store_adc']);
	update_checkbox(p, [null, 'park_after_print']);
	update_checkbox(p, [null, 'sleep_after_print']);
	update_checkbox(p, [null, 'cool_after_print']);
	update_str(p, [null, 'spi_setup']);
	update_float(p, [null, 'temp_scale_min']);
	update_float(p, [null, 'temp_scale_max']);
	set_name(p, 'unit', 0, 0, machines[uuid].unit_name);
	// IDs.
	var ids = ['bed', 'fan', 'spindle'];
	for (var i = 0; i < ids.length; ++i) {
		var group = p.idgroups[ids[i]];
		for (var e = 0; e < group.length; ++e) {
			var obj = group[e];
			obj.checked = obj.obj[0][1] == p[ids[i] + '_id'];
		}
	}
	update_state(machines[uuid].ui, get_value(p, [null, 'status']));
	// Pin ranges.
	for (var r = 0; r < p.pinranges.length; ++r)
		p.pinranges[r].update();
	var m = p.multiples;
	for (var t = 0; t < m.space.length; ++t) {
		// Add table rows.
		while (m.space[t].tr.length < p.machine.spaces.length) { // TODO: set num spaces at initialization.
			var n = m.space[t].create(m.space[t].tr.length);
			m.space[t].tr.push(n);
			m.space[t].table.insertBefore(n, m.space[t].after);
		}
		if (!(m.space[t].after instanceof Comment)) {
			m.space[t].after.RemoveClass('hidden');
		}
	}
	for (var t = 0; t < m.axis.length; ++t) {
		while (m.axis[t].space.length < p.machine.spaces.length) { // TODO: set num spaces at initialization.
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
		while (m.motor[t].space.length < p.machine.spaces.length) { // TODO: set num spaces at initialization.
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
		while (m.temp[t].tr.length > p.machine.num_temps)
			m.temp[t].table.removeChild(m.temp[t].tr.pop());
		while (m.temp[t].tr.length < p.machine.num_temps) {
			var n = m.temp[t].create(m.temp[t].tr.length);
			m.temp[t].tr.push(n);
			m.temp[t].table.insertBefore(n, m.temp[t].after);
		}
		if (!(m.temp[t].after instanceof Comment)) {
			if (p.machine.num_temps >= 2)
				m.temp[t].after.RemoveClass('hidden');
			else
				m.temp[t].after.AddClass('hidden');
		}
	}
	for (var t = 0; t < m.gpio.length; ++t) {
		while (m.gpio[t].tr.length > p.machine.num_gpios)
			m.gpio[t].table.removeChild(m.gpio[t].tr.pop());
		while (m.gpio[t].tr.length < p.machine.num_gpios) {
			var n = m.gpio[t].create(m.gpio[t].tr.length);
			m.gpio[t].tr.push(n);
			m.gpio[t].table.insertBefore(n, m.gpio[t].after);
		}
		if (!(m.gpio[t].after instanceof Comment)) {
			if (p.machine.num_gpios >= 2)
				m.gpio[t].after.RemoveClass('hidden');
			else
				m.gpio[t].after.AddClass('hidden');
		}
	}
	update_profiles(p);
	update_table_visibility(p);
	for (var i = 0; i < p.machine.spaces.length; ++i)
		space_update(p.machine.uuid, i);
	for (var i = 0; i < p.machine.temps.length; ++i)
		temp_update(p.machine.uuid, i);
	for (var i = 0; i < p.machine.gpios.length; ++i)
		gpio_update(p.machine.uuid, i);
	update_canvas_and_spans(p);
} // }}}

function space_update(uuid, index) { // {{{
	var p = machines[uuid].ui;
	if (!get_element(p, [null, 'container']))
		return;
	set_name(p, 'space', index, 0, p.machine.spaces[index].name);
	if (index == 0) {
		var e = get_element(p, [['space', 0], 'type']);
		e.ClearAll();
		e.AddText(space_types[p.machine.spaces[index].type]);
	}
	update_float(p, [['space', index], 'num_axes']);
	var m = p.multiples;
	for (var t = 0; t < m.axis.length; ++t) {
		while (m.axis[t].space[index].tr.length > p.machine.spaces[index].axis.length)
			m.axis[t].table.removeChild(m.axis[t].space[index].tr.pop());
		while (m.axis[t].space[index].tr.length < p.machine.spaces[index].axis.length) {
			var tr = m.axis[t].create(index, m.axis[t].space[index].tr.length);
			m.axis[t].space[index].tr.push(tr);
			m.axis[t].table.insertBefore(tr, m.axis[t].space[index].after);
		}
		if (!(m.axis[t].space[index].after instanceof Comment)) {
			if (p.machine.spaces[index].num_axes >= 2)
				m.axis[t].space[index].after.RemoveClass('hidden');
			else
				m.axis[t].space[index].after.AddClass('hidden');
		}
	}
	for (var t = 0; t < m.motor.length; ++t) {
		while (m.motor[t].space[index].tr.length > p.machine.spaces[index].motor.length)
			m.motor[t].table.removeChild(m.motor[t].space[index].tr.pop());
		while (m.motor[t].space[index].tr.length < p.machine.spaces[index].motor.length) {
			var tr = m.motor[t].create(index, m.motor[t].space[index].tr.length);
			m.motor[t].space[index].tr.push(tr);
			m.motor[t].table.insertBefore(tr, m.motor[t].space[index].after);
		}
		if (!(m.motor[t].space[index].after instanceof Comment)) {
			if (p.machine.spaces[index].num_axes >= 2)
				m.motor[t].space[index].after.RemoveClass('hidden');
			else
				m.motor[t].space[index].after.AddClass('hidden');
		}
	}
	for (var a = 0; a < p.machine.spaces[index].num_axes; ++a) {
		set_name(p, 'axis', index, a, p.machine.spaces[index].axis[a].name);
		if (index == 0) {
			update_float(p, [['axis', [index, a]], 'park']);
			update_float(p, [['axis', [index, a]], 'park_order']);
			update_float(p, [['axis', [index, a]], 'min']);
			update_float(p, [['axis', [index, a]], 'max']);
			update_float(p, [['axis', [index, a]], 'home_pos2']);
		}
		if (index == 1)
			update_float(p, [['axis', [index, a]], 'multiplier']);
	}
	for (var m = 0; m < p.machine.spaces[index].num_motors; ++m) {
		set_name(p, 'motor', index, m, p.machine.spaces[index].motor[m].name);
		update_pin(p, [['motor', [index, m]], 'step_pin']);
		update_pin(p, [['motor', [index, m]], 'dir_pin']);
		update_pin(p, [['motor', [index, m]], 'enable_pin']);
		update_pin(p, [['motor', [index, m]], 'limit_min_pin']);
		update_pin(p, [['motor', [index, m]], 'limit_max_pin']);
		update_float(p, [['motor', [index, m]], 'steps_per_unit']);
		if (index != 1)
			update_float(p, [['motor', [index, m]], 'home_pos']);
		update_float(p, [['motor', [index, m]], 'limit_v']);
		update_float(p, [['motor', [index, m]], 'limit_a']);
		if (index != 1)
			update_float(p, [['motor', [index, m]], 'home_order']);
	}
	if (p.machine.spaces[index].type == TYPE_DELTA) {
		for (var d = 0; d < 3; ++d) {
			update_float(p, [['motor', [index, d]], 'delta_axis_min']);
			update_float(p, [['motor', [index, d]], 'delta_axis_max']);
			update_float(p, [['motor', [index, d]], 'delta_rodlength']);
			update_float(p, [['motor', [index, d]], 'delta_radius']);
		}
		update_float(p, [['space', index], 'delta_angle']);
	}
	if (p.machine.spaces[index].type == TYPE_POLAR) {
		update_float(p, [['space', index], 'polar_max_r']);
	}
	if (p.machine.spaces[index].type == TYPE_EXTRUDER) {
		for (var d = 0; d < p.machine.spaces[index].axis.length; ++d) {
			update_float(p, [['axis', [index, d]], 'extruder_dx']);
			update_float(p, [['axis', [index, d]], 'extruder_dy']);
			update_float(p, [['axis', [index, d]], 'extruder_dz']);
		}
	}
	if (p.machine.spaces[index].type == TYPE_FOLLOWER) {
		for (var d = 0; d < p.machine.spaces[index].motor.length; ++d) {
			update_float(p, [['motor', [index, d]], 'follower_space']);
			update_float(p, [['motor', [index, d]], 'follower_motor']);
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
	for (var h = 0; h < p.hidetypes.length; ++h) {
		var ht = p.hidetypes[h];
		if (!ht.tr.parentNode || !ht.tr.parentNode.parentNode) {
			continue;
		}
		newhidetypes.push(ht);
		if (hide_check(p.machine.spaces[ht.index].type, ht.only)) {
			ht.tr.AddClass('hidden');
		}
		else {
			ht.tr.RemoveClass('hidden');
		}
	}
	p.hidetypes = newhidetypes;
	update_table_visibility(p);
	update_canvas_and_spans(p);
} // }}}

function temp_update(uuid, index) { // {{{
	var p = machines[uuid].ui;
	if (!get_element(p, [null, 'container']))
		return;
	set_name(p, 'temp', index, 0, p.machine.temps[index].name);
	update_pin(p, [['temp', index], 'heater_pin']);
	update_pin(p, [['temp', index], 'fan_pin']);
	update_pin(p, [['temp', index], 'thermistor_pin']);
	update_float(p, [['temp', index], 'fan_duty']);
	update_float(p, [['temp', index], 'heater_limit_l']);
	update_float(p, [['temp', index], 'heater_limit_h']);
	update_float(p, [['temp', index], 'fan_limit_l']);
	update_float(p, [['temp', index], 'fan_limit_h']);
	update_float(p, [['temp', index], 'fan_temp']);
	update_float(p, [['temp', index], 'R0']);
	update_float(p, [['temp', index], 'R1']);
	update_float(p, [['temp', index], 'Rc']);
	update_float(p, [['temp', index], 'Tc']);
	update_float(p, [['temp', index], 'beta']);
	//update_float(p, [['temp', index], 'core_C']);
	//update_float(p, [['temp', index], 'shell_C']);
	//update_float(p, [['temp', index], 'transfer']);
	//update_float(p, [['temp', index], 'radiation']);
	//update_float(p, [['temp', index], 'power']);
	update_float(p, [['temp', index], 'hold_time']);
	update_float(p, [['temp', index], 'value', 'settemp']);
} // }}}

function gpio_update(uuid, index) { // {{{
	var p = machines[uuid].ui;
	if (!get_element(p, [null, 'container']))
		return;
	set_name(p, 'gpio', index, 0, machines[uuid].gpios[index].name);
	update_pin(p, [['gpio', index], 'pin']);
	if (machines[uuid].gpios[index].reset < 2)
		get_element(p, [['gpio', index], 'statespan']).RemoveClass('input');
	else
		get_element(p, [['gpio', index], 'statespan']).AddClass('input');
	get_element(p, [['gpio', index], 'state']).checked = machines[uuid].gpios[index].value;
	get_element(p, [['gpio', index], 'reset']).selectedIndex = machines[uuid].gpios[index].reset;
	update_float(p, [['gpio', index], 'duty']);
} // }}}
// }}}

// Update helpers. {{{
function update_table_visibility(ui) { // {{{
	for (var t = 0; t < ui.tables.length; ++t) {
		var show = false;
		// Start at 1: skip title row.
		for (var c = 1; c < ui.tables[t].children.length; ++c) {
			if (!ui.tables[t].children[c].HaveClass('hidden')) {
				show = true;
				break;
			}
		}
		if (show) {
			ui.tables[t].RemoveClass('hidden');
		}
		else {
			ui.tables[t].AddClass('hidden');
		}
	}
} // }}}

function update_choice(ui, id) { // {{{
	var value = get_value(ui, id);
	var list = choices[make_id(ui, id)][1];
	var container = get_element(ui, [null, 'container']);
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

function update_toggle(ui, id) { // {{{
	var v = get_value(ui, id);
	var e = get_element(ui, id);
	e.ClearAll();
	e.AddText(e.value[Number(v)]);
} // }}}

function update_pin(ui, id) { // {{{
	var e = get_element(ui, id);
	var value = get_value(ui, id);
	var pin = value & 0xff;
	for (var i = 0; i < e.options.length; ++i) {
		if (Number(e.options[i].value) == pin) {
			e.selectedIndex = i;
			break;
		}
	}
	get_element(ui, id, 'valid').checked = Boolean(value & 0x100);
	if (e.can_invert)
		get_element(ui, id, 'inverted').checked = Boolean(value & 0x200);
} // }}}

function update_float(ui, id) { // {{{
	var e = get_element(ui, id);
	if (e !== null) {
		e.ClearAll();
		e.AddText((get_value(ui, id) / e.factor).toFixed(e.digits));
	}
} // }}}

function update_checkbox(ui, id) { // {{{
	var e = get_element(ui, id);
	if (e !== null)
		e.checked = get_value(ui, id);
} // }}}

function update_str(ui, id) { // {{{
	var e = get_element(ui, id);
	if (e !== null) {
		var value = get_value(ui, id);
		e.ClearAll().AddText(value);
	}
} // }}}

function update_floats(ui, id) { // {{{
	var value = get_value(ui, id);
	for (var i = 0; i < value.length; ++i) {
		var e = get_element(ui, id.concat([i]));
		if (e !== null)
			e.value = String(value[i]);
	}
} // }}}

function update_temprange(ui, id) { // {{{
	var value = get_value(ui, id);
	get_element(ui, id).selectedIndex = value != 255 ? 1 + value : 0;
	return value;
} // }}}

function update_profiles(ui) { // {{{
	ui.machine.call('list_profiles', [], {}, function(profiles) {
		var selector = get_element(ui, [null, 'profiles']);
		if (!selector)
			return;
		selector.ClearAll();
		for (var i = 0; i < profiles.length; ++i) {
			selector.AddElement('option').AddText(profiles[i]).value = profiles[i];
			if (ui.profile == profiles[i])
				selector.selectedIndex = i;
		}
	});
} // }}}

function update_state(ui, state, time) { // {{{
	var c = document.getElementById('container');
	var pre;
	c.RemoveClass('idle printing paused');
	if (state === null) {
		c.AddClass('idle');
		pre = '';
	}
	else if (state) {
		c.AddClass('printing');
		if (time !== undefined)
			pre = '(' + time + ') ';
		else
			pre = '(printing) ';
	}
	else {
		c.AddClass('paused');
		pre = '(pause) ';
	}
	if (selected_machine !== null && selected_machine == ui.machine.uuid)
		document.title = pre + ui.machine.profile + ' - Franklin';
} // }}}
// }}}

// Builders. {{{
function pinrange(ui, type, element) { // {{{
	var pins = [];
	var selected = element.options[element.selectedIndex];
	ui.pinranges.push(pins);
	pins.element = element;
	pins.update = function() {
		var t = 0;
		for (var i = 0; i < ui.machine.pin_names.length; ++i) {
			if (~ui.machine.pin_names[i][0] & type)
				continue;
			while (this.length <= t)
				this.push(null);
			if (this[t] !== null && this[t].value == String(i)) {
				this[t].ClearAll().AddText(ui.machine.pin_names[i][1]);
				t += 1;
				continue;
			}
			var node = Create('option').AddText(ui.machine.pin_names[i][1]);
			node.value = String(i);
			if (selected && node.value == selected.value)
				element.selectedIndex = t;
			if (this[t])
				this.element.replaceChild(this[t], node);
			else
				this.element.Add(node);
			this[t] = node;
			t += 1;
		}
		var new_length = t;
		for (; t < this.length; ++t)
			this.element.removeChild(this[t]);
		this.length = new_length;
	};
	pins.update();
} // }}}

function temprange(ui) { // {{{
	var node = Create('option');
	node.AddText('None');
	node.value = String(0xff);
	var ret = [node];
	for (var i = 0; i < ui.num_temps; ++i) {
		node = Create('option');
		node.Add(temp_name(ui, i));
		node.value = String(i);
		ret.push(node);
	}
	return ret;
} // }}}

function create_space_type_select(ui) { // {{{
	var ret = document.createElement('select');
	for (var o = 0; o < space_types.length; ++o)
		ret.AddElement('option').AddText(space_types[o]);
	return ret;
} // }}}

var choices = new Object;

function Choice(ui, obj, options, classes, containerclasses) { // {{{
	var ret = [];
	var id = make_id(ui, obj);
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
		e.id = make_id(ui, obj, String(i));
		if (containerclasses)
			e.containerclass = containerclasses[i];
		e.value = String(i);
		e.ui = ui;
		e.addEventListener('click', function() {
		       	set_value(this.ui, choices[this.name][0], Number(this.value));
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

function UnitTitle(ui, title, post, pre) { // {{{
	var ret = Create('span').AddText(title + ' (' + (pre ? pre : ''));
	ret.Add(add_name(ui, 'unit', 0, 0));
	return ret.AddText((post ? post : '') + ')');
} // }}}

function space_name(ui, index) { // {{{
	if (index === null)
		return 'all spaces';
	return add_name(ui, 'space', index, 0);
} // }}}

function axis_name(ui, index1, index2) { // {{{
	if (index2 === null)
		return ['all ', space_name(ui, index1), ' axes'];
	return add_name(ui, 'axis', index1, index2);
} // }}}

function motor_name(ui, index1, index2) { // {{{
	if (index2 === null)
		return ['all ', space_name(ui, index1), ' motors'];
	return add_name(ui, 'motor', index1, index2);
} // }}}

function delta_name(ui, index1, index2) { // {{{
	if (index1 === null)
		return 'all apexes';
	return motor_name(ui, index1, index2);
} // }}}

function temp_name(ui, index) { // {{{
	if (index === null)
		return 'all temps';
	return add_name(ui, 'temp', index, 0);
} // }}}

function gpio_name(ui, index) { // {{{
	if (index === null)
		return 'all gpios';
	return add_name(ui, 'gpio', index, 0);
} // }}}

function make_pin_title(ui, title, content) { // {{{
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
	value: function(ui, type, template, all, forbidden) {
		var one = function(t, template, i, arg) {
			var ret = template(ui, i, arg, t);
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
			ui.multiples[type].push({table: me, tr: [], after: last, create: function(i) { return one(me, template, i) || document.createComment(''); }});
		}
		else {
			me.appendChild(last);
			ui.multiples[type].push({table: me, space: [], after: last, all: all != false, create: function(i, arg) { return (i != forbidden && one(me, template, i, arg)) || document.createComment(''); }});
		}
		return me;
	}
}); // }}}

function make_table(ui) { // {{{
	var t = document.createElement('table');
	ui.tables.push(t);
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

function make_tablerow(ui, title, cells, classes, id, onlytype, index) { // {{{
	var ret = document.createElement('tr');
	if (id)
		ret.id = make_id(ui, id);
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
			current_cell.id = make_id(ui, id, cell);
		if (cells[cell] instanceof Element) {
			current_cell.Add(cells[cell]);
		}
		else {
			for (var e = 0; e < cells[cell].length; ++e)
				current_cell.Add(cells[cell][e]);
		}
	}
	if (onlytype !== undefined && index != null) {
		ui.hidetypes.push({only: onlytype, index: index, tr: ret});
	}
	return ret;
} // }}}
// }}}

// Set helpers (to server). {{{
function set_pin(ui, id) { // {{{
	var e = get_element(ui, id);
	var valid = get_element(ui, id, 'valid').checked;
	var inverted;
	if (e.can_invert)
		inverted = get_element(ui, id, 'inverted').checked;
	else
		inverted = false;
	set_value(ui, id, Number(e.selectedOptions[0].value) + 0x100 * valid + 0x200 * inverted);
} // }}}

function set_file(ui, id, action) { // {{{
	var element = get_element(ui, id);
	if (element.files.length < 1) {
		alert('please select a file');
		return;
	}
	var post = new XMLHttpRequest();
	var fd = new FormData();
	fd.append('machine', ui.machine.uuid);
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
	var s = Math.floor(t);
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

function update_canvas_and_spans(ui, space) { // {{{
	// The machine may have disappeared.
	if (!machines[ui.machine.uuid])
		return;
	if (space === undefined) {
		space = 0;
	}
	if (space < 2) {	// Ignore follower positions.
		ui.machine.call('get_axis_pos', [space], {}, function(x) {
			//dbg('update ' + space + ',' + axis + ':' + x);
			if (!machines[ui.machine.uuid])
				return;
			for (var i = 0; i < x.length; ++i) {
				if (ui.machine.spaces[space].axis[i]) {
					ui.machine.spaces[space].axis[i].current = x[i];
					update_float(ui, [['axis', [space, i]], 'current']);
				}
			}
			update_canvas_and_spans(ui, space + 1);
		});
		return;
	}
	ui.machine.call('get_print_state', [], {}, function(state) {
		if (!machines[ui.machine.uuid])
			return;
		var e = get_element(ui, [null, 'printstate']).ClearAll();
		if (isNaN(state[1]))
			e.AddText('State: ' + state[0]);
		else {
			var remaining = display_time(state[2] - state[1]);
			e.AddText('State: ' + state[0] + ' - Time: ' + display_time(state[1]) + '/' + display_time(state[2]) + ' - Remaining: ' + remaining);
			update_state(ui, get_value(ui, [null, 'status']), remaining);
		}
		update_float(ui, [null, 'targetangle']);
		ui.machine.tppos = state[3];
		ui.tp_context = state[5];
		update_float(ui, [null, 'tppos']);
		get_element(ui, [null, 'tpmax']).ClearAll().AddText(state[4]);
		redraw_canvas(ui);
	});
}
// }}}

function redraw_canvas(ui) { // {{{
	if (ui.machine.spaces.length < 1 || ui.machine.spaces[0].axis.length < 1)
		return;
	var canvas = document.getElementById(make_id(ui, [null, 'xymap']));
	if (ui.machine.spaces[0].axis.length >= 2) { // Draw XY {{{
		var c = canvas.getContext('2d');
		var box = document.getElementById(make_id(ui, [null, 'map']));
		var extra_height = box.clientHeight - canvas.clientHeight;
		var machinewidth;
		var machineheight;
		var outline, center;
		switch (ui.machine.spaces[0].type) { // Define geometry-specific options, including outline. {{{
		case TYPE_CARTESIAN:
			var xaxis = ui.machine.spaces[0].axis[0];
			var yaxis = ui.machine.spaces[0].axis[1];
			machinewidth = xaxis.max - xaxis.min + .010;
			machineheight = yaxis.max - yaxis.min + .010;
			center = [(xaxis.min + xaxis.max) / 2, (yaxis.min + yaxis.max) / 2];
			outline = function(ui, c) {
				// Rectangle is always drawn; nothing else to do here.
			};
			break;
		case TYPE_DELTA:
			var radius = [];
			var length = [];
			for (var a = 0; a < 3; ++a) {
				radius.push(ui.machine.spaces[0].motor[a].delta_radius);
				length.push(ui.machine.spaces[0].motor[a].delta_rodlength);
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
			outline = function(ui, c) {
				c.save(); // Rotate the outline. {{{
				c.rotate(ui.machine.spaces[0].delta_angle);
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
					var name = ui.machine.spaces[0].motor[a].name;
					var w = c.measureText(name).width;
					c.beginPath();
					c.save(); // Print axis name. {{{
					c.translate(origin[a][0] + dy[a] * 10 - w / 2, origin[a][1] - dx[a] * 10);
					c.rotate(-ui.machine.spaces[0].delta_angle);
					c.scale(1, -1);
					c.fillText(name, 0, 0);
					c.restore(); // }}}
				}
				c.restore(); // }}}
			};
			var extra = c.measureText(ui.machine.spaces[0].motor[0].name).width + .02;
			machinewidth = 2 * (maxx + extra);
			machineheight = 2 * (maxy + extra);
			break;
		case TYPE_POLAR:
			machinewidth = 2 * ui.machine.spaces[0].polar_max_r + 2;
			machineheight = 2 * ui.machine.spaces[0].polar_max_r + 2;
			center = [0, 0];
			outline = function(ui, c) {
				c.beginPath();
				c.arc(0, 0, ui.machine.spaces[0].polar_max_r, 0, 2 * Math.PI, false);
				c.stroke();
			};
			break;
		} // }}}
		// Prepare canvas. {{{
		//var factor = Math.sqrt(machinewidth * machinewidth + machineheight * machineheight);
		var factor = Math.max(machinewidth, machineheight);
		canvas.style.height = canvas.clientWidth + 'px';
		canvas.width = canvas.clientWidth;
		canvas.height = canvas.clientWidth;
		// }}}

		var b = ui.bbox;
		var true_pos = [ui.machine.spaces[0].axis[0].current, ui.machine.spaces[0].axis[1].current];

		c.save(); // Use machine coordinates for canvas. {{{
		// Clear canvas.
		c.clearRect(0, 0, canvas.width, canvas.width);

		c.translate(canvas.width / 2, canvas.width / 2);
		c.scale(canvas.width / factor, -canvas.width / factor);
		c.lineWidth = 1.5 * factor / canvas.width;
		c.translate(-center[0], -center[1]);
		// }}}

		get_pointer_pos_xy = function(ui, e) { // {{{
			var rect = canvas.getBoundingClientRect();
			var x = e.clientX - rect.left - canvas.width / 2;
			var y = e.clientY - rect.top - canvas.width / 2;
			x /= canvas.width / factor;
			y /= -canvas.width / factor;
			return [x, y];
		}; // }}}

		// Draw outline. {{{
		c.strokeStyle = '#888';
		c.fillStyle = '#888';
		outline(ui, c);
		// }}}
		// Draw limits. {{{
		c.beginPath();
		var a = ui.machine.spaces[0].axis;
		c.moveTo(a[0].min, a[1].min);
		c.lineTo(a[0].max, a[1].min);
		c.lineTo(a[0].max, a[1].max);
		c.lineTo(a[0].min, a[1].max);
		c.closePath();
		c.stroke();
		// }}}
		// Draw center. {{{
		c.beginPath();
		c.moveTo(1, 0);
		c.arc(0, 0, 1, 0, 2 * Math.PI);
		c.fillStyle = '#888';
		c.fill();
		// }}}

		c.save(); // Draw context. {{{
		var current_pos = null;
		var center = null;
		var normal = null;
		c.translate(ui.machine.targetx, ui.machine.targety);
		c.rotate(ui.machine.targetangle);
		for (var i = 0; i < ui.tp_context[1].length; ++i) {
			r = ui.tp_context[1][i];
			switch (r[0]) {
				case 'PREARC':
					center = [r[2], r[3], r[4]];
					normal = [r[5], r[6], r[7]];
					continue;
				case 'ARC':
					if (current_pos === null)
						break;
					if (center === null)
						break;
					// TODO: implement this.  Workaround: fall through.
				case 'LINE':
					if (current_pos === null || isNaN(current_pos[0]) || isNaN(current_pos[1]))
						break;
					c.beginPath();
					c.moveTo(current_pos[0], current_pos[1]);
					c.lineTo(r[2], r[3]);
					c.strokeStyle = i + ui.tp_context[0] >= ui.machine.tppos ? '#00f' : '#f00';
					c.stroke();
			}
			current_pos = [r[2], r[3], r[4]];
		}
		c.restore(); // }}}

		// Draw current location. {{{
		c.beginPath();
		c.fillStyle = '#44f';
		c.moveTo(true_pos[0] + 3, true_pos[1]);
		c.arc(true_pos[0], true_pos[1], 3, 0, 2 * Math.PI);
		c.fill();
		// }}}

		c.save(); // Draw bounding box and context at target position. {{{
		c.translate(ui.machine.targetx, ui.machine.targety);
		c.rotate(ui.machine.targetangle);

		c.beginPath();
		if (b[0] != b[1] && b[2] != b[3]) {
			// Draw print bounding box. {{{
			c.rect(b[0], b[2], b[1] - b[0], b[3] - b[2]);
			// }}}

			// Draw tick marks. {{{
			c.moveTo((b[1] + b[0]) / 2, b[2]);
			c.lineTo((b[1] + b[0]) / 2, b[2] + 5);

			c.moveTo((b[1] + b[0]) / 2, b[3]);
			c.lineTo((b[1] + b[0]) / 2, b[3] - 5);

			c.moveTo(b[0], (b[3] + b[2]) / 2);
			c.lineTo(b[0] + 5, (b[3] + b[2]) / 2);

			c.moveTo(b[1], (b[3] + b[2]) / 2);
			c.lineTo(b[1] - 5, (b[3] + b[2]) / 2);
			// }}}

			// Draw central cross. {{{
			c.moveTo((b[1] + b[0]) / 2 - 5, (b[3] + b[2]) / 2);
			c.lineTo((b[1] + b[0]) / 2 + 5, (b[3] + b[2]) / 2);
			c.moveTo((b[1] + b[0]) / 2, (b[3] + b[2]) / 2 + 5);
			c.lineTo((b[1] + b[0]) / 2, (b[3] + b[2]) / 2 - 5);
			// }}}
		}

		// Draw zero. {{{
		c.moveTo(3, 0);
		c.arc(0, 0, 3, 0, 2 * Math.PI);
		// }}}

		// Update it on screen.
		c.strokeStyle = '#000';
		c.stroke();

		c.restore(); // }}}
	} // }}}

	if (ui.machine.spaces[0].axis.length != 2) { // Draw Z {{{
		var zaxis, zoffset;
		if (ui.machine.spaces[0].axis.length >= 3) {
			zaxis = ui.machine.spaces[0].axis[2];
			zoffset = ui.machine.zoffset;
		}
		else {
			zaxis = ui.machine.spaces[0].axis[0];
			zoffset = 0;
		}
		var zcanvas = document.getElementById(make_id(ui, [null, 'zmap']));
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

		get_pointer_pos_z = function(ui, e) {
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

function start_move(ui) { // {{{
	// Update bbox.
	var q = get_element(ui, [null, 'queue']);
	ui.bbox = [null, null, null, null];
	for (var e = 0; e < q.options.length; ++e) {
		if (!q.options[e].selected)
			continue;
		var name = q.options[e].value;
		var item;
		for (item = 0; item < ui.machine.queue.length; ++item)
			if (ui.machine.queue[item][0] == name)
				break;
		if (item >= ui.machine.queue.length)
			continue;
		if (ui.bbox[0] == null || ui.machine.queue[item][1][0] < ui.bbox[0])
			ui.bbox[0] = ui.machine.queue[item][1][0];
		if (ui.bbox[1] == null || ui.machine.queue[item][1][1] > ui.bbox[1])
			ui.bbox[1] = ui.machine.queue[item][1][1];
		if (ui.bbox[2] == null || ui.machine.queue[item][1][2] < ui.bbox[2])
			ui.bbox[2] = ui.machine.queue[item][1][2];
		if (ui.bbox[3] == null || ui.machine.queue[item][1][3] > ui.bbox[3])
			ui.bbox[3] = ui.machine.queue[item][1][3];
	}
	update_canvas_and_spans(ui);
} // }}}

function reset_position(ui) { // {{{
	ui.machine.targetangle = 0;
	update_canvas_and_spans(ui);
} // }}}

var drag = [[NaN, NaN], [NaN, NaN], [NaN, NaN], 0, false];

function xydown(ui, e) { // {{{
	var pos = get_pointer_pos_xy(ui, e);
	drag[0][0] = pos[0];
	drag[1][0] = pos[1];
	drag[4] = false;
	if (e.button == 0) {
		ui.machine.call('get_axis_pos', [0, 0], {}, function(x) {
			ui.machine.call('get_axis_pos', [0, 1], {}, function(y) {
				drag[0][1] = x;
				drag[1][1] = y;
			});
		});
	}
	else {
		drag[0][1] = ui.machine.targetx;
		drag[1][1] = ui.machine.targety;
	}
	return false;
}
// }}}

function xyup(ui, e) { // {{{
	var pos = get_pointer_pos_xy(ui, e);
	if (drag[4])
		return false;
	if (!(e.buttons & 5))
		return false;
	if (e.buttons & 1) {
		ui.machine.call('line_cb', [[[pos[0], pos[1]]]], {}, function() { update_canvas_and_spans(ui); });
	}
	else if (e.buttons & 4) {
		drag[3] += 1;
		ui.machine.call('set_globals', [], {'targetx': pos[0], 'targety': pos[1]}, function() { drag[3] -= 1; });
	}
	return false;
}
// }}}

function xymove(ui, e) { // {{{
	drag[4] = true;
	if (drag[3])
		return false;
	if (!(e.buttons & 5)) {
		drag[0][1] = NaN;
		drag[1][1] = NaN;
		return false;
	}
	var pos = get_pointer_pos_xy(ui, e);
	var dx = pos[0] - drag[0][0];
	var dy = pos[1] - drag[1][0];
	if (isNaN(drag[0][1]) || isNaN(drag[1][1]))
		return false;
	if (e.buttons & 1) {
		drag[3] += 1;
		ui.machine.call('line_cb', [[[drag[0][1] + dx, drag[1][1] + dy]]], {}, function() { drag[3] -= 1; update_canvas_and_spans(ui); });
	}
	else if (e.buttons & 4) {
		drag[3] += 1;
		ui.machine.call('line_cb', [[[dx, dy]]], {relative: true}, function() {
			ui.machine.call('move_target', [dx, dy], {}, function() {
				drag[0][0] += dx;
				drag[1][0] += dy;
				drag[3] -= 1;
			});
		});
	}
	return false;
}
// }}}

function zdown(ui, e) { // {{{
	drag[4] = false;
	drag[2][0] = get_pointer_pos_z(ui, e);
	if (e.button == 0)
		ui.machine.call('get_axis_pos', [0, 2], {}, function(z) { drag[2][1] = z; });
	return false;
}
// }}}

function zup(ui, e) { // {{{
	var pos = get_pointer_pos_z(ui, e);
	if (drag[4])
		return false;
	if (!(e.buttons & 5))
		return false;
	if (e.buttons & 1) {
		ui.machine.call('line_cb', [[{2: pos}]], {}, function() { update_canvas_and_spans(ui); });
	}
	return false;
}
// }}}

function zmove(ui, e) { // {{{
	drag[4] = true;
	if (drag[3])
		return false;
	if (!(e.buttons & 5)) {
		drag[2][1] = NaN;
		return false;
	}
	var dz = get_pointer_pos_z(ui, e) - drag[2][0];
	if (isNaN(drag[2][1]))
		return false;
	if (e.buttons & 1) {
		drag[3] = true;
		ui.machine.call('line_cb', [[{2: drag[2][1] + dz}]], {}, function() { drag[3] = false; update_canvas_and_spans(ui); });
	}
	return false;
}
// }}}

function keypress(event) { // {{{
	var ui = selected_machine;
	if (!ui)
		return;
	if (event.keyCode >= 37 && event.keyCode <= 40 && (event.ctrlKey || event.altKey)) {
		var amount = 10;	// TODO: make this a setting.
		if (event.shiftKey)
			amount /= 10;
		var target = [[-amount, 0], [0, amount], [amount, 0], [0, -amount]][event.keyCode - 37];
		ui.machine.call('line_cb', [[target]], {relative: true}, function() {
			if (event.altKey) {
				ui.machine.call('set_globals', [], {'targetx': ui.machine.targetx + target[0], 'targety': ui.machine.targety + target[1]});
			}
			else
				update_canvas_and_spans(ui);
		});
		event.preventDefault();
	}
	else if (event.charCode >= 48 && event.charCode <= 57 && event.ctrlKey) {
		var minx = ui.bbox[0];
		var maxx = ui.bbox[1];
		var miny = ui.bbox[2];
		var maxy = ui.bbox[3];
		var pos = [[0, 0],
			[minx, miny], [(minx + maxx) / 2, miny], [maxx, miny],
			[minx, (miny + maxy) / 2], [(minx + maxx) / 2, (miny + maxy) / 2], [maxx, (miny + maxy) / 2],
			[minx, maxy], [(minx + maxx) / 2, maxy], [maxx, maxy]][event.charCode - 48];
		var posx = Math.cos(ui.machine.targetangle) * pos[0] - Math.sin(ui.machine.targetangle) * pos[1];
		var posy = Math.cos(ui.machine.targetangle) * pos[1] + Math.sin(ui.machine.targetangle) * pos[0];
		posx += ui.machine.targetx;
		posy += ui.machine.targety;
		ui.machine.call('line_cb', [[[posx, posy]]], {}, function() { update_canvas_and_spans(ui); });
		event.preventDefault();
	}
} // }}}

function update_angle(ui, value) { // {{{
	ui.machine.call('get_axis_pos', [0], {}, function(pos) {
		pos[0] -= ui.machine.targetx;
		pos[1] -= ui.machine.targety;
		var posx = Math.cos(ui.machine.targetangle) * pos[0] + Math.sin(ui.machine.targetangle) * pos[1];
		var posy = Math.cos(ui.machine.targetangle) * pos[1] - Math.sin(ui.machine.targetangle) * pos[0];
		pos[0] = Math.cos(value) * posx - Math.sin(value) * posy + ui.machine.targetx;
		pos[1] = Math.cos(value) * posy + Math.sin(value) * posx + ui.machine.targety;
		set_value(ui, [null, 'targetangle'], value);
		ui.machine.call('line_cb', [[[pos[0], pos[1]]]], {}, function() {
			update_canvas_and_spans(ui);
		});
	});
}
// }}}
// }}}

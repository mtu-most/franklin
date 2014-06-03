// vim: set foldmethod=marker :
// Variables.  {{{
var ports;
var labels_element, printers_element;
var selected_port, selected_printer;
var script_cbs;
var visibles;
var type2plural = {axis: 'axes', extruder: 'extruders', temp: 'temps', gpio: 'gpios'};
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
	visibles = new Object;
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
	register_update('del_script', del_script);
	register_update('del_printer', del_printer);
	register_update('del_port', del_port);
	register_update('variables_update', update_variables);
	register_update('axis_update', update_axis);
	register_update('extruder_update', update_extruder);
	register_update('temp_update', update_temp);
	register_update('gpio_update', update_gpio);
	if (document.location.search.length > 0 && document.location.search[0] == '?') {
		var things = document.location.search.split('&');
		for (var t = 0; t < things.length; ++t) {
			var items = things[t].substring(1).split('=');
			if (items[1] == '0') {
				var e = document.getElementById(items[0] + 'box');
				if (e)
					e.checked = false;
			}
			else if (items[1] == '1') {
				var e = document.getElementById(items[0] + 'box');
				if (e)
					e.checked = true;
			}
		}
	}
	var items = ['tweak', 'expert', 'setup'];
	for (var i = 0; i < items.length; ++i)
		switch_show (document.getElementById (items[i] + 'box').checked, items[i]);
} // }}}

function make_id(printer, id, extra) { // {{{
	var ret = printer ? printer.port.replace(/[^a-zA-Z0-9_]/g, '') : '';
	if (id[0] !== null) {
		if (typeof id[0][0] == 'string')
			ret += '_' + id[0][0];
		else {
			for (var i = 0; i < id[0][0].length; ++i)
				ret += '_' + id[0][0][i];
		}
		ret += '_' + String(id[0][1]);
	}
	ret += '_' + id[1];
	if (id.length > 2 && typeof id[2] == 'number')
		ret += '_' + String(id[2]);
	if (extra !== null && extra !== undefined)
		ret += '_' + String(extra);
	return ret;
} // }}}

function set_value(printer, id, value, reply, arg) { // {{{
	var target;
	if (id.length == 2) {
		var obj = {};
		obj[id[1]] = value;
		if (id[0] === null) {
			printer.call('set_variables', [], obj, reply);
		}
		else if (typeof id[0][0] == 'string') {
			printer.call('set_' + id[0][0], [id[0][1]], obj, reply);
		}
		else {
			for (var i = 1; i < id[0][0].length; ++i) {
				var newobj = {};
				newobj[id[0][0][i]] = obj;
				obj = newobj;
			}
			printer.call('set_' + id[0][0][0], [id[0][1]], obj, reply);
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
		else
			printer.call(id[2] + '_' + (typeof id[0][0] == 'string' ? id[0][0] : id[0][0][0]), [id[0][1], value], {}, reply);
	}
} // }}}

function get_value(printer, id) { // {{{
	if (id[0] === null)
		return printer[id[1]];
	if (typeof id[0][0] == 'string')
		return printer[id[0][0]][id[0][1]][id[1]];
	var obj = printer[id[0][0][0]][id[0][1]];
	for (var i = 1; i < id[0][0].length; ++i)
		obj = obj[id[0][0][i]];
	return obj[id[1]];
} // }}}

function get_element(printer, id, extra) { // {{{
	return document.getElementById(make_id(printer, id, extra));
} // }}}

function switch_show(state, which) { // {{{
	var e = document.getElementById('container');
	if (state)
		e.RemoveClass('no' + which);
	else
		e.AddClass('no' + which);
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
			ports[p][2].RemoveClass('hidden');
			if (ports[p][1])
				ports[p][1].RemoveClass('hidden');
		}
		else {
			ports[p][0].RemoveClass('active');
			ports[p][2].AddClass('hidden');
			if (ports[p][1])
				ports[p][1].AddClass('hidden');
		}
	}
} // }}}

function make_visibles(num) { // {{{
	var ret = [];
	for (var i = 0; i < num; ++i)
		ret[i] = [];
	return ret;
}
// }}}

function upload_buttons(port, buttons) { // {{{
	var ret = document.createElement('ul');
	for (var b = 0; b < buttons.length; ++b) {
		var button = ret.AddElement('li').AddElement('button');
		button.type = 'button';
		button.target = buttons[b][0]
		button.onclick = function() {
			rpc.call ('upload', [port, this.target], {}, function(ret) { alert('upload done: ' + ret);});
		};
		button.AddText(buttons[b][1]);
	}
	return [ret];
}
// }}}
// }}}

// Queue functions.  {{{
function queue_deselect() { // {{{
	var e = document.getElementById('queue');
	for (var i = 1; i < e.options.length; ++i) {
		e.options[i].selected = false;
	}
}
// }}}

function queue_up() { // {{{
	var e = document.getElementById('queue');
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
	var e = document.getElementById('queue');
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
	var rm = [];
	var e = document.getElementById('queue');
	for (var i = 0; i < e.options.length; ++i) {
		if (e.options[i].selected)
			rm.push(e.options[i]);
	}
	for (var i = 0; i < rm.length; ++i)
		rpc.call('queue_remove', [rm[i].value], {});
}
// }}}

function get_queue() { // {{{
	var toprint = [];
	var e = document.getElementById('queue');
	for (var i = 0; i < e.options.length; ++i) {
		if (e.options[i].selected)
			toprint.push(e.options[i].value);
	}
	return toprint;
}
// }}}

function queue_print() { // {{{
	selected_printer.call('get_axis_pos', [0], {}, function(x) {
		selected_printer.call('get_axis_pos', [1], {}, function(y) {
			var r = selected_printer.reference;
			var sina = Math.sin(selected_printer.local_angle);
			var cosa = Math.cos(selected_printer.local_angle);
			selected_printer.call('queue_print', [get_queue(), [x[1] - (r[0] * cosa - r[1] * sina), y[1] - (r[1] * cosa + r[0] * sina)], selected_printer.local_angle * 180 / Math.PI], {});
		});
	});
}
// }}}

function queue_mill () { // {{{
	var q = get_queue();
	selected_printer.call('get_axis_pos', [0], {}, function(x) {
		selected_printer.call('get_axis_pos', [1], {}, function(y) {
			var r = selected_printer.reference;
			var sina = Math.sin(selected_printer.local_angle);
			var cosa = Math.cos(selected_printer.local_angle);
			selected_printer.call('queue_probe', [q, [x[1] - (r[0] * cosa - r[1] * sina), y[1] - (r[1] * cosa + r[0] * sina)], selected_printer.local_angle * 180 / Math.PI], {}, function(map) {
				selected_printer.request_confirmation('Prepare for milling', [], {}, function(success) {
					if (!success)
						return;
					selected_printer.call('queue_print', [q, [x[1] - (r[0] * cosa - r[1] * sina), y[1] - (r[1] * cosa + r[0] * sina)], selected_printer.local_angle * 180 / Math.PI, map], {});
				});
			});
		});
	});
}
// }}}

function quick_print() { // {{{
	selected_printer.call('queue_print', [get_queue(), [0, 0], 0], {});
}
// }}}
// }}}

// Non-update events. {{{
function new_port(port) { // {{{
	ports[port] = [get_elements(build('Label', [port]))[0], get_elements(build('NoPrinter', [port]))[0], null, ''];
	ports[port][0].AddClass('setup');
	labels_element.Add(ports[port][0]);
	printers_element.Add(ports[port][1]);
} // }}}

function new_printer() { // {{{
	printer.reference = [0, 0];
	printer.local_angle = 0;
	visibles[port] = {titles: [], axis: make_visibles(printer.maxaxes), extruder: make_visibles(printer.maxextruders), temp: make_visibles(printer.maxtemps), gpio: make_visibles(printer.maxgpios)};
	ports[port][2] = get_elements(build('Printer', [port]))[0];
	printers_element.Add(ports[port][2]);
	ports[port][0].RemoveClass('setup');
	ports[port][1].RemoveClass('notconnected');
	ports[port][1].AddClass('connected');
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
	delete visibles[port];
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

function do_confirm(message) { // {{{
	var the_printer = printer;
	the_printer.call('confirm', [confirm(message)], {});
} // }}}

function do_queue() { // {{{
	var e = document.getElementById('queue');
	var q = [];
	var must_deselect = queue.length > e.options.length;
	for (var i = 0; i < queue.length; ++i)
		q.push(queue[i][0]);
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
} // }}}
// }}}

// Update events(from server). {{{
function update_variables() { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	if (ports[port][3] != printer.name) {
		labels_element.removeChild(ports[port][0]);
		ports[port][0] = get_elements(build('Label', [port]))[0];
		labels_element.Add(ports[port][0]);
		ports[port][3] = printer.name;
		select_printer();
	}
	get_element(printer, [null, 'export']).href = printer.name + '.ini?port=' + encodeURIComponent(port);
	update_range([null, 'num_axes']);
	update_range([null, 'num_extruders']);
	update_range([null, 'num_temps']);
	update_range([null, 'num_gpios']);
	update_choice([null, 'printer_type']);
	update_pin([null, 'led_pin']);
	update_pin([null, 'probe_pin']);
	update_float([null, 'room_T']);
	update_float([null, 'motor_limit']);
	update_float([null, 'temp_limit']);
	update_float([null, 'feedrate']);
	update_float([null, 'angle']);
	update_toggle([null, 'paused', 'pause']);
	// Update visibility.
	for (var i = 0; i < printer.num_axes; ++i) {
		for (var j = 0; j < visibles[port].axis[i].length; ++j)
			visibles[port].axis[i][j].RemoveClass('hidden');
	}
	for (var i = printer.num_axes; i < printer.maxaxes; ++i) {
		for (var j = 0; j < visibles[port].axis[i].length; ++j)
			visibles[port].axis[i][j].AddClass('hidden');
	}
	for (var i = 0; i < printer.num_extruders; ++i) {
		for (var j = 0; j < visibles[port].extruder[i].length; ++j)
			visibles[port].extruder[i][j].RemoveClass('hidden');
	}
	for (var i = printer.num_extruders; i < printer.maxextruders; ++i) {
		for (var j = 0; j < visibles[port].extruder[i].length; ++j)
			visibles[port].extruder[i][j].AddClass('hidden');
	}
	for (var i = 0; i < printer.num_temps; ++i) {
		for (var j = 0; j < visibles[port].temp[i].length; ++j)
			visibles[port].temp[i][j].RemoveClass('hidden');
	}
	for (var i = printer.num_temps; i < printer.maxtemps; ++i) {
		for (var j = 0; j < visibles[port].temp[i].length; ++j)
			visibles[port].temp[i][j].AddClass('hidden');
	}
	for (var i = 0; i < printer.num_gpios; ++i) {
		for (var j = 0; j < visibles[port].gpio[i].length; ++j)
			visibles[port].gpio[i][j].RemoveClass('hidden');
	}
	for (var i = printer.num_gpios; i < printer.maxgpios; ++i) {
		for (var j = 0; j < visibles[port].gpio[i].length; ++j)
			visibles[port].gpio[i][j].AddClass('hidden');
	}
	for (var i = 0; i < visibles[port].titles.length; ++i) {
		var check_titles = function(items, objs, add, remove) {
			for (var j = 0; j < items.length; ++j) {
				if (printer['num_' + type2plural[items[j]]] != 0)
					break;
			}
			for (var k = 0; k < objs.length; ++k) {
				if (j < items.length) {
					for (var c = 0; c < remove.length; ++c)
						objs[k].RemoveClass(remove[c]);
				}
				else {
					for (var c = 0; c < add.length; ++c)
						objs[k].AddClass(add[c]);
				}
			}
			return j < items.length;
		};
		var classes = ['hidden', 'expert_hidden', 'tweak_hidden', 'basic_hidden'];
		var next = [];
		for (var v = 0; v < visibles[port].titles[i][0].length; ++v) {
			if (check_titles(visibles[port].titles[i][0][v], visibles[port].titles[i][1], next, classes))
				break;
			next = [classes.pop()];
		}
		if (v >= visibles[port].titles[i][0].length)
			check_titles([], visibles[port].titles[i][1], ['hidden'], []);
	}
} // }}}

function update_axis(index) { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	update_motor([['axis', 'motor'], index]);
	update_pin([['axis', index], 'limit_min_pin']);
	update_pin([['axis', index], 'limit_max_pin']);
	update_pin([['axis', index], 'sense_pin']);
	update_float([['axis', index], 'limit_pos']);
	update_float([['axis', index], 'axis_min']);
	update_float([['axis', index], 'axis_max']);
	update_float([['axis', index], 'motor_min']);
	update_float([['axis', index], 'motor_max']);
	update_float([['axis', index], 'delta_length']);
	update_float([['axis', index], 'delta_radius']);
	update_float([['axis', index], 'offset']);
	update_float([['axis', index], 'park']);
} // }}}

function update_extruder(index) { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	update_motor([['extruder', 'motor'], index]);
	update_tempcontent([['extruder', 'temp'], index]);
	update_float([['extruder', index], 'filament_heat']);
	update_float([['extruder', index], 'nozzle_size']);
	update_float([['extruder', index], 'filament_size']);
} // }}}

function update_temp(index) { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	update_tempcontent(['temp', index]);
} // }}}

function update_gpio(index) { // {{{
	if (!get_element(printer, [null, 'container']))
		return;
	update_pin([['gpio', index], 'pin']);
	update_choice([['gpio', index], 'state']);
	var master = update_temprange([['gpio', index], 'master']);
	update_float([['gpio', index], 'value']);
	if (master) {
		get_element(printer, [['gpio', index], 'state'], 0).AddClass('empty');
		get_element(printer, [['gpio', index], 'state']).AddClass('expert');
	}
	else {
		get_element(printer, [['gpio', index], 'state'], 0).RemoveClass('empty');
		get_element(printer, [['gpio', index], 'state']).RemoveClass('expert');
	}
} // }}}
// }}}

// Update helpers. {{{
function update_motor(id) { // {{{
	update_pin([id, 'step_pin']);
	update_pin([id, 'dir_pin']);
	update_pin([id, 'enable_pin']);
	update_float([id, 'steps_per_mm']);
	update_float([id, 'limit_v']);
	update_float([id, 'max_v']);
	update_float([id, 'max_a']);
	update_float([id, 'max_steps']);
	var speed = get_value(printer, [id, 'runspeed']);
	var minus = get_element(printer, [id, 'minus']);
	var plus = get_element(printer, [id, 'plus']);
	var value = get_element(printer, [id, 'value']);
	if (isNaN(speed) || speed == 0) {
		minus.checked = false;
		plus.checked = false;
	}
	else {
		if (speed < 0) {
			minus.checked = true;
			plus.checked = false;
		}
		else {
			minus.checked = false;
			plus.checked = true;
		}
		value.value = String(Math.abs(speed));
	}
	//update_float([id, 'sleeping']);
} // }}}

function update_tempcontent(id) { // {{{
	update_pin([id, 'power_pin']);
	update_pin([id, 'thermistor_pin']);
	update_float([id, 'R0']);
	update_float([id, 'R1']);
	update_float([id, 'Rc']);
	update_float([id, 'Tc']);
	update_float([id, 'beta']);
	update_float([id, 'core_C']);
	update_float([id, 'shell_C']);
	update_float([id, 'transfer']);
	update_float([id, 'radiation']);
	update_float([id, 'power']);
	update_float([id, 'value', 'settemp']);
} // }}}

function update_range(id) { // {{{
	get_element(printer, id).selectedIndex = get_value(printer, id);
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
	e.ClearAll ();
	e.AddText (e.value[Number(v)]);
} // }}}

function update_pin(id) { // {{{
	get_element(printer, id).selectedIndex = get_value(printer, id) & 0xff;
	get_element(printer, id, 'invalid').checked = Boolean(get_value(printer, id) & 0x100);
	get_element(printer, id, 'inverted').checked = Boolean(get_value(printer, id) & 0x200);
} // }}}

function update_float(id) { // {{{
	var e = get_element(printer, id);
	e.ClearAll();
	e.AddText((get_value(printer, id) / e.factor).toFixed(1));
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
	get_element(printer, id).selectedIndex = value > 0 ? 1 + value - 2 - printer.maxaxes : 0;
	return value;
} // }}}
// }}}

// Builders. {{{
function range(num, element, attr) { // {{{
	var ret = [];
	for (var i = 0; i < num + 1; ++i) {
		var node = element.cloneNode(true);
		node.AddText(String(i));
		node[attr] = String(i);
		ret.push(node);
	}
	return ret;
} // }}}

function pinrange(only_analog, element, attr) { // {{{
	var ret = [];
	var pin = 0;
	if (!only_analog) {
		for (var i = 0; i < printer.num_digital_pins; ++i) {
			var node = element.cloneNode(true);
			node.AddText('D' + String(i));
			node[attr] = String(pin);
			ret.push(node);
			pin += 1;
		}
	}
	for (var i = 0; i < printer.num_pins - printer.num_digital_pins; ++i) {
		var node = element.cloneNode(true);
		node.AddText('A' + String(i));
		node[attr] = String(pin);
		ret.push(node);
		pin += 1;
	}
	return ret;
} // }}}

function temprange(element, attr) { // {{{
	var node = element.cloneNode(true);
	node.AddText('None');
	node[attr] = '0';
	var ret = [node];
	for (var i = 0; i < printer.maxextruders; ++i) {
		node = element.cloneNode(true);
		node.AddText('Extruder ' + String(i));
		node[attr] = String(2 + printer.maxaxes + i);
		ret.push(node);
	}
	for (var i = 0; i < printer.maxtemps; ++i) {
		node = element.cloneNode(true);
		node.AddText('Temp ' + String(i));
		node[attr] = String(2 + printer.maxaxes + printer.maxextruders + i);
		ret.push(node);
	}
	return ret;
} // }}}

var choices = new Object;

function choice(obj, options, element, label, classes, containerclasses) { // {{{
	var ret = [];
	var id = make_id(printer, obj);
	choices[id] = [obj, []];
	for (var i = 0; i < options.length; ++i) {
		var e = element.cloneNode(true);
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
		var l = label.cloneNode(true);
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

function multiple_titles(modes, titles, classes, mouseovers) { // {{{
	var ret = document.createElement('tr');
	for (var cell = 0; cell < titles.length; ++cell) {
		var th = ret.AddElement('th', classes[cell]);
		th.AddText(titles[cell]);
		if (mouseovers && mouseovers[cell])
			th.title = mouseovers[cell];
	}
	visibles[port].titles.push([modes, [ret]]);
	ret.AddClass('hidden');
	return ret;
} // }}}

function multiple(template, arg) { // {{{
	var ret = [];
	var type = template.replace(/^.*_(.*?)$/, '$1');
	for (var i = 0; i < printer['max' + type2plural[type]]; ++i) {
		var part = build(template, [i, arg]);
		for (var p = 0; p < part.length; ++p) {
			if (part[p] instanceof Comment)
				continue;
			if (part[p] instanceof Text) {
				if (!/^\s*$/.exec(part[p].data))
					alert('Non-empty text node in result of multiple: ' + part[p].data);
				continue;
			}
			visibles[port][type][i].push(part[p]);
			ret.push(part[p]);
		}
	}
	return ret;
} // }}}

function floats(num, title, obj) { // {{{
	var ret = [title];
	for (var i = 0; i < num; ++i)
		ret = ret.concat(build('Float', [String(i), obj.concat([i])]));
	return ret;
} // }}}

function axis_name(index, as_axis) { // {{{
	return index < 3 ? String.fromCharCode((as_axis ? 'X' : 'U').charCodeAt(0) + index) : 'Axis ' + String(index);
} // }}}

function extruder_name(index) { // {{{
	return 'Extruder ' + String(index);
} // }}}

function temp_name(index) { // {{{
	return index == 0 ? 'Bed' : 'Temp ' + String(index);
} // }}}

function gpio_name(index) { // {{{
	return index == 0 ? 'Fan' : 'Gpio ' + String(index);
} // }}}

function make_pin_title(title, stack) { // {{{
	var t = document.createElement('th');
	t.AddText(title);
	return [t].concat(stack);
} // }}}

function make_table() { // {{{
	var t = document.createElement('table');
	for (var r = 0; r < arguments.length; ++r) {
		if (arguments[r] instanceof Element)
			t.Add(arguments[r]);
		else {
			for (var i = 0; i < arguments[r].length; ++i)
				t.Add(arguments[r][i]);
		}
	}
	return t;
} // }}}

function make_tablerow(title, cells, classes, mainclass, id) { // {{{
	var ret = document.createElement('tr');
	if (mainclass)
		ret.AddClass(mainclass);
	if (id)
		ret.id = make_id(printer, id);
	ret.AddElement('th', classes[0]).AddText(title);
	for (var cell = 0; cell < cells.length; ++cell) {
		var current_cell;
		if (!classes[1] || typeof classes[1] == 'string')
			current_cell = ret.AddElement('td', classes[1]);
		else
			current_cell = ret.AddElement('td', classes[1][cell]);
		if (id)
			current_cell.id = make_id(printer, id, cell);
		if (cells[cell] instanceof Element) {
			while (cells[cell].childNodes.length > 0) {
				var n = cells[cell].removeChild(cells[cell].childNodes[0]);
				current_cell.Add(n);
			}
		}
		else {
			for (var e = 0; e < cells[cell].length; ++e)
				current_cell.Add(cells[cell][e]);
		}
	}
	return ret;
} // }}}
// }}}

// Set helpers(to server). {{{
function set_pin(printer, id) { // {{{
	var value = get_element(printer, id).selectedIndex;
	var invalid = get_element(printer, id, 'invalid').checked;
	var inverted = get_element(printer, id, 'inverted').checked;
	set_value(printer, id, value + 0x100 * invalid + 0x200 * inverted);
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
function move_axis(axes, amounts, update_lock, pos) { // {{{
	if (pos === undefined)
		pos = [];
	if (pos.length < axes.length) {
		selected_printer.call('get_axis_pos', [axes[pos.length]], {}, function(result) {
			pos.push(result[1] + amounts[pos.length]);
			move_axis(axes, amounts, update_lock, pos);
		});
		return;
	}
	var target = new Object;
	for (var a = 0; a < axes.length; ++a)
		target[axes[a]] = pos[a];
	selected_printer.call('goto', [target], {'cb': true}, function() {
		selected_printer.call('wait_for_cb', [], {}, function() {
			update_canvas_and_spans(update_lock);
		});
	});
}
// }}}

function set_reference(x, y, ctrl) { // {{{
	selected_printer.call('get_axis_pos', [0], {}, function(current_x) {
		selected_printer.call('get_axis_pos', [1], {}, function(current_y) {
			if (ctrl) {
				selected_printer.reference = [x, y];
				update_canvas_and_spans(false);
			}
			else {
				var dx = Math.cos(selected_printer.local_angle) * (x - selected_printer.reference[0]) - Math.sin(selected_printer.local_angle) * (y - selected_printer.reference[1]);
				var dy = Math.cos(selected_printer.local_angle) * (y - selected_printer.reference[1]) + Math.sin(selected_printer.local_angle) * (x - selected_printer.reference[0]);
				selected_printer.reference = [x, y];
				selected_printer.call('goto', [[current_x[1] + dx, current_y[1] + dy]], {'cb': true}, function() {
					selected_printer.call('wait_for_cb', [], {}, function() {
						update_canvas_and_spans(false);
					});
				});
			}
		});
	});
}
// }}}

function update_canvas_and_spans(update_lock) { // {{{
	var printer = selected_printer;
	printer.call('get_axis_pos', [0], {}, function(x) {
		printer.call('get_axis_pos', [1], {}, function(y) {
			printer.call('get_axis_pos', [2], {}, function(z) {
				if (update_lock)
					printer.lock = [[x[1], y[1]], [selected_printer.reference[0], selected_printer.reference[1]]];
				var e = document.getElementById('move_span_0');
				e.ClearAll();
				e.AddText(x[1].toFixed(1));
				e = document.getElementById('move_span_1');
				e.ClearAll();
				e.AddText(y[1].toFixed(1));
				e = document.getElementById('move_span_2');
				e.ClearAll();
				e.AddText(z[1].toFixed(1));
				redraw_canvas(x[1], y[1]);
			});
		});
	});
}
// }}}

function redraw_canvas(x, y) { // {{{
	var canvas = document.getElementById('move_view');
	var c = canvas.getContext('2d');
	var box = document.getElementById('movebox');
	var extra_height = box.clientHeight - canvas.clientHeight;
	var printerwidth;
	var printerheight;
	var outline;
	switch (selected_printer.printer_type) {
	case 0:
		var xaxis = selected_printer.axis[0];
		var yaxis = selected_printer.axis[1];
		printerwidth = 2 * Math.max(xaxis.motor_max, -xaxis.motor_min) + 10;
		printerheight = 2 * Math.max(yaxis.motor_max, -yaxis.motor_min) + 10;
		outline = function(c) {
			c.beginPath();
			// Why does this not work?
			//c.rect(xaxis.axis_min, yaxis.axis_min, xaxis.axis_max - xaxis.axis_min, yaxis.axis_max - y.axis_min);
			c.moveTo(xaxis.motor_min, yaxis.motor_min);
			c.lineTo(xaxis.motor_min, yaxis.motor_max);
			c.lineTo(xaxis.motor_max, yaxis.motor_max);
			c.lineTo(xaxis.motor_max, yaxis.motor_min);
			c.lineTo(xaxis.motor_min, yaxis.motor_min);
			c.stroke();
		};
		break;
	case 1:
		var names = ['U', 'V', 'W'];
		var radius = [];
		var length = [];
		for (var a = 0; a < 3; ++a) {
			radius.push(selected_printer.axis[a].delta_radius);
			length.push(selected_printer.axis[a].delta_length);
		}
		var origin = [[radius[0], 0], [radius[1] * -.5, radius[1] * .8660254037844387], [radius[2] * -.5, radius[2] * -.8660254037844387]];
		var dx = [0, -.8660254037844387, .8660254037844387];
		var dy = [1, -.5, -.5];
		var intersects = [];
		var intersect = function(x0, y0, r, x1, y1, dx1, dy1, positive) {
			// Find intersection of circle(x-x0)^2+(y-y0)^2==r^2 and line l=(x1,y1)+t(dx1,dy1); use positive of negative solution for t.
			// Return coordinate.
			var s = positive ? 1 : -1;
			var k = (dx1 * (x1 - x0) + dy1 * (y1 - y0)) / (dx1 * dx1 + dy1 * dy1);
			var t = s * Math.sqrt(r * r - (x1 - x0) * (x1 - x0) - (y1 - y0) * (y1 - y0) + k * k) - k;
			return [x1 + t * dx1, y1 + t * dy1];
		}
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
			c.rotate(-selected_printer.angle * Math.PI / 180);
			c.beginPath();
			c.moveTo(intersects[0][0][0], intersects[0][0][1]);
			for (var a = 0; a < 3; ++a) {
				var A = (a + 1) % 3;
				var B = (a + 2) % 3;
				c.lineTo(intersects[a][1][0], intersects[a][1][1]);
				c.arc(origin[B][0], origin[B][1], length[B], angles[B][1], angles[B][0], false);
			}
			c.stroke();
			for (var a = 0; a < 3; ++a) {
				var w = c.measureText(names[a]).width;
				c.save();
				c.translate(origin[a][0] + dy[a] * 15 - w / 2, origin[a][1] - dx[a] * 15);
				c.rotate(selected_printer.angle * Math.PI / 180);
				c.scale(1, -1);
				c.strokeText(names[a], 0, 0);
				c.restore();
			}
			c.restore();
		};
		var extra = c.measureText(names[0]).width + 20;
		printerwidth = 2 * (maxx + extra);
		printerheight = 2 * (maxy + extra);
		break;
	}
	var r = Math.sqrt(printerwidth * printerwidth + printerheight * printerheight);
	var factor = Math.min(window.innerWidth, window.innerHeight) / r;
	factor /= 2;
	var size = r * factor + 5;
	canvas.height = size;
	canvas.width = size;
	box.style.marginTop = String((window.innerHeight - canvas.height - extra_height) / 2) + 'px';

	var r = selected_printer.reference;
	var b = selected_printer.bbox;
	var l = selected_printer.lock;
	var true_pos = [x, y];

	if (l !== null && l[1] !== r) {
		var real = [l[0][0] - x, l[0][1] - y];
		var target = [l[1][0] - r[0], l[1][1] - r[1]];
		selected_printer.local_angle = Math.atan2(real[1], real[0]) - Math.atan2(target[1], target[0]);
	}

	c.save();
	// Clear canvas.
	c.clearRect(0, 0, canvas.width, canvas.height);

	c.translate(canvas.width / 2, canvas.height / 2);
	c.scale(factor, -factor);

	// Draw outline.
	c.strokeStyle = '#888';
	c.fillStyle = '#888';
	outline(c);
	// Draw center.
	c.beginPath();
	c.moveTo(1, 0);
	c.arc(0, 0, 1, 0, 2 * Math.PI);
	c.fillStyle = '#888';
	c.fill();

	// Draw current location.
	c.beginPath();
	c.fillStyle = '#44f';
	if (l === null) {
		c.moveTo(true_pos[0] + 3, true_pos[1]);
		c.arc(true_pos[0], true_pos[1], 3, 0, 2 * Math.PI);
		c.fill();
	}
	else {
		c.rect(true_pos[0] - 3, true_pos[1] - 3, 6, 6);
		c.fill();
		c.beginPath();
		c.moveTo(l[0][0] - 5, l[0][1] - 5);
		c.lineTo(l[0][0] + 5, l[0][1] + 5);
		c.moveTo(l[0][0] - 5, l[0][1] + 5);
		c.lineTo(l[0][0] + 5, l[0][1] - 5);
		c.strokeStyle = '#44f';
		c.stroke();
	}


	c.save();
	c.translate(x, y);
	c.rotate(selected_printer.local_angle);
	c.translate(-r[0], -r[1]);

	c.beginPath();
	// Draw print bounding box.
	c.rect(b[0][0], b[1][0], b[0][1] - b[0][0], b[1][1] - b[1][0]);

	// Draw tick marks.
	c.moveTo((b[0][1] + b[0][0]) / 2, b[1][0]);
	c.lineTo((b[0][1] + b[0][0]) / 2, b[1][0] + 5);

	c.moveTo((b[0][1] + b[0][0]) / 2, b[1][1]);
	c.lineTo((b[0][1] + b[0][0]) / 2, b[1][1] - 5);

	c.moveTo(b[0][0], (b[1][1] + b[1][0]) / 2);
	c.lineTo(b[0][0] + 5, (b[1][1] + b[1][0]) / 2);

	c.moveTo(b[0][1], (b[1][1] + b[1][0]) / 2);
	c.lineTo(b[0][1] - 5, (b[1][1] + b[1][0]) / 2);

	// Draw central cross.
	c.moveTo((b[0][1] + b[0][0]) / 2 - 5, (b[1][1] + b[1][0]) / 2);
	c.lineTo((b[0][1] + b[0][0]) / 2 + 5, (b[1][1] + b[1][0]) / 2);
	c.moveTo((b[0][1] + b[0][0]) / 2, (b[1][1] + b[1][0]) / 2 + 5);
	c.lineTo((b[0][1] + b[0][0]) / 2, (b[1][1] + b[1][0]) / 2 - 5);

	// Draw zero.
	c.moveTo(3, 0);
	c.arc(0, 0, 3, 0, 2 * Math.PI);

	// Draw lock.
	if (l !== null) {
		// Draw the reference point.
		c.rect(l[1][0] - 5, l[1][1] - 5, 10, 10);
	}

	// Update it on screen.
	c.strokeStyle = '#000';
	c.stroke();

	c.restore();
	c.restore();
}
// }}}

function key_move(key, shift, ctrl) { // {{{
	var value = Number(document.getElementById('move_amount').value);
	if (shift)
		value /= 10.;
	var spanx = document.getElementById('move_span_0');
	var spany = document.getElementById('move_span_1');
	var spanz = document.getElementById('move_span_2');
	// 10: enter
	if (key == 13) {
		if (selected_printer.lock !== null) {
			selected_printer.lock = null;
			update_canvas_and_spans(false);
		}
		else {
			update_canvas_and_spans(true);
		}
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
	// numeric 0: reference point is 0,0.
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
		set_reference(selected_printer.bbox[0][1], selected_printer.bbox[1][1], ctrl);
	else {
		//dbg('other key:' + key);
		return true;
	}
	return false;
} // }}}

function start_move() { // {{{
	// Update bbox.
	var q = document.getElementById('queue');
	selected_printer.bbox = [[0, 0], [0, 0]];
	for (var e = 0; e < q.options.length; ++e) {
		if (!q.options[e].selected)
			continue;
		var name = q.options[e].value;
		var item;
		for (item = 0; item < queue.length; ++item)
			if (queue[item][0] == name)
				break;
		if (item >= queue.length)
			continue;
		if (queue[item][1][0][0] < selected_printer.bbox[0][0])
			selected_printer.bbox[0][0] = queue[item][1][0][0];
		if (queue[item][1][0][1] > selected_printer.bbox[0][1])
			selected_printer.bbox[0][1] = queue[item][1][0][1];
		if (queue[item][1][1][0] < selected_printer.bbox[1][0])
			selected_printer.bbox[1][0] = queue[item][1][1][0];
		if (queue[item][1][1][1] > selected_printer.bbox[1][1])
			selected_printer.bbox[1][1] = queue[item][1][1][1];
	}
	selected_printer.lock = null;
	update_canvas_and_spans(false);
	switch_show(true, 'mover');
	document.getElementById('move_amount').focus();
} // }}}

function reset_position() { // {{{
	selected_printer.lock = null;
	selected_printer.local_angle = 0;
	selected_printer.reference = [0, 0];
	update_canvas_and_spans(false);
} // }}}

// }}}

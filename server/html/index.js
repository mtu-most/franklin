// vim: set foldmethod=marker :
var ports;
var labels, printers;
// General supporting functions.
function get_elements (list) { // {{{
	var ret = [];
	for (var i = 0; i < list.length; ++i) {
		if (!(list[i] instanceof Element))
			continue;
		ret.push (list[i]);
	}
	return ret;
} // }}}

function init () { // {{{
	ports = new Object;
	labels = document.getElementById ('labels');
	printers = document.getElementById ('printers');
	setup ();
	register_update ('new_port', new_port);
	register_update ('new_printer', new_printer);
	register_update ('del_printer', del_printer);
	register_update ('del_port', del_port);
	register_update ('variables_update', update_variables);
	register_update ('axis_update', update_axis);
	register_update ('extruder_update', update_extruder);
	register_update ('temp_update', update_temp);
	register_update ('gpio_update', update_gpio);
} // }}}

function make_id (printer, id, extra) { // {{{
	var ret = printer.port.replace (/[^a-zA-Z0-9_]/g, '');
	if (id[0] !== null) {
		if (typeof id[0][0] == 'string')
			ret += '_' + id[0][0];
		else {
			for (var i = 0; i < id[0][0].length; ++i)
				ret += '_' + id[0][0][i];
		}
		ret += '_' + String (id[0][1]);
	}
	ret += '_' + id[1];
	if (id.length > 2 && typeof id[2] == 'number')
		ret += '_' + String (id[2]);
	if (extra !== null && extra !== undefined)
		ret += '_' + String (extra);
	return ret;
} // }}}

function set_value (printer, id, value, reply) { // {{{
	var target;
	if (id.length == 2) {
		if (id[0] === null)
			printer.call ('set_' + id[1], [value], {}, reply);
		else if (typeof id[0][0] == 'string')
			printer.call (id[0][0] + '_set_' + id[1], [id[0][1], value], {}, reply);
		else {
			var prefix = '';
			for (var i = 0; i < id[0][0].length; ++i)
				prefix += id[0][0][i] + '_';
			printer.call (prefix + 'set_' + id[1], [id[0][1], value], {}, reply);
		}
	}
	else {
		if (typeof id[2] == 'number') {
			printer.call (id[0][0] + '_set_' + id[1], [id[0][1], id[2], value], {}, reply);
		}
		else {
			if (id[0] === null)
				printer.call (id[2], [value], {}, reply);
			else
				printer.call (id[2] + '_' + id[1], [id[0][1], value], {}, reply);
		}
	}
} // }}}

function get_value (printer, id) { // {{{
	if (id[0] === null)
		return printer[id[1]];
	if (typeof id[0][0] == 'string')
		return printer[id[0][0]][id[0][1]][id[1]];
	var obj = printer[id[0][0][0]][id[0][1]];
	for (var i = 1; i < id[0][0].length; ++i)
		obj = obj[id[0][0][i]];
	return obj[id[1]];
} // }}}

function get_element (printer, id, extra) { // {{{
	return document.getElementById (make_id (printer, id, extra));
} // }}}

// Non-update events.
function new_port (port) { // {{{
	ports[port] = [get_elements (build ('Label', [port]))[0], null, ''];
	labels.Add (ports[port][0]);
} // }}}

function new_printer () { // {{{
	ports[port][1] = get_elements (build ('Printer', [port]))[0];
	printers.Add (ports[port][1]);
} // }}}

function del_printer () { // {{{
	printers.removeChild (ports[port][1]);
	ports[port][1] = null;
	ports[port][2] = '';
} // }}}

function del_port () { // {{{
	labels.removeChild (ports[port][0]);
} // }}}

// Update events (from server).
function update_variables () { // {{{
	if (!get_element (printer, [null, 'name']))
		return;
	if (ports[port][2] != printer.name) {
		labels.removeChild (ports[port][0]);
		ports[port][0] = get_elements (build ('Label', [port]))[0];
		labels.Add (ports[port][0]);
		ports[port][2] = printer.name;
	}
	update_text ([null, 'name']);
	document.getElementById ('export').href = printer.name + '.ini?port=' + encodeURIComponent (port);
	update_range ([null, 'num_axes']);
	update_range ([null, 'num_extruders']);
	update_range ([null, 'num_temps']);
	update_range ([null, 'num_gpios']);
	update_choice ([null, 'printer_type']);
	update_pin ([null, 'led_pin']);
	update_float ([null, 'room_T']);
	update_float ([null, 'motor_limit']);
	update_float ([null, 'temp_limit']);
	update_float ([null, 'feedrate']);
	update_checkbox ([null, 'paused', 'pause']);
} // }}}

function update_axis (index) { // {{{
	if (!get_element (printer, [null, 'name']))
		return;
	update_motor ([['axis', 'motor'], index]);
	update_pin ([['axis', index], 'limit_min_pin']);
	update_pin ([['axis', index], 'limit_max_pin']);
	update_pin ([['axis', index], 'sense_pin']);
	update_float ([['axis', index], 'limit_min_pos']);
	update_float ([['axis', index], 'limit_max_pos']);
	update_float ([['axis', index], 'delta_length']);
	update_float ([['axis', index], 'delta_radius']);
	update_float ([['axis', index], 'offset']);
	update_floats ([['axis', index], 'num_displacements']);
	update_float ([['axis', index], 'first_displacement']);
	update_float ([['axis', index], 'displacement_step']);
} // }}}

function update_extruder (index) { // {{{
	if (!get_element (printer, [null, 'name']))
		return;
	update_motor ([['extruder', 'motor'], index]);
	update_tempcontent ([['extruder', 'temp'], index]);
	update_float ([['extruder', index], 'filament_heat']);
	update_float ([['extruder', index], 'nozzle_size']);
	update_float ([['extruder', index], 'filament_size']);
} // }}}

function update_temp (index) { // {{{
	if (!get_element (printer, [null, 'name']))
		return;
	update_tempcontent (['temp', index]);
} // }}}

function update_gpio (index) { // {{{
	if (!get_element (printer, [null, 'name']))
		return;
	update_pin ([['gpio', index], 'pin']);
	update_choice ([['gpio', index], 'state']);
	// TODO: master
	update_float ([['gpio', index], 'value']);
} // }}}

// Update helpers.
function update_motor (id) { // {{{
	update_pin ([id, 'step_pin']);
	update_pin ([id, 'dir_pin']);
	update_pin ([id, 'enable_pin']);
	update_float ([id, 'steps_per_mm']);
	update_float ([id, 'max_v_pos']);
	update_float ([id, 'max_v_neg']);
	update_float ([id, 'max_a']);
	//update_float ([id, 'runspeed']);
	//update_float ([id, 'sleeping']);
} // }}}

function update_tempcontent (id) { // {{{
	update_pin ([id, 'power_pin']);
	update_pin ([id, 'thermistor_pin']);
	update_float ([id, 'R0']);
	update_float ([id, 'R1']);
	update_float ([id, 'Rc']);
	update_float ([id, 'Tc']);
	update_float ([id, 'beta']);
	update_float ([id, 'core_C']);
	update_float ([id, 'shell_C']);
	update_float ([id, 'transfer']);
	update_float ([id, 'radiation']);
	update_float ([id, 'power']);
	update_float ([id, 'value', 'settemp']);
} // }}}

function update_text (id) { // {{{
	get_element (printer, id).value = get_value (printer, id);
} // }}}

function update_range (id) { // {{{
	get_element (printer, id).selectedIndex = get_value (printer, id);
} // }}}

function update_choice (id) { // {{{
	var value = get_value (printer, id);
	var list = choices[make_id (printer, id)][1];
	if (value < list.length)
		list[value].checked = true;
	else {
		for (var i = 0; i < list.length; ++i)
			list[i].checked = false;
	}
} // }}}

function update_checkbox (id) { // {{{
	get_element (printer, id).checked = get_value (printer, id);
} // }}}

function update_pin (id) { // {{{
	get_element (printer, id).selectedIndex = get_value (printer, id) & 0xff;
	get_element (printer, id, 'invalid').checked = Boolean (get_value (printer, id) & 0x100);
	get_element (printer, id, 'inverted').checked = Boolean (get_value (printer, id) & 0x200);
} // }}}

function update_float (id) { // {{{
	var e = get_element (printer, id);
	e.value = String (get_value (printer, id) / e.factor);
} // }}}

function update_floats (id) { // {{{
	var value = get_value (printer, id);
	for (var i = 0; i < value.length; ++i) {
		var e = get_element (printer, id.concat ([i]));
		e.value = String (value[i]);
	}
} // }}}

// Builders.
function range (num, element, attr) { // {{{
	var ret = [];
	for (var i = 0; i < num + 1; ++i) {
		var node = element.cloneNode (true);
		node.AddText (String (i));
		node[attr] = String (i);
		ret.push (node);
	}
	return ret;
} // }}}

function pinrange (only_analog, element, attr) { // {{{
	var ret = [];
	var pin = 0;
	if (!only_analog) {
		for (var i = 0; i < printer.num_digital_pins; ++i) {
			var node = element.cloneNode (true);
			node.AddText ('D' + String (i));
			node[attr] = String (pin);
			ret.push (node);
			pin += 1;
		}
	}
	for (var i = 0; i < printer.num_pins - printer.num_digital_pins; ++i) {
		var node = element.cloneNode (true);
		node.AddText ('A' + String (i));
		node[attr] = String (pin);
		ret.push (node);
		pin += 1;
	}
	return ret;
} // }}}

function temprange (element, obj, attr) { // {{{
	var ret = [];
	for (var i = 0; i < printer.maxextruders; ++i) {
		var node = element.cloneNode (true);
		node.AddText ('Extruder ' + String (i));
		node[attr] = String (2 + printer.maxaxes + i);
		ret.push (node);
	}
	for (var i = 0; i < printer.maxtemps; ++i) {
		var node = element.cloneNode (true);
		node.AddText ('Temp ' + String (i));
		node[attr] = String (2 + printer.maxaxes + printer.maxextruders + i);
		ret.push (node);
	}
	return ret;
} // }}}

var choices = new Object;

function choice (obj, options, element) { // {{{
	var ret = [];
	var id = make_id (printer, obj);
	choices[id] = [obj, []];
	for (var i = 0; i < options.length; ++i) {
		var e = element.cloneNode (true);
		choices[id][1].push (e);
		e.name = id;
		e.value = String (i);
		e.printer = printer;
		e.addEventListener('click', function () { set_value (this.printer, choices[this.name][0], Number (this.value)); }, false);
		ret.push (e, options[i]);
	}
	return ret;
} // }}}

function multiple (template, max, attr) { // {{{
	var ret = [];
	for (var i = 0; i < max; ++i)
		ret = ret.concat (build (template, [i, i < printer[attr]]));
	return ret;
} // }}}

function floats (num, title, obj) { // {{{
	var ret = [title];
	for (var i = 0; i < num; ++i)
		ret = ret.concat (build ('Float', [String (i), obj.concat ([i])]));
	return ret;
} // }}}

function axis_name (index) { // {{{
	return 'Axis ' + String (index) + (index < 3 ? ': ' + String.fromCharCode ('X'.charCodeAt (0) + index) : '');
} // }}}

// Set helpers (to server).
function set_pin (printer, id) { // {{{
	var value = get_element (printer, id).selectedIndex;
	var invalid = get_element (printer, id, 'invalid').checked;
	var invalid = get_element (printer, id, 'inverted').checked;
	set_value (printer, id, value + 0x100 * invalid + 0x200 * inverted);
} // }}}

function set_file (printer, id) { // {{{
	var element = get_element (printer, id);
	if (element.files.length < 1) {
		alert ('please select a file');
		return;
	}
	var reader = new FileReader ();
	reader.name = element.files[0].filename;
	reader.onloadend = function (e) {
		function errors (err) {
			if (err.length == 0) {
				alert ('ok!');
				return;
			}
			alert ('Errors in lines: ' + err.join (', '));
		}
		if (this.readyState != FileReader.DONE)
			return;
		set_value (printer, id, this.result, errors);
	}
	reader.readAsBinaryString (element.files[0]);
} // }}}

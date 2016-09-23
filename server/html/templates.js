/* templates.js - widgets for printer components for Franklin
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

// Primitives. {{{
function Button(printer, title, action, className) { // {{{
	var ret = Create('button', 'wide title');
	ret.AddClass(className);
	ret.AddText(title);
	ret.AddEvent('click', function() { this.printer.call(action, [], {}) });
	ret.type = 'button';
	ret.printer = printer.printer;
	return ret;
} // }}}

function Text(printer, title, obj, className) { // {{{
	var span = Create('span', 'blocktitle').AddClass(className);
	span.AddText(title);
	var input = Create('input', 'text').AddClass(className);
	input.type = 'text';
	input.printer = printer;
	input.AddEvent('keydown', function(event) {
		if (event.keyCode == 13) {
			set_value(this.printer, obj, this.value);
			event.preventDefault();
		}
	});
	return [span, input];
} // }}}

function Name(printer, type, num) { // {{{
	if (num === null || (typeof num != 'number' && num.length >= 2 && num[1] === null))
		return '';
	var ret = Create('input', 'editname');
	ret.type = 'text';
	ret.printer = printer;
	ret.AddEvent('keydown', function(event) {
		if (event.keyCode == 13) {
			if (typeof num != 'number' && num.length == 0)
				set_value(this.printer, [null, type + '_name'], this.value);
			else
				set_value(this.printer, [[type, num], 'name'], this.value);
			event.preventDefault();
		}
	});
	return ret;
} // }}}

function Pin(printer, title, obj, type) { // {{{
	var pinselect = Create('select', 'pinselect');
	pinselect.id = make_id(printer, obj);
	pinselect.obj = obj;
	pinselect.printer = printer;
	pinrange(printer, type, pinselect);
	pinselect.can_invert = Boolean(type & 7);
	var validlabel = Create('label');
	var validinput = validlabel.AddElement('input');
	validlabel.AddText('Valid');
	validinput.type = 'checkbox';
	validinput.id = make_id(printer, obj, 'valid');
	var inverts;
	if (pinselect.can_invert) {
		var invertedlabel = Create('label');
		var invertedinput = invertedlabel.AddElement('input');
		invertedlabel.AddText('Inverted');
		invertedinput.type = 'checkbox';
		invertedinput.id = make_id(printer, obj, 'inverted');
		inverts = [invertedlabel];
	}
	else
		inverts = [];
	var button = Create('button', 'button');
	button.type = 'button';
	button.AddText('Set');
	button.printer = printer;
	button.AddEvent('click', function() { set_pin(this.printer, obj); });
	return make_tablerow(printer, title, [[pinselect], [validlabel], inverts, [button]], ['pintitle', ['pinvalue', 'pinvalue', 'pinvalue', 'pinvalue']]);
} // }}}

function Float(printer, obj, digits, factor, className, set) { // {{{
	var input = Create('input', className);
	var span = Create('span', className);
	input.obj = obj;
	if (factor === undefined)
		factor = 1;
	input.factor = factor;
	span.factor = factor;
	input.id = make_id(printer, obj, 'new');
	span.id = make_id(printer, obj);
	span.digits = digits;
	input.type = 'text';
	input.set = set;
	input.printer = printer;
	input.AddEvent('keydown', function(event) { floatkey(event, this); });
	return [input, /*button,*/ span];
} // }}}

function File(printer, obj, action, buttontext, types, cb) { // {{{
	var input = Create('input');
	input.type = 'file';
	input.accept = types;
	input.id = make_id(printer, obj);
	var button = Create('button', 'button').AddText(buttontext);
	button.type = 'button';
	button.source = obj;
	button.action = action;
	button.extra = cb;
	button.printer = printer;
	button.AddEvent('click', function() { set_file(this.printer, this.source, this.action); if (this.extra !== undefined) this.extra(); });
	return [input, button];
} // }}}

function Checkbox(printer, obj) { // {{{
	var ret = Create('input');
	ret.type = 'checkbox';
	ret.id = make_id(printer, obj);
	ret.obj = obj;
	ret.printer = printer;
	ret.AddEvent('click', function(e) {
		e.preventDefault();
		set_value(this.printer, this.obj, this.checked);
		return false;
	});
	return ret;
} // }}}

function Str(printer, obj) { // {{{
	var ret = Create('span');
	var input = ret.AddElement('input');
	input.type = 'text';
	input.AddEvent('keydown', function(event) {
		if (event.keyCode == 13) {
			set_value(ret.printer, ret.obj, input.value);
			event.preventDefault();
		}
	});
	ret.obj = obj;
	ret.printer = printer;
	/*var e = ret.AddElement('button');
	e.type = 'button';
	e.AddText('Set');
	e.AddEvent('click', function(event) {
		set_value(ret.printer, ret.obj, input.value);
		return false;
	});
	e = ret.AddElement('span');
	e.id = make_id(printer, obj);
	*/
	return ret;
} // }}}

function Id(printer, obj) { // {{{
	if (obj[0][1] === null)
		return '';
	var ret = Create('input');
	ret.type = 'checkbox';
	ret.id = make_id(printer, obj);
	ret.obj = obj;
	ret.printer = printer;
	printer.idgroups[ret.obj[1]].push(ret);
	ret.AddEvent('click', function(e) {
		e.preventDefault();
		if (!ret.checked)
			ret.set_id(255);
		else
			ret.set_id(ret.obj[0][1]);
		return false;
	});
	ret.set_id = function(num) {
		set_value(this.printer, [null, this.obj[1] + '_id'], num);
	};
	return ret;
} // }}}
// }}}


// Space. {{{
function Extruder(printer, space, axis) {
	var e = ['extruder_dx', 'extruder_dy', 'extruder_dz'];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float(printer, [['axis', [space, axis]], e[i]], 1, 1e-3));
		e[i] = div;
	}
	return make_tablerow(printer, axis_name(printer, space, axis), e, ['rowtitle3'], undefined, TYPE_EXTRUDER, space);
}

function Follower(printer, space, motor) {
	var f = ['follower_space', 'follower_motor'];
	for (var i = 0; i < f.length; ++i) {
		var div = Create('div');
		div.Add(Float(printer, [['motor', [space, motor]], f[i]], 0));
		f[i] = div;
	}
	return make_tablerow(printer, motor_name(printer, space, motor), f, ['rowtitle2'], undefined, TYPE_FOLLOWER, space);
}

function Cartesian(printer, num) {
	return make_tablerow(printer, space_name(printer, num), [Float(printer, [['space', num], 'num_axes'], 0, 1)], ['rowtitle1'], undefined, [TYPE_CARTESIAN, TYPE_EXTRUDER, TYPE_FOLLOWER], num);
}

function Delta(printer, space, motor) {
	var e = [['delta_axis_min', 1], ['delta_axis_max', 1], ['delta_rodlength', 3], ['delta_radius', 3]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float(printer, [['motor', [space, motor]], e[i][0]], e[i][1], 1));
		e[i] = div;
	}
	return make_tablerow(printer, motor_name(printer, space, motor), e, ['rowtitle4'], undefined, TYPE_DELTA, space);
}

function Delta_space(printer, num) {
	var div = Create('div');
	div.Add(Float(printer, [['space', num], 'delta_angle'], 2, Math.PI / 180));
	return make_tablerow(printer, space_name(printer, num), [div], ['rowtitle1'], undefined, TYPE_DELTA, num);
}

function Polar_space(printer, num) {
	var div = Create('div');
	div.Add(Float(printer, [['space', num], 'polar_max_r'], 1, 1));
	return make_tablerow(printer, space_name(printer, num), [div], ['rowtitle1'], undefined, TYPE_POLAR, num);
}

function Axis(printer, space, axis) {
	var e = [Name(printer, 'axis', [space, axis]), ['park', 1, 1], ['park_order', 0, 1], ['min', 1, 1], ['max', 1, 1], ['home_pos2', 1, 1]];
	for (var i = 1; i < e.length; ++i) {
		var div = Create('div');
		if (space == 0)
			div.Add(Float(printer, [['axis', [space, axis]], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(printer, axis_name(printer, space, axis), e, ['rowtitle6']);
}

function Motor(printer, space, motor) {
	var e = [['steps_per_unit', 3, 1], ['home_pos', 3, 1], ['home_order', 0, 1], ['limit_v', 0, 1], ['limit_a', 1, 1]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		if (space == 0 || (space == 1 && i != 1 && i != 2) || (space == 2 && (i == 1 || i == 2)))
			div.Add(Float(printer, [['motor', [space, motor]], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(printer, motor_name(printer, space, motor), e, ['rowtitle5']);
}

function Pins_space(printer, space, motor) {
	var e = [['Step', 'step', 1], ['Dir', 'dir', 1], ['Enable', 'enable', 2], ['Min Limit', 'limit_min', 4], ['Max Limit', 'limit_max', 4]];
	for (var i = 0; i < e.length; ++i)
		e[i] = Pin(printer, e[i][0], [['motor', [space, motor]], e[i][1] + '_pin'], e[i][2]);
	return make_pin_title(printer, motor_name(printer, space, motor), e, ['rowtitle6']);
}
// }}}

// Temp. {{{
function Temp_setup(printer, num) {
	var e = [Name(printer, 'temp', num), ['fan_temp', 0, 1], Id(printer, [['temp', num], 'bed'])];
	for (var i = 1; i < e.length - 1; ++i) {
		var div = Create('div');
		div.Add(Float(printer, [['temp', num], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(printer, temp_name(printer, num), e, ['rowtitle3']);
}

function Temp_limits(printer, num) {
	var e = [['heater_limit_l', 0, 1], ['heater_limit_h', 0, 1], ['fan_limit_l', 0, 1], ['fan_limit_h', 0, 1]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float(printer, [['temp', num], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(printer, temp_name(printer, num), e, ['rowtitle4']);
}

function Temp_hardware(printer, num) {
	var e = [['R0', 1, 1e3], ['R1', 1, 1e3], ['Rc', 1, 1e3], ['Tc', 0, 1], ['beta', 0, 1], ['hold_time', 1, 1]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float(printer, [['temp', num], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(printer, temp_name(printer, num), e, ['rowtitle5']);
}

function Temp(printer, num) {
	var div = Create(div);
	div.Add(Float(printer, [['temp', num], 'value', 'settemp'], 0));
	var current = Create('div');
	current.id = make_id(printer, [['temp', num], 'temp']);
	if (num !== null)
		printer.temptargets.push(current.AddElement('span'));
	var name = temp_name(printer, num);
	if (num !== null && num < 5)
		name.AddClass('temp' + num.toFixed(0));
	return make_tablerow(printer, name, [div, current, Float(printer, [['temp', num], 'fan_duty'], 0, 1e-2)], ['rowtitle2']);
}

function Pins_temp(printer, num, dummy, table) {
	var e = [['Heater', 'heater', 2], ['Fan', 'fan', 2], ['Thermistor', 'thermistor', 8]];
	for (var i = 0; i < e.length; ++i)
		e[i] = Pin(printer, e[i][0], [['temp', num], e[i][1] + '_pin'], e[i][2]);
	return make_pin_title(printer, temp_name(printer, num), e);
}
// }}}

// Gpio. {{{
function Gpio(printer, num) {
	var reset = Create('select');
	reset.id = make_id(printer, [['gpio', num], 'reset']);
	reset.AddElement('option').AddText('Off').Value = 0;
	reset.AddElement('option').AddText('On').Value = 1;
	reset.AddElement('option').AddText('Input').Value = 2;
	reset.AddElement('option').AddText('Disabled').Value = 3;
	reset.AddEvent('change', function(e) {
		var value = reset.options[reset.selectedIndex].Value;
		set_value(printer, [['gpio', num], 'reset'], value);
		if (value >= 2)
			set_value(printer, [['gpio', num], 'state'], value);
		e.preventDefault();
		return false;
	});
	return make_tablerow(printer, gpio_name(printer, num), [Name(printer, 'gpio', num), reset, Float(printer, [['gpio', num], 'duty'], 0, 1e-2), Id(printer, [['gpio', num], 'fan']), Id(printer, [['gpio', num], 'spindle'])], ['rowtitle5']);
}

function Pins_gpio(printer, num) {
	var e = [['Pin', 'pin', 6]];
	for (var i = 0; i < e.length; ++i)
		e[i] = Pin(printer, e[i][0], [['gpio', num], e[i][1]], e[i][2]);
	return make_pin_title(printer, gpio_name(printer, num), e);
}
// }}}


function Label(printer) {	// {{{
	var ret = Create('div', 'tab noflash nodetect');
	if (selected_printer == printer.uuid)
		ret.AddClass('active');
	ret.AddEvent('click', function() { select_printer(this.printer.printer); });
	ret.printer = printer;
	ret.span = ret.AddElement('span').AddText(printer.uuid);
	return ret;
}
// }}}

// Printer parts. {{{
function Top(printer) { // {{{
	var ret = Create('div', 'top');
	// Up/remove/down. {{{
	var e = ret.AddElement('div', 'updown');
	e.AddElement('button', 'queue1').AddEvent('click', function() { queue_up(printer); }).AddText('⬆').type = 'button';
	e.AddElement('br');
	e.AddElement('button', 'queue1').AddEvent('click', function() { queue_del(printer); }).AddText('×').type = 'button';
	e.AddElement('br');
	e.AddElement('button', 'queue1').AddEvent('click', function() {queue_down(printer); }).AddText('⬇').type = 'button';
	// }}}
	// Jobs. {{{
	e = ret.AddElement('div', 'jobs').AddElement('select').AddEvent('change', function() { start_move(printer); });
	e.printer = printer;
	e.multiple = true;
	e.id = make_id(printer, [null, 'queue']);
	// }}}
	// Jobbuttons. {{{
	e = ret.AddElement('div', 'jobbuttons');
	e.Add(File(printer, [null, 'queue_add', 'queue_add'], 'queue_add', 'Add', '.gcode,.ngc,application/x-gcode', function() { return queue_deselect(printer); }));
	e.AddElement('br');
	e.Add(File(printer, [null, 'audio_add', 'audio_add'], 'audio_add', 'Add Audio', 'audio/x-wav', function() { return queue_deselect(printer); }), 'benjamin');
	e.AddElement('br');
	var b = e.AddElement('button', 'benjamin').AddText('×').AddEvent('click', function() { audio_del(this.printer); });
	b.type = 'button';
	b.printer = printer;
	e.AddElement('select', 'benjamin').id = make_id(printer, [null, 'audio']);
	var b = e.AddElement('button', 'benjamin').AddText('Play').AddEvent('click', function() { audio_play(this.printer); });
	b.type = 'button';
	b.printer = printer;
	e.AddElement('br');
	b = e.AddElement('button', 'jobbutton').AddEvent('click', function() { queue_print(this.printer); }).AddText('Print selected');
	b.type = 'button';
	b.printer = printer;
	var l = e.AddElement('label');
	b = l.AddElement('input', 'jobbutton');
	l.AddText('Probe');
	var id = make_id(printer, [null, 'probebox']);
	b.id = id;
	b.type = 'checkbox';
	b.htmlFor = id;
	// }}}
	// Stop buttons. {{{
	e = ret.AddElement('div', 'stop');
	b = e.AddElement('button', 'abort').AddText('Abort').AddEvent('click', function() { this.printer.printer.call('abort', [], {}); });
	b.type = 'button';
	b.printer = printer;
	e.AddElement('br');
	b = e.AddElement('button').AddText('Home').AddEvent('click', function() { this.printer.printer.call('home', [], {}, function() { update_canvas_and_spans(b.printer); }); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Pause').AddEvent('click', function() { this.printer.printer.call('pause', [true], {}, function() { update_canvas_and_spans(b.printer); }); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Resume').AddEvent('click', function() { this.printer.printer.call('pause', [false], {}); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Sleep').AddEvent('click', function() { this.printer.printer.call('sleep', [], {}, function() { update_canvas_and_spans(b.printer); }); });
	b.type = 'button';
	b.printer = printer;
	// }}}
	return ret;
}
// }}}

function Map(printer) { // {{{
	var ret = Create('div', 'map');
	ret.id = make_id(printer, [null, 'map']);
	// Current position buttons.
	var t = ret.Add(make_table(printer).AddMultipleTitles([
		'',
		[add_name(printer, 'axis', 0, 0), ' (', add_name(printer, 'unit', 0, 0), ')'],
		[add_name(printer, 'axis', 0, 1), ' (', add_name(printer, 'unit', 0, 0), ')'],
		[add_name(printer, 'axis', 0, 2), ' (', add_name(printer, 'unit', 0, 0), ')'],
		'',
		'',
		''
	], ['', '', '', '', '', ''], null), 'maptable');
	var b = Create('button').AddText('Park').AddEvent('click', function() { this.printer.printer.call('park', [], {}, function() { update_canvas_and_spans(b.printer); }); });
	b.type = 'button';
	b.printer = printer;
	t.Add(make_tablerow(printer, add_name(printer, 'space', 0, 0), [
		Float(printer, [['axis', [0, 0]], 'current'], 2, 1, '', function(v) { b.printer.printer.call('line_cb', [[{0: v}]], {}); b.printer.printer.call('wait_for_cb', [], {}, function() { update_canvas_and_spans(b.printer); }); }),
		Float(printer, [['axis', [0, 1]], 'current'], 2, 1, '', function(v) { b.printer.printer.call('line_cb', [[{1: v}]], {}); b.printer.printer.call('wait_for_cb', [], {}, function() { update_canvas_and_spans(b.printer); }); }),
		Float(printer, [['axis', [0, 2]], 'current'], 2, 1, '', function(v) { b.printer.printer.call('line_cb', [[{2: v}]], {}); b.printer.printer.call('wait_for_cb', [], {}, function() { update_canvas_and_spans(b.printer); }); }),
		b
	], ['', '', '', '', '', '']));
	// Target position buttons.
	var b = Create('button').AddText('Use Current').AddEvent('click', function() {
		b.printer.printer.call('set_globals', [], {'targetx': b.printer.spaces[0].axis[0].current, 'targety': b.printer.spaces[0].axis[1].current});
	});
	b.printer = printer;
	b.type = 'button';
	t.Add(make_tablerow(printer, 'Target:', [
		Float(printer, [null, 'targetx'], 2, 1),
		Float(printer, [null, 'targety'], 2, 1),
		Float(printer, [null, 'zoffset'], 2, 1),
		b,
		['Angle:', Float(printer, [null, 'targetangle'], 1, Math.PI / 180, '', function(v) { update_angle(b.printer, v); }), '°']
	], ['', '', '', '', '', '']));
	// Canvas for xy and for z.
	var c = ret.AddElement('canvas', 'xymap');
	c.AddEvent('mousemove', function(e) { return xymove(b.printer, e); }).AddEvent('mousedown', function(e) { return xydown(b.printer, e); }).AddEvent('mouseup', function(e) { return xyup(b.printer, e); });
	c.id = make_id(printer, [null, 'xymap']);
	c.printer = printer;
	c = ret.AddElement('canvas', 'zmap');
	c.AddEvent('mousemove', function(e) { return zmove(b.printer, e); }).AddEvent('mousedown', function(e) { return zdown(b.printer, e); }).AddEvent('mouseup', function(e) { return zup(b.printer, e); });
	c.id = make_id(printer, [null, 'zmap']);
	c.printer = printer;
	return ret;
}
// }}}

function Temps(printer) { // {{{
	var ret = Create('div', 'temp');
	ret.Add(make_table(printer).AddMultipleTitles([
		'Temp control',
		'Target (°C)',
		'Current (°C)',
		'Fan Power (%)'
	], [
		'htitle6',
		'title6',
		'title6',
		'title6'
	], [
		null,
		'Temperature target.  Set to NaN to disable the heater completely.',
		'Actual temperature from sensor.',
		'Fraction of time that fan is enabled when on.'
	]).AddMultiple(printer, 'temp', Temp));
	ret.AddElement('canvas', 'tempgraph').id = make_id(printer, [null, 'tempgraph']);
	return ret;
}
// }}}

function Multipliers(printer) { // {{{
	var ret = Create('div', 'multipliers');
	var e = ret.AddElement('div').AddText('Feedrate: ');
	e.Add(Float(printer, [null, 'feedrate'], 0, 1e-2));
	e.AddText(' %');
	ret.AddMultiple(printer, 'axis', function(printer, space, axis, obj) {
		if (space != 1)
			return null;
		var e = Create('div');
		e.Add(axis_name(printer, space, axis));
		e.printer = printer;
		e.Add(Float(printer, [['axis', [space, axis]], 'multiplier'], 0, 1e-2));
		e.AddText(' %');
		e.Add(Float(printer, [['axis', [space, axis]], 'current'], 1, 1, '', function(v) {
			var obj = {};
			obj[space] = {};
			obj[space][axis] = v;
			e.printer.printer.call('line_cb', [obj], {}, function() {
				e.printer.printer.call('wait_for_cb', [], {}, function() { update_canvas_and_spans(e.printer); });
			});
		}));
		e.AddText(' ').Add(add_name(printer, 'unit', 0, 0));
		return e;
	}, true);
	e = ret.AddElement('div', 'admin');
	e.AddElement('Label').AddText('Store adc readings').Add(Checkbox(printer, [null, 'store_adc']));
	e.AddElement('a').AddText('Get stored readings').href = 'adc';
	return ret;
}
// }}}

function Gpios(printer) { // {{{
	var ret = Create('div', 'gpios');
	ret.AddMultiple(printer, 'gpio', function(printer, i) {
		var ret = Create('span');
		ret.id = make_id(printer, [['gpio', i], 'statespan']);
		var label = ret.AddElement('label');
		var input = label.AddElement('input');
		label.Add(gpio_name(printer, i));
		var index = i;
		input.AddEvent('click', function(e) {
			set_value(printer, [['gpio', index], 'state'], input.checked ? 1 : 0);
			e.preventDefault();
			return false;
		});
		input.type = 'checkbox';
		input.id = make_id(printer, [['gpio', i], 'state']);
		return ret;
	}, false);
	return ret;
}
// }}}
// }}}

function Printer(printer) {	// {{{
	var ret = Create('div', 'printer hidden');
	ret.printer = printer;
	ret.names = {space: [], axis: [], motor: [], temp: [], gpio: [], unit: []};
	ret.name_values = {space: [], axis: [], motor: [], temp: [], gpio: [], unit: []};
	ret.pinranges = [];
	ret.temphistory = [];
	ret.targetangle = 0;
	ret.nextangle = null;
	ret.disabling = false;
	ret.temptargets = [];
	ret.hidetypes = [];
	ret.tables = [];
	ret.idgroups = {bed: [], fan: [], spindle: []};
	ret.multiples = {space: [], temp: [], gpio: [], axis: [], motor: []};
	// Blocker bar. {{{
	ret.id = make_id(ret, [null, 'container']);
	var blocker = ret.AddElement('div', 'hidden blocker');
	blocker.id = make_id(ret, [null, 'block1']);
	// }}}
	ret.AddElement('div', 'message hidden').id = make_id(ret, [null, 'message1']);
	ret.AddElement('div', 'message').id = make_id(ret, [null, 'printstate']);
	ret.AddElement('div', 'message hidden').id = make_id(ret, [null, 'confirm']);
	ret.AddElement('h2', 'notconnected').AddText('This printer is not connected');
	var selector = ret.AddElement('div').AddText('Profile:').AddElement('select').AddEvent('change', function() {
		this.printer.call('load', [selector.value], {});
	});
	selector.printer = ret;
	selector.id = make_id(ret, [null, 'profiles']);
	update_profiles(ret);
	/*var span = ret.AddElement('span', 'port setup').AddText('@' + port);
	span.AddElement('span', 'ifflash').AddText('[!]');
	span.AddElement('span', 'ifdetect').AddText('[?]');
	*/
	// Setup. {{{
	var setup = ret.AddElement('div', 'setup expert');
	var e = setup.AddElement('div').AddText('Printer UUID:');
	ret.uuid = e.AddElement('span');
	var e = setup.AddElement('div', 'admin').AddText('Printer name:');
	e.Add(Str(ret, [null, 'name']));
	var connected = setup.AddElement('div', 'connected');
	var disable = connected.AddElement('div').AddElement('button').AddText('Disable Printer');
	disable.printer = printer;
	disable.type = 'button';
	disable.AddEvent('click', function() { this.printer.disabling = true; rpc.call('disable', [this.printer.uuid], {}); });
	var remove = setup.AddElement('div', 'admin').AddElement('button').AddText('Remove Printer');
	remove.printer = printer;
	remove.type = 'button';
	remove.AddEvent('click', function() { if (confirm('Do you really want to permanently remove all data about ' + printer.name + '?')) { remove.printer.disabling = true; rpc.call('remove_printer', [remove.printer.uuid], {}); }});
	var notconnected = setup.AddElement('div', 'notconnected');
	var ports = notconnected.AddElement('select');
	ports.id = make_id(ret, [null, 'ports']);
	ports.AddEvent('changed', function() { update_firmwares(ports, ret.firmwares); });
	var b = notconnected.AddElement('button').AddText('Detect');
	b.type = 'button';
	b.AddEvent('click', function() { detect(ports); });
	ret.firmwares = notconnected.AddElement('select');
	b = notconnected.AddElement('button').AddText('Upload');
	b.type = 'button';
	b.AddEvent('click', function() { upload(ports, ret.firmwares); });
	// Save and restore. {{{
	e = setup.AddElement('div', 'admin');
	e.AddText('Profile');
	b = e.AddElement('button').AddText('Save (as)').AddEvent('click', function() {
		this.printer.call('save', [this.saveas.value], {});
	});
	b.type = 'button';
	b.printer = printer;
	b.saveas = e.AddElement('input');
	b.saveas.type = 'text';
	e = setup.AddElement('button', 'admin').AddText('Remove this profile');
	e.type = 'button';
	e.printer = printer;
	e.AddEvent('click', function() {
		this.printer.call('remove_profile', [this.printer.profile], {});
	});
	e = setup.AddElement('button', 'admin').AddText('Set as default profile');
	e.type = 'button';
	e.printer = printer;
	e.AddEvent('click', function() {
		this.printer.call('set_default_profile', [this.printer.profile], {});
	});
	e = setup.AddElement('button').AddText('Reload this profile');
	e.type = 'button';
	e.printer = printer;
	e.AddEvent('click', function() {
		this.printer.call('load', [this.printer.profile], {});
	});
	setup.AddElement('div').Add(File(ret, [null, 'import', 'import_settings'], 'import', 'Import', '.ini'));
	e = setup.AddElement('a', 'title').AddText('Export settings to file');
	e.id = make_id(ret, [null, 'export']);
	e.title = 'Save settings to disk.';
	// }}}
	e = setup.AddElement('div').AddText('Timeout:');
	e.Add(Float(ret, [null, 'timeout'], 0, 60));
	e.AddText(' min');
	e = setup.AddElement('div').AddText('After Print:');
	var l = e.AddElement('label');
	l.Add(Checkbox(ret, [null, 'park_after_print']));
	l.AddText('Park');
	l = e.AddElement('label');
	l.Add(Checkbox(ret, [null, 'sleep_after_print']));
	l.AddText('Sleep');
	l = e.AddElement('label');
	l.Add(Checkbox(ret, [null, 'cool_after_print']));
	l.AddText('Cool');
	e = setup.AddElement('div').AddText('Max Probe Distance:');
	e.Add(Float(ret, [null, 'probe_dist'], 0, 1));
	e.AddText(' ').Add(add_name(ret, 'unit', 0, 0));
	e = setup.AddElement('div').AddText('Probe Safe Retract Distance:');
	e.Add(Float(ret, [null, 'probe_safe_dist'], 0, 1));
	e.AddText(' ').Add(add_name(ret, 'unit', 0, 0));
	e = setup.AddElement('div').AddText('SPI setup:');
	e.Add(Str(ret, [null, 'spi_setup']));
	e = setup.AddElement('div').AddText('Printer Type:');
	var select = e.Add(create_space_type_select());
	var button = e.AddElement('button').AddText('Set');
	button.type = 'button';
	button.obj = select;
	button.printer = ret;
	button.AddEvent('click', function() { set_value(this.printer, [['space', 0], 'type'], this.obj.selectedIndex); });
	e.AddElement('span').id = make_id(ret, [['space', 0], 'type']);
	e = setup.AddElement('div').AddText('Temps:').Add(Float(ret, [null, 'num_temps'], 0));
	e = setup.AddElement('div').AddText('Gpios:').Add(Float(ret, [null, 'num_gpios'], 0));
	e = setup.AddElement('div').AddText('Temp Scale Minimum:');
	e.Add(Float(ret, [null, 'temp_scale_min'], 0, 1));
	e.AddText('°C');
	e = setup.AddElement('div').AddText('Temp Scale Maximum:');
	e.Add(Float(ret, [null, 'temp_scale_max'], 0, 1));
	e.AddText('°C');
	e = setup.AddElement('div').AddText('Max Deviation:');
	e.Add(Float(ret, [null, 'max_deviation'], 2, 1));
	e.AddText(' ').Add(add_name(ret, 'unit', 0, 0));
	e = setup.AddElement('div').AddText('Max v');
	e.Add(Float(ret, [null, 'max_v'], 2, 1));
	e.AddText(' ').Add(add_name(ret, 'unit', 0, 0));
	e.AddText('/s');
	// Cartesian. {{{
	setup.Add([make_table(ret).AddMultipleTitles([
		'Cartesian/Other',
		'Number of Axes'
	], [
		'htitle1',
		'title1'
	], [
		null,
		'Number of axes'
	]).AddMultiple(ret, 'space', Cartesian, false)]);
	// }}}
	// Axis. {{{
	setup.Add([make_table(ret).AddMultipleTitles([
		'Axes',
		'Name',
		UnitTitle(ret, 'Park Pos'),
		'Park Order',
		UnitTitle(ret, 'Min'),
		UnitTitle(ret, 'Max'),
		UnitTitle(ret, '2nd Home Pos')
	], [
		'htitle6',
		'title6',
		'title6',
		'title6',
		'title6',
		'title6',
		'title6'
	], [
		null,
		'Name of the axis',
		'Park position of the nozzle.  This is where the nozzle is sent when it is requested to get out of the way through a park command.',
		'Order when parking.  Equal order parks simultaneously; lower order parks first.',
		'Minimum position that the axis is allowed to go to.  For non-Cartesian, this is normally set to -Infinity for x and y.',
		'Maximum position that the axis is allowed to go to.  For non-Cartesian, this is normally set to Infinity for x and y.',
		'Position to move to after hitting limit switches, before moving in range of limits.'
	]).AddMultiple(ret, 'axis', Axis)]);
	// }}}
	// Motor. {{{
	setup.Add([make_table(ret).AddMultipleTitles([
		'Motor Settings',
		UnitTitle(ret, 'Coupling', null, 'steps/'),
		UnitTitle(ret, 'Switch Pos'),
		'Home Order',
		UnitTitle(ret, 'Limit v', '/s'),
		UnitTitle(ret, 'Limit a', '/s²')
	], [
		'htitle5',
		'title5',
		'title5',
		'title5',
		'title5',
		'title5'
	], [
		null,
		'Number of (micro)steps that the motor needs to do to move the hardware by one unit.',
		'Position of the home switch.',
		'Order when homing.  Equal order homes simultaneously; lower order homes first.',
		'Maximum speed of the motor.',
		'Maximum acceleration of the motor.  4000 is a normal value.'
	]).AddMultiple(ret, 'motor', Motor)]);
	// }}} -->
	// Delta. {{{
	setup.Add([make_table(ret).AddMultipleTitles([
		'Delta',
		UnitTitle(ret, 'Min Distance'),
		UnitTitle(ret, 'Max Distance'),
		UnitTitle(ret, 'Rod Length'),
		UnitTitle(ret, 'Radius')
	], [
		'htitle4',
		'title4',
		'title4',
		'title4',
		'title4'
	], [
		null,
		'Minimum horizontal distance between tie rod pivot points.  Usually 0.',
		'Maximum horizontal distance between tie rod pivot points.  Usually Infinity.',
		'Length of the tie rods between pivot points.  Measure this with as high precision as possible.',
		'Horizontal distance between tie rod pivot points when the end effector is at (0, 0, 0).'
	]).AddMultiple(ret, 'motor', Delta)]);
	setup.Add([make_table(ret).AddMultipleTitles([
		'Delta',
		'Angle'
	], [
		'htitle1',
		'title1'
	], [
		null,
		'Correction angle for the printer. (degrees)'
	]).AddMultiple(ret, 'space', Delta_space, false)]);
	// }}}
	// Polar. {{{
	setup.Add([make_table(ret).AddMultipleTitles([
		'Polar',
		UnitTitle(ret, 'Radius')
	], [
		'htitle1',
		'title1'
	], [
		null,
		'Maximum value for the r motor.'
	]).AddMultiple(ret, 'space', Polar_space, false)]);
	// }}}
	// Extruder. {{{
	setup.Add([make_table(ret).AddMultipleTitles([
		'Extruder',
		'Offset X',
		'Offset Y',
		'Offset Z'
	], [
		'htitle3',
		'title3',
		'title3',
		'title3'
	], [
		null,
		'Offset in X direction when this extruder is in use.  Set to 0 for the first extruder.',
		'Offset in Y direction when this extruder is in use.  Set to 0 for the first extruder.',
		'Offset in Z direction when this extruder is in use.  Set to 0 for the first extruder.'
	]).AddMultiple(ret, 'axis', Extruder, false)]);
	// }}}
	// Follower. {{{
	setup.Add([make_table(ret).AddMultipleTitles([
		'Follower',
		'Space',
		'Motor'
	], [
		'htitle2',
		'title2',
		'title2',
		'title2'
	], [
		null,
		'Space of motor to follow.',
		'Motor to follow.'
	]).AddMultiple(ret, 'motor', Follower, false)]);
	// }}}
	// Temp. {{{
	setup.Add([make_table(ret).AddMultipleTitles([
		'Temp Settings',
		'Name',
		'Fan Temp (°C)',
		'Bed'
	], [
		'htitle3',
		'title3',
		'title3',
		'title3'
	], [
		null,
		'Name of the temperature control',
		'Temerature above which the cooling is turned on.',
		'Whether this Temp is the heated bed, used by G-code commands M140 and M190.'
	]).AddMultiple(ret, 'temp', Temp_setup)]);
	setup.Add([make_table(ret).AddMultipleTitles([
		'Temp Limits',
		'Heater Low Limit (°C)',
		'Heater High Limit (°C)',
		'Fan Low Limit (°C)',
		'Fan High Limit (°C)'
	], [
		'htitle4',
		'title4',
		'title4',
		'title4',
		'title4'
	], [
		null,
		'Temerature below which the heater is never turned on.  Set to NaN to disable limit.',
		'Temerature above which the heater is never turned on.  Set to NaN to disable limit.',
		'Temerature below which the cooling is never turned on.  Set to NaN to disable limit.',
		'Temerature above which the cooling is never turned on.  Set to NaN to disable limit.'
	]).AddMultiple(ret, 'temp', Temp_limits)]);
	setup.Add([make_table(ret).AddMultipleTitles([
		'Temp Hardware',
		'R0 (kΩ) or a',
		'R1 (kΩ) or b',
		'Rc (kΩ) or Scale (%)',
		'Tc (°C) or Offset',
		'β (1) or NaN',
		'Hold Time (s)'
	], [
		'htitle6',
		'title6',
		'title6',
		'title6',
		'title6',
		'title6',
		'title6'
	], [
		null,
		'Resistance on the board in series with the thermistor.  Normally 4.7 or 10.  Or, if β is NaN, the value of this sensor is ax+b with x the measured ADC value; this value is a.',
		'Resistance on the board in parallel with the thermistor.  Normally Infinity.  Or, if β is NaN, the value of this sensor is ax+b with x the measured ADC value; this value is b.',
		'Calibrated resistance of the thermistor.  Normally 100 for extruders, 10 for the heated bed.  Or, if β is NaN, the scale for plotting the value on the temperature graph.',
		'Temperature at which the thermistor has value Rc.  Normally 20.  Or, if β is NaN, the offset for plotting the value on the temperature graph.',
		"Temperature dependence of the thermistor.  Normally around 4000.  It can be found in the thermistor's data sheet.  Or, if NaN, the value of this sensor is ax+b with x the measured ADC value.",
		'Minimum time to keep the heater and fan pins at their values after a change.'
	]).AddMultiple(ret, 'temp', Temp_hardware)]);
	// }}}
	// Gpio. {{{
	setup.Add([make_table(ret).AddMultipleTitles([
		'Gpio',
		'Name',
		'Reset State',
		'Power (%)',
		'Fan',
		'Spindle'
	], [
		'htitle5',
		'title5',
		'title5',
		'title5',
		'title5',
		'title5'
	], [
		null,
		'Name of the Gpio.',
		'Initial state and reset state of the Gpio.  There is a checkbox for the pin if this is not disabled.  If it is input, the checkbox shows the current value.  Otherwise it can be used to change the value.',
		'Fraction of the time that the pin is enabled when on.  Note that this value can only be set up when the corresponding pin is valid.',
		'Whether this Gpio is the fan pin, used by G-code commands M106 and M107.',
		'Whether this Gpio is the spindle pin, used by G-code commands M3, M4 and M5.'
	]).AddMultiple(ret, 'gpio', Gpio)]);
	// }}}
	// Pins. {{{
	var pins = setup.Add(make_table(ret));
	var globalpins = pins.AddElement('tbody');
	globalpins.Add(Pin(ret, 'LED', [null, 'led_pin'], 2));
	globalpins.Add(Pin(ret, 'Stop', [null, 'stop_pin'], 4));
	globalpins.Add(Pin(ret, 'Probe', [null, 'probe_pin'], 4));
	globalpins.Add(Pin(ret, 'SPI SS', [null, 'spiss_pin'], 2));
	pins.AddMultiple(ret, 'motor', Pins_space, false);
	pins.AddMultiple(ret, 'temp', Pins_temp, false);
	pins.AddMultiple(ret, 'gpio', Pins_gpio, false);
	// }}}
	// }}}

	ret.Add(Top(ret));
	ret.AddElement('div', 'spacer');
	ret.Add(Map(ret));
	ret.Add(Gpios(ret));
	ret.Add(Multipliers(ret));
	ret.Add(Temps(ret));
	ret.AddElement('div', 'bottom');
	return ret;
}
// }}}

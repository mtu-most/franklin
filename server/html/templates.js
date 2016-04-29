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
function Button(title, action, className) { // {{{
	var ret = Create('button', 'wide title');
	ret.AddClass(className);
	ret.AddText(title);
	ret.AddEvent('click', function() { this.printer.call(action, [], {}) });
	ret.type = 'button';
	return ret;
} // }}}

function Text(title, obj, className) { // {{{
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

function Name(type, num) { // {{{
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

function Pin(title, obj, type) { // {{{
	var pinselect = Create('select', 'pinselect');
	pinselect.id = make_id(printer, obj);
	pinselect.obj = obj;
	pinselect.printer = printer;
	pinselect.Add(pinrange(type));
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
	return make_tablerow(title, [[pinselect], [validlabel], inverts, [button]], ['pintitle', ['pinvalue', 'pinvalue', 'pinvalue', 'pinvalue']]);
} // }}}

function Float(obj, digits, factor, className, set) { // {{{
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

function File(obj, action, buttontext, types, cb) { // {{{
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

function Checkbox(obj) { // {{{
	var ret = Create('input');
	ret.type = 'checkbox';
	ret.id = make_id(printer, obj);
	ret.obj = obj;
	ret.printer = printer;
	ret.AddEvent('click', function(e) {
		e.preventDefault();
		set_value(ret.printer, ret.obj, ret.checked);
		return false;
	});
	return ret;
} // }}}

function Str(obj) { // {{{
	var ret = Create('span');
	var input = ret.AddElement('input');
	input.type = 'text';
	ret.obj = obj;
	ret.printer = printer;
	var e = ret.AddElement('button');
	e.type = 'button';
	e.AddText('Set');
	e.AddEvent('click', function(event) {
		set_value(ret.printer, ret.obj, input.value);
		return false;
	});
	e = ret.AddElement('span');
	e.id = make_id(printer, obj);
	return ret;
} // }}}

function Id(obj) { // {{{
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
function Extruder(space, axis) {
	var e = ['extruder_dx', 'extruder_dy', 'extruder_dz'];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['axis', [space, axis]], e[i]], 1, 1e-3));
		e[i] = div;
	}
	return make_tablerow(axis_name(space, axis), e, ['rowtitle3'], undefined, TYPE_EXTRUDER, space);
}

function Follower(space, motor) {
	var f = ['follower_space', 'follower_motor'];
	for (var i = 0; i < f.length; ++i) {
		var div = Create('div');
		div.Add(Float([['motor', [space, motor]], f[i]], 0));
		f[i] = div;
	}
	return make_tablerow(motor_name(space, motor), f, ['rowtitle2'], undefined, TYPE_FOLLOWER, space);
}

function Cartesian(num) {
	return make_tablerow(space_name(num), [Float([['space', num], 'num_axes'], 0, 1)], ['rowtitle1'], undefined, [TYPE_CARTESIAN, TYPE_EXTRUDER, TYPE_FOLLOWER], num);
}

function Delta(space, motor) {
	var e = [['delta_axis_min', 1], ['delta_axis_max', 1], ['delta_rodlength', 3], ['delta_radius', 3]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['motor', [space, motor]], e[i][0]], e[i][1], 1));
		e[i] = div;
	}
	return make_tablerow(motor_name(space, motor), e, ['rowtitle4'], undefined, TYPE_DELTA, space);
}

function Delta_space(num) {
	var div = Create('div');
	div.Add(Float([['space', num], 'delta_angle'], 2, Math.PI / 180));
	return make_tablerow(space_name(num), [div], ['rowtitle1'], undefined, TYPE_DELTA, num);
}

function Polar_space(num) {
	var div = Create('div');
	div.Add(Float([['space', num], 'polar_max_r'], 1, 1));
	return make_tablerow(space_name(num), [div], ['rowtitle1'], undefined, TYPE_POLAR, num);
}

function Axis(space, axis) {
	var e = [Name('axis', [space, axis]), ['park', 1, 1], ['park_order', 0, 1], ['min', 1, 1], ['max', 1, 1], ['home_pos2', 1, 1]];
	for (var i = 1; i < e.length; ++i) {
		var div = Create('div');
		if (space == 0)
			div.Add(Float([['axis', [space, axis]], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(axis_name(space, axis), e, ['rowtitle6']);
}

function Motor(space, motor) {
	var e = [['steps_per_unit', 3, 1], ['home_pos', 3, 1], ['home_order', 0, 1], ['limit_v', 0, 1], ['limit_a', 1, 1]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		if (space != 1 || (i != 1 && i != 2))
			div.Add(Float([['motor', [space, motor]], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(motor_name(space, motor), e, ['rowtitle5']);
}

function Pins_space(space, motor) {
	var e = [['Step', 'step', 1], ['Dir', 'dir', 1], ['Enable', 'enable', 2], ['Min Limit', 'limit_min', 4], ['Max Limit', 'limit_max', 4]];
	for (var i = 0; i < e.length; ++i)
		e[i] = Pin(e[i][0], [['motor', [space, motor]], e[i][1] + '_pin'], e[i][2]);
	return make_pin_title(motor_name(space, motor), e, ['rowtitle6']);
}
// }}}

// Temp. {{{
function Temp_setup(num) {
	var e = [Name('temp', num), ['fan_temp', 0, 1], ['heater_limit', 0, 1], ['fan_limit', 0, 1], Id([['temp', num], 'bed'])];
	for (var i = 1; i < e.length - 1; ++i) {
		var div = Create('div');
		div.Add(Float([['temp', num], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(temp_name(printer, num), e, ['rowtitle5']);
}

function Temp_hardware(num) {
	var e = [['R0', 1, 1e3], ['R1', 1, 1e3], ['Rc', 1, 1e3], ['Tc', 0, 1], ['beta', 0, 1], ['hold_time', 1, 1]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['temp', num], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(temp_name(printer, num), e, ['rowtitle5']);
}

function Temp(num) {
	var div = Create(div);
	div.Add(Float([['temp', num], 'value', 'settemp'], 0));
	var current = Create('div');
	current.id = make_id(printer, [['temp', num], 'temp']);
	if (num !== null)
		printer.temptargets.push(current.AddElement('span'));
	var name = temp_name(printer, num);
	if (num !== null && num < 5)
		name.AddClass('temp' + num.toFixed(0));
	return make_tablerow(name, [div, current, Float([['temp', num], 'fan_duty'], 0, 1e-2)], ['rowtitle2']);
}

function Pins_temp(num, dummy, table) {
	var e = [['Heater', 'heater', 2], ['Fan', 'fan', 2], ['Thermistor', 'thermistor', 8]];
	for (var i = 0; i < e.length; ++i)
		e[i] = Pin(e[i][0], [['temp', num], e[i][1] + '_pin'], e[i][2]);
	return make_pin_title(temp_name(printer, num), e);
}
// }}}

// Gpio. {{{
function Gpio(num) {
	var reset = Create('select');
	reset.id = make_id(printer, [['gpio', num], 'reset']);
	reset.AddElement('option').AddText('Off').Value = 0;
	reset.AddElement('option').AddText('On').Value = 1;
	reset.AddElement('option').AddText('Input').Value = 2;
	reset.AddElement('option').AddText('Disabled').Value = 3;
	var p = printer;
	reset.AddEvent('change', function(e) {
		var value = reset.options[reset.selectedIndex].Value;
		set_value(p, [['gpio', num], 'reset'], value);
		if (value >= 2)
			set_value(p, [['gpio', num], 'state'], value);
		e.preventDefault();
		return false;
	});
	return make_tablerow(gpio_name(num), [Name('gpio', num), reset, Float([['gpio', num], 'duty'], 0, 1e-2), Id([['gpio', num], 'fan']), Id([['gpio', num], 'spindle'])], ['rowtitle5']);
}

function Pins_gpio(num) {
	var e = [['Pin', 'pin', 6]];
	for (var i = 0; i < e.length; ++i)
		e[i] = Pin(e[i][0], [['gpio', num], e[i][1]], e[i][2]);
	return make_pin_title(gpio_name(num), e);
}
// }}}


function Label(printer) {	// {{{
	var ret = Create('div', 'tab noflash nodetect');
	if (selected_port == port)
		ret.AddClass('active');
	ret.AddEvent('click', function() { select_printer(this.port); });
	ret.port = port;
	if (printer) {
		ret.AddElement('span', 'setup').AddText(printer.uuid);
		var selector = ret.AddElement('select').AddEvent('change', function() {
			this.printer.call('load', [selector.value], {});
		});
		selector.printer = printer;
		selector.id = make_id(printer, [null, 'profiles']);
		update_profiles(printer);
	}
	var span = ret.AddElement('span', 'port setup').AddText('@' + port);
	span.AddElement('span', 'ifflash').AddText('[!]');
	span.AddElement('span', 'ifdetect').AddText('[?]');
	return ret;
}
// }}}

// Printer parts. {{{
function Top() { // {{{
	var ret = Create('div', 'top');
	// Up/remove/down. {{{
	var e = ret.AddElement('div', 'updown');
	var the_printer = printer;
	e.AddElement('button', 'queue1').AddEvent('click', function() { queue_up(the_printer); }).AddText('⬆').type = 'button';
	e.AddElement('br');
	e.AddElement('button', 'queue1').AddEvent('click', function() { queue_del(the_printer); }).AddText('×').type = 'button';
	e.AddElement('br');
	e.AddElement('button', 'queue1').AddEvent('click', function() {queue_down(the_printer); }).AddText('⬇').type = 'button';
	// }}}
	// Jobs. {{{
	var p = printer;
	e = ret.AddElement('div', 'jobs').AddElement('select').AddEvent('change', function() { start_move(p); });
	e.printer = printer;
	e.multiple = true;
	e.id = make_id(printer, [null, 'queue']);
	// }}}
	// Jobbuttons. {{{
	e = ret.AddElement('div', 'jobbuttons');
	e.Add(File([null, 'queue_add', 'queue_add'], 'queue_add', 'Add', '.gcode,.ngc,application/x-gcode', function() { return queue_deselect(the_printer); }));
	e.AddElement('br');
	e.Add(File([null, 'audio_add', 'audio_add'], 'audio_add', 'Add Audio', 'audio/x-wav', function() { return queue_deselect(the_printer); }), 'benjamin');
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
	b = e.AddElement('button', 'abort').AddText('Abort').AddEvent('click', function() { this.printer.call('abort', [], {}); });
	b.type = 'button';
	b.printer = printer;
	e.AddElement('br');
	b = e.AddElement('button').AddText('Home').AddEvent('click', function() { this.printer.call('home', [], {}, function() { update_canvas_and_spans(b.printer); }); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Pause').AddEvent('click', function() { this.printer.call('pause', [true], {}, function() { update_canvas_and_spans(b.printer); }); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Resume').AddEvent('click', function() { this.printer.call('pause', [false], {}); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Sleep').AddEvent('click', function() { this.printer.call('sleep', [], {}, function() { update_canvas_and_spans(b.printer); }); });
	b.type = 'button';
	b.printer = printer;
	// }}}
	return ret;
}
// }}}

function Map() { // {{{
	var ret = Create('div', 'map');
	ret.id = make_id(printer, [null, 'map']);
	// Current position buttons.
	var t = ret.Add(make_table().AddMultipleTitles([
		'',
		[add_name('axis', 0, 0), ' (', add_name('unit', 0, 0), ')'],
		[add_name('axis', 0, 1), ' (', add_name('unit', 0, 0), ')'],
		[add_name('axis', 0, 2), ' (', add_name('unit', 0, 0), ')'],
		'',
		'',
		''
	], ['', '', '', '', '', ''], null), 'maptable');
	var b = Create('button').AddText('Park').AddEvent('click', function() { this.printer.call('park', [], {}, function() { update_canvas_and_spans(b.printer); }); });
	b.type = 'button';
	b.printer = printer;
	t.Add(make_tablerow(add_name('space', 0, 0), [
		Float([['axis', [0, 0]], 'current'], 2, 1, '', function(v) { b.printer.call('line_cb', [[{0: v}]], {}); b.printer.call('wait_for_cb', [], {}, function() { update_canvas_and_spans(b.printer); }); }),
		Float([['axis', [0, 1]], 'current'], 2, 1, '', function(v) { b.printer.call('line_cb', [[{1: v}]], {}); b.printer.call('wait_for_cb', [], {}, function() { update_canvas_and_spans(b.printer); }); }),
		Float([['axis', [0, 2]], 'current'], 2, 1, '', function(v) { b.printer.call('line_cb', [[{2: v}]], {}); b.printer.call('wait_for_cb', [], {}, function() { update_canvas_and_spans(b.printer); }); }),
		b
	], ['', '', '', '', '', '']));
	// Target position buttons.
	var b = Create('button').AddText('Use Current').AddEvent('click', function() {
		b.printer.call('set_globals', [], {'targetx': b.printer.spaces[0].axis[0].current, 'targety': b.printer.spaces[0].axis[1].current});
	});
	b.printer = printer;
	b.type = 'button';
	t.Add(make_tablerow('Target:', [
		Float([null, 'targetx'], 2, 1),
		Float([null, 'targety'], 2, 1),
		Float([null, 'zoffset'], 2, 1),
		b,
		['Angle:', Float([null, 'targetangle'], 1, Math.PI / 180, '', function(v) { update_angle(b.printer, v); }), '°']
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

function Temps() { // {{{
	var ret = Create('div', 'temp');
	ret.Add(make_table().AddMultipleTitles([
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
	]).AddMultiple('temp', Temp));
	ret.AddElement('canvas', 'tempgraph').id = make_id(printer, [null, 'tempgraph']);
	return ret;
}
// }}}

function Multipliers() { // {{{
	var ret = Create('div', 'multipliers');
	var e = ret.AddElement('div').AddText('Feedrate: ');
	e.Add(Float([null, 'feedrate'], 0, 1e-2));
	e.AddText(' %');
	ret.AddMultiple('axis', function(space, axis, obj) {
		if (space != 1)
			return null;
		var e = Create('div');
		e.Add(axis_name(space, axis));
		e.printer = printer;
		e.Add(Float([['axis', [space, axis]], 'multiplier'], 0, 1e-2));
		e.AddText(' %');
		e.Add(Float([['axis', [space, axis]], 'current'], 1, 1, '', function(v) {
			var obj = {};
			obj[space] = {};
			obj[space][axis] = v;
			e.printer.call('line_cb', [obj], {}, function() {
				e.printer.call('wait_for_cb', [], {}, function() { update_canvas_and_spans(e.printer); });
			});
		}));
		e.AddText(' ').Add(add_name('unit', 0, 0));
		return e;
	}, true);
	e = ret.AddElement('div', 'admin');
	e.AddElement('Label').AddText('Store adc readings').Add(Checkbox([null, 'store_adc']));
	e.AddElement('a').AddText('Get stored readings').href = 'adc';
	return ret;
}
// }}}

function Gpios() { // {{{
	var ret = Create('div', 'gpios');
	var p = printer;
	ret.AddMultiple('gpio', function(i) {
		var ret = Create('span');
		ret.id = make_id(printer, [['gpio', i], 'statespan']);
		var label = ret.AddElement('label');
		var input = label.AddElement('input');
		label.Add(gpio_name(i));
		var index = i;
		input.AddEvent('click', function(e) {
			set_value(p, [['gpio', index], 'state'], input.checked ? 1 : 0);
			e.preventDefault();
			return false;
		});
		input.type = 'checkbox';
		input.id = make_id(p, [['gpio', i], 'state']);
		return ret;
	}, false);
	return ret;
}
// }}}
// }}}

function Printer() {	// {{{
	var ret = Create('div', 'printer');
	// Blocker bar. {{{
	ret.id = make_id(printer, [null, 'container']);
	var blocker = ret.AddElement('div', 'hidden blocker');
	blocker.id = make_id(printer, [null, 'block1']);
	// }}}
	ret.AddElement('div', 'message hidden').id = make_id(printer, [null, 'message1']);
	ret.AddElement('div', 'message').id = make_id(printer, [null, 'printstate']);
	ret.AddElement('div', 'message hidden').id = make_id(printer, [null, 'confirm']);
	// Setup. {{{
	var setup = ret.AddElement('div', 'setup expert');
	// Save and restore. {{{
	var e = setup.AddElement('div', 'admin');
	e.AddText('Profile');
	var b = e.AddElement('button').AddText('Save (as)').AddEvent('click', function() {
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
	setup.AddElement('div').Add(File([null, 'import', 'import_settings'], 'import', 'Import', '.ini'));
	e = setup.AddElement('a', 'title').AddText('Export settings to file');
	e.id = make_id(printer, [null, 'export']);
	e.title = 'Save settings to disk.';
	// }}}
	var disable = setup.AddElement('div').AddElement('button').AddText('Disable Printer');
	disable.port = port;
	disable.printer = printer;
	disable.type = 'button';
	disable.AddEvent('click', function() { this.printer.disabling = true; rpc.call('disable', [this.port], {}); });
	e = setup.AddElement('div').AddText('Timeout:');
	e.Add(Float([null, 'timeout'], 0, 60));
	e.AddText(' min');
	e = setup.AddElement('div').AddText('After Print:');
	var l = e.AddElement('label');
	l.Add(Checkbox([null, 'park_after_print']));
	l.AddText('Park');
	l = e.AddElement('label');
	l.Add(Checkbox([null, 'sleep_after_print']));
	l.AddText('Sleep');
	l = e.AddElement('label');
	l.Add(Checkbox([null, 'cool_after_print']));
	l.AddText('Cool');
	e = setup.AddElement('div').AddText('Max Probe Distance:');
	e.Add(Float([null, 'probe_dist'], 0, 1));
	e.AddText(' ').Add(add_name('unit', 0, 0));
	e = setup.AddElement('div').AddText('Probe Safe Retract Distance:');
	e.Add(Float([null, 'probe_safe_dist'], 0, 1));
	e.AddText(' ').Add(add_name('unit', 0, 0));
	e = setup.AddElement('div').AddText('SPI setup:');
	e.Add(Str([null, 'spi_setup']));
	e = setup.AddElement('div').AddText('Printer Type:');
	var select = e.Add(create_space_type_select());
	var button = e.AddElement('button').AddText('Set');
	button.type = 'button';
	button.obj = select;
	button.printer = printer;
	button.AddEvent('click', function() { set_value(this.printer, [['space', 0], 'type'], this.obj.selectedIndex); });
	e.AddElement('span').id = make_id(printer, [['space', 0], 'type']);
	e = setup.AddElement('div').AddText('Temps:').Add(Float([null, 'num_temps'], 0));
	e = setup.AddElement('div').AddText('Gpios:').Add(Float([null, 'num_gpios'], 0));
	e = setup.AddElement('div').AddText('Temp Scale Minimum:');
	e.Add(Float([null, 'temp_scale_min'], 0, 1));
	e.AddText('°C');
	e = setup.AddElement('div').AddText('Temp Scale Maximum:');
	e.Add(Float([null, 'temp_scale_max'], 0, 1));
	e.AddText('°C');
	e = setup.AddElement('div').AddText('Max Deviation:');
	e.Add(Float([null, 'max_deviation'], 2, 1));
	e.AddText(' ').Add(add_name('unit', 0, 0));
	e = setup.AddElement('div').AddText('Max v');
	e.Add(Float([null, 'max_v'], 2, 1));
	e.AddText(' ').Add(add_name('unit', 0, 0));
	e.AddText('/s');
	// Cartesian. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Cartesian/Other',
		'Number of Axes'
	], [
		'htitle1',
		'title1'
	], [
		null,
		'Number of axes'
	]).AddMultiple('space', Cartesian, false)]);
	// }}}
	// Axis. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Axes',
		'Name',
		UnitTitle('Park Pos'),
		'Park Order',
		UnitTitle('Min'),
		UnitTitle('Max')
		UnitTitle('2nd Home Pos')
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
	]).AddMultiple('axis', Axis)]);
	// }}}
	// Motor. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Motor Settings',
		UnitTitle('Coupling', null, 'steps/'),
		UnitTitle('Switch Pos'),
		'Home Order',
		UnitTitle('Limit v', '/s'),
		UnitTitle('Limit a', '/s²')
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
	]).AddMultiple('motor', Motor)]);
	// }}} -->
	// Delta. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Delta',
		UnitTitle('Min Distance'),
		UnitTitle('Max Distance'),
		UnitTitle('Rod Length'),
		UnitTitle('Radius')
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
	]).AddMultiple('motor', Delta)]);
	setup.Add([make_table().AddMultipleTitles([
		'Delta',
		'Angle'
	], [
		'htitle1',
		'title1'
	], [
		null,
		'Correction angle for the printer. (degrees)'
	]).AddMultiple('space', Delta_space, false)]);
	// }}}
	// Polar. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Polar',
		UnitTitle('Radius')
	], [
		'htitle1',
		'title1'
	], [
		null,
		'Maximum value for the r motor.'
	]).AddMultiple('space', Polar_space, false)]);
	// }}}
	// Extruder. {{{
	setup.Add([make_table().AddMultipleTitles([
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
	]).AddMultiple('axis', Extruder, false)]);
	// }}}
	// Follower. {{{
	setup.Add([make_table().AddMultipleTitles([
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
	]).AddMultiple('motor', Follower, false)]);
	// }}}
	// Temp. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Temp Settings',
		'Name',
		'Fan Temp (°C)',
		'Heater Limit (°C)',
		'Fan Limit (°C)',
		'Bed'
	], [
		'htitle5',
		'title5',
		'title5',
		'title5',
		'title5',
		'title5'
	], [
		null,
		'Name of the temperature control',
		'Temerature above which the cooling is turned on.',
		'Temerature below which the heater is never turned on.  Set to NaN to disable limit.  (Must always be lower than the target.)',
		'Temerature above which the cooling is never turned on.  Set to NaN to disable limit.  (Must always be higher than the target.)',
		'Whether this Temp is the heated bed, used by G-code commands M140 and M190.'
	]).AddMultiple('temp', Temp_setup)]);
	setup.Add([make_table().AddMultipleTitles([
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
	]).AddMultiple('temp', Temp_hardware)]);
	// }}}
	// Gpio. {{{
	setup.Add([make_table().AddMultipleTitles([
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
	]).AddMultiple('gpio', Gpio)]);
	// }}}
	// Pins. {{{
	var pins = setup.Add(make_table());
	var globalpins = pins.AddElement('tbody');
	globalpins.Add(Pin('LED', [null, 'led_pin'], 2));
	globalpins.Add(Pin('Stop', [null, 'stop_pin'], 4));
	globalpins.Add(Pin('Probe', [null, 'probe_pin'], 4));
	globalpins.Add(Pin('SPI SS', [null, 'spiss_pin'], 2));
	pins.AddMultiple('motor', Pins_space, false);
	pins.AddMultiple('temp', Pins_temp, false);
	pins.AddMultiple('gpio', Pins_gpio, false);
	// }}}
	// }}}

	ret.Add(Top());
	ret.AddElement('div', 'spacer');
	ret.Add(Map());
	ret.Add(Gpios());
	ret.Add(Multipliers());
	ret.Add(Temps());
	ret.AddElement('div', 'bottom');
	return ret;
}
// }}}

function NoPrinter(options) { // {{{
	var ret = Create('div', 'noprinter notconnected setup hidden');
	ret.id = make_id({'port': port}, [null, 'nocontainer']);
	var blocker = ret.AddElement('div', 'hidden blocker');
	blocker.id = make_id({'port': port}, [null, 'block2']);
	ret.AddElement('h2').AddText('No printer is found on port ' + port + '.');
	var detect = ret.AddElement('p').AddText('If autodetect does not work, you can request to detect a printer:').AddElement('button', 'upload').AddText('Detect');
	detect.type = 'button';
	detect.port = port;
	detect.AddEvent('click', function() { rpc.call('detect', [this.port], {}); });
	ret.options = ret.AddElement('p', 'admin').AddText('Or you can upload the firmware that fits your hardware.').AddElement('ul');
	var message = ret.AddElement('div', 'message');
	message.id = make_id({'port': port}, [null, 'message2']);
	return ret;
}
// }}}

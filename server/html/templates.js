// vim: set foldmethod=marker :

// Primitives. {{{
function Button(title, action, className) {
	var ret = Create('button', 'wide title');
	ret.AddClass(className);
	ret.AddText(title);
	ret.AddEvent('click', function() { this.printer.call(action, [], {}) });
	ret.type = 'button';
	return ret;
}

function Text(title, obj, className) {
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
}

function Pin(title, obj, analog) {
	var pinselect = Create('select', 'pinselect');
	pinselect.id = make_id(printer, obj);
	pinselect.obj = obj;
	pinselect.printer = printer;
	pinselect.Add(pinrange(analog));
	pinselect.analog = analog;
	var validinput = Create('input');
	validinput.type = 'checkbox';
	validinput.id = make_id(printer, obj, 'valid');
	var validlabel = Create('label').AddText('Valid');
	validlabel.htmlFor = validinput.id;
	var inverts;
	if (!analog) {
		var invertedinput = Create('input');
		invertedinput.type = 'checkbox';
		invertedinput.id = make_id(printer, obj, 'inverted');
		var invertedlabel = Create('label').AddText('Inverted');
		invertedlabel.htmlFor = invertedinput.id;
		inverts = [invertedinput, invertedlabel];
	}
	else
		inverts = [];
	var button = Create('button', 'button');
	button.type = 'button';
	button.AddText('Set');
	button.printer = printer;
	button.AddEvent('click', function() { set_pin(this.printer, obj); });
	return make_tablerow(title, [[pinselect], [validinput, validlabel], inverts, [button]], ['pintitle', ['pinvalue', 'pinvalue', 'pinvalue', 'pinvalue']]);
}

function Float(obj, digits, factor, className, set) {
	var input = Create('input', className);
	//var button = Create('button', className).AddText('Set');
	var span = Create('span', className);
	input.obj = obj;
	//button.obj = obj;
	if (factor === undefined)
		factor = 1;
	input.factor = factor;
	//button.factor = factor;
	span.factor = factor;
	input.id = make_id(printer, obj, 'new');
	span.id = make_id(printer, obj);
	span.digits = digits;
	//button.source = input;
	input.type = 'text';
	input.set = set;
	input.printer = printer;
	input.AddEvent('keydown', function(event) { floatkey(event, this); });
	//button.AddEvent('click', function() { floatkey({keyCode: 13, preventDefault: function() {}}, this.source); });
	return [input, /*button,*/ span];
}

function File(obj, buttontext, cb) {
	var input = Create('input');
	input.type = 'file';
	input.id = make_id(printer, obj);
	var button = Create('button', 'button').AddText(buttontext);
	button.type = 'button';
	button.source = obj;
	button.extra = cb;
	button.printer = printer;
	button.AddEvent('click', function() { set_file(this.printer, this.source); if (this.extra !== undefined) this.extra(); });
	return [input, button];
}

function Spacetype(num) {
	var select = create_space_type_select();
	var button = Create('button').AddText('Set');
	button.type = 'button';
	button.index = num;
	button.obj = select;
	button.printer = printer;
	button.AddEvent('click', function() { set_value(this.printer, [['space', this.index], 'type'], this.obj.selectedIndex); });
	var span = Create('span');
	span.id = make_id(printer, [['space', num], 'type']);
	var div = Create('div').AddText('Max Deviation');
	div.Add(Float([['space', num], 'max_deviation'], 2, 1e-3));
	return make_tablerow(space_name(num), [[select, button, span], div], ['rowtitle1']);
}
// }}}


// Space. {{{
function Extruder(num) {
	var e = ['dx', 'dy', 'dz'];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['space', num], e[i]], 1, 1));
		e[i] = div;
	}
	return make_tablerow(space_name(num), e, ['rowtitle3'], undefined, TYPE_EXTRUDER, num);
}

function Cartesian(num) {
	return make_tablerow(space_name(num), [Float([['space', num], 'num_axes'], 0, 1)], ['rowtitle1'], undefined, TYPE_CARTESIAN, num);
}

function Delta_motor(space, motor) {
	var e = [['delta_axis_min', 1], ['delta_axis_max', 1], ['delta_rodlength', 3], ['delta_radius', 3]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['motor', [space, motor]], e[i][0]], e[i][1], 1e-3));
		e[i] = div;
	}
	return make_tablerow(motor_name(space, motor), e, ['rowtitle4'], undefined, TYPE_DELTA, space);
}

function Delta(num, dummy, table) {
	table.AddMultiple('motor', Delta_motor, true, num);
	return [];
}

function Delta_space(num) {
	var div = Create('div');
	div.Add(Float([['space', num], 'delta_angle'], 2, Math.PI / 180));
	return make_tablerow(space_name(num), [div], ['rowtitle1'], undefined, TYPE_DELTA, num);
}

function Axis_axis(space, axis) {
	var e = [['park', 1, 1e-3], ['park_order', 0, 1], ['max_v', 0, 1e-3], ['min', 1, 1e-3], ['max', 1, 1e-3]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['axis', [space, axis]], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(axis_name(space, axis), e, ['rowtitle5']);
}

function Axis(num, dummy, table) {
	table.AddMultiple('axis', Axis_axis, true, num);
	return [];
}

function Motor_motor(space, motor) {
	var e = [['steps_per_m', 3, 1e3], ['max_steps', 0, 1], ['home_pos', 3, 1e-3], ['home_order', 0, 1], ['limit_v', 0, 1e-3], ['limit_a', 1, 1]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['motor', [space, motor]], e[i][0]], e[i][1], e[i][2]));
		e[i] = div;
	}
	return make_tablerow(motor_name(space, motor), e, ['rowtitle6']);
}


function Motor(num, dummy, table) {
	table.AddMultiple('motor', Motor_motor, true, num);
	return [];
}

function Pins_motor(space, motor) {
	var e = [['Step', 'step'], ['Dir', 'dir'], ['Enable', 'enable'], ['Min Limit', 'limit_min'], ['Max Limit', 'limit_max'], ['Sense', 'sense']];
	for (var i = 0; i < e.length; ++i)
		e[i] = Pin(e[i][0], [['motor', [space, motor]], e[i][1] + '_pin']);
	return make_pin_title(motor_name(space, motor), e, ['rowtitle6']);
}

function Pins_space(num, dummy, table) {
	table.AddMultiple('motor', Pins_motor, false, num);
	return [];
}
// }}}

// Temp. {{{
function Temp_setup(num) {
	var e = [['fan_temp', 0], ['R0', 1, 1e3], ['R1', 1, 1e3], ['Rc', 1, 1e3], ['Tc', 0, 1], ['beta', 0, 1]];
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
	return make_tablerow(temp_name(printer, num), [div, current], ['rowtitle2']);
}

function Pins_temp(num, dummy, table) {
	var e = [['Heater', 'heater', false], ['Fan', 'fan', false], ['Thermistor', 'thermistor', true]];
	for (var i = 0; i < e.length; ++i)
		e[i] = Pin(e[i][0], [['temp', num], e[i][1] + '_pin'], e[i][2]);
	return make_pin_title(temp_name(num), e);
}
// }}}

// Gpio. {{{
function Pins_gpio(num) {
	var e = [['Pin', 'pin']];
	for (var i = 0; i < e.length; ++i)
		e[i] = Pin(e[i][0], [['gpio', num], e[i][1] + '_pin']);
	return make_pin_title(gpio_name(num), [Pin('Pin', [['gpio', num], 'pin'])]);
}
// }}}


function Label() {	// {{{
	var ret = Create('div', 'tab');
	ret.AddEvent('click', function() { select_printer(this.port); });
	ret.port = port;
	if (printer) {
		ret.AddText(printer.name);
		selector = ret.AddElement('select', 'hidden').AddEvent('change', function() {
			this.printer.call('load', [selector.value], {});
		});
		selector.printer = printer;
		selector.id = make_id(printer, [null, 'profiles']);
		update_profiles(printer);
	}
	ret.AddElement('span', 'port setup').AddText('@' + port);
	return ret;
}
// }}}

// Printer parts. {{{
function Top() { // {{{
	var ret = Create('div', 'top');
	// Profile choice. TODO {{{
	// }}}
	// Up/remove/down. {{{
	var e = ret.AddElement('div', 'updown');
	e.AddElement('button', 'queue1').AddEvent('click', queue_up).AddText('⬆').type = 'button';
	e.AddElement('br');
	e.AddElement('button', 'queue1').AddEvent('click', queue_del).AddText('×').type = 'button';
	e.AddElement('br');
	e.AddElement('button', 'queue1').AddEvent('click', queue_down).AddText('⬇').type = 'button';
	// }}}
	// Jobs. {{{
	e = ret.AddElement('div', 'jobs').AddElement('select');
	e.multiple = true;
	e.id = make_id(printer, [null, 'queue']);
	// }}}
	// Jobbuttons. {{{
	e = ret.AddElement('div', 'jobbuttons');
	e.Add(File([null, 'queue_add', 'queue_add'], 'Add', queue_deselect));
	e.AddElement('br');
	var b = e.AddElement('button', 'jobbutton').AddEvent('click', function() { queue_print(this.printer); }).AddText('Print selected');
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('input', 'jobbutton');
	var id = make_id(printer, [null, 'probebox']);
	b.id = id;
	b.type = 'checkbox';
	b = e.AddElement('label').AddText('Probe');
	b.htmlFor = id;
	// }}}
	// Stop buttons. {{{
	e = ret.AddElement('div', 'stop');
	b = e.AddElement('button', 'abort').AddText('Abort').AddEvent('click', function() { this.printer.call('abort', [], {}); });
	b.type = 'button';
	b.printer = printer;
	e.AddElement('br');
	b = e.AddElement('button').AddText('Home').AddEvent('click', function() { this.printer.call('home', [], {}, update_canvas_and_spans); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Pause').AddEvent('click', function() { this.printer.call('pause', [true], {}, update_canvas_and_spans); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Resume').AddEvent('click', function() { this.printer.call('pause', [false], {}); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Sleep').AddEvent('click', function() { this.printer.call('sleep', [], {}, update_canvas_and_spans); });
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
		'X (mm)',
		'Y (mm)',
		'Z (mm)',
		'',
		'',
		''
	], ['', '', '', '', '', ''], null), 'maptable');
	var b = Create('button').AddText('Park').AddEvent('click', function() { this.printer.call('park', [], {}); });
	b.type = 'button';
	b.printer = printer;
	t.Add(make_tablerow('Position:', [
		Float([['axis', [0, 0]], 'current'], 2, 1e-3, '', function(v) { b.printer.call('goto', [[{0: v}]], {cb: true}); b.printer.call('wait_for_cb', [], {}, update_canvas_and_spans); }),
		Float([['axis', [0, 1]], 'current'], 2, 1e-3, '', function(v) { b.printer.call('goto', [[{1: v}]], {cb: true}); b.printer.call('wait_for_cb', [], {}, update_canvas_and_spans); }),
		Float([['axis', [0, 2]], 'current'], 2, 1e-3, '', function(v) { b.printer.call('goto', [[{2: v}]], {cb: true}); b.printer.call('wait_for_cb', [], {}, update_canvas_and_spans); }),
		b,
		'',
		['Z Offset:', Float([null, 'zoffset'], 2, 1e-3), 'mm']
	], ['', '', '', '', '', '']));
	// Target position buttons.
	var b = Create('button').AddText('Use Current').AddEvent('click', function() {
		b.printer.targetx = b.printer.spaces[0].axis[0].current;
		b.printer.targety = b.printer.spaces[0].axis[1].current;
		if (!get_element(b.printer, [null, 'zlock']).checked)
			b.printer.targetz = b.printer.spaces[0].axis[2].current;
		update_canvas_and_spans();
	});
	b.printer = printer;
	b.type = 'button';
	var c = Create('input');
	c.type = 'checkbox';
	c.id = make_id(printer, [null, 'zlock']);
	c.checked = true;
	var l = Create('label').AddText('Lock Z');
	l.htmlFor = c.id;
	t.Add(make_tablerow('Target:', [
		Float([null, 'targetx'], 2, 1e-3, '', function(v) { b.printer.targetx = v; update_canvas_and_spans(); }),
		Float([null, 'targety'], 2, 1e-3, '', function(v) { b.printer.targety = v; update_canvas_and_spans(); }),
		Float([null, 'targetz'], 2, 1e-3, '', function(v) { b.printer.targetz = v; update_canvas_and_spans(); }),
		b,
		[c, l],
		['Angle:', Float([null, 'targetangle'], 1, Math.PI / 180, '', function(v) { b.printer.targetangle = v; update_canvas_and_spans(); }), '°']
	], ['', '', '', '', '', '']));
	// Canvas for xy and for z.
	c = ret.AddElement('canvas', 'xymap');
	c.AddEvent('mousemove', xymove).AddEvent('mousedown', xydown);
	c.id = make_id(printer, [null, 'xymap']);
	c.printer = printer;
	c = ret.AddElement('canvas', 'zmap');
	c.AddEvent('mousemove', zmove).AddEvent('mousedown', zdown);
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
		'Current (°C)'
	], [
		'htitle4',
		'title4',
		'title4'
	], [
		null,
		'Temperature target.  Set to NaN to disable the heater completely.',
		'Request actual temperature from sensor.'
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
	ret.AddMultiple('space', function(space, dummy, obj) {
		if (space == 1)
			obj.AddMultiple('axis', function(space, i) {
				var e = Create('div').AddText(axis_name(1, i) + ': ');
				e.printer = printer;
				e.Add(Float([['axis', [1, i]], 'multiplier'], 0, 1e-2));
				e.AddText(' %');
				e.Add(Float([['axis', [1, i]], 'current'], 1, 1e-3, '', function(v) {
					var obj = {};
					obj[i] = v;
					e.printer.call('goto', [{1: obj}], {cb: true}, function() {
						e.printer.call('wait_for_cb', [], {}, update_canvas_and_spans);
					});
				}));
				e.AddText(' mm');
				return e;
			}, true, 1);
		return [];
	}, false);
	return ret;
}
// }}}

function Gpios() { // {{{
	var ret = Create('div', 'gpios');
	var p = printer;
	ret.AddMultiple('gpio', function(i) {
		var ret = Create('span');
		var input = ret.AddElement('input');
		var index = i;
		input.AddEvent('change', function() { set_value(p, [['gpio', index], 'state'], input.checked ? 1 : 0) });
		input.type = 'checkbox';
		input.id = make_id(p, [['gpio', i], 'state']);
		ret.AddElement('label').AddText(gpio_name(i)).htmlFor = input.id;
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
	ret.AddElement('div', 'message hidden').id = make_id(printer, [null, 'confirm']);
	// Setup. {{{
	var setup = ret.AddElement('div', 'setup');
	// Save and restore. {{{
	var e = setup.AddElement('div');
	e.AddText('Profile');
	var b = e.AddElement('button').AddText('Save as').AddEvent('click', function() {
		this.printer.call('save', [this.saveas.value], {});
	});
	b.type = 'button';
	b.printer = printer;
	b.saveas = e.AddElement('input');
	b.saveas.type = 'text';
	setup.AddElement('div').Add(File([null, 'import', 'import_settings'], 'Import'));
	e = setup.AddElement('a', 'title').AddText('Export settings to file');
	e.id = make_id(printer, [null, 'export']);
	e.title = 'Save settings to disk.';
	// }}}
	var disable = setup.AddElement('div').AddElement('button').AddText('Disable Printer');
	disable.port = port;
	disable.type = 'button';
	disable.AddEvent('click', function() { rpc.call('disable', [this.port], {}); });
	setup.AddElement('div').Add(Text('Name', [null, 'name']));
	e = setup.AddElement('div').AddText('Timeout:');
	e.Add(Float([null, 'timeout'], 0, 60));
	e.AddText(' min');
	e = setup.AddElement('div').AddText('Max Probe Distance:');
	e.Add(Float([null, 'probe_dist'], 0, 1e-3));
	e.AddText(' mm');
	e = setup.AddElement('div').AddText('Probe Safe Retract Distance:');
	e.Add(Float([null, 'probe_safe_dist'], 0, 1e-3));
	e.AddText(' mm');
	e = setup.AddElement('div').AddText('ID of Bed Temp (255 for None):').Add(Float([null, 'bed_id'], 0));
	e = setup.AddElement('div').AddText('Spaces:').Add(Float([null, 'num_spaces'], 0));
	e = setup.AddElement('div').AddText('Temps:').Add(Float([null, 'num_temps'], 0));
	e = setup.AddElement('div').AddText('Gpios:').Add(Float([null, 'num_gpios'], 0));
	// Space. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Spaces',
		'Type',
		'Max Deviation (mm)'
	], [
		'htitle2',
		'title2',
		'title2'
	], [
		null,
		'Printer type',
		'Corners are rounded from requested path to this amount'
	]).AddMultiple('space', Spacetype)]);
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
		'Offset in X direction when this extruder is in use.',
		'Offset in Y direction when this extruder is in use.',
		'Offset in Z direction when this extruder is in use.'
	]).AddMultiple('space', Extruder, false, 0)]);
	// }}}
	// Cartesian. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Cartesian',
		'Number of axes'
	], [
		'htitle1',
		'title1'
	], [
		null,
		'Number of axes'
	]).AddMultiple('space', Cartesian, false, 0)]);
	// }}}
	// Delta. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Delta',
		'Min Axis Distance',
		'Max Axis Distance',
		'Rod Length',
		'Radius'
	], [
		'htitle4',
		'title4',
		'title4',
		'title4',
		'title4'
	], [
		null,
		'Minimum distance of end effector from nozzle.  Usually 0. (mm)',
		'Maximum distance of end effector from nozzle.  Usually rod length. (mm)',
		'Length of the tie rods. (mm)',
		'Length of horizontal projection of the tie rod when the end effector is at (0, 0). (mm)'
	]).AddMultiple('space', Delta, false, 1)]);
	setup.Add([make_table().AddMultipleTitles([
		'Delta',
		'Angle'
	], [
		'htitle1',
		'title1'
	], [
		null,
		'Correction angle for the printer. (degrees)'
	]).AddMultiple('space', Delta_space, false, 1)]);
	// }}}
	// Axis. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Axes',
		'Park pos (mm)',
		'Park order',
		'Max v (mm/s)',
		'Min (mm)',
		'Max (mm)'
	], [
		'htitle5',
		'title5',
		'title5',
		'title5',
		'title5',
		'title5'
	], [
		null,
		'Park position of the nozzle.',
		'Order when parking.  Equal order parks simultaneously; lower order parks first.',
		'Maximum speed that the motor is allowed to move.',
		'Minimum position that the axis is allowed to go to.',
		'Maximum position that the axis is allowed to go to.'
	]).AddMultiple('space', Axis, false)]);
	// }}}
	// Motor. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Motor',
		'Coupling (steps/mm)',
		'Microsteps',
		'Switch pos (mm)',
		'Home order',
		'Limit v (mm/s)',
		'Limit a (m/s²)'
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
		'Number of (micro)steps that the motor needs to do to move the hardware by one mm.',
		'Maximum number of steps to do in one iteration.  Set to number of microsteps.',
		'Position of the home switch. (mm)',
		'Order when homing.  Equal order homes simultaneously; lower order homes first.',
		'Maximum speed of the motor. (mm/s)',
		'Maximum acceleration of the motor. (m/s²)'
	]).AddMultiple('space', Motor, false)]);
	// }}} -->
	// Temp. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Temps',
		'Fan temp (°C)',
		'R0 (kΩ) or a (1000)',
		'R1 (kΩ) or b (1000)',
		'Rc (kΩ)',
		'Tc (°C)',
		'β (1) or NaN'
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
		'Temerature above which the fan is turned on.',
		'Resistance on the board in series with the thermistor.  Normally 4.7 or 10.  Or, if β is NaN, the result is ax+b with x the measured ADC value; this value is a/1000.',
		'Resistance on the board in parallel with the thermistor.  Normally Infinity.  Or, if β is NaN, the result is ax+b with x the measured ADC value; this value is b/1000.',
		'Calibrated resistance of the thermistor.  Normally 100 for extruders, 10 for the heated bed.',
		'Temperature at which the thermistor has value Rc.  Normally 20.',
		"Temperature dependence of the thermistor.  Normally around 3800.  It can be found in the thermistor's data sheet.  Or, if β is NaN, the result is ax+b with x the measured ADC value."
	]).AddMultiple('temp', Temp_setup)]);
	// }}}
	// Pins. {{{
	var pins = setup.Add(make_table(Pin('LED', [null, 'led_pin']), Pin('Probe', [null, 'probe_pin'])));
	pins.AddMultiple('space', Pins_space, false);
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

function NoPrinter() { // {{{
	var ret = Create('div', 'noprinter notconnected setup hidden');
	ret.id = make_id({'port': port}, [null, 'nocontainer']);
	var blocker = ret.AddElement('div', 'hidden blocker');
	blocker.id = make_id({'port': port}, [null, 'block2']);
	ret.AddElement('h2').AddText('No printer is found on port ' + port + '.');
	var detect = ret.AddElement('p').AddText('If autodetect does not work, you can request to detect a printer:').AddElement('button').AddText('Detect');
	detect.type = 'button';
	detect.port = port;
	detect.AddEvent('click', function() { rpc.call('detect', [this.port], {}); });
	ret.AddElement('p').AddText('Or you can upload the firmware that fits your hardware.').Add(upload_buttons(port, [['melzi', 'atmega1284p (Melzi, Sanguinololu)'], ['ramps', 'atmega2560 (Ramps)'], ['mega', 'atmega1280'], ['mini', 'atmega328 (Uno)']));
	var message = ret.AddElement('div', 'message');
	message.id = make_id({'port': port}, [null, 'message2']);
	return ret;
}
// }}}

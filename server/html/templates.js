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
	var input = Create('input', 'title').AddClass(className);
	input.type = 'text';
	input.id = make_id(printer, obj, 'source');
	var button = Create('button', 'button title').AddClass(className);
	button.AddEvent('click', function() { set_value(this.printer, this.obj, document.getElementById(this.source).value); });
	button.AddText('Set');
	button.type = 'button';
	button.source = input.id;
	button.obj = input;
	return [span, input, button];
}

function Pin(title, obj, only_analog) {
	var pinselect = Create('select', 'pinselect');
	pinselect.id = make_id(printer, obj);
	pinselect.obj = obj;
	pinselect.printer = printer;
	pinselect.Add(pinrange());
	var validinput = Create('input');
	validinput.type = 'checkbox';
	validinput.id = make_id(printer, obj, 'valid');
	var validlabel = Create('label').AddText('Valid');
	validlabel.htmlFor = validinput;
	var invertedinput = Create('input');
	invertedinput.type = 'checkbox';
	invertedinput.id = make_id(printer, obj, 'inverted');
	var invertedlabel = Create('label').AddText('Inverted');
	invertedlabel.htmlFor = invertedinput;
	var button = Create('button', 'button');
	button.type = 'button';
	button.AddText('Set');
	button.printer = printer;
	button.AddEvent('click', function() { set_pin(this.printer, obj); });
	return make_tablerow(title, [[pinselect], [validinput, validlabel], [invertedinput, invertedlabel], [button]], ['pintitle', 'pinvalue']);
}

function Float(obj, factor, className, set) {
	var input = Create('input', className);
	var button = Create('button', className).AddText('Set');
	var span = Create('span', className);
	input.obj = obj;
	button.obj = obj;
	if (factor === undefined)
		factor = 1;
	input.factor = factor;
	button.factor = factor;
	span.factor = factor;
	input.id = make_id(printer, obj, 'new');
	span.id = make_id(printer, obj);
	button.source = input;
	input.type = 'text';
	input.set = set;
	input.printer = printer;
	input.AddEvent('keydown', function(event) { floatkey(event, this); });
	button.AddEvent('click', function() { floatkey({keyCode: 13, preventDefault: function() {}}, this.source); });
	return [input, button, span];
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
	div.Add(Float([['space', num], 'max_deviation'], 1e-3));
	return make_tablerow(space_name(num), [[select, button, span], div], ['rowtitle1']);
}
// }}}


// Space. {{{
function Extruder(num) {
	var e = ['dx', 'dy', 'dz'];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['space', num], e[i]], 1));
		e[i] = div;
	}
	return make_tablerow(space_name(num), e, ['rowtitle3'], undefined, TYPE_EXTRUDER, num);
}

function Cartesian(num) {
	return make_tablerow(space_name(num), [Float([['space', num], 'num_axes'], 1)], ['rowtitle1'], undefined, TYPE_CARTESIAN, num);
}

function Delta_motor(space, motor) {
	var e = ['delta_axis_min', 'delta_axis_max', 'delta_rodlength', 'delta_radius'];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['motor', [space, motor]], e[i]], 1e-3));
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
	div.Add(Float([['space', num], 'delta_angle'], Math.PI / 180));
	return make_tablerow(space_name(num), [div], ['rowtitle1'], undefined, TYPE_DELTA, num);
}

function Axis_axis(space, axis) {
	var e = [['park', 1e-3], ['park_order', 1], ['max_v', 1e-3], ['min', 1e-3], ['max', 1e-3]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['axis', [space, axis]], e[i][0]], e[i][1]));
		e[i] = div;
	}
	return make_tablerow(axis_name(space, axis), e, ['rowtitle5']);
}

function Axis(num, dummy, table) {
	table.AddMultiple('axis', Axis_axis, true, num);
	return [];
}

function Motor_motor(space, motor) {
	var e = [['steps_per_m', 1e3], ['max_steps', 1], ['home_pos', 1e-3], ['home_order', 1], ['limit_v', 1e-3], ['limit_a', 1]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['motor', [space, motor]], e[i][0]], e[i][1]));
		e[i] = div;
	}
	return make_tablerow(motor_name(space, motor), e, ['rowtitle6']);
}


function Motor(num, dummy, table) {
	table.AddMultiple('motor', Motor_motor, true, num);
	return [];
}

function Move_axis(space, axis) {
	var input = Create('input');
	input.type = 'text';
	input.id = make_id(printer, [['axis', [space, axis]], 'goto']);
	var button = Create('button').AddText('Go');
	button.printer = printer;
	button.index = [space, axis];
	button.AddEvent('click', function() {
		var target = new Object;
		target[String(this.index[1])] = Number(get_element(this.printer, [['axis', this.index], 'goto']).value) / 1000.;
		var fulltarget = new Object;
		fulltarget[String(this.index[0])] = target;
		this.printer.call('goto', [fulltarget], {cb: true});
		this.printer.call('wait_for_cb', [], {}, update_canvas_and_spans);
	});
	var offset = Create('div');
	offset.Add(Float([['axis', [space, axis]], 'offset'], 1e-3));
	return make_tablerow(axis_name(space, axis), [[input, button], offset], ['rowtitle3', '']);
}

function Move(num, dummy, table) {
	table.AddMultiple('axis', Move_axis, false, num);
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
	var e = [['R0', 1e3], ['R1', 1e3], ['Rc', 1e3], ['Tc', 1], ['beta', 1]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float([['temp', num], e[i][0]], e[i][1]));
		e[i] = div;
	}
	return make_tablerow(temp_name(printer, num), e, ['rowtitle5']);
}

function Temp(num) {
	var current = Create('div');
	var button = current.AddElement('button').AddText('Refresh');
	button.num = num;
	button.printer = printer;
	if (num !== null)
		printer.temptargets.push(current.AddElement('span'));
	button.AddEvent('click', function() { update_temps(this.printer, this.num); });
	var div = Create(div);
	div.Add(Float([['temp', num], 'value', 'settemp']));
	return make_tablerow(temp_name(printer, num), [div, current], ['rowtitle2']);
}

function Pins_temp(num, dummy, table) {
	var e = [['Power', 'power'], ['Thermistor', 'thermistor']];
	for (var i = 0; i < e.length; ++i)
		e[i] = Pin(e[i][0], [['temp', num], e[i][1] + '_pin']);
	return make_pin_title(temp_name(num), e);
}
// }}}

// Gpio. {{{
function Gpio_setup(num) {
	var select = Create('select');
	select.Add(temprange());
	select.id = make_id(printer, [['gpio', num], 'master']);
	var button = Create('button', 'button').AddText('Set');
	button.type = 'button';
	button.index = num;
	button.printer = printer;
	button.AddEvent('click', function() { set_gpio_master(this.printer, this.index); });
	var value = Float([['gpio', num], 'value'], 1);
	return make_tablerow(gpio_name(num), [[select, button], value], ['rowtitle2'], [['gpio', num], 'settings']);
}

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
	if (printer) {
		// TODO: profile selection.
		return ret.AddText(printer.name);
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
	b = e.AddElement('button').AddText('Home').AddEvent('click', function() { this.printer.call('home', [], {}); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Pause').AddEvent('click', function() { this.printer.call('pause', [true], {}); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Resume').AddEvent('click', function() { this.printer.call('pause', [false], {}); });
	b.type = 'button';
	b.printer = printer;
	b = e.AddElement('button').AddText('Sleep').AddEvent('click', function() { this.printer.call('sleep', [], {}); });
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
		Float([null, 'currentx'], 1e-3, '', function(v) { b.printer.call('goto', [[{0: v}]], {cb: true}); b.printer.call('wait_for_cb', [], {}, update_canvas_and_spans); }),
		Float([null, 'currenty'], 1e-3, '', function(v) { b.printer.call('goto', [[{1: v}]], {cb: true}); b.printer.call('wait_for_cb', [], {}, update_canvas_and_spans); }),
		Float([null, 'currentz'], 1e-3, '', function(v) { b.printer.call('goto', [[{2: v}]], {cb: true}); b.printer.call('wait_for_cb', [], {}, update_canvas_and_spans); }),
		b,
		'',
		['Z Offset:', Float([null, 'zoffset'], 1e-3), 'mm']
	], ['', '', '', '', '', '']));
	// Target position buttons.
	var b = Create('button').AddText('Use current').AddEvent('click', function() {
		for (var i = 0; i < (GetElement(this.printer, [null, 'zlock']).checked ? 2 : 3); ++i)
			target[i] = current[i];
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
		Float([null, 'targetx'], 1e-3),
		Float([null, 'targety'], 1e-3),
		Float([null, 'targetz'], 1e-3),
		b,
		[c, l],
		['Angle:', Float([null, 'targetangle'], Math.PI / 180), '°']
	], ['', '', '', '', '', '']));
	// Canvas for xy and for z.
	ret.AddElement('canvas', 'xymap').id = make_id(printer, [null, 'xymap']);
	ret.AddElement('canvas', 'zmap').id = make_id(printer, [null, 'zmap']);
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
	e.Add(Float([null, 'feedrate'], 1e-2));
	e.AddText(' %');
	ret.AddMultiple('axis', function(i) {
		var e = Create('div').AddText(axis_name(1, i) + ': ');
		e.Add(Float([['axis', i], 'multiplier'], 1e-2));
		e.AddText(' %');
		return e;
	}, true, 1);
	return ret;
}
// }}}

function Gpios() { // {{{
	var ret = Create('div', 'gpios');
	var p = printer;
	ret.AddMultiple('gpio', function(i) {
		var ret = Create('span');
		var input = ret.AddElement('input');
		input.type = 'checkbox';
		input.id = make_id(p, [['gpio', i], 'on']);
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
	ret.AddElement('div', 'message').id = make_id(printer, [null, 'message1']);
	// Setup. {{{
	var setup = ret.AddElement('div', 'setup');
	setup.AddElement('div').Add(Text('Name', [null, 'name']));
	// TODO: Profile copy+remove.
	var e = setup.AddElement('div').AddText('Motor Timeout:');
	e.Add(Float([null, 'motor_limit'], 60));
	e.AddText(' min');
	e = setup.AddElement('div').AddText('Temp Timeout:');
	e.Add(Float([null, 'temp_limit'], 60));
	e.AddText(' min');
	e = setup.AddElement('div').AddText('Max Probe Distance:');
	e.Add(Float([null, 'probe_dist'], 1e-3));
	e.AddText(' mm');
	e = setup.AddElement('div').AddText('Probe Safe Retract Distance:');
	e.Add(Float([null, 'probe_safe_dist'], 1e-3));
	e.AddText(' mm');
	e = setup.AddElement('div').AddText('ID of Bed Temp (255 for None):').Add(Float([null, 'bed_id']));
	e = setup.AddElement('div').AddText('Spaces:').Add(Float([null, 'num_spaces']));
	e = setup.AddElement('div').AddText('Temps:').Add(Float([null, 'num_temps']));
	e = setup.AddElement('div').AddText('Gpios:').Add(Float([null, 'num_gpios']));
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
		'R0 (kΩ) or a (1000)',
		'R1 (kΩ) or b (1000)',
		'Rc (kΩ)',
		'Tc (°C)',
		'β (1) or NaN'
	], [
		'htitle5',
		'title5',
		'title5',
		'title5',
		'title5',
		'title5'
	], [
		null,
		'Resistance on the board in series with the thermistor.  Normally 4.7 or 10.  Or, if β is NaN, the result is ax+b with x the measured ADC value; this value is a/1000.',
		'Resistance on the board in parallel with the thermistor.  Normally Infinity.  Or, if β is NaN, the result is ax+b with x the measured ADC value; this value is b/1000.',
		'Calibrated resistance of the thermistor.  Normally 100 for extruders, 10 for the heated bed.',
		'Temperature at which the thermistor has value Rc.  Normally 20.',
		"Temperature dependence of the thermistor.  Normally around 3800.  It can be found in the thermistor's data sheet.  Or, if β is NaN, the result is ax+b with x the measured ADC value."
	]).AddMultiple('temp', Temp_setup)]);
	// }}}
	// Gpio. {{{
	setup.Add([make_table().AddMultipleTitles([
		'Gpio',
		'Master temp',
		'Threshold(°C)'
	], [
		'htitle2',
		'title2',
		'title2'
	], [
		null,
		'If set to something other than None, the pin will automatically be set and reset depending on the value of the selected temperature control.',
		'If a master temp is set, the pin will be on if that temp is above this value, otherwise it will be off.'
	]).AddMultiple('gpio', Gpio_setup)]);
	// }}}
	// Pins. {{{
	var pins = setup.Add(make_table(Pin('LED', [null, 'led_pin']), Pin('Probe', [null, 'probe_pin'])));
	pins.AddMultiple('space', Pins_space, false);
	pins.AddMultiple('temp', Pins_temp, false);
	pins.AddMultiple('gpio', Pins_gpio, false);
	// }}}
	// Save and restore. {{{
	//ret.Add(Button('Save settings to EEPROM', 'save'));
	//ret.Add(Button('Restore settings from EEPROM', 'load'));
	setup.AddElement('div').Add(File([null, 'import', 'import_settings'], 'Import'));
	e = setup.AddElement('a', 'title').AddText('Export settings to file');
	e.id = make_id(printer, [null, 'export']);
	e.title = 'Save settings to disk.';
	// }}}
	var disable = setup.AddElement('div').AddElement('button').AddText('Disable Printer');
	disable.port = port;
	disable.type = 'button';
	disable.AddEvent('click', function() { rpc.call('disable', [this.port], {}); });
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
	ret.AddElement('p').AddText('Or you can upload the firmware that fits your hardware.').Add(upload_buttons(port, [['melzi0', 'mega1284p (Melzi, Sanguinololu)'], ['melzi1', 'mega1284p using serial port 1'], ['ramps', 'mega2560 (Ramps)'], ['mega', 'mega1280']]));
	var message = ret.AddElement('div', 'message');
	message.id = make_id({'port': port}, [null, 'message2']);
	return ret;
}
// }}}

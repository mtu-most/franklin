// vim: set foldmethod=marker :

// {{{ Global variables.
var global;
var active_printer;
var rpc;
// }}}

// {{{ Initialization.
function init ()
{
	rpc = Rpc (null, setup, function () { alert ('connection to server lost'); });
	var proto = Object.prototype;
	proto.Add = function (object, className) { this.appendChild (object); if (className) object.className = className; return object; }
	proto.AddElement = function (name, className) { var element = document.createElement (name); return this.Add (element, className); }
	proto.AddText = function (text) { var t = document.createTextNode (text); return this.Add (t); }
	proto.clearAll = function () { while (this.firstChild) this.removeChild (this.firstChild); }
	proto.AddClass = function (className) {
		var classes = this.className.split (' ');
		if (classes.indexOf (className) >= 0)
			return;
		classes.push (className);
		this.className = classes.join (' ');
	}
	proto.RemoveClass = function (className) {
		var classes = this.className.split (' ');
		var pos = classes.indexOf (className);
		if (pos < 0)
			return;
		classes.splice (pos, 1);
		this.className = classes.join (' ');
	}
}

function setup ()
{
	//setInterval (function () { rpc.call ('status', [], {}, set_status); }, 5000);
	global = Global ();
	document.getElementById ('container').Add (global);
}
// }}}

// {{{ Builders.
function Constant (printer, key, name, cmd) { // {{{
	var constant = document.createElement ('tr');
	cmd.push (printer.get (null, key, null, function (value) {
		printer.constants[key] = value;
		constant.AddElement ('td').AddText (name);
		constant.AddElement ('td').AddText (value);
	}));
	return constant;
} // }}}
function NumFunction (printer, name, object, part, index, negative) { // {{{
	var func = document.createElement ('tr');
	func.AddElement ('td').AddText (name);
	var td = func.AddElement ('td');
	var checkbox = td.AddElement ('input');
	checkbox.type = 'checkbox';
	if (negative) {
		td.AddText ('-');
		var checkbox2 = td.AddElement ('input');
		checkbox2.type = 'checkbox';
		td.AddText ('+');
		checkbox.onclick = function () {
			checkbox2.checked = false;
			var value = Number (checkbox.checked ? -func.entry.value : Number.NaN);
			printer.call (object ? part + '_' + object : part, object ? [index, value] : [value], {}, null);
		};
		checkbox2.onclick = function () {
			checkbox.checked = false;
			var value = Number (checkbox2.checked ? func.entry.value : Number.NaN);
			printer.call (object ? part + '_' + object : part, object ? [index, value] : [value], {}, null);
		};
	}
	else {
		checkbox.onclick = function () {
			var value = Number (checkbox.checked ? func.entry.value : Number.NaN);
			printer.call (object ? part + '_' + object : part, object ? [index, value] : [value], {}, null);
		};
	}
	func.entry = func.AddElement ('input');
	func.entry.value = 'Infinity';
	func.entry.type = 'text';
	return func;
} // }}}
function FileFunction (printer, name, object, part, index, send_name) { // {{{
	var func = document.createElement ('tr');
	func.AddElement ('td').AddText (name);
	var td = func.AddElement ('td');
	if (send_name) {
		func.entry = td.AddElement ('input');
		func.entry.type = 'text';
	}
	func.fileselect = td.AddElement ('input');
	func.fileselect.type = 'file';
	var button = td.AddElement ('button');
	button.type = 'button';
	button.AddText ('Send');
	button.onclick = function () {
		if (send_name && !func.entry.value) {
			alert ('Please select a name');
			return;
		}
		if (func.fileselect.files.length < 1) {
			alert ('Please select a file');
			return;
		}
		var reader = new FileReader ();
		reader.onload = function (e) {
			if (e.target.readyState == FileReader.DONE) {
				if (send_name)
					printer.call (object ? part + '_' + object : part, object ? [index, func.entry.value, e.target.result] : [func.entry.value, e.target.result], {}, null);
				else
					printer.call (object ? part + '_' + object : part, object ? [index, e.target.result] : [e.target.result], {}, null);
			}
		}
		reader.readAsBinaryString (func.fileselect.files[0]);
	};
	return func;
} // }}}
function Button (printer, name, object, part, index) { // {{{
	var tr = document.createElement ('tr');
	var td = tr.AddElement ('td');
	td.colSpan = 2;
	var button = td.AddElement ('button');
	button.type = 'button';
	button.AddText (name);
	button.onclick = function () {
		printer.call (object ? part + '_' + object : part, object ? [index] : [], {}, null);
	};
	return tr;
} // }}}
function Checkbox (printer, name, object, part, index) { // {{{
	var tr = document.createElement ('tr');
	tr.AddElement ('td').AddText (name);
	var checkbox = tr.AddElement ('td').AddElement ('input');
	checkbox.type = 'checkbox';
	checkbox.onclick = function () {
		printer.call (object ? part + '_' + object : part, object ? [index, this.checked] : [this.checked], {}, null);
	};
	return tr;
} // }}}
function Text (printer, name, object, part, index) { // {{{
	var text = document.createElement ('tr');
	text.AddElement ('td').AddText (name);
	text.entry = text.AddElement ('td').AddElement ('input');
	text.entry.type = 'text';
	text.refresh = function (cmd) {
		cmd.push (printer.get (object, part, index, function (value) {
			text.entry.value = String (value);
		}));
	};
	var button = td.AddElement ('button');
	button.type = 'button';
	button.AddText ('Set');
	button.onclick = function () {
		printer.set (object, part, index, text.entry.value);
	};
	return text;
} // }}}
function Range (printer, name, object, part, index, max) { // {{{
	var range = document.createElement ('tr');
	range.AddElement ('td').AddText (name);
	var td = range.AddElement ('td');
	range.select = td.AddElement ('select');
	range.refresh = function (cmd) {
		cmd.push (printer.get (object, part, index, function (value) {
			range.value = Number (value);
			if (range.value >= max || range.value < 0)
				range.value = 0;
			range.select.clearAll ();
			for (var o = 0; o < max; ++o) {
				var option = range.select.AddElement ('option');
				option.value = o;
				option.AddText (String (o));
				if (range.value == o)
					option.selected = 'selected';
			}
		}));
	};
	var button = td.AddElement ('button');
	button.type = 'button';
	button.AddText ('Set');
	button.onclick = function () {
		printer.set (object, part, index, this.value);
	};
	return range;
} // }}}
function Choice (printer, name, object, part, index, list) { // {{{
	var choice = document.createElement ('tr');
	choice.AddElement ('td').AddText (name);
	var td = choice.AddElement ('td');
	choice.select = td.AddElement ('select');
	choice.refresh = function (cmd) {
		cmd.push (printer.get (object, part, index, function (value) {
			choice.value = Number (value);
			if (choice.value >= list.length || choice.value < 0)
				choice.value = 0;
			choice.select.clearAll ();
			for (var o = 0; o < list.length; ++o) {
				var option = choice.select.AddElement ('option');
				option.value = o;
				option.AddText (list[o]);
				if (choice.value == o)
					option.selected = 'selected';
			}
		}));
	};
	var button = td.AddElement ('button');
	button.type = 'button';
	button.AddText ('Set');
	button.onclick = function () {
		printer.set (object, part, index, this.value);
	};
	return choice;
} // }}}
function Float (printer, name, object, part, index, factor) { // {{{
	var the_float = document.createElement ('tr');
	the_float.AddElement ('td').AddText (name);
	var td = the_float.AddElement ('td');
	the_float.entry = td.AddElement ('input');
	the_float.entry.type = 'text';
	the_float.factor = factor ? factor : 1;
	the_float.refresh = function (cmd) {
		cmd.push (printer.get (object, part, index, function (value) {
			the_float.value = Number (value);
			the_float.entry.value = String (Number (value) / the_float.factor);
		}));
	};
	var button = td.AddElement ('button');
	button.type = 'button';
	button.AddText ('Set');
	button.onclick = function () {
		printer.set (object, part, index, the_float.value * the_float.factor);
	};
	return the_float;
} // }}}
function Pin (printer, name, object, part, index, analog) { // {{{
	var pin = document.createElement ('tr');
	pin.AddElement ('td').AddText (name);
	var td = pin.AddElement ('td');
	pin.select = td.AddElement ('select');
	pin.invalid = td.AddElement ('input');
	pin.invalid.type = 'checkbox';
	td.AddText ('Invalid');
	if (!analog) {
		var invertbox = td.AddElement ('span');
		pin.inverted = invertbox.AddElement ('input');
		pin.inverted.type = 'checkbox';
		invertbox.AddText ('Inverted');
	}
	pin.refresh = function (cmd) {
		cmd.push (printer.get (object, part, index, function (value) {
			pin.value = Number (value) & 0xff;
			var is_invalid = Boolean (Number (value) & 0x100);
			var is_inverted = Boolean (Number (value) & 0x200);
			pin.invalid.checked = is_invalid ? 'checked' : null;
			if (!analog)
				pin.inverted.checked = pin.is_inverted ? 'checked' : null;
			pin.select.clearAll ();
			var found = false;
			for (var o = 0; o < (analog ? printer.constants.num_pins - printer.constants.num_digital_pins : printer.constants.num_pins); ++o) {
				var option = pin.select.AddElement ('option');
				option.value = o;
				if (analog) {
					option.AddText ('A' + String (o));
				}
				else {
					if (o < printer.constants.num_digital_pins)
						option.AddText ('D' + String (o));
					else
						option.AddText ('A' + String (o - printer.constants.num_digital_pins));
				}
				if (pin.value == o) {
					option.selected = 'selected';
					found = true;
				}
			}
			if (!found) {
				var option = pin.select.AddElement ('option');
				option.value = -1;
				option.AddText ('?');
				option.selected = 'selected';
			}
		}));
	};
	var button = td.AddElement ('button');
	button.type = 'button';
	button.AddText ('Set');
	button.onclick = function () {
		printer.set (object, part, index, Number (pin.select.value) + (pin.invalid.checked ? 0x100 : 0) + (pin.inverted.checked ? 0x200 : 0));
	};
	return pin;
} // }}}
function Motor (printer, object, index) { // {{{
	var motor = document.createElement ('tr');
	motor.className = 'motor';
	var box = motor.AddElement ('td');
	box.colSpan = 2;
	var table = box.AddElement ('table');
	motor.step_pin = table.Add (Pin (printer, 'Step pin', object + '_motor', 'step_pin', index), 'pin');
	motor.dir_pin = table.Add (Pin (printer, 'Direction pin', object + '_motor', 'dir_pin', index), 'pin');
	motor.enable_pin = table.Add (Pin (printer, 'Enable pin', object + '_motor', 'enable_pin', index), 'pin');
	motor.steps_per_mm = table.Add (Float (printer, 'Steps per mm', object + '_motor', 'steps_per_mm', index), 'setting');
	motor.max_v_neg = table.Add (Float (printer, 'Max speed (-) [mm/s]', object + '_motor', 'max_v_neg', index), 'setting');
	motor.max_v_pos = table.Add (Float (printer, 'Max speed (+) [mm/s]', object + '_motor', 'max_v_pos', index), 'setting');
	motor.max_a = table.Add (Float (printer, 'Max acceleration [mm/s²]', object + '_motor', 'max_a', index), 'setting');
	motor.refresh = function (cmd) {
		this.step_pin.refresh (cmd);
		this.dir_pin.refresh (cmd);
		this.enable_pin.refresh (cmd);
		this.steps_per_mm.refresh (cmd);
		this.max_v_pos.refresh (cmd);
		this.max_v_neg.refresh (cmd);
		this.max_a.refresh (cmd);
	};
	var buttonbox = table.AddElement ('tr', 'setting').AddElement ('td');
	buttonbox.colSpan = 2;
	var button = buttonbox.AddElement ('button');
	button.type = 'button';
	button.AddText ('Setup Motor');
	button.onclick = function () {
		this.step_pin.set ();
		this.dir_pin.set ();
		this.enable_pin.set ();
		this.steps_per_mm.set ();
		this.max_v_pos.set ();
		this.max_v_neg.set ();
		this.max_a.set ();
	};
	table.Add (NumFunction (printer, 'Run', object, 'run', index, true), 'action');
	table.Add (Button (printer, 'Sleep', object, 'sleep', index), 'action');
	return motor;
} // }}}
function Temp (printer, name, object, index) { // {{{
	var temp, table;
	if (name != null) {
		temp = document.createElement ('td', 'temp box inactive');
		table = temp.AddElement ('table');
		var th = table.AddElement ('tr').AddElement ('th');
		th.colSpan = 2;
		th.AddText ('Temp ' + String (name));
	}
	else {
		temp = document.createElement ('tr');
		temp.className = 'temp';
		var td = temp.AddElement ('td');
		td.colSpan = 2;
		table = td.AddElement ('table');
	}
	var settempname;
	if (object) {
		settempname = object;
		object += '_temp';
	}
	else {
		object = 'temp';
		settempname = object;
	}
	temp.power_pin = table.Add (Pin (printer, 'Power pin', object, 'power_pin', index), 'pin');
	temp.thermistor_pin = table.Add (Pin (printer, 'Thermistor pin', object, 'thermistor_pin', index, true), 'pin');
	temp.alpha = table.Add (Float (printer, 'α [1]', object, 'alpha', index), 'future');
	temp.beta = table.Add (Float (printer, 'β [K]', object, 'beta', index), 'future');
	temp.core_C = table.Add (Float (printer, 'Core C [J/K]', object, 'core_C', index), 'future');
	temp.shell_C = table.Add (Float (printer, 'Shell C [J/K]', object, 'shell_C', index), 'future');
	temp.transfer = table.Add (Float (printer, 'Transfer [W/K]', object, 'transfer', index), 'future');
	temp.radiation = table.Add (Float (printer, 'Radiation [W/K⁴]', object, 'radiation', index), 'future');
	temp.power = table.Add (Float (printer, 'Power [W]', object, 'power', index), 'future');
	temp.refresh = function (cmd) {
		if (name != null) {
			if (index < printer.num_temps.value)
				temp.className = 'temp box active';
			else
				temp.className = 'temp box inactive';
		}
		this.power_pin.refresh (cmd);
		this.thermistor_pin.refresh (cmd);
		this.alpha.refresh (cmd);
		this.beta.refresh (cmd);
		this.core_C.refresh (cmd);
		this.shell_C.refresh (cmd);
		this.transfer.refresh (cmd);
		this.radiation.refresh (cmd);
		this.power.refresh (cmd);
	};
	var td = table.AddElement ('tr', 'setting').AddElement ('td');
	td.colSpan = 2;
	var button = td.AddElement ('button');
	button.type = 'button';
	button.AddText ('Setup temp');
	button.onclick = function () {
		this.power_pin.set ();
		this.thermistor_pin.set ();
		this.alpha.set ();
		this.beta.set ();
		this.core_C.set ();
		this.shell_C.set ();
		this.transfer.set ();
		this.radiation.set ();
		this.power.set ();
	};
	table.Add (NumFunction (printer, 'Enable', settempname, 'settemp', index), 'action');
	// TODO: Read
	return temp;
} // }}}
function Axis (printer, name, index) { // {{{
	var axis = document.createElement ('td');
	axis.className = 'axis box inactive';
	var table = axis.AddElement ('table');
	var th = table.AddElement ('tr').AddElement ('th');
	th.colSpan = 2;
	th.AddText (name < 3 ? 'Axis ' + String (name) + ': ' + String.fromCharCode ('X'.charCodeAt (0) + name) : 'Axis ' + String (name));
	axis.motor = table.Add (Motor (printer, 'axis', index));
	axis.limit_min_pin = table.Add (Pin (printer, 'Min limit pin', 'axis', 'limit_min_pin', index), 'pin');
	axis.limit_max_pin = table.Add (Pin (printer, 'Max limit pin', 'axis', 'limit_max_pin', index), 'pin');
	axis.sense_pin = table.Add (Pin (printer, 'Sense pin', 'axis', 'sense_pin', index), 'pin');
	axis.limit_min_pos = table.Add (Float (printer, 'Min limit position', 'axis', 'limit_min_pos', index), 'setting');
	axis.limit_max_pos = table.Add (Float (printer, 'Max limit position', 'axis', 'limit_max_pos', index), 'setting');
	axis.delta_length = table.Add (Float (printer, 'Delta rod length', 'axis', 'delta_length', index), 'setting');
	axis.delta_radius= table.Add (Float (printer, 'Delta radius', 'axis', 'delta_radius', index), 'setting');
	axis.offset = table.Add (Float (printer, 'Offset', 'axis', 'offset', index), 'useful');
	axis.refresh = function (cmd) {
		if (index < printer.num_axes.value)
			axis.className = 'axis box active';
		else
			axis.className = 'axis box inactive';
		this.motor.refresh (cmd);
		this.limit_min_pin.refresh (cmd);
		this.limit_max_pin.refresh (cmd);
		this.sense_pin.refresh (cmd);
		this.limit_min_pos.refresh (cmd);
		this.limit_max_pos.refresh (cmd);
		this.delta_length.refresh (cmd);
		this.delta_radius.refresh (cmd);
		this.offset.refresh (cmd);
	};
	var td = table.AddElement ('tr', 'setting').AddElement ('td');
	td.colSpan = 2;
	var button = td.AddElement ('button');
	button.type = 'button';
	button.AddText ('Setup Axis');
	button.onclick = function () {
		this.motor.set ();
		this.limit_min_pin.set ();
		this.limit_max_pin.set ();
		this.sense_pin.set ();
		this.limit_min_pos.set ();
		this.limit_max_pos.set ();
		this.delta_length.set ();
		this.delta_radius.set ();
		this.offset.set ();
	};
	// TODO: get sense	textbox and clear
	return axis;
} // }}}
function Extruder (printer, name, index) { // {{{
	var extruder = document.createElement ('td');
	extruder.className = 'extruder box inactive';
	var table = extruder.AddElement ('table');
	var th = table.AddElement ('tr').AddElement ('th');
	th.colSpan = 2;
	th.AddText ('Extruder ' + String (name));
	extruder.motor = table.Add (Motor (printer, 'extruder', index));
	extruder.temp = table.Add (Temp (printer, null, 'extruder', index));
	extruder.filament_heat = table.Add (Float (printer, 'Filament heat [J/mm]', 'extruder', 'filament_heat', index), 'future');
	extruder.nozzle_size = table.Add (Float (printer, 'Nozzle diameter [mm]', 'extruder', 'nozzle_size', index), 'future');
	extruder.filament_size = table.Add (Float (printer, 'Filament diameter [mm]', 'extruder', 'filament_size', index), 'future');
	extruder.refresh = function (cmd) {
		if (index < printer.num_extruders.value)
			extruder.className = 'extruder box active';
		else
			extruder.className = 'extruder box inactive';
		this.motor.refresh (cmd);
		this.temp.refresh (cmd);
		this.filament_heat.refresh (cmd);
		this.nozzle_size.refresh (cmd);
		this.filament_size.refresh (cmd);
	};
	var td = table.AddElement ('tr', 'setting').AddElement ('td');
	td.colSpan = 2;
	var button = td.AddElement ('button');
	button.type = 'button';
	button.AddText ('Setup Extruder');
	button.onclick = function () {
		this.motor.set ();
		this.temp.set ();
		this.filament_heat.set ();
		this.nozzle_size.set ();
		this.filament_size.set ();
	};
	return extruder;
} // }}}
function Printer (port) { // {{{
	var printer = document.createElement ('table');
	printer.className = 'printer hidden';
	printer.call = function (name, a, ka, cb) {
		if (active_printer != printer)
			rpc.call ('set_printer', [null, this.port], {}, function () { active_printer = printer; rpc.call (name, a, ka, cb); });
		else
			rpc.call (name, a, ka, cb);
	};
	printer.get = function (object, part, index, cb) {
		if (object)
			return [object + '_get_' + part, [index], {}, cb];
		else
			return ['get_' + part, [], {}, cb];
	};
	printer.set = function (object, part, index, value) {
		if (object)
			this.call (object + '_set_' + part, [index, value], {}, null);
		else
			this.call ('set_' + part, [value], {}, null);
	};
	printer.constants = {};
	var cmd = [];
	printer.namelen = printer.Add (Constant (printer, 'namelen', 'Name length [B]', cmd), 'constants');
	printer.maxaxes = printer.Add (Constant (printer, 'maxaxes', 'Maximum number of axes', cmd), 'constants');
	printer.maxextruders = printer.Add (Constant (printer, 'maxextruders', 'Maximum number of extruders', cmd), 'constants');
	printer.maxtemps = printer.Add (Constant (printer, 'maxtemps', 'Maximum number of temps', cmd), 'constants');
	printer.audio_fragments = printer.Add (Constant (printer, 'audio_fragments', 'Number of audio fragments', cmd), 'constants audio');
	printer.audio_fragment_size = printer.Add (Constant (printer, 'audio_fragment_size', 'Audio fragment size [B]', cmd), 'constants audio');
	printer.num_digital_pins = printer.Add (Constant (printer, 'num_digital_pins', 'Number of digital pins', cmd), 'constants');
	printer.num_pins = printer.Add (Constant (printer, 'num_pins', 'Total number of pins', cmd), 'constants');

	printer.refresh = function () {
		var cmd = [];
		this.name.refresh (cmd);
		this.num_axes.refresh (cmd);
		this.num_extruders.refresh (cmd);
		this.num_temps.refresh (cmd);
		this.type.refresh (cmd);
		this.led_pin.refresh (cmd);
		this.room_T.refresh (cmd);
		this.motor_limit.refresh (cmd);
		this.temp_limit.refresh (cmd);
		this.feedrate.refresh (cmd);
		rpc.multicall (cmd, function () {
			var cmd = [];
			for (var a = 0; a < printer.constants.maxaxes; ++a)
				printer.axes[a].refresh (cmd);
			for (var e = 0; e < printer.constants.maxextruders; ++e)
				printer.extruders[e].refresh (cmd);
			for (var t = 0; t < printer.constants.maxtemps; ++t)
				printer.temps[t].refresh (cmd);
			rpc.multicall (cmd, null);
		});
	};
	printer.axes = [];
	printer.extruders = [];
	printer.temps = [];
	rpc.multicall (cmd, function () {
		printer.name = printer.Add (Text (printer, 'Name', null, 'name', null), 'setting');
		printer.num_axes = printer.Add (Range (printer, 'Number of axes', null, 'num_axes', null, printer.constants.maxaxes), 'setting');
		printer.num_extruders = printer.Add (Range (printer, 'Number of extruders', null, 'num_extruders', null, printer.constants.maxextruders), 'setting');
		printer.num_temps = printer.Add (Range (printer, 'Number of temps', null, 'num_temps', null, printer.constants.maxtemps), 'setting');
		printer.type = printer.Add (Choice (printer, 'Printer type', null, 'printer_type', null, ['Cartesian', 'Delta']), 'setting');
		printer.led_pin = printer.Add (Pin (printer, 'Led pin', null, 'led_pin', null), 'pin');
		printer.room_T = printer.Add (Float (printer, 'Room temperature [°C]', null, 'room_T', null), 'future');
		printer.motor_limit = printer.Add (Float (printer, 'Maximum idle time for motors [s]', null, 'motor_limit', null, 1000), 'setting');
		printer.temp_limit = printer.Add (Float (printer, 'Maximum idle time for temps [s]', null, 'temp_limit', null, 1000), 'setting');
		printer.feedrate = printer.Add (Float (printer, 'Command feed factor', null, 'feedrate', null), 'useful');

		// TODO: goto		special
		printer.Add (Button (printer, 'Load all', null, 'load_all', null), 'action');
		printer.Add (Button (printer, 'Save all', null, 'save_all', null), 'action');
		printer.Add (Checkbox (printer, 'Pause', null, 'pause', null), 'action');
		// TODO: audio play	choice and go
		printer.Add (FileFunction (printer, 'Print G-Code', null, 'gcode', null, false), 'action');
		printer.Add (Button (printer, 'Home all', null, 'home_all', null), 'action');
		printer.Add (Button (printer, 'Sleep all', null, 'sleep_all', null), 'action');
		// TODO: readpin	pin+go to function
		// TODO: get position	button to display

		var td = printer.AddElement ('tr').AddElement ('td');
		td.colSpan = 2;
		var table = td.AddElement ('table');
		var tr = table.AddElement ('tr');
		for (var a = 0; a < printer.constants.maxaxes; ++a)
			printer.axes[a] = tr.Add (Axis (printer, a, a));
		tr = table.AddElement ('tr');
		for (var e = 0; e < printer.constants.maxextruders; ++e)
			printer.extruders[e] = tr.Add (Extruder (printer, e, e));
		tr = table.AddElement ('tr');
		for (var t = 0; t < printer.constants.maxtemps; ++t)
			printer.temps[t] = tr.Add (Temp (printer, t, null, t));
		var button = printer.AddElement ('button', 'action');
		button.type = 'button';
		button.AddText ('Refresh');
		button.onclick = function () {
			printer.refresh ();
		};
		// Unlike all components, a printer will refresh its contents when it is created.
		printer.refresh ();
	});
	return printer;
} // }}}
function Global () { // {{{
	var global = document.createElement ('div');
	global.call = rpc.call;
	global.get = Printer.get;
	global.set = Printer.set;
	var table = global.AddElement ('div', 'views');
	var views = [['view', 'Show visibility', null], ['action', 'Actions', true], ['useful', 'Useful settings', true], ['setting', 'Other settings', false], ['pin', 'Pins', false], ['inactive', 'Disabled elements', false], ['future', 'Unused features', false], ['constants', 'Constants', false], ['portaction', 'Port administration', false], ['audio', 'Audio', false]];
	for (var which = 0; which < views.length; ++which) {
		var tr = table.AddElement ('tr', views[which][2] === null ? undefined : 'view');
		var td = tr.AddElement ('td');
		var checkbox = td.AddElement ('input');
		checkbox.which = which;
		checkbox.type = 'checkbox';
		td.AddText (views[which][1]);
		checkbox.onclick = function () {
			if (this.checked)
				global.RemoveClass ('no' + views[this.which][0]);
			else
				global.AddClass ('no' + views[this.which][0]);
		};
		if (views[which][2])
			checkbox.checked = true;
		checkbox.onclick ();
	}
	table = global.AddElement ('table');
	table.Add (Checkbox (global, 'Autodetect', null, 'set_autodetect', null), 'portaction');
	// TODO: default printer
	global.blacklist = table.Add (Text (global, 'Blacklist', null, 'blacklist', null), 'portaction');
	table.Add (FileFunction (global, 'Upload audio', null, 'audio_load', null, true), 'audio');

	var buttonbox = table.AddElement ('tr').AddElement ('td');
	td.colSpan = 2;
	var button = buttonbox.AddElement ('button', 'portaction');
	button.type = 'button';
	button.AddText ('Rescan ports');
	button.onclick = function () {
		global.rescan ();
	};
	global.ports = global.AddElement ('span');
	global.printers = global.AddElement ('div');
	global.selected = null;
	global.rescan = function () {
		global.ports.clearAll ();
		global.ports.AddText ('Port:');
		global.printers.clearAll ();
		rpc.call ('get_ports', [], {}, function (ports) {
			global.ports_list = {}
			var first = null;
			for (var p = 0; p < ports.length; ++p) {
				var span = global.ports.AddElement ('span', 'box2');
				var inp = span.AddElement ('input');
				inp.type = 'radio';
				inp.name = 'port';
				inp.value = ports[p][0];
				inp.onclick = function () {
					if (global.selected)
						global.selected.className = 'printer hidden';
					global.selected = this.value;
					if (!global.ports_list[this.value][1]) {
						global.disablebutton.className = 'hidden';
						global.detectbutton.className = '';
					}
					else {
						global.disablebutton.className = '';
						global.detectbutton.className = 'hidden';
						global.ports_list[this.value][1].className = 'printer';
					}
				};
				span.AddText (ports[p][1] + '(' + ports[p][0] + ')');
				if (global.selected == ports[p]) {
					inp.selected = 'selected';
					inp.onchange ();
				}
				if (ports[p][1] != null) {
					global.ports_list[ports[p][0]] = [inp, global.printers.Add (Printer (ports[p][0]))];
					if (first == null)
						first = ports[p][0];
				}
				else
					global.ports_list[ports[p][0]] = [inp, null];
			}
			var div = span.AddElement ('span', 'portaction');
			global.disablebutton = div.AddElement ('button');
			global.disablebutton.type = 'button';
			global.disablebutton.AddText ('Disable');
			global.disablebutton.onClick = function () {
				if (!global.ports_list[global.selected][1])
					return;
				rpc.call ('disable', [global.selected], {}, function () {
					global.printers.removeChild (ports_list[global.selected][1]);
					ports_list[global.selected][1] = null;
					global.noprinter.className = '';
				});
			};
			global.detectbutton = div.AddElement ('button');
			global.detectbutton.type = 'button';
			global.detectbutton.AddText ('Detect');
			global.detectbutton.onClick = function () {
				if (global.ports_list[global.selected][1])
					return;
				rpc.call ('detect', [global.selected], {}, function () {
					global.ports_list[global.selected][1] = global.printers.Add (Printer (global.selected));
					ports_list[global.selected][1] = null;
					global.noprinter.className = 'hidden';
				});
			};
			if (global.selected == null && first != null)
				global.ports_list[first][0].click ();
		});
	};
	global.rescan ();
	return global;
} // }}}
// }}}

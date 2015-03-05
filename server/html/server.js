// vim: set foldmethod=marker :

// {{{ Global variables.
var eval_code = ['[!(', ')!]'];
var ref_code = ['[$(', ')$]'];
var _active_printer;
var printer;
var port;
var _update_handle;
var _updater;
var _templates;
var _updates;
var rpc;
var printers;
var _ports;
var autodetect;
var blacklist;
var audio_list;
var scripts;
var data;

var TYPE_CARTESIAN = 0;
var TYPE_DELTA = 1;
var TYPE_EXTRUDER = 2;
// }}}

function dbg(msg) {
	// Don't use Add, because this should be callable from anywhere, including Add.
	var div = document.getElementById('debug');
	var p = document.createElement('p');
	div.appendChild(p);
	p.appendChild(document.createTextNode(msg));
}

// {{{ Events from server.
function trigger_update(called_port, name) {
	//dbg(called_port + ':' + name); // + ',' + arguments[2]);
	if (!(name in _updates)) {
		/*var r = '';
		for (var i in _updates)
			r += ' ' + i;
		dbg('not found:' + r);*/
		return;
	}
	var old_port = port;
	port = called_port;
	var old_printer = printer;
	if (port in printers)
		printer = printers[port];
	else
		printer = null;
	for (var i = 0; i < _updates[name].length; ++i) {
		var args = [];
		for (var a = 2; a < arguments.length; ++a)
			args.push(arguments[a]);
		_updates[name][i].apply(null, args);
	}
	printer = old_printer;
	port = old_port;
}

function _setup_updater() {
	_updates = new Object;
	_updater = {
		signal: function(port, name, arg) {
			trigger_update(port, 'signal', name, arg);
		},
		new_port: function(port) {
			_ports.push(port);
			trigger_update(port, 'new_port');
		},
		del_port: function(port) {
			trigger_update(port, 'del_port');
			_ports.splice(_ports.indexOf(port), 1);
		},
		reset: function(port) {
			trigger_update(port, 'reset');
		},
		stall: function(port) {
			trigger_update(port, 'stall');
		},
		printing: function(port, state) {
			trigger_update(port, 'printing', state);
		},
		confirm: function(port, id, message) {
			trigger_update(port, 'confirm', id, message);
		},
		autodetect: function(state) {
			autodetect = state;
			trigger_update('', 'autodetect');
		},
		blacklist: function(value) {
			blacklist = value;
			trigger_update('', 'blacklist');
		},
		queue: function(port, q) {
			printers[port].queue = q;
			trigger_update(port, 'queue');
		},
		serial: function(port, serialport, data) {
			trigger_update(port, 'serial', serialport, data);
		},
		new_printer: function(port, constants) {
			printers[port] = {
				port: port,
				profile: 'default',
				queue: [],
				uuid: constants[0],
				queue_length: constants[1],
				audio_fragments: constants[2],
				audio_fragment_size: constants[3],
				num_digital_pins: constants[4],
				num_analog_pins: constants[5],
				num_spaces: 0,
				num_temps: 0,
				num_gpios: 0,
				led_pin: 0,
				probe_pin: 0,
				probe_dist: Infinity,
				probe_safe_dist: Infinity,
				bed_id: 255,
				fan_id: 255,
				spindle_id: 255,
				unit_name: 'mm',
				timeout: 0,
				feedrate: 1,
				zoffset: 0,
				message: null,
				spaces: [],
				temps: [],
				gpios: []
			};
			printers[port].call = function(name, a, ka, reply) {
				var p = this.port;
				if (_active_printer != p) {
					rpc.call('set_printer', [null, p], {}, function() { _active_printer = p; rpc.call(name, a, ka, reply); });
				}
				else
					rpc.call(name, a, ka, reply);
			};
			trigger_update(port, 'new_printer');
		},
		del_printer: function(port) {
			if (_active_printer == port)
				_active_printer = null;
			trigger_update(port, 'del_printer');
			for (var cb in _updates) {
				if (cb.substring(0, port.length + 1) == port + ' ')
					delete _updates[cb];
			}
			delete printers[port];
		},
		new_audio: function(list) {
			audio_list = list;
			trigger_update('', 'new_audio');
		},
		new_script: function(name, code, data) {
			scripts[name] = [code, data];
			trigger_update('', 'new_script', name);
		},
		del_script: function(name) {
			trigger_update('', 'del_script', name);
			delete scripts[name];
		},
		new_data: function(name, data) {
			scripts[name][1] = data;
			trigger_update('', 'new_data', name);
		},
		blocked: function(port, reason) {
			trigger_update(port, 'blocked', reason);
		},
		message: function(port, stat) {
			trigger_update(port, 'message', stat);
		},
		globals_update: function(port, values) {
			printers[port].profile = values[0];
			new_num_spaces = values[1];
			new_num_temps = values[2];
			new_num_gpios = values[3];
			printers[port].led_pin = values[4];
			printers[port].probe_pin = values[5];
			printers[port].probe_dist = values[6];
			printers[port].probe_safe_dist = values[7];
			printers[port].bed_id = values[8];
			printers[port].fan_id = values[9];
			printers[port].spindle_id = values[10];
			printers[port].unit_name = values[11];
			printers[port].timeout = values[12];
			printers[port].feedrate = values[13];
			printers[port].zoffset = values[14];
			printers[port].status = values[15];
			for (var i = printers[port].num_spaces; i < new_num_spaces; ++i) {
				printers[port].spaces.push({
					name: null,
					type: TYPE_CARTESIAN,
					max_deviation: 0,
					num_axes: 0,
					num_motors: 0,
					delta_angle: 0,
					axis: [],
					motor: []
				});
			}
			printers[port].spaces.length = new_num_spaces;
			printers[port].num_spaces = new_num_spaces;
			for (var i = printers[port].num_temps; i < new_num_temps; ++i) {
				printers[port].temps.push({
					name: null,
					heater_pin: 0,
					fan_pin: 0,
					thermistor_pin: 0,
					R0: 0,
					R1: 0,
					Rc: 0,
					Tc: 0,
					beta: 0,
					fan_temp: 0,
					value: 0,
					temp: NaN,
					history: []
				});
			}
			printers[port].temps.length = new_num_temps;
			printers[port].num_temps = new_num_temps;
			for (var i = printers[port].num_gpios; i < new_num_gpios; ++i) {
				printers[port].gpios.push({
					name: null,
					pin: 0,
					state: 3,
					reset: 3
				});
			}
			printers[port].gpios.length = new_num_gpios;
			printers[port].num_gpios = new_num_gpios;
			trigger_update(port, 'globals_update');
		},
		space_update: function(port, index, values) {
			printers[port].spaces[index].name = values[0];
			printers[port].spaces[index].type = values[1];
			printers[port].spaces[index].max_deviation = values[2];
			printers[port].spaces[index].num_axes = values[3].length;
			printers[port].spaces[index].num_motors = values[4].length;
			var current = [];
			for (var a = 0; a < printers[port].spaces[index].num_axes; ++a)
				current.push(a < printers[port].spaces[index].axis.length ? printers[port].spaces[index].axis[a].current : NaN);
			printers[port].spaces[index].axis = [];
			printers[port].spaces[index].motor = [];
			for (var a = 0; a < printers[port].spaces[index].num_axes; ++a) {
				printers[port].spaces[index].axis.push({
					current: current[a],
					name: values[3][a][0],
					offset: values[3][a][1],
					park: values[3][a][2],
					park_order: values[3][a][3],
					max_v: values[3][a][4],
					min: values[3][a][5],
					max: values[3][a][6]
				});
			}
			for (var m = 0; m < printers[port].spaces[index].num_motors; ++m) {
				printers[port].spaces[index].motor.push({
					name: values[4][m][0],
					step_pin: values[4][m][1],
					dir_pin: values[4][m][2],
					enable_pin: values[4][m][3],
					limit_min_pin: values[4][m][4],
					limit_max_pin: values[4][m][5],
					sense_pin: values[4][m][6],
					steps_per_unit: values[4][m][7],
					max_steps: values[4][m][8],
					home_pos: values[4][m][9],
					limit_v: values[4][m][10],
					limit_a: values[4][m][11],
					home_order: values[4][m][12]
				});
			}
			if (index == 1) {
				for (var a = 0; a < values[5].length; ++a)
					printers[port].spaces[index].axis[a].multiplier = values[5][a];
			}
			if (printers[port].spaces[index].type == TYPE_DELTA) {
				for (var i = 0; i < 3; ++i) {
					printers[port].spaces[index].motor[i].delta_axis_min = values[6][i][0];
					printers[port].spaces[index].motor[i].delta_axis_max = values[6][i][1];
					printers[port].spaces[index].motor[i].delta_rodlength = values[6][i][2];
					printers[port].spaces[index].motor[i].delta_radius = values[6][i][3];
				}
				printers[port].spaces[index].delta_angle = values[6][3];
			}
			if (printers[port].spaces[index].type == TYPE_EXTRUDER) {
				for (var i = 0; i < printers[port].spaces[index].axis.length; ++i) {
					printers[port].spaces[index].axis[i].extruder_dx = values[6][i][0];
					printers[port].spaces[index].axis[i].extruder_dy = values[6][i][1];
					printers[port].spaces[index].axis[i].extruder_dz = values[6][i][2];
				}
			}
			trigger_update(port, 'space_update', index);
		},
		temp_update: function(port, index, values) {
			printers[port].temps[index].name = values[0];
			printers[port].temps[index].R0 = values[1];
			printers[port].temps[index].R1 = values[2];
			printers[port].temps[index].Rc = values[3];
			printers[port].temps[index].Tc = values[4];
			printers[port].temps[index].beta = values[5];
			printers[port].temps[index].heater_pin = values[6];
			printers[port].temps[index].fan_pin = values[7];
			printers[port].temps[index].thermistor_pin = values[8];
			printers[port].temps[index].fan_temp = values[9];
			printers[port].temps[index].value = values[10];
			trigger_update(port, 'temp_update', index);
		},
		gpio_update: function(port, index, values) {
			printers[port].gpios[index].name = values[0];
			printers[port].gpios[index].pin = values[1];
			printers[port].gpios[index].state = values[2];
			printers[port].gpios[index].reset = values[3];
			trigger_update(port, 'gpio_update', index);
		}
	};
}

function register_update(name, cb) {
	if (!(name in _updates))
		_updates[name] = [];
	_updates[name].push(cb);
	return function() {
		var pos = _updates[name].indexOf(cb);
		_updates[name].splice(pos, 1);
	};
}
// }}}

// {{{ Initialization.
function _reconnect() {
	trigger_update('', 'connect', false);
	rpc = null;
	for (var s in scripts) {
		if (typeof scripts[s] == 'object')
			_updater.del_script(s);
	}
	while (_ports.length > 0) {
		var p = _ports[0];
		if (printers[p] !== undefined)
			_updater.del_printer(p);
		_updater.del_port(p);
	}
	if (!confirm('The connection to the server was lost.  Reconnect?'))
		return;
	// Wait a moment before retrying.
	setTimeout(function() {
		rpc = Rpc(_updater, _setup_connection, _reconnect);
	}, 500);
}

function Create(name, className) {
	return document.createElement(name).AddClass(className);
}

function setup() {
	// Make sure the globals have a value of the correct type.
	printers = new Object;
	_ports = [];
	scripts = new Object;
	autodetect = true;
	blacklist = '';
	var proto = Object.prototype;
	proto.Add = function(object, className) {
		if (!(object instanceof Array))
			object = [object];
		for (var i = 0; i < object.length; ++i) {
			if (typeof object[i] == 'string')
				this.AddText(object[i]);
			else {
				this.appendChild(object[i]);
				object[i].AddClass(className);
			}
		}
		return object[0];
	};
	proto.AddElement = function(name, className) { var element = document.createElement(name); return this.Add(element, className); };
	proto.AddText = function(text) { var t = document.createTextNode(text); this.Add(t); return this; };
	proto.ClearAll = function() { while (this.firstChild) this.removeChild(this.firstChild); return this; };
	proto.AddClass = function(className) {
		if (!className)
			return this;
		var classes = this.className.split(' ');
		var newclasses = className.split(' ');
		for (var i = 0; i < newclasses.length; ++i) {
			if (classes.indexOf(newclasses[i]) < 0)
				classes.push(newclasses[i]);
		}
		this.className = classes.join(' ');
		return this;
	};
	proto.RemoveClass = function(className) {
		if (!className)
			return this;
		var classes = this.className.split(' ');
		var oldclasses = className.split(' ');
		for (var i = 0; i < oldclasses.length; ++i) {
			var pos = classes.indexOf(oldclasses[i]);
			if (pos >= 0)
				classes.splice(pos, 1);
		}
		this.className = classes.join(' ');
		return this;
	};
	proto.HaveClass = function(className) {
		if (!className)
			return true;
		var classes = this.className.split(' ');
		for (var i = 0; i < classes.length; ++i) {
			var pos = classes.indexOf(className);
			if (pos >= 0)
				return true;
		}
		return false;
	};
	proto.AddEvent = function(name, impl) {
		this.addEventListener(name, impl, false);
		return this;
	};
	_setup_updater();
	rpc = Rpc(_updater, _setup_connection, _reconnect);
}

function _setup_connection() {
	trigger_update('', 'connect', true);
	rpc.call('set_monitor', [true], {}, null);
}
// }}}

// {{{ Temperature updates.
function _do_update_temps(queue, pos) {
	if (!rpc)
		return;
	if (!pos)
		pos = 0;
	while (pos < queue.length && !queue[pos][0] ())
		++pos;
	if (pos >= queue.length) {
		if (_update_handle != null)
			clearTimeout(_update_handle);
		_update_handle = setTimeout(_update_temps, 5000);
	}
	else
		rpc.call(queue[pos][1], queue[pos][2], queue[pos][3], function(t) { queue[pos][4] (t); _do_update_temps(queue, pos + 1); });
}

function _update_temps() {
	_update_handle = null;
	if (!rpc)
		return;
	var p = global.ports_list[global.selected];
	if (p && p[1]) {
		if (_active_printer != p[1])
			rpc.call('set_printer', [null, global.selected], {}, function() { _active_printer = p[1]; _do_update_temps(p[1].monitor_queue); });
		else
			_do_update_temps(p[1].monitor_queue);
	}
}
// }}}

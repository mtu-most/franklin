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
var queue;
// }}}

function dbg(msg) {
	document.getElementById('debug').AddElement('p').AddText(msg);
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
		'signal': function(port, name, arg) {
			trigger_update(port, 'signal', name, arg);
		},
		'new_port': function(port) {
			_ports.push(port);
			trigger_update('', 'new_port', port);
		},
		'del_port': function(port) {
			trigger_update(port, 'del_port');
			_ports.splice(_ports.indexOf(port), 1);
		},
		'reset': function(port) {
			trigger_update(port, 'reset');
		},
		'stall': function(port) {
			trigger_update(port, 'stall');
		},
		'printing': function(port, state) {
			trigger_update(port, 'printing', state);
		},
		'confirm': function(port, message) {
			trigger_update(port, 'confirm', message);
		},
		'autodetect': function(state) {
			autodetect = state;
			trigger_update('', 'autodetect');
		},
		'blacklist': function(value) {
			blacklist = value;
			trigger_update('', 'blacklist');
		},
		'queue': function(q) {
			queue = q;
			trigger_update('', 'queue');
		},
		'new_printer': function(port, constants) {
			printers[port] = {
				'port': port,
				'namelen': constants[0],
				'queue_length': constants[1],
				'maxaxes': constants[2],
				'maxextruders': constants[3],
				'maxtemps': constants[4],
				'maxgpios': constants[5],
				'audio_fragments': constants[6],
				'audio_fragment_size': constants[7],
				'num_digital_pins': constants[8],
				'num_pins': constants[9],
				'name': '',
				'num_axes': 0,
				'num_extruders': 0,
				'num_temps': 0,
				'num_gpios': 0,
				'printer_type': 0,
				'led_pin': 0,
				'probe_pin': 0,
				'room_T': 0,
				'motor_limit': 0,
				'temp_limit': 0,
				'feedrate': 1,
				'angle': 0,
				'paused': false,
				'axis': [],
				'extruder': [],
				'temp': [],
				'gpio': []
			};
			for (var i = 0; i < printers[port].maxaxes; ++i) {
				printers[port].axis.push({
					'motor': {
						'step_pin': 0,
						'dir_pin': 0,
						'enable_pin': 0,
						'steps_per_mm': 0,
						'limit_v': 0,
						'max_v': 0,
						'max_a': 0,
						'max_steps': 1,
						'runspeed': 0,
						'sleeping': true
					},
					'limit_min_pin': 0,
					'limit_max_pin': 0,
					'sense_pin': 0,
					'limit_pos': 0,
					'axis_min': 0,
					'axis_max': 0,
					'motor_min': 0,
					'motor_max': 0,
					'park': 0,
					'delta_length': 0,
					'delta_radius': 0,
					'offset': 0
				});
			}
			for (var i = 0; i < printers[port].maxextruders; ++i) {
				printers[port].extruder.push({
					'motor': {
						'step_pin': 0,
						'dir_pin': 0,
						'enable_pin': 0,
						'steps_per_mm': 0,
						'limit_v': 0,
						'max_v': 0,
						'max_a': 0,
						'max_steps': 1,
						'runspeed': 0,
						'sleeping': true
					},
					'temp': {
						'power_pin': 0,
						'thermistor_pin': 0,
						'R0': 0,
						'R1': 0,
						'Rc': 0,
						'Tc': 0,
						'beta': 0,
						'core_C': 0,
						'shell_C': 0,
						'transfer': 0,
						'radiation': 0,
						'power': 0,
						'value': 0
					},
					'filament_heat': 0,
					'nozzle_size': 0,
					'filament_size': 0
				});
			}
			for (var i = 0; i < printers[port].maxtemps; ++i) {
				printers[port].temp.push({
					'power_pin': 0,
					'thermistor_pin': 0,
					'R0': 0,
					'R1': 0,
					'Rc': 0,
					'Tc': 0,
					'beta': 0,
					'core_C': 0,
					'shell_C': 0,
					'transfer': 0,
					'radiation': 0,
					'power': 0,
					'value': 0
				});
			}
			for (var i = 0; i < printers[port].maxgpios; ++i) {
				printers[port].gpio.push({
					'pin': 0,
					'state': 0,
					'master': 0,
					'value': 0
				});
			}
			printers[port].call = function(name, a, ka, reply) {
				var p = this.port;
				if (_active_printer != p) {
					rpc.call('set_printer', [null, p], {}, function() { _active_printer = p; rpc.call(name, a, ka, reply); });
				}
				else
					rpc.call(name, a, ka, reply);
			};
			trigger_update(port, 'new_printer', printers[port]);
		},
		'del_printer': function(port) {
			if (_active_printer == port)
				_active_printer = null;
			trigger_update(port, 'del_printer');
			for (var cb in _updates) {
				if (cb.substring(0, port.length + 1) == port + ' ')
					delete _updates[cb];
			}
			delete printers[port];
		},
		'new_audio': function(list) {
			audio_list = list;
			trigger_update('', 'new_audio');
		},
		'new_script': function(name, code, data) {
			scripts[name] = [code, data];
			trigger_update('', 'new_script', name);
		},
		'del_script': function(name) {
			trigger_update('', 'del_script', name);
			delete scripts[name];
		},
		'new_data': function(name, data) {
			scripts[name][1] = data;
			trigger_update('', 'new_data', name);
		},
		'variables_update': function(port, values) {
			printers[port].name = values[0];
			printers[port].num_axes = values[1];
			printers[port].num_extruders = values[2];
			printers[port].num_temps = values[3];
			printers[port].num_gpios = values[4];
			printers[port].printer_type = values[5];
			printers[port].led_pin = values[6];
			printers[port].probe_pin = values[7];
			printers[port].room_T = values[8];
			printers[port].motor_limit = values[9];
			printers[port].temp_limit = values[10];
			printers[port].feedrate = values[11];
			printers[port].angle = values[12];
			printers[port].paused = values[13];
			trigger_update(port, 'variables_update');
		},
		'axis_update': function(port, index, values) {
			printers[port].axis[index].motor.step_pin = values[0][0];
			printers[port].axis[index].motor.dir_pin = values[0][1];
			printers[port].axis[index].motor.enable_pin = values[0][2];
			printers[port].axis[index].motor.steps_per_mm = values[0][3];
			printers[port].axis[index].motor.limit_v = values[0][4];
			printers[port].axis[index].motor.max_v = values[0][5];
			printers[port].axis[index].motor.max_a = values[0][6];
			printers[port].axis[index].motor.max_steps = values[0][7];
			printers[port].axis[index].motor.runspeed = values[0][8];
			printers[port].axis[index].motor.sleeping = values[0][9];
			printers[port].axis[index].limit_min_pin = values[1];
			printers[port].axis[index].limit_max_pin = values[2];
			printers[port].axis[index].sense_pin = values[3];
			printers[port].axis[index].limit_pos = values[4];
			printers[port].axis[index].axis_min = values[5];
			printers[port].axis[index].axis_max = values[6];
			printers[port].axis[index].motor_min = values[7];
			printers[port].axis[index].motor_max = values[8];
			printers[port].axis[index].park = values[9];
			printers[port].axis[index].delta_length = values[10];
			printers[port].axis[index].delta_radius = values[11];
			printers[port].axis[index].offset = values[12];
			trigger_update(port, 'axis_update', index);
		},
		'extruder_update': function(port, index, values) {
			printers[port].extruder[index].motor.step_pin = values[0][0];
			printers[port].extruder[index].motor.dir_pin = values[0][1];
			printers[port].extruder[index].motor.enable_pin = values[0][2];
			printers[port].extruder[index].motor.steps_per_mm = values[0][3];
			printers[port].extruder[index].motor.limit_v = values[0][4];
			printers[port].extruder[index].motor.max_v = values[0][5];
			printers[port].extruder[index].motor.max_a = values[0][6];
			printers[port].extruder[index].motor.max_steps = values[0][7];
			printers[port].extruder[index].motor.runspeed = values[0][8];
			printers[port].extruder[index].motor.sleeping = values[0][9];
			printers[port].extruder[index].temp.power_pin = values[1][0];
			printers[port].extruder[index].temp.thermistor_pin = values[1][1];
			printers[port].extruder[index].temp.R0 = values[1][2];
			printers[port].extruder[index].temp.R1 = values[1][3];
			printers[port].extruder[index].temp.Rc = values[1][4];
			printers[port].extruder[index].temp.Tc = values[1][5];
			printers[port].extruder[index].temp.beta = values[1][6];
			printers[port].extruder[index].temp.core_C = values[1][7];
			printers[port].extruder[index].temp.shell_C = values[1][8];
			printers[port].extruder[index].temp.transfer = values[1][9];
			printers[port].extruder[index].temp.radiation = values[1][10];
			printers[port].extruder[index].temp.power = values[1][11];
			printers[port].extruder[index].temp.value = values[1][12];
			printers[port].extruder[index].filament_heat = values[2];
			printers[port].extruder[index].nozzle_size = values[3];
			printers[port].extruder[index].filament_size = values[4];
			trigger_update(port, 'extruder_update', index);
		},
		'temp_update': function(port, index, values) {
			printers[port].temp[index].power_pin = values[0];
			printers[port].temp[index].thermistor_pin = values[1];
			printers[port].temp[index].R0 = values[2];
			printers[port].temp[index].R1 = values[3];
			printers[port].temp[index].Rc = values[4];
			printers[port].temp[index].Tc = values[5];
			printers[port].temp[index].beta = values[6];
			printers[port].temp[index].core_C = values[7];
			printers[port].temp[index].shell_C = values[8];
			printers[port].temp[index].transfer = values[9];
			printers[port].temp[index].radiation = values[10];
			printers[port].temp[index].power = values[11];
			printers[port].temp[index].value = values[12];
			trigger_update(port, 'temp_update', index);
		},
		'gpio_update': function(port, index, values) {
			printers[port].gpio[index].pin = values[0];
			printers[port].gpio[index].state = values[1];
			printers[port].gpio[index].master = values[2];
			printers[port].gpio[index].value = values[3];
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
	for (var p in printers) {
		if (typeof printers[p] == 'object') {
			_updater.del_printer(p);
		}
	}
	while (_ports.length > 0)
		_updater.del_port(_ports.pop());
	if (!confirm('The connection to the server was lost.  Reconnect?'))
		return;
	// Try again in 5 seconds.
	setTimeout(function() {
		rpc = Rpc(_updater, _setup_connection, _reconnect);
	}, 5000);
}

function setup() {
	// Make sure the globals have a value of the correct type.
	printers = new Object;
	_ports = [];
	scripts = new Object;
	queue = [];
	autodetect = true;
	blacklist = '';
	var proto = Object.prototype;
	proto.Add = function(object, className) { this.appendChild(object); if (className) object.className = className; return object; };
	proto.AddElement = function(name, className) { var element = document.createElement(name); return this.Add(element, className); };
	proto.AddText = function(text) { var t = document.createTextNode(text); return this.Add(t); };
	proto.ClearAll = function() { while (this.firstChild) this.removeChild(this.firstChild); return this; };
	proto.AddClass = function(className) {
		if (!className)
			return this;
		var classes = this.className.split(' ');
		if (classes.indexOf(className) >= 0)
			return this;
		classes.push(className);
		this.className = classes.join(' ');
		return this;
	};
	proto.RemoveClass = function(className) {
		if (!className)
			return this;
		var classes = this.className.split(' ');
		var pos = classes.indexOf(className);
		if (pos < 0)
			return this;
		classes.splice(pos, 1);
		this.className = classes.join(' ');
		return this;
	};
	_init_templates();
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

// {{{ Template handling.
function _init_templates() {
	_templates = new Object;
	var templatediv = document.getElementById('templates');
	templatediv.parentNode.removeChild(templatediv);
	while (templatediv.firstChild) {
		var current = templatediv.firstChild;
		templatediv.removeChild(current);
		if (current instanceof Text || current instanceof Comment)
			continue;
		_templates[current.id] = current;
	}
	_parse_node(document.getElementById('container'));
}

function build(template, args) {
	var ret = [];
	if (!args)
		args = [];
	if (!(template in _templates)) {
		alert('Pages uses undefined template ' + template);
		return [];
	}
	var newnode = _parse_node(_templates[template].cloneNode(true), args)[0];
	for (var i = 0; i < newnode.childNodes.length; ++i)
		ret.push(newnode.childNodes[i]);
	return ret;
}

function _parse_node(node, args) {
	// Recursively parse all special things in a node.
	var current = node.firstChild;
	var eval_level = 0;
	while (current) {
		var next = current.nextSibling;
		if (current instanceof Comment) {
			current = next;
			continue;
		}
		if (current instanceof Text) {
			// Look for codes.
			var pos = current.data.indexOf(ref_code[0]);
			var result;
			if (pos >= 0) {
				current = _parse_ref(node, current, next, pos, args);
				continue;
			}
			pos = current.data.indexOf(eval_code[0]);
			if (pos >= 0) {
				// Parse evaluations.
				node.removeChild(current);
				var first = document.createTextNode(current.data.substring(0, pos));
				node.insertBefore(first, next);
				var end = current.data.indexOf(eval_code[1], pos);
				var expr;
				if (end >= 0) {
					// Split the text node in two, so the end will be found by the code below.
					next = node.insertBefore(document.createTextNode(current.data.substring(end)), next);
					expr = current.data.substring(pos + eval_code[0].length, end);
				}
				else
					expr = current.data.substring(pos + eval_code[0].length);
				//dbg('expr:' + expr);
				var stack = [];
				var elements = [];
				current = next;
				while (current) {
					next = current.nextSibling;
					node.removeChild(current);
					if (current instanceof Comment) {
						current = next;
						continue;
					}
					if (current instanceof Text) {
						pos = current.data.indexOf(ref_code[0]);
						if (pos >= 0) {
							current = _parse_ref(node, current, next, pos, args);
							continue;
						}
						pos = current.data.indexOf(eval_code[1]);
						if (pos >= 0) {
							if (pos > 0)
								stack.push(current.data.substring(0, pos));
							if (pos + eval_code[1].length < current.data.length) {
								next = node.insertBefore(document.createTextNode(current.data.substring(pos + eval_code[1].length)), next);
							}
							//dbg(expr);
							result = eval('(' + expr + ')');
							if (typeof result == 'string')
								result = [document.createTextNode(String(result))];
							for (var i = 0; i < result.length; ++i) {
								if (typeof result[i] == 'string')
									node.insertBefore(document.createTextNode(result[i]), next);
								else
									node.insertBefore(result[i], next);
							}
							current = next;
							break;
						}
					}
					else {
						// This is always exactly one element, because only Text nodes can expand.
						current = _parse_node(current, args)[0];
						elements.push(current);
					}
					stack.push(current);
					current = next;
				}
				continue;
			}
		}
		else {
			// Because this is not a Text node, it will always return itself, so no need to replace it.
			_parse_node(current, args);
		}
		current = next;
	}
	node.normalize();
	return [node];
}

function _parse_ref(node, current, next, pos, args)
{
	var result;
	var end = current.data.indexOf(ref_code[1], pos);
	if (end < 0) {
		alert('unmatched ' + ref_code[0]);
		return null;
	}
	if (node == current.parentNode)
		node.removeChild(current);
	var data = /\s*(\w+)(?:\s+(.*?))?\s*$/.exec(current.data.substring(pos + ref_code[0].length, end));
	if (end + ref_code[1].length < current.data.length) {
		var rest = document.createTextNode(current.data.substring(end + ref_code[1].length));
		next = node.insertBefore(rest, next);
	}
	if (pos > 0) {
		current = document.createTextNode(current.data.substring(0, pos));
		node.insertBefore(current, next);
	}
	else
		current = null;
	var newargs;
	if (data[2] !== undefined) {
		if (data[2][0] == '[' && data[2][data[2].length - 1] == ']')
			newargs = eval('(' + data[2] + ')');
		else {
			result = data[2];
			pos = 0;
			while (true) {
				pos = result.indexOf(eval_code[0], pos);
				if (pos < 0)
					break;
				end = result.indexOf(eval_code[1], pos);
				result = result.substring(0, pos) + String(eval('(' + result.substring(pos + eval_code[0].length, end) + ')')) + result.substring(end + eval_code[1].length);
				pos = end + eval_code[1].length;
			}
			newargs = result.split('|');
		}
	}
	result = build(data[1], newargs);
	for (var i = 0; i < result.length; ++i) {
		node.insertBefore(result[i], next);
		if (current == null)
			current = result[i];
	}
	if (current == null)
		current = next;
	return current;
}
// }}}

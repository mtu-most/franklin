/* server.js - host communication handling for Franklin
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

// {{{ Global variables.
var _update_handle;
var _updater;
var _update_running = false;
var _pending_updates = [];
var _templates;
var rpc;
var machines;
var all_ports, all_firmwares;
var is_autodetect;
var blacklist;
var audio_list;
var role;

var TYPE_CARTESIAN = 'cartesian';
var TYPE_EXTRUDER = 'extruder';
var TYPE_FOLLOWER = 'follower';
// }}}

// {{{ Events from server.
function trigger_update(machine, name) {
	try {
		if (window[name] === undefined) {
			alert('called undefined update function ' + name);
			return;
		}
		if (!(machine in machines)) {
			if (machine)
				alert('undefined machine referenced: ' + machine);
			machine = null;
		}
		var args = [machine];
		for (var a = 2; a < arguments.length; ++a)
			args.push(arguments[a]);
		//console.log(name, args);
		window[name].apply(null, args);
	} catch (e) {
		console.info(e);
	}
}

function _setup_updater() {
	_updater = {
		signal: function(machine, name, arg) {
			trigger_update(machine, 'signal', name, arg);
		},
		new_port: function(port, options) {
			if (all_ports.indexOf(port) < 0) {
				all_ports.push(port);
				all_firmwares[port] = options;
				trigger_update(null, 'new_port', port);
			}
		},
		del_port: function(port) {
			trigger_update(null, 'del_port', port);
			all_ports.splice(all_ports.indexOf(port), 1);
			delete all_firmwares[port];
		},
		port_state: function(port, state) {
			trigger_update(null, 'port_state', port, state);
		},
		reset: function(machine) {
			trigger_update(machine, 'reset');
		},
		stall: function(machine) {
			trigger_update(machine, 'stall');
		},
		confirm: function(machine, id, message) {
			machines[machine].confirmation = [id, message];
			trigger_update(machine, 'ask_confirmation', id, message);
		},
		autodetect: function(state) {
			is_autodetect = state;
			trigger_update(null, 'autodetect');
		},
		blacklist: function(value) {
			blacklist = value;
			trigger_update(null, 'blacklist');
		},
		queue: function(machine, q) {
			machines[machine].queue = q;
			trigger_update(machine, 'queue');
		},
		audioqueue: function(machine, q) {
			machines[machine].audioqueue = q;
			trigger_update(machine, 'audioqueue');
		},
		serial: function(machine, serialport, data) {
			trigger_update(machine, 'serial', serialport, data);
		},
		new_machine: function(machine, constants) {
			if (machines[machine] === undefined) {
				machines[machine] = {
					uuid: machine,
					connected: false,
					probemap: null,
					profile: 'default',
					user_interface: '',
					ui_update: true,
					queue: [],
					audioqueue: [],
					pin_names: [],
					num_temps: 0,
					num_gpios: 0,
					led_pin: 0,
					stop_pin: 0,
					probe_pin: 0,
					spiss_pin: 0,
					pattern_step_pin: 0,
					pattern_dir_pin: 0,
					probe_dist: Infinity,
					probe_offset: 0,
					probe_safe_dist: Infinity,
					bed_id: 255,
					fan_id: 255,
					spindle_id: 255,
					unit_name: 'mm',
					park_after_job: true,
					sleep_after_job: true,
					cool_after_job: true,
					spi_setup: '',
					status: 'Starting',
					probe_enable: false,
					feedrate: 1,
					max_deviation: 0,
					max_v: 0,
					max_a: 0,
					max_J: 0,
					adjust_speed: 1,
					timeout: Infinity,
					targetangle: 0,
					store_adc: false,
					temp_scale_min: 0,
					temp_scale_max: 0,
					message: null,
					blocked: '',
					confirmation: [null, ''],
					spaces: [{
							name: null,
							type: TYPE_CARTESIAN,
							num_axes: 0,
							num_motors: 0,
							axis: [],
							motor: []
						},
						{
							name: null,
							type: TYPE_EXTRUDER,
							num_axes: 0,
							num_motors: 0,
							axis: [],
							motor: []
						},
						{
							name: null,
							type: TYPE_FOLLOWER,
							num_axes: 0,
							num_motors: 0,
							axis: [],
							motor: []
						}],
					temps: [],
					gpios: []
				};
				for (var c in constants)
					machines[machine][c] = constants[c];
				machines[machine].call = function(name, a, ka, reply) {
					//console.info('calling', name);
					ka.machine = this.uuid;
					rpc.call(name, a, ka, reply);
				};
				trigger_update(machine, 'new_machine');
			}
		},
		del_machine: function(machine) {
			trigger_update(machine, 'del_machine');
			delete machines[machine];
		},
		new_audio: function(list) {
			audio_list = list;
			trigger_update(null, 'new_audio');
		},
		uploading: function(port, message) {
			trigger_update(null, 'uploading', port, message);
		},
		blocked: function(machine, message) {
			machines[machine].blocked = message;
			trigger_update(machine, 'blocked', message);
		},
		message: function(machine, stat) {
			machines[machine].message = stat;
			trigger_update(machine, 'message', stat);
		},
		globals_update: function(machine, values) {
			machines[machine].ui_update |= machines[machine].user_interface != values.user_interface;
			var old_num_temps = machines[machine].num_temps;
			var old_num_gpios = machines[machine].num_gpios;
			var old_connected = machines[machine].connected;
			for (var v in values)
				machines[machine][v] = values[v];
			var nums_changed = machines[machine].num_temps != old_num_temps || machines[machine].num_gpios != old_num_gpios;
			for (var i = old_num_temps; i < machines[machine].num_temps; ++i) {
				machines[machine].temps.push({
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
			machines[machine].temps.length = machines[machine].num_temps;
			for (var i = old_num_gpios; i < machines[machine].num_gpios; ++i) {
				machines[machine].gpios.push({
					name: null,
					pin: 0,
					state: 3,
					reset: 3,
					leader: null,
				});
			}
			machines[machine].gpios.length = machines[machine].num_gpios;
			trigger_update(machine, 'globals_update', machines[machine].ui_update, nums_changed, machines[machine].connected && !old_connected);
		},
		space_update: function(machine, index, values) {
			var nums_changed = false;
			machines[machine].spaces[index].name = values[0];
			machines[machine].spaces[index].type = values[1];
			if (machines[machine].spaces[index].num_axes != values[2].length)
				nums_changed = true;
			machines[machine].spaces[index].num_axes = values[2].length;
			if (machines[machine].spaces[index].num_motors != values[3].length)
				nums_changed = true;
			machines[machine].spaces[index].num_motors = values[3].length;
			var current = [];
			var info = type_info[machines[machine].spaces[index].type];
			for (var a = 0; a < machines[machine].spaces[index].num_axes; ++a)
				current.push(a < machines[machine].spaces[index].axis.length ? machines[machine].spaces[index].axis[a].current : NaN);
			machines[machine].spaces[index].axis = [];
			machines[machine].spaces[index].motor = [];
			for (var a = 0; a < machines[machine].spaces[index].num_axes; ++a) {
				machines[machine].spaces[index].axis.push({
					current: current[a],
					name: values[2][a][0],
					park: values[2][a][1],
					park_order: values[2][a][2],
					min: values[2][a][3],
					max: values[2][a][4],
					offset: values[2][a][5],
					home_pos2: values[2][a][6]
				});
			}
			for (var m = 0; m < machines[machine].spaces[index].num_motors; ++m) {
				machines[machine].spaces[index].motor.push({
					name: values[3][m][0],
					step_pin: values[3][m][1],
					dir_pin: values[3][m][2],
					enable_pin: values[3][m][3],
					limit_min_pin: values[3][m][4],
					limit_max_pin: values[3][m][5],
					steps_per_unit: values[3][m][6],
					home_pos: values[3][m][7],
					limit_v: values[3][m][8],
					limit_a: values[3][m][9],
					home_order: values[3][m][10],
					unit: values[3][m][11]
				});
			}
			if (index == 1) {
				for (var a = 0; a < values[4].length; ++a)
					machines[machine].spaces[index].axis[a].multiplier = values[4][a];
			}
			if (info) {
				if (info.load !== undefined)
					info.load(machine, index, values[5]);
				if (info.aload !== undefined) {
					for (var a = 0; a < machines[machine].spaces[index].num_axes; ++a)
						info.aload(machine, index, a, values[2][a][6]);
				}
				if (info.mload !== undefined) {
					for (var m = 0; m < machines[machine].spaces[index].num_motors; ++m)
						info.mload(machine, index, m, values[3][m][12]);
				}
			}
			trigger_update(machine, 'space_update', index, nums_changed);
		},
		temp_update: function(machine, index, values) {
			for (var v in values)
				machines[machine].temps[index][v] = values[v];
			trigger_update(machine, 'temp_update', index);
		},
		gpio_update: function(machine, index, values) {
			for (var v in values)
				machines[machine].gpios[index][v] = values[v];
			trigger_update(machine, 'gpio_update', index);
		}
	};
}
// }}}

// {{{ Initialization.
function _reconnect() {
	trigger_update(null, 'connect', false);
	rpc = null;
	while (all_ports.length > 0) {
		var p = all_ports[0];
		if (machines[p] !== undefined)
			_updater.del_machine(p);
		_updater.del_port(p);
	}
	try {
		if (!confirm('The connection to the server was lost.  Reconnect?'))
			return;
	}
	catch (e) {
		return;
	}
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
	machines = new Object;
	all_ports = [];
	all_firmwares = {};
	is_autodetect = true;
	blacklist = '';
	_setup_updater();
	rpc = Rpc(_updater, _setup_connection, _reconnect);
}

function _setup_connection() {
	rpc.call('get_role', [], {}, function(r) {
		role = r;
		document.getElementById('container').AddClass('role_' + role);
		rpc.call('get_typeinfo', [], {}, function(info) {
			trigger_update(null, 'connect', true, info);
			rpc.call('set_monitor', [true], {}, null);
		});
	});
}
// }}}

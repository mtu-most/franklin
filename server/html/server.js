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
var eval_code = ['[!(', ')!]'];
var ref_code = ['[$(', ')$]'];
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

var TYPE_CARTESIAN = 0;
var TYPE_DELTA = 1;
var TYPE_POLAR = 2;
var TYPE_EXTRUDER = 3;
var TYPE_FOLLOWER = 4;
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
		//console.log(name);
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
					queue_length: constants[0],
					pin_names: [],
					num_temps: 0,
					num_gpios: 0,
					led_pin: 0,
					stop_pin: 0,
					probe_pin: 0,
					spiss_pin: 0,
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
					timeout: 0,
					feedrate: 1,
					max_deviation: 0,
					max_v: 0,
					targetx: 0,
					targety: 0,
					targetangle: 0,
					zoffset: 0,
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
							delta_angle: 0,
							polar_max_r: Infinity,
							axis: [],
							motor: []
						},
						{
							name: null,
							type: TYPE_EXTRUDER,
							num_axes: 0,
							num_motors: 0,
							delta_angle: 0,
							polar_max_r: Infinity,
							axis: [],
							motor: []
						},
						{
							name: null,
							type: TYPE_FOLLOWER,
							num_axes: 0,
							num_motors: 0,
							delta_angle: 0,
							polar_max_r: Infinity,
							axis: [],
							motor: []
						}],
					temps: [],
					gpios: []
				};
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
			machines[machine].name = values[0];
			machines[machine].profile = values[1];
			var new_num_temps = values[2];
			var new_num_gpios = values[3];
			machines[machine].ui_update |= machines[machine].user_interface != values[4];
			machines[machine].user_interface = values[4];
			machines[machine].pin_names = values[5];
			machines[machine].led_pin = values[6];
			machines[machine].stop_pin = values[7];
			machines[machine].probe_pin = values[8];
			machines[machine].spiss_pin = values[9];
			machines[machine].probe_dist = values[10];
			machines[machine].probe_offset = values[11];
			machines[machine].probe_safe_dist = values[12];
			machines[machine].bed_id = values[13];
			machines[machine].fan_id = values[14];
			machines[machine].spindle_id = values[15];
			machines[machine].unit_name = values[16];
			machines[machine].timeout = values[17];
			machines[machine].feedrate = values[18];
			machines[machine].max_deviation = values[19];
			machines[machine].max_v = values[20];
			machines[machine].targetx = values[21];
			machines[machine].targety = values[22];
			machines[machine].targetangle = values[23];
			machines[machine].zoffset = values[24];
			machines[machine].store_adc = values[25];
			machines[machine].park_after_job = values[26];
			machines[machine].sleep_after_job = values[27];
			machines[machine].cool_after_job = values[28];
			machines[machine].spi_setup = values[29];
			machines[machine].temp_scale_min = values[30];
			machines[machine].temp_scale_max = values[31];
			machines[machine].probemap = values[32];
			machines[machine].connected = values[33];
			machines[machine].status = values[34];
			var nums_changed = machines[machine].num_temps != new_num_temps || machines[machine].num_gpios != new_num_gpios;
			for (var i = machines[machine].num_temps; i < new_num_temps; ++i) {
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
			machines[machine].temps.length = new_num_temps;
			machines[machine].num_temps = new_num_temps;
			for (var i = machines[machine].num_gpios; i < new_num_gpios; ++i) {
				machines[machine].gpios.push({
					name: null,
					pin: 0,
					state: 3,
					reset: 3
				});
			}
			machines[machine].gpios.length = new_num_gpios;
			machines[machine].num_gpios = new_num_gpios;
			trigger_update(machine, 'globals_update', machines[machine].ui_update, nums_changed);
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
					home_pos2: values[2][a][5]
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
					home_order: values[3][m][10]
				});
			}
			if (index == 1) {
				for (var a = 0; a < values[4].length; ++a)
					machines[machine].spaces[index].axis[a].multiplier = values[4][a];
			}
			if (machines[machine].spaces[index].type == TYPE_DELTA) {
				for (var i = 0; i < 3; ++i) {
					machines[machine].spaces[index].motor[i].delta_axis_min = values[5][i][0];
					machines[machine].spaces[index].motor[i].delta_axis_max = values[5][i][1];
					machines[machine].spaces[index].motor[i].delta_rodlength = values[5][i][2];
					machines[machine].spaces[index].motor[i].delta_radius = values[5][i][3];
				}
				machines[machine].spaces[index].delta_angle = values[5][3];
			}
			if (machines[machine].spaces[index].type == TYPE_POLAR) {
				machines[machine].spaces[index].polar_max_r = values[5];
			}
			if (machines[machine].spaces[index].type == TYPE_EXTRUDER) {
				for (var i = 0; i < machines[machine].spaces[index].axis.length; ++i) {
					machines[machine].spaces[index].axis[i].extruder_dx = values[5][i][0];
					machines[machine].spaces[index].axis[i].extruder_dy = values[5][i][1];
					machines[machine].spaces[index].axis[i].extruder_dz = values[5][i][2];
				}
			}
			if (machines[machine].spaces[index].type == TYPE_FOLLOWER) {
				for (var i = 0; i < machines[machine].spaces[index].axis.length; ++i) {
					machines[machine].spaces[index].motor[i].follower_space = values[5][i][0];
					machines[machine].spaces[index].motor[i].follower_motor = values[5][i][1];
				}
			}
			trigger_update(machine, 'space_update', index, nums_changed);
		},
		temp_update: function(machine, index, values) {
			machines[machine].temps[index].name = values[0];
			machines[machine].temps[index].R0 = values[1];
			machines[machine].temps[index].R1 = values[2];
			machines[machine].temps[index].Rc = values[3];
			machines[machine].temps[index].Tc = values[4];
			machines[machine].temps[index].beta = values[5];
			machines[machine].temps[index].heater_pin = values[6];
			machines[machine].temps[index].fan_pin = values[7];
			machines[machine].temps[index].thermistor_pin = values[8];
			machines[machine].temps[index].fan_temp = values[9];
			machines[machine].temps[index].fan_duty = values[10];
			machines[machine].temps[index].heater_limit_l = values[11];
			machines[machine].temps[index].heater_limit_h = values[12];
			machines[machine].temps[index].fan_limit_l = values[13];
			machines[machine].temps[index].fan_limit_h = values[14];
			machines[machine].temps[index].hold_time = values[15];
			machines[machine].temps[index].value = values[16];
			trigger_update(machine, 'temp_update', index);
		},
		gpio_update: function(machine, index, values) {
			machines[machine].gpios[index].name = values[0];
			machines[machine].gpios[index].pin = values[1];
			machines[machine].gpios[index].state = values[2];
			machines[machine].gpios[index].reset = values[3];
			machines[machine].gpios[index].duty = values[4];
			machines[machine].gpios[index].value = values[5];
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
		trigger_update(null, 'connect', true);
		rpc.call('set_monitor', [true], {}, null);
	});
}
// }}}

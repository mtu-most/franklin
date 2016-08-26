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
var _active_printer;
var _update_handle;
var _updater;
var _templates;
var rpc;
var printers;
var _ports;
var autodetect;
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
function trigger_update(printer, name) {
	if (window[name] === undefined) {
		alert('called undefined update function ' + name);
		return;
	}
	if (!(printer in printers)) {
		if (printer)
			alert('undefined printer referenced: ' + printer);
		printer = null;
	}
	var args = [printer];
	for (var a = 2; a < arguments.length; ++a)
		args.push(arguments[a]);
	console.info(name, args);
	window[name].apply(null, args);
}

function _setup_updater() {
	_updater = {
		signal: function(printer, name, arg) {
			trigger_update(printer, 'signal', name, arg);
		},
		new_port: function(port) {
			if (_ports.indexOf(port) < 0) {
				_ports.push(port);
				trigger_update(null, 'new_port', port);
			}
		},
		del_port: function(port) {
			trigger_update(null, 'del_port', port);
			_ports.splice(_ports.indexOf(port), 1);
		},
		port_state: function(port, state) {
			trigger_update(null, 'port_state', port, state);
		},
		reset: function(printer) {
			trigger_update(printer, 'reset');
		},
		stall: function(printer) {
			trigger_update(printer, 'stall');
		},
		confirm: function(printer, id, message) {
			trigger_update(printer, 'ask_confirmation', id, message);
		},
		autodetect: function(state) {
			autodetect = state;
			trigger_update(null, 'autodetect');
		},
		blacklist: function(value) {
			blacklist = value;
			trigger_update(null, 'blacklist');
		},
		queue: function(printer, q) {
			printers[printer].queue = q;
			trigger_update(printer, 'queue');
		},
		audioqueue: function(printer, q) {
			printers[printer].audioqueue = q;
			trigger_update(printer, 'audioqueue');
		},
		serial: function(printer, serialport, data) {
			trigger_update(printer, 'serial', serialport, data);
		},
		new_printer: function(printer, constants) {
			if (printers[printer] === undefined) {
				printers[printer] = {
					uuid: printer,
					profile: 'default',
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
					probe_safe_dist: Infinity,
					bed_id: 255,
					fan_id: 255,
					spindle_id: 255,
					unit_name: 'mm',
					park_after_print: true,
					sleep_after_print: true,
					cool_after_print: true,
					spi_setup: '',
					status: 'Starting',
					timeout: 0,
					feedrate: 1,
					max_deviation: 0,
					max_v: 0,
					targetx: 0,
					targety: 0,
					zoffset: 0,
					store_adc: false,
					temp_scale_min: 0,
					temp_scale_max: 0,
					message: null,
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
				printers[printer].call = function(name, a, ka, reply) {
					console.info('calling', name);
					var uuid = this.uuid;
					if (_active_printer != uuid) {
						rpc.call('set_printer', [uuid], {}, function() { _active_printer = uuid; rpc.call(name, a, ka, reply); });
					}
					else
						rpc.call(name, a, ka, reply);
				};
				trigger_update(printer, 'new_printer');
			}
		},
		del_printer: function(printer) {
			if (_active_printer == printer)
				_active_printer = null;
			trigger_update(printer, 'del_printer');
			delete printers[printer];
		},
		new_audio: function(list) {
			audio_list = list;
			trigger_update(null, 'new_audio');
		},
		blocked: function(printer, reason) {
			trigger_update(printer, 'blocked', reason);
		},
		message: function(printer, stat) {
			trigger_update(printer, 'message', stat);
		},
		globals_update: function(printer, values) {
			printers[printer].profile = values[0];
			var new_num_temps = values[1];
			var new_num_gpios = values[2];
			printers[printer].pin_names = values[3];
			printers[printer].led_pin = values[4];
			printers[printer].stop_pin = values[5];
			printers[printer].probe_pin = values[6];
			printers[printer].spiss_pin = values[7];
			printers[printer].probe_dist = values[8];
			printers[printer].probe_safe_dist = values[9];
			printers[printer].bed_id = values[10];
			printers[printer].fan_id = values[11];
			printers[printer].spindle_id = values[12];
			printers[printer].unit_name = values[13];
			printers[printer].timeout = values[14];
			printers[printer].feedrate = values[15];
			printers[printer].max_deviation = values[16];
			printers[printer].max_v = values[17];
			printers[printer].targetx = values[18];
			printers[printer].targety = values[19];
			printers[printer].zoffset = values[20];
			printers[printer].store_adc = values[21];
			printers[printer].park_after_print = values[22];
			printers[printer].sleep_after_print = values[23];
			printers[printer].cool_after_print = values[24];
			printers[printer].spi_setup = values[25];
			printers[printer].temp_scale_min = values[26];
			printers[printer].temp_scale_max = values[27];
			printers[printer].status = values[28];
			for (var i = printers[printer].num_temps; i < new_num_temps; ++i) {
				printers[printer].temps.push({
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
			printers[printer].temps.length = new_num_temps;
			printers[printer].num_temps = new_num_temps;
			for (var i = printers[printer].num_gpios; i < new_num_gpios; ++i) {
				printers[printer].gpios.push({
					name: null,
					pin: 0,
					state: 3,
					reset: 3
				});
			}
			printers[printer].gpios.length = new_num_gpios;
			printers[printer].num_gpios = new_num_gpios;
			trigger_update(printer, 'globals_update');
		},
		space_update: function(printer, index, values) {
			printers[printer].spaces[index].name = values[0];
			printers[printer].spaces[index].type = values[1];
			printers[printer].spaces[index].num_axes = values[2].length;
			printers[printer].spaces[index].num_motors = values[3].length;
			var current = [];
			for (var a = 0; a < printers[printer].spaces[index].num_axes; ++a)
				current.push(a < printers[printer].spaces[index].axis.length ? printers[printer].spaces[index].axis[a].current : NaN);
			printers[printer].spaces[index].axis = [];
			printers[printer].spaces[index].motor = [];
			for (var a = 0; a < printers[printer].spaces[index].num_axes; ++a) {
				printers[printer].spaces[index].axis.push({
					current: current[a],
					name: values[2][a][0],
					park: values[2][a][1],
					park_order: values[2][a][2],
					min: values[2][a][3],
					max: values[2][a][4],
					home_pos2: values[2][a][5]
				});
			}
			for (var m = 0; m < printers[printer].spaces[index].num_motors; ++m) {
				printers[printer].spaces[index].motor.push({
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
					printers[printer].spaces[index].axis[a].multiplier = values[4][a];
			}
			if (printers[printer].spaces[index].type == TYPE_DELTA) {
				for (var i = 0; i < 3; ++i) {
					printers[printer].spaces[index].motor[i].delta_axis_min = values[5][i][0];
					printers[printer].spaces[index].motor[i].delta_axis_max = values[5][i][1];
					printers[printer].spaces[index].motor[i].delta_rodlength = values[5][i][2];
					printers[printer].spaces[index].motor[i].delta_radius = values[5][i][3];
				}
				printers[printer].spaces[index].delta_angle = values[5][3];
			}
			if (printers[printer].spaces[index].type == TYPE_POLAR) {
				printers[printer].spaces[index].polar_max_r = values[5];
			}
			if (printers[printer].spaces[index].type == TYPE_EXTRUDER) {
				for (var i = 0; i < printers[printer].spaces[index].axis.length; ++i) {
					printers[printer].spaces[index].axis[i].extruder_dx = values[5][i][0];
					printers[printer].spaces[index].axis[i].extruder_dy = values[5][i][1];
					printers[printer].spaces[index].axis[i].extruder_dz = values[5][i][2];
				}
			}
			if (printers[printer].spaces[index].type == TYPE_FOLLOWER) {
				for (var i = 0; i < printers[printer].spaces[index].axis.length; ++i) {
					printers[printer].spaces[index].motor[i].follower_space = values[5][i][0];
					printers[printer].spaces[index].motor[i].follower_motor = values[5][i][1];
				}
			}
			trigger_update(printer, 'space_update', index);
		},
		temp_update: function(printer, index, values) {
			printers[printer].temps[index].name = values[0];
			printers[printer].temps[index].R0 = values[1];
			printers[printer].temps[index].R1 = values[2];
			printers[printer].temps[index].Rc = values[3];
			printers[printer].temps[index].Tc = values[4];
			printers[printer].temps[index].beta = values[5];
			printers[printer].temps[index].heater_pin = values[6];
			printers[printer].temps[index].fan_pin = values[7];
			printers[printer].temps[index].thermistor_pin = values[8];
			printers[printer].temps[index].fan_temp = values[9];
			printers[printer].temps[index].fan_duty = values[10];
			printers[printer].temps[index].heater_limit_l = values[11];
			printers[printer].temps[index].heater_limit_h = values[12];
			printers[printer].temps[index].fan_limit_l = values[13];
			printers[printer].temps[index].fan_limit_h = values[14];
			printers[printer].temps[index].hold_time = values[15];
			printers[printer].temps[index].value = values[16];
			trigger_update(printer, 'temp_update', index);
		},
		gpio_update: function(printer, index, values) {
			printers[printer].gpios[index].name = values[0];
			printers[printer].gpios[index].pin = values[1];
			printers[printer].gpios[index].state = values[2];
			printers[printer].gpios[index].reset = values[3];
			printers[printer].gpios[index].duty = values[4];
			printers[printer].gpios[index].value = values[5];
			trigger_update(printer, 'gpio_update', index);
		}
	};
}
// }}}

// {{{ Initialization.
function _reconnect() {
	trigger_update(null, 'connect', false);
	rpc = null;
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
	autodetect = true;
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

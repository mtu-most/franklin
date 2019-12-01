var TYPE_POLAR = 'polar';

function polar_get_value(ui, id) { // {{{
	return ui.machine.spaces[id[0][1]]['polar_' + id[1]];
} // }}}

function polar_set_value(ui, id, value, reply) { // {{{
	// [['module', 0, 'polar'], 'max_r']
	var o = {type: 'polar'};
	o[id[1]] = value;
	ui.machine.call('set_space', id[0][1], {module: o}, reply);
} // }}}

function polar_update(ui, index) { // {{{
	update_float(ui, [['module', index, 'polar'], 'max_r']);
} // }}}

function polar_draw(ui, context) { // {{{
	machinewidth = 2 * ui.machine.spaces[0].polar_max_r + 2;
	machineheight = 2 * ui.machine.spaces[0].polar_max_r + 2;
	center = [0, 0];
	outline = function(ui, c) {
		c.beginPath();
		c.arc(0, 0, ui.machine.spaces[0].polar_max_r, 0, 2 * Math.PI, false);
		c.stroke();
	};
	return [machinewidth, machineheight, center, outline];
} // }}}

function polar_load(machine, s, m, data) { // {{{
	machines[machine].spaces[s].motor[m].polar_x = data.x;
	machines[machine].spaces[s].motor[m].polar_y = data.y;
} // }}}

// UI modules. {{{
function Polar(ui, space) { // {{{
	var f = Float(ui, [['module', space, 'polar'], 'max_r'], 1, 1);
	return make_tablerow(ui, space_name(ui, space), f, ['rowtitle1'], undefined, TYPE_POLAR, space);
} // }}}

function setup_polar(desc, pos, top) { // {{{
	var ui = top.data;
	var ret = Create('div', 'setup expert');
	ret.update = function() { this.hide(ui.machine.spaces[0].type != TYPE_POLAR); };
	ret.Add([make_table(ui).AddMultipleTitles([
		'Polar',
		UnitTitle(ui, 'max r'),
	], [
		'htitle1',
		'title1'
	], [
		null,
		'maximum radius',
	]).AddMultiple(ui, 'space', Polar, false)]);
	return [ret, pos];
} // }}}
// }}}

AddEvent('setup', function () {
	space_types[TYPE_POLAR] = 'Polar';
	type_info[TYPE_POLAR] = {
		name: 'Polar',
		get_value: polar_get_value,
		set_value: polar_set_value,
		update: polar_update,
		draw: polar_draw,
		load: polar_load
	};
	ui_modules['Polar Setup'] = setup_polar;
});

// vim: set foldmethod=marker :

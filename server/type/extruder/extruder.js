var TYPE_EXTRUDER = 'extruder';

function extruder_get_value(ui, id) { // {{{
	return ui.machine.spaces[id[0][1][0]].axis[id[0][1][1]]['extruder_' + id[1]];
} // }}}

function extruder_set_value(ui, id, value, reply) { // {{{
	// [['module', [0, 1, 'axis'], 'extruder'], 'dx']
	var o = {type: 'extruder'};
	o[id[1]] = value;
	ui.machine.call('set_axis', [[id[0][1][0], id[0][1][1]]], {module: o}, reply);
} // }}}

function extruder_update(ui, index) { // {{{
	for (var a = 0; a < ui.machine.spaces[0].axis.length; ++a) {
		update_float(ui, [['module', [index, a, 'axis'], 'extruder'], 'dx']);
		update_float(ui, [['module', [index, a, 'axis'], 'extruder'], 'dy']);
		update_float(ui, [['module', [index, a, 'axis'], 'extruder'], 'dz']);
	}
} // }}}

function extruder_aload(machine, s, a, data) { // {{{
	machines[machine].spaces[s].axis[a].extruder_x = data.dx;
	machines[machine].spaces[s].axis[a].extruder_y = data.dy;
	machines[machine].spaces[s].axis[a].extruder_z = data.dz;
} // }}}

// UI modules. {{{
function Extruder(ui, space, axis) { // {{{
	if (space != 0)
		return null;
	var e = [['dx', 1], ['dy', 1], ['dz', 1]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float(ui, [['module', [space, axis, 'axis'], 'extruder'], e[i][0]], e[i][1], 1));
		e[i] = div;
	}
	return make_tablerow(ui, motor_name(ui, space, motor), e, ['rowtitle2'], undefined, TYPE_EXTRUDER, space);
} // }}}

function setup_extruder(desc, pos, top) { // {{{
	var ui = top.data;
	var ret = Create('div', 'setup expert');
	ret.update = function() {};
	ret.Add([make_table(ui).AddMultipleTitles([
		'Extruder',
		UnitTitle(ui, 'dx'),
		UnitTitle(ui, 'dy')
		UnitTitle(ui, 'dz')
	], [
		'htitle3',
		'title3',
		'title3'
	], [
		null,
		'x offset for this extruder',
		'y offset for this extruder',
		'z offset for this extruder'
	]).AddMultiple(ui, 'axis', Extruder)]);
	return [ret, pos];
} // }}}
// }}}

AddEvent('setup', function () {
	space_types[TYPE_EXTRUDER] = 'Extruder';
	type_info[TYPE_EXTRUDER] = {
		name: 'Extruder',
		get_value: extruder_get_value,
		set_value: extruder_set_value,
		update: extruder_update,
		draw: null,	// This should never be called.
		aload: extruder_aload
	};
	ui_modules['Extruder Setup'] = setup_extruder;
});

// TODO: set_value id parsing
// vim: set foldmethod=marker :

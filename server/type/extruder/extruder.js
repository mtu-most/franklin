var TYPE_EXTRUDER = 'extruder';

function extruder_update(ui, index) { // {{{
	for (var a = 0; a < ui.machine.spaces[1].axis.length; ++a) {
		update_float(ui, [['axis', [index, a]], [TYPE_EXTRUDER, 'dx']]);
		update_float(ui, [['axis', [index, a]], [TYPE_EXTRUDER, 'dy']]);
		update_float(ui, [['axis', [index, a]], [TYPE_EXTRUDER, 'dz']]);
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
		div.Add(Float(ui, [['axis', [space, axis]], [TYPE_EXTRUDER, e[i][0]]], e[i][1], 1));
		e[i] = div;
	}
	return make_tablerow(ui, axis_name(ui, space, axis), e, ['rowtitle2'], undefined, TYPE_EXTRUDER, space);
} // }}}

function setup_extruder(desc, pos, top) { // {{{
	var ui = top.data;
	var ret = Create('div', 'setup expert');
	ret.update = function() {};
	ret.Add([make_table(ui).AddMultipleTitles([
		'Extruder',
		UnitTitle(ui, 'dx'),
		UnitTitle(ui, 'dy'),
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
		update: extruder_update,
		draw: null,	// This should never be called.
		aload: extruder_aload
	};
	ui_modules['Extruder Setup'] = setup_extruder;
});

// TODO: set_value id parsing
// vim: set foldmethod=marker :

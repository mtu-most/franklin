var TYPE_FOLLOWER = 'follower';

function follower_update(ui, index) { // {{{
	for (var m = 0; m < ui.machine.spaces[index].motor.length; ++m) {
		var selects = get_elements(p, [['motor', [index, m]], 'spacemotor']);
		for (var s = 0; s < selects.length; ++s)
			update_motorselect(s);
	}
} // }}}

function follower_mload(machine, index, m, data) { // {{{
	machines[machine].spaces[index].motor[m].follower_spacemotor = [data.space, data.motor];
} // }}}

// UI modules. {{{
function Follower(ui, space, motor) { // {{{
	if (space != 2)
		return null;
	var e = MotorSelect(ui, [['motor', [space, motor]], [TYPE_FOLLOWER, 'spacemotor']]);
	return make_tablerow(ui, motor_name(ui, space, motor), [e], ['rowtitle1'], undefined, TYPE_FOLLOWER, space);
} // }}}

function setup_follower(desc, pos, top) { // {{{
	var ui = top.data;
	var ret = Create('div', 'setup expert');
	ret.AddElement('div').AddText('Number of Followers:').Add(Float(ui, [['space', 2], 'num_axes'], 0));
	ret.Add([make_table(ui).AddMultipleTitles([
		'Follower',
		'Motor'
	], [
		'htitle1',
		'title1'
	], [
		null,
		'Motor to follow.'
	]).AddMultiple(ui, 'motor', Follower)]);
	return [ret, pos];
} // }}}
// }}}

AddEvent('setup', function () {
	space_types[TYPE_FOLLOWER] = 'Follower';
	type_info[TYPE_FOLLOWER] = {
		name: 'Follower',
		update: follower_update,
		draw: function() { return [0, 0, [0, 0], function() {}]; },
		load: function() {},
		aload: function() {},
		mload: follower_mload
	};
	ui_modules['Follower Setup'] = setup_follower;
});

// TODO: set_value id parsing
// vim: set foldmethod=marker :

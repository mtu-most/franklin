var TYPE_DELTA = 'delta';

function delta_get_value(ui, id) { // {{{
	if (typeof id[0][1] == 'number')
		return ui.machine.spaces[id[0][1]]['delta_' + id[1]];
	return ui.machine.spaces[id[0][1][0]].motor[id[0][1][1]]['delta_' + id[1]];
} // }}}

function delta_set_value(ui, id, value, reply) { // {{{
	if (typeof id[0][1] == 'number') {
		// [['module', 0, 'delta'], 'angle']
		ui.machine.call('set_space', [id[0][1]], {module: {type: 'delta', angle: value}}, reply);
	}
	else if (id[0][1][1] === null) {
		// [['module', [0, null], 'delta'], 'radius']
		var o = {type: 'delta'};
		o[id[1]] = value;
		for (var n = 0; n < spaces[id[0][1][0]].num_motors; ++n)
			ui.machine.call('set_motor', [id[0][1][0], n], {module: o}, reply);
	}
	else {
		// [['module', [0, 1], 'delta'], 'radius']
		var o = {type: 'delta'};
		o[id[1]] = value;
		ui.machine.call('set_motor', [id[0][1]], {module: o}, reply);
	}
} // }}}

function delta_update(ui, index) { // {{{
	for (var d = 0; d < 3; ++d) {
		update_float(ui, [['module', [index, d], 'delta'], 'axis_min']);
		update_float(ui, [['module', [index, d], 'delta'], 'axis_max']);
		update_float(ui, [['module', [index, d], 'delta'], 'rodlength']);
		update_float(ui, [['module', [index, d], 'delta'], 'radius']);
	}
	update_float(ui, [['module', index, 'delta'], 'angle']);
} // }}}

function delta_draw(ui, context) { // {{{
	var radius = [];
	var length = [];
	for (var a = 0; a < 3; ++a) {
		radius.push(ui.machine.spaces[0].motor[a].delta_radius);
		length.push(ui.machine.spaces[0].motor[a].delta_rodlength);
	}
	//var origin = [[radius[0], 0], [radius[1] * -.5, radius[1] * .8660254037844387], [radius[2] * -.5, radius[2] * -.8660254037844387]];
	//var dx = [0, -.8660254037844387, .8660254037844387];
	//var dy = [1, -.5, -.5];
	var origin = [[radius[0] * -.8660254037844387, radius[0] * -.5], [radius[1] * .8660254037844387, radius[1] * -.5], [0, radius[2]]];
	var center = [0, 0];
	var dx = [.5, .5, -1];
	var dy = [-.8660254037844387, .8660254037844387, 0];
	var intersects = [];
	var intersect = function(x0, y0, r, x1, y1, dx1, dy1, positive) {
		// Find intersection of circle(x-x0)^2+(y-y0)^2==r^2 and line l=(x1,y1)+t(dx1,dy1); use positive or negative solution for t.
		// Return coordinate.
		var s = positive ? 1 : -1;
		var k = (dx1 * (x1 - x0) + dy1 * (y1 - y0)) / (dx1 * dx1 + dy1 * dy1);
		var t = s * Math.sqrt(r * r - (x1 - x0) * (x1 - x0) - (y1 - y0) * (y1 - y0) + k * k) - k;
		return [x1 + t * dx1, y1 + t * dy1];
	};
	var angles = [[null, null], [null, null], [null, null]];
	var maxx = 0, maxy = 0;
	for (var a = 0; a < 3; ++a) {
		intersects.push([]);
		for (var aa = 0; aa < 2; ++aa) {
			var A = (a + aa + 1) % 3;
			var point = intersect(origin[A][0], origin[A][1], length[A], origin[a][0], origin[a][1], dx[a], dy[a], aa);
			intersects[a].push(point);
			if (Math.abs(point[0]) > maxx)
				maxx = Math.abs(point[0]);
			if (Math.abs(point[1]) > maxy)
				maxy = Math.abs(point[1]);
			angles[A][aa] = Math.atan2(point[1] - origin[A][1], point[0] - origin[A][0]);
		}
	}
	outline = function(ui, c) {
		c.save(); // Rotate the outline.
		c.rotate(ui.machine.spaces[0].delta_angle);
		c.beginPath();
		c.moveTo(intersects[0][0][0], intersects[0][0][1]);
		for (var a = 0; a < 3; ++a) {
			var A = (a + 1) % 3;
			var B = (a + 2) % 3;
			c.lineTo(intersects[a][1][0], intersects[a][1][1]);
			c.arc(origin[B][0], origin[B][1], length[B], angles[B][1], angles[B][0], false);
		}
		c.closePath();
		c.stroke();
		for (var a = 0; a < 3; ++a) {
			var name = ui.machine.spaces[0].motor[a].name;
			var w = c.measureText(name).width;
			c.beginPath();
			c.save(); // Print axis name.
			c.translate(origin[a][0] + dy[a] * 10 - w / 2, origin[a][1] - dx[a] * 10);
			c.rotate(-ui.machine.spaces[0].delta_angle);
			c.scale(1, -1);
			c.fillText(name, 0, 0);
			c.restore();
		}
		c.restore();
	};
	var extra = context.measureText(ui.machine.spaces[0].motor[0].name).width + .02;
	machinewidth = 2 * (maxx + extra);
	machineheight = 2 * (maxy + extra);
	return [machinewidth, machineheight, center, outline];
} // }}}

function delta_load(machine, index, data) { // {{{
	machines[machine].spaces[index].delta_angle = data.angle;
} // }}}

function delta_mload(machine, index, m, data) { // {{{
	machines[machine].spaces[index].motor[m].delta_axis_min = data.axis_min;
	machines[machine].spaces[index].motor[m].delta_axis_max = data.axis_max;
	machines[machine].spaces[index].motor[m].delta_rodlength = data.rodlength;
	machines[machine].spaces[index].motor[m].delta_radius = data.radius;
} // }}}

// UI modules. {{{
function Delta(ui, space, motor) { // {{{
	if (space != 0)
		return null;
	var e = [['axis_min', 1], ['axis_max', 1], ['rodlength', 3], ['radius', 3]];
	for (var i = 0; i < e.length; ++i) {
		var div = Create('div');
		div.Add(Float(ui, [['module', [space, motor], 'delta'], e[i][0]], e[i][1], 1));
		e[i] = div;
	}
	return make_tablerow(ui, motor_name(ui, space, motor), e, ['rowtitle4'], undefined, TYPE_DELTA, space);
} // }}}

function Delta_space(ui, num) { // {{{
	if (num != 0)
		return null;
	var div = Create('div');
	div.Add(Float(ui, [['module', num, 'delta'], 'angle'], 2, Math.PI / 180));
	return make_tablerow(ui, space_name(ui, num), [div], ['rowtitle1'], undefined, TYPE_DELTA, num);
} // }}}

function setup_delta(desc, pos, top) { // {{{
	var ui = top.data;
	var ret = Create('div', 'setup expert');
	ret.update = function() { this.hide(ui.machine.spaces[0].type != TYPE_DELTA); };
	ret.Add([make_table(ui).AddMultipleTitles([
		'Delta',
		UnitTitle(ui, 'Min Distance'),
		UnitTitle(ui, 'Max Distance'),
		UnitTitle(ui, 'Rod Length'),
		UnitTitle(ui, 'Radius')
	], [
		'htitle4',
		'title4',
		'title4',
		'title4',
		'title4'
	], [
		null,
		'Minimum horizontal distance between tie rod pivot points.  Usually 0.',
		'Maximum horizontal distance between tie rod pivot points.  Usually Infinity.',
		'Length of the tie rods between pivot points.  Measure this with as high precision as possible.',
		'Horizontal distance between tie rod pivot points when the end effector is at (0, 0, 0).'
	]).AddMultiple(ui, 'motor', Delta)]);
	ret.Add([make_table(ui).AddMultipleTitles([
		'Delta',
		'Angle (°)'
	], [
		'htitle1',
		'title1'
	], [
		null,
		'Correction angle for the machine. (degrees)'
	]).AddMultiple(ui, 'space', Delta_space, false)]);
	return [ret, pos];
} // }}}
// }}}

AddEvent('setup', function () {
	space_types[TYPE_DELTA] = 'Delta';
	type_info[TYPE_DELTA] = {
		name: 'Delta',
		get_value: delta_get_value,
		set_value: delta_set_value,
		update: delta_update,
		draw: delta_draw,
		load: delta_load,
		aload: function() {},
		mload: delta_mload
	};
	ui_modules['Delta Setup'] = setup_delta;
});

// TODO: set_value id parsing
// vim: set foldmethod=marker :

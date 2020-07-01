function cartesian_draw(ui, context) {
	var x = ui.machine.spaces[0].axis[0];
	var y = ui.machine.spaces[0].axis[1];
	var w = x.max - x.min;
	var h = y.max - y.min;
	var center = [(x.min + x.max) / 2, (y.min + y.max) / 2];
	return [w, h, center, function() {}];
}

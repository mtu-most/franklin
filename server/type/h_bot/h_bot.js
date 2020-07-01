function h_bot_draw(ui, context) {
	var xaxis = ui.machine.spaces[0].axis[0];
	var yaxis = ui.machine.spaces[0].axis[1];
	machinewidth = xaxis.max - xaxis.min + .010;
	machineheight = yaxis.max - yaxis.min + .010;
	center = [(xaxis.min + xaxis.max) / 2, (yaxis.min + yaxis.max) / 2];
	outline = function(ui, c) {
		// Rectangle is always drawn; nothing else to do here.
	};
	return [machinewidth, machineheight, center, outline];
}

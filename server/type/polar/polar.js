function polar_draw(ui, context) {
	machinewidth = 2 * ui.machine.spaces[0].polar_max_r + 2;
	machineheight = 2 * ui.machine.spaces[0].polar_max_r + 2;
	center = [0, 0];
	outline = function(ui, c) {
		c.beginPath();
		c.arc(0, 0, ui.machine.spaces[0].polar_max_r, 0, 2 * Math.PI, false);
		c.stroke();
	};
	return [machinewidth, machineheight, center, outline];
}

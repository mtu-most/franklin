var TYPE_HBOT = 'h-bot';

function hbot_draw(ui, context) { // {{{
	var xaxis = ui.machine.spaces[0].axis[0];
	var yaxis = ui.machine.spaces[0].axis[1];
	machinewidth = xaxis.max - xaxis.min + .010;
	machineheight = yaxis.max - yaxis.min + .010;
	center = [(xaxis.min + xaxis.max) / 2, (yaxis.min + yaxis.max) / 2];
	outline = function(ui, c) {
		// Rectangle is always drawn; nothing else to do here.
	};
	return [machinewidth, machineheight, center, outline];
} // }}}

AddEvent('setup', function () {
	space_types[TYPE_HBOT] = 'H-bot';
	type_info[TYPE_HBOT] = {
		name: 'H-bot',
		draw: hbot_draw,
	};
	ui_modules['H-bot Setup'] = setup_hbot;
});

// vim: set foldmethod=marker :

function drawbot_draw(ui, context) {
	var motor = ui.machine.spaces[0].motor;
	var pos = [[motor[0].drawbot_x, motor[0].drawbot_y], [motor[1].drawbot_x, motor[1].drawbot_y]];
	var dx = pos[1][0] - pos[0][0];
	var dy = pos[1][1] - pos[0][1];
	var len = [motor[0].home_pos, motor[1].home_pos, Math.sqrt(dx * dx + dy * dy)];
	var Pu = pos[0][1] - Math.sqrt(len[0] * len[0] - dx * dx);
	var Pv = pos[1][1] - Math.sqrt(len[1] * len[1] - dx * dx);
	var end_u_inner = Math.acos((len[0] * len[0] + len[2] * len[2] - len[1] * len[1]) / (2 * len[0] * len[2]));
	var end_u = end_u_inner - Math.atan(dy / dx);
	var start_v_inner = Math.asin(Math.sin(end_u_inner) * len[0] / len[1]);
	var start_v = Math.atan(dx / dy) - start_v_inner + Math.PI / 2;
	var center = [(pos[0][0] + pos[1][0]) / 2, (pos[0][1] - len[0] / 2 + pos[1][1] - len[1] / 2) / 2];
	var outline = function(ui, c) {
		c.beginPath();
		c.moveTo(pos[0][0], Pu);
		c.lineTo(pos[0][0], pos[0][1]);
		c.lineTo(pos[1][0], pos[1][1]);
		c.lineTo(pos[1][0], Pv);
		c.arc(pos[0][0], pos[0][1], len[0], -Math.acos(dx / len[0]), -end_u, true);
		c.arc(pos[1][0], pos[1][1], len[1], -start_v, Math.acos(dx / len[1]) - Math.PI, true);
		c.stroke();
	};
	machinewidth = pos[1][0] - pos[0][0];
	machineheight = Math.max(len[0], len[1]) * 1.4;
	return [machinewidth, machineheight, center, outline];
}

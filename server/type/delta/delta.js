function delta_draw(ui, context) {
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
		c.rotate(ui.machine.spaces[0].delta_angle * Math.PI / 180);
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
			c.rotate(-ui.machine.spaces[0].delta_angle * Math.PI / 180);
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
}

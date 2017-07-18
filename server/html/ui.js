/*
Normal operation:
A bin contains exactly one element.
There are some container elements, which contain bins:
- split: can split horizontal or vertical, position defined by first or second, as pixels or percentage.
- notebook: tabs can be on bottom or top.

During setup, extra elements are visible:
- For every bin, a button to select the contents.
- For every split, controls for the settings.
- For every notebook, a button to add more tabs, remove tabs, and move the tabs position.

Interface: {

class Content:
	destroy()
	copy()
	serialize()
	build(string)

class Bin:
	Content content
	destroy()
	update(bool)
	div selector
	add_split()
	add_notebook()

class Split(Content):
	Bin first, second
	div controls
	swap()

class Notebook(Content):
	div tabs
	Bin pages[]
	div controls


DOM:
<div class='Bin'>
	<div class='Notebook'>
		<div class='Bin Tab'>
			<div class='Split'>
				<div class='Bin Left'>...</div>
				<div class='Bin Right'>...</div>
			</div>
		</div>
		<div class='Bin Tab'>...</div>
	</div>
</div>
}
*/

function ui_build(string, data, pos) {
	if (pos === undefined)
		return ui_build(string, data, 0)[0];
	var ret, sub;
	if (string[pos] == '{') {
		// Split: {AB[dD][hv][0-9.]+[p%]}
		pos += 1;
		ret = new Split(data);
		// Settings.
		if (string[pos++] == 'd')
			ret.dominant_first = false;
		if (string[pos++] == 'v')
			ret.horizontal = false;
		var num = /[0-9.]+/.exec(string.substr(pos));
		pos += num[0].length;
		ret.value = Number(num[0]);
		if (string[pos++] == 'p')
			ret.pixels = true;
		// First child.
		sub = ui_build(string, data, pos);
		ret.first.set_content(sub[0]);
		pos = sub[1];
		// Second child.
		sub = ui_build(string, data, pos);
		ret.second.set_content(sub[0]);
		pos = sub[1]
		if (string[pos++] != '}')
			console.error('invalid final character for Split building', string, pos);
		return [ret, pos];
	}
	if (string[pos] == '[') {
		pos += 1;
		ret = new Notebook();
		// TODO.
	}
	// Other things.
	if (string[pos++] != '(') {
		console.error('invalid initial character for build', string, pos);
		return [null, pos];
	}
	for (var n in ui_modules) {
		if (string.substr(pos, n.length + 1) != n + ':')
			continue;
		pos += n.length + 1;
		sub = ui_modules[n](string, pos, data);
		sub[0].name = n;
		pos = sub[1];
		if (string[pos++] != ')')
			console.error('invalid final character for custom build', string, pos);
		return [sub[0], pos];
	}
	console.error('No valid module found', string, pos);
	return [null, pos];
}

function Bin(data) {
	var self = Create('div', 'Bin');
	self.data = data;
	self.style.position = 'absolute';
	self.style.left = '0px';
	self.style.right = '0px';
	self.style.top = '0px';
	self.style.bottom = '0px';
	self.control = self.AddElement('div');
	self.control.style.position = 'absolute';
	self.control.style.width = '100%';
	self.control.style.bottom = '0px';
	self.control.style.background = 'yellow';
	self.content_selector = self.control.AddElement('select');
	for (var m in ui_modules)
		self.content_selector.AddElement('option').AddText(m).value = m;
	self.content_selector.AddElement('option').AddText('Add Split').value = 'Split';
	self.content_selector.AddElement('option').AddText('Add Notebook').value = 'Notebook';
	self.content_selector.AddEvent('change', function() {
		var target = self.content_selector.selectedOptions[0].value;
		if (target == 'Split')
			self.add_split();
		else if (target == 'Notebook')
			self.add_notebook();
		else {
			self.destroy();
			var c = ui_modules[target]('', 0, self.data);
			c.name = target;
			self.set_content(c)
		}
	});
	self.set_content = function(element) {
		this.ClearAll();
		this.Add(self.control);
		this.content = this.Add(element);
		element.parent_bin = this;
		this.update();
	};
	self.destroy = function() {
		if (this.content.destroy)
			this.content.destroy();
	};
	self.update = function() {
		if (!this.content)
			return;
		if (this.content.name == 'Split' || this.content.name == 'Notebook') {
			this.control.style.display = 'none';
		}
		else {
			this.control.style.display = 'block';
			for (var i = 0; i < this.content_selector.options.length; ++i) {
				if (this.content_selector[i].value == this.content.name)
					this.content_selector.selectedIndex = i;
			}
		}
		if (this.content.update)
			this.content.update();
	};
	self.add_split = function() {
		var content = this.content;
		var copy = this.copy();
		this.removeChild(content);
		this.set_content(new Split(this.data));
		this.content.first.set_content(content);
		this.content.second.set_content(copy);
		this.update();
	};
	self.add_notebook = function() {
		var content = this.content;
		this.content = new Notebook();
		this.content.add_tab(content);
		this.update();
	};
	self.serialize = function() {
		if (!this.content.serialize)
			return '';
		return this.content.serialize();
	};
	self.copy = function() {
		if (this.content.copy)
			return this.content.copy();
		var ret = ui_modules[this.content.name](this.serialize(), 0, this.data)[0];
		ret.name = this.content.name;
		return ret;
	};
	return self;
}

var _Split_id = 0;
function Split(data) {
	var self = Create('div', 'Split');
	self.name = 'Split';
	self.data = data;
	self.control = self.AddElement('div', 'Control'); // {
	self.control.style.zIndex = 10;
	self.control.style.display = 'block';
	self.control.style.position = 'absolute';
	self.control.style.top = '0px';
	self.control.style.left = '0px';
	self.control_x = 0;
	self.control_y = 0;
	self.control.style.background = 'yellow';
	self.control.style.border = '2px solid black';
	var d = self.control.AddElement('div').AddText('Split Controls');
	d.style.textAlign = 'center';
	d.style.background = '#cc0';
	d = self.control.AddElement('div');
	d.style.textAlign = 'center';
	self.x = NaN;
	self.y = NaN;
	d.AddEvent('mousedown', function(event) {
		console.info('down');
		self.x = event.pageX;
		self.y = event.pageY;
		self.pos = [self.control_x, self.control_y];
		event.preventDefault();
	});
	d.AddEvent('mouseup', function(event) {
		console.info('stop');
		self.x = NaN;
		self.pos = [self.control.style.left, self.control.style.top];
		event.preventDefault();
	});
	d.AddEvent('mousemove', function(event) {
		if (isNaN(self.x) || !(event.buttons & 1)) {
			console.info('no');
			self.x = NaN;
			return;
		}
		console.info('move', self.x, self.pos[0], event.pageX);
		self.control_x = self.pos[0] + event.pageX - self.x;
		self.control_y = self.pos[1] + event.pageY - self.y;
		self.control.style.left = self.control_x + 'px';
		self.control.style.top = self.control_y + 'px';
		event.preventDefault();
	});
	self.radio_v_first = d.AddElement('input');
	self.radio_v_first.type = 'radio';
	self.radio_v_first.name = 'Split' + _Split_id;
	self.radio_v_first.AddEvent('click', function() {
		self.horizontal = false;
		self.dominant_first = true;
		self.update();
	});
	self.radio_h_first = self.control.AddElement('input');
	self.radio_h_first.type = 'radio';
	self.radio_h_first.style.float = 'left';
	self.radio_h_first.name = 'Split' + _Split_id;
	self.radio_h_first.AddEvent('click', function() {
		self.horizontal = true;
		self.dominant_first = true;
		self.update();
	});
	self.radio_h_second = self.control.AddElement('input');
	self.radio_h_second.type = 'radio';
	self.radio_h_second.style.float = 'right';
	self.radio_h_second.name = 'Split' + _Split_id;
	self.radio_h_second.AddEvent('click', function() {
		self.horizontal = true;
		self.dominant_first = false;
		self.update();
	});
	d = self.control.AddElement('div');
	d.style.textAlign = 'center';
	d.style.clear = 'both';
	self.radio_v_second = d.AddElement('input');
	self.radio_v_second.type = 'radio';
	self.radio_v_second.name = 'Split' + _Split_id;
	self.radio_v_second.AddEvent('click', function() {
		self.horizontal = false;
		self.dominant_first = false;
		self.update();
	});
	d = self.control.AddElement('div');
	self.input_value = d.AddElement('input');
	self.input_value.style.width = '5em';
	self.input_value.type = 'number';
	self.input_value.min = 0;
	self.input_value.AddEvent('change', function() {
		self.value = Number(this.value);
		self.update();
	});
	self.type_select = d.AddElement('select');
	self.type_select.AddElement('option').AddText('px').value = 'p';
	self.type_select.AddElement('option').AddText('%').value = '%';
	self.type_select.AddEvent('change', function() {
		self.pixels = this.value != '%';
		self.update();
	});
	var button = self.control.AddElement('button').AddText('Remove Split').AddEvent('click', function() {
		var content = self.first.content;
		self.first.content = {};
		self.first.removeChild(content);
		self.destroy();
		self.parent_bin.set_content(content);
		self.parent_bin.update();
	});
	button.type = 'button';
	button.style.width = '100%';
	_Split_id += 1;
	// }
	self.dominant_first = true;
	self.horizontal = true;
	self.pixels = false;
	self.value = 50;
	self.first = self.Add(new Bin(data));
	self.second = self.Add(new Bin(data));
	self.update = function () {
		this.input_value.value = this.value;
		this.type_select.selectedIndex = this.pixels ? 0 : 1;
		var clear = function(target) {
			target.style.position = 'absolute';
			target.style.left = '0px';
			target.style.top = '0px';
			target.style.right = '0px';
			target.style.bottom = '0px';
			target.style.width = '';
			target.style.height = '';
		}
		clear(this.first);
		clear(this.second);
		var attr = this.horizontal ? ['left', 'right', 'width'] : ['top', 'bottom', 'height'];
		var unit = this.pixels ? 'px' : '%';
		if (this.dominant_first) {
			if (this.horizontal)
				this.radio_h_first.checked = true;
			else
				this.radio_v_first.checked = true;
			this.first.style[attr[2]] = this.value + unit;
			this.first.style[attr[1]] = '';
			this.second.style[attr[0]] = this.value + unit;
		}
		else {
			if (this.horizontal)
				this.radio_h_second.checked = true;
			else
				this.radio_v_second.checked = true;
			this.first.style[attr[1]] = this.value + unit;
			this.second.style[attr[2]] = this.value + unit;
			this.second.style[attr[0]] = '';
		}
		this.first.update();
		this.second.update();
	};
	self.destroy = function() {
		this.first.destroy();
		this.second.destroy();
	};
	self.serialize = function() {
		return '{' +
			(this.dominant_first ? 'D' : 'd') +
			(this.horizontal ? 'h' : 'v') +
			this.value + (this.pixels ? 'p' : '%') +
			this.first.serialize() +
			this.second.serialize() +
			'}';
	};
	self.copy = function() {
		var ret = new Split(this.data);
		ret.first.set_content(this.first.copy());
		ret.second.set_content(this.second.copy());
		return ret;
	};
	return self;
}

/*
function Dummy(setup, pos) {
	return [Create('div').AddText('dummy'), pos];
}

var ui_modules = {'dummy': Dummy};

window.AddEvent('load', function() {
	var r = ui_build('{dh25%(dummy:){Dv25%(dummy:)(dummy:)}}');
	document.getElementById('ui').Add(new Bin()).set_content(r);
});
*/

// vim: set foldmethod=marker foldmarker={,} :

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

class Bin:
	Content content	  # this is any html element.
	destroy()
	update(bool)
	div selector
	add_split()
	add_tabs()

class Split:
	Bin first, second
	div controls
	swap()

class Tabs:
	div tabs
	Bin pages[]
	div controls


DOM:
<div class='Bin'>
	<div class='Tabs'>
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
	if (pos === undefined) {
		return ui_build(string, data, 0)[0];
	}
	var ret, sub;
	if (string[pos] == '{') {
		// Split: {AB[dD][hv][0-9.]+[p%]}
		pos += 1;
		ret = new Split(data, false);
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
		pos = sub[1];
		if (string[pos++] != '}')
			console.error('invalid final character for Split building', string, pos);
		return [ret, pos];
	}
	if (string[pos] == '[') {
		pos += 1;
		ret = new Tabs(data, false);
		var t = 0;
		var selectedpos = string.indexOf(':', pos);
		var selected = Number(string.substr(pos, selectedpos - pos));
		pos = selectedpos + 1;
		while (string[pos] != ']') {
			if (string[pos] === undefined) {
				console.info('string ended too soon for Tabs building', string, pos);
				return [ret, pos];
			}
			if (string[pos] == '*') {
				pos += 1;
				ret.selected = t;
			}
			var namepos = string.indexOf(':', pos);
			var tabname = string.substr(pos, namepos - pos);
			pos = namepos + 1;
			sub = ui_build(string, data, pos);
			ret.add_tab(sub[0]);
			ret.tabs[t].tabname = decodeURIComponent(tabname);
			pos = sub[1];
			t += 1;
		}
		pos += 1;
		ret.select(selected);
		return [ret, pos];
	}
	// Other things.
	if (string[pos++] != '(') {
		console.info('invalid initial character for build "' + string + '"[' + pos + ']');
		return [null, pos];
	}
	for (var n in ui_modules) {
		if (string.substr(pos, n.length + 1) != n + ':')
			continue;
		pos += n.length + 1;
		sub = ui_modules[n](string, pos, data, false);
		sub[0].name = n;
		pos = sub[1];
		if (string[pos++] != ')')
			console.error('invalid final character for custom build', string, pos);
		return [sub[0], pos];
	}
	console.error('No valid module found', string, pos);
	return [null, pos];
}

// A Bin is a container that can hold any type of content.
function Bin(data, configuring) {
	var self = Create('div', 'Bin');
	self.data = data;
	if (!data)
		console.error(1);
	self.style.position = 'absolute';
	self.style.left = '0px';
	self.style.right = '0px';
	self.style.top = '0px';
	self.style.bottom = '0px';
	self.control = self.AddElement('div');
	self.configuring = configuring ? true : false;	// This turns undefined into false.
	self.control.style.display = configuring ? '' : 'none';
	self.control.style.position = 'absolute';
	self.control.style.width = '100%';
	self.control.style.bottom = '0px';
	self.control.style.background = 'yellow';
	self.content_selector = self.control.AddElement('select');
	for (var m in ui_modules)
		self.content_selector.AddElement('option').AddText(m).value = m;
	self.content_selector.AddElement('option').AddText('Add Split').value = 'Split';
	self.content_selector.AddElement('option').AddText('Add Tabs').value = 'Tabs';
	self.content_selector.AddEvent('change', function() {
		var target = self.content_selector.selectedOptions[0].value;
		if (target == 'Split')
			self.add_split();
		else if (target == 'Tabs')
			self.add_tabs();
		else {
			self.destroy();
			var c = ui_modules[target]('', 0, self.data, self.configuring)[0];
			c.name = target;
			self.set_content(c);
		}
	});
	self.set_content = function(element) {
		this.ClearAll();
		this.Add(self.control);
		this.content = this.Add(element);
		element.parent_bin = this;
		element.hide = function(hidden) { return this.parent_bin.hide(hidden); };
		this.config(this.configuring);
		this.update();
	};
	self.destroy = function() {
		if (this.content && this.content.destroy)
			this.content.destroy();
	};
	self.update = function() {
		if (!this.content)
			return;
		if (!this.configuring || this.content.name == 'Split' || this.content.name == 'Tabs') {
			this.control.style.display = 'none';
		}
		else {
			this.control.style.display = '';
			for (var i = 0; i < this.content_selector.options.length; ++i) {
				if (this.content_selector[i].value == this.content.name)
					this.content_selector.selectedIndex = i;
			}
		}
		if (this.content.update)
			this.content.update();
	};
	self.config = function(configuring) {
		this.configuring = configuring;
		if (configuring)
			this.update();
		else
			this.control.style.display = 'none';
		if (this.content && this.content.config)
			this.content.config(configuring);
		this.update();
	};
	self.add_split = function() {
		var content = this.content;
		var copy = this.copy();
		this.removeChild(content);
		this.set_content(new Split(this.data, this.configuring));
		this.content.first.set_content(content);
		this.content.second.set_content(copy);
		this.update();
	};
	self.add_tabs = function() {
		var content = this.content;
		this.removeChild(content);
		this.set_content(new Tabs(this.data, this.configuring));
		this.content.add_tab(content);
		this.update();
	};
	self.serialize = function() {
		if (!this.content.serialize)
			return '(' + this.content.name + ':' + ')';
		return this.content.serialize();
	};
	self.copy = function() {
		if (this.content.copy)
			return this.content.copy();
		var ret = ui_modules[this.content.name](this.serialize(), 0, this.data, this.configuring)[0];
		ret.name = this.content.name;
		return ret;
	};
	self.hide = function(hidden) {}; // This function is implemented by Split and Tabs.
	return self;
}

// A Split is a special content type that holds two bins, and provides a split view at both.
var _Split_id = 0;
function Split(data, configuring) {
	var self = Create('div', 'Split');
	self.name = 'Split';
	self.data = data;
	self.configuring = configuring ? true : false; // Treat undefined as false.
	self.control = self.AddElement('div', 'Control'); // {
	self.control.style.zIndex = 10;
	self.control.style.display = configuring ? '' : 'none';
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
		self.x = event.pageX;
		self.y = event.pageY;
		self.pos = [self.control_x, self.control_y];
		event.preventDefault();
	});
	d.AddEvent('mouseup', function(event) {
		self.x = NaN;
		self.pos = [self.control.style.left, self.control.style.top];
		event.preventDefault();
	});
	d.AddEvent('mousemove', function(event) {
		if (isNaN(self.x) || !(event.buttons & 1)) {
			self.x = NaN;
			return;
		}
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
	self.first = self.Add(new Bin(data, self.configuring));
	self.second = self.Add(new Bin(data, self.configuring));
	self.hide_first = false;
	self.hide_second = false;
	var hide = function(hidden) {
		if (this == self.first) {
			if (self.hide_first == hidden)
				return;
			self.hide_first = hidden;
		}
		else {
			if (self.hide_second == hidden)
				return;
			self.hide_second = hidden;
		}
		if (self.parent_bin)
			self.parent_bin.hide(self.hide_first && self.hide_second);
		self.update();
	};
	self.first.hide = hide;
	self.second.hide = hide;
	self.update = function() {
		this.first.update();
		this.second.update();
		if (!this.configuring && this.hide_first && this.hide_second) {
			this.control.style.display = 'none';
			return;
		}
		this.control.style.display = this.configuring ? '' : 'none';
		var clear = function(target) {
			target.style.position = 'absolute';
			target.style.left = '0px';
			target.style.top = '0px';
			target.style.right = '0px';
			target.style.bottom = '0px';
			target.style.width = '';
			target.style.height = '';
		};
		clear(this.first);
		clear(this.second);
		if (!this.configuring && this.hide_first) {
			this.first.style.display = 'none';
			this.second.style.display = '';
			return;
		}
		if (!this.configuring && this.hide_second) {
			this.first.style.display = '';
			this.second.style.display = 'none';
			return;
		}
		this.first.style.display = '';
		this.second.style.display = '';
		this.input_value.value = this.value;
		this.type_select.selectedIndex = this.pixels ? 0 : 1;
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
		var ret = new Split(this.data, this.configuring);
		ret.first.set_content(this.first.copy());
		ret.second.set_content(this.second.copy());
		return ret;
	};
	self.config = function(configuring) {
		this.configuring = configuring;
		this.control.style.display = this.configuring ? '' : 'none';
		this.first.config(configuring);
		this.second.config(configuring);
	};
	return self;
}

function Tabs(data, configuring) {
	var self = Create('div', 'Tabs');
	self.name = 'Tabs';
	self.data = data;
	self.configuring = configuring ? true : false; // Treat undefined as false.
	self.bar = self.AddElement('div');
	self.bar.style.position = 'absolute';
	self.bar.style.top = '0px';
	self.bar.style.width = '100%';
	self.bar.style.height = '2em';
	self.bar.style.background = '#cfc';
	self.plus = self.bar.AddElement('div', 'tab').AddText('+').AddEvent('click', function() {
		self.add_tab(self.tabs[self.selected].copy());
	});
	self.plus.style.float = 'right';
	self.tabs = [];
	self.selected = -1;
	self.add_tab = function(content) {
		var tab = this.Add(new Bin(this.data, this.configuring));
		tab.style.position = 'absolute';
		tab.style.width = '100%';
		tab.style.bottom = '0px';
		tab.style.top = '2em';
		tab.tabname = 'New Tab';
		tab.titlebox = this.bar.AddElement('div', 'tab');
		tab.titlebox.AddEvent('click', function() {
			self.select(self.tabs.indexOf(tab));
		});
		tab.titlespan = tab.titlebox.AddElement('span');
		tab.killbutton = tab.titlebox.AddElement('button').AddText('×').AddEvent('click', function() {
			tab.destroy();
			self.bar.removeChild(tab.titlebox);
			self.removeChild(tab);
			self.tabs.splice(self.tabs.indexOf(tab), 1);
			// Removing the last tab removes the parent Tabs object instead.
			if (self.tabs.length == 0) {
				self.destroy();
				self.parent_bin.removeChild(self);
				tab.style.top = '0px';
				self.parent_bin.set_content(tab);
			}
		});
		tab.killbutton.type = 'button';
		tab.killbutton.style.display = this.configuring ? 'inline' : 'none';
		tab.hide = function(hidden) {
			if (this.hidden == hidden)
				return;
			this.hidden = hidden;
			self.update();
		};
		this.tabs.push(tab);
		tab.set_content(content);
		this.select(this.tabs.length - 1);
		this.config(this.configuring);
		return tab;
	};
	self.select = function(index) {
		this.selected = index;
		this.update();
	};
	self.hide = function(hidden) {
		if (self.hidden == hidden)
			return;
		self.hidden = hidden;
		if (self.parent_bin)
			self.parent_bin.hide(hidden);
	};
	self.update = function() {
		for (var t = 0; t < this.tabs.length; ++t)
			this.tabs[t].update();
		while (this.selected >= 0 && this.tabs[this.selected].hidden && !this.configuring)
			this.selected -= 1;
		if (this.selected < 0) {
			this.selected = this.tabs.length - 1;
			while (this.selected >= 0 && this.tabs[this.selected].hidden && !this.configuring)
				this.selected -= 1;
			if (this.selected < 0) {
				this.hide(true);
				return;
			}
		}
		for (var t = 0; t < this.tabs.length; ++t) {
			if (t == this.selected) {
				this.tabs[t].titlebox.AddClass('active');
				this.tabs[t].style.display = '';
				if (this.configuring) {
					if (this.selected != this.last_selected) {
						this.tabs[t].killbutton.style.display = '';
						var tabinput = this.tabs[t].titlespan.ClearAll().AddElement('input', 'tabinput');
						tabinput.type = 'text';
						tabinput.value = this.tabs[t].tabname;
						tabinput.tab = this.tabs[t];
						tabinput.AddEvent('change', function() {
							this.tab.tabname = this.value;
						});
						this.last_selected = this.selected;
					}
					continue;
				}
			}
			else {
				this.tabs[t].titlebox.RemoveClass('active');
				this.tabs[t].style.display = 'none';
			}
			this.tabs[t].killbutton.style.display = 'none';
			this.tabs[t].titlespan.ClearAll().AddText(this.tabs[t].tabname);
		}
	};
	self.destroy = function() {
		for (var t = 0; t < this.tabs.length; ++t)
			this.tabs[t].destroy();
	};
	self.serialize = function() {
		var ret = '[' + this.selected + ':';
		for (var t = 0; t < this.tabs.length; ++t) {
			if (t == this.selected)
				ret += '*';
			ret += encodeURIComponent(this.tabs[t].tabname) + ':' + this.tabs[t].serialize();
		}
		return ret + ']';
	};
	self.copy = function() {
		var ret = new Tabs(this.data, this.configuring);
		for (var t = 0; t < this.tabs.length; ++t)
			ret.add_tab(this.tabs[t].copy());
		return ret;
	};
	self.config = function(configuring) {
		var tabselement = this;
		this.configuring = configuring;
		this.plus.style.display = this.configuring ? '' : 'none';
		for (var t = 0; t < this.tabs.length; ++t)
			this.tabs[t].config(configuring);
		this.update();
	};
	return self;
}

/*
Example dummy use.

// Content module.
function Dummy(setup, pos) {
	return [Create('div').AddText('dummy'), pos];
}

// Definition of all modules.
var ui_modules = {'dummy': Dummy};

// Create a bin and fill it with the content module.
window.AddEvent('load', function() {
	var r = ui_build('{dh25%(dummy:){Dv25%(dummy:)(dummy:)}}');
	document.getElementById('ui').Add(new Bin()).set_content(r);
});
*/

// vim: set foldmethod=marker foldmarker={,} :

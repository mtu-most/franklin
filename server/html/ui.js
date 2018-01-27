/*
Normal operation:
A bin contains exactly one element.
There are some container elements, which contain bins:
- split: can split horizontal or vertical, position defined by first or second, as em or percentage.
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

function ui_build(string, top, pos) {
	if (pos === undefined) {
		return ui_build(string, top, 0)[0];
	}
	var ret, sub;
	if (string[pos] == '{') {
		// Split: {AB[dD][hv][0-9.]+[p%]}
		pos += 1;
		ret = new Split(top, false);
		// Settings.
		if (string[pos++] == 'd')
			ret.dominant_first = false;
		if (string[pos++] == 'v')
			ret.horizontal = false;
		var num = /[0-9.]+/.exec(string.substr(pos));
		pos += num[0].length;
		ret.value = Number(num[0]);
		ret.em = string[pos++] == 'm';
		// First child.
		sub = ui_build(string, top, pos);
		if (sub[0] !== null)
			ret.first.set_content(sub[0]);
		pos = sub[1];
		// Second child.
		sub = ui_build(string, top, pos);
		if (sub[0] !== null)
			ret.second.set_content(sub[0]);
		pos = sub[1];
		if (string[pos++] != '}')
			console.error('invalid final character for Split building', string, pos);
		return [ret, pos];
	}
	if (string[pos] == '[') {
		pos += 1;
		ret = new Tabs(top, false);
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
			sub = ui_build(string, top, pos);
			if (sub[0] !== null) {
				ret.add_tab(sub[0]);
				ret.tabs[t].tabname = decodeURIComponent(tabname);
				t += 1;
			}
			pos = sub[1];
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
		sub = ui_modules[n](string, pos, top, false);
		sub[0].name = n;
		pos = sub[1];
		if (string[pos++] != ')')
			console.error('invalid final character for custom build', string, pos);
		return [sub[0], pos];
	}
	// Skip over invalid data.
	var oldpos = pos;
	while (pos < string.length && string[pos] != ')')
		pos += 1;
	if (pos < string.length)
		pos += 1;
	console.error('No valid module found', string.substr(oldpos - 1, pos - oldpos + 1));
	return [null, pos];
}

// A Bin is a container that can hold any type of content.
function Bin(top, configuring) {
	var self = Create('div', 'Bin');
	self.emsize = function() {
		var ret;
		if (window.getComputedStyle) {
			ret = window.getComputedStyle(self, null).fontSize;
			if (ret.substring(ret.length - 2) == 'px')
				return Number(ret.substring(0, ret.length - 2));
		}
		return 16;
	};
	self.px2em = function(px) { return px / self.emsize(); };
	self.em2px = function(em) { return em * self.emsize(); };
	self.top = top;
	self.style.position = 'absolute';
	self.style.left = '0em';
	self.style.right = '0em';
	self.style.top = '0em';
	self.style.bottom = '0em';
	self.control = self.AddElement('div');
	self.configuring = configuring ? true : false;	// This turns undefined into false.
	self.control.style.display = (configuring && top.show_hidden) ? '' : 'none';
	self.control.style.position = 'absolute';
	self.control.style.width = '100%';
	self.control.style.bottom = '0em';
	self.control.style.background = 'yellow';
	self.content_selector = self.control.AddElement('select');
	var sorted_names = [];
	for (var m in ui_modules)
		sorted_names.push(m);
	sorted_names.sort();
	for (var m = 0; m < sorted_names.length; ++m)
		self.content_selector.AddElement('option').AddText(sorted_names[m]).value = sorted_names[m];
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
			var c = ui_modules[target]('', 0, self.top, self.configuring)[0];
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
		if (configuring) {
			if (this === this.top)
				this.splitcontrol.style.display = '';
		}
		else {
			this.control.style.display = 'none';
			if (this === this.top && this.splitcontrol)
				this.splitcontrol.style.display = 'none';
		}
		if (this.content && this.content.config)
			this.content.config(configuring);
		this.update();
	};
	self.add_split = function() {
		var content = this.content;
		var copy = this.copy();
		this.removeChild(content);
		this.set_content(new Split(this.top, this.configuring));
		this.content.first.set_content(content);
		this.content.second.set_content(copy);
		this.update();
	};
	self.add_tabs = function() {
		var content = this.content;
		this.removeChild(content);
		this.set_content(new Tabs(this.top, this.configuring));
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
		var ret = ui_modules[this.content.name](this.serialize(), 0, this.top, this.configuring)[0];
		ret.name = this.content.name;
		return ret;
	};
	self.hide = function(hidden) {}; // This function is implemented by Split and Tabs.
	return self;
}

// A Split is a special content type that holds two bins, and provides a split view at both.
function Split(top, configuring) {
	var self = Create('div', 'Split');
	self.name = 'Split';
	self.top = top;
	self.configuring = configuring ? true : false; // Treat undefined as false.
	self.dominant_first = true;
	self.horizontal = true;
	self.em = false;
	self.value = 50;
	self.first = self.Add(new Bin(top, self.configuring));
	self.second = self.Add(new Bin(top, self.configuring));
	self.controldiv = self.AddElement('div');
	self.control = self.controldiv.AddElement('input');
	self.controldiv.style.position = 'absolute';
	self.control.type = 'radio';
	self.control.name = 'ActiveSplit' + top.object_id;
	self.control.AddEvent('click', function() {
		top.active_split = self;
		self.update();
	});
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
		var clear = function(target) {
			target.style.position = 'absolute';
			target.style.left = '0em';
			target.style.top = '0em';
			target.style.right = '0em';
			target.style.bottom = '0em';
			target.style.width = '';
			target.style.height = '';
		};
		clear(this.first);
		clear(this.second);
		if (!(this.configuring && this.top.show_hidden) && this.hide_first) {
			this.first.style.display = 'none';
			this.second.style.display = '';
			return;
		}
		if (!(this.configuring && this.top.show_hidden) && this.hide_second) {
			this.first.style.display = '';
			this.second.style.display = 'none';
			return;
		}
		this.first.style.display = '';
		this.second.style.display = '';
		if (top.active_split === this) {
			top.split_input.value = this.value;
			top.split_type_select.selectedIndex = this.em ? 0 : 1;
		}
		var attr = this.horizontal ? ['left', 'right', 'width'] : ['top', 'bottom', 'height'];
		var unit = this.em ? 'em' : '%';
		if (this.dominant_first) {
			if (top.active_split === this) {
				if (this.horizontal)
					top.radio_h_first.checked = true;
				else
					top.radio_v_first.checked = true;
			}
			this.first.style[attr[2]] = this.value + unit;
			this.first.style[attr[1]] = '';
			this.second.style[attr[0]] = this.value + unit;
		}
		else {
			if (top.active_split === this) {
				if (this.horizontal)
					top.radio_h_second.checked = true;
				else
					top.radio_v_second.checked = true;
			}
			this.first.style[attr[1]] = this.value + unit;
			this.second.style[attr[2]] = this.value + unit;
			this.second.style[attr[0]] = '';
		}
		if (this.configuring) {
			this.controldiv.style.display = '';
			var nonzero = function(value) { return value == '0em' ? '' : value; };
			if (this.horizontal) {
				this.controldiv.style.top = '50%';
				this.controldiv.style.bottom = '';
				this.controldiv.style.left = nonzero(this.second.style.left);
				this.controldiv.style.right = nonzero(this.second.style.width);
			}
			else {
				this.controldiv.style.top = nonzero(this.second.style.top);
				this.controldiv.style.bottom = nonzero(this.second.style.height);
				this.controldiv.style.left = '50%';
				this.controldiv.style.right = '';
			}
		}
		else {
			this.controldiv.style.display = 'none';
		}
	};
	self.destroy = function() {
		this.first.destroy();
		this.second.destroy();
		if (top.active_split === this)
			top.active_split = null;
	};
	self.serialize = function() {
		return '{' +
			(this.dominant_first ? 'D' : 'd') +
			(this.horizontal ? 'h' : 'v') +
			this.value + (this.em ? 'm' : '%') +
			this.first.serialize() +
			this.second.serialize() +
			'}';
	};
	self.copy = function() {
		var ret = new Split(this.top, this.configuring);
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

function Tabs(top, configuring) {
	var self = Create('div', 'Tabs');
	self.name = 'Tabs';
	self.top = top;
	self.configuring = configuring ? true : false; // Treat undefined as false.
	self.bar = self.AddElement('div');
	self.bar.style.position = 'absolute';
	self.bar.style.top = '0em';
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
		var tab = this.Add(new Bin(this.top, this.configuring));
		tab.style.position = 'absolute';
		tab.style.width = '100%';
		tab.style.bottom = '0em';
		tab.style.top = '2em';
		tab.style.overflow = 'auto';
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
				tab.style.top = '0em';
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
		while (this.selected >= 0 && this.tabs[this.selected].hidden && !(this.configuring && this.top.show_hidden))
			this.selected -= 1;
		if (this.selected < 0) {
			this.selected = this.tabs.length - 1;
			while (this.selected >= 0 && this.tabs[this.selected].hidden && !(this.configuring && this.top.show_hidden))
				this.selected -= 1;
			if (this.selected < 0) {
				this.hide(true);
				return;
			}
		}
		var numtabs = 0;
		for (var t = 0; t < this.tabs.length; ++t) {
			if (this.tabs[t].hidden) {
				this.tabs[t].titlebox.style.display = 'none';
				this.tabs[t].style.display = 'none';
				continue;
			}
			this.tabs[t].titlebox.style.display = 'inline';
			this.tabs[t].style.display = 'block';
			numtabs += 1;
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
		this.hide(numtabs == 0);
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
		var ret = new Tabs(this.top, this.configuring);
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

var _UI_id = 0;
function UI_setup(bin_parent, content, data) {
	var bin = bin_parent.Add(new Bin(null, false));
	bin.top = bin;
	bin.data = data;
	bin.object_id = _UI_id;
	_UI_id += 1;
	bin.active_split = null;
	bin.splitcontrol = bin_parent.AddElement('div', 'SplitControl'); // {
	bin.splitcontrol.style.zIndex = 10;
	bin.splitcontrol.style.display = 'none';
	bin.splitcontrol.style.position = 'absolute';
	bin.splitcontrol.style.top = '0em';
	bin.splitcontrol.style.left = '0em';
	bin.splitcontrol.control_x = 0;
	bin.splitcontrol.control_y = 0;
	bin.splitcontrol.style.background = 'yellow';
	bin.splitcontrol.style.border = '2px solid black';
	var d = bin.splitcontrol.AddElement('div').AddText('Split Controls');
	d.style.textAlign = 'center';
	d.style.background = '#cc0';
	d = bin.splitcontrol.AddElement('div');
	d.style.textAlign = 'center';
	bin.splitcontrol.x = NaN;
	bin.splitcontrol.y = NaN;
	d.AddEvent('mousedown', function(event) {
		bin.splitcontrol.x = event.pageX;
		bin.splitcontrol.y = event.pageY;
		bin.splitcontrol.pos = [bin.splitcontrol.control_x, bin.splitcontrol.control_y];
		event.preventDefault();
	});
	bin.AddEvent('mouseup', function(event) {
		if (isNaN(bin.splitcontrol.x))
			return;
		bin.splitcontrol.x = NaN;
		bin.splitcontrol.pos = [bin.splitcontrol.style.left, bin.splitcontrol.style.top];
		event.preventDefault();
	});
	var move_event = function(event) {
		if (isNaN(bin.splitcontrol.x) || !(event.buttons & 1)) {
			bin.splitcontrol.x = NaN;
			return;
		}
		bin.splitcontrol.control_x = bin.splitcontrol.pos[0] + event.pageX - bin.splitcontrol.x;
		bin.splitcontrol.control_y = bin.splitcontrol.pos[1] + event.pageY - bin.splitcontrol.y;
		bin.splitcontrol.style.left = bin.splitcontrol.control_x + 'px';
		bin.splitcontrol.style.top = bin.splitcontrol.control_y + 'px';
		event.preventDefault();
	};
	bin.AddEvent('mousemove', move_event);
	d.AddEvent('mousemove', move_event);
	bin.radio_v_first = d.AddElement('input');
	bin.radio_v_first.type = 'radio';
	bin.radio_v_first.name = 'SplitControl' + bin.object_id;
	bin.radio_v_first.AddEvent('click', function() {
		if (bin.active_split === null)
			return;
		bin.active_split.horizontal = false;
		bin.active_split.dominant_first = true;
		bin.active_split.update();
	});
	bin.radio_h_first = bin.splitcontrol.AddElement('input');
	bin.radio_h_first.type = 'radio';
	bin.radio_h_first.style.float = 'left';
	bin.radio_h_first.name = 'SplitControl' + bin.object_id;
	bin.radio_h_first.AddEvent('click', function() {
		if (bin.active_split === null)
			return;
		bin.active_split.horizontal = true;
		bin.active_split.dominant_first = true;
		bin.active_split.update();
	});
	bin.radio_h_second = bin.splitcontrol.AddElement('input');
	bin.radio_h_second.type = 'radio';
	bin.radio_h_second.style.float = 'right';
	bin.radio_h_second.name = 'SplitControl' + bin.object_id;
	bin.radio_h_second.AddEvent('click', function() {
		if (bin.active_split === null)
			return;
		bin.active_split.horizontal = true;
		bin.active_split.dominant_first = false;
		bin.active_split.update();
	});
	d = bin.splitcontrol.AddElement('div');
	d.style.textAlign = 'center';
	d.style.clear = 'both';
	bin.radio_v_second = d.AddElement('input');
	bin.radio_v_second.type = 'radio';
	bin.radio_v_second.name = 'SplitControl' + bin.object_id;
	bin.radio_v_second.AddEvent('click', function() {
		if (bin.active_split === null)
			return;
		bin.active_split.horizontal = false;
		bin.active_split.dominant_first = false;
		bin.active_split.update();
	});
	d = bin.splitcontrol.AddElement('div');
	bin.split_input = d.AddElement('input');
	bin.split_input.style.width = '5em';
	bin.split_input.type = 'number';
	bin.split_input.min = 0;
	bin.split_input.step = 0.1;
	bin.split_input.AddEvent('change', function() {
		if (bin.active_split === null)
			return;
		bin.active_split.value = Number(this.value);
		bin.active_split.update();
	});
	bin.split_type_select = d.AddElement('select');
	bin.split_type_select.AddElement('option').AddText('em').value = 'em';
	bin.split_type_select.AddElement('option').AddText('%').value = '%';
	bin.split_type_select.AddEvent('change', function() {
		if (bin.active_split === null)
			return;
		bin.active_split.em = this.value != '%';
		bin.active_split.update();
	});
	var button = bin.splitcontrol.AddElement('button').AddText('Remove Split').AddEvent('click', function() {
		if (bin.active_split === null)
			return;
		var active = bin.active_split;
		var content = active.first.content;
		active.first.content = {};
		active.first.removeChild(content);
		active.destroy();
		active.parent_bin.set_content(content);
		active.parent_bin.update();
	});
	button.type = 'button';
	button.style.width = '100%';
	button = bin.splitcontrol.AddElement('button').AddText('Add Split').AddEvent('click', function() {
		if (bin.active_split === null)
			return;
		bin.active_split.parent_bin.add_split();
	});
	button.type = 'button';
	button.style.width = '100%';
	d = bin.splitcontrol.AddElement('div');
	var label = d.AddElement('label');
	var box = label.AddElement('input');
	box.type = 'checkbox';
	bin.show_hidden = box.checked;
	box.AddEvent('change', function() {
		bin.show_hidden = box.checked;
		bin.update();
	});
	label.AddText('Show Hidden');
	// }
	if (typeof content == 'string')
		content = ui_build(content, bin);
	bin.set_content(content);
	return bin;
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

#!/usr/bin/python3

import joystick as js
import time
import os
import sys
import ctypes
import fcntl
import select
import struct
import fhs
import websockets

class Globals: pass

g = Globals()

class MODIFIER: pass	#Make this a unique object.

def ioctl(op, t):
	value = t()
	ret = fcntl.ioctl(g.fd, op, value)
	if ret < 0:
		sys.stderr.write('ioctl failed')
		sys.exit(1)
	return value.value

def main(config = {}, buttons = {}, axes = {}, tick = None):
	if not hasattr(g, 'config'):
		configdata = {'tick_time': .05, 'js': '/dev/input/js0', 'printer': '8000', 'epsilon': 100, 'small': 3277}
		configdata.update(config)
		g.config = fhs.init(configdata)
		g.printer = websockets.RPC(g.config['printer'], tls = False)
		g.fd = os.open(g.config['js'], os.O_RDWR)
		if g.fd < 0:
			sys.stderr.write('Cannot open joystick file')
			sys.exit(1)

		version = ioctl(js.gversion, ctypes.c_uint32)
		if version != js.version:
			sys.stderr.write('version mismatch (%x != %x)' % (version, js.version))

		g.num_axes = ioctl(js.gaxes, ctypes.c_uint8)
		g.num_buttons = ioctl(js.gbuttons, ctypes.c_uint8)

	axis_state = [0.] * g.num_axes
	axis_zero = [0.] * g.num_axes
	button_state = [None] * g.num_buttons

	#print('axes: %d buttons: %d' % (g.num_axes, g.num_buttons))

	controls = {2: [0, 0, -1], 3: [1, 0, 0], 4: [0, -1, 0], 5: [0, 0, 1]}
	controls.update(axes)
	scale = [50., 50., 10.]
	mem = {}

	running = [True]
	def home():
		g.printer.home()
		return True

	def start():
		return False

	def gomem(i):
		def impl():
			if i not in mem:
				print('position %d not stored' % i)
				return
			g.printer.line_cb([mem[i]])
			return True
		return impl

	def storemem(i):
		def impl():
			mem[i] = g.printer.get_axis_pos(0)
			return True
		return impl

	# A, B, X, Y, LT, RT, BACK, START, SELECT, L, R
	button_action = {6: home, 7: start, 8: MODIFIER}
	button_action.update({i: gomem(i) for i in range(4) })
	button_action.update({i + 11: storemem(i) for i in range(4) })
	button_action.update(buttons)

	modifiers = [x for x in button_action if button_action[x] is MODIFIER]
	modifiers.sort()

	def handle_axis(num, value, init):
		if (init or abs(axis_state[num] - value) < g.config['epsilon']) and abs(value) < g.config['small']:
			was_zero = axis_state[num] == axis_zero[num]
			axis_zero[num] = value
		else:
			was_zero = False
		axis_state[num] = value
		value -= axis_zero[num]
		if value == 0 and was_zero:
			return
		#print('axis %d moved to %d (- %d)' % (num, value, axis_zero[num]))

	def handle_button(num, value, init):
		button_state[num] = value
		if num in button_action and button_action[num] is MODIFIER:
			return
		for i, m in enumerate(modifiers):
			if button_state[m]:
				num += g.num_buttons << i
		if num not in button_action:
			return
		if value == 0:
			if any(button_state) or None not in button_action:
				return
			# Handle release of all buttons as event None.
			num = None
		running[0] &= button_action[num]()

	def my_tick():
		if tick:
			ret = tick([a - z for a, z in zip(axis_state, axis_zero)])
			if ret is None:
				return True
			if ret is False:
				return False
		move = [0., 0., 0.]
		for a in controls:
			for c, v in enumerate(controls[a]):
				move[c] += v * (axis_state[a] - axis_zero[a]) * scale[c] * g.config['tick_time'] / (1 << 15)
				#print('move %d %f %d' % (c, move[c], axis_state[a] - axis_zero[a]))
		if any(move):
			#print(repr(move))
			g.printer.line_cb([move], rel = True)
		return True

	def handle_js():
		t, value, type, number = js.event.unpack(os.read(g.fd, js.event.size))
		init = bool(type & js.event_init)
		if type & js.event_axis:
			handle_axis(number, value, init)
		if type & js.event_button:
			handle_button(number, value, init)

	start = time.time()
	while running[0]:
		dt = g.config['tick_time'] - (time.time() - start)
		if dt <= 0:
			running[0] &= my_tick()
			dt += g.config['tick_time']
			start += g.config['tick_time']
		while dt <= 0:
			dt += g.config['tick_time']
			start += g.config['tick_time']
		ret = select.select([g.fd], [], [g.fd], dt)
		if all(len(f) == 0 for f in ret):
			continue
		handle_js()
		while any(len(x) for x in select.select([g.fd], [], [g.fd], 0)):
			handle_js()
	return mem

if __name__ == '__main__':
	while True:
		main()

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
import websocketd

class MODIFIER: pass	#Make this a unique object.

cfg = None
printer = None
fd = None
num_axes = None
num_buttons = None
dead = None
axis_state = []
xy_state = [0., 0.]
button_state = []
dmax = 1
dhalf = dmax / 2
vmax = 60
vhalf = 5
c4 = (vhalf - vmax * dhalf ** 2 / dmax ** 2) / (dhalf ** 4 - dmax ** 2 * dhalf ** 2)
c2 = vmax / dmax ** 2 - c4 * dmax ** 2

def ioctl(op, t):
	value = t()
	ret = fcntl.ioctl(fd, op, value)
	if ret < 0:
		sys.stderr.write('ioctl failed')
		sys.exit(1)
	return value.value

def init(config = {}):
	global cfg, printer, fd, num_axes, num_buttons, dead
	if cfg is not None:
		return
	configdata = {'tick_time': .05, 'js': '/dev/input/js0', 'printer': '8000', 'dead': .1}
	configdata.update(config)
	cfg = fhs.init(configdata)
	dead = float(cfg['dead'])
	printer = websocketd.RPC(cfg['printer'], tls = False)
	fd = os.open(cfg['js'], os.O_RDWR)
	if fd < 0:
		sys.stderr.write('Cannot open joystick file')
		sys.exit(1)

	version = ioctl(js.gversion, ctypes.c_uint32)
	if version != js.version:
		sys.stderr.write('version mismatch (%x != %x)' % (version, js.version))

	num_axes = ioctl(js.gaxes, ctypes.c_uint8)
	num_buttons = ioctl(js.gbuttons, ctypes.c_uint8)

	axis_state[:] = [0.] * num_axes
	button_state[:] = [None] * num_buttons

def main(config = {}, buttons = {}, axes = {}, tick = None):
	init(config)

	#print('axes: %d buttons: %d' % (num_axes, num_buttons))

	controls = {2: [0, 0, 1, 0, 0], 0: [1, 0, 0, 0, 0], 1: [0, -1, 0, 0, 0], 5: [0, 0, -1, 0, 0], 3: [0, 0, 0, 1, 0], 4: [0, 0, 0, 0, -1]}
	controls.update(axes)
	mem = {}

	running = [True]
	def home():
		printer.home()
		return True

	def start():
		return False

	def gomem(i):
		def impl():
			if i not in mem:
				print('position %d not stored' % i)
				return True
			printer.line_cb([mem[i]])
			return True
		return impl

	def storemem(i):
		def impl():
			mem[i] = printer.get_axis_pos(0)
			return True
		return impl

	def abort():
		printer.confirm(None, False)
		printer.pause()
		return True

	def resume():
		printer.confirm(None)
		printer.pause(False)
		return True

	# A, B, X, Y, LT, RT, BACK, START, SELECT, L, R
	button_action = {4: abort, 5: resume, 6: home, 7: start, 8: MODIFIER}
	button_action.update({i: gomem(i) for i in range(4) })
	button_action.update({i + 11: storemem(i) for i in range(4) })
	button_action.update(buttons)

	modifiers = [x for x in button_action if button_action[x] is MODIFIER]
	modifiers.sort()

	def handle_axis(num, value, init):
		axis_state[num] = value

	def handle_button(num, value, init):
		button_state[num] = value
		if num in button_action and button_action[num] is MODIFIER:
			#print('ignoring mod %d %s' % (num, repr(button_state)))
			return
		for i, m in enumerate(modifiers):
			if button_state[m]:
				num += num_buttons << i
		if num not in button_action or button_action[num] is None:
			#print('ignoring no action %d %s' % (num, repr(button_state)))
			return
		if value == 0:
			if any(x for i, x in enumerate(button_state) if i not in modifiers) or None not in button_action:
				#print('ignoring %s' % repr(button_state))
				return
			# Handle release of all buttons as event None.
			num = None
		running[0] &= button_action[num]()

	def my_tick():
		move = [0., 0., 0., 0., 0.]
		n = [0, 0, 0, 0, 0]
		for a in controls:
			if a >= len(axis_state):
				print('invalid control %d >= %d' % (a, len(axes)))
				continue
			for c, v in enumerate(controls[a]):
				if v != 0:
					n[c] += 1
					move[c] += v * axis_state[a] / (1 << 15)
		for i, nn in enumerate(n):
			move[i] /= nn
		for stick in (0, 3):
			d = (move[stick + 0] ** 2 + move[stick + 1] ** 2) ** .5
			if d <= dead:
				move[stick + 0] = 0.
				move[stick + 1] = 0.
			else:
				new_d = d - dead
				if d >= dmax:
					factor = vmax / d
				else:
					factor = c4 * new_d ** 4 + c2 * new_d ** 2
				move[stick + 0] *= factor * cfg['tick_time']
				move[stick + 1] *= factor * cfg['tick_time']
		move[2] *= (c4 * move[2] ** 4 + c2 * move[2] ** 2) * cfg['tick_time']
		if tick:
			ret = tick(axes, move)
			if ret is None:
				return True
			if ret is False:
				return False
		move[0] += move[3]
		move[1] += move[4]
		if any(move[:3]):
			printer.line_cb([move[:3]], relative = True)
		if any(move[3:]):
			target = {}
			g = printer.get_globals()
			if move[3]:
				target['targetx'] = move[3] + g['targetx']
			if move[4]:
				target['targety'] = move[4] + g['targety']
			printer.set_globals(**target)
		return True

	def handle_js():
		t, value, type, number = js.event.unpack(os.read(fd, js.event.size))
		init = bool(type & js.event_init)
		if type & js.event_axis:
			handle_axis(number, value, init)
		if type & js.event_button:
			handle_button(number, value, init)

	start = time.time()
	while running[0]:
		dt = cfg['tick_time'] - (time.time() - start)
		if dt <= 0:
			running[0] &= my_tick()
			dt += cfg['tick_time']
			start += cfg['tick_time']
		while dt <= 0:
			dt += cfg['tick_time']
			start += cfg['tick_time']
		ret = select.select([fd], [], [fd], dt)
		if all(len(f) == 0 for f in ret):
			continue
		handle_js()
		while any(len(x) for x in select.select([fd], [], [fd], 0)):
			handle_js()
	return mem

if __name__ == '__main__':
	while True:
		main()

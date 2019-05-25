#!/usr/bin/python3

from PIL import Image
import fhs
import binascii

config = fhs.init({'src': None, 'dpmm': 10, 'speed': 5, 'margin': 10})
im = Image.open(config['src']).convert(mode = '1', dither = Image.FLOYDSTEINBERG)
data = im.getdata()

h, w = im.size
print(';size: %d,%d' % (w, h))
print('G1 F%f' % (config['speed'] * 60))
for y in range(h):
	line = b''
	# The full range is w/16, rounded up. That is (w+15)//16. Add one bit to make sure the laser is turned off at the end of the line. That makes it (w+16)//16 == w//16+1.
	for x in range(w // 16 + 1):
		for x2 in (0, 8):
			d = 0
			for bit in range(8):
				b = data[y * w + x  + x2 + bit] and 1 if x + x2 + bit < w else 0
				d |= b << bit
			line += bytes((d,))
	code = binascii.b2a_base64(line).strip().decode('utf-8', 'replace')
	print('G1 X%f Y%f' % (-config['margin'], y / config['dpmm']))
	print('G1 X0')
	print('G1 X%f ;PWM:%s' % (w / config['dpmm'], code))
	print('G1 X%f' % (w / config['dpmm'] + config['margin']))
	print('G1 X0 Y0')

from distutils.core import setup, Extension

# Supported values for target: avr, multicore
# Supported variants for multicore: pine64, orangepizero

target = 'avr'
variant = ''

if variant != '':
	macros = [(variant.upper(), None)]
else:
	macros = []

macros.append(('MODULE', None))
macros.append(('CDRIVER', '"franklin-cdriver"'))
macros.append(('SERIAL', None))

cdrivermodule = Extension('cdriver',
		sources = [
			'module.cpp',
			'parse.cpp',
		],
		depends = [
			'setup.py',
			'module.h',
		],
		define_macros = macros,
		language = 'c++',
	)

setup(name = 'cdriver',
		version = '0.1',
		description = 'Internal module for controlling a CNC machine with Franklin',
		ext_modules = [cdrivermodule],
	)

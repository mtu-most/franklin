#!/usr/bin/python3

import websocketd

state_name = { False: 'Paused', True: 'Running', None: 'Idle' }

class monitor:
	def __init__(self, obj):
		'''This function must exist, because the constructor is called
		with an argument.'''
		# Initialize the print state.
		self.state = None
	def globals_update(self, machine, values):
		'''Handle the globals_update event.
		Don't care about all the arguments, just check the last one.'''
		print_state = values[-1]
		if print_state == self.state:
			return
		print('State changed from {} to {}'.format(state_name[self.state], state_name[print_state]))
		self.state = print_state
	def __getattr__(self, attr):
		'''Anything else is requested.
		Return a function that can print the name and arguments when it
		is called.'''
		def call(*args, **kwargs):
			## Uncomment the next line to see all the functions
			## that are called.  To handle any of them, define it
			## like globals_update is defined above.
			#print('call', attr, 'args:', args, 'kwargs:', kwargs)
			pass
		return call

# Open the connection to Franklin.
p = websocketd.RPC('localhost:8000', monitor, tls = False)

# Tell Franklin to inform us of updates.
p.set_monitor(True)

# Start the main loop (in the foreground), waiting for events.
websocketd.fgloop()

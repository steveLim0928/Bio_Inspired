import time
import numpy as np
#from utils import plot_line
from gpiozero import RotaryEncoder

# FIT0521
ppr = 341
tstop = 20 # Execute duration
tsample = 0.02 # Sampling period
tdisp = 0.5 # Display period

encoder = RotaryEncoder(19, 26, max_steps = 0)

curr_angle = 0
t_prev = 0
t_curr = 0
tstart = time.perf_counter()

print('Run for', tstop, 'seconds')
print('Start')
while t_curr <= tstop:
	time.sleep(tsample)
	t_curr = time.perf_counter() - tstart
	curr_angle = 360 / ppr * encoder.steps
	#print('0.0f', encoder.steps)
	
	if (np.floor(t_curr/tdisp) - np.floor(t_prev/tdisp)) == 1:
		print('Angle = {:0.0f} deg'.format(curr_angle))
		
	t_prev = t_curr
	
print('End')
encoder.close()

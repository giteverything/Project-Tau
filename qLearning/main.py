import time, sys
import world
import math
from globalvar import *

print ('Program started')

world = world.World()
world.start()

try:
	while True:
		world.step()
		time.sleep(SLEEPTIME)
except KeyboardInterrupt:    
	world.end()

print ('Program ended')

import world
import time
from globalvar import *

print "Program started"

world = world.World()
world.start()

try:
	while True:
		world.step()
		time.sleep(SLEEPTIME)
		if world.episode == 1000:
			world.endTraining()

except KeyboardInterrupt:    
	world.end()

print "Program ended"

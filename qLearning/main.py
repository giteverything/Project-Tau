import vrep
import time, sys
import cubebot, cubebotWorld
import math
from math import pi as PI 
from globalvar import *

print ('Program started')

world = cubebotWorld.CubebotWorld()
world.start()
try:
	while True:
		world.step()
		time.sleep(SLEEPTIME)
except KeyboardInterrupt:
	print "---------------------------------"
	print "Stopping Program..."
	print "Writing QValue to {} ...".format(RESULTFILE)
	world.saveLearningResult(RESULTFILE)
	print "---------------------------------"        
	world.end()

print ('Program ended')

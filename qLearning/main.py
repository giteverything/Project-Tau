import vrep
import time, sys
import cubebot, cubebotWorld
import math
from math import pi as PI 
from globalvar import *

print ('Program started')

vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID!=-1:
	print ('Connected to remote API server')

	vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)

	# get object handles
	_,body=vrep.simxGetObjectHandle(clientID,'body',vrep.simx_opmode_oneshot_wait)
	upperJoints, lowerJoints = [], []
	for i in range(0, 4):
		_, upperJoint = vrep.simxGetObjectHandle(clientID, "upperj"+str(i), vrep.simx_opmode_oneshot_wait)
		_, lowerJoint = vrep.simxGetObjectHandle(clientID, "lowerj"+str(i), vrep.simx_opmode_oneshot_wait)
		upperJoints.append(upperJoint)
		lowerJoints.append(lowerJoint)

	# initialize robot and abstract environment
	robot = cubebot.Cubebot(clientID, body, upperJoints, lowerJoints)
	world = cubebotWorld.CubebotWorld(robot)


	world.start()

	try:
		while (vrep.simxGetConnectionId(clientID)!=-1):
			# robot.moveLowerJoint(1, 0)
			# robot.moveLowerJoint(0, PI/4)
			# robot.moveUpperJoint(1, PI/3)
			# robot.moveUpperJoint(0, PI/3)
			# robot.moveLowerJoint(0, 0)
			# robot.moveLowerJoint(1, PI/4)
			# robot.moveUpperJoint(0, 0)
			# robot.moveUpperJoint(1, 0)
			world.step()
			time.sleep(SLEEPTIME)
	except KeyboardInterrupt:
		print "---------------------------------"
		print "Stopping Program..."
		print "Writing QValue to {} ...".format(RESULTFILE)
		world.saveLearningResult(RESULTFILE)
		print "---------------------------------"        
		world.end()
		vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
		vrep.simxFinish(clientID)
else:
	print ('Failed connecting to remote API server')
print ('Program ended')

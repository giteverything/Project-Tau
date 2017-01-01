# crawler.py
# ----------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


#!/usr/bin/python
import math
from math import pi as PI
import time
import environment
import random
import vrep
import simCubebot
from globalvar import *

class CubebotEnvironment(environment.Environment):

	def __init__(self):

		self.cubebot = simCubebot.simCubebot()

		# The state is of the form (upperJointAngleBucket, lowerJointAngleBucket)
		# where the angles are bucket numbers, not actual degree measurements        
		self.state = None

		# number of possible states for each joint
		self.nUpperJointStates = 2
		self.nLowerJointStates = 2

		minUpperAngle, maxUpperAngle = self.cubebot.getMinandMaxUpperAngles()
		minLowerAngle, maxLowerAngle = self.cubebot.getMinandMaxLowerAngles()

		upperJointIncre = (maxUpperAngle - minUpperAngle) / (self.nUpperJointStates - 1)
		lowerJointIncre = (maxLowerAngle - minLowerAngle) / (self.nLowerJointStates - 1)

		self.upperJointBuckets = [ minUpperAngle + (upperJointIncre * i) for i in range(self.nUpperJointStates)]
		self.lowerJointBuckets = [ minLowerAngle + (lowerJointIncre * i) for i in range(self.nLowerJointStates)]

		self.reset()


	def getCurrentState(self):
		return self.state

	def getPossibleActions(self, state):
		"""
		  state = (0-U, 0-L, 1-U, 1-L)
		  action = (0, 'U', +1)
		"""		
		actions = list()
		# if any of the legs are lifted:
		for i in range(1, len(state) - 1, 2):
			if state[i] != 0:
				actions.append((i / 2, 'L', -1))
				for j in range(0, len(state) - 1, 2):
					if state[j] > 0: actions.append((j / 2, 'U', -1))
					if state[j] < self.nUpperJointStates - 1: actions.append((j / 2, 'U', 1))
				return actions
		
		# if no leg is lifted, lift any leg:
		for i in range(1, len(state) - 1, 2):
			actions.append((i / 2, 'L', 1))

		return actions

	def doAction(self, action):
		"""
		  state = (0-U, 0-L, 1-U, 1-L, Ori)
		  action = (0, 'U', +1)

		  Returns:
			nextState, reward
		"""

		oldX, oldY = self.cubebot.getPosition()
		_,_,oldGamma = self.cubebot.getOrientation()

		nextState = list(self.state)
		jointNo = action[0]
		jointBuckets, moveJointFunc, jointBucket = None, None, None
		if action[1] == 'U':
			jointBuckets = self.upperJointBuckets
			moveJointFunc = self.cubebot.moveSymUpperJoint
			jointBucket = self.state[2 * jointNo]
			nextState[2 * jointNo] += action[2]
		else:
			jointBuckets = self.lowerJointBuckets
			moveJointFunc = self.cubebot.moveSymLowerJoint
			jointBucket = self.state[2 * jointNo + 1]
			nextState[2 * jointNo + 1] += action[2]

		newAngle = jointBuckets[jointBucket + action[2]]	
		moveJointFunc(jointNo, newAngle)

		newX, newY = self.cubebot.getPosition()
		_,_,gamma = self.cubebot.getOrientation()

		alpha = math.atan2(newY-oldY, newX-oldX)
		disp = math.sqrt((newX-oldX)**2 + (newY-oldY)**2)
		turnPenalty = abs(gamma)
		reward = newX - oldX + disp * math.cos(alpha - oldGamma)


		if abs(gamma) <= 0.785:
			nextState[-1] = 0
			reward += self.cubebot.get10StepXVelocity()
		elif  0.785 < gamma and gamma < 2.356:
			nextState[-1] = 1
			reward = -reward if reward > 0 else reward
		elif -0.785 > gamma and gamma > -2.356 and reward > 0:
			nextState[-1] = 3
			reward = -reward if reward > 0 else reward
		else:
			nextState[-1] = 2
			reward = -reward if reward > 0 else reward


		nextState = tuple(nextState)
		self.state = nextState

		return nextState, reward

	def reset(self):
		"""
		 Resets the Environment to the initial state
		"""
		self.state = (0,0,0,0,0)
		self.cubebot.endSim()
		self.cubebot = simCubebot.simCubebot()

class Cubebot:

	def reinitializeRobot(self, body, upperJoints, lowerJoints):
		self.body = body
		self.upperJoints = upperJoints[:]
		self.lowerJoints = lowerJoints[:]
		self.upperJointAngles = [0.0, 0.0, 0.0, 0.0]
		self.lowerJointAngles = [0.0, 0.0, 0.0, 0.0]		
		self.pos = [[0,0], self.getRobotPosition()]

	def robotMove(self, objectHandle, newAngle):
		_,oldAngle = vrep.simxGetJointPosition(self.clientID, objectHandle, vrep.simx_opmode_oneshot_wait)
		for i in range(STEP):
			vrep.simxSetJointTargetPosition(self.clientID, objectHandle, oldAngle + i * (newAngle - oldAngle)/STEP, vrep.simx_opmode_oneshot)
			time.sleep(SLEEPTIME)

	def robotMoveList(self, handleList, angleList):
		if len(handleList) != len(angleList):
			print "cubebot - robotMoveList: length not equal"
		oldAngleList = []
		for i in range(len(handleList)):
			oldAngleList.append(vrep.simxGetJointPosition(self.clientID, handleList[i], vrep.simx_opmode_oneshot_wait)[1])

		for s in range(STEP):
			vrep.simxPauseCommunication(clientID,True)
			for i in range(len(handleList)):
				vrep.simxSetJointTargetPosition(self.clientID, handleList[i], oldAngleList[i]
				 + s * (angleList[i] - oldAngleList[i])/STEP, vrep.simx_opmode_oneshot)
			vrep.simxPauseCommunication(clientID,False)
			time.sleep(SLEEPTIME)

	def getUpperJointAngles(self):
		return self.upperJointAngles[0], self.upperJointAngles[1], self.upperJointAngles[2], self.upperJointAngles[3]

	def getLowerJointAngles(self):
		return self.lowerJointAngles[0], self.lowerJointAngles[1], self.lowerJointAngles[2], self.lowerJointAngles[3]

	def getUpperJointHandles(self):
		return self.upperJoints

	def getLowerJointHandles(self):
		return self.lowerJoints

	def getRobotPosition(self):
		"""
			returns the (x,y) coordinates of the center point of the robot
		"""
		_,robotPos = vrep.simxGetObjectPosition(self.clientID, self.body, -1, vrep.simx_opmode_oneshot_wait)

		if len(self.pos) == 100:
			self.pos.pop(0)
			self.speed.append(self.pos[-1][0] - self.pos[0][0])
		self.pos.append([ robotPos[0], robotPos[1] ])
		return robotPos[0], robotPos[1]

	def getRobotOrientation(self):
		_, robotOri = vrep.simxGetObjectOrientation(self.clientID, self.dummy, -1, vrep.simx_opmode_oneshot_wait)
		return robotOri

	def get10StepVelocity(self):
		if len(self.pos) == 10:
			return (self.pos[9][0] - self.pos[0][0]) / 10.0
		return 0

	def moveUpperJoint(self, jointNo, angle):
		"""
			move the upper joint jointNo (range from 0 - 3) to angle
		"""
		if jointNo < 2:

			jointList = [self.upperJoints[jointNo], self.upperJoints[jointNo + 2]]
			angleList = [angle, self.maxUpperAngle - angle]
			self.robotMoveList(jointList, angleList)
			self.upperJointAngles[jointNo] = angle
			self.upperJointAngles[jointNo + 2] = angle

		else:

			self.robotMove(self.upperJoints[jointNo], angle)
			self.upperJointAngles[jointNo] = angle


	def moveLowerJoint(self, jointNo, angle):
		"""
			move the lower joint jointNo (range from 0 - 3) to angle
		"""
		if jointNo < 2:

			jointList = [self.lowerJoints[jointNo], self.lowerJoints[jointNo + 2]]
			angleList = [angle, angle]
			self.robotMoveList(jointList, angleList)
			self.lowerJointAngles[jointNo] = angle
			self.lowerJointAngles[jointNo + 2] = angle

		else:

			self.robotMove(self.lowerJoints[jointNo], angle)
			self.lowerJointAngles[jointNo] = angle			

	# def moveUpperJoint(self, jointNo, angle):
	# 	"""
	# 		move the upper joint jointNo (range from 0 - 3) to angle
	# 	"""
	# 	if jointNo > 3 or jointNo < 0:
	# 		raise "cubebot: invalid jointNo", jointNo

	# 	self.robotMove(self.upperJoints[jointNo], angle)
	# 	self.upperJointAngles[jointNo] = angle

	# def moveLowerJoint(self, jointNo, angle):
	# 	"""
	# 		move the lower joint jointNo (range from 0 - 3) to angle
	# 	"""
	# 	if jointNo > 3 or jointNo < 0:
	# 		raise "cubebot: invalid jointNo", jointNo

	# 	self.robotMove(self.lowerJoints[jointNo], angle)
	# 	self.lowerJointAngles[jointNo] = angle

	def getMinandMaxUpperAngles(self):
		"""
			get the lower- and upper- bound
			for the upper joing angles, return (min,max) pair
		"""
		return self.minUpperAngle, self.maxUpperAngle


	def getMinandMaxLowerAngles(self):
		"""
			get the lower- and upper- bound
			for the lower joing angles, return (min,max) pair
		"""
		return self.minLowerAngle, self.maxLowerAngle

	def __init__(self, clientID, body, upperJoints, lowerJoints):

		self.clientID = clientID
		self.body = body
		self.upperJoints = upperJoints
		self.lowerJoints = lowerJoints

		self.upperJointAngles = [0.0, 0.0, 0.0, 0.0]
		self.lowerJointAngles = [0.0, 0.0, 0.0, 0.0]

		self.minUpperAngle, self.maxUpperAngle = 0, PI/3
		self.minLowerAngle, self.maxLowerAngle = 0, PI/4

		self.positions = [0,0]
		self.pos = [[0,0]]

		_, self.dummy = vrep.simxGetObjectHandle(self.clientID, "Dummy", vrep.simx_opmode_oneshot_wait)

		self.speed = []
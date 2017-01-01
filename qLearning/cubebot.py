import simCubebot
import math

class Cubebot():

	def __init__(self):
		self.simCubebot = simCubebot.SimCubebot()

		self.state = (0,0,0,0,0)

		self.nUpperJointStates = 2
		self.nLowerJointStates = 2

		minUpperAngle, maxUpperAngle = self.simCubebot.getMinandMaxUpperAngles()
		minLowerAngle, maxLowerAngle = self.simCubebot.getMinandMaxLowerAngles()

		upperJointIncre = (maxUpperAngle - minUpperAngle) / (self.nUpperJointStates - 1)
		lowerJointIncre = (maxLowerAngle - minLowerAngle) / (self.nLowerJointStates - 1)

		self.upperJointBuckets = [ minUpperAngle + (upperJointIncre * i) for i in range(self.nUpperJointStates)]
		self.lowerJointBuckets = [ minLowerAngle + (lowerJointIncre * i) for i in range(self.nLowerJointStates)]

	def getCurrentState(self):
		return self.state

	def getActions(self, state):
		"""
			passed to qLearner module
			life a leg if no leg is lifted.
			do any action, if some legs are lifted.
			action = (0/1, U/L, 0/1)
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

		oldPos = self.simCubebot.getPosition()
		oldOri = self.simCubebot.getOrientation()

		nextState = list(self.state)
		jointNo = action[0]
		jointBuckets, moveJointFunc, jointBucket = None, None, None
		if action[1] == 'U':
			jointBuckets = self.upperJointBuckets
			moveJointFunc = self.simCubebot.moveSymUpperJoint
			jointBucket = self.state[2 * jointNo]
			nextState[2 * jointNo] += action[2]
		else:
			jointBuckets = self.lowerJointBuckets
			moveJointFunc = self.simCubebot.moveSymLowerJoint
			jointBucket = self.state[2 * jointNo + 1]
			nextState[2 * jointNo + 1] += action[2]

		newAngle = jointBuckets[jointBucket + action[2]]	
		moveJointFunc(jointNo, newAngle)

		newPos = self.simCubebot.getPosition()
		newOri = self.simCubebot.getOrientation()

		nextState = tuple(nextState)
		reward = self.calReward(oldPos, oldOri, newPos, newOri)
		self.state = nextState

		return nextState, reward


	def calReward(self, oldPos, oldOri, newPos, newOri):
		"""
			calculate reward of this step
		"""

		oldX, oldY = oldPos
		newX, newY = newPos
		_, _, oldGamma = oldOri
		_, _, newGamma = newOri

		alpha = math.atan2(newY - oldY, newX - oldX)
		disp = math.sqrt((newX - oldX) ** 2 + (newY - oldY) ** 2)
		reward = newX - oldX + disp * math.cos(alpha - oldGamma)

		if abs(gamma) <= 0.785:
			nextState[-1] = 0
			reward += self.cubebot.get10StepXVelocity()
		elif 0.785 < gamma and gamma < 2.356:
			nextState[-1] = 1
			reward = -reward if reward > 0 else reward
		elif -0.785 > gamma and gamma > -2.356:
			nextState[-1] = 3
			reward = -reward if reward > 0 else reward
		else:
			nextState[-1] = 2
			reward = -reward if reward > 0 else reward

		return reward


	def reset(self):
		state = (0,0,0,0,0)
		self.simCubebot.endSim()
		self.simCubebot = simCubebot.SimCubebot()












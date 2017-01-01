import vrep
import sys, time
from math import pi as PI
from globalvar import *

class simCubebot():
	def __init__(self):
		print "init connection with V-Rep..."
		vrep.simxFinish(-1)
		clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
		if clientID == -1:
			print "connection failed. Abort."
			sys.exit()

		vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)
		_, body = vrep.simxGetObjectHandle(clientID,'body',vrep.simx_opmode_oneshot_wait)
		_, dummy = vrep.simxGetObjectHandle(clientID, "Dummy", vrep.simx_opmode_oneshot_wait)
		upperJoints, lowerJoints = [], []
		for i in range(0, 4):
			_, upperJoint = vrep.simxGetObjectHandle(clientID, "upperj"+str(i), vrep.simx_opmode_oneshot_wait)
			_, lowerJoint = vrep.simxGetObjectHandle(clientID, "lowerj"+str(i), vrep.simx_opmode_oneshot_wait)
			upperJoints.append(upperJoint)
			lowerJoints.append(lowerJoint)

		self.clientID = clientID
		self.body = body
		self.dummy = dummy
		self.upperJoints = upperJoints
		self.lowerJoints = lowerJoints

		self.upperAngles = [0.0, 0.0, 0.0, 0.0]
		self.lowerAngles = [0.0, 0.0, 0.0, 0.0]

		self.minUpperAngle, self.maxUpperAngle = 0, PI/3
		self.minLowerAngle, self.maxLowerAngle = 0, PI/4

		self.positions = []
		self.orientations = []


	def endSim(self):
		vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot)
		vrep.simxFinish(self.clientID)


	def getMinandMaxUpperAngles(self):
		return self.minUpperAngle, self.maxUpperAngle


	def getMinandMaxLowerAngles(self):
		return self.minLowerAngle, self.maxLowerAngle


	def moveUpperJoint(self, jointNo, angle):
		self.move(self.upperJoints[jointNo], angle)
		self.upperAngles[jointNo] = angle


	def moveLowerJoint(self, jointNo, angle):
		self.move(self.lowerJoints[jointNo], angle)
		self.lowerAngles[jointNo] = angle


	def moveSymUpperJoint(self, jointNo, angle):
		"""
			move two central symmetric upper joint
		"""
		jointList = [self.upperJoints[jointNo], self.upperJoints[jointNo + 2]]
		angleList = [angle, self.maxUpperAngle - angle]
		self.move(jointList, angleList)
		self.upperAngles[jointNo] = angle
		self.upperAngles[jointNo + 2] = angle


	def moveSymLowerJoint(self, jointNo, angle):
		"""
			move two central symmetric lower joint
		"""
		jointList = [self.lowerJoints[jointNo], self.lowerJoints[jointNo + 2]]
		angleList = [angle, angle]
		self.move(jointList, angleList)
		self.lowerAngles[jointNo] = angle
		self.lowerAngles[jointNo + 2] = angle


	def getOrientation(self):
		"""
			return the (alpha, beta, gamma) orientation
		"""
		if len(self.orientations) == 10:
			self.orientations.pop(0)
		_, ori = vrep.simxGetObjectOrientation(self.clientID, self.dummy, -1, vrep.simx_opmode_oneshot_wait)
		self.orientations.append(ori)
		return ori


	def getPosition(self):
		"""
			return the (x,y) coordinate 
		"""
		if len(self.positions) == 50:
			self.positions.pop(0)
		_, pos = vrep.simxGetObjectPosition(self.clientID, self.body, -1, vrep.simx_opmode_oneshot_wait)
		self.positions.append(( pos[0], pos[1] ))
		return self.positions[-1]


	def get10StepXVelocity(self):
		"""
			return the 10-step average velocity in x direction
		"""
		if len(self.positions) < 10:
			return 0
		return (self.positions[9][0] - self.positions[0][0]) / 10.0


	def get50StepVelocity(self):
		"""
			return the 50-step average velocity in x direction
		"""
		if len(self.positions) < 50:
			return 0
		return (self.positions[-1][0] - self.positions[0][0]) / 50.0


	def move(self, objHandle, angle):
		try:
			# if it is a list of handles:
			if len(objHandle) != len(angle):
				print "Error: simCubebot: len(objHandle) =", len(objHandle), "len(angle) =", len(angle)

			oldAngle = []
			for i in range(len(objHandle)):
				oldAngle.append(vrep.simxGetJointPosition(self.clientID, objHandle[i], vrep.simx_opmode_oneshot_wait)[1])

			for s in range(STEP):
				vrep.simxPauseCommunication(self.clientID, True)
				for i in range(len(objHandle)):
					vrep.simxSetJointTargetPosition(self.clientID, objHandle[i], oldAngle[i]
					 + s * (angle[i] - oldAngle[i])/STEP, vrep.simx_opmode_oneshot)
				vrep.simxPauseCommunication(self.clientID, False)
				time.sleep(SLEEPTIME)

		except TypeError:
			# if there is a single objhanle:
			_,oldAngle = vrep.simxGetJointPosition(self.clientID, objHandle, vrep.simx_opmode_oneshot_wait)
			for i in range(STEP):
				vrep.simxSetJointTargetPosition(self.clientID, objHandle, oldAngle + i * (angle - oldAngle)/STEP, vrep.simx_opmode_oneshot)
				time.sleep(SLEEPTIME)

import cubebot, qLearner
from globalvar import *

class World():
	def __init__(self):
		self.explorationRate = EPSILON
		self.learningRate = ALPHA
		self.discount = GAMMA
		self.step = 0

		self.robot = cubebot.Cubebot()
		self.learner = qLearner.QLearner(getActions=self.cubebot.getActions, epsilon=self.explorationRate, 
			alpha=self.learningRate, gamma=self.discount)


	def start(self):
		print "======================="
		print "Start"
		print "ExplorationRate:", self.explorationRate
		print "LearningRate:", self.learningRate
		print "Discount:", self.discount
		print "======================="
		self.step = 0


	def step(self):
		self.step += 1
		state = self.robot.getCurrentState()
		actions = self.robot.getActions(state)

		if len(actions) == 0:
			self.robot.reset()
			state = self.robot.getCurrentState()
			actions = self.robot.getActions(state)

		nextState, reward = self.robot.doAction(action)
		self.learner.update(state, action, nextState, reward)

		print "Step: ", self.stepCount
		print "(s , a):", state, action
		print "(s', r):", nextState, "\t\t{:.3f}".format(reward)
		print ""


	def end(self):
		print "======================="
		print "Ending..."

		with open(fileName, 'wb') as file:
			pickle.dump(self.learner.QValue, file)
			print "QValue saved to",fileName

		self.robot.end()

		print "======================="


import pickle
import cubebot, qLearner
from globalvar import *

class World():
	def __init__(self):
		self.episode = 0
		self.robot = cubebot.Cubebot()
		self.learner = qLearner.QLearner(epsilon=EPSILON, alpha=ALPHA, gamma=GAMMA)


	def start(self):
		print "======================="
		print "Start"
		print "ExplorationRate:", EPSILON
		print "LearningRate:", ALPHA
		print "Discount:", GAMMA
		print "======================="
		self.episode = 0


	def step(self):
		self.episode += 1
		state = self.robot.getCurrentState()
		actions = self.robot.getActions(state)

		if len(actions) == 0:
			self.robot.reset()
			state = self.robot.getCurrentState()
			actions = self.robot.getActions(state)

		action = self.learner.chooseAction(state, actions)
		nextState, reward = self.robot.doAction(action)
		self.learner.update(state, action, nextState, reward)

		print "Episode: ", self.episode
		print "(s , a):", state, action
		print "(s', r):", nextState, "\t\t{:.3f}".format(reward)
		print ""


	def endTraining(self):
		print "======================="
		print "Training ended"
		print "======================="
		self.learner.stopLearning()
		self.robot.reset()


	def end(self):
		print "======================="
		print "Ending..."
		with open(RESULTFILE, 'wb') as file:
			pickle.dump(self.learner.QValue, file)
			print "QValue saved to", RESULTFILE
		self.robot.end()
		print "======================="


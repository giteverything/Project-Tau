import random, os, pickle
from globalvar import *

class QValue(dict):
	def __getitem__(self, index):
		self.setdefault(index, 0)
		return dict.__getitem__(self, index)
		
	def max(self, state):
		"""
			return max_action Q(state, action)
		"""
		allQValues = [item[1] for item in self.items() if (item[0][0] == state)]
		if len(allQValues) == 0:
			return 0
		return max(allQValues)

class QLearner():
	"""
		required to provide getActions(state) = [actions] in initialization
	"""
	def __init__(self, getActions, epsilon=EPSILON, alpha=ALPHA, gamma=GAMMA):
		self.getActions = getActions	# function that returns all possible actions in a given state s
		self.epsilon = float(epsilon)	# randomness
		self.alpha = float(alpha)		# learning rate
		self.gamma = float(gamma)		# discount

		if IMPORTQVALUE and os.path.exists(RESULTFILE):
			with open(RESULTFILE, 'rb') as file:
				self.QValue = pickle.loads(file.read())
				print "QValue loaded from file", RESULTFILE
		else:
		  self.QValue = QValue()

	def stopLearning(self):
		self.epsilon = 0.0
		self.alpha = 0.0

	def getQValue(self, state, action):
		return self.QValue[(state, action)]

	def chooseAction(self, state):
		"""
			choose action a in state s from a list of possible actions in s
		"""
		actions = self.getActions(state)
		if random.random() < self.epsilon:
			return random.choice(actions)
		else:
			values = [ self.getQValue(state,action) for action in actions ]
			maxIndex = values.index(max(values))
			return actions[maxIndex]

	def update(self, state, action, nextState, reward):
		self.QValue[(state, action)] = (1 - self.alpha) * self.QValue[(state, action)] + \
			self.alpha * (reward + self.gamma * self.QValue.max(nextState))
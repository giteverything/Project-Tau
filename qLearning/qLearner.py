import random

"""
	default values
"""
EPSILON = 0.5
ALPHA = 0.8
GAMMA = 0.8
NUMTRAINING = 1000

class QValue(dict):
	def __getitem__(self, index):
		self.setdefault(index, 0)
		return dict.__getitem__(self, index)
		
	def max(self, state):
		"""
			return max_action Q(state, action)
		"""
		allQValues = [item[1] for item in self.items() if (item[0][0] == state)]
		return max(allQValues)

	def argMax(self, state):
		"""
			return argMax_action Q(state, action)
		"""
		all = [item for item in self.items() if item[0][0] == state]
		values = [item[1] for item in all]
		maxIndex = values.index(max(values))
		return all[maxIndex][0][1]

class QLearner():
	"""
		required to provide getActions(state) = [actions] in initialization
	"""
	def __init__(self, getActions, epsilon=EPSILON, alpha=ALPHA, gamma=GAMMA):
		self.getActions = getActions	# function that returns all possible actions in a given state s
		self.epsilon = float(epsilon)	# randomness
		self.alpha = float(alpha)		# learning rate
		self.gamma = float(gamma)		# discount
		self.__QValue = QValue()

	def stopLearning():
		self.epsilon = 0.0
		self.alpha = 0.0

	def getQValue(self, state, action):
		return self.__QValue[(state, action)]

	def chooseAction(self, state):
	"""
		choose action a in state s from a list of possible actions in s
	"""
		actions = self.getActions(state)
		if random.random() < self.epsilon:
			return random.choice(actions)
		else:
			return self.QValue.argMax(state)

	def update(self, state, action, nextState, reward):
		self.__QValue[(state, action)] = (1 - self.alpha) * self.__QValue[(state, action)] + self.alpha * (reward + self.gamma * self.__QValue.max(nextState))









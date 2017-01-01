# graphicsCrawlerDisplay.py
# -------------------------
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


# graphicsCrawlerDisplay.py
# -------------------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and Pieter
# Abbeel in Spring 2013.
# For more info, see http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html

import qLearner
import time
import sys, pickle
import cubebot
import math
from math import pi as PI
from globalvar import *

class CubebotWorld:

    def __init__(self):

        self.explorationRate = EPSILON
        self.learningRate = ALPHA
        self.discount = GAMMA
        self.stepCount = 0

        # Init environment
        self.robotEnvironment = cubebot.CubebotEnvironment()

        # Init Agent
        simulationFn = lambda agent: \
          simulation.SimulationEnvironment(self.robotEnvironment,agent)
        getActions = lambda state: \
          self.robotEnvironment.getPossibleActions(state)
        self.learner = qLearner.QLearner(getActions=getActions, epsilon=self.explorationRate, alpha=self.learningRate, gamma=self.discount)

    def step(self):

        self.stepCount += 1

        if self.stepCount == NO_ITR_HALF_EPSILON:
            self.explorationRate /= 2
            self.learningRate /= 2
            # self.learner.setEpsilon(self.explorationRate)

        if self.stepCount == NO_ITR_ZERO_EPSILON:
            self.explorationRate = 0
            self.learningRate = 0
            # self.learner.setEpsilon(self.explorationRate)         

        state = self.robotEnvironment.getCurrentState()
        actions = self.robotEnvironment.getPossibleActions(state)
        if len(actions) == 0.0 or self.stepCount % 3000 == 0:
            print "---------------------------------"
            print "REINITIALIZE SIMULATION"
            print "---------------------------------"                 
            self.robotEnvironment.reset()
            state = self.robotEnvironment.getCurrentState()
            actions = self.robotEnvironment.getPossibleActions(state)

        action = self.learner.chooseAction(state)
        if action == None:
            raise ('None action returned: Code Not Complete')
        nextState, reward = self.robotEnvironment.doAction(action)
        self.learner.update(state, action, nextState, reward)

        print "Step: ", self.stepCount
        print "(s , a):", state, action
        print "(s', r):", nextState, "\t\t{:.3f}".format(reward)
        print ""
        
    def start(self):
        self.stepCount = 0
        print "---------------------------------"
        print "start simulation"
        print "ExplorationRate:", self.explorationRate
        print "LearningRate:", self.learningRate
        print "Discount:", self.discount
        print "---------------------------------"

    def saveLearningResult(self, fileName):
        with open(fileName, 'wb') as file:
            pickle.dump(self.learner.QValue, file)
            print "QValue saved to",fileName

    def end(self):
        self.learner.stopLearning()

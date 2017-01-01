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

import qLearner, cubebot
import time, math
import sys, pickle
from math import pi as PI
from globalvar import *

class CubebotWorld:

    def __init__(self):

        self.explorationRate = EPSILON
        self.learningRate = ALPHA
        self.discount = GAMMA
        self.stepCount = 0

        self.robot = cubebot.Cubebot()

        getActions = lambda state: \
          self.robot.getActions(state)
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

        state = self.robot.getCurrentState()
        actions = self.robot.getActions(state)
        if len(actions) == 0.0 or self.stepCount % 3000 == 0:
            print "---------------------------------"
            print "REINITIALIZE SIMULATION"
            print "---------------------------------"                 
            self.robot.reset()
            state = self.robot.getCurrentState()
            actions = self.robot.getActions(state)

        action = self.learner.chooseAction(state)
        if action == None:
            raise ('None action returned: Code Not Complete')
        nextState, reward = self.robot.doAction(action)
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

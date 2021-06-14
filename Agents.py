import numpy as np # argmax and random number

import random # random on approximateAgent

import math

#Save learning
import os
import json

class ApEpsilonGreedy:

  def __init__(self, initial_epsilon=0.05, min_epsilon=0.005, decay=0.9):
    self.initial_epsilon = initial_epsilon
    self.epsilon = initial_epsilon
    self.min_epsilon = min_epsilon
    self.decay = decay

  def choose(self, action_space, qValues):

    explored = False

    if np.random.rand() < self.epsilon:
      explored = True
      action = random.choice(action_space)
    else:
      action = max(qValues, key=qValues.get) # returns action with max value

    self.epsilon = max(self.epsilon*self.decay, self.min_epsilon)

    return action, explored

  def reset(self):
    self.epsilon = self.initial_epsilon

# ----------------------------- Aproximate SARSA Agent ----------------------------------------------------------------------------

class ApproximateSARSAAgent:

  def __init__(self, agentID, state, action_space, alpha=0.5, gamma=0.95, exploration_strategy=ApEpsilonGreedy()):
    self.agentID = agentID
    self.stateFeaturesLength = len(self._featureFuction(state))
    self.action_space = action_space  #must be array
    self.alpha = alpha
    self.gamma = gamma
    self.exploration = exploration_strategy
    self.acc_reward = 0

    #Creating weigths      
    self.weigths = dict()
    for action in action_space:
      self.weigths[action] = [0 for _ in range(self.stateFeaturesLength)]

  def _featureFuction(self, state):
    # features = list()

    # for index in range(len(state)):
    #   #features.append(1 + state[index] / 1000)
    #   #features.append(math.cos(state[index]) + math.sin(state[index]))

    # return features

    return state

  def act(self, state):
    state_features = self._featureFuction(state)

    qValues = dict()
    for action in self.weigths:
      qValues[action] = self._calculate_QValue(state_features, action)

    chosen_action, explored = self.exploration.choose(self.action_space, qValues)
    return chosen_action, qValues, explored

  def learn(self, state, action, next_state, next_action, reward, simulationStep = 0, printLearning=False):

    info = dict()
    info['learning_state'] = state
    info['learning_action'] = action
    info['learning_next_state'] = next_state
    info['learning_next_action'] = next_action
    info['learning_reward'] = reward

    state_features = self._featureFuction(state)

    s_Qvalue = self._calculate_QValue(state_features, action)
    
    nextStateQvalue = self._calculate_QValue(next_state, next_action)

    #info['learning_a_weigths_before'] = self.weigths[action]

    # dif = (r + gamma * max (Qvalue(nextState, nextAction))) -  Qvalue(state, action)
    dif = (reward + self.gamma*nextStateQvalue) - s_Qvalue

    for index in range(self.stateFeaturesLength):
      self.weigths[action][index] = self.weigths[action][index] + self.alpha * dif *  state_features[index]

    self.acc_reward += reward

    #info['learning_a_weigths_after'] = self.weigths[action]

    info['learning_features'] = state_features
    info['learning_saQvalue'] = s_Qvalue
    info['learning_nsnaQvalue'] = nextStateQvalue
    info['learning_dif'] = dif

    if printLearning:
      print("\n--Learning {} - {}--\n{}\n".format(self.agentID, simulationStep, info))

    return info

  def _calculate_QValue(self, state_features, action):
    #print ("Calculating - {}".format(type(action)))
    qValue = 0
    for index in range(self.stateFeaturesLength):
      qValue += (self.weigths[action][index] * state_features[index])
    return qValue


# Utilities --------------------------------------

  def saveLearning(self, saveLearningDirName):
    print("Saving - {}".format(self.agentID))
    filename = saveLearningDirName + self.agentID + "_Qweight.json"
    with open(filename, 'w') as file:
      json.dump(self.weigths, file, indent = 2)

  def loadLearning(self, saveLearningDirName):
    filename = saveLearningDirName + self.agentID + "_Qweight.json"

    if os.path.exists(filename):
      print("Loading - {}".format(self.agentID))
      with open(filename, 'r') as file:
        self.weigths = json.load(file)
    else:
      print("Creating - {}".format(self.agentID))
      with open(filename, 'w') as file:
        json.dump(self.weigths, file, indent = 2)

    #print(self.weigths)

  def DebugPrint(self):
    print("\n------ DebugPrint Agent ------")
    print("ID:\n\t{}".format(self.agentID))
    print("StateFeaturesLen:\n\t{}".format(self.stateFeaturesLength))
    print("Action Space:\n\t{}".format(self.action_space))
    print("Weights:\n\t{}".format(self.weigths))
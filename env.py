import os
import sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")

import traci
import sumolib
import json
import math
import statistics

# from gym import Env
# import traci.constants as tc
# from ray.rllib.env.multi_agent_env import MultiAgentEnv
# import numpy as np
import pandas as pd

from Agents import ApproximateSARSAAgent, ApEpsilonGreedy
from TrafficLight import TrafficLight
from TLInfo import TLInfo
import util

from enum import Enum

class LearningType(Enum):
  NO_LEARNING   = 1
  APPROXIMATION = 2

# REGION_ACTIONS = [E, NE, N, NW, W, SW, S, SE]
REGION_ACTIONS = ['0', '45', '90', '135', '180', '225', '270', '315']


class SumoEnvironment():
  """
  SUMO Environment for Traffic Signal Control

  :param net_file: (str) SUMO .net.xml file
  :param route_file: (str) SUMO .rou.xml file
  :param use_gui: (bool) Wheter to run SUMO simulation with GUI visualisation
  """

  def __init__(self, net_file, route_file, use_gui, simulationMaxSeconds, warmUpTime, learningtype, saveLearning, saveLearningDirName, printLearning, regionsFileName,
                alphaIntersec, gammaIntersec, alphaRegion, gammaRegion,
                intersectionList, agentsInfo,
                delta_time=5, yellow_time=3):
    self.net = net_file
    self.route = route_file

    self.use_gui = use_gui
    if self.use_gui:
        self._sumo_binary = sumolib.checkBinary('sumo-gui')
    else:
        self._sumo_binary = sumolib.checkBinary('sumo')

    self.simulationMaxSeconds = simulationMaxSeconds
    self.warmUpTime = warmUpTime

    self.learningtype = learningtype
    self.saveLearning = saveLearning
    self.saveLearningDirName = saveLearningDirName
    self.printLearning = printLearning

    self.alphaIntersec = alphaIntersec
    self.gammaIntersec = gammaIntersec
    self.alphaRegion = alphaRegion
    self.gammaRegion = gammaRegion

    self.delta_time = delta_time
    self.yellow_time = yellow_time

    #----------------- Data Structures --------------------

    self.trafficLightsDict = dict() # Main Data structure for the Traffic Lights
    self.simulation_metrics = list() # List of dictionaries with steps info

    self.sumolibNet = sumolib.net.readNet(net_file, withPrograms=True)
    tls = self.sumolibNet.getTrafficLights() #tls is a list of sumoLib TrafficLights

    for sumolibTL in tls:
      tlID = sumolibTL.getID()
      self.trafficLightsDict[tlID] = dict()
      self.trafficLightsDict[tlID]['Info'] = TLInfo(self.sumolibNet, sumolibTL)
      #self.trafficLightsDict[tlID]['Info'].DebugPrint()

    # Hierarchy Data
    self.regionsDict = dict()
    self.regionsOrder = list()
    self.totalLevelNumber = 0
    if regionsFileName != None:
      with open(regionsFileName, 'r') as file:
        filesDict = json.load(file)
        self.regionsOrder = filesDict["Order"]
        self.totalLevelNumber = len(self.regionsOrder)
        self.regionsDict = filesDict["RegionsDict"]


    self.intersectionList = intersectionList

    if self.intersectionList:
      for intersecID in self.intersectionList:
        print(intersecID, intersecID in self.trafficLightsDict.keys())


    self.agentsInfo = agentsInfo

    if self.agentsInfo:
      self.agents_metrics = dict()
      for tlID in self.trafficLightsDict.keys():
        self.agents_metrics[tlID] = list()


    #Counters
    self.indTotal_counter = 0
    self.indIn_counter = 0
    self.indOut_counter = 0
    self.indSup_counter = 0 
    self.counter = 0


# Simulation ---------------------------------------------------------------------------------------------------------------

  def run(self, run, outputDirName):

    sumo_cmd = [self._sumo_binary,
                 '-n', self.net,
                 '-r', self.route,
                 '--tripinfo-output', outputDirName + 'tripinfo_run{}'.format(run),
                 '--statistic-output', outputDirName + 'statinfo_run{}'.format(run),
                 '--log', outputDirName + 'log_run{}'.format(run),
                 '--duration-log.statistics', 'True'
                 #'--vehroutes', 'vehRoutesFile'

                 #How long vehicles wait for departure before being skipped, defaults to -1 which means vehicles are never skipped; default: -1
                 #'--max-depart-delay', '0',
                 #Initialises the random number generator with the current system time; default: false
                 #'--random'
                 ]


    if self.use_gui:
      sumo_cmd.append('--start')

    #------------------ Launcher ------------------
    print("\n-----SUMO START {}----\n".format(run))
    traci.start(sumo_cmd)

    for tlID in self.trafficLightsDict.keys():
        self.trafficLightsDict[tlID]['TraCI'] = TrafficLight(tlID, self.trafficLightsDict[tlID]['Info'].incomingEdges, self.trafficLightsDict[tlID]['Info'].outgoingEdges, self.trafficLightsDict[tlID]['Info'].cycleTime, self.delta_time, self.yellow_time)
        #self.trafficLightsDict[tlID]['TraCI'].DebugPrint()

        #Creating here because it is used on info
        self.trafficLightsDict[tlID]['LastMeasures'] = 0.0
        self.trafficLightsDict[tlID]['AccRewards'] = 0.0

    if self.learningtype is LearningType.NO_LEARNING:
      self._no_learning_loop()
    else:

      # Warm Up
      for _ in range(self.warmUpTime):
        traci.simulationStep()
      
      self._learning_loop()

  def _simulationStep(self):
    #----------- Info -----------#
    info = self._compute_step_info(traci.simulation.getTime())
    self.simulation_metrics.append(info)

    traci.simulationStep()

  def _no_learning_loop(self):
    done = False

    while not done:
      for _ in range(self.delta_time):
        #traci.simulationStep()
        self._simulationStep()

      #----------- Time -----------#
      simulationTime = traci.simulation.getTime()

      #----------- Info -----------#
      # info = self._compute_step_info(simulationTime)
      # self.simulation_metrics.append(info)

      done = simulationTime >= self.simulationMaxSeconds #or (traci.simulation.getMinExpectedNumber() <= 0)


  def _learning_loop(self):

    #Needed for correct Learning    
    for tlID in self.trafficLightsDict.keys():
      self.trafficLightsDict[tlID]['TraCI'].resetPhase()

    # -- Regions --
    self._regionsStates(key='state')
    for regionID in self.regionsDict.keys():
      self.regionsDict[regionID]['LastMeasures'] = 0.0
      self.regionsDict[regionID]['AccRewards'] = 0.0

    # -- Intersections --
    for tlID in self.trafficLightsDict.keys():
      self.trafficLightsDict[tlID]['state'] = self.trafficLightsDict[tlID]['TraCI'].getQState() #initial_state
      self.trafficLightsDict[tlID]['action_space'] = self.trafficLightsDict[tlID]['TraCI'].getActionSpace()

    # -- Add region inicial indication on subordinates states --
    self._regionsIndications(actionKey = 'action', stateKey='state', first = True)

    # -- Intersection Agents --
    for tlID in self.trafficLightsDict.keys():
      #self.trafficLightsDict[tlID]['Agent'] = ApproximateQLAgent(tlID, state_length=len(self.trafficLightsDict[tlID]['state']),
      self.trafficLightsDict[tlID]['Agent'] = ApproximateSARSAAgent(tlID, state = self.trafficLightsDict[tlID]['state'],
                               action_space=self.trafficLightsDict[tlID]['action_space'],
                               alpha= self.alphaIntersec,
                               gamma= self.gammaIntersec,
                               exploration_strategy=ApEpsilonGreedy(decay=1)) #no decay
      if self.saveLearning:
        self.trafficLightsDict[tlID]['Agent'].loadLearning(self.saveLearningDirName)
      #self.trafficLightsDict[tlID]['Agent'].DebugPrint()

    # -- Region Agents --
    for regionID in self.regionsDict.keys():
      #self.regionsDict[regionID]['Agent'] = ApproximateQLAgent(regionID, state_length=len(self.regionsDict[regionID]['state']),
      self.regionsDict[regionID]['Agent'] = ApproximateSARSAAgent(regionID, state = self.regionsDict[regionID]['state'],
                               action_space=REGION_ACTIONS,
                               alpha= self.alphaRegion,
                               gamma= self.gammaRegion,
                               exploration_strategy=ApEpsilonGreedy(decay=1)) #no decay
      if self.saveLearning:
        self.regionsDict[regionID]['Agent'].loadLearning(self.saveLearningDirName)
      #self.regionsDict[regionID]['Agent'].DebugPrint()


    self._intersec_actions(actionKey = 'action', stateKey='state')

    #------------ Loop ------------

    done = False

    while not done:

      #self._intersec_actions(actionKey = 'action', stateKey='state')
      
      for _ in range(self.yellow_time): 
        #traci.simulationStep()
        self._simulationStep()

      for tlID in self.trafficLightsDict.keys():
        self.trafficLightsDict[tlID]['TraCI'].update_phase()

      for _ in range(self.delta_time - self.yellow_time):
        #traci.simulationStep()
        self._simulationStep()


      #----------- Simulation Time -----------#
      simulationTime = traci.simulation.getTime()
      
      
      #----------- Rewards -----------#
      # We calculate rewards before next states and next actions as intersec_action change the intersections actions, changing rygState in rewards_incentive
      intersect_rewards = self._compute_intersec_rewards()
      region_rewards = self._compute_region_rewards(intersect_rewards)

      # Incentive
      #print (simulationTime, intersect_rewards, region_rewards)
      intersect_rewards, region_rewards = self._rewards_incentive(intersect_rewards, region_rewards)
      #print (simulationTime, intersect_rewards, region_rewards)

      for tlID in self.trafficLightsDict.keys():
        self.trafficLightsDict[tlID]['AccRewards'] += intersect_rewards[tlID]
        self.trafficLightsDict[tlID]['info_reward'] = intersect_rewards[tlID]

      for regionID in self.regionsDict.keys():
        self.regionsDict[regionID]['AccRewards'] += region_rewards[regionID]

      #----------- Next States and Next Actions -----------#
      self._regionsStates(key='next_state')

      for tlID in self.trafficLightsDict.keys():
        self.trafficLightsDict[tlID]['next_state'] = self.trafficLightsDict[tlID]['TraCI'].getQState()

      # Add region indication (action) on subordinate states
      self._regionsIndications(actionKey = 'next_action', stateKey='next_state')

      
      self._intersec_actions(actionKey = 'next_action', stateKey='next_state')


      #----------- Learning -----------#

      for tlID in self.trafficLightsDict.keys():
        state = self.trafficLightsDict[tlID]['state']
        action = self.trafficLightsDict[tlID]['action']
        next_state = self.trafficLightsDict[tlID]['next_state']
        next_action = self.trafficLightsDict[tlID]['next_action']

        #self.trafficLightsDict[tlID]['Agent'].learn(state = state, action = action, next_state = next_state, reward=intersect_rewards[tlID], simulationStep=simulationTime, printLearning=True)
        self.trafficLightsDict[tlID]['learning_info'] = self.trafficLightsDict[tlID]['Agent'].learn(state = state, action = action, next_state = next_state, next_action = next_action, reward=intersect_rewards[tlID], simulationStep=simulationTime, printLearning=self.printLearning)

        self.trafficLightsDict[tlID]['state'] = self.trafficLightsDict[tlID]['next_state']
        self.trafficLightsDict[tlID]['action'] = self.trafficLightsDict[tlID]['next_action']

      for regionID in self.regionsDict.keys():
        state = self.regionsDict[regionID]['state']
        action = self.regionsDict[regionID]['action']
        next_state = self.regionsDict[regionID]['next_state']
        next_action = self.regionsDict[regionID]['next_action']

        #self.regionsDict[regionID]['Agent'].learn(state = state, action = action, next_state = next_state, reward=region_rewards[regionID], simulationStep=simulationTime, printLearning=True)
        self.regionsDict[regionID]['learning_info'] = self.regionsDict[regionID]['Agent'].learn(state = state, action = action, next_state = next_state, next_action = next_action, reward=region_rewards[regionID], simulationStep=simulationTime, printLearning=self.printLearning)

        self.regionsDict[regionID]['state'] = self.regionsDict[regionID]['next_state']
        self.regionsDict[regionID]['action'] = self.regionsDict[regionID]['next_action']


      #----------- Info -----------#
      # info = self._compute_step_info(simulationTime)
      # self.simulation_metrics.append(info)

      if self.agentsInfo:
        for agentID in self.trafficLightsDict.keys():
          self.agents_metrics[agentID].append(self._compute_agent_step_info(simulationTime, agentID))

      # DEBUG
      # time = traci.simulation.getTime()
      # if time % 1000 == 0:
      #   input("{} - Press Enter to continue...".format(time))

      #----------- Loop -----------#
      done = simulationTime >= self.simulationMaxSeconds #or (traci.simulation.getMinExpectedNumber() <= 0)

    # --------- After Loop ---------

    # -- Save Learning --
    if self.saveLearning:
      for tlID in self.trafficLightsDict.keys():
        self.trafficLightsDict[tlID]['Agent'].saveLearning(self.saveLearningDirName)

      for regionID in self.regionsDict.keys():
        self.regionsDict[regionID]['Agent'].saveLearning(self.saveLearningDirName)

  def close(self):
    traci.close()

  def _regionsStates(self, key):
    for levelID in range(self.totalLevelNumber):
      #print("---- Regions Level {} ----".format(levelID + 1))
      for regionID in self.regionsOrder[levelID]:
        self.regionsDict[regionID][key] = list()

        # if level is the first Level
        if levelID == 0:
          for subTlID in self.regionsDict[regionID]["subordinates"]:
            self.regionsDict[regionID][key] += self.trafficLightsDict[subTlID]['TraCI'].get_result_vectors()
          #print (regionID, self.regionsDict[regionID]['state'])
        else:
          regionState = list()
          for subID in self.regionsDict[regionID]["subordinates"]:
            regionState += util.vectorAdderAngMag(self.regionsDict[subID][key])
          self.regionsDict[regionID][key] = regionState
          #print(regionID, regionState)


  def _regionsIndications(self, actionKey, stateKey, first=False):
    for levelID in reversed(range(self.totalLevelNumber)):
      for regionID in self.regionsOrder[levelID]:
        if first:
          self.regionsDict[regionID][actionKey] = '0'
        else:
          self.regionsDict[regionID][actionKey], qValues, explored = self.regionsDict[regionID]['Agent'].act(self.regionsDict[regionID][stateKey])

          # # No learning part
          # if len(self.regionsDict[regionID][stateKey]) % 4 != 0:
          #   inNOutSums = util.vectorAdderAngMag(self.regionsDict[regionID][stateKey][:-1])
          #   resultvector = util.addVectorPair(inNOutSums)
          #   #print (self.regionsDict[regionID][stateKey][:-1], 'hey', self.regionsDict[regionID][stateKey][-1], inNOutSums, resultvector, util.angleToIndication(resultvector[0]))

          #   #self.regionsDict[regionID][actionKey] = str (util.angleToIndication(resultvector[0])) # angle of resultant in - out
            
          #   self.regionsDict[regionID][actionKey] = str (util.angleToIndication(inNOutSums[0])) # only in resultant angle
          # else:
          #   inNOutSums = util.vectorAdderAngMag(self.regionsDict[regionID][stateKey])
          #   resultvector = util.addVectorPair(inNOutSums)
          #   #print (self.regionsDict[regionID][stateKey], inNOutSums, resultvector, util.angleToIndication(resultvector[0])) #, self.regionsDict[regionID][actionKey]
            
          #   #self.regionsDict[regionID][actionKey] = str (util.angleToIndication(resultvector[0])) # angle of resultant in - out
            
          #   self.regionsDict[regionID][actionKey] = str (util.angleToIndication(inNOutSums[0])) # only in resultant angle

        if levelID == 0:
          #print (levelID, util.isIndicationEqual(self.regionsDict[regionID][stateKey], int(self.regionsDict[regionID][actionKey])))
          indTotal, indSup, indIn, indOut, self.counter = util.isIndicationEqual(self.regionsDict[regionID][stateKey], int(self.regionsDict[regionID][actionKey]), self.counter)
          if (indTotal):
            self.indTotal_counter +=1
          if indSup:
            self.indSup_counter += 1
          if indIn:
            self.indIn_counter += 1
          if indOut:
            self.indOut_counter +=1
          #print ('C: {}, IndTotal: {} ({}), IndSup: {} ({}), IndIn: {} ({}), IndOut: {} ({})'.format(self.counter, self.indTotal_counter, self.indTotal_counter/self.counter, self.indSup_counter, self.indSup_counter/self.counter, self.indIn_counter, self.indIn_counter/self.counter, self.indOut_counter, self.indOut_counter/self.counter))


        if levelID == 0:
          for subTlID in self.regionsDict[regionID]["subordinates"]:
            self.trafficLightsDict[subTlID][stateKey] += [int(self.regionsDict[regionID][actionKey])] # we use actions as str because JSON turns int to str when saving weights, so we have to cats it back to int
        else:
          for subRegionID in self.regionsDict[regionID]["subordinates"]:
            self.regionsDict[subRegionID][stateKey] += [int(self.regionsDict[regionID][actionKey])] # we use actions as str because JSON turns int to str when saving weights, so we have to cats it back to int


  def _intersec_actions(self, actionKey, stateKey):
    for tlID in self.trafficLightsDict.keys():
      self.trafficLightsDict[tlID][actionKey], qValues, explored = self.trafficLightsDict[tlID]['Agent'].act(self.trafficLightsDict[tlID][stateKey])

      # Information to csv
      self.trafficLightsDict[tlID]['info_state'] = self.trafficLightsDict[tlID][stateKey]
      self.trafficLightsDict[tlID]['info_qValues'] = qValues
      self.trafficLightsDict[tlID]['info_explored'] = explored
      self.trafficLightsDict[tlID]['info_agent_action'] = self.trafficLightsDict[tlID][actionKey]

      next_phase, starved = self.trafficLightsDict[tlID]['TraCI'].set_next_phase(self.trafficLightsDict[tlID][actionKey])

      # We set the action as the next_phase because with the starvation the next_phase may be different than the choosen action
      self.trafficLightsDict[tlID][actionKey] = str(next_phase)

      #Information to csv
      self.trafficLightsDict[tlID]['info_starved'] = starved
      self.trafficLightsDict[tlID]['info_taken_action'] = self.trafficLightsDict[tlID][actionKey]


  def _compute_intersec_rewards(self):
    rewards = dict()
    for tlID in self.trafficLightsDict.keys():
      tl_wait = self.trafficLightsDict[tlID]['TraCI'].get_waiting_time() #/ 100000
      rewards[tlID] = self.trafficLightsDict[tlID]['LastMeasures'] - tl_wait
      self.trafficLightsDict[tlID]['LastMeasures'] = tl_wait
      # self.trafficLightsDict[tlID]['AccRewards'] += rewards[tlID]

      # self.trafficLightsDict[tlID]['info_reward'] = rewards[tlID]
    return rewards


  def _compute_region_rewards(self, intersect_rewards):
    rewards = dict()
    # Direct hierarchical order
    for levelID in range(self.totalLevelNumber):
      for regionID in self.regionsOrder[levelID]:
        if levelID == 0:
          rewardArray = [intersect_rewards[subID] for subID in self.regionsDict[regionID]["subordinates"]]
        else:
          rewardArray = [rewards[subID] for subID in self.regionsDict[regionID]["subordinates"]]

        currentMeasure = sum(rewardArray) / len(rewardArray)
        rewards[regionID] = currentMeasure - self.regionsDict[regionID]['LastMeasures']

        self.regionsDict[regionID]['LastMeasures'] = currentMeasure

    return rewards


  def _rewards_incentive(self, intersect_rewards, region_rewards):
    myRangePer = 1

    for levelID in range(self.totalLevelNumber):
      for regionID in self.regionsOrder[levelID]:
        if levelID == 0:
          for subTlID in self.regionsDict[regionID]["subordinates"]:
            rygState = self.trafficLightsDict[subTlID]['TraCI'].getRedYellowGreenState().upper()
            links = self.trafficLightsDict[subTlID]['Info'].getLinks()
            #print(subTlID, rygState, [links[index] for index in range(len(rygState)) if rygState[index] == 'G'])
            rygStateOpenLinksAngles = [links[index] for index in range(len(rygState)) if rygState[index] == 'G']
            maxSigma = max( [self._sigma_function( angle, int(self.regionsDict[regionID]['action']) ) for angle in rygStateOpenLinksAngles] )
            #print (maxSigma)
            if intersect_rewards[subTlID] >= 0 :
              intersect_rewards[subTlID] = intersect_rewards[subTlID] + intersect_rewards[subTlID] * maxSigma * myRangePer
            else:
              intersect_rewards[subTlID] = intersect_rewards[subTlID] - intersect_rewards[subTlID] * maxSigma * myRangePer
        else:
          for subID in self.regionsDict[regionID]["subordinates"]:
            if region_rewards[subID] >= 0:
              region_rewards[subID] = region_rewards[subID] + region_rewards[subID] * self._sigma_function(int(self.regionsDict[subID]['action']), int(self.regionsDict[regionID]['action'])) * myRangePer
            else:
              region_rewards[subID] = region_rewards[subID] - region_rewards[subID] * self._sigma_function(int(self.regionsDict[subID]['action']), int(self.regionsDict[regionID]['action'])) * myRangePer

    return intersect_rewards, region_rewards


  def _sigma_function(self, subActionAngle, indicationAngle):
    cosBetweenVectors = math.cos(math.radians(subActionAngle - indicationAngle))
    if cosBetweenVectors < 0:
      cosBetweenVectors = 0 # no punishment
    return cosBetweenVectors


# Info/CSV  ---------------------------------------------------------------------------------------------------------

  def _compute_step_info(self, simulationTime):
    infoDict = dict()
    infoDict['step_time'] = simulationTime

    infoDict['ave_rewards'] = round( statistics.mean([self.trafficLightsDict[tlID]['AccRewards'] for tlID in self.trafficLightsDict.keys()]) , 4)

    infoDict['total_veh_in_network'] = traci.vehicle.getIDCount()
    infoDict['departed_veh'] = traci.simulation.getDepartedNumber()
    infoDict['arrived_veh'] = traci.simulation.getArrivedNumber()
    infoDict['startingTeleport_veh'] = traci.simulation.getStartingTeleportNumber()
    infoDict['endingTeleport_veh'] = traci.simulation.getEndingTeleportNumber()

    ave_ocuppancy = [self.trafficLightsDict[tlID]['TraCI'].get_ave_occupancy() for tlID in self.trafficLightsDict.keys()]
    infoDict['ave_occupancy'] = round( statistics.mean(ave_ocuppancy), 4)
    infoDict['stdev_occupancy'] = round( statistics.stdev(ave_ocuppancy), 4)
    
    wait_time = [self.trafficLightsDict[tlID]['TraCI'].get_waiting_time() for tlID in self.trafficLightsDict.keys()]
    #infoDict['total_wait'] = sum(wait_time)
    infoDict['ave_wait'] = round( statistics.mean(wait_time), 4)

    stopped = [self.trafficLightsDict[tlID]['TraCI'].get_stopped_vehicles_num() for tlID in self.trafficLightsDict.keys()]
    #infoDict['total_stopped'] = sum(stopped)
    infoDict['ave_stopped'] = round( statistics.mean(stopped), 4)

    ave_travel_time = [self.trafficLightsDict[tlID]['TraCI'].get_lanes_ave_travel_time() for tlID in self.trafficLightsDict.keys()]
    #infoDict['total_travel_time'] = sum(ave_travel_time)
    infoDict['ave_travel_time'] = round( statistics.mean(ave_travel_time), 4)

    ave_mean_speed = [self.trafficLightsDict[tlID]['TraCI'].get_lanes_ave_mean_speed() for tlID in self.trafficLightsDict.keys()]
    #infoDict['total_mean_speed'] = sum(ave_mean_speed)
    infoDict['ave_mean_speed'] = round( statistics.mean(ave_mean_speed), 4)

    if self.intersectionList:
      infoDict['inter_ave_wait'] = round( statistics.mean([self.trafficLightsDict[tlID]['TraCI'].get_waiting_time() for tlID in self.intersectionList]), 4)
      infoDict['inter_ave_stopped'] = round( statistics.mean([self.trafficLightsDict[tlID]['TraCI'].get_stopped_vehicles_num() for tlID in self.intersectionList]) , 4)
      infoDict['inter_ave_travel_time'] = round( statistics.mean([self.trafficLightsDict[tlID]['TraCI'].get_lanes_ave_travel_time() for tlID in self.intersectionList]), 4)
      infoDict['inter_ave_mean_speed'] = round( statistics.mean([self.trafficLightsDict[tlID]['TraCI'].get_lanes_ave_mean_speed() for tlID in self.intersectionList]), 4)
    
    return infoDict

  def _compute_agent_step_info(self, simulationTime, agentID):
    agent_step_info = dict()

    agent_step_info['step_time'] = simulationTime

    # Current
    agent_step_info['state'] = self.trafficLightsDict[agentID]['info_state']
    
    agent_step_info['qValues'] = self.trafficLightsDict[agentID]['info_qValues']
    agent_step_info['explored'] = self.trafficLightsDict[agentID]['info_explored']

    agent_step_info['agent_action'] = self.trafficLightsDict[agentID]['info_agent_action']
    agent_step_info['starved'] = self.trafficLightsDict[agentID]['info_starved']
    agent_step_info['taken_action'] = self.trafficLightsDict[agentID]['info_taken_action']

    agent_step_info['reward'] = self.trafficLightsDict[agentID]['info_reward']
    agent_step_info['acc_reward'] = self.trafficLightsDict[agentID]['AccRewards']

    agent_step_info.update(self.trafficLightsDict[agentID]['learning_info'])

    agent_step_info.update(self.trafficLightsDict[agentID]['TraCI'].get_controller_info())

    return agent_step_info


  def save_csv(self, out_csv_name, run, options):
    if out_csv_name is not None:
      df = pd.DataFrame(self.simulation_metrics)

      lastStepsTime = self.simulationMaxSeconds - self.simulationMaxSeconds / 10 #last 10% steps
      lastStepsDF = df[df['step_time'] >= lastStepsTime].copy()
      
      df.loc['mean'] = df.mean().round(4)
      df.loc['stdev'] = df.std().round(4)

      lastStepsDF.loc['mean'] = lastStepsDF.mean().round(4)
      lastStepsDF.loc['stdev'] = lastStepsDF.std().round(4)

      ave_wait = df.loc['mean', 'ave_wait']
      ls_ave_wait = lastStepsDF.loc['mean', 'ave_wait']

      df.to_csv(out_csv_name + '_all_run{}_aveWait_{}.csv'.format(run, ave_wait), index=False, decimal=',')
      lastStepsDF.to_csv(out_csv_name + '_lastSteps_run{}_aveWait_{}.csv'.format(run, ls_ave_wait), index=False, decimal=',')

      print ("\n\n---- DataFrame Mean ----\n{}\n".format(df.loc['mean']))
      print ("\n\n---- LastSteps DF Mean ----\n{}\n".format(lastStepsDF.loc['mean']))


      if self.agentsInfo:
        for agentID in self.trafficLightsDict.keys():
          agentDF = pd.DataFrame(self.agents_metrics[agentID])
          agentDF.loc['mean'] = agentDF.mean()
          agentDF.loc['stdev'] = agentDF.std()
          agentDF.to_csv(out_csv_name + '_run{}_{}.csv'.format(run, agentID), index=False, decimal=',')    

    self.simulation_metrics = list()

    if self.agentsInfo:
      for tlID in self.trafficLightsDict.keys():
        self.agents_metrics[tlID] = list()

    if self.totalLevelNumber:
      print ('C: {}, IndTotal: {} ({}), IndSup: {} ({}), IndIn: {} ({}), IndOut: {} ({})'.format(self.counter, self.indTotal_counter, self.indTotal_counter/self.counter, self.indSup_counter, self.indSup_counter/self.counter, self.indIn_counter, self.indIn_counter/self.counter, self.indOut_counter, self.indOut_counter/self.counter))

    return ave_wait, df[options]
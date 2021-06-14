import os
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")

import math
import traci
#import traci.constants as tc

import statistics

class TrafficLight:
  """
  This class represents a Traffic Signal of an intersection
  It is responsible for retrieving information and changing the traffic phase using Traci API
  """

  def __init__(self, tl_id, incomingEdgesDict, outgoingEdgesDict, cycleTime, delta_time, yellow_time, min_green=10):
    self.id = tl_id
    self.incomingEdges = incomingEdgesDict
    self.outgoingEdges = outgoingEdgesDict
    self.controlledLanes = list(dict.fromkeys(traci.trafficlight.getControlledLanes(self.id))) # list without duplicates of the controlled lanes, with python3 the order is kept

    # for lane in self.controlledLanes:
    #   traci.lane.subscribe(lane, [tc.LAST_STEP_OCCUPANCY, tc.LAST_STEP_VEHICLE_HALTING_NUMBER, tc.VAR_LENGTH, tc.VAR_WAITING_TIME, tc.VAR_CURRENT_TRAVELTIME, tc.LAST_STEP_MEAN_SPEED])
    
    self.phases = traci.trafficlight.getCompleteRedYellowGreenDefinition(self.id)[0].phases #get the [0] because we only have one program for each traffic light
    self.greenPhasesIndexes = [phaseIndex for phaseIndex in range(len(self.phases)) if 'G' in self.phases[phaseIndex].state]

    self.min_green = min_green
    self.max_green = cycleTime

    self.redTimeCounters = {phase: 0 for phase in self.greenPhasesIndexes}
    self.maxRed = cycleTime

    self.delta_time = delta_time
    self.yellow_time = yellow_time
    self.time_on_phase = 0.0

    self.next_phase = traci.trafficlight.getPhase(self.id)


  # ---- Controlling Traffic Light Action ----

  def set_next_phase(self, action):
    #Sets what will be the next green phase and sets yellow phase if the next phase is different than the current
    
    new_phase = int(action) #Json saves int as str
    
    currentPhase = traci.trafficlight.getPhase(self.id)

    starved, starvingPhase = self._check_max_red_time()

    if self.time_on_phase < self.min_green:
      self._keep_phase(currentPhase)
    elif starved:
      #print ("starving")
      self._change_phase(currentPhase, starvingPhase)
    elif currentPhase == new_phase:
      self._keep_phase(currentPhase)
    else:
      self._change_phase(currentPhase, new_phase)

    return self.next_phase, starved

  def _keep_phase(self, currentPhase):
    self.time_on_phase += self.delta_time
    self.next_phase = currentPhase

  def _change_phase(self, currentPhase, new_phase):
    self.time_on_phase = self.delta_time - self.yellow_time
    self.next_phase = new_phase
    traci.trafficlight.setPhase(self.id, currentPhase + 1)  # turns yellow

  def _check_max_red_time(self):
    #print("checking")
    redMaxValuePhase = max(self.redTimeCounters, key=self.redTimeCounters.get)

    if self.redTimeCounters[redMaxValuePhase] >= self.maxRed:
      return True, redMaxValuePhase
    else:
      return False, None

  def update_phase(self):
    #Change the next green_phase after it is set by set_next_phase method
    traci.trafficlight.setPhase(self.id, self.next_phase)
    self._updateRedTimeCounters(self.next_phase)

  def _updateRedTimeCounters(self, choosenPhase):
    for counter in self.redTimeCounters:
      if counter != choosenPhase:
        self.redTimeCounters[counter] += self.delta_time
      else:
        self.redTimeCounters[counter] = 0

  def resetPhase(self):
    traci.trafficlight.setPhase(self.id, self.greenPhasesIndexes[0])

  #---- Learning Data ----

  def getQState(self):
    #state = [current phase one-hot encoded, elapsedTime / maxGreenTime, density for each lane, queue for each lane]
    #density = the number of vehicles on link e ∈ E divided by it’s storage capacity 
    #queue = the number of queued vehicles on lane l ∈ L divided by the storage capacity of the lane.
    
    # results = dict()
    # for lane in self.controlledLanes:
    #   results[lane] = traci.lane.getSubscriptionResults(lane)

    #phase_id = [1 if traci.trafficlight.getPhase(self.id) == i else 0 for i in self.greenPhasesIndexes] #one-hot encoding
    #phase_id = [traci.trafficlight.getPhase(self.id)]
    #elapsed = [self.time_on_phase / self.max_green]
    density = self._get_lanes_density()
    queue = self._get_lanes_queue()
    
    #state = phase_id + elapsed + density + queue
    #state = elapsed + density + queue
    state = density + queue

    return state

  def _get_lanes_density(self):
    # for lane in self.controlledLanes:
    #   if traci.lane.getLastStepOccupancy(lane) == traci.lane.getLastStepVehicleNumber(lane) / traci.lane.getLength(lane):
    #     print ('hey')

    return [round( traci.lane.getLastStepOccupancy(lane), 4) for lane in self.controlledLanes]
    #results[self.controlledLanes[0]][tc.LAST_STEP_OCCUPANCY] == traci.lane.getLastStepOccupancy(self.controlledLanes[0])
    #return [results[lane][tc.LAST_STEP_OCCUPANCY] for lane in self.controlledLanes]
    

  def _get_lanes_queue(self):
    vehicle_size_min_gap = 7.5  # 5(vehSize) + 2.5(minGap) meters
    return [ round( (traci.lane.getLastStepHaltingNumber(lane) * vehicle_size_min_gap) / traci.lane.getLength(lane), 4) for lane in self.controlledLanes]
    #results[self.controlledLanes[0]][tc.LAST_STEP_VEHICLE_HALTING_NUMBER] == traci.lane.getLastStepHaltingNumber(self.controlledLanes[0]) and results[self.controlledLanes[0]][tc.VAR_LENGTH] == traci.lane.getLength(self.controlledLanes[0])
    #return [ (results[lane][tc.LAST_STEP_VEHICLE_HALTING_NUMBER] * vehicle_size_min_gap) / results[lane][tc.VAR_LENGTH] for lane in self.controlledLanes]

  def getActionSpace(self):
    return [str(phase) for phase in self.greenPhasesIndexes] #Json saves int as str

  def get_waiting_time(self):
    wait_time = round( sum([traci.lane.getWaitingTime(lane) for lane in self.controlledLanes]), 4)
    return wait_time

  # End Learning Data ----------

  def get_result_vectors(self):
    resultVectorList = list()
    resultVectorList = self._calculate_result_vector(self.incomingEdges)
    resultVectorList += (self._calculate_result_vector(self.outgoingEdges))

    # returns [angle In, mag In, angle Out, magOut]
    return resultVectorList 

  def _calculate_result_vector(self, edgesDict):

    resultX = 0
    resultY = 0
    for edgeID, edgeAngle in edgesDict.items():
      mag = traci.edge.getLastStepVehicleNumber(edgeID)
      xComp = round(math.cos(math.radians(edgeAngle)) * mag, 4)
      yComp = round(math.sin(math.radians(edgeAngle)) * mag, 4)
      resultX += xComp
      resultY += yComp

    mag = round (math.sqrt(resultX * resultX + resultY * resultY), 4)
    degrees = round ( (math.degrees(math.atan2(resultY,resultX)) + 360) % 360, 4 )

    return [degrees, mag]

  def getRedYellowGreenState(self):
    return traci.trafficlight.getRedYellowGreenState(self.id)

  # Metrics --------------------

  def get_ave_occupancy(self):
    ave_occupancy = statistics.mean([traci.lane.getLastStepOccupancy(lane) for lane in self.controlledLanes])
    return round(ave_occupancy, 4)

  def get_stopped_vehicles_num(self):
    # if traci.lane.getSubscriptionResults(self.controlledLanes[0])[tc.LAST_STEP_VEHICLE_HALTING_NUMBER] == traci.lane.getLastStepHaltingNumber(self.controlledLanes[0]):
    #   print("equal")
    stopped_veh = sum([traci.lane.getLastStepHaltingNumber(lane) for lane in self.controlledLanes])
    return round(stopped_veh, 4)

  def get_lanes_ave_travel_time(self):
    travel_time = statistics.mean([traci.lane.getTraveltime(lane) for lane in self.controlledLanes])
    return round(travel_time, 4)

  def get_lanes_ave_mean_speed(self):
    mean_speed = statistics.mean([traci.lane.getLastStepMeanSpeed(lane) for lane in self.controlledLanes])
    return round(mean_speed, 4)

  # End Metrics ----------------

  def get_controller_info(self):
    info = dict()

    vehicle_size_min_gap = 7.5  # 5(vehSize) + 2.5(minGap) meters

    for lane in self.controlledLanes:
      info[lane + '_ocup'] = round( traci.lane.getLastStepOccupancy(lane), 4)
      info[lane + '_queue'] = round( (traci.lane.getLastStepHaltingNumber(lane) * vehicle_size_min_gap) / traci.lane.getLength(lane), 4)
      info[lane+ '_wait'] = round( traci.lane.getWaitingTime(lane), 4)
      info[lane + '_stopped'] = traci.lane.getLastStepHaltingNumber(lane)
      info[lane + '_travel_time'] = round( traci.lane.getTraveltime(lane), 4)
      info[lane + '_mean_speed'] = round( traci.lane.getLastStepMeanSpeed(lane), 4)

    info['phase_index'] = traci.trafficlight.getPhase(self.id)

    info['wait_time'] = self.get_waiting_time()
    info['ave_ocuppancy'] = self.get_ave_occupancy()
    info['stopped'] = self.get_stopped_vehicles_num()
    info['ave_travel'] = self.get_lanes_ave_travel_time()
    info['ave_mean_speed'] = self.get_lanes_ave_mean_speed()

    return info

  def DebugPrint(self):
    print("\n------ DebugPrint Traci TL ------")
    print("ID:\n\t{}".format(self.id))
    print("InEdges:\n\t{}".format(self.incomingEdges))
    print("OutEdges:\n\t{}".format(self.outgoingEdges))
    print("ControlledLanes:\n\t{}".format(self.controlledLanes))
    #print("Phases:\n\t{}".format(self.phases))
    print('GreenPhasesIndexes:\n\t{}'.format(self.greenPhasesIndexes))
    print('RedTimeCounters:\n\t{} - {}'.format(self.redTimeCounters, self.maxRed))
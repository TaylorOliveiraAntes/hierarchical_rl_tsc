import math

class TLInfo:

  def __init__(self, sumolibNet, sumolibTL):
      self.id = sumolibTL.getID()
      self.sumolibTL = sumolibTL
      self.greenPhasesIndexes = self._getGreenPhasesIndexes()
      self.cycleTime = self._getCycleTime()

      #self.net = sumolibNet
      #self.edges = sumolibTL.getEdges() # only return the incoming/controlles edges
      self.incomingSumolibEdges, self.outgoingSumolibEdges = self._incomingOutgoingSumolibEdges()
      self.incomingEdges = {inEdge.getID() : self._edgeVector(inEdge) for inEdge in self.incomingSumolibEdges }  #sumolibNet.getNode(self.id).getIncoming()}
      self.outgoingEdges = {outEdge.getID() : self._edgeVector(outEdge) for outEdge in self.outgoingSumolibEdges }

      self.linksVector = self._getLinks(sumolibNet)

  def _getGreenPhasesIndexes(self):
    program = self.sumolibTL.getPrograms()['0']
    phases = program.getPhases()
    greenPhasesIndexes = [phaseIndex for phaseIndex in range(len(phases)) if 'G' in phases[phaseIndex].state]
    return greenPhasesIndexes

  def _getCycleTime(self):
    program = self.sumolibTL.getPrograms()['0']
    return (sum ([phase.duration for phase in program.getPhases()]) )

  def _incomingOutgoingSumolibEdges(self):
    sumolibTLLinks = self.sumolibTL.getLinks()

    incomingLaneList = list()
    outgoingLaneList = list()

    for link in sumolibTLLinks:
      linkIncomingLane = sumolibTLLinks[link][0][0] 
      linkOutgoingLane = sumolibTLLinks[link][0][1]

      incomingLaneList.append(linkIncomingLane)
      outgoingLaneList.append(linkOutgoingLane)

    incomingEdgeList = list(dict.fromkeys( [lane.getEdge() for lane in incomingLaneList] ) )
    outgoingEdgeList = list(dict.fromkeys( [lane.getEdge() for lane in outgoingLaneList] ) )

    return incomingEdgeList, outgoingEdgeList

  def _edgeVector(self, sumolibEdge):
    fromNode = sumolibEdge.getFromNode().getCoord()
    toNode = sumolibEdge.getToNode().getCoord()
    xDif = toNode[0] - fromNode[0]
    yDif = toNode[1] - fromNode[1]
    #mag = math.sqrt(xDif * xDif + yDif * yDif)
    
    radians = (math.atan2(yDif,xDif))
    degrees = (math.degrees(radians) + 360) % 360

    # print (sumolibEdge.getID())
    # print ("X: {} Y: {}".format (xDif, yDif))
    # print ("Norm: {}, {}".format (round(xDif/mag, 4), round(yDif/mag, 4)) )
    # print ("Degres: {} -> {} -> {} Mag: {}".format( radians, degrees, math.radians(degrees) , 1))
    # print ("Norm: {}, {}".format(round(math.cos(radians), 4), round (math.sin(radians), 4)) )
    # print ("\n")

    #return (round(xDif/mag, 4), round(yDif/mag, 4))
    return round(degrees, 4)

  def _getLinks(self, sumolibNet):
    #Each link will have an angle pointing to the direction of flow that is freed if the link is activated

    linksDict = dict()

    getLinksDict = self.sumolibTL.getLinks()

    sortedList = list(getLinksDict.keys())
    sortedList.sort()

    #Sort to match signalPlan "GGGgrrrrGGGgrrrr" -> 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15

    for linkID in sortedList:
      #firstLaneID = getLinksDict[linkID][0][0].getID()
      #secondLaneID = getLinksDict[linkID][0][1].getID()

      fromNode = getLinksDict[linkID][0][0].getEdge().getFromNode().getCoord()
      toNode = getLinksDict[linkID][0][1].getEdge().getToNode().getCoord()

      xDif = toNode[0] - fromNode[0]
      yDif = toNode[1] - fromNode[1]
      #mag = math.sqrt(xDif * xDif + yDif * yDif)

      radians = (math.atan2(yDif,xDif))
      degrees = (math.degrees(radians) + 360) % 360

      #linksDict[linkID] = (round(xDif/mag, 4), round(yDif/mag, 4))
      linksDict[linkID] = round(degrees, 4)

    return linksDict

  def getLinks(self):
    return self.linksVector

  def DebugPrint(self):
    print("---- Info ID: {} ----".format(self.id))
    print("InEdges:\n{}\n".format(self.incomingEdges))
    print("OutEdges:\n{}\n".format(self.outgoingEdges))
    print("GreenPhases:\n{}\n".format(self.greenPhasesIndexes))
    print("Links:\n{}\n".format(self.linksVector))
    print("CycleTime:\n{}\n".format(self.cycleTime))
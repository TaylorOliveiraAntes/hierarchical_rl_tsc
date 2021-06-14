import math

def angMagToXY(angle, mag):
  x = round(math.cos(math.radians(angle)) * mag, 4)
  y = round(math.sin(math.radians(angle)) * mag, 4)

  return x,y

def xyToAngMag(x,y):
  ang = round ( (math.degrees(math.atan2(y,x)) + 360) % 360)
  mag = round (math.sqrt(x * x + y * y), 4)

  return ang, mag

def vectorAdderAngMag(vectorList):
  #print ("Length % 4: ", len(vectorList) % 4 == 0)
  vectorQnt = int (len(vectorList) / 4)
  #print ("VectorQnt: ", vectorQnt)

  inResultX = 0
  inResultY = 0

  outResultX = 0
  outResultY = 0

  for index in range (vectorQnt):
    angleIn = vectorList[4 * index ]
    magIn = vectorList[4 * index + 1]

    angleOut = vectorList[4 * index + 2]
    magOut = vectorList[4 *index + 3]

    #print (angleIn, magIn, angleOut, magOut)

    inX, inY = angMagToXY(angleIn, magIn)
    inResultX += inX
    inResultY += inY

    outX, outY = angMagToXY(angleOut, magOut)
    outResultX += outX
    outResultY += outY

  inResultAng, inResultMag = xyToAngMag(inResultX,inResultY)
  outResultAng, outResultMag = xyToAngMag(outResultX,outResultY)

  return [inResultAng, inResultMag, outResultAng, outResultMag]

def addVectorPair (vectorList):
  angleIn = vectorList[0]
  magIn = vectorList[1]
  angleOut = vectorList[2]
  magOut = vectorList[3]

  inX, inY = angMagToXY(angleIn, magIn)
  outX, outY = angMagToXY(angleOut, magOut)

  resultAng, resultMag = xyToAngMag(inX-outX, inY-outY)

  return [resultAng, resultMag]

def isInInterval(superiorAction, value):
  return superiorAction - 22.5 <= value <= superiorAction + 22.5

def angleToIndication(angle):
  indicationList = [0, 45, 90, 135, 180, 225, 270, 315]

  indicationMatch = 0
  for indication in indicationList:
    if isInInterval(indication, angle):
      indicationMatch = indication

  return indicationMatch

def isIndicationEqual(state, superiorAction, counter):
  counter +=1
  vectorsNumber = int(len(state)/2)

  sumInXY = [0,0]
  sumOutXY = [0,0]

  for index in range(vectorsNumber):
    angle = state[index*2]
    mag = state[index*2+1]

    x, y = angMagToXY(angle, mag)

    if index % 2 == 0:
      #print('In', angle, mag, x, y)
      sumInXY[0] += x
      sumInXY[1] += y
    else:
      #print('Out', angle, mag, x, y)
      sumOutXY[0] += x
      sumOutXY[1] += y

  sumInAngMag = [0,0]
  sumOutAngMag = [0,0]

  sumInAngMag[0], sumInAngMag[1] = xyToAngMag(sumInXY[0], sumInXY[1])
  sumOutAngMag[0], sumOutAngMag[1] = xyToAngMag(sumOutXY[0], sumOutXY[1])

  #print (sumInXY, sumInAngMag)
  #print (sumOutXY, sumOutAngMag)

  total = [sumInXY[0] - sumOutXY[0], sumInXY[1] - sumOutXY[1]]
  totalAng, totalMag = xyToAngMag(total[0], total[1])
  
  return isInInterval(superiorAction, totalAng), isInInterval(superiorAction, int(state[-1])), isInInterval(superiorAction,sumInAngMag[0]), isInInterval(superiorAction,sumOutAngMag[0]), counter
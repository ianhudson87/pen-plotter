from io import TextIOWrapper
from math import sqrt
from typing import Tuple
from matplotlib.pyplot import draw
import serial
from enum import Enum
import time

class GCode():
    index = 0
    endOfFile = False

    def __init__(self, filePath, scale: float, translationX: float, translationY: float, minStepSize: float = 1):
        with open(filePath) as f:
            self.lines = [line.rstrip('\n') for line in f]
            print(self.lines)
            self.scale = scale
            self.translationX = translationX
            self.translationY = translationY
            self.minStepSize = minStepSize

    # returns ((x,y), isLifted) or None if there is no instruction left
    def getNextInstruction(self, currentX: float, currentY: float) -> Tuple:
        if self.endOfFile: return None

        foundValidLine = False
        while(True):
            words = self.lines[self.index].split()
            self.index += 1

            if(self.index >= len(self.lines)):
                self.endOfFile = True
                return None

            instruction = words[0]
            if instruction in ["G01", "G00"]:
                x = float(words[1][1:])
                y = float(words[2][1:])
                (x, y) = self.__scaleAndTranslate(x,y)
                # print(str(x) + ", " + str(y) + " , " + str(currentX) + " , " + str(currentY) + " , " + str(vecMagnitude(currentX - x, currentY -y)))
                if(vecMagnitude(currentX - x, currentY - y) <= self.minStepSize or vecMagnitude(targetX - x, targetY - y) <= self.minStepSize):
                    # print("skipping step" + str(x) + "," + str(y))
                    continue
                isLifted = (instruction == "G00")
                return ((x, y), isLifted)

    def __scaleAndTranslate(self, x: float, y: float) -> tuple:
        return (x * self.scale + self.translationX, y * self.scale + self.translationY)

class State(Enum):
    waitingForInstruction = 1
    waitingForPlotterResponse = 2
    drawing = 3
    returningToHome = 4
    finished = 5

# returns (xPos, yPos)
def parsePosData(data: str) -> tuple:
    words = data.split()
    if(len(words) > 0 and words[0] == "[pos]"):
        print(data)
        return ((float(words[1]), float(words[2])))
    return None

# returns (xVel, yVel)
def parseVelData(data: str) -> tuple:
    words = data.split()
    if(len(words) > 0 and words[0] == "[vel]"):
        print(data)
        return ((float(words[1]), float(words[2])))
    return None

def writeVelocityAndLift(ser: serial.Serial, xVel: float, yVel: float, isLifted: int):
    output = f'{xVel:.3f}' + " " + f'{yVel:.3f}' + " " + str(isLifted)
    print("writing Velocity and Lift: " + output)
    ser.write(str.encode(output))

def vecMagnitude(x: float, y: float) -> float:
    return sqrt(pow(x, 2) + pow(y, 2))

if __name__ == '__main__':
    prevX = 0
    prevY = 0
    currentX = 0
    currentY = 0
    targetX = 0
    targetY = 0
    currentState = State.waitingForInstruction

    homePosX = -11.05
    homePosY = 11.3

    drawSpeed = 20
    closenessDistance = 0.05

    ser = serial.Serial('COM3', 9600, timeout=0.1)
    gcodeFile = "./gcodeFiles/cat.gcode"
    gcode = GCode(gcodeFile, 1.0, -16, 5, 0.1)

    time.sleep(10)

    while True:
        if currentState == State.finished:
            continue

        data = ser.readline().decode().strip()
        if(data):
            posData = parsePosData(data)
            if(posData != None):
                prevX = currentX
                prevY = currentY
                currentX = posData[0]
                currentY = posData[1]

        if currentState == State.waitingForInstruction:
            instruction = gcode.getNextInstruction(currentX, currentY)
            if(instruction == None):
                targetX = homePosX
                targetY = homePosY
                xDir = targetX - currentX
                yDir = targetY - currentY
                dirMagnitude = vecMagnitude(xDir, yDir)
                writeVelocityAndLift(ser, xDir / dirMagnitude * drawSpeed, yDir / dirMagnitude * drawSpeed, 1)
                print("returning to home!")

                currentState = State.returningToHome
            else:
                targetX = instruction[0][0]
                targetY = instruction[0][1]
                isLifted = 1 if instruction[1] else 0
                print("next coords:" + str(targetX) + "," + str(targetY) + " isLifted: " + str(isLifted))

                xDir = targetX - currentX
                yDir = targetY - currentY

                print("XDIR, YDIR " + str(xDir) + " " + str(yDir))
                dirMagnitude = vecMagnitude(xDir, yDir)

                writeVelocityAndLift(ser, xDir / dirMagnitude * drawSpeed, yDir / dirMagnitude * drawSpeed, isLifted)

                currentState = State.waitingForPlotterResponse

        elif currentState == State.waitingForPlotterResponse:
            velData = parseVelData(data)
            if(velData != None):
                currentState = State.drawing

        elif currentState == State.returningToHome:
            prevDistanceToTarget = vecMagnitude(targetX - prevX, targetY - prevY)
            currentDistanceToTarget = vecMagnitude(targetX - currentX, targetY - currentY)
            if(currentDistanceToTarget < closenessDistance or currentDistanceToTarget > prevDistanceToTarget):
                currentState = State.finished
                writeVelocityAndLift(ser, 0, 0, 1)

        elif currentState == State.drawing:
            prevDistanceToTarget = vecMagnitude(targetX - prevX, targetY - prevY)
            currentDistanceToTarget = vecMagnitude(targetX - currentX, targetY - currentY)
            if(currentDistanceToTarget < closenessDistance or currentDistanceToTarget > prevDistanceToTarget):
                currentState = State.waitingForInstruction

        


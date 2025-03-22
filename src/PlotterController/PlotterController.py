from io import TextIOWrapper
from math import sqrt
import serial
from enum import Enum
import time
from constants import Constants
from GCodeParser import GCodeParser
from helpers import vecMagnitude
import sys

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
        # print(data)
        return ((float(words[1]), float(words[2])))
    return None

# returns (xVel, yVel)
def parseVelData(data: str) -> tuple:
    words = data.split()
    if(len(words) > 0 and words[0] == "[vel]"):
        # print(data)
        return ((float(words[1]), float(words[2])))
    return None

def writeVelocityAndLift(ser: serial.Serial, xVel: float, yVel: float, isLifted: int):
    output = f'{xVel:.3f}' + " " + f'{yVel:.3f}' + " " + str(isLifted)
    print("writing Velocity and Lift: " + output)
    ser.write(str.encode(output))

def plot(gcodeFile:str, traceFile: str = "./trace.log", serialPort: str = "COM3"):

    f = open(traceFile,'w')

    prevX = 0
    prevY = 0
    currentX = 0
    currentY = 0
    targetX = 0
    targetY = 0
    currentState = State.waitingForInstruction

    canvasWidth = 12
    canvasHeight = 13.5
    canvasCenterX = -6
    canvasCenterY = 12.5

    homePosX = -10.2
    homePosY = 10.0

    drawSpeed = 30
    closenessDistance = 0.1

    ser = serial.Serial(serialPort, 9600, timeout=0.1)
    gcode = GCodeParser(gcodeFile, canvasWidth, canvasHeight, canvasCenterX, canvasCenterY, minStepSize = 0.2)

    time.sleep(5) # wait for arduino to accept serial communication

    while True:
        print("loop")
        if currentState == State.finished:
            print("DONE!")
            break

        data = ser.readlines()
        if(data):
            for dataLine in data:
                try:
                    dataLine = dataLine.decode().strip()
                except:
                    continue
                print(dataLine)
                f.write(dataLine + "\n")
                posData = parsePosData(dataLine)
                if(posData != None):
                    prevX = currentX
                    prevY = currentY
                    currentX = posData[0]
                    currentY = posData[1]

        if currentState == State.waitingForInstruction:
            instruction = gcode.getNextInstruction(currentX, currentY, targetX, targetY)
            if(instruction == None):
                targetX = homePosX
                targetY = homePosY
                xDir = targetX - currentX
                yDir = targetY - currentY
                dirMagnitude = vecMagnitude(xDir, yDir)
                writeVelocityAndLift(ser, xDir / dirMagnitude * drawSpeed, yDir / dirMagnitude * drawSpeed, 1)
                print("returning to home!")

                prevX = currentX
                prevY = currentY
                currentState = State.returningToHome
            else:
                targetX = instruction[0][0]
                targetY = instruction[0][1]
                isLifted = 1 if instruction[1] else 0
                print("nextCoords: " + str(targetX) + " " + str(targetY) + " isLifted: " + str(isLifted))
                f.write("nextCoords: " + str(targetX) + " " + str(targetY) + " isLifted: " + str(isLifted) + "\n")

                xDir = targetX - currentX
                yDir = targetY - currentY

                print("XDIR, YDIR " + str(xDir) + " " + str(yDir))
                dirMagnitude = vecMagnitude(xDir, yDir)

                writeVelocityAndLift(ser, xDir / dirMagnitude * drawSpeed, yDir / dirMagnitude * drawSpeed, isLifted)

                currentState = State.waitingForPlotterResponse

        elif currentState == State.waitingForPlotterResponse:
            if(data):
                for dataLine in data:
                    dataLine = dataLine.decode().strip()
                    velData = parseVelData(dataLine)
                    if(velData != None):
                        prevX = currentX
                        prevY = currentY
                        currentState = State.drawing
                        continue

        elif currentState == State.returningToHome:
            prevDistanceToTarget = vecMagnitude(targetX - prevX, targetY - prevY)
            currentDistanceToTarget = vecMagnitude(targetX - currentX, targetY - currentY)
            distances = f"prevDist: {prevDistanceToTarget}, currentDist: {currentDistanceToTarget}"
            if(currentDistanceToTarget < closenessDistance or currentDistanceToTarget > prevDistanceToTarget):
                currentState = State.finished
                writeVelocityAndLift(ser, 0, 0, 1)

        elif currentState == State.drawing:
            prevDistanceToTarget = vecMagnitude(targetX - prevX, targetY - prevY)
            currentDistanceToTarget = vecMagnitude(targetX - currentX, targetY - currentY)
            distances = f"prevDist: {prevDistanceToTarget}, currentDist: {currentDistanceToTarget}"
            print(distances)
            f.write(distances + "\n")
            if(currentDistanceToTarget < closenessDistance or currentDistanceToTarget > prevDistanceToTarget):
                currentState = State.waitingForInstruction


    f.close()

        


if __name__ == '__main__':
    if(len(sys.argv) < 2):
        print("Usage: PlotterController.py [gcodeFilePath]")

    gcodeFilePath = sys.argv[1]
    # total arguments
    # n = len(sys.argv)
    # print("Total arguments passed:", n)

    # print("\nArguments passed:", end = " ")
    # for i in range(1, n):
    #     print(sys.argv[i], end = " ")
        
    plot(gcodeFilePath)
from io import TextIOWrapper
from math import sqrt
from typing import Tuple
from matplotlib.pyplot import draw
import serial
from enum import Enum
import time

gcodeFile = "./gcodeFiles/totoro.gcode"

class GCode():
    index = 0
    endOfFile = False

    def __init__(self, filePath, scale: float, translationX: float, translationY: float):
        with open(filePath) as f:
            self.lines = [line.rstrip('\n') for line in f]
            print(self.lines)
            self.scale = scale
            self.translationX = translationX
            self.translationY = translationY

    def getNextPos(self) -> Tuple:
        if self.endOfFile: return None

        foundValidLine = False
        while(not foundValidLine):
            words = self.lines[self.index].split()
            self.index += 1

            if(self.index >= len(self.lines)):
                self.endOfFile = True
                return None

            if words[0] == "G01" or words[0] == "G00":
                x = float(words[1][1:])
                y = float(words[2][1:])
                return self.__scaleAndTranslate(x,y)

    def __scaleAndTranslate(self, x: float, y: float) -> tuple:
        return (x * self.scale + self.translationX, y * self.scale + self.translationY)

class State(Enum):
    waiting = 1
    drawing = 2
    finished = 3

currentX = 0
currentY = 0
nextX = 0
nextY = 0
currentState = State.waiting

drawSpeed = 20
closenessDistance = 0.1

def parseData(data: str) -> tuple:
    print(data)
    words = data.split()
    if(words[0] == "[pos]"):
        return float(words[1]), float(words[2])
        # print(str(currentX) + "," + str(currentY))

def writeVelocity(ser: serial.Serial, xVel: float, yVel: float):
    output = f'{xVel:.3f}' + " " + f'{yVel:.3f}'
    print("writing Velocity: " + output)
    ser.write(str.encode(output))

def vecMagnitude(x: float, y: float) -> float:
    return sqrt(pow(x, 2) + pow(y, 2))

if __name__ == '__main__':
    ser = serial.Serial('COM3', 9600, timeout=0.1)
    gcode = GCode(gcodeFile, 0.7, -10, 8)

    time.sleep(10)

    while True:
        if currentState == State.finished:
            continue

        data = ser.readline().decode().strip()
        if(data):
            parsedData = parseData(data)
            if(not parsedData == None):
                currentX = parsedData[0]
                currentY = parsedData[1]

        if currentState == State.waiting:
            coords = gcode.getNextPos()
            if(coords == None):
                currentState = State.finished
                writeVelocity(ser, 0, 0)
                print("finished plotting!")
                continue
            else:
                nextX = coords[0]
                nextY = coords[1]
                print("next coords:" + str(nextX) + "," + str(nextY))

                xDir = nextX - currentX
                yDir = nextY - currentY

                print("XDIR, YDIR " + str(xDir) + " " + str(yDir))
                dirMagnitude = vecMagnitude(xDir, yDir)

                writeVelocity(ser, xDir / dirMagnitude * drawSpeed, yDir / dirMagnitude * drawSpeed)

                currentState = State.drawing

        if currentState == State.drawing:
            if(vecMagnitude(nextX - currentX, nextY - currentY) < closenessDistance):
                currentState = State.waiting

        


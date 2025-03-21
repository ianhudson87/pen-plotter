from constants import Constants
from helpers import vecMagnitude
from math import sqrt

class GCodeParser():
    index = 0
    endOfFile = False

    def __init__(self, filePath, canvasWidth: float, canvasHeight: float, canvasCenterX: float, canvasCenterY: float, minStepSize: float = 0.1):
        with open(filePath) as f:
            self.lines = [line.rstrip('\n') for line in f]
            print(self.lines)

            self.canvasWidth = canvasWidth
            self.canvasHeight = canvasHeight
            self.canvasCenterX = canvasCenterX
            self.canvasCenterY = canvasCenterY

            scaleAndTranslate = self.__getScalingAndTranslation()

            self.scale = scaleAndTranslate[0]
            self.translationX = scaleAndTranslate[1][0]
            self.translationY = scaleAndTranslate[1][1]
            self.minStepSize = minStepSize

            print("scale and translate: " + str(self.scale) + ", " + str(self.translationX) + ", " + str(self.translationY))

    # returns ((x,y), isLifted) or None if there is no instruction left
    def getNextInstruction(self, currentX: float, currentY: float, targetX: float, targetY: float) -> tuple:
        if self.endOfFile: return None

        foundValidLine = False
        while(True):
            if(self.index >= len(self.lines)):
                self.endOfFile = True
                return None

            line = self.lines[self.index]
            self.index += 1

            instructionAndCoords = self.__getInstructionAndCoords(line)
            if instructionAndCoords == None:
                continue
            instruction = instructionAndCoords[0]
            x = instructionAndCoords[1][0]
            y = instructionAndCoords[1][1]
            print(str(x) + ", " + str(y) + " , " + str(currentX) + " , " + str(currentY) + " , " + str(vecMagnitude(currentX - x, currentY -y)))
            (x, y) = self.__scaleAndTranslate(x,y)
            if(vecMagnitude(currentX - x, currentY - y) <= self.minStepSize or vecMagnitude(targetX - x, targetY - y) <= self.minStepSize):
                print("skipping step" + str(x) + "," + str(y))
                continue
            isLifted = (instruction == Constants.g0Code)
            return ((x, y), isLifted)

    def __scaleAndTranslate(self, x: float, y: float) -> tuple:
        return (x * self.scale + self.translationX, y * self.scale + self.translationY)

    # return tuple (scale, (translationX, translationY))
    def __getScalingAndTranslation(self):
        i=0
        minX = 9e9
        maxX = -9e9
        minY = 9e9
        maxY = -9e9
        while(i < len(self.lines)):
            line = self.lines[i]
            i += 1

            instructionAndCoords = self.__getInstructionAndCoords(line)
            if instructionAndCoords == None:
                continue
            x = instructionAndCoords[1][0]
            y = instructionAndCoords[1][1]

            minX = min(minX, x)
            minY = min(minY, y)
            maxX = max(maxX, x)
            maxY = max(maxY, y)

        print(minX, minY, maxX, maxY)

        if(self.canvasHeight / self.canvasWidth > ((maxY - minY) / (maxX - minX))):
            # painting is too wide, scale based on width
            scale = self.canvasWidth / (maxX - minX)
        else:
            # painting is too tall, scale based on height
            scale = self.canvasHeight / (maxY - minY)

        scaledCenterX = (maxX + minX) / 2 * scale
        translationX = self.canvasCenterX - scaledCenterX

        scaledCenterY = (maxY + minY) / 2 * scale
        translationY = self.canvasCenterY - scaledCenterY

        return (scale, (translationX, translationY))

    # returns tuple of (instruction, (x,y)) if line is GCode 01 or 00. Otherwise, returns null
    def __getInstructionAndCoords(self, line: str) -> tuple:
        words = line.split()

        instruction = words[0]

        if instruction in [Constants.g0Code, Constants.g1Code]:
            x = float(words[1][1:])
            y = float(words[2][1:])
            return (instruction, (x,y))
        return None
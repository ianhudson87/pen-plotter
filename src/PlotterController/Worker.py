import sys
import PlotterController
import time

if __name__ == '__main__':
    opStatusFilePath = sys.argv[1]
    gcodeFile = sys.argv[2]

    print(opStatusFilePath)

    with open(opStatusFilePath, 'w') as opStatusFile:
        opStatusFile.write("waiting")

    while(True):
        with open(opStatusFilePath, 'r+') as opStatusFile:
            status = opStatusFile.read()
            print(status)
            if status == "queued":
                opStatusFile.write("running")
                PlotterController.plot(gcodeFile)
                opStatusFile.write("waiting")

        time.sleep(2)


        


    
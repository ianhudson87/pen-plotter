import sys
import PlotterController
import time

if __name__ == '__main__':
    if(len(sys.argv) < 3):
        print("Usage: Worker.py [opStatusFilePath] [gcodeFilePath]")

    opStatusFilePath = sys.argv[1]
    gcodeFilePath = sys.argv[2]

    print(opStatusFilePath)

    with open(opStatusFilePath, 'w') as opStatusFile:
        opStatusFile.write("waiting")

    while(True):
        with open(opStatusFilePath, 'r+') as opStatusFile:
            status = opStatusFile.read()
            print(status)
            if status == "queued":
                opStatusFile.write("running")
                PlotterController.plot(gcodeFilePath)
                opStatusFile.write("waiting")

        time.sleep(2)


        


    
import sys
import PlotterController
import time

if __name__ == '__main__':
    if(len(sys.argv) < 4):
        print("Usage: Worker.py [opStatusFilePath] [gcodeFilePath] [serialPort]")

    opStatusFilePath = sys.argv[1]
    gcodeFilePath = sys.argv[2]
    serialPort = sys.argv[3]

    print(opStatusFilePath)

    with open(opStatusFilePath, 'w') as opStatusFile:
        opStatusFile.write("waiting")

    while(True):
        with open(opStatusFilePath, 'r') as opStatusFile:
            status = opStatusFile.read()
            print(f"Status: {status}")

        if status == "queued":
            with open(opStatusFilePath, 'w') as opStatusFile:
                opStatusFile.write("running")
                PlotterController.plot(gcodeFilePath, serialPort=serialPort)
                opStatusFile.write("waiting")

        time.sleep(2)


        


    
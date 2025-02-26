import serial

def readserial(comport, baudrate):
    ser = serial.Serial(comport, baudrate, timeout=0.1)

    while True:
        data = ser.readline().decode().strip()
        if(data):
            print(data)


if __name__ == '__main__':
    print("hi")
    readserial('COM3', 9600)
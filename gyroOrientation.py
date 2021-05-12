import pyautogui as pg
import math
import time
import serial
import re

import numpy as n


INCOMINGDATAPORT = 'COM2'
DATARATE = 115200

ser = serial.Serial(port = INCOMINGDATAPORT, baudrate = DATARATE, timeout = 0.3)

ser.bytesize = serial.EIGHTBITS         # Number of bits per bytes
ser.parity = serial.PARITY_NONE         # Set parity check: no parity

ser.stopbits = serial.STOPBITS_ONE      # Number of stop bits


ser.xonxoff = False     #disable software flow control
ser.rtscts = True     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control

# Frame per second
FPS = 200
maxFrameTime = 1.0/FPS * 1000

# Dimension of the screen
WIDTH, HEIGHT = pg.size()

serialDataList = []

def readSerialData():
    if(ser.isOpen()):
        try:
            line = ser.readline().strip()
            values = line.decode('ascii').split(",")

            for i in values:
                if(i != ''):
                    serialDataList.append(i)

            return True

        except Exception as e1:
            print (str(e1))
            return False

    
    return False


def getAcceleration():
    ax = 0
    ay = 0
    az = 0

    if(len(serialDataList) > 3):
        ax = float(serialDataList.pop(0))
        ay = float(serialDataList.pop(0))
        az = float(serialDataList.pop(0))
        return (ax, ay, az)

    return ()


while(True):
    t = time.time()
    readSerialData()
    print(time.time() - t, end = " ")


    a = getAcceleration()

    print(a)
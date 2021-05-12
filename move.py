import pyautogui as pg
import math
import time
import serial
import re

import functools
import numpy as n


INCOMINGDATAPORT = 'COM2'
DATARATE = 115200

ser = serial.Serial(port = INCOMINGDATAPORT, baudrate = DATARATE, timeout = 1)

ser.bytesize = serial.EIGHTBITS         # Number of bits per bytes
ser.parity = serial.PARITY_NONE         # Set parity check: no parity

ser.stopbits = serial.STOPBITS_ONE      # Number of stop bits


ser.xonxoff = True     #disable software flow control
ser.rtscts = True     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = True       #disable hardware (DSR/DTR) flow control

input()

# Frame per second
FPS = 200
maxFrameTime = 1.0/FPS * 1000

# Dimension of the screen
WIDTH, HEIGHT = pg.size()

MovementThreshold = 0.3

ZeroXValue = 0
ZeroYValue = 9.8

MaxXValue = 20
MaxYValue = 20

MinXValue = -20
MinYValue = -20
MaxMouseMovement = 0.5

def getAcceleration():
    xR = 0
    yR = 0
    zR = 0

    if(ser.isOpen()):
        try:
            line = ser.readline().strip()
            values = line.decode('ascii').split(",")
            print(values)

            if(values != ['']):
                values = [i for i in values if i != '']
                xR = float(values[0])
                yR = float(values[1])
                zR = float(values[2])
                return True, n.array([[xR], [zR]])


        except Exception as e1:
            ser.close()
            print (str(e1))
            return False, n.array([])

    
    return False, n.array([])

def clamp(n, smallest, largest): 
    return max(smallest, min(n, largest))

def normaliseToScreenCoords(x, y):
    x_new = clamp (math.floor((x+1) / 2 * WIDTH), 0, WIDTH - 1)
    y_new = clamp (HEIGHT - math.floor((y+1) / 2 * HEIGHT), 0, HEIGHT - 1)
    
    return x_new, y_new


def screenCoordsToNormalise(x, y):
    x_new = clamp(2 * x / float(WIDTH) - 1, -1, 1) 
    y_new = clamp(-2 * y / float(HEIGHT) + 1, -1, 1)

    return x_new, y_new


def move(p):  
    pg.moveTo(p[0], p[1])
    return True

def normalise(v):
    l = n.linalg.norm(v)
    if(l != 0):
        return 1/l * v
    else:
        return v

def calcNextPos(a):
    ax = a[0, 0]
    ay = a[1, 0]

    dx = 0

    if(MovementThreshold < abs(ax - ZeroXValue)):
        dx = (2 * MaxMouseMovement)/float(MaxXValue - MinXValue) * ( ax - MinXValue ) - MaxMouseMovement
    else:
        dx = 0

    dy = 0
    if(MovementThreshold < abs(ay - ZeroYValue)):
        dy = (2 * MaxMouseMovement)/float(MaxYValue - MinYValue) * ( ay - MinYValue ) - MaxMouseMovement
    else:
        dy = 0

    disp = n.array([[dx], [dy]])
    return disp


def curCursorPostion():
    cusX, cusY = pg.position()

    posX, posY = screenCoordsToNormalise(cusX, cusY)
    return n.array([[posX], [posY]])

pos = curCursorPostion()


# inverse transform on cur position
isPathComplete = True
finalp = None

while(True):
    # cur unix time
    prevTime = time.time() * 1000
    
    # check if there is new a then get a 
    t, a = getAcceleration()
    if t:
        finalPos = calcNextPos(a) + pos
        # print("a")
        # print(a)
        
        # print("finalPos")
        # print(finalPos)
        # calculate path
        x0, y0 = normaliseToScreenCoords(pos[0, 0], pos[1, 0])
        x1, y1 = normaliseToScreenCoords(finalPos[0, 0], finalPos[1, 0])
        # print(pos)
        # print(finalPos)
        print(x1, " ", y1)
        print(x0, " ", y0)
        finalp = (x1, y1)

        isPathComplete = False

    # movetotalTime
    if(not isPathComplete):
        isPathComplete = not move(finalp)
    
    # cur unixtime
    curTime = time.time() * 1000
    frameTime = curTime - prevTime

    # sleep
    #if(frameTime < maxFrameTime):
    #    time.sleep((maxFrameTime - frameTime)/1000.0)
    
    
    pos = curCursorPostion()

ser.close()    

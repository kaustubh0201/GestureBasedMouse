import pyautogui as pg
import math 
import time
import serial
import re

import functools
import numpy as n

import struct

INCOMINGDATAPORT = "COM4"
DATARATE = 115200

ser = serial.Serial(port = INCOMINGDATAPORT, baudrate = DATARATE, timeout = 0.1)

ser.xonxoff = False     
ser.rtscts = False     
ser.dsrdtr = False      

input()

def getAcceleration():
    xR = 0
    yR = 0
    zR = 0


    if(ser.isOpen()):
        try:
            if(ser.inWaiting() > 0):
                xR = ser.read(4)
                yR = ser.read(4)

                fx = float(struct.unpack('f', xR)[0])
                fy = float(struct.unpack('f', yR)[0])

                return  [fx, fy]
        except Exception as e1:
            return None

    
    return None

WIDTH, HEIGHT = pg.size()

pg.moveTo(WIDTH/2, HEIGHT/2, 0)

x_direction = 0
y_direction = 0

xThres = 20
yThres = 20

xMag = 10
yMag = -10

######
# Throw data
p_t = time.time()

while(time.time() - p_t < 4):
    getAcceleration()

######


while(True):
    a = getAcceleration()

   

    if a != None:
        print(a)
        if(-xThres < a[1] < xThres ):
            x_direction = 0
        elif(a[1] > xThres):
            x_direction = 1
        elif(a[1] < -xThres):
            x_direction = -1    

        if(-yThres < a[0] < yThres):
            y_direction = 0
        elif(a[0] > yThres):
            y_direction = 1
        elif(a[0] < -yThres):
            y_direction = -1
        


        currX, currY = pg.position()

        currX = currX + x_direction * xMag
        currY = currY + y_direction * yMag
        
        print(" X-Accn: " + str(a[1]), " x_dir: " + str(x_direction), "|||||", " Y-Accn: " + str(a[0]), " y_dir: " + str(y_direction))

        pg.moveTo(currX, currY, 0)

ser.close()

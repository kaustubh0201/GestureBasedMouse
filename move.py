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

def getFiltered():
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
yThres = 30

xMag = 10
yMag = -5

baseX = 0
baseY = 0

######
# Throw data
num_readings = 100
i = 0
getFiltered()
while(i < num_readings):
   

    f = getFiltered()
    
    if f != None:
        i += 1
        baseX += f[0]
        baseY += f[1]
   

baseX /= num_readings
baseY /= num_readings

print(baseX, " ", baseY)
######

leftClickTime = 0

while(True):
    f = getFiltered()
    if f != None:
        
        f[0] -= baseX
        f[1] -= baseY
        print(f)


        if(-xThres < f[1] < xThres ):
            x_direction = 0
        elif(f[1] > xThres):
            x_direction = 1
        elif(f[1] < -xThres):
            x_direction = -1    

        if(-yThres < f[0] < yThres):
            y_direction = 0
        elif(f[0] > yThres):
            y_direction = 1
        elif(f[0] < -yThres):
            y_direction = -1
        
        
        if(x_direction == 0 and y_direction == 0):
            if(time.time() - leftClickTime > 2):
                pg.click()
        else:
            leftClickTime = time.time()


        currX, currY = pg.position()

        currX = currX + x_direction * xMag
        currY = currY + y_direction * yMag
        
        print(" X-Accn: " + str(f[1]), " x_dir: " + str(x_direction), "|||||", " Y-Accn: " + str(f[0]), " y_dir: " + str(y_direction))

        pg.moveTo(currX, currY, 0)

        

ser.close()

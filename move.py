import pyautogui as pg
import math 
import time
import serial
import re

import functools
import numpy as n

INCOMINGDATAPORT = "COM4"
DATARATE = 115200

ser = serial.Serial(port = INCOMINGDATAPORT, baudrate = DATARATE, timeout = 1)

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
            line = ser.readline().strip()
            values = line.decode('ascii').split(", ")
            if(functools.reduce(lambda a, b: a and b, map(lambda v: re.match(r'[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?', v), values))):
                xR = float(values[0])
                yR = float(values[1])
                zR = float(values[2])


                return  [xR, yR]
        except Exception as e1:
            ser.close()
            print (str(e1))
            return None

    
    return None

WIDTH, HEIGHT = pg.size()

pg.moveTo(WIDTH/2, HEIGHT/2, 0)

x_direction = 0
y_direction = 0

thres = 20

mag = 10

x_avg = 0


######
# Throw data
p_t = time.time()

while(time.time() - p_t < 4):
    getAcceleration()

######


while(True):
    a = getAcceleration()

    print(a)

    if a != None:
        
        if(-thres < a[1] < thres ):
            x_direction = 0
        if(a[1] > thres):
            x_direction = 1
        elif(a[1] < -thres):
            x_direction = -1    
        


        currX, currY = pg.position()

        currX = currX + x_direction * mag
        
        print(" X-Accn: " + str(a[0]), " x_dir: " + str(x_direction))

        pg.moveTo(currX, currY, 0)

    else:
        print("Error")

ser.close()

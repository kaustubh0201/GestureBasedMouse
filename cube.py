from direct.showbase.ShowBase import ShowBase
from direct.showbase.Loader import Loader
from panda3d.core import ClockObject
from direct.gui.OnscreenText import OnscreenText

import math
import time

import pyautogui as pg
import time
import serial
import re

import numpy as n
import struct

INCOMINGDATAPORT = 'COM2'
DATARATE = 115200

ser = serial.Serial(port = INCOMINGDATAPORT, baudrate = DATARATE, timeout = 0.1)

ser.bytesize = serial.EIGHTBITS         # Number of bits per bytes
ser.parity = serial.PARITY_NONE         # Set parity check: no parity

ser.stopbits = serial.STOPBITS_ONE      # Number of stop bits


ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control

# Frame per second
FPS = 60
maxFrameTime = 1.0/FPS

# Dimension of the screen
WIDTH, HEIGHT = pg.size()

serialDataList = []

class AccGyro:
    gx = 0
    gy = 0
    gz = 0

    ax = 0
    ay = 0
    az = 0

    def __init__(self, gx, gy, gz, ax, ay, az):
        self.gx = gx
        self.gy = gy
        self.gz = gz

        self.ax = ax
        self.ay = ay
        self.az = az     

def readSerialData():
    if(ser.isOpen()):
        try:
            if(ser.inWaiting() > 0):
                gx = ser.read(4)
                gy = ser.read(4)
                gz = ser.read(4)

                ax = ser.read(4)
                ay = ser.read(4)
                az = ser.read(4)
                
                serialDataList.append(AccGyro(gx, gy, gz, ax, ay, az))

                return True

        except Exception as e1:
            print (str(e1))
            return False

    
    return False


# def getGyro():
#     gx = 0
#     gy = 0
#     gz = 0
# 
#     if(len(gserialDataList) > 0):
#         #print("sasd")
#         coords = gserialDataList.pop(0)
#         #print(struct.unpack('f', coords[0]))
#         
#         gx = float(struct.unpack('f', coords[0])[0])
#         gy = float(struct.unpack('f', coords[1])[0])
#         gz = float(struct.unpack('f', coords[2])[0])
# 
#         return (gx, gy, gz)
# 
#     return ()
# 
# def getAccleration():
#     ax = 0
#     ay = 0
#     az = 0
# 
#     if(len(aserialDataList) > 0):
#         #print("asd")
#         coords = aserialDataList.pop(0)
#         #print(struct.unpack('f', coords[0]))
#         
#         ax = float(struct.unpack('f', coords[0])[0])
#         ay = float(struct.unpack('f', coords[1])[0])
#         az = float(struct.unpack('f', coords[2])[0])
# 
#         return (ax, ay, az)
# 
#     return ()

def getAccGyro():
    ax = 0
    ay = 0
    az = 0

    gx = 0
    gy = 0
    gz = 0
    if(len(serialDataList) > 0):
        c = serialDataList.pop(0)

        ax = float(struct.unpack('f', c.ax)[0])
        ay = float(struct.unpack('f', c.ay)[0])
        az = float(struct.unpack('f', c.az)[0])

        gx = float(struct.unpack('f', c.gx)[0])
        gy = float(struct.unpack('f', c.gy)[0])
        gz = float(struct.unpack('f', c.gz)[0])

        if(abs(gy) < 0.1):
            gy = 0
        
        return (gx, gy, gz, ax, ay, az)

    return ()

def clamp(n, smallest, largest): 
    return max(smallest, min(n, largest))

FPS = 60
frameTime = 1.0/FPS
AA = 0.98

t = None
class MyApp(ShowBase):
    box = None
    globalClock = None
    textObj = None
    textObj2 = None

    prev_t = time.time()

    gx = 0
    gy = 0
    gz = 0

    cx = 0
    cy = 0

    def __init__(self):
        super().__init__()

        self.box = self.loader.loadModel("./newCube.obj")
        # Reparent the model to render.
        self.box.reparentTo(self.render)
        # Apply scale and position transforms on the model.
        self.box.setScale(0.5, 0.5, 0.5)
        self.box.setPos(0, 0, 0)
        #self.box.place()

        self.cy = self.box.getR()
        self.cx = self.box.getP()

        self.textObj = OnscreenText(text = 'Gyro', pos= (0.0, 0.02), scale=0.07)
        self.textObj2 = OnscreenText(text = 'Acc', pos= (0.0, 0.1), scale=0.07)
        self.taskMgr.add(self.update, "asd")

        self.globalClock = ClockObject.getGlobalClock()

        self.globalClock.setMode(ClockObject.MLimited)
        self.globalClock.setFrameRate(FPS)

    def update(self, task):
        global AA
        readSerialData()
        
        ga = getAccGyro()

        if(ga != ()):
            g = (ga[0], ga[1], ga[2])
            a = (ga[3], ga[4], ga[5])


            cur_t = time.time()
            dt = cur_t - self.prev_t
            self.prev_t = cur_t

            # print(dt)

            gx_rate = g[0] * 0.07
            gy_rate = g[1] * 0.07

            accXangle = math.degrees(math.atan2(a[1], math.sqrt(a[2]**2 + a[0]**2) ))
            accYangle = -math.degrees(math.atan2(a[0], math.sqrt(a[1]**2 + a[2]**2) ))

            self.cx = AA * (self.cx + gx_rate * dt) + (1 - AA) * accXangle
            self.cy = AA * (self.cy + gy_rate * dt) + (1 - AA) * accYangle       

            self.textObj.setText("gx: " + str(gx_rate * dt) + "ay: " + str(gy_rate * dt) )
            self.textObj2.setText("accXangle: " + str(accXangle) + " accYangle: " + str(accYangle))

            self.box.setR(self.box, self.cy)
            self.box.setP(self.box, self.cx)

        return task.cont

app = MyApp()
app.run()
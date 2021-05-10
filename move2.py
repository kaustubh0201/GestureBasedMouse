import pyautogui as pg
import math
import time
import serial
import re
from pynput.mouse import Button, Controller
import functools
import numpy as n

# case x1 > x2, y1 > y2 ✔️✔️✔️ (1008, 567) -> (960, 540)
# case x1 > x2, y1 < y2 ✔️✔️✔️ (1008, 513) -> (960, 540)
# case x1 < x2, y1 > y2 ✔️✔️✔️(912, 567) -> (960, 540)
# case x1 < x2, y1 < y2 ✔️✔️✔️ (960, 540) -> (1008, 567)

# with open("./test_inputs") as file:
#     line = file.readline()

INCOMINGDATAPORT = "/dev/pts/4"
DATARATE = 9600

ser = serial.Serial(port = INCOMINGDATAPORT, baudrate = DATARATE, timeout = 1)

ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control

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


                return True, n.array([[xR], [yR]])            
        except Exception as e1:
            ser.close()
            print (str(e1))
            return False, n.array([])

    
    return False, n.array([])

#correctly-implemented
def midPoint(ix0, iy0, ix1, iy1):
    if(iy0 == iy1):
        xStart = ix0
        xFin = ix1
        i = 1
        if(xStart > xFin):
            i = -1

        path=[]
        for j in range(xStart, xFin + i, i):
            path.append((j, iy0))

        return path  
    
    if(ix0 == ix1):
        yStart = iy0
        yFin = iy1
        i = 1
        if(yStart > yFin):
            i = -1
        
        path = []
        for j in range(yStart, yFin + i, i):
            path.append((ix0, j))

        return path

    path = []
    isReverse = False


    if(ix0 > ix1 and iy0 > iy1):
        ix0, ix1 = ix1, ix0
        iy0, iy1 = iy1, iy0
        isReverse = True
        
    elif(ix0 < ix1 and iy0 > iy1):
        ix0, ix1 = ix1, ix0
        iy0, iy1 = iy1, iy0
        isReverse = True

    xIncr = 1
    compare = lambda x0, x1: x0 < x1
    if(ix0 > ix1 and iy0 < iy1):
        xIncr = -1
        compare = lambda x0, x1: x0 > x1

    # calculate dx & dy
    dx = abs(ix1 - ix0)
    dy = abs(iy1 - iy0)
 
    # initial value of decision parameter d
    d = dy - (dx/2)
    x = ix0
    y = iy0
 
    # Plot initial given point
    # putpixel(x,y) can be used to print pixel
    # of line in graphics
    path.append((x, y))
    # print(x,",",y)
    # iterate through value of X

    while (compare(x, ix1)):
        x=x + xIncr 
        # E or East is chosen
        if(d < 0):
            d = d + dy
 
        # NE or North East is chosen
        else:
            d = d + (dy - dx)
            y=y+1
     
 
        # Plot intermediate points
        # putpixel(x,y) is used to print pixel
        # of line in graphics
        # print(x,",",y)
        
        path.append((x, y))

    if(isReverse):
        path.reverse()

    return path
        



def clamp(n, smallest, largest): 
    return max(smallest, min(n, largest))


# Frame per second
FPS = 600.
maxFrameTime = 1.0/FPS * 1000

# Dimension of the screen
WIDTH, HEIGHT = pg.size()


def normaliseToScreenCoords(x, y):

    x_new = clamp (math.floor((x+1) / 2 * WIDTH), 0, WIDTH - 1)
    y_new = clamp (HEIGHT - math.floor((y+1) / 2 * HEIGHT), 0, HEIGHT - 1)
    
    return x_new, y_new


def screenCoordsToNormalise(x, y):
    x_new = clamp(2 * x / float(WIDTH) - 1, -1, 1) 
    y_new = clamp(-2 * y / float(HEIGHT) + 1, -1, 1)

    return x_new, y_new

class State:
    k = 0
    path = []

    def __init__ (self, path):
        self.path = path
    
    def nextPoint(self):
        return self.path[self.k]
    
    def updateState(self):
        if(self.k + 1 < len(self.path)):
            self.k += 1

            return True
        else:
            return False

def move(s):
    x, y = s.nextPoint()   
    mouse.move(x, y)
    # pg.moveTo(x, y, 0.1)
    if(s.updateState()):
        return True
    else:
        return False


def cleanPath(path):
    v = 200
    
    x0, y0 = path[0]
    x1, y1 = path[-1]

    dist = math.sqrt( (x0 - x1)**2 + (y0 - y1)**2 )   
    t = dist/v

    remPoints = t/0.1

    incre = math.ceil((len(path) - 2)/float(remPoints))

    p = [path[0]]

    for i in range(1, len(path) - 1, incre):
        p.append(path[i])
    
    p.append(path[-1])

    return p



def normalise(v):
    l = n.linalg.norm(v)
    if(l != 0):
        return 1/l * v
    else:
        return v

MOUSESPEED = 100
ACCMAG = 10

def calcPositionVelocity(a):
    timePassed = maxFrameTime/1000

    n_a = normalise(a)
    print("n_a")

    print(n_a)

    disp = 0.1 * n_a
    
    return disp

# inverse transform on cur position


pos = curCursorPostion()
isPathComplete = True
path = []

t = time.time()
m = Controller()


m.position = (500, 500)

while(True):
    if( (time.time() - t) > 2):
        m.position = (0, 0)
        break



ser.close()    
